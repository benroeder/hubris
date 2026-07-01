#!/usr/bin/env python3
"""Re-diff addresses.md against the official RP2350 SVD so the hand-authored table
cannot silently drift.

Parses the base-address table and the IRQ map out of addresses.md, then checks each
entry against RP2350.svd (from raspberrypi/pico-sdk). Exits non-zero on any mismatch.

Usage:  python3 verify.py            # expects RP2350.svd + addresses.md alongside
Provenance: this validates the *dual-source* facts (bases, IRQs). It does NOT check
the ACCESSCTRL priv/open column (datasheet-only) or block sizes (informational).
"""
import re
import sys
from pathlib import Path

HERE = Path(__file__).parent
SVD = HERE / "RP2350.svd"
DOC = HERE / "addresses.md"

# QMI appears as XIP_QMI in some SVD revisions; accept either name.
ALIASES = {"QMI": ("QMI", "XIP_QMI")}

# IRQs present in datasheet Table 95 but deliberately absent from the SVD: the
# CoreSight cross-trigger (CTI) debug interrupts. Not attached to any SVD peripheral
# and not driver-relevant. Verified 2026-07-01. Kept explicit so real drift still fails.
DATASHEET_ONLY_IRQS = {"PROC0_IRQ_CTI": 40, "PROC1_IRQ_CTI": 41}


def load_svd(text):
    bases, irqs = {}, {}
    for m in re.finditer(r"<peripheral[^>]*>(.*?)</peripheral>", text, re.S):
        body = m.group(1)
        nm = re.search(r"<name>([^<]+)</name>", body)
        if not nm:
            continue
        name = nm.group(1)
        ba = re.search(r"<baseAddress>([^<]+)</baseAddress>", body)
        if ba:
            bases[name] = int(ba.group(1), 0)
        for im in re.finditer(
            r"<interrupt>\s*<name>([^<]+)</name>\s*"
            r"(?:<description>[^<]*</description>\s*)?<value>(\d+)</value>",
            body,
        ):
            irqs[im.group(1)] = int(im.group(2))
    return bases, irqs


def svd_base(bases, name):
    for cand in ALIASES.get(name, (name,)):
        if cand in bases:
            return bases[cand]
    return None


def parse_doc_bases(text):
    """Rows like: | UART0 | `0x40070000` | ... |"""
    out = {}
    for name, addr in re.findall(
        r"^\|\s*([A-Z][A-Z0-9_]+)\s*\|\s*`(0x[0-9a-fA-F_]+)`", text, re.M
    ):
        out[name] = int(addr.replace("_", ""), 16)
    return out


def parse_doc_irqs(text):
    """The fenced IRQ block: lines of `NN NAME` pairs (2-4 columns)."""
    out = {}
    block = re.search(r"## Interrupt map.*?```(.*?)```", text, re.S)
    if block:
        for num, nm in re.findall(r"\b(\d{1,2})\s+([A-Z][A-Z0-9_]+)", block.group(1)):
            # skip the "46-51 SPAREIRQ_IRQ_0..5" summary token
            if nm.startswith("SPAREIRQ"):
                continue
            out[nm] = int(num)
    return out


def main():
    if not SVD.exists():
        print(f"FAIL: {SVD.name} missing (fetch from raspberrypi/pico-sdk)", file=sys.stderr)
        return 2
    svd_text = SVD.read_text()
    doc_text = DOC.read_text()
    bases, irqs = load_svd(svd_text)
    doc_bases = parse_doc_bases(doc_text)
    doc_irqs = parse_doc_irqs(doc_text)

    errors, checked = [], 0
    # Only check names the SVD actually knows as peripherals (skips SRAM/XIP regions).
    for name, addr in doc_bases.items():
        want = svd_base(bases, name)
        if want is None:
            continue  # not an SVD peripheral (memory region, alt window, etc.)
        checked += 1
        if want != addr:
            errors.append(f"BASE {name}: doc {addr:#010x} != svd {want:#010x}")
    for name, num in doc_irqs.items():
        if name not in irqs:
            if DATASHEET_ONLY_IRQS.get(name) == num:
                continue  # known datasheet-only debug IRQ; not an error
            errors.append(f"IRQ  {name}: in doc (#{num}) but not in SVD")
            continue
        checked += 1
        if irqs[name] != num:
            errors.append(f"IRQ  {name}: doc {num} != svd {irqs[name]}")

    if errors:
        print(f"MISMATCH ({len(errors)} of {checked} checks):", file=sys.stderr)
        for e in errors:
            print("  " + e, file=sys.stderr)
        return 1
    print(f"OK: {checked} base/IRQ entries in addresses.md match RP2350.svd")
    return 0


if __name__ == "__main__":
    sys.exit(main())
