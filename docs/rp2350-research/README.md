# RP2350 port research

Primary-source reference material for the RP2350 / Pico 2 port, plus the
datasheet-verified data extracted from it. See `../../plans/rp2350-port.md` for the
plan this supports.

## Committed source documents (pinned — this exact revision is what we verified against)
- `rp2350-datasheet.pdf` — RP2350 datasheet (1380 pp). Source:
  https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf
- `pico-2-datasheet.pdf` — https://datasheets.raspberrypi.com/pico/pico-2-datasheet.pdf
- `hardware-design-with-rp2350.pdf` —
  https://datasheets.raspberrypi.com/rp2350/hardware-design-with-rp2350.pdf
- `RP2350.svd` — official System View Description (independent second source for
  addresses/IRQs). From `raspberrypi/pico-sdk`:
  `src/rp2350/hardware_regs/RP2350.svd`

## Derived / verified data (the useful outputs)
- `findings.md` — datasheet-verified facts (IMAGE_DEF bytes §5.9.5.1, no-boot2/XIP,
  §5.4.4 flash-vs-XIP hazard + boot locks, ACCESSCTRL §10.6.2.1 privilege split, IRQ
  map, PR #2210 oracle). Each fact labels its provenance.
- `addresses.md` — peripheral base addresses + IRQ map for `chips/rp235x/chip.toml`,
  cross-checked against both the datasheet and the SVD.
- `verify.py` — reproducible check: re-diffs `addresses.md` against `RP2350.svd`
  (`python3 verify.py`; exits non-zero on any drift). Fetch the SVD if missing.

## Ignored (regenerable, not committed)
- `*.txt` — `pdftotext` dump of the datasheet (regenerate:
  `pdftotext -layout rp2350-datasheet.pdf rp2350-datasheet.txt`).
- `__pycache__/`.
