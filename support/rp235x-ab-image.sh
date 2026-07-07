#!/usr/bin/env bash
# Build an unbrickable A/B image for any Pico 2 W app variant, and (optionally)
# flash it. Produces one combined binary: partition table @0, slot A (higher
# version) @0x2000, slot B (lower version) @0x42000 -- the RP2350 boot ROM boots
# the higher-versioned valid slot, so an interrupted field update (which the
# `update` shell command writes to the INACTIVE slot) never bricks the board.
#
# Usage:   support/rp235x-ab-image.sh <app> [slotA_ver] [slotB_ver] [--flash]
# Example: support/rp235x-ab-image.sh demo-pi-pico-2-wifi 2.0 1.0 --flash
#
# Apps: demo-pi-pico-2, demo-pi-pico-2-wifi, demo-pi-pico-2-wifi-amp,
#       demo-pi-pico-2-amp, demo-pi-pico-2-slink
set -euo pipefail

APP="${1:?usage: $0 <app> [slotA_ver] [slotB_ver] [--flash]}"
AVER="${2:-2.0}"   # slot A: the booted (higher) version
BVER="${3:-1.0}"   # slot B: the spare (lower) version
FLASH=0; [[ "${4:-}" == "--flash" || "${2:-}" == "--flash" || "${3:-}" == "--flash" ]] && FLASH=1

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
DIST="target/$APP/dist/default/final.bin"
OUT="/tmp/${APP}-ab.bin"
PT="/tmp/${APP}-pt.bin"

echo ">> build slot A ($APP v$AVER)"
HUBRIS_IMAGE_VERSION="$AVER" cargo xtask dist "app/$APP/app.toml" >/dev/null
cp "$DIST" "/tmp/${APP}-slotA.bin"
echo ">> build slot B ($APP v$BVER)"
HUBRIS_IMAGE_VERSION="$BVER" cargo xtask dist "app/$APP/app.toml" >/dev/null
cp "$DIST" "/tmp/${APP}-slotB.bin"

echo ">> partition table (picotool partition create)"
picotool partition create chips/rp235x/ab-partitions.json "$PT" -t bin

echo ">> combine: pt@0 + slotA@0x2000 + slotB@0x42000 -> $OUT (520 KiB)"
dd if=/dev/zero of="$OUT" bs=4096 count=130 2>/dev/null   # 0x82000 = 520 KiB
dd if="$PT" of="$OUT" conv=notrunc 2>/dev/null
dd if="/tmp/${APP}-slotA.bin" of="$OUT" bs=1 seek=8192 conv=notrunc 2>/dev/null    # 0x2000
dd if="/tmp/${APP}-slotB.bin" of="$OUT" bs=1 seek=270336 conv=notrunc 2>/dev/null  # 0x42000

# Select a probe with PROBE=<vid:pid> (needed when more than one is attached),
# e.g. PROBE=2e8a:000c support/rp235x-ab-image.sh ... --flash
PROBE_ARG=""; [[ -n "${PROBE:-}" ]] && PROBE_ARG="--probe $PROBE"
FLASH_CMD="probe-rs download --chip RP235x $PROBE_ARG --binary-format bin --base-address 0x10000000 $OUT"
if [[ "$FLASH" == 1 ]]; then
    echo ">> flashing: $FLASH_CMD"
    $FLASH_CMD && probe-rs reset --chip RP235x $PROBE_ARG
else
    echo ">> built $OUT -- flash with:"
    echo "   $FLASH_CMD"
fi
echo ">> after boot, the shell 'slot' command shows verA/verB + the update target;"
echo "   an 'update' writes the inactive (lower-version) slot -- unbrickable."
