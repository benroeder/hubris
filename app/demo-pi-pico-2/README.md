# Raspberry Pi Pico 2 demo application

A minimal Hubris application demonstrating RP2350 (RP235x) bring-up on the Raspberry
Pi Pico 2. Only `jefe` and `idle` run — this is the Phase 1 / G1 milestone: prove the
image builds and carries a valid RP2350 `IMAGE_DEF` boot block.

## Build

    cargo xtask dist app/demo-pi-pico-2/app.toml

## Validate the boot-image format (host-side, no hardware)

    picotool info -a target/demo-pi-pico-2/dist/default/final.bin

`picotool` reports a valid RP2350 Arm-Secure image with an `image def` metadata
block within the first 4 KiB (observed at 0x10000160 = vector table + ImageHeader).
Use `final.bin` (or the `.uf2`), not the loose `final.elf`, which is a relocatable
intermediate that picotool can't parse. See
`docs/rp2350-research/` for the datasheet-verified details behind the IMAGE_DEF block.

## Flash / debug

Flash with `picotool load` (BOOTSEL) or `probe-rs` (RP2350-native). Upstream OpenOCD
0.12 lacks RP2350 support; use the Raspberry Pi OpenOCD fork or probe-rs.
