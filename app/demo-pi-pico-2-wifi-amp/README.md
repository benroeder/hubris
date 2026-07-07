# Hubris on the Raspberry Pi Pico 2 (RP2350)

A Hubris port to the RP2350A (dual Cortex-M33, executing in place from the
Pico 2's 4 MB external QSPI flash). Single-core, Secure, Arm only. Everything
below is verified on hardware.

## What runs

| Task | Purpose |
|---|---|
| `jefe` | supervisor: restarts faulted tasks |
| `sys` | owns RESETS; other drivers reset their blocks through it |
| `gpio_driver` | Bank-0 GPIO Idol server (SIO), incl. pin function select |
| `uart_driver` | UART0 (PL011), interrupt-driven RX ring |
| `spi_driver` | SPI0 (PL022), full-duplex exchange (internal loopback mode) |
| `i2c_driver` | I2C0 (DW_apb_i2c) 7-bit master, 100 kHz |
| `adc_driver` | one-shot ADC reads incl. the die temperature sensor |
| `pwm_driver` | duty-cycle control on the 12 PWM slices, 1 kHz |
| `flash_driver` | flash reads (XIP mirror), boot-ROM table lookup, reboot |
| `usb` | USB CDC-ACM console transport (interrupt-driven) |
| `shell` | interactive command shell, a pure IPC client of everything |
| `idle` | lowest-priority spin |

## Build and flash

    cargo xtask dist app/demo-pi-pico-2/app.toml
    picotool uf2 convert target/demo-pi-pico-2/dist/default/final.bin \
        -t bin pico2.uf2 -o 0x10000000 --family rp2350-arm-s

Hold BOOTSEL while plugging the Pico 2 in (or drag the UF2 onto the mass
storage device), then:

    picotool load pico2.uf2 && picotool reboot

Use `final.bin` (or the UF2), not the loose `final.elf`, which is a
relocatable intermediate that picotool can't parse. `picotool info -a`
validates the image format host-side (Arm-Secure, IMAGE_DEF at 0x10000160).

## Talk to it

The board enumerates as a USB CDC serial device (`/dev/cu.usbmodemHUBRIS_0001`
on macOS, `/dev/ttyACM*` on Linux):

    screen /dev/cu.usbmodemHUBRIS_0001
    hubris> help
    hubris> temp
    die temp 26.6 C (raw 877)
    hubris> status
    uart: rx=15 (loopback OK)      # with a GP0->GP1 jumper
    spi:  loopback OK
    i2c:  no devices
    hubris> flash read 160 32      # hexdump the image's own IMAGE_DEF block
    hubris> led dim 10
    hubris> reboot

The onboard LED blinks ~1 Hz as a liveness heartbeat while the shell is idle.

## Port notes (the non-obvious parts)

- **IMAGE_DEF**: the RP2350 boot ROM refuses images without a PICOBIN
  IMAGE_DEF block in the first 4 KiB; it is emitted through the `.image_def`
  section (see `build/kernel-link.x` and this app's `main.rs`).
- **Clocks**: privileged pre-kernel startup (CLOCKS/PLL/QMI are ACCESSCTRL
  Secure-privileged-only) brings up XOSC -> PLL_SYS 150 MHz, retuning the QMI
  flash divider *before* the ramp (datasheet sec 5.4.4 XIP hazard), plus
  PLL_USB 48 MHz for USB and ADC.
- **PMSAv8 MPU**: overlapping regions are prohibited (UNPREDICTABLE). The
  flash driver reads the whole flash through the *uncached XIP mirror*
  (0x14000000) because the cached window would overlap its own code region,
  and the kernel's per-task null region (0x0..0x20) makes the boot ROM's Arm
  header words unreachable -- the ROM function table is found via the RISC-V
  pointer copy at 0x7df6 and walked in Rust (`lib/rp235x-romapi`).
- **Boot-ROM calls are privilege-gated** (verified by experiment: a
  privileged `flash_op` works; the same call from a task faults). `reboot`
  drives the watchdog/PSM registers directly, and `reboot bootsel` does a
  two-hop: the task marks watchdog scratch0 and watchdog-reboots, then the
  privileged pre-kernel boot path sees the marker and calls the ROM
  reboot-into-BOOTSEL. Reads/lookups work from tasks (table walked in Rust).
- Known gaps: erase/program (flash stage 2: needs a RAM-resident kernel and
  a privileged path for ROM flash calls), analog pad config for ADC 0-3.

## Debug

No SWO on the RP2350; use SWD. Upstream OpenOCD 0.12 lacks RP2350 support --
use the Raspberry Pi OpenOCD fork or probe-rs >= 0.27 (`chip RP235x`,
already configured in `boards/pi-pico-2.toml`).

See `plans/rp2350-port.md` for the full progress log and
`docs/rp2350-research/` for datasheet-verified reference material.
