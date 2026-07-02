# Port Hubris to the Raspberry Pi RP2350 (Pico 2)

## Context

Hubris (Oxide's Rust embedded OS) currently supports STM32 (F3/F4/G0/H7) and NXP
LPC55 chips. We want a **full peripheral port** to the **Raspberry Pi RP2350A** on
the **Pico 2** dev board (dual Cortex-M33, ARMv8-M, XIP from external 4 MB QSPI flash).

The single biggest enabler: **the kernel already supports ARMv8-M / Cortex-M33**
because LPC55 uses it. `sys/kern/src/arch/arm_m.rs` already handles the M33 MPU
(`#[cfg(armv8m)]`, 32-byte regions, RBAR/RLAR/MAIR), FPU context save, SAU-aware
linker veneers, SVC/PendSV/SysTick. **No kernel changes are expected for the
single-core Arm-S bring-up** (Hubris is single-core — `sys/kern/src/startup.rs:41`
forbids multi-core `start_kernel` entry; `dist.rs:2185` emits exactly one kernel
STACK region). The work is a new chip definition, a startup/clock library, a driver
layer, an app, and a **`dist.rs` change to emit the RP2350-mandated boot metadata**
alongside the existing Hubris `ImageHeader`, plus flashing.

> **Reviewed by an adversarial persona ("Dr. Voss", `codex exec`, gpt-5.4) across four
> passes against the live tree — converged to ship-with-changes, all findings closed.**
> Eight under-specified decisions were locked (see *Locked decisions after review*).
> **G0 primary-source verification done:** IMAGE_DEF bytes, no-boot2/XIP, §5.4.4 flash
> hazard + boot locks, ACCESSCTRL privilege split, IRQ map, and the address map are all
> confirmed against the datasheet in `docs/rp2350-research/findings.md`. Reviews:
> `plans/rp2350-port-review-output*.md`.

## Progress — verified on real hardware (Raspberry Pi Pico 2)

Branch `rp2350-port`. Every commit below was flashed and confirmed on a physical
Pico 2 (no debug probe — verified via `picotool`, the boot-accept behaviour, and the
onboard LED / a GP0↔GP1 UART loopback as visual signals):

| Commit | What | Hardware proof |
|---|---|---|
| `docs …` | Plan + datasheet-verified research (`docs/rp2350-research/`) | `verify.py` cross-checks addresses vs SVD |
| G1 boot | `chips/rp235x`, `app/demo-pi-pico-2`, `.image_def` in `kernel-link.x`, `rp235x-pac` | **boot ROM accepts our IMAGE_DEF**; `picotool info` valid; block at 0x10000160 |
| kernel | jefe+idle + `task-rp235x-blinky`; GPIO25 via SIO | **kernel schedules an unprivileged task** (LED blinks) |
| clock | `lib/rp235x-startup`: XOSC 12 MHz | crystal-accurate **1 Hz** blink |
| UART | `lib/rp235x-uart`: PL011, GP0/GP1 | **TX+RX** via loopback (double-blink) |
| 150 MHz | PLL_SYS + **QMI flash retune** (CLKDIV=6) | double-blink at 150 MHz baud — the XIP-flash hazard handled |
| USB clk + IRQ | PLL_USB 48 MHz; UART RX IRQ → task notification | LED toggles only on interrupt delivery |
| USB CDC | `lib/rp235x-usb` (UsbBus adapted from rp-hal) + `task/rp235x-usb` (usbd-serial), IRQ-driven | enumerates as `/dev/cu.usbmodemHUBRIS_0001`; echo verified; console streams |
| GPIO Idol | `drv/rp235x-gpio{,-api}` + `idl/rp235x-gpio.idol` | LED blinks via `gpio.toggle()` IPC from `task-rp235x-logdemo` |
| sys Idol | `drv/rp235x-sys{,-api}`: single RESETS owner | GPIO resets via `sys.leave_reset()` IPC; console keeps streaming |
| UART Idol | `drv/rp235x-uart{,-api}`: IRQ-driven RX ring, lease I/O | loopback `uart_rx=15` (exact marker length) each tick |
| SPI Idol | `drv/rp235x-spi{,-api}`: PL022 mode 0, 1.5 MHz | `spi=OK` — 3-byte full-duplex round-trip via internal loopback |
| I2C Idol | `drv/rp235x-i2c{,-api}`: DW 7-bit master, 100 kHz | empty-bus scan: 112 probes/tick NAK + recover cleanly (`i2c=0`); ACKed-transfer test pending a 2nd Pico as I2CTarget |

**Retired risks (were the plan's biggest unknowns, now proven on silicon):** IMAGE_DEF
byte-correctness; "ARMv8-M kernel for free" (no kernel changes); ACCESSCTRL privilege
split (privileged startup touches CLOCKS/PLL/QMI, unprivileged task reaches SIO/UART
via `uses`); MPU task isolation; the §5.4.4 XIP-during-clock-ramp flash hazard;
peripheral IRQ → notification delivery; USB device controller (greenfield for Hubris).

**Gate status:** G0 ✅ · G1 ✅ (host + hardware) · G2 ✅ · G3 ✅ · G4 ✅ (GPIO, UART,
SPI, I2C all behind Idol servers; I2C ACK test awaits a bus peer) · G5–G6 not started.

**Idiomatic now:** sys/gpio/uart/spi/i2c are `drv/rp235x-*` Idol servers over IPC;
the demo client (`task-rp235x-logdemo`) self-tests all of them every tick over the
USB console (`tick N uart_rx=15 spi=OK i2c=0`). Remaining: flash/update server
(Tier 3), hiffy + on-target tests + debug-probe workflow (Phase 5).

## Next: USB console (talk to the board over its native USB port)

Goal: a **USB CDC-ACM** device so the Pico 2 appears as `/dev/cu.usbmodem…` — an
interactive console over the one USB cable, no adapter/probe. **Hubris has zero USB
code** (confirmed: the only `usb` hits are LPC55 repurposing USB SRAM for DICE), so
this is fully greenfield and the largest driver in the port. **Route B:** use the
`usb-device` + `usbd-serial` crates + a minimal RP2350 `UsbBus` backend (don't
re-implement enumeration/CDC). Debuggable without a probe: macOS logs enumeration.

Do **before** the USB stack, in order:
1. **`clk_usb` = 48 MHz** — add PLL_USB (12 MHz × 40 / (5×2)) to `lib/rp235x-startup`.
   Hard prerequisite; low-risk (no flash); LED-canary verifiable.
2. **Prove peripheral IRQ → task notification** — never tested on this chip; USB is
   IRQ-driven. Cheapest test: enable UART0 RX IRQ, wire
   `interrupts = {"uart0.irq" = "…"}` to a task, blink on RX (reuses the loopback).
   De-risks USB's biggest infrastructure dependency in ~20 lines.
3. **Push the branch** — 6 verified commits are laptop-only; get them off before the
   risky USB work.
4. **`UsbBus` backend decision** — reuse an rp235x backend vs. adapt `rp235x-hal` vs.
   write a minimal one; decides polled-vs-IRQ task architecture.

Then: (5) **enumeration** — device appears in `system_profiler`; (6) **CDC data** —
`/dev/cu.usbmodem…` appears; (7) **console task** — echo/handle input.
*Optional first:* refactor UART into a proper `drv/rp235x-uart` Idol server so USB
follows an established driver template.

What makes RP2350 different from an STM32/LPC55 port (the real work beyond generic M33):
1. **Mandatory `IMAGE_DEF` metadata block** in the first 4 KiB — the boot ROM
   *refuses to start* an image without it. No STM32 analogue.
2. **No internal flash** — execute-in-place from external QSPI; boot ROM sets up
   XIP/QMI for us (unlike RP2040, no boot2 stage needed).
3. **Banked 520 KB SRAM** (striped banks + two 4 KB scratch banks SRAM8/9).
4. **RP2350-E9 erratum** — Bank-0 GPIO input leakage; design drivers/boards around it.
5. TrustZone exists but **Hubris ignores it** and uses the M33 MPU path (matches LPC55).

### Decisions taken (from clarification)
- **Scope:** full peripheral port (bring-up → drivers → I2C/SPI/flash/update).
- **Starting point — hybrid:** use community **PR #2210** (thenewwazoo) purely as a
  *reference oracle* for the bring-up (IMAGE_DEF bytes, `memory.toml` XIP layout,
  boot-to-jefe path). Build the **full port fresh in-tree** using the **LPC55 crates
  as the structural template**. Set exhubris aside (it's an out-of-tree model).
- **Board:** Pico 2, RP2350A + 4 MB external QSPI NOR.
- **Toolchain — picotool first, humility second:**
  - *picotool first* because the riskiest unknown is "will the boot ROM accept our
    image?" — an image-*format* problem. picotool validates/reports on `IMAGE_DEF`,
    speaks UF2 family `0xe48bff59`, loads over USB-BOOTSEL or SWD, and needs **zero**
    changes to Hubris's vendored toolchain. It proves the format and unblocks all
    driver bring-up.
  - *humility second* because it rides on a **stale forked `probe-rs`** that predates
    RP2350 (needs ≥0.27 for ADIv6 + RP235x flash). Bumping that fork is a real,
    invasive prerequisite (a blocker on PR #2210). We pay that cost *after* the image
    is proven, so we integrate against a known-good target and unlock Hubris's native
    introspection (`humility tasks`/`dump`).

### Locked decisions after review (previously "decide during implementation")
1. **Concurrency: single-core.** Only core 0 runs; **core 1 is held in reset for all
   phases.** No "per-core stacks" — one core-0 kernel/MSP stack only. Multicore, if
   ever, is a separate future design phase. (Fixes review #3.)
2. **First-4-KiB byte layout is fixed up front** (fixes review #1/#2; **mechanism now
   taken from PR #2210, which boots**): image = **vector table** → **Hubris's
   `abi::ImageHeader` at its current `.header` offset** (do NOT move it —
   `lib/lpc55-rot-startup/src/images.rs:247`, `drv/lpc55-update-server/src/main.rs:1224`,
   `sys/abi/src/lib.rs:541`) → a **new `.image_def` linker section** holding the
   20-byte block loop. The block is a `#[link_section=".image_def"] #[used] static
   [u32;5]` (in the app crate), pulled by `KEEP(*(.image_def))`; `_stext` is extended by
   `SIZEOF(.image_def)`. **NOT** a `dist.rs` post-patch — this preserves the
   `ImageHeader` invariant untouched. `kernel-link.x` gains 4 ASSERTs (block ≤4 KiB
   from vector-table start; vector+header+image_def ≤0x1000; `SIZEOF(.image_def)==20`;
   image_def precedes `.text`). The 20 bytes are datasheet-verified **and** match
   PR #2210 byte-for-byte (`docs/rp2350-research/findings.md` §1).
3. **Flash image format decided in Phase 1, not Phase 4** (fixes review #6): start
   **single-image / single-slot** (no `PARTITION_TABLE`) for bring-up; A/B is an
   explicit later decision because it changes `memory.toml`, UF2 family targeting,
   and update-server shape. UF2 family = `0xe48bff59` (RP2350 ARM-S).
4. **Boot-ROM access is table-dispatched** (fixes review #6): a minimal
   `lib/rp235x-romapi` shim using `rom_func_lookup` (datasheet §5.4.1) is needed
   before any ROM flash/reboot call — there are **no fixed ROM offsets** (LPC55's
   `lib/lpc55-romapi` does not transfer).
5. **probe-rs/humility is an EXTERNAL dependency, not in-tree code** (fixes review
   #5): this repo does not vendor `probe-rs` (`Cargo.lock` has none); `humility` is
   an external binary (`build/xtask/src/humility.rs:31`). Phase 5a's deliverable is
   "document/require the external humility+probe-rs ≥0.27 build," optionally upstream
   PRs — NOT a Hubris-tree probe-rs bump.
6. **IRQ model** (fixes review #7): model real sources **0–45**; **46–51 are
   `SPAREIRQ_IRQ_0..5`** (datasheet §3.2 Table 95) — expose only if a driver needs a
   software IRQ. Don't imply 52 peripheral IRQs.
7. **Earliest bring-up peripherals include `ACCESSCTRL`, `WATCHDOG`, `POWMAN`** —
   on RP2350 these gate access/reset/power and are not "later polish."
8. **ACCESSCTRL privilege split (datasheet-verified §10.6.2.1 — new, load-bearing):**
   Hubris tasks run **unprivileged**; kernel/pre-main runs **privileged**. On RP2350,
   **CLOCKS/PLL/XOSC/PSM/WATCHDOG/POWMAN/QMI default to Secure-*Privileged*-only**,
   while UART/SPI/I2C/GPIO/RESETS are Secure-*any-privilege*. ⇒ **all clock/PLL/power/
   QMI bring-up happens in the privileged `lib/rp235x-startup`** (matches Hubris
   pattern); ordinary driver tasks need no ACCESSCTRL change. Any *runtime* task that
   must touch a Privileged-only block (notably the flash update-server ⇒ QMI) requires
   the privileged startup to **widen ACCESSCTRL (grant Secure-Unprivileged)** for that
   block. See `docs/rp2350-research/findings.md` §4.

### Out of scope
- RISC-V Hazard3 cores (M33 only).
- TrustZone secure boot / signed images (secp256k1 + OTP) — boot an unsigned
  `EXE_SECURITY_S` image. Revisit only if Oxide wants attestation.
- RP2354 in-package-flash variant (different `memory.toml`, easy follow-up).

---

## Reference: the LPC55 surface we are cloning

| Concern | LPC55 location | RP2350 analogue to create |
|---|---|---|
| Chip peripherals + IRQs | `chips/lpc55/chip.toml` | `chips/rp235x/chip.toml` |
| Memory map | `chips/lpc55/memory.toml` | `chips/rp235x/memory.toml` |
| Debug script | `chips/lpc55/openocd.{cfg,gdb}` | `chips/rp235x/openocd.{cfg,gdb}` (RPi OpenOCD fork) |
| Startup / clocks | `lib/lpc55-rot-startup`, `lib/lpc55-romapi` | `lib/rp235x-startup` |
| Pin codegen | `build/lpc55pins` | `build/rp235xpins` |
| sys/reset/clock driver | `drv/lpc55-syscon{,-api}` | `drv/rp235x-sys{,-api}` |
| GPIO | `drv/lpc55-gpio{,-api}` | `drv/rp235x-gpio{,-api}` |
| UART | `drv/lpc55-usart`, `lib/lpc55-usart` | `drv/rp235x-uart` |
| I2C / SPI | `drv/lpc55-i2c`, `drv/lpc55-spi{,-server}` | `drv/rp235x-i2c`, `drv/rp235x-spi{,-server}` |
| Flash/update | `drv/lpc55-update-server`, `drv/lpc55-flash` | `drv/rp235x-update-server`, `drv/rp235x-flash` |
| IDL | `idl/lpc55-{pins,update}.idol` | `idl/rp235x-{pins,update}.idol` |
| Board map | `boards/lpcxpresso55s69.toml` | `boards/pi-pico-2.toml` |
| App | `app/lpc55xpresso/{app.toml,src/main.rs,build.rs}` | `app/demo-pi-pico-2/` |

Key already-true facts (verified in-tree):
- `thumbv8m.main-none-eabihf` is already in `rust-toolchain.toml`.
- `build/xtask/src/config.rs` already maps that target → 32-byte MPU alignment.
- `build/kernel-link.x:65-84` already reserves a `.header` section right after the
  vector table, sized by `_HUBRIS_IMAGE_HEADER_SIZE` / `_HUBRIS_IMAGE_HEADER_ALIGN`
  (set by xtask). NOTE: this currently holds Hubris's *own* `abi::ImageHeader`
  (written by `update_image_header` in `dist.rs`), not the RP2350 block — the
  IMAGE_DEF needs its own section (see Phase 1). It does prove flash-resident
  metadata in the first bytes is a solved pattern.
- App `src/main.rs` pattern: take PACs → bring up clocks → `kern::startup::start_kernel(cycles_per_ms*1000)`.
- `boards/<board>.toml` `[probe-rs] chip-name` feeds `flash.rs`; the section is
  already tolerated-absent for new chips, so picotool-only works without it.

---

## Plan

### Phase 0 — Prerequisites, reference docs & dependencies
- **First action on approval: populate a local research dir** `docs/rp2350-research/`
  (git-ignored) with the authoritative reference material so register addresses,
  the IMAGE_DEF byte layout, and clock sequences are never guessed:
  - RP2350 datasheet — `https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf`
    (Ch.2 memory map, Ch.3 cores/MPU, Ch.5 Bootrom incl. §5.9 IMAGE_DEF, debug chapter)
  - Pico 2 board datasheet — `https://datasheets.raspberrypi.com/pico/pico-2-datasheet.pdf`
  - Hardware Design with RP2350 — `https://datasheets.raspberrypi.com/rp2350/hardware-design-with-rp2350.pdf`
  - RP2350 security white paper (for the out-of-scope secure-boot notes)
  - Cross-check IMAGE_DEF bitfields against pico-sdk `picobin.h` and the
    `rp235x-hal` `block` module (datasheet §5.9.3.1 has a documented typo).
  - Record the SVD-derived peripheral base addresses (from `rp235x-pac`) into a
    short `addresses.md` in that dir, used to author `chip.toml`.
- Add `rp235x-pac` (svd2rust PAC, `rt` feature for vector table/`device.x`) to the
  workspace `Cargo.toml`. Pull `rp235x-hal` **only as a reference** for the
  `block::ImageDef` bit layout and clock sequence — prefer minimal direct register
  access in our own crates to match Hubris style (don't take a HAL dependency into
  the kernel path).
- Install `picotool` (`cargo binstall picotool`), Raspberry Pi's **OpenOCD fork**
  (`github.com/raspberrypi/openocd` — upstream RP2350 support lags), and a recent
  `probe-rs` (≥0.27) for RTT. Document the debug rig in the app README.
- Confirm `thumbv8m.main-none-eabihf` toolchain target installs.

### Phase 1 — Boot bring-up (the high-risk core)
Goal: kernel + `jefe` + `idle` run on hardware; confirm over SWD (OpenOCD) that
tasks are alive. This is the milestone PR #2210 reached — use it as the oracle.

- `chips/rp235x/memory.toml`: XIP flash region at the QSPI XIP base (`0x10000000`),
  4 MB; SRAM main region at `0x20000000` (520 KB). **Single core-0 kernel/MSP stack**
  placed in non-striped **SRAM8** (`0x20080000`); **core 1 stays in reset** (no
  second stack). No `stage0`/DICE regions (those are LPC55-specific). Reserve a
  **run slot + a separate QSPI staging region** (single-slot updates, decision #3);
  A/B deferred.
- `chips/rp235x/chip.toml`: start minimal — the peripherals Phase 1/2 touch
  (RESETS, CLOCKS, XOSC, PLL_SYS/PLL_USB, IO_BANK0, PADS_BANK0, SIO) **plus the
  access/reset/power-gating blocks `ACCESSCTRL`, `WATCHDOG`, `POWMAN`**. IRQs: model
  real sources **0–45** (`46–51` = `SPAREIRQ`, expose only on demand — locked
  decision #6). Grow it per driver.
- `lib/rp235x-romapi`: minimal boot-ROM shim resolving entries via `rom_func_lookup`
  (datasheet §5.4.1) — needed by reboot/flash paths; **no fixed offsets** (decision #4).
- `lib/rp235x-startup`: clock bring-up — start XOSC (12 MHz crystal), configure
  `pll_sys` (≤150 MHz), switch `clk_ref`/`clk_sys` off ROSC, enable `clk_peri`;
  de-assert RESETS for the blocks we use and poll `RESETS_DONE`. Return
  `cycles_per_ms`. (Boot ROM already set up XIP, so no boot2/QMI code needed on the
  normal path.)
- **`IMAGE_DEF` emission (design LOCKED — decision #2, mechanism from PR #2210 which
  boots):** add a new `.image_def` linker section in `build/kernel-link.x`
  *immediately after* `.header` (so the `ImageHeader` stays at its existing offset,
  untouched), and extend `_stext` by `SIZEOF(.image_def)`. The block is a
  `#[link_section=".image_def"] #[used] static [u32; 5]` in the app crate, pulled by
  `KEEP(*(.image_def))` — **no `dist.rs` post-patch needed** for the block (dist.rs
  still writes only the Hubris `ImageHeader`). Add PR #2210's 4 ASSERTs to
  `kernel-link.x` (≤4 KiB from vector-table start; vector+header+image_def ≤0x1000;
  `SIZEOF(.image_def)==20`; precedes `.text`). The 20 bytes are the LE sequence
  `0xffffded3, 0x10210142, 0x000001ff, 0x00000000, 0xab123579` — **datasheet-verified
  (§5.9.5.1) and byte-for-byte identical to PR #2210's working image**
  (`docs/rp2350-research/findings.md` §1). **Regression-test host-side** (`cargo test`)
  against this golden vector, re-checked by `picotool info`.
- `app/demo-pi-pico-2/`: `app.toml` (target `thumbv8m.main-none-eabihf`, `chip =
  "../../chips/rp235x"`, `board = "pi-pico-2"`, tasks = jefe + idle), `src/main.rs`
  (clocks → `start_kernel`), `build.rs` (`expose_target_board`, `expose_m_profile`).
- `boards/pi-pico-2.toml`: omit `[probe-rs]` for now (picotool path); add chip-name
  in Phase 5.
- **Flash via picotool** (`picotool load -x build/.../final.elf`, or a UF2 with family
  `0xe48bff59`). **Debug via a documented MANUAL path** (fixes review #4). **Preferred
  rig: `probe-rs` 0.31** — it supports RP2350 natively (`probe-rs attach`/`gdb`, RTT),
  and **upstream OpenOCD 0.12 has NO `rp2350.cfg`** (verified — only rp2040). Only use
  OpenOCD if you build the **Raspberry Pi fork** (`raspberrypi/openocd`, ships
  `target/rp2350.cfg`), then `arm-none-eabi-gdb final.elf` → `target extended-remote
  :3333` → `monitor reset halt`. **Do NOT use `cargo xtask gdb`/`flash` yet** — they
  delegate to the external `humility` (`build/xtask/src/main.rs:458`), no RP2350 until
  Phase 5a.

### Phase 2 — Core drivers (usable demo)
- `build/rp235xpins` + `idl/rp235x-pins.idol`: GPIO pin-config codegen analogous to
  `build/lpc55pins` (IO_BANK0 FUNCSEL + PADS_BANK0). **Bake in the E9 workaround**:
  prefer internal pull-ups, expose a "momentary input-enable" read helper, document
  pull-down avoidance.
- `drv/rp235x-sys{,-api}`: **RESETS-only** peripheral-reset/enable server (RESETS is
  Secure/any-priv, so an unprivileged task may drive it). **NOT a clock server** —
  unlike `drv-lpc55-syscon`, CLOCKS/PLL/XOSC/POWMAN are ACCESSCTRL Privileged-only
  (decision #8), so clock/PLL/power bring-up lives in the **privileged
  `lib/rp235x-startup`**, not here. If runtime clock control is ever required, either
  (a) widen ACCESSCTRL for CLOCKS in startup, or (b) expose a narrow kipc/privileged
  path — decide then, don't clone the LPC55 all-in-one syscon pattern.
- `drv/rp235x-gpio{,-api}`: GPIO server (IO_BANK0/PADS_BANK0/SIO), pin IRQs.
- `drv/rp235x-uart` (+ `lib/rp235x-uart` if useful): PL011 UART0 driver, polling
  first. Add `user_leds` + a serial demo to `app/demo-pi-pico-2` to prove
  blink + `println`-over-UART end to end.

### Phase 3 — Extended peripherals
- `drv/rp235x-spi{,-server}` and `drv/rp235x-i2c` (RP2350 SPI/I2C blocks), wired
  through the standard Hubris SPI/I2C server traits.
- Map all peripheral IRQs in `chips/rp235x/chip.toml`; wire `interrupts = {...}` in
  `app.toml` exactly as LPC55 does (chip.toml IRQ → task notification). *Mechanism
  verified via CodeGraph:* `lib/toml-task/src/lib.rs` — `interrupts = {"periph.irq" =
  "notif-name"}`, where the notification's bit is its index in the task's
  `notifications` list. **Hard limit: ≤32 notifications per task** (`notification_bit`
  bails past 31). RP2350's ≤46 IRQs are fine as long as no single task claims >32.
  (Note: the mapped notification name must end in `-irq`, e.g. `"uart0.irq" =
  "usart-irq"` — `dist.rs` validates this.)

### Phase 4 — Flash management & update (full-port completion)
- `drv/rp235x-flash` + `drv/rp235x-update-server` + `idl/rp235x-update.idol`:
  **single-slot** image update (committed — locked decision #3; A/B is explicitly
  future, no `PARTITION_TABLE` now). Flash erase/program goes through the
  `lib/rp235x-romapi` `rom_func_lookup` entry points (datasheet §5.4.1), and must
  **claim boot lock `LOCK_FLASH_OP` (BOOTLOCK1)** around the operation (§5.4.4 —
  the ROM provides the mutex mechanism; `LOCK_ENABLE`/BOOTLOCK7 is off by default).
  The update-server also needs **QMI**, which is ACCESSCTRL Privileged-only ⇒ grant it
  Secure-Unprivileged in privileged startup, or run the flash op privileged (decision
  #8).
- **XIP-safety (datasheet §5.4.4):** programming the QSPI in direct mode causes XIP
  reads to **bus-fault**. The flash erase/program routine and its immediate caller
  must be **RAM-resident** (`#[link_section=".data"]`/RAM text), run with **interrupts
  masked and other tasks quiesced**, and restore XIP before returning. Design this
  execution model now — it is a design constraint, not an implementation detail.
  **⚠ No in-tree precedent for erase-while-executing (CodeGraph + review-verified):**
  every existing flash updater programs flash *separate from the execution source* —
  `drv/ignition-flash` (external SPI IC), `drv/auxflash-server` (slot-addressed aux
  QSPI), `drv/lpc55-update-server` (rejects updating the running image, writes the
  other slot), and `drv/stm32h7-update-server` (updates alternate `bank2`). Erasing the
  flash you are executing from is **novel work for this port** — no copy-paste pattern.
- **Staging-region access = `extern-regions`** (corrected after review — *not*
  `sections`, which only places a section). Reserve the QSPI staging area as a named
  region in `memory.toml`, expose it to the update-server via `extern-regions =
  ["staging"]`, and read the generated `__REGION_STAGING_BASE/END` bounds — exactly
  the pattern `drv/stm32h7-update-server` uses for `bank2`
  (`app/gimletlet/base-gimletlet2.toml` `extern-regions = ["bank2"]` →
  `__REGION_BANK2_BASE/END`, `drv/stm32h7-update-server/src/main.rs:43`). **This is the
  closest structural precedent to clone** for the slot layout; only the RAM-resident
  erase-while-XIP execution is net-new.
- **Single-slot staging model (design constraint — decided now):** you cannot rewrite
  the running XIP slot in place, and 520 KB SRAM cannot hold a whole image. So the
  incoming image is **streamed block-by-block from the host into a dedicated QSPI
  *staging region* distinct from the run slot** (there is room in 4 MB), verified
  there (block loop + `ImageHeader`), then written to the run slot by the RAM-resident
  routine, then **reboot** via the romapi. This update is **non-atomic with no
  rollback** — acceptable for bring-up; **robust A/B is the future upgrade.** The
  staging region must be reserved in `memory.toml` at Phase 1 (see decision #3).
- A/B, multi-image `memory.toml`, and `PARTITION_TABLE` are a separate future phase.

### Phase 5 — humility / probe-rs native path + automated tests
**5a — humility/probe-rs native path (EXTERNAL integration, not in-tree code —
fixes review #5):**
- Hubris does **not** vendor `probe-rs`; `humility` is a separate external binary.
  Deliverable here is therefore: **document/require an external `humility` built
  against `probe-rs` ≥0.27** (ADIv6 + RP235x flash algos), optionally submit upstream
  PRs — NOT a code phase inside this repo.
- Add `[probe-rs] chip-name = "<verified RP235x target string>"` to
  `boards/pi-pico-2.toml` (verify the exact string against the probe-rs target def —
  `"RP235x"` is a guess). Then `cargo xtask flash` / `humility tasks`/`dump`/`ringbuf`
  work natively.
- Decision point: keep picotool+OpenOCD as the supported flow, switch to humility, or
  support both.

**5b — on-target test harness (`test/tests-pi-pico-2/`):**
- Clone `test/tests-lpc55xpresso/` (`test-runner` + `test-suite` + `test-assist` +
  `task-hiffy`). **Replace the result transport:** the stock harness emits over
  ITM/SWO, which RP2350/Pico 2 cannot do (no SWO pin, TPIU pins not broken out).
  Use **RTT** (probe-rs) or a fixed RAM results-buffer read over SWD instead.
  This transport swap is the main non-mechanical work in this sub-phase.
- Add `task/hiffy/src/rp235x.rs` (analogue of `lpc55.rs`) so `humility hiffy` can
  drive RP2350 GPIO/I2C/SPI. **Only usable post-G6** (needs native humility) — it is a
  convenience, not a bring-up dependency — and
  the test suite.

---

## Debug & Test strategy (how we prove each step works as we build)

The hard part of a from-scratch port is the early stretch with **no UART, no LED,
no printf**. We climb a ladder of increasingly capable feedback, and every phase has
an explicit "how do we know it works" gate. Key RP2350 constraint discovered in
research: **the M33 has ITM but RP2350 exposes NO SWO pin** and the parallel TPIU
trace pins are **not broken out on the Pico 2 header** — so Hubris's stock
ITM/SWO test-result transport does **not** work here. We replace it with RTT /
SWD-read buffers (a real porting item, see Phase 5b).

### The debug rig (cheapest working setup)
- A **second Pico (~$4) flashed with `debugprobe_on_pico.uf2`** (or the official
  Raspberry Pi **Debug Probe**, VID `0x2e8a`/PID `0x000c` — update its firmware
  first, factory firmware predates RP2350). 3 wires to the Pico 2 JST-SH header:
  **SWCLK / GND / SWDIO**, short cable, adapter speed ~5 MHz.
- **Host debug software — prefer `probe-rs` 0.31** (installed; RP2350-native:
  `probe-rs attach`, `probe-rs gdb`, RTT — no cfg needed). **Upstream OpenOCD 0.12 does
  NOT support RP2350** (no `rp2350.cfg`, verified); use it only via the **Raspberry Pi
  fork** (`openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg` → `arm-none-eabi-gdb`).
  RP2350 uses **ADIv6** — the reason a stale probe-rs/upstream-OpenOCD fails to attach;
  probe-rs ≥0.27 (we have 0.31) handles it.

### The feedback ladder
| Tier | Capability | Tooling | Available from |
|---|---|---|---|
| 0 | **Host-side, no hardware** | `cargo xtask dist` (builds/links), `cargo test` for pure-logic crates, **`picotool info -a <final.elf>`** validates the IMAGE_DEF block (`final.elf` is inside the dist archive as `img/final.elf` — extract it, or run picotool on the archive's ELF) | Phase 0/1, in CI |
| 1 | **First sign of life** | SWD+GDB: confirm `$pc` is in our `.text` (not boot-looping/USB), breakpoints at `Reset`→`main`→`start_kernel`; **bit-bang a GPIO/LED high in `lib/rp235x-startup`** as a heartbeat before any driver | Phase 1 |
| 2 | **Kernel liveness** | `ringbuf` (`lib/ringbuf`) entries read over SWD via **OpenOCD `mdw`/GDB `x`** (NOT humility — unavailable until G6); watch kernel `TICKS` increment; read `KERNEL_HAS_FAILED`/`KERNEL_EPITAPH` (`sys/kern/src/fail.rs`); inspect task table | Phase 1 |
| 3 | **Real logging** | UART0 console; **RTT** as a no-pin printf-over-SWD — read via **OpenOCD `rtt server`** (no probe-rs/humility needed, so available during driver bring-up) | Phase 2 |
| 4 | **Live driver poking** | **`task-hiffy` + `humility hiffy`** — call IPC ops, GPIO/I2C/SPI live without writing test tasks; needs `task/hiffy/src/rp235x.rs`. **Requires humility ⇒ only after G6** — NOT a bring-up dependency | post-G6 |
| 5 | **Automated regression** | on-target harness `test/tests-pi-pico-2/` (clone `test/tests-lpc55xpresso/`): `test-runner`+`test-suite`+`test-assist`; **result transport swapped from ITM/SWO to RTT/SWD-buffer** | Phase 5b |

### Per-phase verification gates
See the single authoritative **Gates (G0–G6)** section below — those are the
go/no-go checkpoints, one source of truth. (This avoids a second, drifting gate list;
note that `humility` is unavailable until **G6**, so no earlier gate may depend on it.)

### Panic/fault surfacing
Task panics → `sys_panic` (`sys/userlib`) → kernel fault → `jefe` (`task/jefe`)
restarts and records; kernel panics land in `KERNEL_EPITAPH`. All readable over SWD
even with no console — so a crash during bring-up is diagnosable from GDB alone.

## Files to create (representative, not exhaustive)
- `chips/rp235x/{chip.toml, memory.toml, openocd.cfg, openocd.gdb}`
- `lib/rp235x-startup/` (+ `lib/rp235x-uart/` if split)
- `lib/rp235x-romapi/` — boot-ROM shim via `rom_func_lookup` (Phase 1; decision #4)
- `build/rp235xpins/`
- `drv/rp235x-{sys,sys-api,gpio,gpio-api,uart,spi,spi-server,i2c,flash,update-server,update-api}/`
- `idl/rp235x-{pins,update}.idol`
- `app/demo-pi-pico-2/{app.toml, Cargo.toml, build.rs, src/main.rs, README.md}`
- `boards/pi-pico-2.toml`
- `test/tests-pi-pico-2/` (app.toml + src) and `task/hiffy/src/rp235x.rs` (Phase 5b)
- `docs/rp2350-research/` (git-ignored): datasheets + `addresses.md` (Phase 0)
- Workspace edits: add new crates to `Cargo.toml` members; add `rp235x-pac` dep.

## Files we WILL modify (beyond new files)
- **`build/xtask/src/dist.rs`** — extend `update_image_header` (~line 1895) to also
  emit the RP2350 block loop after the `ImageHeader`; size `.header` accordingly.
- **`build/kernel-link.x`** — enlarge/annotate `.header` reservation for the block.

## Files to read but NOT modify
- `sys/kern/src/arch/arm_m.rs`, `sys/kern/src/startup.rs` — confirm single-core M33
  path covers us (and that core 1 stays parked).
- `sys/abi/src/lib.rs:541`, `lib/lpc55-rot-startup/src/images.rs:247`,
  `drv/lpc55-update-server/src/main.rs:1224` — the `ImageHeader` ABI/offset our
  layout must preserve.

---

## Verification (end to end)
End-to-end verification is expressed as the **Gates** below — each phase must pass
its gate before the next begins. Gate↔phase map: **G0**=Phase 0, **G1**=Phase 1
(host/CI), **G2–G3**=Phase 1 (hardware), **G4**=Phases 2–3, **G5**=Phase 4,
**G6**=Phase 5.

## Gates (hard go/no-go checkpoints — do not proceed past a failed gate)

Each gate is a binary, verifiable condition. Earlier gates are cheaper (host-side,
in CI) so the riskiest things fail fast before hardware time.

- **G0 — Design lock (before ANY implementation).** All seven *Locked decisions*
  recorded; `docs/rp2350-research/` populated (datasheets + `addresses.md` from the
  SVD); the first-4-KiB byte map drawn (vector table → `ImageHeader` → block loop);
  single-slot + staging-region layout fixed; **`ACCESSCTRL` reset defaults confirmed**
  (an unsigned Arm-S image has the bus access it needs — resolve here, not later).
  **Gate = the byte map, the ACCESSCTRL answer, and the decisions are written down.**
  No code before this.
- **G1 — Host-side image format (CI, NO hardware) — the primary gate.**
  `cargo xtask dist app/demo-pi-pico-2/app.toml` links, THEN extract `img/final.elf` from
  the dist archive (`target/pi-pico2/dist/<image>/build-*.zip`) and run
  `picotool info -a img/final.elf` — it reports: valid block loop, image-type word =
  EXE/ARM-S/RP2350, block within the **first 4 KiB**, AND Hubris's `ImageHeader`
  present at its offset with magic `0x64CED6CA` (the kernel's `sys/kern/src/header.rs`
  `HEADER` static still resolves — this is the *generic* reader that binds us; the
  LPC55 fixed-offset readers are not in an RP2350 image). A host `cargo test` asserts
  the generated block bytes match the golden vector. **Wire G1 into CI.**
- **G2 — First light (hardware).** After `picotool load`, `probe-rs gdb` (or RPi-fork
  OpenOCD) + `arm-none-eabi-gdb` shows `$pc` inside our `.text` (not boot-looping to
  USB) and breakpoints hit `Reset → main → start_kernel`.
- **G3 — Kernel liveness (hardware).** `ringbuf` boot trace readable over SWD; kernel
  `TICKS` increments; `KERNEL_HAS_FAILED == 0`; jefe is scheduling.
- **G4 — Driver proof (per driver, NO humility — that arrives at G6).** LED blink
  (GPIO) AND UART0 banner AND an RTT log line (RTT read via **OpenOCD's `rtt server`**,
  which needs no probe-rs/humility); SPI MOSI→MISO loopback and I2C device ACK
  confirmed on a **logic analyzer** + manual-GDB memory poke. (`humility hiffy` is a
  post-G6 convenience, not a bring-up dependency.)
- **G5 — Update path (single-slot).** `update-server` streams an image into the QSPI
  **staging region**, verifies it there, then a **RAM-resident** routine (interrupts
  masked, tasks quiesced — XIP bus-faults during direct-mode programming, datasheet
  §5.4.4) writes it to the run slot via `rom_func_lookup` and the device **reboots
  into it**. Non-atomic, no rollback (bring-up scope). No A/B / `PARTITION_TABLE`.
- **G6 — Native tooling.** External humility+probe-rs (≥0.27, **verified** target
  string — not the `"RP235x"` placeholder) flashes and runs `humility tasks`/`dump`;
  `cargo xtask test test/tests-pi-pico-2/app.toml` reports pass/fail over the RTT/SWD
  transport.

### Open items to verify (do not hard-code until checked)
*(These are all **external** — outside this repo — so CodeGraph cannot close them; the
in-tree mechanics they'd otherwise be paired with are already CodeGraph-verified:
interrupt→notification wiring, region access, and the absence of a RAM-resident flash
precedent.)*
- Exact probe-rs/humility RP2350 **target string** (assumed `"RP235x"`). [verify]
- Whether Pico 2 truly leaves **no usable trace pin** broken out — confirm against the
  Pico 2 schematic (GPIO0–22 are exposed). The "no SWO" conclusion holds regardless;
  only the *rationale* needs confirming. [verify]
- (`ACCESSCTRL` defaults **RESOLVED at G0** — §10.6.2.1 verified: clocks/PLL/power/QMI
  are Privileged-only; see locked decision #8 and findings.md §4.)

## Repo conventions & CI gates (match these — verified from the repo)

Hubris's own coding/testing strategy, from `.github/workflows/{ci,build-boards}.yml`,
`CONTRIBUTING.md`, `rust-toolchain.toml`, and the `test/` framework. Every new file we
add must satisfy these or CI fails.

**Coding standards (CI-enforced):**
- **`#![no_std]` Rust, toolchain pinned `1.95.0`** (`rust-toolchain.toml`; targets
  incl. `thumbv8m.main-none-eabihf`). Don't bump the toolchain.
- **MPL-2.0 header on every source file** (the 3-line `// This Source Code Form …`
  block) — checked by skywalking-eyes. Put it on every new `.rs`.
- **`cargo fmt --all --check`** clean (default rustfmt).
- **`cargo clippy … -- --deny warnings`** clean — both `cargo xtask clippy <app.toml>`
  per board and `cargo clippy -pxtask`. Zero warnings.
- Correctness-biased flags come from `dist.rs` automatically (`-C overflow-checks=y`,
  stack-size emission, 32-byte MPU page) — don't fight them.
- Architecture idioms to follow: drivers are **tasks**; interfaces are **Idol `.idol`**
  (generated client/server); config is declarative in `app.toml`; diagnostics via
  **`ringbuf`** + counters; no ad-hoc globals.

**Testing tiers (mirror these):**
1. **Host unit tests** — `cargo test --workspace` runs in CI. Pure-logic code gets
   `#[cfg(test)]` tests. ⇒ **the IMAGE_DEF byte generator MUST have a host test**
   (golden vector) — this is our G1 regression guard, and it fits the existing tier.
2. **"Build is the test"** — `cargo xtask dist` validates memory allocation, task
   wiring, and IPC types at build time. **CI auto-discovers every `app.toml`** (via
   `build/gha-build-boards-matrix.py`), so once `app/demo-pi-pico-2/app.toml` lands,
   CI builds it on Linux **and Windows** — our G1 build must pass on both.
3. **On-target integration** — `test/tests-pi-pico-2/` follows the
   `test-runner`+`test-suite`+`test-assist` pattern (cases are `fn test_*()` via the
   `test_cases!` macro; failures panic; runner scans the task table). **Caveat CI does
   NOT run these** (no hardware in CI) — and the stock runner reports over ITM/SWO,
   which Pico 2 can't do ⇒ RTT/SWD transport swap (Phase 5b) is required, not optional.
4. **Interactive HIL** — `humility hiffy` (post-G6) for live driver poking.

**Process reality (`CONTRIBUTING.md`):** Oxide has ~zero FT engineers on Hubris and
"may not have bandwidth to review outside PRs"; they ask you to raise intent in
Discussions *first*. Combined with the general wariness of AI-authored submissions,
treat this as a **fork / out-of-tree effort** unless Oxide signals interest — which
keeps **exhubris** (the out-of-tree path) on the table if upstreaming isn't the goal.

## Key risks
- **IMAGE_DEF correctness** — **exact bytes now datasheet-verified** (§5.9.5.1, see
  findings.md); residual risk is only the *build-system integration* (does the block
  land <4 KiB after the real vector table + `ImageHeader`, emitted correctly by
  `dist.rs`). Still gated by G1/`picotool info`. Downgraded from "highest risk."
- **`ImageHeader` ABI collision** — the RP2350 block loop shares the first 4 KiB with
  Hubris's existing `ImageHeader`, which has fixed-offset readers. Mislaying either
  bricks boot OR breaks update/caboose. Mitigated by locking the byte map at G0 and
  asserting both in G1.
- **External humility/probe-rs** (Phase 5a) — not in this repo; treated as an external
  dependency, quarantined so it never blocks bring-up or driver work.
- **No SWO on RP2350** — Hubris's stock ITM/SWO test-result transport is unusable on
  Pico 2 (no SWO pin). The harness must be re-pointed at RTT or a SWD-read results
  buffer (Phase 5b). Bring-up is unaffected (uses `ringbuf`-over-SWD, GDB, picotool).
  *Caveat: the "TPIU pins not broken out" rationale is [SUSPECTED] — verify against
  the Pico 2 schematic; the conclusion stands either way.*
- **Boot-ROM via `rom_func_lookup`** — no fixed offsets; the `rp2350-romapi` shim must
  resolve correctly before any flash/reboot call (G5).
- **XIP bus-fault during flash programming** (datasheet §5.4.4) — programming the QSPI
  in direct mode bus-faults any XIP read, so the flash routine + caller must be
  RAM-resident with interrupts/tasks quiesced. Designed-in at Phase 4, gated by G5.
  This is the single nastiest RP2350-specific update hazard — **and CodeGraph confirms
  no in-tree flash driver has ever done erase-while-executing, so there is no pattern
  to lean on.** Highest-effort item in Phase 4.
- **E9 GPIO erratum** — must be handled in the GPIO/pin layer, not papered over.
- **Banked SRAM placement** — stacks in SRAM8/9 to avoid crossbar contention.
