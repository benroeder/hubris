# RP2350 G0 findings — verified against the primary datasheet

Source: `rp2350-datasheet.pdf` (1380 pp, downloaded 2026-07-01). Section refs are to
that PDF. This file replaces the previously *relayed* (second-hand) silicon claims in
`plans/rp2350-port.md` with primary-source-verified facts.

## 1. IMAGE_DEF — the exact minimum Arm block (§5.9.5.1) — RESOLVED (was highest risk)
Minimum valid Arm IMAGE_DEF, assuming `CRIT1.SECURE_BOOT_ENABLE` clear = **20 bytes /
5 little-endian words**, verbatim:

| Word | LE value | Meaning |
|---|---|---|
| 0 | `0xffffded3` | `PICOBIN_BLOCK_MARKER_START` |
| 1 | `0x10210142` | IMAGE_TYPE item: `0x42`=item_type IMAGE_TYPE(1-byte-size), `0x01`=1 word, `0x1021`=EXE \| SECURITY(S) \| CPU(Arm) \| CHIP(RP2350) |
| 2 | `0x000001ff` | LAST item: `0xff`(2-byte-size, BLOCK_ITEM_LAST), `0x0001`=size, `0x00` pad |
| 3 | `0x00000000` | link = self (single-block loop) |
| 4 | `0xab123579` | `PICOBIN_BLOCK_MARKER_END` |

- Must appear **within the first 4 kB** of the flash image.
- With no explicit entry item, the bootrom assumes a **Cortex-M vector table at image
  start** and enters via reset handler (+4) / initial SP (+0). ⇒ our layout (vector
  table first, then ImageHeader, then this block) is correct.
- Block format (§5.9.1): header `0xffffded3`, items, 32-bit relative LINK, footer
  `0xab123579`; all LE; word-aligned; IMAGE_DEF blocks capped at **384 bytes**.
- Cross-check constants against SDK `picobin.h`. (Note: datasheet prints the chip
  constant as "RP23500" — a doc typo; the real packed value is `0x1021`.)

## 2. No boot2; bootrom sets up XIP (§5.9.5) — RESOLVED
> "Unlike RP2040, there is no requirement for flash binaries to have a checksummed
> boot2 flash setup function at flash address 0. The RP2350 bootrom performs a simple
> best-effort XIP setup during flash scanning, and a flash-resident program can
> continue executing in this state." ⇒ **no boot2 code needed from us.**

## 3. Flash-vs-XIP hazard + boot locks (§5.4.4) — RESOLVED, with new detail
> "Using the QSPI direct-mode interface to program the flash causes XIP access to
> return a bus fault."
- Confirms the RAM-resident, interrupts-masked flash routine requirement.
- **New:** the bootrom provides **boot locks** (boot RAM `BOOTLOCK0..7`) for mutual
  exclusion. `LOCK_FLASH_OP` (0x1) guards QSPI direct mode; `LOCK_ENABLE` (0x7) turns
  on lock-checking and is **off by default** (APIs usable without setup). Our flash
  routine should claim `LOCK_FLASH_OP` around the operation.

## 4. ACCESSCTRL default permissions (§10.6.2.1) — RESOLVED, with architecture impact
Hubris boots **Secure**; kernel/pre-main is **Privileged**, tasks are **Unprivileged**.
- **Fully open (any security/privilege):** ROM, **XIP, SRAM**, SYSINFO.
- **Secure + *Privileged* only:** **CLOCKS, XOSC, ROSC, PLL_SYS/PLL_USB, PSM, WATCHDOG,
  POWMAN, QMI, XIP control, SYSCFG, Tick generators**, SHA-256, TRNG (POWMAN/clocks
  also forbid DMA by default).
- **Everything else** (UART, SPI, I2C, IO_BANK0, PADS_BANK0, RESETS, …): **Secure only,
  *any* privilege.**

**Impact on the port (new design constraint):**
- GPIO/UART/SPI/I2C driver *tasks* (unprivileged, Secure) can touch their peripherals
  **out of the box** — no ACCESSCTRL change needed. RESETS too.
- **CLOCKS/PLL/XOSC/PSM/WATCHDOG/POWMAN/QMI are Privileged-only** ⇒ an unprivileged
  `drv-rp2350-sys` task **cannot** touch them by default. Two options:
  1. Do all clock/PLL/power/QMI bring-up in the **privileged pre-main startup**
     (`lib/rp2350-startup`) — matches the existing Hubris pattern; **preferred.**
  2. If a *runtime* task must touch them, the privileged startup must **widen
     ACCESSCTRL** (grant Secure-Unprivileged) for those specific blocks.
- The flash `update-server` needs **QMI** (Privileged-only) ⇒ either grant SU on QMI in
  startup, or perform the flash op from a privileged context. Decide in Phase 4.

## 5. IRQ table (§3.2 / lines ~21494) — CONFIRMED
- IRQ **0–45** are real sources; **46–51 = `SPAREIRQ_IRQ_0..5`, hardwired to zero**
  (never fire; usable as software-pended IRQs). Matches the plan.

## 6. Address map (Table 8) — for `chips/rp2350/chip.toml` + `memory.toml`
```
XIP_BASE          0x10000000   (4 MB external QSPI on Pico 2)
SRAM_BASE         0x20000000   (striped, 512 KB, SRAM0..7)
SRAM_STRIPED_END  0x20080000
SRAM8_BASE        0x20080000   (4 KB, non-striped)  <- core-0 kernel stack
SRAM9_BASE        0x20081000   (4 KB, non-striped)
SRAM_END          0x20082000
SYSINFO_BASE      0x40000000     SYSCFG_BASE     0x40008000
CLOCKS_BASE       0x40010000     PSM_BASE        0x40018000
RESETS_BASE       0x40020000     IO_BANK0_BASE   0x40028000
IO_QSPI_BASE      0x40030000     PADS_BANK0_BASE 0x40038000
PADS_QSPI_BASE    0x40040000     XOSC_BASE       0x40048000
PLL_SYS_BASE      0x40050000     PLL_USB_BASE    0x40058000
ACCESSCTRL_BASE   0x40060000     BUSCTRL_BASE    0x40068000
```
(UART/SPI/I2C/TIMER/ADC/WATCHDOG/POWMAN/QMI bases: extract from the same table +
`rp235x-pac` SVD when authoring the full `chip.toml`.)

## 7. Pico 2 debug note (§ line ~101371)
SWD (SWCLK/SWDIO) is on the 3-pin debug header, not the edge GPIOs; GPIO0/1 route to a
Debug Probe UART. (Confirms the manual-OpenOCD bring-up path.)

## Still open (external — cannot verify from the datasheet)
- probe-rs/humility RP2350 **target string** (assumed `"RP235x"`) — resolve when the
  external humility/probe-rs fork is built (G6).
- (PR #2210 boot claim RESOLVED — see §8 below.)

## 8. PR #2210 oracle (read 2026-07-01) — CONFIRMED it boots
- Author shows `cargo xtask humility ... tasks`: **jefe (0) + idle (1) RUNNING**,
  "runs and passes tests", attached via CMSIS-DAP. 358 add / 13 del, 20 files, OPEN,
  currently CONFLICTING (needs rebase). Branch `bmatt/rp2350`.
- **Pairs with an unsubmitted humility+probe-rs fork** (`thenewwazoo/humility
  bmatt/update-probe-rs`, humility #530) — confirms our Phase 5a "external
  humility/probe-rs" scoping exactly.
- **IMAGE_DEF mechanism (adopt this):** a new `.image_def` linker section AFTER
  `.header`; `_stext += SIZEOF(.image_def)`; block is a `#[link_section=".image_def"]
  #[used] static [u32;5]` in the app; `KEEP(*(.image_def))`. 4 build ASSERTs
  (≤4 KiB, vector+header+image_def ≤0x1000, size==20, precedes .text). NOT a dist.rs
  post-patch. Its 5 words == our datasheet bytes exactly (independent cross-check).
- **Canonical names (align to these):** `chips/rp235x`, `chips/rp235x/memory-pico-2.toml`,
  `boards/pi-pico-2.toml`, `app/demo-pi-pico-2`, PAC `rp235x-pac`,
  `chips/rp235x/openocd.{cfg,gdb}`.
- **Memory (minimal):** flash `0x10000000` size `0x400000`; single ram `0x20000000`
  size `0x82000` (520 KB). The oracle does NOT split SRAM8/9 stacks or reserve a
  staging region — those are our enhancements (for update-server, Phase 4).
- **Startup is a hack in the oracle:** `main.rs` only deasserts IO_BANK0 reset and
  hardcodes cycles_per_ms (6000 on ROSC / 48000 if debugger reconfigured), with
  `// TODO fix/update this for RP2350`. ⇒ our `lib/rp2350-startup` with real XOSC+PLL
  bring-up (running privileged, per ACCESSCTRL §4) is genuine additional work.

## Toolchain installed (2026-07-01) — COMPLETE
- picotool **v2.2.0** ✓ (host-side G1 gate ready).
- probe-rs **0.31.0** ✓ (RP2350 native — use for attach/RTT/gdb; leaner than OpenOCD).
- arm-none-eabi-gdb **15.2.Rel1** ✓ (`gcc-arm-embedded` cask, on PATH).
- openocd **0.12.0** ✓ but **upstream has no `rp2350.cfg`** (only rp2040) ⇒ for
  OpenOCD-based flashing use the RPi fork; otherwise prefer probe-rs.
