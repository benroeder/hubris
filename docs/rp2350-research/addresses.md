# RP2350 peripheral base addresses + IRQ map (for chips/rp235x/chip.toml)

Source: RP2350 datasheet Table 8 (address map) and Table 95 (§3.2 interrupts).

## VALIDATION (how these were checked — do not trust blindly)
Every base address and IRQ number below was **cross-checked against two independent
sources and they agree exactly**: (1) the datasheet PDF Table 8/95 (grep-extracted,
not typed from memory), and (2) the official `RP2350.svd` from `raspberrypi/pico-sdk`
(parsed programmatically 2026-07-01, `RP2350.svd` in this dir). Base addresses,
UART0=33/UART1=34/SPI0=31/SPI1=32/I2C0=36/I2C1=37/IO_BANK0=21/CLOCKS=30/TIMER0_0..3=0..3
all match. Vector-table length confirmed from the SVD: max IRQ = 51 ⇒ 52 entries ⇒
`(16+52)*4 = 0x110`.
- **CORRECTION:** an earlier draft claimed a uniform `0x4000` peripheral window — that
  was wrong (made up). Real `addressBlock` sizes from the SVD vary: RESETS=12 B,
  CLOCKS=212 B, PADS_BANK0=204 B, IO_BANK0=800 B, QMI=84 B, I2C0=256 B, SIO=488 B,
  UART0/SPI0=4096 B. Peripherals are *spaced* `0x8000` apart. For `chip.toml`, size each
  peripheral to cover its registers (round to `0x1000` where convenient), not `0x4000`.
- **The ACCESSCTRL priv/open column below is my derivation** from the verbatim
  §10.6.2.1 list (re-checked against the extracted text) — the SVD does not encode it,
  so it is single-source (datasheet only). The final proof for all of this is **G1**
  (`picotool info` + does it link/boot).

## Memory regions (chips/rp235x/memory-pico-2.toml)
| Region | Base | Size | Notes |
|---|---|---|---|
| XIP flash | `0x10000000` | `0x400000` (4 MB) | external QSPI, Pico 2 specific; execute+read |
| SRAM (striped) | `0x20000000` | `0x80000` (512 KB) | SRAM0–7, main code/data |
| SRAM8 | `0x20080000` | `0x1000` (4 KB) | non-striped — **core-0 kernel/MSP stack** |
| SRAM9 | `0x20081000` | `0x1000` (4 KB) | non-striped — spare/scratch |
| (SRAM end) | `0x20082000` | — | total 520 KB |

Alt XIP windows (for cache control / non-cached access): `0x14000000`
(NOCACHE_NOALLOC), `0x18000000` (MAINTENANCE), `0x1c000000` (NOCACHE_NOTRANSLATE).

## Peripheral bases (APB/AHB)
| Peripheral | Base | Priv-only? (ACCESSCTRL §10.6.2.1) |
|---|---|---|
| SYSINFO | `0x40000000` | open |
| SYSCFG | `0x40008000` | **Priv** |
| CLOCKS | `0x40010000` | **Priv** |
| PSM | `0x40018000` | **Priv** |
| RESETS | `0x40020000` | Secure/any-priv |
| IO_BANK0 | `0x40028000` | Secure/any-priv |
| IO_QSPI | `0x40030000` | Secure/any-priv |
| PADS_BANK0 | `0x40038000` | Secure/any-priv |
| PADS_QSPI | `0x40040000` | Secure/any-priv |
| XOSC | `0x40048000` | **Priv** |
| PLL_SYS | `0x40050000` | **Priv** |
| PLL_USB | `0x40058000` | **Priv** |
| ACCESSCTRL | `0x40060000` | world-read, Priv-write |
| BUSCTRL | `0x40068000` | Secure/any-priv |
| UART0 | `0x40070000` | Secure/any-priv |
| UART1 | `0x40078000` | Secure/any-priv |
| SPI0 | `0x40080000` | Secure/any-priv |
| SPI1 | `0x40088000` | Secure/any-priv |
| I2C0 | `0x40090000` | Secure/any-priv |
| I2C1 | `0x40098000` | Secure/any-priv |
| ADC | `0x400a0000` | Secure/any-priv |
| PWM | `0x400a8000` | Secure/any-priv |
| TIMER0 | `0x400b0000` | Secure/any-priv |
| TIMER1 | `0x400b8000` | Secure/any-priv |
| HSTX_CTRL | `0x400c0000` | Secure/any-priv |
| XIP_CTRL | `0x400c8000` | **Priv** |
| XIP_QMI | `0x400d0000` | **Priv** (flash update-server needs this) |
| WATCHDOG | `0x400d8000` | **Priv** |
| BOOTRAM | `0x400e0000` | Secure-only (holds BOOTLOCKs) |
| ROSC | `0x400e8000` | **Priv** |
| TRNG | `0x400f0000` | **Priv** |
| SHA256 | `0x400f8000` | **Priv** |
| POWMAN | `0x40100000` | **Priv**, DMA-forbidden |
| TICKS | `0x40108000` | **Priv** |
| OTP | `0x40120000` | — |
| OTP_DATA | `0x40130000` | — |
| CORESIGHT | `0x40140000` | — |
| SIO (core-local) | `0xd0000000` | internally banked S/NS |
| Cortex-M33 PPB | `0xe0000000` | internal (NVIC/SysTick/MPU/SAU) |

## Interrupt map (Table 95) — model 0–45; 46–51 are SPAREIRQ (hardwired 0)
```
 0 TIMER0_IRQ_0    12 DMA_IRQ_2        24 IO_IRQ_QSPI_NS   36 I2C0_IRQ
 1 TIMER0_IRQ_1    13 DMA_IRQ_3        25 SIO_IRQ_FIFO     37 I2C1_IRQ
 2 TIMER0_IRQ_2    14 USBCTRL_IRQ      26 SIO_IRQ_BELL     38 OTP_IRQ
 3 TIMER0_IRQ_3    15 PIO0_IRQ_0       27 SIO_IRQ_FIFO_NS  39 TRNG_IRQ
 4 TIMER1_IRQ_0    16 PIO0_IRQ_1       28 SIO_IRQ_BELL_NS  40 PROC0_IRQ_CTI
 5 TIMER1_IRQ_1    17 PIO1_IRQ_0       29 SIO_IRQ_MTIMECMP 41 PROC1_IRQ_CTI
 6 TIMER1_IRQ_2    18 PIO1_IRQ_1       30 CLOCKS_IRQ       42 PLL_SYS_IRQ
 7 TIMER1_IRQ_3    19 PIO2_IRQ_0       31 SPI0_IRQ         43 PLL_USB_IRQ
 8 PWM_IRQ_WRAP_0  20 PIO2_IRQ_1       32 SPI1_IRQ         44 POWMAN_IRQ_POW
 9 PWM_IRQ_WRAP_1  21 IO_IRQ_BANK0     33 UART0_IRQ        45 POWMAN_IRQ_TIMER
10 DMA_IRQ_0       22 IO_IRQ_BANK0_NS  34 UART1_IRQ        46-51 SPAREIRQ_IRQ_0..5
11 DMA_IRQ_1       23 IO_IRQ_QSPI      35 ADC_IRQ_FIFO
```
Driver-relevant: GPIO=21 (IO_IRQ_BANK0), UART0=33, UART1=34, SPI0=31, SPI1=32,
I2C0=36, I2C1=37, USB=14, CLOCKS=30, TIMER0_x=0–3.

> Note: **IRQ 40/41 (`PROC0_IRQ_CTI`/`PROC1_IRQ_CTI`) are datasheet-only** — the SVD
> omits them (CoreSight cross-trigger debug interrupts, not attached to a peripheral).
> Confirmed by `verify.py`, which allowlists exactly these two. Not driver-relevant.

## Debug tooling status (G0)
- **openocd 0.12.0 (upstream, installed) has NO rp2350.cfg** — only rp2040. Options:
  (a) build the **Raspberry Pi OpenOCD fork** for OpenOCD-based SWD/flash, or
  (b) **use probe-rs 0.31 directly** (`probe-rs attach/run`, RTT, `probe-rs gdb`) —
  probe-rs supports RP2350 natively; this avoids the fork and is the leaner rig.
  Prefer (b) for bring-up; humility (G6) uses probe-rs anyway.
- picotool 2.2.0 handles UF2/BOOTSEL load + `picotool info` host-side validation (G1).

## First-4-KiB byte map (G0 deliverable)
```
0x10000000  ┌ .vector_table   (size = (16 + N_int)*4; rp235x-pac gives all 52 → 0x110)
            ├ .header         (abi::ImageHeader, 80 B / 0x50) — read by kernel HEADER static
            ├ .image_def      (20 B / 0x14) — the boot-ROM block loop
            └ .text …         (from _stext = vt + header + image_def)
```
All metadata well within the first 4 KiB (kernel-link.x ASSERTs enforce ≤0x1000).
