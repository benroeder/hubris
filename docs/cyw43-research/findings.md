# Pico 2 W (CYW43439) bring-up research

Host = RP2350 (our port is done). New silicon = Infineon **CYW43439** (2.4 GHz
Wi-Fi 4 + BLE 5.2). This doc is the bring-up reference; datasheets archived
alongside it (`cyw43439-datasheet.pdf`, `pico-2-w-datasheet.pdf`, and their
`*.txt` extractions via `pdftotext`).

## 1. Board wiring (Pico 2 W datasheet §3.8) — CONFIRMED

The CYW43439 is on **internal** RP2350 GPIOs (not on the header):

| RP2350 GPIO | CYW43439 signal | Notes |
|---|---|---|
| **GP23** | WL_REG_ON | power/reset. High = WLAN out of reset; internal 200k pulldown |
| **GP24** | gSPI **DIO** (data) | **HALF-DUPLEX** bidirectional; shared with WLAN IRQ |
| **GP25** | gSPI CS | when high, also enables GP29 ADC to read VSYS |
| **GP29** | gSPI CLK | shared with VSYS sense (ADC3) |

The CYW43's OWN gpios: **WL_GPIO0 = the onboard LED**, WL_GPIO1 = SMPS power-save,
WL_GPIO2 = VBUS sense. So **blinking the LED requires talking to the chip** (gSPI
+ firmware) -- it's the true "hello world". SPI up to 33 MHz on this board.

Two consequences that shape the driver:
- **Half-duplex DIO** means the PL022 SPI (separate MOSI/MISO) CANNOT drive it.
  Must use **PIO** (a custom state-machine program) or bit-bang. We have NO PIO
  driver yet -> that's the first real prerequisite. pico-sdk uses a PIO program.
- **MODE SELECT**: hold GP24 (DATA2) LOW before raising WL_REG_ON to select gSPI
  (not SDIO) mode.

## 2. gSPI protocol (CYW43439 datasheet §4.2)

**Command word** (32 bits, sent first on each transaction):
```
[ wr:1 | incr:1 | func:2 | addr:17 | len:11 ]
```
- `func`: F0 = gSPI/SPI control regs; F1 = backplane (internal SoC address space,
  max 64 B block); F2 = WLAN data path (max ~2048 B).
- `wr` write flag, `incr` address auto-increment, `len` byte count.

**Byte-swap quirk:** right after power-up the device expects word-swapped access;
the very first job is to write the **SPI bus control register (F0 addr 0x00)** to
disable byte-swap + enable high-speed mode (iosoft uses value `0x204b3`), after
which normal little-endian access works.

**F0 0x00 (bus control) bits** (datasheet Table 6): word-length bit0 (0=16/1=32),
endianness bit1, high-speed bit4 (default 1), interrupt-polarity bit5 (default 1
= active high), **wake-up bit7**. Plus status-enable / response-delay registers.

**Chip detect:** read **F0 addr 0x14** -> constant **0xFEEDBEAD** once out of reset.

**Backplane (F1) reads** return 4 leading padding bytes (discard) to give the
peripheral response time; there are window/address registers to move the 32 KB
backplane window over the SoC address space.

## 3. Boot-up sequence (datasheet §4.2.3) -- the driver skeleton

1. GP24 low (select gSPI), assert **WL_REG_ON (GP23)** high.
2. **Wait 50 ms** for out-of-reset.
3. Poll read **F0 0x14** until it returns **0xFEEDBEAD** (device up).
4. Write **F0 0x00** bus-control to fix byte-swap + high-speed (chip now talks
   normally). [This is the "read chip ID over gSPI" milestone.]
5. Set the **wake-up WLAN bit (F0 0x00 bit7)** -> starts crystals/PLL.
6. Wait for the low-power (ALP) clock to be available.
7. Program the PMU crystal-frequency register -> PLL locks -> **chipActive** IRQ
   = device awake/ready.
8. Over F1/backplane: **upload the WLAN firmware + CLM regulatory blob** into the
   WLAN core's RAM, then bring the WLAN ARM core out of reset. [firmware-upload
   milestone -> then the LED (WL_GPIO0) can be driven via a chip command.]
9. Wi-Fi: scan / join / IOCTL commands over F2, then a net stack (smoltcp).

## 4. Firmware blobs (needed before anything transmits)
The CYW43439 has no resident firmware -- ~230 KB Wi-Fi MAC firmware + a CLM
regulatory blob must be embedded and uploaded (step 8). Source:
**embassy `cyw43-firmware`** repo (also in pico-sdk).

## 5. Best references (the gSPI backplane detail is NOT fully in the datasheet)
- **embassy `cyw43`** crate (Rust) -- THE reference: gSPI, backplane, firmware
  upload, Wi-Fi + BLE. Closest to a Rust/Hubris port.
- **pico-sdk `cyw43_driver`** + Damien George's **`cyw43-driver`** (C) -- vendor
  reference + the **PIO gSPI program**.
- **iosoft.blog "PicoWi"** series -- bare-metal, no-SDK gSPI + firmware from
  scratch; the clearest low-level walkthrough (source of the 0x204b3 / 0xFEEDBEAD
  / command-word details above).
- **Infineon WHD** (Wi-Fi Host Driver) -- vendor API/architecture.

## 6. Debug setup
Same probe rig: the Pico 2 W becomes the SWD **target** on the existing probe
wiring (probe-rs `--chip RP235x`), console over its native USB. The CYW43 has no
console, so the probe + reading the shared gSPI state is the instrument -- like
core-1 AMP bring-up.

## 7. Proposed milestones (each probe-verifiable)
1. **PIO gSPI driver** (prerequisite) + power WL_ON, read F0 0x14 = 0xFEEDBEAD.
2. Wake sequence -> chipActive; upload firmware + CLM blob.
3. **Blink WL_GPIO0 (the LED)** via a chip command -- proves gSPI + firmware.
4. Wi-Fi scan -> join an AP.
5. TCP/IP via smoltcp (Hubris already uses it on the STM32H7 net stack).

Steps 1-3 are a bounded first arc; 4-5 are the big net-stack lift. A natural AMP
tie-in later: run the net stack on core 1 (dedicated I/O core).
