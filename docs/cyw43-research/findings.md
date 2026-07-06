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

## 8. Milestone 1 progress (bit-banged gSPI chip-detect)
Board validated first: the RP2350 port runs unchanged on the Pico 2 W (USB shell
works; note the demo "LED" GP25 is the CYW43 CS here, so no LED blink).

First gSPI attempt is in the app's pre-kernel `main` (`cyw43_probe`, results in
the probe-readable `CYW43_PROBE[4]` static @ nm; flash over SWD, read via
`probe-rs read b32`). Pins GP23/24/25/29 as above; WL_ON high + 60 ms; bit-bang a
32-bit command `[wr|incr|func|addr|len]` then clock in 32 bits.

Observed (NOT yet 0xFEEDBEAD): read0(normal cmd)=0x06060606,
read1(16-bit-word-swapped cmd)=0x7d5bfdda, read2(after bus-ctrl write)=0. So the
**chip is powered and gSPI responds with real data** (not floating), but the
framing is wrong. Tuning knobs to try next: command bit order (LSB vs MSB), the
sample clock edge (rising vs falling / add a half-cycle), the F0 response-delay /
leading clocks before data, and the exact 16-bit byte/word swap. Cross-check
against the embassy `cyw43` `bus.rs` and iosoft PicoWi `spi` init once a working
mirror URL is found (both repos moved paths).

## 9. Flash / SRAM budget (reserved region for the blob)
RAM-boot copies the whole image into SRAM (520 KiB). The ~224 KiB CYW43 firmware
blob must stay in FLASH and be STREAMED (never `include_bytes!`d, or it eats
224 KiB of SRAM). Reserved in `chips/rp235x/memory-pico-2.toml`:

```
[[cyw43_fw]]  address = 0x1c20_0000  size = 0x4_0000  read = true   # 256 KiB
```
= physical flash offset 0x20_0000 (2 MiB), read via the no-translate XIP mirror
(0x1c00_0000 base). Grant to the cyw43 driver with `extern-regions=["cyw43_fw"]`;
it reads the blob there and streams it over gSPI. Flash it separately:
`picotool load 43439A0-plus-clm.bin -t bin -o 0x1020_0000`.

Flash map: image (RAM-boot storage) 0x1000_0000.. (<=256 KiB, or A/B <=512 KiB);
blob 0x1020_0000..0x1024_0000; 4 MiB total -> trivially fits. SRAM budget without
the blob: ~180-210 KiB code + ~60-90 KiB data/smoltcp buffers ~= 250-300 KiB of
520 KiB -> ~200 KiB headroom.

## 10. Firmware storage: the correct way = Hubris auxflash (DONE)
Rather than a raw extern-region + manual picotool, we use Hubris's **auxflash**
blob mechanism (the pattern the FPGA bitstream loaders use): build-time TLV-C
packing with SHA3 checksums + 4-byte tags, and a driver that streams by tag.

- Blobs in `support/cyw43-firmware/` (43439A0.bin 231 KB WIFI, 43439A0_clm.bin
  984 B WCLM; redistributed from embassy cyw43-firmware).
- `app.toml`: `[config.auxflash]` memory-size=1.5 MiB, slot-count=6 (auxflash
  minimum; 64 KiB-aligned slots; fw image ~232 KB needs a 256 KiB slot). Slots
  2+ reserve room for the BT firmware later. `[[auxflash.blobs]]` WIFI + WCLM
  (compress=false).
- Build packs `dist/auxi.tlvc` (232160 B: CHCK -> AUXI -> WIFI -> WCLM), and
  sets HUBRIS_AUXFLASH_CHECKSUM for the server.
- Flash it to slot 0: `probe-rs download ... --base-address 0x1020_0000 auxi.tlvc`
  (or `picotool load auxi.tlvc -t bin -o 0x1020_0000`). VERIFIED: reads back at
  the no-translate mirror 0x1c20_0000 as CHCK/AUXI/WIFI.

Next: a **read-only rp235x auxflash server** -- a faithful port of
drv/auxflash-server but with `SlotReader::read_exact` copying from the
memory-mapped XIP mirror (0x1c20_0000 + offset) instead of driving a QSPI chip,
and the write/erase/redundancy ops stubbed (the blob is flashed once, externally).
It reuses drv-auxflash-api + the tlvc crate. Built alongside the cyw43 driver
(its only client), which calls `get_blob_by_tag(*b"WIFI")` and streams it.

## 11. gSPI decode: logical layer CONFIRMED vs embassy; PHY needs PIO
Cross-checked the bit-bang against the authoritative embassy `cyw43` source
(cyw43/src/spi.rs + consts.rs). Our LOGICAL layer is exactly right:
- `cmd_word(write,incr,func,addr,len) = (write<<31)|(incr<<30)|(func&3)<<28
  |(addr&0x1FFFF)<<11|(len&0x7FF)` -- identical to ours.
- `swap16(x) = x.rotate_left(16)`; FUNC_BUS=0, REG_BUS_TEST_RO=0x14, FEEDBEAD.
- Init: pwr LOW 20 ms, HIGH **250 ms** (not the datasheet's 50 ms), then loop
  `read32_swapped(FUNC_BUS, 0x14)` until it returns FEEDBEAD. read32_swapped
  swap16's BOTH cmd and response, so on the wire the raw response is
  swap16(FEEDBEAD) = **0xBEADFEED** (what our probe should see with the swapped
  command 0xA004_4000).
- Then write REG_BUS_CTRL = WORD_LENGTH_32|HIGH_SPEED|INTERRUPT_POLARITY_HIGH|
  WAKE_UP | 0x4<<(8*RESP_DELAY) | STATUS_ENABLE<<(8*STATUS_ENABLE) |
  INTR_WITH_STATUS<<(8*STATUS_ENABLE); afterwards normal (un-swapped) reads work.
  F0 reads have NO response delay; backplane (F1) reads use SPI_RESP_DELAY_F1
  padding (WHD_BUS_SPI_BACKPLANE_READ_PADD_SIZE).

Bit-bang result (GP23/24/25/29, matrix of swap x sample-edge x turnaround, 250 ms
power): never 0xBEADFEED. Got periodic/noise artifacts (0x06060606 is period-8 =
our own clock aliasing, plus 0x7d5bfdda/0xfab7fbb4 = a floating/mis-clocked DIO).
CONCLUSION: bit-banging cannot reliably clock the half-duplex gSPI (DIO turnaround
+ sub-us setup/hold). embassy/pico-sdk/PicoWi ALL use a PIO program for the PHY --
that is the correct next step.

## 12. NEXT: PIO gSPI PHY (milestone 1, proper)
Build a first RP2350 PIO capability + a gSPI PHY state machine:
- A PIO program that shifts out the 32-bit cmd MSB-first on GP24, turns the pin
  around (OUT->IN), and shifts in the response -- CS=GP25, CLK=GP29, the SDIO
  clock derived from a PIO clock divider. Model on the pico-sdk `cyw43_bus_pio`
  / embassy `cyw43-pio` program (both public).
- A Hubris `rp235x-pio` driver (none exists yet -- this is the first PIO use in
  the port; reusable for other PIO peripherals later).
- The cyw43 driver then implements the embassy `SpiBusCyw43` contract
  (`cmd_read`/`cmd_write`) over the PIO SM, runs the init above to read FEEDBEAD,
  then streams the WIFI blob (from the auxflash server) to the chip.
The firmware-storage half (auxflash) is already done + verified; the PHY is the
remaining prerequisite for chip-detect and everything after.

## 13. P1 done, P2 PHY runs (read-window alignment WIP)
P1 (PIO plumbing proof): VERIFIED. `pio_echo_test` loads a hand-assembled
3-instruction echo program (pull/mov/push) into PIO0 SM0; PIO_PROBE[0] read back
0xc0de1234 exactly. Proves reset, instruction-memory load, SM config, FIFO
access -- AND that our hand-assembled PIO encoding is correct.

P2 (gSPI PHY): the 7-instruction gSPI program runs deterministically (~1 MHz,
clkdiv 75). Drive: poke X=31/Y=31/pindirs via sm_instr, push swapped cmd
0xA004_4000, poll RXF. mode-select fix landed -- DIO must be SIO-LOW while WL_ON
rises to latch gSPI (not SDIO) mode, THEN hand DIO/CLK to PIO funcsel 6; this
changed the read from 0xffffffff (SDIO/floating) to 0x00000000. Still not
0xBEADFEED: the read window is misaligned (reading a DIO-low span). This is
read-phase timing: candidates are the sample edge vs the 2-cycle PIO input
synchronizer (embassy `in side 1` samples at the rising edge -- may need a
half-cycle offset), and the turnaround length after `set pindirs, 0`. NEXT
diagnostic: capture a longer bitstream (2+ words) to locate the response, then
sweep read-edge/turnaround. Hand-assembled gSPI program:
[0x6001,0x1020,0xE080,0xB042,0xA042,0x4801,0x0045], wrap 6->0.

## 14. Doc-driven PIO gSPI debug -> root cause: CS (GP25) stuck HIGH (hardware)
Read the embassy cyw43-pio source (not guessing) and applied every difference:
- Wrong program variant: at ~1 MHz use embassy's LOW-SPEED program (1 nop side0),
  not the overclock one (2 nops). Fixed.
- gSPI mode-select: DIO must be SIO-LOW while WL_ON rises, then funcsel 6. Fixed
  (moved read 0xffffffff -> 0x00000000).
- CLK (side-set pin) + DIO must be OUTPUT and driven LOW at idle
  (set_pin_dirs(Out)+set_pins(Low)); input_sync_bypass(true) on DIO. Applied.
- Verified funcsel 6 = PIO0 (PAC), dbg_padoe = 0x21000000 (DIO+CLK ARE outputs),
  turnaround works (padoe bit24 clears after read).

On-target register probes (CYW43_PIO[0..3]):
- Pull-up on DIO + 256-bit read: still all 0 -> the device ACTIVELY drives DIO
  low (overpowers the pull-up) => the CYW43 is ALIVE and connected.
- Pin levels at CS-low: 0x02820032 -> WL_ON(23)=1 (powered), but **CS(25)=1**.
- SIO OE=0x02800000 (CS output-enabled), SIO OUT bit25=0 (driven low),
  GP25 io_ctrl=0x05 (funcsel SIO), GP25 pad=0x56 (od=0, iso=0, ie=1): everything
  says GP25 should be LOW, yet gpio_in reads it HIGH.

ROOT CAUSE: CS (GP25) is driven low push-pull by the RP2350 but reads HIGH ->
something external holds it high. With CS high the CYW43 is deselected (GP29
becomes VSYS/ADC, DIO becomes the IRQ line held low), so no clock/command reaches
it and it never returns FEEDBEAD. This is a HARDWARE anomaly, not software:
needs physical investigation -- confirm the board is a Pico 2 W, measure GP25,
and check the probe/rig for anything tied to GP25/24/29 (though those are
internal to the CYW43 on the W and not on the header). The PIO gSPI PHY itself
is correct and ready; it is blocked on CS reaching the chip.
