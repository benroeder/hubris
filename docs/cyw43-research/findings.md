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

## 15. CS (GP25) hard-held HIGH -- confirmed hardware, not drive strength
Bumped GP25 drive to 12 mA (pad 0x76, matching embassy). CS STILL reads high
while SIO drives it low (SIO OUT bit25=0). A 12 mA push-pull that cannot pull a
pin below Vih means a HARD external hold-high (equiv < ~250 ohm, or a driver /
short), not a normal pull-up. Pin map re-verified against the Pico 2 W datasheet:
GP23=WL_ON, GP24=DIO, GP25=CS, GP29=CLK -- all correct.

=> Physical investigation needed on the target board's GP25:
- Confirm the board really is a Pico 2 W (not a plain Pico 2).
- Meter GP25 vs 3V3 / GND for a short or hard pull while the board runs.
- Check the probe rig / any hat/jumper touching GP25 (GP23-25/29 are internal to
  the CYW43 on the W and NOT on the 40-pin header, so external wiring should be
  impossible -- which points at the board itself or the specific unit).
The PIO gSPI PHY + init are correct and will read FEEDBEAD once CS reaches the
chip low. Instrumentation left in cyw43_pio_detect (CYW43_PIO[0..3]).

## 16. CORRECTION: CS is NOT stuck -- it has RC settling; chip still silent
The earlier "CS stuck high" (sec 14-15) was a MEASUREMENT ARTIFACT: I read CS
immediately after driving it low. A clean toggle test with a settling delay
(rd_cs: drive low, wait ~100us, read) shows CS is fully controllable both before
AND after power-up AND through the whole PIO/SM setup (all reads = 0). GP25 (CS)
has an RC on the Pico 2 W (the VSYS-ADC gating net), so it needs ~us to settle
low. Added a 1 ms settle after CS-low before clocking.

Even so, the device is still silent: OR of a 256-bit read = 0 with NO pull on DIO
(embassy Pull::None) at 500 kHz and 100 kHz. A clean all-0 with no pull means the
CYW43 actively holds DIO low (it is alive + powered), but never returns FEEDBEAD.

Verified correct on the RP2350 side (registers): mode-select, low-speed program,
CLK+DIO output & idle-low, input_sync_bypass, funcsel 6 = PIO0, dbg_padoe DIO+CLK
outputs, turnaround, CS controllable + settled, clock 100k-500k. Everything the
docs specify is applied and confirmed by register readback.

=> Exhausted software/register debugging. The definitive next step is a LOGIC
ANALYZER on GP24 (DIO) / GP25 (CS) / GP29 (CLK): confirm on the WIRE that CS is
low, CLK toggles cleanly, the 32-bit command (0xA004_4000 MSB-first) is correct,
and whether the device drives a response. Without wire visibility this is the
limit of no-guess debugging. The PIO gSPI PHY code is correct and ready.

## 17. Board + rig VALIDATED via benchmarks; chip confirmed alive (live gpio)
Per the user: the board-to-board rig is the same as the plain-Pico tests, so the
benchmarks validate it. Ran them across the Pico 2 W (1301) + the plain Pico
(1201):
- SPI board-to-board: exchange works both ways (W controller a5 5a 3c c3 <->
  peripheral de ad be ef), `spi bench 8192` = 56496 B/s (30% of theoretical).
- UART loopback: status shows rx=32 (works).
- SPI self-loopback OK on both.
=> The Pico 2 W's RP2350, GPIO, drivers, and the rig wiring are all HEALTHY. The
CYW43 problem is isolated to the CYW43 interface, NOT the board.

Live gpio probe (shell `gpio` cmd) with WL_ON(23) high: DIO(24) reads 0 under
BOTH pull-up and pull-down => the CYW43 actively drives DIO low (alive + powered),
independently confirming sec 16. (CS/CLK read post-boot reflect the LED task
owning GP25, not gSPI state.)

Conclusion: board good, chip alive, RP2350-side gSPI verified vs embassy at the
register level, yet the chip returns no FEEDBEAD. Remaining work needs wire-level
visibility (logic analyzer on GP24/25/29) or a faithful re-port; the `gpio` shell
cmd + a future `wifi` shell cmd give a live debug loop (suspend the LED with
`led off` first, since GP25 = LED = CS).

## 18. Matched pico-sdk exactly + loopback: PIO clock works, still no response
Researched the canonical pico-sdk C driver (cyw43_bus_pio_spi.c/.pio) and matched
every difference from mine:
- Per-transaction pio_sm_restart (clears ISR/OSR/shift counters/PC) +
  pio_sm_clkdiv_restart + clear_fifos (FJOIN_RX toggle) -- we were missing these.
- Pads: DIO pull-DOWN + schmitt, CLK pull-DOWN, WL_ON pull-UP.
- X/Y loaded via FIFO put + `out x/y,32` (autopull), not `set`.
- Confirmed byte-order: pico-sdk's DMA bswap+MSB-first-shift == our direct FIFO
  push of the swapped u32 (equivalent wire order), so bswap is NOT the issue.
Result: still all-0 from the CYW43.

Loopback validation via the peer (user's hint -- same rig as the SPI tests):
pointed a write-only PIO gSPI at the HEADER pins GP18=CLK/GP19=DIO/GP17=CS (wired
to the peer's SPI) and had the peer capture. The peer CAPTURED 8 bytes => our PIO
CLOCK generation works and reaches a peer. Data read back as 0x55 (alternating),
not the command A0 04 40 00 -- but that is likely a PL022 CPOL/CPHA mismatch (the
peer isn't a clean logic analyzer), so it's inconclusive on the data path.

STATE: board+rig validated (SPI 56 KB/s, UART), CYW43 alive (drives DIO), PIO
clock confirmed, gSPI matches pico-sdk AND embassy byte-for-byte -- yet no
FEEDBEAD. This is past what's resolvable without wire visibility. Recommend a
cheap USB logic analyzer (~$10-15 Saleae clone, sigrok/PulseView) on GP24/25/29:
it will show in minutes whether CS is low, CLK toggles at the chip, the command
bits are right, and if the chip drives a response. Every software avenue against
three reference implementations has been exhausted.

## 19. BREAKTHROUGH: MicroPython WiFi WORKS -- hardware 100% good, bug is ours
User's idea: flash a known-good stack. Flashed MicroPython v1.28.0 for
RPI_PICO2_W (picotool). `network.WLAN(STA_IF).active(True)` + `.scan()` returned
**6 APs** (darkworks, darkworksiot, ...). So the CYW43 chip, wiring, and
firmware-upload are ALL functional. My "maybe hardware / need a logic analyzer"
was WRONG -- the bug is in our Hubris gSPI code.

Introspected the WORKING config via the MicroPython REPL (machine.mem32):
- CYW43 gSPI runs on **PIO2 SM0** (funcsel 8 on GP24/29); GP23/25 = SIO (5).
- SM0: clkdiv=0x00020000 (div 2 -> ~37.5 MHz), shift=0x00030000
  (autopull+autopush, thresh 32, shift LEFT both), pinctrl=0x241c7718
  (sideset_base=29, out/set/in_base=24). exec wrap 26..31 (program at offset 26).
- GPIOBASE = 0 for PIO0 AND PIO2.
- Pads: GP23(WL_ON)=0x5a (pull-UP+schmitt), GP24(DIO)=0x56 (pull-DOWN+schmitt),
  GP25(CS)=0x56 (pull-DOWN+schmitt), GP29(CLK)=0x77 (pull-DOWN+schmitt+FAST
  slew+12mA).

Our SM config MATCHES (shift, pinctrl, GPIOBASE). Remaining differences to try:
(1) clock -- ours ~500 kHz vs 37.5 MHz; (2) pads -- add schmitt to DIO/CS + fast
slew to CLK; (3) PIO0 vs PIO2 (shouldn't matter -- pico-sdk uses PIO2 only
because MicroPython claimed 0/1). Also revisit the reset/power timing and the
boot-vs-runtime context. Note: MicroPython can be re-flashed anytime from
scratchpad/mp_pico2w.uf2 to re-introspect; Hubris re-flash via probe-rs/picotool.

## 20. FOUND the working config in MicroPython; porting to Hubris (cold-chip?)
Live MicroPython rp2.PIO experiment on the working chip: our EXACT gSPI approach
(low-speed program, cmd 0xA004_4000, shift-left autopull/autopush, sideset=CLK
out/in/set=DIO) reads **0xBEADFEED at PIO clock >= 4 MHz** (garbage < 4 MHz). So
the gSPI logic/program/command are CORRECT; the CYW43 gSPI just has an effective
minimum clock (~2 MHz SDIO). Our early Hubris runs at 0.5-1 MHz were below it.

Ported to Hubris (clkdiv=4 -> 37.5 MHz PIO; switched PIO0->PIO2 like MicroPython;
matched pads). Verified via probe that Hubris now byte-matches the working config:
SM shift=0x30000, pinctrl=0x241c7718, DBG_PADOE bit29(CLK)=out bit24(DIO)=in, SM
clocking (PC stalled at instr 4 `in`, RXSTALL). Yet DIO reads 0 -> chip silent in
Hubris.

Only remaining difference: chip RESET/POWER state. The MicroPython experiment ran
AFTER WLAN.active(True) had warmed the chip; Hubris hits it cold at boot. Next
decisive test: run the raw gSPI read in MicroPython on a COLD boot (before any
WLAN.active). If it fails cold too, the chip needs a fuller reset/init than our
WL_ON toggle; if it works cold, the Hubris boot/context differs. (Hubris flashes
over SWD; MicroPython needs BOOTSEL.)

## 21. THE TWO requirements found (via MicroPython): clock >=4MHz + PRIMING LOOP
Iterating live on the working chip in MicroPython (rp2.PIO) pinned BOTH things
our Hubris code was missing:
1. **Clock >= 4 MHz PIO** (~2 MHz SDIO). Below that -> garbage (0x03030303). Our
   early Hubris ran 0.5-1 MHz.
2. **PRIMING LOOP.** The CYW43 gSPI returns GARBAGE on the FIRST transaction
   after power-up and locks on from the 2nd. Proven: 4 reads at 16 MHz ->
   read0=0x03030303, reads 1..3 = **0xbeadfeed**. This is why embassy LOOPS
   `read32_swapped(0x14) until FEEDBEAD`; we did a single read. Each rp2 read is
   a FRESH StateMachine (pio_sm_init: disable+clear_fifos+restart+clkdiv_restart
   +jmp), and looping those primes the chip.

Ported to Hubris (16.7 MHz clock, 32-pass priming loop with full clean per pass)
-- config verified byte-identical to MicroPython's working SM (shift=0x30000,
pinctrl=0x241c7718, funcsels, pads, DBG_PADOE) -- but our RAW-REGISTER drive still
reads 0. Reproduced in MicroPython: rp2.StateMachine + Y=63 + loop = reliable
0xbeadfeed, but the SAME sequence done via raw machine.mem32 pokes = 0/garbage,
inconsistently. So the last gap is a subtle SM-driving detail in the raw register
sequence that rp2.StateMachine gets right and our hand-rolled poke order doesn't
(candidates: exact pindir setup via set_pindirs_with_mask, OSR/autopull state
before `out y`, the enable/put(cmd) ordering).

NEXT (clean, well-scoped): dump EVERY register write rp2.StateMachine.__init__ +
active() makes (instrument MicroPython, or read pico-sdk pio_sm_init +
sm_set_pindirs_with_mask + the cyw43 read fn) and replicate that exact byte
sequence. Everything else is proven: hardware works, chip-detect returns
0xFEEDBEAD, clock + priming requirements known, config matched.

## 22. Deep raw-poke debug: found a real Y-load bug + an unexplained gap
Instrumented both the WORKING rp2.StateMachine and my raw machine.mem32 drive on
the live chip and compared register-for-register:
- State right BEFORE enable is BYTE-IDENTICAL (padoe=0x21000000 DIO+CLK out,
  padout=0, fdebug=0, fstat=0x0f000f00, pinctrl=0x241c7718). Only PC differs
  (rp2 program at offset 20 so PC=0x1a; mine at 0).
- FOUND A REAL BUG: loading Y via `put(N); out(y,32)` (autopull) does NOT work
  when exec'd via sm_instr while the SM is DISABLED -- autopull doesn't fire, so
  Y got garbage (FLEVEL showed RX filled to 4 words + RXSTALL => Y was huge, not
  63). Fix: `set y,31` (immediate) for a single-word read, or an explicit `pull`.
- BUT even with Y fixed, my raw drive reads 0x0 while rp2 reads 0x03030303 (both
  first-read garbage) -- the chip drives DIO differently for the two, despite
  identical clocking/pins/config. Priming (loop) makes rp2 reach 0xBEADFEED;
  my raw loop never does.

Verdict: hand-rolled raw PIO register pokes do NOT reliably reproduce
rp2.StateMachine's behavior on this chip, for reasons not visible in the register
dump (likely exec/enable timing or an RP2350 PIO subtlety). RECOMMENDATION for
the Hubris driver: do NOT hand-roll the SM drive with raw sm_instr pokes. Use a
proper PIO StateMachine abstraction -- port `rp235x-hal`'s PIO module or the
`pio`/`pio-proc` crates (which implement the pico-sdk pio_sm_init + set_pindirs +
put/exec sequence correctly) -- then the proven recipe applies: 16 MHz clock,
low-speed program, cmd 0xA004_4000, **loop read 0x14 until 0xBEADFEED** (priming).

CONFIRMED FACTS (unchanged): CYW43 hardware works (MicroPython WiFi, 6 APs),
chip-detect returns 0xFEEDBEAD via rp2, clock must be >=4 MHz PIO, and the read
must be looped (first read = garbage). MicroPython flashes over SWD now too
(scratchpad/mp.bin), so no BOOTSEL dance for future debugging.

## 23. *** SOLVED: CYW43 chip-detect WORKS in Hubris (0xFEEDBEAD) ***
Root cause: my HAND-ASSEMBLED PIO program had 3 wrong instruction encodings,
present in every test AND in the Hubris driver the whole time. Dumping
MicroPython's rp2 ASSEMBLER output exposed it:
  wrong:   [0x6001, 0x1020, 0xE080, 0xA042, 0x4801, 0x0044]
  correct: [0x6001, 0x1040, 0xE080, 0xA042, 0x5001, 0x0084]
  - `jmp x--`: I used condition !X (0x1020); X-- is 0x1040.
  - `in pins,1 side1`: I misplaced the side-set bit (0x4801 = side0+delay8);
    correct is 0x5001.
  - `jmp y--`: I used X-- (0x0044); Y-- is 0x0084.
This is why the isolation (rp2's CORRECT program + my raw read) always worked
while my own setup (my wrong program) never did -- every register matched, only
the program bytes differed.

Result on the live Pico 2 W in HUBRIS: CYW43_PIO[0] = 0xBEADFEED (= swap16 of
FEEDBEAD) on pass 2 (pass 1 = priming garbage). CHIP DETECTED.

The complete working recipe (all verified on hardware):
- PIO2 SM0, GP24=DIO/GP29=CLK funcsel 8, GP23=WL_ON/GP25=CS SIO.
- 6-instr low-speed gSPI program (correct bytes above), wrap 0->5.
- clkdiv ~16 MHz (>=4 MHz required), shift-left autopull/autopush thresh 32,
  sideset=CLK out/set/in=DIO, input_sync_bypass on DIO.
- Power WL_ON low 20 ms / high 250 ms (DIO SIO-low = gSPI mode select).
- Per read: CS pulse, clear FIFOs, DIO pindir out, SM restart + clkdiv restart,
  load X=31/Y=63 via FIFO+autopull (restart empties OSR so autopull works),
  jmp 0, enable, push cmd 0xA004_4000, read.
- LOOP the read until 0xBEADFEED (the chip's FIRST read after power-up is
  garbage; it locks on from the 2nd). Milestone 1 (chip-detect) COMPLETE.

## 24. gSPI read + write + REG_BUS_CTRL init all verified
Refactored the boot probe into a reusable `xfer(x_bits, y_bits, words)` gSPI
transaction and ran the embassy init prologue on the live Pico 2 W:
- [0] chip-detect (read TEST_RO swapped, primed) = 0xFEEDBEAD
- [1] WRITE path: write TEST_RW (0x18) = 0x12345678, read back = 0x12345678
- [2] write REG_BUS_CTRL = 0x304B1 (WORD_LENGTH_32|HIGH_SPEED|INT_POL_HIGH|WAKE_UP
      |0x4<<8|(STATUS_ENABLE|INTR_WITH_STATUS)<<16); then read TEST_RO NON-swapped
      = 0xFEEDBEAD -> the bus is now 32-bit little-endian, write path confirmed.
- [3] detect pass count = 2 (priming).
So the full gSPI byte layer (read8/16/32, write, mode switch) is proven. NEXT:
ALP/HT clock request + backplane (F1) window access, then stream the WIFI blob
from the auxflash server into the WLAN core (firmware upload), then LED.

## 25. Backplane (F1) access + ALP clock + chip-ID all verified
Extended the boot probe with the embassy init prologue past bus config:
- [2] ALP clock: write8(F1, CHIP_CLOCK_CSR 0x1000E, ALP_AVAIL_REQ 0x08); poll
  read8 -> 0x48 (ALP_AVAIL 0x40 | ALP_AVAIL_REQ 0x08) on the 1st poll. So direct
  F1 read/write works, and the ALP clock is up. (Set SPI_RESP_DELAY_F1 0x1d = 4
  first so F1 reads return [padding, data] -- take the 2nd word.)
- [4] Windowed backplane read: set the 32 KiB window to CHIPCOMMON_BASE
  (0x18000000) via SBADDR HIGH/MID/LOW (F1 regs 0x1000C/B/A), read window offset 0
  with the 32-bit flag (0x8000) -> 0x1545A9AF. Low 16 bits = 0xA9AF = 43439 =
  the CYW43439 chip id. The windowed path (what firmware upload uses to reach the
  WLAN-core RAM) works.
The whole gSPI + backplane stack is now proven. NEXT: reset the WLAN ARM core,
stream the 224 KB WIFI blob (from the memory-mapped auxflash region at 0x1c200000)
into WLAN RAM via windowed backplane writes, bring the core out of reset, LED.

## 26. FIRMWARE DOWNLOAD verified -- 231 KB streamed into WLAN RAM
Streamed the WIFI blob (memory-mapped auxflash mirror, TLV-C body at 0x1c200048,
231077 bytes -- note the TLV-C chunk header is 12 bytes: tag+len+header_cksum, so
body is at tag+12 not tag+8) into WLAN-core RAM (backplane addr 0) via windowed
backplane WRITE BURSTS: 64 words/burst, chunked to the 32 KiB window, cmd is
WRITE|INC F1 with the 32-bit flag, len = n*4 bytes, then n data words (xfer waits
on TX-FIFO not-full so bursts don't TXOVER). Verified at RAM[0]/[0x8000]/[0x38000]
against the source -> 0x600D600D. Chip-reset prep first: disable WLAN + SOCSRAM
cores, reset SOCSRAM up, 43439 socsram_init (bp_write32 socsram+0x10=3, +0x44=0).
Backplane windowing works for LOW/MID/HIGH SBADDR alike (0x8000/0x10000/0x18000
all verified). NEXT: write NVRAM + magic at top of RAM, reset_core_up(WLAN),
check core is up, then drive the LED (WL_GPIO0) once firmware runs.

## 27. *** WLAN CORE UP -- CYW43439 IS RUNNING FIRMWARE ***
After the firmware download: wrote the Pico-W NVRAM (744 B, from cyw43-driver
wifi_nvram_43439.h) near the top of RAM at RAM_SIZE-4-nvram_len (0x7FD14), then
the length-magic word ((~words&0xFFFF)<<16 | words = 0xFF4500BA) at RAM_SIZE-4
(0x7FFFC) so the firmware can locate the NVRAM. Then reset_core_up(WLAN wrapper
0x18103000): IOCTRL = FGC|CLOCK_EN, RESETCTRL = 0, IOCTRL = CLOCK_EN. Result:
- [5] firmware verify = 0x600D600D
- [6] NVRAM magic readback = 0xFF4500BA
- [7] WLAN core-up check (IOCTRL&3==CLOCK_EN, RESETCTRL&1==0) = 0xC0DE600D
THE CHIP IS EXECUTING ITS WI-FI FIRMWARE, driven end-to-end from Hubris on the
RP2350. Full chain proven: PIO gSPI -> bus cfg -> ALP -> backplane -> core reset
-> 231 KB firmware upload -> NVRAM -> WLAN core boot. NEXT: F2 (WLAN data)
IORDY handshake + the CDC/BDC ioctl path -> drive WL_GPIO0 (onboard LED),
then scan/join.

## 28. Firmware loads + core resets, but HT clock / F2 not up yet (WIP)
Added the post-download bring-up: HT-clock poll (CHIP_CLOCK_CSR & 0x80), F2
watermark (F1 0x10008 = 0x20), BUS_INTERRUPT_ENABLE (F0 0x06 = IRQ_F2_PACKET
0x20), and the F2-ready poll (REG_BUS_STATUS F0 0x8 & STATUS_F2_RX_READY 0x20).
Also tried the pre-download HT start (clear PULL_UP F1 0x1000F=0, request HT).
RESULT: WLAN core is out of reset (0xC0DE600D) and firmware verifies in RAM, but
- [8]/[11] HT clock = 0x50 (ALP_AVAIL 0x40 | HT_AVAIL_REQ 0x10) -- HT_AVAIL 0x80
  NEVER asserts, pre OR post download.
- [9] F2 status = 0, never F2_RX_READY. So the firmware is not fully executing.
Hardware/crystal are fine (MicroPython runs Wi-Fi on this board). The gap is the
exact clock/PMU init embassy does that I only partially replicated -- notably the
full ALP dance (CHIP_CLOCK_CSR = FORCE_HW_CLKREQ_OFF|ALP_AVAIL_REQ|FORCE_ALP, poll
ALP, then CSR=0) and the WAKEUP_CTRL / SLEEP_CSR / watermark ordering in
runner.rs ~460-530, plus whether the CR4 boot ROM needs anything more to jump
into the loaded firmware (blob[0]=0x00000000, so a boot ROM reads the header, not
a raw reset vector). NEXT: reproduce embassy's clock init byte-for-byte in order,
then re-check HT (0x80) and F2 (0x20); once F2 is ready, the CDC/BDC ioctl path
drives WL_GPIO0 (LED).

## 29. *** FIRMWARE BOOTED -- HT clock + F2 ready ***
Root cause of the firmware not running: TWO backplane bugs found by reading the
georgerobotics/pico-sdk C driver (cyw43_ll.c / cyw43_bus_pio_spi.c):
1. CYW43_BUS_MAX_BLOCK_SIZE = 64 BYTES for SPI. My 256-byte (64-word) backplane
   write bursts silently truncated at 64 bytes -- only the first 16 words of each
   burst landed, the rest was dropped. The 3-point verify only checked the FIRST
   word of a burst, so it passed while the firmware was full of holes. Fix: cap
   write bursts at 16 words (64 bytes).
2. CYW43_BACKPLANE_READ_PAD_LEN_BYTES = 16 for SPI (not 4) -- burst reads carry 4
   padding words, not 1 (only matters for multi-word reads; single reads with
   SPI_RESP_DELAY_F1=4 are self-consistent and fine).
Also replaced the TX-FIFO pacing hack (fixed per-word delay) with a proper
FLEVEL poll (push only when the 4-deep TX FIFO has room) -- the C driver paces
via DMA/DREQ; FLEVEL is the register-poll equivalent.
RESULT on the live Pico 2 W: firmware verify [12]=0xFFFFFFFF (all good), HT clock
[8]=0x800000d0 (HT_AVAIL 0x80 set), F2 status [9] & STATUS_F2_RX_READY (0x20) set
after 169 polls. THE CYW43439 IS RUNNING ITS FIRMWARE AND F2 IS READY. NEXT: the
CDC/BDC ioctl path (CLM upload + country + WL_GPIO0 LED), then scan/join.

## 30. LED ioctl over F2 (SDPCM/CDC) -- WL_GPIO0 blink
With F2 ready, sent a SET_VAR "gpioout" ioctl over F2 (WLAN data) to drive the
onboard LED (WL_GPIO0). Frame (44 B): SdpcmHeader(12) + CdcHeader(16) +
"gpioout\0"(8) + mask(4) + value(4), prepended with the gSPI cmd
(WRITE|INC func2 addr0 len44 = 0xE000002C). SdpcmHeader: len/len_inv, seq,
channel=CONTROL(0), header_length=12. CdcHeader: cmd=SET_VAR(263), len=16,
flags=Set(2), id. mask=1<<0, value=1<<0 (on). Blinks by toggling value with
seq/id incrementing each frame. [15]=0x11EDB11C when the loop finished.
(Pending: physical confirmation the LED blinks; if not, read the F2 response and
check the CDC status, and add CLM/country init first.)

## 31. ioctl path works; gpioout returns -23 (needs CLM/init) -- LED pending
Built the SDPCM/CDC ioctl send + a demux receiver (read F2 frames, skip async
events on channel 1, find the CONTROL-channel-0 response, return its CDC status).
Sent SET_VAR "gpioout" (mask=1<<0, value=1<<0) to drive WL_GPIO0 (onboard LED).
RESULT: the chip PARSES and RESPONDS -- CDC status = -23 = BCME_UNSUPPORTED. So
the frame is correct but the firmware won't do gpioout until its control-layer
init runs. Per embassy control.rs init(): load CLM (SET_VAR "clmload" with the
984-byte WCLM blob at auxflash mirror 0x1c238700 + a 12-byte DownloadHeader
{flag=BEGIN|END|HANDLER_VER=0x1006, type=CLM=2, len, crc=0}), then "country",
then WLC_UP, then gpioout works. First CLM-upload attempt HANGS on the ~1 KB F2
write/response -- large-frame F2 transfer over the hand-rolled PIO needs more
work (the small ioctl round-trips fine). NEXT: debug the large F2 transfer (or
chunk it), finish the CLM+country+up init -> LED, then scan/join. This whole
control layer belongs in the drv/rp235x-cyw43 task, not the pre-kernel probe.

## 32. Proper SDPCM flow control done; CLM/country/up accepted; gpioout = -23 (WIP)
Implemented full SDPCM flow control (cyw43_ll.c): tx_seq + credit tracking (may
send only while credit != tx_seq; each rx SDPCM header's bus_data_credit advances
credit if delta<=20), response demux by CDC id (flags[31:16]). Frame layout
VERIFIED byte-for-byte vs the C driver (sdpcm_header_t + ioctl_header_t; flags =
(id<<16)|SDPCM_SET(2)|(iface<<12)). Ran the real init: CLM (SET_VAR clmload) ->
bus:txglom=0 -> apsta=1 -> country (CountryInfo XX/-1/XX) -> WLC_UP(2). ALL return
CDC status 0 (accepted). BUT SET_VAR "gpioout" (mask/value = 1<<0, the WL_GPIO0
LED, exactly as cyw43_ll_gpio_set) returns CDC status -23 = BCME_UNSUPPORTED,
regardless of init. So the frame/flow are correct but this firmware rejects
gpioout. Hypothesis: the embassy support/cyw43-firmware/43439A0.bin variant lacks
GPIO support, while MicroPython's firmware (WiFi + LED both work on this same
board) has it -- OR the Pico 2 W LED uses a different mechanism. GET "ver" to
confirm the fw version didn't return (id-match issue). NEXT: confirm fw version /
diff MicroPython's firmware; or pivot to scan/join (same working ioctl path, no
gpioout needed) to prove Wi-Fi. Everything up to and including arbitrary ioctls
works; only gpioout is blocked.

## 33. *** LED WORKS -- full Wi-Fi ioctl path proven end-to-end ***
gpioout finally ACCEPTED (CDC status 0) and the onboard WL_GPIO0 LED BLINKS,
driven from Hubris on the RP2350.
ROOT CAUSE of the persistent -23: a byte-order typo in the "gpio" word. LE
encoding of "gpio" = 'g'|'p'<<8|'i'<<16|'o'<<24 = 0x6f697067, but I had
0x6f696770 = bytes 'p','g','i','o' = "pgio" -> the iovar was "pgioout", unknown
-> BCME_UNSUPPORTED (-23). ("out\0"=0x0074756f and "clmload" encoded symmetrically
by luck, so only gpioout failed while CLM/country/up all succeeded, which is what
made it so baffling.) Verified along the way: firmware 7.95.61 identical to
MicroPython's, NVRAM identical (muxenab=0x100), CLM loads (clmload_status=0),
frame layout byte-for-byte vs cyw43_ll.c. The full working recipe: PIO gSPI ->
firmware upload -> boot -> SDPCM/CDC ioctls with proper flow control (tx_seq +
bus_data_credit, response demux by CDC id) -> CLM -> bus:txglom -> apsta ->
gpioout. WIFI BRING-UP COMPLETE (LED milestone). NEXT: scan/join reuse this exact
ioctl path; then move the whole stack into the drv/rp235x-cyw43 task.

## 34. *** Wi-Fi SCAN works -- 13 APs, real SSID "darkworks" ***
Full active escan over the byte-based ioctl path. Sequence: CLM -> bus:txglom ->
apsta -> GET cur_etheraddr (MAC 2c:cf:67:e8:5a:18, RPi OUI) -> bsscfg:event_msgs
(bitmask byte8 bit5 = ESCAN_RESULT event 69) -> WLC_UP -> escan (ScanParams 74 B
after "escan\0"; version=1, action=1, bssid=ff*6, bss_type=2, nprobes/times=-1).
Chip streamed 13 channel-1 ESCAN_RESULT event frames. Decoded the first frame's
BssInfo: ssid_len=9 "darkworks", BSSID 6c:63:f8:85:a9:aa. Key lesson repeated:
built ALL new payloads as byte arrays packed LE into words (do_ioctl_b) to avoid
the "gpio"-style byte-order bug. The count loop is slow (each iter does a gSPI
status read) -- 15k iters ~10 s covers the scan; 90k iters overran the probe
wait. NEXT: proper SDPCM/BDC/event parser to list all SSIDs+RSSI (belongs in the
drv/rp235x-cyw43 task, not the pre-kernel probe).

## 35. *** SCAN lists real SSIDs on-device: darkworks, darkworksiot, USB-Insight-Hub ***
Extended the scan to store each ESCAN_RESULT's BssInfo ssid region (frame words
31..39, i.e. ssid_len at byte 124) into a SSIDS[] static, read back over the
probe. Live result: "USB-Insight-Hub-b43a45b54af4", "darkworksiot", "darkworks"
-- the real nearby networks. Duplicates = multiple beacons/probe-resps per AP
across the scan window (dedup by BSSID later). One frame (AP 7) decoded short --
the fixed word-31 offset holds for most escan_result frames but a proper parser
(BDC data_offset + event header walk) is needed for 100%. Full Wi-Fi RECEIVE
path now proven: PIO gSPI -> fw -> ioctls -> events -> BssInfo -> SSID. This
closes Wi-Fi bring-up on the probe; the productionization is the driver task.

## 36. *** Productionised: Wi-Fi is now a proper Hubris task (drv-rp235x-cyw43) ***
Moved the whole gSPI/SDPCM stack out of the pre-kernel probe into a real task.
Structure: new drv/rp235x-cyw43 (owns pio2 + shares sio/io_bank0/pads_bank0/
resets) + drv/rp235x-cyw43-api + idl/rp235x-cyw43.idol. Firmware + CLM stream
from the auxflash server over IPC (get_blob_by_tag + read_slot_with_offset, 128 B
chunks) -- verified byte-perfect vs the file at 3 RAM readback points. The probe's
closures became struct methods; setup (pads/funcsel/power-on/PIO program/SM) runs
once at task start.
KEY LESSON -- PREEMPTION: at priority 4 (below usb/shell/hiffy) the CYW43 boot was
flaky (HT intermittent, F2 never ready) because a task switch mid-gSPI-transaction
holds CS low too long and corrupts sparse boot writes. Fix: cyw43 priority 3 (above
the noisy tasks), auxflash priority 2 (cyw43 calls it, so auxflash must be higher --
Hubris forbids priority inversion). Then HT/F2/CLM/MAC are 100% reliable across
resets. Also use userlib::hl::sleep_for (timer, preemption-safe) for the post-boot
settle, not busy delays. Idol verified via humility hiffy: get_mac => 2c:cf:67:e8:
5a:18, led(on/off) works, scan => 13 APs. Hubris ALLOWS sharing a peripheral across
tasks (map shows [sio] gpio_driver, cyw43). DIAG[16] static = bring-up telemetry.

## 37. AP mode -- open SoftAP for provisioning (Phase 1 of the captive portal)
Added Cyw43::ap_start (+ Idol "ap" op, shell `wifi ap`): brings up an OPEN SoftAP
"Pico2W-Setup" on ch 6, mirroring cyw43_ll_wifi_ap_init/set_up for the open case.
Added an `iface` arg to do_ioctl/do_ioctl_b -> CDC flags |= iface<<12 (STA=0,
AP=1). Sequence: country + WLC_UP -> ampdu_ba_wsize=2 -> bsscfg:ssid=[AP=1,len,
ssid] -> WLC_SET_CHANNEL=6 -> bsscfg:wsec=[AP=1,0] -> mfp=0/gmode=1/2g_mrate=22/
dtim=1 (AP iface) -> bss=[AP=1,up=1]. bss-up ioctl returned status 0. NEXT phases
(the real work): F2 DATA path (Ethernet over SDPCM ch 2), smoltcp, DHCP server,
DNS hijack, HTTP captive portal, credential storage, STA join. See
[[pico2w-iot-provisioning]].

## 38. F2 DATA path RX proven -- decoded a real mDNS packet from the phone
Added data_poll (Idol op) counting channel-2 (DATA) frames. When an iPhone joins
the open SoftAP it associates fine and sends real IP traffic; captured + decoded
a live frame: SDPCM(len 459, chan 2) + 2B pad + BDC(flags 0x20, data_offset 1) ->
Ethernet (dst 01:00:5e:00:00:fb, type 0x0800) -> IPv4 (proto 17) -> UDP :5353
mDNS to 224.0.0.251. KEY for smoltcp RX: the Ethernet frame starts at byte
18 + BDC.data_offset*4 (= 22 here); BDC.data_offset is frame byte 17. "unable to
join" on iOS = missing DHCP, NOT an association failure (the phone clearly assoc'd
+ sent traffic). AP SSID now hubris-<64-bit chip id> (= USB serial id), read from
watchdog scratch1/2. GOTCHA: adding "watchdog" to cyw43 uses hit the RP2350 MPU
per-task region cap (2 mem + 6 periph = 8 > 7) -> xtask dist PANICKED (dist.rs:689
`7 - n` underflow) and silently kept flashing the STALE binary; fixed by moving the
PIO2-out-of-reset to the privileged pre-kernel main and dropping "resets" from the
task. Next: build the smoltcp phy::Device on send_frame/recv_frame (TX + this RX
offset), then DHCP/DNS/HTTP.
