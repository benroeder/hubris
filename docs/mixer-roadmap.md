# RP2350 network mixer -- architecture and roadmap

Status: design record, 2026-07-15. Follows the MusicPi bring-up (PRs #18/#19:
I2S DAC out, SD FLAC playback, buttons, RGB, TFT, all hardware-validated).

## Product

A network-controlled audio mixer on a Pico 2 W + MusicPi (later a custom
carrier): two stereo line inputs + an SD disk stream, mixed with per-source
gains and ducking, out the PCM5100A DAC; controlled over an HTTP API on WiFi
and wired ethernet; field-updatable via the A/B image mechanism.

## Target block diagram

```
            +---------------- RP2350 (Pico 2 W) ---------------+
 line in A -| PCM1802 \   PIO1 RX     +-- mixer/ducker --+     |
 line in B -| PCM1802 /  (shared clk) |  gains, duck     |- PIO0 I2S - PCM5100A - out
 SD card  --| FLAC/WAV decode --------+  (per-source)    |     |
            |          ^ control                         |     |
 WiFi ------| CYW43 -+                                   |     |
 Ethernet --| W5500 -+- HTTP API: /source /mix /duck /update   |
            +---------------------------------------------------+
```

## Audio input (first phase, single-core)

2x PCM1802 breakouts (line-level stereo each):

| Signal | Pico | Note |
|---|---|---|
| BCK/LRCK (both) | GP10/GP11 | tap the existing I2S out clocks; capture is sample-locked to playback (clocks run continuously by design) |
| SCKI/MCLK (both) | GP5 | 256*fs square from PIO1 |
| DOUT A / B | GP27 / GP28 | consecutive -- one `in pins, 2` SM can capture both |
| power | VBUS/GND | boards have onboard regulators |

Check the FMT/MODE solder straps before wiring (slave, 256fs, I2S) -- the
GY-PCM5102 lesson. PIO1: 1 SM MCLK + RX SM(s); DMA ch2/ch3 (SECCFG clears in
rp235x-startup). Note: 256fs divides 150 MHz evenly only at fs=48828 Hz; at
44.1/48k the PIO divider is fractional (fine in practice, noted for quality
work). The proper endgame ADC is the PCM1865 (4-ch, BCK-PLL, no MCLK, I2C) --
no hobby breakout exists; needs the TI EVM or a custom PCB.

## Sprint-2 capture design (worked out, ready to implement)

PIO1 (add `[pio1]` 0x5030_0000 to chip.toml; un-reset in the pre-kernel main
like pio0/pio2). Four SMs, no MCLK needed until real PCM1802s arrive:

- SM0/SM1: **I2S slave receivers** for inputs A (GP27) and B (GP28), clocked
  by reading the GP10/11 BCK/LRCK we already drive (any PIO can `wait gpio` /
  `in pins` another block's outputs). One shared 8-instruction program at
  offset 0; the SMs differ only in PINCTRL in_base:
  `[0x208B, 0x200B, 0x208A, 0x200A,  ; one-time sync: LRCK hi, LRCK lo,`
  ` 0x208A, 0x4001, 0x200A, 0x0004]  ;   skip 1 BCK (I2S delay); loop: BCK hi, in 1, BCK lo, jmp 4`
  autopush 32, ISR shift LEFT. KEY INSIGHT: sync ONCE then free-run -- the
  I2S 1-bit delay pushes each frame's last bit across the frame boundary, so
  a per-frame resync would skip alternate frames; free-running after one
  aligned sync yields exact (L<<16)|R words forever (same clock domain, zero
  drift). Captured via DMA ch2/ch3 (ring on WRITE) into 4 KiB capture rings.

- SM2/SM3: **simadc fake ADCs** (feature-gated): I2S slave transmitters
  driving GP27/GP28 as outputs; the receivers read the same pads back (ie=1)
  -- zero wires. 6-instruction program at offset 8:
  `[0x208B, 0x200B,                  ; sync to LRCK fall`
  ` 0x208A, 0x200A, 0x6001, 0x000A]  ; loop: BCK hi, BCK lo, out 1 (data changes on fall), jmp`
  autopull 32, OSR shift LEFT. Fed by DMA ch4/ch5 looping tiny cycle-snapped
  sine rings (256 frames, filled once at init; distinct pitches per input so
  channel identity is audible). DMA ch2-5 need SECCFG_CHn.P clears in
  rp235x-startup.

- Mixer: sources A/B read their capture rings ((L<<16)|R stereo) instead of
  the DDS when capture is enabled; DDS stays as the no-hardware fallback.
  When the PCM1802s arrive: disable simadc, add the MCLK SM (256*fs on GP5),
  check the boards' FMT/MODE straps, plug in -- the receive path will already
  be validated against real I2S timing.

## Core split (AMP; builds on the rp2350-amp branch: core-1 launch + SIO
## mailbox proven; RCP-salt caveat still open)

Principle: hard-real-time audio data plane on core 1; everything bursty on
core 0. AMP = two complete Hubris images (kernel + jefe + sys + idle each).

| Core 1 -- audio engine | Core 0 -- control plane |
|---|---|
| mixer task (absorbs i2s: capture rings, gains+duck, I2S out, SD decode) | cyw43 + net + httpd (API + /update) |
| sdcard driver (SPI0) | W5500 net (PIO-SPI) |
| ws2812 (shares PIO0 with I2S out; doubles as VU meter) | tft (SPI1), buttons/gpio, shell, usb |
| jefe/sys/idle | flash_driver + updater, jefe/sys/idle |

Peripheral ownership by convention -- core 1: PIO0, PIO1, SPI0, DMA ch0-3;
core 0: PIO2 (CYW43 gSPI), SPI1, USB, QMI/flash. NEVER split one PIO block
across cores (CTRL RMWs are only priority-serialized within one kernel).

RAM: the real forcing function for AMP. Single-core pool is at 98% already;
core 1 ~ decode stack (160K) + rings ~ 220K of its ~260K half; core 0 gets
CYW43 + smoltcp + httpd + TFT. CPU: FLAC decode is the hog and gets a core
to itself (mixing math is trivial).

## Inter-core communication

1. **Shared control page** (~4K SRAM both images map, seqlock'd): core 0
   writes mix state (per-source Q15 gains, source enables, duck
   threshold/attack/release); core 1 reads once per mix block (~5 ms).
   Reverse: core 1 writes VU levels + playback position for HTTP/TFT.
   No syscalls in the audio hot path.
2. **SIO FIFO mailbox** (proven): doorbell + events ("state updated",
   "play file X", "track ended"), interrupt-driven.
3. **No bulk audio crosses cores** (decode lives with the mixer). Revisit
   only if audio-over-ethernet streaming lands (open question below).

Hubris shape: one small `icc` task per core owns the FIFO IRQ + shared page;
all other tasks reach it via normal (intra-core) Idol IPC:
`httpd -> icc0 == FIFO+page == icc1 -> mixer`. Versioned message ABI in one
place per side.

## Supervision / "reboot the WiFi core"

- Tier 1: core 0's jefe restarts wedged tasks (works today; covers most
  failures).
- Tier 2: heartbeat over the mailbox; on N misses core 1 force-cycles proc0
  via PSM (opposite direction of the proven core-1 launch). Audio never
  stops. REQUIRES a warm-boot flag (watchdog scratch, like the BOOTSEL
  two-hop): the restarted core 0 must SKIP chip-global clock init and the
  core-1 launch. Design this into the AMP boot path from day one.
- Tier 3: full watchdog chip reset; A/B boot selection guarantees a valid
  image (validated end-to-end).

Asymmetry is deliberate: core 1 small and near-crashproof; core 0 carries
the complexity and is the disposable, restartable one.

## Networking: dual NIC

- CYW43 WiFi (convenience/control): zero HAT pins; requires swapping the
  Pico 2 for a Pico 2 W and FINISHING the cyw43 bring-up (chip-detect WIP on
  the pico2w-wifi branch -- the riskiest single item in this plan).
- W5500 ethernet (deterministic distribution: installs, firmware): the old
  SPI0 wiring collides with the MusicPi everywhere; re-home it on **PIO-SPI**
  using the HAT's last free pins (GP0/GP1/GP8, UART banner sacrificed --
  USB CDC is the console). PIO-SPI is proven on this port (cyw43 gSPI).
  Alternative (rejected): share SPI1 with the TFT via a mux task -- adds IPC
  to the display's pixel hot path.
- One net task, two smoltcp interfaces, same HTTP API + updater on both.
  Firmware transport preference: ethernet -> WiFi -> USB CDC, all feeding the
  same A/B updater (transport-agnostic back end, per the update design doc).

## A/B provisioning + updates

See docs/firmware-update-research/ota-update-design.md: implement the
Hubris `update-api` shape for RP2350 (rp235x-update-server: QSPI writes +
version-based slot logic), caboose for versions, salty/sha2 for signed
images + anti-rollback before any network-pull distribution. The A/B slots
hold the combined two-core image, so one update path covers both cores.

## Phasing

1. Inputs + mixer, single-core, shell-controlled (boards on order) -- prove
   capture -> mix/duck -> out.
2. CYW43 finish on a Pico 2 W under the MusicPi.
3. HTTP control API over WiFi (port the PR-#17 httpd; add mixer endpoints).
4. W5500 via PIO-SPI as the second interface.
5. Multicore split (audio engine to core 1) when CPU/RAM demand it -- with
   the warm-boot supervision designed in.
6. A/B provisioning + signed OTA (hardening pass).

Each phase ships something usable, MusicPi-style stage discipline.

## Open questions

- "Distribution" scope: firmware/control only (assumed), or **audio**
  distribution over ethernet (multi-room streaming)? The latter is a much
  bigger ask (RTP-ish transport, buffering, inter-unit sync) and would add a
  cross-core PCM ring the current split deliberately avoids.
- PCM1865 custom carrier: Pico 2 W + W5500 + PCM1865 + PCM5100A + TFT on one
  PCB -- every driver carries over, only pin constants move. Design the pin
  map once the breadboard architecture is proven.
- rp2350-amp RCP-salt caveat must be resolved before core-1 launch is
  production-grade.
- RAM budget tetris is perpetual: capture rings + WiFi + httpd all compete;
  the AMP split is what makes the sets fit.
