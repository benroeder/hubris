# Example 05 — Sony S-Link / Control-A1 (one-wire control bus)

One board sends a Sony **S-Link** (aka **Control-A1**) control frame; the other
receives and decodes it. S-Link is the one-wire bus Sony A/V gear (amps, CD/MD
decks, tuners) uses for inter-component control. This shows Hubris bit-banging a
real, timing-critical consumer protocol on a bare GPIO.

Board A = sender, Board B = listener (either board can do both -- the line is
bidirectional).

## The protocol
A single **bidirectional, open-collector** wire, **idle HIGH** (pull-up). A
transmitter pulls it **LOW** for a mark whose *width* encodes the symbol, each
mark followed by a ~600 us HIGH delimiter:

| Symbol | LOW width |
|---|---|
| SYNC (frame start) | 2400 us |
| logical 1 | 1200 us |
| logical 0 | 600 us |
| delimiter (HIGH gap) | 600 us |

Bytes are **MSB-first**; a frame is **2-3 bytes** (device id + command(s)).
It's slow (~355 bps) but that makes it easy to bit-bang precisely. Decode
tolerance is +-20%.

## Wiring (1 signal wire + ground) -- reuses the I2C-SDA wire
S-Link needs just one shared wire. We drive it on **GP4** (the same pin/wire as
I2C SDA in example 03), so if that straight jumper + ground are in place, there
is nothing to rewire.

```
   BOARD A                          BOARD B
   GP4 (pin6) ────────────────────── (pin6) GP4     straight, single wire
   GND (pin8) ────────────────────── (pin8) GND     common
```

The driver reconfigures GP4 as an SIO open-drain line: "drive LOW" enables the
output (OUT held 0); "release" disables it so the pad pull-up returns the line
HIGH -- exactly how a real S-Link node behaves. Both boards' internal pull-ups
in parallel are enough on a short bench wire.

## Run it
```
# Board B (listener) first -- it waits up to 6 s for a frame:
hubris> slink listen 6000
listening 6000 ms...

# Board A (sender):
hubris> slink send 90 2e
sent 2 bytes: 90 2e

# Board B then prints:
rx 2 bytes: 90 2e
```

`slink send` takes 2 or 3 hex bytes (e.g. `slink send 90 2e 01`). `90` is a
typical device id; `2e` a command.

## Measured (verified on hardware, GP4 + GND, both directions)
```
A->B  slink send 90 2e      -> B rx 2 bytes: 90 2e        PASS
A->B  slink send 90 2e 01   -> B rx 3 bytes: 90 2e 01     PASS
A->B  slink send ff 00 aa   -> B rx 3 bytes: ff 00 aa     PASS   (all-1s / all-0s / alt)
B->A  slink send c3 55      -> A rx 2 bytes: c3 55        PASS   (reverse direction)
A->B  slink send 55 aa      -> B rx 2 bytes: 55 aa        PASS
```

Every bit pattern and both directions decode byte-exact -- 0xff is the longest
run of 1200 us marks, 0x00 the shortest 600 us marks, 0x55/0xaa the alternating
worst case; all pass, so the sender's timing and the receiver's width
classification are solid.

## Stress test (`slink flood` / `slink soak`)
`slink flood <n>` sends `n` self-checking frames `[seq, seq^0xa5, seq+0x33]`
back-to-back (seq wraps 0..255, so `n>=256` exercises **every byte value**);
`slink soak <n> <ms>` receives and validates each independently (so a dropped
frame never desyncs the rest) and reports errors plus the measured mark-width
extremes.

```
# Board B:                     # Board A:
hubris> slink soak 1000 90000  hubris> slink flood 1000
```

Verified on hardware:

| Run | Frames | Errors | ones width (nom 1200) | zeros width (nom 600) |
|---|---|---|---|---|
| idle          | 500  | **0** | 1185-1195 us | 590-600 us |
| under IRQ load | 1000 | **0** | 1165-1195 us | 570-600 us |

Zero bit errors over 1500 frames total, both directions and every byte value.
The second run adds heavy USB-RX interrupt load to the listener *during* its
bit-timing: the measured width spread widens from ~10 us to ~30 us -- visible
SysTick/USB-IRQ jitter -- but stays far from the 900/1800 us decision thresholds,
so the protocol's +-20% tolerance absorbs it with no errors. That margin is the
headline result: the bit-bang timing is solid enough to survive real interrupt
load.

## Notes
- **Timing** is `cortex_m::asm::delay` (150 cycles/us @ 150 MHz) for the sender;
  the receiver measures each LOW mark by counting fixed delay steps. The marks
  are hundreds of microseconds, so the occasional 1 ms SysTick / USB IRQ jitter
  stays well inside the +-20% tolerance.
- **Real Sony gear:** this speaks the same electrical + framing protocol, so
  with a 3.5 mm minijack to a Sony component's CONTROL-A1 port it should drive /
  read real device commands (device ids and command tables are published in the
  Control-A1 references). Untested against real hardware here.
- Driver: `drv/rp235x-slink`; the GP4 line is shared with I2C's SDA position, so
  I2C and S-Link are mutually exclusive on a given build.
