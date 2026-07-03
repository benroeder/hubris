# Example 03 — I2C target (ACKed transaction + speed test)

One board becomes an **I2C target** (slave) that ACKs an address and serves a
byte window; the other is the **controller** that scans the bus, finds it, and
reads it back. Exercises the DW_apb_i2c in slave mode over a real bus (vs the
single-board NAK-on-empty-bus scan). Board A = target, Board B = controller.

## Wiring (2 signal wires + ground, shared bus)

I2C0 is muxed to **GP4 = SDA, GP5 = SCL** (funcsel 3), on the left edge near the
USB end. I2C is a shared bus: connect like to like (no crossover), plus ground.

```
   BOARD A (target)                 BOARD B (controller)

   GP4 SDA (pin6) ────────────────── (pin6) GP4 SDA     straight
   GP5 SCL (pin7) ────────────────── (pin7) GP5 SCL     straight
   GND     (pin8) ────────────────── (pin8) GND         common
```

| Signal | A pin | | B pin | Type |
|---|---|---|---|---|
| SDA | GP4/pin6 | -- | GP4/pin6 | straight |
| SCL | GP5/pin7 | -- | GP5/pin7 | straight |
| GND | pin8 | -- | pin8 | common |

**Pull-ups:** both boards enable I2C0's internal pull-ups (~50k each) on SDA/SCL,
so on a short bench wire the two in parallel are usually enough. If the bus is
flaky, add external ~4.7k pull-ups to 3V3.

## Run it

```
# Board A: become an I2C target at 0x42 serving "HUBRIS!"
hubris> i2c target 42 48554252495321
serving 7 bytes at 0x42 (this board is now an I2C target)

# Board B: scan the bus and read the target
hubris> i2c scan
0x42
hubris> i2c read 42 7
rx: 48 55 42 52 49 53 21          <- "HUBRIS!"
```

`48554252495321` is ASCII `H U B R I S !`. The target keeps serving until it is
rebooted (slave mode takes over I2C0; the controller ops on that board stop
working until reboot).

## Speed test

```
# Board B (controller):
hubris> i2c read 42 <n>   # repeated reads; time in the shell / logic analyzer
```

At 100 kHz the line rate is ~11 kB/s (9 bits/byte with ACKs). Like SPI, the
per-byte IPC + FIFO overhead means the measured rate sits under that.

### Measured

Verified on hardware (two boards, GP4/GP5 + GND, internal pull-ups):

```
target   i2c target 42 48554252495321   -> serving 7 bytes at 0x42
control  i2c scan                       -> i2c:  0x42        (target ACKs)
control  i2c read 42 7                  -> rx: 48 55 42 52 49 53 21   ("HUBRIS!")
```

Closes the long-staged ACK test (previously only NAK-on-empty-bus was verified).

**Speed sweep** (`i2c speed <khz>` then `i2c bench 42 4096`, both boards set):

| Mode | Clock | Measured | Efficiency |
|------|-------|----------|-----------|
| standard       | 100 kHz | 10113 B/s | 91% |
| fast           | 400 kHz | 33573 B/s | 75% |
| fast-mode-plus | 1 MHz   | 64000 B/s | 57% |

Same driver, same code -- just a faster clock -- and the efficiency *drops*
(91 -> 75 -> 57%) as the per-byte IPC/FIFO overhead becomes a bigger fraction of
each byte's shrinking on-wire time. That is the exact mechanism that caps SPI
(30% at 1.5 MHz). In absolute terms fast-mode-plus I2C (64 KB/s) even edges past
SPI (56.9 KB/s). (`i2c scan` sweeps 112 addresses, each NAK timing out on an
empty slot, so a full scan takes ~1.5 s at 100 kHz -- that is scan cost, not bus
speed.)

## Notes

- The target is interrupt-driven: the DW slave stretches SCL on a read-request
  until the driver reloads its TX FIFO, so it never misses a byte regardless of
  IPC latency (`drv/rp235x-i2c/src/main.rs`, RD_REQ handler).
- Two register traps behind the driver (both livelocked the priority-2 I2C task
  and killed the console): IC_INTR_MASK resets to 0x8ff so a partial write
  leaves TX_EMPTY firing, and the PAC's mask enabled()/disabled() enum is
  inverted vs the hardware. The mask is written as raw bits (0x64).
