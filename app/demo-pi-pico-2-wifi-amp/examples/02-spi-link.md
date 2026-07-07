# Example 02 — SPI link (controller <-> peripheral + speed test)

Two boards on a real SPI0 bus: one **controller** clocks the bus, one
**peripheral** (slave) responds. Exercises the PL022 in slave mode over real
SCK/MOSI/MISO/CS wires (vs the single-board internal loopback). Board A =
controller, Board B = peripheral.

## Wiring (5 wires, on GP16-19)

SPI0 is muxed to **GP16-19**, on the right edge near the bottom-right corner
(the end away from USB). Counting up from that corner: GP16, GP17, GND(pin23),
GP18, GP19.

```
   BOARD A (controller)             BOARD B (peripheral)

   GP18 SCK  (pin24) ─────────────── (pin24) GP18 SCK     clock  (straight)
   GP17 CSn  (pin22) ─────────────── (pin22) GP17 CSn     select (straight)
   GP19 TX   (pin25) ────────────►   (pin21) GP16 RX      MOSI: A out -> B in
   GP16 RX   (pin21) ◄────────────   (pin25) GP19 TX      MISO: B out -> A in
   GND       (pin23) ─────────────── (pin23) GND          common
```

| Signal | A pin | | B pin | Type |
|---|---|---|---|---|
| SCK | GP18/pin24 | -- | GP18/pin24 | straight |
| CS  | GP17/pin22 | -- | GP17/pin22 | straight |
| MOSI | GP19/pin25 | -> | GP16/pin21 | cross |
| MISO | GP16/pin21 | <- | GP19/pin25 | cross |
| GND | GND/pin23 | -- | GND/pin23 | common |

Rule: **clock + chip-select straight, the two data lines cross** (each board's
TX to the other's RX), same idea as the UART crossover.

## Pre-flight (recommended): single-board SPI loopback

Before trusting 5 wires, confirm each board's SPI datapath with the built-in
internal loopback: `status` reports `spi: loopback OK` (no wiring needed -- the
PL022 loops TX->RX internally). If that passes on both boards, the SPI blocks
are healthy and only the interconnect is in question.

## Run it

```
# Board B (peripheral): become a slave and stage a response
hubris> spi role peripheral
spi role = peripheral (slave)
hubris> spi load de ad be ef
loaded 4 bytes into TX FIFO

# Board A (controller): clock a transfer
hubris> spi role controller
spi role = controller
hubris> spi xfer a5 5a 3c 00
rx: de ad be ef        <- the peripheral's staged bytes came back

# Board B: see what the controller clocked in
hubris> spi recv
rx 4: a5 5a 3c 00       <- the controller's bytes arrived
```

## Speed test

```
# Board A (controller):
hubris> spi bench 4096
spi: 4096 bytes in <ms> ms = <B/s> B/s (<pct>% of 187500 theoretical)
```

At 1.5 MHz SCK the line rate is 187500 B/s, but each byte crosses an IPC and a
FIFO poll, so expect the measured number well under that -- the gap is the
per-byte software overhead (and where DMA would help). `spi bench` runs on the
controller alone (RX shifts in line state), so it works with or without the
peripheral attached.

### Measured

Verified on hardware (two boards, 5-wire cross, controller <-> peripheral):

```
controller  spi xfer aa bb cc dd  ->  rx: 11 22 33 44   (peripheral's staged bytes)
peripheral  spi recv              ->  rx 4: aa bb cc dd  (controller's bytes, exact)

spi bench 4096  ->  56888 B/s (30% of 187500 theoretical @ 1.5 MHz SCK)
```

SPI runs ~5x faster than the UART link (56.9 KB/s vs 11.5 KB/s) but at only
30% of its line rate: the SCK is fast enough that the per-byte IPC + FIFO-poll
overhead dominates (the opposite of UART, which hit ~100% because its slow byte
period hides that overhead). This gap is where batching / DMA would pay off.

## Notes

- Clock speed is set by `CPSDVSR` in `drv/rp235x-spi/src/main.rs` (100 -> 1.5
  MHz at clk_peri 150 MHz). Lowering it raises SCK for a faster bench.
- The peripheral only shifts while the controller drives SCK, so `spi load`
  must run before the controller's `spi xfer`.
