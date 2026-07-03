# Example 01 — UART link (board-to-board messaging + speed test)

The simplest two-board demo. Two boards talk over UART0 with the existing driver
(no peripheral-mode work needed -- UART is symmetric, both boards TX and RX).
Sends a message each way and measures throughput.

## Wiring

Cross-connect the two UART0s (one board's TX to the other's RX) and share
ground. UART0 is GP0 = TX, GP1 = RX on both boards.

```
      Board A                         Board B
   (H....serialA)                 (H....serialB)

   GP0 (TX)  o--------------------->o  GP1 (RX)
   GP1 (RX)  o<---------------------o  GP0 (TX)
   GND       o----------------------o  GND

   USB to host                     USB to host
   (console A)                     (console B)
```

- Remove any GP0->GP1 self-loopback jumper first (that was for the single-board
  `status` self-test).
- 3 wires total: A.GP0->B.GP1, B.GP0->A.GP1, GND-GND.
- Each board keeps its own USB cable to the host for its console.

## Run it

Flash both boards with this demo firmware, then open both consoles.

Messaging (type on A, read on B):

```
# On board A:
hubris> uart send hello from A
sent 15 bytes

# On board B:
hubris> uart recv
rx 15 bytes: hello from A
```

Reverse direction works the same (`uart send` on B, `uart recv` on A).

## Speed test

`uart bench [n]` sends N bytes (default 4096) and times it. `write` blocks on the
TX-FIFO, so the elapsed time reflects the wire rate. Optionally run
`uart rxbench` on the far board to confirm the receive rate matches.

```
# On board B (start the receive counter first):
hubris> uart rxbench

# On board A (within the 3 s window):
hubris> uart bench 4096
uart tx: 4096 bytes in <ms> ms = <B/s> B/s (<pct>% of 11520 theoretical)

# Board B then prints:
uart rx: <bytes> bytes in 3000 ms = <B/s> B/s (<pct>% of 11520 theoretical)
```

### Measured

UART0 at 115200 8N1, theoretical max 11520 B/s (10 bits/byte). Measured on
hardware:

```
uart bench 4096  -> 11636 B/s (101% of theoretical)
uart bench 16384 -> 11546 B/s (100% of theoretical)
```

UART runs essentially at line rate -- the byte period (~87 us) dwarfs the
per-byte IPC/FIFO-poll overhead, so the driver keeps the wire full. (The tiny
overshoot past 100% is the last FIFO load still draining when the timer stops.)
This is the opposite of SPI, where the clock is fast enough that the poll
overhead will dominate -- see example 02.

## Notes

- 115200 8N1 is set in `lib/rp235x-uart/src/lib.rs` (`BAUD`); raising it there
  (and rebuilding both boards) lets the bench show a higher number.
- The driver moves one byte per FIFO poll, so expect close to the line rate here
  (UART is slow enough that per-byte overhead is negligible) -- unlike SPI, where
  the IPC/poll overhead will dominate.
