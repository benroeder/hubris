# RP2350 two-board demos

Documented, reproducible examples that exercise the Hubris RP2350 bus drivers
against *real external signals* between two Pico 2 / Waveshare RP2350 boards,
each running this demo firmware. Every example has a wiring diagram, the exact
shell commands for each board, expected output, and a measured speed number.

Both boards run the same image (distinguished by their per-board USB serial
`H<chipid>`); open each board's `/dev/cu.usbmodem*` console in a separate
terminal. Drive one board, observe the other.

| # | Example | Buses | What it shows |
|---|---------|-------|---------------|
| 01 | [UART link](01-uart-link.md) | UART0 | cross-board messaging + throughput |
| 02 | [SPI link](02-spi-link.md) | SPI0 | controller/peripheral exchange + throughput |
| 03 | [I2C target](03-i2c-target.md) | I2C0 | ACKed transaction + throughput |
| 04 | [Firmware push](04-uart-update.md) | UART0 | one board updates the other over the wire |

## Speed comparison (filled in as examples land)

| Bus | Clock | Measured | Theoretical | % of max |
|-----|-------|----------|-------------|----------|
| UART0 | 115200 8N1 | 11636 B/s | 11520 B/s | ~100% |
| SPI0 | 1.5 MHz | 56888 B/s | 187500 B/s | 30% |
| I2C0 | 100 kHz | 10113 B/s | 11111 B/s | 91% |
| I2C0 fast | 400 kHz | 33573 B/s | 44444 B/s | 75% |
| I2C0 FM+ | 1 MHz | 64000 B/s | 111111 B/s | 57% |
| USB CDC | 12 Mbit FS | ~16 KB/s | — | — |

Firmware transfer (66 KB image, end-to-end incl. flash writes):

| Transport | Time | Effective |
|-----------|------|-----------|
| USB | ~3.6 s | ~18 KB/s |
| UART | ~10 s | ~6.6 KB/s (per-page CRC) |
| SPI | tbd | tbd |

The drivers poll the peripheral FIFO one byte per IPC, so measured throughput
sits well under the line-rate theoretical max; that gap is itself a result and
shows where batching / DMA would pay off.
