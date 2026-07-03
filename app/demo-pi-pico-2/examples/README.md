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
| 02 | SPI link (planned) | SPI0 | controller/peripheral exchange + throughput |
| 03 | I2C target (planned) | I2C0 | ACKed transaction + throughput |
| 04 | Firmware push (planned) | UART0 | one board updates the other's A/B slot |

## Speed comparison (filled in as examples land)

| Bus | Clock | Measured | Theoretical | % of max |
|-----|-------|----------|-------------|----------|
| UART0 | 115200 8N1 | 11636 B/s | 11520 B/s | ~100% |
| SPI0 | _tbd_ | _tbd_ | SCK/8 | _tbd_ |
| I2C0 | _tbd_ | _tbd_ | SCK/9 | _tbd_ |
| USB CDC | 12 Mbit FS | ~16 KB/s | — | — |

The drivers poll the peripheral FIFO one byte per IPC, so measured throughput
sits well under the line-rate theoretical max; that gap is itself a result and
shows where batching / DMA would pay off.
