# Scope: PIO gSPI PHY for the CYW43439 (Pico 2 W)

Goal: replace the failed bit-bang with a **PIO-driven gSPI PHY** and reach
**chip-detect** (read `0xFEEDBEAD` from `REG_BUS_TEST_RO`). This is the one
remaining prerequisite for the whole Wi-Fi driver; the firmware-storage half
(auxflash) is already done + verified. It also stands up the **first PIO
capability in the RP2350 port** -- reusable for future PIO peripherals.

"Done" for this scope = `cmd_read(swap16(cmd_word(READ,INC,0,0x14,4)))` returns
`0xBEADFEED` (= swap16(FEEDBEAD)) reliably, read out over the probe.

## 1. The reference PIO program (embassy cyw43-pio) -- ~10 instructions

```
.side_set 1                 ; CLK = side-set pin (GP29)
.wrap_target
lp:
  out pins, 1   side 0      ; drive next cmd bit onto DIO (GP24), CLK low
  jmp x-- lp    side 1      ; ...clock it in on CLK rising; loop X+1 = write bits
  set pindirs,0 side 0      ; TURNAROUND: DIO output -> input, CLK low
  nop           side 1
  nop           side 0
lp2:
  in pins, 1    side 1      ; sample DIO on CLK high; autopush to RX FIFO
  jmp y-- lp2   side 0      ; loop Y+1 = read bits
  wait 1 pin 0  side 0      ; wait for the device's response/IRQ line
  irq 0         side 0      ; signal the CPU that the transaction is done
.wrap
```

Protocol: the SM pulls the write-bit-count into X and the read-bit-count into Y,
`out`s the command+data words (autopull, MSB-first), flips DIO to input, then
`in`s the response (autopush). CLK is entirely side-set -- deterministic, which
is exactly what the bit-bang lacked. (embassy ships two variants -- a plain one
and an "overclock" one for >=75 MHz PIO clock; for bring-up we run the plain one
SLOW, ~1-4 MHz, then raise the clock divider later.)

## 2. RP2350 PIO essentials
- **3 PIO blocks** (PIO0/1/2), each **4 state machines**, **32 shared instr
  slots**, per-SM 4-word TX/RX FIFOs (joinable to 8), clock divider, and pin
  maps (in/out/set/side-set).
- **Pin funcsel** to PIO: PIO0 = 6, PIO1 = 7, PIO2 = 8 (vs SIO=5). GP24 (DIO),
  GP29 (CLK) go to PIO; **CS (GP25) stays CPU-driven** (SIO) around each
  transaction, as embassy does.
- Registers via `rp235x-pac` (`PIO0`): `sm0_*` (clkdiv, execctrl, shiftctrl,
  pinctrl), `instr_mem0..31`, `txf0/rxf0`, `fstat`, `sm0_instr` (exec one op).

## 3. Hubris integration design
- **Ownership:** only the cyw43 driver needs PIO now, so it owns PIO0 via
  `uses = ["pio0"]` (a new peripheral grant in the chip's `chip.toml`). No
  generic PIO server yet -- factor one out only when a 2nd PIO user appears
  (YAGNI; mirror how embassy keeps cyw43-pio cyw43-specific).
- **Program assembly:** use the `pio` crate's `pio_asm!` (compile-time assemble
  to `[u16; N]`), if it builds no_std for `thumbv8m` in the Hubris workspace.
  FALLBACK: hand-assemble the ~10 ops to a `const [u16; N]` (each op is one
  16-bit word; the encoding is in the RP2350 datasheet PIO chapter).
- **Pin mux:** GP24/GP29 -> PIO0 funcsel in the app's privileged pre-kernel
  `main` (same place we mux UART/SPI/I2C/LED); GP25 (CS) + GP23 (WL_ON) stay SIO.
- **Transaction API** = embassy's `SpiBusCyw43` contract: `cmd_read(cmd, &mut
  [u32])` and `cmd_write(&[u32])`. Drive by: CS low (SIO) -> push bit-counts +
  words to TXF -> poll `fstat`/RX FIFO (or the SM's `irq 0`) -> read RXF -> CS
  high.
- **Poll, no DMA, no IRQ initially.** Chip-detect is single 32-bit words --
  FIFO polling is plenty. DMA + the SM IRQ are a later optimization, and matter
  only for streaming the 224 KB firmware blob (P4, out of this scope).

## 4. Work breakdown (each independently verifiable over the probe)

**P1 -- PIO plumbing proof (de-risk PIO itself, no gSPI).** New `drv/rp235x-pio`
(or the cyw43 crate skeleton): `uses=["pio0"]`, load a trivial program (e.g.
toggle a spare GPIO at a known rate, or push a constant to RX FIFO), and confirm
it runs -- read the pin/FIFO over the probe. Proves: funcsel-to-PIO, instruction
memory load, SM config, FIFO access. THIS is where the real unknowns live.

**P2 -- gSPI PHY.** Load the ~10-instr program; configure the SM (side-set=GP29,
out/in/set pins=GP24, MSB-first shift, autopull/autopush, clkdiv for ~1-4 MHz).
Implement `cmd_write`/`cmd_read` (bit-count feeding + FIFO). Verify with a
loopback or a scope-able waveform first.

**P3 -- chip-detect (the payoff).** Wire the init: WL_ON low 20 ms / high 250 ms,
then loop `read32_swapped(FUNC_BUS, 0x14)` until `FEEDBEAD`. Verify: raw
`0xBEADFEED` on the wire -> `FEEDBEAD` decoded. **MILESTONE 1 DONE.**

(Beyond this scope: P4 write REG_BUS_CTRL, ALP clock, ramp the PIO clock, then
stream the WIFI blob from the auxflash server -> firmware upload -> LED.)

## 5. Risks / unknowns
- **First PIO on the port** -- no precedent; P1 exists to burn this down early.
- **`pio` crate no_std fit** in the Hubris workspace -- verify early; hand-assembly
  is a safe fallback (only ~10 words).
- **rp235x-pac PIO coverage** -- CONFIRMED present (rp235x-pac 0.2.0 has
  `inner/pio0/`: `instr_mem`, `txf`, `rxf`, `ctrl`, `fstat`, `fdebug`, per-SM
  regs). One thing to confirm in P1: the top-level `Peripherals` exposure/naming.
- **`pio` assembler crate** -- not yet in the workspace Cargo.lock; add as a
  build-dep or hand-assemble the ~10 words.
- **Turnaround / `wait 1 pin 0`** -- the program waits on the device IRQ line
  (DIO shared with IRQ when CS high); confirm the pin mapping for `wait`. For a
  pure register read this may need adjusting vs a data transfer.
- **Proven on this silicon:** pico-sdk's cyw43 driver + embassy both run this
  program on the RP2350/Pico 2 W, so the approach is known-good -- the work is
  the Hubris plumbing, not the PIO logic.

## 6. Effort
- P1: ~1 session (the plumbing is the risk, not the code volume).
- P2: ~1-2 sessions (SM config + the cmd_read/write FIFO dance).
- P3: quick once P2 clocks correctly.
=> ~2-4 focused sessions to chip-detect. The rest of the driver (P4+) is more,
but the auxflash streaming path is already built and waiting.

## 7. Decisions to confirm before starting
1. Crate shape: one `drv/rp235x-cyw43` owning PIO0, or split `rp235x-pio`
   (generic) + `rp235x-cyw43` (logic)? Recommend: single cyw43 crate now, split
   later if a 2nd PIO user appears.
2. `pio_asm!` crate vs hand-assembled const -- try the crate, fall back to hand.
3. PIO block: PIO0 (fine; all three are equivalent).

## References
- embassy `cyw43-pio/src/lib.rs` (the program above + SM setup).
- pico-sdk `cyw43_bus_pio_spi.pio` (the C/pioasm original).
- RP2350 datasheet PIO chapter (instruction encoding, SM config regs) --
  archived under docs/rp2350-research.
