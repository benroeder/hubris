# Example 04 — cross-board firmware update over UART

One board ("updater") streams a firmware image to another board ("target")
over the wire; the target writes it to flash and reboots into it. **No PC
carries the image** -- the host only kicks off the two commands. This is the
real Hubris/Oxide field-update pattern (a controller updating a device it
manages), and it reuses everything the earlier stages built: the paced
protocol, the flash driver, the A/B slot logic, and the UART link.

Board A = updater, Board B = target. Uses the same UART crossover as example 01
(GP0/GP1/GND) -- if you left the wires in, nothing to rewire.

## How it works

The update path is transport-agnostic (`receive_update(Link, size, crc)` in the
shell): read `size` bytes from a link in 256-byte pages, one `.` ACK per page
back over the same link, write each page to flash, then CRC-verify the whole
image by readback. `Link` is `Usb` (the host `update` command) or `Uart` (this
example). The updater's `push` streams its own flash image over UART with the
same protocol.

```
   [host] --triggers-->  A: push <size> <crc>  --UART-->  B: uart-update <size> <crc>
                         (streams A's flash)               (writes flash, verifies)
```

## Run it

Compute the image size and CRC32 on the host (the image is whatever is in the
updater's flash slot 0):

```
size = <bytes of the image>          # e.g. 66624 = 0x10440
crc  = crc32(image)                  # e.g. 0e350eda
```

Then, in order:

```
# Board B (target) FIRST -- it sends GO and waits for the image:
hubris> uart-update 10440 0e350eda
waiting for image over UART...
target flash 0x00000000

# Board A (updater) -- streams its own flash to B:
hubris> push 10440 0e350eda
waiting for peer GO over UART...
push OK: peer verified; 66624 bytes in 9386 ms

# Board B then reports:
OK crc verified; `reboot` to apply

# Apply on B:
hubris> reboot         # ONLY if B said "OK crc verified"
```

B boots the pushed image. (Build the updater with a distinct greeting to see B
visibly change.)

### Measured

Verified on hardware: **66624 bytes, A -> B over UART, ~9.4 s**, CRC-verified
byte-exact. That is ~7 KB/s effective -- below the raw 11.5 KB/s UART line rate
(example 01) because each page waits for B's ACK *and* B's flash write
(sector erase + program) before the next page.

## Important caveats (and the roadmap)

- **In-place is not crash-safe.** On a single-image board `target flash` is
  `0x00000000` -- B overwrites its only image. If the transfer is corrupted or
  interrupted and you reboot anyway, B is bricked until BOOTSEL. The end-to-end
  CRC *catches* corruption ("do NOT reboot; retry") but does not prevent it.
- **The fix is A/B.** Provision B with the partition table (the A/B machinery on
  `rp2350-port`): `target flash` becomes the *inactive* slot, the ROM boots the
  higher version, and the running image stays as an automatic fallback --
  **unbrickable**, and the real point of this stage.
- **Per-page integrity is the next robustness step.** Today a single UART bit
  error fails the whole transfer (end-to-end CRC, full retry). A per-page
  checksum + page-level retry would recover from transient errors without
  restarting the 66 KB transfer -- important on a raw UART with no flow control.

## Next transports

The `Link` abstraction is the point: SPI and I2C updates (examples 05, 06) are
new `Link` variants + their controller-driven pacing (the updater clocks/
addresses the target rather than the target reading freely). Same receive/write/
verify core, faster wire -- the SPI push should drop the ~9 s to ~1 s.
