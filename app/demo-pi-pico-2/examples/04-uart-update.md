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

### A/B (unbrickable) variant

Provision the target with two slots so the push never overwrites the running
image:

```
# once, from BOOTSEL: partition table + slot A (v2) + slot B (v1)
picotool partition create ab.json pt.bin -t bin
# combine pt.bin @0, slotA @0x2000, slotB @0x42000 into one image; picotool load it
```

B boots slot A (v2). `push` a v3 image: B writes the *inactive* slot B
(`target flash 0x00042000`), reboots, and the ROM boots the highest version
(v3). `slot` afterwards shows `verA=2.0 verB=3.0 target=0x2000` -- slot A is
untouched, so an interrupted transfer just leaves B booting v2. Unbrickable.

### Measured

Verified on hardware, **visibly**: board A (built with a distinct greeting)
pushed its ~66 KB flash image to board B over UART; B verified byte-exact and
rebooted into it -- its greeting changed to the pushed image's.

```
A: push OK: peer verified; 66912 bytes in 10146 ms   (~6.6 KB/s)
B: OK crc verified; `reboot` to apply
B (after reboot): Hubris [PUSHED-OVER-UART-v2] ...
```

~6.6 KB/s effective -- below the raw 11.5 KB/s UART line rate (example 01)
because each page waits for B's ACK, B's flash write (sector erase + program),
and now carries a 2-byte checksum.

## Important caveats (and the roadmap)

- **In-place is not crash-safe.** On a single-image board `target flash` is
  `0x00000000` -- B overwrites its only image. If the transfer is corrupted or
  interrupted and you reboot anyway, B is bricked until BOOTSEL. The end-to-end
  CRC *catches* corruption ("do NOT reboot; retry") but does not prevent it.
- **A/B: DONE and proven unbrickable.** Provision B with a partition table +
  two image slots (picotool combines `pt + slotA(v2) + slotB(v1)`); B boots the
  higher version (slot A, v2). A push then reports `target flash 0x00042000` --
  the *inactive* slot B -- writes there, and on reboot the ROM boots the highest
  version (the pushed v3 in slot B). Verified on hardware: after the push B
  boots `[PUSHED-A-B-v3]` and `slot` shows `verA=2.0 verB=3.0 target=0x2000` --
  **slot A (v2) is untouched**, so a failed/interrupted transfer never bricks the
  running image. This is the real, unbrickable form of the stage.
- **Per-page integrity: DONE.** Each UART page carries a 2-byte checksum; a bad
  page is NAK'd (`!`) and the sender resends just that page (bounded retries),
  so a transient bit error no longer fails the whole 66 KB transfer. (An earlier
  run without this hit a UART glitch, the end-to-end CRC caught it, but an
  in-place reboot into the corrupt image bricked the target -- motivating both
  this and A/B.) USB CDC is reliable, so it skips the per-page check.

## Next transports

The `Link` abstraction is the point: SPI and I2C updates (examples 05, 06) are
new `Link` variants + their controller-driven pacing (the updater clocks/
addresses the target rather than the target reading freely). Same receive/write/
verify core, faster wire -- the SPI push should drop the ~9 s to ~1 s.
