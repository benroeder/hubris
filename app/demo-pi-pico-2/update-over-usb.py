#!/usr/bin/env python3
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
"""Update a running Hubris/RP2350 board over its USB console.

Usage: update-over-usb.py <serial-device> <final.bin>

Speaks the shell's `update` protocol: sends the image size and CRC32,
streams the image in 256-byte pages (each ACKed with a `.` by the board,
which paces the transfer), waits for the on-board CRC verification, and
reboots into the new image. No BOOTSEL, no probe, no picotool.
"""

import binascii
import os
import select
import sys
import time


def read_until(fd, token, timeout):
    """Read from fd until `token` appears or `timeout` seconds pass."""
    buf = b""
    deadline = time.time() + timeout
    while time.time() < deadline:
        r, _, _ = select.select([fd], [], [], 0.2)
        if r:
            try:
                buf += os.read(fd, 4096)
            except OSError:
                pass
            if token in buf:
                return buf
    raise TimeoutError(f"waiting for {token!r}; got {buf[-120:]!r}")


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    dev, image_path = sys.argv[1], sys.argv[2]
    image = open(image_path, "rb").read()
    crc = binascii.crc32(image) & 0xFFFFFFFF
    print(f"image: {len(image)} bytes, crc32 {crc:08x}")

    fd = os.open(dev, os.O_RDWR | os.O_NONBLOCK)
    time.sleep(0.3)
    os.write(fd, f"update {len(image):x} {crc:x}\r".encode())
    read_until(fd, b"GO", 5)

    t0 = time.time()
    for off in range(0, len(image), 256):
        os.write(fd, image[off : off + 256])
        read_until(fd, b".", 10)  # per-page ACK paces the stream
        done = off + 256
        if done % 16384 < 256:
            print(f"  {min(done, len(image))}/{len(image)} bytes")

    result = read_until(fd, b"\r\n", 30)
    tail = read_until(fd, b"reboot", 30) if b"OK" not in result else result
    print(tail.decode("ascii", "replace").strip().splitlines()[-1])
    if b"OK" not in tail:
        sys.exit("update FAILED -- board not rebooted; safe to retry")

    print(f"transfer+verify took {time.time() - t0:.1f}s; rebooting...")
    os.write(fd, b"reboot\r")
    time.sleep(0.5)
    os.close(fd)
    print("done -- board is booting the new image")


if __name__ == "__main__":
    main()
