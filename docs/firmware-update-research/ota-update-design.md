# RP2350 firmware-update / OTA design notes

Status: **research / decision record** (2026-07-15). Not yet implemented.
Context: we shipped an ad-hoc `POST /update` in `task/rp235x-net` that writes
the stale A/B slot over the web UI (unbrickable, version-based boot). This note
captures how to evolve that into a "correct" update story, and what to reuse.

## 1. Push vs pull (chosen by scale, not correctness)

- **Push (what we have):** operator uploads the image to the device's web UI.
  Zero infrastructure, works on an isolated LAN, no outbound internet, full
  operator control. Right for one board / a dev loop. Does not scale.
- **Pull (device polls a server):** device periodically checks a manifest, sees
  a newer version, fetches + verifies + writes the stale slot. Standard for
  **fleets** (cf. Mender, hawkBit, Balena, ESPHome/Tasmota). Needs a server and
  outbound network on the device.

Decision: for the bench (one board), the web-UI push is fine — do **not** add a
server yet. Pull is the right shape only if we want self-updating boards.

## 2. Transport (RSS / feed) is a footnote

A feed is just a **manifest**: `{version, url, size, hash, signature}`. RSS/Atom
works but XML is expensive to parse no_std; a tiny **signed JSON manifest** is
more idiomatic. The manifest format is the least important decision.

## 3. The decision that actually matters: signing

Today the device flashes **whatever bytes it is handed** — CRC (integrity) + the
bootrom hash check, but **no authenticity**. Fine for push on a trusted LAN
where the operator is the uploader. The moment the device *pulls from a
network*, "flash whatever the server says" is remote-code-execution-as-a-service.

Before any pull model: the device MUST **verify a cryptographic signature** over
the image against a public key baked into the firmware, and reject downgrades
(anti-rollback). Ed25519 (small, no_std) is the usual pick.

TLS on RP2350 is heavy (no crypto accel, tight RAM), so the pragmatic embedded
pattern is **signed images over plain HTTP** — trust the signature, not the
transport. That fits smoltcp far better than HTTPS.

## 4. What to reuse from Hubris (this is Oxide's tree)

Reuses cleanly:
- **`update-api` Idol interface** (`idl/stm32h7-update.idol`, `idl/lpc55-update.idol`):
  `prep_image_update` / `write_one_block` / `finish_image_update` /
  `abort_update` / `current_version` / `read_caboose_value` /
  `get_pending_boot_slot` / `switch_default_image` / `reset`. Our `/update`
  reinvents a crude subset. Adopt this **interface shape** so the web UI and any
  future pull-client call one proper update-server.
- **Caboose** (`drv/caboose`): images carry a metadata blob (version, git hash,
  board). The correct replacement for our hand-rolled offset reads
  (`VER_OFF=0x184`, `HDR_OFF=0x110`) in `task/rp235x-net`.
- **In-tree crypto**, all no_std, already vetted: **`salty`** (ed25519),
  **`sha2`/`sha3`**, **`p256`** (ECDSA). Use these to verify signed images.

Does NOT reuse:
- **No `rp235x-update-server` exists.** `stm32h7-update-server` /
  `lpc55-update-server` implement their chip's flash mechanics (STM32/LPC
  internal flash) — not our QSPI-over-QMI.
- **Boot model differs — the real blocker.** The Hubris update-servers assume a
  **Hubris-managed bootloader** with a persistent "preferred slot" flag — that
  is exactly what `set_pending_boot_slot` / `switch_default_image` write. On
  RP2350 the **silicon bootrom** selects the slot by **image version**; there is
  no preferred-slot flag to set. This mismatch is why "change the boot image"
  (force the other slot) is hard here — the API has an op, the chip has no
  mechanism behind it. See [[rp235x-w5500-ethernet]] (the httpd/A-B work).
- **LPC55 path is welded to the Oxide RoT** (DICE, sprot, ROM crypto,
  manufacturing keys). Not portable to a Pico.

## 5. Recommended path (own branch, after the I2S work)

1. Implement the **`update-api` shape for RP2350**: a real `rp235x-update-server`
   doing the QSPI writes + the bootrom version-based slot logic, replacing the
   ad-hoc `/update`.
2. Reuse **caboose** for versions and **salty + sha2** for signed images
   (our own keys, NOT Oxide's infra).
3. Keep **transport orthogonal**: the update-server is the local flash-write
   side; the web-UI upload or a fetch-from-server + verify client sits in front
   and calls the same API.

For the demo we have now, the shipped `/update` is fine — this is a future,
separately-scoped piece of work.
