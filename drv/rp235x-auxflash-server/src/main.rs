// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Read-only auxiliary-flash server for the RP2350 (RP235x).
//!
//! A faithful, read-only port of `drv/auxflash-server`. Instead of driving a
//! dedicated QSPI flash chip, it reads the auxflash blob store from a REGION of
//! the main QSPI flash, which the RP2350 exposes MEMORY-MAPPED through the
//! uncached no-translate XIP mirror (the `cyw43_fw` extern-region at
//! 0x1c20_0000). So the `TlvcRead` impl is just a `copy` from flash -- no SPI
//! transactions. The image is flashed once, externally (picotool / probe-rs to
//! 0x1020_0000), so erase/write/redundancy are unsupported here.
//!
//! Everything else -- TLV-C parsing, SHA3 checksum verification, active-slot
//! selection, and `get_blob_by_tag` -- is the shared `drv-auxflash-api`
//! machinery over a `TlvcRead`. The cyw43 driver is the client:
//! `get_blob_by_tag(*b"WIFI")` then `read_slot_with_offset` to stream it.

#![no_std]
#![no_main]

use drv_auxflash_api::{
    AuxFlashBlob, AuxFlashChecksum, AuxFlashError, AuxFlashId, TlvcReadAuxFlash,
    SLOT_COUNT, SLOT_SIZE,
};
use idol_runtime::{
    ClientError, Leased, NotificationHandler, RequestError, W,
};
use tlvc::{TlvcRead, TlvcReadError};
use userlib::RecvMessage;

extern "C" {
    /// Base of the auxflash region (the `cyw43_fw` grant, no-translate XIP
    /// mirror of the reserved physical flash).
    static __REGION_CYW43_FW_BASE: [u8; 0];
}

fn aux_base() -> usize {
    &raw const __REGION_CYW43_FW_BASE as usize
}

/// A `TlvcRead` over one auxflash slot in the memory-mapped flash region.
#[derive(Copy, Clone)]
struct SlotReader {
    /// Byte offset of this slot from the region base.
    base: u32,
}

impl TlvcRead for SlotReader {
    type Error = AuxFlashError;
    fn extent(&self) -> Result<u64, TlvcReadError<Self::Error>> {
        Ok(SLOT_SIZE as u64)
    }
    fn read_exact(
        &self,
        offset: u64,
        dest: &mut [u8],
    ) -> Result<(), TlvcReadError<Self::Error>> {
        let src = aux_base() + self.base as usize + offset as usize;
        // SAFETY: `src..src+len` is inside the `cyw43_fw` region granted to this
        // task; the TLV-C reader keeps offsets within the slot. The flash mirror
        // is plain read-only memory (no side effects).
        unsafe {
            core::ptr::copy_nonoverlapping(
                src as *const u8,
                dest.as_mut_ptr(),
                dest.len(),
            );
        }
        Ok(())
    }
}

include!(concat!(env!("OUT_DIR"), "/checksum.rs")); // const AUXI_CHECKSUM

struct ServerImpl {
    active_slot: Option<u32>,
}

impl idl::InOrderAuxFlashImpl for ServerImpl {
    fn read_id(
        &mut self,
        _: &RecvMessage,
    ) -> Result<AuxFlashId, RequestError<AuxFlashError>> {
        // No separate flash chip to interrogate; the store is a region of the
        // main flash. Report a synthetic id.
        Ok(AuxFlashId {
            mfr_id: 0,
            memory_type: 0,
            capacity: 0,
            unique_id: [0; 8],
        })
    }

    fn read_status(
        &mut self,
        _: &RecvMessage,
    ) -> Result<u8, RequestError<AuxFlashError>> {
        Ok(0) // memory-mapped read-only: never busy
    }

    fn slot_count(
        &mut self,
        _: &RecvMessage,
    ) -> Result<u32, RequestError<AuxFlashError>> {
        Ok(SLOT_COUNT)
    }

    fn slot_size(
        &mut self,
        _: &RecvMessage,
    ) -> Result<u32, RequestError<AuxFlashError>> {
        Ok(SLOT_SIZE as u32)
    }

    fn read_slot_chck(
        &mut self,
        _: &RecvMessage,
        slot: u32,
    ) -> Result<AuxFlashChecksum, RequestError<AuxFlashError>> {
        Ok(read_and_check_slot_checksum(slot)?)
    }

    // --- write path: unsupported (blob flashed externally, read-only region) --
    fn erase_slot(
        &mut self,
        _: &RecvMessage,
        _slot: u32,
    ) -> Result<(), RequestError<AuxFlashError>> {
        Err(AuxFlashError::SlotActive.into())
    }
    fn slot_sector_erase(
        &mut self,
        _: &RecvMessage,
        _slot: u32,
        _offset: u32,
    ) -> Result<(), RequestError<AuxFlashError>> {
        Err(AuxFlashError::SlotActive.into())
    }
    fn write_slot_with_offset(
        &mut self,
        _: &RecvMessage,
        _slot: u32,
        _offset: u32,
        _data: Leased<idol_runtime::R, [u8]>,
    ) -> Result<(), RequestError<AuxFlashError>> {
        Err(AuxFlashError::SlotActive.into())
    }

    fn read_slot_with_offset(
        &mut self,
        _: &RecvMessage,
        slot: u32,
        offset: u32,
        dest: Leased<W, [u8]>,
    ) -> Result<(), RequestError<AuxFlashError>> {
        if slot >= SLOT_COUNT {
            return Err(AuxFlashError::InvalidSlot.into());
        }
        if offset as usize + dest.len() > SLOT_SIZE {
            return Err(AuxFlashError::AddressOverflow.into());
        }
        let mut addr = slot as usize * SLOT_SIZE + offset as usize;
        let end = addr + dest.len();
        let mut write = 0usize;
        let mut buf = [0u8; 256];
        while addr < end {
            let amount = (end - addr).min(buf.len());
            let src = aux_base() + addr;
            // SAFETY: within the granted region (bounds checked above).
            unsafe {
                core::ptr::copy_nonoverlapping(
                    src as *const u8,
                    buf.as_mut_ptr(),
                    amount,
                );
            }
            dest.write_range(write..(write + amount), &buf[..amount])
                .map_err(|_| RequestError::Fail(ClientError::WentAway))?;
            write += amount;
            addr += amount;
        }
        Ok(())
    }

    fn scan_and_get_active_slot(
        &mut self,
        msg: &RecvMessage,
    ) -> Result<u32, RequestError<AuxFlashError>> {
        self.get_active_slot(msg)
    }

    fn get_active_slot(
        &mut self,
        _: &RecvMessage,
    ) -> Result<u32, RequestError<AuxFlashError>> {
        self.active_slot
            .ok_or_else(|| AuxFlashError::NoActiveSlot.into())
    }

    fn ensure_redundancy(
        &mut self,
        _: &RecvMessage,
    ) -> Result<(), RequestError<AuxFlashError>> {
        Ok(()) // single read-only image; no redundancy to maintain
    }

    fn get_blob_by_tag(
        &mut self,
        _: &RecvMessage,
        tag: [u8; 4],
    ) -> Result<AuxFlashBlob, RequestError<AuxFlashError>> {
        let active = self
            .active_slot
            .ok_or(RequestError::from(AuxFlashError::NoActiveSlot))?;
        let handle = SlotReader {
            base: active * SLOT_SIZE as u32,
        };
        handle.get_blob_by_tag(active, tag).map_err(RequestError::from)
    }
}

impl NotificationHandler for ServerImpl {
    fn current_notification_mask(&self) -> u32 {
        0
    }
    fn handle_notification(&mut self, _bits: userlib::NotificationBits) {}
}

/// Find the slot whose stored checksum both matches the build's AUXI checksum
/// and matches its own recomputed hash.
fn scan_for_active_slot() -> Option<u32> {
    for i in 0..SLOT_COUNT {
        let handle = SlotReader {
            base: i * SLOT_SIZE as u32,
        };
        let Ok(chck) = handle.read_stored_checksum() else {
            continue;
        };
        if chck.0 != AUXI_CHECKSUM {
            continue;
        }
        let Ok(actual) = handle.calculate_checksum() else {
            continue;
        };
        if chck == actual {
            return Some(i);
        }
    }
    None
}

fn read_and_check_slot_checksum(
    slot: u32,
) -> Result<AuxFlashChecksum, AuxFlashError> {
    if slot >= SLOT_COUNT {
        return Err(AuxFlashError::InvalidSlot);
    }
    let handle = SlotReader {
        base: slot * SLOT_SIZE as u32,
    };
    let claimed = handle.read_stored_checksum()?;
    let actual = handle.calculate_checksum()?;
    if claimed == actual {
        Ok(actual)
    } else {
        Err(AuxFlashError::ChckMismatch)
    }
}

#[export_name = "main"]
fn main() -> ! {
    let active_slot = scan_for_active_slot();
    let mut server = ServerImpl { active_slot };
    let mut buffer = [0u8; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut buffer, &mut server);
    }
}

mod idl {
    use drv_auxflash_api::{
        AuxFlashBlob, AuxFlashChecksum, AuxFlashError, AuxFlashId,
    };
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
