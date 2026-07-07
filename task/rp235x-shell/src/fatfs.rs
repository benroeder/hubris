// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Read-only FAT filesystem layer over the raw microSD block device.
//!
//! Bridges embedded-sdmmc's `BlockDevice`/`TimeSource` traits onto the
//! `Rp235xSdcard` Idol client, which serves exactly one 512-byte block per
//! IPC (the same lease idiom as the flash driver's `read`). READ-ONLY: the
//! `write` half of `BlockDevice` returns `Unsupported`, so no FAT mutation can
//! ever reach the card.

use drv_rp235x_sdcard_api::Rp235xSdcard;
use embedded_sdmmc::{
    Block, BlockCount, BlockDevice, BlockIdx, TimeSource, Timestamp,
};

/// Errors surfaced by the block-device adapter. Kept tiny; embedded-sdmmc only
/// requires `Debug`.
#[derive(Debug)]
pub enum Error {
    /// An underlying sdcard IPC `read_block` failed.
    Io,
    /// Writes are not implemented -- this is a read-only mount.
    Unsupported,
}

/// Presents the sdcard Idol client as an embedded-sdmmc `BlockDevice`.
///
/// Holds a clone of the shell's `Rp235xSdcard` client (just a task id), so the
/// `VolumeManager` can own it outright for the lifetime of a command.
pub struct SdBlockDevice {
    sdcard: Rp235xSdcard,
}

impl SdBlockDevice {
    pub fn new(sdcard: Rp235xSdcard) -> Self {
        Self { sdcard }
    }
}

impl BlockDevice for SdBlockDevice {
    type Error = Error;

    fn read(
        &self,
        blocks: &mut [Block],
        start_block_idx: BlockIdx,
    ) -> Result<(), Self::Error> {
        for (i, block) in blocks.iter_mut().enumerate() {
            let lba = start_block_idx.0 + i as u32;
            self.sdcard
                .read_block(lba, &mut block.contents)
                .map_err(|_| Error::Io)?;
        }
        Ok(())
    }

    fn write(
        &self,
        _blocks: &[Block],
        _start_block_idx: BlockIdx,
    ) -> Result<(), Self::Error> {
        // Read-only: refuse every write so embedded-sdmmc cannot dirty the card.
        Err(Error::Unsupported)
    }

    fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
        // We never expose capacity to the user; reads target specific LBAs, so
        // a large placeholder is enough for embedded-sdmmc's bounds checks.
        Ok(BlockCount(0x4000_0000))
    }
}

/// Fixed timestamp source. The SD image carries no RTC and this layer is
/// read-only, so nothing consumes the value; DS1302-backed time is a later
/// step. Corresponds to 2025-01-01 00:00:00 (year_since_1970 = 55).
pub struct DummyTime;

impl TimeSource for DummyTime {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 55,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}
