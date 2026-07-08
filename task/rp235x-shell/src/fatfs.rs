// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! FAT filesystem layer over the raw microSD block device.
//!
//! Bridges embedded-sdmmc's `BlockDevice`/`TimeSource` traits onto the
//! `Rp235xSdcard` Idol client, which serves exactly one 512-byte block per
//! IPC (the same lease idiom as the flash driver's `read`/`program`). Both
//! halves of `BlockDevice` are wired: `read` fans out to `read_block` and
//! `write` to `write_block`, so embedded-sdmmc can mutate the FAT.

use drv_rp235x_sdcard_api::Rp235xSdcard;
use embedded_sdmmc::{
    Block, BlockCount, BlockDevice, BlockIdx, Directory, File, TimeSource,
    Timestamp,
};

/// Errors surfaced by the block-device adapter. Kept tiny; embedded-sdmmc only
/// requires `Debug`.
#[derive(Debug)]
pub enum Error {
    /// An underlying sdcard IPC `read_block`/`write_block` failed.
    Io,
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
        blocks: &[Block],
        start_block_idx: BlockIdx,
    ) -> Result<(), Self::Error> {
        for (i, block) in blocks.iter().enumerate() {
            let lba = start_block_idx.0 + i as u32;
            self.sdcard
                .write_block(lba, &block.contents)
                .map_err(|_| Error::Io)?;
        }
        Ok(())
    }

    fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
        // We never expose capacity to the user; reads target specific LBAs, so
        // a large placeholder is enough for embedded-sdmmc's bounds checks.
        Ok(BlockCount(0x4000_0000))
    }
}

/// Fixed timestamp source. Used only in builds without the `ds1302` feature
/// (no RTC available). Corresponds to 2025-01-01 00:00:00 (year_since_1970 =
/// 55).
#[cfg(not(feature = "ds1302"))]
pub struct DummyTime;

#[cfg(not(feature = "ds1302"))]
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

/// DS1302-backed timestamp source. Reads the live clock over IPC so files
/// written to the card carry real calendar dates. Only present in `ds1302`
/// builds.
#[cfg(feature = "ds1302")]
pub struct RtcTime {
    rtc: drv_rp235x_ds1302_api::Rp235xDs1302,
}

#[cfg(feature = "ds1302")]
impl RtcTime {
    pub fn new(rtc: drv_rp235x_ds1302_api::Rp235xDs1302) -> Self {
        Self { rtc }
    }
}

#[cfg(feature = "ds1302")]
impl TimeSource for RtcTime {
    fn get_timestamp(&self) -> Timestamp {
        // Packed u64 from the DS1302 driver: byte0=sec, byte1=min, byte2=hour,
        // byte3=date, byte4=month, byte5=weekday, byte6=year (0-99).
        let packed = self.rtc.now();
        let sec = packed as u8;
        let min = (packed >> 8) as u8;
        let hour = (packed >> 16) as u8;
        let date = (packed >> 24) as u8;
        let month = (packed >> 32) as u8;
        let year = (packed >> 48) as u8;
        // Clamp every field to its valid range: an unset/halted DS1302 returns
        // garbage BCD, and the masked decode can yield out-of-range values
        // (e.g. month up to 25). Clamping keeps a corrupt clock from stamping an
        // invalid FAT date rather than propagating it into on-card metadata.
        let year = year.min(99);
        let month = month.clamp(1, 12);
        let date = date.clamp(1, 31);
        let hour = hour.min(23);
        let min = min.min(59);
        let sec = sec.min(59);
        Timestamp {
            // year_since_1970 = 2000 + year - 1970 = year + 30.
            year_since_1970: year + 30,
            zero_indexed_month: month - 1,
            zero_indexed_day: date - 1,
            hours: hour,
            minutes: min,
            seconds: sec,
        }
    }
}

/// The FAT `TimeSource` this build uses: the live DS1302 clock when the
/// `ds1302` feature is on, otherwise the fixed `DummyTime`. A single alias so
/// every `sd` command builds its `VolumeManager` with the same source.
#[cfg(feature = "ds1302")]
pub type FatTime = RtcTime;
#[cfg(not(feature = "ds1302"))]
pub type FatTime = DummyTime;

/// A `VolumeManager` directory over the sdcard block device, with the FAT
/// `TimeSource` this build uses. The const generics are embedded-sdmmc's
/// defaults (MAX_DIRS = 4, MAX_FILES = 4, MAX_VOLUMES = 1), so a plain
/// `VolumeManager::new(...)` produces exactly this type. Spelled once here so
/// the shell's `with_root` helper can name the directory it hands back.
pub type FatDir<'a> = Directory<'a, SdBlockDevice, FatTime, 4, 4, 1>;

/// A file handle within a `FatDir`, matching const generics as above.
pub type FatFile<'a> = File<'a, SdBlockDevice, FatTime, 4, 4, 1>;
