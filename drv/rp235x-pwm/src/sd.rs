// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! SD-backed `ByteSource` for the WAV player.
//!
//! The pwm task is a FAT client only for the duration of a `play_file` call, so
//! this module carries its own copy of the shell's `SdBlockDevice`/`TimeSource`
//! adapters (both tiny; they depend only on the `Rp235xSdcard` IPC client) plus
//! an `SdFileSource` that streams one file's bytes to the decoder.
//!
//! LIFETIME NOTE: embedded-sdmmc's typed `Volume`/`Directory`/`File` handles
//! borrow the `VolumeManager` in a chain that cannot be stored alongside the
//! manager. `SdFileSource` therefore uses the RAW-HANDLE API: it owns the
//! `VolumeManager` outright and holds only integer `RawVolume`/`RawDirectory`/
//! `RawFile` handles, reading through `VolumeManager::read` (which takes `&self`
//! via interior mutability). The handles are released on `Drop`.

use rp235x_audio_decode::ByteSource;
use drv_rp235x_sdcard_api::Rp235xSdcard;
use embedded_sdmmc::{
    Block, BlockCount, BlockDevice, BlockIdx, Mode, RawDirectory, RawFile,
    RawVolume, TimeSource, Timestamp, VolumeIdx, VolumeManager,
};

/// Errors surfaced by the block-device adapter. embedded-sdmmc only requires
/// `Debug`.
#[derive(Debug)]
pub enum Error {
    /// An underlying sdcard IPC `read_block` failed.
    Io,
}

/// Presents the sdcard Idol client as an embedded-sdmmc `BlockDevice`. Holds a
/// clone of the `Rp235xSdcard` client (just a task id), so the `VolumeManager`
/// can own it outright for the play duration.
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
        // The player only reads specific LBAs, so a large placeholder is enough
        // for embedded-sdmmc's bounds checks.
        Ok(BlockCount(0x4000_0000))
    }
}

/// Fixed timestamp source. The player never writes, so the value is irrelevant;
/// a constant keeps the pwm task free of an RTC dependency. 2025-01-01 00:00:00.
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

/// embedded-sdmmc defaults (MAX_DIRS = 4, MAX_FILES = 4, MAX_VOLUMES = 1) --
/// the same const generics `VolumeManager::new` produces.
type Vm = VolumeManager<SdBlockDevice, DummyTime, 4, 4, 1>;

/// Streams one FAT file's bytes to the decoder. Owns the `VolumeManager` and
/// the raw handles opened against it; `read` pulls sequential bytes via
/// `VolumeManager::read`, which advances the file's own offset.
pub struct SdFileSource {
    vm: Vm,
    volume: RawVolume,
    dir: RawDirectory,
    file: RawFile,
    /// Sticky flag: set if a `VolumeManager::read` ever returned an error. Lets
    /// the caller tell a real IO fault (truncated playback) from a clean EOF,
    /// since `read` reports both as 0 bytes.
    io_error: bool,
}

impl SdFileSource {
    /// Mount FAT volume 0, open the root directory, and open `name` read-only.
    /// On any failure the partially-opened handles are released and `None` is
    /// returned. `name` is a raw byte slice (an 8.3 short name) from the lease.
    pub fn open(sdcard: Rp235xSdcard, name: &[u8]) -> Option<Self> {
        // The FAT short name must be valid UTF-8 (ASCII in practice) for
        // embedded-sdmmc's ToShortFileName.
        let name = core::str::from_utf8(name).ok()?;

        let vm = VolumeManager::new(SdBlockDevice::new(sdcard), DummyTime);
        let volume = vm.open_raw_volume(VolumeIdx(0)).ok()?;
        let dir = match vm.open_root_dir(volume) {
            Ok(d) => d,
            Err(_) => {
                let _ = vm.close_volume(volume);
                return None;
            }
        };
        let file = match vm.open_file_in_dir(dir, name, Mode::ReadOnly) {
            Ok(f) => f,
            Err(_) => {
                let _ = vm.close_dir(dir);
                let _ = vm.close_volume(volume);
                return None;
            }
        };
        Some(Self {
            vm,
            volume,
            dir,
            file,
            io_error: false,
        })
    }
}

impl ByteSource for SdFileSource {
    fn read(&mut self, out: &mut [u8]) -> usize {
        // VolumeManager::read advances the file offset and returns a short read
        // at EOF (0 = end). An IO error also yields 0 (so the player drains and
        // stops rather than looping), but we latch it so the caller can report
        // the playback was truncated rather than silently claiming success.
        match self.vm.read(self.file, out) {
            Ok(n) => n,
            Err(_) => {
                self.io_error = true;
                0
            }
        }
    }

    fn had_error(&self) -> bool {
        self.io_error
    }
}

impl Drop for SdFileSource {
    fn drop(&mut self) {
        // Release the handles in reverse open order. Best-effort: a close error
        // cannot be surfaced from Drop, and the whole VolumeManager is dropped
        // immediately after regardless.
        let _ = self.vm.close_file(self.file);
        let _ = self.vm.close_dir(self.dir);
        let _ = self.vm.close_volume(self.volume);
    }
}
