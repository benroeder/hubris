// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Client API for the RP2350 (RP235x) microSD raw block-device driver.
//!
//! RAW BLOCK DEVICE ONLY -- no filesystem. `init` runs the SD SPI-mode
//! power-on handshake; `read_block` returns one 512-byte block.

#![no_std]

use derive_idol_err::IdolError;
use userlib::{FromPrimitive, sys_send};

/// Status bit returned by `init`: card uses block (SDHC/SDXC) addressing.
pub const STATUS_CCS: u32 = 1 << 0;
/// Status bit returned by `init`: card responded as SD version 2 (CMD8 ok).
pub const STATUS_V2: u32 = 1 << 1;

#[derive(Copy, Clone, Debug, FromPrimitive, IdolError, counters::Count)]
pub enum SdError {
    /// A step of the power-on init handshake failed.
    Init = 1,
    /// A card operation did not complete within its bounded retry budget.
    Timeout = 2,
    /// The card returned an unexpected command response (R1 error bits set).
    Cmd = 3,
    /// The card returned a data error token instead of the 0xFE start token.
    DataError = 4,
    /// A block operation was attempted before a successful `init`.
    NotInitialized = 5,
}

include!(concat!(env!("OUT_DIR"), "/client_stub.rs"));
