// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Client API for the RP2350 (RP235x) I2S audio driver (PCM5102A DAC).

#![no_std]

use derive_idol_err::IdolError;
use userlib::{sys_send, FromPrimitive};

#[derive(Copy, Clone, Debug, FromPrimitive, IdolError, counters::Count)]
pub enum I2sError {
    /// Malformed argument (rate/frequency out of range).
    BadArg = 1,
    /// The PIO state machine did not drain its TX FIFO (not clocking); the
    /// write was dropped rather than blocking the server forever.
    Stalled = 2,
}

include!(concat!(env!("OUT_DIR"), "/client_stub.rs"));
