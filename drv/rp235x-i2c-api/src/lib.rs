// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Client API for the RP2350 (RP235x) I2C0 driver.

#![no_std]

use derive_idol_err::IdolError;
use userlib::{sys_send, FromPrimitive};

#[derive(Copy, Clone, Debug, FromPrimitive, IdolError, counters::Count)]
pub enum I2cError {
    /// The addressed device did not ACK (or aborted the transfer).
    Nak = 1,
    /// The controller did not complete the transfer in time.
    Timeout = 2,
}

include!(concat!(env!("OUT_DIR"), "/client_stub.rs"));
