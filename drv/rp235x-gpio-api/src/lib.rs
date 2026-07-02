// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Client API for the RP2350 (RP235x) GPIO driver.

#![no_std]

use derive_idol_err::IdolError;
use userlib::{sys_send, FromPrimitive};

#[derive(Copy, Clone, Debug, FromPrimitive, IdolError, counters::Count)]
pub enum GpioError {
    /// Pin number is out of the supported range (Bank 0, pins 0-31).
    InvalidPin = 1,
}

include!(concat!(env!("OUT_DIR"), "/client_stub.rs"));
