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

/// `set_pull` argument: no pulls.
pub const PULL_NONE: u8 = 0;
/// `set_pull` argument: pull-up (preferred for inputs; see RP2350-E9).
pub const PULL_UP: u8 = 1;
/// `set_pull` argument: pull-down. NOTE erratum RP2350-E9: a Bank 0 pad
/// whose input has latched high (input driven above ~2.2 V then released)
/// is NOT recovered by the weak internal pull-down; use an external
/// pull-down (< 8.2 kOhm) or prefer pull-ups where possible.
pub const PULL_DOWN: u8 = 2;

include!(concat!(env!("OUT_DIR"), "/client_stub.rs"));
