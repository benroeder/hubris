// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Client API for the RP2350 (RP235x) DS1302 real-time-clock driver.
//!
//! Time crosses the IPC boundary packed into a single `u64` (see the byte
//! layout in `idl/rp235x-ds1302.idol`): `now` returns it, `set` takes it.

#![no_std]

use derive_idol_err::IdolError;
use userlib::{FromPrimitive, sys_send};

#[derive(Copy, Clone, Debug, FromPrimitive, IdolError, counters::Count)]
pub enum Ds1302Error {
    /// A field in the packed time was out of range (e.g. hour > 23).
    BadArg = 1,
}

include!(concat!(env!("OUT_DIR"), "/client_stub.rs"));
