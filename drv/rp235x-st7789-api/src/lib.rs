// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Client API for the RP2350 (RP235x) ST7789V TFT driver (MusicPi HAT).

#![no_std]

use derive_idol_err::IdolError;
use userlib::{sys_send, FromPrimitive};

#[derive(Copy, Clone, Debug, FromPrimitive, IdolError, counters::Count)]
pub enum St7789Error {
    /// Coordinates/geometry out of range.
    BadArg = 1,
    /// The tft server died / was restarted mid-call. Returned to the client
    /// by the IPC layer; `text` takes a lease, so idol requires this.
    #[idol(server_death)]
    ServerDied = 2,
}

include!(concat!(env!("OUT_DIR"), "/client_stub.rs"));
