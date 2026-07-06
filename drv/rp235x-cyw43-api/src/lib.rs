// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Client API for the RP2350 (RP235x) CYW43439 Wi-Fi driver.

#![no_std]

use derive_idol_err::IdolError;
use userlib::{sys_send, FromPrimitive};

#[derive(Copy, Clone, Debug, FromPrimitive, IdolError, counters::Count)]
pub enum Cyw43Error {
    /// The chip is not initialized (firmware not yet up).
    NotReady = 1,
    /// A control-plane (SDPCM/CDC ioctl) transaction failed.
    IoctlFailed = 2,
    /// The scan could not be started.
    ScanFailed = 3,
    /// The requested scan-result index is out of range.
    InvalidIndex = 4,
    /// The driver server died and restarted mid-call.
    #[idol(server_death)]
    ServerRestarted = 5,
}

include!(concat!(env!("OUT_DIR"), "/client_stub.rs"));
