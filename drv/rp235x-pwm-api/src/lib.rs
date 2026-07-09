// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Client API for the RP2350 (RP235x) PWM driver.

#![no_std]

use derive_idol_err::IdolError;
use userlib::{sys_send, FromPrimitive};

#[derive(Copy, Clone, Debug, FromPrimitive, IdolError, counters::Count)]
pub enum PwmError {
    /// Slice > 11 or percent > 100.
    BadArg = 1,
    /// `play_file` could not open the named file on the SD card (no card, no
    /// FAT volume, or the file does not exist).
    OpenFailed = 2,
    /// `play_file` opened the file but it is not a supported WAV (bad header,
    /// truncated, or non-PCM/non-16-bit/too-many-channels).
    BadWav = 3,
    /// The pwm server died / was restarted mid-call. Returned to the client by
    /// the IPC layer; `play_file` takes a lease, so idol requires this variant.
    #[idol(server_death)]
    ServerDied = 4,
}

include!(concat!(env!("OUT_DIR"), "/client_stub.rs"));
