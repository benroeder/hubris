// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Client API for the RP2350 (RP235x) UART0 driver.

#![no_std]

use userlib::sys_send;

include!(concat!(env!("OUT_DIR"), "/client_stub.rs"));
