// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Client API for the RP2350 minimal network task (W5500 MACRAW + smoltcp).

#![no_std]

use userlib::sys_send;

/// `status` bit: the ethernet link is up (PHY reports link).
pub const STATUS_LINK_UP: u64 = 1 << 63;
/// `status` bit: DHCP has bound and an IPv4 address is configured.
pub const STATUS_BOUND: u64 = 1 << 62;
/// `status` bit: the static-IP fallback is active (DHCP did not lease).
pub const STATUS_STATIC: u64 = 1 << 61;

/// Unpack the IPv4 address octets from a `status` word (a.b.c.d).
pub fn status_ip(status: u64) -> [u8; 4] {
    (status as u32).to_be_bytes()
}

/// Unpack the IPv4 prefix length from a `status` word.
pub fn status_prefix(status: u64) -> u8 {
    (status >> 32) as u8
}

include!(concat!(env!("OUT_DIR"), "/client_stub.rs"));
