// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Core-1 liveness task (AMP two-kernel experiment, branch rp2350-amp).
//!
//! A minimal task scheduled by core 1's OWN Hubris kernel. It writes a magic
//! word and an incrementing heartbeat into the shared SRAM region so core 0
//! (or the debug probe over `--core 1`) can confirm that core 1's kernel is
//! alive AND scheduling tasks -- not merely launched. The heartbeat advances
//! via `sleep_for`, which exercises core 1's kernel timer (SysTick) too.

#![no_std]
#![no_main]

extern crate userlib;
use userlib::hl;

extern "C" {
    static mut __REGION_SHARED_BASE: [u8; 0];
}

/// Magic that means "a task on core 1's kernel is running".
const BEAT_MAGIC: u32 = 0xBEA7_1234;

#[export_name = "main"]
fn main() -> ! {
    let base = &raw mut __REGION_SHARED_BASE as *mut u32;
    // SAFETY: `shared` extern region granted to this task covers this address.
    unsafe { base.write_volatile(BEAT_MAGIC) };
    let mut beat: u32 = 0;
    loop {
        beat = beat.wrapping_add(1);
        unsafe { base.add(1).write_volatile(beat) };
        hl::sleep_for(100);
    }
}
