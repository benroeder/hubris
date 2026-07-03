// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Core-1 kernel entry (AMP two-kernel experiment, branch rp2350-amp).
//!
//! A second, minimal Hubris kernel that runs on core 1. Unlike core 0's app
//! main it does NO hardware init -- clocks, resets, and pads are global and
//! core 0 already set them up before launching us. We just start the kernel.

#![no_std]
#![no_main]

use rp235x_pac as _;
use cortex_m_rt::entry;

/// System clock is 150 MHz (core 0 configured it); kernel SysTick wants cycles
/// per millisecond.
const CYCLES_PER_MS: u32 = 150_000;

#[entry]
fn main() -> ! {
    unsafe { kern::startup::start_kernel(CYCLES_PER_MS) }
}
