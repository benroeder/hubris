// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Minimal blinky task for the Raspberry Pi Pico 2.
//!
//! Toggles the onboard LED (GPIO25) via the SIO atomic-XOR register on a timer.
//! GPIO25 is configured as an SIO output by the app's privileged pre-kernel
//! startup (see `app/demo-pi-pico-2/src/main.rs`); this task only needs SIO
//! access (`uses = ["sio"]`) to toggle it.
//!
//! A blinking LED proves the full stack: the boot ROM accepted our IMAGE_DEF,
//! the kernel started, it scheduled this (unprivileged) task, and the task's
//! `sleep_for` timer notifications are being delivered.

#![no_std]
#![no_main]

use userlib::hl;

/// Pico 2 onboard LED.
const LED_PIN: u32 = 25;

#[export_name = "main"]
pub fn main() -> ! {
    let p = unsafe { rp235x_pac::Peripherals::steal() };
    let mask = 1u32 << LED_PIN;

    loop {
        // Atomic toggle of GPIO25's output level. With the crystal-accurate clock
        // (12 MHz, cycles_per_ms = 12000), a 500 ms half-period gives a precise
        // 1 Hz blink -- verifiable against a stopwatch.
        p.SIO.gpio_out_xor().write(|w| unsafe { w.bits(mask) });
        hl::sleep_for(500);
    }
}
