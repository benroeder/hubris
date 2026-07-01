// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Blinky + UART loopback self-test for the Raspberry Pi Pico 2.
//!
//! Each cycle the task sends a marker byte out UART0 TX (GP0) and tries to read it
//! back on RX (GP1). With a jumper wire GP0 -> GP1 this proves the UART's TX and RX
//! paths, with no host serial adapter, using the onboard LED (GPIO25) as the display:
//!
//! * **double blink** each ~1 s  => loopback OK (byte sent and received back)
//! * **one long blink** each ~1 s => no echo (RX did not see the byte)
//!
//! Only touches SIO (LED) and UART0, so the task runs unprivileged with
//! `uses = ["sio", "uart0"]`.

#![no_std]
#![no_main]

use userlib::hl;

/// Pico 2 onboard LED.
const LED_PIN: u32 = 25;
/// Marker byte for the loopback (alternating bits).
const MARKER: u8 = 0x55;

#[export_name = "main"]
pub fn main() -> ! {
    let p = unsafe { rp235x_pac::Peripherals::steal() };
    let mask = 1u32 << LED_PIN;
    let led_on = || p.SIO.gpio_out_set().write(|w| unsafe { w.bits(mask) });
    let led_off = || p.SIO.gpio_out_clr().write(|w| unsafe { w.bits(mask) });

    loop {
        // Discard anything already sitting in the RX FIFO (e.g. the boot banner).
        while rp235x_uart::rx_ready(&p) {
            let _ = rp235x_uart::read_byte(&p);
        }

        // Send the marker and wait (bounded) for it to loop back on RX.
        rp235x_uart::write_all(&p, &[MARKER]);
        let mut ok = false;
        for _ in 0..1_000_000u32 {
            if rp235x_uart::rx_ready(&p) {
                ok = rp235x_uart::read_byte(&p) == MARKER;
                break;
            }
        }

        if ok {
            // Loopback verified: quick double blink.
            led_on();
            hl::sleep_for(100);
            led_off();
            hl::sleep_for(120);
            led_on();
            hl::sleep_for(100);
            led_off();
        } else {
            // No echo: one long blink.
            led_on();
            hl::sleep_for(500);
            led_off();
        }

        hl::sleep_for(700);
    }
}
