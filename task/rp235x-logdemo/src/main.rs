// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Demo client exercising two Hubris IPC servers:
//!
//! * the **GPIO Idol driver** (`drv-rp235x-gpio`) -- it toggles the onboard LED
//!   (GPIO25) each tick via a typed `gpio.toggle(...)` call, not by poking registers;
//! * the **USB console** (`task-rp235x-usb`) -- it logs an incrementing counter over
//!   USB once a second via a raw IPC message.
//!
//! So the LED blink now proves the GPIO Idol driver works end to end, and the USB
//! log proves the console, both from an ordinary unprivileged client task.

#![no_std]
#![no_main]

use drv_rp235x_gpio_api::Rp235xGpio;
use userlib::{hl, sys_send, task_slot};

task_slot!(USB, usb);
task_slot!(GPIO, gpio_driver);

/// IPC operation understood by the USB console server: "write these bytes".
const OP_WRITE: u16 = 1;
/// Pico 2 onboard LED.
const LED_PIN: u8 = 25;

#[export_name = "main"]
pub fn main() -> ! {
    let usb = USB.get_task_id();
    let gpio = Rp235xGpio::from(GPIO.get_task_id());

    // Drive the LED through the GPIO driver.
    let _ = gpio.configure_output(LED_PIN);

    // Give USB enumeration a moment before the first line (early writes may be
    // dropped if no host is draining yet).
    hl::sleep_for(1500);

    let mut n: u32 = 0;
    loop {
        let _ = gpio.toggle(LED_PIN);

        let mut line = [0u8; 32];
        let msg = format_line(&mut line, n);
        let _ = sys_send(usb, OP_WRITE, msg, &mut [], &[]);
        n = n.wrapping_add(1);
        hl::sleep_for(1000);
    }
}

/// Write `"logdemo tick <n>\r\n"` into `buf` and return the used slice.
fn format_line(buf: &mut [u8; 32], n: u32) -> &[u8] {
    const PREFIX: &[u8] = b"logdemo tick ";
    let mut i = 0;
    for &b in PREFIX {
        buf[i] = b;
        i += 1;
    }
    // Decimal-format n into a temporary, then copy in order.
    let mut tmp = [0u8; 10];
    let mut j = tmp.len();
    let mut m = n;
    if m == 0 {
        j -= 1;
        tmp[j] = b'0';
    } else {
        while m > 0 {
            j -= 1;
            tmp[j] = b'0' + (m % 10) as u8;
            m /= 10;
        }
    }
    while j < tmp.len() {
        buf[i] = tmp[j];
        i += 1;
        j += 1;
    }
    buf[i] = b'\r';
    buf[i + 1] = b'\n';
    &buf[..i + 2]
}
