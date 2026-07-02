// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Demo client exercising three RP2350 Idol/IPC servers from one unprivileged task:
//!
//! * the **GPIO driver** (`drv-rp235x-gpio`) -- toggles the onboard LED (GPIO25) each
//!   tick via a typed `gpio.toggle(...)` call, not by poking registers;
//! * the **UART driver** (`drv-rp235x-uart`) -- writes a marker out TX and reads back
//!   whatever RX buffered (echoes with a GP0->GP1 loopback jumper);
//! * the **USB console** (`task-rp235x-usb`) -- logs a counter and the UART RX byte
//!   count over USB once a second.

#![no_std]
#![no_main]

use drv_rp235x_gpio_api::Rp235xGpio;
use drv_rp235x_spi_api::Rp235xSpi;
use drv_rp235x_uart_api::Rp235xUart;
use userlib::{hl, sys_send, task_slot};

task_slot!(USB, usb);
task_slot!(GPIO, gpio_driver);
task_slot!(UART, uart_driver);
task_slot!(SPI, spi_driver);

/// IPC operation understood by the USB console server: "write these bytes".
const OP_WRITE: u16 = 1;
/// Pico 2 onboard LED.
const LED_PIN: u8 = 25;

#[export_name = "main"]
pub fn main() -> ! {
    let usb = USB.get_task_id();
    let gpio = Rp235xGpio::from(GPIO.get_task_id());
    let uart = Rp235xUart::from(UART.get_task_id());
    let spi = Rp235xSpi::from(SPI.get_task_id());

    // Drive the LED through the GPIO driver.
    let _ = gpio.configure_output(LED_PIN);

    // Give USB enumeration a moment before the first line (early writes may be
    // dropped if no host is draining yet).
    hl::sleep_for(1500);

    let mut n: u32 = 0;
    loop {
        let _ = gpio.toggle(LED_PIN);

        // UART self-test: send a marker out TX, then read back whatever arrived.
        // With a GP0->GP1 loopback jumper this echoes; without it rx_count is 0.
        uart.write(b"uart-loopback\r\n");
        hl::sleep_for(5);
        let mut rx = [0u8; 32];
        let rx_count = uart.read(&mut rx);

        // SPI self-test: full-duplex exchange through the PL022 internal loopback,
        // so the bytes we send should come back verbatim.
        let tx = [0xA5u8, 0x5A, 0x3C];
        let mut srx = [0u8; 3];
        let _ = spi.exchange(&tx, &mut srx);
        let spi_ok = srx == tx;

        let mut line = [0u8; 64];
        let msg = format_line(&mut line, n, rx_count as u32, spi_ok);
        let _ = sys_send(usb, OP_WRITE, msg, &mut [], &[]);
        n = n.wrapping_add(1);
        hl::sleep_for(1000);
    }
}

/// Write `"tick <n> uart_rx=<rx> spi=OK|BAD\r\n"` into `buf`; return the used slice.
fn format_line(buf: &mut [u8], n: u32, rx: u32, spi_ok: bool) -> &[u8] {
    let mut i = 0;
    i = append(buf, i, b"tick ");
    i = append_u32(buf, i, n);
    i = append(buf, i, b" uart_rx=");
    i = append_u32(buf, i, rx);
    i = append(buf, i, if spi_ok { b" spi=OK" } else { b" spi=BAD" });
    i = append(buf, i, b"\r\n");
    &buf[..i]
}

fn append(buf: &mut [u8], mut i: usize, s: &[u8]) -> usize {
    for &b in s {
        buf[i] = b;
        i += 1;
    }
    i
}

fn append_u32(buf: &mut [u8], i: usize, n: u32) -> usize {
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
    append(buf, i, &tmp[j..])
}
