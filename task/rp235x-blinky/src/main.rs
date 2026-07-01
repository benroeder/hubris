// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Blinky + UART-RX interrupt self-test for the Raspberry Pi Pico 2.
//!
//! Each cycle the task sends a marker byte out UART0 TX (GP0) and then **blocks
//! waiting for the UART RX interrupt** (UART0_IRQ, routed to this task as a
//! notification). With a GP0 -> GP1 jumper the byte loops back, the RX FIFO fills,
//! the interrupt fires, the kernel posts the notification, and the task wakes and
//! toggles the onboard LED (GPIO25).
//!
//! Because the toggle only happens on the interrupt notification, the LED proves the
//! whole peripheral-IRQ -> task-notification path on RP2350 (never exercised before --
//! everything prior was polled / SysTick). This is the infrastructure USB depends on.
//!
//! LED reading (with the LED left dark by the app after clock bring-up):
//! * **blinking** => clocks OK and the interrupt path delivers
//! * **dark, never blinks** => clocks OK but the RX interrupt is not reaching us
//! * **solid on** => hung earlier, in clock bring-up (see app `main`)
//!
//! Runs unprivileged with `uses = ["sio", "uart0"]` and the `uart-irq` notification.

#![no_std]
#![no_main]

use userlib::{hl, sys_irq_control, sys_recv_open};

/// Pico 2 onboard LED.
const LED_PIN: u32 = 25;
/// Marker byte for the loopback (alternating bits).
const MARKER: u8 = 0x55;

#[export_name = "main"]
pub fn main() -> ! {
    let p = unsafe { rp235x_pac::Peripherals::steal() };
    let led = 1u32 << LED_PIN;
    let irq = notifications::UART_IRQ_MASK;

    // Enable NVIC delivery of UART0_IRQ to this task.
    sys_irq_control(irq, true);

    loop {
        // Discard anything already in the RX FIFO, then send the marker.
        while rp235x_uart::rx_ready(&p) {
            let _ = rp235x_uart::read_byte(&p);
        }
        rp235x_uart::write_all(&p, &[MARKER]);

        // Block until the UART RX interrupt notification arrives.
        let _ = sys_recv_open(&mut [], irq);

        // Interrupt handled: toggle the LED, drain + acknowledge, re-enable the IRQ.
        p.SIO.gpio_out_xor().write(|w| unsafe { w.bits(led) });
        while rp235x_uart::rx_ready(&p) {
            let _ = rp235x_uart::read_byte(&p);
        }
        rp235x_uart::clear_rx_interrupt(&p);
        sys_irq_control(irq, true);

        hl::sleep_for(500);
    }
}

// Pulls in the `notifications` module generated from this task's app.toml
// `notifications`/`interrupts` (provides `UART_IRQ_MASK`).
include!(concat!(env!("OUT_DIR"), "/notifications.rs"));
