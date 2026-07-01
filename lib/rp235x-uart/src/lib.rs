// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Minimal polling PL011 UART0 driver for the RP2350 / Pico 2.
//!
//! [`configure`] resets UART0, function-selects GP0 (TX) / GP1 (RX), sets 115200
//! 8N1 from a 12 MHz peripheral clock, and enables the UART. It touches RESETS /
//! PADS_BANK0 / IO_BANK0 / UART0, so it must run in the app's privileged pre-kernel
//! startup.
//!
//! [`write_all`] / [`write_u32`] busy-poll the TX FIFO and only touch UART0, so they
//! can be called from an unprivileged task that has `uses = ["uart0"]`.
//!
//! Pico 2's USB is the RP2350's native USB, not a UART bridge, so TX appears on the
//! physical GP0 pin -- view it with a USB-serial adapter (adapter RX <- GP0, GND <-
//! GND), or loop GP0 -> GP1 for an on-board RX test.

#![no_std]

use rp235x_pac::Peripherals;

/// UART TX pin (Pico 2 GP0) and RX pin (GP1). UART function is FUNCSEL 2.
const TX_PIN: usize = 0;
const RX_PIN: usize = 1;
const UART_FUNCSEL: u8 = 2;

const BAUD: u32 = 115_200;

/// Reset, pin-mux, and enable UART0 at 115200 8N1, given the `clk_peri` frequency.
///
/// Baud divisors are computed from `clk_peri_hz`, so this stays correct as the clock
/// changes (12 MHz -> 150 MHz). Privileged; call once at startup.
pub fn configure(p: &Peripherals, clk_peri_hz: u32) {
    // PL011 integer baud calc (as in pico-sdk uart_set_baudrate):
    // div = 8 * f / baud; IBRD = div >> 7; FBRD = ((div & 0x7f) + 1) / 2.
    let div = 8 * clk_peri_hz / BAUD;
    let (baud_int, baud_frac): (u16, u8) = if div >> 7 == 0 {
        (1, 0)
    } else if div >> 7 >= 65535 {
        (65535, 0)
    } else {
        ((div >> 7) as u16, (div & 0x7f).div_ceil(2) as u8)
    };

    p.RESETS.reset().modify(|_, w| w.uart0().clear_bit());
    while p.RESETS.reset_done().read().uart0().bit_is_clear() {}

    // TX pin: output; RX pin: input-enabled. Clear the RP2350 pad isolation latch.
    p.PADS_BANK0
        .gpio(TX_PIN)
        .modify(|_, w| w.od().clear_bit().iso().clear_bit());
    p.PADS_BANK0
        .gpio(RX_PIN)
        .modify(|_, w| w.od().clear_bit().iso().clear_bit().ie().set_bit());
    p.IO_BANK0
        .gpio(TX_PIN)
        .gpio_ctrl()
        .modify(|_, w| unsafe { w.funcsel().bits(UART_FUNCSEL) });
    p.IO_BANK0
        .gpio(RX_PIN)
        .gpio_ctrl()
        .modify(|_, w| unsafe { w.funcsel().bits(UART_FUNCSEL) });

    // UART0 comes out of reset disabled. Program baud, then 8N1 + FIFO, then enable.
    p.UART0
        .uartibrd()
        .write(|w| unsafe { w.baud_divint().bits(baud_int) });
    p.UART0
        .uartfbrd()
        .write(|w| unsafe { w.baud_divfrac().bits(baud_frac) });
    p.UART0.uartlcr_h().write(|w| {
        unsafe { w.wlen().bits(0b11) }; // 8 data bits
        w.fen().set_bit() // enable FIFOs
    });
    p.UART0
        .uartcr()
        .write(|w| w.uarten().set_bit().txe().set_bit().rxe().set_bit());
}

/// Busy-poll the TX FIFO and send every byte. Only touches UART0.
pub fn write_all(p: &Peripherals, bytes: &[u8]) {
    for &b in bytes {
        while p.UART0.uartfr().read().txff().bit_is_set() {}
        p.UART0.uartdr().write(|w| unsafe { w.data().bits(b) });
    }
}

/// Unmask the RX interrupts (RX FIFO level + receive timeout) so a received byte
/// asserts UART0_IRQ. Privileged; the kernel/NVIC side is enabled per task via
/// `sys_irq_control`.
pub fn enable_rx_interrupt(p: &Peripherals) {
    p.UART0
        .uartimsc()
        .modify(|_, w| w.rxim().set_bit().rtim().set_bit());
}

/// Acknowledge the RX interrupts at the peripheral (call after handling).
/// UARTICR is write-1-to-clear: RXIC = bit 4, RTIC = bit 6.
pub fn clear_rx_interrupt(p: &Peripherals) {
    p.UART0
        .uarticr()
        .write(|w| unsafe { w.bits((1 << 4) | (1 << 6)) });
}

/// True if the RX FIFO has at least one byte waiting.
pub fn rx_ready(p: &Peripherals) -> bool {
    !p.UART0.uartfr().read().rxfe().bit_is_set()
}

/// Read one byte from the RX FIFO (only valid when [`rx_ready`] is true).
pub fn read_byte(p: &Peripherals) -> u8 {
    p.UART0.uartdr().read().data().bits()
}

/// Send `n` as decimal ASCII.
pub fn write_u32(p: &Peripherals, mut n: u32) {
    let mut buf = [0u8; 10];
    let mut i = buf.len();
    if n == 0 {
        write_all(p, b"0");
        return;
    }
    while n > 0 {
        i -= 1;
        buf[i] = b'0' + (n % 10) as u8;
        n /= 10;
    }
    write_all(p, &buf[i..]);
}
