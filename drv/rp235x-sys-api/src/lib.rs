// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Client API for the RP2350 (RP235x) system-control (RESETS) driver.
//!
//! Block constants are RESETS register bit positions (datasheet / SVD order).

#![no_std]

use userlib::sys_send;

// RESETS bit for each peripheral block (see docs/rp2350-research/addresses.md).
pub const ADC: u32 = 1 << 0;
pub const DMA: u32 = 1 << 2;
pub const I2C0: u32 = 1 << 4;
pub const I2C1: u32 = 1 << 5;
pub const IO_BANK0: u32 = 1 << 6;
pub const PADS_BANK0: u32 = 1 << 9;
pub const PIO0: u32 = 1 << 11;
pub const PIO1: u32 = 1 << 12;
pub const PIO2: u32 = 1 << 13;
pub const PWM: u32 = 1 << 16;
pub const SPI0: u32 = 1 << 18;
pub const SPI1: u32 = 1 << 19;
pub const TIMER0: u32 = 1 << 23;
pub const TIMER1: u32 = 1 << 24;
pub const UART0: u32 = 1 << 26;
pub const UART1: u32 = 1 << 27;
pub const USBCTRL: u32 = 1 << 28;

include!(concat!(env!("OUT_DIR"), "/client_stub.rs"));
