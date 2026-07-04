// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Sony S-Link / Control-A1 driver for the RP2350 (RP235x), branch `slink`.
//!
//! S-Link is a single bidirectional open-collector control wire: idle HIGH
//! (pull-up), and a transmitter pulls it LOW for a mark whose width encodes the
//! symbol, each mark followed by a ~600 us HIGH delimiter:
//!   SYNC = 2400 us, logical one = 1200 us, logical zero = 600 us.
//! Bytes are sent MSB-first; a frame is 2-3 bytes (device id + command(s)).
//!
//! We bit-bang it on GP4 (the existing board-to-board straight wire, I2C-SDA
//! position, + ground). "Drive LOW" = enable the SIO output (OUT is held 0);
//! "release" = disable the output so the pad's pull-up returns the line HIGH --
//! open-drain by output-enable toggling, exactly as an S-Link node behaves.
//!
//! Timing is done with `cortex_m::asm::delay` (cycle-accurate busy-wait) since
//! the marks are hundreds of microseconds and Hubris's timer is milliseconds.
//! The receiver measures each LOW mark by counting fixed delay steps and
//! classifies it with the protocol's generous +-20% tolerance.

#![no_std]
#![no_main]

use core::convert::Infallible;
use idol_runtime::RequestError;
use userlib::RecvMessage;

/// S-Link line pin: GP4 (reuses the I2C-SDA straight wire between the boards).
const PIN: u32 = 4;
/// System clock: 150 MHz -> 150 cycles per microsecond for `asm::delay`.
const CYCLES_PER_US: u32 = 150;

// Symbol mark widths (microseconds).
const SYNC_US: u32 = 2400;
const ONE_US: u32 = 1200;
const ZERO_US: u32 = 600;
const DELIM_US: u32 = 600;
/// Wait for a quiet line before transmitting.
const LINE_READY_US: u32 = 3000;
/// A HIGH gap longer than this ends the frame.
const FRAME_GAP_US: u32 = 3000;

// Classification thresholds (midpoints between the nominal widths).
const T_SYNC: u32 = 1800; // >= this -> SYNC
const T_ONE: u32 = 900; //  >= this (and < T_SYNC) -> one
const T_ZERO: u32 = 300; //  >= this (and < T_ONE)  -> zero

/// Busy-wait `us` microseconds.
fn dly_us(us: u32) {
    cortex_m::asm::delay(us.saturating_mul(CYCLES_PER_US));
}

struct ServerImpl {
    sio: rp235x_pac::SIO,
}

impl ServerImpl {
    /// Drive the line LOW (enable the output; OUT is held at 0).
    fn drive_low(&self) {
        self.sio
            .gpio_oe_set()
            .write(|w| unsafe { w.bits(1 << PIN) });
    }
    /// Release the line (disable the output; the pull-up returns it HIGH).
    fn release(&self) {
        self.sio
            .gpio_oe_clr()
            .write(|w| unsafe { w.bits(1 << PIN) });
    }
    /// Read the line: true = HIGH.
    fn is_high(&self) -> bool {
        (self.sio.gpio_in().read().bits() >> PIN) & 1 == 1
    }

    /// One mark: LOW for `low_us`, then the HIGH delimiter.
    fn mark(&self, low_us: u32) {
        self.drive_low();
        dly_us(low_us);
        self.release();
        dly_us(DELIM_US);
    }

    fn send_byte(&self, b: u8) {
        for i in (0..8).rev() {
            self.mark(if b & (1 << i) != 0 { ONE_US } else { ZERO_US });
        }
    }

    /// Measure the current LOW mark: returns its width in microseconds (assumes
    /// the line is already LOW; returns when it goes HIGH or a cap is hit).
    fn measure_low(&self) -> u32 {
        let mut us = 0u32;
        while !self.is_high() {
            dly_us(5);
            us += 5;
            if us > 4000 {
                break;
            }
        }
        us
    }
}

impl idl::InOrderRp235xSlinkImpl for ServerImpl {
    fn send(
        &mut self,
        _: &RecvMessage,
        b0: u8,
        b1: u8,
        b2: u8,
        n: u8,
    ) -> Result<(), RequestError<Infallible>> {
        let bytes = [b0, b1, b2];
        let n = (n as usize).min(3);
        // Idle high, wait for a quiet line, then SYNC + the bytes.
        self.release();
        dly_us(LINE_READY_US);
        self.mark(SYNC_US);
        for &b in &bytes[..n] {
            self.send_byte(b);
        }
        self.release();
        Ok(())
    }

    fn listen(
        &mut self,
        _: &RecvMessage,
        timeout_ms: u32,
    ) -> Result<u32, RequestError<Infallible>> {
        // Wait (bounded) for the line to fall -- the start of a mark.
        let mut waited_us = 0u32;
        while self.is_high() {
            dly_us(100);
            waited_us += 100;
            if waited_us >= timeout_ms.saturating_mul(1000) {
                return Ok(0);
            }
        }
        // The first mark must be SYNC, else it's noise.
        if self.measure_low() < T_SYNC {
            return Ok(0);
        }
        // Collect bits until the line stays idle (end of frame).
        let mut bytes = [0u8; 3];
        let mut nbits = 0u32;
        let mut nbytes = 0usize;
        loop {
            // HIGH delimiter/idle; a long gap ends the frame.
            let mut gap = 0u32;
            while self.is_high() {
                dly_us(20);
                gap += 20;
                if gap > FRAME_GAP_US {
                    return Ok(pack(&bytes, nbytes));
                }
            }
            // Next mark.
            let w = self.measure_low();
            if w >= T_SYNC {
                // Unexpected re-sync: restart the frame.
                bytes = [0; 3];
                nbits = 0;
                nbytes = 0;
                continue;
            }
            let bit = if w >= T_ONE {
                1
            } else if w >= T_ZERO {
                0
            } else {
                continue; // glitch, ignore
            };
            if nbytes < 3 {
                bytes[nbytes] = (bytes[nbytes] << 1) | bit;
            }
            nbits += 1;
            if nbits == 8 {
                nbits = 0;
                nbytes += 1;
                if nbytes == 3 {
                    return Ok(pack(&bytes, 3));
                }
            }
        }
    }
}

/// Pack a decoded frame into the reply word: (count<<24)|(b0<<16)|(b1<<8)|b2.
fn pack(bytes: &[u8; 3], n: usize) -> u32 {
    ((n as u32) << 24)
        | ((bytes[0] as u32) << 16)
        | ((bytes[1] as u32) << 8)
        | bytes[2] as u32
}

impl idol_runtime::NotificationHandler for ServerImpl {
    fn current_notification_mask(&self) -> u32 {
        0
    }
    fn handle_notification(&mut self, _bits: userlib::NotificationBits) {}
}

#[export_name = "main"]
fn main() -> ! {
    let p = unsafe { rp235x_pac::Peripherals::steal() };

    // Configure GP4 as an SIO open-drain S-Link line: function-select SIO (5),
    // clear pad isolation/output-disable, enable input, enable the pull-up (so
    // the released line idles HIGH), hold OUT at 0 (drive LOW = enable output),
    // and start released (output disabled = input).
    p.PADS_BANK0.gpio(PIN as usize).modify(|_, w| {
        w.od().clear_bit();
        w.iso().clear_bit();
        w.ie().set_bit();
        w.pue().set_bit();
        w.pde().clear_bit()
    });
    p.IO_BANK0
        .gpio(PIN as usize)
        .gpio_ctrl()
        .modify(|_, w| unsafe { w.funcsel().bits(5) });
    p.SIO.gpio_out_clr().write(|w| unsafe { w.bits(1 << PIN) });
    p.SIO.gpio_oe_clr().write(|w| unsafe { w.bits(1 << PIN) });

    let mut server = ServerImpl { sio: p.SIO };
    let mut incoming = [0u8; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
