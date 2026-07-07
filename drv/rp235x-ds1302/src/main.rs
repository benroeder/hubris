// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! DS1302 real-time-clock driver server for the RP2350 (RP235x).
//!
//! The DS1302 is a 3-wire serial RTC: CE (chip-enable, active HIGH), IO
//! (bidirectional data), and SCLK. We bit-bang all three on SIO GPIOs
//! (GP6 = CE, GP7 = IO, GP8 = SCLK, all function-select SIO = 5), driving the
//! outputs via the SIO `gpio_out_set/clr` + `gpio_oe_set/clr` registers and
//! reading IO back via `gpio_in`, exactly the idiom the S-Link driver uses.
//!
//! Protocol (datasheet): each transfer is CE HIGH, then a command byte, then a
//! data byte -- both LSB-first. The DS1302 samples IO on the SCLK RISING edge
//! (writes) and presents the next bit on the FALLING edge (reads), so a read
//! samples IO BEFORE pulsing the clock. Register values are BCD.
//!
//! Timing: the marks are ~1 us (`cortex_m::asm::delay`, ~150 cycles at
//! 150 MHz) between every edge. The DS1302 clocks up to 2 MHz, so this is very
//! conservative.
//!
//! NOTE: the exact edge timing and the read-bit polarity are UNPROVEN on
//! hardware at the time of writing.

#![no_std]
#![no_main]

use core::convert::Infallible;
use drv_rp235x_ds1302_api::Ds1302Error;
use idol_runtime::RequestError;
use userlib::RecvMessage;

/// CE / RST: chip-enable, active HIGH.
const CE: u32 = 6;
/// IO: bidirectional serial data.
const IO: u32 = 7;
/// SCLK: serial clock.
const SCLK: u32 = 8;
/// funcsel that routes a GPIO to SIO on the RP2350.
const FUNCSEL_SIO: u8 = 5;
/// clk_sys cycles for the inter-edge settle (~1 us at 150 MHz).
const EDGE_DELAY: u32 = 150;

// DS1302 register command bytes (write address; read = write | 1). bit7 = 1,
// bit6 = 0 (clock), bits5..1 = address, bit0 = read/write.
const SECONDS_W: u8 = 0x80;
const MINUTES_W: u8 = 0x82;
const HOURS_W: u8 = 0x84;
const DATE_W: u8 = 0x86;
const MONTH_W: u8 = 0x88;
const WEEKDAY_W: u8 = 0x8a;
const YEAR_W: u8 = 0x8c;
const WP_W: u8 = 0x8e;
/// Read command = the write command with bit0 set.
const READ: u8 = 0x01;

/// Busy-wait one inter-edge settle period.
fn settle() {
    cortex_m::asm::delay(EDGE_DELAY);
}

/// BCD -> decimal.
fn bcd2dec(b: u8) -> u8 {
    (b >> 4) * 10 + (b & 0x0f)
}

/// Decimal -> BCD.
fn dec2bcd(d: u8) -> u8 {
    ((d / 10) << 4) | (d % 10)
}

struct ServerImpl {
    sio: rp235x_pac::SIO,
}

impl ServerImpl {
    /// Drive `pin` HIGH (`high = true`) or LOW. Assumes `pin` is an output.
    fn set_pin(&self, pin: u32, high: bool) {
        if high {
            self.sio
                .gpio_out_set()
                .write(|w| unsafe { w.bits(1 << pin) });
        } else {
            self.sio
                .gpio_out_clr()
                .write(|w| unsafe { w.bits(1 << pin) });
        }
    }

    /// Make IO an output (enable the SIO output driver).
    fn io_output(&self) {
        self.sio.gpio_oe_set().write(|w| unsafe { w.bits(1 << IO) });
    }

    /// Make IO an input (release the SIO output driver so the DS1302 drives it).
    fn io_input(&self) {
        self.sio.gpio_oe_clr().write(|w| unsafe { w.bits(1 << IO) });
    }

    /// Read the IO line: true = HIGH.
    fn io_read(&self) -> bool {
        (self.sio.gpio_in().read().bits() >> IO) & 1 == 1
    }

    /// One SCLK pulse: HIGH, settle, LOW, settle.
    fn clock_pulse(&self) {
        self.set_pin(SCLK, true);
        settle();
        self.set_pin(SCLK, false);
        settle();
    }

    /// Shift a byte out LSB-first. IO is driven; the DS1302 samples on the
    /// SCLK rising edge.
    fn write_byte(&self, b: u8) {
        for i in 0..8 {
            self.set_pin(IO, (b >> i) & 1 == 1);
            settle();
            self.clock_pulse();
        }
    }

    /// Shift a byte in LSB-first. IO must already be an input; the DS1302
    /// presents the next bit on the SCLK falling edge, so the current bit is
    /// valid before we pulse -- read first, then clock.
    fn read_byte(&self) -> u8 {
        self.io_input();
        // The last command bit drove IO (read commands end in a 1), so the line
        // is left high; give the released line time to settle to the DS1302's
        // bit 0 before sampling, else bit 0 reads the residual high.
        cortex_m::asm::delay(EDGE_DELAY * 4);
        let mut result = 0u8;
        for i in 0..8 {
            if self.io_read() {
                result |= 1 << i;
            }
            self.clock_pulse();
        }
        result
    }

    /// Full single-byte register read: CE high, command, read, CE low.
    fn read_reg(&self, cmd: u8) -> u8 {
        self.set_pin(CE, true);
        settle();
        self.write_byte(cmd);
        let val = self.read_byte();
        self.set_pin(CE, false);
        settle();
        // Leave IO as an output again for the next write-side transfer.
        self.io_output();
        val
    }

    /// Full single-byte register write: CE high, command, data, CE low.
    fn write_reg(&self, cmd: u8, val: u8) {
        self.set_pin(CE, true);
        settle();
        self.write_byte(cmd);
        self.write_byte(val);
        self.set_pin(CE, false);
        settle();
    }
}

impl idl::InOrderRp235xDs1302Impl for ServerImpl {
    fn now(
        &mut self,
        _: &RecvMessage,
    ) -> Result<u64, RequestError<Infallible>> {
        // Mask off the DS1302 status/mode bits before decoding: seconds bit7 is
        // CH (clock-halt), hours bit7 selects 12/24h (we run 24h -> keep 0x3f).
        let sec = bcd2dec(self.read_reg(SECONDS_W | READ) & 0x7f);
        let min = bcd2dec(self.read_reg(MINUTES_W | READ) & 0x7f);
        let hour = bcd2dec(self.read_reg(HOURS_W | READ) & 0x3f);
        let date = bcd2dec(self.read_reg(DATE_W | READ) & 0x3f);
        let month = bcd2dec(self.read_reg(MONTH_W | READ) & 0x1f);
        let weekday = bcd2dec(self.read_reg(WEEKDAY_W | READ) & 0x07);
        let year = bcd2dec(self.read_reg(YEAR_W | READ));
        Ok((sec as u64)
            | (min as u64) << 8
            | (hour as u64) << 16
            | (date as u64) << 24
            | (month as u64) << 32
            | (weekday as u64) << 40
            | (year as u64) << 48)
    }

    fn set(
        &mut self,
        _: &RecvMessage,
        packed: u64,
    ) -> Result<(), RequestError<Ds1302Error>> {
        let sec = packed as u8;
        let min = (packed >> 8) as u8;
        let hour = (packed >> 16) as u8;
        let date = (packed >> 24) as u8;
        let month = (packed >> 32) as u8;
        let weekday = (packed >> 40) as u8;
        let year = (packed >> 48) as u8;
        // Reject out-of-range fields rather than writing garbage BCD.
        if sec > 59
            || min > 59
            || hour > 23
            || !(1..=31).contains(&date)
            || !(1..=12).contains(&month)
            || weekday > 7
            || year > 99
        {
            return Err(Ds1302Error::BadArg.into());
        }
        // Clear write-protect (WP bit7 = 0) before writing any clock register.
        self.write_reg(WP_W, 0x00);
        // seconds bit7 = CH: writing 0 starts the oscillator.
        self.write_reg(SECONDS_W, dec2bcd(sec));
        self.write_reg(MINUTES_W, dec2bcd(min));
        self.write_reg(HOURS_W, dec2bcd(hour));
        self.write_reg(DATE_W, dec2bcd(date));
        self.write_reg(MONTH_W, dec2bcd(month));
        self.write_reg(WEEKDAY_W, dec2bcd(weekday));
        self.write_reg(YEAR_W, dec2bcd(year));
        Ok(())
    }
}

impl idol_runtime::NotificationHandler for ServerImpl {
    fn current_notification_mask(&self) -> u32 {
        0
    }
    fn handle_notification(&mut self, _bits: userlib::NotificationBits) {}
}

/// One-time hardware setup: route GP6/7/8 to SIO, make CE/SCLK/IO outputs, and
/// set the idle state (CE low, SCLK low).
fn setup(p: &rp235x_pac::Peripherals) {
    for pin in [CE, IO, SCLK] {
        p.PADS_BANK0.gpio(pin as usize).modify(|_, w| {
            w.od().clear_bit();
            w.iso().clear_bit();
            w.ie().set_bit()
        });
        p.IO_BANK0
            .gpio(pin as usize)
            .gpio_ctrl()
            .modify(|_, w| unsafe { w.funcsel().bits(FUNCSEL_SIO) });
    }
    // Idle: CE low, SCLK low, IO output-low. Set the levels before enabling the
    // output drivers so the lines never glitch high.
    p.SIO
        .gpio_out_clr()
        .write(|w| unsafe { w.bits((1 << CE) | (1 << IO) | (1 << SCLK)) });
    p.SIO
        .gpio_oe_set()
        .write(|w| unsafe { w.bits((1 << CE) | (1 << IO) | (1 << SCLK)) });
}

#[export_name = "main"]
fn main() -> ! {
    let p = unsafe { rp235x_pac::Peripherals::steal() };
    setup(&p);
    let mut server = ServerImpl { sio: p.SIO };
    let mut incoming = [0u8; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    use drv_rp235x_ds1302_api::Ds1302Error;
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
