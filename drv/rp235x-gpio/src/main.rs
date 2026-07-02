// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! GPIO driver server for the RP2350 (RP235x).
//!
//! An idiomatic Hubris Idol server: other tasks call `configure_output`,
//! `set_high`/`set_low`/`toggle`, `configure_input`, and `read` over IPC (see
//! `idl/rp235x-gpio.idol`) instead of touching the registers directly. Covers Bank 0
//! pins 0-31 via the single-cycle IO block (SIO). Owns IO_BANK0 / PADS_BANK0 / SIO
//! and (at startup) RESETS via `uses`; runs unprivileged (those blocks are ACCESSCTRL
//! Secure/any-privilege).

#![no_std]
#![no_main]

use drv_rp235x_gpio_api::GpioError;
use drv_rp235x_sys_api::{self as sys_api, Rp235xSys};
use idol_runtime::RequestError;
use userlib::{RecvMessage, task_slot};

task_slot!(SYS, sys);

/// Highest Bank 0 pin reachable through the SIO low registers.
const MAX_PIN: u8 = 31;
/// IO_BANK0 FUNCSEL value that connects a pin to the SIO block.
const FUNCSEL_SIO: u8 = 5;

struct ServerImpl {
    sio: rp235x_pac::SIO,
    io_bank0: rp235x_pac::IO_BANK0,
    pads_bank0: rp235x_pac::PADS_BANK0,
}

impl ServerImpl {
    fn check(pin: u8) -> Result<usize, RequestError<GpioError>> {
        if pin > MAX_PIN {
            Err(GpioError::InvalidPin.into())
        } else {
            Ok(pin as usize)
        }
    }

    fn funcsel_sio(&self, pin: usize) {
        self.io_bank0
            .gpio(pin)
            .gpio_ctrl()
            .modify(|_, w| unsafe { w.funcsel().bits(FUNCSEL_SIO) });
    }
}

impl idl::InOrderRp235xGpioImpl for ServerImpl {
    fn configure_output(
        &mut self,
        _: &RecvMessage,
        pin: u8,
    ) -> Result<(), RequestError<GpioError>> {
        let p = Self::check(pin)?;
        // Enable the output driver (OD=0) and clear the RP2350 isolation latch.
        self.pads_bank0
            .gpio(p)
            .modify(|_, w| w.od().clear_bit().iso().clear_bit());
        self.funcsel_sio(p);
        self.sio
            .gpio_oe_set()
            .write(|w| unsafe { w.bits(1u32 << pin) });
        Ok(())
    }

    fn configure_input(
        &mut self,
        _: &RecvMessage,
        pin: u8,
    ) -> Result<(), RequestError<GpioError>> {
        let p = Self::check(pin)?;
        // Disable the output driver, enable the input buffer, clear isolation.
        self.pads_bank0
            .gpio(p)
            .modify(|_, w| w.od().set_bit().ie().set_bit().iso().clear_bit());
        self.funcsel_sio(p);
        self.sio
            .gpio_oe_clr()
            .write(|w| unsafe { w.bits(1u32 << pin) });
        Ok(())
    }

    fn set_high(
        &mut self,
        _: &RecvMessage,
        pin: u8,
    ) -> Result<(), RequestError<GpioError>> {
        Self::check(pin)?;
        self.sio
            .gpio_out_set()
            .write(|w| unsafe { w.bits(1u32 << pin) });
        Ok(())
    }

    fn set_low(
        &mut self,
        _: &RecvMessage,
        pin: u8,
    ) -> Result<(), RequestError<GpioError>> {
        Self::check(pin)?;
        self.sio
            .gpio_out_clr()
            .write(|w| unsafe { w.bits(1u32 << pin) });
        Ok(())
    }

    fn toggle(
        &mut self,
        _: &RecvMessage,
        pin: u8,
    ) -> Result<(), RequestError<GpioError>> {
        Self::check(pin)?;
        self.sio
            .gpio_out_xor()
            .write(|w| unsafe { w.bits(1u32 << pin) });
        Ok(())
    }

    fn read(
        &mut self,
        _: &RecvMessage,
        pin: u8,
    ) -> Result<u8, RequestError<GpioError>> {
        Self::check(pin)?;
        Ok(((self.sio.gpio_in().read().bits() >> pin) & 1) as u8)
    }

    fn set_function(
        &mut self,
        _: &RecvMessage,
        pin: u8,
        funcsel: u8,
    ) -> Result<(), RequestError<GpioError>> {
        Self::check(pin)?;
        if funcsel > 0x1f {
            // Not literally a pin problem, but the only error this API has.
            return Err(GpioError::InvalidPin.into());
        }
        // Un-isolate the pad and enable both directions (output drive for
        // output functions, input buffer for input functions -- harmless for
        // the unused direction), then route the pin.
        self.pads_bank0
            .gpio(pin as usize)
            .modify(|_, w| w.od().clear_bit().ie().set_bit().iso().clear_bit());
        self.io_bank0
            .gpio(pin as usize)
            .gpio_ctrl()
            .modify(|_, w| unsafe { w.funcsel().bits(funcsel) });
        Ok(())
    }
}

impl idol_runtime::NotificationHandler for ServerImpl {
    fn current_notification_mask(&self) -> u32 {
        0 // no notifications
    }
    fn handle_notification(&mut self, _bits: userlib::NotificationBits) {}
}

#[export_name = "main"]
fn main() -> ! {
    // Bring the GPIO blocks out of reset via the sys server (single RESETS owner).
    let sys = Rp235xSys::from(SYS.get_task_id());
    sys.leave_reset(sys_api::IO_BANK0 | sys_api::PADS_BANK0);

    let p = unsafe { rp235x_pac::Peripherals::steal() };
    let mut server = ServerImpl {
        sio: p.SIO,
        io_bank0: p.IO_BANK0,
        pads_bank0: p.PADS_BANK0,
    };
    let mut incoming = [0u8; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    use super::GpioError;
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
