// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! PWM driver server for the RP2350 (RP235x).
//!
//! Minimal duty-cycle control behind an Idol interface (`idl/rp235x-pwm.idol`):
//! `set_duty` runs one channel of a slice at 1 kHz with a percent duty cycle;
//! `disable` stops a slice. The GPIO must separately be function-selected to
//! PWM (funcsel 4) via the GPIO driver.
//!
//! 1 kHz from clk_sys = 150 MHz: divider 150 with TOP = 999, so the duty
//! compare value is simply percent * 10. Brings the PWM block out of reset
//! via the sys server. Runs unprivileged with `uses = ["pwm"]`.

#![no_std]
#![no_main]

use drv_rp235x_pwm_api::PwmError;
use drv_rp235x_sys_api::{self as sys_api, Rp235xSys};
use idol_runtime::RequestError;
use userlib::{RecvMessage, task_slot};

task_slot!(SYS, sys);

/// The RP2350 has twelve PWM slices.
const MAX_SLICE: u8 = 11;
/// Counter wrap for a 1 kHz period at clk_sys/150 = 1 MHz.
const TOP: u16 = 999;
/// Integer clock divider: 150 MHz / 150 = 1 MHz count rate.
const DIV_INT: u8 = 150;

struct ServerImpl {
    pwm: rp235x_pac::PWM,
}

impl idl::InOrderRp235xPwmImpl for ServerImpl {
    fn set_duty(
        &mut self,
        _: &RecvMessage,
        slice: u8,
        chan_b: u8,
        percent: u8,
    ) -> Result<(), RequestError<PwmError>> {
        if slice > MAX_SLICE || percent > 100 {
            return Err(PwmError::BadArg.into());
        }
        let ch = self.pwm.ch(slice as usize);
        ch.div().write(|w| unsafe { w.int().bits(DIV_INT) });
        ch.top().write(|w| unsafe { w.top().bits(TOP) });
        let level = percent as u16 * 10;
        if chan_b != 0 {
            ch.cc().modify(|_, w| unsafe { w.b().bits(level) });
        } else {
            ch.cc().modify(|_, w| unsafe { w.a().bits(level) });
        }
        ch.csr().modify(|_, w| w.en().set_bit());
        Ok(())
    }

    fn disable(
        &mut self,
        _: &RecvMessage,
        slice: u8,
    ) -> Result<(), RequestError<PwmError>> {
        if slice > MAX_SLICE {
            return Err(PwmError::BadArg.into());
        }
        self.pwm
            .ch(slice as usize)
            .csr()
            .modify(|_, w| w.en().clear_bit());
        Ok(())
    }
}

impl idol_runtime::NotificationHandler for ServerImpl {
    fn current_notification_mask(&self) -> u32 {
        0
    }
    fn handle_notification(&mut self, _bits: userlib::NotificationBits) {}
}

#[export_name = "main"]
fn main() -> ! {
    // Bring the PWM block out of reset via the sys server.
    let sys = Rp235xSys::from(SYS.get_task_id());
    sys.leave_reset(sys_api::PWM);

    let p = unsafe { rp235x_pac::Peripherals::steal() };
    let mut server = ServerImpl { pwm: p.PWM };
    let mut incoming = [0u8; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    use drv_rp235x_pwm_api::PwmError;
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
