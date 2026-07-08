// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! PWM driver server for the RP2350 (RP235x).
//!
//! Minimal duty-cycle control behind an Idol interface (`idl/rp235x-pwm.idol`):
//! `set_duty` runs one channel of a slice at 1 kHz with a percent duty cycle;
//! `disable` stops a slice; `tone` plays a ~50% square wave on channel A at a
//! given frequency for a bounded time, then silences the slice. The GPIO must
//! separately be function-selected to PWM (funcsel 4) via the GPIO driver.
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

/// Integer clock divider for `tone`: 150 MHz / 25 = 6 MHz count rate. Chosen
/// so `top = 6_000_000 / freq_hz - 1` stays inside u16 across the audible
/// range (100 Hz -> 59999, 6000 Hz -> 999; both fit).
const TONE_DIV_INT: u8 = 25;
/// PWM count rate under `TONE_DIV_INT`, in Hz.
const TONE_CLK_HZ: u32 = 6_000_000;
/// Lowest tone frequency accepted by `tone`, in Hz.
const TONE_MIN_HZ: u32 = 100;
/// Highest tone frequency accepted by `tone`, in Hz.
const TONE_MAX_HZ: u32 = 6000;
/// Longest tone duration accepted by `tone`, in ms (clamped, not rejected).
const TONE_MAX_MS: u32 = 5000;

struct ServerImpl {
    pwm: rp235x_pac::PWM,
}

impl ServerImpl {
    /// Program a slice's integer clock divider and counter wrap (`top`). Shared
    /// by `set_duty` (fixed 1 kHz) and `tone` (variable), which differ only in
    /// the divider/top values and which channel compare they load.
    fn program_slice(&self, slice: usize, div_int: u8, top: u16) {
        let ch = self.pwm.ch(slice);
        ch.div().write(|w| unsafe { w.int().bits(div_int) });
        ch.top().write(|w| unsafe { w.top().bits(top) });
    }

    /// Enable (`on = true`) or disable a slice's counter.
    fn enable_slice(&self, slice: usize, on: bool) {
        self.pwm.ch(slice).csr().modify(|_, w| w.en().bit(on));
    }
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
        self.program_slice(slice as usize, DIV_INT, TOP);
        let level = percent as u16 * 10;
        let ch = self.pwm.ch(slice as usize);
        if chan_b != 0 {
            ch.cc().modify(|_, w| unsafe { w.b().bits(level) });
        } else {
            ch.cc().modify(|_, w| unsafe { w.a().bits(level) });
        }
        self.enable_slice(slice as usize, true);
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
        self.enable_slice(slice as usize, false);
        Ok(())
    }

    fn tone(
        &mut self,
        _: &RecvMessage,
        slice: u8,
        freq_hz: u32,
        ms: u32,
    ) -> Result<(), RequestError<PwmError>> {
        if slice > MAX_SLICE || !(TONE_MIN_HZ..=TONE_MAX_HZ).contains(&freq_hz)
        {
            return Err(PwmError::BadArg.into());
        }
        // 6 MHz count rate / freq_hz gives the period in counts; minus one for
        // the wrap value. Both bounds fit u16 (see TONE_DIV_INT).
        let top = (TONE_CLK_HZ / freq_hz - 1) as u16;
        // ~50% duty: channel A compare at half the period (rounded up).
        let cc_a = (top / 2) + 1;
        self.program_slice(slice as usize, TONE_DIV_INT, top);
        self.pwm
            .ch(slice as usize)
            .cc()
            .modify(|_, w| unsafe { w.a().bits(cc_a) });
        self.enable_slice(slice as usize, true);

        // Play for the bounded duration, then silence the slice. NOTE: this
        // sleeps INSIDE the op handler, so the single-threaded PWM server serves
        // no other IPC for up to TONE_MAX_MS. Harmless while the shell is the
        // only, strictly-synchronous PWM client; if a second client is added,
        // move the timing to the caller (start/stop ops) or drive the samples
        // via DMA so the server stays responsive.
        let ms = ms.clamp(1, TONE_MAX_MS);
        userlib::hl::sleep_for(ms as u64);
        self.enable_slice(slice as usize, false);
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
