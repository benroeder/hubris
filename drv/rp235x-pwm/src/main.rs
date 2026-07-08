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

// 256-entry sine LUT (u16, scaled 0..=SINE_TOP with a half-scale DC offset),
// generated on the host by build.rs. Pulled in as `SINE_LUT`.
include!(concat!(env!("OUT_DIR"), "/sine_lut.rs"));

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

// --- Sine (PWM-as-1-bit-DAC via DDS) ---
//
// The carrier is a fast, ultrasonic PWM whose DUTY we modulate at an audio
// sample rate to trace a sine. A phase accumulator (DDS) indexes the sine LUT;
// each sample writes a new channel-A compare value, and the RC of the load (or
// the piezo's own mass) averages the 1-bit carrier back into an analog voltage.

/// Peak duty compare value for sine samples: 10-bit resolution. MUST match
/// `SINE_TOP` in build.rs (the LUT is scaled to this counter wrap).
const SINE_TOP: u16 = 1023;
/// Integer divider for the sine carrier: 150 MHz / (SINE_TOP + 1 = 1024) gives
/// a ~146 kHz PWM carrier -- comfortably ultrasonic, so the carrier itself is
/// inaudible and only the duty-modulated audio envelope is heard.
const SINE_DIV_INT: u8 = 1;
/// Audio sample rate of the DDS loop, in Hz. 20 kHz covers the 50..=5000 Hz
/// tone range with headroom above Nyquist.
const SAMPLE_RATE: u32 = 20_000;
/// Approximate clk_sys cycles between samples at SAMPLE_RATE: 150 MHz / 20 kHz
/// = 7500. The DDS loop paces each sample with `cortex_m::asm::delay`, a
/// calibrated busy-wait of roughly this many CPU cycles. Because the per-sample
/// LUT read plus PAC write (tens of cycles) is NOT subtracted, the true sample
/// period is a little longer than SINE_CYCLES_PER_SAMPLE, so the effective
/// sample rate is slightly below SAMPLE_RATE and the pitch is APPROXIMATE (a
/// few percent flat, constant across frequency). This is deliberate: the
/// RP2350 hardware microsecond TIMER is not usable here because its feeding
/// TICKS tick generator is never enabled at boot (neither the ROM nor
/// rp235x-startup turns it on), so TIMERAWL stays stuck at zero -- the same
/// wedge the earlier DWT (CYCCNT) approach hit. An exact hardware-timed rate
/// lands with the planned DMA-paced follow-up. `asm::delay` never wedges: it is
/// a self-contained loop that does not depend on any counter peripheral.
const SINE_CYCLES_PER_SAMPLE: u32 = 150_000_000 / SAMPLE_RATE;
/// DDS phase accumulator right-shift to index SINE_LUT: 32 - log2(SINE_LUT_LEN
/// = 256) = 24. Pairs with build.rs's SINE_LUT_LEN (change both together).
const SINE_LUT_INDEX_SHIFT: u32 = 24;
/// Lowest sine frequency accepted, in Hz.
const SINE_MIN_HZ: u32 = 50;
/// Highest sine frequency accepted, in Hz.
const SINE_MAX_HZ: u32 = 5000;
/// Longest sine duration accepted, in ms (clamped, not rejected). Bounded to
/// 2000 because the DDS loop is a hard busy-wait: it monopolizes the
/// single-threaded PWM server's CPU with no yield point for up to SINE_MAX_MS
/// (unlike `tone`, which sleeps cooperatively). The busy-wait is required for
/// accurate sub-ms sample pacing without DMA and is acceptable for the single
/// synchronous shell client; the non-blocking fix is DMA-fed PWM (planned
/// follow-up).
const SINE_MAX_MS: u32 = 2000;

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

    fn sine(
        &mut self,
        _: &RecvMessage,
        slice: u8,
        freq_hz: u32,
        ms: u32,
    ) -> Result<(), RequestError<PwmError>> {
        if slice > MAX_SLICE || !(SINE_MIN_HZ..=SINE_MAX_HZ).contains(&freq_hz)
        {
            return Err(PwmError::BadArg.into());
        }

        // Start the ultrasonic carrier. The DDS itself starts at mid-scale
        // (phase 0 -> SINE_LUT[0], a mid-scale zero-crossing) and sample 0 is
        // written on the first loop iteration, so there is no start transient
        // and no separate park write is needed.
        self.program_slice(slice as usize, SINE_DIV_INT, SINE_TOP);
        self.enable_slice(slice as usize, true);

        // DDS: a 32-bit phase accumulator; the top 8 bits index the 256-entry
        // LUT. phase_inc = freq / sample_rate in Q32 fixed point. n is the
        // number of samples for the requested (clamped) duration.
        //
        // Pacing: like `tone`, this blocks the single-threaded PWM server for
        // the whole play duration (busy-wait DDS in-handler, up to SINE_MAX_MS).
        // Fine while the shell is the only, strictly-synchronous PWM client; the
        // non-blocking follow-up is to drive the samples via DMA. Each sample is
        // paced by `cortex_m::asm::delay(SINE_CYCLES_PER_SAMPLE)`, a calibrated
        // busy-wait that needs no counter peripheral, so it cannot wedge (unlike
        // the DWT/TIMER approaches, whose counters never tick on this board).
        // The pitch is APPROXIMATE (see SINE_CYCLES_PER_SAMPLE): the per-sample
        // work is not subtracted, so the rate runs slightly below SAMPLE_RATE.
        let ms = ms.clamp(1, SINE_MAX_MS);
        let phase_inc = (((freq_hz as u64) << 32) / SAMPLE_RATE as u64) as u32;
        let n = (ms as u64 * SAMPLE_RATE as u64 / 1000) as u32;
        let mut phase: u32 = 0;
        for _ in 0..n {
            let idx = (phase >> SINE_LUT_INDEX_SHIFT) as usize;
            self.pwm
                .ch(slice as usize)
                .cc()
                .modify(|_, w| unsafe { w.a().bits(SINE_LUT[idx]) });
            phase = phase.wrapping_add(phase_inc);
            cortex_m::asm::delay(SINE_CYCLES_PER_SAMPLE);
        }

        // Silence: stop the counter so the carrier no longer drives the pin.
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

    // The `sine` DDS loop paces samples with `cortex_m::asm::delay`, a
    // self-contained busy-wait, so no cycle-counter or timer peripheral needs
    // enabling here (the DWT/CYCCNT and RP2350 TIMER both stay stuck on this
    // board and would wedge the loop).
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
