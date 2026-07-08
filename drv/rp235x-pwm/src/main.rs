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

use core::ptr::addr_of_mut;
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

// --- DMA-fed continuous audio (milestone 2: STEREO sine on the jack) ---
//
// Unlike `sine` (a busy-wait DDS that blocks the server and lands a few percent
// flat), this path is the FIRST non-blocking, exact-pitch audio: a DMA ring
// buffer of precomputed samples is streamed into the PWM slice-1 CC register,
// one sample per counter wrap. The DMA is paced entirely by the hardware wrap
// (TREQ = PWM_WRAP1), so it consumes zero CPU and the sample rate is exactly
// the wrap rate. The buffer holds a whole number of sine cycles, so the ring
// loops seamlessly with no phase discontinuity.
//
// STEREO: each ring slot is now a u32 = `(right << 16) | left`, written whole
// into the 32-bit CC register in ONE DMA transfer -- CC bits 0..15 = channel A
// (GP18 = left), bits 16..31 = channel B (GP19 = right). `audio_start` fills
// both halves with the same tone (both ears); `audio_stereo` fills each half
// with its own frequency (true stereo).

/// Samples in the live audio ring (the aligned window the DMA reads). Each
/// sample is a u32 stereo pair, so 1024 u32 = 4096 bytes, matching the ring
/// wrap (`AUDIO_RING_SIZE`). Longer than the LUT so several whole sine cycles
/// fit, keeping the loop-boundary pitch error small.
const AUDIO_BUF_SAMPLES: usize = 1024;
/// Ring size in bytes: 1024 u32 = 4096. The DMA read address wraps at this
/// power-of-two boundary, so the aligned window must start on a 4096-byte
/// boundary (see `aligned_ring_ptr`).
const AUDIO_RING_BYTES: usize = AUDIO_BUF_SAMPLES * 4;
/// DMA read-address ring wrap, as log2 of the ring size in bytes (4096 -> 12).
/// The read address wraps back to the window start every 4096 bytes, so the
/// same samples replay forever.
const AUDIO_RING_SIZE: u8 = 12;
/// Integer clock divider for the audio carrier slice.
const AUDIO_DIV_INT: u8 = 6;
/// Counter wrap for the audio carrier: 10-bit-ish so LUT values (0..=1023) map
/// straight to the channel-A compare with no scaling.
const AUDIO_TOP: u16 = 1023;
/// Sample rate = one sample per wrap = clk_sys / (AUDIO_DIV_INT * (AUDIO_TOP+1))
/// = 150e6 / (6 * 1024) = 24414 Hz. Both the pitch snap and the DMA pacing use
/// this exact rate. Frequency resolution is AUDIO_SAMPLE_RATE / AUDIO_BUF_SAMPLES
/// = 24414 / 1024 ~= 24 Hz -- the whole-cycle snap already quantizes to this,
/// so the coarser step (vs the old 2048-sample buffer) is acceptable.
const AUDIO_SAMPLE_RATE: u32 = 24414;
/// DMA channel used for the audio ring.
const AUDIO_DMA_CH: usize = 0;
/// PWM slice that carries the audio (slice 1: chan A = GP18 = jack left,
/// chan B = GP19 = jack right). A word write to the CC register drives both.
const AUDIO_SLICE: usize = 1;
/// DMA TREQ (dreq) select for the PWM slice-1 wrap: pace one transfer per wrap.
const AUDIO_TREQ_PWM_WRAP1: u8 = 33;
/// DMA CTRL data size: 2 = word (u32). Each transfer writes the full 32-bit CC
/// register in one go: low half = channel A (left), high half = channel B
/// (right). The read address auto-increments 4 bytes per transfer.
const AUDIO_DATA_SIZE_WORD: u8 = 2;
/// Bit position of channel B (right) in the packed u32 sample / the 32-bit CC
/// register: A (left) in bits 0-15, B (right) in bits 16-31.
const CH_B_SHIFT: u32 = 16;
/// DMA NORMAL-mode transfer reload (28-bit max). ENDLESS mode with COUNT=0
/// transferred nothing on HW, so a large finite reload is used: at
/// `AUDIO_SAMPLE_RATE` this is ~3 h of continuous audio before the channel
/// halts (a documented silent-stop, not a hang; re-issue `audio` to resume).
const AUDIO_TRANS_COUNT: u32 = 0x0fff_ffff;
/// Lowest audio frequency accepted by `audio_start`, in Hz.
const AUDIO_MIN_HZ: u32 = 50;
/// Highest audio frequency accepted by `audio_start`, in Hz.
const AUDIO_MAX_HZ: u32 = 8000;

/// Backing storage for the audio ring, sized to hold a whole 4096-byte
/// naturally-aligned window regardless of where the linker places the static.
///
/// The DMA read-address ring wraps at a power-of-two boundary (the low
/// `AUDIO_RING_SIZE` address bits are cycled), so the samples the DMA reads MUST
/// occupy a 4096-byte-aligned 4096-byte window. A `#[repr(align(4096))]` static
/// cannot guarantee that here: on this thumbv8m target Hubris aligns task RAM
/// regions only to 32-byte chunks (MpuAlignment::Chunk(32)), so a 4096-aligned
/// static forces unbounded intra-region padding whose size the task autosizer
/// mis-measures (the region base is 32-aligned, not 4096-aligned). Instead we
/// over-allocate by one extra ring and pick the first 4096-aligned window inside
/// it at runtime (`aligned_ring_ptr`): 2x the ring guarantees a full aligned
/// window fits after up to 4092 bytes of skip. The DMA reads are word (u32).
/// Unit is ELEMENTS (u32 stereo samples): two rings' worth (2 * 1024 = 2048 u32
/// = 8192 bytes, same store size as the old u16 version), hence the 2x
/// over-allocation. The 4096-byte alignment requirement is unchanged.
const AUDIO_STORE_LEN: usize = 2 * AUDIO_BUF_SAMPLES;

/// Backing storage. The DMA engine (a bus master, not bound by the task MPU)
/// reads the aligned window directly. Accessed only via raw pointers to avoid
/// creating references to the `static mut` (`static_mut_refs`).
static mut AUDIO_STORE: [u32; AUDIO_STORE_LEN] = [0; AUDIO_STORE_LEN];

/// Address of the first 4096-byte-aligned window inside `AUDIO_STORE`. This is
/// the DMA read base and the fill target. Because the store is twice the ring
/// size, a full 4096-byte window always fits at or after this address.
fn aligned_ring_ptr() -> *mut u32 {
    let base = addr_of_mut!(AUDIO_STORE) as usize;
    let aligned = (base + (AUDIO_RING_BYTES - 1)) & !(AUDIO_RING_BYTES - 1);
    aligned as *mut u32
}

struct ServerImpl {
    pwm: rp235x_pac::PWM,
    dma: rp235x_pac::DMA,
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

    /// Abort the audio DMA channel and WAIT for the abort to drain. Per the
    /// RP2350 datasheet, CHAN_ABORT self-clears once the in-flight transfer has
    /// flushed, and it is unsafe to reconfigure or restart the channel until it
    /// reads back zero. Called before (re)arming in `audio_start` and in
    /// `audio_stop`. On an already-idle channel this returns immediately.
    fn abort_audio_dma(&self) {
        self.dma
            .chan_abort()
            .write(|w| unsafe { w.chan_abort().bits(1 << AUDIO_DMA_CH) });
        while self.dma.chan_abort().read().bits() & (1 << AUDIO_DMA_CH) != 0 {}
    }

    /// Snap a request to a whole number of sine cycles across the buffer so the
    /// ring loops with no phase discontinuity. k = round(freq * samples /
    /// sample_rate); the actual played frequency is k * sample_rate / samples.
    /// k >= 1.
    fn cycle_count(freq_hz: u32) -> u32 {
        ((freq_hz as u64 * AUDIO_BUF_SAMPLES as u64
            + AUDIO_SAMPLE_RATE as u64 / 2)
            / AUDIO_SAMPLE_RATE as u64)
            .max(1) as u32
    }

    /// Fill the aligned u32 ring window with a stereo sine pair: `k_left` whole
    /// cycles on channel A (low half, GP18/left) and `k_right` whole cycles on
    /// channel B (high half, GP19/right). For each slot, index the 256-entry
    /// SINE_LUT so that k whole cycles span the AUDIO_BUF_SAMPLES-slot window.
    /// The LUT is already scaled 0..=AUDIO_TOP, so its values are the compare
    /// value directly. All arithmetic is u32-safe: i * k * 256 stays well under
    /// u32::MAX (i < 1024, k <= ~336, so < 89M).
    fn fill_ring(&self, k_left: u32, k_right: u32) {
        let lut_len = SINE_LUT.len() as u32;
        let ring = aligned_ring_ptr();
        for i in 0..AUDIO_BUF_SAMPLES {
            let idx_l = ((i as u32).wrapping_mul(k_left).wrapping_mul(lut_len)
                / AUDIO_BUF_SAMPLES as u32)
                % lut_len;
            let idx_r =
                ((i as u32).wrapping_mul(k_right).wrapping_mul(lut_len)
                    / AUDIO_BUF_SAMPLES as u32)
                    % lut_len;
            let left = SINE_LUT[idx_l as usize] as u32;
            let right = SINE_LUT[idx_r as usize] as u32;
            let pair = (right << CH_B_SHIFT) | left;
            // SAFETY: `ring` is the 4096-aligned window inside AUDIO_STORE,
            // which is 2x the ring size, so slots 0..AUDIO_BUF_SAMPLES are in
            // bounds. Single-threaded server, no references taken, no aliasing.
            unsafe {
                ring.add(i).write_volatile(pair);
            }
        }
    }

    /// Shared arm path for `audio_start` and `audio_stereo`: abort any running
    /// audio DMA, fill the ring with the two (possibly equal) tones, start the
    /// carrier slice with both CC channels parked at mid-scale, and arm the DMA
    /// for word (32-bit) writes into the full CC register.
    fn arm_audio(&self, freq_left: u32, freq_right: u32) {
        // Abort + drain any currently-running audio DMA before refilling the
        // ring or re-arming. Reconfiguring or re-triggering a BUSY channel
        // (e.g. back-to-back `audio <hz>` with no `stop`) is undefined on the
        // RP2350 and can desync the read pointer or wedge the channel.
        self.abort_audio_dma();

        self.fill_ring(
            Self::cycle_count(freq_left),
            Self::cycle_count(freq_right),
        );

        // Program slice 1 as the carrier and park BOTH channels at mid-scale so
        // both pins sit at the DC bias until the first DMA sample lands.
        self.program_slice(AUDIO_SLICE, AUDIO_DIV_INT, AUDIO_TOP);
        self.pwm.ch(AUDIO_SLICE).cc().modify(|_, w| unsafe {
            w.a().bits(AUDIO_TOP / 2);
            w.b().bits(AUDIO_TOP / 2);
            w
        });
        self.enable_slice(AUDIO_SLICE, true);

        // Configure DMA channel 0 to stream the ring into the slice-1 CC
        // register, one WORD per PWM_WRAP1 dreq. A 32-bit write sets both
        // channel A (low half) and channel B (high half). Read increments +
        // wraps the ring (RING_SIZE); write is fixed at CC. NORMAL mode with a
        // large finite reload loops for hours until audio_stop aborts. Requires
        // SECCFG_CH0.P cleared in the pre-kernel main so this unprivileged task
        // may program the channel.
        let read_addr = aligned_ring_ptr() as u32;
        let write_addr = self.pwm.ch(AUDIO_SLICE).cc().as_ptr() as u32;
        let ch = self.dma.ch(AUDIO_DMA_CH);
        ch.ch_read_addr().write(|w| unsafe { w.bits(read_addr) });
        ch.ch_write_addr().write(|w| unsafe { w.bits(write_addr) });
        // NORMAL mode with a large finite reload (AUDIO_TRANS_COUNT): ENDLESS
        // mode with COUNT=0 started zero transfers on HW (CC stuck at the park
        // value, BUSY=0), so a nonzero reload is what actually arms the
        // sequence.
        ch.ch_trans_count()
            .write(|w| unsafe { w.count().bits(AUDIO_TRANS_COUNT) });
        ch.ch_ctrl_trig().write(|w| unsafe {
            w.en().bit(true);
            w.data_size().bits(AUDIO_DATA_SIZE_WORD);
            w.incr_read().bit(true);
            w.incr_write().bit(false);
            w.ring_size().bits(AUDIO_RING_SIZE);
            w.ring_sel().bit(false); // ring on the READ address
            w.treq_sel().bits(AUDIO_TREQ_PWM_WRAP1);
            w.chain_to().bits(AUDIO_DMA_CH as u8); // chain to self = no chain
            w.high_priority().bit(false);
            w
        });
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

    fn audio_start(
        &mut self,
        _: &RecvMessage,
        freq_hz: u32,
    ) -> Result<(), RequestError<PwmError>> {
        if !(AUDIO_MIN_HZ..=AUDIO_MAX_HZ).contains(&freq_hz) {
            return Err(PwmError::BadArg.into());
        }
        // Same tone in both ears: arm with equal left/right frequencies. The
        // shell reports the request, not the snapped value.
        self.arm_audio(freq_hz, freq_hz);
        Ok(())
    }

    fn audio_stereo(
        &mut self,
        _: &RecvMessage,
        hz_left: u32,
        hz_right: u32,
    ) -> Result<(), RequestError<PwmError>> {
        if !(AUDIO_MIN_HZ..=AUDIO_MAX_HZ).contains(&hz_left)
            || !(AUDIO_MIN_HZ..=AUDIO_MAX_HZ).contains(&hz_right)
        {
            return Err(PwmError::BadArg.into());
        }
        // True stereo: independent tone per ear (GP18 = left, GP19 = right).
        self.arm_audio(hz_left, hz_right);
        Ok(())
    }

    fn audio_stop(
        &mut self,
        _: &RecvMessage,
    ) -> Result<(), RequestError<PwmError>> {
        // Abort + DRAIN the channel (abort_audio_dma polls CHAN_ABORT back to 0 --
        // it is unsafe to restart until then), then clear EN and stop the carrier.
        self.abort_audio_dma();
        self.dma
            .ch(AUDIO_DMA_CH)
            .ch_ctrl_trig()
            .write(|w| w.en().bit(false));
        self.enable_slice(AUDIO_SLICE, false);
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
    // Bring the PWM block out of reset via the sys server. The DMA block used by
    // the audio path is un-reset once in the pre-kernel startup (alongside the
    // SECCFG_CH0.P grant -- see rp235x-startup), so it is not re-done here.
    let sys = Rp235xSys::from(SYS.get_task_id());
    sys.leave_reset(sys_api::PWM);

    // The `sine` DDS loop paces samples with `cortex_m::asm::delay`, a
    // self-contained busy-wait, so no cycle-counter or timer peripheral needs
    // enabling here (the DWT/CYCCNT and RP2350 TIMER both stay stuck on this
    // board and would wedge the loop).
    let p = unsafe { rp235x_pac::Peripherals::steal() };
    let mut server = ServerImpl {
        pwm: p.PWM,
        dma: p.DMA,
    };
    let mut incoming = [0u8; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    use drv_rp235x_pwm_api::PwmError;
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
