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
use idol_runtime::{Leased, LenLimit, R, RequestError};
#[cfg(feature = "sdcard")]
use userlib::sys_get_timer;
use userlib::{RecvMessage, task_slot};

#[cfg(feature = "sdcard")]
use rp235x_audio_decode as decoder;
#[cfg(feature = "sdcard")]
mod sd;

#[cfg(feature = "sdcard")]
use drv_rp235x_gpio_api::Rp235xGpio;
#[cfg(feature = "sdcard")]
use drv_rp235x_sdcard_api::Rp235xSdcard;

task_slot!(SYS, sys);
#[cfg(feature = "sdcard")]
task_slot!(SDCARD, sdcard);
#[cfg(feature = "sdcard")]
task_slot!(GPIO, gpio);

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

// --- Streaming WAV player (milestone 3: PCM off the SD card -> jack) ---
//
// Reuses the PROVEN `audio` DMA path verbatim: PWM slice 1, DMA ch0, one u32
// (stereo pair) per PWM_WRAP1, ring over AUDIO_STORE. The only differences are
// (a) the slice divider is programmed for the FILE's sample rate instead of the
// fixed AUDIO_SAMPLE_RATE, and (b) the ring is refilled from a decoder instead
// of holding a fixed sine. The player BLOCKS the pwm server for the whole track
// (it poll-refills the ring half the DMA is not reading, in-handler) -- the same
// accepted trade-off as `tone`/`sine`, since the shell is the only synchronous
// client.

/// clk_sys in Hz: the PWM count clock before the slice divider.
#[cfg(feature = "sdcard")]
const CLK_SYS_HZ: u32 = 150_000_000;
/// Player carrier wrap: 10-bit, so a sample maps straight to the compare with
/// no scaling (same as AUDIO_TOP). One sample plays per wrap.
#[cfg(feature = "sdcard")]
const PLAY_TOP: u16 = AUDIO_TOP;
/// Lowest file sample rate the player accepts, in Hz (clamped, not rejected).
#[cfg(feature = "sdcard")]
const PLAY_MIN_RATE: u32 = 8_000;
/// Highest file sample rate the player accepts, in Hz (clamped, not rejected).
/// 48 kHz is the practical ceiling for a 10-bit wrap off a 150 MHz clock (the
/// divider bottoms out near 1.0).
#[cfg(feature = "sdcard")]
const PLAY_MAX_RATE: u32 = 48_000;
/// Right-shift turning a signed 16-bit sample into a 10-bit unsigned duty
/// around mid-scale: `(s >> 6) + 512`. 16-bit - 10-bit = 6 bits.
#[cfg(feature = "sdcard")]
const PLAY_SAMPLE_SHIFT: i32 = 6;
/// Mid-scale duty (silence) for the 10-bit player carrier: PLAY_TOP / 2 + 1.
#[cfg(feature = "sdcard")]
const PLAY_MID: i32 = 512;
/// Samples per ring HALF: the player refills one half while the DMA drains the
/// other. AUDIO_BUF_SAMPLES (1024) u32 slots total, so 512 per half.
#[cfg(feature = "sdcard")]
const PLAY_HALF_SAMPLES: usize = AUDIO_BUF_SAMPLES / 2;
/// Ring byte span of one half (512 u32 = 2048 bytes). Used to map the DMA read
/// address to the half it is currently draining.
#[cfg(feature = "sdcard")]
const PLAY_HALF_BYTES: u32 = (AUDIO_RING_BYTES / 2) as u32;
/// Stall timeout for the player poll loops, in ms. The loops make progress only
/// on DMA ring-half crossings; if the read pointer does not cross for this long
/// the channel has wedged, so the loop bails rather than spinning forever and
/// hanging the pwm server (which has no other escape). 500 ms is ~10x a ring
/// half even at the slowest supported rate (~43 ms at 8 kHz).
#[cfg(feature = "sdcard")]
const PLAY_STALL_MS: u64 = 500;
/// Check the stall timeout only once every this many poll iterations (a
/// power-of-two mask). The `sys_get_timer` kipc is far heavier than the
/// register read the loop spins on, so calling it every iteration could itself
/// eat the refill margin; amortize it. 256K polls is far below one ring half.
#[cfg(feature = "sdcard")]
const PLAY_STALL_POLL_MASK: u32 = 0x3ffff;

// --- Controllable 2-source mixer (milestone 4: WAV + sine, button crossfade) ---
//
// Reuses the whole `play_file` DMA path (slice 1, DMA ch0, the aligned ring)
// but fills the ring by BLENDING two sources per sample instead of copying one:
//   A = the WAV file (WavDecoder -> i16, mapped to a centered 10-bit duty).
//   B = a fixed 660 Hz DDS sine over SINE_LUT (distinct from the 440 Hz test
//       file so the blend is audible), whose phase ALWAYS advances so a fade-in
//       has no restart transient.
// out = PLAY_MID + ((ga * wav_centered + gb * sine_centered) >> 8), clamped to
// 0..=PLAY_TOP, packed into both CC halves. `ga`/`gb` are Q8 gains 0..=256.
//
// Auto-fade: the buttons pick a TARGET source; the gains ramp toward it one Q8
// step PER SAMPLE (no per-half zipper). GP20 -> WAV (ga->256, gb->0); GP21 ->
// sine (ga->0, gb->256). The linear crossfade dips slightly at the midpoint;
// TODO equal-power (multiply by sqrt curves) if the dip is audible.

/// Fixed sine frequency for source B, in Hz. 660 (an E5, a fifth above the
/// 440 Hz TEST.WAV) so the two sources are clearly distinguishable by ear as
/// the crossfade sweeps between them.
#[cfg(feature = "sdcard")]
const MIX_SINE_HZ: u32 = 660;
/// Full Q8 gain (unity): a source at gain 256 contributes its whole centered
/// amplitude. The `>> 8` in the mix undoes this scale.
#[cfg(feature = "sdcard")]
const MIX_GAIN_FULL: i32 = 256;
/// Q8 fixed-point shift for the mix gains (256 = 1.0).
#[cfg(feature = "sdcard")]
const MIX_GAIN_SHIFT: i32 = 8;
/// Gain ramp step per sample, in Q8 units. One step per sample means a full
/// 0..=256 crossfade takes 256 samples (~12 ms at 22050 Hz) -- smooth and
/// click-free.
#[cfg(feature = "sdcard")]
const MIX_RAMP_STEP: i32 = 1;
/// Crossfade target selector: source A = the WAV file.
#[cfg(feature = "sdcard")]
const MIX_TARGET_WAV: u8 = 0;
/// Crossfade target selector: source B = the fixed sine.
#[cfg(feature = "sdcard")]
const MIX_TARGET_SINE: u8 = 1;
/// Seengreat push buttons: GP20 selects the WAV, GP21 selects the sine. Each
/// wires to GND, so with a pull-up the pin idles 1 (released) and reads 0 while
/// pressed (active-low) -- same wiring the shell's `buttons` command uses.
#[cfg(feature = "sdcard")]
const MIX_BTN_WAV: u8 = 20;
#[cfg(feature = "sdcard")]
const MIX_BTN_SINE: u8 = 21;

// --- Real-time mic -> jack passthrough (ADC free-run -> DMA -> PWM CC) ---
//
// The ADC free-runs on the mic channel (round-robin off, single channel),
// pushing each 12-bit sample into its FIFO with a DREQ. DMA channel 0 is armed
// to move one HALFWORD (u16) per DREQ from the fixed ADC FIFO register straight
// into the slice-1 channel-A compare (GP18 = jack left). No ring, no CPU: the
// ADC paces the DMA and the DMA paces the PWM duty. MONO -- only channel A is
// fed; channel B (GP19 = right) stays parked at mid-scale and is silent.
//
// Carrier: slice 1 with DIV_INT=1, TOP=4095. The sample is 12-bit (0..=4095),
// so a raw sample maps DIRECTLY and linearly to duty -- 2048 (the mic's mid-rail
// DC bias) = 50% duty = silence, and the AC swing rides around it with no clip.
// (TOP=1023 would clamp every sample >1023 to full duty and rail the output.)
// Carrier rate = 150e6 / (1 * 4096) = 36.6 kHz, ultrasonic.

/// ADC channel for the Seengreat mic: channel 2 = GP28.
const MIC_ADC_CH: u8 = 2;
/// ADC clock divider (DIV.INT). clk_adc = 48 MHz; sample period = 96 * (1 + INT)
/// cycles, so INT=11 -> 96 * 12 = 1152 cycles -> ~41.7 kS/s.
const MIC_DIV_INT: u16 = 11;
/// DMA TREQ (dreq) select for the ADC FIFO (DREQ_ADC = 48): pace one transfer
/// per ADC sample.
const MIC_TREQ_ADC: u8 = 48;
/// Counter wrap for the passthrough carrier: 12-bit, so a raw 0..=4095 ADC
/// sample IS the channel-A compare value with no scaling.
const MIC_CARRIER_TOP: u16 = 4095;
/// Integer clock divider for the passthrough carrier: 150 MHz / (1 * 4096) =
/// 36.6 kHz PWM carrier, ultrasonic.
const MIC_CARRIER_DIV: u8 = 1;
/// DMA CTRL data size: 1 = halfword (u16). One 12-bit ADC sample per transfer
/// into channel A (low half of the CC register).
const MIC_DATA_SIZE_HALFWORD: u8 = 1;
/// ADC FIFO threshold: raise a DREQ once a single sample is present.
const MIC_FIFO_THRESH: u8 = 1;

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

/// Pack the player diagnostics into the u32 IPC reply so the shell can report
/// them without an extra op: rate in bits 0..15 (<= 48000 fits), underruns in
/// 16..22, a truncated (SD read error) flag in bit 23, refills in 24..31.
/// underruns/refills saturate. The truncated flag is what keeps a faulted read
/// from being reported as a clean, complete playback.
#[cfg(feature = "sdcard")]
fn pack_play_reply(
    rate: u32,
    underruns: u32,
    refills: u32,
    truncated: u32,
) -> u32 {
    (refills.min(255) << 24)
        | ((truncated & 1) << 23)
        | (underruns.min(127) << 16)
        | (rate & 0xffff)
}

/// Running state of the 2-source mixer, carried across `mix_fill` calls so the
/// sine phase and gain ramps are continuous over the whole track (per-half
/// refills must not reset either, or the sine would restart and the gains would
/// step). `ga`/`gb` are the live Q8 gains; `target` is the button-selected
/// source the gains ramp toward.
#[cfg(feature = "sdcard")]
struct MixState {
    /// DDS phase accumulator for the fixed sine (source B); the top 8 bits index
    /// SINE_LUT. Advances every sample regardless of gb, so a fade-in of the
    /// sine has no phase discontinuity.
    phase: u32,
    /// Q32 phase increment per sample for MIX_SINE_HZ at the file's rate.
    sine_inc: u32,
    /// Live Q8 gain of source A (the WAV), 0..=MIX_GAIN_FULL.
    ga: i32,
    /// Live Q8 gain of source B (the sine), 0..=MIX_GAIN_FULL.
    gb: i32,
    /// Crossfade target: MIX_TARGET_WAV or MIX_TARGET_SINE. The gains ramp one
    /// MIX_RAMP_STEP per sample toward (256, 0) or (0, 256) accordingly.
    target: u8,
}

struct ServerImpl {
    pwm: rp235x_pac::PWM,
    dma: rp235x_pac::DMA,
    adc: rp235x_pac::ADC,
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

    /// Program slice 1's 8.4 fixed-point divider so one sample plays per wrap at
    /// `rate` Hz: `rate = clk_sys / (div * (PLAY_TOP + 1))`, so `div = clk_sys /
    /// (rate * (PLAY_TOP + 1))`. Computed in Q4 (times 16) to fill both the
    /// 8-bit integer and 4-bit fractional fields, then clamped to the divider's
    /// range (1.0 ..= 255.9375). Also sets TOP and parks both channels at
    /// mid-scale, ready for the DMA's first sample.
    #[cfg(feature = "sdcard")]
    fn program_player_slice(&self, rate: u32) {
        let period = (PLAY_TOP as u32 + 1) as u64; // counts per wrap
        // div * 16, rounded: (clk_sys * 16 + half) / (rate * period).
        let denom = rate as u64 * period;
        let div_q4 =
            ((CLK_SYS_HZ as u64 * 16 + denom / 2) / denom).clamp(16, 0xfff);
        let div_int = (div_q4 >> 4) as u8;
        let div_frac = (div_q4 & 0xf) as u8;

        let ch = self.pwm.ch(AUDIO_SLICE);
        ch.div().write(|w| unsafe {
            w.int().bits(div_int);
            w.frac().bits(div_frac);
            w
        });
        ch.top().write(|w| unsafe { w.top().bits(PLAY_TOP) });
        // Park both channels at the SAME mid-scale the running samples use
        // (PLAY_MID) so the pre-DMA silence value cannot drift from the silence
        // value fill_half/mix_fill write.
        ch.cc().modify(|_, w| unsafe {
            w.a().bits(PLAY_MID as u16);
            w.b().bits(PLAY_MID as u16);
            w
        });
    }

    /// Map a signed 16-bit PCM sample to a 10-bit unsigned duty around mid-scale:
    /// shift to 10-bit, bias to PLAY_MID, clamp to 0..=PLAY_TOP.
    #[cfg(feature = "sdcard")]
    fn sample_to_duty(sample: i16) -> u32 {
        (((sample as i32) >> PLAY_SAMPLE_SHIFT) + PLAY_MID)
            .clamp(0, PLAY_TOP as i32) as u32
    }

    /// Pack a stereo L/R pair into the u32 CC value: channel A (low 16 bits) =
    /// left = GP18, channel B (high 16 bits) = right = GP19.
    #[cfg(feature = "sdcard")]
    fn stereo_to_cc(l: i16, r: i16) -> u32 {
        (Self::sample_to_duty(r) << CH_B_SHIFT) | Self::sample_to_duty(l)
    }

    /// Fill one ring half (`half` = 0 = lower slots, 1 = upper slots) with
    /// samples pulled from `dec`. The decoder yields INTERLEAVED L, R pairs (2
    /// i16 per ring slot); slots past what it yields are filled with mid-scale
    /// silence. Returns the i16 count written (0 once the decoder is exhausted).
    /// Writes are volatile through the aligned ring pointer (the DMA is a bus
    /// master reading the same window).
    #[cfg(feature = "sdcard")]
    fn fill_half<D: decoder::Decoder>(
        &self,
        dec: &mut D,
        half: usize,
    ) -> usize {
        let ring = aligned_ring_ptr();
        let base = half * PLAY_HALF_SAMPLES;
        // 2 i16 (L, R) per ring slot.
        let mut pcm = [0i16; PLAY_HALF_SAMPLES * 2];
        let got = dec.next_pcm(&mut pcm);
        // Samples past `got` play mid-scale silence (the decoder is exhausted).
        for slot in pcm.iter_mut().skip(got) {
            *slot = 0;
        }
        for i in 0..PLAY_HALF_SAMPLES {
            let l = pcm[2 * i];
            let r = pcm[2 * i + 1];
            // SAFETY: `ring` is the 4096-aligned window inside AUDIO_STORE (2x
            // the ring), so base + i (< AUDIO_BUF_SAMPLES) is in bounds. Single-
            // threaded server, no references taken.
            unsafe {
                ring.add(base + i).write_volatile(Self::stereo_to_cc(l, r));
            }
        }
        got
    }

    /// Arm DMA channel 0 to stream the ring into the slice-1 CC register, one
    /// word per PWM_WRAP1 dreq -- identical configuration to `arm_audio` (same
    /// ring size, TREQ, trans count, read = ring base, write = CC). The ring is
    /// expected to be primed (both halves filled) before this is called.
    #[cfg(feature = "sdcard")]
    fn arm_player_dma(&self) {
        let read_addr = aligned_ring_ptr() as u32;
        let write_addr = self.pwm.ch(AUDIO_SLICE).cc().as_ptr() as u32;
        let ch = self.dma.ch(AUDIO_DMA_CH);
        ch.ch_read_addr().write(|w| unsafe { w.bits(read_addr) });
        ch.ch_write_addr().write(|w| unsafe { w.bits(write_addr) });
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

    /// Which ring half the DMA is currently reading (0 = lower, 1 = upper),
    /// derived from the channel's live read address relative to the ring base
    /// and wrapped to the 4096-byte window.
    #[cfg(feature = "sdcard")]
    fn dma_half(&self) -> usize {
        let base = aligned_ring_ptr() as u32;
        let addr = self.dma.ch(AUDIO_DMA_CH).ch_read_addr().read().bits();
        let off = addr.wrapping_sub(base) & (AUDIO_RING_BYTES as u32 - 1);
        (off / PLAY_HALF_BYTES) as usize
    }

    /// Spin until the DMA read pointer crosses `laps` ring-half boundaries, then
    /// return. Bails early if no crossing happens for PLAY_STALL_MS (the channel
    /// wedged) so a DMA fault cannot hang the server here. Shared by the player
    /// and mixer EOF drain (both let the silence tail play out one lap = 2
    /// crossings before stopping).
    #[cfg(feature = "sdcard")]
    fn drain_crossings(&self, laps: u32) {
        let mut crossings = 0u32;
        let mut prev = self.dma_half();
        let mut last_progress = sys_get_timer().now;
        let mut polls = 0u32;
        while crossings < laps {
            let now = self.dma_half();
            if now != prev {
                crossings += 1;
                prev = now;
                last_progress = sys_get_timer().now;
            }
            polls = polls.wrapping_add(1);
            if polls & PLAY_STALL_POLL_MASK == 0
                && sys_get_timer().now.wrapping_sub(last_progress)
                    > PLAY_STALL_MS
            {
                break;
            }
        }
    }

    /// Blocking play loop: prime both halves, arm the DMA, then repeatedly
    /// refill whichever half the DMA is NOT currently reading, pulling PCM from
    /// `dec`, until the decoder is exhausted. After EOF, let the DMA lap once
    /// more so the silence-filled tail plays out, then abort the channel and
    /// stop the carrier. Blocks the pwm server for the whole track.
    #[cfg(feature = "sdcard")]
    fn play_loop<D: decoder::Decoder>(&self, dec: &mut D) -> (u32, u32) {
        // Prime BOTH halves so the DMA has a full ring before it starts.
        let mut eof = self.fill_half(dec, 0) == 0;
        eof &= self.fill_half(dec, 1) == 0;

        self.enable_slice(AUDIO_SLICE, true);
        self.arm_player_dma();

        // Instrumentation for the refill-vs-DMA race: `refills` counts how many
        // half-refills we did; `underruns` counts refills that lost the race --
        // i.e. the DMA had ALREADY advanced into the half we just wrote by the
        // time the (blocking SD read + convert) finished. underruns == 0 means
        // the refill always stayed a full half ahead of the read pointer.
        let mut refills = 0u32;
        let mut underruns = 0u32;

        // Refill the far half whenever the DMA crosses into a new half. Track
        // the last half we refilled so a single visit does not refill twice
        // (the read address dwells in one half for PLAY_HALF_SAMPLES wraps).
        //
        // Seed `last_filled` to the CURRENT far half so the first refill waits
        // for a real crossing: both halves were just primed with fresh file
        // data, so the far half must NOT be clobbered until the DMA actually
        // leaves the near half and starts draining it. At arm time the DMA sits
        // at the ring base (near half 0), so far = 1 = the just-primed upper
        // half -- refilling it now would drop the second primed chunk unplayed.
        let mut last_filled = self.dma_half() ^ 1;
        let mut last_progress = sys_get_timer().now;
        let mut polls = 0u32;
        while !eof {
            let live = self.dma_half();
            let far = live ^ 1;
            if far != last_filled {
                eof = self.fill_half(dec, far) == 0;
                last_filled = far;
                refills += 1;
                // If the DMA is now reading `far`, our refill did not finish
                // before the read pointer crossed into it -> margin exhausted.
                if self.dma_half() == far {
                    underruns += 1;
                }
                last_progress = sys_get_timer().now;
            }
            // Stall escape: if the DMA read pointer stops crossing halves (a
            // wedged channel), bail rather than spinning forever. Checked only
            // every PLAY_STALL_POLL_MASK+1 polls so the kipc stays off the hot
            // path.
            polls = polls.wrapping_add(1);
            if polls & PLAY_STALL_POLL_MASK == 0
                && sys_get_timer().now.wrapping_sub(last_progress)
                    > PLAY_STALL_MS
            {
                break;
            }
        }

        // EOF: the last real samples plus the silence tail are in the ring.
        // Let the DMA drain one more full lap so nothing is cut off, then stop.
        // Two half-crossings = one lap past the point EOF was detected.
        self.drain_crossings(2);

        self.abort_audio_dma();
        self.dma
            .ch(AUDIO_DMA_CH)
            .ch_ctrl_trig()
            .write(|w| w.en().bit(false));
        self.enable_slice(AUDIO_SLICE, false);
        (underruns, refills)
    }

    /// Fill one ring half by BLENDING the WAV (source A) with the fixed sine
    /// (source B), advancing the sine phase and ramping the gains one Q8 step
    /// per sample toward `state.target`. Mirrors `fill_half` (mid-scale silence
    /// past decoder EOF, volatile writes through the aligned ring) but composes
    /// two centered sources instead of copying one. Returns the number of real
    /// WAV samples written (0 once the decoder is exhausted).
    #[cfg(feature = "sdcard")]
    fn mix_fill<D: decoder::Decoder>(
        &self,
        dec: &mut D,
        half: usize,
        state: &mut MixState,
    ) -> usize {
        let ring = aligned_ring_ptr();
        let base = half * PLAY_HALF_SAMPLES;
        // 2 i16 (L, R) per ring slot -- source A is stereo.
        let mut pcm = [0i16; PLAY_HALF_SAMPLES * 2];
        let got = dec.next_pcm(&mut pcm);
        // Samples past `got` are silence (0 -> mid-scale for source A).
        for slot in pcm.iter_mut().skip(got) {
            *slot = 0;
        }
        let lut_len = SINE_LUT.len() as u32;
        for i in 0..PLAY_HALF_SAMPLES {
            // Source A: the WAV L/R pair, each -> 10-bit duty centered on mid.
            let wav_l = Self::sample_to_duty(pcm[2 * i]) as i32 - PLAY_MID;
            let wav_r = Self::sample_to_duty(pcm[2 * i + 1]) as i32 - PLAY_MID;

            // Source B: the fixed sine over the LUT, centered around mid (mono,
            // so applied to both channels). Advance the phase once per output
            // frame so gb can fade in with no restart transient.
            let idx = (state.phase >> SINE_LUT_INDEX_SHIFT) % lut_len;
            let sine_centered = SINE_LUT[idx as usize] as i32 - PLAY_MID;
            state.phase = state.phase.wrapping_add(state.sine_inc);

            // Ramp the gains one step per output frame toward the target.
            // TODO equal-power: this is a LINEAR crossfade; multiply by sqrt
            // curves if the midpoint dip proves audible.
            let (ta, tb) = if state.target == MIX_TARGET_SINE {
                (0, MIX_GAIN_FULL)
            } else {
                (MIX_GAIN_FULL, 0)
            };
            state.ga = ramp_toward(state.ga, ta);
            state.gb = ramp_toward(state.gb, tb);

            // Blend each channel (WAV L or R) with the shared sine in the
            // centered domain, rebias to mid, clamp, and pack L->A / R->B.
            let mixed_l = state.ga * wav_l + state.gb * sine_centered;
            let mixed_r = state.ga * wav_r + state.gb * sine_centered;
            let duty_l = (PLAY_MID + (mixed_l >> MIX_GAIN_SHIFT))
                .clamp(0, PLAY_TOP as i32) as u32;
            let duty_r = (PLAY_MID + (mixed_r >> MIX_GAIN_SHIFT))
                .clamp(0, PLAY_TOP as i32) as u32;
            let pair = (duty_r << CH_B_SHIFT) | duty_l;
            // SAFETY: `ring` is the 4096-aligned window inside AUDIO_STORE (2x
            // the ring), so base + i (< AUDIO_BUF_SAMPLES) is in bounds. Single-
            // threaded server, no references taken.
            unsafe {
                ring.add(base + i).write_volatile(pair);
            }
        }
        got
    }

    /// Blocking mix loop: the `play_loop` structure with `fill_half` swapped for
    /// `mix_fill` and a once-per-refill button read that steers the crossfade.
    /// Reads GP20/GP21 (already configured input + pull-up by the op) once per
    /// half-refill -- ~129 reads over a 3 s track, negligible next to the SD
    /// read. On a fresh press edge (active-low, 0 = pressed) GP20 targets the
    /// WAV, GP21 targets the sine. Returns packed (underruns, refills).
    #[cfg(feature = "sdcard")]
    fn mix_loop<D: decoder::Decoder>(
        &self,
        dec: &mut D,
        gpio: &Rp235xGpio,
        state: &mut MixState,
    ) -> (u32, u32) {
        // Prime BOTH halves so the DMA has a full ring before it starts.
        let mut eof = self.mix_fill(dec, 0, state) == 0;
        eof &= self.mix_fill(dec, 1, state) == 0;

        self.enable_slice(AUDIO_SLICE, true);
        self.arm_player_dma();

        let mut refills = 0u32;
        let mut underruns = 0u32;

        // Button edge state: 1 = released (idle with pull-up), 0 = pressed.
        let mut last_wav = 1u8;
        let mut last_sine = 1u8;

        // Same refill-crossing scheme as play_loop; seed last_filled to the far
        // half so the first refill waits for a real DMA crossing (both halves
        // were just primed with mixed data).
        let mut last_filled = self.dma_half() ^ 1;
        let mut last_progress = sys_get_timer().now;
        let mut polls = 0u32;
        while !eof {
            let live = self.dma_half();
            let far = live ^ 1;
            if far != last_filled {
                // Read the buttons ONCE per refill (not per poll). Active-low;
                // act only on the release->press edge, like the shell's watch.
                let wav = gpio.read(MIX_BTN_WAV).unwrap_or(1);
                let sine = gpio.read(MIX_BTN_SINE).unwrap_or(1);
                if wav == 0 && last_wav == 1 {
                    state.target = MIX_TARGET_WAV;
                }
                if sine == 0 && last_sine == 1 {
                    state.target = MIX_TARGET_SINE;
                }
                last_wav = wav;
                last_sine = sine;

                eof = self.mix_fill(dec, far, state) == 0;
                last_filled = far;
                refills += 1;
                if self.dma_half() == far {
                    underruns += 1;
                }
                last_progress = sys_get_timer().now;
            }
            // Stall escape, same as play_loop: bail if the DMA stops crossing
            // halves, checked off the hot path.
            polls = polls.wrapping_add(1);
            if polls & PLAY_STALL_POLL_MASK == 0
                && sys_get_timer().now.wrapping_sub(last_progress)
                    > PLAY_STALL_MS
            {
                break;
            }
        }

        // Drain one more lap so the silence tail plays out, then stop.
        self.drain_crossings(2);

        self.abort_audio_dma();
        self.dma
            .ch(AUDIO_DMA_CH)
            .ch_ctrl_trig()
            .write(|w| w.en().bit(false));
        self.enable_slice(AUDIO_SLICE, false);
        (underruns, refills)
    }
}

/// Move `gain` one MIX_RAMP_STEP toward `target`, without overshooting. Q8
/// units; used per-sample by the mixer so a source crossfades in/out over
/// MIX_GAIN_FULL samples with no step (zipper) artifact.
#[cfg(feature = "sdcard")]
fn ramp_toward(gain: i32, target: i32) -> i32 {
    if gain < target {
        (gain + MIX_RAMP_STEP).min(target)
    } else if gain > target {
        (gain - MIX_RAMP_STEP).max(target)
    } else {
        gain
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

    fn audio_mic_start(
        &mut self,
        _: &RecvMessage,
    ) -> Result<(), RequestError<PwmError>> {
        // Abort + drain any prior sine/audio DMA on ch0 before re-arming it for
        // the passthrough (reconfiguring a BUSY channel is undefined on HW).
        self.abort_audio_dma();

        // Free-run the ADC on the mic channel. Per the datasheet the ADC must be
        // enabled and started BEFORE the DMA is armed. Order: divider, then the
        // FIFO (enable + DREQ + a 1-sample threshold), then CS (enable the
        // converter, select the channel, and kick off continuous conversions).
        self.adc
            .div()
            .write(|w| unsafe { w.int().bits(MIC_DIV_INT) });
        self.adc.fcs().write(|w| unsafe {
            w.en().bit(true);
            w.dreq_en().bit(true);
            w.thresh().bits(MIC_FIFO_THRESH);
            w
        });
        // MODIFY (not write): keep the bits the adc_driver set at boot -- notably
        // ts_en (temperature-sensor bias) and EN -- while selecting the mic
        // channel and kicking off continuous conversions.
        self.adc.cs().modify(|_, w| unsafe {
            w.en().bit(true);
            w.ainsel().bits(MIC_ADC_CH);
            w.start_many().bit(true);
            w
        });

        // Program slice 1 as the 12-bit carrier and park BOTH channels at
        // mid-scale (silence) until the first sample lands. Only channel A
        // (GP18/left) is DMA-fed; channel B (GP19/right) stays parked = silent.
        self.program_slice(AUDIO_SLICE, MIC_CARRIER_DIV, MIC_CARRIER_TOP);
        self.pwm.ch(AUDIO_SLICE).cc().modify(|_, w| unsafe {
            w.a().bits(MIC_CARRIER_TOP / 2);
            w.b().bits(MIC_CARRIER_TOP / 2);
            w
        });
        self.enable_slice(AUDIO_SLICE, true);

        // Arm DMA channel 0 to stream the ADC FIFO into the slice-1 channel-A
        // compare, one HALFWORD (12-bit sample) per ADC DREQ. Read is the fixed
        // FIFO register (no increment, no ring -- it is a streaming FIFO, not a
        // buffer); write is the fixed CC register. A large finite reload runs for
        // hours until audio_mic_stop aborts. Requires SECCFG_CH0.P cleared in the
        // pre-kernel main (already done for the audio path this shares).
        let read_addr = self.adc.fifo().as_ptr() as u32;
        let write_addr = self.pwm.ch(AUDIO_SLICE).cc().as_ptr() as u32;
        let ch = self.dma.ch(AUDIO_DMA_CH);
        ch.ch_read_addr().write(|w| unsafe { w.bits(read_addr) });
        ch.ch_write_addr().write(|w| unsafe { w.bits(write_addr) });
        ch.ch_trans_count()
            .write(|w| unsafe { w.count().bits(AUDIO_TRANS_COUNT) });
        ch.ch_ctrl_trig().write(|w| unsafe {
            w.en().bit(true);
            w.data_size().bits(MIC_DATA_SIZE_HALFWORD);
            w.incr_read().bit(false);
            w.incr_write().bit(false);
            w.ring_size().bits(0); // no ring: streaming from a FIFO
            w.ring_sel().bit(false);
            w.treq_sel().bits(MIC_TREQ_ADC);
            w.chain_to().bits(AUDIO_DMA_CH as u8); // chain to self = no chain
            w.high_priority().bit(false);
            w
        });
        Ok(())
    }

    fn audio_mic_stop(
        &mut self,
        _: &RecvMessage,
    ) -> Result<(), RequestError<PwmError>> {
        // Abort + DRAIN the DMA channel, clear its EN, stop the ADC free-run, and
        // silence the carrier. CRITICAL: only clear START_MANY -- leave the ADC
        // ENABLED (and ts_en set). The adc_driver task enables the ADC once at
        // boot and never re-enables it, so disabling it here would hang the next
        // adc read/temp/mic (the driver spins on READY forever). Also stop the
        // FIFO so it does not keep filling.
        self.abort_audio_dma();
        self.dma
            .ch(AUDIO_DMA_CH)
            .ch_ctrl_trig()
            .write(|w| w.en().bit(false));
        self.adc.cs().modify(|_, w| w.start_many().bit(false));
        self.adc
            .fcs()
            .modify(|_, w| w.en().bit(false).dreq_en().bit(false));
        self.enable_slice(AUDIO_SLICE, false);
        Ok(())
    }

    #[cfg(not(feature = "sdcard"))]
    fn play_file(
        &mut self,
        _: &RecvMessage,
        _name: LenLimit<Leased<R, [u8]>, 12>,
    ) -> Result<u32, RequestError<PwmError>> {
        // No SD card in this build: the player is unavailable.
        Err(PwmError::OpenFailed.into())
    }

    // `inline(never)`: the MP3 decoder holds ~20 KiB of buffers on the stack for
    // the whole play loop. Inlining this (and play_mix) into `main`/`dispatch`
    // would sum both decoders' frames into one; keeping each out-of-line means
    // only the active op's frame is live at a time.
    #[cfg(feature = "sdcard")]
    #[inline(never)]
    fn play_file(
        &mut self,
        _: &RecvMessage,
        name: LenLimit<Leased<R, [u8]>, 12>,
    ) -> Result<u32, RequestError<PwmError>> {
        // Copy the 8.3 short name out of the lease into a local buffer.
        let n = name.len().min(12);
        let mut name_buf = [0u8; 12];
        name.read_range(0..n, &mut name_buf[..n])
            .map_err(|()| RequestError::went_away())?;

        // Open the file and parse the WAV header. Distinct errors so the shell
        // can tell "no such file" from "not a WAV".
        let source = sd::SdFileSource::open(
            Rp235xSdcard::from(SDCARD.get_task_id()),
            &name_buf[..n],
        )
        .ok_or(PwmError::OpenFailed)?;
        // Route by file extension: ".mp3" -> nanomp3, everything else -> WAV.
        // Both wrap in one enum so the generic play loop is unchanged.
        let mut dec = if decoder::is_mp3_name(&name_buf[..n]) {
            decoder::AnyDecoder::Mp3(
                decoder::Nanomp3Decoder::new(source)
                    .map_err(|_| PwmError::BadWav)?,
            )
        } else {
            decoder::AnyDecoder::Wav(
                decoder::WavDecoder::new(source)
                    .map_err(|_| PwmError::BadWav)?,
            )
        };

        // Clamp the file rate to the player's supported range and program the
        // slice divider to it. The DMA path (ch0) is shared with the other audio
        // ops, so abort + drain it first, exactly like they do.
        let rate = decoder::Decoder::sample_rate(&dec)
            .clamp(PLAY_MIN_RATE, PLAY_MAX_RATE);
        self.abort_audio_dma();
        self.program_player_slice(rate);

        // Blocking: streams the whole track, then stops the DMA + carrier.
        let (underruns, refills) = self.play_loop(&mut dec);
        let truncated = decoder::Decoder::had_error(&dec) as u32;
        Ok(pack_play_reply(rate, underruns, refills, truncated))
    }

    #[cfg(not(feature = "sdcard"))]
    fn play_mix(
        &mut self,
        _: &RecvMessage,
        _name: LenLimit<Leased<R, [u8]>, 12>,
    ) -> Result<u32, RequestError<PwmError>> {
        // No SD card in this build: the mixer is unavailable.
        Err(PwmError::OpenFailed.into())
    }

    // See play_file: kept out-of-line to bound the pwm task stack.
    #[cfg(feature = "sdcard")]
    #[inline(never)]
    fn play_mix(
        &mut self,
        _: &RecvMessage,
        name: LenLimit<Leased<R, [u8]>, 12>,
    ) -> Result<u32, RequestError<PwmError>> {
        // Copy the 8.3 short name out of the lease into a local buffer.
        let n = name.len().min(12);
        let mut name_buf = [0u8; 12];
        name.read_range(0..n, &mut name_buf[..n])
            .map_err(|()| RequestError::went_away())?;

        // Open the file and parse the WAV header (same as play_file).
        let source = sd::SdFileSource::open(
            Rp235xSdcard::from(SDCARD.get_task_id()),
            &name_buf[..n],
        )
        .ok_or(PwmError::OpenFailed)?;
        // Same extension routing as play_file: the mixer accepts an .mp3 as
        // source A too (WAV stays the default for any other extension).
        let mut dec = if decoder::is_mp3_name(&name_buf[..n]) {
            decoder::AnyDecoder::Mp3(
                decoder::Nanomp3Decoder::new(source)
                    .map_err(|_| PwmError::BadWav)?,
            )
        } else {
            decoder::AnyDecoder::Wav(
                decoder::WavDecoder::new(source)
                    .map_err(|_| PwmError::BadWav)?,
            )
        };

        // Clamp the file rate and program the slice divider to it.
        let rate = decoder::Decoder::sample_rate(&dec)
            .clamp(PLAY_MIN_RATE, PLAY_MAX_RATE);
        self.abort_audio_dma();
        self.program_player_slice(rate);

        // Build the gpio client from the task slot and configure the two mixer
        // buttons as inputs with pull-ups (idle high, active-low when pressed).
        let gpio = Rp235xGpio::from(GPIO.get_task_id());
        let _ = gpio.configure_input(MIX_BTN_WAV);
        let _ = gpio.set_pull(MIX_BTN_WAV, drv_rp235x_gpio_api::PULL_UP);
        let _ = gpio.configure_input(MIX_BTN_SINE);
        let _ = gpio.set_pull(MIX_BTN_SINE, drv_rp235x_gpio_api::PULL_UP);

        // Start on the WAV only (ga = full, gb = 0). The sine phase increment is
        // for the CLAMPED rate so the mix and the DMA agree on the sample rate.
        let mut state = MixState {
            phase: 0,
            sine_inc: (((MIX_SINE_HZ as u64) << 32) / rate as u64) as u32,
            ga: MIX_GAIN_FULL,
            gb: 0,
            target: MIX_TARGET_WAV,
        };

        // Blocking: streams the whole track (mixing + reading the buttons per
        // refill), then stops the DMA + carrier. Same packed reply as play_file.
        let (underruns, refills) = self.mix_loop(&mut dec, &gpio, &mut state);
        let truncated = decoder::Decoder::had_error(&dec) as u32;
        Ok(pack_play_reply(rate, underruns, refills, truncated))
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
        adc: p.ADC,
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
