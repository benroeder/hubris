// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! RP2350 (RP235x) I2S audio driver for the PCM5102A DAC -- Stage 1 (test tone).
//!
//! 3-wire I2S via PIO0 SM0 (SB Components MusicPi: PCM5100A + PAM8908 amp):
//! BCK (GP10) + LRCK (GP11) are the side-set pair, DIN (GP9) is the OUT pin,
//! XSMT (GP22) is driven high to un-mute, amp gain selects on GP20/GP21. 16-bit stereo, one 32-bit FIFO word per frame packed
//! `(right << 16) | left`, BCK = 32*fs (the canonical pico-extras audio_i2s
//! format); the DAC's SCK pin is grounded so its internal PLL derives the
//! master clock from BCK. See docs/pcm5102a-research/.
//!
//! REUSE: the output engine is the pwm driver's DMA ring loop (drv/rp235x-pwm
//! `audio_start`) retargeted from the PWM compare register to the PIO TX FIFO --
//! a ring of 32-bit words is streamed to the FIFO, one word per PIO-TX DREQ,
//! looping with zero CPU. The only genuinely new part vs the PWM path is the PIO
//! I2S program + the sample packing (i16 -> u16<<16; no PlayQuant dither, the
//! DAC does the real conversion).
//!
//! Stage 1 arms a fixed square-wave tone at boot and serves the Idol API so the
//! shell can retune/stop it. The SD/decode path (reusing lib/rp235x-audio-decode
//! and the pwm `sd.rs`) is Stage 2.

#![no_std]
#![no_main]

use core::ptr::addr_of_mut;
use drv_rp235x_i2s_api::I2sError;
use idol_runtime::RequestError;
use userlib::sys_get_timer;
use userlib::RecvMessage;
#[cfg(feature = "sdcard")]
use idol_runtime::{Leased, LenLimit, R};
#[cfg(feature = "sdcard")]
use userlib::task_slot;

#[cfg(feature = "sdcard")]
use drv_rp235x_sdcard_api::Rp235xSdcard;
#[cfg(feature = "sdcard")]
use rp235x_audio_decode as decoder;

#[cfg(feature = "sdcard")]
mod sd;

#[cfg(feature = "sdcard")]
task_slot!(SDCARD, sdcard);

// ---- Pins (Pico 2 GPIO numbers) -- SB Components MusicPi HAT ----------------
// PCM5100A DAC (same PCM510xA family/datasheet) + PAM8908 headphone amp.
/// DIN (data) -- the PIO OUT pin.
const DIN: u32 = 9;
/// BCK (bit clock) -- PIO side-set bit 0 (sideset_base).
const BCK: u32 = 10;
/// LRCK (word clock) -- PIO side-set bit 1 (must be BCK+1).
const LRCK: u32 = 11;
/// XSMT (DAC soft mute, active low) AND the PAM8908 headphone-amp EN -- one
/// net, 10K pull-up to 3V3 (schematic sheet 1). Driven HIGH = unmuted+enabled.
const XSMT: u32 = 22;
// NOTE: GAIN_SEL0/1 (GP20/GP21) are driven by the on-board DIP switches to
// VBUS (5V!) or GND (schematic "MODE SEL") -- NEVER drive them as outputs; a
// GPIO fighting a closed switch to VBUS is contention on a 5V rail.
/// funcsel that routes a GPIO to PIO0 on the RP2350 (PIO0=6, PIO1=7, PIO2=8).
const FUNCSEL_PIO0: u8 = 6;

// ---- Clocking ---------------------------------------------------------------
/// System clock (matches lib/rp235x-startup SYS_CLK_HZ).
const SYS_CLK_HZ: u32 = 150_000_000;
/// PIO cycles per audio frame: 2 PIO cycles per BCK * 32 BCK/frame (16-bit L +
/// 16-bit R) = 64. So the PIO clock is 64 * fs.
const PIO_CYCLES_PER_FRAME: u32 = 64;
/// Default sample rate for the boot tone. 48 kHz divides the 150 MHz clock
/// exactly (150e6 / 6.144e6 = 24 + 106/256).
const DEFAULT_RATE_HZ: u32 = 48_000;

// ---- Tone -------------------------------------------------------------------
/// Boot-tone amplitude (i16), ~-12 dBFS -- audible but not blasting.
const TONE_AMPL: i16 = 0x2000;
/// Boot-tone frequency (Hz), snapped to whole cycles across the ring.
const DEFAULT_TONE_HZ: u32 = 440;
const TONE_MIN_HZ: u32 = 50;
const TONE_MAX_HZ: u32 = 8000;
/// Accepted PLL sample rates. Datasheet Table 11: at BCK = 32*fs the DAC's PLL
/// locks for fs >= 32 kHz (16 kHz would need 64*fs), so the floor is 32 kHz.
const RATE_MIN_HZ: u32 = 32_000;
const RATE_MAX_HZ: u32 = 48_000;

// ---- DMA / ring -------------------------------------------------------------
/// DMA channel. BRING-UP: temporarily ch0 -- its SECCFG clear is proven by the
/// pwm audio path, isolating a "DMA armed but never transfers" fault to the
/// TREQ index vs the (new, unproven) SECCFG_CH1 clear. ch0 collides with pwm
/// audio commands, so move back to ch1 once the root cause is pinned.
const DMA_CH: usize = 0;
/// TREQ (DREQ) select for the PIO0 SM0 TX FIFO. RP2350 DREQ index 0 = PIO0_TX0.
/// (If the tone is silent/garbled, this is the first thing to re-check.)
const TREQ_PIO0_TX0: u8 = 0;
/// DMA transfer size: 32-bit word.
const DATA_SIZE_WORD: u8 = 2;
/// Ring wrap: 2^14 = 16384 bytes = 4096 u32 = the whole ring -- the same size
/// as the pwm player's, banking ~93 ms of slack at 44.1 kHz against the FLAC
/// decoder's bursty whole-block prefetch.
const RING_SIZE_BITS: u8 = 14;
/// Large finite reload -- loops for hours until aborted (ENDLESS/COUNT=0 does
/// not arm on this HW; see the pwm driver note).
const TRANS_COUNT: u32 = 0x0fff_ffff;

/// Frames in the ring. Each frame is ONE 32-bit word, `(right << 16) | left`
/// (the same stereo packing as the pwm driver's ring), so the ring is 4096
/// words = 16 KiB (matches RING_SIZE_BITS).
const RING_FRAMES: usize = 4096;
const RING_WORDS: usize = RING_FRAMES;

// ---- SD player (port of the pwm driver's play engine, minus PlayQuant:
// samples go to the DAC as-is) ------------------------------------------------
/// Ring chunks the producer refills behind the DMA read pointer.
const PLAY_CHUNKS: usize = 8;
const PLAY_CHUNK_FRAMES: usize = RING_FRAMES / PLAY_CHUNKS;
const PLAY_CHUNK_BYTES: u32 = (RING_BYTES / PLAY_CHUNKS) as u32;
/// Refill-stall escape: if the DMA read pointer stops moving this long, bail.
const PLAY_STALL_MS: u64 = 500;
const PLAY_STALL_POLL_MASK: u32 = 0x3ffff;
const RING_BYTES: usize = RING_WORDS * 4;

/// Backing store, 2x the ring so a 16 KiB window can be found 4096-byte aligned
/// (the DMA ring-wrap requires the base aligned to the ring size).
const STORE_WORDS: usize = RING_WORDS * 2;
/// SAFETY: single-threaded task; the only writers are `fill_ring_tone` (before
/// arming) and the DMA reads it after.
static mut RING_STORE: [u32; STORE_WORDS] = [0; STORE_WORDS];

/// First 16 KiB-aligned window inside `RING_STORE` -- the DMA read base + fill
/// target.
fn ring_ptr() -> *mut u32 {
    let base = addr_of_mut!(RING_STORE) as usize;
    let aligned = (base + (RING_BYTES - 1)) & !(RING_BYTES - 1);
    aligned as *mut u32
}

// ---- PIO I2S program --------------------------------------------------------
// Transcribed from pico-extras `audio_i2s.pio` (the canonical, hardware-proven
// I2S output for PCM510x DACs) -- NOT hand-derived: an earlier hand-rolled
// variant clocked data on the wrong BCK edge. 16-bit L + 16-bit R per 32-bit
// FIFO word, MSB-first; BCK = 32*fs. side_set 2 with base = BCK: side bit0 =
// BCK (GP26), bit1 = LRCK (GP27). Data changes while BCK is low, the rising
// edge on the `jmp`/`set` clocks it into the DAC, and LRCK transitions one BCK
// early (the I2S 1-bit delay). x = 14 -> 15 loop iterations + the closing
// `out` = 16 bits per half-frame. Wrap 0..7; entry point is instr 7.
//
//                                  ; side 0b<LRCK><BCK>
//   0 bitloop1: out  pins,1 side 0b10   ; right-channel bits (LRCK hi)
//   1           jmp  x--,0  side 0b11
//   2           out  pins,1 side 0b00   ; last right bit; LRCK -> lo
//   3           set  x,14   side 0b01
//   4 bitloop0: out  pins,1 side 0b00   ; left-channel bits (LRCK lo)
//   5           jmp  x--,4  side 0b01
//   6           out  pins,1 side 0b10   ; last left bit; LRCK -> hi
//   7 entry:    set  x,14   side 0b11
static I2S_PROG: [u16; 8] = [
    0x7001, 0x1840, 0x6001, 0xe82e, 0x6001, 0x0844, 0x7001, 0xf82e,
];
/// `set pindirs, 1` (side 0b00) -- make the OUT pin (DIN) an output.
const SET_PINDIRS_1: u16 = 0xe081;
/// `set pindirs, 3` (side 0b00) -- make the side-set pair (BCK, LRCK) outputs.
const SET_PINDIRS_3: u16 = 0xe083;
/// The entry instruction (`set x, 14` side 0b11), exec'd to prime the bit
/// counter before the SM is enabled at PC 0.
const ENTRY_SET_X: u16 = 0xf82e;

/// Configure a GPIO as an SIO output driven to `high`.
fn sio_output(p: &rp235x_pac::Peripherals, pin: u32, high: bool) {
    p.PADS_BANK0.gpio(pin as usize).modify(|_, w| {
        w.od().clear_bit();
        w.iso().clear_bit();
        w.ie().set_bit()
    });
    p.IO_BANK0
        .gpio(pin as usize)
        .gpio_ctrl()
        .modify(|_, w| unsafe { w.funcsel().bits(5) });
    if high {
        p.SIO.gpio_out_set().write(|w| unsafe { w.bits(1 << pin) });
    } else {
        p.SIO.gpio_out_clr().write(|w| unsafe { w.bits(1 << pin) });
    }
    p.SIO.gpio_oe_set().write(|w| unsafe { w.bits(1 << pin) });
}

/// Configure a GPIO pad + route it to PIO0 (mirrors the ws2812/cyw43 PIO pins).
fn route_pio_pin(p: &rp235x_pac::Peripherals, pin: u32) {
    p.PADS_BANK0.gpio(pin as usize).modify(|_, w| {
        w.od().clear_bit();
        w.iso().clear_bit();
        w.ie().set_bit();
        unsafe { w.drive().bits(3) }
    });
    p.IO_BANK0
        .gpio(pin as usize)
        .gpio_ctrl()
        .modify(|_, w| unsafe { w.funcsel().bits(FUNCSEL_PIO0) });
}

/// (int, frac) clock divider for the PIO SM at `rate` Hz (PIO clk = 128*rate).
fn clkdiv_for(rate: u32) -> (u16, u8) {
    let pio_clk = PIO_CYCLES_PER_FRAME * rate;
    let int = SYS_CLK_HZ / pio_clk;
    let frac = ((SYS_CLK_HZ % pio_clk) as u64 * 256 / pio_clk as u64) as u8;
    (int as u16, frac)
}

/// One-time PIO setup: route the 3 pins, load the program, configure SM0's
/// shift/pin/exec, set pin directions, and prime the bit counter. Leaves SM0
/// disabled -- `set_rate` enables it.
fn pio_init(p: &rp235x_pac::Peripherals) {
    route_pio_pin(p, DIN);
    route_pio_pin(p, BCK);
    route_pio_pin(p, LRCK);

    let pio = &p.PIO0;
    // Disable OUR SM only -- SM1 belongs to the ws2812 driver on the MusicPi.
    pio.ctrl().modify(|r, w| unsafe {
        w.sm_enable().bits(r.sm_enable().bits() & !1)
    });
    for (i, insn) in I2S_PROG.iter().enumerate() {
        pio.instr_mem(i).write(|w| unsafe { w.bits(*insn as u32) });
    }

    let sm = pio.sm(0);
    // MSB-first (shift left), autopull every 32 bits (pull_thresh 0 == 32).
    sm.sm_shiftctrl().modify(|_, w| unsafe {
        w.out_shiftdir().clear_bit();
        w.autopull().set_bit();
        w.pull_thresh().bits(0)
    });
    // OUT -> DIN (1 pin); side-set base = BCK (2 pins BCK,LRCK); SET base = DIN
    // for the first `set pindirs`.
    sm.sm_pinctrl().modify(|_, w| unsafe {
        w.out_base().bits(DIN as u8);
        w.out_count().bits(1);
        w.sideset_base().bits(BCK as u8);
        w.sideset_count().bits(2);
        w.set_base().bits(DIN as u8);
        w.set_count().bits(1)
    });
    sm.sm_execctrl()
        .modify(|_, w| unsafe { w.wrap_top().bits(7).wrap_bottom().bits(0) });

    // Canonical pio_sm_init tail (pico-sdk): restart the SM + its clock divider
    // to clear latched OSR/ISR shift counters and stall state -- the pre-kernel
    // PIO echo self-test ran on this same SM0 and leaves stale state behind.
    pio.ctrl().modify(|_, w| unsafe {
        w.sm_restart().bits(1);
        w.clkdiv_restart().bits(1)
    });

    // Pin directions. DIN and the BCK/LRCK pair are not contiguous, so drive
    // `set pindirs` twice, moving SET base between them.
    sm.sm_instr().write(|w| unsafe { w.sm0_instr().bits(SET_PINDIRS_1) });
    sm.sm_pinctrl()
        .modify(|_, w| unsafe { w.set_base().bits(BCK as u8).set_count().bits(2) });
    sm.sm_instr().write(|w| unsafe { w.sm0_instr().bits(SET_PINDIRS_3) });
    // Force the PC to the entry point (instr 7, `set x,14`): `jmp 7`
    // (side 0b00). The echo self-test leaves the PC wherever it stalled.
    sm.sm_instr().write(|w| unsafe { w.sm0_instr().bits(0x0007) });
}

/// Set the SM clock for `rate` and enable it.
fn set_rate(p: &rp235x_pac::Peripherals, rate: u32) {
    let pio = &p.PIO0;
    // Bitwise: never clobber SM1 (ws2812).
    pio.ctrl().modify(|r, w| unsafe {
        w.sm_enable().bits(r.sm_enable().bits() & !1)
    });
    let (int, frac) = clkdiv_for(rate);
    pio.sm(0)
        .sm_clkdiv()
        .write(|w| unsafe { w.int().bits(int).frac().bits(frac) });
    // Flush any stale words out of the TX FIFO: joining + unjoining clears
    // both FIFOs. (SM_RESTART does NOT flush FIFOs.)
    pio.sm(0)
        .sm_shiftctrl()
        .modify(|_, w| w.fjoin_tx().set_bit());
    pio.sm(0)
        .sm_shiftctrl()
        .modify(|_, w| w.fjoin_tx().clear_bit());
    // Clear the (write-1-to-clear) stall/overrun flags so post-enable FDEBUG
    // reads are fresh evidence, then restart SM0's shift state and divider.
    pio.fdebug().write(|w| unsafe { w.bits(0xffff_ffff) });
    pio.ctrl().modify(|_, w| unsafe {
        w.sm_restart().bits(1);
        w.clkdiv_restart().bits(1)
    });
    // CRITICAL: force the PC back to the entry instruction. SM_RESTART clears
    // OSR/shift counters but NOT the program counter -- a previously-stopped
    // SM sits stalled at an arbitrary mid-frame `out`, and resuming there
    // slips the I2S frame alignment by a partial word: every sample then
    // straddles two frames (sine -> buzz, music -> noise). Diagnosed on
    // hardware: the freshly-entered boot tone was pure while every re-armed
    // `play` buzzed.
    pio.sm(0).sm_instr().write(|w| unsafe { w.sm0_instr().bits(0x0007) });
    pio.ctrl().modify(|r, w| unsafe {
        w.sm_enable().bits(r.sm_enable().bits() | 1)
    });
}

/// Snap `freq` to a whole number of cycles across the ring so the loop is
/// seamless. Returns the cycle count (>= 1).
fn cycle_count(freq: u32, rate: u32) -> u32 {
    let f = freq.clamp(TONE_MIN_HZ, TONE_MAX_HZ);
    let c = (RING_FRAMES as u32 * f + rate / 2) / rate;
    c.max(1)
}

/// Integer sine via the Bhaskara I approximation (~0.2% error, no tables/FP):
/// `deg` in 0..360, returns -TONE_AMPL..=TONE_AMPL. A SMOOTH full-range test
/// signal -- a square tone still sounds "like a tone" through many bit-level
/// manglings (sign flips, shifts) that turn real audio into noise, so the
/// boot/test tone must be a sine to actually validate sample integrity.
fn isin(deg: u32) -> i16 {
    let (half, neg) = if deg < 180 { (deg, false) } else { (deg - 180, true) };
    // 4*x*(180-x) / (40500 - x*(180-x)), scaled by TONE_AMPL.
    let q = half * (180 - half);
    let s = (4 * q as u64 * TONE_AMPL as u64 / (40500 - q) as u64) as i16;
    if neg {
        -s
    } else {
        s
    }
}

/// Fill the ring with a stereo sine: `cyc_l` / `cyc_r` whole cycles for the
/// left / right channels. One word per frame, canonical pico-extras packing
/// `(left << 16) | right` (top half plays while ws=0 = LEFT).
fn fill_ring_tone(cyc_l: u32, cyc_r: u32) {
    let ring = ring_ptr();
    let sine = |frame: usize, cyc: u32| -> u16 {
        let pos = (frame as u32).wrapping_mul(cyc) % RING_FRAMES as u32;
        let deg = pos * 360 / RING_FRAMES as u32;
        isin(deg) as u16
    };
    for f in 0..RING_FRAMES {
        let l = sine(f, cyc_l) as u32;
        let r = sine(f, cyc_r) as u32;
        // SAFETY: ring is the aligned window in RING_STORE; f < RING_WORDS.
        unsafe {
            ring.add(f).write_volatile((l << 16) | r);
        }
    }
}

/// Abort the audio DMA and wait for it to drain (CHAN_ABORT self-clears).
fn abort_dma(p: &rp235x_pac::Peripherals) {
    p.DMA
        .chan_abort()
        .write(|w| unsafe { w.chan_abort().bits(1 << DMA_CH) });
    while p.DMA.chan_abort().read().bits() & (1 << DMA_CH) != 0 {}
}

/// Arm DMA ch0 to stream the ring into the PIO0 TX FIFO, one word per TX DREQ,
/// looping (read address wraps the ring; write fixed at TXF0).
fn arm_dma(p: &rp235x_pac::Peripherals) {
    abort_dma(p);
    let read_addr = ring_ptr() as u32;
    let write_addr = p.PIO0.txf(0).as_ptr() as u32;
    let ch = p.DMA.ch(DMA_CH);
    ch.ch_read_addr().write(|w| unsafe { w.bits(read_addr) });
    ch.ch_write_addr().write(|w| unsafe { w.bits(write_addr) });
    ch.ch_trans_count()
        .write(|w| unsafe { w.count().bits(TRANS_COUNT) });
    ch.ch_ctrl_trig().write(|w| unsafe {
        w.en().bit(true);
        w.data_size().bits(DATA_SIZE_WORD);
        w.incr_read().bit(true);
        w.incr_write().bit(false);
        w.ring_size().bits(RING_SIZE_BITS);
        w.ring_sel().bit(false); // ring on the READ address
        w.treq_sel().bits(TREQ_PIO0_TX0);
        w.chain_to().bits(DMA_CH as u8); // chain to self = no chain
        w.high_priority().bit(false);
        w
    });
}

/// Retune/restart the tone: validate, set the rate, refill the ring, re-arm.
fn play(
    p: &rp235x_pac::Peripherals,
    hz_left: u32,
    hz_right: u32,
    rate: u32,
) -> Result<(), RequestError<I2sError>> {
    if !(RATE_MIN_HZ..=RATE_MAX_HZ).contains(&rate) {
        return Err(I2sError::BadArg.into());
    }
    set_rate(p, rate);
    fill_ring_tone(cycle_count(hz_left, rate), cycle_count(hz_right, rate));
    arm_dma(p);
    Ok(())
}

/// Fill one ring chunk from the decoder: 16-bit interleaved stereo straight
/// into `(right << 16) | left` frames -- no quantizer, the DAC gets the
/// samples as-is. Frames past decoder EOF are zero (the DAC's zero-data
/// detect then analog-mutes the tail). Returns the sample count decoded.
#[cfg(feature = "sdcard")]
fn fill_chunk<D: decoder::Decoder>(dec: &mut D, chunk: usize) -> usize {
    let ring = ring_ptr();
    let base = chunk * PLAY_CHUNK_FRAMES;
    let mut pcm = [0i16; PLAY_CHUNK_FRAMES * 2];
    let got = dec.next_pcm(&mut pcm);
    for slot in pcm.iter_mut().skip(got) {
        *slot = 0;
    }
    for i in 0..PLAY_CHUNK_FRAMES {
        let l = pcm[2 * i] as u16 as u32;
        let r = pcm[2 * i + 1] as u16 as u32;
        // Canonical pico-extras FIFO format: bits 31:16 play while ws=0 (the
        // LEFT channel in I2S), bits 15:0 while ws=1 (right).
        // SAFETY: ring is the aligned window in RING_STORE; base + i <
        // RING_FRAMES. Single-threaded server, no references taken.
        unsafe {
            ring.add(base + i).write_volatile((l << 16) | r);
        }
    }
    got
}

/// Which ring chunk the DMA is currently reading, from its live read address.
#[cfg(feature = "sdcard")]
fn dma_chunk(p: &rp235x_pac::Peripherals) -> usize {
    let base = ring_ptr() as u32;
    let addr = p.DMA.ch(DMA_CH).ch_read_addr().read().bits();
    let off = addr.wrapping_sub(base) & (RING_BYTES as u32 - 1);
    (off / PLAY_CHUNK_BYTES) as usize
}

/// Spin until the DMA read pointer crosses `laps` chunk boundaries (EOF tail
/// drain), bailing if it stalls for PLAY_STALL_MS.
#[cfg(feature = "sdcard")]
fn drain_crossings(p: &rp235x_pac::Peripherals, laps: u32) {
    let mut crossings = 0u32;
    let mut prev = dma_chunk(p);
    let mut last_progress = sys_get_timer().now;
    let mut polls = 0u32;
    while crossings < laps {
        let now = dma_chunk(p);
        if now != prev {
            crossings += 1;
            prev = now;
            last_progress = sys_get_timer().now;
        }
        polls = polls.wrapping_add(1);
        if polls & PLAY_STALL_POLL_MASK == 0
            && sys_get_timer().now.wrapping_sub(last_progress) > PLAY_STALL_MS
        {
            break;
        }
    }
}

/// Blocking play loop (port of the pwm player's): prime the whole ring, arm
/// the DMA, keep the producer one lap behind the read pointer, `prefetch` in
/// the idle slack, and after EOF let the silence tail drain one lap. Returns
/// (underruns, refills).
#[cfg(feature = "sdcard")]
fn play_loop<D: decoder::Decoder>(
    p: &rp235x_pac::Peripherals,
    dec: &mut D,
) -> (u32, u32) {
    let mut eof = false;
    for chunk in 0..PLAY_CHUNKS {
        eof = fill_chunk(dec, chunk) == 0;
    }
    arm_dma(p);

    let mut refills = 0u32;
    let mut underruns = 0u32;
    let mut next_fill = 0usize;
    let mut last_progress = sys_get_timer().now;
    let mut polls = 0u32;
    while !eof {
        if next_fill != dma_chunk(p) {
            eof = fill_chunk(dec, next_fill) == 0;
            refills += 1;
            if dma_chunk(p) == next_fill {
                underruns += 1; // producer fell a whole ring behind
            }
            next_fill = (next_fill + 1) % PLAY_CHUNKS;
            last_progress = sys_get_timer().now;
            continue;
        }
        // Caught up: burn the slack pre-decoding the next (FLAC) block so
        // refills stay pure copy.
        dec.prefetch();
        polls = polls.wrapping_add(1);
        if polls & PLAY_STALL_POLL_MASK == 0
            && sys_get_timer().now.wrapping_sub(last_progress) > PLAY_STALL_MS
        {
            break;
        }
    }
    drain_crossings(p, PLAY_CHUNKS as u32);
    abort_dma(p);
    (underruns, refills)
}

/// Pack the player reply like the pwm driver: rate in bits 0..15, underruns
/// 16..22 (saturating), truncated flag bit 23, refills 24..31 (saturating).
#[cfg(feature = "sdcard")]
fn pack_play_reply(
    rate: u32,
    underruns: u32,
    refills: u32,
    truncated: u32,
) -> u32 {
    (rate & 0xffff)
        | (underruns.min(0x7f) << 16)
        | ((truncated & 1) << 23)
        | (refills.min(0xff) << 24)
}

struct ServerImpl;

impl idl::InOrderRp235xI2sImpl for ServerImpl {
    fn tone(
        &mut self,
        _: &RecvMessage,
        freq_hz: u32,
        rate_hz: u32,
        _ms: u32,
    ) -> Result<(), RequestError<I2sError>> {
        // Stage 1: `tone` starts a continuous tone (the timed variant lands with
        // the notification-driven stop in Stage 2).
        let p = unsafe { rp235x_pac::Peripherals::steal() };
        play(&p, freq_hz, freq_hz, rate_hz)
    }

    fn audio_start(
        &mut self,
        _: &RecvMessage,
        freq_hz: u32,
        rate_hz: u32,
    ) -> Result<(), RequestError<I2sError>> {
        let p = unsafe { rp235x_pac::Peripherals::steal() };
        play(&p, freq_hz, freq_hz, rate_hz)
    }

    fn audio_stereo(
        &mut self,
        _: &RecvMessage,
        hz_left: u32,
        hz_right: u32,
        rate_hz: u32,
    ) -> Result<(), RequestError<I2sError>> {
        let p = unsafe { rp235x_pac::Peripherals::steal() };
        play(&p, hz_left, hz_right, rate_hz)
    }

    #[cfg(not(feature = "sdcard"))]
    fn play_file(
        &mut self,
        _: &RecvMessage,
        _name: LenLimit<Leased<R, [u8]>, 12>,
    ) -> Result<u32, RequestError<I2sError>> {
        Err(I2sError::OpenFailed.into())
    }

    // `inline(never)`: the decoders hold tens of KiB on the stack for the
    // whole play loop; keep that frame out of the dispatch path.
    #[cfg(feature = "sdcard")]
    #[inline(never)]
    fn play_file(
        &mut self,
        _: &RecvMessage,
        name: LenLimit<Leased<R, [u8]>, 12>,
    ) -> Result<u32, RequestError<I2sError>> {
        let n = name.len().min(12);
        let mut name_buf = [0u8; 12];
        name.read_range(0..n, &mut name_buf[..n])
            .map_err(|()| RequestError::went_away())?;

        let source = sd::SdFileSource::open(
            Rp235xSdcard::from(SDCARD.get_task_id()),
            &name_buf[..n],
        )
        .ok_or(I2sError::OpenFailed)?;
        // Route by 8.3 extension, exactly like the pwm player.
        let mut dec = if decoder::is_flac_name(&name_buf[..n]) {
            decoder::AnyDecoder::Flac(
                decoder::FlacDecoder::new(source)
                    .map_err(|_| I2sError::BadFile)?,
            )
        } else if decoder::is_mp3_name(&name_buf[..n]) {
            decoder::AnyDecoder::Mp3(
                decoder::Nanomp3Decoder::new(source)
                    .map_err(|_| I2sError::BadFile)?,
            )
        } else {
            decoder::AnyDecoder::Wav(
                decoder::WavDecoder::new(source)
                    .map_err(|_| I2sError::BadFile)?,
            )
        };

        // The DAC's PLL only locks for fs >= 32 kHz at BCK = 32*fs: reject
        // rather than clamp (a clamped rate would pitch-shift the audio).
        let rate = decoder::Decoder::sample_rate(&dec);
        if !(RATE_MIN_HZ..=RATE_MAX_HZ).contains(&rate) {
            return Err(I2sError::BadRate.into());
        }

        let p = unsafe { rp235x_pac::Peripherals::steal() };
        abort_dma(&p);
        set_rate(&p, rate);
        let (underruns, refills) = play_loop(&p, &mut dec);
        // Leave the ring zeroed so the DAC's zero-data detect mutes.
        let ring = ring_ptr();
        for i in 0..RING_WORDS {
            // SAFETY: i < RING_WORDS, within the aligned window.
            unsafe { ring.add(i).write_volatile(0) };
        }
        arm_dma(&p);
        let truncated = decoder::Decoder::had_error(&dec) as u32;
        Ok(pack_play_reply(rate, underruns, refills, truncated))
    }

    fn dbg(
        &mut self,
        _: &RecvMessage,
        which: u8,
    ) -> Result<u32, RequestError<I2sError>> {
        let p = unsafe { rp235x_pac::Peripherals::steal() };
        let ch = p.DMA.ch(DMA_CH);
        Ok(match which {
            0 => ch.ch_ctrl_trig().read().bits(),
            1 => ch.ch_trans_count().read().bits(),
            2 => ch.ch_read_addr().read().bits(),
            3 => ch.ch_write_addr().read().bits(),
            4 => p.PIO0.fstat().read().bits(),
            5 => p.PIO0.fdebug().read().bits(),
            6 => p.PIO0.flevel().read().bits(),
            7 => p.PIO0.ctrl().read().bits(),
            8 => p.PIO0.sm(0).sm_execctrl().read().bits(),
            // SM0 program counter: cycling 0..7 = running, pinned = stalled.
            9 => p.PIO0.sm(0).sm_addr().read().bits(),
            // IO_BANK0 GPIO_CTRL (funcsel/overrides) + GPIO_STATUS
            // (OUTTOPAD/OETOPAD -- does the PIO output reach the pad?).
            10 => p.IO_BANK0.gpio(BCK as usize).gpio_ctrl().read().bits(),
            11 => p.IO_BANK0.gpio(BCK as usize).gpio_status().read().bits(),
            12 => p.IO_BANK0.gpio(LRCK as usize).gpio_status().read().bits(),
            13 => p.IO_BANK0.gpio(DIN as usize).gpio_status().read().bits(),
            14 => p.PADS_BANK0.gpio(BCK as usize).read().bits(),
            // SM config readbacks: is the clock divider / pin mapping what we
            // programmed? And RP2350's PIO GPIOBASE (0 or 16): a nonzero base
            // shifts every side-set/out pin index -- silent wrong-pin output.
            15 => p.PIO0.sm(0).sm_clkdiv().read().bits(),
            16 => p.PIO0.sm(0).sm_pinctrl().read().bits(),
            17 => p.PIO0.sm(0).sm_shiftctrl().read().bits(),
            18 => p.PIO0.gpiobase().read().bits(),
            // 64 fast SM_ADDR samples -> 4-bit saturating count per address
            // (nibble n = hits at addr n). Distinguishes free-running (spread
            // over 0,1,4,5) from hard-stalled (one saturated nibble).
            19 => {
                let mut hist = [0u32; 8];
                for _ in 0..64 {
                    let a = (p.PIO0.sm(0).sm_addr().read().bits() & 7) as usize;
                    if hist[a] < 15 {
                        hist[a] += 1;
                    }
                }
                hist.iter()
                    .enumerate()
                    .fold(0u32, |acc, (i, c)| acc | (c << (4 * i)))
            }
            // In-task transition counter (software frequency probe): read
            // GPIO_STATUS.INFROMPAD (bit 17) 8192x back-to-back and count
            // transitions + ones. (transitions << 16) | ones. A 1.5 MHz clock
            // shows thousands of transitions; a stuck pin shows 0.
            20 | 21 | 22 => {
                let pin = match which {
                    20 => BCK,
                    21 => LRCK,
                    _ => DIN,
                } as usize;
                let st = p.IO_BANK0.gpio(pin).gpio_status();
                let mut prev = (st.read().bits() >> 17) & 1;
                let (mut trans, mut ones) = (0u32, 0u32);
                for _ in 0..8192 {
                    let b = (st.read().bits() >> 17) & 1;
                    if b != prev {
                        trans += 1;
                        prev = b;
                    }
                    ones += b;
                }
                (trans.min(0xffff) << 16) | ones.min(0xffff)
            }
            _ => 0,
        })
    }

    fn audio_stop(
        &mut self,
        _: &RecvMessage,
    ) -> Result<(), RequestError<I2sError>> {
        // Abort the DMA and fill the ring with silence; the DAC's zero-data
        // detector then analog-mutes.
        let p = unsafe { rp235x_pac::Peripherals::steal() };
        abort_dma(&p);
        let ring = ring_ptr();
        for i in 0..RING_WORDS {
            // SAFETY: i < RING_WORDS, within the aligned window.
            unsafe { ring.add(i).write_volatile(0) };
        }
        arm_dma(&p);
        Ok(())
    }
}

impl idol_runtime::NotificationHandler for ServerImpl {
    fn current_notification_mask(&self) -> u32 {
        0
    }
    fn handle_notification(&mut self, _bits: userlib::NotificationBits) {}
}

#[unsafe(export_name = "main")]
fn main() -> ! {
    let p = unsafe { rp235x_pac::Peripherals::steal() };
    // BRING-UP INSTRUMENTATION (no debug probe on this board): pause between
    // stages so the pin states can be sampled over the shell to localise a
    // failure. Expected walk: ~0-3s BCK=1,LRCK=1 (post-init park); ~3-6s
    // BCK=0,LRCK=1 (SM enabled, stalled on empty FIFO at the first `out`);
    // >6s toggling (DMA feeding). asm::delay is cycles: 150e6 * 3 = 3 s.
    // MusicPi control pin: XSMT/amp-EN high = DAC unmuted + headphone amp on.
    // (Amp gain is set by the on-board DIP switches; see the note above.)
    sio_output(&p, XSMT, true);
    pio_init(&p);
    // Boot tone: a 440 Hz square at 48 kHz, so the DAC makes sound immediately
    // (proves PIO I2S + PLL) without needing a shell command.
    let _ = play(&p, DEFAULT_TONE_HZ, DEFAULT_TONE_HZ, DEFAULT_RATE_HZ);

    let mut server = ServerImpl;
    let mut incoming = [0u8; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    use drv_rp235x_i2s_api::I2sError;
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
