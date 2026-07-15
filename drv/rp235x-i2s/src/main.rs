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
//! Arms a sine test tone at boot and serves the Idol API (tone/stereo/stop,
//! `play_file` for SD WAV/FLAC/MP3 via the shared decode stack, and a `dbg`
//! register peek used during bring-up).

#![no_std]
#![no_main]

use core::ptr::addr_of_mut;
use drv_rp235x_gpio_api::Rp235xGpio;
use drv_rp235x_i2s_api::I2sError;
use idol_runtime::RequestError;
use userlib::sys_get_timer;
use userlib::sys_set_timer;
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
task_slot!(GPIO, gpio);

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
/// DMA channel: ch1 (SECCFG_CH1.P cleared in the pre-kernel main). ch0 belongs
/// to the pwm driver's tone/mic paths -- sharing it let a pwm command clobber
/// an i2s stream mid-song.
const DMA_CH: usize = 1;
/// TREQ (DREQ) select for the PIO0 SM0 TX FIFO. RP2350 DREQ index 0 = PIO0_TX0.
/// (If the tone is silent/garbled, this is the first thing to re-check.)
const TREQ_PIO0_TX0: u8 = 0;
/// DMA transfer size: 32-bit word.
const DATA_SIZE_WORD: u8 = 2;
/// Ring wrap: 2^14 = 16384 bytes = 4096 u32 = the whole ring -- the same size
/// as the pwm player's, banking ~93 ms of slack at 44.1 kHz against the FLAC
/// decoder's bursty whole-block prefetch.
const RING_SIZE_BITS: u8 = 14;
/// Large finite reload: ~93 minutes at 48 kHz, after which an unattended tone
/// goes silent (the next tone/stop/play re-arms cleanly). ENDLESS/COUNT=0 does
/// not arm on this HW (see the pwm driver note); a completion-IRQ re-arm is
/// the fix if a truly endless stream is ever needed.
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
const RING_BYTES: usize = RING_WORDS * 4;

/// Backing store, 2x the ring so a RING_BYTES-aligned window always fits (the
/// DMA ring-wrap requires the read base aligned to the ring size).
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

/// Configure a GPIO as an SIO output driven to `high`. Muxing goes through
/// the gpio task (this task has no IO_BANK0/PADS_BANK0 grant -- MPU regions
/// are budgeted); the level itself is set via SIO, which we do own.
fn sio_output(p: &rp235x_pac::Peripherals, pin: u32, high: bool) {
    let gpio = Rp235xGpio::from(GPIO.get_task_id());
    let _ = gpio.set_function(pin as u8, 5);
    if high {
        p.SIO.gpio_out_set().write(|w| unsafe { w.bits(1 << pin) });
    } else {
        p.SIO.gpio_out_clr().write(|w| unsafe { w.bits(1 << pin) });
    }
    p.SIO.gpio_oe_set().write(|w| unsafe { w.bits(1 << pin) });
}

/// Route a pad to PIO0 via the gpio task (un-isolates + enables the buffers).
fn route_pio_pin(_p: &rp235x_pac::Peripherals, pin: u32) {
    let gpio = Rp235xGpio::from(GPIO.get_task_id());
    let _ = gpio.set_function(pin as u8, FUNCSEL_PIO0);
}

/// (int, frac) clock divider for the PIO SM at `rate` Hz (PIO clk =
/// PIO_CYCLES_PER_FRAME * rate = 64 * rate). frac rounds to nearest.
fn clkdiv_for(rate: u32) -> (u16, u8) {
    let pio_clk = PIO_CYCLES_PER_FRAME * rate;
    let int = SYS_CLK_HZ / pio_clk;
    let frac = (((SYS_CLK_HZ % pio_clk) as u64 * 256 + pio_clk as u64 / 2)
        / pio_clk as u64) as u8;
    (int as u16, frac)
}

/// Arm a free-running ring DMA channel. `ring_on_write` selects which address
/// wraps; the other stays fixed (a PIO FIFO).
fn arm_ring_dma(
    p: &rp235x_pac::Peripherals,
    ch_i: usize,
    read: u32,
    write: u32,
    treq: u8,
    ring_bits: u8,
    ring_on_write: bool,
) {
    p.DMA
        .chan_abort()
        .write(|w| unsafe { w.chan_abort().bits(1 << ch_i) });
    while p.DMA.chan_abort().read().bits() & (1 << ch_i) != 0 {}
    let ch = p.DMA.ch(ch_i);
    ch.ch_read_addr().write(|w| unsafe { w.bits(read) });
    ch.ch_write_addr().write(|w| unsafe { w.bits(write) });
    ch.ch_trans_count()
        .write(|w| unsafe { w.count().bits(TRANS_COUNT) });
    ch.ch_ctrl_trig().write(|w| unsafe {
        w.en().bit(true);
        w.data_size().bits(DATA_SIZE_WORD);
        w.incr_read().bit(!ring_on_write);
        w.incr_write().bit(ring_on_write);
        w.ring_size().bits(ring_bits);
        w.ring_sel().bit(ring_on_write);
        w.treq_sel().bits(treq);
        w.chain_to().bits(ch_i as u8);
        w.high_priority().bit(false);
        w
    });
}

/// PIO1 bring-up: the capture receivers (SM0/SM1) and, with `simadc`, the
/// fake-ADC transmitters (SM2/SM3) that drive the same pads. All SMs run at
/// full clk_sys (they are `wait`-paced by the PIO0-driven BCK/LRCK).
fn pio1_init(p: &rp235x_pac::Peripherals) {
    // Input pads via the gpio task (un-isolates + enables both buffers). With
    // simadc the same pads are also PIO1 OUTPUTS -- the receivers read the
    // driven value back.
    {
        let gpio = Rp235xGpio::from(GPIO.get_task_id());
        for pin in [IN_A, IN_B] {
            let _ = gpio.set_function(pin as u8, FUNCSEL_PIO1);
        }
    }

    let pio = &p.PIO1;
    pio.ctrl().modify(|_, w| unsafe { w.sm_enable().bits(0) });
    for (i, insn) in PIO1_RX_PROG.iter().enumerate() {
        pio.instr_mem(i).write(|w| unsafe { w.bits(*insn as u32) });
    }
    #[cfg(feature = "simadc")]
    for (i, insn) in PIO1_SIM_PROG.iter().enumerate() {
        pio.instr_mem(8 + i)
            .write(|w| unsafe { w.bits(*insn as u32) });
    }

    // Receivers: SM0 reads IN_A, SM1 reads IN_B. autopush 32, shift LEFT.
    for (smi, pin) in [(0usize, IN_A), (1, IN_B)] {
        let sm = pio.sm(smi);
        sm.sm_clkdiv().write(|w| unsafe { w.int().bits(1).frac().bits(0) });
        sm.sm_shiftctrl().modify(|_, w| unsafe {
            w.in_shiftdir().clear_bit();
            w.autopush().set_bit();
            w.push_thresh().bits(0)
        });
        sm.sm_pinctrl()
            .modify(|_, w| unsafe { w.in_base().bits(pin as u8) });
        sm.sm_execctrl()
            .modify(|_, w| unsafe { w.wrap_top().bits(7).wrap_bottom().bits(4) });
        // Start at the sync preamble.
        sm.sm_instr().write(|w| unsafe { w.sm0_instr().bits(0x0000) });
    }

    // Fake ADCs: SM2 drives IN_A, SM3 drives IN_B. autopull 32, shift LEFT.
    #[cfg(feature = "simadc")]
    for (smi, pin) in [(2usize, IN_A), (3, IN_B)] {
        let sm = pio.sm(smi);
        sm.sm_clkdiv().write(|w| unsafe { w.int().bits(1).frac().bits(0) });
        sm.sm_shiftctrl().modify(|_, w| unsafe {
            w.out_shiftdir().clear_bit();
            w.autopull().set_bit();
            w.pull_thresh().bits(0)
        });
        sm.sm_pinctrl().modify(|_, w| unsafe {
            w.out_base().bits(pin as u8);
            w.out_count().bits(1);
            w.set_base().bits(pin as u8);
            w.set_count().bits(1)
        });
        sm.sm_execctrl()
            .modify(|_, w| unsafe {
                w.wrap_top().bits(13).wrap_bottom().bits(10)
            });
        // Pin to output, then start at the sync preamble (instr 8).
        sm.sm_instr()
            .write(|w| unsafe { w.sm0_instr().bits(SET_PINDIRS_1) });
        sm.sm_instr().write(|w| unsafe { w.sm0_instr().bits(0x0008) });
    }

    // Fill the simadc rings with cycle-snapped sines: A ~523 Hz, B ~659 Hz at
    // 48 kHz (they track fs). Same value both channels of each fake input.
    #[cfg(feature = "simadc")]
    {
        // SAFETY: pre-arm, single-threaded.
        let (a, b) = unsafe {
            (
                &mut (*core::ptr::addr_of_mut!(SIM_A)).0,
                &mut (*core::ptr::addr_of_mut!(SIM_B)).0,
            )
        };
        for (ring, hz) in [(a, 523u32), (b, 659u32)] {
            let cyc = ((SIM_FRAMES as u32 * hz + 24_000) / 48_000).max(1);
            for (f, w) in ring.iter_mut().enumerate() {
                let deg =
                    (f as u32 * cyc % SIM_FRAMES as u32) * 360 / SIM_FRAMES as u32;
                let v = isin(deg) as u16 as u32;
                *w = (v << 16) | v;
            }
        }
        arm_ring_dma(
            p,
            DMA_SIM_A,
            core::ptr::addr_of!(SIM_A) as u32,
            pio.txf(2).as_ptr() as u32,
            TREQ_PIO1_TX2,
            SIM_RING_BITS,
            false,
        );
        arm_ring_dma(
            p,
            DMA_SIM_B,
            core::ptr::addr_of!(SIM_B) as u32,
            pio.txf(3).as_ptr() as u32,
            TREQ_PIO1_TX3,
            SIM_RING_BITS,
            false,
        );
    }

    // Capture DMA: RXF -> rings, ring on the WRITE address.
    arm_ring_dma(
        p,
        DMA_RX_A,
        pio.rxf(0).as_ptr() as u32,
        core::ptr::addr_of!(CAP_A) as u32,
        TREQ_PIO1_RX0,
        CAP_RING_BITS,
        true,
    );
    arm_ring_dma(
        p,
        DMA_RX_B,
        pio.rxf(1).as_ptr() as u32,
        core::ptr::addr_of!(CAP_B) as u32,
        TREQ_PIO1_RX1,
        CAP_RING_BITS,
        true,
    );

    // Enable: transmitters first (they park in the sync preamble with data
    // banked), then the receivers.
    #[cfg(feature = "simadc")]
    pio.ctrl().modify(|r, w| unsafe {
        w.sm_enable().bits(r.sm_enable().bits() | 0b1100)
    });
    pio.ctrl().modify(|r, w| unsafe {
        w.sm_enable().bits(r.sm_enable().bits() | 0b0011)
    });
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



/// Which ring chunk the DMA is currently reading, from its live read address.
#[cfg(feature = "sdcard")]
fn dma_chunk(p: &rp235x_pac::Peripherals) -> usize {
    let base = ring_ptr() as u32;
    let addr = p.DMA.ch(DMA_CH).ch_read_addr().read().bits();
    let off = addr.wrapping_sub(base) & (RING_BYTES as u32 - 1);
    (off / PLAY_CHUNK_BYTES) as usize
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

/// Mixer tick period (ms). One ring chunk is ~10 ms at 48 kHz; a 3 ms tick
/// keeps the producer comfortably ahead with the 8-chunk ring banking slack.
const TICK_MS: u64 = 3;

/// One mixer source's control state.
#[derive(Copy, Clone)]
struct Src {
    enable: bool,
    /// Q15 gain (0..=32767).
    gain: u16,
    /// DDS phase + step (tone sources only; step derives from `hz` and fs).
    /// Unused on capture (`simadc`) builds, where A/B come from the rings.
    #[cfg_attr(feature = "simadc", allow(dead_code))]
    phase: u32,
    hz: u32,
}

/// Ducking config + envelope state: the SD source ducks under the live bus.
#[derive(Copy, Clone)]
struct Duck {
    enable: bool,
    /// Q15 envelope threshold.
    threshold: u16,
    attack_ms: u16,
    release_ms: u16,
    /// Q15 gain the SD source ducks down to.
    floor: u16,
    /// Q15 envelope of the live bus.
    env: u16,
    /// Q15 current duck gain (slews between 32767 and `floor`).
    gain: u16,
}

// ---- Capture path (PIO1): I2S slave receivers on the input pins, clocked by
// reading our own BCK/LRCK. See docs/mixer-roadmap.md "Sprint-2 capture".
/// Input A / B data pins (MusicPi free pins; PCM1802 DOUTs land here).
const IN_A: u32 = 27;
const IN_B: u32 = 28;
/// funcsel routing a GPIO to PIO1.
const FUNCSEL_PIO1: u8 = 7;
/// DMA channels: capture writes (ring on WRITE), simadc feeders (ring on READ).
const DMA_RX_A: usize = 2;
const DMA_RX_B: usize = 3;
#[cfg(feature = "simadc")]
const DMA_SIM_A: usize = 4;
#[cfg(feature = "simadc")]
const DMA_SIM_B: usize = 5;
/// RP2350 DREQ indexes: PIO1 TX0..3 = 8..11, RX0..3 = 12..15.
const TREQ_PIO1_RX0: u8 = 12;
const TREQ_PIO1_RX1: u8 = 13;
#[cfg(feature = "simadc")]
const TREQ_PIO1_TX2: u8 = 10;
#[cfg(feature = "simadc")]
const TREQ_PIO1_TX3: u8 = 11;

/// Capture rings: 1024 frames = 4 KiB each ((L << 16) | R words), ring-aligned.
const CAP_FRAMES: usize = 1024;
const CAP_RING_BITS: u8 = 12;
#[repr(align(4096))]
struct CapRing([u32; CAP_FRAMES]);
static mut CAP_A: CapRing = CapRing([0; CAP_FRAMES]);
static mut CAP_B: CapRing = CapRing([0; CAP_FRAMES]);

/// simadc feeder rings: 256 frames = 1 KiB, filled once with cycle-snapped
/// sines (distinct pitches so channel identity is audible), looped by DMA.
#[cfg(feature = "simadc")]
const SIM_FRAMES: usize = 128;
#[cfg(feature = "simadc")]
const SIM_RING_BITS: u8 = 9;
#[cfg(feature = "simadc")]
#[repr(align(512))]
struct SimRing([u32; SIM_FRAMES]);
#[cfg(feature = "simadc")]
static mut SIM_A: SimRing = SimRing([0; SIM_FRAMES]);
#[cfg(feature = "simadc")]
static mut SIM_B: SimRing = SimRing([0; SIM_FRAMES]);

/// PIO1 program: shared I2S slave receiver (offset 0, SM0/SM1; the SMs differ
/// only in in_base). Sync ONCE to a LRCK fall + skip one BCK (the I2S 1-bit
/// delay), then free-run 32 bits/frame -- a per-frame resync would skip
/// alternate frames because the delay pushes each frame's last bit across the
/// boundary. autopush 32, ISR shifts left -> exact (L<<16)|R words.
///   0 wait 1 gpio 11 ; 1 wait 0 gpio 11 ; 2 wait 1 gpio 10 ; 3 wait 0 gpio 10
///   4 wait 1 gpio 10 ; 5 in pins,1     ; 6 wait 0 gpio 10 ; 7 jmp 4
const PIO1_RX_PROG: [u16; 8] =
    [0x208b, 0x200b, 0x208a, 0x200a, 0x208a, 0x4001, 0x200a, 0x0004];
/// simadc fake-ADC transmitter (offset 8, SM2/SM3): sync to a LRCK fall, then
/// out one bit per BCK falling edge (data changes on the fall, the receivers
/// sample on the rise). autopull 32, OSR shifts left.
///   8 wait 1 gpio 11 ; 9 wait 0 gpio 11
///  10 wait 1 gpio 10 ; 11 wait 0 gpio 10 ; 12 out pins,1 ; 13 jmp 10
#[cfg(feature = "simadc")]
const PIO1_SIM_PROG: [u16; 6] =
    [0x208b, 0x200b, 0x208a, 0x200a, 0x6001, 0x000a];

/// Which decoder (if any) the SD source is running. The decoders themselves
/// live in the statics below (not on an op's stack) so they survive across
/// mixer ticks.
#[derive(Copy, Clone, PartialEq)]
enum SdKind {
    None,
    Flac,
    Wav,
}

/// FLAC decoder slot: ~37 KiB, initialised IN PLACE via `FlacDecoder::new_at`
/// (placement-new) so it never exists on the stack. Valid only while
/// `sd_kind == Flac`. SAFETY: single-threaded task; only play_file/stop_file
/// and the tick touch these.
#[cfg(feature = "sdcard")]
static mut FLAC_SLOT: core::mem::MaybeUninit<
    decoder::FlacDecoder<sd::SdFileSource>,
> = core::mem::MaybeUninit::uninit();

/// WAV decoder slot (under 1 KiB -- a by-value move is fine).
#[cfg(feature = "sdcard")]
static mut WAV_SLOT: Option<decoder::WavDecoder<sd::SdFileSource>> = None;

/// Drop whatever decoder is live (closes the SD file handles).
#[cfg(feature = "sdcard")]
fn drop_sd(kind: SdKind) {
    // SAFETY: single-threaded; kind tracks slot validity.
    unsafe {
        match kind {
            SdKind::Flac => {
                core::ptr::drop_in_place(
                    (*core::ptr::addr_of_mut!(FLAC_SLOT)).as_mut_ptr(),
                );
            }
            SdKind::Wav => {
                *core::ptr::addr_of_mut!(WAV_SLOT) = None;
            }
            SdKind::None => {}
        }
    }
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

struct ServerImpl {
    /// Tone A, tone B (sim stand-ins for the capture inputs), SD source gain.
    tone_a: Src,
    tone_b: Src,
    sd_gain: u16,
    sd_kind: SdKind,
    duck: Duck,
    /// Current sample rate (sets the PIO clock + DDS steps).
    rate: u32,
    /// Next ring chunk the tick will refill.
    next_fill: usize,
    /// Persistent capture-ring read cursor (frames). Advances exactly one
    /// chunk per mix_chunk; the capture DMA shares our audio clock, so the
    /// lag set at init never drifts. Recomputing from the live write pointer
    /// per chunk re-read overlapping data when a tick filled several chunks
    /// back-to-back (audible as a buzz of phase jumps).
    #[cfg(feature = "simadc")]
    cap_rd: usize,
    /// VU peaks (abs, Q15) since the last status read.
    vu_bus: u16,
    vu_sd: u16,
    vu_out: u16,
}

impl ServerImpl {
    #[cfg_attr(feature = "simadc", allow(dead_code))]
    fn dds_step(hz: u32, rate: u32) -> u32 {
        // 32-bit phase accumulator: step = hz * 2^32 / rate.
        ((hz as u64) << 32).div_euclid(rate as u64) as u32
    }

    /// Duck slew coefficient (Q15 per chunk) for a time constant in ms.
    fn slew(ms: u16) -> u32 {
        let chunk_ms =
            (PLAY_CHUNK_FRAMES as u32 * 1000 / 48_000).max(1);
        (32767 * chunk_ms / (ms as u32).max(chunk_ms)).min(32767)
    }

    /// Mix one ring chunk from the enabled sources; updates duck + VU state.
    fn mix_chunk(&mut self, p: &rp235x_pac::Peripherals, chunk: usize) {
        let ring = ring_ptr();
        let base = chunk * PLAY_CHUNK_FRAMES;

        // Pull SD samples for this chunk (interleaved stereo), if active.
        let mut sd = [0i16; PLAY_CHUNK_FRAMES * 2];
        #[cfg(feature = "sdcard")]
        if self.sd_kind != SdKind::None {
            // SAFETY: single-threaded; sd_kind tracks slot validity.
            let got = unsafe {
                match self.sd_kind {
                    SdKind::Flac => {
                        let d = (*core::ptr::addr_of_mut!(FLAC_SLOT))
                            .assume_init_mut();
                        let got = decoder::Decoder::next_pcm(d, &mut sd);
                        if got != 0 {
                            decoder::Decoder::prefetch(d);
                        }
                        got
                    }
                    SdKind::Wav => {
                        let w = (*core::ptr::addr_of_mut!(WAV_SLOT))
                            .as_mut()
                            .unwrap();
                        decoder::Decoder::next_pcm(w, &mut sd)
                    }
                    SdKind::None => 0,
                }
            };
            for s in sd.iter_mut().skip(got) {
                *s = 0;
            }
            if got == 0 {
                drop_sd(self.sd_kind);
                self.sd_kind = SdKind::None;
            }
        }

        // DDS steps drive the tones only on non-capture builds.
        #[cfg(not(feature = "simadc"))]
        let step_a = Self::dds_step(self.tone_a.hz, self.rate);
        #[cfg(not(feature = "simadc"))]
        let step_b = Self::dds_step(self.tone_b.hz, self.rate);
        let (mut pk_bus, mut pk_sd, mut pk_out) = (0u16, 0u16, 0u16);

        // Capture mode: sources A/B pull from the PIO1 capture rings (the fake
        // or real ADCs), read a fixed lag behind each ring's DMA write pointer.
        // Both rings share our clock, so the lag never drifts.
        #[cfg(feature = "simadc")]
        let (cap_a, cap_b, rd_a, rd_b) = {
            let _ = p;
            // SAFETY: DMA writes these; the cursor stays a fixed lag behind.
            let rd = self.cap_rd;
            self.cap_rd = (self.cap_rd + PLAY_CHUNK_FRAMES) % CAP_FRAMES;
            unsafe {
                (
                    &(*core::ptr::addr_of!(CAP_A)).0,
                    &(*core::ptr::addr_of!(CAP_B)).0,
                    rd,
                    rd,
                )
            }
        };

        // With BOTH tones on, A pans hard left and B hard right (doubles as
        // the stereo-identity test); a single tone plays in both ears.
        let both = self.tone_a.enable && self.tone_b.enable;
        for i in 0..PLAY_CHUNK_FRAMES {
            let (mut bus_l, mut bus_r) = (0i32, 0i32);
            if self.tone_a.enable {
                // Capture builds take source A from ring A; DDS otherwise.
                #[cfg(feature = "simadc")]
                let (sa_l, sa_r) = {
                    let w = cap_a[(rd_a + i) % CAP_FRAMES];
                    ((w >> 16) as i16 as i32, w as i16 as i32)
                };
                #[cfg(not(feature = "simadc"))]
                let (sa_l, sa_r) = {
                    let s = isin(
                        (self.tone_a.phase >> 23) as u32 * 360 / 512,
                    ) as i32;
                    self.tone_a.phase =
                        self.tone_a.phase.wrapping_add(step_a);
                    (s, s)
                };
                let g = self.tone_a.gain as i32;
                let (vl, vr) = (sa_l * g >> 15, sa_r * g >> 15);
                bus_l += vl;
                if !both {
                    bus_r += vr;
                } else {
                    // hard-pan A left in two-source mode
                }
            }
            if self.tone_b.enable {
                #[cfg(feature = "simadc")]
                let (sb_l, sb_r) = {
                    let w = cap_b[(rd_b + i) % CAP_FRAMES];
                    ((w >> 16) as i16 as i32, w as i16 as i32)
                };
                #[cfg(not(feature = "simadc"))]
                let (sb_l, sb_r) = {
                    let s = isin(
                        (self.tone_b.phase >> 23) as u32 * 360 / 512,
                    ) as i32;
                    self.tone_b.phase =
                        self.tone_b.phase.wrapping_add(step_b);
                    (s, s)
                };
                let g = self.tone_b.gain as i32;
                let (vl, vr) = (sb_l * g >> 15, sb_r * g >> 15);
                bus_r += vr;
                if !both {
                    bus_l += vl;
                }
            }
            let bus_pk = bus_l.abs().max(bus_r.abs());
            pk_bus = pk_bus.max(bus_pk.unsigned_abs().min(32767) as u16);

            let (sl, sr) = (sd[2 * i] as i32, sd[2 * i + 1] as i32);
            pk_sd = pk_sd.max(sl.unsigned_abs().min(32767) as u16);
            let sd_g = (self.sd_gain as i32 * self.duck.gain as i32) >> 15;
            let l = (bus_l + (sl * sd_g >> 15)).clamp(-32768, 32767);
            let r = (bus_r + (sr * sd_g >> 15)).clamp(-32768, 32767);
            pk_out = pk_out.max(l.unsigned_abs().min(32767) as u16);

            // SAFETY: ring window; base + i < RING_FRAMES.
            unsafe {
                ring.add(base + i).write_volatile(
                    ((r as u16 as u32) << 16) | l as u16 as u32,
                );
            }
        }

        // Duck: smooth the bus envelope, slew the SD gain toward floor when
        // over threshold (attack) and back to unity when under (release).
        let d = &mut self.duck;
        let ec = Self::slew(20) as i32; // fixed 20 ms envelope smoothing
        d.env = (d.env as i32 + ((pk_bus as i32 - d.env as i32) * ec >> 15))
            .clamp(0, 32767) as u16;
        let (target, rate_ms) = if d.enable && d.env > d.threshold {
            (d.floor, d.attack_ms)
        } else {
            (32767u16, d.release_ms)
        };
        let sc = Self::slew(rate_ms) as i32;
        d.gain =
            (d.gain as i32 + ((target as i32 - d.gain as i32) * sc >> 15))
                .clamp(0, 32767) as u16;

        self.vu_bus = self.vu_bus.max(pk_bus);
        self.vu_sd = self.vu_sd.max(pk_sd);
        self.vu_out = self.vu_out.max(pk_out);
    }

    /// Timer tick: refill every chunk the DMA has finished since last time.
    fn tick(&mut self, p: &rp235x_pac::Peripherals) {
        let cur = dma_chunk(p);
        while self.next_fill != cur {
            let c = self.next_fill;
            self.mix_chunk(p, c);
            self.next_fill = (self.next_fill + 1) % PLAY_CHUNKS;
        }
    }
}


impl idl::InOrderRp235xI2sImpl for ServerImpl {
    fn tone(
        &mut self,
        _: &RecvMessage,
        freq_hz: u32,
        _rate_hz: u32,
        _ms: u32,
    ) -> Result<(), RequestError<I2sError>> {
        // Tone = enable mixer source A at full gain (the mixer runs always).
        if !(TONE_MIN_HZ..=TONE_MAX_HZ).contains(&freq_hz) {
            return Err(I2sError::BadArg.into());
        }
        self.tone_a = Src { enable: true, gain: 24576, phase: 0, hz: freq_hz };
        self.tone_b.enable = false;
        Ok(())
    }

    fn audio_start(
        &mut self,
        msg: &RecvMessage,
        freq_hz: u32,
        rate_hz: u32,
    ) -> Result<(), RequestError<I2sError>> {
        self.tone(msg, freq_hz, rate_hz, 0)
    }

    fn audio_stereo(
        &mut self,
        _: &RecvMessage,
        hz_left: u32,
        hz_right: u32,
        _rate_hz: u32,
    ) -> Result<(), RequestError<I2sError>> {
        // Two mixer tones. (Both are mono into both ears now -- per-channel
        // panning arrives with the real capture inputs.)
        if !(TONE_MIN_HZ..=TONE_MAX_HZ).contains(&hz_left)
            || !(TONE_MIN_HZ..=TONE_MAX_HZ).contains(&hz_right)
        {
            return Err(I2sError::BadArg.into());
        }
        self.tone_a = Src { enable: true, gain: 16384, phase: 0, hz: hz_left };
        self.tone_b = Src { enable: true, gain: 16384, phase: 0, hz: hz_right };
        Ok(())
    }

    fn mixer_set(
        &mut self,
        _: &RecvMessage,
        src: u8,
        enable: u8,
        gain: u16,
        hz: u32,
    ) -> Result<(), RequestError<I2sError>> {
        let gain = gain.min(32767);
        match src {
            0 | 1 => {
                // hz == 0 means "keep the current frequency" (and is
                // meaningless on capture builds anyway).
                if hz != 0 && !(TONE_MIN_HZ..=TONE_MAX_HZ).contains(&hz) {
                    return Err(I2sError::BadArg.into());
                }
                let t = if src == 0 {
                    &mut self.tone_a
                } else {
                    &mut self.tone_b
                };
                t.enable = enable != 0;
                t.gain = gain;
                if hz != 0 {
                    t.hz = hz;
                }
            }
            2 => {
                self.sd_gain = gain;
                if enable == 0 {
                    #[cfg(feature = "sdcard")]
                    drop_sd(self.sd_kind);
                    self.sd_kind = SdKind::None;
                }
            }
            _ => return Err(I2sError::BadArg.into()),
        }
        Ok(())
    }

    fn duck_set(
        &mut self,
        _: &RecvMessage,
        enable: u8,
        threshold: u16,
        attack_ms: u16,
        release_ms: u16,
        floor: u16,
    ) -> Result<(), RequestError<I2sError>> {
        self.duck.enable = enable != 0;
        self.duck.threshold = threshold.min(32767);
        self.duck.attack_ms = attack_ms.max(1);
        self.duck.release_ms = release_ms.max(1);
        self.duck.floor = floor.min(32767);
        Ok(())
    }

    fn status(
        &mut self,
        _: &RecvMessage,
    ) -> Result<u32, RequestError<I2sError>> {
        // Q15 -> Q7 VU fields; peaks reset on read.
        let v = ((self.vu_bus >> 8) as u32 & 0x7f)
            | (((self.vu_sd >> 8) as u32 & 0x7f) << 8)
            | (((self.vu_out >> 8) as u32 & 0x7f) << 16)
            | (((self.sd_kind != SdKind::None) as u32) << 24)
            | (((self.duck.gain < 30000) as u32) << 25);
        self.vu_bus = 0;
        self.vu_sd = 0;
        self.vu_out = 0;
        Ok(v)
    }

    fn stop_file(
        &mut self,
        _: &RecvMessage,
    ) -> Result<(), RequestError<I2sError>> {
        #[cfg(feature = "sdcard")]
        drop_sd(self.sd_kind);
        self.sd_kind = SdKind::None;
        Ok(())
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

        // Stop + drop any current SD source first (frees its file handles).
        drop_sd(self.sd_kind);
        self.sd_kind = SdKind::None;

        let source = sd::SdFileSource::open(
            Rp235xSdcard::from(SDCARD.get_task_id()),
            &name_buf[..n],
        )
        .ok_or(I2sError::OpenFailed)?;

        // Construct the decoder in its static slot. FLAC uses the in-place
        // constructor (placement-new: the ~37 KiB struct never exists on this
        // stack); WAV is small enough to move by value.
        let (kind, rate) = if decoder::is_flac_name(&name_buf[..n]) {
            // SAFETY: slot is not live (sd_kind None, dropped above); on Err
            // the slot stays uninitialised and kind stays None.
            unsafe {
                let slot =
                    (*core::ptr::addr_of_mut!(FLAC_SLOT)).as_mut_ptr();
                decoder::FlacDecoder::new_at(slot, source)
                    .map_err(|_| I2sError::BadFile)?;
                (
                    SdKind::Flac,
                    decoder::Decoder::sample_rate(&*slot),
                )
            }
        } else {
            let w = decoder::WavDecoder::new(source)
                .map_err(|_| I2sError::BadFile)?;
            let rate = decoder::Decoder::sample_rate(&w);
            // SAFETY: single-threaded; see WAV_SLOT.
            unsafe {
                *core::ptr::addr_of_mut!(WAV_SLOT) = Some(w);
            }
            (SdKind::Wav, rate)
        };

        // The DAC's PLL only locks for fs >= 32 kHz at BCK = 32*fs: reject
        // rather than clamp (a clamped rate would pitch-shift the audio).
        if !(RATE_MIN_HZ..=RATE_MAX_HZ).contains(&rate) {
            drop_sd(kind);
            return Err(I2sError::BadRate.into());
        }

        // Retune the whole mixer to the file's rate (tones re-derive their
        // DDS steps from `self.rate`). The timer tick does all further work;
        // this op returns immediately.
        let p = unsafe { rp235x_pac::Peripherals::steal() };
        if rate != self.rate {
            abort_dma(&p);
            set_rate(&p, rate);
            arm_dma(&p);
            self.rate = rate;
            self.next_fill = 0;
        }
        self.sd_kind = kind;
        Ok(pack_play_reply(rate, 0, 0, 0))
    }

    #[cfg(not(feature = "sdcard"))]
    fn list_files(
        &mut self,
        _: &RecvMessage,
        _out: LenLimit<Leased<idol_runtime::W, [u8]>, 768>,
    ) -> Result<u32, RequestError<I2sError>> {
        Err(I2sError::OpenFailed.into())
    }

    #[cfg(feature = "sdcard")]
    fn list_files(
        &mut self,
        _: &RecvMessage,
        out: LenLimit<Leased<idol_runtime::W, [u8]>, 768>,
    ) -> Result<u32, RequestError<I2sError>> {
        let mut buf = [0u8; 768];
        let cap = out.len().min(768);
        let (n, count) = sd::SdFileSource::list(
            Rp235xSdcard::from(SDCARD.get_task_id()),
            &mut buf[..cap],
        )
        .ok_or(I2sError::OpenFailed)?;
        out.write_range(0..n, &buf[..n])
            .map_err(|()| RequestError::went_away())?;
        Ok(((n as u32) << 8) | count.min(255))
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
        // Silence the tone sources; the mixer keeps streaming (zeros when
        // nothing is enabled -- the DAC's zero-data detect analog-mutes).
        self.tone_a.enable = false;
        self.tone_b.enable = false;
        Ok(())
    }
}

impl idol_runtime::NotificationHandler for ServerImpl {
    fn current_notification_mask(&self) -> u32 {
        notifications::TIMER_MASK
    }
    fn handle_notification(&mut self, bits: userlib::NotificationBits) {
        if bits.check_notification_mask(notifications::TIMER_MASK) {
            let p = unsafe { rp235x_pac::Peripherals::steal() };
            self.tick(&p);
            sys_set_timer(
                Some(sys_get_timer().now + TICK_MS),
                notifications::TIMER_MASK,
            );
        }
    }
}

#[unsafe(export_name = "main")]
fn main() -> ! {
    let p = unsafe { rp235x_pac::Peripherals::steal() };
    // MusicPi control pin: XSMT/amp-EN high = DAC unmuted + headphone amp on.
    // (Amp gain is set by the on-board DIP switches; see the note above.)
    sio_output(&p, XSMT, true);
    pio_init(&p);
    pio1_init(&p);
    set_rate(&p, DEFAULT_RATE_HZ);

    let mut server = ServerImpl {
        // Boot tone: source A at 440 Hz (the liveness signal, as ever).
        tone_a: Src {
            enable: true,
            gain: 16384,
            phase: 0,
            hz: DEFAULT_TONE_HZ,
        },
        tone_b: Src { enable: false, gain: 16384, phase: 0, hz: 880 },
        sd_gain: 24576,
        sd_kind: SdKind::None,
        duck: Duck {
            enable: false,
            threshold: 4096,
            attack_ms: 30,
            release_ms: 400,
            floor: 4096,
            env: 0,
            gain: 32767,
        },
        rate: DEFAULT_RATE_HZ,
        next_fill: 0,
        #[cfg(feature = "simadc")]
        cap_rd: 0,
        vu_bus: 0,
        vu_sd: 0,
        vu_out: 0,
    };
    // Start the capture read cursor a full chunk + margin behind the write
    // pointer so a whole chunk read never crosses it.
    #[cfg(feature = "simadc")]
    {
        let a = p.DMA.ch(DMA_RX_A).ch_write_addr().read().bits();
        let wr = ((a.wrapping_sub(core::ptr::addr_of!(CAP_A) as u32)
            as usize)
            / 4)
            % CAP_FRAMES;
        server.cap_rd =
            (wr + CAP_FRAMES - PLAY_CHUNK_FRAMES - 128) % CAP_FRAMES;
    }
    // Prime the whole ring, then arm the DMA and the mixer tick.
    for c in 0..PLAY_CHUNKS {
        server.mix_chunk(&p, c);
    }
    arm_dma(&p);
    sys_set_timer(
        Some(sys_get_timer().now + TICK_MS),
        notifications::TIMER_MASK,
    );

    let mut incoming = [0u8; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    use drv_rp235x_i2s_api::I2sError;
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}

include!(concat!(env!("OUT_DIR"), "/notifications.rs"));
