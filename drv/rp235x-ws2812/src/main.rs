// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! WS2812 (NeoPixel) RGB LED driver server for the RP2350 (RP235x).
//!
//! Drives a single WS2812 pixel on GP22 via PIO0 SM0 behind an Idol interface
//! (`idl/rp235x-ws2812.idol`): `set(grb)` clocks a 24-bit G<<16 | R<<8 | B
//! colour out at the WS2812's 800 kHz bit rate.
//!
//! The PIO program is the canonical pico-sdk `ws2812.pio` (4 instructions,
//! side-set 1 on GP22, timing T1=2 T2=5 T3=3 -> 10 SM cycles per bit). At an
//! 8 MHz SM clock that gives the 1.25 us bit period WS2812 expects. clk_sys is
//! 150 MHz, so the clock divider is 150 / 8 = 18.75 (int 18, frac 192/256).
//!
//! GP22 is routed to PIO0 with funcsel 6 (RP2350: PIO0=6, PIO1=7, PIO2=8).
//! PIO0 is brought out of reset by the privileged pre-kernel `main` (it also
//! runs the PIO0 echo self-test), so this task needs only `pio0`, `io_bank0`,
//! `pads_bank0`, and `sio`.

#![no_std]
#![no_main]

use drv_rp235x_ws2812_api::Ws2812Error;
use idol_runtime::RequestError;
use userlib::RecvMessage;

/// GPIO the WS2812 data line lives on.
const PIN: u8 = 22;
/// funcsel that routes a GPIO to PIO0 on the RP2350.
const FUNCSEL_PIO0: u8 = 6;

/// Canonical pico-sdk WS2812 program (side-set 1 on GP22, T1=2 T2=5 T3=3):
///   0: `out x, 1   side 0 [2]`
///   1: `jmp !x, 3  side 1 [1]`
///   2: `jmp 0      side 1 [4]`
///   3: `nop        side 0 [4]`
const WS2812: [u16; 4] = [0x6221, 0x1123, 0x1400, 0xa422];

/// `set pindirs, 1` (SET dst=PINDIRS, data=1) run as an immediate exec to make
/// GP22 a PIO output. Uses the SM's SET pin mapping (set_base = 22).
const SET_PINDIRS_OUT: u16 = 0xe081;

/// Integer part of the clk_sys/8 MHz divider (150 MHz / 8 MHz = 18.75).
const DIV_INT: u16 = 18;
/// Fractional part: 0.75 * 256 = 192.
const DIV_FRAC: u8 = 192;

/// clk_sys cycles to hold the data line low for the WS2812 reset/latch. The
/// part needs >50 us; 10_000 cycles at 150 MHz is ~67 us. This is a hardware
/// timing gap, not a scheduling delay, so it's a busy-wait rather than a sleep.
const RESET_CYCLES: u32 = 10_000;

struct ServerImpl {
    pio: rp235x_pac::PIO0,
}

impl idl::InOrderRp235xWs2812Impl for ServerImpl {
    fn set(
        &mut self,
        _: &RecvMessage,
        grb: u32,
    ) -> Result<(), RequestError<Ws2812Error>> {
        // Wait for the SM0 TX FIFO to have space (txfull bit 0 clear). Bound the
        // spin: if the SM is not clocking, the 4-deep FIFO never drains, so give
        // up rather than wedge the server (and every client) forever. The SM is
        // enabled + fed an 8 MHz clock in setup(), so this bound is only reached
        // if PIO0 was never brought out of reset.
        let mut spins = 0u32;
        while self.pio.fstat().read().txfull().bits() & 1 != 0 {
            spins += 1;
            if spins > 1_000_000 {
                return Err(Ws2812Error::Stalled.into());
            }
        }
        // Left-justify the 24-bit GRB value for the MSB-first 24-bit autopull.
        self.pio.txf(0).write(|w| unsafe { w.bits(grb << 8) });
        Ok(())
    }
}

impl idol_runtime::NotificationHandler for ServerImpl {
    fn current_notification_mask(&self) -> u32 {
        0
    }
    fn handle_notification(&mut self, _bits: userlib::NotificationBits) {}
}

/// One-time hardware setup: route GP22 to PIO0, load the WS2812 program, and
/// configure + enable SM0.
fn setup(p: &rp235x_pac::Peripherals) {
    // Pad: drive enabled (clear output-disable + isolation), input-enable set,
    // full drive strength -- mirrors the cyw43 PIO pin setup.
    p.PADS_BANK0.gpio(PIN as usize).modify(|_, w| {
        w.od().clear_bit();
        w.iso().clear_bit();
        w.ie().set_bit();
        unsafe { w.drive().bits(3) }
    });
    // Route the GPIO to PIO0.
    p.IO_BANK0
        .gpio(PIN as usize)
        .gpio_ctrl()
        .modify(|_, w| unsafe { w.funcsel().bits(FUNCSEL_PIO0) });

    let pio = &p.PIO0;
    // Disable all state machines before touching config.
    pio.ctrl().modify(|_, w| unsafe { w.sm_enable().bits(0) });
    // Load the 4-instruction program at offset 0.
    for (i, insn) in WS2812.iter().enumerate() {
        pio.instr_mem(i).write(|w| unsafe { w.bits(*insn as u32) });
    }

    let sm = pio.sm(0);
    // 8 MHz SM clock: 150 MHz / 18.75.
    sm.sm_clkdiv()
        .write(|w| unsafe { w.int().bits(DIV_INT).frac().bits(DIV_FRAC) });
    // MSB-first out (shift left), autopull every 24 bits.
    sm.sm_shiftctrl().modify(|_, w| unsafe {
        w.out_shiftdir().clear_bit();
        w.autopull().set_bit();
        w.pull_thresh().bits(24)
    });
    // GP22 is the single side-set pin, the single OUT pin, and the SET pin
    // (so `set pindirs` below targets it).
    sm.sm_pinctrl().modify(|_, w| unsafe {
        w.sideset_count().bits(1);
        w.sideset_base().bits(PIN);
        w.out_base().bits(PIN);
        w.out_count().bits(1);
        w.set_base().bits(PIN);
        w.set_count().bits(1)
    });
    // The 4-instruction program wraps 0..3.
    sm.sm_execctrl()
        .modify(|_, w| unsafe { w.wrap_top().bits(3).wrap_bottom().bits(0) });
    // Make GP22 a PIO output via an immediate `set pindirs, 1`.
    sm.sm_instr()
        .write(|w| unsafe { w.sm0_instr().bits(SET_PINDIRS_OUT) });
    // Enable SM0. With an empty FIFO it stalls on the `out` instruction holding
    // the line low (side 0).
    pio.ctrl().modify(|_, w| unsafe { w.sm_enable().bits(1) });
    // Blank the pixel. GP22 floats (pindir=0) between the funcsel routing above
    // and the `set pindirs` output enable, so the WS2812 can latch a garbage
    // colour during boot. Clearing it needs a >50 us low (reset) BEFORE the 0
    // frame -- otherwise the pixel treats the lone word as data for a second
    // (absent) pixel and keeps the garbage. The enabled SM holds the line low,
    // so busy-wait one reset period, then clock the 0 frame to latch off.
    cortex_m::asm::delay(RESET_CYCLES);
    pio.txf(0).write(|w| unsafe { w.bits(0) });
}

#[export_name = "main"]
fn main() -> ! {
    let p = unsafe { rp235x_pac::Peripherals::steal() };
    setup(&p);
    let mut server = ServerImpl { pio: p.PIO0 };
    let mut incoming = [0u8; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    use drv_rp235x_ws2812_api::Ws2812Error;
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
