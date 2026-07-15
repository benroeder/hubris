// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! ST7789V TFT driver for the SB Components MusicPi HAT (RP2350 / Pico 2).
//!
//! 1.14" 135x240 panel, driven in landscape (240 wide x 135 high) over SPI1
//! (PL022, 25 MHz, write-only): CLK=GP14, MOSI=GP15 (funcsel 1); D/C=GP6,
//! CS=GP13, RST=GP12, backlight=GP7 as SIO outputs. Init parameters verified
//! against SB's own Demo_Display.py: the panel's RAM window sits at offset
//! (+40, +53) in landscape inside the controller's 240x320 RAM, colours are
//! RGB565 (COLMOD 0x55), and 135x240 panels need inversion ON (INVON).
//! See docs/musicpi-research/musicpi.md and the datasheet caveat there.
//!
//! No framebuffer: fill/rect/text stream pixels through a CASET/RASET window
//! + RAMWR, so the task needs only a tiny stack. Text is the public-domain
//! font8x8 scaled 2x (16x16 glyph cells; 15 columns x 8 rows).

#![no_std]
#![no_main]

use drv_rp235x_st7789_api::St7789Error;
use drv_rp235x_sys_api::{self as sys_api, Rp235xSys};
use idol_runtime::{Leased, LenLimit, RequestError, R};
use userlib::{task_slot, RecvMessage};

task_slot!(SYS, sys);

// ---- Pins (Pico 2 GPIO numbers, MusicPi wiring) ------------------------------
const SCK: u32 = 14;
const MOSI: u32 = 15;
/// Data/command select: low = command byte, high = pixel/parameter data.
const DC: u32 = 6;
const CS: u32 = 13;
const RST: u32 = 12;
/// Backlight enable (drives a BC817 transistor; high = on).
const BL: u32 = 7;
/// funcsel routing a GPIO to the SPI function / to SIO on the RP2350.
const FUNCSEL_SPI: u8 = 1;
const FUNCSEL_SIO: u8 = 5;

// ---- Panel geometry (landscape, MADCTL 0x60) ---------------------------------
const WIDTH: u16 = 240;
const HEIGHT: u16 = 135;
/// RAM-window offsets of the 1.14" panel inside the ST7789V's 240x320 RAM,
/// in landscape orientation (Demo_Display.py: rowstart=40, colstart=53).
const XOFF: u16 = 40;
const YOFF: u16 = 53;

// ---- Text (font8x8 scaled 2x) -------------------------------------------------
const CELL: u16 = 16;
const TEXT_COLS: u8 = (WIDTH / CELL) as u8; // 15
const TEXT_ROWS: u8 = (HEIGHT / CELL) as u8; // 8

// ---- ST7789 commands ----------------------------------------------------------
const CMD_SLPOUT: u8 = 0x11;
const CMD_NORON: u8 = 0x13;
const CMD_INVON: u8 = 0x21;
const CMD_DISPON: u8 = 0x29;
const CMD_CASET: u8 = 0x2a;
const CMD_RASET: u8 = 0x2b;
const CMD_RAMWR: u8 = 0x2c;
const CMD_MADCTL: u8 = 0x36;
const CMD_COLMOD: u8 = 0x3a;
/// MADCTL for landscape: row/column exchange + mirror X (the adafruit
/// rotation=90 mapping for this panel).
const MADCTL_LANDSCAPE: u8 = 0x60;
/// COLMOD 16-bit RGB565.
const COLMOD_RGB565: u8 = 0x55;

// ---- PL022 clocking (SPI clock = clk_peri / (CPSDVSR * (1 + SCR))) -----------
/// 150 MHz / 6 = 25 MHz -- same recipe as the sdcard driver; comfortably
/// inside the ST7789V's write timing.
const CPSDVSR: u8 = 6;
const SCR: u8 = 0;
/// 8-bit frames (DSS field value).
const DSS_8BIT: u8 = 7;

/// ~1 ms of busy-wait at 150 MHz (hardware settling delays during init only).
const MS: u32 = 150_000;

struct ServerImpl {
    spi: rp235x_pac::SPI1,
    sio: rp235x_pac::SIO,
}

impl ServerImpl {
    fn pin_set(&self, pin: u32, high: bool) {
        if high {
            self.sio.gpio_out_set().write(|w| unsafe { w.bits(1 << pin) });
        } else {
            self.sio.gpio_out_clr().write(|w| unsafe { w.bits(1 << pin) });
        }
    }

    /// Push one byte into the PL022 TX FIFO (blocking on FIFO space).
    fn spi_byte(&self, b: u8) {
        while !self.spi.sspsr().read().tnf().bit_is_set() {}
        self.spi
            .sspdr()
            .write(|w| unsafe { w.data().bits(b as u16) });
    }

    /// Wait for the PL022 to finish clocking everything out.
    fn spi_drain(&self) {
        while self.spi.sspsr().read().bsy().bit_is_set() {}
        // Drop anything the (unused) RX side latched so the FIFO never fills,
        // and clear the sticky receive-overrun status it accumulates during
        // long pixel streams -- a latched ROR would look like a live fault to
        // anyone reading the status registers later.
        while self.spi.sspsr().read().rne().bit_is_set() {
            let _ = self.spi.sspdr().read();
        }
        self.spi.sspicr().write(|w| w.roric().clear_bit_by_one());
    }

    /// Send a command byte, leaving CS asserted for its data bytes.
    fn cmd(&self, c: u8) {
        self.pin_set(CS, false);
        self.pin_set(DC, false);
        self.spi_byte(c);
        self.spi_drain();
        self.pin_set(DC, true);
    }

    /// End the current command sequence (deassert CS).
    fn done(&self) {
        self.spi_drain();
        self.pin_set(CS, true);
    }

    fn data(&self, bytes: &[u8]) {
        for &b in bytes {
            self.spi_byte(b);
        }
    }

    /// Open a RAMWR window for the rectangle. CONTRACT: the caller has already
    /// clipped to the panel (as `fill_rect` does) -- the release build has no
    /// overflow checks, so unclipped args would wrap silently.
    fn window(&self, x: u16, y: u16, w: u16, h: u16) {
        debug_assert!(
            w > 0 && h > 0 && x + w <= WIDTH && y + h <= HEIGHT,
            "window args must be pre-clipped"
        );
        let x0 = x + XOFF;
        let x1 = x + w - 1 + XOFF;
        let y0 = y + YOFF;
        let y1 = y + h - 1 + YOFF;
        self.cmd(CMD_CASET);
        self.data(&[
            (x0 >> 8) as u8,
            x0 as u8,
            (x1 >> 8) as u8,
            x1 as u8,
        ]);
        self.done();
        self.cmd(CMD_RASET);
        self.data(&[
            (y0 >> 8) as u8,
            y0 as u8,
            (y1 >> 8) as u8,
            y1 as u8,
        ]);
        self.done();
        self.cmd(CMD_RAMWR);
        // Caller streams w*h pixels, then calls done().
    }

    /// Fill a clipped rectangle with one colour.
    fn fill_rect(&self, x: u16, y: u16, w: u16, h: u16, color: u16) {
        if x >= WIDTH || y >= HEIGHT || w == 0 || h == 0 {
            return;
        }
        let w = w.min(WIDTH - x);
        let h = h.min(HEIGHT - y);
        self.window(x, y, w, h);
        let hi = (color >> 8) as u8;
        let lo = color as u8;
        for _ in 0..(w as u32 * h as u32) {
            self.spi_byte(hi);
            self.spi_byte(lo);
        }
        self.done();
    }

    /// Render one 8x8 glyph scaled 2x at a 16x16 cell position. Codes outside
    /// the font (>= 128) render as a blank cell rather than skipping -- a skip
    /// would leave the cell's previous pixels behind while still advancing the
    /// cursor.
    fn glyph(&self, col: u8, row: u8, ch: u8, color: u16) {
        let g: [u8; 8] = *font8x8::legacy::BASIC_LEGACY
            .get(ch as usize)
            .unwrap_or(&[0u8; 8]);
        let x = col as u16 * CELL;
        let y = row as u16 * CELL;
        self.window(x, y, CELL, CELL);
        for py in 0..CELL {
            let bits = g[(py / 2) as usize];
            for px in 0..CELL {
                // font8x8: bit 0 is the leftmost pixel of the row.
                let on = bits >> (px / 2) & 1 != 0;
                let c = if on { color } else { 0 };
                self.spi_byte((c >> 8) as u8);
                self.spi_byte(c as u8);
            }
        }
        self.done();
    }
}

impl idl::InOrderRp235xSt7789Impl for ServerImpl {
    fn fill(
        &mut self,
        _: &RecvMessage,
        color: u16,
    ) -> Result<(), RequestError<St7789Error>> {
        self.fill_rect(0, 0, WIDTH, HEIGHT, color);
        Ok(())
    }

    fn rect(
        &mut self,
        _: &RecvMessage,
        x: u16,
        y: u16,
        w: u16,
        h: u16,
        color: u16,
    ) -> Result<(), RequestError<St7789Error>> {
        if x >= WIDTH || y >= HEIGHT {
            return Err(St7789Error::BadArg.into());
        }
        self.fill_rect(x, y, w, h, color);
        Ok(())
    }

    fn bars(&mut self, _: &RecvMessage) -> Result<(), RequestError<St7789Error>> {
        // The classic 8-colour bars: white, yellow, cyan, green, magenta,
        // red, blue, black (RGB565).
        const BARS: [u16; 8] = [
            0xffff, 0xffe0, 0x07ff, 0x07e0, 0xf81f, 0xf800, 0x001f, 0x0000,
        ];
        for (i, &c) in BARS.iter().enumerate() {
            let x = i as u16 * (WIDTH / 8);
            self.fill_rect(x, 0, WIDTH / 8, HEIGHT, c);
        }
        Ok(())
    }

    fn text(
        &mut self,
        _: &RecvMessage,
        col: u8,
        row: u8,
        color: u16,
        chars: LenLimit<Leased<R, [u8]>, 48>,
    ) -> Result<(), RequestError<St7789Error>> {
        if col >= TEXT_COLS || row >= TEXT_ROWS {
            return Err(St7789Error::BadArg.into());
        }
        let n = chars.len().min(48);
        let mut buf = [0u8; 48];
        chars.read_range(0..n, &mut buf[..n])
            .map_err(|()| RequestError::went_away())?;
        let (mut c, mut r) = (col, row);
        for &ch in &buf[..n] {
            self.glyph(c, r, ch, color);
            c += 1;
            if c >= TEXT_COLS {
                c = 0;
                r += 1;
                if r >= TEXT_ROWS {
                    break;
                }
            }
        }
        Ok(())
    }

    fn backlight(
        &mut self,
        _: &RecvMessage,
        on: u8,
    ) -> Result<(), RequestError<St7789Error>> {
        self.pin_set(BL, on != 0);
        Ok(())
    }
}

impl idol_runtime::NotificationHandler for ServerImpl {
    fn current_notification_mask(&self) -> u32 {
        0
    }
    fn handle_notification(&mut self, _bits: userlib::NotificationBits) {}
}

/// One-time hardware setup: pins, PL022, panel reset + init sequence.
fn setup(p: &rp235x_pac::Peripherals) {
    let sys = Rp235xSys::from(SYS.get_task_id());
    sys.leave_reset(sys_api::SPI1);

    // SCK/MOSI -> SPI1 (funcsel 1). Output pads: drive enabled, no isolation.
    for pin in [SCK, MOSI] {
        p.PADS_BANK0.gpio(pin as usize).modify(|_, w| {
            w.od().clear_bit();
            w.iso().clear_bit();
            w.ie().set_bit()
        });
        p.IO_BANK0
            .gpio(pin as usize)
            .gpio_ctrl()
            .modify(|_, w| unsafe { w.funcsel().bits(FUNCSEL_SPI) });
    }
    // Control pins -> SIO outputs. CS/RST idle high, D/C high, backlight off
    // until the panel is initialised (avoids flashing garbage at power-on).
    for (pin, level) in [(DC, true), (CS, true), (RST, true), (BL, false)] {
        p.PADS_BANK0.gpio(pin as usize).modify(|_, w| {
            w.od().clear_bit();
            w.iso().clear_bit();
            w.ie().set_bit()
        });
        p.IO_BANK0
            .gpio(pin as usize)
            .gpio_ctrl()
            .modify(|_, w| unsafe { w.funcsel().bits(FUNCSEL_SIO) });
        if level {
            p.SIO.gpio_out_set().write(|w| unsafe { w.bits(1 << pin) });
        } else {
            p.SIO.gpio_out_clr().write(|w| unsafe { w.bits(1 << pin) });
        }
        p.SIO.gpio_oe_set().write(|w| unsafe { w.bits(1 << pin) });
    }

    // PL022: 8-bit Motorola mode 0 at 25 MHz, then enable.
    let spi = &p.SPI1;
    spi.sspcr1().write(|w| w.sse().clear_bit());
    spi.sspcpsr().write(|w| unsafe { w.cpsdvsr().bits(CPSDVSR) });
    spi.sspcr0().write(|w| unsafe {
        w.dss().bits(DSS_8BIT);
        w.spo().clear_bit();
        w.sph().clear_bit();
        w.scr().bits(SCR)
    });
    spi.sspcr1().write(|w| w.sse().set_bit());
}

/// Panel hardware reset + command init, then screen clear + backlight on.
fn panel_init(server: &ServerImpl) {
    // Hardware reset: >10 us low, then 120 ms for the controller to boot.
    server.pin_set(RST, false);
    cortex_m::asm::delay(MS); // 1 ms
    server.pin_set(RST, true);
    cortex_m::asm::delay(120 * MS);

    server.cmd(CMD_SLPOUT);
    server.done();
    cortex_m::asm::delay(120 * MS);

    server.cmd(CMD_COLMOD);
    server.data(&[COLMOD_RGB565]);
    server.done();
    server.cmd(CMD_MADCTL);
    server.data(&[MADCTL_LANDSCAPE]);
    server.done();
    // 135x240 ST7789 panels are wired inverted -- INVON gives true colours.
    server.cmd(CMD_INVON);
    server.done();
    server.cmd(CMD_NORON);
    server.done();
    server.cmd(CMD_DISPON);
    server.done();
    cortex_m::asm::delay(20 * MS);

    // Clear to black BEFORE the backlight comes on: the RAM powers up with
    // random contents and would flash noise otherwise.
    server.fill_rect(0, 0, WIDTH, HEIGHT, 0);
    server.pin_set(BL, true);
}

#[unsafe(export_name = "main")]
fn main() -> ! {
    let p = unsafe { rp235x_pac::Peripherals::steal() };
    setup(&p);
    let mut server = ServerImpl {
        spi: p.SPI1,
        sio: p.SIO,
    };
    panel_init(&server);

    let mut incoming = [0u8; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    use drv_rp235x_st7789_api::St7789Error;
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
