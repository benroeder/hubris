// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! microSD raw block-device driver server for the RP2350 (RP235x), SD SPI mode.
//!
//! RAW BLOCK DEVICE ONLY -- there is no filesystem here. The card is spoken to
//! over SPI1 (PL022) in the SD card's "SPI mode": 8-bit frames, Motorola SPI
//! mode 0 (CPOL=0 CPHA=0). Two Idol ops (`idl/rp235x-sdcard.idol`):
//! * `init` runs the standard SD SPI-mode power-on handshake
//!   (CMD0 -> CMD8 -> ACMD41 -> CMD58) and records the card kind;
//! * `read_block` reads one 512-byte block into a write lease (same lease
//!   idiom as the flash driver's `read`);
//! * `write_block` programs one 512-byte block from a read lease (CMD24
//!   single-block write; same read-lease idiom as the flash driver's
//!   `program`).
//!
//! Pins (assumed; confirmed on hardware at bring-up): SPI1 funcsel 1 on
//! SCK=GP10, MOSI/TX=GP11, MISO/RX=GP12. CS=GP13 is driven MANUALLY as a SIO
//! GPIO (funcsel 5), NOT the PL022 auto-CS -- an SD transaction holds CS low
//! across many byte exchanges, and the >=74-clock wake burst needs CS HIGH,
//! neither of which the PL022's per-frame auto-CS can express.
//!
//! NOTE: the SD init handshake and the SPI timing here are UNPROVEN on
//! hardware at the time of writing. `init` is an explicit op (not run in
//! `main`) so bring-up can retry and observe it.

#![no_std]
#![no_main]

use drv_rp235x_sdcard_api::{STATUS_CCS, STATUS_V2, SdError};
use drv_rp235x_sys_api::{self as sys_api, Rp235xSys};
use idol_runtime::{Leased, LenLimit, R, RequestError, W};
use userlib::{RecvMessage, task_slot};

task_slot!(SYS, sys);

// --- Pin map (named consts so hardware bring-up can retarget the mux) --------
/// SPI1 SCK.
const SCK: u32 = 10;
/// SPI1 MOSI / TX (controller out).
const MOSI: u32 = 11;
/// SPI1 MISO / RX (controller in).
const MISO: u32 = 12;
/// Chip-select, driven manually as a SIO GPIO (active LOW).
const CS: u32 = 13;
/// funcsel that routes a GPIO to the SPI function on the RP2350.
const FUNCSEL_SPI: u8 = 1;
/// funcsel that routes a GPIO to SIO on the RP2350.
const FUNCSEL_SIO: u8 = 5;

// --- PL022 clock dividers (SPI clock = clk_peri / (CPSDVSR * (1 + SCR))) ------
// clk_peri is 150 MHz.
/// INIT-speed prescale: 150 MHz / (254 * 2) = ~295 kHz (SD requires <400 kHz
/// during the power-on handshake).
const INIT_CPSDVSR: u8 = 254;
/// INIT-speed serial-clock-rate: divide-by-2 with CPSDVSR above.
const INIT_SCR: u8 = 1;
/// DATA-speed prescale: 150 MHz / 6 = 25 MHz (SCR = 0) -- the SD SPI-mode
/// default-speed ceiling. The card is clocked at this rate only after the
/// <400 kHz init handshake succeeds.
const DATA_CPSDVSR: u8 = 6;
/// DATA-speed serial-clock-rate.
const DATA_SCR: u8 = 0;
/// PL022 data-size select for 8-bit frames (DSS = N-1).
const DSS_8BIT: u8 = 0x7;

/// Bound on a single-byte FIFO spin. The PL022 completes an 8-bit frame in a
/// handful of SPI clocks; this cap only trips if SPI1 is not clocking, so we
/// return (possibly garbage) rather than wedging the server forever.
const SPIN_LIMIT: u32 = 1_000_000;
/// R1-response poll length: the card sends up to 8 0xFF bytes before R1.
const R1_POLLS: u32 = 8;
/// CMD0-into-idle retries.
const CMD0_RETRIES: u32 = 16;
/// ACMD41 ready-poll budget. One iteration is CMD55+CMD41 plus a ~1 ms delay.
/// The SD spec's power-up ceiling is 1 s, but some cards take longer, so allow
/// ~2 s of headroom.
const ACMD41_RETRIES: u32 = 2000;
/// clk_sys cycles for a ~1 ms inter-poll delay in the ACMD41 loop.
const MS_CYCLES: u32 = 150_000;
/// Data-start-token poll budget for `read_block` (each iteration is one byte
/// exchange; the card asserts the token within a few hundred bytes).
const TOKEN_POLLS: u32 = 100_000;
/// Busy-line poll budget for `write_block`. The card holds MISO low while it
/// programs the block, which can take many milliseconds. At DATA speed one
/// byte exchange is ~1.3 us, so this budget covers a couple of seconds --
/// comfortably above the SD spec's single-block write ceiling.
const WRITE_BUSY: u32 = 2_000_000;

/// One 512-byte SD block.
const BLOCK_LEN: usize = 512;
/// PL022 TX/RX FIFO depth -- how many byte transfers can be kept in flight when
/// pipelining a block transfer without overrunning the RX FIFO.
const FIFO_DEPTH: usize = 8;
/// SD data-start token that precedes a read data block.
const TOKEN_START: u8 = 0xFE;
/// Filler byte clocked out to read a byte in (MOSI held high).
const FILLER: u8 = 0xFF;

struct ServerImpl {
    spi: rp235x_pac::SPI1,
    sio: rp235x_pac::SIO,
    /// CCS: card uses block (SDHC/SDXC) rather than byte (SDSC) addressing.
    ccs: bool,
    /// Card answered CMD8 as SD version 2.
    v2: bool,
    /// A successful `init` has run.
    ready: bool,
}

impl ServerImpl {
    /// Drive CS low (select the card).
    fn cs_low(&self) {
        self.sio
            .gpio_out_clr()
            .write(|w| unsafe { w.bits(1 << CS) });
    }

    /// Drive CS high (deselect the card).
    fn cs_high(&self) {
        self.sio
            .gpio_out_set()
            .write(|w| unsafe { w.bits(1 << CS) });
    }

    /// Full-duplex byte exchange: clock `out` out on MOSI while capturing the
    /// byte shifted in on MISO. Spins are bounded so a dead SPI can't hang the
    /// server (and thus every client) forever.
    fn xfer(&self, out: u8) -> u8 {
        let mut s = 0u32;
        while self.spi.sspsr().read().tnf().bit_is_clear() {
            s += 1;
            if s > SPIN_LIMIT {
                break;
            }
        }
        self.spi
            .sspdr()
            .write(|w| unsafe { w.data().bits(out as u16) });
        let mut s = 0u32;
        while self.spi.sspsr().read().rne().bit_is_clear() {
            s += 1;
            if s > SPIN_LIMIT {
                return FILLER;
            }
        }
        self.spi.sspdr().read().data().bits() as u8
    }

    /// Clock in one byte (send 0xFF).
    fn read_byte(&self) -> u8 {
        self.xfer(FILLER)
    }

    /// Pipeline `n` byte transfers through the PL022's 8-deep FIFOs: keep the TX
    /// FIFO fed while draining RX, so the SPI clocks continuously instead of
    /// stalling a full poll-write-poll-read per byte. `tx_at(i)` supplies the
    /// i-th outbound byte; `rx_at(i, b)` receives the i-th inbound byte (both
    /// monomorphize away, so read and write share one FIFO invariant). Bytes in
    /// flight are capped at the RX FIFO depth so RX never overruns. Bounded by
    /// SPIN_LIMIT no-progress iterations (returns false) so a bus that stalls
    /// mid-transfer errors instead of wedging the server, like `xfer`.
    fn pipeline(
        &self,
        n: usize,
        tx_at: impl Fn(usize) -> u8,
        mut rx_at: impl FnMut(usize, u8),
    ) -> bool {
        let (mut tx, mut rx) = (0usize, 0usize);
        let mut spins = 0u32;
        while rx < n {
            let before = rx;
            while tx < n
                && tx - rx < FIFO_DEPTH
                && self.spi.sspsr().read().tnf().bit_is_set()
            {
                self.spi
                    .sspdr()
                    .write(|w| unsafe { w.data().bits(tx_at(tx) as u16) });
                tx += 1;
            }
            while rx < tx && self.spi.sspsr().read().rne().bit_is_set() {
                rx_at(rx, self.spi.sspdr().read().data().bits() as u8);
                rx += 1;
            }
            // Only iterations that drain nothing count toward the bound, so a
            // stalled bus (no RX progress) trips SPIN_LIMIT while normal
            // in-flight latency just resets the counter.
            if rx == before {
                spins += 1;
                if spins > SPIN_LIMIT {
                    return false;
                }
            } else {
                spins = 0;
            }
        }
        true
    }

    /// Set the PL022 SPI clock divider. SSE must be cleared before touching the
    /// clock registers (PL022), then re-enabled.
    fn set_clock(&self, cpsdvsr: u8, scr: u8) {
        self.spi.sspcr1().modify(|_, w| w.sse().clear_bit());
        self.spi
            .sspcpsr()
            .write(|w| unsafe { w.cpsdvsr().bits(cpsdvsr) });
        self.spi
            .sspcr0()
            .modify(|_, w| unsafe { w.scr().bits(scr) });
        self.spi.sspcr1().modify(|_, w| w.sse().set_bit());
    }

    /// Send a 6-byte SD command frame and return the R1 response byte (the
    /// first byte polled with bit7 clear, or 0xFF on timeout). Assumes CS is
    /// already low. CRC only matters for CMD0 (0x95) and CMD8 (0x87); 0x01 is
    /// a valid stand-in elsewhere (SPI-mode CRC is off by default).
    fn command(&self, cmd: u8, arg: u32, crc: u8) -> u8 {
        // A dummy byte before the command gives the card a clock edge to
        // finish any prior internal work (some cards need it).
        self.xfer(FILLER);
        self.xfer(0x40 | cmd);
        self.xfer((arg >> 24) as u8);
        self.xfer((arg >> 16) as u8);
        self.xfer((arg >> 8) as u8);
        self.xfer(arg as u8);
        self.xfer(crc);
        // Poll for R1: the first byte with bit7 == 0.
        let mut r = FILLER;
        for _ in 0..R1_POLLS {
            r = self.read_byte();
            if r & 0x80 == 0 {
                return r;
            }
        }
        r
    }

    /// Run the SD SPI-mode power-on handshake. Leaves CS high and the bus at
    /// DATA speed on success; records the card kind in `self`.
    fn run_init(&mut self) -> Result<u32, SdError> {
        self.ready = false;

        // 1. Wake the card: INIT clock, CS HIGH, >=74 clocks of 0xFF.
        self.set_clock(INIT_CPSDVSR, INIT_SCR);
        self.cs_high();
        for _ in 0..10 {
            self.xfer(FILLER);
        }

        // 2. CMD0: GO_IDLE_STATE -> R1 == 0x01 (idle). Retry a few times.
        self.cs_low();
        let mut idle = false;
        for _ in 0..CMD0_RETRIES {
            if self.command(0, 0, 0x95) == 0x01 {
                idle = true;
                break;
            }
        }
        if !idle {
            self.cs_high();
            return Err(SdError::Init);
        }

        // 3. CMD8: SEND_IF_COND (0x1AA = 2.7-3.6 V, check pattern 0xAA).
        //    R1==0x01 + echoed pattern -> SD v2. Illegal-command bit -> SD v1.
        let r = self.command(8, 0x0000_01AA, 0x87);
        let v2 = if r == 0x01 {
            let r7 = [
                self.read_byte(),
                self.read_byte(),
                self.read_byte(),
                self.read_byte(),
            ];
            if r7[2] == 0x01 && r7[3] == 0xAA {
                true
            } else {
                // Voltage/pattern mismatch: not a card we can talk to.
                self.cs_high();
                return Err(SdError::Init);
            }
        } else if r != 0xFF && r & 0x04 != 0 {
            // Illegal command: an SD v1 (or MMC) card. Proceed as v1. (0xFF is
            // the no-response value, which also has bit2 set -- exclude it so an
            // absent/misread card errors here instead of misreading it as v1.)
            false
        } else {
            self.cs_high();
            return Err(SdError::Init);
        };

        // 4. ACMD41 (CMD55 then CMD41) until the card leaves idle (R1 == 0x00).
        //    HCS (bit30) is only set for v2 to allow high-capacity cards.
        let hcs = if v2 { 0x4000_0000 } else { 0 };
        let mut ready = false;
        for _ in 0..ACMD41_RETRIES {
            // CMD55 (APP_CMD) must return with the app-cmd bit; ignore its
            // exact value and just issue CMD41 next.
            self.command(55, 0, 0x01);
            let r = self.command(41, hcs, 0x01);
            if r == 0x00 {
                ready = true;
                break;
            }
            cortex_m::asm::delay(MS_CYCLES);
        }
        if !ready {
            self.cs_high();
            return Err(SdError::Timeout);
        }

        // 5. CMD58 (READ_OCR): CCS = OCR bit30 = bit6 of the first OCR byte.
        let ccs = {
            let r = self.command(58, 0, 0x01);
            if r != 0x00 {
                self.cs_high();
                return Err(SdError::Cmd);
            }
            let ocr = [
                self.read_byte(),
                self.read_byte(),
                self.read_byte(),
                self.read_byte(),
            ];
            ocr[0] & 0x40 != 0
        };
        // SDSC (byte addressing): fix the block length at 512 with CMD16.
        if !ccs {
            let r = self.command(16, BLOCK_LEN as u32, 0x01);
            if r != 0x00 {
                self.cs_high();
                return Err(SdError::Cmd);
            }
        }

        // 6. Deselect, one trailing clock byte, switch to DATA speed.
        self.cs_high();
        self.xfer(FILLER);
        self.set_clock(DATA_CPSDVSR, DATA_SCR);

        self.ccs = ccs;
        self.v2 = v2;
        self.ready = true;

        let mut status = 0u32;
        if ccs {
            status |= STATUS_CCS;
        }
        if v2 {
            status |= STATUS_V2;
        }
        Ok(status)
    }
}

impl idl::InOrderRp235xSdcardImpl for ServerImpl {
    fn init(&mut self, _: &RecvMessage) -> Result<u32, RequestError<SdError>> {
        Ok(self.run_init()?)
    }

    fn read_block(
        &mut self,
        _: &RecvMessage,
        block: u32,
        dest: LenLimit<Leased<W, [u8]>, BLOCK_LEN>,
    ) -> Result<(), RequestError<SdError>> {
        if !self.ready {
            return Err(SdError::NotInitialized.into());
        }
        // SDHC/SDXC address by block; SDSC by byte. wrapping_mul keeps a debug
        // build from panicking on the (in-range) SDSC multiply.
        let addr = if self.ccs {
            block
        } else {
            block.wrapping_mul(BLOCK_LEN as u32)
        };

        self.cs_low();
        // CMD17: READ_SINGLE_BLOCK.
        if self.command(17, addr, 0x01) != 0x00 {
            self.cs_high();
            return Err(SdError::Cmd.into());
        }
        // Poll for the data-start token (0xFE). A byte with the top three bits
        // clear is an error token (0x0X); 0xFF means "still waiting".
        let mut got_token = false;
        for _ in 0..TOKEN_POLLS {
            let b = self.read_byte();
            if b == TOKEN_START {
                got_token = true;
                break;
            }
            if b != FILLER {
                self.cs_high();
                return Err(SdError::DataError.into());
            }
        }
        if !got_token {
            self.cs_high();
            return Err(SdError::Timeout.into());
        }

        // Read the 512 data bytes (FIFO-pipelined), then discard the 2 CRC.
        let mut buf = [0u8; BLOCK_LEN];
        if !self.pipeline(BLOCK_LEN, |_| FILLER, |i, b| buf[i] = b) {
            self.cs_high();
            return Err(SdError::Timeout.into());
        }
        self.read_byte();
        self.read_byte();

        self.cs_high();
        self.xfer(FILLER);

        // Copy out only as much as the caller's lease holds (LenLimit bounds it
        // to <=512), mirroring the flash driver -- a short lease should get its
        // bytes, not a spurious lease error.
        let n = dest.len().min(BLOCK_LEN);
        dest.write_range(0..n, &buf[..n])
            .map_err(|_| RequestError::went_away())?;
        Ok(())
    }

    fn write_block(
        &mut self,
        _: &RecvMessage,
        block: u32,
        src: LenLimit<Leased<R, [u8]>, BLOCK_LEN>,
    ) -> Result<(), RequestError<SdError>> {
        if !self.ready {
            return Err(SdError::NotInitialized.into());
        }

        // Pull the client's bytes into a local block buffer, zero-padding a
        // short lease out to the full 512 (mirrors the flash driver's
        // read-lease `program`).
        let mut buf = [0u8; BLOCK_LEN];
        let n = src.len().min(BLOCK_LEN);
        src.read_range(0..n, &mut buf[..n])
            .map_err(|_| RequestError::went_away())?;

        // SDHC/SDXC address by block; SDSC by byte (same rule as read_block).
        let addr = if self.ccs {
            block
        } else {
            block.wrapping_mul(BLOCK_LEN as u32)
        };

        self.cs_low();
        // CMD24: WRITE_BLOCK.
        if self.command(24, addr, 0x01) != 0x00 {
            self.cs_high();
            return Err(SdError::Cmd.into());
        }

        // One gap byte, the data-start token, the 512-byte payload, then two
        // dummy CRC bytes (SPI-mode CRC is off by default).
        self.xfer(FILLER);
        self.xfer(TOKEN_START);
        if !self.pipeline(BLOCK_LEN, |i| buf[i], |_, _| {}) {
            self.cs_high();
            return Err(SdError::Timeout.into());
        }
        self.xfer(FILLER);
        self.xfer(FILLER);

        // Data-response byte: the card may clock out 0xFF wait bytes first, so
        // poll (bounded) for the real response. Low five bits == 0x05 means the
        // block was accepted; anything else is a CRC/write error.
        let mut resp = FILLER;
        for _ in 0..R1_POLLS {
            resp = self.read_byte();
            if resp != FILLER {
                break;
            }
        }
        if resp & 0x1F != 0x05 {
            self.cs_high();
            return Err(SdError::DataError.into());
        }

        // The card pulls MISO low (0x00) while it programs the block. Wait for
        // busy to ASSERT before waiting for it to release: a stalled SPI makes
        // xfer() return 0xFF, so requiring a 0x00 first stops a dead bus from
        // being misread as an instant completion (it never sees busy -> Timeout,
        // not a false Ok). A single-block program always holds busy for ms.
        let mut busy_seen = false;
        let mut released = false;
        for _ in 0..WRITE_BUSY {
            if self.read_byte() == 0x00 {
                busy_seen = true;
            } else if busy_seen {
                released = true;
                break;
            }
        }
        if !(busy_seen && released) {
            self.cs_high();
            return Err(SdError::Timeout.into());
        }

        self.cs_high();
        self.xfer(FILLER);
        Ok(())
    }
}

impl idol_runtime::NotificationHandler for ServerImpl {
    fn current_notification_mask(&self) -> u32 {
        0
    }
    fn handle_notification(&mut self, _bits: userlib::NotificationBits) {}
}

/// One-time hardware setup: route the SPI1 pins + the manual CS, and configure
/// SPI1 as an 8-bit mode-0 controller at INIT speed (SSE enabled). Does NOT run
/// the SD handshake -- that is the explicit `init` op.
fn setup(p: &rp235x_pac::Peripherals) {
    // SCK/MOSI/MISO -> SPI1 (funcsel 1). Enable the input buffer on all (MISO
    // needs it to read; harmless on the outputs).
    for pin in [SCK, MOSI, MISO] {
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
    // Pull MISO up: SD DAT/CMD lines idle high, and a floating MISO otherwise
    // reads as a spurious 0x00 before the card drives it.
    p.PADS_BANK0.gpio(MISO as usize).modify(|_, w| {
        w.pue().set_bit();
        w.pde().clear_bit()
    });

    // CS -> SIO output, idle HIGH (deselected). Set the level before enabling
    // the output driver so the line never glitches low.
    p.PADS_BANK0.gpio(CS as usize).modify(|_, w| {
        w.od().clear_bit();
        w.iso().clear_bit();
        w.ie().set_bit()
    });
    p.IO_BANK0
        .gpio(CS as usize)
        .gpio_ctrl()
        .modify(|_, w| unsafe { w.funcsel().bits(FUNCSEL_SIO) });
    p.SIO.gpio_out_set().write(|w| unsafe { w.bits(1 << CS) });
    p.SIO.gpio_oe_set().write(|w| unsafe { w.bits(1 << CS) });

    // SPI1: configure with SSE off, then enable. 8-bit Motorola SPI mode 0
    // (SPO=SPH=0) at INIT speed (~295 kHz).
    let spi = &p.SPI1;
    spi.sspcr1().write(|w| w.sse().clear_bit());
    spi.sspcpsr()
        .write(|w| unsafe { w.cpsdvsr().bits(INIT_CPSDVSR) });
    spi.sspcr0().write(|w| unsafe {
        w.dss().bits(DSS_8BIT);
        w.spo().clear_bit();
        w.sph().clear_bit();
        w.scr().bits(INIT_SCR)
    });
    // Controller (MS=0), loopback off, enable.
    spi.sspcr1().write(|w| {
        w.ms().clear_bit();
        w.lbm().clear_bit();
        w.sse().set_bit()
    });
}

#[export_name = "main"]
fn main() -> ! {
    // Bring SPI1 out of reset via the sys server before touching its registers.
    let sys = Rp235xSys::from(SYS.get_task_id());
    sys.leave_reset(sys_api::SPI1);

    let p = unsafe { rp235x_pac::Peripherals::steal() };
    setup(&p);

    let mut server = ServerImpl {
        spi: p.SPI1,
        sio: p.SIO,
        ccs: false,
        v2: false,
        ready: false,
    };
    let mut incoming = [0u8; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    use drv_rp235x_sdcard_api::SdError;
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
