// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! SPI0 (PL022) driver server for the RP2350 (RP235x).
//!
//! Minimal full-duplex controller behind an Idol interface (`idl/rp235x-spi.idol`):
//! `exchange` clocks out `tx` while capturing `rx`. Brings SPI0 out of reset via the
//! sys server. Runs unprivileged with `uses = ["spi0"]`.
//!
//! NOTE: configured with the PL022 internal loopback (LBM) enabled, so `exchange`
//! echoes without external wiring -- this validates the clock/FIFO/shift datapath
//! during bring-up. Driving a real external device additionally needs the SCK/TX/RX
//! (and CS) pins muxed to the SPI function and LBM cleared.

#![no_std]
#![no_main]

use core::convert::Infallible;
use drv_rp235x_sys_api::{self as sys_api, Rp235xSys};
use idol_runtime::{Leased, LenLimit, R, RequestError, W};
use userlib::{RecvMessage, task_slot};

task_slot!(SYS, sys);

/// PL022 data-size select for 8-bit frames (DSS = N-1).
const DSS_8BIT: u8 = 0x7;
/// Clock prescale: clk_peri (150 MHz) / 100 = 1.5 MHz SPI clock (SCR = 0).
const CPSDVSR: u8 = 100;

struct ServerImpl {
    spi: rp235x_pac::SPI0,
}

impl idl::InOrderRp235xSpiImpl for ServerImpl {
    fn exchange(
        &mut self,
        _: &RecvMessage,
        tx: LenLimit<Leased<R, [u8]>, 256>,
        rx: LenLimit<Leased<W, [u8]>, 256>,
    ) -> Result<usize, RequestError<Infallible>> {
        let n = tx.len().min(rx.len());
        for i in 0..n {
            let mut b = [0u8; 1];
            tx.read_range(i..i + 1, &mut b)
                .map_err(|_| RequestError::went_away())?;

            // Wait for TX FIFO space, push the byte.
            while self.spi.sspsr().read().tnf().bit_is_clear() {}
            self.spi
                .sspdr()
                .write(|w| unsafe { w.data().bits(b[0] as u16) });

            // Wait for the RX FIFO to fill, capture the shifted-in byte.
            while self.spi.sspsr().read().rne().bit_is_clear() {}
            let r = self.spi.sspdr().read().data().bits() as u8;
            rx.write_range(i..i + 1, &[r])
                .map_err(|_| RequestError::went_away())?;
        }
        Ok(n)
    }

    fn set_role(
        &mut self,
        _: &RecvMessage,
        peripheral: u8,
    ) -> Result<(), RequestError<Infallible>> {
        // SSE must be cleared before changing MS/LBM (PL022). Then re-enable
        // as controller (MS=0) or peripheral (MS=1), loopback off either way.
        self.spi.sspcr1().modify(|_, w| w.sse().clear_bit());
        self.spi.sspcr1().write(|w| {
            w.ms().bit(peripheral != 0);
            w.lbm().clear_bit();
            w.sse().set_bit()
        });
        Ok(())
    }

    fn load_tx(
        &mut self,
        _: &RecvMessage,
        data: LenLimit<Leased<R, [u8]>, 8>,
    ) -> Result<usize, RequestError<Infallible>> {
        // Peripheral role: preload the TX FIFO (depth 8) with the bytes the
        // far controller will clock out on the next transfer.
        let n = data.len().min(8);
        for i in 0..n {
            let mut b = [0u8; 1];
            data.read_range(i..i + 1, &mut b)
                .map_err(|_| RequestError::went_away())?;
            while self.spi.sspsr().read().tnf().bit_is_clear() {}
            self.spi
                .sspdr()
                .write(|w| unsafe { w.data().bits(b[0] as u16) });
        }
        Ok(n)
    }

    fn drain_rx(
        &mut self,
        _: &RecvMessage,
        dest: LenLimit<Leased<W, [u8]>, 256>,
    ) -> Result<usize, RequestError<Infallible>> {
        // Non-blocking: read whatever the RX FIFO currently holds.
        let mut n = 0;
        while n < dest.len() && self.spi.sspsr().read().rne().bit_is_set() {
            let r = self.spi.sspdr().read().data().bits() as u8;
            dest.write_range(n..n + 1, &[r])
                .map_err(|_| RequestError::went_away())?;
            n += 1;
        }
        Ok(n)
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
    // Bring SPI0 out of reset via the sys server.
    let sys = Rp235xSys::from(SYS.get_task_id());
    sys.leave_reset(sys_api::SPI0);

    let p = unsafe { rp235x_pac::Peripherals::steal() };
    let spi = p.SPI0;

    // Configure with SSE off: 8-bit Motorola SPI mode 0 (SPO=SPH=0), SCR=0.
    spi.sspcpsr()
        .write(|w| unsafe { w.cpsdvsr().bits(CPSDVSR) });
    spi.sspcr0().write(|w| unsafe {
        w.dss().bits(DSS_8BIT);
        w.spo().clear_bit();
        w.sph().clear_bit();
        w.scr().bits(0)
    });
    // Master, internal loopback, then enable.
    spi.sspcr1().write(|w| {
        w.ms().clear_bit();
        w.lbm().set_bit();
        w.sse().set_bit()
    });

    let mut server = ServerImpl { spi };
    let mut incoming = [0u8; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
