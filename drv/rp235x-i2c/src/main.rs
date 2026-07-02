// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! I2C0 (DesignWare DW_apb_i2c) driver server for the RP2350 (RP235x).
//!
//! Minimal 7-bit master behind an Idol interface (`idl/rp235x-i2c.idol`):
//! `probe` (does anything ACK this address?), `write`, and `read`, all ending
//! in STOP. 100 kHz standard mode from clk_sys (150 MHz).
//!
//! The SDA/SCL pin mux (GP4/GP5, funcsel 3, pull-ups) is done by the app's
//! privileged pre-kernel `main`; I2C0 comes out of reset via the sys server.
//! Runs unprivileged with `uses = ["i2c0"]`.
//!
//! NOTE: unlike the PL022 SPI, this block has no internal loopback, so a real
//! transaction test needs a device on the bus (e.g. a second Pico running an
//! I2C peripheral, or any sensor breakout). Without one, every probe NAKs --
//! which still exercises the controller datapath end to end.

#![no_std]
#![no_main]

use drv_rp235x_i2c_api::I2cError;
use drv_rp235x_sys_api::{self as sys_api, Rp235xSys};
use idol_runtime::{Leased, LenLimit, RequestError, R, W};
use userlib::{task_slot, RecvMessage};

task_slot!(SYS, sys);

/// I2C source clock: clk_sys.
const CLK_HZ: u32 = 150_000_000;
/// Bus speed: 100 kHz (standard mode).
const BAUD_HZ: u32 = 100_000;

/// Bounded-spin limit for FIFO waits. At 150 MHz this is tens of ms -- far
/// beyond any legal 100 kHz bus transaction, so hitting it means a wedged bus.
const SPIN_LIMIT: u32 = 2_000_000;

struct ServerImpl {
    i2c: rp235x_pac::I2C0,
}

impl ServerImpl {
    /// Point the controller at a new target address (requires disable/enable).
    fn set_target(&mut self, addr: u8) {
        self.i2c.ic_enable().write(|w| w.enable().disabled());
        while self.i2c.ic_enable_status().read().ic_en().bit_is_set() {}
        self.i2c
            .ic_tar()
            .write(|w| unsafe { w.ic_tar().bits(addr as u16) });
        self.i2c.ic_enable().write(|w| w.enable().enabled());
    }

    /// If the controller aborted (NAK, arbitration loss, ...), clear the abort
    /// latch and report it.
    fn check_abort(&mut self) -> Result<(), I2cError> {
        if self.i2c.ic_raw_intr_stat().read().tx_abrt().is_active() {
            // Reading IC_CLR_TX_ABRT clears the abort status + source register.
            let _ = self.i2c.ic_clr_tx_abrt().read();
            Err(I2cError::Nak)
        } else {
            Ok(())
        }
    }

    /// Spin until `cond` holds, checking for aborts; error on abort or timeout.
    fn wait(
        &mut self,
        cond: impl Fn(&rp235x_pac::I2C0) -> bool,
    ) -> Result<(), I2cError> {
        for _ in 0..SPIN_LIMIT {
            self.check_abort()?;
            if cond(&self.i2c) {
                return Ok(());
            }
        }
        Err(I2cError::Timeout)
    }

    /// Write `data` to `addr`, issuing STOP after the last byte.
    fn do_write(&mut self, addr: u8, data: &[u8]) -> Result<(), I2cError> {
        self.set_target(addr);
        let last = data.len() - 1;
        for (i, &b) in data.iter().enumerate() {
            self.wait(|i2c| i2c.ic_status().read().tfnf().is_not_full())?;
            self.i2c.ic_data_cmd().write(|w| {
                if i == last {
                    w.stop().enable();
                }
                w.cmd().write();
                unsafe { w.dat().bits(b) }
            });
        }
        // Wait for the transfer to finish (STOP seen), then final abort check.
        self.wait(|i2c| i2c.ic_raw_intr_stat().read().stop_det().is_active())?;
        let _ = self.i2c.ic_clr_stop_det().read();
        self.check_abort()
    }

    /// Read `dest.len()` bytes from `addr`, issuing STOP after the last byte.
    fn do_read(&mut self, addr: u8, dest: &mut [u8]) -> Result<(), I2cError> {
        self.set_target(addr);
        let last = dest.len() - 1;
        for (i, slot) in dest.iter_mut().enumerate() {
            self.wait(|i2c| i2c.ic_status().read().tfnf().is_not_full())?;
            self.i2c.ic_data_cmd().write(|w| {
                if i == last {
                    w.stop().enable();
                }
                w.cmd().read()
            });
            self.wait(|i2c| i2c.ic_rxflr().read().rxflr().bits() > 0)?;
            *slot = self.i2c.ic_data_cmd().read().dat().bits();
        }
        let _ = self.i2c.ic_clr_stop_det().read();
        Ok(())
    }
}

impl idl::InOrderRp235xI2cImpl for ServerImpl {
    fn probe(
        &mut self,
        _: &RecvMessage,
        addr: u8,
    ) -> Result<bool, RequestError<I2cError>> {
        // The standard DW bus-scan idiom: attempt a 1-byte read; an ACKing
        // device answers, an empty address aborts with NAK.
        let mut byte = [0u8; 1];
        match self.do_read(addr, &mut byte) {
            Ok(()) => Ok(true),
            Err(I2cError::Nak) => Ok(false),
            Err(e) => Err(e.into()),
        }
    }

    fn write(
        &mut self,
        _: &RecvMessage,
        addr: u8,
        data: LenLimit<Leased<R, [u8]>, 256>,
    ) -> Result<usize, RequestError<I2cError>> {
        let len = data.len();
        if len == 0 {
            return Ok(0);
        }
        let mut buf = [0u8; 256];
        data.read_range(0..len, &mut buf[..len])
            .map_err(|_| RequestError::went_away())?;
        self.do_write(addr, &buf[..len])?;
        Ok(len)
    }

    fn read(
        &mut self,
        _: &RecvMessage,
        addr: u8,
        dest: LenLimit<Leased<W, [u8]>, 256>,
    ) -> Result<usize, RequestError<I2cError>> {
        let len = dest.len();
        if len == 0 {
            return Ok(0);
        }
        let mut buf = [0u8; 256];
        self.do_read(addr, &mut buf[..len])?;
        dest.write_range(0..len, &buf[..len])
            .map_err(|_| RequestError::went_away())?;
        Ok(len)
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
    // Bring I2C0 out of reset via the sys server.
    let sys = Rp235xSys::from(SYS.get_task_id());
    sys.leave_reset(sys_api::I2C0);

    let p = unsafe { rp235x_pac::Peripherals::steal() };
    let i2c = p.I2C0;

    // Configure while disabled: 7-bit master, standard speed, restarts allowed.
    i2c.ic_enable().write(|w| w.enable().disabled());
    i2c.ic_con().write(|w| {
        w.master_mode().enabled();
        w.ic_slave_disable().slave_disabled();
        w.ic_restart_en().enabled();
        w.speed().standard()
    });

    // 100 kHz timing from clk_sys (pico-sdk formula): period in ic_clk cycles,
    // low gets 3/5 of it, high the rest; SDA hold = 3/10 us.
    let period = CLK_HZ / BAUD_HZ; // 1500
    let lcnt = period * 3 / 5; // 900
    let hcnt = period - lcnt; // 600
    i2c.ic_ss_scl_hcnt()
        .write(|w| unsafe { w.ic_ss_scl_hcnt().bits(hcnt as u16) });
    i2c.ic_ss_scl_lcnt()
        .write(|w| unsafe { w.ic_ss_scl_lcnt().bits(lcnt as u16) });
    let sda_hold = CLK_HZ / 10_000_000 * 3 + 1; // 46
    i2c.ic_sda_hold()
        .write(|w| unsafe { w.ic_sda_tx_hold().bits(sda_hold as u16) });

    i2c.ic_enable().write(|w| w.enable().enabled());

    let mut server = ServerImpl { i2c };
    let mut incoming = [0u8; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    use drv_rp235x_i2c_api::I2cError;
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
