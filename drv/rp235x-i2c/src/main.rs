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
use idol_runtime::{Leased, LenLimit, R, RequestError, W};
use userlib::{RecvMessage, sys_irq_control, task_slot};

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
    /// Bytes served on every controller read while in target (slave) mode.
    window: [u8; 16],
    window_len: usize,
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

    fn set_speed(
        &mut self,
        _: &RecvMessage,
        khz: u32,
    ) -> Result<(), RequestError<core::convert::Infallible>> {
        // Reconfigure SCL timing. speed=standard for <=100 kHz, else fast
        // (which covers both fast 400k and fast-mode-plus 1M). Same count
        // split as the initial config: low = 3/5 of the period, high = the
        // rest. The slave (target) uses `speed` too, for its spike filter.
        let hz = khz.max(1) * 1000;
        let period = CLK_HZ / hz;
        let lcnt = (period * 3 / 5) as u16;
        let hcnt = (period - period * 3 / 5) as u16;
        self.i2c.ic_enable().write(|w| w.enable().disabled());
        while self.i2c.ic_enable_status().read().ic_en().bit_is_set() {}
        if hz > 100_000 {
            self.i2c.ic_con().modify(|_, w| w.speed().fast());
            self.i2c
                .ic_fs_scl_hcnt()
                .write(|w| unsafe { w.ic_fs_scl_hcnt().bits(hcnt) });
            self.i2c
                .ic_fs_scl_lcnt()
                .write(|w| unsafe { w.ic_fs_scl_lcnt().bits(lcnt) });
        } else {
            self.i2c.ic_con().modify(|_, w| w.speed().standard());
            self.i2c
                .ic_ss_scl_hcnt()
                .write(|w| unsafe { w.ic_ss_scl_hcnt().bits(hcnt) });
            self.i2c
                .ic_ss_scl_lcnt()
                .write(|w| unsafe { w.ic_ss_scl_lcnt().bits(lcnt) });
        }
        self.i2c.ic_enable().write(|w| w.enable().enabled());
        Ok(())
    }

    fn serve(
        &mut self,
        _: &RecvMessage,
        addr: u8,
        data: LenLimit<Leased<R, [u8]>, 16>,
    ) -> Result<(), RequestError<I2cError>> {
        // Switch the block to target (slave) mode at `addr` and stash the
        // window it serves. Reads are handled asynchronously in the RD_REQ
        // interrupt (the DW slave stretches SCL until we provide data, so the
        // IPC/IRQ latency is covered).
        let len = data.len().min(16);
        data.read_range(0..len, &mut self.window[..len])
            .map_err(|_| RequestError::went_away())?;
        self.window_len = len;

        self.i2c.ic_enable().write(|w| w.enable().disabled());
        while self.i2c.ic_enable_status().read().ic_en().bit_is_set() {}
        self.i2c.ic_con().write(|w| {
            w.master_mode().disabled();
            w.ic_slave_disable().slave_enabled();
            w.ic_restart_en().enabled();
            w.speed().standard()
        });
        self.i2c
            .ic_sar()
            .write(|w| unsafe { w.ic_sar().bits(addr as u16) });
        // Enable ONLY read-request (bit5, controller wants data), tx-abort
        // (bit6, the DW flushes our TX FIFO when the controller ends a read
        // early -- so each read starts fresh from window[0]) and rx-full (bit2,
        // controller wrote us). Written as raw bits because two traps combine:
        // IC_INTR_MASK resets to 0x8ff (most enabled) so a partial svd2rust
        // `write` leaves TX_EMPTY enabled (fires continuously on the empty
        // FIFO), AND the PAC's enabled()/disabled() enum is inverted vs the
        // hardware (bit=1 enables). Either alone livelocks this priority-2
        // task and starves USB/shell. 0x64 = (1<<2)|(1<<5)|(1<<6).
        self.i2c
            .ic_intr_mask()
            .write(|w| unsafe { w.bits(0x0000_0064) });
        self.i2c.ic_enable().write(|w| w.enable().enabled());
        sys_irq_control(notifications::I2C_IRQ_MASK, true);
        Ok(())
    }
}

impl idol_runtime::NotificationHandler for ServerImpl {
    fn current_notification_mask(&self) -> u32 {
        notifications::I2C_IRQ_MASK
    }

    fn handle_notification(&mut self, bits: userlib::NotificationBits) {
        if !bits.check_notification_mask(notifications::I2C_IRQ_MASK) {
            return;
        }
        let stat = self.i2c.ic_raw_intr_stat().read();
        if stat.rd_req().is_active() {
            // Controller is reading us: clear the request and (re)load the
            // whole window into the TX FIFO. The DW slave shifts it out as the
            // controller clocks; leftover bytes are flushed on the next STOP.
            let _ = self.i2c.ic_clr_rd_req().read();
            for &b in &self.window[..self.window_len] {
                self.i2c.ic_data_cmd().write(|w| unsafe { w.dat().bits(b) });
            }
        }
        if stat.tx_abrt().is_active() {
            let _ = self.i2c.ic_clr_tx_abrt().read();
        }
        if stat.rx_full().is_active() {
            // Controller wrote to us; drain (unused by the demo).
            while self.i2c.ic_rxflr().read().rxflr().bits() > 0 {
                let _ = self.i2c.ic_data_cmd().read();
            }
        }
        sys_irq_control(notifications::I2C_IRQ_MASK, true);
    }
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

    let mut server = ServerImpl {
        i2c,
        window: [0u8; 16],
        window_len: 0,
    };
    let mut incoming = [0u8; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    use drv_rp235x_i2c_api::I2cError;
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}

include!(concat!(env!("OUT_DIR"), "/notifications.rs"));
