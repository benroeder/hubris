// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! QSPI flash driver server for the RP2350 (RP235x) -- stage 1.
//!
//! Serves flash *reads* (safe while XIP is live: the driver just copies from
//! the memory-mapped XIP window) and boot-ROM table lookups, over the Idol
//! interface in `idl/rp235x-flash.idol`.
//!
//! Runs unprivileged with no peripherals; instead it gets two `extern-regions`
//! from `memory-pico-2.toml`:
//! * `rom` (0x20..0x8000, r-x) -- read the ROM lookup table (and later, call
//!   the ROM flash functions). Starts at 0x20, not 0, to avoid overlapping
//!   the kernel's per-task null region (see the memory toml);
//! * `xip` (0x14000000 uncached mirror, r--) -- read any flash address.
//!
//! Erase/program are NOT here yet. They require (datasheet sec 5.4.8.9):
//! the kernel text + vector table RAM-resident (XIP fetches bus-error while
//! the QMI is in direct mode, and SysTick fires every ms), ACCESSCTRL opened
//! for QMI (Secure-Privileged-only by default; the ROM flash code runs at the
//! caller's privilege), and a RAM-resident wrapper for the multi-call
//! exit-XIP/op/restore sequence. That is stage 2.

#![no_std]
#![no_main]

use drv_rp235x_flash_api::FlashError;
use idol_runtime::{Leased, LenLimit, R, RequestError, W};
use userlib::RecvMessage;

// XIP_BASE (the uncached NO-TRANSLATE XIP mirror,
// XIP_NOCACHE_NOALLOC_NOTRANSLATE_BASE 0x1c000000) and FLASH_SIZE, generated
// by build.rs from this task's `xip` extern region. The no-translate mirror
// gives PHYSICAL flash addresses -- the same space QMI direct-mode writes
// use -- so reads and writes agree even when the boot ROM has set QMI address
// translation for an A/B partition (see the memory toml).
include!(concat!(env!("OUT_DIR"), "/flash_config.rs"));

/// 4 KiB flash sector (erase granularity).
const SECTOR: u32 = 4096;
/// 256-byte flash page (program granularity).
const PAGE: u32 = 256;
/// Bounded spin for flash-busy polling. A sector erase is ~45 ms typical /
/// 400 ms max (W25Q32JV); this is comfortably beyond that at 150 MHz.
const BUSY_SPINS: u32 = 30_000_000;

/// Minimal QMI direct-mode SPI master for talking to the flash chip
/// (datasheet sec 12.14.5). While direct mode is active, XIP accesses
/// bus-error -- safe here because the whole image runs from SRAM and this
/// server serializes its own read ops with erase/program.
struct Direct<'a> {
    qmi: &'a rp235x_pac::QMI,
}

impl<'a> Direct<'a> {
    /// Enter direct mode. CLKDIV=30 -> 5 MHz SCK at 150 MHz clk_sys:
    /// conservative and far inside every W25Q rating.
    fn new(qmi: &'a rp235x_pac::QMI) -> Self {
        qmi.direct_csr().write(|w| unsafe {
            w.clkdiv().bits(30);
            w.en().set_bit()
        });
        while qmi.direct_csr().read().busy().bit_is_set() {}
        Direct { qmi }
    }

    /// Run one CS-framed SPI transaction: clock out every byte of `tx`, then
    /// clock `rx.len()` more bytes capturing the responses.
    fn transact(&self, tx: &[u8], rx: &mut [u8]) {
        self.qmi
            .direct_csr()
            .modify(|_, w| w.assert_cs0n().set_bit());
        for &b in tx {
            self.xfer(b);
        }
        for slot in rx.iter_mut() {
            *slot = self.xfer(0);
        }
        self.qmi
            .direct_csr()
            .modify(|_, w| w.assert_cs0n().clear_bit());
    }

    /// Clock one byte out (single-lane, output enabled) and return the byte
    /// clocked in.
    fn xfer(&self, b: u8) -> u8 {
        while self.qmi.direct_csr().read().txfull().bit_is_set() {}
        self.qmi.direct_tx().write(|w| unsafe {
            w.oe().set_bit();
            w.data().bits(b as u16)
        });
        while self.qmi.direct_csr().read().rxempty().bit_is_set() {}
        self.qmi.direct_rx().read().direct_rx().bits() as u8
    }
}

impl Drop for Direct<'_> {
    /// Leave direct mode; the QMI resumes memory-mapped (XIP) service with
    /// its M0 window configuration untouched.
    fn drop(&mut self) {
        while self.qmi.direct_csr().read().busy().bit_is_set() {}
        self.qmi.direct_csr().modify(|_, w| w.en().clear_bit());
    }
}

struct ServerImpl {
    qmi: rp235x_pac::QMI,
}

impl ServerImpl {
    /// Issue WRITE ENABLE, run `op`, then poll the status register until the
    /// chip finishes (or we time out).
    fn write_op(&mut self, op: &[u8]) -> Result<(), FlashError> {
        let d = Direct::new(&self.qmi);
        d.transact(&[0x06], &mut []); // WRITE ENABLE
        d.transact(op, &mut []);
        // Poll READ STATUS-1 until the BUSY bit clears.
        let mut sr = [0u8; 1];
        let mut done = false;
        for _ in 0..BUSY_SPINS {
            d.transact(&[0x05], &mut sr);
            if sr[0] & 0x01 == 0 {
                done = true;
                break;
            }
        }
        // Leave the chip in a pristine power-on-like state: exit any
        // continuous-read mode (FFh) and soft-reset the volatile config
        // (66h + 99h). Without this, chip state left by direct-mode traffic
        // survives a watchdog reboot (and even an apparent power cycle, since
        // the SWD wires can back-feed the board) and can make the boot ROM's
        // flash probe fail -- observed as a bricked-until-BOOTSEL reboot
        // right after a self-update.
        d.transact(&[0xff], &mut []);
        d.transact(&[0x66], &mut []);
        d.transact(&[0x99], &mut []);
        // t_RST is 30 us; at 5 MHz SCK one dummy frame comfortably covers it.
        d.transact(&[0xff], &mut []);
        if done {
            Ok(())
        } else {
            Err(FlashError::Timeout)
        }
    }
}

impl idl::InOrderRp235xFlashImpl for ServerImpl {
    fn read(
        &mut self,
        _: &RecvMessage,
        offset: u32,
        dest: LenLimit<Leased<W, [u8]>, 256>,
    ) -> Result<usize, RequestError<FlashError>> {
        if offset >= FLASH_SIZE {
            return Err(FlashError::BadAddress.into());
        }
        // Short read at the end of the device: return what fits.
        let len = dest.len().min((FLASH_SIZE - offset) as usize);
        let mut buf = [0u8; 256];
        // SAFETY: the range is inside the `xip` extern region granted to this
        // task, and reads from live XIP are side-effect free.
        unsafe {
            core::ptr::copy_nonoverlapping(
                (XIP_BASE + offset) as *const u8,
                buf.as_mut_ptr(),
                len,
            );
        }
        dest.write_range(0..len, &buf[..len])
            .map_err(|_| RequestError::went_away())?;
        Ok(len)
    }

    fn erase(
        &mut self,
        _: &RecvMessage,
        offset: u32,
    ) -> Result<(), RequestError<FlashError>> {
        if offset >= FLASH_SIZE {
            return Err(FlashError::BadAddress.into());
        }
        if !offset.is_multiple_of(SECTOR) {
            return Err(FlashError::BadAlignment.into());
        }
        // SECTOR ERASE (20h) + 24-bit address.
        let a = offset.to_be_bytes();
        self.write_op(&[0x20, a[1], a[2], a[3]])?;
        Ok(())
    }

    fn program(
        &mut self,
        _: &RecvMessage,
        offset: u32,
        data: LenLimit<Leased<R, [u8]>, 256>,
    ) -> Result<(), RequestError<FlashError>> {
        let len = data.len() as u32;
        if offset >= FLASH_SIZE || len > FLASH_SIZE - offset {
            return Err(FlashError::BadAddress.into());
        }
        // Must lie within a single 256-byte page (the chip wraps otherwise).
        if len == 0 || (offset % PAGE) + len > PAGE {
            return Err(FlashError::BadAlignment.into());
        }
        let mut buf = [0u8; 4 + 256];
        // PAGE PROGRAM (02h) + 24-bit address + data.
        buf[0] = 0x02;
        let a = offset.to_be_bytes();
        buf[1..4].copy_from_slice(&a[1..4]);
        data.read_range(0..len as usize, &mut buf[4..4 + len as usize])
            .map_err(|_| RequestError::went_away())?;
        self.write_op(&buf[..4 + len as usize])?;
        Ok(())
    }

    fn rom_lookup(
        &mut self,
        _: &RecvMessage,
        code: u16,
    ) -> Result<u32, RequestError<core::convert::Infallible>> {
        // Match Secure-Arm functions and data entries, so the shell can probe
        // both kinds. (An entry never carries both flags, per the bootrom.)
        Ok(rp235x_romapi::rom_table_lookup(
            code.to_le_bytes(),
            rp235x_romapi::RT_FLAG_FUNC_ARM_SEC | rp235x_romapi::RT_FLAG_DATA,
        ) as u32)
    }

    fn reboot(
        &mut self,
        _: &RecvMessage,
        bootsel: u8,
    ) -> Result<u32, RequestError<core::convert::Infallible>> {
        // ROM functions cannot be called from unprivileged tasks (privilege-
        // gated; verified by experiment), so both variants reboot via direct
        // watchdog/PSM writes. For BOOTSEL, a magic in watchdog scratch0
        // (which survives the reset) asks the app's *privileged* pre-kernel
        // boot path to complete the hop by calling the ROM reboot-to-BOOTSEL.
        // Needs the watchdog + psm MPU grants (app.toml) and ACCESSCTRL
        // opened for them (rp235x-startup).
        const BOOTSEL_MAGIC: u32 = 0xb007_5e1f;
        let psm = unsafe { &*rp235x_pac::PSM::ptr() };
        let wd = unsafe { &*rp235x_pac::WATCHDOG::ptr() };
        wd.scratch0().write(|w| unsafe {
            w.bits(if bootsel != 0 { BOOTSEL_MAGIC } else { 0 })
        });
        // Select everything but the processor cold domain for watchdog
        // reset, clear any stale vectored-boot magic, and force the reset.
        psm.wdsel().write(|w| unsafe { w.bits(0xffff_fffe) });
        wd.scratch4().write(|w| unsafe { w.bits(0) });
        // Clear CTRL first -- notably the PAUSE_DBG0/1/PAUSE_JTAG bits, which
        // reset to 1: with a debugger attached (or having been attached this
        // power session), triggering the watchdog with pause bits set wedges
        // the chip in an unrecoverable-until-BOOTSEL state. The boot ROM's
        // own reboot code does exactly this, "to ensure we reboot even under
        // debugger".
        wd.ctrl().write(|w| unsafe { w.bits(0) });
        wd.ctrl().write(|w| w.trigger().set_bit());
        // Reset is effectively immediate; this reply is best-effort.
        Ok(0)
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
    let p = unsafe { rp235x_pac::Peripherals::steal() };
    let mut server = ServerImpl { qmi: p.QMI };
    let mut incoming = [0u8; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    use drv_rp235x_flash_api::FlashError;
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
