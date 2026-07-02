// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! System-control (RESETS) driver server for the RP2350 (RP235x).
//!
//! Owns the RESETS block so peripheral driver tasks bring their block out of reset
//! via `sys.leave_reset(BLOCK)` (see `idl/rp235x-sys.idol` and the block constants in
//! `drv-rp235x-sys-api`) rather than each poking RESETS. RESETS is ACCESSCTRL
//! Secure/any-privilege, so this runs unprivileged.

#![no_std]
#![no_main]

use userlib::RecvMessage;

struct ServerImpl {
    resets: rp235x_pac::RESETS,
}

impl idl::InOrderRp235xSysImpl for ServerImpl {
    fn leave_reset(&mut self, _: &RecvMessage, mask: u32) -> Result<(), idol_runtime::RequestError<core::convert::Infallible>> {
        self.resets
            .reset()
            .modify(|r, w| unsafe { w.bits(r.bits() & !mask) });
        while self.resets.reset_done().read().bits() & mask != mask {}
        Ok(())
    }

    fn enter_reset(&mut self, _: &RecvMessage, mask: u32) -> Result<(), idol_runtime::RequestError<core::convert::Infallible>> {
        self.resets
            .reset()
            .modify(|r, w| unsafe { w.bits(r.bits() | mask) });
        Ok(())
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
    let mut server = ServerImpl { resets: p.RESETS };
    let mut incoming = [0u8; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
