// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Inter-core mailbox driver for the RP2350 (RP235x) -- core 0 side.
//!
//! Owns the SIO inter-core FIFO (shared with the GPIO driver, which uses a
//! different sub-region of the same SIO block) and bridges it to Hubris IPC:
//! `exchange(req)` pushes a 32-bit word to the core-1 payload and returns its
//! reply. This is the AMP "tasks post between cores" path (branch rp2350-amp)
//! -- an explicit request/reply channel, not transparent cross-core IPC.
//!
//! The core-1 payload (a bare compute loop launched by the app's pre-kernel
//! main) reads each request and replies with a computed word.

#![no_std]
#![no_main]

use core::convert::Infallible;
use idol_runtime::RequestError;
use userlib::{RecvMessage, hl};

/// Bounded spin for a core-1 reply, in ~1 ms sleep units. Core 1 answers in
/// microseconds; this only bounds a wedged/absent core.
const REPLY_TIMEOUT: u32 = 200;

struct ServerImpl {
    sio: rp235x_pac::SIO,
}

impl idl::InOrderRp235xMailboxImpl for ServerImpl {
    fn exchange(
        &mut self,
        _: &RecvMessage,
        req: u32,
    ) -> Result<u32, RequestError<Infallible>> {
        // Drop any stale words core 1 may have left in our read FIFO.
        while self.sio.fifo_st().read().vld().bit_is_set() {
            let _ = self.sio.fifo_rd().read().bits();
        }
        // Push the request (wait for TX-FIFO space). Core 1 busy-polls the
        // FIFO, so no SEV is needed to wake it.
        while self.sio.fifo_st().read().rdy().bit_is_clear() {}
        self.sio.fifo_wr().write(|w| unsafe { w.bits(req) });
        // Wait for the reply, bounded.
        let mut spins = 0u32;
        while self.sio.fifo_st().read().vld().bit_is_clear() {
            spins += 1;
            if spins > REPLY_TIMEOUT {
                return Ok(0xffff_ffff);
            }
            hl::sleep_for(1);
        }
        Ok(self.sio.fifo_rd().read().bits())
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
    let mut server = ServerImpl { sio: p.SIO };
    let mut incoming = [0u8; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
