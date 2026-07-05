// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Core-1 inter-core FIFO responder (AMP two-kernel, branch rp2350-amp).
//!
//! A task scheduled by core 1's OWN Hubris kernel that owns the core-1 side of
//! the SIO inter-core FIFO and answers requests from core 0: read a request
//! word, reply `req*2 + 1`. Together with core 0's `mailbox` driver this makes
//! `core1 <n>` work in the two-kernel model -- cross-core IPC between two
//! independent Hubris kernels.
//!
//! It busy-polls the FIFO for lowest latency. It runs at priority 2, below the
//! beat task (1); Hubris preempts a running task when a higher-priority one
//! becomes runnable via interrupt, so core 1's SysTick still wakes `beat` on
//! time despite this tight loop. (A later revision can make it interrupt-driven
//! off SIO_IRQ_FIFO so `idle` can WFI too.)

#![no_std]
#![no_main]

extern crate userlib;

extern "C" {
    static __REGION_SHARED_BUF_BASE: [u8; 0];
}

#[export_name = "main"]
fn main() -> ! {
    // SAFETY: this task is granted SIO (`uses = ["sio"]`); the FIFO registers
    // are core-local, so this reads/writes core 1's end of the inter-core FIFO.
    let sio = unsafe { &*rp235x_pac::SIO::ptr() };
    // The shared bulk-transfer buffer (granted via extern-regions).
    let buf = &raw const __REGION_SHARED_BUF_BASE as *const u32;
    loop {
        if sio.fifo_st().read().vld().bit_is_set() {
            let req = sio.fifo_rd().read().bits();
            let reply = if req & 0x8000_0000 != 0 {
                // Bulk doorbell: core 0 wrote `len` bytes to the shared buffer.
                // Read them out (as words) and reply with a checksum -- this is
                // the consume side of the one-way core0->core1 bulk transfer.
                let words = ((req & 0x7fff_ffff).min(4096) / 4) as usize;
                let mut sum = 0u32;
                let mut i = 0;
                while i < words {
                    sum =
                        sum.wrapping_add(unsafe { buf.add(i).read_volatile() });
                    i += 1;
                }
                sum
            } else {
                req.wrapping_mul(2).wrapping_add(1)
            };
            while sio.fifo_st().read().rdy().bit_is_clear() {}
            sio.fifo_wr().write(|w| unsafe { w.bits(reply) });
        }
    }
}
