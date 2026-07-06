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
    static __REGION_SHARED_BASE: [u8; 0];
}

/// Shared-window layout (see the mailbox driver): buffer 0 at +0x800 (SRAM8),
/// buffer 1 at +0x1000 (SRAM9) -- different banks; 2 KiB / 512 words each.
const BUF0_OFF: usize = 0x800 / 4;
const BUF1_OFF: usize = 0x1000 / 4;
const BUF_WORDS: usize = 512;

/// Sum `words` u32s from `base + off` -- the consume side of a bulk transfer.
fn sum(base: *const u32, off: usize, words: usize) -> u32 {
    let mut s = 0u32;
    let mut i = 0;
    while i < words {
        s = s.wrapping_add(unsafe { base.add(off + i).read_volatile() });
        i += 1;
    }
    s
}

#[export_name = "main"]
fn main() -> ! {
    // SAFETY: this task is granted SIO (`uses = ["sio"]`); the FIFO registers
    // are core-local, so this reads/writes core 1's end of the inter-core FIFO.
    let sio = unsafe { &*rp235x_pac::SIO::ptr() };
    let base = &raw const __REGION_SHARED_BASE as *const u32;
    loop {
        if sio.fifo_st().read().vld().bit_is_set() {
            let req = sio.fifo_rd().read().bits();
            let reply = if req & 0x8000_0000 != 0 {
                if req & 0x4000_0000 != 0 {
                    // Pipelined "ready": read+sum buffer b (fixed 2 KiB).
                    let b = (req & 1) as usize;
                    let off = BUF0_OFF + b * (BUF1_OFF - BUF0_OFF);
                    sum(base, off, BUF_WORDS)
                } else {
                    // Single bulk doorbell: `len` bytes in buffer 0.
                    let words = ((req & 0xffff).min(2048) / 4) as usize;
                    sum(base, BUF0_OFF, words)
                }
            } else {
                req.wrapping_mul(2).wrapping_add(1)
            };
            while sio.fifo_st().read().rdy().bit_is_clear() {}
            sio.fifo_wr().write(|w| unsafe { w.bits(reply) });
        }
    }
}
