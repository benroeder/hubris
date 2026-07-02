// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! UART0 driver server for the RP2350 (RP235x).
//!
//! An idiomatic Idol server (see `idl/rp235x-uart.idol`): `write` busy-polls bytes
//! out UART0 TX; `read` returns bytes the RX interrupt has buffered. RX is
//! interrupt-driven -- UART0_IRQ (delivered as the `uart-irq` notification) drains the
//! hardware FIFO into a ring buffer that `read` serves from, so no bytes are lost
//! between calls.
//!
//! UART0 itself (baud, 8N1, pin mux) is brought up by the app's privileged
//! pre-kernel `main` via `lib/rp235x-uart`; this task adds the RX interrupt and the
//! IPC surface. Runs unprivileged with `uses = ["uart0"]` and the `uart-irq`
//! notification.

#![no_std]
#![no_main]

use core::convert::Infallible;
use idol_runtime::{Leased, LenLimit, RequestError, R, W};
use userlib::{sys_irq_control, RecvMessage};

const RX_RING_LEN: usize = 256;

struct ServerImpl {
    p: rp235x_pac::Peripherals,
    ring: [u8; RX_RING_LEN],
    head: usize, // next slot the IRQ will write
    tail: usize, // next slot `read` will consume
}

impl ServerImpl {
    /// Push one received byte into the ring, dropping the oldest on overflow.
    fn push(&mut self, b: u8) {
        let next = (self.head + 1) % RX_RING_LEN;
        if next == self.tail {
            // Full: drop the oldest byte to make room.
            self.tail = (self.tail + 1) % RX_RING_LEN;
        }
        self.ring[self.head] = b;
        self.head = next;
    }

    fn pop(&mut self) -> Option<u8> {
        if self.tail == self.head {
            None
        } else {
            let b = self.ring[self.tail];
            self.tail = (self.tail + 1) % RX_RING_LEN;
            Some(b)
        }
    }
}

impl idl::InOrderRp235xUartImpl for ServerImpl {
    fn write(
        &mut self,
        _: &RecvMessage,
        data: LenLimit<Leased<R, [u8]>, 256>,
    ) -> Result<usize, RequestError<Infallible>> {
        let len = data.len();
        let mut chunk = [0u8; 64];
        let mut off = 0;
        while off < len {
            let n = (len - off).min(chunk.len());
            data.read_range(off..off + n, &mut chunk[..n])
                .map_err(|_| RequestError::went_away())?;
            rp235x_uart::write_all(&self.p, &chunk[..n]);
            off += n;
        }
        Ok(len)
    }

    fn read(
        &mut self,
        _: &RecvMessage,
        dest: LenLimit<Leased<W, [u8]>, 256>,
    ) -> Result<usize, RequestError<Infallible>> {
        let cap = dest.len();
        let mut count = 0;
        while count < cap {
            match self.pop() {
                Some(b) => {
                    dest.write_range(count..count + 1, &[b])
                        .map_err(|_| RequestError::went_away())?;
                    count += 1;
                }
                None => break,
            }
        }
        Ok(count)
    }
}

impl idol_runtime::NotificationHandler for ServerImpl {
    fn current_notification_mask(&self) -> u32 {
        notifications::UART_IRQ_MASK
    }

    fn handle_notification(&mut self, bits: userlib::NotificationBits) {
        if bits.check_notification_mask(notifications::UART_IRQ_MASK) {
            // Drain the RX FIFO into the ring.
            while rp235x_uart::rx_ready(&self.p) {
                let b = rp235x_uart::read_byte(&self.p);
                self.push(b);
            }
            rp235x_uart::clear_rx_interrupt(&self.p);
            sys_irq_control(notifications::UART_IRQ_MASK, true);
        }
    }
}

#[export_name = "main"]
fn main() -> ! {
    let p = unsafe { rp235x_pac::Peripherals::steal() };

    // UART0 (baud/8N1/pins) is already configured by the app's pre-kernel main;
    // here we just arm the RX interrupt path.
    rp235x_uart::enable_rx_interrupt(&p);
    sys_irq_control(notifications::UART_IRQ_MASK, true);

    let mut server = ServerImpl {
        p,
        ring: [0u8; RX_RING_LEN],
        head: 0,
        tail: 0,
    };
    let mut incoming = [0u8; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}

include!(concat!(env!("OUT_DIR"), "/notifications.rs"));
