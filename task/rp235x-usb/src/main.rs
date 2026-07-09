// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! USB CDC-ACM console server for the Raspberry Pi Pico 2.
//!
//! Presents the board as a USB serial port (/dev/cu.usbmodem...) and serves as
//! the system console transport over an Idol interface (`idl/rp235x-usbcons.idol`):
//!
//! * `write`: clients print bytes to the port (best effort; dropped if the host
//!   is not draining).
//! * `read`: clients drain host->device bytes (keystrokes), buffered here in a
//!   ring on the USB interrupt so nothing is lost between calls.
//!
//! Interrupt-driven (IRQ 14 as the `usb-irq` notification), so it does not
//! busy-spin. Needs `clk_usb` = 48 MHz (from the app's `lib/rp235x-startup`).
//! Runs unprivileged with `uses = ["usbctrl", "usb_dpram", "resets"]`.

#![no_std]
#![no_main]

use core::convert::Infallible;
use idol_runtime::{Leased, LenLimit, R, RequestError, W};
use userlib::{RecvMessage, sys_irq_control};

use rp235x_usb::UsbBus;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::StringDescriptors;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

/// Host->device bytes buffered between `read` calls. Sized for the shell's
/// firmware-update protocol: a full 256-byte page chunk must fit with slack
/// (drop-oldest on overflow).
const RX_RING_LEN: usize = 512;

/// The RX ring lives in a static (BSS), NOT in `ServerImpl`, because the server
/// is a stack local in `main`; an 8 KiB inline array would blow the task stack.
static mut USB_RX_RING: [u8; RX_RING_LEN] = [0; RX_RING_LEN];

/// Borrow the RX ring. SAFETY: the usb task is single-threaded and the only
/// accessors are `push`/`pop` (both `&mut self`), so this is the only live ref.
fn rx_ring() -> &'static mut [u8; RX_RING_LEN] {
    unsafe { &mut *core::ptr::addr_of_mut!(USB_RX_RING) }
}

struct ServerImpl {
    dev: UsbDevice<'static, UsbBus>,
    serial: SerialPort<'static, UsbBus>,
    head: usize,
    tail: usize,
}

impl ServerImpl {
    fn push(&mut self, b: u8) {
        let next = (self.head + 1) % RX_RING_LEN;
        if next == self.tail {
            self.tail = (self.tail + 1) % RX_RING_LEN;
        }
        rx_ring()[self.head] = b;
        self.head = next;
    }

    fn pop(&mut self) -> Option<u8> {
        if self.tail == self.head {
            None
        } else {
            let b = rx_ring()[self.tail];
            self.tail = (self.tail + 1) % RX_RING_LEN;
            Some(b)
        }
    }

    /// Service the device: run enumeration/control traffic and drain any
    /// received bytes into the ring.
    fn poll(&mut self) {
        if self.dev.poll(&mut [&mut self.serial]) {
            let mut rx = [0u8; 64];
            while let Ok(n) = self.serial.read(&mut rx) {
                if n == 0 {
                    break;
                }
                for &b in &rx[..n] {
                    self.push(b);
                }
            }
        }
    }
}

impl idl::InOrderUsbConsImpl for ServerImpl {
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
            // Push into the CDC endpoint, servicing the device between
            // attempts so the endpoint drains. Bounded retries: if the host
            // stops reading entirely we still drop rather than wedge.
            let mut sent = 0usize;
            for _ in 0..2000 {
                match self.serial.write(&chunk[sent..n]) {
                    Ok(k) => sent += k,
                    Err(_) => {}
                }
                let _ = self.dev.poll(&mut [&mut self.serial]);
                if sent == n {
                    break;
                }
            }
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
        notifications::USB_IRQ_MASK
    }

    fn handle_notification(&mut self, bits: userlib::NotificationBits) {
        if bits.check_notification_mask(notifications::USB_IRQ_MASK) {
            self.poll();
            sys_irq_control(notifications::USB_IRQ_MASK, true);
        }
    }
}

/// The `usb-device` objects borrow the bus allocator for their whole life, so
/// it must live in a static; initialized exactly once at task start (and again
/// only if jefe restarts the task, which re-runs main from scratch).
static mut USB_ALLOC: Option<UsbBusAllocator<UsbBus>> = None;

/// USB serial-number string, filled from the chip's unique 64-bit device id
/// so multiple boards enumerate distinctly on one host. Static because the
/// string descriptors borrow it for the device's life.
static mut SERIAL: [u8; 17] = [0; 17];

/// Format the device id (matching `picotool info` byte order, `H`-prefixed
/// to mark Hubris) into [`SERIAL`]. The id is stashed in watchdog scratch1/2
/// by the app's privileged pre-kernel main (OTP itself is privileged-only).
fn unique_serial(p: &rp235x_pac::Peripherals) -> &'static str {
    let id: u64 = (p.WATCHDOG.scratch1().read().bits() as u64) << 32
        | p.WATCHDOG.scratch2().read().bits() as u64;
    const D: &[u8; 16] = b"0123456789ABCDEF";
    // SAFETY: single-threaded task; written once here at startup, before the
    // descriptor ever renders it.
    unsafe {
        let buf = &mut *core::ptr::addr_of_mut!(SERIAL);
        buf[0] = b'H';
        for i in 0..16 {
            buf[i + 1] = D[((id >> (60 - 4 * i)) & 0xf) as usize];
        }
        core::str::from_utf8_unchecked(&*core::ptr::addr_of!(SERIAL))
    }
}

#[export_name = "main"]
pub fn main() -> ! {
    let p = unsafe { rp235x_pac::Peripherals::steal() };
    let serial_str = unique_serial(&p);

    // Force VBUS detect since the Pico 2 has no dedicated VBUS sense to the controller.
    let bus = UsbBus::new(p.USB, p.USB_DPRAM, true, &p.RESETS);
    // SAFETY: single-threaded task; main runs once per task (re)start.
    let alloc: &'static UsbBusAllocator<UsbBus> = unsafe {
        (*core::ptr::addr_of_mut!(USB_ALLOC)).insert(UsbBusAllocator::new(bus))
    };
    let serial = SerialPort::new(alloc);
    let dev = UsbDeviceBuilder::new(alloc, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Oxide Hubris")
            .product("RP2350 Pico 2 CDC")
            .serial_number(serial_str)])
        .unwrap()
        .device_class(USB_CLASS_CDC)
        .max_packet_size_0(64)
        .unwrap()
        .build();

    sys_irq_control(notifications::USB_IRQ_MASK, true);

    let mut server = ServerImpl {
        dev,
        serial,
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

// Pulls in the `notifications` module generated from this task's app.toml (provides
// `USB_IRQ_MASK`).
include!(concat!(env!("OUT_DIR"), "/notifications.rs"));
