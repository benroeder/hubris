// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! USB CDC-ACM console server for the Raspberry Pi Pico 2.
//!
//! Presents the board as a USB serial port (/dev/cu.usbmodem...) and acts as a debug
//! console for the rest of the system:
//!
//! * On the USB interrupt (IRQ 14, delivered as the `usb-irq` notification) it polls
//!   the device -- servicing enumeration, and echoing anything typed into the port.
//! * On an IPC message from another task it writes the message bytes to the serial
//!   port. So any task with a task-slot to this one can "print" over USB (best effort;
//!   bytes are dropped if the host is not draining).
//!
//! Interrupt-driven, so it does not busy-spin. Needs `clk_usb` = 48 MHz (from the
//! app's `lib/rp235x-startup`). Runs unprivileged with
//! `uses = ["usbctrl", "usb_dpram", "resets"]` and the `usb-irq` notification.

#![no_std]
#![no_main]

// Pulls in userlib's task runtime (`_start`) and panic handler.
use userlib::{sys_irq_control, sys_recv_open, sys_reply, TaskId};

use rp235x_usb::UsbBus;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::StringDescriptors;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

#[export_name = "main"]
pub fn main() -> ! {
    let p = unsafe { rp235x_pac::Peripherals::steal() };

    // Force VBUS detect since the Pico 2 has no dedicated VBUS sense to the controller.
    let bus = UsbBus::new(p.USB, p.USB_DPRAM, true, &p.RESETS);
    let alloc = UsbBusAllocator::new(bus);
    let mut serial = SerialPort::new(&alloc);
    let mut usb_dev = UsbDeviceBuilder::new(&alloc, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Oxide Hubris")
            .product("RP2350 Pico 2 CDC")
            .serial_number("HUBRIS-0001")])
        .unwrap()
        .device_class(USB_CLASS_CDC)
        .max_packet_size_0(64)
        .unwrap()
        .build();

    let irq = notifications::USB_IRQ_MASK;
    sys_irq_control(irq, true);

    let mut msg = [0u8; 128];
    loop {
        let rm = sys_recv_open(&mut msg, irq);
        if rm.sender == TaskId::KERNEL {
            // USB interrupt: service the device, echo anything received.
            if usb_dev.poll(&mut [&mut serial]) {
                let mut rx = [0u8; 64];
                if let Ok(n) = serial.read(&mut rx) {
                    let _ = serial.write(&rx[..n]);
                }
            }
            sys_irq_control(irq, true);
        } else {
            // A task wants to log: write its bytes to the serial port (best effort),
            // then poll once to push them out, and reply so the client unblocks.
            let n = rm.message_len.min(msg.len());
            let _ = serial.write(&msg[..n]);
            let _ = usb_dev.poll(&mut [&mut serial]);
            sys_reply(rm.sender, 0, &[]);
        }
    }
}

// Pulls in the `notifications` module generated from this task's app.toml (provides
// `USB_IRQ_MASK`).
include!(concat!(env!("OUT_DIR"), "/notifications.rs"));
