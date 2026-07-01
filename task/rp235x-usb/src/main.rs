// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! USB CDC-ACM (virtual serial port) task for the Raspberry Pi Pico 2.
//!
//! Builds a `usb-device` stack on our `rp235x-usb::UsbBus` backend and a
//! `usbd-serial` CDC-ACM class, then polls the device. Once enumerated it appears on
//! the host as `/dev/cu.usbmodem...` and echoes received bytes.
//!
//! This first cut polls in a tight loop (USB enumeration is timing-sensitive) with a
//! slow LED heartbeat so we can see the task is alive; a later revision will drive it
//! from the USB interrupt (IRQ 14), now that the interrupt path is proven.
//!
//! Needs `clk_usb` = 48 MHz (brought up by the app's `lib/rp235x-startup`). Runs
//! unprivileged with `uses = ["sio", "usbctrl", "usb_dpram", "resets"]`.

#![no_std]
#![no_main]

// Pulls in userlib's task runtime (`_start`) and panic handler.
use userlib as _;

use rp235x_usb::UsbBus;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::StringDescriptors;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

/// Pico 2 onboard LED (heartbeat).
const LED: u32 = 1 << 25;

#[export_name = "main"]
pub fn main() -> ! {
    let p = unsafe { rp235x_pac::Peripherals::steal() };

    // Build the USB bus (clk_usb is already 48 MHz). Force VBUS detect since the
    // Pico 2 has no dedicated VBUS sense routed to the controller.
    let bus = UsbBus::new(p.USB, p.USB_DPRAM, true, &p.RESETS);
    let sio = p.SIO;

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

    let mut heartbeat: u32 = 0;
    loop {
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            if let Ok(n) = serial.read(&mut buf) {
                let _ = serial.write(&buf[..n]);
            }
        }

        heartbeat = heartbeat.wrapping_add(1);
        if heartbeat.is_multiple_of(300_000) {
            sio.gpio_out_xor().write(|w| unsafe { w.bits(LED) });
        }
    }
}
