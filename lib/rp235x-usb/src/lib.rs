// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! `usb-device` `UsbBus` backend for the RP2350 (RP235x) USB device controller.
//!
//! Derived from the `usb` module of `rp235x-hal`
//! (<https://github.com/rp-rs/rp-hal>, Copyright (c) The rp-rs Developers,
//! dual-licensed Apache-2.0 OR MIT), a proven `UsbBus` for this exact controller.
//! This attribution is retained per those licenses; see also the repo's
//! third-party licensing notes. Changes from upstream: the HAL clock token /
//! reset-subsystem trait / `itertools` dependencies are removed (clk_usb must
//! already be 48 MHz -- brought up by `lib/rp235x-startup`; the controller is reset
//! directly via RESETS), so this depends only on `rp235x-pac`, `usb-device`,
//! `critical-section`, and `cortex-m`.
//!
//! `Sync` comes from `critical_section::Mutex`, whose implementation Hubris provides
//! (a no-op -- tasks are single-threaded/cooperative) via `sys/userlib`. The bus is
//! only ever touched from the one USB task's poll loop, so there is no real
//! contention.

#![no_std]

use core::cell::RefCell;
use critical_section::Mutex;
use rp235x_pac as pac;
use usb_device::{
    bus::{PollResult, UsbBus as UsbBusTrait},
    endpoint::{EndpointAddress, EndpointType},
    Result as UsbResult, UsbDirection, UsbError,
};

#[allow(clippy::bool_to_int_with_if)]
fn ep_addr_to_ep_buf_ctrl_idx(ep_addr: EndpointAddress) -> usize {
    ep_addr.index() * 2 + (if ep_addr.is_in() { 0 } else { 1 })
}

struct Endpoint {
    ep_type: EndpointType,
    max_packet_size: u16,
    buffer_offset: u16,
}
impl Endpoint {
    unsafe fn get_buf_parts(&self) -> (*mut u8, usize) {
        const DPRAM_BASE: *mut u8 = pac::USB_DPRAM::ptr() as *mut u8;
        if self.ep_type == EndpointType::Control {
            (DPRAM_BASE.offset(0x100), self.max_packet_size as usize)
        } else {
            (
                DPRAM_BASE.offset(0x180 + (self.buffer_offset * 64) as isize),
                self.max_packet_size as usize,
            )
        }
    }
    fn get_buf(&self) -> &[u8] {
        // SAFETY: offset is checked by Inner::ep_allocate.
        unsafe {
            let (base, len) = self.get_buf_parts();
            core::slice::from_raw_parts(base as *const _, len)
        }
    }
    fn get_buf_mut(&mut self) -> &mut [u8] {
        // SAFETY: offset is checked by Inner::ep_allocate.
        unsafe {
            let (base, len) = self.get_buf_parts();
            core::slice::from_raw_parts_mut(base, len)
        }
    }
}

struct Inner {
    ctrl_reg: pac::USB,
    ctrl_dpram: pac::USB_DPRAM,
    in_endpoints: [Option<Endpoint>; 16],
    out_endpoints: [Option<Endpoint>; 16],
    next_offset: u16,
    read_setup: bool,
}
impl Inner {
    fn ep_allocate(
        &mut self,
        ep_addr: Option<EndpointAddress>,
        ep_dir: UsbDirection,
        ep_type: EndpointType,
        max_packet_size: u16,
    ) -> UsbResult<EndpointAddress> {
        let ep_addr = ep_addr
            .or_else(|| {
                let eps = if ep_dir == UsbDirection::In {
                    self.in_endpoints.iter()
                } else {
                    self.out_endpoints.iter()
                };
                let mut iter = eps.enumerate();
                if ep_type != EndpointType::Control {
                    iter.next(); // reserve ep0 for control
                }
                iter.find(|(_, ep)| ep.is_none())
                    .map(|(index, _)| EndpointAddress::from_parts(index, ep_dir))
            })
            .ok_or(UsbError::EndpointOverflow)?;

        let is_ep0 = ep_addr.index() == 0;
        let is_ctrl_ep = ep_type == EndpointType::Control;
        if !(is_ep0 ^ !is_ctrl_ep) {
            return Err(UsbError::Unsupported);
        }

        let eps = if ep_addr.is_in() {
            &mut self.in_endpoints
        } else {
            &mut self.out_endpoints
        };
        let maybe_ep = eps
            .get_mut(ep_addr.index())
            .ok_or(UsbError::EndpointOverflow)?;
        if maybe_ep.is_some() {
            return Err(UsbError::InvalidEndpoint);
        }

        if (!matches!(ep_type, EndpointType::Isochronous { .. }) && max_packet_size > 64)
            || max_packet_size > 1023
        {
            return Err(UsbError::Unsupported);
        }

        if ep_addr.index() == 0 {
            *maybe_ep = Some(Endpoint {
                ep_type,
                max_packet_size,
                buffer_offset: 0,
            });
        } else {
            let aligned_sized = max_packet_size.div_ceil(64);
            if (self.next_offset + aligned_sized) > (4096 / 64) {
                return Err(UsbError::EndpointMemoryOverflow);
            }
            let buffer_offset = self.next_offset;
            self.next_offset += aligned_sized;
            *maybe_ep = Some(Endpoint {
                ep_type,
                max_packet_size,
                buffer_offset,
            });
        }
        Ok(ep_addr)
    }

    fn ep_reset_all(&mut self) {
        self.ctrl_reg
            .sie_ctrl()
            .modify(|_, w| w.ep0_int_1buf().set_bit());
        self.ctrl_dpram
            .ep_buffer_control(0)
            .write(|w| w.pid_0().set_bit());
        self.ctrl_dpram
            .ep_buffer_control(1)
            .write(|w| w.pid_0().set_bit());
        cortex_m::asm::delay(12);
        self.ctrl_dpram
            .ep_buffer_control(1)
            .write(|w| w.available_0().set_bit());

        // Interleave IN[1..] and OUT[1..] to match the DPRAM ep_control layout
        // (EP1 IN, EP1 OUT, EP2 IN, ...). `index` counts every interleaved slot,
        // present or not, exactly like the original's `enumerate()`.
        use pac::usb_dpram::ep_control::ENDPOINT_TYPE_A;
        for i in 1..16usize {
            for dir_in in [true, false] {
                let index = (i - 1) * 2 + usize::from(!dir_in);
                let ep = if dir_in {
                    self.in_endpoints[i].as_ref()
                } else {
                    self.out_endpoints[i].as_ref()
                };
                let Some(ep) = ep else { continue };
                let ep_type = match ep.ep_type {
                    EndpointType::Bulk => ENDPOINT_TYPE_A::BULK,
                    EndpointType::Isochronous { .. } => ENDPOINT_TYPE_A::ISOCHRONOUS,
                    EndpointType::Control => ENDPOINT_TYPE_A::CONTROL,
                    EndpointType::Interrupt => ENDPOINT_TYPE_A::INTERRUPT,
                };
                let max_packet_size = ep.max_packet_size;
                let buffer_offset = ep.buffer_offset;
                self.ctrl_dpram.ep_control(index).modify(|_, w| unsafe {
                    w.endpoint_type().variant(ep_type);
                    w.interrupt_per_buff().set_bit();
                    w.enable().set_bit();
                    w.buffer_address().bits(0x180 + (buffer_offset << 6))
                });
                let buf_control = &self.ctrl_dpram.ep_buffer_control(index + 2);
                if (index & 1) == 0 {
                    buf_control.write(|w| w.pid_0().set_bit());
                } else {
                    buf_control.write(|w| unsafe {
                        w.pid_0().clear_bit();
                        w.length_0().bits(max_packet_size)
                    });
                    cortex_m::asm::delay(12);
                    buf_control.modify(|_, w| w.available_0().set_bit());
                }
            }
        }
    }

    fn ep_write(&mut self, ep_addr: EndpointAddress, buf: &[u8]) -> UsbResult<usize> {
        let index = ep_addr.index();
        let ep = self
            .in_endpoints
            .get_mut(index)
            .and_then(Option::as_mut)
            .ok_or(UsbError::InvalidEndpoint)?;

        let buf_control = &self.ctrl_dpram.ep_buffer_control(index * 2);
        if buf_control.read().available_0().bit_is_set() {
            return Err(UsbError::WouldBlock);
        }

        let ep_buf = ep.get_buf_mut();
        if ep_buf.len() < buf.len() {
            return Err(UsbError::BufferOverflow);
        }
        ep_buf[..buf.len()].copy_from_slice(buf);

        buf_control.modify(|r, w| unsafe {
            w.length_0().bits(buf.len() as u16);
            w.full_0().set_bit();
            w.pid_0().bit(!r.pid_0().bit())
        });
        cortex_m::asm::delay(12);
        buf_control.modify(|_, w| w.available_0().set_bit());
        Ok(buf.len())
    }

    fn ep_read(&mut self, ep_addr: EndpointAddress, buf: &mut [u8]) -> UsbResult<usize> {
        let index = ep_addr.index();
        let ep = self
            .out_endpoints
            .get_mut(index)
            .and_then(Option::as_mut)
            .ok_or(UsbError::InvalidEndpoint)?;

        let buf_control = &self.ctrl_dpram.ep_buffer_control(index * 2 + 1);
        let buf_control_val = buf_control.read();

        let process_setup = index == 0 && self.read_setup;
        if process_setup {
            let len = 8;
            let ep_buf =
                unsafe { core::slice::from_raw_parts(pac::USB_DPRAM::ptr() as *const u8, len) };
            if len > buf.len() {
                return Err(UsbError::BufferOverflow);
            }
            buf[..len].copy_from_slice(&ep_buf[..len]);

            self.ctrl_dpram
                .ep_buffer_control(0)
                .modify(|_, w| w.pid_0().clear_bit());
            self.ctrl_reg
                .sie_status()
                .write(|w| w.setup_rec().clear_bit_by_one());
            self.ctrl_reg.buff_status().write(|w| unsafe { w.bits(2) });

            let is_in_request = (buf[0] & 0x80) == 0x80;
            let data_length = u16::from(buf[6]) | (u16::from(buf[7]) << 8);
            let expect_data_or_zlp = is_in_request || data_length != 0;

            buf_control.modify(|_, w| unsafe {
                w.length_0().bits(ep.max_packet_size);
                w.full_0().clear_bit();
                w.pid_0().set_bit()
            });
            cortex_m::asm::delay(12);
            buf_control.modify(|_, w| w.available_0().bit(expect_data_or_zlp));

            self.read_setup = false;
            Ok(len)
        } else {
            if buf_control_val.full_0().bit_is_clear() {
                return Err(UsbError::WouldBlock);
            }
            let len = buf_control_val.length_0().bits().into();
            if len > buf.len() {
                return Err(UsbError::BufferOverflow);
            }
            buf[..len].copy_from_slice(&ep.get_buf()[..len]);
            self.ctrl_reg
                .buff_status()
                .write(|w| unsafe { w.bits(1 << (index * 2 + 1)) });

            buf_control.modify(|r, w| unsafe {
                w.length_0().bits(ep.max_packet_size);
                w.full_0().clear_bit();
                w.pid_0().bit(!r.pid_0().bit())
            });
            if index != 0 || len == ep.max_packet_size.into() {
                cortex_m::asm::delay(12);
                buf_control.modify(|_, w| w.available_0().set_bit());
            }
            Ok(len)
        }
    }
}

/// USB bus for the RP2350 device controller.
pub struct UsbBus {
    inner: Mutex<RefCell<Inner>>,
}

impl UsbBus {
    /// Reset and initialise the USB device controller, and return a bus.
    ///
    /// `clk_usb` must already be running at 48 MHz. `force_vbus_detect` overrides
    /// VBUS detection (useful on boards without a VBUS sense line).
    pub fn new(
        ctrl_reg: pac::USB,
        ctrl_dpram: pac::USB_DPRAM,
        force_vbus_detect: bool,
        resets: &pac::RESETS,
    ) -> Self {
        // Reset the USB controller.
        resets.reset().modify(|_, w| w.usbctrl().set_bit());
        resets.reset().modify(|_, w| w.usbctrl().clear_bit());
        while resets.reset_done().read().usbctrl().bit_is_clear() {}

        // Clear the control registers and the DPRAM.
        unsafe {
            core::slice::from_raw_parts_mut(pac::USB::ptr() as *mut u32, 1 + 0x98 / 4).fill(0);
            core::slice::from_raw_parts_mut(pac::USB_DPRAM::ptr() as *mut u32, 1 + 0xfc / 4)
                .fill(0);
        }

        ctrl_reg.usb_muxing().modify(|_, w| {
            w.to_phy().set_bit();
            w.softcon().set_bit()
        });
        if force_vbus_detect {
            ctrl_reg.usb_pwr().modify(|_, w| {
                w.vbus_detect().set_bit();
                w.vbus_detect_override_en().set_bit()
            });
        }
        ctrl_reg.main_ctrl().modify(|_, w| {
            w.sim_timing().clear_bit();
            w.host_ndevice().clear_bit();
            w.controller_en().set_bit()
        });

        Self {
            inner: Mutex::new(RefCell::new(Inner {
                ctrl_reg,
                ctrl_dpram,
                in_endpoints: Default::default(),
                out_endpoints: Default::default(),
                next_offset: 0,
                read_setup: false,
            })),
        }
    }
}

impl UsbBusTrait for UsbBus {
    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        _interval: u8,
    ) -> UsbResult<EndpointAddress> {
        critical_section::with(|cs| {
            self.inner
                .borrow(cs)
                .borrow_mut()
                .ep_allocate(ep_addr, ep_dir, ep_type, max_packet_size)
        })
    }

    fn enable(&mut self) {
        critical_section::with(|cs| {
            let inner = self.inner.borrow(cs).borrow_mut();
            inner.ctrl_reg.inte().modify(|_, w| {
                w.buff_status()
                    .set_bit()
                    .bus_reset()
                    .set_bit()
                    .dev_resume_from_host()
                    .set_bit()
                    .dev_suspend()
                    .set_bit()
                    .setup_req()
                    .set_bit()
            });
            inner
                .ctrl_reg
                .sie_ctrl()
                .modify(|_, w| w.pullup_en().set_bit());
        })
    }

    fn reset(&self) {
        critical_section::with(|cs| {
            let mut inner = self.inner.borrow(cs).borrow_mut();
            inner
                .ctrl_reg
                .sie_status()
                .write(|w| w.bus_reset().clear_bit_by_one());
            inner
                .ctrl_reg
                .buff_status()
                .write(|w| unsafe { w.bits(0xFFFF_FFFF) });
            inner.ep_reset_all();
            inner.ctrl_reg.addr_endp().reset();
        })
    }

    fn set_device_address(&self, addr: u8) {
        critical_section::with(|cs| {
            let inner = self.inner.borrow(cs).borrow_mut();
            inner
                .ctrl_reg
                .addr_endp()
                .modify(|_, w| unsafe { w.address().bits(addr & 0x7F) });
            inner
                .ctrl_dpram
                .ep_buffer_control(0)
                .modify(|_, w| w.pid_0().set_bit());
            inner
                .ctrl_dpram
                .ep_buffer_control(1)
                .modify(|_, w| w.pid_0().set_bit());
        })
    }

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> UsbResult<usize> {
        critical_section::with(|cs| self.inner.borrow(cs).borrow_mut().ep_write(ep_addr, buf))
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> UsbResult<usize> {
        critical_section::with(|cs| self.inner.borrow(cs).borrow_mut().ep_read(ep_addr, buf))
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        critical_section::with(|cs| {
            let inner = self.inner.borrow(cs).borrow_mut();
            if ep_addr.index() == 0 {
                inner.ctrl_reg.ep_stall_arm().modify(|_, w| {
                    if ep_addr.is_in() {
                        w.ep0_in().bit(stalled)
                    } else {
                        w.ep0_out().bit(stalled)
                    }
                });
            }
            let index = ep_addr_to_ep_buf_ctrl_idx(ep_addr);
            inner
                .ctrl_dpram
                .ep_buffer_control(index)
                .modify(|_, w| w.stall().bit(stalled));
        })
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        critical_section::with(|cs| {
            let inner = self.inner.borrow(cs).borrow_mut();
            let index = ep_addr_to_ep_buf_ctrl_idx(ep_addr);
            inner
                .ctrl_dpram
                .ep_buffer_control(index)
                .read()
                .stall()
                .bit_is_set()
        })
    }

    fn suspend(&self) {}
    fn resume(&self) {}

    fn poll(&self) -> PollResult {
        critical_section::with(|cs| {
            let mut inner = self.inner.borrow(cs).borrow_mut();

            let ints = inner.ctrl_reg.ints().read();
            let mut buff_status = inner.ctrl_reg.buff_status().read().bits();

            if ints.bus_reset().bit_is_set() {
                return PollResult::Reset;
            } else if buff_status == 0 && ints.setup_req().bit_is_clear() {
                if ints.dev_suspend().bit_is_set() {
                    inner
                        .ctrl_reg
                        .sie_status()
                        .write(|w| w.suspended().clear_bit_by_one());
                    return PollResult::Suspend;
                } else if ints.dev_resume_from_host().bit_is_set() {
                    inner
                        .ctrl_reg
                        .sie_status()
                        .write(|w| w.resume().clear_bit_by_one());
                    return PollResult::Resume;
                }
                return PollResult::None;
            }

            let (mut ep_out, mut ep_in_complete, mut ep_setup): (u16, u16, u16) = (0, 0, 0);

            // IN Complete is reported once.
            inner
                .ctrl_reg
                .buff_status()
                .write(|w| unsafe { w.bits(0x5555_5555) });

            for i in 0..32u32 {
                if buff_status == 0 {
                    break;
                } else if (buff_status & 1) == 1 {
                    let is_in = (i & 1) == 0;
                    let ep_idx = i / 2;
                    if is_in {
                        ep_in_complete |= 1 << ep_idx;
                    } else {
                        ep_out |= 1 << ep_idx;
                    }
                }
                buff_status >>= 1;
            }

            if ints.setup_req().bit_is_set() {
                // Small max_packet_size_ep0 work-around (see rp-hal notes).
                inner
                    .ctrl_dpram
                    .ep_buffer_control(0)
                    .modify(|_, w| w.available_0().clear_bit());
                ep_setup |= 1;
                inner.read_setup = true;
            }

            PollResult::Data {
                ep_out,
                ep_in_complete,
                ep_setup,
            }
        })
    }

    const QUIRK_SET_ADDRESS_BEFORE_STATUS: bool = false;
}
