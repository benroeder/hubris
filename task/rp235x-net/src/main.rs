// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Minimal network task for the RP2350: WIZnet W5500 (SPI, MACRAW) + smoltcp
//! + DHCPv4.
//!
//! Owns SPI0's PL022 directly (the sdcard/SPI1 pattern): a W5500 access holds
//! CS low across a whole `[addr][control][data]` frame, which the shared IPC
//! SPI server cannot express, and an IPC round-trip per register access would
//! dominate the packet path. Pins: SCK=GP2, MOSI=GP3, MISO=GP16 (SPI0
//! funcsel), CS=GP17 and RST=GP14 (manual SIO, active low). INT (the W5500's
//! open-drain active-low interrupt) is wired to GP9 but unused for now -- we
//! poll; it is reserved so an interrupt-driven RX fast-path is a firmware-only
//! change with no rewiring. The `w5500` driver
//! core (lib/w5500) runs socket 0 in MACRAW mode -- raw Ethernet frames rather
//! than the chip's TCP/IP offload -- so this task implements the embedded-hal
//! 1.0 SPI traits over the PL022, wraps the driver in a `smoltcp::phy::Device`,
//! and runs the interface poll loop off the kernel timer, exactly as it would
//! over a plain MAC+PHY.
//!
//! With IPv4 enabled, smoltcp answers ICMP echo at the interface level, so
//! the first end-to-end milestone -- the board answering `ping` -- needs no
//! socket beyond the DHCPv4 client that obtains the address.

#![no_std]
#![no_main]

use core::convert::Infallible;

use drv_rp235x_sys_api::{self as sys_api, Rp235xSys};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::{ErrorType, Operation, SpiDevice};
use smoltcp::iface::{Config, Interface, SocketSet, SocketStorage};
use smoltcp::phy::{self, DeviceCapabilities, Medium};
use smoltcp::socket::dhcpv4;
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpCidr, Ipv4Address, Ipv4Cidr};
use userlib::{RecvMessage, sys_get_timer, sys_set_timer, task_slot};

use ringbuf::{ringbuf, ringbuf_entry};

task_slot!(SYS, sys);

/// Traffic trace, read with `humility ringbuf`. `Rx`/`Tx` carry the ethernet
/// EtherType (0x0806 = ARP, 0x0800 = IPv4) and frame length so we can see
/// whether broadcast ARP requests reach smoltcp and whether a reply goes out.
#[derive(Copy, Clone, PartialEq)]
enum Trace {
    None,
    Rx {
        ethertype: u16,
        len: u16,
    },
    Tx {
        ethertype: u16,
        len: u16,
        ok: bool,
        attempts: u8,
    },
    DhcpConfigured(u32),
    DhcpDeconfigured,
    StaticFallback(u32),
    /// Periodic W5500 receiver health: (Sn_RX_RSR, Sn_IR, Sn_SR, PHYCFGR).
    /// RX size climbing with no `Rx` events means the chip receives but we fail
    /// to drain; `sr != 0x42` means socket 0 fell out of MACRAW; `phy & 1` is
    /// link.
    RxDiag {
        rx_size: u16,
        ir: u8,
        sr: u8,
        phy: u8,
    },
    /// W5500 VERSIONR at bring-up (expect 0x04).
    Version(u8),
    /// The RX watchdog fired: link up but no frames received for the timeout,
    /// so the chip was hardware-reset and reinitialised.
    RxWatchdogReset,
}

ringbuf!(Trace, 128, Trace::None);

/// EtherType field (bytes 12..14) of an ethernet frame, or 0 if too short.
fn ethertype(frame: &[u8]) -> u16 {
    if frame.len() >= 14 {
        u16::from_be_bytes([frame[12], frame[13]])
    } else {
        0
    }
}

// --- Pin map (SPI0 mux group; see the module docs) ---------------------------
/// SPI0 SCK (funcsel 1).
const SCK: u32 = 2;
/// SPI0 MOSI / TX (funcsel 1).
const MOSI: u32 = 3;
/// SPI0 MISO / RX (funcsel 1).
const MISO: u32 = 16;
/// W5500 hardware reset, manual SIO GPIO (active low).
const RST: u32 = 14;
/// Chip select, manual SIO GPIO (active low).
const CS: u32 = 17;
/// funcsel routing a pin to the SPI function.
const FUNCSEL_SPI: u8 = 1;
/// funcsel routing a pin to SIO.
const FUNCSEL_SIO: u8 = 5;

// --- PL022 clocking -----------------------------------------------------------
/// SPI clock = clk_peri / (CPSDVSR * (1 + SCR)) = 150 MHz / 8 = 18.75 MHz.
/// The W5500 handles up to ~80 MHz SPI, so this is very conservative -- chosen
/// to stay well within signal-integrity margin on jumper wiring; it can be
/// raised later if the layout allows.
const CPSDVSR: u8 = 8;
/// Serial clock rate divider (see CPSDVSR).
const SCR: u8 = 0;
/// PL022 data-size select for 8-bit frames (DSS = N-1).
const DSS_8BIT: u8 = 0x7;
/// Bound on FIFO-status spins so a dead SPI cannot wedge the server.
const SPIN_LIMIT: u32 = 1_000_000;

/// Locally-administered MAC address (bit 1 of the first octet set). Fixed for
/// bring-up; deriving from the chip's unique ID is a follow-up.
const MAC_ADDR: [u8; 6] = [0x02, 0xC0, 0xFF, 0xEE, 0x28, 0x61];

/// Static-IP fallback: if DHCP does not bind within `STATIC_FALLBACK_MS` of
/// boot, self-assign this address so the board is still reachable on networks
/// whose DHCP server will not lease to it. DHCP stays primary -- a later lease
/// replaces the static address.
const STATIC_ADDR: Ipv4Address = Ipv4Address::new(10, 110, 10, 201);
const STATIC_PREFIX: u8 = 24;
const STATIC_GW: Ipv4Address = Ipv4Address::new(10, 110, 10, 1);
/// Grace period for DHCP before falling back to the static address.
const STATIC_FALLBACK_MS: u64 = 8_000;

/// Poll cadence when idle, in ms. Inbound packets are discovered by polling the
/// W5500's RX size, so a short interval keeps ping latency low and drains the
/// RX buffer often enough that a busy segment cannot overflow it between polls
/// (the earlier 20 ms dropped ~20% under broadcast load). 5 ms is still cheap on
/// the 150 MHz core; the INT line (GP9) is wired for an interrupt fast-path if
/// this ever needs to be tighter.
const POLL_MS: u64 = 5;

/// RX watchdog: after this long with the link up but no frames received, the
/// driver checks whether socket 0 is still in MACRAW and hardware-resets only
/// if it is not. Silence alone never triggers a reset (it may be a quiet
/// network), so this window can be short without risking spurious resets.
const RX_WATCHDOG_MS: u64 = 4_000;

/// clk_sys cycles per microsecond (150 MHz).
const CYCLES_PER_US: u32 = 150;

// --- embedded-hal 1.0 shims over the PL022 -----------------------------------

/// `SpiDevice` over SPI0 with manual CS: one `transaction` = CS asserted
/// around all operations, which is exactly the contract the W5500's
/// `[addr][control][data]` frames need (CS low for the whole frame).
struct Spi0Dev {
    spi: rp235x_pac::SPI0,
    sio: rp235x_pac::SIO,
}

impl Spi0Dev {
    /// Full-duplex byte exchange (bounded spins; see SPIN_LIMIT).
    fn xfer(&mut self, out: u8) -> u8 {
        let mut s = 0u32;
        while self.spi.sspsr().read().tnf().bit_is_clear() {
            s += 1;
            if s > SPIN_LIMIT {
                break;
            }
        }
        self.spi
            .sspdr()
            .write(|w| unsafe { w.data().bits(out as u16) });
        let mut s = 0u32;
        while self.spi.sspsr().read().rne().bit_is_clear() {
            s += 1;
            if s > SPIN_LIMIT {
                return 0xFF;
            }
        }
        self.spi.sspdr().read().data().bits() as u8
    }
}

impl ErrorType for Spi0Dev {
    type Error = Infallible;
}

impl SpiDevice<u8> for Spi0Dev {
    fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Infallible> {
        self.sio
            .gpio_out_clr()
            .write(|w| unsafe { w.bits(1 << CS) });
        for op in operations {
            match op {
                Operation::Read(buf) => {
                    for b in buf.iter_mut() {
                        *b = self.xfer(0);
                    }
                }
                Operation::Write(buf) => {
                    for b in buf.iter() {
                        self.xfer(*b);
                    }
                }
                Operation::Transfer(rd, wr) => {
                    // Not used by the w5500 core; provided for trait
                    // completeness (unequal lengths pad with zeros).
                    let n = rd.len().max(wr.len());
                    for i in 0..n {
                        let out = wr.get(i).copied().unwrap_or(0);
                        let v = self.xfer(out);
                        if let Some(r) = rd.get_mut(i) {
                            *r = v;
                        }
                    }
                }
                Operation::TransferInPlace(buf) => {
                    for b in buf.iter_mut() {
                        *b = self.xfer(*b);
                    }
                }
                Operation::DelayNs(ns) => {
                    cortex_m::asm::delay(ns.div_ceil(1000) * CYCLES_PER_US);
                }
            }
        }
        self.sio
            .gpio_out_set()
            .write(|w| unsafe { w.bits(1 << CS) });
        Ok(())
    }
}

/// W5500 hardware reset line: GP14, active low. Driven through the SIO atomic
/// set/clear registers via a raw pointer so it can coexist with the CS line
/// (GP17) that owns the `SIO` peripheral -- the two touch disjoint bits of the
/// write-1-to-act registers, so there is no interference. The driver pulses
/// this in `init()`/`reinit()` for a clean hardware reset at bring-up and on
/// watchdog recovery.
struct RstPin;

impl embedded_hal::digital::ErrorType for RstPin {
    type Error = Infallible;
}

impl OutputPin for RstPin {
    fn set_low(&mut self) -> Result<(), Infallible> {
        // SAFETY: atomic write-1-to-clear of a single GPIO bit; disjoint from
        // the CS bit driven via the owned SIO handle.
        unsafe {
            (*rp235x_pac::SIO::ptr())
                .gpio_out_clr()
                .write(|w| w.bits(1 << RST));
        }
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Infallible> {
        // SAFETY: atomic write-1-to-set of a single GPIO bit.
        unsafe {
            (*rp235x_pac::SIO::ptr())
                .gpio_out_set()
                .write(|w| w.bits(1 << RST));
        }
        Ok(())
    }
}

/// Busy-wait `DelayNs` (only used during driver construction).
struct SpinDelay;

impl DelayNs for SpinDelay {
    fn delay_ns(&mut self, ns: u32) {
        cortex_m::asm::delay(ns.div_ceil(1000) * CYCLES_PER_US);
    }
}

// --- smoltcp phy::Device over the driver core ---------------------------------

type Driver = w5500::W5500<Spi0Dev, RstPin>;

/// smoltcp device: the driver core plus one receive-side frame buffer. The
/// transmit token writes frames into a second buffer and hands them straight
/// to the chip (which has its own 8 KiB packet memory).
struct EthDevice {
    drv: Driver,
    rx: [u8; w5500::MTU],
    tx: [u8; w5500::MTU],
    /// Total frames pulled from the chip -- the RX watchdog watches this for
    /// forward progress.
    rx_count: u32,
}

struct EthRxToken<'a> {
    frame: &'a mut [u8],
}

struct EthTxToken<'a> {
    drv: &'a mut Driver,
    buf: &'a mut [u8; w5500::MTU],
}

impl phy::RxToken for EthRxToken<'_> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        f(self.frame)
    }
}

impl phy::TxToken for EthTxToken<'_> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let r = f(&mut self.buf[..len]);
        let et = ethertype(&self.buf[..len]);
        let report = self.drv.transmit(&self.buf[..len]);
        ringbuf_entry!(Trace::Tx {
            ethertype: et,
            len: len as u16,
            ok: report.ok,
            attempts: report.attempts,
        });
        r
    }
}

impl phy::Device for EthDevice {
    type RxToken<'a>
        = EthRxToken<'a>
    where
        Self: 'a;
    type TxToken<'a>
        = EthTxToken<'a>
    where
        Self: 'a;

    fn receive(
        &mut self,
        _: Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        // The W5500 driver returns None when the MACRAW RX buffer is empty.
        let n = self.drv.receive(&mut self.rx)?;
        self.rx_count = self.rx_count.wrapping_add(1);
        ringbuf_entry!(Trace::Rx {
            ethertype: ethertype(&self.rx[..n as usize]),
            len: n as u16,
        });
        Some((
            EthRxToken {
                frame: &mut self.rx[..n as usize],
            },
            EthTxToken {
                drv: &mut self.drv,
                buf: &mut self.tx,
            },
        ))
    }

    fn transmit(&mut self, _: Instant) -> Option<Self::TxToken<'_>> {
        Some(EthTxToken {
            drv: &mut self.drv,
            buf: &mut self.tx,
        })
    }

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = w5500::MTU;
        caps.medium = Medium::Ethernet;
        caps
    }
}

// --- The server ----------------------------------------------------------------

struct ServerImpl<'a> {
    dev: EthDevice,
    iface: Interface,
    sockets: SocketSet<'a>,
    dhcp_handle: smoltcp::iface::SocketHandle,
    /// DHCP has configured an address.
    bound: bool,
    /// The static fallback address is currently installed.
    static_active: bool,
    /// Cached (address, prefix) for whichever address is installed.
    addr: (Ipv4Address, u8),
    /// Poll counter, for throttling periodic diagnostics.
    polls: u32,
    /// Kernel timer (ms) sampled at task start; the DHCP grace period is
    /// measured relative to this so a task restart on a long-up system still
    /// gives DHCP its full window.
    start_ms: u64,
    /// Last observed `dev.rx_count` and the timestamp it last advanced -- the
    /// RX watchdog hard-resets the chip if it goes deaf while the link is up.
    last_rx_count: u32,
    last_rx_ms: u64,
}

impl ServerImpl<'_> {
    /// One interface poll pass: drive smoltcp, apply DHCP state changes.
    fn poll(&mut self) {
        // Periodic receiver health check (~every 250 polls) so a chip that is
        // receiving-but-not-draining vs hearing-nothing is distinguishable
        // even when zero frames reach the driver.
        self.polls = self.polls.wrapping_add(1);
        if self.polls % 250 == 0 {
            let (rx_size, ir, sr, phy) = self.dev.drv.rx_diag();
            ringbuf_entry!(Trace::RxDiag {
                rx_size,
                ir,
                sr,
                phy
            });
        }

        let now = Instant::from_millis(sys_get_timer().now as i64);
        self.iface.poll(now, &mut self.dev, &mut self.sockets);

        let dhcp = self.sockets.get_mut::<dhcpv4::Socket>(self.dhcp_handle);
        match dhcp.poll() {
            None => {}
            Some(dhcpv4::Event::Configured(config)) => {
                self.bound = true;
                self.static_active = false;
                self.addr =
                    (config.address.address(), config.address.prefix_len());
                ringbuf_entry!(Trace::DhcpConfigured(u32::from_be_bytes(
                    config.address.address().0
                )));
                self.iface.update_ip_addrs(|addrs| {
                    addrs.clear();
                    let _ = addrs.push(IpCidr::Ipv4(config.address));
                });
                if let Some(router) = config.router {
                    let _ =
                        self.iface.routes_mut().add_default_ipv4_route(router);
                } else {
                    self.iface.routes_mut().remove_default_ipv4_route();
                }
            }
            Some(dhcpv4::Event::Deconfigured) => {
                self.bound = false;
                self.addr = (Ipv4Address::UNSPECIFIED, 0);
                ringbuf_entry!(Trace::DhcpDeconfigured);
                self.iface.update_ip_addrs(|addrs| addrs.clear());
                self.iface.routes_mut().remove_default_ipv4_route();
            }
        }

        let now_ms = sys_get_timer().now;

        // Static-IP fallback: if DHCP has not bound within the grace period and
        // we have not already self-assigned, install the static address so the
        // board is reachable even where the DHCP server will not lease to it.
        // Measured from task start (`start_ms`), NOT system uptime -- a task
        // restart on a long-up system must still give DHCP its full grace
        // window rather than falling back on the first poll. DHCP stays primary
        // -- a later `Configured` clears `static_active` and overwrites `addr`.
        if !self.bound
            && !self.static_active
            && now_ms.wrapping_sub(self.start_ms) >= STATIC_FALLBACK_MS
        {
            self.static_active = true;
            self.addr = (STATIC_ADDR, STATIC_PREFIX);
            self.iface.update_ip_addrs(|addrs| {
                addrs.clear();
                let _ = addrs.push(IpCidr::Ipv4(Ipv4Cidr::new(
                    STATIC_ADDR,
                    STATIC_PREFIX,
                )));
            });
            let _ = self.iface.routes_mut().add_default_ipv4_route(STATIC_GW);
            ringbuf_entry!(Trace::StaticFallback(u32::from_be_bytes(
                STATIC_ADDR.0
            )));
        }

        // RX watchdog: track forward progress; on prolonged RX silence with the
        // link up, hard-reset the chip ONLY if socket 0 has actually fallen out
        // of MACRAW (a definite failure). Mere silence is not treated as a fault
        // -- it may be a quiet network -- so this neither resets a healthy idle
        // link (or disrupts an in-flight DHCP handshake) nor leaves a genuinely
        // broken chip stuck, whether or not a lease is currently held. The two
        // extra SPI reads (link, then Sn_SR) run at most once per watchdog
        // window thanks to `&&` short-circuiting.
        if self.dev.rx_count != self.last_rx_count {
            self.last_rx_count = self.dev.rx_count;
            self.last_rx_ms = now_ms;
        } else if now_ms.wrapping_sub(self.last_rx_ms) > RX_WATCHDOG_MS
            && self.dev.drv.is_link_up()
        {
            self.last_rx_ms = now_ms;
            if !self.dev.drv.in_macraw() {
                ringbuf_entry!(Trace::RxWatchdogReset);
                self.dev.drv.reinit(&mut SpinDelay);
            }
        }
    }

    /// Arm the kernel timer for the next poll: smoltcp's own deadline when it
    /// has one sooner than the idle cadence.
    fn arm_timer(&mut self) {
        let now_ms = sys_get_timer().now;
        let now = Instant::from_millis(now_ms as i64);
        let dt = self
            .iface
            .poll_delay(now, &self.sockets)
            .map(|d| d.total_millis() as u64)
            .unwrap_or(POLL_MS)
            .clamp(1, POLL_MS);
        sys_set_timer(Some(now_ms + dt), notifications::TIMER_MASK);
    }
}

impl idl::InOrderRp235xNetImpl for ServerImpl<'_> {
    fn status(
        &mut self,
        _: &RecvMessage,
    ) -> Result<u64, idol_runtime::RequestError<core::convert::Infallible>>
    {
        let mut s = 0u64;
        if self.dev.drv.is_link_up() {
            s |= task_rp235x_net_api::STATUS_LINK_UP;
        }
        if self.bound || self.static_active {
            s |= if self.bound {
                task_rp235x_net_api::STATUS_BOUND
            } else {
                task_rp235x_net_api::STATUS_STATIC
            };
            s |= (self.addr.1 as u64) << 32;
            s |= u32::from_be_bytes(self.addr.0.0) as u64;
        }
        Ok(s)
    }
}

impl idol_runtime::NotificationHandler for ServerImpl<'_> {
    fn current_notification_mask(&self) -> u32 {
        notifications::TIMER_MASK
    }

    fn handle_notification(&mut self, bits: userlib::NotificationBits) {
        if bits.check_notification_mask(notifications::TIMER_MASK) {
            self.poll();
            self.arm_timer();
        }
    }
}

/// One-time hardware setup: SPI0 pin mux + manual CS + PL022 mode-0 at
/// 18.75 MHz. Mirrors the sdcard driver's SPI1 bring-up.
fn setup(p: &rp235x_pac::Peripherals) {
    for pin in [SCK, MOSI, MISO] {
        p.PADS_BANK0.gpio(pin as usize).modify(|_, w| {
            w.od().clear_bit();
            w.iso().clear_bit();
            w.ie().set_bit()
        });
        p.IO_BANK0
            .gpio(pin as usize)
            .gpio_ctrl()
            .modify(|_, w| unsafe { w.funcsel().bits(FUNCSEL_SPI) });
    }

    // CS: SIO output, idle high; level set before enabling the driver so the
    // chip never sees a spurious select.
    p.PADS_BANK0.gpio(CS as usize).modify(|_, w| {
        w.od().clear_bit();
        w.iso().clear_bit();
        w.ie().set_bit()
    });
    p.IO_BANK0
        .gpio(CS as usize)
        .gpio_ctrl()
        .modify(|_, w| unsafe { w.funcsel().bits(FUNCSEL_SIO) });
    p.SIO.gpio_out_set().write(|w| unsafe { w.bits(1 << CS) });
    p.SIO.gpio_oe_set().write(|w| unsafe { w.bits(1 << CS) });

    // RST: SIO output, idle high (deasserted). The driver's init() pulses it
    // low to hardware-reset the W5500 before configuring it.
    p.PADS_BANK0.gpio(RST as usize).modify(|_, w| {
        w.od().clear_bit();
        w.iso().clear_bit();
        w.ie().set_bit()
    });
    p.IO_BANK0
        .gpio(RST as usize)
        .gpio_ctrl()
        .modify(|_, w| unsafe { w.funcsel().bits(FUNCSEL_SIO) });
    p.SIO.gpio_out_set().write(|w| unsafe { w.bits(1 << RST) });
    p.SIO.gpio_oe_set().write(|w| unsafe { w.bits(1 << RST) });

    let spi = &p.SPI0;
    spi.sspcr1().write(|w| w.sse().clear_bit());
    spi.sspcpsr()
        .write(|w| unsafe { w.cpsdvsr().bits(CPSDVSR) });
    spi.sspcr0().write(|w| unsafe {
        w.dss().bits(DSS_8BIT);
        w.spo().clear_bit();
        w.sph().clear_bit();
        w.scr().bits(SCR)
    });
    spi.sspcr1().write(|w| {
        w.ms().clear_bit();
        w.lbm().clear_bit();
        w.sse().set_bit()
    });
}

#[export_name = "main"]
fn main() -> ! {
    // Crash-loop breather: if anything below panics, jefe restarts this task
    // immediately; without a pause the fault-restart cycle can consume the
    // system. 300 ms per lap keeps everything else responsive while still
    // retrying.
    userlib::hl::sleep_for(300);

    let sys = Rp235xSys::from(SYS.get_task_id());
    sys.leave_reset(sys_api::SPI0);

    let p = unsafe { rp235x_pac::Peripherals::steal() };
    setup(&p);

    let spi = Spi0Dev {
        spi: p.SPI0,
        sio: p.SIO,
    };
    let drv = w5500::W5500::new(spi, Some(RstPin), &mut SpinDelay, MAC_ADDR);

    let mut dev = EthDevice {
        drv,
        rx: [0; w5500::MTU],
        tx: [0; w5500::MTU],
        rx_count: 0,
    };
    // Bring-up smoke test: VERSIONR reads 0x04 on a healthy W5500, and a good
    // read also proves the SPI link is sane.
    ringbuf_entry!(Trace::Version(dev.drv.version()));

    let mut config = Config::new();
    config.hardware_addr = Some(EthernetAddress(MAC_ADDR).into());
    config.random_seed = 0x0060_2860_c0ff_ee00;
    let iface = Interface::new(config, &mut dev);

    // Socket storage: just the DHCP client (ICMP echo needs no socket).
    static mut SOCKET_STORAGE: [SocketStorage<'_>; 1] =
        [SocketStorage::EMPTY; 1];
    // SAFETY: taken exactly once here; the task is single-threaded.
    #[allow(static_mut_refs)]
    let mut sockets = SocketSet::new(unsafe { &mut SOCKET_STORAGE[..] });
    let dhcp_handle = sockets.add(dhcpv4::Socket::new());

    let mut server = ServerImpl {
        dev,
        iface,
        sockets,
        dhcp_handle,
        bound: false,
        static_active: false,
        addr: (Ipv4Address::UNSPECIFIED, 0),
        polls: 0,
        start_ms: sys_get_timer().now,
        last_rx_count: 0,
        last_rx_ms: 0,
    };
    server.poll();
    server.arm_timer();

    let mut incoming = [0u8; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}

include!(concat!(env!("OUT_DIR"), "/notifications.rs"));
