// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Minimal WIZnet W5500 driver in **MACRAW** mode.
//!
//! The W5500 is a "hardwired TCP/IP" controller, but socket 0 has a MACRAW mode
//! that turns the on-chip stack off and moves raw Ethernet frames instead --
//! so from the host's point of view it behaves like a plain MAC+PHY (the same
//! role the ENC28J60 played) and smoltcp runs the IP/ARP/DHCP/TCP stack on top.
//! MACRAW is far simpler and more robust than the ENC28J60 path: no receive
//! buffer pointer errata, no bank switching, no analog-sensitive receive FIFO.
//!
//! ## SPI framing (datasheet sec. 2)
//! Every access is `[addr_hi][addr_lo][control][data...]`, CS held low for the
//! whole frame. The control byte is `(BSB << 3) | (RWB << 2) | OM`, where BSB is
//! the block-select (common / socket-0 register / socket-0 TX / socket-0 RX),
//! RWB is 1 for write and 0 for read, and OM = 0 selects variable-length data
//! mode (address auto-increments, and socket-buffer accesses auto-wrap within
//! the buffer -- so the pointer is passed raw and no manual wrap is needed,
//! unlike the older W5100).

#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::{Operation, SpiDevice};

/// Largest Ethernet frame handled (1500 payload + 14 header). Matches the
/// caller-side buffer sizing used by the net task.
pub const MTU: usize = 1514;

/// Outcome of a `transmit()` call, for diagnostics/observability.
#[derive(Copy, Clone, PartialEq, Eq)]
pub struct TxReport {
    /// The frame was accepted and SEND completed (SENDOK).
    pub ok: bool,
    /// Always 1: the W5500 does its own retransmit/backoff. Reserved so the
    /// caller's TX ringbuf entry has an attempt slot if TX retries are added.
    pub attempts: u8,
    /// Final `Sn_IR` snapshot (SENDOK / TIMEOUT bits) for debugging.
    pub eir: u8,
}

// --- Block-select values (control byte bits 7:3) ------------------------------
const BSB_COMMON: u8 = 0x00;
const BSB_S0_REG: u8 = 0x01;
const BSB_S0_TX: u8 = 0x02;
const BSB_S0_RX: u8 = 0x03;

// --- Common registers ---------------------------------------------------------
const MR: u16 = 0x0000;
const SHAR: u16 = 0x0009; // source MAC, 6 bytes
const PHYCFGR: u16 = 0x002E;
const VERSIONR: u16 = 0x0039; // reads 0x04 on the W5500

// --- Socket 0 registers (BSB_S0_REG) ------------------------------------------
const SN_MR: u16 = 0x0000;
const SN_CR: u16 = 0x0001;
const SN_IR: u16 = 0x0002;
const SN_SR: u16 = 0x0003;
const SN_RXBUF_SIZE: u16 = 0x001E;
const SN_TXBUF_SIZE: u16 = 0x001F;
const SN_TX_FSR: u16 = 0x0020; // free size, 2 bytes
const SN_TX_WR: u16 = 0x0024; // write pointer, 2 bytes
const SN_RX_RSR: u16 = 0x0026; // received size, 2 bytes
const SN_RX_RD: u16 = 0x0028; // read pointer, 2 bytes
const SN_RX_WR: u16 = 0x002A; // write pointer, 2 bytes

// --- Register bit/command values ----------------------------------------------
const MR_RST: u8 = 0x80; // software reset
const SN_MR_MACRAW: u8 = 0x04;
const SN_MR_MF: u8 = 0x80; // MAC filter: accept own-MAC + broadcast only
const SN_MR_MMB: u8 = 0x20; // block multicast (unused by an IPv4 unicast host)
const SN_MR_MIP6B: u8 = 0x10; // block IPv6 (this host is IPv4-only)
const CMD_OPEN: u8 = 0x01;
const CMD_SEND: u8 = 0x20;
const CMD_RECV: u8 = 0x40;
const SR_MACRAW: u8 = 0x42; // Sn_SR once opened in MACRAW
const IR_SENDOK: u8 = 0x10;
const IR_TIMEOUT: u8 = 0x08;
const PHY_LINK: u8 = 0x01; // PHYCFGR bit 0

/// Bound on status-poll spins so a dead chip cannot wedge the caller.
const SPIN_LIMIT: u32 = 200_000;

/// W5500 driver over a blocking `SpiDevice` (CS managed by the device) and an
/// optional active-low reset line.
pub struct W5500<SPI, RST> {
    spi: SPI,
    rst: Option<RST>,
    mac: [u8; 6],
}

impl<SPI, RST> W5500<SPI, RST>
where
    SPI: SpiDevice<u8>,
    RST: OutputPin,
{
    /// Construct and initialise: reset, verify the part, program the MAC, hand
    /// the whole 16 KiB buffer to socket 0, and open it in MACRAW mode.
    pub fn new<D: DelayNs>(
        spi: SPI,
        rst: Option<RST>,
        delay: &mut D,
        mac: [u8; 6],
    ) -> Self {
        let mut dev = W5500 { spi, rst, mac };
        dev.init(delay);
        dev
    }

    /// Full (re)initialisation. Also used by the caller's RX watchdog.
    pub fn reinit<D: DelayNs>(&mut self, delay: &mut D) {
        self.init(delay);
    }

    fn init<D: DelayNs>(&mut self, delay: &mut D) {
        // Hardware reset if a RST pin is wired (>= 500 us low per datasheet),
        // then a software reset for good measure.
        if let Some(rst) = &mut self.rst {
            let _ = rst.set_high();
            delay.delay_ms(1);
            let _ = rst.set_low();
            delay.delay_us(500);
            let _ = rst.set_high();
            delay.delay_ms(2);
        }
        self.write_u8(BSB_COMMON, MR, MR_RST);
        delay.delay_ms(2);

        // Program the source MAC (copy out first to avoid aliasing &self.mac
        // with the &mut self write path).
        let mac = self.mac;
        self.write(BSB_COMMON, SHAR, &mac);

        // Give socket 0 the entire 16 KiB RX and 16 KiB TX buffer, and take the
        // other seven sockets down to 0 (default is 2 KiB each = 16 KiB total,
        // so socket 0 cannot grow without shrinking the rest).
        self.write_u8(BSB_S0_REG, SN_RXBUF_SIZE, 16);
        self.write_u8(BSB_S0_REG, SN_TXBUF_SIZE, 16);
        for n in 1..8u8 {
            let bsb = 4 * n + 1; // socket n register block-select
            self.write_u8(bsb, SN_RXBUF_SIZE, 0);
            self.write_u8(bsb, SN_TXBUF_SIZE, 0);
        }

        // Open socket 0 in MACRAW filtering to what an IPv4 unicast host needs:
        // MAC filter (own-MAC + broadcast) plus multicast and IPv6 blocking, so
        // the RX buffer is not churned by IPv6 ND/multicast we would only
        // discard in smoltcp anyway. Then wait for it to report SOCK_MACRAW.
        self.write_u8(
            BSB_S0_REG,
            SN_MR,
            SN_MR_MACRAW | SN_MR_MF | SN_MR_MMB | SN_MR_MIP6B,
        );
        self.write_u8(BSB_S0_REG, SN_CR, CMD_OPEN);
        for _ in 0..SPIN_LIMIT {
            if self.read_u8(BSB_S0_REG, SN_SR) == SR_MACRAW {
                break;
            }
        }
    }

    /// Chip version register (`VERSIONR`); reads `0x04` on a healthy W5500.
    /// A bring-up smoke test that also confirms the SPI link is sane.
    pub fn version(&mut self) -> u8 {
        self.read_u8(BSB_COMMON, VERSIONR)
    }

    /// True if the PHY reports link up.
    pub fn is_link_up(&mut self) -> bool {
        self.read_u8(BSB_COMMON, PHYCFGR) & PHY_LINK != 0
    }

    /// True if socket 0 is still in MACRAW (`Sn_SR == SOCK_MACRAW`). A `false`
    /// here is an unambiguous "the receiver is broken" signal -- the socket
    /// fell out of raw mode -- which the caller's watchdog uses to decide a
    /// hardware reset (as opposed to mere RX silence, which may just be a quiet
    /// network).
    pub fn in_macraw(&mut self) -> bool {
        self.read_u8(BSB_S0_REG, SN_SR) == SR_MACRAW
    }

    /// Receive one Ethernet frame into `buf`, or `None` if the RX buffer is
    /// empty. Each MACRAW packet in the buffer is `[u16 len][frame]` where `len`
    /// counts the 2-byte header itself.
    pub fn receive(&mut self, buf: &mut [u8]) -> Option<usize> {
        let rsr = self.read_u16_stable(BSB_S0_REG, SN_RX_RSR);
        if rsr < 2 {
            return None;
        }
        let rd = self.read_u16(BSB_S0_REG, SN_RX_RD);

        let mut hdr = [0u8; 2];
        self.read(BSB_S0_RX, rd, &mut hdr);
        let plen = u16::from_be_bytes(hdr);

        // A sane packet is at least the 2-byte header and no larger than what
        // the chip says is buffered. Anything else means the pointers are out
        // of sync: flush the buffer (RX_RD = RX_WR) and resync via RECV.
        if plen < 2 || plen > rsr {
            let wr = self.read_u16_stable(BSB_S0_REG, SN_RX_WR);
            self.write_u16(BSB_S0_REG, SN_RX_RD, wr);
            self.write_u8(BSB_S0_REG, SN_CR, CMD_RECV);
            return None;
        }

        let frame_len = (plen - 2) as usize;

        // Consume the whole packet from the chip regardless, so the pointer
        // stays in sync. A frame too big for `buf` (cannot happen at MTU sizing,
        // but guard rather than hand smoltcp a silently truncated frame) is
        // dropped by returning None after advancing.
        let fits = frame_len <= buf.len();
        if fits {
            self.read(BSB_S0_RX, rd.wrapping_add(2), &mut buf[..frame_len]);
        }
        self.write_u16(BSB_S0_REG, SN_RX_RD, rd.wrapping_add(plen));
        self.write_u8(BSB_S0_REG, SN_CR, CMD_RECV);
        fits.then_some(frame_len)
    }

    /// Transmit one Ethernet frame. The caller supplies a complete frame
    /// (dst/src/ethertype + payload); the W5500 appends the FCS.
    pub fn transmit(&mut self, frame: &[u8]) -> TxReport {
        let len = frame.len() as u16;

        // Wait for enough free space in the TX buffer.
        let mut spins = 0u32;
        while self.read_u16_stable(BSB_S0_REG, SN_TX_FSR) < len {
            spins += 1;
            if spins > SPIN_LIMIT {
                return TxReport {
                    ok: false,
                    attempts: 1,
                    eir: 0,
                };
            }
        }

        let wr = self.read_u16(BSB_S0_REG, SN_TX_WR);
        self.write(BSB_S0_TX, wr, frame);
        self.write_u16(BSB_S0_REG, SN_TX_WR, wr.wrapping_add(len));
        self.write_u8(BSB_S0_REG, SN_CR, CMD_SEND);

        // Wait for SENDOK (or a TIMEOUT abort); clear whichever fired.
        let mut spins = 0u32;
        loop {
            let ir = self.read_u8(BSB_S0_REG, SN_IR);
            if ir & (IR_SENDOK | IR_TIMEOUT) != 0 {
                self.write_u8(BSB_S0_REG, SN_IR, IR_SENDOK | IR_TIMEOUT);
                return TxReport {
                    ok: ir & IR_SENDOK != 0,
                    attempts: 1,
                    eir: ir,
                };
            }
            spins += 1;
            if spins > SPIN_LIMIT {
                return TxReport {
                    ok: false,
                    attempts: 1,
                    eir: 0,
                };
            }
        }
    }

    /// Receiver diagnostics for observability: `(RX received size, Sn_IR,
    /// Sn_SR, PHYCFGR)`. RX size climbing with no frames drained means we are
    /// failing to pull; `Sn_SR != 0x42` means the socket fell out of MACRAW.
    pub fn rx_diag(&mut self) -> (u16, u8, u8, u8) {
        (
            self.read_u16(BSB_S0_REG, SN_RX_RSR),
            self.read_u8(BSB_S0_REG, SN_IR),
            self.read_u8(BSB_S0_REG, SN_SR),
            self.read_u8(BSB_COMMON, PHYCFGR),
        )
    }

    // --- SPI primitives -------------------------------------------------------

    /// Control byte for a block-select and direction (variable data mode).
    fn ctrl(bsb: u8, write: bool) -> u8 {
        (bsb << 3) | ((write as u8) << 2)
    }

    fn read(&mut self, bsb: u8, addr: u16, buf: &mut [u8]) {
        let hdr = [(addr >> 8) as u8, addr as u8, Self::ctrl(bsb, false)];
        let _ = self
            .spi
            .transaction(&mut [Operation::Write(&hdr), Operation::Read(buf)]);
    }

    fn write(&mut self, bsb: u8, addr: u16, data: &[u8]) {
        let hdr = [(addr >> 8) as u8, addr as u8, Self::ctrl(bsb, true)];
        let _ = self
            .spi
            .transaction(&mut [Operation::Write(&hdr), Operation::Write(data)]);
    }

    fn read_u8(&mut self, bsb: u8, addr: u16) -> u8 {
        let mut b = [0u8; 1];
        self.read(bsb, addr, &mut b);
        b[0]
    }

    fn write_u8(&mut self, bsb: u8, addr: u16, val: u8) {
        self.write(bsb, addr, &[val]);
    }

    /// Read a 16-bit big-endian register in one transaction. For host-controlled
    /// registers (Sn_RX_RD, Sn_TX_WR) that do not change under us between writes.
    fn read_u16(&mut self, bsb: u8, addr: u16) -> u16 {
        let mut a = [0u8; 2];
        self.read(bsb, addr, &mut a);
        u16::from_be_bytes(a)
    }

    /// Read a chip-updated 16-bit register (Sn_RX_RSR, Sn_TX_FSR, Sn_RX_WR),
    /// which can change under an in-flight DMA: read until two reads agree
    /// (datasheet's recommended read-until-consistent). Bounded by SPIN_LIMIT so
    /// a wedged or absent chip -- whose reads never stabilise -- cannot hang the
    /// poll loop (and thus the `eth` IPC); returns the last sample at the bound.
    fn read_u16_stable(&mut self, bsb: u8, addr: u16) -> u16 {
        let mut a = [0u8; 2];
        self.read(bsb, addr, &mut a);
        for _ in 0..SPIN_LIMIT {
            let mut b = [0u8; 2];
            self.read(bsb, addr, &mut b);
            if a == b {
                break;
            }
            a = b;
        }
        u16::from_be_bytes(a)
    }

    fn write_u16(&mut self, bsb: u8, addr: u16, val: u16) {
        self.write(bsb, addr, &val.to_be_bytes());
    }
}
