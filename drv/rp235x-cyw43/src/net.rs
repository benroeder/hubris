// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! smoltcp transport over the CYW43 F2 DATA path.
//!
//! smoltcp owns Ethernet/ARP/IP/UDP/TCP + checksums + the TCP state machine.
//! On top we hand-roll only the servers smoltcp does not provide: a DHCP server
//! (UDP :67), a DNS hijack (UDP :53), and the HTTP captive portal (TCP :80).
//!
//! `Cyw43Device` is a no-alloc `phy::Device`: the RxToken owns a fixed frame
//! buffer (no Vec) and the TxToken holds `&mut` the wifi + scratch and calls
//! `send_frame` on consume.

use core::sync::atomic::Ordering::SeqCst;

use smoltcp::iface::{Config, Interface, SocketSet, SocketStorage};
use smoltcp::phy::{self, Device, DeviceCapabilities, Medium};
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpCidr, Ipv4Address, Ipv4Cidr};

use crate::{Cyw43, DIAG};

const MTU: usize = 1536;

pub struct Cyw43Device<'a> {
    wifi: &'a mut Cyw43,
    fr: &'a mut [u32; 512],
    /// DHCP is served here at the frame level (like Oxide's VLanEthernet handles
    /// VLAN), NOT via a smoltcp socket: smoltcp discards the DISCOVER before any
    /// socket because its source is 0.0.0.0 (non-unicast, dropped in process_ipv4).
    leases: Leases,
}

pub struct Cyw43RxToken {
    buf: [u8; MTU],
    len: usize,
}

impl phy::RxToken for Cyw43RxToken {
    fn consume<R, F: FnOnce(&mut [u8]) -> R>(mut self, f: F) -> R {
        f(&mut self.buf[..self.len])
    }
}

pub struct Cyw43TxToken<'a> {
    wifi: &'a mut Cyw43,
    fr: &'a mut [u32; 512],
}

impl phy::TxToken for Cyw43TxToken<'_> {
    fn consume<R, F: FnOnce(&mut [u8]) -> R>(self, len: usize, f: F) -> R {
        DIAG[7].store(DIAG[7].load(SeqCst).wrapping_add(1), SeqCst); // smoltcp TX (ARP etc.)
        let mut buf = [0u8; MTU];
        let r = f(&mut buf[..len]);
        self.wifi.send_frame(&buf[..len], self.fr);
        r
    }
}

impl Device for Cyw43Device<'_> {
    type RxToken<'b>
        = Cyw43RxToken
    where
        Self: 'b;
    type TxToken<'b>
        = Cyw43TxToken<'b>
    where
        Self: 'b;

    fn capabilities(&self) -> DeviceCapabilities {
        let mut c = DeviceCapabilities::default();
        c.max_transmission_unit = 1514;
        c.medium = Medium::Ethernet;
        c
    }

    fn receive(
        &mut self,
        _t: Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        // Drain DHCP frames inline (serve them by hand -- smoltcp drops the
        // 0.0.0.0-source DISCOVER) and keep pulling frames, so DHCP never starves
        // smoltcp's receive of the ARP/DNS/TCP frames behind it in the FIFO.
        loop {
            let mut buf = [0u8; MTU];
            let n = self.wifi.recv_frame(&mut buf, self.fr);
            if n == 0 {
                return None;
            }
            DIAG[9].store(DIAG[9].load(SeqCst).wrapping_add(1), SeqCst); // raw frames
            if is_dhcp_request(&buf[..n]) {
                handle_dhcp(self.wifi, self.fr, &buf[..n], &mut self.leases);
                continue;
            }
            if buf[12] == 0x08 && buf[13] == 0x06 {
                DIAG[6].store(DIAG[6].load(SeqCst).wrapping_add(1), SeqCst); // ARP in
            }
            // Count IPv4 frames addressed TO us (.1) -- does unicast-to-host work?
            if buf[12] == 0x08
                && buf[13] == 0x00
                && n >= 34
                && buf[30] == 192
                && buf[31] == 168
                && buf[32] == 4
                && buf[33] == 1
            {
                DIAG[5].store(DIAG[5].load(SeqCst).wrapping_add(1), SeqCst); // IPv4 to .1
            }
            let rx = Cyw43RxToken { buf, len: n };
            let tx = Cyw43TxToken {
                wifi: &mut *self.wifi,
                fr: &mut *self.fr,
            };
            return Some((rx, tx));
        }
    }

    fn transmit(&mut self, _t: Instant) -> Option<Self::TxToken<'_>> {
        Some(Cyw43TxToken {
            wifi: &mut *self.wifi,
            fr: &mut *self.fr,
        })
    }
}

impl Cyw43Device<'_> {
    /// Announce the gateway (192.168.4.1 -> our MAC) via a gratuitous ARP, so
    /// clients cache it without ARPing us. The CYW43 AP firmware does not hand
    /// client ARP-for-.1 to the host, so without this the gateway resolves very
    /// slowly (clients infer it from DHCP). With it, ping/DNS/HTTP to .1 work.
    fn send_gratuitous_arp(&mut self) {
        let mut arp = [0u8; 42];
        arp[0..6].copy_from_slice(&[0xff; 6]); // L2 broadcast
        arp[6..12].copy_from_slice(&self.wifi.mac);
        arp[12..14].copy_from_slice(&[0x08, 0x06]); // ARP
        arp[14..16].copy_from_slice(&[0, 1]); // htype ethernet
        arp[16..18].copy_from_slice(&[8, 0]); // ptype IPv4
        arp[18] = 6; // hlen
        arp[19] = 4; // plen
        arp[20..22].copy_from_slice(&[0, 1]); // oper = request (announcement)
        arp[22..28].copy_from_slice(&self.wifi.mac); // sender MAC = us
        arp[28..32].copy_from_slice(&[192, 168, 4, 1]); // sender IP = gateway
        arp[38..42].copy_from_slice(&[192, 168, 4, 1]); // target IP = gateway
        self.wifi.send_frame(&arp, self.fr);
    }
}

/// DHCP lease table: client MAC per pool slot; IP = 192.168.4.(2 + slot).
struct Leases {
    macs: [[u8; 6]; 8],
    count: u8,
}

impl Leases {
    fn ip_for(&mut self, mac: &[u8; 6]) -> u8 {
        for i in 0..self.count as usize {
            if &self.macs[i] == mac {
                return 2 + i as u8;
            }
        }
        let slot = (self.count as usize).min(7);
        self.macs[slot] = *mac;
        if (self.count as usize) < 8 {
            self.count += 1;
        }
        2 + slot as u8
    }
}

/// Build a DHCP reply *payload* (BOOTP + magic + options) into `out`.
/// smoltcp wraps it in UDP/IP/Ethernet, so no headers/checksums here.
/// Returns the payload length, or None if the request is not a BOOTREQUEST.
fn build_dhcp_reply(req: &[u8], out: &mut [u8], leases: &mut Leases) -> Option<usize> {
    // Fixed BOOTP section is 236 bytes, then 4-byte magic, then options.
    if req.len() < 240 || req[0] != 1 {
        return None; // not a BOOTREQUEST
    }
    // DHCP message type (option 53): 1=DISCOVER -> OFFER(2), 3=REQUEST -> ACK(5).
    let mut msgtype = 0u8;
    let mut o = 240;
    while o + 1 < req.len() {
        let code = req[o];
        if code == 255 {
            break;
        }
        if code == 0 {
            o += 1;
            continue;
        }
        let len = req[o + 1] as usize;
        if code == 53 && len >= 1 && o + 2 < req.len() {
            msgtype = req[o + 2];
        }
        o += 2 + len;
    }
    let reply_type = match msgtype {
        1 => 2, // DISCOVER -> OFFER
        3 => 5, // REQUEST  -> ACK
        _ => return None,
    };
    let mut cmac = [0u8; 6];
    cmac.copy_from_slice(&req[28..34]);
    let yip = leases.ip_for(&cmac);

    out[..240].fill(0);
    out[0] = 2; // op = BOOTREPLY
    out[1] = 1; // htype ethernet
    out[2] = 6; // hlen
    out[4..8].copy_from_slice(&req[4..8]); // xid echo
    out[16..20].copy_from_slice(&[192, 168, 4, yip]); // yiaddr
    out[20..24].copy_from_slice(&[192, 168, 4, 1]); // siaddr
    out[28..34].copy_from_slice(&cmac); // chaddr
    out[236..240].copy_from_slice(&[99, 130, 83, 99]); // magic cookie

    let opts: [&[u8]; 7] = [
        &[53, 1, reply_type],
        &[54, 4, 192, 168, 4, 1],   // server id
        &[51, 4, 0, 1, 0x51, 0x80], // lease 86400 s
        &[1, 4, 255, 255, 255, 0],  // subnet mask
        &[3, 4, 192, 168, 4, 1],    // router
        &[6, 4, 192, 168, 4, 1],    // DNS
        b"\x72\x16http://192.168.4.1/api", // option 114 (RFC 8910)
    ];
    let mut p = 240;
    for opt in opts {
        out[p..p + opt.len()].copy_from_slice(opt);
        p += opt.len();
    }
    out[p] = 255;
    p += 1;
    DIAG[11].store(0xD000_0000 | msgtype as u32, SeqCst);
    if reply_type == 2 {
        DIAG[13].store(DIAG[13].load(SeqCst).wrapping_add(1), SeqCst); // OFFERs built
    } else {
        DIAG[14].store(DIAG[14].load(SeqCst).wrapping_add(1), SeqCst); // ACKs built
    }
    Some(p)
}

/// True if the frame is a DHCP BOOTREQUEST (IPv4 / UDP / dst port 67).
fn is_dhcp_request(f: &[u8]) -> bool {
    if f.len() < 42 || f[12] != 0x08 || f[13] != 0x00 || f[14 + 9] != 17 {
        return false;
    }
    let udp = 14 + (f[14] & 0x0f) as usize * 4;
    if udp + 4 > f.len() {
        return false;
    }
    (((f[udp + 2] as u16) << 8) | f[udp + 3] as u16) == 67
}

/// 1's-complement IP header checksum.
fn ip_checksum(hdr: &[u8]) -> u16 {
    let mut sum = 0u32;
    let mut i = 0;
    while i + 1 < hdr.len() {
        sum += ((hdr[i] as u32) << 8) | hdr[i + 1] as u32;
        i += 2;
    }
    while (sum >> 16) != 0 {
        sum = (sum & 0xffff) + (sum >> 16);
    }
    !(sum as u16)
}

/// Serve a DHCP BOOTREQUEST: build the full OFFER/ACK frame (Ethernet + IP + UDP
/// + DHCP payload) and L2/L3-broadcast it (the client has no IP yet). This is the
/// frame-level DHCP server smoltcp cannot provide.
fn handle_dhcp(wifi: &mut Cyw43, fr: &mut [u32; 512], rx: &[u8], leases: &mut Leases) {
    let ihl = (rx[14] & 0x0f) as usize * 4;
    let dh = 14 + ihl + 8; // DHCP payload start
    if rx.len() < dh + 240 {
        return;
    }
    let mut payload = [0u8; 320];
    let plen = match build_dhcp_reply(&rx[dh..], &mut payload, leases) {
        Some(l) => l,
        None => return,
    };
    let mut tx = [0u8; 400];
    tx[0..6].copy_from_slice(&[0xff; 6]); // L2 broadcast
    tx[6..12].copy_from_slice(&wifi.mac);
    tx[12..14].copy_from_slice(&[0x08, 0x00]);
    tx[14] = 0x45;
    tx[22] = 64;
    tx[23] = 17; // TTL 64, UDP
    tx[26..30].copy_from_slice(&[192, 168, 4, 1]); // src IP
    tx[30..34].copy_from_slice(&[255, 255, 255, 255]); // dst broadcast
    tx[34..36].copy_from_slice(&[0, 67]); // UDP src 67
    tx[36..38].copy_from_slice(&[0, 68]); // UDP dst 68
    tx[42..42 + plen].copy_from_slice(&payload[..plen]);
    let flen = 42 + plen;
    tx[16..18].copy_from_slice(&((flen - 14) as u16).to_be_bytes()); // IP total len
    tx[38..40].copy_from_slice(&((flen - 34) as u16).to_be_bytes()); // UDP len
    let ck = ip_checksum(&tx[14..34]);
    tx[24..26].copy_from_slice(&ck.to_be_bytes());
    wifi.send_frame(&tx[..flen], fr);
    DIAG[12].store(DIAG[12].load(SeqCst).wrapping_add(1), SeqCst); // DHCP replies sent
}

/// Provisioning loop: smoltcp Interface at 192.168.4.1/24. smoltcp owns
/// ARP/IP/UDP/TCP (and auto-answers ARP for the gateway); DHCP is served in the
/// Device at the frame level. Later phases add DNS/TCP sockets to the SocketSet.
pub fn run_portal(wifi: &mut Cyw43, fr: &mut [u32; 512]) -> ! {
    let mac = EthernetAddress::from_bytes(&wifi.mac);
    let mut device = Cyw43Device {
        wifi,
        fr,
        leases: Leases {
            macs: [[0; 6]; 8],
            count: 0,
        },
    };

    let mut config = Config::new();
    config.hardware_addr = Some(mac.into());
    let mut iface = Interface::new(config, &mut device);
    iface.update_ip_addrs(|addrs| {
        addrs
            .push(IpCidr::Ipv4(Ipv4Cidr::new(
                Ipv4Address::new(192, 168, 4, 1),
                24,
            )))
            .ok();
    });

    // No sockets yet (ARP is handled by the Interface itself; DHCP is frame-level
    // in the Device). DNS (UDP) and HTTP (TCP) sockets land here in later phases.
    let mut socket_storage: [SocketStorage<'_>; 4] = [SocketStorage::EMPTY; 4];
    let mut sockets = SocketSet::new(&mut socket_storage[..]);

    DIAG[5].store(0, SeqCst); // IPv4 frames addressed to us (.1)
    DIAG[6].store(0, SeqCst); // ARP frames in
    DIAG[7].store(0, SeqCst); // smoltcp TX out (ARP replies etc.)
    DIAG[9].store(0, SeqCst); // raw frames into smoltcp
    DIAG[12].store(0, SeqCst); // DHCP replies sent
    DIAG[13].store(0, SeqCst); // OFFERs built (= DISCOVERs seen)
    DIAG[14].store(0, SeqCst); // ACKs built (= REQUESTs seen)
    let mut last_garp = 0u64;
    loop {
        let now = userlib::sys_get_timer().now;
        iface.poll(Instant::from_millis(now as i64), &mut device, &mut sockets);
        // Re-announce the gateway ~1/s so clients keep .1 -> our MAC cached.
        if now.wrapping_sub(last_garp) >= 1000 {
            last_garp = now;
            device.send_gratuitous_arp();
        }
    }
}
