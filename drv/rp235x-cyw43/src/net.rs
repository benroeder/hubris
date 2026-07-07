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
use smoltcp::socket::{tcp, udp};
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
            // L2 unicast addressed to our MAC (definitive unicast-to-host test).
            if buf[0..6] == self.wifi.mac {
                DIAG[2].store(DIAG[2].load(SeqCst).wrapping_add(1), SeqCst);
                // Capture the first 64 bytes for offline decode (eth + IP hdr).
                for i in 0..16 {
                    let p = i * 4;
                    let w = (buf[p] as u32)
                        | ((buf[p + 1] as u32) << 8)
                        | ((buf[p + 2] as u32) << 16)
                        | ((buf[p + 3] as u32) << 24);
                    crate::DATA_FRAME[i].store(w, SeqCst);
                }
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

/// Captive-portal DNS: answer every A query with 192.168.4.1 (and empty for
/// non-A) so all lookups resolve to us. Builds the DNS payload only; smoltcp
/// wraps UDP/IP/Ethernet. Returns the reply length.
fn build_dns_reply(q: &[u8], out: &mut [u8]) -> Option<usize> {
    if q.len() < 12 {
        return None;
    }
    // Walk the question name (labels terminated by a 0 byte).
    let qstart = 12;
    let mut p = qstart;
    while p < q.len() && q[p] != 0 {
        p += 1 + q[p] as usize;
    }
    if p + 5 > q.len() {
        return None;
    }
    let qtype = ((q[p + 1] as u16) << 8) | q[p + 2] as u16;
    let qlen = (p + 5) - qstart; // name + null + qtype(2) + qclass(2)
    let is_a = qtype == 1;

    out[0..2].copy_from_slice(&q[0..2]); // id echo
    out[2..4].copy_from_slice(&[0x81, 0x80]); // response, RA, no error
    out[4..6].copy_from_slice(&[0, 1]); // qdcount 1
    out[6..8].copy_from_slice(&[0, if is_a { 1 } else { 0 }]); // ancount
    out[8..12].copy_from_slice(&[0, 0, 0, 0]); // ns/ar count 0
    out[12..12 + qlen].copy_from_slice(&q[qstart..qstart + qlen]); // echo question
    let mut len = 12 + qlen;
    if is_a {
        let a = len;
        out[a..a + 2].copy_from_slice(&[0xc0, 0x0c]); // name ptr -> offset 12
        out[a + 2..a + 4].copy_from_slice(&[0, 1]); // type A
        out[a + 4..a + 6].copy_from_slice(&[0, 1]); // class IN
        out[a + 6..a + 10].copy_from_slice(&[0, 0, 0, 60]); // TTL 60
        out[a + 10..a + 12].copy_from_slice(&[0, 4]); // rdlength
        out[a + 12..a + 16].copy_from_slice(&[192, 168, 4, 1]); // A = us
        len = a + 16;
    }
    Some(len)
}

const HTTP_OK_HTML: &[u8] =
    b"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n";

/// The portal form, built at startup: prefix + one <option> per scanned SSID +
/// suffix. Includes the HTTP headers so the built buffer is served directly.
const FORM_PREFIX: &[u8] = b"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n<!DOCTYPE html><html><head><meta name=viewport content=\"width=device-width,initial-scale=1\"><title>Pico 2 W Setup</title></head><body style=\"font-family:sans-serif;max-width:420px;margin:2em auto;padding:0 1em\"><h1>Pico 2 W Wi-Fi Setup</h1><form method=POST action=/connect><p>Network<br><select name=ssid style=\"width:100%;font-size:1.2em\">";
const FORM_SUFFIX: &[u8] = b"</select></p><p>Password<br><input name=password type=password style=\"width:100%;font-size:1.2em\"></p><p><button style=\"font-size:1.2em;padding:.4em 1em\">Connect</button></p></form></body></html>";

/// Append `src` to `out` at `*n`, clamped to the buffer.
fn push(out: &mut [u8], n: &mut usize, src: &[u8]) {
    let e = (*n + src.len()).min(out.len());
    out[*n..e].copy_from_slice(&src[..e - *n]);
    *n = e;
}

/// Build the portal form (with a network dropdown from SCAN_SSIDS) into `out`.
fn build_form(out: &mut [u8]) -> usize {
    let mut raw = [0u8; 400];
    for (i, w) in crate::SCAN_SSIDS.iter().enumerate() {
        raw[i * 4..i * 4 + 4].copy_from_slice(&w.load(SeqCst).to_le_bytes());
    }
    let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
    let mut n = 0;
    push(out, &mut n, FORM_PREFIX);
    for ssid in raw[..end].split(|&b| b == b'\n') {
        if ssid.is_empty() {
            continue;
        }
        push(out, &mut n, b"<option>");
        push(out, &mut n, ssid);
        push(out, &mut n, b"</option>");
    }
    push(out, &mut n, FORM_SUFFIX);
    n
}

const CONNECTING_BODY: &[u8] = b"<!DOCTYPE html><html><head><meta name=viewport content=\"width=device-width,initial-scale=1\"><title>Connecting</title></head><body style=\"font-family:sans-serif;max-width:420px;margin:2em auto\"><h1>Connecting...</h1><p>The Pico is joining your network. You can close this window.</p></body></html>";

const API_JSON: &[u8] = b"HTTP/1.0 200 OK\r\nContent-Type: application/captive+json\r\nConnection: close\r\n\r\n{\"captive\":true,\"user-portal-url\":\"http://192.168.4.1/\"}";

fn hexval(c: u8) -> u8 {
    match c {
        b'0'..=b'9' => c - b'0',
        b'a'..=b'f' => c - b'a' + 10,
        b'A'..=b'F' => c - b'A' + 10,
        _ => 0,
    }
}

/// URL-decode `src` into `dst` ('+' -> space, %XX -> byte); returns the length.
fn urldecode(src: &[u8], dst: &mut [u8]) -> usize {
    let mut n = 0;
    let mut i = 0;
    while i < src.len() && n < dst.len() {
        match src[i] {
            b'+' => {
                dst[n] = b' ';
                i += 1;
            }
            b'%' if i + 2 < src.len() => {
                dst[n] = (hexval(src[i + 1]) << 4) | hexval(src[i + 2]);
                i += 3;
            }
            c => {
                dst[n] = c;
                i += 1;
            }
        }
        n += 1;
    }
    n
}

/// Store the submitted SSID + password into the CREDS static for the STA join.
fn store_creds(ssid: &[u8], pass: &[u8]) {
    let sl = ssid.len().min(32);
    let pl = pass.len().min(64);
    let pack = |bytes: &[u8], base: usize, words: usize| {
        for i in 0..words {
            let mut w = 0u32;
            for j in 0..4 {
                let k = i * 4 + j;
                if k < bytes.len() {
                    w |= (bytes[k] as u32) << (8 * j);
                }
            }
            crate::CREDS[base + i].store(w, SeqCst);
        }
    };
    pack(&ssid[..sl], 3, 8);
    pack(&pass[..pl], 11, 16);
    crate::CREDS[1].store(sl as u32, SeqCst);
    crate::CREDS[2].store(pl as u32, SeqCst);
    crate::CREDS[0].store(1, SeqCst); // ready
}

/// Parse a urlencoded POST body for ssid + password and store them.
fn handle_post(req: &[u8]) -> bool {
    let mut body: &[u8] = &[];
    let mut i = 0;
    while i + 4 <= req.len() {
        if &req[i..i + 4] == b"\r\n\r\n" {
            body = &req[i + 4..];
            break;
        }
        i += 1;
    }
    let mut ssid = [0u8; 32];
    let mut sl = 0;
    let mut pass = [0u8; 64];
    let mut pl = 0;
    let mut got = false;
    for field in body.split(|&b| b == b'&') {
        if let Some(eq) = field.iter().position(|&b| b == b'=') {
            match &field[..eq] {
                b"ssid" => {
                    sl = urldecode(&field[eq + 1..], &mut ssid);
                    got = true;
                }
                b"password" => pl = urldecode(&field[eq + 1..], &mut pass),
                _ => {}
            }
        }
    }
    if got {
        store_creds(&ssid[..sl], &pass[..pl]);
    }
    got
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

    // DNS responder on UDP :53 (hijack all lookups -> 192.168.4.1). ARP is
    // handled by the Interface; DHCP is frame-level in the Device. Socket buffers
    // live in statics (Oxide task/net pattern), off the stack.
    fn zero() -> u8 {
        0
    }
    fn meta() -> udp::PacketMetadata {
        udp::PacketMetadata::EMPTY
    }
    fn store() -> SocketStorage<'static> {
        SocketStorage::EMPTY
    }
    let (dns_rx_meta, dns_rx_pl, dns_tx_meta, dns_tx_pl, http_rx, http_tx, form_buf, socket_storage) = mutable_statics::mutable_statics! {
        static mut DNS_RX_META: [udp::PacketMetadata; 8] = [meta; _];
        static mut DNS_RX_PL: [u8; 768] = [zero; _];
        static mut DNS_TX_META: [udp::PacketMetadata; 8] = [meta; _];
        static mut DNS_TX_PL: [u8; 768] = [zero; _];
        static mut HTTP_RX: [u8; 1024] = [zero; _];
        static mut HTTP_TX: [u8; 2048] = [zero; _];
        static mut FORM_BUF: [u8; 2048] = [zero; _];
        static mut SOCKET_STORAGE: [SocketStorage<'static>; 4] = [store; _];
    };
    let form_len = build_form(form_buf);
    let dns_rx = udp::PacketBuffer::new(&mut dns_rx_meta[..], &mut dns_rx_pl[..]);
    let dns_tx = udp::PacketBuffer::new(&mut dns_tx_meta[..], &mut dns_tx_pl[..]);
    let mut dns_sock = udp::Socket::new(dns_rx, dns_tx);
    dns_sock.bind(53).ok();
    // HTTP captive portal on TCP :80.
    let http_sock = tcp::Socket::new(
        tcp::SocketBuffer::new(&mut http_rx[..]),
        tcp::SocketBuffer::new(&mut http_tx[..]),
    );
    let mut sockets = SocketSet::new(&mut socket_storage[..]);
    let dns_handle = sockets.add(dns_sock);
    let http_handle = sockets.add(http_sock);

    for i in 96..122 {
        crate::DATA_FRAME[i].store(0, SeqCst); // channel + chan-2-drop + RX-iface histograms
    }
    DIAG[2].store(0, SeqCst); // L2 unicast-to-us frames (was polluted by ALP init)
    DIAG[5].store(0, SeqCst); // IPv4 frames addressed to us (.1)
    DIAG[6].store(0, SeqCst); // ARP frames in
    DIAG[7].store(0, SeqCst); // smoltcp TX out (ARP replies etc.)
    DIAG[9].store(0, SeqCst); // raw frames into smoltcp
    DIAG[12].store(0, SeqCst); // DHCP replies sent
    DIAG[13].store(0, SeqCst); // OFFERs built (= DISCOVERs seen)
    DIAG[14].store(0, SeqCst); // ACKs built (= REQUESTs seen)
    DIAG[4].store(0, SeqCst); // HTTP requests served
    DIAG[10].store(0, SeqCst); // DNS replies sent
    loop {
        let now = userlib::sys_get_timer().now;
        iface.poll(Instant::from_millis(now as i64), &mut device, &mut sockets);
        // DNS hijack: answer every query -> 192.168.4.1.
        let dns = sockets.get_mut::<udp::Socket<'_>>(dns_handle);
        while dns.can_recv() {
            let mut qbuf = [0u8; 768];
            match dns.recv_slice(&mut qbuf) {
                Ok((n, ep)) => {
                    let mut rbuf = [0u8; 768];
                    if let Some(len) = build_dns_reply(&qbuf[..n], &mut rbuf) {
                        let _ = dns.send_slice(&rbuf[..len], ep);
                        DIAG[10].store(DIAG[10].load(SeqCst).wrapping_add(1), SeqCst);
                    }
                }
                Err(_) => break,
            }
        }

        // HTTP captive portal on :80. iOS (given option 114 -> http://192.168.4.1
        // /api) fetches /api expecting RFC 8908 JSON; return captive=true with a
        // user-portal-url so the CNA opens the portal page (served elsewhere).
        // One connection at a time: serve, close, re-listen.
        let http = sockets.get_mut::<tcp::Socket<'_>>(http_handle);
        if !http.is_open() {
            http.listen(80).ok();
        }
        if http.can_recv() {
            let mut req = [0u8; 1024];
            let n = http.recv_slice(&mut req).unwrap_or(0);
            if n > 0 {
                let r = &req[..n];
                if r.starts_with(b"POST ") {
                    // Credentials submitted -> store them, show "Connecting...".
                    handle_post(r);
                    let _ = http.send_slice(HTTP_OK_HTML);
                    let _ = http.send_slice(CONNECTING_BODY);
                } else if n >= 9 && &r[0..9] == b"GET /api " {
                    let _ = http.send_slice(API_JSON); // RFC 8908 -> open portal
                } else {
                    let _ = http.send_slice(&form_buf[..form_len]); // portal form
                }
                http.close();
                DIAG[4].store(DIAG[4].load(SeqCst).wrapping_add(1), SeqCst); // HTTP served
            }
        }
    }
}
