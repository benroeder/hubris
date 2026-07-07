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

use cyw43_portal::{
    Leases, build_dhcp_reply, build_dns_reply, http_request_len, ip_checksum,
    is_dhcp_request, parse_creds,
};
use smoltcp::iface::{Config, Interface, SocketSet, SocketStorage};
use smoltcp::phy::{self, Device, DeviceCapabilities, Medium};
use smoltcp::socket::{dhcpv4, tcp, udp};
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
    /// True once we join a network as a station: stop serving the AP DHCP so we
    /// don't answer DHCP requests on the real LAN.
    sta_mode: bool,
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
            if !self.sta_mode && is_dhcp_request(&buf[..n]) {
                handle_dhcp(self.wifi, self.fr, &buf[..n], &mut self.leases);
                continue;
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

/// Serve a DHCP BOOTREQUEST: build the full OFFER/ACK frame (Ethernet + IP + UDP
/// + DHCP payload) and L2/L3-broadcast it (the client has no IP yet). This is the
/// frame-level DHCP server smoltcp cannot provide.
fn handle_dhcp(
    wifi: &mut Cyw43,
    fr: &mut [u32; 512],
    rx: &[u8],
    leases: &mut Leases,
) {
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
}

const HTTP_OK_HTML: &[u8] =
    b"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n";

/// The portal form, built per attempt: head + optional error banner + form open
/// + one <option> per scanned SSID + suffix. Includes the HTTP headers so the
/// built buffer is served directly.
const FORM_HEAD: &[u8] = b"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n<!DOCTYPE html><html><head><meta name=viewport content=\"width=device-width,initial-scale=1\"><title>Pico 2 W Setup</title></head><body style=\"font-family:sans-serif;max-width:420px;margin:2em auto;padding:0 1em\"><h1>Pico 2 W Wi-Fi Setup</h1>";
const FORM_ERR: &[u8] = b"<p style=\"color:#b00;font-weight:bold\">Could not connect -- check the password and try again.</p>";
const FORM_OPEN: &[u8] = b"<form method=POST action=/connect><p>Network<br><select name=ssid style=\"width:100%;font-size:1.2em\">";
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
    push(out, &mut n, FORM_HEAD);
    if crate::CREDS[26].load(SeqCst) != 0 {
        push(out, &mut n, FORM_ERR); // a prior join failed
    }
    push(out, &mut n, FORM_OPEN);
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
    let Some(c) = parse_creds(body) else {
        return false;
    };
    store_creds(&c.ssid[..c.ssid_len], &c.pass[..c.pass_len]);
    true
}

/// Serve one pooled HTTP connection: (re-)listen when closed; respond only once
/// the whole request is buffered, then close. A POST stores creds (checked by
/// the caller via CREDS) and shows Connecting; a POST that parses no creds
/// re-serves the form instead of dead-ending. GET /api -> RFC 8908 JSON.
fn serve_http(sock: &mut tcp::Socket<'_>, form: &[u8]) {
    if !sock.is_open() {
        sock.listen(80).ok();
        return;
    }
    if !sock.can_recv() {
        return;
    }
    let mut req = [0u8; 1024];
    let mut reqlen = 0usize;
    let ready = sock
        .recv(|data| match http_request_len(data) {
            Some(total) => {
                let n = total.min(req.len());
                req[..n].copy_from_slice(&data[..n]);
                reqlen = n;
                (total, true)
            }
            None => (0, false), // incomplete -> leave buffered, wait
        })
        .unwrap_or(false);
    if !ready {
        return;
    }
    let r = &req[..reqlen];
    if r.starts_with(b"POST ") {
        if handle_post(r) {
            let _ = sock.send_slice(HTTP_OK_HTML);
            let _ = sock.send_slice(CONNECTING_BODY);
        } else {
            let _ = sock.send_slice(form); // no creds parsed -> re-show form
        }
    } else if r.len() >= 9 && &r[0..9] == b"GET /api " {
        let _ = sock.send_slice(API_JSON); // captive detect -> open portal
    } else {
        let _ = sock.send_slice(form);
    }
    sock.close();
}

/// Provisioning loop: smoltcp Interface at 192.168.4.1/24. smoltcp owns
/// ARP/IP/UDP/TCP (and auto-answers ARP for the gateway); DHCP is served in the
/// Device at the frame level. Later phases add DNS/TCP sockets to the SocketSet.
pub fn run_portal(wifi: &mut Cyw43, fr: &mut [u32; 512]) -> ! {
    let mac = EthernetAddress::from_bytes(&wifi.mac);
    let mut device = Cyw43Device {
        wifi,
        fr,
        leases: Leases::new(),
        sta_mode: false,
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
    let (
        dns_rx_meta,
        dns_rx_pl,
        dns_tx_meta,
        dns_tx_pl,
        hrx0,
        htx0,
        hrx1,
        htx1,
        hrx2,
        htx2,
        form_buf,
        socket_storage,
    ) = mutable_statics::mutable_statics! {
        static mut DNS_RX_META: [udp::PacketMetadata; 8] = [meta; _];
        static mut DNS_RX_PL: [u8; 768] = [zero; _];
        static mut DNS_TX_META: [udp::PacketMetadata; 8] = [meta; _];
        static mut DNS_TX_PL: [u8; 768] = [zero; _];
        static mut HTTP_RX0: [u8; 1024] = [zero; _];
        static mut HTTP_TX0: [u8; 2048] = [zero; _];
        static mut HTTP_RX1: [u8; 1024] = [zero; _];
        static mut HTTP_TX1: [u8; 2048] = [zero; _];
        static mut HTTP_RX2: [u8; 1024] = [zero; _];
        static mut HTTP_TX2: [u8; 2048] = [zero; _];
        static mut FORM_BUF: [u8; 2048] = [zero; _];
        static mut SOCKET_STORAGE: [SocketStorage<'static>; 6] = [store; _];
    };
    let mut form_len = build_form(form_buf);
    let dns_rx =
        udp::PacketBuffer::new(&mut dns_rx_meta[..], &mut dns_rx_pl[..]);
    let dns_tx =
        udp::PacketBuffer::new(&mut dns_tx_meta[..], &mut dns_tx_pl[..]);
    let mut dns_sock = udp::Socket::new(dns_rx, dns_tx);
    dns_sock.bind(53).ok();
    let mut sockets = SocketSet::new(&mut socket_storage[..]);
    let dns_handle = sockets.add(dns_sock);
    // HTTP captive portal on :80 -- a small pool so the CNA's concurrent
    // connections (portal + /api probe + retries) aren't RST-ed by a single
    // listener (the "web page couldn't be loaded" symptom).
    let http_handles = [
        sockets.add(tcp::Socket::new(
            tcp::SocketBuffer::new(&mut hrx0[..]),
            tcp::SocketBuffer::new(&mut htx0[..]),
        )),
        sockets.add(tcp::Socket::new(
            tcp::SocketBuffer::new(&mut hrx1[..]),
            tcp::SocketBuffer::new(&mut htx1[..]),
        )),
        sockets.add(tcp::Socket::new(
            tcp::SocketBuffer::new(&mut hrx2[..]),
            tcp::SocketBuffer::new(&mut htx2[..]),
        )),
    ];

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
                    }
                }
                Err(_) => break,
            }
        }

        // HTTP captive portal on :80: serve every pooled socket. serve_http waits
        // for the full request (headers + Content-Length body) before responding,
        // so a segmented POST does not drop credentials, and iOS's option-114
        // /api probe + the portal page can be served on separate connections.
        for &h in &http_handles {
            let sock = sockets.get_mut::<tcp::Socket<'_>>(h);
            serve_http(sock, &form_buf[..form_len]);
        }

        // Credentials submitted -> flush the "Connecting..." page to the client,
        // then attempt the join (this brings the AP down).
        if crate::CREDS[0].load(SeqCst) == 1 {
            for _ in 0..400u32 {
                let t = userlib::sys_get_timer().now;
                iface.poll(
                    Instant::from_millis(t as i64),
                    &mut device,
                    &mut sockets,
                );
                cortex_m::asm::delay(30_000);
            }
            let mut res = device.wifi.sta_join(device.fr);
            DIAG[15].store(
                if res == 0 { 0x00C0_FFEE } else { 0x0BAD_0BAD },
                SeqCst,
            );
            if res == 0 {
                // Associated -- become a station: TX on the STA interface, stop
                // acting as a DHCP server (we would answer real-LAN requests),
                // drop the AP static IP, and run a DHCP client. Bail after ~30s if
                // no initial lease arrives, so a dead/filtering DHCP server can't
                // wedge us with the AP down; once leased, stay connected.
                device.wifi.tx_iface = 0;
                device.sta_mode = true;
                iface.update_ip_addrs(|addrs| {
                    addrs.clear();
                });
                let dhcp_handle = sockets.add(dhcpv4::Socket::new());
                let start = userlib::sys_get_timer().now;
                let mut leased = false;
                loop {
                    let t = userlib::sys_get_timer().now;
                    iface.poll(
                        Instant::from_millis(t as i64),
                        &mut device,
                        &mut sockets,
                    );
                    // Bind the event first so the dhcpv4 socket borrow ends before
                    // we re-borrow `sockets` to abort the portal listeners.
                    let ev = sockets
                        .get_mut::<dhcpv4::Socket<'_>>(dhcp_handle)
                        .poll();
                    let mut just_leased = false;
                    match ev {
                        Some(dhcpv4::Event::Configured(cfg)) => {
                            iface.update_ip_addrs(|addrs| {
                                addrs.push(IpCidr::Ipv4(cfg.address)).ok();
                            });
                            if let Some(gw) = cfg.router {
                                iface
                                    .routes_mut()
                                    .add_default_ipv4_route(gw)
                                    .ok();
                            }
                            DIAG[3].store(
                                u32::from_be_bytes(cfg.address.address().0),
                                SeqCst,
                            ); // leased IP -- read via probe, then ping it
                            DIAG[15].store(0x001E_A5ED, SeqCst); // got a lease
                            just_leased = true;
                        }
                        Some(dhcpv4::Event::Deconfigured) => {
                            iface.update_ip_addrs(|addrs| {
                                addrs.clear();
                            });
                        }
                        None => {}
                    }
                    // The dhcpv4 event borrow of `sockets` has ended: on the first
                    // lease, abort the portal listeners so we don't linger as a
                    // rogue :80 on the real LAN.
                    if just_leased && !leased {
                        for &h in &http_handles {
                            sockets.get_mut::<tcp::Socket<'_>>(h).abort();
                        }
                    }
                    leased |= just_leased;
                    if !leased && t.wrapping_sub(start) > 30_000 {
                        break; // no initial lease -> give up, re-provision
                    }
                }
                // Only reached on the no-lease timeout: restore AP mode + retry.
                sockets.remove(dhcp_handle);
                device.wifi.tx_iface = 1;
                device.sta_mode = false;
                iface.update_ip_addrs(|addrs| {
                    addrs.clear();
                    addrs
                        .push(IpCidr::Ipv4(Ipv4Cidr::new(
                            Ipv4Address::new(192, 168, 4, 1),
                            24,
                        )))
                        .ok();
                });
                res = 3; // DHCP-timeout failure
            }
            // Failed (bad password / no network / no lease): show the error and
            // re-provision. Abort every pool socket so they re-listen cleanly.
            crate::CREDS[26].store(res, SeqCst);
            crate::CREDS[0].store(0, SeqCst);
            device.wifi.ap_start(device.fr);
            form_len = build_form(form_buf);
            for &h in &http_handles {
                sockets.get_mut::<tcp::Socket<'_>>(h).abort();
            }
        }
    }
}
