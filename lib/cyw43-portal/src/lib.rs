// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Pure, testable network parsers/builders for the CYW43 captive portal.
//!
//! These functions operate only on their arguments and local state -- no
//! hardware, no statics -- so they can be host-tested here even though the
//! driver crate is a `no_std` binary. The driver re-exports and calls them.

#![cfg_attr(not(test), no_std)]

/// DHCP lease table: client MAC per pool slot; IP = 192.168.4.(2 + slot).
pub struct Leases {
    pub macs: [[u8; 6]; 8],
    pub count: u8,
}

impl Leases {
    /// A fresh, empty lease table.
    pub fn new() -> Leases {
        Leases {
            macs: [[0; 6]; 8],
            count: 0,
        }
    }

    pub fn ip_for(&mut self, mac: &[u8; 6]) -> u8 {
        // Known MAC -> its existing slot (idempotent across DISCOVER/REQUEST).
        for (i, m) in self.macs.iter().enumerate() {
            if m == mac {
                return 2 + i as u8;
            }
        }
        // New MAC -> next slot round-robin, evicting the oldest when full. Keeps
        // all 8 pool IPs distinct; only >8 simultaneous clients recycle a slot
        // (fine for a provisioning AP), instead of collapsing onto .9.
        let slot = (self.count as usize) % self.macs.len();
        self.macs[slot] = *mac;
        self.count = self.count.wrapping_add(1);
        2 + slot as u8
    }
}

impl Default for Leases {
    fn default() -> Self {
        Leases::new()
    }
}

/// Build a DHCP reply *payload* (BOOTP + magic + options) into `out`.
/// smoltcp wraps it in UDP/IP/Ethernet, so no headers/checksums here.
/// Returns the payload length, or None if the request is not a BOOTREQUEST.
pub fn build_dhcp_reply(
    req: &[u8],
    out: &mut [u8],
    leases: &mut Leases,
) -> Option<usize> {
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
        &[54, 4, 192, 168, 4, 1],          // server id
        &[51, 4, 0, 1, 0x51, 0x80],        // lease 86400 s
        &[1, 4, 255, 255, 255, 0],         // subnet mask
        &[3, 4, 192, 168, 4, 1],           // router
        &[6, 4, 192, 168, 4, 1],           // DNS
        b"\x72\x16http://192.168.4.1/api", // option 114 (RFC 8910)
    ];
    let mut p = 240;
    for opt in opts {
        out[p..p + opt.len()].copy_from_slice(opt);
        p += opt.len();
    }
    out[p] = 255;
    p += 1;
    Some(p)
}

/// True if the frame is a DHCP BOOTREQUEST (IPv4 / UDP / dst port 67).
pub fn is_dhcp_request(f: &[u8]) -> bool {
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
pub fn ip_checksum(hdr: &[u8]) -> u16 {
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

/// Captive-portal DNS: answer every A query with 192.168.4.1 (and empty for
/// non-A) so all lookups resolve to us. Builds the DNS payload only; smoltcp
/// wraps UDP/IP/Ethernet. Returns the reply length.
pub fn build_dns_reply(q: &[u8], out: &mut [u8]) -> Option<usize> {
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

    // Bound the whole reply against the output buffer BEFORE writing. A crafted
    // query whose name fills the packet would otherwise push the 16-byte A-record
    // past `out` and fault the task -- remotely triggerable on :53.
    let reply_len = 12 + qlen + if is_a { 16 } else { 0 };
    if reply_len > out.len() {
        return None;
    }

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

pub fn hexval(c: u8) -> u8 {
    match c {
        b'0'..=b'9' => c - b'0',
        b'a'..=b'f' => c - b'a' + 10,
        b'A'..=b'F' => c - b'A' + 10,
        _ => 0,
    }
}

/// URL-decode `src` into `dst` ('+' -> space, %XX -> byte); returns the length.
pub fn urldecode(src: &[u8], dst: &mut [u8]) -> usize {
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

/// Credentials parsed out of a urlencoded POST body.
pub struct ParsedCreds {
    pub ssid: [u8; 32],
    pub ssid_len: usize,
    pub pass: [u8; 64],
    pub pass_len: usize,
}

/// Parse a urlencoded POST body for `ssid` + `password`, urldecoding each value.
/// Returns None when the body carries no `ssid` field. Storing the result into
/// hardware statics stays with the caller.
pub fn parse_creds(body: &[u8]) -> Option<ParsedCreds> {
    let mut c = ParsedCreds {
        ssid: [0u8; 32],
        ssid_len: 0,
        pass: [0u8; 64],
        pass_len: 0,
    };
    let mut got = false;
    for field in body.split(|&b| b == b'&') {
        if let Some(eq) = field.iter().position(|&b| b == b'=') {
            match &field[..eq] {
                b"ssid" => {
                    c.ssid_len = urldecode(&field[eq + 1..], &mut c.ssid);
                    got = true;
                }
                b"password" => {
                    c.pass_len = urldecode(&field[eq + 1..], &mut c.pass)
                }
                _ => {}
            }
        }
    }
    got.then_some(c)
}

/// Case-insensitively find `Content-Length:` in the header block and parse it.
pub fn content_length(hdrs: &[u8]) -> usize {
    let needle = b"content-length:";
    let last = hdrs.len().saturating_sub(needle.len());
    'scan: for i in 0..=last {
        for (j, &nc) in needle.iter().enumerate() {
            let c = hdrs[i + j];
            let lc = if c.is_ascii_uppercase() { c + 32 } else { c };
            if lc != nc {
                continue 'scan;
            }
        }
        let mut k = i + needle.len();
        while k < hdrs.len() && hdrs[k] == b' ' {
            k += 1;
        }
        let mut n = 0usize;
        while k < hdrs.len() && hdrs[k].is_ascii_digit() {
            n = n * 10 + (hdrs[k] - b'0') as usize;
            k += 1;
        }
        return n;
    }
    0
}

/// If `data` holds a COMPLETE HTTP request, return its total byte length; else
/// None so the caller waits for more (avoids parsing a truncated header/body --
/// e.g. a POST whose credential body lands in a later TCP segment).
pub fn http_request_len(data: &[u8]) -> Option<usize> {
    let mut i = 0;
    let hdr_end = loop {
        if i + 4 > data.len() {
            return None; // headers not yet terminated
        }
        if &data[i..i + 4] == b"\r\n\r\n" {
            break i + 4;
        }
        i += 1;
    };
    if data.starts_with(b"POST ") {
        let total = hdr_end + content_length(&data[..hdr_end]);
        (data.len() >= total).then_some(total)
    } else {
        Some(hdr_end)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Build a minimal DNS query: 12-byte header + one label name + qtype/qclass.
    fn dns_query(id: [u8; 2], name: &[&[u8]], qtype: u16) -> Vec<u8> {
        let mut q = Vec::new();
        q.extend_from_slice(&id);
        q.extend_from_slice(&[0x01, 0x00]); // flags: standard query
        q.extend_from_slice(&[0, 1]); // qdcount 1
        q.extend_from_slice(&[0, 0, 0, 0, 0, 0]); // an/ns/ar 0
        for label in name {
            q.push(label.len() as u8);
            q.extend_from_slice(label);
        }
        q.push(0); // root
        q.extend_from_slice(&qtype.to_be_bytes()); // qtype
        q.extend_from_slice(&[0, 1]); // qclass IN
        q
    }

    #[test]
    fn dns_reply_a_record() {
        let q = dns_query([0xab, 0xcd], &[b"example", b"com"], 1);
        let mut out = [0u8; 768];
        let len = build_dns_reply(&q, &mut out).expect("A query -> reply");
        // id echoed
        assert_eq!(&out[0..2], &[0xab, 0xcd]);
        // response flag set
        assert_eq!(&out[2..4], &[0x81, 0x80]);
        // qdcount 1, ancount 1
        assert_eq!(&out[4..6], &[0, 1]);
        assert_eq!(&out[6..8], &[0, 1]);
        // A record ends the reply with 192.168.4.1
        assert_eq!(&out[len - 4..len], &[192, 168, 4, 1]);
        // name pointer to offset 12
        assert_eq!(out[len - 16], 0xc0);
        assert_eq!(out[len - 15], 0x0c);
    }

    #[test]
    fn dns_reply_output_bounds() {
        // A type-A query whose name nearly fills a ~760-byte packet: the reply
        // would need name+16 bytes of A record past a tight output buffer.
        let mut q = Vec::new();
        q.extend_from_slice(&[0x00, 0x01, 0x01, 0x00, 0, 1, 0, 0, 0, 0, 0, 0]);
        // Fill ~748 bytes of labels (each 63-byte label + length byte).
        while q.len() < 12 + 748 {
            q.push(63);
            q.extend_from_slice(&[b'a'; 63]);
        }
        q.push(0);
        q.extend_from_slice(&[0, 1]); // qtype A
        q.extend_from_slice(&[0, 1]); // qclass
        // Output buffer the same size as the driver's 768-byte reply buffer.
        let mut out = [0u8; 768];
        assert_eq!(build_dns_reply(&q, &mut out), None);
    }

    #[test]
    fn dns_reply_non_a_query() {
        let q = dns_query([0x12, 0x34], &[b"example", b"com"], 28); // AAAA
        let mut out = [0u8; 768];
        let len = build_dns_reply(&q, &mut out).expect("AAAA -> empty reply");
        // ancount 0
        assert_eq!(&out[6..8], &[0, 0]);
        // no answer appended: length is exactly header + question
        assert_eq!(len, 12 + (q.len() - 12));
    }

    #[test]
    fn urldecode_cases() {
        let mut out = [0u8; 64];

        let n = urldecode(b"hunter2%21", &mut out);
        assert_eq!(&out[..n], b"hunter2!");

        let n = urldecode(b"a+b", &mut out);
        assert_eq!(&out[..n], b"a b");

        // Trailing '%' with fewer than 2 hex chars -> kept literal.
        let n = urldecode(b"ab%", &mut out);
        assert_eq!(&out[..n], b"ab%");

        let n = urldecode(b"abc", &mut out);
        assert_eq!(&out[..n], b"abc");
    }

    #[test]
    fn http_request_len_get_complete() {
        let req = b"GET / HTTP/1.1\r\nHost: x\r\n\r\n";
        assert_eq!(http_request_len(req), Some(req.len()));
    }

    #[test]
    fn http_request_len_headers_unterminated() {
        let req = b"GET / HTTP/1.1\r\nHost: x\r\n";
        assert_eq!(http_request_len(req), None);
    }

    #[test]
    fn http_request_len_post_body_short() {
        let req = b"POST /c HTTP/1.1\r\nContent-Length: 5\r\n\r\nab";
        assert_eq!(http_request_len(req), None);
    }

    #[test]
    fn http_request_len_post_body_complete() {
        let req = b"POST /c HTTP/1.1\r\nContent-Length: 5\r\n\r\nabcde";
        assert_eq!(http_request_len(req), Some(req.len()));
    }

    #[test]
    fn http_request_len_lowercase_header() {
        let req = b"POST /c HTTP/1.1\r\ncontent-length: 5\r\n\r\nabcde";
        assert_eq!(http_request_len(req), Some(req.len()));
    }

    // Build a minimal Ethernet/IPv4/UDP frame to `dst_port`.
    fn udp_frame(dst_port: u16) -> Vec<u8> {
        let mut f = vec![0u8; 42];
        f[12] = 0x08;
        f[13] = 0x00; // EtherType IPv4
        f[14] = 0x45; // IPv4, IHL 5
        f[14 + 9] = 17; // protocol UDP
        let dp = dst_port.to_be_bytes();
        f[36] = dp[0]; // UDP dst port (udp offset 34, dst at +2)
        f[37] = dp[1];
        f
    }

    #[test]
    fn is_dhcp_request_cases() {
        assert!(is_dhcp_request(&udp_frame(67)));
        assert!(!is_dhcp_request(&udp_frame(68)));
        assert!(!is_dhcp_request(&[0u8; 20]));
    }

    // Build a minimal DHCP request payload with option 53 = msgtype.
    fn dhcp_payload(msgtype: u8, mac: [u8; 6]) -> Vec<u8> {
        let mut p = vec![0u8; 240];
        p[0] = 1; // BOOTREQUEST
        p[4..8].copy_from_slice(&[0xde, 0xad, 0xbe, 0xef]); // xid
        p[28..34].copy_from_slice(&mac); // chaddr
        p[236..240].copy_from_slice(&[99, 130, 83, 99]); // magic
        p.extend_from_slice(&[53, 1, msgtype]); // option 53
        p.push(255); // end
        p
    }

    #[test]
    fn dhcp_discover_offer_request_ack() {
        let mut leases = Leases::new();
        let mut out = [0u8; 320];

        let disc = dhcp_payload(1, [1, 2, 3, 4, 5, 6]);
        build_dhcp_reply(&disc, &mut out, &mut leases).expect("offer");
        assert_eq!(out[0], 2); // BOOTREPLY
        assert_eq!(out[242], 2); // option 53 value = OFFER

        let req = dhcp_payload(3, [1, 2, 3, 4, 5, 6]);
        build_dhcp_reply(&req, &mut out, &mut leases).expect("ack");
        assert_eq!(out[242], 5); // option 53 value = ACK
    }

    #[test]
    fn leases_stable_and_round_robin() {
        let mut leases = Leases::new();
        let mac_a = [0xaa; 6];
        let first = leases.ip_for(&mac_a);
        let again = leases.ip_for(&mac_a);
        assert_eq!(first, again, "same MAC -> same IP");

        // 9 distinct MACs: the 9th must not collide with a still-active earlier
        // IP. Round-robin recycles slot 0 (the oldest), never landing on .9.
        let mut leases = Leases::new();
        let mut ips = Vec::new();
        for i in 1..=9u8 {
            let mac = [i; 6];
            ips.push(leases.ip_for(&mac));
        }
        // First 8 fill slots 0..8 -> .2 ..= .9
        assert_eq!(&ips[..8], &[2, 3, 4, 5, 6, 7, 8, 9]);
        // 9th recycles slot 0 -> .2, and slot 0's old MAC ([1;6]) was evicted so
        // no active lease still claims .2 twice. It is NOT .9.
        assert_eq!(ips[8], 2);
        assert_ne!(ips[8], 9);
    }

    #[test]
    fn ip_checksum_known_header() {
        // Standard RFC 1071 worked example. The 20-byte header below has a zero
        // checksum field; the computed checksum is 0xb861.
        let hdr = [
            0x45, 0x00, 0x00, 0x73, 0x00, 0x00, 0x40, 0x00, 0x40, 0x11, 0x00,
            0x00, 0xc0, 0xa8, 0x00, 0x01, 0xc0, 0xa8, 0x00, 0xc7,
        ];
        assert_eq!(ip_checksum(&hdr), 0xb861);
    }

    #[test]
    fn parse_creds_cases() {
        let c = parse_creds(b"ssid=Net&password=pw%21").expect("has ssid");
        assert_eq!(&c.ssid[..c.ssid_len], b"Net");
        assert_eq!(&c.pass[..c.pass_len], b"pw!");

        assert!(parse_creds(b"password=only").is_none());
    }
}
