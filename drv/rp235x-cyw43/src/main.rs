// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! CYW43439 Wi-Fi driver server for the RP2350 (RP235x), Pico 2 W.
//!
//! Owns PIO2 (the gSPI PHY) plus the four CYW43 control pins and drives the
//! full bring-up: power-on, chip-detect, firmware + NVRAM upload (streamed from
//! the auxflash server), WLAN-core boot, and the SDPCM/CDC control plane (CLM,
//! `bus:txglom`, `apsta`). Exposes the LED, MAC and Wi-Fi scan over Idol
//! (`idl/rp235x-cyw43.idol`).
//!
//! This is the productionised form of the pre-kernel `cyw43_pio_detect` bring-up
//! probe: the gSPI/SDPCM logic is unchanged (only the firmware source moved from
//! a raw flash-mirror read to auxflash IPC), and the pin/PIO setup that the probe
//! ran privileged now runs once here at task start.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering::SeqCst};
use drv_auxflash_api::AuxFlash;
use drv_rp235x_cyw43_api::Cyw43Error;
use idol_runtime::RequestError;
use userlib::{task_slot, RecvMessage};

task_slot!(AUXFLASH, auxflash);

mod net;

const WL_ON: u32 = 23; // WL_REG_ON (power/reset)
const DIO: u32 = 24; // gSPI DIO (half-duplex data)
const CS: u32 = 25; // gSPI CS
const CLK: u32 = 29; // gSPI CLK

/// Bring-up diagnostics, readable over the probe before an Idol client exists.
/// [0]=chip-detect [1]=chip-id [2]=alp [3]=core-up [4]=ht [5]=f2 [6]=clm
/// [7]=firmware-bytes [8..14]=mac words [15]=0x600DF00D once init completes.
#[no_mangle]
#[used]
static DIAG: [AtomicU32; 16] = [const { AtomicU32::new(0) }; 16];

/// Capture of the first channel-2 (DATA) frame seen -- e.g. a client's DHCP
/// DISCOVER after it joins the SoftAP. Proves the F2 DATA path (smoltcp's phy).
#[no_mangle]
#[used]
static DATA_FRAME: [AtomicU32; 128] =
    [const { AtomicU32::new(0) }; 128];

/// Credentials captured from the captive-portal POST, for the STA join.
/// [0]=ready(1) [1]=ssid_len [2]=pass_len [3..11]=ssid(32B) [11..27]=pass(64B).
#[no_mangle]
#[used]
static CREDS: [AtomicU32; 27] = [const { AtomicU32::new(0) }; 27];

/// Nearby SSIDs from the startup scan, packed as newline-separated names for the
/// portal's network dropdown (up to ~400 bytes).
#[no_mangle]
#[used]
static SCAN_SSIDS: [AtomicU32; 100] = [const { AtomicU32::new(0) }; 100];

// Pico W CYW43439 NVRAM (config vars), from cyw43-driver wifi_nvram_43439.h.
static NVRAM: [u32; 186] = [
    0x4152564e, 0x7665524d, 0x6552243d, 0x6d002476, 0x69666e61, 0x78303d64,
    0x00306432, 0x646f7270, 0x303d6469, 0x32373078, 0x65760037, 0x6469646e,
    0x3178303d, 0x00346534, 0x69766564, 0x78303d64, 0x32653334, 0x616f6200,
    0x79746472, 0x303d6570, 0x38383078, 0x6f620037, 0x72647261, 0x303d7665,
    0x30313178, 0x6f620030, 0x6e647261, 0x323d6d75, 0x616d0032, 0x64646163,
    0x30303d72, 0x3a30413a, 0x623a3035, 0x39353a35, 0x0065353a, 0x6d6f7273,
    0x3d766572, 0x62003131, 0x6472616f, 0x67616c66, 0x78303d73, 0x30343030,
    0x31303034, 0x616f6200, 0x6c666472, 0x33736761, 0x3078303d, 0x30303034,
    0x00303030, 0x6c617478, 0x71657266, 0x3437333d, 0x6e003030, 0x6372636f,
    0x6100313d, 0x323d3067, 0x61003535, 0x3d673261, 0x63630031, 0x3d65646f,
    0x004c4c41, 0x69306170, 0x69737374, 0x78303d74, 0x65003032, 0x61707478,
    0x6e696167, 0x303d6732, 0x32617000, 0x3d306761, 0x3836312d, 0x3631372c,
    0x382d2c31, 0x41003032, 0x696d5676, 0x30635f64, 0x3078303d, 0x6378302c,
    0x63630038, 0x7277706b, 0x7366666f, 0x3d307465, 0x616d0035, 0x67327078,
    0x383d3061, 0x78740034, 0x62727770, 0x666f6b63, 0x6300363d, 0x77626b63,
    0x67323032, 0x303d6f70, 0x67656c00, 0x6d64666f, 0x30327762, 0x6f706732,
    0x3678303d, 0x31313136, 0x00313131, 0x6273636d, 0x32303277, 0x3d6f7067,
    0x37377830, 0x31313137, 0x70003131, 0x62706f72, 0x32303277, 0x3d6f7067,
    0x64647830, 0x64666f00, 0x6769646d, 0x746c6966, 0x65707974, 0x0038313d,
    0x6d64666f, 0x66676964, 0x74746c69, 0x62657079, 0x38313d65, 0x70617000,
    0x646f6d64, 0x00313d65, 0x64706170, 0x696c6176, 0x73657464, 0x00313d74,
    0x61636170, 0x7864696c, 0x343d6732, 0x61700035, 0x70656470, 0x66666f73,
    0x3d746573, 0x0030332d, 0x64706170, 0x69646e65, 0x353d7864, 0x746c0038,
    0x6d786365, 0x303d7875, 0x65746c00, 0x61707863, 0x6d756e64, 0x3078303d,
    0x00323031, 0x6365746c, 0x736e6678, 0x303d6c65, 0x00343478, 0x6365746c,
    0x69636778, 0x6f697067, 0x3078303d, 0x6c690031, 0x63616d30, 0x72646461,
    0x3a30303d, 0x343a3039, 0x35633a63, 0x3a32313a, 0x77003833, 0x6469306c,
    0x3478303d, 0x00623133, 0x64616564, 0x5f6e616d, 0x303d6f74, 0x66666678,
    0x66666666, 0x756d0066, 0x616e6578, 0x78303d62, 0x00303031, 0x72757073,
    0x666e6f63, 0x303d6769, 0x67003378, 0x6374696c, 0x61625f68, 0x5f646573,
    0x6d737263, 0x313d6e69, 0x63746200, 0x646f6d5f, 0x00313d65, 0x00000000,
];

/// gSPI read program (embassy low-speed variant), verified against the rp2
/// assembler. 0:out pins,1 side0  1:jmp x-- 0 side1  2:set pindirs,0 side0
/// 3:nop side0  4:in pins,1 side1  5:jmp y-- 4 side0.
const GSPI: [u16; 6] = [0x6001, 0x1040, 0xE080, 0xA042, 0x5001, 0x0084];

const IOCTRL: u32 = 0x408;
const RESETCTRL: u32 = 0x800;
const WLAN_WRAP: u32 = 0x1810_3000;
const SOCSRAM_WRAP: u32 = 0x1810_4000;
const SOCSRAM_BASE: u32 = 0x1800_4000;
const RAM_SIZE: u32 = 0x8_0000;

struct Cyw43 {
    pio: rp235x_pac::PIO2,
    sio: rp235x_pac::SIO,
    tx_seq: u8,
    credit: u8,
    mac: [u8; 6],
    status: u32,
    ssid: [u8; 32],
    ssid_len: u8,
    /// gSPI CS-low settle delay (cycles). Large + safe during bring-up, then
    /// dropped for the runtime loop so it keeps up with client packet bursts.
    settle: u32,
    /// DHCP lease table: client MAC per pool slot; IP = 192.168.4.(2 + slot).
    leases: [[u8; 6]; 8],
    lease_count: u8,
    /// BDC interface index for TX frames: 1 = AP (portal), 0 = STA (after join).
    tx_iface: u32,
}

impl Cyw43 {
    fn cmd_word(wr: bool, func: u32, addr: u32, len: u32) -> u32 {
        ((wr as u32) << 31) | (1 << 30) | (func << 28) | (addr << 11) | len
    }
    fn swap16(x: u32) -> u32 {
        x.rotate_left(16)
    }

    /// Overwrite the SM's `set` target pin and execute a one-off instruction.
    fn set_pin(&self, pin: u32, insn: u16) {
        let sm = self.pio.sm(0);
        sm.sm_pinctrl()
            .modify(|_, w| unsafe { w.set_base().bits(pin as u8) });
        sm.sm_instr().write(|w| unsafe { w.sm0_instr().bits(insn) });
    }

    /// One gSPI transaction: clock out `out_words`, turn DIO around, clock in
    /// `in_words`. CS is CPU-driven (SIO) around the transfer.
    fn xfer(&self, out_words: &[u32], in_words: &mut [u32]) {
        let pio = &self.pio;
        let sio = &self.sio;
        let sm = pio.sm(0);
        let x_bits = out_words.len() as u32 * 32 - 1;
        let y_bits = in_words.len() as u32 * 32 - 1;
        sio.gpio_out_set().write(|w| unsafe { w.bits(1 << CS) });
        cortex_m::asm::delay(1500);
        sio.gpio_out_clr().write(|w| unsafe { w.bits(1 << CS) });
        cortex_m::asm::delay(self.settle);
        pio.ctrl().modify(|_, w| unsafe { w.sm_enable().bits(0) });
        sm.sm_shiftctrl().modify(|_, w| w.fjoin_rx().set_bit());
        sm.sm_shiftctrl().modify(|_, w| w.fjoin_rx().clear_bit());
        self.set_pin(DIO, 0xE081); // DIO pindir out
        pio.ctrl()
            .modify(|_, w| unsafe { w.sm_restart().bits(1).clkdiv_restart().bits(1) });
        pio.txf(0).write(|w| unsafe { w.bits(x_bits) });
        sm.sm_instr().write(|w| unsafe { w.sm0_instr().bits(0x6020) }); // out x,32
        pio.txf(0).write(|w| unsafe { w.bits(y_bits) });
        sm.sm_instr().write(|w| unsafe { w.sm0_instr().bits(0x6040) }); // out y,32
        sm.sm_instr().write(|w| unsafe { w.sm0_instr().bits(0x0000) }); // jmp 0
        pio.ctrl().modify(|_, w| unsafe { w.sm_enable().bits(1) });
        for &word in out_words {
            let mut s = 0u32;
            while (pio.flevel().read().bits() & 0xF) >= 4 && s < 100_000 {
                s += 1;
            }
            pio.txf(0).write(|w| unsafe { w.bits(word) });
        }
        for slot in in_words.iter_mut() {
            let mut spins = 0u32;
            while pio.fstat().read().rxempty().bits() & 1 != 0 {
                spins += 1;
                if spins > 8_000 {
                    break;
                }
            }
            *slot = pio.rxf(0).read().bits();
        }
        sio.gpio_out_set().write(|w| unsafe { w.bits(1 << CS) });
    }

    fn bp_set_window(&self, addr: u32) {
        let base = addr & !0x7FFF;
        self.xfer(&[Self::cmd_word(true, 1, 0x1000C, 1), (base >> 24) & 0xff], &mut [0u32; 1]);
        self.xfer(&[Self::cmd_word(true, 1, 0x1000B, 1), (base >> 16) & 0xff], &mut [0u32; 1]);
        self.xfer(&[Self::cmd_word(true, 1, 0x1000A, 1), (base >> 8) & 0xff], &mut [0u32; 1]);
    }
    fn bp_read8(&self, addr: u32) -> u32 {
        self.bp_set_window(addr);
        let mut r = [0u32; 2];
        self.xfer(&[Self::cmd_word(false, 1, addr & 0x7FFF, 1)], &mut r);
        r[1]
    }
    fn bp_write8(&self, addr: u32, val: u32) {
        self.bp_set_window(addr);
        self.xfer(&[Self::cmd_word(true, 1, addr & 0x7FFF, 1), val], &mut [0u32; 1]);
    }
    fn bp_read32(&self, addr: u32) -> u32 {
        self.bp_set_window(addr);
        let mut r = [0u32; 2];
        self.xfer(&[Self::cmd_word(false, 1, (addr & 0x7FFF) | 0x8000, 4)], &mut r);
        r[1]
    }
    fn bp_write32(&self, addr: u32, val: u32) {
        self.bp_set_window(addr);
        self.xfer(&[Self::cmd_word(true, 1, (addr & 0x7FFF) | 0x8000, 4), val], &mut [0u32; 1]);
    }

    /// Stream `src` bytes into WLAN-core RAM at backplane `dest`, in <=64-byte
    /// bursts that never cross the 32 KiB window. A partial final word is zero
    /// padded.
    fn bp_stream_bytes(&self, dest: u32, src: &[u8]) {
        let mut burst = [0u32; 17];
        let words = src.len().div_ceil(4);
        let mut i = 0usize;
        while i < words {
            let addr = dest + (i * 4) as u32;
            let window_rem = (0x8000 - (addr & 0x7FFF)) as usize;
            let n = (window_rem / 4).min(16).min(words - i);
            self.bp_set_window(addr);
            burst[0] = Self::cmd_word(true, 1, (addr & 0x7FFF) | 0x8000, (n * 4) as u32);
            for k in 0..n {
                let b = (i + k) * 4;
                burst[1 + k] = u32::from_le_bytes([
                    src.get(b).copied().unwrap_or(0),
                    src.get(b + 1).copied().unwrap_or(0),
                    src.get(b + 2).copied().unwrap_or(0),
                    src.get(b + 3).copied().unwrap_or(0),
                ]);
            }
            self.xfer(&burst[..1 + n], &mut [0u32; 1]);
            i += n;
        }
    }

    fn disable_core(&self, base: u32) {
        if self.bp_read8(base + RESETCTRL) & 0x1 != 0 {
            return;
        }
        self.bp_write8(base + IOCTRL, 0);
        let _ = self.bp_read8(base + IOCTRL);
        self.bp_write8(base + RESETCTRL, 0x1);
        let _ = self.bp_read8(base + RESETCTRL);
    }
    fn reset_core_up(&self, base: u32) {
        self.bp_write8(base + IOCTRL, 0x2 | 0x1);
        let _ = self.bp_read8(base + IOCTRL);
        self.bp_write8(base + RESETCTRL, 0);
        cortex_m::asm::delay(200_000);
        self.bp_write8(base + IOCTRL, 0x1);
        let _ = self.bp_read8(base + IOCTRL);
        cortex_m::asm::delay(200_000);
    }

    /// Read one F2 frame. Returns (got, channel, cdc_status, bus_credit, cdc_id).
    fn rx(&self, fr: &mut [u32; 512]) -> (bool, u32, u32, u32, u32) {
        let mut rr = [0u32; 1];
        self.xfer(&[Self::cmd_word(false, 0, 0x8, 4)], &mut rr);
        if rr[0] & 0x100 == 0 {
            return (false, 0xFF, 0, 0xFF, 0);
        }
        let len = ((rr[0] >> 9) & 0x7FF) as usize;
        if len < 12 {
            return (false, 0xFF, 0, 0xFF, 0);
        }
        let w = len.div_ceil(4).min(512);
        self.xfer(&[Self::cmd_word(false, 2, 0, len as u32)], &mut fr[..w]);
        let chan = (fr[1] >> 8) & 0xFF;
        let bdc = (fr[2] >> 8) & 0xFF;
        let (status, id) = if chan == 0 {
            let hl = (((fr[1] >> 24) & 0xFF) / 4) as usize;
            (fr[hl + 3], (fr[hl + 2] >> 16) & 0xFFFF)
        } else {
            (0xEEEE_EEEE, 0)
        };
        (true, chan, status, bdc, id)
    }

    /// Advance the SDPCM credit from a received frame's bus_data_credit.
    fn take_credit(&mut self, chan: u32, bdc: u32) {
        if chan < 3 && (bdc.wrapping_sub(self.credit as u32) & 0xFF) <= 20 {
            self.credit = bdc as u8;
        }
    }

    /// SET ioctl with a u32-word payload; returns the CDC status.
    fn do_ioctl(
        &mut self,
        cmd: u32,
        id: u32,
        iface: u32,
        payload: &[u32],
        pbytes: usize,
        fr: &mut [u32; 512],
    ) -> u32 {
        let mut g = 0u32;
        while self.credit == self.tx_seq && g < 8000 {
            g += 1;
            let (got, chan, _s, bdc, _i) = self.rx(fr);
            if got {
                self.take_credit(chan, bdc);
            } else {
                cortex_m::asm::delay(10_000);
            }
        }
        let mut b = [0u32; 264];
        let total = 12 + 16 + pbytes;
        b[0] = 0xE000_0000 | total as u32;
        b[1] = (total as u32 & 0xFFFF) | (((!(total as u32)) & 0xFFFF) << 16);
        b[2] = 0x0C00_0000 | self.tx_seq as u32;
        b[4] = cmd;
        b[5] = pbytes as u32;
        b[6] = 0x0000_0002 | (iface << 12) | (id << 16);
        for (k, &word) in payload.iter().enumerate() {
            b[8 + k] = word;
        }
        self.xfer(&b[..8 + pbytes.div_ceil(4)], &mut [0u32; 1]);
        self.tx_seq = self.tx_seq.wrapping_add(1);
        let mut st = 0xEEEE_EEEEu32;
        for _ in 0..3000u32 {
            let (got, chan, status, bdc, rid) = self.rx(fr);
            if got {
                self.take_credit(chan, bdc);
                if chan == 0 && rid == id {
                    st = status;
                    break;
                }
            } else {
                cortex_m::asm::delay(20_000);
            }
        }
        st
    }

    /// ioctl with a byte payload packed little-endian into words (no manual
    /// u32 byte-order). kind 2=SET, 0=GET; cdc_len is the CDC length field.
    fn do_ioctl_b(
        &mut self,
        kind: u32,
        cmd: u32,
        id: u32,
        iface: u32,
        payload: &[u8],
        cdc_len: usize,
        fr: &mut [u32; 512],
    ) -> u32 {
        let mut g = 0u32;
        while self.credit == self.tx_seq && g < 8000 {
            g += 1;
            let (got, chan, _s, bdc, _i) = self.rx(fr);
            if got {
                self.take_credit(chan, bdc);
            } else {
                cortex_m::asm::delay(10_000);
            }
        }
        let mut b = [0u32; 264];
        let total = 12 + 16 + cdc_len;
        b[0] = 0xE000_0000 | total as u32;
        b[1] = (total as u32 & 0xFFFF) | (((!(total as u32)) & 0xFFFF) << 16);
        b[2] = 0x0C00_0000 | self.tx_seq as u32;
        b[4] = cmd;
        b[5] = cdc_len as u32;
        b[6] = kind | (iface << 12) | (id << 16);
        for (j, &byte) in payload.iter().enumerate() {
            b[8 + j / 4] |= (byte as u32) << (8 * (j % 4));
        }
        self.xfer(&b[..8 + cdc_len.div_ceil(4)], &mut [0u32; 1]);
        self.tx_seq = self.tx_seq.wrapping_add(1);
        let mut st = 0xEEEE_EEEEu32;
        for _ in 0..3000u32 {
            let (got, chan, status, bdc, rid) = self.rx(fr);
            if got {
                self.take_credit(chan, bdc);
                if chan == 0 && rid == id {
                    st = status;
                    break;
                }
            } else {
                cortex_m::asm::delay(20_000);
            }
        }
        st
    }

    /// Full bring-up. Assumes the pins + PIO SM have been configured by `setup`.
    fn new(aux: &AuxFlash, fr: &mut [u32; 512]) -> Result<Self, Cyw43Error> {
        let p = unsafe { rp235x_pac::Peripherals::steal() };
        // Board id from watchdog scratch1/2 (stashed by the pre-kernel main, same
        // source as the USB serial) -> SoftAP SSID "hubris-<16 hex>".
        let id: u64 = (p.WATCHDOG.scratch1().read().bits() as u64) << 32
            | p.WATCHDOG.scratch2().read().bits() as u64;
        setup(&p);
        let mut me = Cyw43 {
            pio: p.PIO2,
            sio: p.SIO,
            tx_seq: 0,
            credit: 1,
            mac: [0; 6],
            status: 0,
            ssid: [0; 32],
            ssid_len: 0,
            settle: 30_000, // conservative during bring-up (firmware upload)
            leases: [[0; 6]; 8],
            lease_count: 0,
            tx_iface: 1, // AP interface until the STA join switches it to 0
        };
        const HEX: &[u8; 16] = b"0123456789ABCDEF";
        me.ssid[..7].copy_from_slice(b"hubris-");
        for i in 0..16 {
            me.ssid[7 + i] = HEX[((id >> (60 - 4 * i)) & 0xf) as usize];
        }
        me.ssid_len = 23;
        // cyw43_spi_init: CLK + DIO output-low.
        me.set_pin(CLK, 0xE081);
        me.set_pin(CLK, 0xE000);
        me.set_pin(DIO, 0xE081);
        me.set_pin(DIO, 0xE000);

        // Chip-detect: read F0 TEST_RO swap16'd until FEEDBEAD.
        let read_ro = Self::swap16(Self::cmd_word(false, 0, 0x14, 4));
        let mut pass = 0u32;
        let chip = loop {
            let mut b = [0u32; 1];
            me.xfer(&[read_ro], &mut b);
            pass += 1;
            let v = Self::swap16(b[0]);
            if v == 0xFEED_BEAD || pass >= 32 {
                break v;
            }
        };
        DIAG[0].store(chip, SeqCst);

        // REG_BUS_CTRL: 32-bit words | high-speed | int-pol-high | wake |
        // resp-delay 0x4 | status-enable | intr-with-status.
        let bus_ctrl: u32 = 0x1 | 0x10 | 0x20 | 0x80 | (0x4 << 8) | ((0x1 | 0x2) << 16);
        me.xfer(&[Self::swap16(Self::cmd_word(true, 0, 0x00, 4)), Self::swap16(bus_ctrl)], &mut [0u32; 1]);
        me.xfer(&[Self::cmd_word(true, 0, 0x1d, 1), 4], &mut [0u32; 1]); // SPI_RESP_DELAY_F1
        // ALP clock.
        me.xfer(&[Self::cmd_word(true, 1, 0x1000E, 1), 0x08], &mut [0u32; 1]);
        me.xfer(&[Self::cmd_word(true, 1, 0x1_0008, 1), 0x10], &mut [0u32; 1]);
        let mut aspin = 0u32;
        let alp = loop {
            let mut r = [0u32; 2];
            me.xfer(&[Self::cmd_word(false, 1, 0x1000E, 1)], &mut r);
            aspin += 1;
            if (r[1] & 0x40) != 0 || aspin >= 2000 {
                break r[1];
            }
        };
        me.xfer(&[Self::cmd_word(true, 1, 0x1000E, 1), 0], &mut [0u32; 1]);
        DIAG[2].store(alp, SeqCst);

        // Chip-ID via a windowed backplane read.
        me.bp_set_window(0x1800_0000);
        let mut cid = [0u32; 2];
        me.xfer(&[Self::cmd_word(false, 1, 0x8000, 4)], &mut cid);
        DIAG[1].store(cid[1], SeqCst);

        // Prep the cores, then upload the firmware from auxflash into WLAN RAM.
        me.disable_core(WLAN_WRAP);
        me.disable_core(SOCSRAM_WRAP);
        me.reset_core_up(SOCSRAM_WRAP);
        me.bp_write32(SOCSRAM_BASE + 0x10, 3);
        me.bp_write32(SOCSRAM_BASE + 0x44, 0);
        let fw_bytes = me.upload_blob(aux, *b"WIFI", 0)?;
        DIAG[7].store(fw_bytes, SeqCst);
        // Read back the firmware at 3 points to check the auxflash stream landed.
        DIAG[10].store(me.bp_read32(0), SeqCst);
        DIAG[11].store(me.bp_read32(0x8000), SeqCst);
        DIAG[12].store(me.bp_read32(0x3_8000), SeqCst);
        // NVRAM near the top of RAM + the length-magic word.
        let nvram_len = (NVRAM.len() * 4) as u32;
        let nvram_addr = RAM_SIZE - 4 - nvram_len;
        let nvram_bytes: &[u8] = unsafe {
            core::slice::from_raw_parts(NVRAM.as_ptr() as *const u8, NVRAM.len() * 4)
        };
        me.bp_stream_bytes(nvram_addr, nvram_bytes);
        let nvram_words = nvram_len / 4;
        let magic = ((!nvram_words & 0xFFFF) << 16) | (nvram_words & 0xFFFF);
        me.bp_write32(RAM_SIZE - 4, magic);

        // Boot the WLAN core.
        me.reset_core_up(WLAN_WRAP);
        userlib::hl::sleep_for(20); // guaranteed wall-clock settle (preemption-safe)
        let io = me.bp_read8(WLAN_WRAP + IOCTRL) & 0xff;
        let rc = me.bp_read8(WLAN_WRAP + RESETCTRL) & 0xff;
        let core_up = (io & 0x3) == 0x1 && (rc & 0x1) == 0;
        DIAG[3].store(if core_up { 0xC0DE_600D } else { (io << 8) | rc }, SeqCst);

        // HT clock + F2 ready.
        userlib::hl::sleep_for(40); // let the firmware spin up before requesting HT
        me.xfer(&[Self::cmd_word(true, 1, 0x1000F, 1), 0], &mut [0u32; 1]); // PULL_UP = 0
        me.xfer(&[Self::cmd_word(true, 1, 0x1000E, 1), 0x10], &mut [0u32; 1]); // HT_AVAIL_REQ
        let mut htspin = 0u32;
        let ht = loop {
            let mut r = [0u32; 2];
            me.xfer(&[Self::cmd_word(false, 1, 0x1000E, 1)], &mut r);
            htspin += 1;
            if (r[1] & 0x80) != 0 || htspin >= 40000 {
                break r[1];
            }
        };
        DIAG[4].store(ht, SeqCst);
        me.xfer(&[Self::cmd_word(true, 1, 0x1_0008, 1), 0x20], &mut [0u32; 1]);
        me.xfer(&[Self::cmd_word(true, 0, 0x06, 2), 0x0020], &mut [0u32; 1]);
        userlib::hl::sleep_for(50); // let the firmware bring up the F2 data path
        let mut f2spin = 0u32;
        let f2 = loop {
            let mut r = [0u32; 1];
            me.xfer(&[Self::cmd_word(false, 0, 0x8, 4)], &mut r);
            f2spin += 1;
            if (r[0] & 0x20) != 0 || f2spin >= 40000 {
                break r[0];
            }
        };
        DIAG[5].store(f2, SeqCst);

        // Control-plane init: CLM -> bus:txglom -> apsta -> read MAC.
        me.load_clm(aux, fr)?;
        let txglom = [0x3a73_7562, 0x6c67_7874, 0x0000_6d6f, 0x0000_0000];
        me.do_ioctl(0x107, 5, 0, &txglom, 11 + 4, fr);
        let apsta = [0x7473_7061, 0x0001_0061, 0x0000_0000];
        me.do_ioctl(0x107, 6, 0, &apsta, 6 + 4, fr);
        // AMPDU config from cyw43_ll_wifi_on. ampdu_rx_factor/ampdu_mpdu set up
        // RX de-aggregation; without them the firmware cannot reassemble
        // AMPDU-aggregated UNICAST data frames and drops unicast-to-host (while
        // non-aggregated broadcast/multicast, e.g. DHCP, still arrive).
        let mut abw = [0u8; 20];
        abw[..15].copy_from_slice(b"ampdu_ba_wsize\0");
        abw[15] = 8;
        me.do_ioctl_b(2, 0x107, 8, 0, &abw, 19, fr);
        let mut amp = [0u8; 16];
        amp[..11].copy_from_slice(b"ampdu_mpdu\0");
        amp[11] = 4;
        me.do_ioctl_b(2, 0x107, 9, 0, &amp, 15, fr);
        let mut arf = [0u8; 20];
        arf[..16].copy_from_slice(b"ampdu_rx_factor\0"); // value 0
        me.do_ioctl_b(2, 0x107, 10, 0, &arf, 20, fr);
        let mac_stat = me.do_ioctl_b(0, 0x106, 7, 0, b"cur_etheraddr\0", 14 + 6, fr);
        if mac_stat == 0 {
            let hl = (((fr[1] >> 24) & 0xFF) / 4) as usize;
            let w0 = fr[hl + 4];
            let w1 = fr[hl + 5];
            me.mac = [
                w0 as u8,
                (w0 >> 8) as u8,
                (w0 >> 16) as u8,
                (w0 >> 24) as u8,
                w1 as u8,
                (w1 >> 8) as u8,
            ];
            DIAG[8].store(w0, SeqCst);
            DIAG[9].store(w1, SeqCst);
        }

        me.status = chip;
        // Keep the full CS-settle at runtime: DHCP TX must be reliable (a
        // corrupted OFFER/ACK -> the client never finalizes its lease). The
        // provisioning traffic is paced, so the slower loop is fine.
        DIAG[15].store(0x600D_F00D, SeqCst);
        Ok(me)
    }

    /// Stream an uncompressed auxflash blob into WLAN RAM at `dest`, returning
    /// the byte count.
    fn upload_blob(
        &self,
        aux: &AuxFlash,
        tag: [u8; 4],
        dest: u32,
    ) -> Result<u32, Cyw43Error> {
        let blob = aux.get_blob_by_tag(tag).map_err(|_| Cyw43Error::NotReady)?;
        let mut buf = [0u8; 128];
        let mut pos = blob.start;
        let mut off = 0u32;
        while pos < blob.end {
            let amount = (blob.end - pos).min(buf.len() as u32);
            aux.read_slot_with_offset(blob.slot, pos, &mut buf[..amount as usize])
                .map_err(|_| Cyw43Error::NotReady)?;
            self.bp_stream_bytes(dest + off, &buf[..amount as usize]);
            off += amount;
            pos += amount;
        }
        Ok(off)
    }

    /// Stream the CLM blob into a SET_VAR "clmload" ioctl.
    fn load_clm(&mut self, aux: &AuxFlash, fr: &mut [u32; 512]) -> Result<(), Cyw43Error> {
        let mut clm_pl = [0u32; 251];
        clm_pl[0] = 0x6c6d_6c63; // "clml"
        clm_pl[1] = 0x0064_616f; // "oad\0"
        clm_pl[2] = 0x0002_1006; // flag=BEGIN|END|HANDLER_VER, dload_type=CLM
        clm_pl[3] = 984; // len
        let blob = aux.get_blob_by_tag(*b"WCLM").map_err(|_| Cyw43Error::NotReady)?;
        let mut buf = [0u8; 128];
        let mut pos = blob.start;
        let mut coff = 0usize;
        while pos < blob.end {
            let amount = (blob.end - pos).min(buf.len() as u32);
            aux.read_slot_with_offset(blob.slot, pos, &mut buf[..amount as usize])
                .map_err(|_| Cyw43Error::NotReady)?;
            for &byte in &buf[..amount as usize] {
                clm_pl[5 + coff / 4] |= (byte as u32) << (8 * (coff % 4));
                coff += 1;
            }
            pos += amount;
        }
        let clm = me_do_clm(self, &clm_pl, fr);
        DIAG[6].store(clm, SeqCst);
        Ok(())
    }

    /// Drive the WL_GPIO0 LED via a `gpioout` SET_VAR ioctl.
    fn led(&mut self, on: bool, fr: &mut [u32; 512]) {
        let mut g = 0u32;
        while self.credit == self.tx_seq && g < 8000 {
            g += 1;
            let (got, chan, _s, bdc, _i) = self.rx(fr);
            if got {
                self.take_credit(chan, bdc);
            } else {
                cortex_m::asm::delay(10_000);
            }
        }
        let id = 20u32;
        let val = if on { 1u32 } else { 0u32 };
        let mut b = [0u32; 16];
        let total = 12 + 16 + 16usize;
        b[0] = 0xE000_0000 | total as u32;
        b[1] = (total as u32 & 0xFFFF) | (((!(total as u32)) & 0xFFFF) << 16);
        b[2] = 0x0C00_0000 | self.tx_seq as u32;
        b[4] = 0x0000_0107;
        b[5] = 16;
        b[6] = 0x0000_0002 | (id << 16);
        b[8] = 0x6f69_7067; // "gpio"
        b[9] = 0x0074_756f; // "out\0"
        b[10] = 0x0000_0001; // mask = 1<<0
        b[11] = val;
        self.xfer(&b[..12], &mut [0u32; 1]);
        self.tx_seq = self.tx_seq.wrapping_add(1);
        for _ in 0..2000u32 {
            let (got, chan, _s, bdc, rid) = self.rx(fr);
            if got {
                self.take_credit(chan, bdc);
                if chan == 0 && rid == id {
                    break;
                }
            } else {
                cortex_m::asm::delay(20_000);
            }
        }
    }

    /// Run an active escan of all channels; returns the number of result frames.
    fn scan(&mut self, fr: &mut [u32; 512]) -> u32 {
        let mut evt = [0u8; 40];
        evt[..18].copy_from_slice(b"bsscfg:event_msgs\0");
        evt[22] = 0x49;
        evt[23] = 0x10;
        evt[24] = 0x01;
        evt[27] = 0x40;
        evt[30] = 0x20; // event 69 (ESCAN_RESULT)
        evt[32] = 0x01;
        self.do_ioctl_b(2, 0x107, 8, 0, &evt, 40, fr);
        self.do_ioctl_b(2, 2, 9, 0, &[], 0, fr); // WLC_UP
        let mut esc = [0u8; 80];
        esc[..6].copy_from_slice(b"escan\0");
        esc[6] = 1;
        esc[10] = 1;
        esc[12] = 0x34;
        esc[13] = 0x12;
        esc[50..56].iter_mut().for_each(|x| *x = 0xff);
        esc[56] = 2;
        esc[58..74].iter_mut().for_each(|x| *x = 0xff);
        self.do_ioctl_b(2, 0x107, 10, 0, &esc, 80, fr);
        let mut events = 0u32;
        let mut ssids = [0u8; 384]; // packed "ssid\nssid\n..."
        let mut slen = 0usize;
        for _ in 0..15_000u32 {
            let (got, chan, _s, bdc, _i) = self.rx(fr);
            if got {
                self.take_credit(chan, bdc);
                if chan == 1 {
                    events += 1;
                    // Parse the AP SSID from the escan-result event: SSID_len is at
                    // SDPCM header_length + 110, the name follows. Add it, deduped.
                    let byte = |p: usize| (fr[p / 4] >> (8 * (p % 4))) as u8;
                    let lp = ((fr[1] >> 24) & 0xFF) as usize + 110;
                    let sl = byte(lp) as usize;
                    if (1..=32).contains(&sl) {
                        let mut ssid = [0u8; 32];
                        let mut ok = true;
                        for (i, s) in ssid[..sl].iter_mut().enumerate() {
                            let c = byte(lp + 1 + i);
                            if !(0x20..=0x7e).contains(&c) {
                                ok = false;
                                break;
                            }
                            *s = c;
                        }
                        let seen =
                            ssids[..slen].split(|&b| b == b'\n').any(|e| e == &ssid[..sl]);
                        if ok && !seen && slen + sl < ssids.len() {
                            ssids[slen..slen + sl].copy_from_slice(&ssid[..sl]);
                            slen += sl;
                            ssids[slen] = b'\n';
                            slen += 1;
                        }
                    }
                }
            } else {
                cortex_m::asm::delay(8_000);
            }
        }
        // Pack the deduped SSID list into SCAN_SSIDS for the portal.
        for (i, w) in SCAN_SSIDS.iter().enumerate() {
            let mut word = 0u32;
            for j in 0..4 {
                let k = i * 4 + j;
                if k < slen {
                    word |= (ssids[k] as u32) << (8 * j);
                }
            }
            w.store(word, SeqCst);
        }
        events
    }

    /// Send one Ethernet frame over the F2 DATA channel (SDPCM ch2 + 2B pad +
    /// BDC, data_offset 0 so the frame starts at byte 18). Honors SDPCM flow
    /// control. Returns false if no credit was available. This is smoltcp's TX.
    fn send_frame(&mut self, eth: &[u8], fr: &mut [u32; 512]) -> bool {
        self.settle = 30_000; // TX must be reliable (corrupt OFFER/ACK -> no lease)
        let mut g = 0u32;
        while self.credit == self.tx_seq && g < 8000 {
            g += 1;
            let (got, chan, _s, bdc, _i) = self.rx(fr);
            if got {
                self.take_credit(chan, bdc);
            } else {
                cortex_m::asm::delay(10_000);
            }
        }
        if self.credit == self.tx_seq {
            return false;
        }
        let total = 18 + eth.len(); // SDPCM(12) + pad(2) + BDC(4) + eth
        let total_pad = (total + 3) & !3;
        let mut b = [0u32; 400];
        b[0] = 0xE000_0000 | total as u32; // gSPI F2 write
        b[1] = (total as u32 & 0xFFFF) | (((!(total as u32)) & 0xFFFF) << 16);
        b[2] = (self.tx_seq as u32) | (2 << 8) | (14 << 24); // seq, chan=2, hdr_len=14
        b[3] = 0;
        b[4] = 0x20u32 << 16; // pad,pad,BDC.flags=0x20,BDC.priority=0
        b[5] = self.tx_iface; // BDC.flags2 = interface (1 AP portal, 0 STA)
        for (j, &byte) in eth.iter().enumerate() {
            let pos = 18 + j;
            b[1 + pos / 4] |= (byte as u32) << (8 * (pos % 4));
        }
        self.xfer(&b[..1 + total_pad / 4], &mut [0u32; 1]);
        self.tx_seq = self.tx_seq.wrapping_add(1);
        true
    }

    /// Try to receive one Ethernet frame (SDPCM channel 2). Returns the number
    /// of bytes copied into `out` (0 if no DATA frame). This is smoltcp's RX.
    fn recv_frame(&mut self, out: &mut [u8], fr: &mut [u32; 512]) -> usize {
        self.settle = 3_000; // RX can be fast; must keep up with client bursts (ARP/REQUEST)
        let (got, chan, _s, bdc, _i) = self.rx(fr);
        if !got {
            return 0;
        }
        // Diagnostic: per-SDPCM-channel histogram in DATA_FRAME[96..112].
        if (chan as usize) < 16 {
            let i = 96 + chan as usize;
            DATA_FRAME[i].store(DATA_FRAME[i].load(SeqCst).wrapping_add(1), SeqCst);
        }
        self.take_credit(chan, bdc);
        if chan != 2 {
            return 0;
        }
        // SDPCM header_length is PER-FRAME (SDPCM byte 7), not always 14. The old
        // hardcode (eth_start = 18 + data_offset*4 == 14+4+...) only worked for
        // frames with header_length==14 (broadcast DHCP/mDNS). Unicast/ARP data
        // frames can use a different header_length, so the Ethernet frame was
        // extracted from the wrong offset -> L2 dst never matched our MAC and they
        // looked dropped. Match the reference: eth_start = hdr_len + 4 + off*4.
        let hdr_len = ((fr[1] >> 24) & 0xFF) as usize; // SDPCM header_length
        let do_byte = hdr_len + 3; // BDC.data_offset byte
        let data_offset =
            ((fr[do_byte / 4] >> (8 * (do_byte % 4))) & 0xFF) as usize;
        let eth_start = hdr_len + 4 + data_offset * 4;
        let total = (fr[0] & 0xFFFF) as usize; // SDPCM len
        if total <= eth_start {
            DATA_FRAME[116].store(DATA_FRAME[116].load(SeqCst).wrapping_add(1), SeqCst); // chan-2 length drops
            return 0;
        }
        DATA_FRAME[122].store(hdr_len as u32, SeqCst); // last RX header_length (diag)
        let n = (total - eth_start).min(out.len());
        for (i, o) in out[..n].iter_mut().enumerate() {
            let pos = eth_start + i;
            *o = (fr[pos / 4] >> (8 * (pos % 4))) as u8;
        }
        n
    }

    /// Join the WPA2-PSK network stored in CREDS as a station. Brings the AP
    /// (bsscfg 1) down first, runs the WPA2 join sequence (cyw43_ll_wifi_join
    /// order), then polls WLC_GET_BSSID for a stable association. Returns
    /// 0 = connected, 1 = failed (wrong password / not found / timeout).
    fn sta_join(&mut self, fr: &mut [u32; 512]) -> u32 {
        let sl = CREDS[1].load(SeqCst) as usize;
        let pl = CREDS[2].load(SeqCst) as usize;
        let mut ssid = [0u8; 32];
        let mut pass = [0u8; 64];
        for i in 0..8 {
            ssid[i * 4..i * 4 + 4]
                .copy_from_slice(&CREDS[3 + i].load(SeqCst).to_le_bytes());
        }
        for i in 0..16 {
            pass[i * 4..i * 4 + 4]
                .copy_from_slice(&CREDS[11 + i].load(SeqCst).to_le_bytes());
        }

        // Bring the AP (bsscfg 1) down.
        let mut bss = [0u8; 12];
        bss[..4].copy_from_slice(b"bss\0");
        bss[4] = 1; // AP index; value (down) stays 0
        self.do_ioctl_b(2, 0x107, 60, 0, &bss, 12, fr);

        // Enable the WPA supplicant on the STA (bsscfg 0).
        let mut sw = [0u8; 24];
        sw[..15].copy_from_slice(b"bsscfg:sup_wpa\0"); // 14 chars + null
        sw[19] = 1; // index(15..19)=0, value(19..23)=1
        self.do_ioctl_b(2, 0x107, 61, 0, &sw, 23, fr);

        // Security + auth (all STA iface 0).
        self.do_ioctl(134, 62, 0, &[0x04], 4, fr); // WLC_SET_WSEC = WPA
        self.do_ioctl(20, 63, 0, &[1], 4, fr); // WLC_SET_INFRA = 1
        self.do_ioctl(22, 64, 0, &[0], 4, fr); // WLC_SET_AUTH = open
        self.do_ioctl(165, 65, 0, &[0x80], 4, fr); // WLC_SET_WPA_AUTH = WPA2-PSK

        // Passphrase: wsec_pmk_t = key_len(u16) + flags(u16=1 passphrase) + key[64].
        let mut pmk = [0u32; 17];
        pmk[0] = (pl as u32) | (1u32 << 16);
        for i in 0..pl.min(64) {
            pmk[1 + i / 4] |= (pass[i] as u32) << (8 * (i % 4));
        }
        self.do_ioctl(268, 66, 0, &pmk, 68, fr); // WLC_SET_WSEC_PMK

        // Join: wl_ssid_t = ssid_len(u32) + ssid[32].
        let mut js = [0u32; 9];
        js[0] = sl as u32;
        for i in 0..sl.min(32) {
            js[1 + i / 4] |= (ssid[i] as u32) << (8 * (i % 4));
        }
        self.do_ioctl(26, 67, 0, &js, 36, fr); // WLC_SET_SSID (join)

        // Poll WLC_GET_BSSID: success once associated + stable through the 4-way
        // handshake window (a wrong password associates then deauths -> resets).
        // Mode-switch + scan + assoc can be slow, so allow ~20s.
        let mut ok = 0u32;
        for t in 0..40u32 {
            userlib::hl::sleep_for(500);
            let st = self.do_ioctl_b(0, 23, 68, 0, &[], 6, fr); // WLC_GET_BSSID
            ok = if st == 0 { ok + 1 } else { 0 };
            DIAG[7].store(0x5A00_0000 | (t << 8) | ok, SeqCst);
            if ok >= 4 {
                return 0; // stable association -> connected
            }
        }
        1 // timeout / failed
    }

    /// Bring up an OPEN SoftAP with `ssid` on channel 6 (provisioning portal).
    /// Mirrors cyw43_ll_wifi_ap_init/set_up for the open case. AP-interface
    /// (iface 1) ioctls: mfp, gmode, 2g_mrate, dtim. Returns the bss-up status.
    fn ap_start(&mut self, fr: &mut [u32; 512]) -> u32 {
        // Copy the board SSID to a local so it doesn't alias the &mut self below.
        let mut ssid_buf = [0u8; 32];
        let n = self.ssid_len as usize;
        ssid_buf[..n].copy_from_slice(&self.ssid[..n]);
        let ssid = &ssid_buf[..n];
        // Radio on: country + WLC_UP (as cyw43_wifi_on before ap_init).
        let country = [0x6e75_6f63, 0x0079_7274, 0x0000_5858, 0xFFFF_FFFF, 0x0000_5858];
        self.do_ioctl(0x107, 40, 0, &country, 8 + 12, fr);
        self.do_ioctl_b(2, 2, 41, 0, &[], 0, fr); // WLC_UP
        // ampdu_ba_wsize = 2 (STA).
        let mut abw = [0u8; 20];
        abw[..15].copy_from_slice(b"ampdu_ba_wsize\0");
        abw[15] = 2;
        self.do_ioctl_b(2, 0x107, 42, 0, &abw, 19, fr);
        // bsscfg:ssid = [AP=1, ssid_len, ssid[32]] (AP index carried in payload).
        let mut sb = [0u8; 52];
        sb[..12].copy_from_slice(b"bsscfg:ssid\0");
        sb[12] = 1;
        sb[16] = n as u8;
        sb[20..20 + n].copy_from_slice(&ssid[..n]);
        self.do_ioctl_b(2, 0x107, 43, 0, &sb, 52, fr);
        // Channel 6 (STA).
        self.do_ioctl(30, 44, 0, &[6], 4, fr);
        // bsscfg:wsec = [AP=1, 0=open].
        let mut ws = [0u8; 20];
        ws[..12].copy_from_slice(b"bsscfg:wsec\0");
        ws[12] = 1;
        self.do_ioctl_b(2, 0x107, 45, 0, &ws, 20, fr);
        // mfp = 0, gmode = 1, 2g_mrate = 22, dtim = 1 (all on the AP interface).
        let mut mfp = [0u8; 8];
        mfp[..4].copy_from_slice(b"mfp\0");
        self.do_ioctl_b(2, 0x107, 46, 1, &mfp, 8, fr);
        self.do_ioctl(110, 47, 1, &[1], 4, fr);
        let mut mr = [0u8; 16];
        mr[..9].copy_from_slice(b"2g_mrate\0");
        mr[9] = 22;
        self.do_ioctl_b(2, 0x107, 48, 1, &mr, 13, fr);
        self.do_ioctl(78, 49, 1, &[1], 4, fr);
        // Bring the AP up: bss = [AP=1, up=1].
        let mut bss = [0u8; 12];
        bss[..4].copy_from_slice(b"bss\0");
        bss[4] = 1;
        bss[8] = 1;
        let status = self.do_ioctl_b(2, 0x107, 50, 0, &bss, 12, fr);
        // In apsta mode the AP interface (iface 1) has its OWN MAC; clients send
        // gateway traffic to THAT, not the STA MAC we read at init. Adopt the AP
        // MAC so our gratuitous ARP + smoltcp identity match what the AP receives
        // on (otherwise unicast to .1 is addressed to a MAC the AP ignores).
        let ms = self.do_ioctl_b(0, 0x106, 52, 1, b"cur_etheraddr\0", 14 + 6, fr);
        if ms == 0 {
            let hl = (((fr[1] >> 24) & 0xFF) / 4) as usize;
            let w0 = fr[hl + 4];
            let w1 = fr[hl + 5];
            self.mac = [
                w0 as u8,
                (w0 >> 8) as u8,
                (w0 >> 16) as u8,
                (w0 >> 24) as u8,
                w1 as u8,
                (w1 >> 8) as u8,
            ];
            DIAG[8].store(w0, SeqCst); // AP MAC low word (vs STA MAC for compare)
        }
        // Also read the AP's actual BSSID (WLC_GET_BSSID, cmd 23) on iface 1 --
        // this is the MAC clients associate with + address unicast to. If it
        // differs from cur_etheraddr, that's why unicast-to-host is dropped.
        let bs = self.do_ioctl_b(0, 23, 53, 1, &[], 6, fr);
        if bs == 0 {
            let hl = (((fr[1] >> 24) & 0xFF) / 4) as usize;
            DIAG[3].store(fr[hl + 4], SeqCst); // BSSID low word
        }
        status
    }
}

/// The CLM ioctl, split out so the streaming borrow of `clm_pl` ends first.
fn me_do_clm(me: &mut Cyw43, clm_pl: &[u32; 251], fr: &mut [u32; 512]) -> u32 {
    me.do_ioctl(0x107, 1, 0, clm_pl, 8 + 12 + 984, fr)
}

/// One-time hardware setup: pad config, funcsel, CYW43 power-on (gSPI mode),
/// PIO2 program load + SM configuration. Runs once at task start.
fn setup(p: &rp235x_pac::Peripherals) {
    let sio = &p.SIO;
    for pin in [WL_ON, CS, DIO, CLK] {
        p.PADS_BANK0.gpio(pin as usize).modify(|_, w| {
            w.od().clear_bit();
            w.iso().clear_bit();
            w.ie().set_bit();
            unsafe { w.drive().bits(3) }
        });
        p.IO_BANK0
            .gpio(pin as usize)
            .gpio_ctrl()
            .modify(|_, w| unsafe { w.funcsel().bits(5) }); // SIO
        sio.gpio_oe_set().write(|w| unsafe { w.bits(1 << pin) });
    }
    sio.gpio_out_set().write(|w| unsafe { w.bits(1 << CS) });
    sio.gpio_out_clr()
        .write(|w| unsafe { w.bits((1 << WL_ON) | (1 << DIO) | (1 << CLK)) });
    // Power up: WL_ON low 20 ms (DIO low = gSPI mode), high 250 ms.
    cortex_m::asm::delay(150_000 * 20);
    sio.gpio_out_set().write(|w| unsafe { w.bits(1 << WL_ON) });
    cortex_m::asm::delay(150_000 * 250);
    // gSPI latched: route DIO + CLK to PIO2.
    sio.gpio_oe_clr()
        .write(|w| unsafe { w.bits((1 << DIO) | (1 << CLK)) });
    for pin in [DIO, CLK] {
        p.IO_BANK0
            .gpio(pin as usize)
            .gpio_ctrl()
            .modify(|_, w| unsafe { w.funcsel().bits(8) }); // PIO2
    }
    p.PADS_BANK0.gpio(DIO as usize).modify(|_, w| {
        w.pue().clear_bit();
        w.pde().set_bit();
        w.schmitt().set_bit()
    });
    p.PADS_BANK0
        .gpio(CLK as usize)
        .modify(|_, w| w.pue().clear_bit().pde().set_bit());
    p.PADS_BANK0
        .gpio(WL_ON as usize)
        .modify(|_, w| w.pde().clear_bit().pue().set_bit());

    // PIO2 is brought out of reset by the privileged pre-kernel main (so this
    // task doesn't need the RESETS peripheral -- it's at the MPU region limit).
    let pio = &p.PIO2;
    pio.ctrl().modify(|_, w| unsafe { w.sm_enable().bits(0) });
    for (i, insn) in GSPI.iter().enumerate() {
        pio.instr_mem(i).write(|w| unsafe { w.bits(*insn as u32) });
    }
    pio.input_sync_bypass()
        .write(|w| unsafe { w.bits(1 << DIO) });
    let sm = pio.sm(0);
    sm.sm_clkdiv()
        .write(|w| unsafe { w.int().bits(9).frac().bits(0x60) });
    sm.sm_shiftctrl().modify(|_, w| unsafe {
        w.out_shiftdir().clear_bit();
        w.in_shiftdir().clear_bit();
        w.autopull().set_bit();
        w.autopush().set_bit();
        w.pull_thresh().bits(0);
        w.push_thresh().bits(0)
    });
    sm.sm_pinctrl().modify(|_, w| unsafe {
        w.sideset_count().bits(1);
        w.sideset_base().bits(CLK as u8);
        w.out_base().bits(DIO as u8);
        w.out_count().bits(1);
        w.set_base().bits(DIO as u8);
        w.set_count().bits(1);
        w.in_base().bits(DIO as u8)
    });
    sm.sm_execctrl()
        .modify(|_, w| unsafe { w.wrap_top().bits(5).wrap_bottom().bits(0) });
}

struct ServerImpl {
    wifi: Cyw43,
    fr: [u32; 512],
}

impl idl::InOrderRp235xCyw43Impl for ServerImpl {
    fn wifi_status(
        &mut self,
        _: &RecvMessage,
    ) -> Result<u32, RequestError<Cyw43Error>> {
        Ok(self.wifi.status)
    }
    fn get_mac(
        &mut self,
        _: &RecvMessage,
    ) -> Result<[u8; 6], RequestError<Cyw43Error>> {
        Ok(self.wifi.mac)
    }
    fn led(
        &mut self,
        _: &RecvMessage,
        on: bool,
    ) -> Result<(), RequestError<Cyw43Error>> {
        self.wifi.led(on, &mut self.fr);
        Ok(())
    }
    fn scan(
        &mut self,
        _: &RecvMessage,
    ) -> Result<u32, RequestError<Cyw43Error>> {
        Ok(self.wifi.scan(&mut self.fr))
    }
    fn ap(
        &mut self,
        _: &RecvMessage,
    ) -> Result<u32, RequestError<Cyw43Error>> {
        Ok(self.wifi.ap_start(&mut self.fr))
    }
}

impl idol_runtime::NotificationHandler for ServerImpl {
    fn current_notification_mask(&self) -> u32 {
        0
    }
    fn handle_notification(&mut self, _bits: userlib::NotificationBits) {}
}

#[export_name = "main"]
fn main() -> ! {
    let aux = AuxFlash::from(AUXFLASH.get_task_id());
    let mut fr = [0u32; 512];
    let wifi = match Cyw43::new(&aux, &mut fr) {
        Ok(w) => w,
        Err(_) => {
            // Firmware/init failed: park so the diagnostics stay readable.
            loop {
                cortex_m::asm::nop();
            }
        }
    };
    let mut server = ServerImpl { wifi, fr };
    // Scan nearby networks (STA) before bringing the AP up, to populate the
    // portal's network dropdown.
    server.wifi.scan(&mut server.fr);
    // Provisioning mode: bring the SoftAP up and serve DHCP continuously, so a
    // joining client always gets a lease (192.168.4.2) regardless of when it
    // retries. (Idol serving is suspended while provisioning; the probe stays
    // free for live DIAG reads.)
    server.wifi.ap_start(&mut server.fr);
    let _ = &idl::INCOMING_SIZE;
    // Transport is smoltcp (net::run_portal): it owns ARP/IP/UDP/TCP; we run the
    // DHCP/DNS/HTTP servers on its sockets. Never returns.
    net::run_portal(&mut server.wifi, &mut server.fr)
}

mod idl {
    use drv_rp235x_cyw43_api::Cyw43Error;
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
