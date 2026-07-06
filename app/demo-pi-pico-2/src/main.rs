// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

// Pull in the PAC so its interrupt vector table is linked into the image.
use rp235x_pac as _;

use cortex_m_rt::entry;

mod image_version {
    include!(concat!(env!("OUT_DIR"), "/image_version.rs"));
}

/// Pico 2 onboard LED.
const LED_PIN: u32 = 25;

/// RP2350 boot metadata: IMAGE_DEF block for a RAM ("packaged") image.
///
/// The boot ROM refuses images without an IMAGE_DEF in the first 4 KiB (the
/// linker places `.image_def` right after the vector table + Hubris
/// `ImageHeader`; see `build/kernel-link.x`). This block additionally carries
/// a LOAD_MAP: the ROM copies the whole 256 KiB code window from flash into
/// SRAM before boot (datasheet sec 5.1.10 "packaged binaries"), so nothing
/// ever executes from flash at runtime and the sec 5.4.4 XIP erase/program
/// hazard does not exist on this system.
///
/// Words verified against the datasheet, pico-sdk picobin.h, and the bootrom
/// source (varm_blocks.c); see also docs/rp2350-research/findings.md.
#[link_section = ".image_def"]
#[used]
pub static RP235X_IMAGE_DEF_ARM_RAM: [u32; 13] = [
    0xffff_ded3, // PICOBIN_BLOCK_MARKER_START
    0x1021_0142, // IMAGE_TYPE item: EXE | SECURITY(S) | CPU(Arm) | CHIP(RP2350)
    // VECTOR_TABLE item (type 0x03, 2 words): the runtime vector table is at
    // the image's RAM base, where the LOAD_MAP below puts it.
    0x0000_0203,
    0x2000_0000,
    // LOAD_MAP item (type 0x06, size 4 words, RELATIVE + 1 entry = 0x01).
    // Relative addressing makes the copy source position-independent -- the
    // same binary boots correctly wherever it is stored in flash (flash base,
    // or partition A/B). storage_start is relative to this item's own header
    // address: the block sits at 0x10000160, the item is its 5th word (0x10),
    // so the item header is at 0x10000170 and 0x10000000 - 0x10000170 =
    // 0xffff_fe90. The relative form's third word is a byte size, not an end
    // address (contrast the absolute form; see bootrom varm_blocks.c).
    0x0100_0406,
    0xffff_fe90, // entry 0: storage start, relative to the load-map item
    0x2000_0000, // entry 0: runtime start (SRAM, absolute)
    // Copy size (128 KiB). MUST be strictly less than the enclosing
    // partition/window size: the bootrom rejects the block if
    // from_storage + size >= window_end (varm_blocks.c). Our A/B partitions
    // are 256 KiB, so copying the full 256 KiB fails the check -- it only
    // worked as a single image because the window was the whole 4 MB flash.
    // 128 KiB comfortably covers the ~60 KiB image with room to grow while
    // staying under the 256 KiB partition.
    0x0002_0000, // entry 0: size in bytes (128 KiB)
    // VERSION item (type 0x48, 2 words, no rollback rows): the boot ROM uses
    // this to choose between A/B partitions -- the higher version boots. The
    // second word ((major << 16) | minor) is stamped by build.rs from
    // HUBRIS_IMAGE_VERSION. Placed AFTER the load map so the load-map item's
    // address (and thus its relative storage offset) is unchanged.
    0x0000_0248,
    image_version::IMAGE_VERSION_WORD,

    0x0000_09ff, // BLOCK_ITEM_LAST, size = 9 words of items
    0x0000_0000, // link = self (single-block loop)
    0xab12_3579, // PICOBIN_BLOCK_MARKER_END
];
/// Watchdog scratch0 marker: "this reboot wants to land in BOOTSEL". Written
/// by the flash driver's `reboot(bootsel)`; scratch registers survive a
/// watchdog reset. Scratch 4-7 are reserved for the ROM's vectored-boot
/// protocol; 0 is free for the application.
const BOOTSEL_MAGIC: u32 = 0xb007_5e1f;

// --- PIO plumbing proof (scope P1: first PIO use on the port) -----------------
//
// Before the gSPI PHY, prove the RP2350 PIO works at all under our port: load a
// tiny 3-instruction "echo" program (TXF -> OSR -> ISR -> RXF) into PIO0 SM0 and
// confirm a value written to the TX FIFO comes back from the RX FIFO. No pins,
// no clock rate dependence -- this isolates the plumbing (reset, instruction
// memory load, SM config, FIFO access) from the gSPI complexity. Result in the
// probe-readable PIO_PROBE static (nm + `probe-rs read`).
#[no_mangle]
#[used]
static PIO_PROBE: [core::sync::atomic::AtomicU32; 2] = [
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
];

fn pio_echo_test(p: &rp235x_pac::Peripherals) {
    use core::sync::atomic::Ordering::SeqCst;

    // Bring PIO0 out of reset (privileged, pre-kernel).
    p.RESETS.reset().modify(|_, w| w.pio0().clear_bit());
    while p.RESETS.reset_done().read().pio0().bit_is_clear() {}

    let pio = &p.PIO0;
    // Ensure SM0 is stopped before we reconfigure it.
    pio.ctrl().modify(|_, w| unsafe { w.sm_enable().bits(0) });

    // Echo program (hand-assembled; each is one 16-bit PIO instruction):
    //   0: pull block     (100 00000 1 0 1 00000 = 0x80A0)  OSR <- TXF
    //   1: mov isr, osr   (101 00000 110 00 111 = 0xA0C7)  ISR <- OSR
    //   2: push block     (100 00000 0 0 1 00000 = 0x8020)  RXF <- ISR
    // then wrap 2 -> 0.
    const PROG: [u16; 3] = [0x80A0, 0xA0C7, 0x8020];
    for (i, insn) in PROG.iter().enumerate() {
        pio.instr_mem(i)
            .write(|w| unsafe { w.bits(*insn as u32) });
    }

    let sm = pio.sm(0);
    // Wrap after instr 2 back to instr 0; explicit pull/push (no autopull/push).
    sm.sm_execctrl()
        .modify(|_, w| unsafe { w.wrap_top().bits(2).wrap_bottom().bits(0) });
    sm.sm_shiftctrl()
        .modify(|_, w| w.autopull().clear_bit().autopush().clear_bit());
    // Force PC to 0 (execute `jmp 0` = 0x0000 immediately).
    sm.sm_instr().write(|w| unsafe { w.sm0_instr().bits(0x0000) });

    // Enable SM0.
    pio.ctrl().modify(|_, w| unsafe { w.sm_enable().bits(1) });

    // Push a test value; expect the same value back from the RX FIFO.
    const TEST: u32 = 0xc0de_1234;
    pio.txf(0).write(|w| unsafe { w.bits(TEST) });
    let mut spins = 0u32;
    while pio.fstat().read().rxempty().bits() & 1 != 0 {
        spins += 1;
        if spins > 1_000_000 {
            PIO_PROBE[1].store(0xdead_0000, SeqCst); // timed out: RXF stayed empty
            return;
        }
    }
    PIO_PROBE[0].store(pio.rxf(0).read().bits(), SeqCst); // want 0xc0de_1234
    PIO_PROBE[1].store(0x5010_0000 | (spins & 0xffff), SeqCst); // ran + spin count
}
// -----------------------------------------------------------------------------

// Pico 2 W CYW43439 gSPI pins (internal RP2350 GPIOs).
const WL_ON: u32 = 23; // WL_REG_ON (power/reset)
const DIO: u32 = 24; // gSPI DIO (half-duplex data)
const CS: u32 = 25; // gSPI CS
const CLK: u32 = 29; // gSPI CLK

// --- Loopback validation: clock the gSPI command out on the HEADER SPI pins ---
// (GP17=CS, GP18=CLK, GP19=DIO) which are wired to the peer board's SPI, so the
// peer (as an SPI peripheral) captures what our PIO gSPI actually produces --
// validating the PIO output independently of the CYW43. Expect the peer to
// receive the command bytes A0 04 40 00 (0xA004_4000 MSB-first).
fn pio_output_test(p: &rp235x_pac::Peripherals) {
    const TCS: u32 = 17;
    const TCLK: u32 = 18;
    const TDIO: u32 = 19;
    let sio = &p.SIO;
    p.PADS_BANK0
        .gpio(TCS as usize)
        .modify(|_, w| w.od().clear_bit().iso().clear_bit().ie().set_bit());
    p.IO_BANK0
        .gpio(TCS as usize)
        .gpio_ctrl()
        .modify(|_, w| unsafe { w.funcsel().bits(5) });
    sio.gpio_oe_set().write(|w| unsafe { w.bits(1 << TCS) });
    sio.gpio_out_set().write(|w| unsafe { w.bits(1 << TCS) });
    for pin in [TCLK, TDIO] {
        p.PADS_BANK0.gpio(pin as usize).modify(|_, w| {
            w.od().clear_bit();
            w.iso().clear_bit();
            w.ie().set_bit()
        });
        p.IO_BANK0
            .gpio(pin as usize)
            .gpio_ctrl()
            .modify(|_, w| unsafe { w.funcsel().bits(6) });
    }
    let pio = &p.PIO0;
    pio.ctrl().modify(|_, w| unsafe { w.sm_enable().bits(0) });
    // Write-only: 0:out pins,1 side0  1:jmp x-- 0 side1. wrap 1 -> 0.
    for (i, insn) in [0x6001u16, 0x1020].iter().enumerate() {
        pio.instr_mem(i).write(|w| unsafe { w.bits(*insn as u32) });
    }
    let sm = pio.sm(0);
    sm.sm_clkdiv()
        .write(|w| unsafe { w.int().bits(600).frac().bits(0) }); // ~125 kHz
    sm.sm_shiftctrl().modify(|_, w| unsafe {
        w.out_shiftdir().clear_bit();
        w.autopull().set_bit();
        w.pull_thresh().bits(0)
    });
    sm.sm_pinctrl().modify(|_, w| unsafe {
        w.sideset_count().bits(1);
        w.sideset_base().bits(TCLK as u8);
        w.out_base().bits(TDIO as u8);
        w.out_count().bits(1);
        w.set_base().bits(TDIO as u8);
        w.set_count().bits(1)
    });
    sm.sm_execctrl()
        .modify(|_, w| unsafe { w.wrap_top().bits(1).wrap_bottom().bits(0) });
    let cmd = ((1u32 << 30) | (0x14 << 11) | 4).rotate_left(16); // 0xA004_4000
    sio.gpio_out_clr().write(|w| unsafe { w.bits(1 << TCS) }); // CS low
    cortex_m::asm::delay(15_000);
    let set_pin = |pin: u32, insn: u16| {
        sm.sm_pinctrl()
            .modify(|_, w| unsafe { w.set_base().bits(pin as u8) });
        sm.sm_instr().write(|w| unsafe { w.sm0_instr().bits(insn) });
    };
    set_pin(TCLK, 0xE081);
    set_pin(TCLK, 0xE000);
    set_pin(TDIO, 0xE081);
    set_pin(TDIO, 0xE000);
    pio.ctrl()
        .modify(|_, w| unsafe { w.sm_restart().bits(1).clkdiv_restart().bits(1) });
    pio.txf(0).write(|w| unsafe { w.bits(31) });
    sm.sm_instr().write(|w| unsafe { w.sm0_instr().bits(0x6020) }); // out x,32
    sm.sm_instr().write(|w| unsafe { w.sm0_instr().bits(0x0000) }); // jmp 0
    pio.ctrl().modify(|_, w| unsafe { w.sm_enable().bits(1) });
    pio.txf(0).write(|w| unsafe { w.bits(cmd) });
    cortex_m::asm::delay(150_000 * 3); // ~3 ms to clock 32 bits at 125 kHz
    sio.gpio_out_set().write(|w| unsafe { w.bits(1 << TCS) }); // CS high
}
// -----------------------------------------------------------------------------

// --- P2/P3: CYW43439 gSPI chip-detect over PIO --------------------------------
//
// The bit-bang could not clock the half-duplex gSPI; this drives it with a PIO
// state machine (deterministic timing), the approach embassy/pico-sdk use. A
// 7-instruction program (hand-assembled) clocks out a 32-bit command MSB-first
// on DIO (GP24), flips the pin to input, and clocks in the 32-bit response;
// CLK is side-set on GP29, CS (GP25) + WL_ON (GP23) are CPU-driven (SIO).
// Result (raw wire response) in CYW43_PIO -- expect swap16(FEEDBEAD)=0xBEADFEED.
#[no_mangle]
#[used]
static CYW43_PIO: [core::sync::atomic::AtomicU32; 16] = [
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
    core::sync::atomic::AtomicU32::new(0),
];

// Pico W CYW43439 NVRAM (config vars), from cyw43-driver wifi_nvram_43439.h,
// packed little-endian and zero-padded to a word. Written near the top of WLAN
// RAM during firmware download; the firmware reads it to find its calibration.
static NVRAM: [u32; 186] = [
    0x4152564e, 0x7665524d, 0x6552243d, 0x6d002476, 0x69666e61, 0x78303d64, 0x00306432, 0x646f7270,
    0x303d6469, 0x32373078, 0x65760037, 0x6469646e, 0x3178303d, 0x00346534, 0x69766564, 0x78303d64,
    0x32653334, 0x616f6200, 0x79746472, 0x303d6570, 0x38383078, 0x6f620037, 0x72647261, 0x303d7665,
    0x30313178, 0x6f620030, 0x6e647261, 0x323d6d75, 0x616d0032, 0x64646163, 0x30303d72, 0x3a30413a,
    0x623a3035, 0x39353a35, 0x0065353a, 0x6d6f7273, 0x3d766572, 0x62003131, 0x6472616f, 0x67616c66,
    0x78303d73, 0x30343030, 0x31303034, 0x616f6200, 0x6c666472, 0x33736761, 0x3078303d, 0x30303034,
    0x00303030, 0x6c617478, 0x71657266, 0x3437333d, 0x6e003030, 0x6372636f, 0x6100313d, 0x323d3067,
    0x61003535, 0x3d673261, 0x63630031, 0x3d65646f, 0x004c4c41, 0x69306170, 0x69737374, 0x78303d74,
    0x65003032, 0x61707478, 0x6e696167, 0x303d6732, 0x32617000, 0x3d306167, 0x3836312d, 0x3631372c,
    0x382d2c31, 0x41003032, 0x696d5676, 0x30635f64, 0x3078303d, 0x6378302c, 0x63630038, 0x7277706b,
    0x7366666f, 0x3d307465, 0x616d0035, 0x67327078, 0x383d3061, 0x78740034, 0x62727770, 0x666f6b63,
    0x6300363d, 0x77626b63, 0x67323032, 0x303d6f70, 0x67656c00, 0x6d64666f, 0x30327762, 0x6f706732,
    0x3678303d, 0x31313136, 0x00313131, 0x6273636d, 0x32303277, 0x3d6f7067, 0x37377830, 0x31313137,
    0x70003131, 0x62706f72, 0x32303277, 0x3d6f7067, 0x64647830, 0x64666f00, 0x6769646d, 0x746c6966,
    0x65707974, 0x0038313d, 0x6d64666f, 0x66676964, 0x74746c69, 0x62657079, 0x38313d65, 0x70617000,
    0x646f6d64, 0x00313d65, 0x64706170, 0x696c6176, 0x73657464, 0x00313d74, 0x61636170, 0x7864696c,
    0x343d6732, 0x61700035, 0x70656470, 0x66666f73, 0x3d746573, 0x0030332d, 0x64706170, 0x69646e65,
    0x353d7864, 0x746c0038, 0x6d786365, 0x303d7875, 0x65746c00, 0x61707863, 0x6d756e64, 0x3078303d,
    0x00323031, 0x6365746c, 0x736e6678, 0x303d6c65, 0x00343478, 0x6365746c, 0x69636778, 0x6f697067,
    0x3078303d, 0x6c690031, 0x63616d30, 0x72646461, 0x3a30303d, 0x343a3039, 0x35633a63, 0x3a32313a,
    0x77003833, 0x6469306c, 0x3478303d, 0x00623133, 0x64616564, 0x5f6e616d, 0x303d6f74, 0x66666678,
    0x66666666, 0x756d0066, 0x616e6578, 0x78303d62, 0x00303031, 0x72757073, 0x666e6f63, 0x303d6769,
    0x67003378, 0x6374696c, 0x61625f68, 0x5f646573, 0x6d737263, 0x313d6e69, 0x63746200, 0x646f6d5f,
    0x00313d65, 0x00000000,
];

fn cyw43_pio_detect(p: &rp235x_pac::Peripherals) {
    use core::sync::atomic::Ordering::SeqCst;
    let sio = &p.SIO;

    // Start ALL four pins as SIO. Crucially DIO (GP24) must be driven LOW while
    // WL_ON rises -- that is the gSPI (vs SDIO) mode-select, latched at power-up.
    // We only hand DIO + CLK to the PIO AFTER the chip is powered and has latched
    // gSPI mode.
    for pin in [WL_ON, CS, DIO, CLK] {
        p.PADS_BANK0.gpio(pin as usize).modify(|_, w| {
            w.od().clear_bit();
            w.iso().clear_bit();
            w.ie().set_bit();
            unsafe { w.drive().bits(3) } // 12 mA (embassy uses 12 mA)
        });
        p.IO_BANK0
            .gpio(pin as usize)
            .gpio_ctrl()
            .modify(|_, w| unsafe { w.funcsel().bits(5) }); // SIO
        sio.gpio_oe_set().write(|w| unsafe { w.bits(1 << pin) });
    }
    // Localize when CS control is lost: read CS-low BEFORE and AFTER powering the
    // CYW43 (WL_ON high). rd_cs drives GP25 low, settles, reads it back.
    let rd_cs = || {
        sio.gpio_out_clr().write(|w| unsafe { w.bits(1 << CS) });
        cortex_m::asm::delay(15_000);
        let v = (sio.gpio_in().read().bits() >> CS) & 1;
        sio.gpio_out_set().write(|w| unsafe { w.bits(1 << CS) });
        v
    };
    let cs_pre = rd_cs(); // before power-up: expect 0 (controllable)

    sio.gpio_out_set().write(|w| unsafe { w.bits(1 << CS) }); // CS idle high
    sio.gpio_out_clr()
        .write(|w| unsafe { w.bits((1 << WL_ON) | (1 << DIO) | (1 << CLK)) });

    // Power up: WL_ON low 20 ms (DIO held low = gSPI mode), high 250 ms.
    cortex_m::asm::delay(150_000 * 20);
    sio.gpio_out_set().write(|w| unsafe { w.bits(1 << WL_ON) });
    cortex_m::asm::delay(150_000 * 250);

    let cs_post = rd_cs(); // after power-up: expect 0

    // gSPI mode latched -- release the SIO output drivers on DIO + CLK and route
    // them to PIO0 (funcsel 6) so only the PIO drives them.
    sio.gpio_oe_clr()
        .write(|w| unsafe { w.bits((1 << DIO) | (1 << CLK)) });
    for pin in [DIO, CLK] {
        p.IO_BANK0
            .gpio(pin as usize)
            .gpio_ctrl()
            .modify(|_, w| unsafe { w.funcsel().bits(8) }); // PIO2 (as MicroPython)
    }
    // Match pico-sdk pads: DIO pull-DOWN + schmitt/hysteresis, CLK pull-DOWN,
    // WL_ON pull-UP.
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

    // Load the gSPI read program into PIO0 (overwriting the P1 echo program).
    // This is embassy's LOW-SPEED variant (< 75 MHz PIO clock), which is the one
    // that matches our ~1 MHz clock -- the turnaround (single `nop side 0`)
    // determines exactly when the read samples relative to the response.
    // .side_set 1 (CLK): 0:out pins,1 side0  1:jmp x-- 0 side1  2:set pindirs,0
    // side0  3:nop side0  4(lp2):in pins,1 side1  5:jmp y-- 4 side0.
    // Assembled bytes verified against MicroPython's rp2 assembler (my earlier
    // hand-assembly had 3 wrong encodings: jmp x-- was !X, `in` had the side-set
    // bit misplaced, jmp y-- was x--). 0:out pins,1 side0  1:jmp x-- 0 side1
    // 2:set pindirs,0 side0  3:nop side0  4:in pins,1 side1  5:jmp y-- 4 side0.
    const GSPI: [u16; 6] = [0x6001, 0x1040, 0xE080, 0xA042, 0x5001, 0x0084];
    // Use PIO2 (like MicroPython/pico-sdk), not PIO0 -- the last unmatched
    // variable. Bring it out of reset first.
    p.RESETS.reset().modify(|_, w| w.pio2().clear_bit());
    while p.RESETS.reset_done().read().pio2().bit_is_clear() {}
    let pio = &p.PIO2;
    pio.ctrl().modify(|_, w| unsafe { w.sm_enable().bits(0) });
    for (i, insn) in GSPI.iter().enumerate() {
        pio.instr_mem(i).write(|w| unsafe { w.bits(*insn as u32) });
    }
    // Bypass the 2-flop input synchronizer on DIO (GP24), as embassy does.
    pio.input_sync_bypass()
        .write(|w| unsafe { w.bits(1 << DIO) });
    let sm = pio.sm(0);
    // The CYW43 gSPI has an effective MINIMUM clock (~2 MHz SDIO) -- verified on
    // the live chip via MicroPython. Run PIO at 150/9 = ~16.7 MHz -> ~8.3 MHz
    // SDIO (well inside the known-good 4-32 MHz range).
    sm.sm_clkdiv()
        .write(|w| unsafe { w.int().bits(9).frac().bits(0x60) }); // 16 MHz (rp2)
    // MSB-first (shift left), autopull/autopush at 32 bits (thresh 0 == 32).
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

    let _ = (cs_pre, cs_post); // (earlier CS-control diagnostic; unused now)
    // gSPI command word [wr|incr|func:2|addr:17|len:11]; the chip powers up in a
    // 16-bit-swapped mode, so pre-config accesses are swap16'd (rotate 16).
    let cmd_word = |wr: bool, func: u32, addr: u32, len: u32| -> u32 {
        ((wr as u32) << 31) | (1 << 30) | (func << 28) | (addr << 11) | len
    };
    let swap16 = |x: u32| x.rotate_left(16);
    let set_pin = |pin: u32, insn: u16| {
        sm.sm_pinctrl()
            .modify(|_, w| unsafe { w.set_base().bits(pin as u8) });
        sm.sm_instr().write(|w| unsafe { w.sm0_instr().bits(insn) });
    };
    // One gSPI transaction: clean the SM (clear FIFOs, restart, empty the OSR so
    // the X/Y autopulls work), clock out `out_words` (32 bits each), turn DIO
    // around, clock in `in_words` (32 bits each) filling the slice. The X (write)
    // and Y (read) bit counts are derived from the slice lengths.
    let xfer = |out_words: &[u32], in_words: &mut [u32]| {
        let x_bits = out_words.len() as u32 * 32 - 1;
        let y_bits = in_words.len() as u32 * 32 - 1;
        sio.gpio_out_set().write(|w| unsafe { w.bits(1 << CS) }); // CS high (pulse)
        cortex_m::asm::delay(1500);
        sio.gpio_out_clr().write(|w| unsafe { w.bits(1 << CS) }); // CS low
        cortex_m::asm::delay(30_000); // ~200 us settle
        pio.ctrl().modify(|_, w| unsafe { w.sm_enable().bits(0) });
        sm.sm_shiftctrl().modify(|_, w| w.fjoin_rx().set_bit());
        sm.sm_shiftctrl().modify(|_, w| w.fjoin_rx().clear_bit());
        set_pin(DIO, 0xE081); // DIO pindir out (the turnaround left it input)
        pio.ctrl()
            .modify(|_, w| unsafe { w.sm_restart().bits(1).clkdiv_restart().bits(1) });
        pio.txf(0).write(|w| unsafe { w.bits(x_bits) });
        sm.sm_instr().write(|w| unsafe { w.sm0_instr().bits(0x6020) }); // out x,32
        pio.txf(0).write(|w| unsafe { w.bits(y_bits) });
        sm.sm_instr().write(|w| unsafe { w.sm0_instr().bits(0x6040) }); // out y,32
        sm.sm_instr().write(|w| unsafe { w.sm0_instr().bits(0x0000) }); // jmp 0
        pio.ctrl().modify(|_, w| unsafe { w.sm_enable().bits(1) });
        for &word in out_words {
            // Pace to the SM via the TX-FIFO level (FLEVEL bits [3:0] = SM0 TX
            // count): push only when the 4-deep FIFO has room. Without this a
            // burst overruns the FIFO and words are silently dropped. Bounded so a
            // stalled SM can't hang the CPU.
            let mut s = 0u32;
            while (pio.flevel().read().bits() & 0xF) >= 4 && s < 100_000 {
                s += 1;
            }
            pio.txf(0).write(|w| unsafe { w.bits(word) });
        }
        for slot in in_words.iter_mut() {
            // A valid word arrives within ~2 us; fail fast if it doesn't so a
            // short/absent frame doesn't waste ~1 ms per missing word.
            let mut spins = 0u32;
            while pio.fstat().read().rxempty().bits() & 1 != 0 {
                spins += 1;
                if spins > 8_000 {
                    break;
                }
            }
            *slot = pio.rxf(0).read().bits();
        }
        sio.gpio_out_set().write(|w| unsafe { w.bits(1 << CS) }); // CS high
    };
    // CLK + DIO output-low once (cyw43_spi_init).
    set_pin(CLK, 0xE081);
    set_pin(CLK, 0xE000);
    set_pin(DIO, 0xE081);
    set_pin(DIO, 0xE000);

    // 1. Prime + chip-detect: read F0 TEST_RO (0x14) swap16'd until FEEDBEAD -- the
    //    first transaction after power-up is garbage, the chip locks on from 2nd.
    let read_ro = swap16(cmd_word(false, 0, 0x14, 4));
    let mut pass = 0u32;
    let chip = loop {
        let mut b = [0u32; 1];
        xfer(&[read_ro], &mut b);
        pass += 1;
        let v = swap16(b[0]);
        if v == 0xFEED_BEAD || pass >= 32 {
            break v;
        }
    };
    // 2. WRITE path: write F0 TEST_RW (0x18) = 0x12345678, read it back.
    xfer(
        &[swap16(cmd_word(true, 0, 0x18, 4)), swap16(0x1234_5678)],
        &mut [0u32; 1],
    );
    let mut b = [0u32; 1];
    xfer(&[swap16(cmd_word(false, 0, 0x18, 4))], &mut b);
    let rw = swap16(b[0]);
    // 3. Configure REG_BUS_CTRL (0x00): 32-bit words | high-speed | int-pol-high |
    //    wake | resp-delay 0x4 | status-enable | intr-with-status. After this the
    //    gSPI is 32-bit little-endian -- subsequent access is NON-swapped.
    let bus_ctrl: u32 = 0x1 | 0x10 | 0x20 | 0x80 | (0x4 << 8) | ((0x1 | 0x2) << 16);
    xfer(
        &[swap16(cmd_word(true, 0, 0x00, 4)), swap16(bus_ctrl)],
        &mut [0u32; 1],
    );
    // 4. Set the F1 (backplane) read response delay to 4 bytes = 1 padding word,
    //    so backplane reads return [padding, data] and we take the 2nd word.
    xfer(&[cmd_word(true, 0, 0x1d, 1), 4], &mut [0u32; 1]); // write8 F0 SPI_RESP_DELAY_F1
    // 5. ALP clock init (embassy init(), SPI path): request ALP, set the F2
    //    watermark to 0x10, wait for ALP_AVAIL (0x40), then CLEAR CHIP_CLOCK_CSR
    //    back to 0 -- releasing the request so the firmware can manage the clock
    //    and bring up HT itself. Leaving the request held blocks the HT clock.
    xfer(&[cmd_word(true, 1, 0x1000E, 1), 0x08], &mut [0u32; 1]); // ALP_AVAIL_REQ
    xfer(&[cmd_word(true, 1, 0x1_0008, 1), 0x10], &mut [0u32; 1]); // FUNCTION2_WATERMARK = 0x10
    let mut aspin = 0u32;
    let alp = loop {
        let mut r = [0u32; 2]; // F1 read returns [padding, data]
        xfer(&[cmd_word(false, 1, 0x1000E, 1)], &mut r);
        aspin += 1;
        if (r[1] & 0x40) != 0 || aspin >= 2000 {
            break r[1];
        }
    };
    xfer(&[cmd_word(true, 1, 0x1000E, 1), 0], &mut [0u32; 1]); // CHIP_CLOCK_CSR = 0 (release)
    CYW43_PIO[11].store(alp, SeqCst); // ALP_AVAIL before release

    // 6. Windowed backplane read: point the 32 KiB window at CHIPCOMMON_BASE
    //    (0x18000000) via the SBADDR registers, then read the chip-ID register
    //    (window offset 0, with the 32-bit-access flag 0x8000). This exercises the
    //    same windowed path the firmware upload uses to reach the WLAN core RAM.
    let win: u32 = 0x1800_0000;
    xfer(&[cmd_word(true, 1, 0x1000C, 1), (win >> 24) & 0xff], &mut [0u32; 1]); // HIGH
    xfer(&[cmd_word(true, 1, 0x1000B, 1), (win >> 16) & 0xff], &mut [0u32; 1]); // MID
    xfer(&[cmd_word(true, 1, 0x1000A, 1), (win >> 8) & 0xff], &mut [0u32; 1]); // LOW
    let mut cid = [0u32; 2];
    xfer(&[cmd_word(false, 1, 0x8000, 4)], &mut cid);
    let chip_id = cid[1];

    // Windowed backplane byte/word access: set the 32 KiB window via SBADDR, then
    // access the in-window offset (32-bit access sets flag 0x8000). Used for the
    // AI-wrapper core-reset registers and the WLAN-core RAM (firmware upload).
    let bp_set_window = |addr: u32| {
        let base = addr & !0x7FFF;
        xfer(&[cmd_word(true, 1, 0x1000C, 1), (base >> 24) & 0xff], &mut [0u32; 1]);
        xfer(&[cmd_word(true, 1, 0x1000B, 1), (base >> 16) & 0xff], &mut [0u32; 1]);
        xfer(&[cmd_word(true, 1, 0x1000A, 1), (base >> 8) & 0xff], &mut [0u32; 1]);
    };
    let bp_read8 = |addr: u32| -> u32 {
        bp_set_window(addr);
        let mut r = [0u32; 2];
        xfer(&[cmd_word(false, 1, addr & 0x7FFF, 1)], &mut r);
        r[1]
    };
    let bp_write8 = |addr: u32, val: u32| {
        bp_set_window(addr);
        xfer(&[cmd_word(true, 1, addr & 0x7FFF, 1), val], &mut [0u32; 1]);
    };
    let bp_read32 = |addr: u32| -> u32 {
        bp_set_window(addr);
        let mut r = [0u32; 2];
        xfer(&[cmd_word(false, 1, (addr & 0x7FFF) | 0x8000, 4)], &mut r);
        r[1]
    };
    let bp_write32 = |addr: u32, val: u32| {
        bp_set_window(addr);
        xfer(&[cmd_word(true, 1, (addr & 0x7FFF) | 0x8000, 4), val], &mut [0u32; 1]);
    };
    // AI-wrapper core reset (embassy chip.rs). WLAN wrapper 0x18103000, SOCSRAM
    // wrapper 0x18104000; IOCTRL 0x408 (FGC 0x2, CLOCK_EN 0x1), RESETCTRL 0x800
    // (RESET 0x1).
    const IOCTRL: u32 = 0x408;
    const RESETCTRL: u32 = 0x800;
    let disable_core = |base: u32| {
        if bp_read8(base + RESETCTRL) & 0x1 != 0 {
            return;
        }
        bp_write8(base + IOCTRL, 0);
        let _ = bp_read8(base + IOCTRL);
        bp_write8(base + RESETCTRL, 0x1);
        let _ = bp_read8(base + RESETCTRL);
    };
    let reset_core_up = |base: u32| {
        // Core is already disabled (in reset) by the caller. Bring it up like
        // embassy's reset_device_core, with 1 ms settle delays (~150k cycles).
        bp_write8(base + IOCTRL, 0x2 | 0x1); // FGC | CLOCK_EN
        let _ = bp_read8(base + IOCTRL);
        bp_write8(base + RESETCTRL, 0); // out of reset -> CPU starts fetching
        cortex_m::asm::delay(200_000); // ~1.3 ms
        bp_write8(base + IOCTRL, 0x1); // CLOCK_EN (drop FGC)
        let _ = bp_read8(base + IOCTRL);
        cortex_m::asm::delay(200_000); // ~1.3 ms
    };
    const WLAN_WRAP: u32 = 0x1810_3000;
    const SOCSRAM_WRAP: u32 = 0x1810_4000;
    const SOCSRAM_BASE: u32 = 0x1800_4000;
    // Prep for firmware download: park WLAN + SOCSRAM cores, bring SOCSRAM back up,
    // run the 43439 SOCSRAM init, then prove we can read/write the WLAN core RAM.
    disable_core(WLAN_WRAP);
    disable_core(SOCSRAM_WRAP);
    reset_core_up(SOCSRAM_WRAP);
    bp_write32(SOCSRAM_BASE + 0x10, 3);
    bp_write32(SOCSRAM_BASE + 0x44, 0);
    // Stream `words` u32s from `src` into backplane `dest`, in <=64-word bursts
    // that never cross the 32 KiB window (matches embassy's bp_write).
    let bp_stream = |dest: u32, src: *const u32, words: usize| {
        let mut burst = [0u32; 17];
        let mut i = 0usize;
        while i < words {
            let addr = dest + (i * 4) as u32;
            let window_rem = (0x8000 - (addr & 0x7FFF)) as usize;
            // CYW43_BUS_MAX_BLOCK_SIZE for SPI is 64 bytes = 16 words per write.
            let n = (window_rem / 4).min(16).min(words - i);
            bp_set_window(addr);
            burst[0] = cmd_word(true, 1, (addr & 0x7FFF) | 0x8000, (n * 4) as u32);
            for k in 0..n {
                burst[1 + k] = unsafe { core::ptr::read_volatile(src.add(i + k)) };
            }
            xfer(&burst[..1 + n], &mut [0u32; 1]);
            i += n;
        }
    };
    // FIRMWARE DOWNLOAD: WIFI blob (auxflash mirror body at 0x1c200048, 231077 B)
    // into WLAN-core RAM at backplane addr 0.
    let fw_src = 0x1c20_0048 as *const u32;
    let fw_len: usize = 231077;
    let fw_words = fw_len.div_ceil(4);
    bp_stream(0, fw_src, fw_words);
    // Verify the first 64 firmware words with SINGLE reads only (proven reliable
    // via chip-id/3-point) -- no burst read, to isolate write vs burst-read bugs.
    let mut first_bad = 0xFFFF_FFFFu32;
    let mut bad_ram = 0u32;
    let mut bad_src = 0u32;
    for i in 0..64usize {
        let v = bp_read32((i * 4) as u32);
        let s = unsafe { core::ptr::read_volatile(fw_src.add(i)) };
        if v != s {
            first_bad = i as u32;
            bad_ram = v;
            bad_src = s;
            break;
        }
    }
    CYW43_PIO[12].store(first_bad, SeqCst); // first bad word (single-read), 0xFFFFFFFF=ok
    CYW43_PIO[13].store(bad_ram, SeqCst);
    CYW43_PIO[14].store(bad_src, SeqCst);
    // NVRAM near the top of RAM, then the length-magic word at RAM_SIZE-4 (the
    // firmware needs this to locate the NVRAM, or F2 IORDY never asserts).
    const RAM_SIZE: u32 = 0x8_0000;
    let nvram_len = (NVRAM.len() * 4) as u32;
    let nvram_addr = RAM_SIZE - 4 - nvram_len;
    bp_stream(nvram_addr, NVRAM.as_ptr(), NVRAM.len());
    let nvram_words = nvram_len / 4;
    let magic = ((!nvram_words & 0xFFFF) << 16) | (nvram_words & 0xFFFF);
    bp_write32(RAM_SIZE - 4, magic);
    // Verify: firmware at 3 offsets + NVRAM magic readback.
    let ok0 = bp_read32(0) == unsafe { core::ptr::read_volatile(fw_src) };
    let ok1 = bp_read32(0x8000) == unsafe { core::ptr::read_volatile(fw_src.add(0x8000 / 4)) };
    let ok2 = bp_read32(0x3_8000) == unsafe { core::ptr::read_volatile(fw_src.add(0x3_8000 / 4)) };
    let magic_ok = bp_read32(RAM_SIZE - 4) == magic;
    // Bring the WLAN core out of reset -- the firmware starts executing.
    reset_core_up(WLAN_WRAP);
    cortex_m::asm::delay(1_500_000); // ~10 ms for the core to spin up
    // Check the core is up (embassy check_device_core_is_up): IOCTRL has CLOCK_EN
    // and not FGC; RESETCTRL RESET clear.
    let io = bp_read8(WLAN_WRAP + IOCTRL) & 0xff;
    let rc = bp_read8(WLAN_WRAP + RESETCTRL) & 0xff;
    let core_up = (io & 0x3) == 0x1 && (rc & 0x1) == 0;

    // init_cyw43439 (post-download): clear the backplane pull-ups, request the HT
    // clock, and wait for HT_AVAIL (CHIP_CLOCK_CSR & 0x80). The now-running
    // firmware locks the PLL in response.
    cortex_m::asm::delay(3_000_000); // ~20 ms for the firmware to spin up
    xfer(&[cmd_word(true, 1, 0x1000F, 1), 0], &mut [0u32; 1]); // PULL_UP = 0
    xfer(&[cmd_word(true, 1, 0x1000E, 1), 0x10], &mut [0u32; 1]); // HT_AVAIL_REQ
    let mut htspin = 0u32;
    let ht = loop {
        let mut r = [0u32; 2];
        xfer(&[cmd_word(false, 1, 0x1000E, 1)], &mut r);
        htspin += 1;
        if (r[1] & 0x80) != 0 || htspin >= 12000 {
            break r[1];
        }
    };
    // Lower the F2 watermark and enable the F2-packet interrupt.
    xfer(&[cmd_word(true, 1, 0x1_0008, 1), 0x20], &mut [0u32; 1]); // FUNCTION2_WATERMARK
    xfer(&[cmd_word(true, 0, 0x06, 2), 0x0020], &mut [0u32; 1]); // BUS_INTERRUPT_ENABLE = F2
    // Poll REG_BUS_STATUS (F0 0x8) for STATUS_F2_RX_READY (0x20).
    let mut f2spin = 0u32;
    let f2 = loop {
        let mut r = [0u32; 1];
        xfer(&[cmd_word(false, 0, 0x8, 4)], &mut r);
        f2spin += 1;
        if (r[0] & 0x20) != 0 || f2spin >= 8000 {
            break r[0];
        }
    };
    // Turn on the onboard LED (WL_GPIO0) with a SET_VAR "gpioout" ioctl over F2.
    // Frame = SdpcmHeader(12) + CdcHeader(16) + "gpioout\0"(8) + mask(4) + val(4)
    // = 44 bytes. wlan_write prepends the gSPI cmd (WRITE INC F2 addr0 len44).
    if (f2 & 0x20) != 0 {
        // SDPCM flow control (cyw43_ll.c:651,845): the host may send only while
        // credit != tx_seq. tx_seq starts 0, credit starts 1; each received SDPCM
        // header's bus_data_credit advances our credit (if the delta is <= 20).
        // Every frame we send uses sequence = tx_seq, then tx_seq += 1.
        let mut tx_seq = 0u8;
        let mut credit = 1u8;
        let mut fr = [0u32; 512];
        // rx: read one F2 frame -> (got, channel, cdc_status, bus_credit, cdc_id).
        // For a CONTROL frame the CDC id is in flags bits[31:16] (CDCF_IOC_ID).
        let rx = |fr: &mut [u32; 512]| -> (bool, u32, u32, u32, u32) {
            let mut rr = [0u32; 1];
            xfer(&[cmd_word(false, 0, 0x8, 4)], &mut rr);
            if rr[0] & 0x100 == 0 {
                return (false, 0xFF, 0, 0xFF, 0);
            }
            let len = ((rr[0] >> 9) & 0x7FF) as usize;
            if len < 12 {
                return (false, 0xFF, 0, 0xFF, 0);
            }
            let w = len.div_ceil(4).min(512);
            xfer(&[cmd_word(false, 2, 0, len as u32)], &mut fr[..w]);
            let chan = (fr[1] >> 8) & 0xFF;
            let bdc = (fr[2] >> 8) & 0xFF; // SdpcmHeader.bus_data_credit
            let (status, id) = if chan == 0 {
                let hl = (((fr[1] >> 24) & 0xFF) / 4) as usize;
                (fr[hl + 3], (fr[hl + 2] >> 16) & 0xFFFF)
            } else {
                (0xEEEE_EEEE, 0)
            };
            (true, chan, status, bdc, id)
        };
        // do_ioctl: one flow-controlled SET_VAR/command. Wait for send credit,
        // stamp the SDPCM sequence, write [gSPI cmd | SDPCM | CDC | payload],
        // advance tx_seq, then read frames (each advances credit) until the
        // CONTROL response, whose CDC status it returns.
        let do_ioctl = |cmd: u32,
                        id: u32,
                        payload: &[u32],
                        pbytes: usize,
                        tx_seq: &mut u8,
                        credit: &mut u8,
                        fr: &mut [u32; 512]|
         -> u32 {
            let mut g = 0u32;
            while *credit == *tx_seq && g < 8000 {
                g += 1;
                let (got, chan, _s, bdc, _i) = rx(fr);
                if got {
                    if chan < 3 && (bdc.wrapping_sub(*credit as u32) & 0xFF) <= 20 {
                        *credit = bdc as u8;
                    }
                } else {
                    cortex_m::asm::delay(10_000);
                }
            }
            let mut b = [0u32; 264];
            let total = 12 + 16 + pbytes;
            b[0] = 0xE000_0000 | total as u32;
            b[1] = (total as u32 & 0xFFFF) | (((!(total as u32)) & 0xFFFF) << 16);
            b[2] = 0x0C00_0000 | *tx_seq as u32;
            b[4] = cmd;
            b[5] = pbytes as u32;
            b[6] = 0x0000_0002 | (id << 16); // flags=Set, id
            for (k, &w) in payload.iter().enumerate() {
                b[8 + k] = w;
            }
            xfer(&b[..8 + pbytes.div_ceil(4)], &mut [0u32; 1]);
            *tx_seq = tx_seq.wrapping_add(1);
            // Read frames until the CONTROL response whose id matches our request.
            let mut st = 0xEEEE_EEEEu32;
            for _ in 0..3000u32 {
                let (got, chan, status, bdc, rid) = rx(fr);
                if got {
                    if chan < 3 && (bdc.wrapping_sub(*credit as u32) & 0xFF) <= 20 {
                        *credit = bdc as u8;
                    }
                    if chan == 0 && rid == id {
                        st = status;
                        break;
                    }
                } else {
                    cortex_m::asm::delay(20_000);
                }
            }
            st
        };
        // Byte-based ioctl: payload as bytes, packed little-endian into words (no
        // manual u32 byte-order mistakes). kind 2=SET, 0=GET. cdc_len = the CDC
        // length (payload len for SET; name+response size for GET). Leaves the
        // matched CONTROL response frame in fr; returns the CDC status.
        let do_ioctl_b = |kind: u32,
                          cmd: u32,
                          id: u32,
                          payload: &[u8],
                          cdc_len: usize,
                          tx_seq: &mut u8,
                          credit: &mut u8,
                          fr: &mut [u32; 512]|
         -> u32 {
            let mut g = 0u32;
            while *credit == *tx_seq && g < 8000 {
                g += 1;
                let (got, chan, _s, bdc, _i) = rx(fr);
                if got {
                    if chan < 3 && (bdc.wrapping_sub(*credit as u32) & 0xFF) <= 20 {
                        *credit = bdc as u8;
                    }
                } else {
                    cortex_m::asm::delay(10_000);
                }
            }
            let mut b = [0u32; 264];
            let total = 12 + 16 + cdc_len;
            b[0] = 0xE000_0000 | total as u32;
            b[1] = (total as u32 & 0xFFFF) | (((!(total as u32)) & 0xFFFF) << 16);
            b[2] = 0x0C00_0000 | *tx_seq as u32;
            b[4] = cmd;
            b[5] = cdc_len as u32;
            b[6] = kind | (id << 16);
            for (j, &byte) in payload.iter().enumerate() {
                b[8 + j / 4] |= (byte as u32) << (8 * (j % 4));
            }
            xfer(&b[..8 + cdc_len.div_ceil(4)], &mut [0u32; 1]);
            *tx_seq = tx_seq.wrapping_add(1);
            let mut st = 0xEEEE_EEEEu32;
            for _ in 0..3000u32 {
                let (got, chan, status, bdc, rid) = rx(fr);
                if got {
                    if chan < 3 && (bdc.wrapping_sub(*credit as u32) & 0xFF) <= 20 {
                        *credit = bdc as u8;
                    }
                    if chan == 0 && rid == id {
                        st = status;
                        break;
                    }
                } else {
                    cortex_m::asm::delay(20_000);
                }
            }
            st
        };
        // The firmware init the LED (gpioout) needs: CLM -> country -> WLC_UP.
        // 1. CLM: SET_VAR "clmload" + DownloadHeader + 984-byte WCLM blob.
        let clm_src = 0x1c23_8700 as *const u32;
        let mut clm_pl = [0u32; 251];
        clm_pl[0] = 0x6c6d_6c63; // "clml"
        clm_pl[1] = 0x0064_616f; // "oad\0"
        clm_pl[2] = 0x0002_1006; // flag=BEGIN|END|HANDLER_VER, dload_type=CLM
        clm_pl[3] = 984; // len
        for k in 0..246 {
            clm_pl[5 + k] = unsafe { core::ptr::read_volatile(clm_src.add(k)) };
        }
        let clm = do_ioctl(0x107, 1, &clm_pl, 8 + 12 + 984, &mut tx_seq, &mut credit, &mut fr);
        CYW43_PIO[11].store(clm, SeqCst);
        // Verify the CLM actually loaded: GET "clmload_status" (embassy asserts 0).
        {
            let mut gg = 0u32;
            while credit == tx_seq && gg < 8000 {
                gg += 1;
                let (got, chan, _s, bdc, _i) = rx(&mut fr);
                if got {
                    if chan < 3 && (bdc.wrapping_sub(credit as u32) & 0xFF) <= 20 {
                        credit = bdc as u8;
                    }
                } else {
                    cortex_m::asm::delay(10_000);
                }
            }
            let mut b = [0u32; 40];
            let pb: usize = 64;
            let tot = 12 + 16 + pb;
            b[0] = 0xE000_0000 | tot as u32;
            b[1] = (tot as u32 & 0xFFFF) | (((!(tot as u32)) & 0xFFFF) << 16);
            b[2] = 0x0C00_0000 | tx_seq as u32;
            b[4] = 0x0000_0106; // WLC_GET_VAR
            b[5] = pb as u32;
            b[6] = 9 << 16; // GET (kind 0), id=9
            b[8] = 0x6c6d_6c63; // "clml"
            b[9] = 0x5f64_616f; // "oad_"
            b[10] = 0x7461_7473; // "stat"
            b[11] = 0x0000_7375; // "us\0"
            xfer(&b[..8 + pb.div_ceil(4)], &mut [0u32; 1]);
            tx_seq = tx_seq.wrapping_add(1);
            for _ in 0..3000u32 {
                let (got, chan, _s, bdc, rid) = rx(&mut fr);
                if got {
                    if chan < 3 && (bdc.wrapping_sub(credit as u32) & 0xFF) <= 20 {
                        credit = bdc as u8;
                    }
                    if chan == 0 && rid == 9 {
                        let hl = (((fr[1] >> 24) & 0xFF) / 4) as usize;
                        CYW43_PIO[13].store(fr[hl + 3], SeqCst); // GET CDC status
                        CYW43_PIO[14].store(fr[hl + 4], SeqCst); // clmload_status (want 0)
                        break;
                    }
                } else {
                    cortex_m::asm::delay(20_000);
                }
            }
        }
        // Match cyw43_ll_bus_init: after the CLM, bus:txglom=0 and apsta=1 -- then
        // drive the LED via gpioout, BEFORE any WLC_UP (once the interface is up
        // the chip reassigns WL_GPIO0 and gpioout returns -23).
        let txglom = [0x3a737562, 0x6c67_7874, 0x0000_6d6f, 0x0000_0000]; // "bus:txglom\0"+le32(0)
        do_ioctl(0x107, 5, &txglom, 11 + 4, &mut tx_seq, &mut credit, &mut fr);
        let apsta = [0x74737061, 0x0001_0061, 0x0000_0000]; // "apsta\0" + le32(1)
        do_ioctl(0x107, 6, &apsta, 6 + 4, &mut tx_seq, &mut credit, &mut fr);
        // GET the chip's MAC ("cur_etheraddr") -- validates the byte-based ioctl
        // helper and reads a real chip value. Response = 6-byte MAC at payload 0.
        let mac_stat = do_ioctl_b(
            0,
            0x106,
            7,
            b"cur_etheraddr\0",
            14 + 6,
            &mut tx_seq,
            &mut credit,
            &mut fr,
        );
        {
            let hl = (((fr[1] >> 24) & 0xFF) / 4) as usize;
            CYW43_PIO[5].store(mac_stat, SeqCst); // GET status (0 = ok)
            CYW43_PIO[6].store(fr[hl + 4], SeqCst); // MAC bytes 0-3
            CYW43_PIO[7].store(fr[hl + 5], SeqCst); // MAC bytes 4-5
        }
        // Blink WL_GPIO0 forever via gpioout, honouring SDPCM flow control.
        CYW43_PIO[15].store(0x11ED_B11C, SeqCst);
        let mut i = 0u32;
        loop {
            let mut gg = 0u32;
            while credit == tx_seq && gg < 8000 {
                gg += 1;
                let (got, chan, _s, bdc, _i) = rx(&mut fr);
                if got {
                    if chan < 3 && (bdc.wrapping_sub(credit as u32) & 0xFF) <= 20 {
                        credit = bdc as u8;
                    }
                } else {
                    cortex_m::asm::delay(10_000);
                }
            }
            let val = if i & 1 == 0 { 1u32 } else { 0u32 };
            let id = 10 + (i & 0xFF);
            let mut b = [0u32; 16];
            let tot = 12 + 16 + 16usize;
            b[0] = 0xE000_0000 | tot as u32;
            b[1] = (tot as u32 & 0xFFFF) | (((!(tot as u32)) & 0xFFFF) << 16);
            b[2] = 0x0C00_0000 | tx_seq as u32;
            b[4] = 0x0000_0107; // SET_VAR
            b[5] = 16;
            b[6] = 0x0000_0002 | (id << 16);
            b[8] = 0x6f69_7067; // "gpio" (LE: 'g' 'p' 'i' 'o')
            b[9] = 0x0074_756f; // "out\0"
            b[10] = 0x0000_0001; // mask = 1<<0
            b[11] = val; // value (on/off)
            xfer(&b[..12], &mut [0u32; 1]);
            tx_seq = tx_seq.wrapping_add(1);
            for _ in 0..2000u32 {
                let (got, chan, _s, bdc, rid) = rx(&mut fr);
                if got {
                    if chan < 3 && (bdc.wrapping_sub(credit as u32) & 0xFF) <= 20 {
                        credit = bdc as u8;
                    }
                    if chan == 0 && rid == id {
                        let hl = (((fr[1] >> 24) & 0xFF) / 4) as usize;
                        CYW43_PIO[12].store(fr[hl + 3], SeqCst); // gpioout status
                        break;
                    }
                } else {
                    cortex_m::asm::delay(20_000);
                }
            }
            cortex_m::asm::delay(30_000_000); // ~200 ms
            i = i.wrapping_add(1);
        }
    }

    CYW43_PIO[5].store(if ok0 && ok1 && ok2 { 0x600D_600D } else { 0xBAD0_0000 }, SeqCst);
    CYW43_PIO[6].store(if magic_ok { magic } else { 0xBAD0_0001 }, SeqCst); // NVRAM magic
    CYW43_PIO[7].store(if core_up { 0xC0DE_600D } else { (io << 8) | rc }, SeqCst); // WLAN up
    CYW43_PIO[8].store(ht, SeqCst); // CHIP_CLOCK_CSR (want HT_AVAIL 0x80 in a lane)
    CYW43_PIO[9].store(f2, SeqCst); // REG_BUS_STATUS (want F2_RX_READY 0x20)
    CYW43_PIO[10].store(f2spin, SeqCst); // F2-ready poll count

    CYW43_PIO[0].store(chip, SeqCst); // want 0xFEEDBEAD (chip-detect)
    CYW43_PIO[1].store(rw, SeqCst); // want 0x12345678 (write path verified)
    CYW43_PIO[2].store(alp, SeqCst); // CHIP_CLOCK_CSR (want ALP_AVAIL 0x40 in a lane)
    CYW43_PIO[3].store(aspin, SeqCst); // ALP poll count
    CYW43_PIO[4].store(chip_id, SeqCst); // CHIPCOMMON chip-id (low 16 bits = 0x4345)
    sio.gpio_out_set().write(|w| unsafe { w.bits(1 << CS) });
}
// -----------------------------------------------------------------------------


#[entry]
fn main() -> ! {
    let p = unsafe { rp235x_pac::Peripherals::steal() };

    // Two-hop BOOTSEL reboot: calling boot-ROM functions from unprivileged
    // tasks faults (privilege-gated; verified by experiment -- a privileged
    // flash_op here works), so the runtime `reboot bootsel` instead marks
    // watchdog scratch0 and does a plain watchdog reboot; *this* privileged
    // boot path completes the hop by calling the ROM reboot into BOOTSEL.
    if p.WATCHDOG.scratch0().read().bits() == BOOTSEL_MAGIC {
        p.WATCHDOG.scratch0().write(|w| unsafe { w.bits(0) });
        let rb = rp235x_romapi::rom_table_lookup(
            *b"RB",
            rp235x_romapi::RT_FLAG_FUNC_ARM_SEC,
        );
        if rb != 0 {
            let f: unsafe extern "C" fn(u32, u32, u32, u32) -> i32 =
                unsafe { core::mem::transmute(rb) };
            // REBOOT_TYPE_BOOTSEL (0x2) | NO_RETURN_ON_SUCCESS (0x100).
            unsafe { f(0x0102, 10, 0, 0) };
        }
        // Lookup/call failure: fall through to a normal boot.
    }

    // Bring IO_BANK0 and PADS_BANK0 out of reset. This runs in the privileged
    // pre-kernel context (MPU not yet enabled), so no `uses` grant is required.
    p.RESETS
        .reset()
        .modify(|_, w| w.io_bank0().clear_bit().pads_bank0().clear_bit());
    while p.RESETS.reset_done().read().io_bank0().bit_is_clear() {}
    while p.RESETS.reset_done().read().pads_bank0().bit_is_clear() {}

    // Configure GPIO25 (Pico 2 onboard LED) as an SIO output, initially high.
    // On RP2350 the pad resets isolated (ISO=1) and output-disabled (OD=1); clear
    // both so the SIO output reaches the pin. Then function-select SIO (5) and
    // enable the output driver. The blinky task toggles it from here.
    p.PADS_BANK0
        .gpio(LED_PIN as usize)
        .modify(|_, w| w.od().clear_bit().iso().clear_bit());
    p.IO_BANK0
        .gpio(LED_PIN as usize)
        .gpio_ctrl()
        .modify(|_, w| unsafe { w.funcsel().bits(5) });
    p.SIO.gpio_oe_set().write(|w| unsafe { w.bits(1 << LED_PIN) });
    p.SIO.gpio_out_set().write(|w| unsafe { w.bits(1 << LED_PIN) });

    // I2C0 pins: GP4 = SDA, GP5 = SCL (funcsel 3), open-drain bus with the
    // internal pull-ups enabled and inputs on. Privileged (touches PADS/IO_BANK0);
    // the I2C driver task itself only gets the i2c0 register window.
    for pin in [4usize, 5] {
        p.PADS_BANK0.gpio(pin).modify(|_, w| {
            w.od().clear_bit();
            w.iso().clear_bit();
            w.ie().set_bit();
            w.pue().set_bit();
            w.pde().clear_bit()
        });
        p.IO_BANK0
            .gpio(pin)
            .gpio_ctrl()
            .modify(|_, w| unsafe { w.funcsel().bits(3) });
    }

    // SPI0 pins for the board-to-board link (funcsel 1): GP16 = RX (data in),
    // GP17 = CSn, GP18 = SCK, GP19 = TX (data out). Push-pull, no pulls. The
    // SPI driver defaults to internal loopback (ignores these pads) until a
    // `spi role` command switches it to a real controller/peripheral.
    for pin in [16usize, 17, 18, 19] {
        p.PADS_BANK0.gpio(pin).modify(|_, w| {
            w.od().clear_bit();
            w.iso().clear_bit();
            w.ie().set_bit()
        });
        p.IO_BANK0
            .gpio(pin)
            .gpio_ctrl()
            .modify(|_, w| unsafe { w.funcsel().bits(1) });
    }

    // Bring up XOSC + PLL_SYS to a known 150 MHz and get the accurate tick
    // divisor. Runs here (privileged, pre-kernel) because CLOCKS/PLL are
    // ACCESSCTRL Privileged-only. If this hangs, the LED (lit above) stays solid.
    let cycles_per_ms = rp235x_startup::init_clocks(&p);

    // Let unprivileged tasks (with the matching MPU grants) use the boot-ROM
    // reboot API: open ACCESSCTRL for the WATCHDOG/TICKS blocks it drives.
    rp235x_startup::open_accessctrl_for_reboot(&p);

    // Clocks (incl. PLL_USB, for USB) survived -- turn the LED off so the USB task's
    // heartbeat toggle starts from a known dark state. (Solid LED here => hung in
    // init_clocks; a heartbeat blink => the USB task is running.)
    p.SIO.gpio_out_clr().write(|w| unsafe { w.bits(1 << LED_PIN) });

    // Bring up UART0 (115200 8N1 on GP0/GP1) and print a boot banner on GP0. Baud is
    // derived from the now-known clk_peri frequency. Privileged (touches
    // RESETS/PADS/IO_BANK0). The USB console is the primary I/O; this is just a
    // secondary banner for a serial adapter on GP0.
    rp235x_uart::configure(&p, cycles_per_ms * 1000);
    rp235x_uart::write_all(&p, b"\r\nHubris booting on RP2350 / Pico 2\r\n");

    // Stash the unique 64-bit device id (OTP CHIPID0..3) in watchdog
    // scratch1/2 for the USB task, which builds its per-board USB serial
    // string from it (identical serials collide on the host). Read here
    // because OTP is privileged-only. Scratch registers survive resets and,
    // unlike USB DPRAM, are live before the USB block is unreset; the ROM's
    // vectored-boot protocol only claims scratch 4-7 (and our BOOTSEL marker
    // uses scratch0), and this runs after the BOOTSEL-hop check.
    let id: u64 = (p.OTP_DATA.chipid3().read().bits() as u64) << 48
        | (p.OTP_DATA.chipid2().read().bits() as u64) << 32
        | (p.OTP_DATA.chipid1().read().bits() as u64) << 16
        | p.OTP_DATA.chipid0().read().bits() as u64;
    p.WATCHDOG
        .scratch1()
        .write(|w| unsafe { w.bits((id >> 32) as u32) });
    p.WATCHDOG
        .scratch2()
        .write(|w| unsafe { w.bits(id as u32) });

    // P1: prove PIO works (echo through PIO0 SM0; result in PIO_PROBE).
    pio_echo_test(&p);

    // P2/P3: chip-detect the CYW43439 over PIO-driven gSPI (result in CYW43_PIO,
    // want 0xBEADFEED). Harmless on a plain Pico 2 (GP23-29 float).
    cyw43_pio_detect(&p);
    let _ = pio_output_test; // loopback validator (peer-captured; kept for reuse)

    unsafe { kern::startup::start_kernel(cycles_per_ms) }
}
