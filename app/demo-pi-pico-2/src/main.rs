// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

// Pull in the PAC so its interrupt vector table is linked into the image.
use rp235x_pac as _;

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::entry;

mod image_version {
    include!(concat!(env!("OUT_DIR"), "/image_version.rs"));
}

/// Pico 2 onboard LED.
const LED_PIN: u32 = 25;

// --- AMP milestone 1: launch a bare payload on core 1 -------------------------
//
// Optional multicore experiment (branch rp2350-amp). Core 0 runs single-core
// Hubris exactly as before; just before starting the kernel it wakes core 1
// with a minimal bare-metal payload that has NO kernel -- it only writes a
// shared mailbox in RAM so we can confirm, over the debug probe (`--core 1`),
// that core 1 launched and is running our code. Nothing here touches the
// kernel or any peripheral core 0 owns.

/// Shared mailbox: [0] = magic ("core 1 reached my code"), [1] = heartbeat.
/// Read these over SWD to see core 1 alive. `#[used]`/no-mangle so the symbol
/// address is easy to find with `nm`.
#[no_mangle]
#[used]
static CORE1_MAILBOX: [AtomicU32; 4] = [
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
];

/// Core 1's stack (1 KiB) -- it has no kernel, so this is its whole stack.
static mut CORE1_STACK: [u32; 256] = [0; 256];

const CORE1_MAGIC: u32 = 0xC0FF_EE01;

/// Bare payload that runs on core 1 after launch: announce via the mailbox,
/// then act as a compute engine over the SIO inter-core FIFO -- read each
/// request word from core 0 and reply with `req*2 + 1` -- while bumping a
/// heartbeat. Loop-only (no stack-heavy calls) so the compiler emits no
/// RCP/canary ops core 1's coprocessor is not salted for.
extern "C" fn core1_main() -> ! {
    let sio = unsafe { &*rp235x_pac::SIO::ptr() };
    CORE1_MAILBOX[0].store(CORE1_MAGIC, Ordering::SeqCst);
    let mut beat: u32 = 0;
    loop {
        if sio.fifo_st().read().vld().bit_is_set() {
            let req = sio.fifo_rd().read().bits();
            let reply = req.wrapping_mul(2).wrapping_add(1);
            while sio.fifo_st().read().rdy().bit_is_clear() {}
            sio.fifo_wr().write(|w| unsafe { w.bits(reply) });
        }
        beat = beat.wrapping_add(1);
        CORE1_MAILBOX[1].store(beat, Ordering::SeqCst);
    }
}

/// Wake core 1 (asleep in the bootrom) and start it at `core1_main`, using the
/// SIO inter-core FIFO handshake from datasheet sec 5.3:
/// send {0, 0, 1, VTOR, SP, entry}, each echoed back before advancing.
fn launch_core1(sio: &rp235x_pac::SIO) {
    let vtor = 0x2000_0000u32; // the RAM image's vector table (unused by the loop)
    let sp = core::ptr::addr_of!(CORE1_STACK) as u32 + 256 * 4;
    let entry = core1_main as *const () as u32 | 1; // thumb bit
    let seq = [0u32, 0, 1, vtor, sp, entry];
    let mut i = 0usize;
    while i < seq.len() {
        let cmd = seq[i];
        if cmd == 0 {
            // Drain anything core 1 left in our read FIFO, then wake it.
            while sio.fifo_st().read().vld().bit_is_set() {
                let _ = sio.fifo_rd().read().bits();
            }
            cortex_m::asm::sev();
        }
        while sio.fifo_st().read().rdy().bit_is_clear() {}
        sio.fifo_wr().write(|w| unsafe { w.bits(cmd) });
        cortex_m::asm::sev();
        while sio.fifo_st().read().vld().bit_is_clear() {
            cortex_m::asm::wfe();
        }
        let resp = sio.fifo_rd().read().bits();
        i = if cmd == resp { i + 1 } else { 0 };
    }
}
// -----------------------------------------------------------------------------

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

    // AMP milestone 1: wake core 1 with the bare payload before core 0 enters
    // the kernel. Core 0 continues to single-core Hubris; core 1 spins its loop
    // and reports via CORE1_MAILBOX (read it over `probe-rs --core 1`).
    launch_core1(&p.SIO);

    unsafe { kern::startup::start_kernel(cycles_per_ms) }
}
