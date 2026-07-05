// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Test-image entry for the RP2350 (RP235x) Hubris kernel test suite.
//!
//! Minimal boot: emit the RAM-image IMAGE_DEF the boot ROM requires, bring up
//! the clocks, and start the kernel. The test tasks (runner/suite/assist/idol/
//! hiffy) come from the shared test framework; `humility test` drives them over
//! the probe. No peripheral bring-up is needed -- the IRQ-notification test
//! pends its interrupt through the NVIC via the runner.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use rp235x_pac as _;

/// RP2350 boot metadata: IMAGE_DEF for a RAM ("packaged") image, identical in
/// shape to the demo app's so the ROM copies the whole code window from flash
/// into SRAM before boot. Copy size is the full 256 KiB SRAM code window (the
/// test image is larger than the demo's 128 KiB); as a single non-partitioned
/// image the copy window is the whole flash, so this stays within bounds.
/// See app/demo-pi-pico-2/src/main.rs and docs/rp2350-research/findings.md.
#[link_section = ".image_def"]
#[used]
pub static RP235X_IMAGE_DEF_ARM_RAM: [u32; 13] = [
    0xffff_ded3, // PICOBIN_BLOCK_MARKER_START
    0x1021_0142, // IMAGE_TYPE: EXE | SECURITY(S) | CPU(Arm) | CHIP(RP2350)
    0x0000_0203, // VECTOR_TABLE item (type 0x03, 2 words)
    0x2000_0000,
    0x0100_0406, // LOAD_MAP item (type 0x06, RELATIVE + 1 entry)
    0xffff_fe90, // entry 0: storage start, relative to the load-map item
    0x2000_0000, // entry 0: runtime start (SRAM)
    0x0004_0000, // entry 0: size in bytes (256 KiB, the full code window)
    0x0000_0248, // VERSION item (type 0x48, 2 words)
    0x0001_0000, // version 1.0 (tests are single-image; no A/B arbitration)
    0x0000_09ff, // BLOCK_ITEM_LAST, size = 9 words of items
    0x0000_0000, // link = self (single-block loop)
    0xab12_3579, // PICOBIN_BLOCK_MARKER_END
];

#[entry]
fn main() -> ! {
    let p = unsafe { rp235x_pac::Peripherals::steal() };
    // Bring XOSC + PLL_SYS up to 150 MHz (privileged, pre-kernel) and get the
    // accurate tick divisor, exactly as the demo app does.
    let cycles_per_ms = rp235x_startup::init_clocks(&p);
    unsafe { kern::startup::start_kernel(cycles_per_ms) }
}
