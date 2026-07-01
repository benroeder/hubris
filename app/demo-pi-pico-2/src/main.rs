// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

// Pull in the PAC so its interrupt vector table is linked into the image.
use rp235x_pac as _;

use cortex_m_rt::entry;

/// RP2350 boot metadata: the minimum valid Arm IMAGE_DEF block loop.
///
/// The RP2350 boot ROM refuses to start a flash image that lacks this. It must
/// appear within the first 4 KiB (the linker places `.image_def` right after the
/// vector table + Hubris `ImageHeader`; see `build/kernel-link.x`).
///
/// The five little-endian words are datasheet-verified (RP2350 datasheet
/// §5.9.5.1, "Minimum Arm IMAGE_DEF") and cross-checked in
/// `docs/rp2350-research/findings.md`.
#[link_section = ".image_def"]
#[used]
pub static RP235X_IMAGE_DEF_ARM_MIN: [u32; 5] = [
    0xffff_ded3, // PICOBIN_BLOCK_MARKER_START
    0x1021_0142, // IMAGE_TYPE item: EXE | SECURITY(S) | CPU(Arm) | CHIP(RP2350)
    0x0000_01ff, // BLOCK_ITEM_LAST, size = 1 word
    0x0000_0000, // link = self (single-block loop)
    0xab12_3579, // PICOBIN_BLOCK_MARKER_END
];

#[entry]
fn main() -> ! {
    let p = unsafe { rp235x_pac::Peripherals::steal() };

    // Bring IO_BANK0 out of reset so later phases (GPIO/UART) can use it.
    p.RESETS.reset().modify(|_, w| w.io_bank0().clear_bit());
    while p.RESETS.reset_done().read().io_bank0().bit_is_clear() {}

    // TODO(phase 1): replace with real XOSC + PLL bring-up in `lib/rp235x-startup`
    // (must run privileged: CLOCKS/PLL are ACCESSCTRL Privileged-only). For now,
    // estimate the tick divisor from the current clk_sys source.
    let cycles_per_ms = if p.CLOCKS.clk_sys_ctrl().read().src().is_clk_ref() {
        // Reset state: running from the ~6 MHz ROSC directly out of flash.
        6_000
    } else {
        // A resident debugger has likely switched clk_sys to the 48 MHz USB PLL.
        48_000
    };

    unsafe { kern::startup::start_kernel(cycles_per_ms) }
}
