// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! RP2350 (RP235x) clock bring-up for Hubris.
//!
//! This runs in the app's privileged pre-kernel `main`, which matters because
//! CLOCKS / XOSC / PLL_SYS are ACCESSCTRL Secure-*Privileged*-only on RP2350 (see
//! `docs/rp2350-research/findings.md` §4) — an unprivileged driver task could not
//! touch them.
//!
//! It starts the Pico 2's 12 MHz crystal (XOSC) and runs `clk_sys` (and hence the
//! CPU and kernel SysTick) directly from it, giving a **crystal-accurate** clock in
//! place of the earlier ROSC-frequency guess. `clk_peri` (which feeds UART/SPI) is
//! also pointed at this clock.
//!
//! We deliberately do NOT ramp to the PLL's 150 MHz here: the boot ROM configures
//! flash XIP timing for the boot clock, and pushing `clk_sys` far above it without
//! retuning the QMI divider would make XIP flash reads fail (RP2350 datasheet §5.9.5
//! notes the bootrom XIP state is only valid around the boot clock). PLL_SYS →
//! 150 MHz with matching QMI timing is a separate follow-up.

#![no_std]

use rp235x_pac::Peripherals;

/// System clock after bring-up, in Hz: the 12 MHz crystal, run straight through.
pub const SYS_CLK_HZ: u32 = 12_000_000;

/// Start XOSC, run `clk_sys`/`clk_peri` from it, and return `cycles_per_ms`.
///
/// If a "wait for stable/selected" spin never completes (a misconfiguration), this
/// hangs here — by design the caller lights the LED *before* calling this, so a hang
/// shows as a solid (non-blinking) LED.
pub fn init_clocks(p: &Peripherals) -> u32 {
    // --- Start the 12 MHz crystal oscillator (XOSC) ---
    // FREQ_RANGE selects the 1-15 MHz band. STARTUP.DELAY is
    // (f_xosc_khz + 128) / 256 = (12000 + 128) / 256 = 47.
    p.XOSC.ctrl().write(|w| w.freq_range()._1_15mhz());
    p.XOSC.startup().write(|w| unsafe { w.delay().bits(47) });
    p.XOSC.ctrl().modify(|_, w| w.enable().enable());
    while p.XOSC.status().read().stable().bit_is_clear() {}

    // --- Run clk_ref from XOSC (glitchless mux; XOSC = SRC value 2) ---
    // clk_sys runs from clk_ref by reset default, so this also makes clk_sys the
    // crystal. clk_ref_selected is one-hot on the selected SRC.
    p.CLOCKS.clk_ref_ctrl().modify(|_, w| w.src().xosc_clksrc());
    while p.CLOCKS.clk_ref_selected().read().bits() & (1 << 2) == 0 {}

    // --- clk_peri from clk_sys (feeds UART/SPI in later phases) ---
    p.CLOCKS
        .clk_peri_ctrl()
        .write(|w| w.enable().set_bit().auxsrc().clk_sys());

    SYS_CLK_HZ / 1000
}
