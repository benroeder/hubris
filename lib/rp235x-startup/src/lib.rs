// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! RP2350 (RP235x) clock bring-up for Hubris.
//!
//! Runs in the app's privileged pre-kernel `main` -- CLOCKS / XOSC / PLL_SYS / QMI are
//! ACCESSCTRL Secure-*Privileged*-only on RP2350 (see
//! `docs/rp2350-research/findings.md` sec 4).
//!
//! Sequence: start the 12 MHz crystal (XOSC), run `clk_ref` from it, slow the flash
//! XIP clock (QMI) to stay in spec, bring PLL_SYS up to 150 MHz, switch `clk_sys` (CPU
//! + kernel SysTick) and `clk_peri` (UART/SPI) to it, and return `cycles_per_ms`.
//!
//! **QMI note:** the boot ROM configures flash XIP timing for the ~11 MHz boot clock.
//! Ramping `clk_sys` to 150 MHz without slowing the QMI clock divider first would run
//! flash reads far out of spec and hang the chip (datasheet sec 5.4.4 / sec 5.9.5). We set a
//! conservative `CLKDIV` (-> 25 MHz flash SCK) *before* the ramp, while still slow.

#![no_std]

use rp235x_pac::Peripherals;

/// System clock after bring-up, in Hz. 12 MHz XOSC x 125 / (5 x 2) = 150 MHz.
pub const SYS_CLK_HZ: u32 = 150_000_000;

/// Bring up XOSC + PLL_SYS to [`SYS_CLK_HZ`] and return `cycles_per_ms`.
///
/// If a "wait for stable/lock/selected" spin never completes, this hangs here -- by
/// design the caller lights the LED *before* calling this, so a hang shows as a solid
/// (non-blinking) LED.
pub fn init_clocks(p: &Peripherals) -> u32 {
    // --- 1. Start the 12 MHz crystal oscillator (XOSC) ---
    // STARTUP.DELAY = (f_xosc_khz + 128) / 256 = (12000 + 128) / 256 = 47.
    p.XOSC.ctrl().write(|w| w.freq_range()._1_15mhz());
    p.XOSC.startup().write(|w| unsafe { w.delay().bits(47) });
    p.XOSC.ctrl().modify(|_, w| w.enable().enable());
    while p.XOSC.status().read().stable().bit_is_clear() {}

    // --- 2. Run clk_ref (and thus clk_sys, by reset default) from XOSC ---
    p.CLOCKS.clk_ref_ctrl().modify(|_, w| w.src().xosc_clksrc());
    while p.CLOCKS.clk_ref_selected().read().bits() & (1 << 2) == 0 {}

    // --- 3. Slow the flash XIP clock BEFORE ramping clk_sys ---
    // At clk_sys = 150 MHz, CLKDIV = 6 gives a 25 MHz flash SCK, safe for the basic
    // serial read the boot ROM leaves configured. Do this while clk_sys is still
    // 12 MHz (SCK becomes 2 MHz here -- harmless) so flash is never over-clocked.
    p.QMI
        .m0_timing()
        .modify(|_, w| unsafe { w.clkdiv().bits(6).rxdelay().bits(1) });

    // --- 4. Configure PLL_SYS: VCO = 12 MHz / 1 x 125 = 1500 MHz ---
    p.RESETS.reset().modify(|_, w| w.pll_sys().clear_bit());
    while p.RESETS.reset_done().read().pll_sys().bit_is_clear() {}

    p.PLL_SYS.cs().modify(|_, w| unsafe { w.refdiv().bits(1) });
    p.PLL_SYS
        .fbdiv_int()
        .write(|w| unsafe { w.fbdiv_int().bits(125) });
    p.PLL_SYS
        .pwr()
        .modify(|_, w| w.pd().clear_bit().vcopd().clear_bit());
    while p.PLL_SYS.cs().read().lock().bit_is_clear() {}

    // Post-dividers: 1500 MHz / (5 x 2) = 150 MHz, then power the post-divider.
    p.PLL_SYS
        .prim()
        .write(|w| unsafe { w.postdiv1().bits(5).postdiv2().bits(2) });
    p.PLL_SYS.pwr().modify(|_, w| w.postdivpd().clear_bit());

    // --- 5. Switch clk_sys to PLL_SYS (glitchless: set AUX source, then flip SRC) ---
    p.CLOCKS
        .clk_sys_ctrl()
        .modify(|_, w| w.auxsrc().clksrc_pll_sys());
    p.CLOCKS
        .clk_sys_ctrl()
        .modify(|_, w| w.src().clksrc_clk_sys_aux());
    while p.CLOCKS.clk_sys_selected().read().bits() & (1 << 1) == 0 {}

    // --- 6. clk_peri from clk_sys (feeds UART/SPI), now 150 MHz ---
    p.CLOCKS
        .clk_peri_ctrl()
        .write(|w| w.enable().set_bit().auxsrc().clk_sys());

    // --- 7. PLL_USB -> clk_usb = 48 MHz (required by the USB device controller) ---
    // VCO = 12 MHz x 100 = 1200 MHz (in the 750-1600 MHz range); / (5 x 5) = 48 MHz.
    p.RESETS.reset().modify(|_, w| w.pll_usb().clear_bit());
    while p.RESETS.reset_done().read().pll_usb().bit_is_clear() {}
    p.PLL_USB.cs().modify(|_, w| unsafe { w.refdiv().bits(1) });
    p.PLL_USB
        .fbdiv_int()
        .write(|w| unsafe { w.fbdiv_int().bits(100) });
    p.PLL_USB
        .pwr()
        .modify(|_, w| w.pd().clear_bit().vcopd().clear_bit());
    while p.PLL_USB.cs().read().lock().bit_is_clear() {}
    p.PLL_USB
        .prim()
        .write(|w| unsafe { w.postdiv1().bits(5).postdiv2().bits(5) });
    p.PLL_USB.pwr().modify(|_, w| w.postdivpd().clear_bit());
    p.CLOCKS
        .clk_usb_ctrl()
        .write(|w| w.enable().set_bit().auxsrc().clksrc_pll_usb());

    // --- 8. clk_adc = 48 MHz from PLL_USB (feeds the ADC) ---
    p.CLOCKS
        .clk_adc_ctrl()
        .write(|w| w.enable().set_bit().auxsrc().clksrc_pll_usb());

    SYS_CLK_HZ / 1000
}

/// Pre-kernel privileged peripheral setup for unprivileged tasks. Despite the
/// `_for_reboot` name (its original sole purpose), this now also brings up the
/// DMA block for the audio path (see the end of the body).
///
/// Opens ACCESSCTRL so *unprivileged* code can reach the WATCHDOG and TICKS
/// blocks. The boot-ROM `reboot` API (datasheet sec 5.4.8.24) arms the reboot
/// through watchdog scratch/trigger registers, and ROM code runs at the
/// caller's privilege -- without this grant an unprivileged task calling it
/// faults. ACCESSCTRL writes require 0xacce in the top 16 bits (sec 10.6.3).
///
/// Security tradeoff, accepted for this board: any task granted the watchdog
/// MMIO region in its app.toml `uses` can now reboot the system. Tasks
/// without the MPU grant still cannot (the MPU check comes first).
///
/// Privileged; call from the app's pre-kernel main.
pub fn open_accessctrl_for_reboot(p: &Peripherals) {
    // Default is 0xb8 (SP | CORE0 | CORE1 | DMA); add SU (bit 2).
    const GRANT_SU: u32 = 0xacce_00bc;
    p.ACCESSCTRL
        .watchdog()
        .write(|w| unsafe { w.bits(GRANT_SU) });
    // The ROM reboot code writes PSM.WDSEL (which reset domains a watchdog
    // reset covers) -- PSM's ACCESSCTRL register is named `rsm`.
    p.ACCESSCTRL.rsm().write(|w| unsafe { w.bits(GRANT_SU) });
    // QMI direct mode: lets the flash driver task erase/program the QSPI
    // flash. Safe on this system because the whole image runs from SRAM
    // (LOAD_MAP boot) -- nothing fetches from flash at runtime.
    p.ACCESSCTRL.xip_qmi().write(|w| unsafe { w.bits(GRANT_SU) });
    // DMA bring-up for the PWM audio path (a continuous ring-buffer DMA feeds
    // the PWM compare register). Un-reset DMA, then clear SECCFG_CH0.P so an
    // UNPRIVILEGED task may program channel 0: at reset P=1 makes the channel
    // "controllable only from a Privileged context", so the task's writes to the
    // channel CTRL/READ_ADDR/WRITE_ADDR/TRANS_COUNT registers bus-fault. Keep S=1
    // (secure). No ACCESSCTRL grant is needed -- the DMA entry already defaults to
    // secure-any-master; SECCFG was the real gate (pinpointed via humility).
    // Harmless in images that never touch DMA (the block just sits idle).
    p.RESETS.reset().modify(|_, w| w.dma().clear_bit());
    while !p.RESETS.reset_done().read().dma().bit_is_set() {}
    p.DMA.seccfg_ch0().modify(|_, w| w.p().clear_bit());
}
