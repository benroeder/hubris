// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

// Pull in the PAC so its interrupt vector table is linked into the image.
use rp235x_pac as _;

use cortex_m_rt::entry;

/// Pico 2 onboard LED.
const LED_PIN: u32 = 25;

/// RP2350 boot metadata: the minimum valid Arm IMAGE_DEF block loop.
///
/// The RP2350 boot ROM refuses to start a flash image that lacks this. It must
/// appear within the first 4 KiB (the linker places `.image_def` right after the
/// vector table + Hubris `ImageHeader`; see `build/kernel-link.x`).
///
/// The five little-endian words are datasheet-verified (RP2350 datasheet
/// sec 5.9.5.1, "Minimum Arm IMAGE_DEF") and cross-checked in
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

    unsafe { kern::startup::start_kernel(cycles_per_ms) }
}
