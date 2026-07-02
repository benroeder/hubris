// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! RP2350 boot-ROM table lookup (datasheet sec 5.4).
//!
//! The 32 kB boot ROM at address 0 exports functions and data through a
//! lookup table; codes are two ASCII characters, e.g. `"FO"` = `flash_op`.
//! This crate resolves codes to ROM addresses. Wrappers that *call* the
//! flash-related ROM functions arrive with the erase/program stage of the
//! flash driver, which also needs ACCESSCTRL opened for QMI and a
//! RAM-resident kernel + call sequence (datasheet sec 5.4.8.9: XIP accesses
//! bus-error while the QMI is in direct mode).
//!
//! Lookup mechanics cross-checked against the bootrom source
//! (`raspberrypi/pico-bootrom-rp2350`, `arm8_bootrom_rt0.S`) and the
//! datasheet's well-known-address table (Table 454).

#![no_std]

/// 16-bit pointer to the ROM entry table (BOOTROM_ROMTABLE_START), at the
/// RISC-V well-known location (datasheet Table 454). The Arm copy lives at
/// 0x14 -- unreadable from a Hubris task, because the kernel gives every task
/// a no-access MPU region over 0x0..0x20 to catch null dereferences and
/// PMSAv8 forbids overlapping regions. Both point at the *same* table.
const ROM_TABLE_PTR: *const u16 = 0x7df6 as _;

/// One past the last valid ROM byte address (the ROM is 32 kB at 0).
const ROM_END: usize = 0x8000;

/// Lookup-mask bit: function callable from Secure Arm code.
pub const RT_FLAG_FUNC_ARM_SEC: u32 = 0x0004;
/// Lookup-mask bit: data entry.
pub const RT_FLAG_DATA: u32 = 0x0040;

/// Look up a ROM entry by its two-character code, e.g. `*b"FO"` for
/// `flash_op`. Returns 0 if the entry is absent (or the table looks insane).
///
/// This walks the ROM table directly instead of calling the ROM's own lookup
/// helper: the helper's address is published at 0x16, inside the task-MPU
/// null region (see [`ROM_TABLE_PTR`]). Entry format, from the bootrom source
/// (`arm8_bootrom_rt0.S`): each entry is a 2-char symbol (u16), an hword of
/// flags, then one hword of data per set flag bit; a zero symbol ends the
/// table. The data hword for a given flag bit is at the index of that bit
/// among the set bits (LSB first).
///
/// The returned value is a single hword. "Far" entries (32-bit data split
/// across two adjacent flag bits, e.g. some RISC-V entries) would be
/// truncated -- do not use this for multi-bit masks selecting such entries.
/// [`RT_FLAG_FUNC_ARM_SEC`] and [`RT_FLAG_DATA`] entries are single-hword.
///
/// # Safety-relevant precondition (not enforced here)
/// The caller must be able to read + execute the ROM: privileged code always
/// can; an unprivileged task needs the `rom` (0x20..0x8000, r-x) extern
/// region from the chip memory config.
pub fn rom_table_lookup(code: [u8; 2], mask: u32) -> usize {
    let tag = u16::from_le_bytes(code);
    let mut p = unsafe { core::ptr::read_volatile(ROM_TABLE_PTR) } as usize;
    if !(0x20..ROM_END).contains(&p) || !p.is_multiple_of(2) {
        return 0;
    }
    // Bounded walk: never read past the ROM even if the table were corrupt
    // (a mask ROM should make that impossible; a clean 0 beats a fault).
    while p + 4 <= ROM_END {
        // SAFETY: p is inside the ROM (bounds-checked above and per
        // iteration); the table is zero-terminated by the ROM.
        let sym = unsafe { core::ptr::read_volatile(p as *const u16) };
        if sym == 0 {
            return 0;
        }
        let flags =
            unsafe { core::ptr::read_volatile((p + 2) as *const u16) } as u32;
        if sym == tag && (flags & mask) != 0 {
            // Index of the first mask-selected bit among the set flag bits.
            let sel = flags & mask;
            let first = sel & sel.wrapping_neg();
            let idx = (flags & (first - 1)).count_ones() as usize;
            let addr = p + 4 + 2 * idx;
            if addr + 2 > ROM_END {
                return 0;
            }
            let val =
                unsafe { core::ptr::read_volatile(addr as *const u16) };
            return val as usize;
        }
        p += 4 + 2 * flags.count_ones() as usize;
    }
    0
}

