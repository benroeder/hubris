// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Build script for the RP2350 shell task.
//!
//! Emits a signed 16-bit sine table used by the `wavgen` command to synthesize
//! a valid PCM WAV file on the SD card. Runs on the host, so `f64::sin` from
//! std is available. The table has a power-of-two length so a DDS phase
//! accumulator indexes it with an exact right-shift.

use std::fmt::Write as _;
use std::io::Write as _;

/// Entries in the sine table. Power of two -> the `wavgen` DDS indexes it by
/// `phase >> (32 - log2(LEN))` with no rounding. 256 -> shift 24.
const SINE_LUT_LEN: usize = 256;

/// Peak amplitude of the generated i16 samples. Below full scale (32767) to
/// leave headroom and avoid any codec/PWM clipping at the extremes.
const SINE_AMPL: f64 = 20000.0;

fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    use std::f64::consts::PI;

    let mut body = String::new();
    write!(body, "pub static SINE_I16_LUT: [i16; {SINE_LUT_LEN}] = [")?;
    for i in 0..SINE_LUT_LEN {
        let s = ((2.0 * PI * i as f64 / SINE_LUT_LEN as f64).sin() * SINE_AMPL)
            .round() as i16;
        write!(body, "{s}, ")?;
    }
    body.push_str("];\n");

    let out_dir = std::env::var("OUT_DIR")?;
    let path = std::path::Path::new(&out_dir).join("sine_i16_lut.rs");
    std::fs::File::create(path)?.write_all(body.as_bytes())?;
    Ok(())
}
