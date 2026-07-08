// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use std::fmt::Write as _;
use std::io::Write as _;

/// Peak PWM duty compare value for the sine samples. MUST match `SINE_TOP` in
/// the driver (`src/main.rs`): the LUT is scaled to this counter wrap, so an
/// out-of-sync value would clip or under-drive every sample. 1023 = 10-bit.
const SINE_TOP: u16 = 1023;

/// Number of entries in the generated sine LUT. Pairs with the driver's
/// SINE_LUT_INDEX_SHIFT (`src/main.rs`): the DDS shifts the phase accumulator
/// right by 32 - log2(SINE_LUT_LEN) to index this table, so the two must change
/// together. 256 is a power of two, so the shift is exact.
const SINE_LUT_LEN: usize = 256;

fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    idol::Generator::new()
        .with_counters(
            idol::CounterSettings::default().with_server_counters(false),
        )
        .build_server_support(
            "../../idl/rp235x-pwm.idol",
            "server_stub.rs",
            idol::server::ServerStyle::InOrder,
        )?;

    generate_sine_lut()?;
    Ok(())
}

/// Emit a 256-entry unsigned sine table, scaled 0..=SINE_TOP with a DC offset
/// of half-scale, to `$OUT_DIR/sine_lut.rs`. The driver `include!`s it as
/// `SINE_LUT`. Runs on the host, so `f64::sin` from std is available.
fn generate_sine_lut() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    use std::f64::consts::PI;

    let mut body = String::new();
    write!(body, "pub static SINE_LUT: [u16; {SINE_LUT_LEN}] = [")?;
    for i in 0..SINE_LUT_LEN {
        // (sin + 1)/2 maps [-1, 1] -> [0, 1]; scale to the counter wrap.
        let sample =
            (((2.0 * PI * i as f64 / SINE_LUT_LEN as f64).sin() + 1.0) / 2.0)
                * SINE_TOP as f64;
        let value = sample.round() as u16;
        write!(body, "{value}, ")?;
    }
    body.push_str("];\n");

    let out_dir = std::env::var("OUT_DIR")?;
    let path = std::path::Path::new(&out_dir).join("sine_lut.rs");
    let mut file = std::fs::File::create(path)?;
    file.write_all(body.as_bytes())?;
    Ok(())
}
