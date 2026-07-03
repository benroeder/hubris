// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use std::io::Write;

fn main() {
    build_util::expose_target_board();

    // The IMAGE_DEF VERSION item's second word: (major << 16) | minor. The
    // boot ROM boots the higher-versioned valid image across A/B partitions,
    // so bumping this is how an update wins. Set via HUBRIS_IMAGE_VERSION
    // ("major.minor"); defaults to 1.0.
    println!("cargo:rerun-if-env-changed=HUBRIS_IMAGE_VERSION");
    let v =
        std::env::var("HUBRIS_IMAGE_VERSION").unwrap_or_else(|_| "1.0".into());
    let (maj, min) = v.split_once('.').unwrap_or((v.as_str(), "0"));
    let maj: u16 = maj.parse().expect("HUBRIS_IMAGE_VERSION major");
    let min: u16 = min.parse().expect("HUBRIS_IMAGE_VERSION minor");
    let word = ((maj as u32) << 16) | (min as u32);

    let out = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap())
        .join("image_version.rs");
    let mut f = std::fs::File::create(out).unwrap();
    writeln!(f, "pub const IMAGE_VERSION_WORD: u32 = {word:#010x};").unwrap();
}
