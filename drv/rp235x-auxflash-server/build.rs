// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use std::io::Write;

fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    idol::Generator::new()
        .with_counters(
            idol::CounterSettings::default().with_server_counters(false),
        )
        .build_server_support(
            "../../idl/auxflash.idol",
            "server_stub.rs",
            idol::server::ServerStyle::InOrder,
        )?;
    // The build packs the auxflash image and exports its SHA3 checksum; the
    // server uses it to pick the active slot.
    let e = build_util::env_var("HUBRIS_AUXFLASH_CHECKSUM")?;
    let out = build_util::out_dir().join("checksum.rs");
    let mut f = std::fs::File::create(out)?;
    writeln!(&mut f, "const AUXI_CHECKSUM: [u8; 32] = {e};")?;
    Ok(())
}
