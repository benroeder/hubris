// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use std::io::Write;

#[derive(serde::Deserialize)]
struct Region {
    address: u32,
    size: u32,
}

fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    idol::Generator::new()
        .with_counters(
            idol::CounterSettings::default().with_server_counters(false),
        )
        .build_server_support(
            "../../idl/rp235x-flash.idol",
            "server_stub.rs",
            idol::server::ServerStyle::InOrder,
        )?;

    // Derive the flash window base/size from this task's `xip` extern region
    // (chips/rp235x/memory-*.toml), so the driver tracks the board config
    // instead of hardcoding the Pico 2's 4 MB.
    let regions =
        build_util::task_extern_regions::<Region>().map_err(|e| e.to_string())?;
    let xip = regions
        .get("xip")
        .ok_or("xip extern region not found in task config")?;
    let out = build_util::out_dir().join("flash_config.rs");
    let mut f = std::fs::File::create(out)?;
    writeln!(
        f,
        "pub const XIP_BASE: u32 = {:#x};\npub const FLASH_SIZE: u32 = {:#x};",
        xip.address, xip.size
    )?;
    Ok(())
}
