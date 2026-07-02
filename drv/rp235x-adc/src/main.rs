// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! ADC driver server for the RP2350 (RP235x).
//!
//! One-shot 12-bit conversions behind an Idol interface (`idl/rp235x-adc.idol`).
//! Channels 0-3 are GPIO26-29. NOTE: no API configures those pads for analog
//! use yet (which wants OD=1, IE=0) -- reads of 0-3 work but sample the pad
//! as reset/left, so treat them as approximate until a pad-config op exists.
//! Channel 4 is the internal temperature sensor (datasheet sec 12.4.6),
//! whose bias is enabled at init so reads are always settled.
//!
//! Needs `clk_adc` = 48 MHz (from `lib/rp235x-startup`). Brings the ADC out
//! of reset via the sys server. Runs unprivileged with `uses = ["adc"]`.

#![no_std]
#![no_main]

use drv_rp235x_adc_api::AdcError;
use drv_rp235x_sys_api::{self as sys_api, Rp235xSys};
use idol_runtime::RequestError;
use userlib::{RecvMessage, task_slot};

task_slot!(SYS, sys);

/// Highest valid channel (RP2350A / QFN-60: 0-3 pins + 4 = temp sensor).
const MAX_CHANNEL: u8 = 4;

struct ServerImpl {
    adc: rp235x_pac::ADC,
}

impl idl::InOrderRp235xAdcImpl for ServerImpl {
    fn read(
        &mut self,
        _: &RecvMessage,
        channel: u8,
    ) -> Result<u16, RequestError<AdcError>> {
        if channel > MAX_CHANNEL {
            return Err(AdcError::InvalidChannel.into());
        }
        self.adc
            .cs()
            .modify(|_, w| unsafe { w.ainsel().bits(channel) });
        self.adc.cs().modify(|_, w| w.start_once().set_bit());
        while self.adc.cs().read().ready().bit_is_clear() {}
        Ok(self.adc.result().read().result().bits())
    }
}

impl idol_runtime::NotificationHandler for ServerImpl {
    fn current_notification_mask(&self) -> u32 {
        0
    }
    fn handle_notification(&mut self, _bits: userlib::NotificationBits) {}
}

#[export_name = "main"]
fn main() -> ! {
    // Bring the ADC out of reset via the sys server.
    let sys = Rp235xSys::from(SYS.get_task_id());
    sys.leave_reset(sys_api::ADC);

    let p = unsafe { rp235x_pac::Peripherals::steal() };
    let adc = p.ADC;

    // Enable the converter and the temperature-sensor bias (so channel-4
    // reads are settled), then wait for ready.
    adc.cs().write(|w| w.en().set_bit().ts_en().set_bit());
    while adc.cs().read().ready().bit_is_clear() {}

    let mut server = ServerImpl { adc };
    let mut incoming = [0u8; idl::INCOMING_SIZE];
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

mod idl {
    use drv_rp235x_adc_api::AdcError;
    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
