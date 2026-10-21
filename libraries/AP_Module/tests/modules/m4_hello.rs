// SPDX-License-Identifier: GPL-3.0-or-later
//! First module on the Cortex-M4: one host text and one M7-served gcs.send_text, then a sleep
//! loop until stopped. No float formatting, so the thumb build links no f64 software-float routine.
#![no_std]
#![no_main]

use ap_rust_sdk::{enums::MavSeverity, Context, Result};

ap_rust_sdk::module!(run);

fn run(context: &Context<'_>) -> Result {
    context.text(6, "m4 hello")?;
    context.gcs().send_text(MavSeverity::INFO, "m4 hello gcs")?;
    while !context.stop_requested() {
        context.sleep_ms(1000)?;
    }
    Ok(())
}
