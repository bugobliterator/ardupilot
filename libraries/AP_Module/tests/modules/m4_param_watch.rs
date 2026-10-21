// SPDX-License-Identifier: GPL-3.0-or-later
//! The Parameter mirror on the M4: reads PMW_VALUE through a handle every 5 ms and reports each new
//! value as the named float PMW; PMW_CMD makes it write PMW_VALUE itself (by name, by handle, by
//! handle with save) and check it reads its own write at once. No float formatting, so the thumb
//! build links no f64 software-float routine.
#![no_std]
#![no_main]

use ap_rust_sdk::{enums::MavSeverity, Context, Error, Result};

ap_rust_sdk::module!(run);

const TABLE_KEY: u8 = 195;

fn report(context: &Context<'_>, ok: bool, pass: &str, fail: &str) -> Result {
    context.gcs().send_text(MavSeverity::INFO, if ok { pass } else { fail })
}

fn run(context: &Context<'_>) -> Result {
    // a MOD_RESTART finds the table already added: the results are not needed
    let _ = context.param().add_table(TABLE_KEY, "PMW_", 2)?;
    let _ = context.param().add_param(TABLE_KEY, 1, "VALUE", 0.0)?;
    let _ = context.param().add_param(TABLE_KEY, 2, "CMD", 0.0)?;
    let watched = context.parameter("PMW_VALUE")?.ok_or(Error::Host)?;
    let second = context.parameter("PMW_VALUE")?.ok_or(Error::Host)?;
    let command = context.parameter("PMW_CMD")?.ok_or(Error::Host)?;
    context.gcs().send_text(MavSeverity::INFO, "pmw ready")?;
    let mut last_bits: Option<u32> = None;
    let mut last_command = command.get()?.unwrap_or(0.0);
    while !context.stop_requested() {
        let value = watched.get()?.ok_or(Error::Host)?;
        if last_bits != Some(value.to_bits()) {
            last_bits = Some(value.to_bits());
            context.gcs().send_named_float("PMW", value)?;
        }
        let requested = command.get()?.unwrap_or(0.0);
        if requested != last_command {
            last_command = requested;
            if requested == 1.0 {
                let _ = context.param().set("PMW_VALUE", 41.0)?;
                report(context, watched.get()? == Some(41.0), "pmw ryw name ok", "pmw ryw name fail")?;
            } else if requested == 2.0 {
                let _ = second.set(42.0)?;
                report(context, watched.get()? == Some(42.0), "pmw ryw handle ok", "pmw ryw handle fail")?;
            } else if requested == 3.0 {
                let _ = second.set_and_save(43.0)?;
                report(context, watched.get()? == Some(43.0), "pmw ryw save ok", "pmw ryw save fail")?;
            }
        }
        context.sleep_ms(5)?;
    }
    Ok(())
}
