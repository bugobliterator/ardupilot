// SPDX-License-Identifier: GPL-3.0-or-later
//! Module fault kinds for the Cortex-M4 (Renode and the bench): the module reads RUST_M4_INJECT
//! every 100 ms and performs the kind it names. 14 writes to its own `start` function: RX is
//! read-only under the M4's MPU (W^X), so the write is a MemManage fault from module code.
#![no_std]
#![no_main]

use ap_rust_sdk::{api, Context, Result};
use core::ffi::c_void;
use core::sync::atomic::{AtomicPtr, Ordering};

#[global_allocator]
static AP_RUST_ALLOCATOR: ap_rust_sdk::Allocator = ap_rust_sdk::Allocator;

#[panic_handler]
fn ap_rust_panic(info: &core::panic::PanicInfo<'_>) -> ! {
    ap_rust_sdk::panic(info)
}

static HOST: AtomicPtr<api::HostApi> = AtomicPtr::new(core::ptr::null_mut());
static CONTEXT: AtomicPtr<c_void> = AtomicPtr::new(core::ptr::null_mut());

#[no_mangle]
pub unsafe extern "C" fn start(host: *const api::HostApi, context: *mut c_void) -> i32 {
    HOST.store(host as *mut api::HostApi, Ordering::Relaxed);
    CONTEXT.store(context, Ordering::Relaxed);
    ap_rust_sdk::enter(host, context, run)
}

fn write_code(context: &Context<'_>) -> Result {
    context.text(6, "m4 fault 14")?;
    // the Thumb bit off: the halfword-aligned address of start's first instruction
    let code = (start as *const () as usize & !1) as *mut u32;
    unsafe { core::ptr::write_volatile(code, 0) };
    Ok(())
}

fn run(context: &Context<'_>) -> Result {
    while !context.stop_requested() {
        let kind = context.param().get("RUST_M4_INJECT")?.unwrap_or(0.0);
        if kind == 14.0 {
            write_code(context)?;
        }
        context.sleep_ms(100)?;
    }
    Ok(())
}
