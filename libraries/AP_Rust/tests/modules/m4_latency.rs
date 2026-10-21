// SPDX-License-Identifier: GPL-3.0-or-later
//! Call latency probe load: one binding called in a loop at a fixed rate per profile, then a
//! burst of 40 calls per tick like an AirShow tick, each profile for LATP_SECS seconds, with one
//! GCS line and one LATP log row per profile. The stage times come from the call latency probe
//! (RUST_M4_OPTS bit 2 on the M4, RUST_DEBUG_OPTS bit 3 in process); this module times each call
//! from its own side with time_us(). Integer arithmetic only: the thumb build links no f64
//! software-float routine. Returns once its profiles have run.
//!
//! Profiles: 1-12 are binding (Parameter.get, vehicle.get_mode, ahrs.get_origin, logger.write)
//! times rate (10/s, 200/s, 1000/s): profile = 1 + 3 * binding + rate index. 13 is the burst.
//! LATP_PROF 0 runs 1 to 13 in turn.
#![no_std]
#![no_main]

use ap_rust_sdk::{enums::MavSeverity, Context, Error, LogValue as V, Parameter, Result};

ap_rust_sdk::module!(run);

const TABLE_KEY: u8 = 77;
const PROFILE_COUNT: u32 = 13;
const BURST_PROFILE: u32 = 13;
const RATES: [u32; 3] = [10, 200, 1000];
const NAMES: [&str; 4] = ["param", "mode", "origin", "log"];
// the burst: 40 calls a tick at 30 Hz, Parameter.get first as the AirShow's tick does
const BURST_CALLS: [(usize, u32); 4] = [(0, 22), (1, 6), (2, 6), (3, 6)];
const BURST_RATE: u32 = 30;
// a profile that falls behind catches up by at most this many calls a wake
const CATCH_UP: u32 = 4;

#[derive(Default)]
struct Tally {
    calls: u32,
    busy: u32,
    errors: u32,
    total_us: u64,
    max_us: u64,
}

struct Probe<'a, 'h> {
    c: &'a Context<'h>,
    parameter: Parameter<'a, 'h>,
    written: u32,
}

impl Probe<'_, '_> {
    // one call of binding, timed from this side; Busy (duty-cap refusal) and other errors counted
    fn call(&mut self, binding: usize, tally: &mut Tally) -> Result {
        let start = self.c.time_us();
        let result = match binding {
            0 => self.parameter.get().map(|value| {
                core::hint::black_box(value);
            }),
            1 => self.c.vehicle().get_mode().map(|mode| {
                core::hint::black_box(mode);
            }),
            2 => self.c.ahrs().get_origin().map(|origin| {
                core::hint::black_box(origin.is_some());
            }),
            _ => {
                self.written = self.written.wrapping_add(1);
                self.c.logger().write("LATW", "N", None, &[V::U32(self.written)])
            }
        };
        let elapsed = self.c.time_us().saturating_sub(start);
        tally.calls += 1;
        tally.total_us += elapsed;
        tally.max_us = tally.max_us.max(elapsed);
        match result {
            Ok(()) => Ok(()),
            Err(Error::Stopped) => Err(Error::Stopped),
            Err(Error::Busy) => {
                tally.busy += 1;
                Ok(())
            }
            Err(_) => {
                tally.errors += 1;
                Ok(())
            }
        }
    }

    // one tick of the profile: one call, or the burst
    fn tick(&mut self, profile: u32, tally: &mut Tally) -> Result {
        if profile == BURST_PROFILE {
            for (binding, count) in BURST_CALLS {
                for _ in 0..count {
                    self.call(binding, tally)?;
                }
            }
            return Ok(());
        }
        self.call(((profile - 1) / 3) as usize, tally)
    }
}

fn rate_of(profile: u32) -> u32 {
    if profile == BURST_PROFILE {
        BURST_RATE
    } else {
        RATES[((profile - 1) % 3) as usize]
    }
}

fn name_of(profile: u32) -> &'static str {
    if profile == BURST_PROFILE {
        "burst40"
    } else {
        NAMES[((profile - 1) / 3) as usize]
    }
}

// LATP row: profile, phase (0 start, 1 end), and at the end the module's own figures
fn log_profile(c: &Context<'_>, profile: u32, phase: u8, tally: &Tally) -> Result {
    let mean = if tally.calls != 0 { tally.total_us / u64::from(tally.calls) } else { 0 };
    match c.logger().write(
        "LATP",
        "Prof,Ph,Rate,N,Busy,Err,Mean,Max",
        None,
        &[
            V::U8(profile as u8),
            V::U8(phase),
            V::U16(rate_of(profile) as u16),
            V::U32(tally.calls),
            V::U32(tally.busy),
            V::U32(tally.errors),
            V::U32(mean.min(u64::from(u32::MAX)) as u32),
            V::U32(tally.max_us.min(u64::from(u32::MAX)) as u32),
        ],
    ) {
        Err(Error::Stopped) => Err(Error::Stopped),
        _ => Ok(()),
    }
}

fn run_profile(probe: &mut Probe<'_, '_>, profile: u32, seconds: u32) -> Result {
    let c = probe.c;
    let mut tally = Tally::default();
    log_profile(c, profile, 0, &tally)?;
    let period = 1_000_000 / u64::from(rate_of(profile));
    let start = c.time_us();
    let end = start + u64::from(seconds) * 1_000_000;
    let mut next = start;
    loop {
        if c.stop_requested() {
            return Err(Error::Stopped);
        }
        let now = c.time_us();
        if now >= end {
            break;
        }
        if now < next {
            let remaining = next - now;
            // sleep whole milliseconds; the last one is yielded away
            c.sleep_ms(if remaining >= 2000 { (remaining / 1000 - 1) as u32 } else if remaining >= 1000 { 1 } else { 0 })?;
            continue;
        }
        let mut ticks = 0;
        while next <= c.time_us() && ticks < CATCH_UP {
            probe.tick(profile, &mut tally)?;
            next += period;
            ticks += 1;
        }
        if next <= c.time_us() {
            next = c.time_us() + period;   // too far behind: the rate is what the calls allow
        }
    }
    log_profile(c, profile, 1, &tally)?;
    let elapsed_ms = (c.time_us() - start) / 1000;
    let per_second = if elapsed_ms != 0 { u64::from(tally.calls) * 1000 / elapsed_ms } else { 0 };
    let mean = if tally.calls != 0 { tally.total_us / u64::from(tally.calls) } else { 0 };
    c.gcs().send_text_fmt(
        MavSeverity::INFO,
        format_args!(
            "LAT {} {} {}/s n{} b{} e{} avg{} max{}",
            profile,
            name_of(profile),
            per_second,
            tally.calls,
            tally.busy,
            tally.errors,
            mean,
            tally.max_us
        ),
    )
}

fn setting(c: &Context<'_>, name: &str, default: u32) -> Result<u32> {
    Ok(match c.param().get(name)? {
        Some(value) if value >= 0.0 && value < 100000.0 => value as u32,
        _ => default,
    })
}

fn run(c: &Context<'_>) -> Result {
    if c.param().add_table(TABLE_KEY, "LATP_", 3)? {
        c.param().add_param(TABLE_KEY, 1, "PROF", 0.0)?;
        c.param().add_param(TABLE_KEY, 2, "SECS", 10.0)?;
        c.param().add_param(TABLE_KEY, 3, "WAIT", 5.0)?;
    }
    let profile = setting(c, "LATP_PROF", 0)?;
    let seconds = setting(c, "LATP_SECS", 10)?.max(1);
    let wait = setting(c, "LATP_WAIT", 5)?;
    let parameter = c.parameter("SCHED_LOOP_RATE")?.ok_or(Error::Unavailable)?;
    let mut probe = Probe { c, parameter, written: 0 };
    c.sleep_ms(wait * 1000)?;
    // the cost of the module's own timing: one time_us() call
    let start = c.time_us();
    for _ in 0..100 {
        core::hint::black_box(c.time_us());
    }
    let clock_cost = (c.time_us() - start) / 100;
    c.gcs().send_text_fmt(
        MavSeverity::INFO,
        format_args!("LAT start prof {} {}s time_us {}us", profile, seconds, clock_cost),
    )?;
    let (first, last) = if profile == 0 { (1, PROFILE_COUNT) } else { (profile, profile) };
    if first < 1 || last > PROFILE_COUNT {
        return Err(Error::Invalid);
    }
    for selected in first..=last {
        run_profile(&mut probe, selected, seconds)?;
    }
    c.gcs().send_text(MavSeverity::INFO, "LAT done")
}
