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
//! 14 is vehicle.get_mode every 5013 us, a period that is not a multiple of the 2500 us main loop.
//! 15-17 are ahrs.get_location at 10/s, 200/s, 1000/s; 18 is Location.offset at 200/s on a copy of
//! the location taken at the profile start. LATP_PROF 0 runs 1 to 18 in turn, all but 14. Each profile's first call starts at time_us() % 2500 ==
//! LATP_OFFS % 2500; the achieved phase is in a 'LATPH profile phase' text and the end LATP row (Phs).
#![no_std]
#![no_main]

use ap_rust_sdk::{enums::MavSeverity, Context, Error, Location, LogValue as V, Parameter, Result};

ap_rust_sdk::module!(run);

const TABLE_KEY: u8 = 77;
const PROFILE_COUNT: u32 = 18;
const BURST_PROFILE: u32 = 13;
const CONTROL_PROFILE: u32 = 14;
const CONTROL_PERIOD_US: u64 = 5013;
// the M7 main loop period (SCHED_LOOP_RATE 400) the start phase is taken against
const LOOP_US: u64 = 2500;
const RATES: [u32; 3] = [10, 200, 1000];
// short names: the LAT text must fit the 50 characters of a STATUSTEXT
const NAMES: [&str; 6] = ["param", "mode", "origin", "log", "getloc", "locoff"];
const OFFSET_BINDING: usize = 5;
// profiles after the control: (profile, binding, rate)
const LATER_PROFILES: [(u32, usize, u32); 4] = [(15, 4, 10), (16, 4, 200), (17, 4, 1000), (18, OFFSET_BINDING, 200)];
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
    // the Location.offset profile's start location
    location: Location,
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
            4 => self.c.ahrs().get_location().map(|location| {
                core::hint::black_box(location.is_some());
            }),
            OFFSET_BINDING => {
                let mut location = self.location;
                location.offset(self.c, 1.0, 1.0).map(|()| {
                    core::hint::black_box(location);
                })
            }
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
        self.call(binding_of(profile), tally)
    }
}

fn later(profile: u32) -> (u32, usize, u32) {
    LATER_PROFILES[(profile - CONTROL_PROFILE - 1) as usize]
}

// the binding of a single-binding profile
fn binding_of(profile: u32) -> usize {
    if profile < BURST_PROFILE {
        ((profile - 1) / 3) as usize
    } else if profile == CONTROL_PROFILE {
        1
    } else {
        later(profile).1
    }
}

fn rate_of(profile: u32) -> u32 {
    if profile == BURST_PROFILE {
        BURST_RATE
    } else if profile == CONTROL_PROFILE {
        (1_000_000 / CONTROL_PERIOD_US) as u32
    } else if profile < BURST_PROFILE {
        RATES[((profile - 1) % 3) as usize]
    } else {
        later(profile).2
    }
}

fn period_of(profile: u32) -> u64 {
    if profile == CONTROL_PROFILE {
        CONTROL_PERIOD_US
    } else {
        1_000_000 / u64::from(rate_of(profile))
    }
}

fn name_of(profile: u32) -> &'static str {
    if profile == BURST_PROFILE {
        "burst40"
    } else if profile == CONTROL_PROFILE {
        "mode5013"
    } else {
        NAMES[binding_of(profile)]
    }
}

// LATP row: profile, phase (0 start, 1 end), and at the end the module's own figures and the
// achieved start phase
fn log_profile(c: &Context<'_>, profile: u32, phase: u8, tally: &Tally, start_phase: u64) -> Result {
    let mean = if tally.calls != 0 { tally.total_us / u64::from(tally.calls) } else { 0 };
    match c.logger().write(
        "LATP",
        "Prof,Ph,Rate,N,Busy,Err,Mean,Max,Phs",
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
            V::U16(start_phase as u16),
        ],
    ) {
        Err(Error::Stopped) => Err(Error::Stopped),
        _ => Ok(()),
    }
}

// wait until time_us() % LOOP_US == offset: sleep whole milliseconds, spin the last 1-2 ms
fn align_phase(c: &Context<'_>, offset: u64) -> Result {
    loop {
        let now = c.time_us();
        let target = now + (offset + LOOP_US - now % LOOP_US) % LOOP_US;
        if target - now >= 2000 {
            c.sleep_ms(((target - now) / 1000 - 1) as u32)?;
            if c.time_us() > target {
                continue;   // overslept: take the next loop period
            }
        }
        while c.time_us() < target {}
        return Ok(());
    }
}

fn run_profile(probe: &mut Probe<'_, '_>, profile: u32, seconds: u32, offset: u64) -> Result {
    let c = probe.c;
    let mut tally = Tally::default();
    if profile != BURST_PROFILE && binding_of(profile) == OFFSET_BINDING {
        probe.location = match c.ahrs().get_location() {
            Err(Error::Stopped) => return Err(Error::Stopped),
            Ok(Some(location)) => location,
            _ => Location::default(),
        };
    }
    log_profile(c, profile, 0, &tally, 0)?;
    let period = period_of(profile);
    align_phase(c, offset)?;
    let start = c.time_us();
    let start_phase = start % LOOP_US;
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
    log_profile(c, profile, 1, &tally, start_phase)?;
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
    )?;
    // its own line: STATUSTEXT holds 50 characters, which the LAT line can fill
    c.gcs().send_text_fmt(MavSeverity::INFO, format_args!("LATPH {} {}", profile, start_phase))
}

fn setting(c: &Context<'_>, name: &str, default: u32) -> Result<u32> {
    Ok(match c.param().get(name)? {
        Some(value) if value >= 0.0 && value < 100000.0 => value as u32,
        _ => default,
    })
}

fn run(c: &Context<'_>) -> Result {
    if c.param().add_table(TABLE_KEY, "LATP_", 4)? {
        c.param().add_param(TABLE_KEY, 1, "PROF", 0.0)?;
        c.param().add_param(TABLE_KEY, 2, "SECS", 10.0)?;
        c.param().add_param(TABLE_KEY, 3, "WAIT", 5.0)?;
        c.param().add_param(TABLE_KEY, 4, "OFFS", 0.0)?;
    }
    let profile = setting(c, "LATP_PROF", 0)?;
    let seconds = setting(c, "LATP_SECS", 10)?.max(1);
    let wait = setting(c, "LATP_WAIT", 5)?;
    let offset = u64::from(setting(c, "LATP_OFFS", 0)?.min(5000)) % LOOP_US;
    let parameter = c.parameter("SCHED_LOOP_RATE")?.ok_or(Error::Unavailable)?;
    let mut probe = Probe { c, parameter, written: 0, location: Location::default() };
    c.sleep_ms(wait * 1000)?;
    // the cost of the module's own timing: one time_us() call
    let start = c.time_us();
    for _ in 0..100 {
        core::hint::black_box(c.time_us());
    }
    let clock_cost = (c.time_us() - start) / 100;
    c.gcs().send_text_fmt(
        MavSeverity::INFO,
        format_args!("LAT start prof {} {}s time_us {}us offs {}", profile, seconds, clock_cost, offset),
    )?;
    let (first, last) = if profile == 0 { (1, PROFILE_COUNT) } else { (profile, profile) };
    if first < 1 || last > PROFILE_COUNT {
        return Err(Error::Invalid);
    }
    for selected in first..=last {
        if profile == 0 && selected == CONTROL_PROFILE {
            continue;
        }
        run_profile(&mut probe, selected, seconds, offset)?;
    }
    c.gcs().send_text(MavSeverity::INFO, "LAT done")
}
