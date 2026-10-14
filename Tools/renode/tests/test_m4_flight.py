# AP_FLAKE8_CLEAN
"""Renode flights of the CubeOrangePlus-M4Lua Copter with Lua on the M4: the real M7 and M4
images fly an AUTO mission against the physics sidecar (Tools/renode/renode_physics.cpp), as the
CubeOrangePlus QuadPlane flight of test_physics_flight.py does, with a script running on the M4.
Functional only: Renode is not cycle-accurate.

Needs RENODE, a CubeOrangePlus-M4Lua build and the physics sidecar:
  ./waf configure --board CubeOrangePlus-M4Lua && ./waf copter
  ./waf configure --board sitl --enable-scripting-m4 && ./waf --targets tool/renode-physics
"""

import math
import os
import re
import subprocess
import sys
import time

from pathlib import Path

import pytest
from pymavlink import DFReader, mavutil

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import fat_image  # noqa: E402
from test_m4_containment import ROOT, STEP_TIMEOUT, start_m4lua  # noqa: E402,F401
from test_m4_scripts import (SCRIPT_TIMEOUT, START_TIMEOUT, host_thread_stack, upload_files,  # noqa: E402
                             upload_scripts)
from test_mission import (FORCE_ARM_MAGIC, distance_metres, offset_location, send_command,  # noqa: E402
                          set_mode, set_parameter, stop_process_group, unused_tcp_port, upload_mission)
from test_physics_flight import (CANBERRA, CUBEORANGEPLUS_SENSOR_IDS, PHYSICS_RATE_HZ,  # noqa: E402
                                 check_sensor_ids, check_sidecar, ordered_subsequence, wait_for_sidecar,
                                 wait_ready)

PHYSICS = Path(os.environ.get('M4LUA_PHYSICS', ROOT / 'build' / 'sitl' / 'tool' / 'renode-physics'))
PHYSICS_MODEL = 'x'          # SITL quad X: FRAME_CLASS 1, FRAME_TYPE 1
WALL_PER_EMULATED = 20       # wall seconds per emulated second: about 4x the slowest flight seen (5.2x)
LEG_TIMEOUT = 120 * WALL_PER_EMULATED   # wall seconds for a flight leg (the climb, the landing): 2 emulated minutes
FLIGHT_DEADLINE = 3600       # wall seconds for a flight test (start_m4lua's 2700 s is for ground tests)
COPTER_AUTO = 3
HOLD_ITEM, RELEASE_ITEM, LAND_ITEM = 3, 4, 6
HOLD_SECONDS = 10            # emulated seconds the vehicle holds at the LOITER_UNLIM before the release
ARM_SCRIPTING = 4            # AP_Arming::Method::SCRIPTING: arming:arm() from the script
DISARM_LANDED = 13           # AP_Arming::Method::LANDED: the land detector, not a crash or a failsafe
MAX_LEAN_DEG = 30            # Copter's default ANGLE_MAX: the controller never asks for more
LANDING_RADIUS_M = 1         # as SITL: the LAND item is at home
CRASH_PATTERN = r'Crash|[Ff]ailsafe|^Lua: '
CRASH_TEXT = re.compile(CRASH_PATTERN)
FLIGHT_FAILURE_TEXT = re.compile(CRASH_PATTERN + r'|^Scripting: (stopped|restarted)')
M4_PC = r'PC=0x081[C-F][0-9A-F]{4}$'
FAULT_STEPS = (
    (1, r'^Scripting: M4 HardFault ' + M4_PC),   # the service: an undefined instruction
    (9, r'^Scripting: M4 MemManage ' + M4_PC),   # the Lua thread: a stray RCC_GCR write
    (5, r'^Scripting: M4 protocol errors$'),     # ring corruption
)
FAULT_LIMIT_TEXT = 'Scripting: M4 fault limit, off until reboot'
HOLD_CHECK_SECONDS = 5       # emulated seconds of the hold check after each reload
# binding_bench.lua: each update calls every case BENCH_CALLS times; a report every 10 s
BENCH_CASES = ('ahrs:get_location', 'vehicle:get_mode', 'param:get', 'millis', 'gps:status',
               'battery:voltage', 'SRV_Channels:get_output_pwm', 'mount:get_mode')
BENCH_CALLS = 50

# set over MAVLink on a first launch (never built in): Renode supplies the flight state and the
# sensors, not RC, the safety switch, the power monitor or calibration. As the QuadPlane flight,
# with the KakuteF4 Copter flight's tune
FLIGHT_PARAMETERS = {
    'ARMING_SKIPCHK': -1,          # forced arm from the GCS: the always-on checks and arming only
    'BATT_MONITOR': 0,
    'BRD_SAFETY_DEFLT': 0,
    'BRD_HEAT_TARG': -1,           # emulated IMUs never warm up
    'RC_PROTOCOLS': 0,
    'FS_THR_ENABLE': 0,
    'GPS1_TYPE': 1,
    'SERIAL2_PROTOCOL': 5,         # the emulated u-blox is on SERIAL2
    'AUTO_OPTIONS': 3,             # arm and take off in AUTO
    'FRAME_CLASS': 1,
    'FRAME_TYPE': 1,
    'SCHED_LOOP_RATE': 300,        # as the QuadPlane flight: a rate the emulation sustains
    'WP_SPD': 3,
    'WP_ACC': 0.5,
    'WP_YAW_BEHAVIOR': 0,
    # standard SITL multicopter tune
    'ATC_ANG_RLL_P': 2,
    'ATC_ANG_PIT_P': 2,
    'ATC_RAT_RLL_P': 0.05,
    'ATC_RAT_RLL_I': 0.05,
    'ATC_RAT_RLL_D': 0.001,
    'ATC_RAT_PIT_P': 0.05,
    'ATC_RAT_PIT_I': 0.05,
    'ATC_RAT_PIT_D': 0.001,
    'ATC_RAT_YAW_P': 0.3,
    'ATC_RAT_YAW_I': 0.02,
    'MOT_THST_EXPO': 0.65,
    'MOT_THST_HOVER': 0.39,
    'MOT_BAT_VOLT_MIN': 9.6,
    'MOT_BAT_VOLT_MAX': 12.8,
    # calibrated simulated sensors
    'INS_ACCOFFS_X': 0.001,
    'INS_ACCOFFS_Y': 0.001,
    'INS_ACCOFFS_Z': 0.001,
    'INS_ACCSCAL_X': 1.001,
    'INS_ACCSCAL_Y': 1.001,
    'INS_ACCSCAL_Z': 1.001,
    'INS_ACC2OFFS_X': 0.001,
    'INS_ACC2OFFS_Y': 0.001,
    'INS_ACC2OFFS_Z': 0.001,
    'INS_ACC2SCAL_X': 1.001,
    'INS_ACC2SCAL_Y': 1.001,
    'INS_ACC2SCAL_Z': 1.001,
    'INS_GYR_CAL': 0,
    'INS_FAST_SAMPLE': 0,
    'COMPASS_OFS_X': -0.453570,
    'COMPASS_OFS_Y': -0.585846,
    'COMPASS_OFS_Z': 6.815743,
    'LOG_DISARMED': 1,
    'LOG_DARM_RATEMAX': 0,
    'LOG_FILE_RATEMAX': 0,
    'SCR_DEBUG_OPTS': 8,           # LOG_RUNTIME: SCR and SRPC rows
}


@pytest.fixture
def start_m4_flight(start_m4lua, tmp_path):  # noqa: F811
    '''start() launches the board with a physics sidecar of its own; both stop at the end.'''
    if not PHYSICS.is_file():
        pytest.skip('build the physics sidecar: %s' % PHYSICS)
    start_m4lua.set_deadline(FLIGHT_DEADLINE)   # replaces start_m4lua's ground-test deadline
    sidecars = []
    runs = []

    def start():
        port = unused_tcp_port()
        log_path = tmp_path / ('physics-%u.log' % len(sidecars))
        with log_path.open('w') as log:
            process = subprocess.Popen([str(PHYSICS), '--physics-port', str(port), '--model', PHYSICS_MODEL],
                                       cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        sidecars.append(process)
        wait_for_sidecar(process, log_path, port)
        extra = ['--imu', 'icm42688_ext', '--imu', 'icm20948_ext', '--imu', 'icm20649',
                 '--device', '{"device":"ublox-gps","port":"SERIAL2"}',
                 '--exec', 'sysbus.physics Connect %u "%s" %.7f %.7f %.1f %.1f %u' % (
                     port, PHYSICS_MODEL, *CANBERRA, PHYSICS_RATE_HZ)]
        run = start_m4lua(extra_args=extra, extra_check=lambda: check_sidecar(process, log_path))
        run.physics = (process, log_path)
        runs.append(run)
        return run

    try:
        yield start
    finally:
        # stop every Renode, then every sidecar, even if one stop fails; report the first failure
        errors = []
        for stop in [run.stop for run in runs] + [lambda process=process: stop_process_group(process)
                                                  for process in sidecars]:
            try:
                stop()
            except Exception as error:
                errors.append(error)
        if errors:
            raise errors[0]


def set_flight_parameter(run, name, value, attempts=3):
    '''Sets a parameter; setting one element of a vector parameter (COMPASS_OFS_X) also sends the
    other elements' old values, which the next set can take for its own reply: set it again.'''
    for attempt in range(attempts):
        try:
            run.set(name, value)
            return
        except RuntimeError as error:
            if 'instead of' not in str(error) or attempt == attempts - 1:
                raise


def start_flight(start, scripts, parameters, files=()):
    '''Sets the parameters and uploads the scripts on a first launch, then launches again: the frame,
    the GPS port and the safety switch act at boot. Values are compared as float32.'''
    first = start()
    for name, value in parameters.items():
        set_flight_parameter(first, name, value)
    first.wait_text(r'^Scripting: started on M4 \(0 scripts, heap \d+KB\)$', timeout=START_TIMEOUT)
    upload_scripts(first, scripts)
    if files:
        upload_files(first, files)
    first.stop()
    run = start()
    for name, value in parameters.items():
        got = run.get(name)
        assert math.isclose(got, value, rel_tol=1e-6, abs_tol=1e-6), \
            'board state did not persist across the relaunch: %s=%s, set %s' % (name, got, value)
    run.wait_m4_started()
    return run


def navigation_ready(run):
    '''Waits for GPS and EKF, checks the sensors are a real CubeOrangePlus's; returns home.'''
    deadline = time.monotonic() + LEG_TIMEOUT
    physics, physics_log = run.physics
    home = wait_ready(run.mav, run.process, run.log_path, physics, physics_log, deadline)
    check_sensor_ids(run.mav, run.process, run.log_path, deadline, CUBEORANGEPLUS_SENSOR_IDS)
    return home


def flight_mission(home_lat, home_lon):
    '''0 home, 1 take off to 10 m, 2 a waypoint 10 m north, 3 LOITER_UNLIM there (m4_mission_flight.lua
    moves the mission on), 4 and 5 two waypoints, 6 LAND at home.'''
    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    waypoint = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT

    def item(command, north, east, altitude):
        lat, lon = offset_location(home_lat, home_lon, north, east)
        return (frame, command, 0, 1, 0, 0, 0, float('nan'), lat, lon, altitude)

    return [(frame, waypoint, 0, 1, 0, 0, 0, 0, home_lat, home_lon, 0),
            item(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 10),
            item(waypoint, 10, 0, 10),
            item(mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 10, 0, 10),
            item(waypoint, 10, 10, 10),
            item(waypoint, 0, 10, 10),
            item(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0)]


def harness_mission(home_lat, home_lon):
    '''0 home, 1 take off to 10 m, 2 a waypoint 10 m north, 3 LAND at home.'''
    items = flight_mission(home_lat, home_lon)
    return items[:3] + items[-1:]


class FlightTrack:
    '''What the vehicle reports while a test waits: armed, mode, mission item, height, position,
    the item it was on when it came down below 1 m, and the mode and height at the disarm.'''

    def __init__(self, run):
        self.armed = False
        self.was_armed = False
        self.mode = None
        self.current = None
        self.currents = []        # each new MISSION_CURRENT seq, in order
        self.reached = set()
        self.altitude = 0.0       # relative, m
        self.max_altitude = 0.0
        self.position = None      # (lat, lon) in degrees
        self.touchdown_item = None   # the mission item when the height first fell below 1 m after the climb
        self.disarm = None        # (mode, height, index into run.texts) at the first disarm after arming
        self.item_heights = {}    # mission item: [lowest, highest] height while armed on it, m
        self._run = run
        self._printed_altitude = 0.0
        run.mav.message_hooks.append(self._record)

    def _record(self, connection, message):
        kind = message.get_type()
        if kind == 'HEARTBEAT' and message.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed != self.armed:
                print('armed' if armed else 'disarmed', flush=True)
            if self.armed and not armed and self.disarm is None:
                self.disarm = (message.custom_mode, self.altitude, len(self._run.texts))
            self.armed = armed
            self.was_armed |= armed
            self.mode = message.custom_mode
        elif kind == 'MISSION_CURRENT' and message.seq != self.current:
            self.current = message.seq
            self.currents.append(message.seq)
            print('mission item %u' % message.seq, flush=True)
        elif kind == 'MISSION_ITEM_REACHED':
            self.reached.add(message.seq)
        elif kind == 'GLOBAL_POSITION_INT':
            self.altitude = message.relative_alt * 0.001
            self.max_altitude = max(self.max_altitude, self.altitude)
            self.position = (message.lat * 1.0e-7, message.lon * 1.0e-7)
            if self.armed and self.current is not None:
                heights = self.item_heights.setdefault(self.current, [self.altitude, self.altitude])
                heights[0], heights[1] = min(heights[0], self.altitude), max(heights[1], self.altitude)
            if self.armed and self.max_altitude >= 8 and self.altitude < 1 and self.touchdown_item is None:
                self.touchdown_item = self.current
            if abs(self.altitude - self._printed_altitude) >= 2:
                self._printed_altitude = self.altitude
                print('altitude %.1f m' % self.altitude, flush=True)


def wait_until(run, condition, timeout, what):
    deadline = time.monotonic() + timeout
    while not condition():
        run.check()
        if time.monotonic() > deadline:
            raise RuntimeError('timed out waiting for %s; last texts %s' % (what, run.texts[-15:]))
        run.mav.recv_match(blocking=True, timeout=1)


def read_logs(run, directory):
    '''Stops Renode, which closes the logs, and returns every DataFlash log of the SD card image.'''
    run.stop()
    logs = fat_image.extract_files(run.sdcard_image, '/APM/LOGS', '*.BIN', directory)
    assert logs, 'no DataFlash log on the SD card'
    return sorted(logs, key=lambda log: log.name)


def log_rows(logs, name):
    rows = []
    for log in logs:
        reader = DFReader.DFReader_binary(str(log), zero_time_base=True)
        while True:
            row = reader.recv_match(type=name)
            if row is None:
                break
            rows.append(row)
    return rows


def supervisor_texts(run, start=0):
    '''The supervisor's fault, restart and off texts from start on; the start text the fixture
    waits for is not one of them.'''
    return [text for text in run.texts[start:]
            if text.startswith('Scripting: M4') and text != 'Scripting: M4 started']


def armed_rows(logs, name):
    '''Rows of the named message logged while armed, and each disarm's ARM.Method.'''
    rows, methods = [], []
    for log in logs:
        reader = DFReader.DFReader_binary(str(log), zero_time_base=True)
        armed = False
        while True:
            row = reader.recv_match(type=[name, 'ARM'])
            if row is None:
                break
            if row.get_type() == 'ARM':
                if armed and not row.ArmState:
                    methods.append(row.Method)
                armed = bool(row.ArmState)
            elif armed:
                rows.append(row)
    return rows, methods


def check_m4_kept_running(logs):
    '''In each log, once M4ST shows the M4 running (St 2) it stays running with no new fault
    and no restart.'''
    for log in logs:
        rows = log_rows([log], 'M4ST')
        running = [index for index, row in enumerate(rows) if row.St == 2]
        assert running, 'M4ST never shows the M4 running in %s' % log.name
        first = rows[running[0]]
        changed = [(row.TimeUS, row.St, row.Flt, row.Rst) for row in rows[running[0]:]
                   if row.St != 2 or row.Flt != first.Flt or row.Rst != 0]
        assert not changed, 'M4ST (TimeUS, St, Flt, Rst) after the start in %s: %s' % (log.name, changed[:5])


def test_m4_flight_harness(start_m4_flight, tmp_path):
    '''The M4Lua Copter arms (forced, from the GCS) and flies a short AUTO mission with the M4 running
    a script; the M4 neither faults nor stops.'''
    run = start_flight(start_m4_flight, ['tests/m4_loop.lua'], FLIGHT_PARAMETERS)
    started = len(run.texts)
    home = navigation_ready(run)
    home_lat, home_lon = home.lat * 1.0e-7, home.lon * 1.0e-7
    upload_mission(run.mav, run.process, run.log_path, harness_mission(home_lat, home_lon), timeout=STEP_TIMEOUT)
    track = FlightTrack(run)
    set_mode(run.mav, run.process, run.log_path, 'AUTO', timeout=STEP_TIMEOUT)
    wall, emulated_ms = time.monotonic(), run.boot_ms
    send_command(run.mav, run.process, run.log_path, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                 (1, FORCE_ARM_MAGIC), timeout=STEP_TIMEOUT)
    texts_at_arm = len(run.texts)
    wait_until(run, lambda: track.was_armed and not track.armed, LEG_TIMEOUT, 'the landing')
    seconds = (run.boot_ms - emulated_ms) / 1000.0
    print('flight: %.0f s emulated in %.0f s of wall time (%.1fx)' % (
        seconds, time.monotonic() - wall, (time.monotonic() - wall) / max(seconds, 1)), flush=True)
    landing = distance_metres(home_lat, home_lon, *track.position)
    disarm_mode, disarm_altitude, texts_at_disarm = track.disarm
    print('flight: %.1f m high, items %s, reached %s, down at item %s, disarmed in mode %u at %.1f m, '
          '%.1f m from home' % (track.max_altitude, track.currents, sorted(track.reached), track.touchdown_item,
                                disarm_mode, disarm_altitude, landing), flush=True)
    assert track.max_altitude >= 8, 'reached only %.1f m' % track.max_altitude
    assert 2 in track.reached, 'mission items reached: %s' % sorted(track.reached)
    assert ordered_subsequence(track.currents, [1, 2, 3]), 'mission items %s' % track.currents
    assert track.touchdown_item == 3, 'came down below 1 m at item %s, not the LAND' % track.touchdown_item
    assert disarm_mode == COPTER_AUTO, 'disarmed in mode %u, not AUTO' % disarm_mode
    assert disarm_altitude < 1, 'disarmed %.1f m up' % disarm_altitude
    assert landing <= LANDING_RADIUS_M, 'landed %.1f m from home' % landing
    # m4_loop.lua prints once a second: it ran through the flight and still runs after the landing
    loops = [text for text in run.texts[texts_at_arm:texts_at_disarm] if re.match(r'^M4 loop \d+$', text)]
    assert len(loops) >= max(5, 0.5 * seconds), \
        'the M4 script printed %u times in %.0f s of flight' % (len(loops), seconds)
    wait_until(run, lambda: any(re.match(r'^M4 loop \d+$', text) for text in run.texts[texts_at_disarm:]),
               STEP_TIMEOUT, 'an M4 loop text after the landing')
    run.wait_alive(10)
    failures = [text for text in run.texts[started:] if FLIGHT_FAILURE_TEXT.search(text)]
    assert not failures, failures
    assert not supervisor_texts(run, started), supervisor_texts(run, started)
    logs = read_logs(run, tmp_path / 'logs')
    check_m4_kept_running(logs)
    attitudes, disarms = armed_rows(logs, 'ATT')
    assert attitudes, 'no ATT rows while armed: the flight was not logged'
    lean = max(max(abs(row.Roll), abs(row.Pitch)) for row in attitudes)
    print('flight: largest roll or pitch %.1f deg over %u ATT rows' % (lean, len(attitudes)), flush=True)
    assert lean < MAX_LEAN_DEG, 'leaned %.1f deg in flight' % lean
    assert disarms == [DISARM_LANDED], 'disarm methods %s, not the land detector (%u)' % (disarms, DISARM_LANDED)


def script_counts(texts):
    '''The "M4 count <name> <n>" lines and "M4 total <n>" of m4_mission_flight.lua.'''
    counts = {}
    total = None
    for text in texts:
        match = re.match(r'^M4 count (\S+) (\d+)$', text)
        if match:
            counts[match.group(1)] = int(match.group(2))
        match = re.match(r'^M4 total (\d+)$', text)
        if match:
            total = int(match.group(1))
    return counts, total


def check_mission_counts(counts, total, totals):
    '''The rules ScriptingM4MissionFlight applies in SITL: SRPC holds every binding call the script
    made (rules 1 and 2), and the script's own per-update identities hold (rules 3 to 5).'''
    missing = [name for name in ('vehicle:get_mode', 'mission:get_current_nav_index', 'param:get',
                                 'mission:set_current_cmd') if name not in counts]
    assert not missing and total is not None, 'script count lines missing: %s, total %s' % (missing, total)
    for name, count in counts.items():
        assert totals.get(name, 0) <= count, 'SRPC %s: %u logged, the script made %u' % (name, totals[name], count)
    logged = sum(n for name, n in totals.items()
                 if name == 'm4:other' or not (name.startswith('m4:') or name == 'global:print'))
    print('SRPC binding calls logged %u, script made %u' % (logged, total), flush=True)
    assert logged == total, 'SRPC binding calls: %u logged, the script made %u (%s)' % (logged, total, totals)
    check_per_update(counts)
    assert counts.get('arming:arm', 0) == 1, counts


def check_per_update(counts):
    '''m4_mission_flight.lua's per-update identities: one get_mode, get_current_nav_index and
    param:get an update, two param:get at load, one release.'''
    updates = counts['vehicle:get_mode']
    assert counts['mission:get_current_nav_index'] == updates, counts
    assert counts['param:get'] == updates + 2, counts
    assert counts['mission:set_current_cmd'] == 1, counts


def request_message_rate(run, message_id, rate_hz):
    run.mav.mav.command_long_send(run.mav.target_system, run.mav.target_component,
                                  mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                  message_id, 1.0e6 / rate_hz, 0, 0, 0, 0, 0)


def check_holding(run, track, item, seconds):
    '''The vehicle stays armed in AUTO at mission item item, 8-12 m up, for seconds of emulated
    time, checked on every message received. MISSION_CURRENT (10 Hz requested, 4 Hz from the
    stream) is the only one that refreshes the item: at least 2 a second must arrive.'''
    start = run.boot_ms
    deadline = time.monotonic() + seconds * WALL_PER_EMULATED
    samples = currents = 0
    while run.boot_ms - start < seconds * 1000:
        run.check()
        assert time.monotonic() < deadline, 'M7 time did not advance %u s from %u ms' % (seconds, start)
        message = run.mav.recv_match(type=['MISSION_CURRENT', 'GLOBAL_POSITION_INT', 'HEARTBEAT'],
                                     blocking=True, timeout=1)
        if message is None:
            continue
        samples += 1
        currents += message.get_type() == 'MISSION_CURRENT'
        assert (track.current == item and track.armed and track.mode == COPTER_AUTO and
                8 <= track.altitude <= 12), 'not holding item %u: current %s, armed %s, mode %s, %.1f m' % (
                    item, track.current, track.armed, track.mode, track.altitude)
    print('held item %u for %u s: %u samples, %u MISSION_CURRENT' % (item, seconds, samples, currents), flush=True)
    assert currents >= 2 * seconds, 'only %u MISSION_CURRENT in %u s of the hold' % (currents, seconds)


def test_m4_mission_flight(start_m4_flight, tmp_path):
    '''m4_mission_flight.lua on the M4 arms the vehicle in AUTO; the vehicle holds at its LOITER_UNLIM
    until SCR_USER2 lets the script move the mission on; it lands at home; SRPC holds the script's
    calls, as in SITL.'''
    parameters = dict(FLIGHT_PARAMETERS, SCR_USER1=HOLD_ITEM, SCR_USER2=0, SCR_USER3=1)
    run = start_flight(start_m4_flight, ['tests/m4_mission_flight.lua'], parameters)
    started = len(run.texts)
    run.wait_text(r'^Scripting: started on M4 \(1 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    home = navigation_ready(run)
    home_lat, home_lon = home.lat * 1.0e-7, home.lon * 1.0e-7
    upload_mission(run.mav, run.process, run.log_path, flight_mission(home_lat, home_lon), timeout=STEP_TIMEOUT)
    track = FlightTrack(run)
    request_message_rate(run, mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT, 10)
    set_mode(run.mav, run.process, run.log_path, 'AUTO', timeout=STEP_TIMEOUT)
    # the script, not the GCS, arms the vehicle
    assert run.wait_text(r'^M4 mission armed ', timeout=STEP_TIMEOUT) == 'M4 mission armed true'
    wait_until(run, lambda: track.current == HOLD_ITEM, LEG_TIMEOUT, 'the hold item')
    # with SCR_USER2 0 the script leaves the LOITER_UNLIM alone: the vehicle holds there
    check_holding(run, track, HOLD_ITEM, HOLD_SECONDS)
    released = [text for text in run.texts if text.startswith('M4 mission release')]
    assert not released, released
    set_parameter(run.mav, run.process, run.log_path, 'SCR_USER2', RELEASE_ITEM, timeout=STEP_TIMEOUT)
    assert run.wait_text(r'^M4 mission release ', timeout=STEP_TIMEOUT) == 'M4 mission release 3 to 4 true'
    wait_until(run, lambda: track.current == RELEASE_ITEM, STEP_TIMEOUT, 'the release')
    run.wait_text(r'^M4 mission done$', timeout=STEP_TIMEOUT)
    counts, total = script_counts(run.texts)
    print('script counts %s, total %s' % (counts, total), flush=True)
    wait_until(run, lambda: track.was_armed and not track.armed, LEG_TIMEOUT, 'the landing')
    landing = distance_metres(home_lat, home_lon, *track.position)
    disarm_mode, disarm_altitude, _ = track.disarm
    print('flight: %.1f m high, items %s, down at item %s, disarmed in mode %u at %.1f m, %.1f m from home' % (
        track.max_altitude, track.currents, track.touchdown_item, disarm_mode, disarm_altitude, landing), flush=True)
    assert track.max_altitude >= 8, 'reached only %.1f m' % track.max_altitude
    # the mission goes back to item 1 at the disarm, so the landing item is the one at touchdown
    assert ordered_subsequence(track.currents, [HOLD_ITEM, RELEASE_ITEM, 5, LAND_ITEM]), \
        'mission items %s' % track.currents
    assert track.touchdown_item == LAND_ITEM, 'came down below 1 m at item %s, not the LAND' % track.touchdown_item
    assert disarm_mode == COPTER_AUTO, 'disarmed in mode %u, not AUTO' % disarm_mode
    assert disarm_altitude < 1, 'disarmed %.1f m up' % disarm_altitude
    assert landing <= LANDING_RADIUS_M, 'landed %.1f m from home' % landing
    # the script's last window is logged within two summaries
    for _ in range(2):
        run.wait_text(r'^Scripting RPC \d+/s p50<\d+ p99<\d+ max \d+us$', timeout=STEP_TIMEOUT)
    # arming:arm() and set_current_cmd() ran on the host thread: it keeps 2 KB of its stack. With
    # ARMING_SKIPCHK -1 the arm runs the always-on checks and arming only: AP_Arming::arm_checks,
    # and with it PrepForArming's log-open wait, is skipped
    free, size = host_thread_stack(run, tmp_path / 'threads.txt')
    print('ScriptingM4 host thread stack after a successful arm (always-on checks and arming, '
          'no log-open wait): %u of %u bytes free' % (free, size), flush=True)
    assert free >= 2048, 'ScriptingM4 host thread stack: %u of %u bytes free' % (free, size)
    run.wait_alive(10)
    failures = [text for text in run.texts[started:] if FLIGHT_FAILURE_TEXT.search(text)]
    assert not failures, failures
    assert not supervisor_texts(run, started), supervisor_texts(run, started)
    logs = read_logs(run, tmp_path / 'logs')
    # the count rules apply to the flight's log only: the first launch's log has its own
    # 0-script session, whose m4:config and m4:bindings rows would hide a dropped first window
    flight_logs = [log for log in logs if any(row.ArmState for row in log_rows([log], 'ARM'))]
    assert len(flight_logs) == 1, 'logs with an arm: %s' % [log.name for log in flight_logs]
    totals = {}
    for row in log_rows(flight_logs, 'SRPC'):
        totals[row.Name] = totals.get(row.Name, 0) + row.N
    print('SRPC totals %s' % totals, flush=True)
    # preconditions of the count rules, each naming its own cause: the logger drops rows from other
    # threads until its startup header is written, so a lost window is reported as rows dropped
    missing = [name for name in ('m4:config', 'm4:bindings') if totals.get(name, 0) < 1]
    assert not missing, 'rows dropped: no SRPC %s rows, the session\'s first window was not logged' % missing
    dropped = max(row.Dp for row in log_rows(flight_logs, 'DSF'))
    print('logger buffer drops (DSF.Dp) %u' % dropped, flush=True)
    assert dropped == 0, 'rows dropped: the logger dropped %u rows (DSF.Dp)' % dropped
    m4st = log_rows(flight_logs, 'M4ST')
    lost, refused = sum(row.Lost for row in m4st), sum(row.QHit for row in m4st)
    assert lost == 0 and refused == 0, 'M4 calls lost %u, refused by quota %u' % (lost, refused)
    check_mission_counts(counts, total, totals)
    # SCR rows carry the script name cut to the log's 16 characters
    name = 'm4_mission_flight.lua'[:16]
    scripts = set(row.Name for row in log_rows(flight_logs, 'SCR'))
    assert name in scripts, 'SCR rows for %s' % sorted(scripts)
    check_m4_kept_running(logs)
    assert sum(row.PErr for row in m4st) == 0, 'M4ST protocol errors'
    attitudes, disarms = armed_rows(logs, 'ATT')
    lean = max(max(abs(row.Roll), abs(row.Pitch)) for row in attitudes)
    print('flight: largest roll or pitch %.1f deg over %u ATT rows' % (lean, len(attitudes)), flush=True)
    assert lean < MAX_LEAN_DEG, 'leaned %.1f deg in flight' % lean
    assert disarms == [DISARM_LANDED], 'disarm methods %s, not the land detector (%u)' % (disarms, DISARM_LANDED)
    arms = [row.Method for row in log_rows(logs, 'ARM') if row.ArmState]
    assert arms == [ARM_SCRIPTING], 'arm methods %s, not the script (%u)' % (arms, ARM_SCRIPTING)


def bench_reports(texts):
    '''The complete BENCH reports of binding_bench.lua in texts, each a list of (case, n); a report
    cut short by an M4 fault is left out.'''
    reports, report = [], None
    for text in texts:
        match = re.match(r'^BENCH (\S+) n=(\d+) ', text)
        if not match:
            continue
        if match.group(1) == BENCH_CASES[0]:
            report = []
        if report is not None:
            report.append((match.group(1), int(match.group(2))))
            if len(report) == len(BENCH_CASES):
                reports.append(report)
                report = None
    return reports


def check_bench_reports(texts):
    '''Each BENCH report covers every case, each over the same whole number of updates of BENCH_CALLS
    calls (n is the script's own tally: it shows the updates ran, not the calls SRPC served).
    Returns the number of reports.'''
    reports = bench_reports(texts)
    for report in reports:
        cases = tuple(case for case, _ in report)
        assert cases == BENCH_CASES, 'BENCH cases %s' % (cases,)
        counts = set(n for _, n in report)
        assert len(counts) == 1 and report[0][1] > 0 and report[0][1] % BENCH_CALLS == 0, \
            'BENCH report not whole updates of %u calls a case: %s' % (BENCH_CALLS, report)
    return len(reports)


def wait_bench_report(run):
    '''Waits for the next complete BENCH report and checks every report from its first line on.'''
    run.wait_text(r'^BENCH %s n=\d+ ' % re.escape(BENCH_CASES[0]), timeout=SCRIPT_TIMEOUT)
    first = run.next_text - 1
    run.wait_text(r'^BENCH %s n=\d+ ' % re.escape(BENCH_CASES[-1]), timeout=SCRIPT_TIMEOUT)
    assert check_bench_reports(run.texts[first:run.next_text]) >= 1, run.texts[first:run.next_text]


def check_update_counts(counts, total):
    '''The per-update rules of an m4_mission_flight.lua session started in flight, and the vehicle
    found armed (arming:is_armed once, no arming:arm).
    Accepted limit: these are the script's own tallies, so only the is_armed result depends on the
    M7. With binding_bench.lua in the flight its methods hold the SRPC top-8 rows and the mission
    script's own methods fold into m4:other, so no per-method SRPC bound for the restarted session
    holds. On the M7 side the release is pinned here by MISE (served once) and the arm by ARM;
    per-update serving is pinned by test_m4_mission_flight, without faults.'''
    missing = [name for name in ('vehicle:get_mode', 'mission:get_current_nav_index', 'param:get',
                                 'arming:is_armed', 'mission:set_current_cmd') if name not in counts]
    assert not missing and total is not None, 'script count lines missing: %s, total %s' % (missing, total)
    check_per_update(counts)
    assert counts['arming:is_armed'] == 1 and 'arming:arm' not in counts, counts


def check_flown_in_auto(flight_logs):
    '''From the flight's log, up to the disarm: once in AUTO the vehicle never changed mode, not even
    for a moment in a fault window, and the mission started its items once each, in order. The
    logger writes the current mode again at the arm.'''
    modes, started, armed = [], [], False
    reader = DFReader.DFReader_binary(str(flight_logs[0]), zero_time_base=True)
    while True:
        row = reader.recv_match(type=['MODE', 'MISE', 'ARM'])
        if row is None or (row.get_type() == 'ARM' and armed and not row.ArmState):
            break
        if row.get_type() == 'ARM':
            armed = bool(row.ArmState)
        elif row.get_type() == 'MODE':
            modes.append(row.Mode)
        else:
            started.append(row.CNum)
    print('to the disarm: MODE rows %s, mission items started (MISE) %s' % (modes, started), flush=True)
    assert COPTER_AUTO in modes and set(modes[modes.index(COPTER_AUTO):]) == {COPTER_AUTO}, \
        'MODE rows %s: left AUTO' % modes
    assert started == list(range(1, LAND_ITEM + 1)), 'mission items started (MISE) %s' % started


def test_m4_faults_mid_auto(start_m4_flight, tmp_path):
    '''M4 faults while the vehicle holds a LOITER_UNLIM in AUTO: it stays armed, in AUTO, at the item
    and height; the scripts, the benchmark among them, reload; the restarted script moves the mission
    on. The fourth fault turns scripting off in flight; the mission lands; prearm then reports it.'''
    parameters = dict(FLIGHT_PARAMETERS, SCR_USER1=HOLD_ITEM, SCR_USER2=0, SCR_USER3=1, SCR_M4_RST_MAX=3)
    run = start_flight(start_m4_flight,
                       ['tests/m4_mission_flight.lua', 'tests/m4_loop.lua', 'tests/binding_bench.lua'], parameters)
    started = len(run.texts)
    run.wait_text(r'^Scripting: started on M4 \(3 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    home = navigation_ready(run)
    home_lat, home_lon = home.lat * 1.0e-7, home.lon * 1.0e-7
    upload_mission(run.mav, run.process, run.log_path, flight_mission(home_lat, home_lon), timeout=STEP_TIMEOUT)
    track = FlightTrack(run)
    request_message_rate(run, mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT, 10)
    set_mode(run.mav, run.process, run.log_path, 'AUTO', timeout=STEP_TIMEOUT)
    assert run.wait_text(r'^M4 mission armed ', timeout=STEP_TIMEOUT) == 'M4 mission armed true'
    wait_until(run, lambda: track.current == HOLD_ITEM, LEG_TIMEOUT, 'the hold item')
    wait_bench_report(run)   # the benchmark runs in flight
    check_holding(run, track, HOLD_ITEM, HOLD_CHECK_SECONDS)
    for used, (kind, reason) in enumerate(FAULT_STEPS, start=1):
        set_parameter(run.mav, run.process, run.log_path, 'SCR_M4_INJECT', kind, timeout=STEP_TIMEOUT)
        run.wait_text(reason)
        run.wait_text(re.escape('Scripting: M4 restarting (%u/3)' % used))
        run.wait_text(re.escape('Scripting: M4 restarted (%u/3)' % used))
        run.wait_text(r'^M4 loop 1$', timeout=SCRIPT_TIMEOUT)
        run.wait_text(r'^M4 loop 2$', timeout=SCRIPT_TIMEOUT)
        check_holding(run, track, HOLD_ITEM, HOLD_CHECK_SECONDS)
        wait_bench_report(run)   # the benchmark reloaded
    released = [text for text in run.texts if text.startswith('M4 mission release')]
    assert not released, released
    # every height sample on the hold item, the fault windows included, not only the checked ones
    lowest, highest = track.item_heights[HOLD_ITEM]
    print('hold item %u: %.1f to %.1f m through the faults' % (HOLD_ITEM, lowest, highest), flush=True)
    assert 8 <= lowest and highest <= 12, 'hold item %u: %.1f to %.1f m' % (HOLD_ITEM, lowest, highest)
    set_parameter(run.mav, run.process, run.log_path, 'SCR_USER2', RELEASE_ITEM, timeout=STEP_TIMEOUT)
    assert run.wait_text(r'^M4 mission release ', timeout=SCRIPT_TIMEOUT) == 'M4 mission release 3 to 4 true'
    wait_until(run, lambda: track.current == RELEASE_ITEM, STEP_TIMEOUT, 'the release')
    run.wait_text(r'^M4 mission done$', timeout=STEP_TIMEOUT)
    counts, total = script_counts(run.texts)   # only the session that released reports
    print('script counts %s, total %s' % (counts, total), flush=True)
    check_update_counts(counts, total)
    # the fourth fault: the budget is spent, scripting stays off, the mission flies on
    set_parameter(run.mav, run.process, run.log_path, 'SCR_M4_INJECT', 1, timeout=STEP_TIMEOUT)   # 5 to 1
    run.wait_text(FAULT_STEPS[0][1])
    run.wait_text(re.escape(FAULT_LIMIT_TEXT))
    off_at = run.next_text   # texts after the fault limit text
    assert track.armed and track.mode == COPTER_AUTO and track.touchdown_item is None, \
        'at the fault limit: armed %s, mode %s, down at item %s' % (track.armed, track.mode, track.touchdown_item)
    print('fault limit at mission item %s, %.1f m up' % (track.current, track.altitude), flush=True)
    assert track.current in (RELEASE_ITEM, 5) and 8 <= track.altitude <= 12, \
        'at the fault limit: item %s, %.1f m up' % (track.current, track.altitude)
    wait_until(run, lambda: track.was_armed and not track.armed, LEG_TIMEOUT, 'the landing')
    landing = distance_metres(home_lat, home_lon, *track.position)
    disarm_mode, disarm_altitude, _ = track.disarm
    print('flight: %.1f m high, items %s, down at item %s, disarmed in mode %u at %.1f m, %.1f m from home' % (
        track.max_altitude, track.currents, track.touchdown_item, disarm_mode, disarm_altitude, landing), flush=True)
    assert ordered_subsequence(track.currents, [HOLD_ITEM, RELEASE_ITEM, 5, LAND_ITEM]), \
        'mission items %s' % track.currents
    assert track.touchdown_item == LAND_ITEM, 'came down below 1 m at item %s, not the LAND' % track.touchdown_item
    assert disarm_mode == COPTER_AUTO, 'disarmed in mode %u, not AUTO' % disarm_mode
    assert disarm_altitude < 1, 'disarmed %.1f m up' % disarm_altitude
    assert landing <= LANDING_RADIUS_M, 'landed %.1f m from home' % landing
    # m4_loop.lua prints once a second: none of the scripts ran from the fault limit to the landing
    ran = [text for text in run.texts[off_at:] if re.match(r'^(M4 |BENCH )', text)]
    assert not ran, 'a script ran after scripting went off: %s' % ran
    # after the landing, with the arming checks on, the arm attempt reports scripting off. The
    # loop-rate check comes before the scripting one and ends the system checks if it fails
    set_parameter(run.mav, run.process, run.log_path, 'ARMING_SKIPCHK', 0, timeout=STEP_TIMEOUT)
    armed_at = len(run.texts)
    try:
        run.arm_expect('Scripting core not running')
    except RuntimeError:
        slow = [text for text in run.texts[armed_at:] if 'Main loop slow' in text]
        assert not slow, 'the arm check stopped at "%s" before the scripting check' % slow[-1]
        raise
    run.wait_alive(10)
    failures = [text for text in run.texts[started:] if CRASH_TEXT.search(text)]
    assert not failures, failures
    expected = []
    for used, (_, reason) in enumerate(FAULT_STEPS, start=1):
        expected += [reason, re.escape('Scripting: M4 restarting (%u/3)' % used),
                     re.escape('Scripting: M4 restarted (%u/3)' % used)]
    expected += [FAULT_STEPS[0][1], re.escape(FAULT_LIMIT_TEXT)]
    supervisor = supervisor_texts(run, started)
    assert len(supervisor) == len(expected) and all(
        re.search(pattern, text) for pattern, text in zip(expected, supervisor)), supervisor
    assert check_bench_reports(run.texts[started:]) >= 1 + len(FAULT_STEPS)
    logs = read_logs(run, tmp_path / 'logs')
    flight_logs = [log for log in logs if any(row.ArmState for row in log_rows([log], 'ARM'))]
    assert len(flight_logs) == 1, 'logs with an arm: %s' % [log.name for log in flight_logs]
    # the MODE and MISE rows are the only check of a brief exit from AUTO: none may be lost
    dropped = max(row.Dp for row in log_rows(flight_logs, 'DSF'))
    print('logger buffer drops (DSF.Dp) %u' % dropped, flush=True)
    assert dropped == 0, 'rows dropped: the logger dropped %u rows (DSF.Dp)' % dropped
    check_flown_in_auto(flight_logs)
    # the M7 sends texts in bursts, so the reload time comes from the log's MSG rows
    messages = [(row.TimeUS, row.Message) for row in log_rows(flight_logs, 'MSG')]
    for used in range(1, len(FAULT_STEPS) + 1):
        restarting = next((time_us for time_us, text in messages
                           if text == 'Scripting: M4 restarting (%u/3)' % used), None)
        assert restarting is not None, 'no MSG row "restarting (%u/3)"' % used
        loop = next((time_us for time_us, text in messages if time_us >= restarting and text == 'M4 loop 1'), None)
        assert loop is not None, 'no MSG row "M4 loop 1" after "restarting (%u/3)"' % used
        print('fault step %u: "restarting" to "M4 loop 1" in %.0f ms emulated' % (
            used, (loop - restarting) / 1000.0), flush=True)
    m4st = log_rows(flight_logs, 'M4ST')
    assert m4st, 'no M4ST rows in the flight log'
    restarts = max(row.Rst for row in m4st)
    assert restarts == 3 and m4st[-1].St == 4, 'M4ST: Rst %u, last state %u, not 3 and Off (4)' % (
        restarts, m4st[-1].St)
    attitudes, disarms = armed_rows(logs, 'ATT')
    assert attitudes, 'no ATT rows while armed: the flight was not logged'
    lean = max(max(abs(row.Roll), abs(row.Pitch)) for row in attitudes)
    print('flight: largest roll or pitch %.1f deg over %u ATT rows' % (lean, len(attitudes)), flush=True)
    assert lean < MAX_LEAN_DEG, 'leaned %.1f deg in flight' % lean
    assert disarms == [DISARM_LANDED], 'disarm methods %s, not the land detector (%u)' % (disarms, DISARM_LANDED)
    arms = [row.Method for row in log_rows(logs, 'ARM') if row.ArmState]
    assert arms == [ARM_SCRIPTING], 'arm methods %s, not the script (%u)' % (arms, ARM_SCRIPTING)
