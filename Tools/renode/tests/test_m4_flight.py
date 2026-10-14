# AP_FLAKE8_CLEAN
"""Renode flights of the CubeOrangePlus-M4Lua Copter with Lua on the M4: the real M7 and M4
images fly AUTO missions against the physics sidecar (Tools/renode/renode_physics.cpp), as the
CubeOrangePlus QuadPlane flight of test_physics_flight.py does, with scripts on the M4 and M4
faults injected mid-mission. Functional only: Renode is not cycle-accurate.

Needs RENODE, a CubeOrangePlus-M4Lua build and the physics sidecar:
  ./waf configure --board CubeOrangePlus-M4Lua && ./waf copter
  ./waf configure --board sitl --enable-scripting-m4 && ./waf --targets tool/renode-physics
"""

import math
import os
import re
import signal
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
from test_m4_scripts import START_TIMEOUT, upload_files, upload_scripts  # noqa: E402
from test_mission import (FORCE_ARM_MAGIC, distance_metres, offset_location, send_command,  # noqa: E402
                          set_mode, stop_process_group, unused_tcp_port, upload_mission)
from test_physics_flight import (CANBERRA, CUBEORANGEPLUS_SENSOR_IDS, PHYSICS_RATE_HZ,  # noqa: E402
                                 check_sensor_ids, check_sidecar, wait_for_sidecar, wait_ready)

PHYSICS = Path(os.environ.get('M4LUA_PHYSICS', ROOT / 'build' / 'sitl' / 'tool' / 'renode-physics'))
PHYSICS_MODEL = 'x'          # SITL quad X: FRAME_CLASS 1, FRAME_TYPE 1
FLIGHT_DEADLINE = 14400      # wall seconds for a flight test (start_m4lua's 2700 s is for ground tests)
FLIGHT_TIMEOUT = 7200        # wall seconds for a flight leg, such as arming to landing
COPTER_AUTO = 3
HOLD_ITEM, RELEASE_ITEM, LAND_ITEM = 3, 4, 6

# set over MAVLink on a first launch (never built in): Renode supplies the flight state and the
# sensors, not RC, the safety switch, the power monitor or calibration. As the QuadPlane flight,
# with the KakuteF4 Copter flight's tune
FLIGHT_PARAMETERS = {
    'ARMING_SKIPCHK': -1,          # the scripting check is set back to on after landing
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
    signal.alarm(FLIGHT_DEADLINE)   # replaces start_m4lua's ground-test deadline
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
        for run in runs:   # Renode before its sidecar
            run.stop()
        for process in sidecars:
            stop_process_group(process)


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
    deadline = time.monotonic() + FLIGHT_TIMEOUT
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
    '''What the vehicle reports while a test waits: armed, mode, mission item, height, position.'''

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
        self._printed_altitude = 0.0
        run.mav.message_hooks.append(self._record)

    def _record(self, connection, message):
        kind = message.get_type()
        if kind == 'HEARTBEAT' and message.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed != self.armed:
                print('armed' if armed else 'disarmed', flush=True)
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
    wait_until(run, lambda: track.was_armed and not track.armed, FLIGHT_TIMEOUT, 'the landing')
    seconds = (run.boot_ms - emulated_ms) / 1000.0
    print('flight: %.0f s emulated in %.0f s of wall time (%.1fx)' % (
        seconds, time.monotonic() - wall, (time.monotonic() - wall) / max(seconds, 1)), flush=True)
    landing = distance_metres(home_lat, home_lon, *track.position)
    print('flight: %.1f m high, items reached %s, landed %.1f m from home' % (
        track.max_altitude, sorted(track.reached), landing), flush=True)
    assert track.max_altitude >= 8, 'reached only %.1f m' % track.max_altitude
    assert 2 in track.reached, 'mission items reached: %s' % sorted(track.reached)
    assert landing <= 3, 'landed %.1f m from home' % landing
    loops = [text for text in run.texts[texts_at_arm:] if re.match(r'^M4 loop \d+$', text)]
    assert len(loops) >= 5, 'the M4 script printed %u times in flight' % len(loops)
    assert not supervisor_texts(run, started), supervisor_texts(run, started)
    run.wait_alive(10)
    logs = read_logs(run, tmp_path / 'logs')
    m4st = log_rows(logs, 'M4ST')
    assert m4st and max(row.Rst for row in m4st) == 0, 'M4ST.Rst %s' % sorted(set(row.Rst for row in m4st))
    assert log_rows(logs, 'ATT'), 'no ATT rows: the flight was not logged'
