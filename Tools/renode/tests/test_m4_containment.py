# AP_FLAKE8_CLEAN
"""Renode containment tests for Lua scripting on the CubeOrangePlus Cortex-M4.

Faults injected into the real M4 image never reboot or stall the M7, restarts
are capped, a lockup turns scripting off, a stray RCC write is stopped by the
M4 MPU, and RCC_GCR carries the ES0445 reset-scope bits. Functional only:
Renode is not cycle-accurate.

Needs RENODE and a CubeOrangePlus-M4Lua build:
  ./waf configure --board CubeOrangePlus-M4Lua && ./waf copter
"""

import os
import re
import signal
import subprocess
import sys
import time

from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from launch import MonitorClient  # noqa: E402
from pymavlink import mavutil  # noqa: E402
from test_mission import (check_process, log_tail, set_parameter,  # noqa: E402
                          stop_process_group, unused_tcp_port)

ROOT = Path(__file__).resolve().parents[3]
BOARD = 'CubeOrangePlus-M4Lua'
BUILD = ROOT / 'build' / BOARD / 'bin'
RCC_GCR = 0x580244A0
RESET_SCOPE = 0x3      # WW1RSC | WW2RSC
BOOT_C2 = 0x8
M7_PEER_MAGIC = 0x30040000
M4_PEER_MAGIC = 0x30040040
PEER_MAGIC = 0x48504643
STEP_TIMEOUT = 300     # wall seconds: Renode runs both cores well below real time
TEST_DEADLINE = 2700   # wall seconds for a whole test: a wait without its own bound fails the test instead of hanging
START_TIMEOUT = 120    # wall seconds from MAVLink connect to "Scripting: M4 started" (11-17 s seen)
M4_PC = r'PC=0x081[C-F][0-9A-F]{4}'


class M4Run:
    '''One Renode run of the M7 firmware and the M4 image, with MAVLink and the monitor.'''

    def __init__(self, process, log_path, uart_port, monitor_port):
        self.process = process
        self.log_path = log_path
        self.uart_port = uart_port
        self.monitor_port = monitor_port
        self.mav = None
        self.monitor = None
        self.texts = []
        self.next_text = 0
        self.boot_ms = 0
        self.reboot = None

    def connect(self):
        # every STATUSTEXT is kept from the first byte on: boot texts such as
        # "Scripting: M4 started" can arrive before the first HEARTBEAT
        deadline = time.monotonic() + STEP_TIMEOUT
        while self.mav is None:
            check_process(self.process, self.log_path)
            if time.monotonic() > deadline:
                raise RuntimeError('timed out connecting to MAVLink\n%s' % log_tail(self.log_path))
            try:
                self.mav = mavutil.mavlink_connection('tcp:127.0.0.1:%u' % self.uart_port,
                                                      source_system=255, source_component=190)
            except OSError:
                time.sleep(0.2)
        self.mav.message_hooks.append(self._record)
        self.recv('HEARTBEAT')
        self.mav.mav.request_data_stream_send(self.mav.target_system, self.mav.target_component,
                                              mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
        # SYSTEM_TIME at 2 Hz shows the M7 running and catches a reboot
        self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                                       mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                       mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME, 500000, 0, 0, 0, 0, 0)
        self.monitor = MonitorClient('127.0.0.1', self.monitor_port)
        self.monitor.connect()

    def stop(self):
        '''Close the links and stop Renode, which saves the board flash.'''
        if self.monitor is not None:
            self.monitor.close()
            self.monitor = None
        if self.mav is not None:
            self.mav.close()
            self.mav = None
        stop_process_group(self.process)

    def _record(self, connection, message):
        if message.get_type() == 'SYSTEM_TIME':
            if message.time_boot_ms < self.boot_ms and self.reboot is None:
                self.reboot = 'M7 rebooted: time_boot_ms %u after %u' % (message.time_boot_ms, self.boot_ms)
            self.boot_ms = message.time_boot_ms
        elif message.get_type() == 'STATUSTEXT':
            text = message.text
            if isinstance(text, bytes):
                text = text.decode(errors='replace')
            text = text.rstrip('\x00')
            self.texts.append(text)
            print('[mav] %s' % text, flush=True)

    def recv(self, message_type, timeout=STEP_TIMEOUT):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.check()
            message = self.mav.recv_match(type=message_type, blocking=True, timeout=1)
            if message is not None:
                return message
        raise RuntimeError('timed out waiting for %s\n%s' % (message_type, log_tail(self.log_path)))

    def check(self):
        '''Renode still runs and the M7 has not rebooted (SYSTEM_TIME never goes back).'''
        check_process(self.process, self.log_path)
        if self.reboot is not None:
            raise RuntimeError(self.reboot)

    def monitor_value(self, command, pattern):
        output = self.monitor.command(command)
        values = re.findall(pattern, output)
        if not values:
            raise RuntimeError('no value from %r: %r' % (command, output))
        return values[-1]

    def read_word(self, address):
        return int(self.monitor_value('sysbus ReadDoubleWord 0x%08X' % address,
                                      r'(?m)^\s*(0x[0-9A-Fa-f]{1,8})\s*$'), 16)

    def m4_halted(self):
        return self.monitor_value('cpu2 IsHalted', r'(?m)^\s*(True|False)\s*$') == 'True'

    def wait_text(self, pattern, timeout=STEP_TIMEOUT):
        '''First STATUSTEXT after the previous match that matches pattern.'''
        regex = re.compile(pattern)
        deadline = time.monotonic() + timeout
        while True:
            self.check()
            for index in range(self.next_text, len(self.texts)):
                if regex.search(self.texts[index]):
                    self.next_text = index + 1
                    return self.texts[index]
            if time.monotonic() > deadline:
                raise RuntimeError('timed out waiting for %r; last texts %s\n%s' %
                                   (pattern, self.texts[-15:], log_tail(self.log_path)))
            self.mav.recv_match(blocking=True, timeout=1)

    def wait_m4_started(self):
        '''The M7 released the M4 (BOOT_C2, cpu2 running) and the M4 published its peer header.'''
        started = time.monotonic()
        self.wait_text(r'Scripting: M4 (started|.*off until reboot)', timeout=START_TIMEOUT)
        print('M4 start text %.0f s after connect' % (time.monotonic() - started), flush=True)
        assert self.texts[self.next_text - 1] == 'Scripting: M4 started', 'M7 did not start the M4'
        assert not self.m4_halted(), 'cpu2 is halted after "Scripting: M4 started"'
        assert self.read_word(RCC_GCR) & BOOT_C2, 'RCC_GCR.BOOT_C2 not set'
        assert self.read_word(M7_PEER_MAGIC) == PEER_MAGIC, 'M7 peer header not published'
        assert self.read_word(M4_PEER_MAGIC) == PEER_MAGIC, 'M4 peer header not published'

    def set(self, name, value):
        set_parameter(self.mav, self.process, self.log_path, name, value)

    def get(self, name):
        deadline = time.monotonic() + STEP_TIMEOUT
        while time.monotonic() < deadline:
            self.mav.mav.param_request_read_send(self.mav.target_system, self.mav.target_component,
                                                 name.encode('ascii'), -1)
            retry = time.monotonic() + 2
            while time.monotonic() < retry:
                self.check()
                message = self.mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
                if message is None:
                    continue
                param_id = message.param_id
                if isinstance(param_id, bytes):
                    param_id = param_id.decode(errors='replace')
                if param_id.rstrip('\x00') == name:
                    return message.param_value
        raise RuntimeError('timed out reading %s\n%s' % (name, log_tail(self.log_path)))

    def wait_alive(self, seconds):
        '''The M7 keeps running: time_boot_ms advances over the next seconds of wall time.'''
        start = self.boot_ms
        until = time.monotonic() + seconds
        deadline = until + STEP_TIMEOUT
        while time.monotonic() < until or self.boot_ms <= start:
            self.check()
            assert time.monotonic() < deadline, 'M7 time did not advance from %u ms' % start
            self.mav.recv_match(blocking=True, timeout=1)

    def arm_expect(self, prearm_text):
        self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                                       mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        # the arm command reports "Arm: ...", the periodic disarmed check "PreArm: ..."
        self.wait_text(r'^(Pre)?Arm: ' + re.escape(prearm_text))
        heartbeat = self.recv('HEARTBEAT')
        assert not heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 'armed with scripting off'


@pytest.fixture
def start_m4lua(tmp_path):
    '''Starts Renode runs that share one board state directory (flash, so parameters persist).'''
    renode = os.environ.get('RENODE')
    elf = Path(os.environ.get('M4LUA_ELF', BUILD / 'arducopter'))
    m4_elf = Path(os.environ.get('M4LUA_M4_ELF', BUILD / 'scripting_m4.elf'))
    if not renode or not elf.is_file() or not m4_elf.is_file():
        pytest.skip('set RENODE and build %s (arducopter, scripting_m4.elf)' % BOARD)
    state_dir = tmp_path / 'state'
    state_dir.mkdir()
    runs = []

    def deadline_passed(signum, frame):
        pytest.fail('test passed its %u s wall-clock deadline' % TEST_DEADLINE)

    previous_handler = signal.signal(signal.SIGALRM, deadline_passed)
    signal.alarm(TEST_DEADLINE)

    def start():
        uart_port = unused_tcp_port()
        monitor_port = unused_tcp_port()
        while monitor_port == uart_port:
            monitor_port = unused_tcp_port()
        log_path = tmp_path / ('renode-%u.log' % len(runs))
        command = [sys.executable, str(ROOT / 'Tools' / 'renode' / 'run.py'), BOARD,
                   '--vehicle', 'arducopter', '--elf', str(elf), '--m4-elf', str(m4_elf),
                   '--state-dir', str(state_dir), '--uart-port', str(uart_port),
                   '--port', str(monitor_port), '--unthrottled', '--renode', renode]
        env = os.environ.copy()
        env['XDG_CONFIG_HOME'] = str(state_dir)
        env['TMPDIR'] = str(state_dir)
        with log_path.open('w') as log:
            process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        run = M4Run(process, log_path, uart_port, monitor_port)
        run.sdcard_image = state_dir / 'sdcard.img'
        runs.append(run)
        run.connect()
        return run

    try:
        yield start
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, previous_handler)
        # stop every Renode even if one stop fails, then report the first failure
        errors = []
        for run in runs:
            try:
                run.stop()
            except Exception as error:
                errors.append(error)
        if errors:
            raise errors[0]


def test_m4_fault_containment(start_m4lua):
    # the arming check stops at an unset frame before it reaches scripting;
    # the frame is set up at boot, so set it and start the board again
    first = start_m4lua()
    first.set('FRAME_CLASS', 1)
    first.set('FRAME_TYPE', 1)
    first.wait_alive(10)
    first.stop()
    m4lua = start_m4lua()
    for name in ('FRAME_CLASS', 'FRAME_TYPE'):
        value = m4lua.get(name)
        assert value == 1, 'board state did not persist across the relaunch: %s=%s' % (name, value)
    m4lua.wait_m4_started()
    gcr = m4lua.read_word(RCC_GCR)
    assert gcr & RESET_SCOPE == RESET_SCOPE, 'ES0445 reset scope not set: RCC_GCR=0x%08X' % gcr
    m4lua.set('SCR_M4_RST_MAX', 4)
    steps = (
        (1, r'Scripting: M4 HardFault ' + M4_PC),
        (2, r'Scripting: M4 MemManage ' + M4_PC),
        (3, r'Scripting: M4 heartbeat lost'),
        (5, r'Scripting: M4 protocol errors'),
    )
    for used, (kind, reason) in enumerate(steps, start=1):
        gcr_before = m4lua.read_word(RCC_GCR)
        m4lua.set('SCR_M4_INJECT', kind)
        m4lua.wait_text(reason)
        m4lua.wait_text(re.escape('Scripting: M4 restarting (%u/4)' % used))
        m4lua.wait_text(re.escape('Scripting: M4 restarted (%u/4)' % used))
        assert m4lua.read_word(RCC_GCR) == gcr_before, 'a stray M4 write reached RCC_GCR'
    m4lua.set('SCR_M4_INJECT', 0)
    m4lua.set('SCR_M4_INJECT', 1)
    m4lua.wait_text(r'Scripting: M4 HardFault ' + M4_PC)
    m4lua.wait_text(re.escape('Scripting: M4 fault limit, off until reboot'))
    m4lua.arm_expect('Scripting core not running')
    m4lua.wait_alive(20)


def test_m4_lockup_turns_scripting_off(start_m4lua):
    m4lua = start_m4lua()
    m4lua.wait_m4_started()
    m4lua.set('SCR_M4_INJECT', 4)
    m4lua.wait_text(r'Scripting: M4 heartbeat lost')
    m4lua.wait_text(re.escape('Scripting: M4 restarting (1/3)'))
    m4lua.wait_text(re.escape('Scripting: M4 not responding, off until reboot'))
    m4lua.wait_alive(20)


def test_m4_off_parks_a_stalled_m4(start_m4lua):
    m4_elf = Path(os.environ.get('M4LUA_M4_ELF', BUILD / 'scripting_m4.elf'))
    symbols = subprocess.check_output(['arm-none-eabi-nm', '-S', str(m4_elf)], text=True)
    park = re.search(r'(?m)^([0-9a-f]{8}) ([0-9a-f]{8}) T m4_park_forever$', symbols)
    assert park, 'm4_park_forever not in %s' % m4_elf
    park_start = int(park.group(1), 16)
    park_end = park_start + int(park.group(2), 16)
    m4lua = start_m4lua()
    m4lua.wait_m4_started()
    # no restarts: the stall, with interrupts on, turns scripting off at once
    m4lua.set('SCR_M4_RST_MAX', 0)
    m4lua.set('SCR_M4_INJECT', 3)
    m4lua.wait_text(r'Scripting: M4 heartbeat lost')
    m4lua.wait_text(re.escape('Scripting: M4 fault limit, off until reboot'))
    m4lua.wait_alive(5)
    # Off rang the restart doorbell: the M4 left the stall loop and parked
    for _ in range(3):
        pc = int(m4lua.monitor_value('cpu2 PC', r'(0x[0-9A-Fa-f]+)'), 16)
        assert park_start <= pc < park_end, 'M4 not parked after Off: PC=0x%08X' % pc
        m4lua.wait_alive(2)
