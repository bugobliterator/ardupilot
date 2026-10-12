# AP_FLAKE8_CLEAN
"""Renode tests of the M7 objects M4 scripts own or make on the CubeOrangePlus: parameters and
serial ports in the M7's object pool, released when the M4 collects them or when its session
ends; the M7 host thread's stack under them. Functional only: Renode is not cycle-accurate.

Needs RENODE and a CubeOrangePlus-M4Lua build:
  ./waf configure --board CubeOrangePlus-M4Lua && ./waf copter
"""

import sys

from pathlib import Path

from pymavlink import DFReader

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from test_m4_containment import start_m4lua  # noqa: E402,F401
from test_m4_scripts import (SCRIPT_TIMEOUT, host_thread_stack, start_with_scripts,  # noqa: E402
                             stop_and_read_log, wait_all)

# a simulated device on the first scripting serial port (serial_loopback.lua, m4_objects.lua)
SERIAL_DEVICE = {'SCR_VM_I_COUNT': 1000000, 'SCR_SDEV_EN': 1, 'SCR_SDEV1_PROTO': 28}


def assert_host_stack(run, tmp_path):
    '''The M7 host thread ran the M4's object calls and keeps 2 KB of its stack.'''
    free, size = host_thread_stack(run, tmp_path / 'threads.txt')
    print('ScriptingM4 host thread stack: %u of %u bytes free' % (free, size), flush=True)
    assert free >= 2048, 'ScriptingM4 host thread stack: %u of %u bytes free' % (free, size)


def test_m4_objects(start_m4lua, tmp_path):  # noqa: F811
    run = start_with_scripts(start_m4lua, ['tests/serial_loopback.lua', 'tests/m4_objects.lua'], SERIAL_DEVICE)
    run.wait_text(r'^Scripting: started on M4 \(2 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    wait_all(run, [r'^driver -> device good$', r'^device -> driver good$', r'^M4 objects (passed|failed \d+)$'],
             SCRIPT_TIMEOUT, failure=r'^(Lua: |M4 objects FAIL)')
    assert 'M4 objects passed' in run.texts, run.texts[-15:]
    assert_host_stack(run, tmp_path)
    run.wait_alive(10)
    run.stop()


def test_m4_serial_ports(start_m4lua, tmp_path):  # noqa: F811
    '''readstring answers in chunks of up to 768 bytes on the host thread stack.'''
    run = start_with_scripts(start_m4lua, ['tests/m4_serial_ports.lua'], SERIAL_DEVICE)
    run.wait_text(r'^Scripting: started on M4 \(1 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    wait_all(run, [r'^M4 serial ports (passed|failed \d+)$'], SCRIPT_TIMEOUT, failure=r'^(Lua: |M4 serial FAILED)')
    assert 'M4 serial ports passed' in run.texts, run.texts[-15:]
    assert_host_stack(run, tmp_path)
    run.wait_alive(10)
    run.stop()


LOG_ROWS = {'LOG_DISARMED': 1, 'LOG_DARM_RATEMAX': 0, 'LOG_FILE_RATEMAX': 0}


def test_m4_objects_end_with_the_session(start_m4lua, tmp_path):  # noqa: F811
    '''A restarted M4 starts clean: the M7 freed the objects the first session held (the new one gets
    all 64 again), and the release queue, which lives in .lua_bss that the M4 zeroes at boot, sends
    no stale release (no protocol error).'''
    run = start_with_scripts(start_m4lua, ['tests/m4_objects_hold.lua'], dict(SERIAL_DEVICE, **LOG_ROWS))
    run.wait_text(r'^Scripting: started on M4 \(1 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    held = r'^M4 holding \d+ objects, auth \S+$'
    first = run.wait_text(held, timeout=SCRIPT_TIMEOUT)
    assert first == 'M4 holding 64 objects, auth 0', first
    run.wait_text(r'^M4 hold: ', timeout=SCRIPT_TIMEOUT)
    run.set('SCR_M4_INJECT', 1)   # a fault in the M4 service: the M7 restarts it
    run.wait_text(r'^Scripting: M4 restarted \(1/\d+\)$', timeout=SCRIPT_TIMEOUT)
    # without the session end the pool still holds the first session's objects and the new one gets fewer
    second = run.wait_text(held, timeout=SCRIPT_TIMEOUT)
    assert second == 'M4 holding 64 objects, auth 0', second
    state = run.wait_text(r'^M4 hold: ', timeout=SCRIPT_TIMEOUT)
    assert state.endswith('left 0, port ok'), state
    assert_host_stack(run, tmp_path)
    run.wait_alive(10)   # M4ST rows are written once a second
    log = stop_and_read_log(run, tmp_path / 'logs')
    reader = DFReader.DFReader_binary(str(log), zero_time_base=True)
    rows = 0
    errors = 0
    while True:
        row = reader.recv_match(type='M4ST')
        if row is None:
            break
        rows += 1
        errors += row.PErr
    assert rows > 0, 'no M4ST rows in the log'
    assert errors == 0, '%u M4 protocol errors across the restart' % errors
