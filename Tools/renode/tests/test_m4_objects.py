# AP_FLAKE8_CLEAN
"""Renode tests of the M7 objects M4 scripts own or make on the CubeOrangePlus: parameters and
serial ports in the M7's object pool, released when the M4 collects them; the M7 host thread's
stack under them. Functional only: Renode is not cycle-accurate.

Needs RENODE and a CubeOrangePlus-M4Lua build:
  ./waf configure --board CubeOrangePlus-M4Lua && ./waf copter
"""

import sys

from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from test_m4_containment import start_m4lua  # noqa: E402,F401
from test_m4_scripts import SCRIPT_TIMEOUT, host_thread_stack, start_with_scripts, wait_all  # noqa: E402

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
