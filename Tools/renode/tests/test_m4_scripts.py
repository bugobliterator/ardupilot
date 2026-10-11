# AP_FLAKE8_CLEAN
"""Renode tests of Lua on the CubeOrangePlus Cortex-M4: example and test scripts
run unmodified on the M4 image, and faults of the unprivileged Lua thread are
contained. Functional only: Renode is not cycle-accurate.

Needs RENODE and a CubeOrangePlus-M4Lua build:
  ./waf configure --board CubeOrangePlus-M4Lua && ./waf copter
"""

import re
import sys
import time

from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pymavlink import mavftp  # noqa: E402
from test_m4_containment import M4_PC, RCC_GCR, ROOT, start_m4lua  # noqa: E402,F401

SCRIPTS = ROOT / 'libraries' / 'AP_Scripting'
SCRIPT_TIMEOUT = 900   # wall seconds: math.lua at SCR_VM_I_COUNT 1000000 is slow under Renode


def upload_scripts(run, paths):
    '''Copies scripts to APM/scripts on the board's SD card over MAVLink FTP.'''
    ftp = mavftp.MAVFTP(run.mav, run.mav.target_system, run.mav.target_component)
    ftp.cmd_mkdir(['APM/scripts'])   # AP_Scripting creates it at boot: a failure here is fine
    for path in paths:
        source = SCRIPTS / path
        ret = ftp.cmd_put([str(source), 'APM/scripts/' + source.name])
        assert ret.error_code == 0, 'put %s: %s' % (path, ret)
        ret = ftp.process_ftp_reply('CreateFile', timeout=600)
        assert ret.error_code == 0, 'upload of %s failed: %s' % (path, ret)


def host_thread_stack(run, local_path):
    '''(free, size) bytes of the M7 ScriptingM4 host thread stack, from @SYS/threads.txt.'''
    ftp = mavftp.MAVFTP(run.mav, run.mav.target_system, run.mav.target_component)
    ret = ftp.cmd_get(['@SYS/threads.txt', str(local_path)])
    assert ret.error_code == 0, 'get threads.txt: %s' % ret
    ret = ftp.process_ftp_reply('OpenFileRO', timeout=60)
    assert ret.error_code == 0, 'get threads.txt failed: %s' % ret
    match = re.search(r'(?m)^ScriptingM4\s.*STACK=(\d+)/(\d+)', Path(local_path).read_text())
    assert match, 'no ScriptingM4 thread in threads.txt'
    return int(match.group(1)), int(match.group(2))


def wait_all(run, patterns, timeout):
    '''Each pattern matches a STATUSTEXT received after the last wait_text() match, in any order.'''
    start = run.next_text
    pending = [re.compile(pattern) for pattern in patterns]
    deadline = time.monotonic() + timeout
    while True:
        run.check()
        pending = [regex for regex in pending if not any(regex.search(text) for text in run.texts[start:])]
        if not pending:
            return
        if time.monotonic() > deadline:
            raise RuntimeError('timed out waiting for %s; last texts %s' %
                               ([regex.pattern for regex in pending], run.texts[-15:]))
        run.mav.recv_match(blocking=True, timeout=1)


def start_with_scripts(start_m4lua, paths, parameters):  # noqa: F811
    '''Sets parameters and uploads scripts on a first run, then starts the board again so they load.'''
    first = start_m4lua()
    for name, value in parameters.items():
        first.set(name, value)
    upload_scripts(first, paths)
    first.stop()
    run = start_m4lua()
    for name, value in parameters.items():
        assert run.get(name) == value, 'board state did not persist across the relaunch: %s' % name
    run.wait_m4_started()
    return run


def test_m4_runs_scripts(start_m4lua, tmp_path):  # noqa: F811
    run = start_with_scripts(start_m4lua,
                             ['examples/hello_world.lua', 'tests/strings.lua', 'tests/math.lua',
                              'tests/m4_loop.lua'],
                             {'SCR_VM_I_COUNT': 1000000, 'SCR_DEBUG_OPTS': 8})
    run.wait_text(r'^Scripting: started on M4 \(4 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    wait_all(run, [r'^hello, world$', r'^String tests passed$', r'^Math tests passed$', r'^M4 loop 3$'],
             SCRIPT_TIMEOUT)
    # the stats pipeline: per-method round trips summarised every 10 s (LOG_RUNTIME)
    run.wait_text(r'^Scripting RPC \d+/s p50<\d+ p99<\d+ max \d+us$', timeout=SCRIPT_TIMEOUT)
    # the host thread serves every script call: it keeps 1 KB of its 4 KB stack free
    free, size = host_thread_stack(run, tmp_path / 'threads.txt')
    print('ScriptingM4 host thread stack: %u of %u bytes free' % (free, size), flush=True)
    assert free >= 1024, 'ScriptingM4 host thread stack: %u of %u bytes free' % (free, size)
    run.wait_alive(10)
