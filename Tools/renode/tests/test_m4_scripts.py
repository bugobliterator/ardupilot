# AP_FLAKE8_CLEAN
"""Renode tests of Lua on the CubeOrangePlus Cortex-M4: example and test scripts
run unmodified on the M4 image, and faults of the unprivileged Lua thread are
contained. Functional only: Renode is not cycle-accurate.

Needs RENODE and a CubeOrangePlus-M4Lua build:
  ./waf configure --board CubeOrangePlus-M4Lua && ./waf copter
"""

import os
import re
import shutil
import subprocess
import sys
import time

from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pymavlink import DFReader, mavftp  # noqa: E402
from test_m4_containment import (BUILD, M4_PEER_MAGIC, RCC_GCR, ROOT, STEP_TIMEOUT,  # noqa: E402,F401
                                 start_m4lua)
from test_mission import download_log  # noqa: E402

SCRIPTS = ROOT / 'libraries' / 'AP_Scripting'
START_TIMEOUT = 120    # wall seconds: boot to the first M4 start text
M4_SESSION = M4_PEER_MAGIC + 16   # PeerHeader::Half.session
FLOOD_MS = 2000      # AP_ScriptingM4_Layout.h: how long a request flood lasts
LOG_DOWNLOAD_TIMEOUT = 60   # wall seconds without a finished download
SCRIPT_TIMEOUT = 900   # wall seconds: math.lua at SCR_VM_I_COUNT 1000000 is slow under Renode


def upload_scripts(run, paths):
    '''Copies scripts to APM/scripts on the board's SD card over MAVLink FTP.'''
    ftp = mavftp.MAVFTP(run.mav, run.mav.target_system, run.mav.target_component)
    ftp.cmd_mkdir(['APM/scripts'])   # AP_Scripting creates it at boot: a failure here is fine
    ftp.process_ftp_reply('CreateDirectory', timeout=60)
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
    match = re.search(r'(?m)^ScriptingM4\s.*STACK=\s*(\d+)/\s*(\d+)', Path(local_path).read_text())
    assert match, 'no ScriptingM4 thread in threads.txt'
    return int(match.group(1)), int(match.group(2))


def check_log_rows(run, local_path):
    '''The DataFlash log carries an SCR row per script and SRPC rows for the M4's calls.'''
    for attempt in range(5):   # a download can stall: start it again
        try:
            download_log(run.mav, run.process, run.log_path, local_path, timeout=LOG_DOWNLOAD_TIMEOUT)
            break
        except RuntimeError as error:
            assert 'timed out downloading' in str(error), error
            print('log download attempt %u stalled' % (attempt + 1), flush=True)
            run.mav.mav.log_request_end_send(run.mav.target_system, run.mav.target_component)
    else:
        raise RuntimeError('DataFlash log download stalled 5 times')
    reader = DFReader.DFReader_binary(str(local_path), zero_time_base=True)
    scripts = set()
    calls = set()
    while True:
        message = reader.recv_match(type=['SCR', 'SRPC'])
        if message is None:
            break
        if message.get_type() == 'SCR':
            scripts.add(message.Name)
        else:
            calls.add(message.Name)
            assert message.P99 >= message.P50, 'SRPC p99 below p50: %s' % message
    for name in ('hello_world.lua', 'strings.lua', 'math.lua', 'm4_loop.lua'):
        assert name in scripts, 'no SCR row for %s (got %s)' % (name, scripts)
    for name in ('m4:config', 'm4:read_script', 'gcs:send_text'):
        assert name in calls, 'no SRPC row for %s (got %s)' % (name, calls)


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
    # scripts are listed when the M4 starts: wait for that before uploading, or the
    # count races with the upload
    first.wait_text(r'^Scripting: started on M4 \(0 scripts, heap \d+KB\)$', timeout=START_TIMEOUT)
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
                              'tests/m4_loop.lua', 'examples/fault_handling.lua'],
                             {'SCR_VM_I_COUNT': 1000000, 'SCR_DEBUG_OPTS': 8,   # 8: LOG_RUNTIME, SCR and SRPC rows
                              'LOG_DISARMED': 1, 'LOG_DARM_RATEMAX': 0, 'LOG_FILE_RATEMAX': 0})
    run.wait_text(r'^Scripting: started on M4 \(5 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    wait_all(run, [r'^hello, world$', r'^String tests passed$', r'^Math tests passed$', r'^M4 loop 3$',
                   r'^Test 100 fault_count 100$'],
             SCRIPT_TIMEOUT)
    # the stats pipeline: per-method round trips summarised every 10 s (LOG_RUNTIME)
    run.wait_text(r'^Scripting RPC \d+/s p50<\d+ p99<\d+ max \d+us$', timeout=SCRIPT_TIMEOUT)
    # the host thread serves every script call: it keeps 1 KB of its 4 KB stack free
    free, size = host_thread_stack(run, tmp_path / 'threads.txt')
    print('ScriptingM4 host thread stack: %u of %u bytes free' % (free, size), flush=True)
    assert free >= 1024, 'ScriptingM4 host thread stack: %u of %u bytes free' % (free, size)
    check_log_rows(run, tmp_path / 'flight.BIN')
    run.wait_alive(10)


def test_m4_heap_hog_stops_alone(start_m4lua):  # noqa: F811
    # SCR_HEAP_SIZE above the M4's linker heap: the M4 caps it, keeping newlib's reserve
    run = start_with_scripts(start_m4lua, ['tests/m4_heap_hog.lua', 'tests/m4_loop.lua'],
                             {'SCR_HEAP_SIZE': 300000, 'SCR_VM_I_COUNT': 1000000})
    run.wait_text(r'^Scripting: started on M4 \(2 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    session = run.read_word(M4_SESSION)
    hog_text = len(run.texts)
    # lstring.c MEMERRMSG: the hog alone stops, and the other script keeps running after it
    run.wait_text(r'^Lua: not enough mem, increase SCR_HEAP_SIZE$', timeout=SCRIPT_TIMEOUT)
    for _ in range(3):
        run.wait_text(r'^M4 loop \d+$', timeout=SCRIPT_TIMEOUT)
    supervisor = [text for text in run.texts[hog_text:] if text.startswith('Scripting: M4')]
    assert not supervisor, 'the heap hog gave %s' % supervisor
    assert run.read_word(M4_SESSION) == session, 'the M4 restarted after the heap hog'
    assert not run.m4_halted(), 'cpu2 halted after the heap hog'
    run.wait_alive(10)


def test_m4_long_numeral_with_full_heap(start_m4lua):  # noqa: F811
    # a 3000-digit numeral parsed with the Lua heap full would take newlib's reserve and
    # fault the M4: it reads as not a number instead (AP_LUA_MAX_NUMBER_TEXT)
    run = start_with_scripts(start_m4lua, ['tests/m4_number_text.lua', 'tests/m4_loop.lua'],
                             {'SCR_HEAP_SIZE': 300000, 'SCR_VM_I_COUNT': 1000000})
    run.wait_text(r'^Scripting: started on M4 \(2 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    session = run.read_word(M4_SESSION)
    first_text = len(run.texts)
    run.wait_text(r'^M4 number text ok$', timeout=SCRIPT_TIMEOUT)
    run.wait_text(r'^M4 loop \d+$', timeout=SCRIPT_TIMEOUT)
    supervisor = [text for text in run.texts[first_text:] if text.startswith('Scripting: M4')]
    assert not supervisor, 'the long numeral gave %s' % supervisor
    assert run.read_word(M4_SESSION) == session, 'the M4 restarted after the long numeral'
    assert not run.m4_halted(), 'cpu2 halted after the long numeral'
    run.wait_alive(10)


def m4_symbols(elf):
    '''Demangled nm -S listing of the M4 image.'''
    if shutil.which('arm-none-eabi-nm') is None:
        pytest.skip('arm-none-eabi-nm not on PATH')
    return subprocess.check_output(['arm-none-eabi-nm', '-S', '-C', str(elf)], text=True)


def symbol_range(symbols, name):
    '''[start, end) of a symbol (a function name is followed by its argument list).'''
    match = re.search(r'(?m)^([0-9a-f]{8}) ([0-9a-f]{8}) [TtBbDd] ' + re.escape(name) + r'(\(|$)', symbols)
    assert match, '%s not in the M4 image' % name
    start = int(match.group(1), 16)
    return start, start + int(match.group(2), 16)


def test_m4_lua_thread_faults_are_contained(start_m4lua):  # noqa: F811
    symbols = m4_symbols(Path(os.environ.get('M4LUA_M4_ELF', BUILD / 'scripting_m4.elf')))
    # the AIRCR and RCC writes are the stores in port_lua_fault, the overflow the frames of overflow_stack
    fault_ranges = {
        7: symbol_range(symbols, 'ScriptingM4::port_lua_fault'),
        9: symbol_range(symbols, 'ScriptingM4::port_lua_fault'),
        8: symbol_range(symbols, 'overflow_stack'),
    }
    flood_ring_full = symbol_range(symbols, 'ScriptingM4::flood_ring_full')[0]
    run = start_with_scripts(start_m4lua, ['tests/m4_loop.lua'], {'SCR_M4_RST_MAX': 4})
    gcr = run.read_word(RCC_GCR)
    # "M4 loop N" comes only from tests/m4_loop.lua, whatever else the SD card holds
    run.wait_text(r'^M4 loop 2$', timeout=SCRIPT_TIMEOUT)
    session = run.read_word(M4_SESSION)

    # a request flood costs the M7 bounded time and is no fault
    flood_text = len(run.texts)
    flood_ms = run.boot_ms
    run.set('SCR_M4_INJECT', 6)
    run.wait_alive(10)
    deadline = time.monotonic() + STEP_TIMEOUT
    while run.boot_ms - flood_ms < FLOOD_MS + 1000:
        run.check()
        assert time.monotonic() < deadline, 'M7 time stopped during a request flood at %u ms' % run.boot_ms
        run.mav.recv_match(blocking=True, timeout=1)
    run.next_text = len(run.texts)
    run.wait_text(r'^M4 loop \d+$')       # the script still runs after the flood
    supervisor = [text for text in run.texts[flood_text:] if text.startswith('Scripting: M4')]
    assert not supervisor, 'a request flood gave %s' % supervisor
    ring_full = run.read_word(flood_ring_full)
    print('M4 flood passes that found the ring full: %u' % ring_full, flush=True)
    assert ring_full > 0, 'the request flood never filled the ring'
    assert run.read_word(M4_SESSION) == session, 'the M4 restarted during a request flood'
    assert run.read_word(RCC_GCR) == gcr, 'RCC_GCR changed during a request flood'
    assert not run.m4_halted(), 'cpu2 halted after a request flood'

    steps = (
        (7, 'BusFault'),    # AIRCR SYSRESETREQ from the Lua thread
        (9, 'MemManage'),   # RCC_GCR write from the Lua thread
        (8, 'MemManage'),   # the Lua stack into privileged RAM
    )
    for used, (kind, fault) in enumerate(steps, start=1):
        run.set('SCR_M4_INJECT', kind)
        text = run.wait_text(r'^Scripting: M4 %s PC=0x[0-9A-F]{8}$' % fault)
        pc = int(text.rsplit('=', 1)[1], 16)
        start, end = fault_ranges[kind]
        assert start <= pc < end, 'kind %u faulted at PC=0x%08X, outside 0x%08X-0x%08X' % (kind, pc, start, end)
        run.wait_text(re.escape('Scripting: M4 restarting (%u/4)' % used))
        run.wait_text(re.escape('Scripting: M4 restarted (%u/4)' % used))
        # the scripts load again (the counter starts over) and their calls reach the M7
        run.wait_text(r'^M4 loop 1$', timeout=SCRIPT_TIMEOUT)
        run.wait_text(r'^M4 loop 2$', timeout=SCRIPT_TIMEOUT)
        assert run.read_word(M4_SESSION) == session + used, 'kind %u: M4 session %u, expected %u' % (
            kind, run.read_word(M4_SESSION), session + used)
        assert run.read_word(RCC_GCR) == gcr, 'kind %u changed RCC_GCR' % kind
        assert not run.m4_halted(), 'cpu2 halted after kind %u' % kind
    # check() inside every wait: the M7 never rebooted (SYSTEM_TIME never went back)
    run.wait_alive(10)
