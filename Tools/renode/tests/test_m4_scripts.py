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
import threading
import time

from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import fat_image  # noqa: E402
from pymavlink import DFReader, mavftp  # noqa: E402
from test_m4_containment import (BUILD, M4_PEER_MAGIC, RCC_GCR, ROOT, STEP_TIMEOUT,  # noqa: E402,F401
                                 start_m4lua)

SCRIPTS = ROOT / 'libraries' / 'AP_Scripting'
START_TIMEOUT = 120    # wall seconds: boot to the first M4 start text
M4_SESSION = M4_PEER_MAGIC + 16   # PeerHeader::Half.session
FLOOD_MS = 2000      # AP_ScriptingM4_Layout.h: how long a request flood lasts
UPLOAD_TIMEOUT = 600   # wall seconds for a script upload
SCRIPT_TIMEOUT = 900   # wall seconds: math.lua at SCR_VM_I_COUNT 1000000 is slow under Renode


def remote_file_sizes(ftp, directory):
    '''{name: size} of the files in a board directory, as their FAT directory entries give them.'''
    ftp.list_result = []
    ftp.cmd_list([directory])
    return {entry.name: entry.size_b for entry in ftp.list_result if not entry.is_dir}


def make_directories(ftp, directory):
    '''Creates a board directory and its parents; one that exists already is fine.'''
    parts = directory.split('/')
    for end in range(1, len(parts) + 1):
        ftp.cmd_mkdir(['/'.join(parts[:end])])
        ftp.process_ftp_reply('CreateDirectory', timeout=60)


def upload_files(run, files):
    '''Copies (local path, board directory) pairs to the board's SD card over MAVLink FTP.'''
    ftp = mavftp.MAVFTP(run.mav, run.mav.target_system, run.mav.target_component)
    expected = {}
    for source, directory in files:
        if directory not in expected:
            make_directories(ftp, directory)   # AP_Scripting creates APM/scripts at boot
            expected[directory] = {}
        finished = []   # the put's callback: the length once every block is acknowledged, None on failure
        ret = ftp.cmd_put([str(source), directory + '/' + source.name], callback=finished.append)
        assert ret.error_code == 0, 'put %s: %s' % (source, ret)
        deadline = time.monotonic() + UPLOAD_TIMEOUT
        while not finished:
            assert time.monotonic() < deadline, 'upload of %s did not finish' % source
            ftp.process_ftp_reply('CreateFile', timeout=60)
        assert finished == [source.stat().st_size], 'upload of %s failed: %s' % (source, finished)
        expected[directory][source.name] = source.stat().st_size
    # the board writes a file's size into its directory entry when the FTP session ends;
    # process_ftp_reply() returns after 1.2 s without a reply, so a stalled Renode can
    # leave that TerminateSession unprocessed, and a stop then leaves an empty script
    # (it loads, runs once and makes no call): end the session until the sizes show
    for directory, wanted in expected.items():
        deadline = time.monotonic() + UPLOAD_TIMEOUT
        while True:
            sizes = remote_file_sizes(ftp, directory)
            if all(sizes.get(name) == size for name, size in wanted.items()):
                break
            assert time.monotonic() < deadline, '%s on the board holds %s, expected %s' % (directory, sizes, wanted)
            ftp.cmd_cancel()


def upload_scripts(run, paths):
    '''Copies scripts to APM/scripts on the board's SD card over MAVLink FTP.'''
    upload_files(run, [(SCRIPTS / path, 'APM/scripts') for path in paths])


def host_thread_stack(run, local_path):
    '''(free, size) bytes of the M7 ScriptingM4 host thread stack, from @SYS/threads.txt.'''
    ftp = mavftp.MAVFTP(run.mav, run.mav.target_system, run.mav.target_component)
    # pymavlink's default temp file, /tmp/temp_mavftp_file, is shared by every process's get
    ftp.temp_filename = str(local_path) + '.part'
    ret = ftp.cmd_get(['@SYS/threads.txt', str(local_path)])
    assert ret.error_code == 0, 'get threads.txt: %s' % ret
    ret = ftp.process_ftp_reply('OpenFileRO', timeout=60)
    assert ret.error_code == 0, 'get threads.txt failed: %s' % ret
    ftp.cmd_cancel()   # a @SYS file has no known size: end the read session so no burst reads linger
    match = re.search(r'(?m)^ScriptingM4\s.*STACK=\s*(\d+)/\s*(\d+)', Path(local_path).read_text())
    assert match, 'no ScriptingM4 thread in threads.txt'
    return int(match.group(1)), int(match.group(2))


def stop_and_read_log(run, directory):
    '''Stops Renode, which closes the board's logs, and returns the newest DataFlash log of its SD card image.'''
    run.stop()
    logs = fat_image.extract_files(run.sdcard_image, '/APM/LOGS', '*.BIN', directory)
    assert logs, 'no DataFlash log on the SD card'
    return max(logs, key=lambda log: log.name)


def check_log_rows(run, directory, skipped=()):
    '''The DataFlash log carries an SCR row per script (but the skipped ones) and SRPC rows for the
    M4's calls.'''
    log = stop_and_read_log(run, directory)
    reader = DFReader.DFReader_binary(str(log), zero_time_base=True)
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
        if name in skipped:
            continue
        assert name in scripts, 'no SCR row for %s (got %s)' % (name, scripts)
    for name in ('m4:config', 'm4:read_script', 'gcs:send_text'):
        assert name in calls, 'no SRPC row for %s (got %s)' % (name, calls)


def wait_all(run, patterns, timeout, failure=None):
    '''Each pattern matches a STATUSTEXT received after the last wait_text() match, in any order.
    A text matching the failure pattern ends the wait at once.'''
    start = run.next_text
    pending = [re.compile(pattern) for pattern in patterns]
    failure = re.compile(failure) if failure else None
    deadline = time.monotonic() + timeout
    while True:
        run.check()
        if failure:
            failed = [text for text in run.texts[start:] if failure.search(text)]
            if failed:
                raise RuntimeError('failure text %s; waiting for %s' % (failed[0], [regex.pattern for regex in pending]))
        pending = [regex for regex in pending if not any(regex.search(text) for text in run.texts[start:])]
        if not pending:
            return
        if time.monotonic() > deadline:
            raise RuntimeError('timed out waiting for %s; last texts %s' %
                               ([regex.pattern for regex in pending], run.texts[-15:]))
        run.mav.recv_match(blocking=True, timeout=1)


def start_with_scripts(start_m4lua, paths, parameters, files=()):  # noqa: F811
    '''Sets parameters and uploads scripts (and other (path, directory) files) on a first
    run, then starts the board again so they load.'''
    first = start_m4lua()
    for name, value in parameters.items():
        first.set(name, value)
    # scripts are listed when the M4 starts: wait for that before uploading, or the
    # count races with the upload
    first.wait_text(r'^Scripting: started on M4 \(0 scripts, heap \d+KB\)$', timeout=START_TIMEOUT)
    upload_scripts(first, paths)
    if files:
        upload_files(first, files)
    first.stop()
    run = start_m4lua()
    for name, value in parameters.items():
        assert run.get(name) == value, 'board state did not persist across the relaunch: %s' % name
    run.wait_m4_started()
    return run


def test_upload_survives_a_stall(start_m4lua, tmp_path, monkeypatch):  # noqa: F811
    # Renode stalls for 5 s once every block is acknowledged, before the FTP session
    # ends; a stop right after the upload still finds the whole script on the SD card
    run = start_m4lua()
    run.wait_text(r'^Scripting: started on M4 \(\d+ scripts, heap \d+KB\)$', timeout=START_TIMEOUT)
    put = mavftp.MAVFTP.cmd_put
    stalls = []

    def stalling_put(ftp, args, fh=None, callback=None, progress_callback=None):
        def progress(fraction):
            if fraction == 1.0 and not stalls:
                run.monitor.command('pause')
                stalls.append(threading.Timer(5, run.monitor.command, ('start',)))
                stalls[0].start()
        return put(ftp, args, fh, callback, progress)

    monkeypatch.setattr(mavftp.MAVFTP, 'cmd_put', stalling_put)
    upload_scripts(run, ['tests/m4_bindings.lua'])
    assert stalls, 'the upload never reached its end'
    stalls[0].join()
    run.stop()
    source = SCRIPTS / 'tests' / 'm4_bindings.lua'
    copies = fat_image.extract_files(tmp_path / 'state' / 'sdcard.img', '/APM/SCRIPTS', source.name,
                                     tmp_path / 'scripts')
    contents = [copy.read_bytes() for copy in copies]
    assert contents == [source.read_bytes()], 'the board holds %s of %u bytes' % (
        [len(content) for content in contents], source.stat().st_size)


def test_m4_runs_scripts(start_m4lua, tmp_path):  # noqa: F811
    skipped = ()
    scripts = ['examples/hello_world.lua', 'tests/strings.lua', 'tests/math.lua',
               'tests/m4_loop.lua', 'examples/fault_handling.lua']
    texts = [r'^hello, world$', r'^String tests passed$', r'^Math tests passed$', r'^M4 loop 3$',
             r'^Test 100 fault_count 100$']
    if os.environ.get('M4LUA_BOARD') == 'CubeOrangePlus-M4LuaNet':
        print('skipping tests/strings.lua on CubeOrangePlus-M4LuaNet: '
              'needs > 110 KB heap; peak not monotonic (GC timing)', flush=True)
        skipped = ('strings.lua',)
        scripts.remove('tests/strings.lua')
        texts.remove(r'^String tests passed$')
    run = start_with_scripts(start_m4lua, scripts,
                             {'SCR_VM_I_COUNT': 1000000, 'SCR_DEBUG_OPTS': 8,   # 8: LOG_RUNTIME, SCR and SRPC rows
                              'LOG_DISARMED': 1, 'LOG_DARM_RATEMAX': 0, 'LOG_FILE_RATEMAX': 0})
    run.wait_text(r'^Scripting: started on M4 \(%u scripts, heap \d+KB\)$' % len(scripts), timeout=SCRIPT_TIMEOUT)
    wait_all(run, texts, SCRIPT_TIMEOUT)
    # the stats pipeline: per-method round trips summarised every 10 s (LOG_RUNTIME)
    run.wait_text(r'^Scripting RPC \d+/s p50<\d+ p99<\d+ max \d+us$', timeout=SCRIPT_TIMEOUT)
    # the host thread serves every script call: it keeps 1 KB of its 4 KB stack free
    free, size = host_thread_stack(run, tmp_path / 'threads.txt')
    print('ScriptingM4 host thread stack: %u of %u bytes free' % (free, size), flush=True)
    assert free >= 1024, 'ScriptingM4 host thread stack: %u of %u bytes free' % (free, size)
    run.wait_alive(10)
    check_log_rows(run, tmp_path / 'logs', skipped)   # stops Renode: last


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
