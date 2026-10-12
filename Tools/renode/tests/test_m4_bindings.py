# AP_FLAKE8_CLEAN
"""Renode tests of the generated bindings on the CubeOrangePlus Cortex-M4: the
M7 serves the binding calls of scripts run on the M4 image, with the in-process
results and error texts. Functional only: Renode is not cycle-accurate.

Needs RENODE and a CubeOrangePlus-M4Lua build:
  ./waf configure --board CubeOrangePlus-M4Lua && ./waf copter
"""

import re
import sys

from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pymavlink import DFReader  # noqa: E402
from test_m4_containment import start_m4lua  # noqa: E402,F401
from test_m4_scripts import (M4_SESSION, SCRIPT_TIMEOUT, host_thread_stack,  # noqa: E402
                             start_with_scripts, stop_and_read_log, wait_all)

LOG_PARAMETERS = {'SCR_DEBUG_OPTS': 8,   # 8: LOG_RUNTIME, SRPC rows
                  'LOG_DISARMED': 1, 'LOG_DARM_RATEMAX': 0, 'LOG_FILE_RATEMAX': 0}


def srpc_totals(run, directory):
    '''Sum of SRPC.N per method name in the board's DataFlash log; stops Renode.'''
    log = stop_and_read_log(run, directory)
    reader = DFReader.DFReader_binary(str(log), zero_time_base=True)
    totals = {}
    while True:
        message = reader.recv_match(type=['SRPC'])
        if message is None:
            return totals
        assert message.P99 >= message.P50, 'SRPC p99 below p50: %s' % message
        totals[message.Name] = totals.get(message.Name, 0) + message.N


def test_m4_generated_bindings(start_m4lua, tmp_path):  # noqa: F811
    run = start_with_scripts(start_m4lua, ['tests/m4_bindings.lua'], dict(LOG_PARAMETERS, SCR_USER2=0))
    run.wait_text(r'^Scripting: started on M4 \(1 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    run.wait_text(r'^M4 bindings passed$', timeout=SCRIPT_TIMEOUT)
    # the statistics window of the last calls is logged, and the log written out, within two summaries
    for _ in range(2):
        run.wait_text(r'^Scripting RPC \d+/s p50<\d+ p99<\d+ max \d+us$', timeout=SCRIPT_TIMEOUT)
    expected = {}
    script_total = None
    for text in run.texts:
        match = re.match(r'^M4 count (\S+) (\d+)$', text)
        if match:
            expected[match.group(1)] = int(match.group(2))
        match = re.match(r'^M4 total (\d+)$', text)
        if match:
            script_total = int(match.group(1))
    assert len(expected) >= 15 and script_total is not None, (expected, script_total)
    # the host thread ran mode changes and a GUIDED set_target_location for the script (its
    # deepest binding path): it keeps 2 KB of its 8 KB stack
    free, size = host_thread_stack(run, tmp_path / 'threads.txt')
    print('ScriptingM4 host thread stack: %u of %u bytes free' % (free, size), flush=True)
    assert free >= 2048, 'ScriptingM4 host thread stack: %u of %u bytes free' % (free, size)
    run.wait_alive(10)
    totals = srpc_totals(run, tmp_path / 'logs')   # stops Renode
    print('SRPC logged %s' % sorted(totals.items()), flush=True)
    # a method can fold into m4:other in a busy window: never more than the script made
    for name, count in sorted(expected.items()):
        assert totals.get(name, 0) <= count, \
            'SRPC %s: %u calls logged, the script made %u' % (name, totals.get(name, 0), count)
    # every binding call is logged once: the binding rows and m4:other add up to the script's
    # total (the system rows, m4:sysother included, and print are not the script's)
    logged = sum(count for name, count in totals.items()
                 if name == 'm4:other' or not (name.startswith('m4:') or name == 'global:print'))
    print('SRPC binding calls logged %u, script made %u' % (logged, script_total), flush=True)
    assert logged == script_total, 'SRPC binding calls: %u logged, the script made %u (%s)' % (
        logged, script_total, totals)


def test_m4_example_bindings(start_m4lua, tmp_path):  # noqa: F811
    run = start_with_scripts(start_m4lua,
                             ['tests/binding_bench.lua', 'examples/copy_userdata.lua', 'examples/simple_loop.lua'],
                             LOG_PARAMETERS)
    run.wait_text(r'^Scripting: started on M4 \(3 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    wait_all(run, [r'^vdiff=\(100\.00,300\.00\)$', r'^locdiff=\(\d+\.\d\d,\d+\.\d\d\)$', r'^hello, world$',
                   r'^BENCH ahrs:get_location n=\d+'],
             SCRIPT_TIMEOUT)
    run.wait_alive(10)
    totals = srpc_totals(run, tmp_path / 'logs')   # stops Renode
    for name in ('ahrs:get_location', 'vehicle:get_mode', 'param:get', 'global:millis', 'gps:status'):
        assert totals.get(name, 0) > 0, 'no SRPC rows for %s (got %s)' % (name, sorted(totals))
    # copy_userdata.lua's Location and vector math, binding_bench's Locations: all on the M4
    from test_m4_pure_data import local_rows   # here, not at the top: test_m4_pure_data imports this module
    assert not local_rows(totals), 'SRPC rows for local pure-data methods: %s' % local_rows(totals)


def test_m4_number_text_under_full_heap(start_m4lua):  # noqa: F811
    # a long numeral parsed with the Lua heap full ran newlib's reserve dry; the heap setting
    # above the M4's linker heap is capped, so the heap is as large as the M4 allows
    run = start_with_scripts(start_m4lua, ['tests/m4_number_text.lua', 'tests/m4_loop.lua'],
                             {'SCR_HEAP_SIZE': 300000, 'SCR_VM_I_COUNT': 1000000})
    run.wait_text(r'^Scripting: started on M4 \(2 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    session = run.read_word(M4_SESSION)
    first_text = len(run.texts)
    run.wait_text(r'^M4 number text ok$', timeout=SCRIPT_TIMEOUT)
    run.wait_text(r'^M4 loop \d+$', timeout=SCRIPT_TIMEOUT)
    supervisor = [text for text in run.texts[first_text:] if text.startswith('Scripting: M4')]
    assert not supervisor, 'long numerals gave %s' % supervisor
    assert run.read_word(M4_SESSION) == session, 'the M4 restarted'
    assert not run.m4_halted(), 'cpu2 halted'
    run.wait_alive(10)
