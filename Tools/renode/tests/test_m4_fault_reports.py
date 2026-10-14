# AP_FLAKE8_CLEAN
"""Renode tests of how the M7 reports M4 faults that the containment tests do not reach: a fault of
a restarted M4 right after it publishes its header, alone and with a STOP taken before its heartbeat
times out. Functional only: Renode is not cycle-accurate.

Needs RENODE and a CubeOrangePlus-M4Lua build:
  ./waf configure --board CubeOrangePlus-M4Lua && ./waf copter
"""

import os
import re
import sys

from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pymavlink import mavutil  # noqa: E402
from test_m4_containment import BUILD, start_m4lua  # noqa: E402,F401
from test_m4_scripts import SCRIPT_TIMEOUT, m4_symbols, start_with_scripts, symbol_range  # noqa: E402

M4_ELF = Path(os.environ.get('M4LUA_M4_ELF', BUILD / 'scripting_m4.elf'))
FAULT_PC = r'PC=(0x[0-9A-F]{8})$'


def expect_supervisor_texts(run, start, patterns):
    '''The "Scripting: M4" texts after start are exactly patterns, in order; returns them.'''
    texts = [text for text in run.texts[start:] if text.startswith('Scripting: M4')]
    print('supervisor texts: %s' % texts, flush=True)
    assert len(texts) == len(patterns) and all(re.search(p, t) for p, t in zip(patterns, texts)), \
        'supervisor texts %s, expected %s' % (texts, patterns)
    return texts


def test_m4_early_fault_is_reported(start_m4lua):  # noqa: F811
    '''A restarted M4 that faults right after its publish() (SCR_M4_INJECT 12) is reported as that
    fault with its PC, charged once; before the M2 fix the M7 said "heartbeat lost".'''
    symbols = m4_symbols(M4_ELF)
    # the trap is in inject_fault() when it is not inlined into main()
    ranges = [symbol_range(symbols, 'main')]
    if re.search(r'(?m) inject_fault\(', symbols):
        ranges.append(symbol_range(symbols, 'inject_fault'))
    run = start_with_scripts(start_m4lua, ['tests/m4_loop.lua'], {'SCR_M4_RST_MAX': 3})
    run.wait_text(r'^M4 loop 2$', timeout=SCRIPT_TIMEOUT)
    start = len(run.texts)
    run.set('SCR_M4_INJECT', 12)
    run.wait_text(re.escape('Scripting: M4 restarted (2/3)'), timeout=SCRIPT_TIMEOUT)
    run.wait_text(r'^M4 loop 2$', timeout=SCRIPT_TIMEOUT)
    texts = expect_supervisor_texts(run, start, [
        r'^Scripting: M4 HardFault ' + FAULT_PC, r'^Scripting: M4 restarting \(1/3\)$',
        r'^Scripting: M4 restarted \(1/3\)$',
        r'^Scripting: M4 HardFault ' + FAULT_PC, r'^Scripting: M4 restarting \(2/3\)$',
        r'^Scripting: M4 restarted \(2/3\)$'])
    pc = int(re.search(FAULT_PC, texts[3]).group(1), 16)
    # not the first fault's record reported again: that one is the service's, in inject_fault()
    assert texts[3] != texts[0], 'the early fault repeats the first fault: %s' % texts[3]
    assert any(low <= pc < high for low, high in ranges), \
        'the early fault at PC=0x%08X is outside %s' % (pc, ['0x%08X-0x%08X' % r for r in ranges])
    run.wait_alive(10)


def test_m4_stop_in_the_early_fault_window(start_m4lua):  # noqa: F811
    '''A STOP taken by the early-faulting session before its heartbeat times out: the fault is still
    reported and charged, and the stop then acts on its fault restart (no script runs after it).'''
    run = start_with_scripts(start_m4lua, ['tests/m4_loop.lua'], {'SCR_M4_RST_MAX': 3})
    run.wait_text(r'^M4 loop 2$', timeout=SCRIPT_TIMEOUT)
    start = len(run.texts)
    run.set('SCR_M4_INJECT', 12)
    run.wait_text(re.escape('Scripting: M4 restarting (1/3)'), timeout=SCRIPT_TIMEOUT)
    # held in AP_Scripting's flags while the M4 restarts; taken at the new session's first pass
    run.mav.mav.command_long_send(run.mav.target_system, run.mav.target_component,
                                  mavutil.mavlink.MAV_CMD_SCRIPTING, 0,
                                  mavutil.mavlink.SCRIPTING_CMD_STOP, 0, 0, 0, 0, 0, 0)
    run.wait_text(re.escape('Scripting: M4 restarted (2/3)'), timeout=SCRIPT_TIMEOUT)
    run.wait_alive(20)
    texts = expect_supervisor_texts(run, start, [
        r'^Scripting: M4 HardFault ' + FAULT_PC, r'^Scripting: M4 restarting \(1/3\)$',
        r'^Scripting: M4 restarted \(1/3\)$',
        r'^Scripting: M4 HardFault ' + FAULT_PC, r'^Scripting: M4 restarting \(2/3\)$',
        r'^Scripting: M4 restarted \(2/3\)$'])
    after = run.texts[start:]
    assert 'Scripting: stopped' in after, 'no "Scripting: stopped" in %s' % after
    # the stop came between the early fault's restart and its end: it reached the session in its
    # start window, not after the heartbeat timeout reported the fault
    assert after.index(texts[4]) < after.index('Scripting: stopped') < after.index(texts[5]), \
        'the STOP missed the start window: %s' % after
    loops = [text for text in after[after.index(texts[5]):] if re.match(r'^M4 loop \d+$', text)]
    assert not loops, 'the stopped session ran a script: %s' % loops
