# AP_FLAKE8_CLEAN
"""Renode test of millis() and micros() on the CubeOrangePlus Cortex-M4: the M4 reads the
M7's clock (TIM5) itself, with the M7's values, and makes no RPC for them after the
first millis(). Functional only: Renode is not cycle-accurate.

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
from test_m4_scripts import M4_SESSION, SCRIPT_TIMEOUT, start_with_scripts, stop_and_read_log  # noqa: E402

LOG_PARAMETERS = {'SCR_DEBUG_OPTS': 8,   # 8: LOG_RUNTIME, SRPC rows
                  'LOG_DISARMED': 1, 'LOG_DARM_RATEMAX': 0, 'LOG_FILE_RATEMAX': 0}
SAMPLES = 50
# a text is logged after the script read the clock: by at most this much emulated time
LOG_DELAY_MS = 50
WRAP = 1 << 32


T_TEXT = r'^T (\d+) (\d+) (\d+)$'


def time_sessions(log):
    '''Per M4 session (split at "started on M4"): the (TimeUS, millis, micros, micros after the
    previous call) of each T row, and the SRPC call totals of the whole log.'''
    reader = DFReader.DFReader_binary(str(log), zero_time_base=False)
    sessions = [[]]
    totals = {}
    while True:
        message = reader.recv_match(type=['MSG', 'SRPC'])
        if message is None:
            return [session for session in sessions if session], totals
        if message.get_type() == 'SRPC':
            totals[message.Name] = totals.get(message.Name, 0) + message.N
            continue
        if 'started on M4' in message.Message:
            sessions.append([])
            continue
        match = re.match(T_TEXT, message.Message)
        if match:
            sessions[-1].append((message.TimeUS, int(match.group(1)), int(match.group(2)), int(match.group(3))))


def check_session(samples):
    '''The M4's clock against the M7's in one session: each text's M7 log time lies between the
    M4's micros() before its call and after it (offset 0, whatever the call latency). Returns the
    largest TimeUS - micros, the latency the bracket allowed.'''
    for time_us, millis, micros, _ in samples:
        # the M7's millis() and micros() at the read: behind the logging time by little
        assert 0 <= time_us // 1000 - millis <= LOG_DELAY_MS, (time_us, millis)
        assert 0 <= (time_us - micros) % WRAP <= LOG_DELAY_MS * 1000, (time_us, micros)
        # micros() was read just after millis()
        assert 0 <= (micros - millis * 1000) % WRAP < 5000, (millis, micros)
    for (time_us, _, before, _), (_, _, _, after) in zip(samples, samples[1:]):
        assert 0 <= (time_us - before) % WRAP <= (after - before) % WRAP < WRAP // 2, \
            'M7 log time %u outside the M4 micros() bracket [%u, %u]' % (time_us, before, after)
    for (_, _, before, _), (_, _, after, _) in zip(samples, samples[1:]):
        step = (after - before) % WRAP
        assert 0 < step < WRAP // 2, 'micros() went from %u to %u' % (before, after)
    return max((time_us - micros) % WRAP for time_us, _, micros, _ in samples)


def test_m4_local_time(start_m4lua, tmp_path):  # noqa: F811
    run = start_with_scripts(start_m4lua, ['tests/m4_time.lua'], LOG_PARAMETERS)
    run.wait_text(r'^Scripting: started on M4 \(1 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    session = run.read_word(M4_SESSION)
    first_text = len(run.texts)
    for _ in range(SAMPLES + 2):
        run.wait_text(T_TEXT, timeout=SCRIPT_TIMEOUT)
    supervisor = [text for text in run.texts[first_text:] if text.startswith('Scripting: M4')]
    assert not supervisor, 'the M4 reported %s' % supervisor
    assert run.read_word(M4_SESSION) == session, 'the M4 restarted'
    run.wait_alive(10)
    sessions, totals = time_sessions(stop_and_read_log(run, tmp_path / 'logs'))
    samples = sessions[-1]
    print('T samples %u, SRPC %s' % (len(samples), sorted(totals.items())), flush=True)
    assert len(samples) >= SAMPLES, 'only %u T rows logged' % len(samples)
    check_session(samples)
    # micros() never crossed to the M7; millis() once, for the TIM5 wrap count
    assert totals.get('global:micros', 0) == 0, totals
    assert totals.get('global:millis', 0) <= 1, totals


def test_m4_local_time_after_fault_restart(start_m4lua, tmp_path):  # noqa: F811
    # after a fault (warm) restart the M4 reads the M7's clock with the offset it has after boot:
    # none. The M7 log time of every text is inside the M4's micros() bracket of its call in both
    # sessions, so a slower call after the restart cannot be taken for a clock offset
    run = start_with_scripts(start_m4lua, ['tests/m4_time.lua'], LOG_PARAMETERS)
    run.wait_text(r'^Scripting: started on M4 \(1 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    for _ in range(SAMPLES // 2):
        run.wait_text(T_TEXT, timeout=SCRIPT_TIMEOUT)
    run.set('SCR_M4_INJECT', 1)
    run.wait_text(r'^Scripting: M4 HardFault ', timeout=SCRIPT_TIMEOUT)
    run.wait_text(r'^Scripting: M4 restarted \(1/\d+\)$', timeout=SCRIPT_TIMEOUT)
    run.set('SCR_M4_INJECT', 0)
    run.wait_text(r'^Scripting: started on M4 \(1 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    for _ in range(SAMPLES // 2):
        run.wait_text(T_TEXT, timeout=SCRIPT_TIMEOUT)
    run.wait_alive(10)
    sessions, totals = time_sessions(stop_and_read_log(run, tmp_path / 'logs'))
    assert len(sessions) >= 2, [len(session) for session in sessions]
    boot, restarted = sessions[-2], sessions[-1]
    assert len(boot) >= SAMPLES // 4 and len(restarted) >= SAMPLES // 4, (len(boot), len(restarted))
    latency = [check_session(boot), check_session(restarted)]
    print('largest TimeUS - micros: after boot %u us, after the fault restart %u us; SRPC %s' %
          (latency[0], latency[1], sorted(totals.items())), flush=True)
    # each session calibrates once (one remote millis()), micros() is never remote
    assert totals.get('global:micros', 0) == 0, totals
    assert totals.get('global:millis', 0) <= 2, totals
