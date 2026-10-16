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


def test_m4_local_time(start_m4lua, tmp_path):  # noqa: F811
    run = start_with_scripts(start_m4lua, ['tests/m4_time.lua'], LOG_PARAMETERS)
    run.wait_text(r'^Scripting: started on M4 \(1 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    session = run.read_word(M4_SESSION)
    first_text = len(run.texts)
    for _ in range(SAMPLES + 2):
        run.wait_text(r'^T \d+ \d+$', timeout=SCRIPT_TIMEOUT)
    supervisor = [text for text in run.texts[first_text:] if text.startswith('Scripting: M4')]
    assert not supervisor, 'the M4 reported %s' % supervisor
    assert run.read_word(M4_SESSION) == session, 'the M4 restarted'
    run.wait_alive(10)
    log = stop_and_read_log(run, tmp_path / 'logs')
    reader = DFReader.DFReader_binary(str(log), zero_time_base=False)
    samples = []
    totals = {}
    while True:
        message = reader.recv_match(type=['MSG', 'SRPC'])
        if message is None:
            break
        if message.get_type() == 'SRPC':
            totals[message.Name] = totals.get(message.Name, 0) + message.N
            continue
        match = re.match(r'^T (\d+) (\d+)$', message.Message)
        if match:
            samples.append((message.TimeUS, int(match.group(1)), int(match.group(2))))
    print('T samples %u, SRPC %s' % (len(samples), sorted(totals.items())), flush=True)
    assert len(samples) >= SAMPLES, 'only %u T rows logged' % len(samples)
    for time_us, millis, micros in samples:
        # the M7's millis() and micros() at the read: behind the logging time by little
        assert 0 <= time_us // 1000 - millis <= LOG_DELAY_MS, (time_us, millis)
        assert 0 <= (time_us - micros) % WRAP <= LOG_DELAY_MS * 1000, (time_us, micros)
        # micros() was read just after millis()
        assert 0 <= (micros - millis * 1000) % WRAP < 5000, (millis, micros)
    for (_, _, before), (_, _, after) in zip(samples, samples[1:]):
        step = (after - before) % WRAP
        assert 0 < step < WRAP // 2, 'micros() went from %u to %u' % (before, after)
    # micros() never crossed to the M7; millis() once, for the TIM5 wrap count
    assert totals.get('global:micros', 0) == 0, totals
    assert totals.get('global:millis', 0) <= 1, totals
