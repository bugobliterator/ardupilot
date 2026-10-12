# AP_FLAKE8_CLEAN
"""Renode test of the pure-data userdata on the CubeOrangePlus Cortex-M4: Vector2f,
Vector3f, Quaternion and Location run on the M4 itself, with the results the M7 gives
in-process, and make no RPC. The reference is the stock CubeOrangePlus firmware running
the same script in-process. Functional only: Renode is not cycle-accurate.

Needs RENODE and the stock and the M4 builds:
  ./waf configure --board CubeOrangePlus && ./waf copter
  ./waf configure --board CubeOrangePlus-M4Lua && ./waf copter
"""

import os
import re
import signal
import subprocess
import sys

from decimal import Decimal, InvalidOperation
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from test_m4_bindings import LOG_PARAMETERS, srpc_totals  # noqa: E402
from test_m4_containment import M4Run, ROOT, TEST_DEADLINE, start_m4lua  # noqa: E402,F401
from test_m4_scripts import SCRIPT_TIMEOUT, start_with_scripts, upload_scripts  # noqa: E402
from test_mission import unused_tcp_port  # noqa: E402

STOCK_BOARD = 'CubeOrangePlus'
STOCK_ELF = ROOT / 'build' / STOCK_BOARD / 'bin' / 'arducopter'
SCRIPT = 'tests/m4_pure_data.lua'
PD_DONE = r'^PD done texts=(\d+)$'
FIRST_LINE_TIMEOUT = 120   # wall seconds from the start text: a script uploaded as 0 bytes sends nothing
# SRPC names of the types the M4 runs itself, and the Location rows that stay remote
LOCAL_TYPE_PREFIXES = ('Vector2f:', 'Vector3f:', 'Quaternion:', 'Location:')
LOCATION_STATE_ROWS = {'Location:change_alt_frame', 'Location:get_alt_m', 'Location:get_vector_from_origin_NEU',
                       'Location:get_vector_from_origin_NEU_m', 'Location:get_vector_from_origin_NEU_cm'}
# the Location calls that read M7 state stay remote, by name and count in m4_pure_data.lua
LOCATION_STATE_CALLS = {'Location:change_alt_frame': 2, 'Location:get_alt_m': 1,
                        'Location:get_vector_from_origin_NEU_m': 1}
# R2: two float ulp on every float token, on Renode only
FLOAT_ULP_TOLERANCE = 2
# the 59 PD lines of m4_pure_data.lua and its "PD done" text
PD_LINE_COUNT = 60
# PD line numbers (1-based) of the tolerated M4 and M7 differences
# F1: M7 vfma.f64 against the M4's soft double: one integer on these lines
INTEGER_LINES = set(range(26, 35)) | {37, 39, 40, 41}
# F4: AP printf on the M7 against newlib on the M4: the last printed digit
PRINTF_DIGIT_LINES = {37, 39, 40, 41}


def local_rows(totals):
    '''SRPC rows that name a method the M4 should have run itself.'''
    return {name: count for name, count in totals.items()
            if name.startswith(LOCAL_TYPE_PREFIXES) and name not in LOCATION_STATE_ROWS}


def pure_data_lines(run):
    '''The PD lines of m4_pure_data.lua, in order, and its text count, once it is done.'''
    done = run.wait_text(PD_DONE, timeout=SCRIPT_TIMEOUT)
    lines = [text for text in run.texts if text.startswith('PD ')]
    assert len(lines) == PD_LINE_COUNT, '%u PD lines, expected %u: %s' % (len(lines), PD_LINE_COUNT, lines)
    return lines, int(re.match(PD_DONE, done).group(1))


def float_ordinal(token):
    '''Monotonic integer of the float whose bit pattern is the 8 hex digits of token.'''
    bits = int(token, 16)
    return -(bits & 0x7fffffff) if bits & 0x80000000 else bits


def tolerated_difference(line_number, got, want):
    '''Why two differing tokens of a PD line are tolerated, or None.'''
    # a hex letter in either token: an 8-digit decimal integer is not a float bit pattern
    if (re.fullmatch(r'[0-9a-f]{8}', got) and re.fullmatch(r'[0-9a-f]{8}', want) and re.search('[a-f]', got + want)
            and abs(float_ordinal(got) - float_ordinal(want)) <= FLOAT_ULP_TOLERANCE):
        return 'R2 float ulp +-%u' % FLOAT_ULP_TOLERANCE
    if (line_number in INTEGER_LINES and re.fullmatch(r'-?\d+', got) and re.fullmatch(r'-?\d+', want)
            and abs(int(got) - int(want)) <= 1):
        return 'F1 integer +-1'
    if line_number in PRINTF_DIGIT_LINES:
        try:
            step = 10 ** max(Decimal(got).as_tuple().exponent, Decimal(want).as_tuple().exponent)
            if len(got) == len(want) and abs(Decimal(got) - Decimal(want)) <= step:
                return 'F4 last printed digit'
        except (InvalidOperation, TypeError):   # TypeError: nan or inf against a number
            pass
    return None


def compare_pure_data(lines, reference):
    '''Asserts the PD lines of the M4 match the in-process ones, except the tolerated differences, which are logged.'''
    assert len(lines) == len(reference), '%u PD lines on the M4, %u in-process' % (len(lines), len(reference))
    tolerated = 0
    for index, (got, want) in enumerate(zip(lines, reference)):
        line_number = index + 1
        if got == want:
            continue
        got_tokens = got.split(' ')
        want_tokens = want.split(' ')
        assert len(got_tokens) == len(want_tokens) and got_tokens[:2] == want_tokens[:2], \
            'PD line %u: M4 %r, in-process %r' % (line_number, got, want)
        for position, (got_token, want_token) in enumerate(zip(got_tokens, want_tokens)):
            # R1: the M7's printf prints %x uppercase, the M4's newlib lowercase
            if got_token == want_token or (re.fullmatch(r'[0-9a-fA-F]{8}', got_token)
                                           and got_token.lower() == want_token.lower()):
                continue
            reason = tolerated_difference(line_number, got_token.lower(), want_token.lower())
            assert reason, 'PD line %u: M4 %r, in-process %r (token %u)' % (line_number, got, want, position)
            tolerated += 1
            print('TOLERATED PD %02u token %u: M4 %s, in-process %s (%s)' % (
                line_number, position, got_token, want_token, reason), flush=True)
    print('PD lines compared: %u, tolerated token differences: %u' % (len(lines), tolerated), flush=True)


@pytest.fixture(scope='module')
def in_process_reference(tmp_path_factory):
    '''PD lines of the script run in-process by the stock CubeOrangePlus firmware.'''
    renode = os.environ.get('RENODE')
    if not renode or not STOCK_ELF.is_file():
        pytest.skip('set RENODE and build %s (arducopter)' % STOCK_BOARD)
    tmp_path = tmp_path_factory.mktemp('stock')
    state_dir = tmp_path / 'state'
    state_dir.mkdir()
    runs = []

    def deadline_passed(signum, frame):
        pytest.fail('the in-process reference passed its %u s wall-clock deadline' % TEST_DEADLINE)

    previous_handler = signal.signal(signal.SIGALRM, deadline_passed)
    signal.alarm(TEST_DEADLINE)

    def start():
        uart_port = unused_tcp_port()
        monitor_port = unused_tcp_port()
        while monitor_port == uart_port:
            monitor_port = unused_tcp_port()
        log_path = tmp_path / ('renode-%u.log' % len(runs))
        command = [sys.executable, str(ROOT / 'Tools' / 'renode' / 'run.py'), STOCK_BOARD,
                   '--vehicle', 'arducopter', '--elf', str(STOCK_ELF),
                   '--state-dir', str(state_dir), '--uart-port', str(uart_port),
                   '--port', str(monitor_port), '--unthrottled', '--renode', renode]
        env = dict(os.environ, XDG_CONFIG_HOME=str(state_dir), TMPDIR=str(state_dir))
        with log_path.open('w') as log:
            process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        run = M4Run(process, log_path, uart_port, monitor_port)
        run.sdcard_image = state_dir / 'sdcard.img'
        runs.append(run)
        run.connect()
        return run

    try:
        first = start()
        first.set('SCR_ENABLE', 1)
        upload_scripts(first, [SCRIPT])
        first.stop()
        run = start()
        lines, _ = pure_data_lines(run)
        run.stop()
        print('in-process reference: %u PD lines' % len(lines), flush=True)
    finally:
        for run in runs:
            run.stop()
        signal.alarm(0)
        signal.signal(signal.SIGALRM, previous_handler)
    yield lines


def test_m4_pure_data_matches_in_process(in_process_reference, start_m4lua, tmp_path):  # noqa: F811
    run = start_with_scripts(start_m4lua, [SCRIPT], LOG_PARAMETERS)
    run.wait_text(r'^Scripting: started on M4 \(1 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    run.wait_text(r'^PD 01 ', timeout=FIRST_LINE_TIMEOUT)
    lines, texts = pure_data_lines(run)
    compare_pure_data(lines, in_process_reference)
    # the statistics window of the last texts is logged, and the log written out, within two summaries
    for _ in range(2):
        run.wait_text(r'^Scripting RPC \d+/s p50<\d+ p99<\d+ max \d+us$', timeout=SCRIPT_TIMEOUT)
    run.wait_alive(10)
    totals = srpc_totals(run, tmp_path / 'logs')   # stops Renode
    print('SRPC logged %s' % sorted(totals.items()), flush=True)
    assert not local_rows(totals), 'SRPC rows for local pure-data methods: %s' % local_rows(totals)
    logged = sum(count for name, count in totals.items()
                 if name == 'm4:other' or not (name.startswith('m4:') or name == 'global:print'))
    state_calls = {name: totals.get(name, 0) for name in LOCATION_STATE_CALLS}
    assert state_calls == LOCATION_STATE_CALLS, 'Location state calls %s, expected %s' % (
        state_calls, LOCATION_STATE_CALLS)
    expected = texts + sum(LOCATION_STATE_CALLS.values())
    assert logged == expected, 'SRPC binding calls: %u logged, the script sent %u texts and %u Location state calls (%s)' % (
        logged, texts, sum(LOCATION_STATE_CALLS.values()), totals)
