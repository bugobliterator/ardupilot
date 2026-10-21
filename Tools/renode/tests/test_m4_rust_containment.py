# AP_FLAKE8_CLEAN
"""Renode: faults the M4 Rust service performs (RUST_M4_INJECT 1 to 6, 11, and 12 on the M7) are
contained. Each gives its text and an M4 restart after the backoff, the M7 keeps running and
RCC_GCR is never written; a lockup or a fault inside an M4 handler turns the M4 off until reboot.
The module fault kinds (the m4_faults test module) are not covered here. Functional only: Renode
is not cycle-accurate.

Needs RENODE and:
  ./waf configure --board CubeOrangePlus-SimOnHardWare-M4Rust && ./waf plane
"""

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from m4_rust_harness import M4_PC, RCC_GCR, START_TIMEOUT, STEP_TIMEOUT, start_m4  # noqa: E402, F401

RESTART_MAX = 8


def start_with_parameters(start_m4, parameters):  # noqa: F811
    '''RUST_ENABLE is read at boot: set the parameters, then start the board again.'''
    first = start_m4()
    for name, value in parameters.items():
        first.set(name, value)
    first.wait_alive(5)
    first.stop()
    run = start_m4()
    for name, value in parameters.items():
        assert run.get(name) == value, 'board state did not persist across the relaunch: %s' % name
    run.wait_m4_started()
    # the M7's time (SYSTEM_TIME, 2 Hz) at each text's arrival, after the harness recorded the text:
    # the backoff counts M7 time
    run.text_times = []

    def record(connection, message):
        if message.get_type() == 'STATUSTEXT':
            run.text_times.append((run.boot_ms, run.texts[-1]))
    run.mav.message_hooks.append(record)
    return run


def restart_wait_ms(run, restart_text):
    '''M7 milliseconds from the restart text to the next start text.'''
    times = run.text_times
    index = max(i for (i, (_, text)) in enumerate(times) if text == restart_text)
    started = next(ms for (ms, text) in times[index + 1:] if text.startswith('Rust: started on M4 ('))
    return started - times[index][0]


def inject(run, kind):
    '''RUST_M4_INJECT acts on a change: back to 0 first.'''
    run.set('RUST_M4_INJECT', 0)
    run.set('RUST_M4_INJECT', kind)


def run_fault_steps(run, steps):
    '''Each (kind, detail patterns, reason, restarts): its texts and restarts, each after its backoff of
    M7 time; the M7 keeps running and RCC_GCR is never written.'''
    rcc_before = run.read_word(RCC_GCR)
    used = 0
    for kind, details, reason, restarts in steps:
        inject(run, kind)
        for restart in range(restarts):
            used += 1
            for detail in details:
                run.wait_text(detail)
            restart_text = 'Rust: M4 fault (%s), restarting (%u/%u)' % (reason, used, RESTART_MAX)
            run.wait_text('^' + re.escape(restart_text) + '$')
            # Renode runs the M7 well below real time: the backoff takes long in wall time
            run.wait_text(r'^Rust: started on M4 \(', timeout=START_TIMEOUT + STEP_TIMEOUT)
            waited = restart_wait_ms(run, restart_text)
            backoff = min(1000 << (used - 1), 30000)
            print('kind %u: restart %u after %u ms of M7 time (backoff %u ms)' % (kind, used, waited, backoff),
                  flush=True)
            # SYSTEM_TIME comes twice a second
            assert waited >= backoff - 1000, 'restart %u after %u ms, before its %u ms backoff' % (used, waited, backoff)
        run.wait_alive(5)
        assert run.read_word(RCC_GCR) == rcc_before, 'kind %u: RCC_GCR written' % kind
    off = [text for text in run.texts if 'off until reboot' in text]
    assert not off, off


def test_m4_rust_service_faults_restart_with_backoff(start_m4):  # noqa: F811
    run = start_with_parameters(start_m4, {'RUST_ENABLE': 1, 'RUST_M4_RST_MAX': RESTART_MAX})
    run_fault_steps(run, (
        (1, [r'^Rust: M4 HardFault ' + M4_PC + '$'], 'HardFault', 1),
        (2, [r'^Rust: M4 MemManage ' + M4_PC + '$'], 'MemManage', 1),
        (3, [], 'no heartbeat', 1),
    ))


def test_m4_rust_ring_faults_and_flood(start_m4):  # noqa: F811
    run = start_with_parameters(start_m4, {'RUST_ENABLE': 1, 'RUST_M4_RST_MAX': RESTART_MAX})
    run_fault_steps(run, (
        (5, [], 'bad frames', 1),
        # a HardFault, then the restarted M4 faults right after it publishes its header
        (12, [r'^Rust: M4 HardFault ' + M4_PC + '$'], 'HardFault', 2),
    ))
    # a request flood costs the M7 bounded time and is no fault
    rcc_before = run.read_word(RCC_GCR)
    flood_start = len(run.texts)
    inject(run, 6)
    run.wait_alive(15)
    faults = [text for text in run.texts[flood_start:] if text.startswith('Rust: M4 ')]
    assert not faults, 'flood: %s' % faults
    assert run.read_word(RCC_GCR) == rcc_before


def test_m4_rust_lockup_turns_the_m4_off(start_m4):  # noqa: F811
    run = start_with_parameters(start_m4, {'RUST_ENABLE': 1, 'RUST_M4_RST_MAX': 3})
    rcc_before = run.read_word(RCC_GCR)
    # interrupts off: no heartbeat, and the restart doorbell is never taken
    inject(run, 4)
    run.wait_text(r'^Rust: M4 fault \(no heartbeat\), restarting \(1/3\)$')
    run.wait_text(r'^Rust: M4 not responding, off until reboot$')
    run.wait_alive(10)
    assert run.read_word(RCC_GCR) == rcc_before
    run.arm_expect('Rust: M4 not running')


def test_m4_rust_handler_fault_turns_the_m4_off(start_m4):  # noqa: F811
    run = start_with_parameters(start_m4, {'RUST_ENABLE': 1, 'RUST_M4_RST_MAX': 3})
    rcc_before = run.read_word(RCC_GCR)
    # a fault inside an M4 interrupt handler: reported and granted a restart, which parks
    inject(run, 11)
    run.wait_text(r'^Rust: M4 HardFault ' + M4_PC + '$')
    run.wait_text(r'^Rust: M4 fault \(HardFault\), restarting \(1/3\)$')
    run.wait_text(r'^Rust: M4 not responding, off until reboot$')
    run.wait_alive(10)
    assert run.read_word(RCC_GCR) == rcc_before
