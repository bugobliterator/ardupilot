# AP_FLAKE8_CLEAN
"""Renode: the M4 Rust image boots on CubeOrangePlus-SimOnHardWare-M4Module, publishes its
peer header, passes its CRC self-check and is supervised by the M7. Functional only: Renode is not cycle-accurate.

Needs RENODE and: ./waf configure --board CubeOrangePlus-SimOnHardWare-M4Module && ./waf plane
"""

import re
import sys
from pathlib import Path

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parent))

from m4_module_harness import M4_PEER_MAGIC, RCC_GCR, start_m4  # noqa: E402, F401

MEMINFO_ID = 152


def test_m4_module_boots(start_m4):  # noqa: F811
    first = start_m4()
    first.set('MOD_ENABLE', 1)   # read at boot: relaunch on the same board state
    first.wait_alive(10)
    first.stop()
    run = start_m4()
    assert run.get('MOD_ENABLE') == 1
    text = run.wait_m4_started()
    assert re.fullmatch(r'Module: started on M4 \(0 modules, M4 RAM 224KB\)', text), text
    assert run.read_word(M4_PEER_MAGIC) != 0
    rcc_before = run.read_word(RCC_GCR)
    run.wait_alive(30)
    assert run.read_word(RCC_GCR) == rcc_before
    # no M4 fault, restart or stop: a failed CRC self-check faults the M4 before its first beat
    failures = [text for text in run.texts if text.startswith('Module: M4 ')]
    assert not failures, failures
    # the variant's defaults.parm keeps the SimOnHardWare chain
    assert run.get('SCHED_LOOP_RATE') == 400
    assert run.get('AHRS_EKF_TYPE') == 10
    assert run.get('GPS1_TYPE') == 100
    # stop line: M7 free RAM at boot >= 32768 B
    run.mav.mav.command_long_send(run.mav.target_system, run.mav.target_component,
                                  mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                  MEMINFO_ID, 1000000, 0, 0, 0, 0, 0)
    meminfo = run.recv('MEMINFO')
    freemem = meminfo.freemem32 if getattr(meminfo, 'freemem32', 0) else meminfo.freemem
    print('M7 MEMINFO freemem %u' % freemem, flush=True)
    assert freemem >= 32768, 'M7 free RAM %u below the 32768 B stop line' % freemem
