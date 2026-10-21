# AP_FLAKE8_CLEAN
"""Renode: the M4's hardware CRC (STM32_CRC model on the H743 base platform, 0x58024C00) gives
zlib.crc32's results, as the M4 reports them in its Config request. If the model returns zeros
(no CRC model on the M4 bus), the test skips with that reason and the CRC check of Bench AirShow
becomes mandatory.

Needs RENODE and: ./waf configure --board CubeOrangePlus-SimOnHardWare-M4Module && ./waf plane
"""

import re
import sys
import zlib
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from m4_module_harness import STEP_TIMEOUT, start_m4  # noqa: E402, F401

PATTERN = bytes(((i * 7 + 3) & 0xFF) for i in range(2048))


def test_m4_hardware_crc_matches_zlib(start_m4):  # noqa: F811
    first = start_m4()
    first.set('MOD_ENABLE', 1)
    first.set('MOD_M4_OPTS', 2)   # bit 1: the self-check values, once per session
    first.wait_alive(10)
    first.stop()
    run = start_m4()
    started = run.wait_m4_started()
    assert re.fullmatch(r'Module: started on M4 \(0 modules, M4 RAM 224KB\)', started), started
    text = run.wait_text(r'Module: M4 CRC [0-9A-F]{8} [0-9A-F]{8}', STEP_TIMEOUT)
    short, pattern = (int(word, 16) for word in text.split()[-2:])
    print('M4 CRC self-check 0x%08X 0x%08X' % (short, pattern), flush=True)
    if short == 0 and pattern == 0:
        pytest.skip('Renode STM32_CRC model returns 0: no CRC model on the M4 bus; Bench AirShow must verify')
    assert short == zlib.crc32(b'123456789')
    assert pattern == zlib.crc32(PATTERN)
    run.wait_alive(10)
    failures = [text for text in run.texts if text.startswith('Module: M4 ') and not text.startswith('Module: M4 CRC ')]
    failures += [text for text in run.texts if 'CRC unit check failed' in text]
    assert not failures, failures
