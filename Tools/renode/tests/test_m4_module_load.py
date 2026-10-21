# AP_FLAKE8_CLEAN
"""Renode: the M4 loads a native Rust module from the SD card and starts it unprivileged.

m4_hello (thumbv7em-none-eabihf) loads, prints its host text and its M7-served gcs.send_text,
and its remote import is named in the M7's SRPC rows. m4_faults with MOD_M4_INJECT 14 writes
to its own code: a MemManage fault from module code (W^X), after which the M4 restarts and the
module loads again. Functional only: Renode is not cycle-accurate.

Needs RENODE, rustc with thumbv7em-none-eabihf, pyfatfs and:
  ./waf configure --board CubeOrangePlus-SimOnHardWare-M4Module && ./waf plane
"""

import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
from pymavlink import DFReader

sys.path.insert(0, str(Path(__file__).resolve().parent))

from m4_module_harness import (RCC_GCR, ROOT, START_TIMEOUT, start_m4,  # noqa: E402, F401
                             stop_and_read_log, wait_all)
import fat_image  # noqa: E402  (Tools/renode, on the path the harness sets)

MODULES = ROOT / 'libraries' / 'AP_Module' / 'tests' / 'modules'
# the M4's peer header half (M4_PEER_MAGIC) and its fault record's CFSR word
M4_FAULT_CFSR = 0x30040040 + 32
CFSR_IACCVIOL = 1 << 0
CFSR_DACCVIOL = 1 << 1
MODULE_AREA = (0x30008000, 0x30040000)
# the module tools need pyelftools, which the Renode environment may lack
TOOLS_PYTHON = os.environ.get('RUST_TOOLS_PYTHON', shutil.which('python3') or sys.executable)


def build_module(name, directory):
    '''m4 build of a test module (stack 16384); fails on any f64 software-float routine in it.'''
    output = directory / (name + '.apmod')
    try:
        subprocess.run([TOOLS_PYTHON, str(ROOT / 'Tools' / 'modules' / 'rust' / 'build.py'), str(MODULES / (name + '.rs')),
                        str(output), '--target', 'thumbv7em-none-eabihf', '--stack-size', '16384'],
                       cwd=ROOT, check=True, capture_output=True, text=True)
    except subprocess.CalledProcessError as error:
        pytest.fail('build.py %s failed:\n%s%s' % (name, error.stdout, error.stderr))
    check = subprocess.run([TOOLS_PYTHON, str(ROOT / 'Tools' / 'modules' / 'check_f64.py'),
                            str(output.with_suffix('.elf'))], cwd=ROOT, capture_output=True, text=True)
    assert check.returncode == 0, 'check_f64 %s:\n%s%s' % (name, check.stdout, check.stderr)
    return output


def install_module(image, module):
    '''Writes module into the board's SD card image as /APM/rust/<name>, alone in that directory.'''
    fat_image._require_pyfatfs()
    with fat_image.PyFatFS(str(image)) as filesystem:
        filesystem.makedirs('/APM/rust', recreate=True)
        for name in filesystem.listdir('/APM/rust'):
            filesystem.remove('/APM/rust/' + name)
        with filesystem.openbin('/APM/rust/' + module.name, 'w') as stream:
            stream.write(module.read_bytes())


def start_with_module(start_m4, module, parameters):  # noqa: F811
    '''Sets the parameters on a first run (which creates the SD card), installs the module, and
    starts the board again so the M4 loads it.'''
    first = start_m4()
    for name, value in parameters.items():
        first.set(name, value)
    first.wait_alive(5)
    first.stop()
    install_module(first.sdcard_image, module)
    run = start_m4()
    for name, value in parameters.items():
        assert run.get(name) == value, 'board state did not persist across the relaunch: %s' % name
    text = run.wait_m4_started()
    assert text == 'Module: started on M4 (1 module, M4 RAM 224KB)', text
    return run


def test_m4_module_loads_m4_hello(start_m4, tmp_path):  # noqa: F811
    module = build_module('m4_hello', tmp_path)
    run = start_with_module(start_m4, module, {'MOD_ENABLE': 1, 'LOG_DISARMED': 1})
    rcc_before = run.read_word(RCC_GCR)
    run.wait_text(r'^Module: loaded 1 modules$', timeout=START_TIMEOUT)
    # the module's host text is queued on the M7 (as the PR's Runtime::text), its gcs.send_text is not
    wait_all(run, [r'^Module: m4 hello$', r'^m4 hello gcs$'], START_TIMEOUT, failure=r'load failed|missing API')
    run.wait_alive(10)
    assert run.read_word(RCC_GCR) == rcc_before
    failures = [text for text in run.texts if text.startswith('Module: M4 ') or 'load failed' in text or
                'missing API' in text]
    assert not failures, failures
    # the first load is over before the log starts (about 10 s of boot time in Renode): a
    # MOD_RESTART reloads the module, whose send_text then gets an SRPC row
    run.set('MOD_RESTART', 1)
    run.wait_text(r'^Module: loaded 1 modules$', timeout=START_TIMEOUT)
    wait_all(run, [r'^m4 hello gcs$'], START_TIMEOUT, failure=r'load failed|missing API')
    run.wait_alive(5)
    log = stop_and_read_log(run, tmp_path / 'logs')
    reader = DFReader.DFReader_binary(str(log), zero_time_base=True)
    names = set()
    while True:
        message = reader.recv_match(type='SRPC')
        if message is None:
            break
        names.add(message.Name)
    assert any('send_text' in name for name in names), 'no SRPC row named for send_text: %s' % names


def test_m4_module_code_is_not_writable(start_m4, tmp_path):  # noqa: F811
    module = build_module('m4_faults', tmp_path)
    run = start_with_module(start_m4, module, {'MOD_ENABLE': 1, 'MOD_M4_RST_MAX': 5})
    run.wait_text(r'^Module: loaded 1 modules$', timeout=START_TIMEOUT)
    rcc_before = run.read_word(RCC_GCR)
    run.set('MOD_M4_INJECT', 14)
    # the module's code ran (its text) and its write to its own code faulted (MemManage)
    run.wait_text(r'^Module: m4 fault 14$')
    detail = run.wait_text(r'^Module: M4 (MemManage|HardFault|BusFault|UsageFault)')
    run.set('MOD_M4_INJECT', 0)
    match = re.match(r'^Module: M4 MemManage PC=0x([0-9A-F]{8})$', detail)
    assert match, 'not a MemManage at a PC: %r' % detail
    pc = int(match.group(1), 16)
    assert MODULE_AREA[0] <= pc < MODULE_AREA[1], 'fault PC 0x%08X is not module code' % pc
    cfsr = run.read_word(M4_FAULT_CFSR)
    print('W^X fault: %s, CFSR 0x%08X' % (detail, cfsr), flush=True)
    assert cfsr & CFSR_DACCVIOL and not cfsr & CFSR_IACCVIOL, 'not a data access violation: CFSR 0x%08X' % cfsr
    run.wait_text(r'^Module: M4 fault \(MemManage\), restarting \(\d/5\)$')
    # the M4 restarts, loads the module again and keeps running; the M7 never stopped
    run.wait_text(r'^Module: loaded 1 modules$', timeout=START_TIMEOUT)
    run.wait_alive(10)
    assert run.read_word(RCC_GCR) == rcc_before
    off = [text for text in run.texts if 'off until reboot' in text]
    assert not off, off
