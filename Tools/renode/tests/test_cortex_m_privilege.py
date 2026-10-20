# AP_FLAKE8_CLEAN
"""MPU PRIVDEFENA and PPB privilege checks; requires patches/cortex-m-privilege.patch."""

import re
import shutil
import subprocess

import pytest

from test_peripherals import renode  # noqa: F401

RESULT = 0x20000100
SUCCESS = 0x600D0000
MEMMANAGE = 0x4D454D00
BUSFAULT = 0x42555300
HARDFAULT = 0x48415200
CORTEX_M4_CPUID = 0x410FC240

# Handlers record: marker, CFSR, HFSR, BFAR, MMFAR at RESULT.
VECTORS = '''
.syntax unified
.cpu cortex-m4
.thumb
.section .vectors,"a"
.word 0x20004000
.word reset
.word fault
.word hardfault
.word memmanage
.word busfault
.rept 10
.word fault
.endr
.section .text,"ax"
.macro record marker
 ldr r0, =0x20000100
 ldr r1, =\\marker
 str r1, [r0]
 ldr r2, =0xe000ed28
 ldr r1, [r2]
 str r1, [r0, #4]
 ldr r2, =0xe000ed2c
 ldr r1, [r2]
 str r1, [r0, #8]
 ldr r2, =0xe000ed38
 ldr r1, [r2]
 str r1, [r0, #12]
 ldr r2, =0xe000ed34
 ldr r1, [r2]
 str r1, [r0, #16]
 b .
.endm
.thumb_func
fault:
 record 0xbad00000
.thumb_func
hardfault:
 record 0x48415200
.thumb_func
memmanage:
 record 0x4d454d00
.thumb_func
busfault:
 record 0x42555300
.thumb_func
reset:
'''

ENABLE_FAULTS = ' ldr r0, =0xe000ed24\n ldr r1, =0x70000\n str r1, [r0]\n'
DROP_PRIVILEGE = ' movs r0, #1\n msr control, r0\n isb\n'
SUCCESS_TAIL = ' ldr r0, =0x20000100\n ldr r1, =0x600d0000\n str r1, [r0]\n b .\n'
READS = ''.join('sysbus ReadDoubleWord 0x%08X\n' % (RESULT + 4 * index) for index in range(6))


def mpu_setup(privdefena):
    '''Region 0: flash 64 KB read-only, executable. Region 1: RAM 0x20000000
    16 KB read-write, XN (holds the stack and RESULT). 0x20008000 is outside
    every region.'''
    return '''
 ldr r0, =0xe000ed98
 movs r1, #0
 str r1, [r0]
 ldr r0, =0xe000ed9c
 ldr r1, =0x08000000
 str r1, [r0]
 ldr r0, =0xe000eda0
 ldr r1, =0x0600001f
 str r1, [r0]
 ldr r0, =0xe000ed98
 movs r1, #1
 str r1, [r0]
 ldr r0, =0xe000ed9c
 ldr r1, =0x20000000
 str r1, [r0]
 ldr r0, =0xe000eda0
 ldr r1, =0x1300001b
 str r1, [r0]
 ldr r0, =0xe000ed94
 movs r1, #%d
 str r1, [r0]
 dsb
 isb
''' % (5 if privdefena else 1)


def results(output):
    words = re.findall(r'^(0x[0-9A-F]{8})\s*$', output, re.M)[-6:]
    return [int(word, 16) for word in words]


@pytest.fixture
def image(tmp_path, renode):  # noqa: F811
    compiler = shutil.which('arm-none-eabi-gcc')
    if compiler is None:
        pytest.skip('arm-none-eabi-gcc is required')
    platform = tmp_path / 'cpu.repl'
    platform.write_text('''
cpu: CPU.CortexM @ sysbus
    cpuType: "cortex-m4"
    nvic: nvic
    numberOfMPURegions: 8
nvic: IRQControllers.NVIC @ sysbus 0xE000E000
    IRQ -> cpu@0
flash: Memory.MappedMemory @ sysbus 0x08000000
    size: 0x10000
ram: Memory.MappedMemory @ sysbus 0x20000000
    size: 0x10000
''')
    members = renode(f'mach create\nmachine LoadPlatformDescription @{platform}\ncpu\n')
    if 'BusFaultAddress' not in members:
        pytest.skip('Renode requires Tools/renode/patches/cortex-m-privilege.patch')
    linker = tmp_path / 'cpu.ld'
    linker.write_text('ENTRY(reset)\nSECTIONS { . = 0x08000000; .vectors : { *(.vectors) } '
                      '.text : { *(.text*) } }')

    def build(body):
        source = tmp_path / 'cpu.S'
        source.write_text(VECTORS + body)
        elf = tmp_path / 'cpu.elf'
        subprocess.run([compiler, '-nostdlib', '-mcpu=cortex-m4', '-mthumb',
                        '-T', str(linker), str(source), '-o', str(elf)], check=True, capture_output=True)
        return (f'mach create\nmachine LoadPlatformDescription @{platform}\n'
                f'sysbus LoadELF @{elf}\ncpu Step 400\n{READS}')

    return build


@pytest.mark.parametrize('privdefena', [True, False])
def test_privdefena_controls_privileged_background_access(image, renode, privdefena):  # noqa: F811
    output = renode(image(ENABLE_FAULTS + mpu_setup(privdefena) +
                          ' ldr r0, =0x20008000\n ldr r1, [r0]\n' + SUCCESS_TAIL))
    marker, cfsr, _hfsr, _bfar, mmfar, _ = results(output)
    if privdefena:
        assert marker == SUCCESS, output
    else:
        assert marker == MEMMANAGE, output
        assert cfsr & 0x82 == 0x82, hex(cfsr)  # MMARVALID | DACCVIOL
        assert mmfar == 0x20008000, hex(mmfar)


def test_privileged_ppb_ignores_mpu(image, renode):  # noqa: F811
    output = renode(image(ENABLE_FAULTS + mpu_setup(False) +
                          ' ldr r0, =0xe000ed00\n ldr r1, [r0]\n'
                          ' ldr r0, =0x20000114\n str r1, [r0]\n' + SUCCESS_TAIL))
    marker, _cfsr, _hfsr, _bfar, _mmfar, cpuid = results(output)
    assert marker == SUCCESS, output
    assert cpuid == CORTEX_M4_CPUID, hex(cpuid)


def test_unprivileged_scs_read_busfaults(image, renode):  # noqa: F811
    output = renode(image(ENABLE_FAULTS + DROP_PRIVILEGE +
                          ' ldr r0, =0xe000ed00\n ldr r1, [r0]\n' + SUCCESS_TAIL))
    marker, cfsr, _hfsr, bfar, _mmfar, _ = results(output)
    assert marker == BUSFAULT, output
    assert cfsr & 0x8200 == 0x8200, hex(cfsr)  # BFARVALID | PRECISERR
    assert bfar == 0xE000ED00, hex(bfar)


def test_unprivileged_aircr_write_busfaults(image, renode):  # noqa: F811
    output = renode(image(ENABLE_FAULTS + DROP_PRIVILEGE +
                          ' ldr r0, =0xe000ed0c\n ldr r1, =0x05fa0004\n str r1, [r0]\n dsb\n' +
                          SUCCESS_TAIL))
    marker, cfsr, _hfsr, bfar, _mmfar, _ = results(output)
    assert marker == BUSFAULT, output  # a reset would rerun reset: and never reach a handler
    assert cfsr & 0x8200 == 0x8200, hex(cfsr)
    assert bfar == 0xE000ED0C, hex(bfar)


def test_unprivileged_scs_access_escalates_without_busfaultena(image, renode):  # noqa: F811
    output = renode(image(DROP_PRIVILEGE + ' ldr r0, =0xe000ed00\n ldr r1, [r0]\n' + SUCCESS_TAIL))
    marker, _cfsr, hfsr, _bfar, _mmfar, _ = results(output)
    assert marker == HARDFAULT, output
    assert hfsr & 0x40000000, hex(hfsr)  # FORCED
