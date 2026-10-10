# AP_FLAKE8_CLEAN
"""Dual-core STM32H757 Renode platform: M4 hold and release, HSEM, boot handshake."""

import os
import re

from pathlib import Path

import pytest

import dual_core_images
from test_peripherals import renode  # noqa: F401

ROOT = Path(__file__).resolve().parents[3]
RENODE_DIR = ROOT / 'Tools' / 'renode'
RCC_GCR = 0x580244A0
PROBE = 0x38800F00
PROBE_BOOT_ADDRESS = 0x081E0000
CORTEX_M4_CPUID = 0x410FC240
TEST_START = 'dual-core test start'
# The AP_* peripherals the H743 base and the overlay instantiate. Not a
# directory filter over stm32/ and common/: AP_STM32_Timer_Actuators.cs needs
# AP_Physics and AP_SPIMultiplexer.cs needs AP_Sigrok, which the filter does
# not include, so they fail to compile; compiling every include of the board
# script would also exceed the 60 s fixture timeout.
PLATFORM_PERIPHERALS = ['AP_STM32_IWDG', 'AP_STM32H7_FlashController', 'AP_STM32_ADC_v3', 'AP_STM32H7_PWR',
                        'AP_STM32H7_RCC', 'AP_STM32H7_HSEM', 'AP_NVIC_RettobaseFix', 'AP_DWT',
                        'AP_STM32H7_SDMMC', 'AP_STM32_OTG']


def peripheral_includes():
    resc = (RENODE_DIR / 'scripts' / 'ardupilot_h743.resc').read_text()
    names = '|'.join(PLATFORM_PERIPHERALS)
    pattern = r'include \$repo/Tools/renode/peripherals/(stm32|common)/(%s)\.cs$' % names
    return [line for line in resc.splitlines() if re.match(pattern, line)]


def platform_script(m7_elf, m4_elf, m4_boot_address=None):
    lines = ['$repo=@%s' % ROOT] + peripheral_includes() + [
        'mach create "dualcore"',
        'machine LoadPlatformDescription @%s' % (RENODE_DIR / 'platforms' / 'stm32h743_base.repl'),
        'machine LoadPlatformDescription @%s' % (RENODE_DIR / 'platforms' / 'stm32h757_m4.repl'),
        'sysbus LoadELF @%s' % m7_elf,
        'sysbus LoadELF @%s' % m4_elf,
        'cpu VectorTableOffset 0x08000000',
        'machine SetSerialExecution true',
        'emulation SetGlobalQuantum "0.00001"',
    ]
    if m4_boot_address is not None:
        lines.append('sysbus.rcc Cm4BootAddress 0x%08X' % m4_boot_address)
    lines.append('echo "%s"' % TEST_START)
    return '\n'.join(lines) + '\n'


def monitor_values(output):
    '''`sysbus ReadDoubleWord` results and boolean property reads, in order.'''
    # SetSerialExecution echoes its previous value; skip the platform setup output.
    return re.findall(r'^(0x[0-9A-F]{8}|True|False)\s*$', output.split(TEST_START, 1)[-1], re.M)


@pytest.fixture
def images(tmp_path):
    def build(*names, cubeframework=None):
        try:
            return [dual_core_images.build(name, tmp_path / 'images', cubeframework) for name in names]
        except FileNotFoundError as error:
            pytest.skip(str(error))
    return build


def test_m4_held_until_boot_c2(renode, images):  # noqa: F811
    idle, probe = images('idle_m7', 'probe_m4')
    output = renode(platform_script(idle, probe, PROBE_BOOT_ADDRESS) + '\n'.join([
        'emulation RunFor "0.02"',
        'cpu2 IsHalted',
        'sysbus ReadDoubleWord 0x%08X' % PROBE,
        'sysbus WriteDoubleWord 0x%08X 0x9' % RCC_GCR,  # WW1RSC | BOOT_C2
        'emulation RunFor "0.02"',
        'cpu2 IsHalted',
        'sysbus ReadDoubleWord 0x%08X' % PROBE,
        'sysbus ReadDoubleWord 0x%08X' % (PROBE + 4),
        'sysbus ReadDoubleWord 0x%08X' % (PROBE + 8),
        'sysbus ReadDoubleWord 0x%08X' % (PROBE + 12),
        'machine Reset',
        'cpu2 IsHalted',
        'sysbus ReadDoubleWord 0x%08X' % RCC_GCR,
    ]) + '\n')
    values = monitor_values(output)
    assert values[:3] == ['True', '0x00000000', 'False'], output
    assert values[3:6] == ['0x5055344D', '0x%08X' % CORTEX_M4_CPUID, '0x%08X' % PROBE_BOOT_ADDRESS], output
    assert 0x38800E00 <= int(values[6], 16) <= 0x38800F00, output  # MSP from the vector table
    assert values[7:] == ['True', '0x00000001'], output  # held again; WW1RSC kept, BOOT_C2 cleared


def test_m7_write_releases_m4(renode, images):  # noqa: F811
    release, probe = images('release_m7', 'probe_m4')
    output = renode(platform_script(release, probe, PROBE_BOOT_ADDRESS) + '\n'.join([
        'emulation RunFor "0.02"',
        'cpu2 IsHalted',
        'sysbus ReadDoubleWord 0x%08X' % RCC_GCR,
        'sysbus ReadDoubleWord 0x%08X' % PROBE,
    ]) + '\n')
    assert monitor_values(output) == ['False', '0x00000008', '0x5055344D'], output


HSEM_RESULT = 0x38001000
HSEM_FIELDS = [
    'm7_two_step_lock', 'm7_wrong_coreid', 'm4_lock_while_held', 'm4_foreign_free',
    'm4_read_lock', 'm7_irq_masked_status', 'm7_irq_count', 'm7_core2_raw_status',
    'm4_irq_masked_status', 'm4_irq_count', 'm7_wrong_key', 'm7_cleared_10',
    'm7_cleared_11', 'm7_cleared_5', 'm4_step', 'm7_step',
]
HSEM_EXPECTED = {
    'm7_two_step_lock': 0x80000300,
    'm7_wrong_coreid': 0,
    'm4_lock_while_held': 0x80000300,
    'm4_foreign_free': 0x80000300,
    'm4_read_lock': 0x80000100,
    'm7_irq_masked_status': 1 << 7,
    'm7_irq_count': 1,
    'm4_irq_masked_status': 1 << 9,
    'm4_irq_count': 1,
    'm7_wrong_key': 0x80000300,
    'm7_cleared_10': 0,
    'm7_cleared_11': 0,
    'm7_cleared_5': 0,
    'm4_step': 2,
    'm7_step': 3,
}


def test_hsem_model(renode, images):  # noqa: F811
    m7, m4 = images('hsem_m7', 'hsem_m4')
    reads = ['sysbus ReadDoubleWord 0x%08X' % (HSEM_RESULT + 4 * index) for index in range(len(HSEM_FIELDS))]
    output = renode(platform_script(m7, m4) + '\n'.join(['emulation RunFor "0.05"'] + reads + [
        'sysbus ReadDoubleWord 0x580264B0',  # RLR[12] from the monitor: no lock
        'sysbus ReadDoubleWord 0x58026430',  # R[12]
    ]) + '\n')
    values = [int(value, 16) for value in monitor_values(output)]
    result = dict(zip(HSEM_FIELDS, values))
    for name, expected in HSEM_EXPECTED.items():
        assert result[name] == expected, '%s = 0x%X\n%s' % (name, result[name], output)
    assert result['m7_core2_raw_status'] & (1 << 7), output
    assert values[len(HSEM_FIELDS):] == [0, 0], output


def test_hsem_debugger_access_has_no_side_effects(renode, images):  # noqa: F811
    idle, probe = images('idle_m7', 'probe_m4')
    output = renode(platform_script(idle, probe) + '\n'.join([
        'sysbus ReadDoubleWord 0x58026480 cpu',              # RLR[0] through the CPU context
        'sysbus ReadDoubleWord 0x58026400',                  # R[0]
        'sysbus WriteDoubleWord 0x58026404 0x80000300 cpu',  # R[1] lock through the CPU context
        'sysbus ReadDoubleWord 0x58026404',
        'sysbus ReadDoubleWord 0x58026518',                  # C2ISR
    ]) + '\n')
    assert monitor_values(output) == ['0x00000000'] * 4, output


MAILBOX = 0x38000000
MAILBOX_FIELDS = [
    'm7_stage', 'm7_peer_status', 'round_trips', 'm7_doorbell_irqs', 'm7_error',
    'm4_stage', 'm4_peer_status', 'pings_answered', 'm4_doorbell_irqs', 'm4_cpu_id',
    'm4_error', 'done',
]
PEER_HEADER_MAGIC = 0x48504643  # "CFPH"
PEER_COMPATIBLE = 2
STAGE_ROUND_TRIPS = 5
STAGE_FINISHED = 6
DONE_MAGIC = 0x444F4E45  # "DONE"
ROUND_TRIPS = 100


def test_boot_handshake(renode, images):  # noqa: F811
    cubeframework = dual_core_images.cubeframework_dir()
    if cubeframework is None:
        pytest.skip('set CUBEFRAMEWORK_DIR to a CubeFramework checkout with Channel and PeerHeader')
    m7, m4 = images('handshake_m7', 'handshake_m4', cubeframework=cubeframework)
    reads = ['sysbus ReadDoubleWord 0x%08X' % (MAILBOX + 4 * index) for index in range(len(MAILBOX_FIELDS))]
    output = renode(platform_script(m7, m4) + '\n'.join([
        'emulation RunFor "0.2"',
        'cpu2 IsHalted',
        'sysbus ReadDoubleWord 0x30040000',  # M7 peer header magic
        'sysbus ReadDoubleWord 0x30040040',  # M4 peer header magic
    ] + reads) + '\n')
    values = monitor_values(output)
    assert values[0] == 'False', output
    assert values[1:3] == ['0x%08X' % PEER_HEADER_MAGIC] * 2, output
    box = dict(zip(MAILBOX_FIELDS, (int(value, 16) for value in values[3:])))
    assert box['m7_error'] == 0 and box['m4_error'] == 0, box
    assert box['m7_peer_status'] == PEER_COMPATIBLE and box['m4_peer_status'] == PEER_COMPATIBLE, box
    assert box['m4_cpu_id'] == CORTEX_M4_CPUID, box
    assert box['round_trips'] == ROUND_TRIPS and box['pings_answered'] == ROUND_TRIPS, box
    assert box['m7_stage'] == STAGE_FINISHED and box['m4_stage'] == STAGE_ROUND_TRIPS, box
    assert box['done'] == DONE_MAGIC, box
    assert box['m7_doorbell_irqs'] >= 1 and box['m4_doorbell_irqs'] >= 1, box


XCORE_BENCH_BASE = {'SRAM3': 0x30040000, 'SRAM4': 0x38000000}
XCORE_BENCH_DONE = 0x58424E44  # "XBND"
XCORE_BENCH_RENODE_SAMPLES = 1000


@pytest.mark.parametrize('region', sorted(XCORE_BENCH_BASE))
def test_xcore_bench_hsem_pipeline(renode, region):  # noqa: F811
    '''Functional only: the round trips complete. Renode cycle counts are not a measurement.'''
    bench = os.environ.get('XCORE_BENCH_DIR')
    if not bench:
        pytest.skip('set XCORE_BENCH_DIR to the CubeFramework xcore_bench worktree')
    build = Path(bench).expanduser() / ('build-xb-renode-%s-HSEM' % region) / 'examples' / 'xcore_bench'
    base = XCORE_BENCH_BASE[region]
    reads = ['sysbus ReadDoubleWord 0x%08X' % (base + 4 * word) for word in range(8, 13)]
    output = renode(platform_script(build / 'xcore_bench_m7', build / 'xcore_bench_m4') +
                    '\n'.join(['emulation RunFor "0.3"', 'cpu2 IsHalted'] + reads) + '\n')
    values = monitor_values(output)
    assert values[0] == 'False', output
    magic, count, p50, p99, maximum = (int(value, 16) for value in values[1:])
    assert magic == XCORE_BENCH_DONE, output
    assert count == XCORE_BENCH_RENODE_SAMPLES, output
    assert p50 <= p99 <= maximum, output
