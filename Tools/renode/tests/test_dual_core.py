# AP_FLAKE8_CLEAN
"""Dual-core STM32H757 Renode platform: M4 hold and release, HSEM, boot handshake."""

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
PLATFORM_PERIPHERALS = ['AP_STM32_IWDG', 'AP_STM32H7_FlashController', 'AP_STM32_ADC_v3', 'AP_STM32H7_PWR',
                        'AP_STM32H7_RCC', 'AP_NVIC_RettobaseFix', 'AP_DWT', 'AP_STM32H7_SDMMC', 'AP_STM32_OTG']


def peripheral_includes():
    resc = (RENODE_DIR / 'scripts' / 'ardupilot_h743.resc').read_text()
    # Only the peripherals the H743 base and the overlay instantiate: compiling
    # every include of the board script would exceed the fixture timeout.
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
