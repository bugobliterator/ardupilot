# AP_FLAKE8_CLEAN
"""Build bare-metal STM32H757 test images for the Renode dual-core tests."""

import os
import shutil
import subprocess

from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SOURCES = Path(__file__).resolve().parent / 'dual_core'
CMSIS = ROOT / 'modules' / 'ChibiOS' / 'os' / 'common' / 'ext'

CORES = {
    'm7': {
        'cpu': ['-mcpu=cortex-m7', '-mthumb', '-mfloat-abi=hard', '-mfpu=fpv5-d16'],
        'defines': ['-DCORE_CM7', '-DHSEM_IRQ_NUMBER=125'],
        'flash': (0x08000000, 0x100000),
        'ram': (0x24000000, 0x80000),
    },
    'm4': {
        'cpu': ['-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=hard', '-mfpu=fpv4-sp-d16'],
        'defines': ['-DCORE_CM4', '-DHSEM_IRQ_NUMBER=126'],
        'flash': (0x08100000, 0x100000),
        'ram': (0x30000000, 0x20000),
    },
}

# name -> (core, sources, needs CubeFramework, memory layout overrides)
PROGRAMS = {
    'idle_m7': ('m7', ['idle_m7.c'], False, {}),
    'release_m7': ('m7', ['release_m7.c'], False, {}),
    # Tail of bank 2 and backup SRAM: also usable next to real ArduPilot M7
    # firmware, which uses neither.
    'probe_m4': ('m4', ['probe_m4.c'], False,
                 {'flash': (0x081E0000, 0x20000), 'ram': (0x38800000, 0xF00)}),
}

CUBEFRAMEWORK_SOURCES = [
    'CubePilotFW/Channel.cpp',
    'CubePilotFW/SpscRing.cpp',
    'CubePilotFW/PeerHeader.cpp',
    'CubePilotFW/pal/HSEM.cpp',
]

LINKER_SCRIPT = '''ENTRY(Reset_Handler)
MEMORY
{{
    FLASH (rx) : ORIGIN = 0x{flash_origin:08X}, LENGTH = 0x{flash_length:X}
    RAM (rwx) : ORIGIN = 0x{ram_origin:08X}, LENGTH = 0x{ram_length:X}
}}
_estack = ORIGIN(RAM) + LENGTH(RAM);
SECTIONS
{{
    .vectors : {{ KEEP(*(.vectors)) }} > FLASH
    .text : {{ *(.text*) *(.rodata*) . = ALIGN(4); }} > FLASH
    .ARM.exidx : {{ *(.ARM.exidx*) }} > FLASH
    .init_array : {{
        __init_array_start = .;
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        __init_array_end = .;
    }} > FLASH
    _sidata = LOADADDR(.data);
    .data : {{ _sdata = .; *(.data*) . = ALIGN(4); _edata = .; }} > RAM AT > FLASH
    .bss (NOLOAD) : {{ _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; }} > RAM
    end = _ebss;
}}
'''


def cubeframework_dir():
    value = os.environ.get('CUBEFRAMEWORK_DIR')
    return Path(value).expanduser() if value else None


def _run(command):
    result = subprocess.run([str(part) for part in command], capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError('%s\n%s%s' % (' '.join(map(str, command)), result.stdout, result.stderr))


def build(name, outdir, cubeframework=None):
    '''Compile and link PROGRAMS[name]; return the ELF path.'''
    core_name, sources, needs_cubeframework, layout = PROGRAMS[name]
    core = dict(CORES[core_name], **layout)
    if needs_cubeframework and cubeframework is None:
        raise ValueError('%s needs a CubeFramework checkout (CUBEFRAMEWORK_DIR)' % name)
    gcc = shutil.which('arm-none-eabi-gcc')
    gxx = shutil.which('arm-none-eabi-g++')
    if gcc is None or gxx is None:
        raise FileNotFoundError('arm-none-eabi-gcc and arm-none-eabi-g++ are required')
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    linker = outdir / (name + '.ld')
    linker.write_text(LINKER_SCRIPT.format(
        flash_origin=core['flash'][0], flash_length=core['flash'][1],
        ram_origin=core['ram'][0], ram_length=core['ram'][1]))
    includes = ['-I%s' % SOURCES,
                '-I%s' % (CMSIS / 'ARM' / 'CMSIS' / 'Core' / 'Include'),
                '-I%s' % (CMSIS / 'ST' / 'STM32H7xx')]
    files = [SOURCES / 'startup.c', SOURCES / 'stub_pal.c'] + [SOURCES / source for source in sources]
    if needs_cubeframework:
        includes.append('-I%s' % cubeframework)
        files += [Path(cubeframework) / source for source in CUBEFRAMEWORK_SOURCES]
    common = (core['cpu'] + core['defines'] + includes +
              ['-DSTM32H757xx', '-DCF_PLATFORM_BETAFLIGHT', '-Os', '-g',
               '-ffunction-sections', '-fdata-sections', '-Wall'])
    objects = []
    for source in files:
        target = outdir / ('%s-%s.o' % (name, source.stem))
        if source.suffix == '.cpp':
            compiler = [gxx, '-std=c++17', '-fno-exceptions', '-fno-rtti', '-fno-threadsafe-statics']
        else:
            compiler = [gcc, '-std=c11', '-Wno-override-init']
        _run(compiler + common + ['-c', source, '-o', target])
        objects.append(target)
    elf = outdir / (name + '.elf')
    _run([gxx] + core['cpu'] + ['-nostartfiles', '--specs=nano.specs', '--specs=nosys.specs',
                                '-Wl,--gc-sections', '-T', linker] + objects + ['-o', elf])
    return elf
