#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Whether a native module fits the CubeOrangePlus M4's module area with the staged loader.

Mirrors AP_ModuleM4_CodeWindow.h and AP_ModuleM4_LoadLayout.h: the whole file is
staged at the bottom of the module area, RX is loaded top-aligned in an MPU
region ending at the shared window, RW directly below it; after the load the
stack (above a 32 B guard) and the heap are carved from the bottom. The stack
is the image's required stack plus the host callback margin, the heap its
initial heap; MOD_STACK_MAX and MOD_HEAP_MAX only cap them (module_host.cpp).
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import pack  # noqa: E402

AREA_BASE = 0x30008000   # AP_MODULE_M4_RAM_BASE + AP_MODULE_M4_PRIV_RAM_SIZE
AREA_SIZE = 229376       # up to the shared window
TOP = AREA_BASE + AREA_SIZE
GUARD = 32
CODE_WINDOW_MIN = 256
CODE_WINDOW_MAX = 128 * 1024
STACK_MARGIN = 4096        # the PR's thumbv7em host callback margin (Module.cpp)
HEAP_BITMAP_MAX = 65536    # MODULE_HEAP_MAX in module_host.h


def round_up(value, alignment):
    return (value + alignment - 1) & ~(alignment - 1)


def code_window(rx_size, rx_alignment, top=TOP):
    if rx_size <= 0 or rx_size > CODE_WINDOW_MAX or rx_alignment <= 0 or rx_alignment & (rx_alignment - 1):
        return None
    size = CODE_WINDOW_MIN
    while size <= CODE_WINDOW_MAX:
        if size >= rx_size:
            span = round_up(rx_size, max(size // 8, rx_alignment))
            if span <= size:
                return dict(base=top - size, size=size, rx_base=top - span)
        size *= 2
    return None


def module_sizes(stack_required, heap_initial, stack_max, heap_max):
    """The stack and heap the loader gives a module, or the reason it refuses the image."""
    if heap_max < 256 or stack_max <= STACK_MARGIN:
        return dict(refused='invalid memory limits')
    if stack_required > stack_max - STACK_MARGIN:
        return dict(refused='stack exceeds MOD_STACK_MAX')
    if heap_initial > heap_max:
        return dict(refused='initial heap exceeds MOD_HEAP_MAX')
    if heap_initial > HEAP_BITMAP_MAX:
        return dict(refused='insufficient M4 RAM to load')
    return dict(stack_size=stack_required + STACK_MARGIN, heap_size=heap_initial)


def layout(file_size, rx_size, rx_alignment, rw_memory_size, rw_alignment, stack_size, heap_size,
           area_base=AREA_BASE, area_size=AREA_SIZE):
    top = area_base + area_size
    window = code_window(rx_size, rx_alignment, top)
    if window is None:
        return dict(result='code-too-large')
    rw_base = (window['rx_base'] - rw_memory_size) & ~(rw_alignment - 1)
    stack_top = area_base + GUARD + round_up(stack_size, 32)
    heap_end = round_up(stack_top, 16) + heap_size
    load_peak = file_size + (top - rw_base)
    run_need = (heap_end - area_base) + (top - rw_base)
    if area_base + file_size > rw_base:
        result = 'no-room-to-stage'
    elif heap_end > rw_base:
        result = 'no-room-to-run'
    else:
        result = 'ok'
    return dict(result=result, window=window, rw_base=rw_base, load_peak=load_peak, run_need=run_need,
                load_margin=area_size - load_peak, run_margin=area_size - run_need)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('module', type=Path)
    parser.add_argument('--stack-max', type=int, default=32768, help='MOD_STACK_MAX')
    parser.add_argument('--heap-max', type=int, default=32768, help='MOD_HEAP_MAX')
    args = parser.parse_args()
    data = args.module.read_bytes()
    module = pack.unpack(data)
    contract = module['contract']
    if contract is None:
        print('result: refused (unsupported image format)')
        return 1
    rx_size = module['rx_size']
    rw_memory = module['memory_size'] - rx_size
    print(f'module: {args.module}')
    print(f'file {len(data)} B, RX {rx_size} B (alignment {module["rx_alignment"]}), '
          f'RW memory {rw_memory} B (alignment {module["rw_alignment"]})')
    print(f'declared stack {contract["stack_size"]} B, initial heap {contract["heap_size"]} B')
    sizes = module_sizes(contract['stack_size'], contract['heap_size'], args.stack_max, args.heap_max)
    if 'refused' in sizes:
        print(f'result: refused ({sizes["refused"]})')
        return 1
    fit = layout(len(data), rx_size, module['rx_alignment'], rw_memory, module['rw_alignment'],
                 sizes['stack_size'], sizes['heap_size'])
    if fit['result'] == 'code-too-large':
        print('result: code-too-large (RX above 128 KB: no region-7 window)')
        return 1
    window = fit['window']
    print(f'region 7: base 0x{window["base"]:08x} size {window["size"]} B, RX at 0x{window["rx_base"]:08x}')
    print(f'load-time peak {fit["load_peak"]} B of {AREA_SIZE} (margin {fit["load_margin"]})')
    print(f'run-time need {fit["run_need"]} B of {AREA_SIZE} (margin {fit["run_margin"]}) '
          f'at stack {sizes["stack_size"]}, heap {sizes["heap_size"]}')
    print(f'result: {fit["result"]}')
    return 0 if fit['result'] == 'ok' else 1


if __name__ == '__main__':
    sys.exit(main())
