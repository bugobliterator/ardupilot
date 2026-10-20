#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Fail if a native module built for the Cortex-M4 links f64 software-float routines.

The M4's FPU (FPv4-SP) is single precision. f64 arithmetic and f32/f64
conversions in a module become compiler-builtins calls linked into the
module itself. Reads the unstripped ELF that build.py writes beside the
.apmod (same stem, .elf).
"""

import argparse
import re
import sys

from elftools.elf.elffile import ELFFile
from elftools.elf.relocation import RelocationSection

AEABI = re.compile(r'^__aeabi_(d[a-z0-9]+|f2d|i2d|ui2d|l2d|ul2d)$')
GENERIC = re.compile(r'^__((add|sub|mul|div)df3|negdf2|extendsfdf2|truncdfsf2|(fix|fixuns)df[sd]i|'
                     r'float(un)?[sd]idf|(eq|ne|lt|le|gt|ge|cmp|unord)df2)$')


def is_f64_routine(name):
    return bool(AEABI.match(name) or GENERIC.match(name))


def scan(path):
    '''{f64 routine: [functions that call it]} for one ELF.'''
    with open(path, 'rb') as handle:
        elf = ELFFile(handle)
        symtab = elf.get_section_by_name('.symtab')
        if symtab is None:
            raise ValueError(f'{path}: no symbol table (use the .elf build.py writes beside the .apmod)')
        found = {symbol.name: set() for symbol in symtab.iter_symbols() if is_f64_routine(symbol.name)}
        functions = [(symbol['st_value'] & ~1, symbol['st_size'], symbol.name) for symbol in symtab.iter_symbols()
                     if symbol['st_info']['type'] == 'STT_FUNC' and symbol['st_size'] > 0]
        for section in elf.iter_sections():
            if not isinstance(section, RelocationSection) or section['sh_link'] != elf.get_section_index('.symtab'):
                continue
            for relocation in section.iter_relocations():
                name = symtab.get_symbol(relocation['r_info_sym']).name
                if name not in found:
                    continue
                # executables keep virtual addresses in r_offset (--emit-relocs)
                address = relocation['r_offset']
                callers = [function for start, size, function in functions if start <= address < start + size]
                found[name].update(callers or ['0x%08x' % address])
    return {name: sorted(callers) for name, callers in sorted(found.items())}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('elf', nargs='+')
    args = parser.parse_args()
    status = 0
    for path in args.elf:
        found = scan(path)
        if not found:
            print(f'{path}: f64 check OK (no f64 software-float routine)')
            continue
        status = 1
        print(f'{path}: f64 check FAILED')
        for name, callers in found.items():
            print(f'  {name} <- {", ".join(callers) if callers else "(no relocation found)"}')
    return status


if __name__ == '__main__':
    sys.exit(main())
