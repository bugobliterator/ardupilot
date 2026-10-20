# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""M4 build of native modules: which lane serves each BindingId on the M7, its quota class, and the
local set (libm and the AP_Math value adapters) the M4 answers itself."""

import json
from pathlib import Path

import math_binding

POLICY_PATH = Path(__file__).with_name('m4_policy.json')
# libraries/AP_ModuleM4/AP_ModuleM4_Quota.h QuotaClass
QUOTA_CLASSES = [('None', 0), ('Text', 1), ('Storage', 2), ('Led', 3), ('FileLog', 4), ('FileIo', 5), ('Can', 6),
                 ('MavTx', 7), ('Log', 8), ('Osd', 9), ('OsdFrame', 10)]
FNV_OFFSET = 2166136261
FNV_PRIME = 16777619


def read_policy():
    return json.loads(POLICY_PATH.read_text())


def local_sizes(contract):
    '''{local id: (input bytes, output bytes)}: the exact wire of every binding the M4 answers'''
    sizes = {}
    for owner in contract['math']['types']:
        for method in owner['methods']:
            inputs, outputs = math_binding.wire_fields(owner['name'], method)
            sizes[method['id']] = tuple(sum(math_binding.size(kind) for _, kind in fields)
                                        for fields in (inputs, outputs))
    for method in contract['libm']:
        sizes[method['id']] = (method['arity'] * 4, 4)
    return sizes


def local_ids(contract):
    return set(local_sizes(contract))


def policy_table(entries, policy, local):
    quota_values = dict(QUOTA_CLASSES)
    names = {entry['contract']['name']: entry['id'] for entry in entries}
    namespaces = {name.split('.', 1)[0] for name in names}
    stale = ([name for name in policy['slow_methods'] if name not in names] +
             [name for name in policy['quotas'] if name not in names] +
             [name for name in policy['fast_namespaces'] if name not in namespaces])
    if stale:
        raise ValueError('stale M4 policy entries: ' + ', '.join(sorted(stale)))
    for name, quota in policy['quotas'].items():
        if quota not in quota_values:
            raise ValueError(f'unknown quota class {quota} for {name}')
    lanes = {}
    quotas = {}
    for name, binding_id in names.items():
        quota = quota_values[policy['quotas'].get(name, 'None')]
        fast = (name.split('.', 1)[0] in policy['fast_namespaces'] and name not in policy['slow_methods'] and
                quota == 0)
        lanes.setdefault(binding_id, set()).add('Local' if binding_id in local else 'Fast' if fast else 'Slow')
        quotas[binding_id] = max(quota, quotas.get(binding_id, 0))
    table = {}
    for binding_id in range(max(names.values()) + 1):
        found = lanes.get(binding_id, set())
        # an alias (two public names, one id) is fast only if every name is
        lane = next((choice for choice in ('Local', 'Slow', 'Fast') if choice in found), 'Unknown')
        table[binding_id] = (lane, quotas.get(binding_id, 0))
    return table


def catalogue_hash(entries, local):
    state = FNV_OFFSET
    for binding_id, binding_hash in sorted({(e['id'], e['hash']) for e in entries if e['id'] in local}):
        for byte in f'{binding_id}:{binding_hash};'.encode('ascii'):
            state = ((state ^ byte) * FNV_PRIME) & 0xFFFFFFFF
    return state


def policy_header(table, local_hash):
    lines = ['#pragma once', '#include <stdint.h>', '', 'namespace AP_Module', '{',
             'enum class M4Lane : uint8_t { Unknown = 0, Fast = 1, Slow = 2, Local = 3 };',
             'struct M4BindingPolicy {', '    M4Lane lane;', '    uint8_t quota;', '};',
             f'static const uint32_t M4_BINDING_COUNT = {len(table)}U;',
             'static const M4BindingPolicy M4_BINDING_POLICY[M4_BINDING_COUNT] = {']
    lines += [f'    {{ M4Lane::{lane}, {quota} }}, // {binding_id}' for binding_id, (lane, quota) in sorted(table.items())]
    lines += ['};', f'static const uint32_t M4_LOCAL_CATALOGUE_HASH = 0x{local_hash:08X}U;', '}', '']
    return '\n'.join(lines)


def local_dispatch(libm, sizes, entries):
    libm_by_id = {method['id']: method for method in libm}
    imports = sorted({(int(entry['hash'], 16), entry['id']) for entry in entries if entry['id'] in sizes})
    lines = ['#include <math.h>', '#include <string.h>', '#include <AP_Module/Bindings.h>', '#include <AP_Module/HostApi.h>',
             '', 'namespace AP_Module', '{', 'static float read_float(const uint8_t *p)', '{',
             '    float value;', '    memcpy(&value, p, sizeof(value));', '    return value;', '}', '',
             'bool binding_is_local(uint32_t id)', '{', '    switch (id) {']
    lines += [f'    case {binding_id}:' for binding_id in sorted(sizes)]
    lines += ['        return true;', '    default:', '        return false;', '    }', '}', '',
              'bool binding_local_wire(uint32_t id, uint32_t &input_size, uint32_t &output_size)', '{',
              '    switch (id) {']
    lines += [f'    case {binding_id}: input_size = {ni}U; output_size = {no}U; return true;'
              for binding_id, (ni, no) in sorted(sizes.items())]
    lines += ['    default:', '        return false;', '    }', '}', '',
              'int32_t binding_local_dispatch(uint32_t id, const uint8_t *input, uint32_t input_size,',
              '                               uint8_t *output, uint32_t output_size)', '{',
              '    // exact sizes before any read or write: the adapters read and write fixed sizes',
              '    switch (id) {']
    for binding_id, (ni, no) in sorted(sizes.items()):
        lines += [f'    case {binding_id}: {{', f'        if (input_size != {ni}U || output_size != {no}U) {{',
                  '            return AP_MODULE_STATUS_INVALID;', '        }']
        method = libm_by_id.get(binding_id)
        if method is not None:
            arguments = ', '.join(f'read_float(input + {i * 4})' for i in range(method['arity']))
            lines += [f'        const float value = ::{method["name"]}({arguments});',
                      '        memcpy(output, &value, sizeof(value));', '        return AP_MODULE_STATUS_OK;', '    }']
        else:
            lines += ['        return binding_math_dispatch(BindingId(id), input, input_size, output, output_size);',
                      '    }']
    lines += ['    default:', '        return AP_MODULE_STATUS_INVALID;', '    }', '}', '',
              '// every public name of a local id, by hash: the M4 loader checks the M7 resolved them alike',
              'const LocalImport M4_LOCAL_IMPORTS[] = {']
    lines += [f'    {{ 0x{binding_hash:016X}ULL, {binding_id}U }},' for binding_hash, binding_id in imports]
    lines += ['};', f'const uint32_t M4_LOCAL_IMPORT_COUNT = {len(imports)}U;', '}', '']
    return '\n'.join(lines)


def emit(contract, entries):
    sizes = local_sizes(contract)
    local = set(sizes)
    table = policy_table(entries, read_policy(), local)
    return {'BindingM4Policy_generated.h': policy_header(table, catalogue_hash(entries, local)),
            'BindingLocal_generated.cpp': local_dispatch(contract['libm'], sizes, entries)}
