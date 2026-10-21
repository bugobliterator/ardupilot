# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""M4 build of native modules: which lane serves each BindingId on the M7, its quota class, the lock
its generated adapter takes, and the local set (libm and the AP_Math value adapters) the M4 answers
itself."""

import json
from pathlib import Path

import math_binding

POLICY_PATH = Path(__file__).with_name('m4_policy.json')
# libraries/AP_RustM4/AP_RustM4_Quota.h QuotaClass
QUOTA_CLASSES = [('None', 0), ('Text', 1), ('Storage', 2), ('Led', 3), ('FileLog', 4), ('FileIo', 5), ('Can', 6),
                 ('MavTx', 7), ('Log', 8), ('Osd', 9), ('OsdFrame', 10)]
FNV_OFFSET = 2166136261
FNV_PRIME = 16777619
FLAG_POSTED = 1
FLAG_MIRROR_GET = 2
FLAG_MIRROR_HANDLE_WRITER = 4
FLAG_MIRROR_NAME_WRITER = 8
MIRROR_ROLES = {'get': FLAG_MIRROR_GET, 'handle_writer': FLAG_MIRROR_HANDLE_WRITER,
                'name_writer': FLAG_MIRROR_NAME_WRITER}
# binding_mirror_role() values on the M4 (AP_RustM4_HostCalls.h MirrorRole)
ROLE_VALUES = ((FLAG_MIRROR_GET, 1), (FLAG_MIRROR_HANDLE_WRITER, 2), (FLAG_MIRROR_NAME_WRITER, 3))
# M4_CALL_POLICY bytes (AP_Rust/Bindings.h M4_CALL_*): posted, the mirror role, and for a local id
# 1 + the index of its [input, output] sizes in M4_LOCAL_WIRES
CALL_POSTED = 1
CALL_ROLE_SHIFT = 1
CALL_WIRE_SHIFT = 3
CALL_WIRE_MAX = 0xFF >> CALL_WIRE_SHIFT


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


def check_posted(name, contract, policy):
    '''a posted binding returns nothing the module waits for and is no vehicle command'''
    namespace = name.split('.', 1)[0]
    if namespace in policy['fast_namespaces'] or name in policy['slow_methods']:
        raise ValueError(f'posted binding {name} is a vehicle command (fast namespace or slow method)')
    outputs = [a for a in contract['arguments'] if 'out' in str(a.get('direction', ''))]
    if contract['result'] != 'void' or outputs:
        raise ValueError(f'posted binding {name} has an output')


def policy_table(entries, policy, local):
    quota_values = dict(QUOTA_CLASSES)
    names = {entry['contract']['name']: entry['id'] for entry in entries}
    contracts_by_name = {entry['contract']['name']: entry['contract'] for entry in entries}
    namespaces = {name.split('.', 1)[0] for name in names}
    posted = policy.get('posted', [])
    mirrored = policy.get('mirrored', {})
    stale = ([name for name in policy['slow_methods'] if name not in names] +
             [name for name in policy['quotas'] if name not in names] +
             [name for name in policy['fast_namespaces'] if name not in namespaces] +
             [name for name in posted if name not in names] + [name for name in mirrored if name not in names])
    if stale:
        raise ValueError('stale M4 policy entries: ' + ', '.join(sorted(stale)))
    for name, quota in policy['quotas'].items():
        if quota not in quota_values:
            raise ValueError(f'unknown quota class {quota} for {name}')
    for name, role in mirrored.items():
        if role not in MIRROR_ROLES:
            raise ValueError(f'unknown mirror role {role} for {name}')
    flags = {}
    for name in posted:
        check_posted(name, contracts_by_name[name], policy)
        flags[names[name]] = flags.get(names[name], 0) | FLAG_POSTED
    for name, role in mirrored.items():
        flags[names[name]] = flags.get(names[name], 0) | MIRROR_ROLES[role]
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
        table[binding_id] = (lane, quotas.get(binding_id, 0), flags.get(binding_id, 0))
    return table


# the lock a generated singleton adapter takes (singleton_binding.py BindingLock), for the call
# latency probe: the scheduler's and AP_AHRS's semaphores are named, any other is Other
LOCKS = [('None', 0), ('Scheduler', 1), ('Ahrs', 2), ('Other', 3)]


def lock_table(methods, count):
    '''{id: lock name} for every id below count; hand-written adapters are None'''
    table = {binding_id: 'None' for binding_id in range(count)}
    for method in methods:
        lock = method.get('lock')
        if lock is None:
            continue
        if lock == 'scheduler-semaphore':
            name = 'Scheduler'
        elif lock == 'semaphore' and method['owner'] == 'AP_AHRS':
            name = 'Ahrs'
        else:
            name = 'Other'
        if method['id'] in table:
            table[method['id']] = name
    return table


def catalogue_hash(entries, local, table=None):
    texts = [f'{binding_id}:{binding_hash};'
             for binding_id, binding_hash in sorted({(e['id'], e['hash']) for e in entries if e['id'] in local})]
    if table is not None:
        mirror_mask = FLAG_MIRROR_GET | FLAG_MIRROR_HANDLE_WRITER | FLAG_MIRROR_NAME_WRITER
        texts += [f'P{binding_id};' for binding_id, (_, _, flags) in sorted(table.items()) if flags & FLAG_POSTED]
        texts += [f'M{binding_id}:{flags & mirror_mask};'
                  for binding_id, (_, _, flags) in sorted(table.items()) if flags & mirror_mask]
    state = FNV_OFFSET
    for byte in ''.join(texts).encode('ascii'):
        state = ((state ^ byte) * FNV_PRIME) & 0xFFFFFFFF
    return state


def policy_header(table, local_hash, locks=None, policy=None):
    locks = locks if locks is not None else {}
    lines = ['#pragma once', '#include <stdint.h>', '', 'namespace AP_Rust', '{',
             'enum class M4Lane : uint8_t { Unknown = 0, Fast = 1, Slow = 2, Local = 3 };',
             'enum class M4Lock : uint8_t { ' + ', '.join(f'{name} = {value}' for name, value in LOCKS) + ' };',
             'struct M4BindingPolicy {', '    M4Lane lane;', '    uint8_t quota;', '    uint8_t flags;', '};',
             'static const uint8_t M4_FLAG_POSTED = 1U;', 'static const uint8_t M4_FLAG_MIRROR_GET = 2U;',
             'static const uint8_t M4_FLAG_MIRROR_HANDLE_WRITER = 4U;',
             'static const uint8_t M4_FLAG_MIRROR_NAME_WRITER = 8U;',
             f'static const uint32_t M4_BINDING_COUNT = {len(table)}U;',
             'static const M4BindingPolicy M4_BINDING_POLICY[M4_BINDING_COUNT] = {']
    lines += [f'    {{ M4Lane::{lane}, {quota}, {flags} }}, // {binding_id}'
              for binding_id, (lane, quota, flags) in sorted(table.items())]
    lines += ['};', '// the lock the generated adapter of each id takes',
              'static const M4Lock M4_BINDING_LOCK[M4_BINDING_COUNT] = {']
    lines += [f'    M4Lock::{locks.get(binding_id, "None")}, // {binding_id}' for binding_id in sorted(table)]
    lines += ['};', f'static const uint32_t M4_LOCAL_CATALOGUE_HASH = 0x{local_hash:08X}U;']
    lines += call_policy_declarations(*policy) if policy is not None else []
    lines += ['}', '']
    return '\n'.join(lines)


def local_dispatch(libm, sizes, entries):
    libm_by_id = {method['id']: method for method in libm}
    imports = sorted({(int(entry['hash'], 16), entry['id']) for entry in entries if entry['id'] in sizes})
    lines = ['#include <math.h>', '#include <string.h>', '#include <AP_Rust/Bindings.h>', '#include <AP_Rust/HostApi.h>',
             '', 'namespace AP_Rust', '{', 'static float read_float(const uint8_t *p)', '{',
             '    float value;', '    memcpy(&value, p, sizeof(value));', '    return value;', '}', '',
             'int32_t binding_local_dispatch(uint32_t id, const uint8_t *input, uint32_t input_size,',
             '                               uint8_t *output, uint32_t output_size)', '{',
             '    // exact sizes before any read or write: the adapters read and write fixed sizes',
             '    switch (id) {']
    for binding_id, (ni, no) in sorted(sizes.items()):
        lines += [f'    case {binding_id}: {{', f'        if (input_size != {ni}U || output_size != {no}U) {{',
                  '            return AP_RUST_STATUS_INVALID;', '        }']
        method = libm_by_id.get(binding_id)
        if method is not None:
            arguments = ', '.join(f'read_float(input + {i * 4})' for i in range(method['arity']))
            lines += [f'        const float value = ::{method["name"]}({arguments});',
                      '        memcpy(output, &value, sizeof(value));', '        return AP_RUST_STATUS_OK;', '    }']
        else:
            lines += ['        return binding_math_dispatch(BindingId(id), input, input_size, output, output_size);',
                      '    }']
    lines += ['    default:', '        return AP_RUST_STATUS_INVALID;', '    }', '}', '',
              '// every public name of a local id, by hash: the M4 loader checks the M7 resolved them alike',
              'const LocalImport M4_LOCAL_IMPORTS[] = {']
    lines += [f'    {{ 0x{binding_hash:016X}ULL, {binding_id}U }},' for binding_hash, binding_id in imports]
    lines += ['};', f'const uint32_t M4_LOCAL_IMPORT_COUNT = {len(imports)}U;', '}', '']
    return '\n'.join(lines)


def call_policy(table, sizes):
    '''(first id, [policy byte per id from first], [(input, output) per distinct local wire]): the ids
    the M4 does not forward as plain calls, from the lowest to the highest'''
    wires = sorted(set(sizes.values()))
    if len(wires) > CALL_WIRE_MAX or any(size > 0xFF for wire in wires for size in wire):
        raise ValueError(f'{len(wires)} local wires do not fit the M4 call policy byte')
    policy = {}
    for binding_id, (_, _, flags) in table.items():
        role = next((value for flag, value in ROLE_VALUES if flags & flag), 0)
        wire = wires.index(sizes[binding_id]) + 1 if binding_id in sizes else 0
        value = (CALL_POSTED if flags & FLAG_POSTED else 0) | role << CALL_ROLE_SHIFT | wire << CALL_WIRE_SHIFT
        if value:
            policy[binding_id] = value
    if not policy:
        return 0, [0], [(0, 0)]
    first = min(policy)
    return first, [policy.get(i, 0) for i in range(first, max(policy) + 1)], wires or [(0, 0)]


def call_policy_declarations(first, values, wires):
    return ['// one call policy byte per id from M4_CALL_POLICY_FIRST (Bindings.h M4_CALL_*)',
            f'static const uint32_t M4_CALL_POLICY_FIRST = {first}U;',
            f'static const uint32_t M4_CALL_POLICY_COUNT = {len(values)}U;',
            f'static const uint32_t M4_LOCAL_WIRE_COUNT = {len(wires)}U;',
            'extern const uint8_t M4_CALL_POLICY[M4_CALL_POLICY_COUNT];',
            'extern const uint8_t M4_LOCAL_WIRES[M4_LOCAL_WIRE_COUNT][2];']


def policy_functions(table, sizes):
    first, values, wires = call_policy(table, sizes)
    lines = ['#include <AP_Rust/BindingM4Policy_generated.h>', '', 'namespace AP_Rust', '{',
             'const uint8_t M4_CALL_POLICY[M4_CALL_POLICY_COUNT] = {']
    lines += [f'    {value}U, // {first + offset}' for offset, value in enumerate(values)]
    lines += ['};', '', '// [input bytes, output bytes]', 'const uint8_t M4_LOCAL_WIRES[M4_LOCAL_WIRE_COUNT][2] = {']
    lines += [f'    {{ {ni}U, {no}U }},' for ni, no in wires]
    lines += ['};', '',
              'static uint8_t call_policy(uint32_t id)', '{',
              '    const uint32_t slot = id - M4_CALL_POLICY_FIRST;',
              '    return slot < M4_CALL_POLICY_COUNT ? M4_CALL_POLICY[slot] : 0U;', '}', '',
              'bool binding_is_local(uint32_t id)', '{', '    return (call_policy(id) >> M4_CALL_WIRE_SHIFT) != 0;', '}',
              '',
              'bool binding_local_wire(uint32_t id, uint32_t &input_size, uint32_t &output_size)', '{',
              '    const uint8_t wire = call_policy(id) >> M4_CALL_WIRE_SHIFT;', '    if (wire == 0) {',
              '        return false;', '    }', '    input_size = M4_LOCAL_WIRES[wire - 1][0];',
              '    output_size = M4_LOCAL_WIRES[wire - 1][1];', '    return true;', '}', '',
              'bool binding_is_posted(uint32_t id)', '{', '    return (call_policy(id) & M4_CALL_POSTED) != 0;', '}', '',
              'uint8_t binding_mirror_role(uint32_t id)', '{',
              '    return (call_policy(id) >> M4_CALL_ROLE_SHIFT) & M4_CALL_ROLE_MASK;', '}', '}', '']
    return '\n'.join(lines)


def emit(contract, entries):
    sizes = local_sizes(contract)
    local = set(sizes)
    table = policy_table(entries, read_policy(), local)
    locks = lock_table(contract['methods'], len(table))
    return {'BindingM4Policy_generated.h': policy_header(table, catalogue_hash(entries, local, table), locks,
                                                         call_policy(table, sizes)),
            'BindingLocal_generated.cpp': local_dispatch(contract['libm'], sizes, entries) + '\n' +
            policy_functions(table, sizes)}
