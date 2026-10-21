# AP_FLAKE8_CLEAN
import json

import pytest

import generate
import m4_binding
from bindings import ROOT

QUOTA = dict(m4_binding.QUOTA_CLASSES)


def entry(binding_id, name, result='void', arguments=()):
    return dict(id=binding_id, hash='%016x' % binding_id,
                contract=dict(name=name, arguments=list(arguments), result=result))


ENTRIES = [entry(1, 'ahrs.get_location'), entry(1, 'ahrs.get_position'), entry(2, 'vehicle.set_mode'),
           entry(3, 'gcs.send_text'), entry(4, 'file.read_at'), entry(5, 'libm.sinf'), entry(6, 'Vector3f.length'),
           entry(8, 'ahrs.set_home'), entry(9, 'logger.write'),
           entry(10, 'Parameter.get', 'bool-u8', [dict(type='f32-le', direction='optional-out')]),
           entry(11, 'Parameter.set', 'bool-u8'), entry(12, 'param.set', 'bool-u8')]
POLICY = dict(fast_namespaces=['ahrs', 'vehicle'], slow_methods=['vehicle.set_mode', 'ahrs.set_home'],
              quotas={'gcs.send_text': 'Text'}, posted=['logger.write'],
              mirrored={'Parameter.get': 'get', 'Parameter.set': 'handle_writer', 'param.set': 'name_writer'})
LOCAL = {5, 6}


def test_lanes_and_quotas():
    table = m4_binding.policy_table(ENTRIES, POLICY, LOCAL)
    assert table[1] == ('Fast', QUOTA['None'], 0)
    assert table[2] == ('Slow', QUOTA['None'], 0)          # listed slow method of a fast namespace
    assert table[3] == ('Slow', QUOTA['Text'], 0)
    assert table[4] == ('Slow', QUOTA['None'], 0)          # file access is never fast
    assert table[5] == ('Local', QUOTA['None'], 0) and table[6] == ('Local', QUOTA['None'], 0)
    assert table[7] == ('Unknown', QUOTA['None'], 0)       # gap in the ids
    assert table[0] == ('Unknown', QUOTA['None'], 0)


def test_stale_policy_names_fail():
    for broken in (dict(POLICY, slow_methods=['vehicle.no_such']), dict(POLICY, quotas={'gcs.no_such': 'Text'}),
                   dict(POLICY, fast_namespaces=['no_such'])):
        with pytest.raises(ValueError, match='stale M4 policy'):
            m4_binding.policy_table(ENTRIES, broken, LOCAL)


def test_unknown_quota_class_fails():
    with pytest.raises(ValueError, match='quota class'):
        m4_binding.policy_table(ENTRIES, dict(POLICY, quotas={'gcs.send_text': 'Nope'}), LOCAL)


def test_posted_and_mirror_flags():
    table = m4_binding.policy_table(ENTRIES, POLICY, LOCAL)
    assert table[9] == ('Slow', QUOTA['None'], m4_binding.FLAG_POSTED)
    assert table[10][2] == m4_binding.FLAG_MIRROR_GET
    assert table[11][2] == m4_binding.FLAG_MIRROR_HANDLE_WRITER
    assert table[12][2] == m4_binding.FLAG_MIRROR_NAME_WRITER
    assert table[3][2] == 0


def test_posted_rules_fail():
    fast = [entry(13, 'vehicle.set_rudder_offset')]
    with pytest.raises(ValueError, match='posted'):
        m4_binding.policy_table(ENTRIES + fast, dict(POLICY, posted=['vehicle.set_rudder_offset']), LOCAL)
    with pytest.raises(ValueError, match='posted'):   # a result: the module waits for it
        m4_binding.policy_table(ENTRIES, dict(POLICY, posted=['Parameter.set']), LOCAL)
    with pytest.raises(ValueError, match='posted'):   # an out argument
        m4_binding.policy_table(ENTRIES, dict(POLICY, posted=['Parameter.get']), LOCAL)
    with pytest.raises(ValueError, match='posted'):   # slow_methods
        m4_binding.policy_table(ENTRIES, dict(POLICY, posted=['ahrs.set_home']), LOCAL)


def test_stale_posted_or_mirrored_names_fail():
    for broken in (dict(POLICY, posted=['logger.no_such']), dict(POLICY, mirrored={'Parameter.no_such': 'get'})):
        with pytest.raises(ValueError, match='stale M4 policy'):
            m4_binding.policy_table(ENTRIES, broken, LOCAL)
    with pytest.raises(ValueError, match='mirror role'):
        m4_binding.policy_table(ENTRIES, dict(POLICY, mirrored={'Parameter.get': 'reader'}), LOCAL)


def test_catalogue_hash_covers_posted_and_mirrored():
    table = m4_binding.policy_table(ENTRIES, POLICY, LOCAL)
    first = m4_binding.catalogue_hash(ENTRIES, LOCAL, table)
    unposted = m4_binding.policy_table(ENTRIES, dict(POLICY, posted=[]), LOCAL)
    unmirrored = m4_binding.policy_table(ENTRIES, dict(POLICY, mirrored={}), LOCAL)
    assert m4_binding.catalogue_hash(ENTRIES, LOCAL, unposted) != first
    assert m4_binding.catalogue_hash(ENTRIES, LOCAL, unmirrored) != first
    assert m4_binding.catalogue_hash(ENTRIES, LOCAL, table) == first


def test_policy_functions_emission():
    source = m4_binding.policy_functions(m4_binding.policy_table(ENTRIES, POLICY, LOCAL))
    posted = source[source.index('bool binding_is_posted'):source.index('uint8_t binding_mirror_role')]
    assert 'case 9:' in posted and 'case 3:' not in posted
    role = source[source.index('uint8_t binding_mirror_role'):]
    assert 'case 10: return 1U;' in role and 'case 11: return 2U;' in role and 'case 12: return 3U;' in role


def test_catalogue_hash_depends_on_ids_and_hashes_only():
    first = m4_binding.catalogue_hash(ENTRIES, LOCAL)
    assert first == m4_binding.catalogue_hash(list(reversed(ENTRIES)), LOCAL)
    changed = [dict(e, hash='ffffffffffffffff') if e['id'] == 5 else e for e in ENTRIES]
    assert m4_binding.catalogue_hash(changed, LOCAL) != first


CONTRACT = dict(math=dict(types=[dict(name='Vector3f', methods=[
                    dict(id=6, name='length', result='float', receiver='in', arguments=[])])]),
                libm=[dict(id=5, name='sinf', arity=1)])


def test_local_sizes_follow_the_wire():
    assert m4_binding.local_sizes(CONTRACT) == {5: (4, 4), 6: (12, 4)}


def test_header_and_dispatch_emission():
    header = m4_binding.policy_header(m4_binding.policy_table(ENTRIES, POLICY, LOCAL), 0x1234ABCD)
    assert 'M4_BINDING_COUNT = 13U' in header and 'M4_LOCAL_CATALOGUE_HASH = 0x1234ABCDU' in header
    assert '{ M4Lane::Local, 0, 0 }, // 5' in header
    source = m4_binding.local_dispatch(CONTRACT['libm'], m4_binding.local_sizes(CONTRACT), ENTRIES)
    assert 'case 5:' in source and '::sinf(' in source and 'binding_math_dispatch' in source


def test_every_local_id_checks_its_exact_sizes_first():
    source = m4_binding.local_dispatch(CONTRACT['libm'], m4_binding.local_sizes(CONTRACT), ENTRIES)
    dispatch = source[source.index('int32_t binding_local_dispatch'):]
    for case, sizes in (('case 5: {', 'input_size != 4U || output_size != 4U'),
                        ('case 6: {', 'input_size != 12U || output_size != 4U')):
        body = dispatch[dispatch.index(case):]
        # the size check comes before the adapter call in the same case
        assert body.index(sizes) < min(i for i in (body.find('::sinf('), body.find('binding_math_dispatch')) if i >= 0)
    assert 'default:\n        return AP_MODULE_STATUS_INVALID;' in dispatch
    assert 'case 5: input_size = 4U; output_size = 4U; return true;' in source


def test_local_import_table_lists_hashes_of_local_ids():
    source = m4_binding.local_dispatch(CONTRACT['libm'], m4_binding.local_sizes(CONTRACT), ENTRIES)
    assert '{ 0x0000000000000005ULL, 5U },' in source and '{ 0x0000000000000006ULL, 6U },' in source
    assert '0x0000000000000003ULL' not in source       # gcs.send_text is not local
    assert 'M4_LOCAL_IMPORT_COUNT = 2U' in source


def test_full_generation_emits_the_m4_files():
    outputs = generate.generate(json.loads((ROOT / 'generator/host_api.json').read_text()))
    local_rows = outputs['BindingM4Policy_generated.h'].count('M4Lane::Local,')
    assert local_rows > 0
    assert outputs['BindingLocal_generated.cpp'].count('return AP_MODULE_STATUS_INVALID;') == local_rows + 1
