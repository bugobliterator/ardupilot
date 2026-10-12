#!/usr/bin/env python3
"""
gen-bindings -r fails closed: a crafted bindings.desc line that the M4 tables
cannot represent must stop generation with a message; a changed row must change
BINDING_HASH. Builds the generator with the host gcc, as the waf rule does.

Run: python3 -m pytest libraries/AP_Scripting/tests/test_gen_bindings_remote.py

AP_FLAKE8_CLEAN
"""

import os
import re
import shutil
import subprocess
import tempfile
import unittest

SCRIPTING = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
MAIN_C = os.path.join(SCRIPTING, 'generator', 'src', 'main.c')
DESCRIPTION = os.path.join(SCRIPTING, 'generator', 'description', 'bindings.desc')
CFLAGS = ['-std=c99', '-Wno-error=missing-field-initializers', '-Wall', '-Werror', '-Wextra']


class GenBindingsRemote(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix='gen_bindings_remote')
        cls.generator = os.path.join(cls.tmp, 'gen-bindings')
        subprocess.run(['gcc'] + CFLAGS + ['-o', cls.generator, MAIN_C], check=True)
        with open(DESCRIPTION) as f:
            cls.description = f.read()

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp)

    def generate(self, name, extra_lines):
        work = os.path.join(self.tmp, name)
        os.makedirs(work)
        desc = os.path.join(work, 'bindings.desc')
        with open(desc, 'w') as f:
            f.write(self.description + '\n' + '\n'.join(extra_lines) + '\n')
        result = subprocess.run([self.generator, '-o', os.path.join(work, 'gen'),
                                 '-r', os.path.join(work, 'remote'), '-i', desc],
                                capture_output=True, text=True)
        binding_hash = None
        contract = os.path.join(work, 'remote_contract.h')
        if result.returncode == 0:
            with open(contract) as f:
                binding_hash = re.search(r'BINDING_HASH = (0x[0-9a-f]+)U', f.read()).group(1)
        return result.returncode, result.stdout + result.stderr, binding_hash

    def assert_refused(self, name, extra_lines, message):
        code, output, _ = self.generate(name, extra_lines)
        self.assertNotEqual(code, 0, output)
        self.assertIn(message, output)

    def test_description_generates(self):
        code, output, binding_hash = self.generate('plain', [])
        self.assertEqual(code, 0, output)
        self.assertIsNotNone(binding_hash)

    def test_unlisted_userdata(self):
        self.assert_refused('userdata', ["userdata FooState field value float'skip_check read write"],
                            'remote: FooState is in no remote list')

    def test_unlisted_ap_object(self):
        self.assert_refused('ap_object', ['ap_object FooBackend method get_value float'],
                            'remote: FooBackend is in no remote list')

    def test_blob_field_the_wire_cannot_carry(self):
        # Parameter is deferred (it carries M7 state): a blob cannot hold it
        self.assert_refused('blob_field', ['userdata Vector2f field param Parameter read write'],
                            'remote: blob Vector2f.param has a type the wire cannot carry')

    def test_valid_mask_without_wire_member(self):
        self.assert_refused('valid_mask', ["userdata Vector2f field masked float'skip_check read write valid_mask x 1"],
                            'remote: Vector2f.masked has a valid_mask')

    def test_label_longer_than_an_srpc_name(self):
        self.assert_refused('label', ['singleton AP_AHRS method ' + 'a' * 60 + ' boolean'],
                            'is longer than an SRPC name')

    def test_dispatch_codecs(self):
        code, output, _ = self.generate('dispatch', [])
        self.assertEqual(code, 0, output)
        with open(os.path.join(self.tmp, 'dispatch', 'remote_dispatch.cpp')) as f:
            dispatch = f.read()

        def body(function):
            # the definition, not the declaration before it
            start = re.search(r'^static \w+ ' + function + r'\(.*\) \{$', dispatch, re.M).start()
            return dispatch[start:dispatch.index('\n}\n', start)]

        # Location: described fields with their setter ranges, then the hidden loiter_ccw as 0 or 1
        read_location = body('read_Location')
        self.assertIn('wide < int64_t(-900000000) || wide > int64_t(900000000)', read_location)
        self.assertTrue(read_location.rstrip().endswith('return true;'))
        last_read = read_location[read_location.rindex('if (!r.read(v))'):]
        self.assertIn('if (v > 1)', last_read)
        self.assertIn('ud->loiter_ccw = (v != 0);', last_read)
        self.assertIn('w.write(uint8_t(ud->loiter_ccw ? 1 : 0));', body('write_Location'))
        # EFI_State's hidden enum members are range-checked; hidden members have their wire width
        self.assertIn('v > uint8_t(Engine_State::FAULT)', body('read_EFI_State'))
        self.assertIn('static_assert(sizeof(ud->present_types) == sizeof(uint16_t)', dispatch)
        # a field with depends changes the wire length: writer and reader take the same guard
        guard = '#if AP_EXTENDED_ESC_TELEM_ENABLED\n'
        self.assertIn(guard, body('write_AP_ESC_Telem_Backend__TelemetryData'))
        self.assertIn(guard, body('read_AP_ESC_Telem_Backend__TelemetryData'))
        self.assertIn('< ScriptingM4::QUOTA_CLASSES', dispatch)
        # described fields match their members: width, and an array's length
        write_frame = body('write_AP_HAL__CANFrame')
        self.assertIn('static_assert(sizeof(ud->data) / sizeof(ud->data[0]) == size_t(int(ARRAY_SIZE(ud->data)))', write_frame)
        self.assertIn('static_assert(sizeof(ud->dlc) == sizeof(uint8_t)', write_frame)
        self.assertIn('static_assert(sizeof(ud->fuel_pressure_status) <= sizeof(int32_t)', body('write_EFI_State'))
        # every blob's wire form fits a blob value
        self.assertEqual(dispatch.count('<= ScriptingM4::MAX_BLOB'), 17)
        self.assertIn('+ wire_size_Cylinder_Status(nullptr)', dispatch)

    def test_quota_classes(self):
        code, output, _ = self.generate('quotas', [])
        self.assertEqual(code, 0, output)
        with open(os.path.join(self.tmp, 'quotas', 'remote_dispatch.cpp')) as f:
            dispatch = f.read()

        def table(name):
            # one entry per method; of a guarded entry, the #if branch
            start = dispatch.index(name + '[ScriptingM4::BINDING_METHOD_COUNT] = {')
            block = dispatch[dispatch.index('\n', start) + 1:dispatch.index('\n};', start)]
            entries = []
            in_else = False
            for line in block.split('\n'):
                line = line.strip()
                if line.startswith('#else'):
                    in_else = True
                elif line.startswith('#endif'):
                    in_else = False
                elif not line.startswith('#') and not in_else:
                    entries.append(line.rstrip(','))
            return entries

        functions = table('remote_functions')
        flags = table('remote_method_flags')
        self.assertEqual(len(functions), len(flags))
        # the high nibble is the quota class (AP_ScriptingM4_Quota.h): 1 Text, 2 Storage, 3 Led, 4 FileLog
        quota_classes = {function: int(flag, 16) >> 4 for function, flag in zip(functions, flags) if int(flag, 16) >> 4}
        self.assertEqual(quota_classes, {
            'GCS_send_text': 1,
            'GCS_send_named_float': 1,
            'GCS_send_named_int': 1,
            'GCS_send_named_string': 1,
            'AP_Param_set_and_save_by_name': 2,
            'AP_Mission_set_item': 2,
            'AP_Mission_clear': 2,
            'AP_SerialLED_send': 3,
            'AP_Logger_log_file_content': 4,
        })
        self.assertIn('static_assert(4 < ScriptingM4::QUOTA_CLASSES', dispatch)

    def test_changed_row_changes_the_hash(self):
        _, _, before = self.generate('hash_before', [])
        code, output, after = self.generate('hash_after', ["userdata Vector2f field extra float'skip_check read write"])
        self.assertEqual(code, 0, output)
        self.assertNotEqual(before, after)


if __name__ == '__main__':
    unittest.main()
