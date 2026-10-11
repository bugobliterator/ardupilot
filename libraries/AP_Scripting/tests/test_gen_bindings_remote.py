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

    def test_changed_row_changes_the_hash(self):
        _, _, before = self.generate('hash_before', [])
        code, output, after = self.generate('hash_after', ["userdata Vector2f field extra float'skip_check read write"])
        self.assertEqual(code, 0, output)
        self.assertNotEqual(before, after)


if __name__ == '__main__':
    unittest.main()
