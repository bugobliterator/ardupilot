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
        # Parameter is owned (an M7 object): a blob cannot hold it
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
        # the high nibble is the quota class (AP_ScriptingM4_Quota.h): 1 Text, 2 Storage, 3 Led, 4 FileLog,
        # 6 Can
        quota_classes = {function: int(flag, 16) >> 4 for function, flag in zip(functions, flags) if int(flag, 16) >> 4}
        self.assertEqual(quota_classes, {
            'GCS_send_text': 1,
            'GCS_send_named_float': 1,
            'GCS_send_named_int': 1,
            'GCS_send_named_string': 1,
            'AP_Param_set_and_save_by_name': 2,
            'Parameter_set_and_save': 2,
            'AP_Mission_set_item': 2,
            'AP_Mission_clear': 2,
            'AP_SerialLED_send': 3,
            'AP_Logger_log_file_content': 4,
            'lua_m4_ScriptingCANBuffer_write_frame': 6,
        })
        self.assertIn('static_assert(6 < ScriptingM4::QUOTA_CLASSES', dispatch)
        # write_frame returns false over its quota; DroneCAN broadcast and request charge their
        # frames in their ports, so the dispatch charges them nothing more
        self.assertEqual(flags[functions.index('lua_m4_ScriptingCANBuffer_write_frame')], '0x62')
        for function in ('DroneCAN_Handle__broadcast', 'DroneCAN_Handle__request'):
            self.assertEqual(flags[functions.index(function)], '0x00', function)

    def test_changed_row_changes_the_hash(self):
        _, _, before = self.generate('hash_before', [])
        code, output, after = self.generate('hash_after', ["userdata Vector2f field extra float'skip_check read write"])
        self.assertEqual(code, 0, output)
        self.assertNotEqual(before, after)

    def read(self, name, suffix):
        with open(os.path.join(self.tmp, name, 'remote_' + suffix)) as f:
            return f.read()

    @staticmethod
    def body(source, function):
        # the definition, not a declaration before it
        start = re.search(r'^static \w+ ' + function + r'\(.*\) \{$', source, re.M).start()
        return source[start:source.index('\n}\n', start)]

    def test_local_blob_types(self):
        code, output, _ = self.generate('local', [])
        self.assertEqual(code, 0, output)
        tables = self.read('local', 'tables.cpp')
        dispatch = self.read('local', 'dispatch.cpp')
        local = self.read('local', 'local.cpp')
        types = re.findall(r'^    \{ "([^"]+)", (\d+), (\d+), (\d+), (\d+), (\d+) \},$', tables, re.M)
        rows = re.findall(r'^    \{ "([^"]+)", (\d+), 0x([0-9a-f]{2}), (\d+) \},$', tables, re.M)
        by_name = {t[0]: t for t in types}
        for name in ('Vector2f', 'Vector3f', 'Quaternion', 'Location'):
            self.assertEqual(int(by_name[name][1]), 5, name)            # BindingKind::LocalBlob
        for name in ('Vector2f', 'Vector3f', 'Quaternion'):
            self.assertEqual(int(by_name[name][4]), 0, name)
        first, count = int(by_name['Location'][3]), int(by_name['Location'][4])
        self.assertEqual(sorted(row[0] for row in rows[first:first + count]),
                         ['change_alt_frame', 'get_alt_m', 'get_vector_from_origin_NEU',
                          'get_vector_from_origin_NEU_cm', 'get_vector_from_origin_NEU_m'])
        # the M7 keeps the codecs (singleton methods carry these types) and the state rows only
        self.assertIn('static bool read_Vector3f(', dispatch)
        self.assertIn('read_Location', dispatch)
        self.assertNotIn('Vector3f_length(lua_State', dispatch)
        self.assertNotIn('Location_get_distance(lua_State', dispatch)
        self.assertNotIn('lua_new_Vector3f(lua_State', dispatch)
        self.assertIn('Location_change_alt_frame(lua_State', dispatch)
        # the M4 runs the rest, for every board (no guard), and trusts the M7's ranges on decode
        self.assertIn('static int Vector3f_length(lua_State', local)
        self.assertIn('static int Quaternion_earth_to_body(lua_State', local)
        self.assertNotIn('#if AP_AHRS_ENABLED', local)
        self.assertNotIn('900000000', self.body(local, 'read_Location'))
        self.assertIn('ud->loiter_ccw = (v != 0);', self.body(local, 'read_Location'))
        self.assertIn('ScriptingM4::remote_row_index(L, ', local)
        self.assertIn('remote_local_codecs[] = {', local)

    def test_new_method_of_local_type_stays_remote(self):
        # a method not in remote_local_methods may read vehicle state: the M7 serves it
        code, output, _ = self.generate('new_local_method', ['userdata Vector3f method length_squared float'])
        self.assertEqual(code, 0, output)
        self.assertIn('{ "length_squared", ', self.read('new_local_method', 'tables.cpp'))
        self.assertIn('Vector3f_length_squared(lua_State', self.read('new_local_method', 'dispatch.cpp'))

    def test_local_method_with_depends(self):
        self.assert_refused('local_depends', ['userdata Vector3f method length depends AP_AHRS_ENABLED'],
                            'remote: local method Vector3f:length has a depends')

    def test_local_type_with_enum(self):
        self.assert_refused('local_enum', ['userdata Vector3f enum ZERO ONE'],
                            'remote: local type Vector3f has enums')

    def outputs(self, name):
        work = os.path.join(self.tmp, name)
        with open(os.path.join(work, 'remote_tables.cpp')) as f:
            tables = f.read()
        with open(os.path.join(work, 'remote_dispatch.cpp')) as f:
            dispatch = f.read()
        local = os.path.join(work, 'remote_local.cpp')
        with open(local) as f:
            local_text = f.read()
        return tables, dispatch, local_text

    @staticmethod
    def type_ids(tables):
        names = re.findall(r'^    \{ "([^"]+)", (\d+), \d+, \d+, \d+, \d+ \},$', tables, re.M)
        return {name: (index + 1, int(kind)) for index, (name, kind) in enumerate(names)}

    @staticmethod
    def row_flags(tables, type_id, name):
        match = re.search(r'^    \{ "%s", (\d+), (0x[0-9a-f]{2}), %d \},$' % (re.escape(name), type_id), tables, re.M)
        return (int(match.group(1)), int(match.group(2), 16)) if match else None

    def test_owned_types(self):
        code, output, _ = self.generate('owned', [])
        self.assertEqual(code, 0, output)
        tables, dispatch, _ = self.outputs('owned')
        types = self.type_ids(tables)
        parameter, kind = types['Parameter']
        self.assertEqual(kind, 6)                            # ScriptingM4::BindingKind::Owned
        # the constructor is the ported creation function, a creation row
        _, flags = self.row_flags(tables, parameter, 'new')
        self.assertEqual(flags, 0x08)
        self.assertIn('    lua_new_Parameter,\n', dispatch)
        self.assertIn('remote_check_owned(L, arg, %d)' % parameter, dispatch)
        self.assertIn('remote_new_owned(L, %d, ScriptingM4::destroy_object<Parameter>)' % parameter, dispatch)
        self.assertIn('static_assert(sizeof(Parameter) <= ScriptingM4::ObjectPool::SLOT_SIZE', dispatch)
        # the methods are the in-process ones, on the object itself
        self.assertIn('Parameter * ud = check_Parameter(L, 1);', dispatch)
        # set_and_save is a Storage write: over its quota it returns false
        method, _ = self.row_flags(tables, parameter, 'set_and_save')
        start = dispatch.index('remote_method_flags[ScriptingM4::BINDING_METHOD_COUNT] = {')
        flags_table = re.findall(r'0x[0-9a-f]{2}', dispatch[start:dispatch.index('};', start)])
        self.assertEqual(flags_table[method - 64], '0x22')
        # init_by_info allocates M7 memory nothing frees: its row runs a refusing port, and the
        # generated body, which would make the allocation, is not in the dispatch
        self.assertIn('    lua_m4_Parameter_init_by_info,\n', dispatch)
        self.assertNotIn('static int Parameter_init_by_info(', dispatch)

    def test_serial_ports(self):
        code, output, _ = self.generate('serial', [])
        self.assertEqual(code, 0, output)
        tables, dispatch, _ = self.outputs('serial')
        types = self.type_ids(tables)
        serial, _ = types['serial']
        access, kind = types['AP_Scripting_SerialAccess']
        self.assertEqual(kind, 6)
        # a dot call (serial.find_serial(0)) sends a nil self and every argument
        for name in ('find_serial', 'find_simulated_device'):
            self.assertEqual(self.row_flags(tables, serial, name)[1], 0x11, name)
        for name in ('begin', 'writestring', 'readstring'):
            self.assertEqual(self.row_flags(tables, access, name)[1], 0x00, name)
        self.assertIsNone(self.row_flags(tables, access, 'new'))   # no constructor, as in-process
        self.assertIn('    lua_serial_find_serial,\n', dispatch)
        self.assertIn('AP_Scripting_SerialAccess * ud = check_AP_Scripting_SerialAccess(L, 1);', dispatch)

    def test_dot_access_names_a_singleton_manual(self):
        # every remote_dot_access entry is checked against the description: without the check a
        # renamed manual function would lose its dot-call fix silently
        description = self.description.replace('lua_serial_find_serial', 'lua_serial_find_serial_renamed')
        work = os.path.join(self.tmp, 'dot')
        os.makedirs(work)
        desc = os.path.join(work, 'bindings.desc')
        with open(desc, 'w') as f:
            f.write(description)
        result = subprocess.run([self.generator, '-o', os.path.join(work, 'gen'), '-r', os.path.join(work, 'remote'),
                                 '-i', desc], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('remote: dot access lua_serial_find_serial is no manual function of a singleton',
                      result.stdout + result.stderr)

    def test_replaced_method(self):
        code, output, _ = self.generate('replaced', [])
        self.assertEqual(code, 0, output)
        _, dispatch, _ = self.outputs('replaced')
        # analog:channel() runs the M7 port; the generated body is not emitted, so nothing can reach it
        self.assertIn('    lua_m4_analog_channel,\n', dispatch)
        self.assertNotIn('static int hal_analogin_channel(', dispatch)

    def test_replacement_names_a_method(self):
        # without the matched check a renamed method leaves its replacement in the list unused, silently
        description = self.description.replace(
            "singleton hal.analogin method channel AP_HAL::AnalogSource ANALOG_INPUT_NONE'literal",
            "singleton hal.analogin method channel_renamed AP_HAL::AnalogSource ANALOG_INPUT_NONE'literal")
        self.assertNotEqual(description, self.description)
        work = os.path.join(self.tmp, 'replacement')
        os.makedirs(work)
        desc = os.path.join(work, 'bindings.desc')
        with open(desc, 'w') as f:
            f.write(description)
        result = subprocess.run([self.generator, '-o', os.path.join(work, 'gen'), '-r', os.path.join(work, 'remote'),
                                 '-i', desc], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('remote: replacement lua_m4_analog_channel of hal.analogin:channel matches no method',
                      result.stdout + result.stderr)

    def test_i2c_wrapper(self):
        code, output, _ = self.generate('i2c', [])
        self.assertEqual(code, 0, output)
        tables, dispatch, local = self.outputs('i2c')
        types = self.type_ids(tables)
        device, kind = types['AP_HAL::I2CDevice']
        self.assertEqual(kind, 2)                            # a session handle
        method, flags = self.row_flags(tables, device, 'read_registers')
        self.assertEqual(flags, 0x20)
        # the M4 end of the row: its wrapper, by method id
        self.assertIn('    case %d:\n        return wrap_I2CDevice_read_registers;\n' % method, local)
        self.assertIn('    AP_HAL__I2CDevice_read_registers,\n', dispatch)
        self.assertEqual(self.row_flags(tables, types['i2c'][0], 'get_device')[1], 0x11)

    def test_wrapper_names_a_ported_function(self):
        description = self.description.replace('AP_HAL__I2CDevice_read_registers', 'AP_HAL__I2CDevice_read_regs')
        self.assertNotEqual(description, self.description)
        work = os.path.join(self.tmp, 'wrapper')
        os.makedirs(work)
        desc = os.path.join(work, 'bindings.desc')
        with open(desc, 'w') as f:
            f.write(description)
        result = subprocess.run([self.generator, '-o', os.path.join(work, 'gen'), '-r', os.path.join(work, 'remote'),
                                 '-i', desc], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('remote: M4 wrapper wrap_I2CDevice_read_registers of AP_HAL__I2CDevice_read_registers '
                      'matches no ported row', result.stdout + result.stderr)

    def test_i2c_methods_bound_the_semaphore(self):
        code, output, _ = self.generate('i2c_sem', [])
        self.assertEqual(code, 0, output)
        _, dispatch, _ = self.outputs('i2c_sem')
        for name in ('set_address', 'write_register'):
            text = self.body(dispatch, 'AP_HAL__I2CDevice_' + name)
            self.assertIn('get_semaphore()->take(I2C_SEMAPHORE_WAIT_MS)', text)
            self.assertNotIn('take_blocking', text)
            self.assertIn('if (!semaphore_taken) {', text)
        self.assertIn('lua_pushboolean(L, 0);', self.body(dispatch, 'AP_HAL__I2CDevice_write_register'))
        # set_retries runs the port that caps the count; the generated body is not in the dispatch
        self.assertIn('    lua_m4_I2CDevice_set_retries,\n', dispatch)
        self.assertNotIn('static int AP_HAL__I2CDevice_set_retries(', dispatch)
        # the in-process text keeps the unbounded take
        with open(os.path.join(self.tmp, 'i2c_sem', 'gen.cpp')) as f:
            self.assertIn('get_semaphore()->take_blocking();', self.body(f.read(), 'AP_HAL__I2CDevice_write_register'))

    def test_dronecan_handles(self):
        code, output, _ = self.generate('dronecan', [])
        self.assertEqual(code, 0, output)
        tables, dispatch, _ = self.outputs('dronecan')
        types = self.type_ids(tables)
        handle, kind = types['DroneCAN_Handle']
        self.assertEqual(kind, 6)
        # the release is the in-process __gc, not the destructor; the C++ names lose their ::
        self.assertIn('remote_new_owned(L, %d, DroneCAN_Handle__release)' % handle, dispatch)
        for name in ('DroneCAN_Handle__new_handle', 'DroneCAN_Handle__broadcast', 'DroneCAN_Handle__request',
                     'DroneCAN_Handle__check_message'):
            self.assertIn('    %s,\n' % name, dispatch)
        self.assertNotIn('DroneCAN_Handle::', dispatch.split('remote_functions[')[1].split('};')[0])
        self.assertIsNone(self.row_flags(tables, handle, '__gc'))
        self.assertEqual(self.row_flags(tables, types['CAN'][0], 'get_device2')[1], 0x11)
        self.assertEqual(types['ScriptingCANBuffer'][1], 2)
        # write_frame runs the port that clamps its timeout; the generated body is not in the dispatch
        self.assertIn('    lua_m4_ScriptingCANBuffer_write_frame,\n', dispatch)
        self.assertNotIn('static int ScriptingCANBuffer_write_frame(', dispatch)
        # subscribe runs the port that caps the session's subscribers
        self.assertIn('    lua_m4_DroneCAN_Handle_subscribe,\n', dispatch)
        self.assertNotIn('static int DroneCAN_Handle_subscribe(', dispatch)

    def test_crsf_menus(self):
        code, output, _ = self.generate('crsf', [])
        self.assertEqual(code, 0, output)
        tables, dispatch, _ = self.outputs('crsf')
        types = self.type_ids(tables)
        menu, kind = types['CRSFMenu']
        self.assertEqual(kind, 6)
        parameter, kind = types['CRSFParameter']
        self.assertEqual(kind, 6)
        self.assertEqual(self.row_flags(tables, menu, 'new')[1], 0x08)
        self.assertIsNone(self.row_flags(tables, parameter, 'new'))
        self.assertNotIn('lua_new_CRSFParameter', dispatch)
        for name in ('lua_CRSF_new_menu', 'lua_CRSF_add_parameter', 'lua_CRSF_add_menu', 'lua_CRSF_param_data',
                     'lua_CRSF_add_root_menu', 'lua_CRSF_get_menu_event', 'lua_CRSF_peek_menu_event',
                     'lua_CRSF_send_response'):
            self.assertIn('    %s,\n' % name, dispatch)

    def test_sockets(self):
        code, output, _ = self.generate('sockets', [])
        self.assertEqual(code, 0, output)
        tables, dispatch, local = self.outputs('sockets')
        types = self.type_ids(tables)
        socket, kind = types['SocketAPM']
        self.assertEqual(kind, 2)
        # send and sendto run the M7 ports that bound the length by the string; the generated
        # bodies, which pass the script's length on, are not emitted at all
        for name in ('send', 'sendto'):
            self.assertNotIn('static int SocketAPM_%s(' % name, dispatch)
            self.assertIn('    SocketAPM_%s_checked,\n' % name, dispatch)
        method, flags = self.row_flags(tables, socket, 'sendfile')
        self.assertEqual(flags, 0x20)
        self.assertIn('    case %d:\n        return wrap_SocketAPM_sendfile;\n' % method, local)
        self.assertIn('    SocketAPM_close,\n', dispatch)
        # the M7 never blocks: pollin/pollout clamp their wait, set_blocking keeps the socket
        # non-blocking; without the replacements the generated bodies pass the script's values on
        for name in ('pollin', 'pollout', 'set_blocking'):
            self.assertNotIn('static int SocketAPM_%s(' % name, dispatch)
            self.assertIn('    lua_m4_SocketAPM_%s,\n' % name, dispatch)
        # the M4 waits in place of a blocking call: generated and replaced methods get wrappers too
        wrappers = {'connect': 'wrap_SocketAPM_connect', 'send': 'wrap_SocketAPM_send',
                    'sendto': 'wrap_SocketAPM_sendto', 'set_blocking': 'wrap_SocketAPM_set_blocking'}
        for name, wrapper in wrappers.items():
            method, flags = self.row_flags(tables, socket, name)
            self.assertEqual(flags, 0x20, name)
            self.assertIn('    case %d:\n        return %s;\n' % (method, wrapper), local)
        self.assertIn('static int SocketAPM_connect(', dispatch)
        # Socket(): the M4 keeps whether the socket is a datagram one, which a send never cuts
        method, flags = self.row_flags(tables, types['global'][0], 'Socket')
        self.assertEqual(flags, 0x20)
        self.assertIn('    case %d:\n        return wrap_SocketAPM_new;\n' % method, local)
        self.assertEqual(self.row_flags(tables, socket, 'pollin')[1], 0)
        self.assertEqual(self.row_flags(tables, socket, 'recv')[1], 0)


if __name__ == '__main__':
    unittest.main()
