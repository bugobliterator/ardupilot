# AP_FLAKE8_CLEAN
"""Renode tests of the M7 objects M4 scripts own or make on the CubeOrangePlus: parameters and
serial ports in the M7's object pool, released when the M4 collects them or when its session
ends; the M7 host thread's stack under them. Functional only: Renode is not cycle-accurate.

Needs RENODE and a CubeOrangePlus-M4Lua build:
  ./waf configure --board CubeOrangePlus-M4Lua && ./waf copter
"""

import os
import sys
import time

from pathlib import Path

from pymavlink import DFReader

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from test_m4_containment import BUILD, STEP_TIMEOUT, start_m4lua  # noqa: E402,F401
from test_m4_scripts import (SCRIPT_TIMEOUT, host_thread_stack, m4_symbols, start_with_scripts,  # noqa: E402
                             stop_and_read_log, symbol_range, wait_all)

# a simulated device on the first scripting serial port (serial_loopback.lua, m4_objects.lua)
SERIAL_DEVICE = {'SCR_VM_I_COUNT': 1000000, 'SCR_SDEV_EN': 1, 'SCR_SDEV1_PROTO': 28}


# I2C devices, PWM sources, sockets and CRSF menus on the board's own drivers. Renode models no I2C
# device at 0x50: the read finds nothing and returns nil, as in-process on such a bus. The
# script keeps its objects in the update closure, so a restart must free them for the next
# session to get the same again (4 devices, 4 sources, 10 menus). With networking off (NET_ENABLE 0)
# Socket() gives nil, and the M7 keeps running with no protocol error
DEVICES_SCRIPT = '''
local failures = 0
local function check(ok, name)
  if not ok then
    failures = failures + 1
    gcs:send_text(3, "M4 devices FAIL " .. name)
  end
end
local held = {devices = {}, sources = {}, menus = {}}
for i = 1, 4 do
  held.devices[i] = i2c:get_device(0, 0x50)
  check(held.devices[i] ~= nil, "i2c device " .. i)
end
local ok, err = pcall(i2c.get_device, i2c, 0, 0x50)
check(not ok and string.find(tostring(err), "no i2c devices available", 1, true) ~= nil, "fifth i2c device")
local regs = held.devices[1] and held.devices[1]:read_registers(0, 2)
check(regs == nil or (type(regs) == "table" and #regs == 2), "read_registers")
for i = 1, 4 do
  held.sources[i] = PWMSource()
  check(held.sources[i] ~= nil, "PWMSource " .. i)
end
ok, err = pcall(PWMSource)
check(not ok and string.find(tostring(err), "no PWMSources available", 1, true) ~= nil, "fifth PWMSource")
-- NET_ENABLE 0: lwIP was never initialised, and a socket call would lock its null core mutex
check(Socket(0) == nil, "stream socket without networking")
check(Socket(1) == nil, "datagram socket without networking")
-- menus are built by the M7: names are cut to 15 characters, 20 parameters per menu, 10 menus
local menu = crsf:add_menu("M4 menu")
check(menu ~= nil and menu:name() == "M4 menu", "crsf menu")
held.menus[1] = menu
if menu then
  for _ = 1, 30 do
    menu:add_parameter("z")
  end
  check(menu:num_params() == 20, "parameters in a menu " .. menu:num_params())
end
local cut = crsf:add_menu("0123456789ABCDEFGHIJ")
check(cut ~= nil and cut:name() == "0123456789ABCDE", "menu name cut")
held.menus[2] = cut
local menus = 2
for _ = 1, 20 do
  local more = crsf:add_menu("M4 more")
  if more == nil then
    break
  end
  held.menus[#held.menus + 1] = more
  menus = menus + 1
end
check(menus == 10, "menus in the session " .. menus)
if failures == 0 then
  gcs:send_text(6, "M4 devices passed")
else
  gcs:send_text(3, "M4 devices failed " .. failures)
end
local function update()
  if held == nil then
    return
  end
  return update, 1000
end
return update, 1000
'''
DEVICES_SOCKET_CHECKS = '''check(Socket(0) == nil, "stream socket without networking")
check(Socket(1) == nil, "datagram socket without networking")
'''
# M4LuaNet has no Socket global until Lua sockets run on the M4
NET_DEVICES_SOCKET_CHECKS = '''check(Socket == nil, "no Lua sockets on M4LuaNet yet")
'''
NETWORKING_BOARD = os.environ.get('M4LUA_BOARD') == 'CubeOrangePlus-M4LuaNet'


def assert_host_stack(run, tmp_path):
    '''The M7 host thread ran the M4's object calls and keeps 2 KB of its stack.'''
    free, size = host_thread_stack(run, tmp_path / 'threads.txt')
    print('ScriptingM4 host thread stack: %u of %u bytes free' % (free, size), flush=True)
    assert free >= 2048, 'ScriptingM4 host thread stack: %u of %u bytes free' % (free, size)


def assert_no_protocol_errors(run, tmp_path):
    '''Stops Renode: no M4ST row of the run carries a protocol error.'''
    run.wait_alive(10)   # M4ST rows are written once a second
    log = stop_and_read_log(run, tmp_path / 'logs')
    reader = DFReader.DFReader_binary(str(log), zero_time_base=True)
    rows = 0
    errors = 0
    while True:
        row = reader.recv_match(type='M4ST')
        if row is None:
            break
        rows += 1
        errors += row.PErr
    assert rows > 0, 'no M4ST rows in the log'
    assert errors == 0, '%u M4 protocol errors across the restart' % errors


def test_m4_objects(start_m4lua, tmp_path):  # noqa: F811
    run = start_with_scripts(start_m4lua, ['tests/serial_loopback.lua', 'tests/m4_objects.lua'], SERIAL_DEVICE)
    run.wait_text(r'^Scripting: started on M4 \(2 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    wait_all(run, [r'^driver -> device good$', r'^device -> driver good$', r'^M4 objects (passed|failed \d+)$'],
             SCRIPT_TIMEOUT, failure=r'^(Lua: |M4 objects FAIL)')
    assert 'M4 objects passed' in run.texts, run.texts[-15:]
    assert_host_stack(run, tmp_path)
    run.wait_alive(10)
    run.stop()


def test_m4_serial_ports(start_m4lua, tmp_path):  # noqa: F811
    '''readstring answers in chunks of up to 768 bytes on the host thread stack.'''
    run = start_with_scripts(start_m4lua, ['tests/m4_serial_ports.lua'], SERIAL_DEVICE)
    run.wait_text(r'^Scripting: started on M4 \(1 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    wait_all(run, [r'^M4 serial ports (passed|failed \d+)$'], SCRIPT_TIMEOUT, failure=r'^(Lua: |M4 serial FAILED)')
    assert 'M4 serial ports passed' in run.texts, run.texts[-15:]
    assert_host_stack(run, tmp_path)
    run.wait_alive(10)
    run.stop()


LOG_ROWS = {'LOG_DISARMED': 1, 'LOG_DARM_RATEMAX': 0, 'LOG_FILE_RATEMAX': 0}


def test_m4_objects_end_with_the_session(start_m4lua, tmp_path):  # noqa: F811
    '''A restarted M4 starts clean: the M7 freed the objects, aux auth ids and port buffers the first
    session held (the new one gets 64 objects, id 0 and empty buffers). The release queue is not
    covered here: nothing is queued at this restart (see the dropped-objects test).'''
    run = start_with_scripts(start_m4lua, ['tests/m4_objects_hold.lua'], dict(SERIAL_DEVICE, **LOG_ROWS))
    run.wait_text(r'^Scripting: started on M4 \(1 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    held = r'^M4 holding \d+ objects, auth \S+$'
    first = run.wait_text(held, timeout=SCRIPT_TIMEOUT)
    assert first == 'M4 holding 64 objects, auth 0', first
    run.wait_text(r'^M4 hold: ', timeout=SCRIPT_TIMEOUT)
    run.set('SCR_M4_INJECT', 1)   # a fault in the M4 service: the M7 restarts it
    run.wait_text(r'^Scripting: M4 restarted \(1/\d+\)$', timeout=SCRIPT_TIMEOUT)
    # without the session end the pool still holds the first session's objects and the new one gets fewer
    second = run.wait_text(held, timeout=SCRIPT_TIMEOUT)
    assert second == 'M4 holding 64 objects, auth 0', second
    state = run.wait_text(r'^M4 hold: ', timeout=SCRIPT_TIMEOUT)
    assert state.endswith('left 0, port ok'), state
    assert_host_stack(run, tmp_path)
    assert_no_protocol_errors(run, tmp_path)


def test_m4_dropped_objects_do_not_cross_a_restart(start_m4lua, tmp_path):  # noqa: F811
    '''A restart while releases are queued on the M4: the queue (pending_releases, in .lua_bss) is
    non-empty when the M4 faults, and the new session starts with an empty one, so it sends no
    stale release (no protocol error) and holds its 64 objects again.'''
    symbols = m4_symbols(os.environ.get('M4LUA_M4_ELF', BUILD / 'scripting_m4.elf'))
    # ReleaseQueue: 64 entries of 8 bytes, then the count byte
    queue_count = symbol_range(symbols, 'ScriptingM4::pending_releases')[0] + 64 * 8
    run = start_with_scripts(start_m4lua, ['tests/m4_objects_drop.lua'],
                             dict({'SCR_VM_I_COUNT': 1000000}, **LOG_ROWS))
    run.wait_text(r'^Scripting: started on M4 \(1 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    held = r'^M4 holding \d+ objects$'
    first = run.wait_text(held, timeout=SCRIPT_TIMEOUT)
    assert first == 'M4 holding 64 objects', first
    run.wait_text(r'^M4 dropping$', timeout=SCRIPT_TIMEOUT)
    deadline = time.monotonic() + STEP_TIMEOUT
    while run.read_word(queue_count) & 0xFF == 0:   # the collector queued a release; none is sent
        run.check()
        assert time.monotonic() < deadline, 'no release was queued on the M4'
        run.mav.recv_match(blocking=True, timeout=1)
    queued = run.read_word(queue_count) & 0xFF
    assert not any(text.startswith('M4 spin done') for text in run.texts), 'the spin ended: nothing was pending'
    run.set('SCR_M4_INJECT', 1)   # a fault in the M4 service: the M7 restarts it
    run.wait_text(r'^Scripting: M4 restarted \(1/\d+\)$', timeout=SCRIPT_TIMEOUT)
    assert not any(text.startswith('M4 spin done') for text in run.texts), 'the spin ended before the restart'
    print('ScriptingM4: %u releases queued at the restart' % queued, flush=True)
    second = run.wait_text(held, timeout=SCRIPT_TIMEOUT)
    assert second == 'M4 holding 64 objects', second
    assert_host_stack(run, tmp_path)
    assert_no_protocol_errors(run, tmp_path)


def test_m4_devices(start_m4lua, tmp_path):  # noqa: F811
    '''I2C, PWM sources and CRSF menus on the real drivers, again after a restart: the M7 freed
    the first session's devices, sources and menus, or the second would find the tables full.
    Socket() with networking off is refused before lwIP is called.'''
    script = tmp_path / 'm4_devices_renode.lua'
    assert DEVICES_SCRIPT.count(DEVICES_SOCKET_CHECKS) == 1
    if NETWORKING_BOARD:
        script.write_text(DEVICES_SCRIPT.replace(DEVICES_SOCKET_CHECKS, NET_DEVICES_SOCKET_CHECKS))
    else:
        script.write_text(DEVICES_SCRIPT)
    run = start_with_scripts(start_m4lua, [], dict({'SCR_VM_I_COUNT': 1000000, 'NET_ENABLE': 0}, **LOG_ROWS),
                             files=[(script, 'APM/scripts')])
    run.wait_text(r'^Scripting: started on M4 \(1 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    outcome = r'^M4 devices (passed|failed \d+)$'
    wait_all(run, [outcome], SCRIPT_TIMEOUT, failure=r'^(Lua: |M4 devices FAIL)')
    assert 'M4 devices passed' in run.texts, run.texts[-15:]
    run.set('SCR_M4_INJECT', 1)   # a fault in the M4 service: the M7 restarts it
    run.wait_text(r'^Scripting: M4 restarted \(1/\d+\)$', timeout=SCRIPT_TIMEOUT)
    wait_all(run, [outcome], SCRIPT_TIMEOUT, failure=r'^(Lua: |M4 devices FAIL)')
    assert run.texts.count('M4 devices passed') == 2, run.texts[-15:]
    assert not any(text.startswith('Lua: ') for text in run.texts), run.texts[-15:]
    assert_host_stack(run, tmp_path)
    assert_no_protocol_errors(run, tmp_path)
