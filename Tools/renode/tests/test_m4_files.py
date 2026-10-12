# AP_FLAKE8_CLEAN
"""Renode test of the M4 file proxy on the CubeOrangePlus: scripts on the M4 load
modules and read and write the SD card through the M7 (Lua io, require, dirlist,
remove), with the reach of in-process scripting. Functional only: Renode is not cycle-accurate.

Needs RENODE and a CubeOrangePlus-M4Lua build:
  ./waf configure --board CubeOrangePlus-M4Lua && ./waf copter
"""

import sys

from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import fat_image  # noqa: E402
from test_m4_containment import start_m4lua  # noqa: E402,F401
from test_m4_scripts import SCRIPTS, SCRIPT_TIMEOUT, host_thread_stack, start_with_scripts, wait_all  # noqa: E402

MODULES = [(SCRIPTS / 'tests' / 'modules' / 'test' / name, 'APM/scripts/modules/test')
           for name in ('nested.lua', 'top.lua')]

# FatFs lists the SD root for "" (SITL's POSIX backend cannot tell: its opendir("") fails)
ROOT_LISTING_SCRIPT = '''
local found = false
for _, name in ipairs(dirlist("") or {}) do
  if name == "APM" then
    found = true
  end
end
gcs:send_text(6, found and "M4 root listing ok" or "M4 root listing failed")
'''


def test_m4_module_loading(start_m4lua, tmp_path):  # noqa: F811
    root_listing = tmp_path / 'm4_root_listing.lua'
    root_listing.write_text(ROOT_LISTING_SCRIPT)
    run = start_with_scripts(start_m4lua, ['tests/scripting_test.lua', 'tests/m4_files.lua'],
                             {'SCR_VM_I_COUNT': 1000000},
                             files=MODULES + [(root_listing, 'APM/scripts')])
    run.wait_text(r'^Scripting: started on M4 \(3 scripts, heap \d+KB\)$', timeout=SCRIPT_TIMEOUT)
    wait_all(run, [r'^Internal tests passed$', r'^M4 files (passed|failed \d+)$',
                   r'^M4 root listing (ok|failed)$'], SCRIPT_TIMEOUT, failure=r'^(Lua: |M4 files FAIL)')
    failed = [text for text in run.texts
              if text.startswith('M4 files FAIL') or text.startswith('M4 files failed') or text.startswith('Lua: ')
              or text == 'M4 root listing failed']
    assert not failed, failed
    assert 'M4 files passed' in run.texts, run.texts[-15:]
    assert 'M4 root listing ok' in run.texts, run.texts[-15:]
    # the file calls ran on FatFs under the RPC dispatch: the host thread keeps 2 KB of its 8 KB stack
    free, size = host_thread_stack(run, tmp_path / 'threads.txt')
    print('ScriptingM4 host thread stack: %u of %u bytes free' % (free, size), flush=True)
    assert free >= 2048, 'ScriptingM4 host thread stack: %u of %u bytes free' % (free, size)
    run.wait_alive(10)
    run.stop()
    # a ".." above the root lands at the SD root on FatFs: the refused writes left nothing there,
    # and the script removed its relative file (also at the root) and its own files
    outside = fat_image.extract_files(run.sdcard_image, '/', 'M4_ESCAPE*', tmp_path / 'root')
    assert not outside, 'refused writes reached the SD root: %s' % outside
    relative = [leftover for pattern in ('M4_FILES*.TXT', 'MISS.TXT')
                for leftover in fat_image.extract_files(run.sdcard_image, '/', pattern, tmp_path / 'root')]
    assert not relative, 'm4_files.lua left %s at the SD root' % relative
    left = fat_image.extract_files(run.sdcard_image, '/APM/SCRIPTS', 'M4_FILES*.TXT', tmp_path / 'scripts')
    assert not left, 'm4_files.lua left %s' % left
