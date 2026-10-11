# encoding: utf-8
"""
CubeFramework transport and the Lua scripting Cortex-M4 image.

Enabled by the board (hwdef "env SCRIPTING_M4 1": CubeOrangePlus-M4Lua) or,
on SITL, by --enable-scripting-m4. On ChibiOS the M4 image is a separate
ChibiOS RT program built with make (libraries/AP_Scripting/m4/chibios/Makefile);
on SITL it is a host program the M7 SITL runs as a second process.

AP_FLAKE8_CLEAN
"""

from waflib.Configure import conf

CUBEFRAMEWORK = 'modules/CubeFramework'
CF_SOURCES = ['CubePilotFW/Channel.cpp', 'CubePilotFW/SpscRing.cpp', 'CubePilotFW/PeerHeader.cpp']
CF_CHIBIOS_SOURCES = CF_SOURCES + ['CubePilotFW/pal/HSEM.cpp']
CF_POSIX_SOURCES = CF_SOURCES + ['CubePilotFW/pal/Posix.cpp', 'CubePilotFW/pal/PosixHSEM.cpp',
                                 'CubePilotFW/pal/PosixChannel.cpp']
# CubeFramework keeps its own warning set, not ArduPilot's -Werror list
CF_CXXFLAGS = ['-Wno-error', '-Wno-cast-align', '-Wno-shadow']
M4_DIR = 'libraries/AP_Scripting/m4'
LUA_DIR = 'libraries/AP_Scripting/lua/src'
# the Lua core the M4 runtime links: no io, package, os, debug, coroutine, bit32 or utf8
# library (io and require come with the IO and Manual Bindings sub-phase)
M4_LUA_SOURCES = ['lapi.c', 'lauxlib.c', 'lbaselib.c', 'lcode.c', 'lctype.c', 'ldebug.c', 'ldo.c',
                  'ldump.c', 'lfunc.c', 'lgc.c', 'llex.c', 'lmathlib.c', 'lmem.c', 'lobject.c',
                  'lopcodes.c', 'lparser.c', 'lstate.c', 'lstring.c', 'lstrlib.c', 'ltable.c',
                  'ltablib.c', 'ltm.c', 'lundump.c', 'lvm.c', 'lzio.c']
M4_RUNTIME_SOURCES = ['m4_service.cpp', 'm4_lua.cpp', 'm4_rpc_client.cpp']


def _cf(paths):
    return ['%s/%s' % (CUBEFRAMEWORK, path) for path in paths]


def configure(cfg):
    env = cfg.env
    if cfg.srcnode.find_node(CUBEFRAMEWORK + '/CubePilotFW/Channel.h') is None:
        cfg.fatal('CubeFramework missing: git submodule update --init ' + CUBEFRAMEWORK)
    env.prepend_value('INCLUDES', [cfg.srcnode.find_dir(CUBEFRAMEWORK).abspath()])
    # CubeFramework headers need C++17 (std::atomic<T>::is_always_lock_free); the later -std wins
    env.CXXFLAGS += ['-std=gnu++17']
    if env.BOARD_CLASS == 'ChibiOS':
        # gnu++17 turns on sized deallocation, which AP_Common/c++.cpp warns about
        env.CXXFLAGS += ['-Wno-error=sized-deallocation']
    env.DEFINES += ['AP_CUBEFRAMEWORK_ENABLED=1', 'AP_SCRIPTING_M4_ENABLED=1']


@conf
def cubeframework(bld):
    '''M7 side of the CubeFramework transport, as a static library'''
    if bld.env.BOARD_CLASS == 'ChibiOS':
        return bld.stlib(name='cubeframework', target='cubeframework',
                         source=_cf(CF_CHIBIOS_SOURCES), features=['ch_ap_library'],
                         defines=['USE_CHIBIOS=1'], includes=[bld.bldnode.abspath()],
                         cxxflags=CF_CXXFLAGS,
                         use=[])  # ap_library's compile database dry run reads .use
    # HSEM.h picks the M7 semaphore mapping from __CORTEX_M
    return bld.stlib(name='cubeframework', target='cubeframework',
                     source=_cf(CF_POSIX_SOURCES), defines=['__CORTEX_M=7'],
                     cxxflags=CF_CXXFLAGS, use=[])


@conf
def scripting_m4_image(bld):
    '''the Cortex-M4 image (ChibiOS) or the M4 process (SITL), in bin/'''
    if bld.env.BOARD_CLASS == 'ChibiOS':
        _chibios_image(bld)
    else:
        _sitl_program(bld)
    bld.add_pre_fun(_add_image_target)


def _add_image_target(bld):
    # the image is no vehicle's dependency; build it with any targeted build (run after
    # the build commands have set bld.targets)
    if bld.targets:
        bld.targets += ',scripting_m4_image'


def _chibios_image(bld):
    src = bld.srcnode
    makefile = src.find_node(M4_DIR + '/chibios/Makefile')
    sources = (src.ant_glob(M4_DIR + '/**/*') +
               src.ant_glob('libraries/AP_Scripting/AP_ScriptingM4_*.h') +
               [src.find_node('libraries/AP_Scripting/AP_Scripting_BindingHistogram.h')] +
               src.ant_glob(LUA_DIR + '/*.h') +
               [src.find_node('%s/%s' % (LUA_DIR, path)) for path in M4_LUA_SOURCES] +
               [src.find_node('libraries/AP_HAL_ChibiOS/ap_setjmp.c')] +
               src.ant_glob(CUBEFRAMEWORK + '/CubePilotFW/**/*.h') +
               [src.find_node(path) for path in _cf(CF_CHIBIOS_SOURCES)])
    out = bld.bldnode.make_node('scripting_m4')
    bin_dir = bld.bldnode.make_node('bin')
    targets = [bin_dir.find_or_declare('scripting_m4.elf'), bin_dir.find_or_declare('scripting_m4.bin')]
    rule = ("'%s' -s -j%u -f '%s' CHIBIOS='%s' AP_ROOT='%s' BUILDDIR='%s' M4_LUA_SOURCES='%s' && "
            "mkdir -p '%s' && cp '%s/scripting_m4.elf' '%s/scripting_m4.bin' '%s'") % (
        bld.env.get_flat('MAKE'), bld.options.jobs, makefile.abspath(),
        src.find_dir('modules/ChibiOS').abspath(), src.abspath(), out.abspath(), ' '.join(M4_LUA_SOURCES),
        bin_dir.abspath(), out.abspath(), out.abspath(), bin_dir.abspath())
    bld(rule=rule, source=sources, target=targets, name='scripting_m4_image')


def _sitl_program(bld):
    # the shim directory comes first: the Lua core finds its AP headers there
    bld.program(target='bin/scripting_m4', name='scripting_m4_image',
                source=(['%s/%s' % (M4_DIR, path) for path in M4_RUNTIME_SOURCES] +
                        [M4_DIR + '/sitl/main.cpp', M4_DIR + '/sitl/lua_port.cpp'] +
                        ['%s/%s' % (LUA_DIR, path) for path in M4_LUA_SOURCES] +
                        _cf(CF_POSIX_SOURCES)),
                includes=[M4_DIR + '/shim', 'libraries/AP_Scripting', CUBEFRAMEWORK],
                defines=['__CORTEX_M=4'], cxxflags=CF_CXXFLAGS, cflags=['-Wno-error'],
                linkflags=['-pthread'], use=[])
