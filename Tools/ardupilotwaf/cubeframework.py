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
# the Lua core the M4 runtime links: no os, debug, coroutine, bit32 or utf8 library
NET_M4_DIR = 'libraries/AP_Networking/m4'
LWIP_DIR = 'modules/lwip/src'
# lwIP on the M4, as the M4 Makefile builds it: no slipif.c, bridgeif*, lowpan6* or zepif.c
LWIP_PATTERNS = ['core/*.c', 'core/ipv4/*.c', 'api/*.c', 'netif/ppp/*.c', 'netif/ethernet.c']
# the M4's arch/sys_arch.h must come before the M7 port's (lwip_hal/include)
NET_M4_INCLUDES = [NET_M4_DIR, LWIP_DIR + '/include', 'libraries/AP_Networking/lwip_hal/include',
                   'libraries/AP_Networking']
M4_LUA_SOURCES = ['lapi.c', 'lauxlib.c', 'lbaselib.c', 'lcode.c', 'lctype.c', 'ldebug.c', 'ldo.c',
                  'ldump.c', 'lfunc.c', 'lgc.c', 'liolib.c', 'llex.c', 'lmathlib.c', 'lmem.c', 'loadlib.c',
                  'lobject.c', 'lopcodes.c', 'lparser.c', 'lstate.c', 'lstring.c', 'lstrlib.c',
                  'ltable.c', 'ltablib.c', 'ltm.c', 'lundump.c', 'lvm.c', 'lzio.c']


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
    if env.NETWORKING_M4:
        env.DEFINES += ['AP_NETWORKING_M4_ENABLED=1']


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
        # cubeframework first: posted from inside a vehicle library's use processing,
        # this stlib runs the compile database dry run there, which posts the other
        # targets; a test program's use of 'ap' then clears dronecan_libs'
        # tmp_use_objects and the vehicle library is archived without DroneCAN
        bld.targets = 'cubeframework,' + bld.targets + ',scripting_m4_image'


def _chibios_image(bld):
    src = bld.srcnode
    makefile = src.find_node(M4_DIR + '/chibios/Makefile')
    generator_inputs = [src.find_node('libraries/AP_Scripting/generator/description/bindings.desc'),
                        src.find_node('libraries/AP_Scripting/generator/src/main.c'),
                        src.find_node('libraries/AP_Scripting/generator/gen/generated_cpp_deps.h')]
    sources = (src.ant_glob(M4_DIR + '/**/*') +
               src.ant_glob('libraries/AP_Scripting/AP_ScriptingM4_*.h') +
               [src.find_node('libraries/AP_Scripting/AP_Scripting_BindingHistogram.h')] +
               src.ant_glob(LUA_DIR + '/*.h') +
               src.ant_glob('libraries/AP_Math/*') +
               [src.find_node('libraries/AP_Common/Location.h'),
                src.find_node('libraries/AP_Common/Location.cpp'),
                src.find_node('libraries/AP_Common/AP_Common.h')] +
               [src.find_node('%s/%s' % (LUA_DIR, path)) for path in M4_LUA_SOURCES] +
               [src.find_node('libraries/AP_HAL_ChibiOS/ap_setjmp.c')] +
               src.ant_glob(CUBEFRAMEWORK + '/CubePilotFW/**/*.h') +
               [src.find_node(path) for path in _cf(CF_CHIBIOS_SOURCES)] +
               generator_inputs)
    if bld.env.NETWORKING_M4:
        sources += (src.ant_glob(NET_M4_DIR + '/**/*') +
                    src.ant_glob('libraries/AP_Networking/AP_Networking_M4_*') +
                    src.ant_glob('libraries/AP_Networking/lwip_hal/include/**/*') +
                    src.ant_glob(LWIP_DIR + '/**/*'))
    out = bld.bldnode.make_node('scripting_m4')
    generated_root = bld.bldnode.make_node('libraries')
    bin_dir = bld.bldnode.make_node('bin')
    targets = [bin_dir.find_or_declare('scripting_m4.elf'), bin_dir.find_or_declare('scripting_m4.bin')]
    rule = ("'%s' -s -j%u -f '%s' CHIBIOS='%s' AP_ROOT='%s' BUILDDIR='%s' M4_LUA_SOURCES='%s' "
            "M4_GENERATED_ROOT='%s' NETWORKING_M4='%s' && "
            "mkdir -p '%s' && cp '%s/scripting_m4.elf' '%s/scripting_m4.bin' '%s'") % (
        bld.env.get_flat('MAKE'), bld.options.jobs, makefile.abspath(),
        src.find_dir('modules/ChibiOS').abspath(), src.abspath(), out.abspath(), ' '.join(M4_LUA_SOURCES),
        generated_root.abspath(), '1' if bld.env.NETWORKING_M4 else '0',
        bin_dir.abspath(), out.abspath(), out.abspath(), bin_dir.abspath())
    bld(rule=rule, source=sources, target=targets, name='scripting_m4_image')


def _sitl_program(bld):
    # m4/*.cpp is the portable runtime (generated tables come in through wrappers there);
    # the shim directory comes first: the Lua core finds its AP headers there. 'libraries'
    # also reaches the generated headers in the build directory (AP_Scripting/lua_remote_*)
    runtime = [node.path_from(bld.srcnode) for node in bld.srcnode.ant_glob(M4_DIR + '/*.cpp')]
    # AP_Math and AP_Common/Location for the M4's local pure-data types: the real sources
    # through wrappers, with the SITL vehicle's own flags, as the SITL M7 compiles them
    local = [node.path_from(bld.srcnode) for node in bld.srcnode.ant_glob(M4_DIR + '/local/*.cpp')]
    network = []
    network_includes = []
    network_defines = []
    network_use = []
    if bld.env.NETWORKING_M4:
        network = [NET_M4_DIR + '/net_m4.cpp', NET_M4_DIR + '/net_sys_arch.cpp', NET_M4_DIR + '/net_sys_arch_posix.cpp',
                   'libraries/AP_Networking/AP_Networking_M4_Rings.cpp']
        network_includes = NET_M4_INCLUDES
        network_defines = ['AP_NETWORKING_M4_ENABLED=1', 'NET_M4_POSIX=1']
        # lwIP in its own objects: its warnings are not the Lua core's
        bld.objects(target='scripting_m4_lwip',
                    source=[node.path_from(bld.srcnode) for pattern in LWIP_PATTERNS
                            for node in bld.srcnode.ant_glob('%s/%s' % (LWIP_DIR, pattern))],
                    includes=network_includes, defines=network_defines,
                    cflags=['-Wno-error', '-Wno-implicit-fallthrough', '-Wno-parentheses-equality'])
        network_use = ['scripting_m4_lwip']
    bld.program(target='bin/scripting_m4', name='scripting_m4_image',
                source=(runtime + local +
                        [M4_DIR + '/sitl/main.cpp', M4_DIR + '/sitl/lua_port.cpp'] +
                        ['%s/%s' % (LUA_DIR, path) for path in M4_LUA_SOURCES] +
                        _cf(CF_POSIX_SOURCES) + network),
                includes=network_includes + [M4_DIR + '/shim', M4_DIR, 'libraries/AP_Scripting', 'libraries',
                                             CUBEFRAMEWORK],
                defines=['__CORTEX_M=4', 'AP_LUA_MAX_NUMBER_TEXT=100'] + network_defines, cxxflags=CF_CXXFLAGS,
                cflags=['-Wno-error'], linkflags=['-pthread'], use=network_use)
