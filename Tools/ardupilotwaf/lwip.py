#!/usr/bin/env python
# encoding: utf-8


from waflib import Build, Context, Task
from waflib.Configure import conf
from waflib.TaskGen import feature, before_method, after_method
from waflib.Errors import WafError

def configure(cfg):
    env = cfg.env
    cfg.load('cmake')

    env.LWIP_PREFIX_REL = 'lwip'

    bldnode = cfg.bldnode.make_node(cfg.variant)
    prefix_node = bldnode.make_node(env.LWIP_PREFIX_REL)
    env.INCLUDES += [
        cfg.srcnode.find_dir('modules/lwip/src/include').abspath(),
        cfg.srcnode.find_dir('libraries/AP_HAL_SITL/lwip').abspath(),
        cfg.srcnode.find_dir('modules/lwip-contrib/ports/unix/port/include').abspath()
    ]

    # env.INCLUDES_LWIP = [prefix_node.make_node('include').abspath()]
    env.STLIBPATH_LWIP = [prefix_node.make_node('.').abspath()]
    env.STLIB_LWIP = ['lwip', 'lwipcore', 'lwipcontribportunix', 'lwipallapps']

    env.append_value('GIT_SUBMODULES', 'lwip')
    env.HAS_LWIP = True

@conf
def liblwip(bld):
    prefix_node = bld.bldnode.make_node(bld.env.LWIP_PREFIX_REL)

    lwip = bld.cmake(
        name='lwip',
        cmake_src=bld.env.LWIP_CONFIG,
        cmake_bld=['lwip'],
        cmake_vars=dict(
            CMAKE_BUILD_TYPE='Release',
            CMAKE_INSTALL_PREFIX=prefix_node.abspath()
        ),
    )

    prefix_node = bld.bldnode.make_node(bld.env.LWIP_PREFIX_REL)
    output_paths = (
        'liblwip.a',
        'liblwipcore.a',
        'liblwipcontribportunix.a',
        'liblwipallapps.a'
    )
    outputs = [prefix_node.make_node(path) for path in output_paths]
    lwip.build('lwip', 
                group='dynamic_sources',
                target=outputs)

@feature('lwip')
@before_method('process_use')
def append_lwip_use(self):
    self.use = self.to_list(getattr(self, 'use', []))
    if 'LWIP' not in self.use:
        self.use.append('LWIP')

# @feature('lwip')
# @before_method('process_source')
# def wait_for_lwip_install(self):
#     bld_task = self.create_cmake_build_task(
#         'lwip',
#         'lib/liblwip.a'
#     )
