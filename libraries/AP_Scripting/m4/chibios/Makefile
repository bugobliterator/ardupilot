# Lua scripting Cortex-M4 image for the STM32H757: ChibiOS RT only (no HAL), the
# Lua core and the M4 Lua runtime. Built by waf (Tools/ardupilotwaf/cubeframework.py),
# which passes M4_LUA_SOURCES and M4_GENERATED_ROOT; by hand:
#   make -f libraries/AP_Scripting/m4/chibios/Makefile CHIBIOS=$PWD/modules/ChibiOS \
#        AP_ROOT=$PWD BUILDDIR=/tmp/scripting_m4 M4_LUA_SOURCES="lapi.c lauxlib.c ..." \
#        M4_GENERATED_ROOT=$PWD/build/CubeOrangePlus-M4Lua/libraries

ifeq ($(M4_LUA_SOURCES),)
$(error M4_LUA_SOURCES is not set: waf passes the Lua core file list (Tools/ardupilotwaf/cubeframework.py))
endif
ifeq ($(M4_GENERATED_ROOT),)
$(error M4_GENERATED_ROOT is not set: waf passes the build's libraries directory (gen-bindings output))
endif

USE_OPT = -Os -ggdb -fomit-frame-pointer -falign-functions=16
# NET_INCOPT: the M4's arch/sys_arch.h ahead of the M7 port's, which the sorted INCDIR puts first
USE_COPT = $(NET_INCOPT)
USE_CPPOPT = -std=gnu++17 -fno-rtti -fno-exceptions -fno-threadsafe-statics -fno-use-cxa-atexit $(NET_INCOPT)
USE_LINK_GC = yes
# scripting_m4.ld asserts its memory against AP_ScriptingM4_Placement.h, read through
# the preprocessor (the header's values may depend on the networking switch);
# a value the header does not define as a number, or a preprocessor failure, stops make
NETWORKING_M4 ?= 0
PLACEMENT_DEFS = -DAP_NETWORKING_M4_ENABLED=$(NETWORKING_M4)
PLACEMENT_H = $(AP_ROOT)/libraries/AP_Scripting/AP_ScriptingM4_Placement.h
placement_expr = $(shell value=$$(printf '\#include "%s"\n%s\n' '$(PLACEMENT_H)' '$(1)' | \
                   $(TRGT)gcc -E -P $(PLACEMENT_DEFS) -x c -) || exit 1; \
                   value=$$(printf '%s\n' "$$value" | tail -1 | tr -d U); \
                   printf '%s\n' "$$value" | grep -Eq '^[0-9a-fA-FxX ()*+/-]+$$' && echo $$(($$value)))
placement_value = $(or $(call placement_expr,$(1)),$(error $(1): no numeric value in $(PLACEMENT_H) with $(PLACEMENT_DEFS)))
USE_LDOPT = --defsym=placement_flash_base=$(call placement_value,AP_SCRIPTING_M4_FLASH_BASE),--defsym=placement_flash_size=$(call placement_value,AP_SCRIPTING_M4_FLASH_SIZE),--defsym=placement_ram_base=$(call placement_value,AP_SCRIPTING_M4_RAM_BASE),--defsym=placement_ram_size=$(call placement_value,AP_SCRIPTING_M4_RAM_SIZE),--defsym=placement_priv_ram_size=$(call placement_value,AP_SCRIPTING_M4_PRIV_RAM_SIZE),--defsym=placement_lua_stack_size=$(call placement_value,AP_SCRIPTING_M4_LUA_STACK_SIZE),--defsym=placement_lua_heap_min=$(call placement_value,AP_SCRIPTING_M4_LUA_HEAP_MIN)
USE_LTO = no
USE_VERBOSE_COMPILE = no
USE_SMART_BUILD = yes
USE_PROCESS_STACKSIZE = 0x800
USE_EXCEPTIONS_STACKSIZE = 0x800
USE_FPU = hard
USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv4-sp-d16

PROJECT = scripting_m4
MCU = cortex-m4
M4DIR := $(AP_ROOT)/libraries/AP_Scripting/m4
CONFDIR := $(M4DIR)/chibios
DEPDIR := $(BUILDDIR)/.dep
CUBEFRAMEWORK := $(AP_ROOT)/modules/CubeFramework
LUA := $(AP_ROOT)/libraries/AP_Scripting/lua/src
# AP_Math and AP_Common/Location for the M4's local pure-data types, through wrappers
# (location.o and Location.o would collide on a case-insensitive file system)
M4_LOCAL_SOURCES = m4_math_ap_math.cpp m4_math_vector2.cpp m4_math_vector3.cpp m4_math_matrix3.cpp \
                   m4_math_quaternion.cpp m4_math_location.cpp m4_math_ap_location.cpp
# networking on the M4 (NETWORKING_M4, from Network RAM Split): lwIP and the network service,
# privileged. Not slipif.c (it calls sio_*), bridgeif*, lowpan6*, zepif.c: none is used
NETM4 := $(AP_ROOT)/libraries/AP_Networking
LWIP := $(AP_ROOT)/modules/lwip/src
ifeq ($(NETWORKING_M4),1)
NET_CSRC = $(wildcard $(LWIP)/core/*.c $(LWIP)/core/ipv4/*.c $(LWIP)/api/*.c $(LWIP)/netif/ppp/*.c) \
           $(LWIP)/netif/ethernet.c $(NETM4)/m4/net_libc.c
# the network service: every C++ file in AP_Networking/m4 but the SITL process's POSIX ones
NET_CPPSRC = $(filter-out %_posix.cpp,$(wildcard $(NETM4)/m4/*.cpp)) $(NETM4)/AP_Networking_M4_Rings.cpp \
             $(CONFDIR)/net_platform_chibios.cpp
NET_INCDIR = $(NETM4)/m4 $(LWIP)/include $(NETM4)/lwip_hal/include $(NETM4)
NET_INCOPT = -I$(NETM4)/m4
NET_PRIVILEGED_LIBC = strcpy strncpy
endif

include $(CHIBIOS)/os/license/license.mk
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32h7xx_m4.mk
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv7-M/compilers/GCC/mk/port.mk

LDSCRIPT = $(CONFDIR)/scripting_m4.ld

# ap_setjmp.c saves d8-d15, which the gcc 10 newlib setjmp does not (Lua errors use it)
CSRC = $(ALLCSRC) \
       $(addprefix $(LUA)/,$(M4_LUA_SOURCES)) \
       $(AP_ROOT)/libraries/AP_HAL_ChibiOS/ap_setjmp.c \
       $(NET_CSRC)
CPPSRC = $(ALLCPPSRC) \
         $(CONFDIR)/main.cpp \
         $(CONFDIR)/fault.cpp \
         $(CONFDIR)/lua_port.cpp \
         $(CONFDIR)/lua_heap.cpp \
         $(M4DIR)/m4_service.cpp \
         $(M4DIR)/m4_lua.cpp \
         $(M4DIR)/m4_rpc_client.cpp \
         $(M4DIR)/m4_binding_tables.cpp \
         $(M4DIR)/m4_bindings.cpp \
         $(M4DIR)/m4_boxed_numerics.cpp \
         $(M4DIR)/m4_binding_local.cpp \
         $(M4DIR)/m4_file_proxy.cpp \
         $(M4DIR)/m4_wrappers.cpp \
         $(addprefix $(M4DIR)/local/,$(M4_LOCAL_SOURCES)) \
         $(CUBEFRAMEWORK)/CubePilotFW/Channel.cpp \
         $(CUBEFRAMEWORK)/CubePilotFW/SpscRing.cpp \
         $(CUBEFRAMEWORK)/CubePilotFW/PeerHeader.cpp \
         $(CUBEFRAMEWORK)/CubePilotFW/pal/HSEM.cpp \
         $(NET_CPPSRC)
ASMSRC = $(ALLASMSRC)
ASMXSRC = $(ALLXASMSRC)
# the Lua core finds its AP headers in the shim directory. rules.mk sorts INCDIR, so
# $(AP_ROOT)/libraries would come before the shim and shadow it; the generated root
# (<AP_Scripting/lua_remote_*>) holds only build outputs; the generated local userdata
# code finds m4_binding_deps.h in $(M4DIR)
INCDIR = $(NET_INCDIR) $(CONFDIR) $(ALLINC) $(M4DIR)/shim $(M4DIR) $(AP_ROOT)/libraries/AP_Scripting $(M4_GENERATED_ROOT) \
         $(CUBEFRAMEWORK)

CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes
CPPWARN = -Wall -Wextra -Wundef

# CORE_CM4 selects the M4 view of the CMSIS device header; CF_PLATFORM_BETAFLIGHT
# makes CubeFramework's Config.h include it (HSEM.cpp built without USE_CHIBIOS,
# as Tools/renode/tests/dual_core_images.py does). PORT_USE_SYSCALL: the
# unprivileged Lua thread. ARDUPILOT_BUILD and LUA_32BITS as the M7's Lua build.
UDEFS = -DCORE_CM4 -DSTM32H757xx -DCF_PLATFORM_BETAFLIGHT -DPORT_USE_SYSCALL=1 -DARDUPILOT_BUILD -DLUA_32BITS=1 -DAP_LUA_NO_PRINTF_A=1 -DAP_LUA_MAX_NUMBER_TEXT=100 \
        $(PLACEMENT_DEFS)
UADEFS = -DCORE_CM4 -DSTM32H757xx
UINCDIR =
# scripting_m4.ld includes the Lua RAM lists written below
ULIBDIR = $(BUILDDIR)
# newlib-nano: printf without float unless _printf_float is pulled in (Lua's tostring
# and string.format need it). The Lua core's file calls go to m4_file_proxy.cpp
# (m4/shim/AP_Filesystem/posix_compat.h), never to newlib's nosys stubs
ULIBS = --specs=nano.specs --specs=nosys.specs -u _printf_float -lm

# RAM privilege per object file (scripting_m4.ld): statics are privileged unless
# their object is listed here. These run on the unprivileged Lua thread and go to
# Lua RAM: the Lua core, the M4 Lua runtime (lua_alloc, its heap, the RPC client, the
# binding loader, the file proxy and its stream pool, the local userdata and the local
# pure-data types (AP_Math, Location))
# and newlib (the malloc behind lua_alloc, errno and the reent state of printf and strtod).
LUA_RAM_OBJECTS = $(M4_LUA_SOURCES:.c=.o) ap_setjmp.o m4_lua.o lua_heap.o m4_rpc_client.o \
                  m4_bindings.o m4_boxed_numerics.o m4_binding_local.o m4_file_proxy.o \
                  m4_wrappers.o \
                  $(M4_LOCAL_SOURCES:.cpp=.o)
LUA_RAM_ARCHIVES = libc_nano.a libg_nano.a libm.a libnosys.a
lua_ram_input = $(foreach object,$(LUA_RAM_OBJECTS),'*/obj/$(object)($(1))') \
                $(foreach archive,$(LUA_RAM_ARCHIVES),'*/$(archive):*($(1))')
# written at parse time, replaced only when the list changes
write_if_changed = printf '%s\n' $(2) > '$(1).new' && { cmp -s '$(1).new' '$(1)' || mv '$(1).new' '$(1)'; } && rm -f '$(1).new'
LUA_RAM_LISTS := $(BUILDDIR)/lua_ram_data.ld $(BUILDDIR)/lua_ram_bss.ld
$(shell mkdir -p '$(BUILDDIR)' && \
        $(call write_if_changed,$(BUILDDIR)/lua_ram_data.ld,$(call lua_ram_input,.data .data.*)) && \
        $(call write_if_changed,$(BUILDDIR)/lua_ram_bss.ld,$(call lua_ram_input,.bss .bss.* COMMON)))

# privileged code calls or copies through these: they must be in privileged RAM
# (HSEM ISR callbacks, the window pointers inside channel and service, the LuaLink)
PRIVILEGED_SYMBOLS = CubePilotFW::hsemCallbacks channel service call_link lua_link service_wake

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk

# objects that compile AP_Math take the M7's flags for it (Tools/ardupilotwaf/boards.py,
# chibios.py), so the M4 computes what the M7 computes in-process: float literals, and
# sqrtf without an errno call; the M7's warning set for the AP_Math sources.
# CPPFLAGS reads CPPOPT when a recipe runs (rules.mk)
M4_MATH_OBJECTS = $(addprefix $(OBJDIR)/,$(M4_LOCAL_SOURCES:.cpp=.o) m4_binding_local.o)
$(M4_MATH_OBJECTS): CPPOPT += -fsingle-precision-constant -fno-math-errno \
                              -Wno-unused-parameter -Wno-unknown-pragmas

$(BUILDDIR)/$(PROJECT).elf: $(LUA_RAM_LISTS)

# fails (and removes the image) unless each symbol exists and lies in ram0. The addresses are
# compared as strings: awk reads a hex address such as 300138e0 as a number in exponent form
all: $(BUILDDIR)/privileged_symbols.ok
$(BUILDDIR)/privileged_symbols.ok: $(BUILDDIR)/$(PROJECT).elf
	@$(TRGT)nm -C '$<' | awk -v wanted_list='$(PRIVILEGED_SYMBOLS)' ' \
	    BEGIN { count = split(wanted_list, names, " "); for (i = 1; i <= count; i++) wanted[names[i]] = 1 } \
	    { name = $$0; sub(/^[0-9a-f]+ [A-Za-z] /, "", name) } \
	    name == "__ram0_base__" { privileged_base = $$1 } \
	    name == "__ram1_base__" { lua_base = $$1 } \
	    (name in wanted) { found[name] = found[name] " " $$1 } \
	    END { \
	        status = 0; \
	        for (name in wanted) { \
	            if (!(name in found)) { print "scripting_m4: " name " not found"; status = 1; continue } \
	            count = split(found[name], addresses, " "); \
	            for (i = 1; i <= count; i++) { \
	                if ("x" addresses[i] < "x" privileged_base || "x" addresses[i] >= "x" lua_base) { \
	                    print "scripting_m4: " name " at 0x" addresses[i] " is not in privileged RAM"; status = 1 \
	                } \
	            } \
	        } \
	        exit status \
	    }' || { rm -f '$<'; exit 1; }
	@touch '$@'

# The local pure-data types are built against shims with no vehicle state (AP_AHRS.h,
# AP_Terrain.h, AP_CustomRotations.h): a Location method that reads home, the EKF origin
# or terrain returns false there, and a custom rotation is left unapplied. Those methods
# must stay remote rows; fails (and removes the image) if any is in the image
# nm -C prints a return type before template members and template arguments before the
# parameters (bool Location::get_vector_from_origin_NEU_m<Vector3<float> >(...) const)
STATE_READING_SYMBOLS = Location::(get_alt_cm|get_alt_m|change_alt_frame|get_height_above|get_distance_NED_alt_frame|get_vector_from_origin_|get_vector_xy_from_origin_)[A-Za-z_]*(<.*>)?\\( \
                        Vector3<[a-z]+>::rotate(_inverse)?\\(Rotation QuaternionT<[a-z]+>::(from_rotation|rotate)\\(Rotation AP_CustomRotations::
# demangled names copied from nm -C of the M7 image: "state" lines must match, "local" must not
STATE_READING_SAMPLES = $(M4DIR)/chibios/state_reading_samples.txt
empty :=
space := $(empty) $(empty)
all: $(BUILDDIR)/local_pure_data.ok
$(BUILDDIR)/local_pure_data.ok: $(BUILDDIR)/$(PROJECT).elf $(STATE_READING_SAMPLES)
	@awk -F'|' -v pattern='(^| )($(subst $(space),|,$(STATE_READING_SYMBOLS)))' ' \
	    ($$1 == "state" && $$2 !~ pattern) { print "scripting_m4: sample not matched by STATE_READING_SYMBOLS: " $$2; status = 1 } \
	    ($$1 == "local" && $$2 ~ pattern) { print "scripting_m4: sample matched by STATE_READING_SYMBOLS: " $$2; status = 1 } \
	    END { exit status }' '$(STATE_READING_SAMPLES)' || { rm -f '$<'; exit 1; }
	@$(TRGT)nm -C --defined-only '$<' | awk -v pattern='(^| )($(subst $(space),|,$(STATE_READING_SYMBOLS)))' ' \
	    { name = $$0; sub(/^[0-9a-f]+ [A-Za-z] /, "", name) } \
	    name ~ pattern { print "scripting_m4: " name " is state-reading and must not be local"; status = 1 } \
	    END { exit status }' || { rm -f '$<'; exit 1; }
	@touch '$@'

# Privileged objects may call only each other, ChibiOS, CubeFramework and the
# stateless libc/libgcc helpers below: newlib's errno, malloc, stdio and locale
# state is in Lua RAM, where Lua can rewrite it. Outside ChibiOS itself none may
# allocate from memcore, a heap, a pool or the factory: those hand out Lua RAM.
# lua_port.o's _sbrk sets errno and lua_thread enters lua_main, both on the Lua thread.
PRIVILEGED_OBJECTS = $(filter-out $(addprefix $(OBJDIR)/,$(LUA_RAM_OBJECTS)),$(OBJS))
CHIBIOS_OBJECTS = $(addprefix $(OBJDIR)/,$(notdir $(ALLCSRC:.c=.o) $(ALLCPPSRC:.cpp=.o) $(ALLASMSRC:.s=.o) $(ALLXASMSRC:.S=.o)))
PRIVILEGED_LIBC = memcpy memmove memset memcmp strcmp strncmp strlen $(NET_PRIVILEGED_LIBC)
LUA_PORT_LUA_IMPORTS = __errno _ZN11ScriptingM48lua_mainEv
all: $(BUILDDIR)/privileged_imports.ok
$(BUILDDIR)/privileged_imports.ok: $(BUILDDIR)/$(PROJECT).elf
	@{ $(TRGT)nm -A -g --defined-only $(PRIVILEGED_OBJECTS) | sed 's/^/D /'; \
	   $(TRGT)nm -A -u $(PRIVILEGED_OBJECTS) | sed 's/^/U /'; } | \
	awk -v chibios_list='$(CHIBIOS_OBJECTS)' -v libc_list='$(PRIVILEGED_LIBC)' \
	    -v lua_port='$(OBJDIR)/lua_port.o' -v lua_port_list='$(LUA_PORT_LUA_IMPORTS)' ' \
	    BEGIN { \
	        split(chibios_list, names, " "); for (i in names) chibios[names[i]] = 1; \
	        split(libc_list, names, " "); for (i in names) libc[names[i]] = 1; \
	        split(lua_port_list, names, " "); for (i in names) lua_port_imports[names[i]] = 1 \
	    } \
	    { object = $$2; sub(/:.*/, "", object); symbol = $$NF } \
	    $$1 == "D" { defined[symbol] = 1; next } \
	    { imports[++count] = object " " symbol } \
	    END { \
	        status = 0; \
	        for (i = 1; i <= count; i++) { \
	            split(imports[i], entry, " "); object = entry[1]; symbol = entry[2]; \
	            if (symbol ~ /^(chCoreAlloc|chHeapAlloc|chPool|chFactory|chThdCreateFromHeap|chThdCreateFromMemoryPool)/) { \
	                if (!(object in chibios)) { print "scripting_m4: privileged " object " allocates with " symbol; status = 1 } \
	                continue \
	            } \
	            if (symbol in libc || symbol ~ /^__aeabi_/ || symbol ~ /^__[a-z0-9_]+__$$/ || symbol in defined) continue; \
	            if (object == lua_port && symbol in lua_port_imports) continue; \
	            print "scripting_m4: privileged " object " uses " symbol; status = 1 \
	        } \
	        exit status \
	    }' || { rm -f '$<'; exit 1; }
	@touch '$@'
