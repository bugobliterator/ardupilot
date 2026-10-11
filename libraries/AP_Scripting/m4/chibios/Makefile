# Lua scripting Cortex-M4 image for the STM32H757: ChibiOS RT only (no HAL), the
# Lua core and the M4 Lua runtime. Built by waf (Tools/ardupilotwaf/cubeframework.py),
# which passes M4_LUA_SOURCES; by hand:
#   make -f libraries/AP_Scripting/m4/chibios/Makefile CHIBIOS=$PWD/modules/ChibiOS \
#        AP_ROOT=$PWD BUILDDIR=/tmp/scripting_m4 M4_LUA_SOURCES="lapi.c lauxlib.c ..."

ifeq ($(M4_LUA_SOURCES),)
$(error M4_LUA_SOURCES is not set: waf passes the Lua core file list (Tools/ardupilotwaf/cubeframework.py))
endif

USE_OPT = -Os -ggdb -fomit-frame-pointer -falign-functions=16
USE_COPT =
USE_CPPOPT = -std=gnu++17 -fno-rtti -fno-exceptions -fno-threadsafe-statics -fno-use-cxa-atexit
USE_LINK_GC = yes
# scripting_m4.ld asserts its memory against AP_ScriptingM4_Placement.h;
# a value missing from the header becomes 0 and fails the assert
PLACEMENT_H = $(AP_ROOT)/libraries/AP_Scripting/AP_ScriptingM4_Placement.h
placement_value = $(shell value=$$(sed -n 's/^.define $(1)  *//p' '$(PLACEMENT_H)' | tr -d U); echo $$(($${value:-0})))
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

include $(CHIBIOS)/os/license/license.mk
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32h7xx_m4.mk
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv7-M/compilers/GCC/mk/port.mk

LDSCRIPT = $(CONFDIR)/scripting_m4.ld

# ap_setjmp.c saves d8-d15, which the gcc 10 newlib setjmp does not (Lua errors use it)
CSRC = $(ALLCSRC) \
       $(addprefix $(LUA)/,$(M4_LUA_SOURCES)) \
       $(AP_ROOT)/libraries/AP_HAL_ChibiOS/ap_setjmp.c
CPPSRC = $(ALLCPPSRC) \
         $(CONFDIR)/main.cpp \
         $(CONFDIR)/fault.cpp \
         $(CONFDIR)/lua_port.cpp \
         $(M4DIR)/m4_service.cpp \
         $(M4DIR)/m4_lua.cpp \
         $(M4DIR)/m4_rpc_client.cpp \
         $(CUBEFRAMEWORK)/CubePilotFW/Channel.cpp \
         $(CUBEFRAMEWORK)/CubePilotFW/SpscRing.cpp \
         $(CUBEFRAMEWORK)/CubePilotFW/PeerHeader.cpp \
         $(CUBEFRAMEWORK)/CubePilotFW/pal/HSEM.cpp
ASMSRC = $(ALLASMSRC)
ASMXSRC = $(ALLXASMSRC)
# the shim directory first: the Lua core finds its AP headers there
INCDIR = $(CONFDIR) $(ALLINC) $(M4DIR)/shim $(AP_ROOT)/libraries/AP_Scripting $(CUBEFRAMEWORK)

CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes
CPPWARN = -Wall -Wextra -Wundef

# CORE_CM4 selects the M4 view of the CMSIS device header; CF_PLATFORM_BETAFLIGHT
# makes CubeFramework's Config.h include it (HSEM.cpp built without USE_CHIBIOS,
# as Tools/renode/tests/dual_core_images.py does). PORT_USE_SYSCALL: the
# unprivileged Lua thread. ARDUPILOT_BUILD and LUA_32BITS as the M7's Lua build.
UDEFS = -DCORE_CM4 -DSTM32H757xx -DCF_PLATFORM_BETAFLIGHT -DPORT_USE_SYSCALL=1 -DARDUPILOT_BUILD -DLUA_32BITS=1 -DAP_LUA_NO_PRINTF_A=1
UADEFS = -DCORE_CM4 -DSTM32H757xx
UINCDIR =
# scripting_m4.ld includes the Lua RAM lists written below
ULIBDIR = $(BUILDDIR)
# newlib-nano: printf without float unless _printf_float is pulled in (Lua's tostring
# and string.format need it); nosys stubs the file calls, which fail on the M4
ULIBS = --specs=nano.specs --specs=nosys.specs -u _printf_float -lm

# RAM privilege per object file (scripting_m4.ld): statics are privileged unless
# their object is listed here. These run on the unprivileged Lua thread and go to
# Lua RAM: the Lua core, the M4 Lua runtime (lua_alloc, the RPC client) and newlib
# (the malloc behind lua_alloc, errno and the reent state of printf and strtod).
LUA_RAM_OBJECTS = $(M4_LUA_SOURCES:.c=.o) ap_setjmp.o m4_lua.o m4_rpc_client.o
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

$(BUILDDIR)/$(PROJECT).elf: $(LUA_RAM_LISTS)

# fails (and removes the image) unless each symbol exists and lies in ram0
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
	                if (addresses[i] < privileged_base || addresses[i] >= lua_base) { \
	                    print "scripting_m4: " name " at 0x" addresses[i] " is not in privileged RAM"; status = 1 \
	                } \
	            } \
	        } \
	        exit status \
	    }' || { rm -f '$<'; exit 1; }
	@touch '$@'

# Privileged objects may call only each other, ChibiOS, CubeFramework and the
# stateless libc/libgcc helpers below: newlib's errno, malloc, stdio and locale
# state is in Lua RAM, where Lua can rewrite it. Outside ChibiOS itself none may
# allocate from memcore, a heap, a pool or the factory: those hand out Lua RAM.
# lua_port.o's _sbrk sets errno and lua_thread enters lua_main, both on the Lua thread.
PRIVILEGED_OBJECTS = $(filter-out $(addprefix $(OBJDIR)/,$(LUA_RAM_OBJECTS)),$(OBJS))
CHIBIOS_OBJECTS = $(addprefix $(OBJDIR)/,$(notdir $(ALLCSRC:.c=.o) $(ALLCPPSRC:.cpp=.o) $(ALLASMSRC:.s=.o) $(ALLXASMSRC:.S=.o)))
PRIVILEGED_LIBC = memcpy memmove memset memcmp strcmp strncmp strlen
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
