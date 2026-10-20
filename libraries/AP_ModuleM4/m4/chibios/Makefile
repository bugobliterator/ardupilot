# Native Rust module Cortex-M4 image for the STM32H757: ChibiOS RT only (no HAL)
# and the privileged M4 service. Built by waf (Tools/ardupilotwaf/cubeframework.py),
# which passes M4_GENERATED_ROOT; by hand:
#   make -f libraries/AP_ModuleM4/m4/chibios/Makefile CHIBIOS=$PWD/modules/ChibiOS \
#        AP_ROOT=$PWD BUILDDIR=/tmp/module_m4 \
#        M4_GENERATED_ROOT=$PWD/build/CubeOrangePlus-SimOnHardWare-M4Module/libraries

ifeq ($(M4_GENERATED_ROOT),)
$(error M4_GENERATED_ROOT is not set: waf passes the build's libraries directory (generated headers))
endif

USE_OPT = -Os -ggdb -fomit-frame-pointer -falign-functions=16
USE_COPT =
USE_CPPOPT = -std=gnu++17 -fno-rtti -fno-exceptions -fno-threadsafe-statics -fno-use-cxa-atexit
USE_LINK_GC = yes
# module_m4.ld asserts its memory against AP_ModuleM4_Placement.h;
# a value missing from the header becomes 0 and fails the assert
PLACEMENT_H = $(AP_ROOT)/libraries/AP_ModuleM4/AP_ModuleM4_Placement.h
placement_value = $(shell value=$$(sed -n 's/^.define $(1)  *//p' '$(PLACEMENT_H)' | tr -d U); echo $$(($${value:-0})))
USE_LDOPT = --defsym=placement_flash_base=$(call placement_value,AP_MODULE_M4_FLASH_BASE),--defsym=placement_flash_size=$(call placement_value,AP_MODULE_M4_FLASH_SIZE),--defsym=placement_ram_base=$(call placement_value,AP_MODULE_M4_RAM_BASE),--defsym=placement_ram_size=$(call placement_value,AP_MODULE_M4_RAM_SIZE),--defsym=placement_priv_ram_size=$(call placement_value,AP_MODULE_M4_PRIV_RAM_SIZE)
USE_LTO = no
USE_VERBOSE_COMPILE = no
USE_SMART_BUILD = yes
USE_PROCESS_STACKSIZE = 0x800
USE_EXCEPTIONS_STACKSIZE = 0x800
USE_FPU = hard
USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv4-sp-d16

PROJECT = module_m4
MCU = cortex-m4
M4DIR := $(AP_ROOT)/libraries/AP_ModuleM4/m4
CONFDIR := $(M4DIR)/chibios
DEPDIR := $(BUILDDIR)/.dep
CUBEFRAMEWORK := $(AP_ROOT)/modules/CubeFramework

include $(CHIBIOS)/os/license/license.mk
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32h7xx_m4.mk
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv7-M/compilers/GCC/mk/port.mk

LDSCRIPT = $(CONFDIR)/module_m4.ld

CSRC = $(ALLCSRC)
CPPSRC = $(ALLCPPSRC) \
         $(CONFDIR)/main.cpp \
         $(CONFDIR)/fault.cpp \
         $(M4DIR)/m4_service.cpp \
         $(CUBEFRAMEWORK)/CubePilotFW/Channel.cpp \
         $(CUBEFRAMEWORK)/CubePilotFW/SpscRing.cpp \
         $(CUBEFRAMEWORK)/CubePilotFW/PeerHeader.cpp \
         $(CUBEFRAMEWORK)/CubePilotFW/pal/HSEM.cpp
ASMSRC = $(ALLASMSRC)
ASMXSRC = $(ALLXASMSRC)
# rules.mk sorts INCDIR, so $(AP_ROOT)/libraries would come before the shim and shadow it: the
# shim directory forwards the few real headers the M4 needs by relative path instead
INCDIR = $(CONFDIR) $(ALLINC) $(M4DIR)/shim $(M4DIR) $(AP_ROOT)/libraries/AP_ModuleM4 $(M4_GENERATED_ROOT) \
         $(CUBEFRAMEWORK)

CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes
CPPWARN = -Wall -Wextra -Wundef

# CORE_CM4 selects the M4 view of the CMSIS device header; CF_PLATFORM_BETAFLIGHT
# makes CubeFramework's Config.h include it (HSEM.cpp built without USE_CHIBIOS,
# as Tools/renode/tests/dual_core_images.py does). PORT_USE_SYSCALL: the
# unprivileged module thread. ARDUPILOT_BUILD as the M7's build. Fault injection
# (MOD_M4_INJECT) is on in every M4 build.
UDEFS = -DCORE_CM4 -DSTM32H757xx -DCF_PLATFORM_BETAFLIGHT -DPORT_USE_SYSCALL=1 -DARDUPILOT_BUILD -DAP_MODULE_M4_FAULT_INJECTION_ENABLED=1
UADEFS = -DCORE_CM4 -DSTM32H757xx
UINCDIR =
ULIBDIR =
ULIBS = --specs=nano.specs --specs=nosys.specs -lm

# privileged code calls or copies through these: they must be in privileged RAM
# (HSEM ISR callbacks, the window pointers inside channel and service, the ModuleLink)
PRIVILEGED_SYMBOLS = CubePilotFW::hsemCallbacks channel service call_link

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk

# fails (and removes the image) unless each symbol exists and lies in ram0, below the module
# area, and unless the kernel's core allocator is absent (it would hand out module area RAM)
all: $(BUILDDIR)/privileged_symbols.ok
$(BUILDDIR)/privileged_symbols.ok: $(BUILDDIR)/$(PROJECT).elf
	@! $(TRGT)nm '$<' | grep -q -w chCoreAlloc || { echo "module_m4: chCoreAlloc is linked"; rm -f '$<'; exit 1; }
	@$(TRGT)nm -C '$<' | awk -v wanted_list='$(PRIVILEGED_SYMBOLS)' ' \
	    BEGIN { count = split(wanted_list, names, " "); for (i = 1; i <= count; i++) wanted[names[i]] = 1 } \
	    { name = $$0; sub(/^[0-9a-f]+ [A-Za-z] /, "", name) } \
	    name == "__ram0_base__" { privileged_base = $$1 } \
	    name == "__module_area_base__" { module_base = $$1 } \
	    (name in wanted) { found[name] = found[name] " " $$1 } \
	    END { \
	        status = 0; \
	        if (module_base == "") { print "module_m4: __module_area_base__ not found"; status = 1 } \
	        for (name in wanted) { \
	            if (!(name in found)) { print "module_m4: " name " not found"; status = 1; continue } \
	            count = split(found[name], addresses, " "); \
	            for (i = 1; i <= count; i++) { \
	                if (addresses[i] < privileged_base || addresses[i] >= module_base) { \
	                    print "module_m4: " name " at 0x" addresses[i] " is not in privileged RAM"; status = 1 \
	                } \
	            } \
	        } \
	        exit status \
	    }' || { rm -f '$<'; exit 1; }
	@touch '$@'

# fails (and removes the image) if an M4 object reached a board header of the M7
all: $(BUILDDIR)/no_board_headers.ok
$(BUILDDIR)/no_board_headers.ok: $(BUILDDIR)/$(PROJECT).elf
	@! grep -l -E '(^|[ /])hwdef\.h|AP_HAL_ChibiOS/' $(BUILDDIR)/.dep/*.d || { echo "module_m4: an M4 object includes M7 board headers"; rm -f '$<'; exit 1; }
	@touch '$@'

# Privileged objects may call only each other, ChibiOS, CubeFramework and the
# stateless libc/libgcc helpers below: newlib's errno, malloc, stdio and locale
# state is shared with the unprivileged side. Outside ChibiOS itself none may
# allocate from memcore, a heap, a pool or the factory: those hand out module area RAM.
PRIVILEGED_OBJECTS = $(OBJS)
CHIBIOS_OBJECTS = $(addprefix $(OBJDIR)/,$(notdir $(ALLCSRC:.c=.o) $(ALLCPPSRC:.cpp=.o) $(ALLASMSRC:.s=.o) $(ALLXASMSRC:.S=.o)))
PRIVILEGED_LIBC = memcpy memmove memset memcmp strcmp strncmp strlen
all: $(BUILDDIR)/privileged_imports.ok
$(BUILDDIR)/privileged_imports.ok: $(BUILDDIR)/$(PROJECT).elf
	@{ $(TRGT)nm -A -g --defined-only $(PRIVILEGED_OBJECTS) | sed 's/^/D /'; \
	   $(TRGT)nm -A -u $(PRIVILEGED_OBJECTS) | sed 's/^/U /'; } | \
	awk -v chibios_list='$(CHIBIOS_OBJECTS)' -v libc_list='$(PRIVILEGED_LIBC)' ' \
	    BEGIN { \
	        split(chibios_list, names, " "); for (i in names) chibios[names[i]] = 1; \
	        split(libc_list, names, " "); for (i in names) libc[names[i]] = 1 \
	    } \
	    { object = $$2; sub(/:.*/, "", object); symbol = $$NF } \
	    $$1 == "D" { defined[symbol] = 1; next } \
	    { imports[++count] = object " " symbol } \
	    END { \
	        status = 0; \
	        for (i = 1; i <= count; i++) { \
	            split(imports[i], entry, " "); object = entry[1]; symbol = entry[2]; \
	            if (symbol ~ /^(chCoreAlloc|chHeapAlloc|chPool|chFactory|chThdCreateFromHeap|chThdCreateFromMemoryPool)/) { \
	                if (!(object in chibios)) { print "module_m4: privileged " object " allocates with " symbol; status = 1 } \
	                continue \
	            } \
	            if (symbol in libc || symbol ~ /^__aeabi_/ || symbol ~ /^__[a-z0-9_]+__$$/ || symbol in defined) continue; \
	            print "module_m4: privileged " object " uses " symbol; status = 1 \
	        } \
	        exit status \
	    }' || { rm -f '$<'; exit 1; }
	@touch '$@'
