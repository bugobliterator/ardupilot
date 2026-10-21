# Native Rust module Cortex-M4 image for the STM32H757: ChibiOS RT only (no HAL)
# and the privileged M4 service. Built by waf (Tools/ardupilotwaf/cubeframework.py),
# which passes M4_GENERATED_ROOT; by hand:
#   make -f libraries/AP_RustM4/m4/chibios/Makefile CHIBIOS=$PWD/modules/ChibiOS \
#        AP_ROOT=$PWD BUILDDIR=/tmp/rust_m4 \
#        M4_GENERATED_ROOT=$PWD/build/CubeOrangePlus-SimOnHardWare-M4Rust/libraries

ifeq ($(M4_GENERATED_ROOT),)
$(error M4_GENERATED_ROOT is not set: waf passes the build's libraries directory (generated headers))
endif

USE_OPT = -Os -ggdb -fomit-frame-pointer -falign-functions=16
USE_COPT =
USE_CPPOPT = -std=gnu++17 -fno-rtti -fno-exceptions -fno-threadsafe-statics -fno-use-cxa-atexit
USE_LINK_GC = yes
# rust_m4.ld asserts its memory against AP_RustM4_Placement.h;
# a value missing from the header becomes 0 and fails the assert
PLACEMENT_H = $(AP_ROOT)/libraries/AP_RustM4/AP_RustM4_Placement.h
placement_value = $(shell value=$$(sed -n 's/^.define $(1)  *//p' '$(PLACEMENT_H)' | tr -d U); echo $$(($${value:-0})))
USE_LDOPT = --defsym=placement_flash_base=$(call placement_value,AP_RUST_M4_FLASH_BASE),--defsym=placement_flash_size=$(call placement_value,AP_RUST_M4_FLASH_SIZE),--defsym=placement_ram_base=$(call placement_value,AP_RUST_M4_RAM_BASE),--defsym=placement_ram_size=$(call placement_value,AP_RUST_M4_RAM_SIZE),--defsym=placement_priv_ram_size=$(call placement_value,AP_RUST_M4_PRIV_RAM_SIZE)
USE_LTO = no
USE_VERBOSE_COMPILE = no
USE_SMART_BUILD = yes
USE_PROCESS_STACKSIZE = 0x800
USE_EXCEPTIONS_STACKSIZE = 0x800
USE_FPU = hard
USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv4-sp-d16

PROJECT = rust_m4
MCU = cortex-m4
M4DIR := $(AP_ROOT)/libraries/AP_RustM4/m4
CONFDIR := $(M4DIR)/chibios
DEPDIR := $(BUILDDIR)/.dep
CUBEFRAMEWORK := $(AP_ROOT)/modules/CubeFramework

include $(CHIBIOS)/os/license/license.mk
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32h7xx_m4.mk
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv7-M/compilers/GCC/mk/port.mk

LDSCRIPT = $(CONFDIR)/rust_m4.ld

CSRC = $(ALLCSRC)
CPPSRC = $(ALLCPPSRC) \
         $(CONFDIR)/main.cpp \
         $(CONFDIR)/fault.cpp \
         $(CONFDIR)/crc_port.cpp \
         $(CONFDIR)/module_port.cpp \
         $(CONFDIR)/trampolines.cpp \
         $(CONFDIR)/m4_selftest.cpp \
         $(M4DIR)/m4_service.cpp \
         $(M4DIR)/module_host.cpp \
         $(AP_ROOT)/libraries/AP_Rust/Image.cpp \
         $(CUBEFRAMEWORK)/CubePilotFW/Channel.cpp \
         $(CUBEFRAMEWORK)/CubePilotFW/SpscRing.cpp \
         $(CUBEFRAMEWORK)/CubePilotFW/PeerHeader.cpp \
         $(CUBEFRAMEWORK)/CubePilotFW/pal/HSEM.cpp
# the local set the M4 answers itself: AP_Math through wrappers (m4/local), the generated math
# adapters and the generated local dispatch
M4_LOCAL_SOURCES = m4_math_ap_math.cpp m4_math_vector3.cpp m4_math_matrix3.cpp m4_math_quaternion.cpp
CPPSRC += $(addprefix $(M4DIR)/local/,$(M4_LOCAL_SOURCES)) \
          $(M4_GENERATED_ROOT)/AP_Rust/BindingMath_generated.cpp \
          $(M4_GENERATED_ROOT)/AP_Rust/BindingLocal_generated.cpp
ASMSRC = $(ALLASMSRC)
ASMXSRC = $(ALLXASMSRC)
# rules.mk sorts INCDIR, so $(AP_ROOT)/libraries would come before the shim and shadow it: the
# shim directory forwards the few real headers the M4 needs by relative path instead
INCDIR = $(CONFDIR) $(ALLINC) $(M4DIR)/shim $(M4DIR) $(AP_ROOT)/libraries/AP_RustM4 $(M4_GENERATED_ROOT) \
         $(CUBEFRAMEWORK)

CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes
CPPWARN = -Wall -Wextra -Wundef

# CORE_CM4 selects the M4 view of the CMSIS device header; CF_PLATFORM_BETAFLIGHT
# makes CubeFramework's Config.h include it (HSEM.cpp built without USE_CHIBIOS,
# as Tools/renode/tests/dual_core_images.py does). PORT_USE_SYSCALL: the
# unprivileged module thread. ARDUPILOT_BUILD as the M7's build. Fault injection
# (RUST_M4_INJECT) is on in every M4 build.
UDEFS = -DCORE_CM4 -DSTM32H757xx -DCF_PLATFORM_BETAFLIGHT -DPORT_USE_SYSCALL=1 -DARDUPILOT_BUILD -DAP_RUST_M4_FAULT_INJECTION_ENABLED=1 \
        -DCF_PAL_CRC32_EXTERNAL -DAP_SCRIPTING_RUST_ENABLED=1
UADEFS = -DCORE_CM4 -DSTM32H757xx
UINCDIR =
ULIBDIR =
ULIBS = --specs=nano.specs --specs=nosys.specs -lm

# privileged code calls or copies through these: they must be in privileged RAM
# (HSEM ISR callbacks, the window pointers inside channel and service, the ModuleLink, the module
# thread's syscall stack and its context, the loader with the loaded module's context and heap bitmaps)
PRIVILEGED_SYMBOLS = CubePilotFW::hsemCallbacks channel service call_link module_wa module_context module_host

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk

# objects that compile AP_Math take the M7's flags for it (Tools/ardupilotwaf/boards.py,
# chibios.py): float literals, and sqrtf without an errno call; the M7's warning set for the
# AP_Math sources. CPPFLAGS reads CPPOPT when a recipe runs (rules.mk)
M4_MATH_OBJECTS = $(addprefix $(OBJDIR)/,$(M4_LOCAL_SOURCES:.cpp=.o) BindingMath_generated.o BindingLocal_generated.o)
$(M4_MATH_OBJECTS): CPPOPT += -fsingle-precision-constant -fno-math-errno -Wno-unused-parameter -Wno-unknown-pragmas

# fails (and removes the image) unless each symbol exists and lies in ram0, below the module
# area, and unless the kernel's core allocator is absent (it would hand out module area RAM).
# nm prints fixed-width lower-case hex, compared as strings: awk would read 300016e8 as a number
all: $(BUILDDIR)/privileged_symbols.ok
$(BUILDDIR)/privileged_symbols.ok: $(BUILDDIR)/$(PROJECT).elf
	@! $(TRGT)nm '$<' | grep -q -w chCoreAlloc || { echo "rust_m4: chCoreAlloc is linked"; rm -f '$<'; exit 1; }
	@$(TRGT)nm -C '$<' | awk -v wanted_list='$(PRIVILEGED_SYMBOLS)' ' \
	    BEGIN { count = split(wanted_list, names, " "); for (i = 1; i <= count; i++) wanted[names[i]] = 1 } \
	    { name = $$0; sub(/^[0-9a-f]+ [A-Za-z] /, "", name) } \
	    name == "__ram0_base__" { privileged_base = $$1 } \
	    name == "__module_area_base__" { module_base = $$1 } \
	    (name in wanted) { found[name] = found[name] " " $$1 } \
	    END { \
	        status = 0; \
	        if (module_base == "") { print "rust_m4: __module_area_base__ not found"; status = 1 } \
	        for (name in wanted) { \
	            if (!(name in found)) { print "rust_m4: " name " not found"; status = 1; continue } \
	            count = split(found[name], addresses, " "); \
	            for (i = 1; i <= count; i++) { \
	                if ((addresses[i] "") < (privileged_base "") || (addresses[i] "") >= (module_base "")) { \
	                    print "rust_m4: " name " at 0x" addresses[i] " is not in privileged RAM"; status = 1 \
	                } \
	            } \
	        } \
	        exit status \
	    }' || { rm -f '$<'; exit 1; }
	@touch '$@'

# fails (and removes the image) if the selftest left code or constants outside .m4_selftest, its
# RX range (rust_m4.ld)
all: $(BUILDDIR)/selftest_section.ok
$(BUILDDIR)/selftest_section.ok: $(BUILDDIR)/$(PROJECT).elf
	@$(TRGT)size -A $(OBJDIR)/m4_selftest.o | awk '$$1 ~ /^\.(text|rodata)/ && $$2 != 0 { print "rust_m4: m4_selftest.o has " $$1; bad = 1 } END { exit bad }' || { rm -f '$<'; exit 1; }
	@$(TRGT)nm '$<' | awk '$$3 == "__m4_selftest_start__" { start = $$1 } $$3 == "__m4_selftest_end__" { end = $$1 } END { exit !(start != "" && end > start) }' || { echo "rust_m4: .m4_selftest is empty"; rm -f '$<'; exit 1; }
	@touch '$@'

# fails (and removes the image) if an M4 object reached a board header of the M7
all: $(BUILDDIR)/no_board_headers.ok
$(BUILDDIR)/no_board_headers.ok: $(BUILDDIR)/$(PROJECT).elf
	@grep -l -E '(^|[ /])hwdef\.h|AP_HAL_ChibiOS/' $(BUILDDIR)/.dep/*.d; status=$$?; \
	if [ $$status -eq 0 ]; then echo "rust_m4: an M4 object includes M7 board headers"; fi; \
	if [ $$status -gt 1 ]; then echo "rust_m4: cannot read the dependency files"; fi; \
	if [ $$status -ne 1 ]; then rm -f '$<'; exit 1; fi
	@touch '$@'

# Privileged objects may call only each other, ChibiOS, CubeFramework and the
# stateless libc/libgcc helpers below: newlib's errno, malloc, stdio and locale
# state is shared with the unprivileged side. Outside ChibiOS itself none may
# allocate from memcore, a heap, a pool or the factory: those hand out module area RAM.
PRIVILEGED_OBJECTS = $(OBJS)
CHIBIOS_OBJECTS = $(addprefix $(OBJDIR)/,$(notdir $(ALLCSRC:.c=.o) $(ALLCPPSRC:.cpp=.o) $(ALLASMSRC:.s=.o) $(ALLXASMSRC:.S=.o)))
# the float libm the local set calls (fmod: AP_Math.cpp's wrap_2PI(double), which no local id
# reaches and the linker drops; the image's f64 audit finds it if it is ever linked)
PRIVILEGED_LIBM = acosf asinf atan2f ceilf cosf expf floorf fmodf logf powf roundf sinf tanf fmod
PRIVILEGED_LIBC = memcpy memmove memset memcmp strcmp strncmp strlen $(PRIVILEGED_LIBM)
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
	                if (!(object in chibios)) { print "rust_m4: privileged " object " allocates with " symbol; status = 1 } \
	                continue \
	            } \
	            if (symbol in libc || symbol ~ /^__aeabi_/ || symbol ~ /^__[a-z0-9_]+__$$/ || symbol in defined) continue; \
	            print "rust_m4: privileged " object " uses " symbol; status = 1 \
	        } \
	        exit status \
	    }' || { rm -f '$<'; exit 1; }
	@touch '$@'

# fails (and removes the image) if a software CRC-32 is linked: CRC-32 runs on the CRC unit
# (crc_port.cpp), which also defines CubeFramework's cubefw_pal_crc32
all: $(BUILDDIR)/software_crc_absent.ok
$(BUILDDIR)/software_crc_absent.ok: $(BUILDDIR)/$(PROJECT).elf
	@! $(TRGT)nm -C '$<' | grep -E 'crc32_software|crc_crc32|crc32_table|crc_table|kNibble' || { echo "rust_m4: software CRC in the M4 image"; rm -f '$<'; exit 1; }
	@$(TRGT)nm '$<' | grep -q ' T cubefw_pal_crc32$$' || { echo "rust_m4: cubefw_pal_crc32 is not the unit's"; rm -f '$<'; exit 1; }
	@touch '$@'
