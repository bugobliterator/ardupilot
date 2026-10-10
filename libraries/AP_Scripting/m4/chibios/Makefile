# Lua scripting Cortex-M4 image for the STM32H757: ChibiOS RT only (no HAL).
# Built by waf (Tools/ardupilotwaf/cubeframework.py); by hand:
#   make -f libraries/AP_Scripting/m4/chibios/Makefile CHIBIOS=$PWD/modules/ChibiOS \
#        AP_ROOT=$PWD BUILDDIR=/tmp/scripting_m4

USE_OPT = -Os -ggdb -fomit-frame-pointer -falign-functions=16
USE_COPT =
USE_CPPOPT = -std=gnu++17 -fno-rtti -fno-exceptions -fno-threadsafe-statics -fno-use-cxa-atexit
USE_LINK_GC = yes
# scripting_m4.ld asserts its flash0 against AP_ScriptingM4_Placement.h;
# a value missing from the header becomes 0 and fails the assert
PLACEMENT_H = $(AP_ROOT)/libraries/AP_Scripting/AP_ScriptingM4_Placement.h
placement_value = $(shell value=$$(sed -n 's/^.define $(1)  *//p' '$(PLACEMENT_H)' | tr -d U); echo $$(($${value:-0})))
USE_LDOPT = --defsym=placement_flash_base=$(call placement_value,AP_SCRIPTING_M4_FLASH_BASE),--defsym=placement_flash_size=$(call placement_value,AP_SCRIPTING_M4_FLASH_SIZE)
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

include $(CHIBIOS)/os/license/license.mk
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32h7xx_m4.mk
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv7-M/compilers/GCC/mk/port.mk

LDSCRIPT = $(CONFDIR)/scripting_m4.ld

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
INCDIR = $(CONFDIR) $(ALLINC) $(AP_ROOT)/libraries/AP_Scripting $(CUBEFRAMEWORK)

CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes
CPPWARN = -Wall -Wextra -Wundef

# CORE_CM4 selects the M4 view of the CMSIS device header; CF_PLATFORM_BETAFLIGHT
# makes CubeFramework's Config.h include it (HSEM.cpp built without USE_CHIBIOS,
# as Tools/renode/tests/dual_core_images.py does)
UDEFS = -DCORE_CM4 -DSTM32H757xx -DCF_PLATFORM_BETAFLIGHT
UADEFS = -DCORE_CM4 -DSTM32H757xx
UINCDIR =
ULIBDIR =
ULIBS = --specs=nano.specs

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk
