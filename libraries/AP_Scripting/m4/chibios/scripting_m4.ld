/*
    Lua scripting Cortex-M4 image for the STM32H757. Must match
    libraries/AP_Scripting/AP_ScriptingM4_Placement.h (asserted below) and
    FLASH_RESERVE_END_KB in hwdef/CubeOrangePlus-M4Lua. SRAM3 (the shared
    window) is accessed by address, not linked.

    RAM privilege is set per object file. ram0 is privileged (MPU region 2)
    and the default: the stacks, the ChibiOS kernel, CubeFramework, the
    service and every object not listed as a Lua object. ram1 is Lua RAM,
    open to the unprivileged Lua thread: the Lua stack first, then .data and
    .bss of the Lua objects the Makefile lists (the Lua core, the M4 Lua
    runtime, newlib), then the Lua heap.
*/
MEMORY
{
    flash0 (rx) : org = 0x081C0000, len = 256k      /* last 256 KB of bank 2 */
    flash1 (rx) : org = 0x00000000, len = 0
    flash2 (rx) : org = 0x00000000, len = 0
    flash3 (rx) : org = 0x00000000, len = 0
    flash4 (rx) : org = 0x00000000, len = 0
    flash5 (rx) : org = 0x00000000, len = 0
    flash6 (rx) : org = 0x00000000, len = 0
    flash7 (rx) : org = 0x00000000, len = 0
    ram0   (wx) : org = 0x30000000, len = placement_priv_ram_size        /* D2 SRAM1, privileged */
    ram1   (wx) : org = 0x30000000 + placement_priv_ram_size, len = placement_ram_size - placement_priv_ram_size
    ram2   (wx) : org = 0x00000000, len = 0
    ram3   (wx) : org = 0x00000000, len = 0
    ram4   (wx) : org = 0x00000000, len = 0
    ram5   (wx) : org = 0x00000000, len = 0
    ram6   (wx) : org = 0x00000000, len = 0
    ram7   (wx) : org = 0x00000000, len = 0
}

REGION_ALIAS("VECTORS_FLASH", flash0);
REGION_ALIAS("VECTORS_FLASH_LMA", flash0);
REGION_ALIAS("XTORS_FLASH", flash0);
REGION_ALIAS("XTORS_FLASH_LMA", flash0);
REGION_ALIAS("TEXT_FLASH", flash0);
REGION_ALIAS("TEXT_FLASH_LMA", flash0);
REGION_ALIAS("RODATA_FLASH", flash0);
REGION_ALIAS("RODATA_FLASH_LMA", flash0);
REGION_ALIAS("VARIOUS_FLASH", flash0);
REGION_ALIAS("VARIOUS_FLASH_LMA", flash0);
REGION_ALIAS("RAM_INIT_FLASH_LMA", flash0);

REGION_ALIAS("MAIN_STACK_RAM", ram0);
REGION_ALIAS("PROCESS_STACK_RAM", ram0);
REGION_ALIAS("DATA_RAM", ram0);
REGION_ALIAS("DATA_RAM_LMA", flash0);
REGION_ALIAS("BSS_RAM", ram0);
REGION_ALIAS("HEAP_RAM", ram1);

/* placement_*: from AP_ScriptingM4_Placement.h, passed by the Makefile */
ASSERT(ORIGIN(flash0) == placement_flash_base && LENGTH(flash0) == placement_flash_size,
       "scripting_m4.ld flash0 does not match AP_ScriptingM4_Placement.h")
ASSERT(ORIGIN(ram0) == placement_ram_base && LENGTH(ram0) == placement_priv_ram_size &&
       ORIGIN(ram1) == ORIGIN(ram0) + LENGTH(ram0) && LENGTH(ram0) + LENGTH(ram1) == placement_ram_size,
       "scripting_m4.ld RAM split does not match AP_ScriptingM4_Placement.h")

/* the stacks as rules_stacks.ld, 32-byte aligned: the main thread's lowest 32 bytes are an MPU guard (region 5) */
SECTIONS
{
    .mstack (NOLOAD) :
    {
        . = ALIGN(32);
        __main_stack_base__ = .;
        . += __main_stack_size__;
        . = ALIGN(32);
        __main_stack_end__ = .;
    } > MAIN_STACK_RAM

    .pstack (NOLOAD) :
    {
        . = ALIGN(32);
        __process_stack_base__ = .;
        __main_thread_stack_base__ = .;
        . += __process_stack_size__;
        . = ALIGN(8);
        __process_stack_end__ = .;
        __main_thread_stack_end__ = .;
    } > PROCESS_STACK_RAM
}

INCLUDE rules_code.ld

/* Lua RAM, ahead of rules_data.ld so the Lua objects' input sections land here;
   .lua_data and .lua_bss are set up by __late_init() (main.cpp) */
SECTIONS
{
    /* the Lua thread's stack (lua_port.cpp), directly above privileged RAM */
    .lua_stack (NOLOAD) :
    {
        __lua_stack_base__ = .;
        KEEP(*(.bss.lua_stack))
        __lua_stack_end__ = .;
    } > ram1

    .lua_data : ALIGN(4)
    {
        __lua_data_base__ = .;
        INCLUDE lua_ram_data.ld
        . = ALIGN(4);
        __lua_data_end__ = .;
    } > ram1 AT > flash0
    __lua_data_load__ = LOADADDR(.lua_data);

    .lua_bss (NOLOAD) : ALIGN(4)
    {
        __lua_bss_base__ = .;
        INCLUDE lua_ram_bss.ld
        *(.bss.m4lua)
        . = ALIGN(4);
        __lua_bss_end__ = .;
    } > ram1
}

INCLUDE rules_data.ld
INCLUDE rules_memory.ld

ASSERT(__lua_stack_base__ == ORIGIN(ram1) && __lua_stack_end__ - __lua_stack_base__ == placement_lua_stack_size,
       "the Lua stack is not first in Lua RAM")
ASSERT(__heap_end__ - __heap_base__ >= placement_lua_heap_min,
       "the Lua heap is below AP_SCRIPTING_M4_LUA_HEAP_MIN: Lua RAM is too full")
