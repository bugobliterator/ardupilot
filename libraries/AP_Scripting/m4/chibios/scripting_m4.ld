/*
    Lua scripting Cortex-M4 image for the STM32H757. Must match
    libraries/AP_Scripting/AP_ScriptingM4_Placement.h (AP_SCRIPTING_M4_FLASH_*,
    AP_SCRIPTING_M4_RAM_*) and FLASH_RESERVE_END_KB in hwdef/CubeOrangePlus-M4Lua.
    SRAM3 (the shared window) is accessed by address, not linked.
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
    ram0   (wx) : org = 0x30000000, len = 256k      /* D2 SRAM1 + SRAM2 */
    ram1   (wx) : org = 0x00000000, len = 0
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
REGION_ALIAS("HEAP_RAM", ram0);

/* placement_flash_*: from AP_ScriptingM4_Placement.h, passed by the Makefile */
ASSERT(ORIGIN(flash0) == placement_flash_base && LENGTH(flash0) == placement_flash_size,
       "scripting_m4.ld flash0 does not match AP_ScriptingM4_Placement.h")

INCLUDE rules_stacks.ld
INCLUDE rules_code.ld
INCLUDE rules_data.ld
INCLUDE rules_memory.ld
