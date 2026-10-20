/*
    Native Rust module Cortex-M4 image for the STM32H757. Must match
    libraries/AP_RustM4/AP_RustM4_Placement.h (asserted below) and
    FLASH_RESERVE_END_KB in hwdef/CubeOrangePlus-SimOnHardWare-M4Rust. SRAM3
    (the shared window) is accessed by address, not linked.

    ram0 is privileged RAM (MPU region 2) and holds everything the image
    links: the stacks, the ChibiOS kernel, CubeFramework and the service.
    ram1 is the module area, open to unprivileged code; nothing is linked
    there.
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
    ram0   (wx) : org = 0x30000000, len = 32k       /* D2 SRAM1, privileged */
    ram1   (wx) : org = 0x30008000, len = 224k      /* rest of SRAM1 + SRAM2: the module area, placed at run time */
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

/* placement_*: from AP_RustM4_Placement.h, passed by the Makefile */
ASSERT(ORIGIN(flash0) == placement_flash_base && LENGTH(flash0) == placement_flash_size,
       "rust_m4.ld flash0 does not match AP_RustM4_Placement.h")
ASSERT(ORIGIN(ram0) == placement_ram_base && LENGTH(ram0) == placement_priv_ram_size &&
       ORIGIN(ram1) == ORIGIN(ram0) + LENGTH(ram0) && LENGTH(ram0) + LENGTH(ram1) == placement_ram_size,
       "rust_m4.ld RAM split does not match AP_RustM4_Placement.h")

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

INCLUDE rules_data.ld
INCLUDE rules_memory.ld

/* nothing is linked into the module area: the module host places stack, heap, RW and RX there */
__module_area_base__ = ORIGIN(ram1);
__module_area_end__ = ORIGIN(ram1) + LENGTH(ram1);
ASSERT(__heap_base__ == ORIGIN(ram1), "something is linked into the module area (ram1)")
/* __ram0_free__ is the first free address of ram0 */
ASSERT(ORIGIN(ram0) + LENGTH(ram0) - __ram0_free__ >= 4096, "privileged RAM has less than 4 KB free")
