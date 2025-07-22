/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright 2008-2013 Code Red Technologies Ltd,
 * Copyright 2013-2025 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 * Generated linker script file for MCXE247
 * Created from linkscript.ldt by FMCreateLinkLibraries
 * Using Freemarker v2.3.30
 * MCUXpresso IDE v25.6 [Build 136] [2025-06-27] on Jul 21, 2025, 10:09:04 AM
 */

INCLUDE "an-mc-pmsm-mcxe247_Debug_library.ld"
INCLUDE "an-mc-pmsm-mcxe247_Debug_memory.ld"

ENTRY(ResetISR)

SECTIONS
{
     .text_Flash2 : ALIGN(8)
    {
       FILL(0xff)
        *(.text_Flash2) /* for compatibility with previous releases */
        *(.text_DATA_FLASH) /* for compatibility with previous releases */
        *(.text.$Flash2)
        *(.text.$DATA_FLASH)
        *(.text_Flash2.*) /* for compatibility with previous releases */
        *(.text_DATA_FLASH.*) /* for compatibility with previous releases */
        *(.text.$Flash2.*)
        *(.text.$DATA_FLASH.*)
        *(.rodata.$Flash2)
        *(.rodata.$DATA_FLASH)
        *(.rodata.$Flash2.*)
        *(.rodata.$DATA_FLASH.*)            } > DATA_FLASH

    /* MAIN TEXT SECTION */
    .text : ALIGN(8)
    {
        FILL(0xff)
        __vectors_start__ = ABSOLUTE(.) ;
        KEEP(*(.isr_vector))
        /* Global Section Table */
        . = ALIGN(4) ;
        __section_table_start = .;
        __data_section_table = .;
        LONG(LOADADDR(.data));
        LONG(    ADDR(.data));
        LONG(  SIZEOF(.data));
        LONG(LOADADDR(.data_RAM2));
        LONG(    ADDR(.data_RAM2));
        LONG(  SIZEOF(.data_RAM2));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
        LONG(    ADDR(.bss_RAM2));
        LONG(  SIZEOF(.bss_RAM2));
        __bss_section_table_end = .;
        __section_table_end = . ;
        /* End of Global Section Table */

        *(.after_vectors*)

        /* Kinetis Flash Configuration data */
        . = 0x400 ;
        PROVIDE(__FLASH_CONFIG_START__ = .) ;
        KEEP(*(.FlashConfig))
        PROVIDE(__FLASH_CONFIG_END__ = .) ;
        ASSERT(!(__FLASH_CONFIG_START__ == __FLASH_CONFIG_END__), "Linker Flash Config Support Enabled, but no .FlashConfig section provided within application");
        /* End of Kinetis Flash Configuration data */
        
       *(.text*)
       *(.rodata .rodata.* .constdata .constdata.*)
       . = ALIGN(8);
    } > PROGRAM_FLASH
    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this.
     */
    .ARM.extab : ALIGN(8)
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > PROGRAM_FLASH

    .ARM.exidx : ALIGN(8)
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > PROGRAM_FLASH
 
    _etext = .;
        
    /* possible MTB section for SRAM_UPPER */
    .mtb_buffer_RAM2 (NOLOAD) :
    {
        KEEP(*(.mtb.$RAM2*))
        KEEP(*(.mtb.$SRAM_UPPER*))
    } > SRAM_UPPER

    /* DATA section for SRAM_UPPER */

    .data_RAM2 : ALIGN(8)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM2 = .) ;
        PROVIDE(__start_data_SRAM_UPPER = .) ;
        *(.ramfunc.$RAM2)
        *(.ramfunc.$SRAM_UPPER)
       *(NonCacheable.init)
        *(.data.$RAM2)
        *(.data.$SRAM_UPPER)
        *(.data.$RAM2.*)
        *(.data.$SRAM_UPPER.*)
        . = ALIGN(8) ;
        PROVIDE(__end_data_RAM2 = .) ;
        PROVIDE(__end_data_SRAM_UPPER = .) ;
     } > SRAM_UPPER AT>PROGRAM_FLASH

    /* MAIN DATA SECTION */
    /* Default MTB section */
    .mtb_buffer_default (NOLOAD) :
    {
        KEEP(*(.mtb*))
    } > SRAM_LOWER AT > SRAM_LOWER
    .uninit_RESERVED (NOLOAD) : ALIGN(8)
    {
        _start_uninit_RESERVED = .;
        KEEP(*(.bss.$RESERVED*))
       . = ALIGN(8) ;
        _end_uninit_RESERVED = .;
    } > SRAM_LOWER AT> SRAM_LOWER

    /* Main DATA section (SRAM_LOWER) */
    .data : ALIGN(8)
    {
       FILL(0xff)
       _data = . ;
       PROVIDE(__start_data_RAM = .) ;
       PROVIDE(__start_data_SRAM_LOWER = .) ;
       *(vtable)
       *(.ramfunc*)
       KEEP(*(CodeQuickAccess))
       KEEP(*(DataQuickAccess))
       *(RamFunction)
       *(.data*)
       . = ALIGN(8) ;
       _edata = . ;
       PROVIDE(__end_data_RAM = .) ;
       PROVIDE(__end_data_SRAM_LOWER = .) ;
    } > SRAM_LOWER AT>PROGRAM_FLASH

    /* BSS section for SRAM_UPPER */
    .bss_RAM2 (NOLOAD) : ALIGN(8)
    {
       PROVIDE(__start_bss_RAM2 = .) ;
       PROVIDE(__start_bss_SRAM_UPPER = .) ;
       *(NonCacheable)
       *(.bss.$RAM2)
       *(.bss.$SRAM_UPPER)
       *(.bss.$RAM2.*)
       *(.bss.$SRAM_UPPER.*)
       . = ALIGN (. != 0 ? 8 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM2 = .) ;
       PROVIDE(__end_bss_SRAM_UPPER = .) ;
    } > SRAM_UPPER AT> SRAM_UPPER

    /* MAIN BSS SECTION */
    .bss (NOLOAD) : ALIGN(8)
    {
        _bss = .;
        PROVIDE(__start_bss_RAM = .) ;
        PROVIDE(__start_bss_SRAM_LOWER = .) ;
        *(.bss*)
        *(COMMON)
        . = ALIGN(8) ;
        _ebss = .;
        PROVIDE(__end_bss_RAM = .) ;
        PROVIDE(__end_bss_SRAM_LOWER = .) ;
        PROVIDE(end = .);
    } > SRAM_LOWER AT> SRAM_LOWER

    /* NOINIT section for SRAM_UPPER */
    .noinit_RAM2 (NOLOAD) : ALIGN(8)
    {
       PROVIDE(__start_noinit_RAM2 = .) ;
       PROVIDE(__start_noinit_SRAM_UPPER = .) ;
       *(.noinit.$RAM2)
       *(.noinit.$SRAM_UPPER)
       *(.noinit.$RAM2.*)
       *(.noinit.$SRAM_UPPER.*)
       . = ALIGN(8) ;
       PROVIDE(__end_noinit_RAM2 = .) ;
       PROVIDE(__end_noinit_SRAM_UPPER = .) ;
    } > SRAM_UPPER AT> SRAM_UPPER

    /* DEFAULT NOINIT SECTION */
    .noinit (NOLOAD): ALIGN(8)
    {
        _noinit = .;
        PROVIDE(__start_noinit_RAM = .) ;
        PROVIDE(__start_noinit_SRAM_LOWER = .) ;
        *(.noinit*)
         . = ALIGN(8) ;
        _end_noinit = .;
       PROVIDE(__end_noinit_RAM = .) ;
       PROVIDE(__end_noinit_SRAM_LOWER = .) ;        
    } > SRAM_LOWER AT> SRAM_LOWER

    /* Reserve and place Heap within memory map */
    _HeapSize = 0x100;
    .heap (NOLOAD) :  ALIGN(8)
    {
        _pvHeapStart = .;
        . += _HeapSize;
        . = ALIGN(8);
        _pvHeapLimit = .;
    } > SRAM_LOWER

     _StackSize = 0x100;
     /* Reserve space in memory for Stack */
    .heap2stackfill (NOLOAD) :
    {
        . += _StackSize;
    } > SRAM_LOWER
    /* Locate actual Stack in memory map */
    .stack ORIGIN(SRAM_LOWER) + LENGTH(SRAM_LOWER) - _StackSize - 0 (NOLOAD) :  ALIGN(8)
    {
        _vStackBase = .;
        . = ALIGN(8);
        _vStackTop = . + _StackSize;
    } > SRAM_LOWER

    /* Provide basic symbols giving location and size of main text
     * block, including initial values of RW data sections. Note that
     * these will need extending to give a complete picture with
     * complex images (e.g multiple Flash banks).
     */
    _image_start = LOADADDR(.text);
    _image_end = LOADADDR(.data) + SIZEOF(.data);
    _image_size = _image_end - _image_start;
}