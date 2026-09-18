ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_stack_end = 0x20030000;    /* end of RAM */
/* Generate a link error if heap and stack don't fit into RAM */
_stack_size = 0x400; /* required amount of stack */

/* Base of the CPU2 radio firmware, and the ceiling for everything CPU1 owns.
 * The hard limit at runtime is the SFSA option byte, which FUS sets from the
 * installed stack; furi_hal_flash_get_free_end_address reads SFSA directly, so
 * internal storage is already correct. This constant only exists so the link
 * fails when the image itself would reach the radio, which a FLASH region of
 * the full 1024K never catches.
 *
 * 0x080E0000 is where the BLE HCILayer stack loads, which is what this fork
 * runs. A stock BLE Stack light device would be 0x080D7000 instead.
 * scripts/flipper/assets/coprobin.py prints the address for any stack. */
__radio_base__ = 0x080E0000;

MEMORY {
    /* 896K, not the full 1024K: the region stops at __radio_base__. An image
     * that would reach the radio then overflows the region at link time, and
     * the free space reported after a build is what CPU1 can really use. */
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 896K
    RAM1  (xrw) : ORIGIN = 0x20000008, LENGTH = 0x2FFF8
    RAM2A (xrw) : ORIGIN = 0x20030000, LENGTH = 10K
    RAM2B (xrw) : ORIGIN = 0x20038000, LENGTH = 10K
}

SECTIONS {
    /* The startup code goes first into FLASH */
    .isr_vector : {
        . = ALIGN(4);
        KEEP(*(.isr_vector)) /* Startup code */
        . = ALIGN(4);
    } >FLASH

    /* The program code and other data goes into FLASH */
    .text : {
        . = ALIGN(4);
        *lib*.a:*(.text .text.*) /* code from libraries before apps */
        *(.text)           /* .text sections (code) */
        *(.text*)          /* .text* sections (code) */
        *(.glue_7)         /* glue arm to thumb code */
        *(.glue_7t)        /* glue thumb to arm code */
        *(.eh_frame)

        KEEP (*(.init))
        KEEP (*(.fini))

        . = ALIGN(4);
        _etext = .;        /* define a global symbols at end of code */
    } >FLASH

    /* Constant data goes into FLASH */
    .rodata : {
        . = ALIGN(4);
        *(.rodata)         /* .rodata sections (constants, strings, etc.) */
        *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
        . = ALIGN(4);
    } >FLASH

    .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
    .ARM : {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >FLASH

    .preinit_array : {
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
    } >FLASH
    .init_array : {
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
    } >FLASH
    .fini_array : {
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
    } >FLASH

    /* used by the startup to initialize data */
    _sidata = LOADADDR(.data);

    /* Initialized data sections goes into RAM, load LMA copy after code */
    .data : {
        . = ALIGN(4);
        _sdata = .;        /* create a global symbol at data start */
        *(.data)           /* .data sections */
        *(.data*)          /* .data* sections */
        *(*_DRIVER_CONTEXT)

        . = ALIGN(4);
        _edata = .;        /* define a global symbol at data end */
    } >RAM1 AT> FLASH

    /* NimBLE's HCI transport pools (TASK-785). They are held for the life of
     * the host, so they live in the SRAM2B the radio core leaves free instead of
     * this core's RAM. They sit before __sram2b_start__, which is where
     * furi_hal_memory starts its runtime pool, so the two cannot overlap. */
    .sram2b_pools (NOLOAD)   :
    {
        *(.bss.pool_acl_buf)
        *(.bss.pool_evt_lo_buf)
        *(.bss.pool_evt_buf)
        *(.bss.pool_cmd_buf)
        . = ALIGN(4);
    } >RAM2B

    /* Uninitialized data section */
    . = ALIGN(4);
    .bss : {
        /* This is used by the startup in order to initialize the .bss secion */
        _sbss = .;         /* define a global symbol at bss start */
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)

        . = ALIGN(4);
        _ebss = .;         /* define a global symbol at bss end */
        __bss_end__ = _ebss;
    } >RAM1

    /* User_heap_stack section, used to check that there is enough RAM left */
    /* DSECT: https://software-dl.ti.com/ccs/esd/documents/sdto_cgt_linker_special_section_types.html */
    ._user_heap_stack(DSECT) : {
        . = ALIGN(8);
        __heap_start__ = .;
        . = ORIGIN(RAM1) + LENGTH(RAM1) - _stack_size;
        __heap_end__ = .;
        . = . + _stack_size;
        . = ALIGN(8);
    } >RAM1

    /* Free Flash space, that can be used for internal storage. It stops at the
     * radio rather than the end of flash, so the size reported after a build is
     * what CPU1 can really still use. */
    .free_flash(DSECT) : {
        __free_flash_start__ = .;
        . = ORIGIN(FLASH) + LENGTH(FLASH);
    } >FLASH

    /* Remove information from the standard libraries */
    /DISCARD/ : {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }

    .ARM.attributes 0       : { *(.ARM.attributes) }
    ._sram2a_start          : { . = ALIGN(4); __sram2a_start__ = .; } >RAM2A
    MAPPING_TABLE (NOLOAD)  : { *(MAPPING_TABLE) } >RAM2A
    MB_MEM1 (NOLOAD)        : { *(MB_MEM1) } >RAM2A
    MB_MEM2 (NOLOAD)        : { _sMB_MEM2 = . ; *(MB_MEM2) ; _eMB_MEM2 = . ; } >RAM2A
    ._sram2a_free           : { . = ALIGN(4); __sram2a_free__ = .; } >RAM2A
    ._sram2b_start          : { . = ALIGN(4); __sram2b_start__ = .; } >RAM2B

    /* Default debug-related rules from ld */

    /* Stabs debugging sections.  */
    .stab          0 : { *(.stab) }
    .stabstr       0 : { *(.stabstr) }
    .stab.excl     0 : { *(.stab.excl) }
    .stab.exclstr  0 : { *(.stab.exclstr) }
    .stab.index    0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment       0 : { *(.comment) }
    .gnu.build.attributes : { *(.gnu.build.attributes .gnu.build.attributes.*) }
    /* DWARF debug sections.
        Symbols in the DWARF debugging sections are relative to the beginning
        of the section so we begin them at 0.  */
    /* DWARF 1.  */
    .debug          0 : { *(.debug) }
    .line           0 : { *(.line) }
    /* GNU DWARF 1 extensions.  */
    .debug_srcinfo  0 : { *(.debug_srcinfo) }
    .debug_sfnames  0 : { *(.debug_sfnames) }
    /* DWARF 1.1 and DWARF 2.  */
    .debug_aranges  0 : { *(.debug_aranges) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    /* DWARF 2.  */
    .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
    .debug_abbrev   0 : { *(.debug_abbrev) }
    .debug_line     0 : { *(.debug_line .debug_line.* .debug_line_end) }
    .debug_frame    0 : { *(.debug_frame) }
    .debug_str      0 : { *(.debug_str) }
    .debug_loc      0 : { *(.debug_loc) }
    .debug_macinfo  0 : { *(.debug_macinfo) }
    /* SGI/MIPS DWARF 2 extensions.  */
    .debug_weaknames 0 : { *(.debug_weaknames) }
    .debug_funcnames 0 : { *(.debug_funcnames) }
    .debug_typenames 0 : { *(.debug_typenames) }
    .debug_varnames  0 : { *(.debug_varnames) }
    /* DWARF 3.  */
    .debug_pubtypes 0 : { *(.debug_pubtypes) }
    .debug_ranges   0 : { *(.debug_ranges) }
    /* DWARF 5.  */
    .debug_addr     0 : { *(.debug_addr) }
    .debug_line_str 0 : { *(.debug_line_str) }
    .debug_loclists 0 : { *(.debug_loclists) }
    .debug_macro    0 : { *(.debug_macro) }
    .debug_names    0 : { *(.debug_names) }
    .debug_rnglists 0 : { *(.debug_rnglists) }
    .debug_str_offsets 0 : { *(.debug_str_offsets) }
    .debug_sup      0 : { *(.debug_sup) }
    .note.gnu.arm.ident 0 : { KEEP (*(.note.gnu.arm.ident)) }
    /DISCARD/ : { *(.note.GNU-stack) *(.gnu_debuglink) *(.gnu.lto_*) }
}

ASSERT(
    __free_flash_start__ <= __radio_base__,
    "Firmware image reaches the CPU2 radio at __radio_base__ (0x080E0000). Shrink the image; flashing it would corrupt the radio."
)
