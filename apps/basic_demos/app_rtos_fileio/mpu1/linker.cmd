/* linker options */
__STACK_SIZE = 0x10000;
__TI_STACK_SIZE = __STACK_SIZE;

REGION_ALIAS("REGION_TEXT", DDR_MPU1);
REGION_ALIAS("REGION_BSS", DDR_MPU1);
REGION_ALIAS("REGION_DATA", DDR_MPU1);
REGION_ALIAS("REGION_STACK", DDR_MPU1);
REGION_ALIAS("REGION_HEAP", DDR_MPU1);

SECTIONS {

    .vecs : {
        *(.vecs)
    } > MSMC_MPU1 AT> MSMC_MPU1

    .text : {
        CREATE_OBJECT_SYMBOLS
        *(.text)
        *(.text.*)
        . = ALIGN(0x8);
        KEEP (*(.ctors))
        . = ALIGN(0x4);
        KEEP (*(.dtors))
        . = ALIGN(0x8);
        __init_array_start = .;
        KEEP (*(.init_array*))
        __init_array_end = .;
        *(.init)
        *(.fini*)
    } > REGION_TEXT AT> REGION_TEXT

    PROVIDE (__etext = .);
    PROVIDE (_etext = .);
    PROVIDE (etext = .);

    .rodata : {
        *(.rodata)
        *(.rodata*)
    } > REGION_TEXT AT> REGION_TEXT

    .data : ALIGN (8) {
        __data_load__ = LOADADDR (.data);
        __data_start__ = .;
        *(.data)
        *(.data*)
        . = ALIGN (8);
        __data_end__ = .;
    } > REGION_DATA AT> REGION_TEXT

    .bss : {
        __bss_start__ = .;
        *(.shbss)
        *(.bss)
        *(.bss.*)
        . = ALIGN (8);
        __bss_end__ = .;
        . = ALIGN (8);
        *(COMMON)
    } > REGION_BSS AT> REGION_BSS

    .heap : {
        __heap_start__ = .;
        end = __heap_start__;
        _end = end;
        __end = end;
        KEEP(*(.heap))
        __heap_end__ = .;
        __HeapLimit = __heap_end__;
    } > REGION_HEAP AT> REGION_HEAP

    .stack (NOLOAD) : ALIGN(16) {
        _stack = .;
        __stack = .;
        KEEP(*(.stack))
    } > REGION_STACK AT> REGION_STACK

	__TI_STACK_BASE = __stack;
    
    .bss:taskStackSection : {} > DDR_MPU1

    .bss:app_log_mem        (NOLOAD) : {} > APP_LOG_MEM
    .bss:app_fileio_msg_mem          : {} > APP_FILIO_MEM
    .bss:ddr_shared_mem   (NOLOAD) : {} > DDR_MPU1
}
