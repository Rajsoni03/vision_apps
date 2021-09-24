/* linker options */
--fill_value=0
--stack_size=0x2000
--heap_size=0x1000

#define ATCM_START 0x00000000

-e __VECS_ENTRY_POINT

SECTIONS
{
    .vecs       : {
        __VECS_ENTRY_POINT = .;
    } palign(8) > R5F_TCMB0
    .init_text  : {
                     boot.*(.text)
                     *(.text:ti_sysbios_family_arm_MPU_*)
                     *(.text:ti_sysbios_family_arm_v7r_Cache_*)
                  }  > R5F_TCMB0
    .text:xdc_runtime_Startup_reset__I     : {} palign(8) > R5F_TCMB0
    .text       : {} palign(8)   > DDR_MCU1_1
    .cinit      : {} palign(8)   > DDR_MCU1_1
    .bss        : {} align(8)    > DDR_MCU1_1
    .const      : {} palign(8)   > DDR_MCU1_1
    .data       : {} palign(128) > DDR_MCU1_1
    .sysmem     : {} align(8)    > DDR_MCU1_1
    .stack      : {} align(4)    > DDR_MCU1_1
    .bss:taskStackSection > DDR_MCU1_1

}
