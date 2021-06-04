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
    } palign(8) > ATCM_START
    .init_text  : {
                     boot.*(.text)
                     *(.text:ti_sysbios_family_arm_MPU_*)
                     *(.text:ti_sysbios_family_arm_v7r_Cache_*)
                  }  > R5F_TCMA
    .text:xdc_runtime_Startup_reset__I     : {} palign(8) > R5F_TCMA
    .text       : {} palign(8)   > DDR_MCU2_0
    .cinit      : {} palign(8)   > DDR_MCU2_0
    .bss        : {} align(8)    > DDR_MCU2_0
    .const      : {} palign(8)   > DDR_MCU2_0
    .data       : {} palign(128) > DDR_MCU2_0
    .sysmem     : {} align(8)    > DDR_MCU2_0
    .stack      : {} align(4)    > DDR_MCU2_0
    .bss:taskStackSection > DDR_MCU2_0

    .far:CPSW_DMA_DESC_MEMPOOL  (NOLOAD) {} ALIGN (128) > DDR_MCU2_0
    .far:CPSW_DMA_RING_MEMPOOL (NOLOAD) {} ALIGN (128) > DDR_MCU2_0
    .far:CPSW_DMA_PKT_MEMPOOL (NOLOAD) {} ALIGN (128) > DDR_MCU2_0
    .bss:NDK_MMBUFFER  (NOLOAD) {} ALIGN (128) > DDR_MCU2_0
    .bss:NDK_PACKETMEM (NOLOAD) {} ALIGN (128) > DDR_MCU2_0

    .resource_table : {
        __RESOURCE_TABLE = .;
    } > DDR_MCU2_0_RESOURCE_TABLE

    .bss:l3mem              (NOLOAD) : {} > MAIN_OCRAM_MCU2_0
    .bss:ddr_shared_mem     (NOLOAD) : {} > DDR_MCU2_0_LOCAL_HEAP
    .bss:ddr_non_cache_mem  (NOLOAD) : {} > DDR_MCU2_0_NON_CACHE
    .bss:app_log_mem        (NOLOAD) : {} > APP_LOG_MEM
    .bss:tiovx_obj_desc_mem (NOLOAD) : {} > TIOVX_OBJ_DESC_MEM
    .bss:ipc_vring_mem      (NOLOAD) : {} > IPC_VRING_MEM

    .pcie_queue_shmem (NOLOAD) : {} palign(8)  > PCIE_QUEUE_SHARED_MEM
}
