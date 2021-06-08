/*
 *
 * Copyright (c) 2018 Texas Instruments Incorporated
 *
 * All rights reserved not granted herein.
 *
 * Limited License.
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive
 * license under copyrights and patents it now or hereafter owns or controls to make,
 * have made, use, import, offer to sell and sell ("Utilize") this software subject to the
 * terms herein.  With respect to the foregoing patent license, such license is granted
 * solely to the extent that any such patent is necessary to Utilize the software alone.
 * The patent license shall not apply to any combinations which include this software,
 * other than combinations with devices manufactured by or for TI ("TI Devices").
 * No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce this license
 * (including the above copyright notice and the disclaimer and (if applicable) source
 * code license limitations below) in the documentation and/or other materials provided
 * with the distribution
 *
 * Redistribution and use in binary form, without modification, are permitted provided
 * that the following conditions are met:
 *
 * *       No reverse engineering, decompilation, or disassembly of this software is
 * permitted with respect to any software provided in binary form.
 *
 * *       any redistribution and use are licensed by TI for use only with TI Devices.
 *
 * *       Nothing shall obligate TI to provide you with source code for the software
 * licensed and provided to you in object code.
 *
 * If software source code is provided to you, modification and redistribution of the
 * source code are permitted provided that the following conditions are met:
 *
 * *       any redistribution and use of the source code, including any resulting derivative
 * works, are licensed by TI for use only with TI Devices.
 *
 * *       any redistribution and use of any object code compiled from the source code
 * and any resulting derivative works, are licensed by TI for use only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its suppliers
 *
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <app.h>
#include <utils/console_io/include/app_log.h>
#include <utils/misc/include/app_misc.h>
#include <stdio.h>
#include <string.h>
#include <ti/osal/osal.h>
#include <ti/osal/TaskP.h>
#include <ti/sysbios/family/c7x/Hwi.h>
#include <ti/sysbios/family/c7x/Cache.h>
#include <app_mem_map.h>
#include <app_ipc_rsctable.h>
#include <ti/csl/soc.h>
#include <ti/csl/csl_clec.h>

/* For J7ES/J721E/TDA4VM the upper 2GB DDR starts from 0x0008_8000_0000 */
/* This address is mapped to a virtual address of 0x0001_0000_0000 */
#define DDR_C7X_1_LOCAL_HEAP_VADDR (DDR_C7X_1_LOCAL_HEAP_ADDR)
#define DDR_C7X_1_LOCAL_HEAP_PADDR (0x880000000u)

static void appMain(void* arg0, void* arg1)
{
    appInit();
    appRun();
    #if 1
    while(1)
    {
        appLogWaitMsecs(100u);
    }
    #else
    appDeInit();
    #endif
}

void StartupEmulatorWaitFxn (void)
{
    volatile uint32_t enableDebug = 0;
    do
    {
    }while (enableDebug);
}

/* To set C71 timer interrupts */
void appTimerInterruptInit(void)
{
    CSL_ClecEventConfig   cfgClec;
    CSL_CLEC_EVTRegs     *clecBaseAddr = (CSL_CLEC_EVTRegs*)CSL_COMPUTE_CLUSTER0_CLEC_REGS_BASE;

    uint32_t input         = 1249; /* Used for Timer Interrupt */
    uint32_t corepackEvent = 15;

    /* Configure CLEC */
    cfgClec.secureClaimEnable = FALSE;
    cfgClec.evtSendEnable     = TRUE;
    cfgClec.rtMap             = CSL_CLEC_RTMAP_CPU_ALL;
    cfgClec.extEvtNum         = 0;
    cfgClec.c7xEvtNum         = corepackEvent;
    CSL_clecConfigEvent(clecBaseAddr, input, &cfgClec);
    CSL_clecConfigEventLevel(clecBaseAddr, input, 0); /* configure interrupt as pulse */
    Hwi_setPriority(corepackEvent, 1);
}

/* IMPORTANT NOTE: For C7x,
 * - stack size and stack ptr MUST be 8KB aligned
 * - AND min stack size MUST be 16KB
 * - AND stack assigned for task context is "size - 8KB"
 *       - 8KB chunk for the stack area is used for interrupt handling in this task context
 */
static uint8_t gTskStackMain[64*1024]
__attribute__ ((section(".bss:taskStackSection")))
__attribute__ ((aligned(8192)))
    ;

/* DRU configuration */
#define J721E_DDR_QOS_EXP_DRU_QUEUE_PRIORITY   (2)
#define J721E_DDR_QOS_EXP_DRU_QUEUE_ORDER_ID   (4)
#define J7ES_DRU_NUM_CH (5)
#define J7ES_DRU_CFG_y(i) (0x6D008000 + ((i) * 8))
#define writel(x,y) (*((uint32_t *)(y))=(x))

void setup_dru_qos(void)
{
	unsigned int channel;

	for (channel = 0; channel < J7ES_DRU_NUM_CH; ++channel) 
	{
		writel((J721E_DDR_QOS_EXP_DRU_QUEUE_ORDER_ID << 4) | J721E_DDR_QOS_EXP_DRU_QUEUE_PRIORITY, J7ES_DRU_CFG_y(channel));
	}
}

int main(void)
{
    TaskP_Params tskParams;
    TaskP_Handle task;

    setup_dru_qos();

    appTimerInterruptInit();

    OS_init();

    TaskP_Params_init(&tskParams);
    tskParams.priority = 8u;
    tskParams.stack = gTskStackMain;
    tskParams.stacksize = sizeof (gTskStackMain);
    task = TaskP_create(appMain, &tskParams);
    if(NULL == task)
    {
        OS_stop();
    }
    OS_start();

    return 0;
}

#include <ti/sysbios/family/c7x/Mmu.h>

uint32_t g_app_rtos_c7x_mmu_map_error = 0;

void appMmuMap(Bool is_secure)
{
    Bool            retVal;
    Mmu_MapAttrs    attrs;

    uint32_t ns = 1;

    if(is_secure)
        ns = 0;
    else
        ns = 1;

    Mmu_initMapAttrs(&attrs);

    attrs.attrIndx = Mmu_AttrIndx_MAIR0;
    attrs.ns = ns;

    retVal = Mmu_map(0x00000000, 0x00000000, 0x20000000, &attrs, is_secure);
    if(retVal==FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x20000000, 0x20000000, 0x20000000, &attrs, is_secure);
    if(retVal==FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x40000000, 0x40000000, 0x20000000, &attrs, is_secure);
    if(retVal==FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x60000000, 0x60000000, 0x10000000, &attrs, is_secure);
    if(retVal==FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x70000000, 0x70000000, 0x10000000, &attrs, is_secure);
    if(retVal==FALSE)
    {
        goto mmu_exit;
    }

    Mmu_initMapAttrs(&attrs);

    attrs.attrIndx = Mmu_AttrIndx_MAIR4;
    attrs.ns = ns;

    retVal = Mmu_map(APP_LOG_MEM_ADDR, APP_LOG_MEM_ADDR, APP_LOG_MEM_SIZE, &attrs, is_secure);
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(TIOVX_OBJ_DESC_MEM_ADDR, TIOVX_OBJ_DESC_MEM_ADDR, TIOVX_OBJ_DESC_MEM_SIZE, &attrs, is_secure);
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(IPC_VRING_MEM_ADDR, IPC_VRING_MEM_ADDR, IPC_VRING_MEM_SIZE, &attrs, is_secure);
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(DDR_C7x_1_IPC_ADDR, DDR_C7x_1_IPC_ADDR, DDR_C7x_1_IPC_SIZE, &attrs, is_secure); /* ddr            */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(TIOVX_LOG_RT_MEM_ADDR, TIOVX_LOG_RT_MEM_ADDR, TIOVX_LOG_RT_MEM_SIZE, &attrs, is_secure); 
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    Mmu_initMapAttrs(&attrs);
    attrs.attrIndx = Mmu_AttrIndx_MAIR7;
    attrs.ns = ns;
    retVal = Mmu_map(0x70000000, 0x70000000, 0x00800000, &attrs, is_secure); /* msmc  448KB      */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    Mmu_initMapAttrs(&attrs);
    attrs.attrIndx = Mmu_AttrIndx_MAIR7;
    attrs.ns = ns;
    retVal = Mmu_map(0x64800000, 0x64800000, 0x00200000, &attrs, is_secure); /* L2 sram 448KB        */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    #if 0
    Mmu_initMapAttrs(&attrs);
    attrs.attrIndx = Mmu_AttrIndx_MAIR0;
    attrs.ns = ns;
    retVal = Mmu_map(0x64E00000, 0x64E00000, 0x00200000, &attrs, is_secure); /* L1D sram 16KB        */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }
    #endif

    Mmu_initMapAttrs(&attrs);
    attrs.attrIndx = Mmu_AttrIndx_MAIR7;
    attrs.ns = ns;

    retVal = Mmu_map(DDR_C7x_1_DTS_ADDR, DDR_C7x_1_DTS_ADDR, DDR_C7x_1_DTS_SIZE, &attrs, is_secure); /* ddr            */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(DDR_C7X_1_LOCAL_HEAP_VADDR, DDR_C7X_1_LOCAL_HEAP_PADDR, DDR_C7X_1_LOCAL_HEAP_SIZE, &attrs, is_secure); /* ddr            */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(DDR_C7X_1_SCRATCH_ADDR, DDR_C7X_1_SCRATCH_ADDR, DDR_C7X_1_SCRATCH_SIZE, &attrs, is_secure); /* ddr            */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(DDR_SHARED_MEM_ADDR, DDR_SHARED_MEM_ADDR, DDR_SHARED_MEM_SIZE, &attrs, is_secure); /* ddr            */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

mmu_exit:
    if(retVal == FALSE)
    {
        g_app_rtos_c7x_mmu_map_error++;
    }



    return;
}

void appCacheInit()
{
    ti_sysbios_family_c7x_Cache_Size  cacheSize;
    
    /* init cache size here, since this needs to be done in secure mode */
    cacheSize.l1pSize = ti_sysbios_family_c7x_Cache_L1Size_32K;
    cacheSize.l1dSize = ti_sysbios_family_c7x_Cache_L1Size_32K;
    cacheSize.l2Size  = ti_sysbios_family_c7x_Cache_L2Size_64K;
    Cache_setSize(&cacheSize);
}

void appMmuInit(void)
{
    /* This is for debug purpose - see the description of function header */
    StartupEmulatorWaitFxn();

    g_app_rtos_c7x_mmu_map_error = 0;

    appC7xClecInitForNonSecAccess();

    appMmuMap(FALSE);
    appMmuMap(TRUE);
    
    appCacheInit();
}

/* Offset to be added to convert virutal address to physical address */
#define VIRT_PHY_ADDR_OFFSET (DDR_C7X_1_LOCAL_HEAP_PADDR - DDR_C7X_1_LOCAL_HEAP_VADDR)

uint64_t appUdmaVirtToPhyAddrConversion(const void *virtAddr,
                                      uint32_t chNum,
                                      void *appData)
{
  uint64_t phyAddr = (uint64_t)virtAddr;

  if ((uint64_t)virtAddr >= DDR_C7X_1_LOCAL_HEAP_VADDR)
  {
    phyAddr = ((uint64_t)virtAddr + VIRT_PHY_ADDR_OFFSET);
  }

  return phyAddr;
}
