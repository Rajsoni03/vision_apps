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
#include <stdio.h>
#include <string.h>
#include <ti/osal/osal.h>
#include <ti/osal/TaskP.h>
#include <ti/sysbios/family/c66/Cache.h>
#include <ti/drv/sciclient/sciclient.h>
#include <app_ipc_rsctable.h>

void appTimerInterruptInit(void);
void appCacheMarInit(void);

void _system_post_cinit(void)
{
}

static void appMain(void* arg0, void* arg1)
{
    appInit();
    appTimerInterruptInit();
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

static uint8_t gTskStackMain[8*1024]
__attribute__ ((section(".bss:taskStackSection")))
__attribute__ ((aligned(8192)))
    ;

int main(void)
{
    TaskP_Params tskParams;
    TaskP_Handle task;

    /* This is for debug purpose - see the description of function header */
    StartupEmulatorWaitFxn();

    /* set cache, non-cache sections for C6x */
    appCacheMarInit();

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

void appCacheMarInit(void)
{
    /* enable cache for cached sections */
    Cache_setMar((Ptr)DDR_C66x_2_DTS_ADDR, DDR_C66x_2_DTS_SIZE, Cache_Mar_ENABLE);
    Cache_setMar((Ptr)DDR_C66X_2_LOCAL_HEAP_ADDR, DDR_C66X_2_LOCAL_HEAP_SIZE, Cache_Mar_ENABLE);
    Cache_setMar((Ptr)DDR_C66X_2_SCRATCH_ADDR, DDR_C66X_2_SCRATCH_SIZE, Cache_Mar_ENABLE);
    Cache_setMar((Ptr)DDR_SHARED_MEM_ADDR, DDR_SHARED_MEM_SIZE, Cache_Mar_ENABLE);

    /* disable cache for non-cached sections */
    Cache_setMar((Ptr)DDR_C66x_2_IPC_ADDR, DDR_C66x_2_IPC_SIZE, Cache_Mar_DISABLE);
    Cache_setMar((Ptr)APP_LOG_MEM_ADDR, APP_LOG_MEM_SIZE, Cache_Mar_DISABLE);
    Cache_setMar((Ptr)TIOVX_OBJ_DESC_MEM_ADDR, TIOVX_OBJ_DESC_MEM_SIZE, Cache_Mar_DISABLE);
    Cache_setMar((Ptr)IPC_VRING_MEM_ADDR, IPC_VRING_MEM_SIZE, Cache_Mar_DISABLE);
    Cache_setMar((Ptr)TIOVX_LOG_RT_MEM_ADDR, TIOVX_LOG_RT_MEM_SIZE, Cache_Mar_DISABLE);
}

/* To set C66 timer interrupts on J7ES VLAB */
void appTimerInterruptInit(void)
{
#if defined (_TMS320C6X)
    int32_t status = 0;

    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;

    /* On C66x builds we define OS timer tick in the configuration file to
     * trigger event #21 for C66x_1 and #20 for C66x_2. Map
     * DMTimer 0 interrupt to these events through DMSC RM API.
     */
    rmIrqReq.valid_params           = TISCI_MSG_VALUE_RM_DST_ID_VALID |
                                      TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    rmIrqReq.src_id                 = TISCI_DEV_TIMER0;
    rmIrqReq.src_index              = 0U;
    rmIrqReq.dst_id                 = TISCI_DEV_C66SS1_CORE0;
    rmIrqReq.dst_host_irq           = 20U;
    /* Unused params */
    rmIrqReq.global_event           = 0U;
    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;
    rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    status = Sciclient_rmIrqSet(&rmIrqReq, &rmIrqResp, SCICLIENT_SERVICE_WAIT_FOREVER);
    if(status != 0)
    {
        appLogPrintf(" ERROR: failed to setup timer interrupt !!!\n" );
    }
#endif

    return;
}

