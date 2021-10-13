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
#include <app_cfg_mpu1.h>
#include <utils/console_io/include/app_log.h>
#include <stdio.h>
#include <string.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

static Void appMain(UArg arg0, UArg arg1)
{
    //printf("Starting App init\n");
    appInit();
    //printf("Starting App Run\n");
    appTestRun();
    //printf("Going to App wait\n");
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


static uint8_t gTskStackMain[32*1024] 
__attribute__ ((section(".bss:taskStackSection")))
__attribute__ ((aligned(8192)))
    ;

int main(void)
{
    Task_Params tskParams;
    Error_Block eb;
    Task_Handle task;

    /* This is for debug purpose - see the description of function header */
    StartupEmulatorWaitFxn();

    Error_init(&eb);
    Task_Params_init(&tskParams);

    tskParams.arg0 = (UArg) NULL;
    tskParams.arg1 = (UArg) NULL;
    tskParams.priority = 8u;
    tskParams.stack = gTskStackMain;
    tskParams.stackSize = sizeof (gTskStackMain);
    task = Task_create(appMain, &tskParams, &eb);
    if(NULL == task)
    {
        BIOS_exit(0);
    }
    BIOS_start();

    return 0;
}

#include <ti/sysbios/family/arm/v8a/Mmu.h>

void InitMmu(void)
{
    Bool            retVal;
    Mmu_MapAttrs    attrs;

    Mmu_initMapAttrs(&attrs);

    attrs.attrIndx = 0;

    retVal = Mmu_map(0x00000000, 0x00000000, 0x20000000, &attrs);
    if(retVal==FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x20000000, 0x20000000, 0x20000000, &attrs);
    if(retVal==FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x40000000, 0x40000000, 0x20000000, &attrs);
    if(retVal==FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x60000000, 0x60000000, 0x10000000, &attrs);
    if(retVal==FALSE)
    {
        goto mmu_exit;
    }

    attrs.attrIndx = 7;

    retVal = Mmu_map(0x80000000, 0x80000000, 0x20000000, &attrs); /* ddr            */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0xA0000000, 0xA0000000, 0x20000000, &attrs); /* ddr            */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x70000000, 0x70000000, 0x10000000, &attrs); /* msmc        */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

mmu_exit:
    if(retVal == FALSE)
    {
         printf(" ERROR: MMU init failed (status = %d) !!!", retVal);
    }

    return;
}
#include <ti/sysbios/hal/Cache.h>

Void ti_sysbios_hal_Cache_wb__E(Ptr blockPtr, SizeT byteCnt, Bits16 type, Bool wait)
{
    ti_sysbios_hal_Cache_CacheProxy_wb__E(blockPtr, byteCnt, type, wait);
}

Void ti_sysbios_hal_Cache_wbInv__E(Ptr blockPtr, SizeT byteCnt, Bits16 type, Bool wait)
{
    ti_sysbios_hal_Cache_CacheProxy_wbInv__E(blockPtr, byteCnt, type, wait);
}

Void ti_sysbios_hal_Cache_inv__E(Ptr blockPtr, SizeT byteCnt, Bits16 type, Bool wait)
{
    ti_sysbios_hal_Cache_CacheProxy_inv__E(blockPtr, byteCnt, type, wait);
}
