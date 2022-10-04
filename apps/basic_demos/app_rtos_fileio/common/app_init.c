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


#include APP_CFG_FILE
#include <app.h>
#include <stdio.h>
#include <string.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <utils/mem/include/app_mem.h>
#include <utils/mmc_sd/include/app_mmc_sd.h>
#include <utils/console_io/include/app_log.h>
#include <utils/console_io/include/app_cli.h>
#include <ti/board/board.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
#include <utils/perf_stats/include/app_perf_stats.h>

app_log_shared_mem_t g_app_log_shared_mem
__attribute__ ((section(".bss:app_log_mem")))
__attribute__ ((aligned(4096)))
        ;

uint8_t g_ddr_shared_mem[DDR_SHARED_MEM_SIZE]
__attribute__ ((section(".bss:ddr_shared_mem")))
__attribute__ ((aligned(4096)))
        ;
#ifdef ENABLE_UART
void appLogDeviceWrite(char *string, uint32_t max_size)
{
    UART_puts(string, max_size);
}
#endif

int32_t appInit()
{
    int32_t status = 0;
    app_mem_init_prm_t mem_init_prm;
    app_log_init_prm_t log_init_prm;
    app_mem_heap_prm_t *heap_prm;

    appMemInitPrmSetDefault(&mem_init_prm);
    appLogInitPrmSetDefault(&log_init_prm);

    heap_prm = &mem_init_prm.heap_info[APP_MEM_HEAP_DDR];
    heap_prm->base = g_ddr_shared_mem;
    strncpy(heap_prm->name, "DDR_SHARED_MEM", APP_MEM_HEAP_NAME_MAX);
    heap_prm->size = DDR_SHARED_MEM_SIZE;
    heap_prm->flags = APP_MEM_HEAP_FLAGS_IS_SHARED;

    log_init_prm.shared_mem = &g_app_log_shared_mem;
    log_init_prm.self_cpu_index = 0;
    #ifdef CPU_mpu1
    strncpy(log_init_prm.self_cpu_name, "MPU1" , APP_LOG_MAX_CPU_NAME);
    #endif
    #ifdef ENABLE_UART
    log_init_prm.log_rd_max_cpus = 2;
    log_init_prm.device_write = appLogDeviceWrite;
    #endif

    #ifdef ENABLE_UART
    {
        Board_initCfg boardCfg;

        boardCfg = BOARD_INIT_PINMUX_CONFIG |
               BOARD_INIT_UART_STDIO;
        Board_init(boardCfg);
    }
    #endif

    status = appLogWrInit(&log_init_prm);
    APP_ASSERT_SUCCESS(status);

    appLogPrintf("APP: Init ... !!!\n");

    #ifdef ENABLE_UART
    log_init_prm.log_rd_poll_interval_in_msecs = 1;
    status = appLogRdInit(&log_init_prm);
    APP_ASSERT_SUCCESS(status);

    {
        app_cli_init_prm_t cli_init_prm;

        appCliInitPrmSetDefault(&cli_init_prm);
        status = appCliInit(&cli_init_prm);
        APP_ASSERT_SUCCESS(status);
    }
    #endif

    status = appMemInit(&mem_init_prm);
    APP_ASSERT_SUCCESS(status);

    #ifdef ENABLE_MMC_SD
    status = appFatFsInit();
    APP_ASSERT_SUCCESS(status);
    #endif

    appLogPrintf("APP: Init ... Done !!!\n");
    appPerfStatsInit();

    return status;
}

void appDeInit()
{
    appLogPrintf("APP: Deinit ... !!!\n");

    #ifdef ENABLE_MMC_SD
    appFatFsDeInit();
    #endif
    appMemDeInit();
    appLogWrDeInit();
    #ifdef ENABLE_UART
    appLogRdDeInit();
    appCliDeInit();
    #endif

    appLogPrintf("APP: Deinit ... Done !!!\n");
}

void appIdleLoop(void)
{
    #if defined(__C7100__) || defined(__C7120__) || defined(__C7504__) || defined(_TMS320C6X)
   __asm(" IDLE");
   #endif
}
