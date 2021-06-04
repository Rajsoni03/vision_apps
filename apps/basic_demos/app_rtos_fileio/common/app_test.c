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

#include "ti_file_io.h"

MSG_Q g_msgQ
__attribute__ ((section(".bss:app_fileio_msg_mem")))
__attribute__ ((aligned(4096)))
        = {0};


void appMenuMain()
{
    uint32_t is_exit = 0;

    appCliShowBanner();

    while(!is_exit)
    {
        appCliShowPrompt(&is_exit);
    }
}

#ifdef ENABLE_UART_TEST
void appUartTest(void)
{
#define INPUT_BUF_LENGTH (128u)
    char buf[INPUT_BUF_LENGTH];

    UART_printf("Enter a string of data : ");
    UART_gets(buf, INPUT_BUF_LENGTH);
    UART_printf("Data received is [%s]\r\n", buf);
}
#endif

#ifdef ENABLE_RTOS_TEST
void appRtosTest()
{
    uint32_t count = 0;

    while (count < 10)
    {
        appLogPrintf (" %d: Core is UP!!! \n", count);
        count++;
        #ifdef CPU_mpu1
        Task_sleep(1U);
        #endif
        #ifdef CPU_mcu1_0
        Task_sleep(1U);
        #endif
        #ifdef CPU_mcu2_0
        Task_sleep(1U);
        #endif
        #ifdef CPU_mcu3_0
        Task_sleep(1U);
        #endif
        #ifdef CPU_c6x_1
        Task_sleep(1U);
        #endif
        #ifdef CPU_c6x_2
        Task_sleep(1U);
        #endif
        #ifdef CPU_c7x_1
        Task_sleep(1U);
        #endif
    }
}
#endif

#ifdef ENABLE_MEM_ALLOC_TEST
void appMemAllocTest()
{
    void *ptr;
    uint32_t size = 1024;

    ptr = appMemAlloc(APP_MEM_HEAP_DDR, size, 1);

    appLogPrintf(" Allocated memory @ 0x%08x of size %d bytes\r\n", ptr, size);

    appMemFree(APP_MEM_HEAP_DDR, ptr, size);

    appLogPrintf(" Free'ed memory @ 0x%08x of size %d bytes\r\n", ptr, size);
}
#endif

#if defined(ENABLE_MMC_SD_TEST) && defined(ENABLE_MMC_SD)

#define FS_APP_PATH_BUF_SIZE  (512U)
#define FS_APP_DATA_BUF_SIZE  (512U)
static char gFsAppFileName[FS_APP_PATH_BUF_SIZE];
static char gFsAppDataBuf[FS_APP_DATA_BUF_SIZE];

int32_t appFileIoTest()
{
    FILE *src=NULL, *dst=NULL;
    uint32_t bytesWrite = 0;
    uint32_t bytesRead = 0;
    uint32_t done = 0;
    int32_t retStat = 0;
    size_t bufSize = FS_APP_PATH_BUF_SIZE;
    uint32_t enableHostTest = 1;
    if(enableHostTest)
    {
        strcpy(gFsAppFileName, "test_rd.txt");
        /* Open the file for reading. */
        src = fopen(gFsAppFileName, "rb");
        /* If there was some problem opening the file, then return an error. */
        if(src == NULL)
        {
            appLogPrintf("Failed to open file %s for read !!!\n", gFsAppFileName);
            retStat = -1;
        }

        if(retStat == 0)
        {
            size_t fileSize;
            
            fseek(src, 0, SEEK_END);
            fileSize = ftell(src);
        
            appLogPrintf("Opened %s file of size %d bytes !!!\n", gFsAppFileName, fileSize);
        }

        fseek(src, 0, SEEK_SET);
        strcpy(gFsAppFileName, "test_wr.txt");
        dst = fopen(gFsAppFileName, "wb");
        if(dst == NULL)
        {
            appLogPrintf("Failed to open file %s for write !!!\n", gFsAppFileName);
            retStat = -1;
        }
        appLogPrintf("Opened %s file of for writing !!!\n", gFsAppFileName);

        if(retStat == 0)
        {
            /*
            * Enter a loop to repeatedly read data from the file and copy to destination file,
            * until the end of the file is reached.
            */
            do
            {
                /*
                * Read a block of data from the file.  Read as much as can fit in
                * temporary buffer, including a space for the trailing null.
                */
                bytesRead = fread(gFsAppDataBuf, 1, bufSize, src);
                appLogPrintf("Read %d bytes from Src!!!\n", bytesRead);
                
                /*
                * If there was an error reading, then print a newline and return
                * error to the user.
                */
                if(bytesRead != bufSize)
                {
                    done = 1;
                }
                
                if(bytesRead > 0)
                {
                    /*
                    * Write the data to the destination file user has selected.
                    * If there was an error writing, then print a newline and return
                    * error to the user.
                    */
                    bytesWrite = fwrite(gFsAppDataBuf, 1, bytesRead, dst);
                    if(bytesRead != bytesWrite)
                    {
                        appLogPrintf("Fail to write into file !!!!\n");
                        done = 1;
                        retStat = -1;
                    }
                    appLogPrintf("Wrote %d bytes from Dst!!!\n", bytesRead);
                }
                /*
                * Continue reading until less than the full number of bytes are
                * read. That means the end of the buffer was reached.
                */
            } while(!done);
        }
        appLogPrintf("Write complted!!!\n");
        /*
        * Close the files.
        */
        if(src!=NULL)
        {
            fclose(src);
        }
        if(dst!=NULL)
        {
            fclose(dst);
        }
    }
    retStat = fileIOServerInit(&g_msgQ);
    if(retStat == -1)
    {
        appLogPrintf("FILE IO Server init Failed!!!\n");
    }
    while(1)
    {
        /* appLogPrintf("FILE IO Server Run with %d msgs!!! %p %d \n", g_msgQ.numMsg, &g_msgQ.sync, g_msgQ.sync); */
        retStat = fileIOServerRun(&g_msgQ);
        if(retStat == -1)
        {
            appLogPrintf("FILE IO Server Run Failed!!!\n");
        }
       appLogWaitMsecs(1);
    } 

    return(retStat);
}

#endif

void appTestRun()
{
    appLogPrintf("APP: Run ... !!!\n");
    
    #ifdef ENABLE_MEM_ALLOC_TEST
    appMemAllocTest();
    #endif
    #ifdef ENABLE_RTOS_TEST
    appRtosTest();
    #endif
    #if defined(ENABLE_UART_TEST) && defined(ENABLE_UART)
    appUartTest();
    #endif
    #if defined(ENABLE_MMC_SD_TEST) && defined(ENABLE_MMC_SD)
    appFileIoTest();
    #endif
    appMenuMain();
    
    appLogPrintf("APP: Run ... Done !!!\n");
}


