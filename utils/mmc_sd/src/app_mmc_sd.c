/*
 *
 * Copyright (c) 2017 Texas Instruments Incorporated
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

#include <stdio.h>
#include <string.h>
#include <utils/mmc_sd/include/app_mmc_sd.h>
#include <utils/console_io/include/app_log.h>

#include <ti/board/board.h>
#include <ti/drv/mmcsd/MMCSD.h>
#include <ti/fs/fatfs/FATFS.h>
#include <ti/fs/fatfs/ff.h>

#include <ti/osal/SemaphoreP.h>


#define MMCSD_INSTANCE_MMCSD    (0U)

/* #define APP_MMC_SD_DEBUG */

/* MMCSD function table for MMCSD implementation */
FATFS_DrvFxnTable FATFS_drvFxnTable = {
    MMCSD_close,
    MMCSD_control,
    MMCSD_init,
    MMCSD_open,
    MMCSD_write,
    MMCSD_read
};

/* FATFS configuration structure */
FATFS_HwAttrs FATFS_initCfg[_VOLUMES] =
{
    {  /* MMC1 is SD card for J7 */
        1U
    },
    {
        1U
    },
    {
        2U
    },
    {
        3U
    }
};

/* FATFS objects */
FATFS_Object FATFS_objects[_VOLUMES];

/* FATFS configuration structure */
const FATFSConfigList FATFS_config = {
    {
        &FATFS_drvFxnTable,
        &FATFS_objects[0],
        &FATFS_initCfg[0]
    },

    {
         &FATFS_drvFxnTable,
         &FATFS_objects[1],
         &FATFS_initCfg[1]
    },

    {
         &FATFS_drvFxnTable,
         &FATFS_objects[2],
         &FATFS_initCfg[2]
    },

    {NULL, NULL, NULL},

    {NULL, NULL, NULL}
};

typedef struct {
    MMCSD_Handle handle;
    FATFS_Handle fatfsHandle;
    SemaphoreP_Handle lock;
} app_mmcsd_obj_t;

static app_mmcsd_obj_t g_app_mmcsd_obj;

int32_t appFatFsInit(void)
{
    int32_t status = 0;
    app_mmcsd_obj_t *obj = &g_app_mmcsd_obj;
    FATFS_Error fatfs_ret;

    appLogPrintf("FATFS: Init ... !!!\n");

    obj->fatfsHandle = NULL;
    obj->lock = NULL;

    /* MMCSD FATFS initialization/mount device */
    fatfs_ret = FATFS_init();
    if(fatfs_ret == FATFS_ERR) {
        appLogPrintf("FATFS: Init failed!! \n");
        status = -1;
    }

    if(0 == status)
    {
        fatfs_ret = FATFS_open(MMCSD_INSTANCE_MMCSD, NULL, &obj->fatfsHandle);
        if(fatfs_ret == FATFS_ERR) {
            appLogPrintf("FATFS: Open failed!! \n");
            status = -1;
        }
    }

    if(0 == status)
    {
        SemaphoreP_Params semParams;

        /* Default parameter initialization */
        SemaphoreP_Params_init(&semParams);

        semParams.mode = SemaphoreP_Mode_BINARY;

        obj->lock = SemaphoreP_create(1U, &semParams);

        if(obj->lock==NULL)
        {
            status = -1;
        }
    }

    appLogPrintf("FATFS: Init ... Done !!!\n");

    return status;
}

void appFatFsLock(void)
{
    if(g_app_mmcsd_obj.lock!=NULL)
    {
        SemaphoreP_pend(g_app_mmcsd_obj.lock, SemaphoreP_WAIT_FOREVER);
    }
}

void appFatFsUnLock(void)
{
    if(g_app_mmcsd_obj.lock!=NULL)
    {
        SemaphoreP_post(g_app_mmcsd_obj.lock);
    }
}

int32_t appFatFsDeInit(void)
{
    int32_t status = 0;

    appLogPrintf("FATFS: Deinit ... !!!\n");

    FATFS_close(g_app_mmcsd_obj.fatfsHandle);
    g_app_mmcsd_obj.fatfsHandle = NULL;

    if(g_app_mmcsd_obj.lock!=NULL)
    {
        SemaphoreP_delete(g_app_mmcsd_obj.lock);
        g_app_mmcsd_obj.lock = NULL;
    }

    appLogPrintf("FATFS: Deinit ... Done !!!\n");

    return status;
}

#if 0

#include <ti/osal/osal.h>
#include <ti/drv/mmcsd/soc/MMCSD_v2.h>

#define MMCSD_INSTANCE_SDCARD   (1U)

#if 0
void MCMSD_configIntRouter(MMCSD_v2_HwAttrs *hwAttrsConfig)
{
    CSL_IntrRouterCfg  intrRouterCfg;
    uint32_t mmcsdIntRtrInIntNum,mmcsdIntRtrOutIntNum;

    /* Initialize Main to MCU Interrupt Router conifg structure */
    intrRouterCfg.pIntrRouterRegs = (CSL_intr_router_cfgRegs *)(uintptr_t)(CSL_MAIN2MCU_LVL_INTRTR0_CFG_BASE);
    intrRouterCfg.pIntdRegs       = (CSL_intr_router_intd_cfgRegs *)(uintptr_t)NULL;
    intrRouterCfg.numInputIntrs   = 192;
    intrRouterCfg.numOutputIntrs  = 64;

    /* Route MMCSD int router output to MAIN2MCU int router output, MAIN2MCU int router output int number
     * is derived from the MCU GIC interrupt number configured in HwAttr in MMCSD_soc.*/
    mmcsdIntRtrInIntNum = hwAttrsConfig->eventId;
    mmcsdIntRtrOutIntNum = hwAttrsConfig->intNum - CSL_MCU0_INTR_MAIN2MCU_LVL_INTRTR0_OUTL_0;
    CSL_intrRouterCfgMux(&intrRouterCfg, mmcsdIntRtrInIntNum, mmcsdIntRtrOutIntNum);
}
#endif

int32_t appMmcSdInit(void)
{
    int32_t status = 0;
    /* Call board init functions */
    Board_initCfg boardCfg;
    MMCSD_v2_HwAttrs hwAttrsConfig;
    MMCSD_v2_HwAttrs hwAttrsConfigDefault;
    MMCSD_mediaParams mediaParams;
    app_mmcsd_obj_t *obj = &g_app_mmcsd_obj;

    obj->handle = NULL;

    boardCfg = BOARD_INIT_PINMUX_CONFIG | BOARD_INIT_UART_STDIO | BOARD_INIT_MODULE_CLOCK;

    status = Board_init(boardCfg);

    if(0 == status)
    {
        if(MMCSD_socGetInitCfg(MMCSD_INSTANCE_SDCARD,&hwAttrsConfig)!=0)
        {
            appLogPrintf("MMC-SD: Unable to obtain config.Exiting. TEST FAILED.\r\n");
            status = -1;
        }
    }

    hwAttrsConfigDefault = hwAttrsConfig;
    hwAttrsConfig.cardType=MMCSD_CARD_SD;
    hwAttrsConfig.supportedModes = (MMCSD_SupportedSDModes_e) MMCSD_SUPPORT_SD_DS;
    hwAttrsConfig.supportedBusWidth = MMCSD_BUS_WIDTH_1BIT;
    hwAttrsConfig.supportedBusVoltages = (MMCSD_BusVoltage_e)MMCSD_BUS_VOLTAGE_3_3V;
    hwAttrsConfig= hwAttrsConfigDefault;

    /* MCMSD_configIntRouter(&hwAttrsConfig); */

    if(MMCSD_socSetInitCfg(MMCSD_INSTANCE_SDCARD,&hwAttrsConfig)!=0)
    {
        appLogPrintf("MMC-SD: Unable to set config.Exiting. TEST FAILED.\r\n");
        status = -1;
    }

    if(0 == status)
    {
        MMCSD_init();
        status=MMCSD_open(MMCSD_INSTANCE_SDCARD, NULL, &obj->handle);
    }

    if((status=MMCSD_control(obj->handle, MMCSD_CMD_GETMEDIAPARAMS, (void *)&mediaParams))!=MMCSD_OK)
    {
        appLogPrintf("MMC-SD: Getting SD Card parameters failed !!\n");
        status = -1;
    }

    appLogPrintf("MMC-SD: Init done !!\n");

    return status;
}

int32_t appMmcSdDeInit(void)
{
    int32_t status = 0;

    MMCSD_close(g_app_mmcsd_obj.handle);
    g_app_mmcsd_obj.handle = NULL;

    appLogPrintf("MMC-SD: De-Init done !!\n");

    return status;
}

#define SOC_CACHELINE_SIZE    (64U)
#define FS_APP_PATH_BUF_SIZE  (512U)
#define FS_APP_DATA_BUF_SIZE  (512U)
static FIL gFsAppReadFileObj  __attribute__ ((aligned (SOC_CACHELINE_SIZE)));
static FIL gFsAppWriteFileObj  __attribute__ ((aligned (SOC_CACHELINE_SIZE)));
static char gFsAppFileName[FS_APP_PATH_BUF_SIZE]
                          __attribute__ ((aligned (SOC_CACHELINE_SIZE)));
static char gFsAppDataBuf[FS_APP_DATA_BUF_SIZE]
                          __attribute__ ((aligned (SOC_CACHELINE_SIZE)));

/* Basic FatFs test, user should use fopen, fread, fwrite APIs from stdio.h
 * Keeping this test just in case it required for Si bringup.
 */
int32_t appFatFsTest()
{
    FRESULT status = FR_NOT_READY;
    FRESULT fRd_status = FR_NOT_READY;
    FRESULT fWt_status = FR_NOT_READY;
    uint32_t bytesWrite = 0;
    uint32_t bytesRead = 0;
    int32_t retStat = 0;

    strcpy(gFsAppFileName, "test_rd.txt");
    /* Open the file for reading. */
    fRd_status = f_open(&gFsAppReadFileObj, gFsAppFileName, FA_READ);
    /* If there was some problem opening the file, then return an error. */
    if(fRd_status != FR_OK)
    {
        appLogPrintf("Failed to open file %s for read !!!\n", gFsAppFileName);
        retStat = -1;
    }

    strcpy(gFsAppFileName, "test_wr.txt");
    fWt_status = f_open(&gFsAppWriteFileObj, gFsAppFileName, FA_WRITE|FA_OPEN_ALWAYS);
    if(fWt_status != FR_OK)
    {
        appLogPrintf("Failed to open file %s for write !!!\n", gFsAppFileName);
        retStat = -1;
    }

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
            status = f_read(&gFsAppReadFileObj, gFsAppDataBuf,
                            sizeof(gFsAppDataBuf) - 1, &bytesRead);
            /*
             * If there was an error reading, then print a newline and return
             * error to the user.
             */
            if(status != FR_OK)
            {
                appLogPrintf("Fail to read from file !!!!\n");
                retStat = -1;
            }
            /*
             * Write the data to the destination file user has selected.
             * If there was an error writing, then print a newline and return
             * error to the user.
             */
            status = f_write(&gFsAppWriteFileObj, gFsAppDataBuf,
                                   bytesRead, &bytesWrite);
            if((status != FR_OK) || (bytesRead != bytesWrite))
            {
                appLogPrintf("Fail to write into file !!!!\n");
                retStat = -1;
            }
            /*
             * Continue reading until less than the full number of bytes are
             * read. That means the end of the buffer was reached.
             */
        }
        while((bytesRead == (sizeof(gFsAppDataBuf) - 1)) && (retStat==0));
    }
    /*
     * Close the Read file.
     * If there was an error writing, then print a newline and return the
     * error to the user.
     */
    if(fRd_status == FR_OK)
    {
        status = f_close(&gFsAppReadFileObj);
        if(status != FR_OK)
        {
            appLogPrintf("Fail to close read file !!!\n");
            retStat = -1;
        }
    }
    /*
     * Close the Write file.
     * If there was an error writing, then print a newline and return the
     * error to the user.
     */
    if(fWt_status == FR_OK)
    {
        status = f_close(&gFsAppWriteFileObj);
        if(status != FR_OK)
        {
            appLogPrintf("Fail to close write file !!!\n");
            retStat = -1;
        }
    }
    return(retStat);
}
#endif
