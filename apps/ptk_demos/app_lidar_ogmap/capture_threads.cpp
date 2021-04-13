/*
 *
 * Copyright (c) 2019 Texas Instruments Incorporated
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
#include <perception/drv/ins_capture.h>

#include "lidar_ogmap_main.h"
#include "lidar_ogmap_renderer.h"

#define MAX_NUM_LIDAR_FRAME_BUFFS   (4U)
#define MAX_NUM_INS_FRAME_BUFFS     (4U)

int32_t ParseAndPublishPacket(uint8_t   *insPkt)
{
    PTK_INS_RetCode     insRetCode;
    uint64_t            timestamp;

    /* Parse and publish the message. */
    timestamp = *(uint64_t *)insPkt;

    insPkt += INS_CAPTURE_DRV_TS_FIELD_LENGTH;

    insRetCode = PTK_INS_Novatel_processMessage(insPkt, timestamp);

    if (insRetCode != PTK_INS_RETURN_CODE_OK)
    {
        PTK_printf("[%s] PTK_INS_Novatel_processMessage() failed.\n",
                   __FUNCTION__);

        return -1;
    }

    return 0;
}

void fillDefaultCaptureConfigParams(PTK_Drv_InsDrvParams   *insDrvConfig)
{
    PTK_Drv_InsLogCmd  *logCmds;
    uint32_t            numCmds;

    insDrvConfig->drvType          = PTK_Drv_InsDrvType_NETWORK;
    insDrvConfig->portNum          = INS_CAPTURE_DRV_IMU_PORT;
    insDrvConfig->numMsgsPerBuffer = 1;

    numCmds           = 0;
    logCmds           = &insDrvConfig->logCmds[numCmds++];
    logCmds->msgId    = NOVATEL_MESSAGE_ID_INSPVA;
    logCmds->logCtrl  = NOVATEL_LOG_ON_TIME;
    logCmds->interval = 0.01f;

    logCmds          = &insDrvConfig->logCmds[numCmds++];
    logCmds->msgId   = NOVATEL_MESSAGE_ID_INSCOV;
    logCmds->logCtrl = NOVATEL_LOG_ON_NEW;
    logCmds->interval = 0.0f;

    insDrvConfig->numLogCmds = numCmds;
}

void insCaptureCb(PTK_IPC_BuffDesc *desc, void *appData)
{
    LIDAR_AppContext  * appCntxt;
    uint8_t           * insPkt;
    int32_t             status;

    appCntxt = (LIDAR_AppContext *)appData;
    insPkt = (uint8_t *)desc->payload[0];

    status = ParseAndPublishPacket(insPkt);

    if (status < 0)
    {
        PTK_printf("ParseAndPublishPacket() failed with status %d.\n", status);
    }

    /* Release the descriptor. */
    PTK_IPC_BuffDescRelease(desc);

    if (appCntxt->firstInsPkt)
    {
        PTK_RigidTransform_d    ecef_w;
        PTK_Position            ref;

        appCntxt->firstInsPkt = 0;

        /* Set up world reference frame
         * (SensorDataPlayerINS handles initializing this at create time) */
        PTK_INS_getReferenceFrame(&ecef_w, &ref);

        /* Set the reference frame in the sensor module. */
        LIDAROGAPPLIB_setRootPosition(appCntxt->lidarHdl, &ecef_w, &ref);
    }

    //PTK_printf("%s completed successfully.\n", __FUNCTION__);
}

void lidarCaptureCb(PTK_IPC_BuffDesc *desc, void *appData)
{
    LIDAR_AppContext  * appCntxt;
    uint8_t           * lidarPkt;

    appCntxt = (LIDAR_AppContext *)appData;
    lidarPkt = (uint8_t *)desc->payload[0];

    LIDAROGAPPLIB_process(appCntxt->lidarHdl, lidarPkt);

    /* Release the descriptor. */
    PTK_IPC_BuffDescRelease(desc);

    PTK_printf("%s completed successfully.\n", __FUNCTION__);
}

int32_t ins_capture_thread(LIDAR_AppContext   * appCntxt,
                           PTK_IPC_Cb           cb)
{
    void                  * insFrames[MAX_NUM_INS_FRAME_BUFFS];
    PTK_Api_MemoryReq       memReq;
    uint32_t                outBuffSize;
    uint32_t                descSize;
    uint32_t                memBlkSize;
    int32_t                 status;

    status = PTK_Drv_InsCaptureConfig(&appCntxt->insDrvConfig, &memReq);

    if (status != PTK_DRV_RET_SUCCESS)
    {
        PTK_printf("[%s] PTK_Drv_InsCaptureConfig() failed\n", __FUNCTION__);
        return -1;
    }

    appCntxt->insCapHandle = PTK_InsCapCreate(&appCntxt->insDrvConfig);

    PTK_InsCapRegisterAppCb(appCntxt->insCapHandle, cb, appCntxt);

    /* Based on the Ins INS Driver API documentation, the second and
     * final entry indicates the size of the output buffer.
     */
    outBuffSize = memReq.blks[1].size;
    descSize    = sizeof(PTK_IPC_BuffDesc);

    PTK_printf("INS OUTPUT BUFFER SIZE = %d bytes.\n", outBuffSize);

    memBlkSize = outBuffSize + descSize;

    /* Enque free buffers. */
    for (uint32_t i = 0; i < MAX_NUM_INS_FRAME_BUFFS; i++)
    {
        PTK_IPC_BuffDesc  * desc;
        void              * mem;

        mem = malloc(memBlkSize);

        if (!mem)
        {
            int32_t j;

            printf("[%s] Memory allocation for [%d] bytes failed\n",
                   __FUNCTION__, memBlkSize);

            /* Release the memory alloated so far. */
            for (j = i-1; j >= 0; j--)
            {
                free(insFrames[j]);
            }

            /* De-initialize the capture context. */
            PTK_InsCapDelete(appCntxt->insCapHandle);

            return -1;
        }

        insFrames[i]         = mem;
        desc                 = (PTK_IPC_BuffDesc *)mem;
        desc->numBuffs       = 1;
        desc->payloadSize[0] = outBuffSize;
        desc->payload[0]     = (uint8_t *)mem + descSize;

        PTK_InsCapEnqueDesc(appCntxt->insCapHandle, desc);
    }

    PTK_InsCapSpawnThreads(appCntxt->insCapHandle);

    return 0;
}

int32_t lidar_capture_thread(LIDAR_AppContext * appCntxt,
                             PTK_IPC_Cb         cb)
{
    void                      * lidarFrames[MAX_NUM_LIDAR_FRAME_BUFFS];
    PTK_Api_MemoryReq           memReq;
    uint32_t                    outBuffSize;
    uint32_t                    descSize;
    uint32_t                    memBlkSize;
    int32_t                     status;

    /* Get the output buffer size. */
    status = PTK_Drv_LidarCaptureConfig(&appCntxt->lidarDrvConfig, &memReq);

    if (status != PTK_DRV_RET_SUCCESS)
    {
        PTK_printf("[%s] PTK_Drv_LidarCaptureConfig() failed\n",
                   __FUNCTION__);
        return -1;
    }

    appCntxt->lidarCapHandle = PTK_LidarCapCreate(&appCntxt->lidarDrvConfig);

    PTK_LidarCapRegisterAppCb(appCntxt->lidarCapHandle,
                              cb,
                              appCntxt);

    /* Based on the Lidar Driver API documentation, the second and
     * final entry indicates the size of the output buffer.
     */
    outBuffSize = memReq.blks[1].size;
    descSize    = sizeof(PTK_IPC_BuffDesc);
    memBlkSize  = outBuffSize + descSize;

    /* Enque free buffers. */
    for (uint32_t i = 0; i < MAX_NUM_LIDAR_FRAME_BUFFS; i++)
    {
        PTK_IPC_BuffDesc  * desc;
        void              * mem;

        mem = malloc(memBlkSize);

        if (!mem)
        {
            int32_t j;

            printf("[%s] Memory allocation for [%d] bytes failed\n",
                   __FUNCTION__, memBlkSize);

            /* Release the memory alloated so far. */
            for (j = i-1; j >= 0; j--)
            {
                free(lidarFrames[j]);
            }

            /* De-initialize the capture context. */
            PTK_LidarCapDelete(appCntxt->lidarCapHandle);

            return -1;
        }

        lidarFrames[i]       = mem;
        desc                 = (PTK_IPC_BuffDesc *)mem;
        desc->numBuffs       = 1;
        desc->payloadSize[0] = outBuffSize;
        desc->payload[0]     = (uint8_t *)mem + descSize;

        PTK_LidarCapEnqueDesc(appCntxt->lidarCapHandle, desc);
    }

    PTK_LidarCapSpawnThreads(appCntxt->lidarCapHandle);

    return 0;
}

int32_t LIDAR_launchCaptureThreads(LIDAR_AppContext   * appCntxt)
{
    PTK_Drv_InsDrvParams      * insDrvConfig;
    PTK_Drv_LidarDrvParams    * lidarDrvConfig;

    insDrvConfig = &appCntxt->insDrvConfig;

    fillDefaultCaptureConfigParams(insDrvConfig);

    lidarDrvConfig          = &appCntxt->lidarDrvConfig;
    lidarDrvConfig->drvType = PTK_Drv_LidarDrvType_NETWORK;

    /* Spawn the capture threads. */
    ins_capture_thread(appCntxt, insCaptureCb);
    lidar_capture_thread(appCntxt, lidarCaptureCb);

    while (1)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}

