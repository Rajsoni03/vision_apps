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
#include "radar_gtrack_applib_priv.h"

#define RADAR_GTRACK_APPLIB_MAX_LINE_LEN   (256U)

static vx_status RADAR_GTRACK_APPLIB_setupTrackerNode(
       RADAR_GTRACK_APPLIB_Context     *appCntxt,
       RADAR_GTRACK_APPLIB_graphParams *gpDesc,
       uint8_t                         *objData,
       uint32_t                         sensorId,
       uint64_t                         timestamp)
{
    void                       *ptr;
    PTK_Alg_RadarDetOutput     *gatedObjData;
    PTK_Alg_RadarSensorConfig  *sensorCfg;
    vx_user_data_object         vxObjData;
    vx_user_data_object         vxPoseAndRef;
    vx_user_data_object         vxSensorCfg;
    PTK_INS_RetCode             retCode;
    vx_status                   vxStatus;

    sensorCfg    = &appCntxt->ogConfig.cfg[sensorId];
    vxObjData    = gpDesc->vxObjData;
    vxPoseAndRef = gpDesc->vxPoseAndRef;
    vxSensorCfg  = gpDesc->vxSensorCfg;
    vxStatus     = VX_SUCCESS;

    /* Get a reference to the Pose and reference buffer. */
    ptr = ptkdemo_getUserObjDataPayload(vxPoseAndRef);

    if (ptr == NULL)
    {
        PTK_printf("[%s:%d] ptkdemo_getUserObjDataPayload() failed.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        PTK_InsPoseAndRef  *poseAndRef;
        PTK_INS_Record     *curInsRec;
        PTK_INS_Record     *prevInsRec;

        poseAndRef = (PTK_InsPoseAndRef*)ptr;
        curInsRec  = &poseAndRef->curInsRec;
        prevInsRec = &poseAndRef->prevInsRec;

        retCode = PTK_INS_getRecordLinearInterp(PTK_INS_RECORD_TYPE_INSPVA,
                                                timestamp,
                                                curInsRec);

        if (( retCode != PTK_INS_RETURN_CODE_OK ) &&
            ( retCode != PTK_INS_RETURN_CODE_RECORD_PARTIALLY_AVAIL ))
        {
            PTK_printf("[%s:%d] PTK_INS_getRecordLinearInterp() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            retCode = PTK_INS_getRecordBefore(PTK_INS_RECORD_TYPE_INSPVA,
                                              timestamp,
                                              prevInsRec);

            if ( retCode != PTK_INS_RETURN_CODE_OK )
            {
                PTK_printf("[%s:%d] PTK_INS_getRecordBefore() failed.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        ptr = ptkdemo_getUserObjDataPayload(vxObjData);

        if (ptr == NULL)
        {
            PTK_printf("[%s:%d] ptkdemo_getUserObjDataPayload() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        int32_t status;

        gatedObjData = (PTK_Alg_RadarDetOutput*)ptr;

        /* Parse object data */
        status = PTK_Alg_RadarOgmapParseObjData(objData,
                                                &appCntxt->parsedObjData);

        if (status != PTK_ALG_RET_SUCCESS)
        {
            PTK_printf("[%s:%d] PTK_Alg_RadarOgmapParseObjData() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            /* Object gating for angle, range, and SNR */
            PTK_Alg_RadarOgmapGateObjData(sensorCfg,
                                          &appCntxt->parsedObjData,
                                          gatedObjData);

            /* Update the sensor configuration. */
            vxStatus = vxCopyUserDataObject(vxSensorCfg,
                                            0,
                                            sizeof(PTK_Alg_RadarSensorConfig),
                                            sensorCfg,
                                            VX_WRITE_ONLY,
                                            VX_MEMORY_TYPE_HOST);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] vxCopyUserDataObject() failed.\n",
                           __FUNCTION__, __LINE__);
            }
        }
    }

    return vxStatus;

} /* RADAR_GTRACK_APPLIB_setupTrackerNode */

static vx_status RADAR_GTRACK_APPLIB_createGraph(
        RADAR_GTRACK_APPLIB_Context  *appCntxt)
{
    uint32_t    cfgDataSize;
    uint32_t    sensorCfgSize;
    uint32_t    objDataSize;
    uint32_t    trackInfoSize;
    vx_status   vxStatus;
    uint32_t    i;

    vxStatus      = VX_SUCCESS;
    cfgDataSize   = sizeof(PTK_Alg_RadarGTrackParams);
    sensorCfgSize = sizeof(PTK_Alg_RadarSensorConfig);
    objDataSize   = sizeof(PTK_Alg_RadarDetOutput);
    trackInfoSize = sizeof(PTK_Alg_RadarGTrackTargetInfo);

    appCntxt->vxGraph = vxCreateGraph(appCntxt->vxContext);

    if (appCntxt->vxGraph == NULL)
    {
        PTK_printf("[%s:%d] vxCreateGraph() failed\n",
                    __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }
    else
    {
        vxSetReferenceName((vx_reference)appCntxt->vxGraph,
                           "Radar Gtrack Graph");
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Configuration context. */
        appCntxt->vxTrackConfig =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "PTK_Alg_RadarGTrackParams",
                                   cfgDataSize,
                                   &appCntxt->gTrackParams);

        if (appCntxt->vxTrackConfig == NULL)
        {
            PTK_printf("[%s:%d] vxCreateUserDataObject() failed\n",
                        __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxTrackConfig,
                               "RadarGTrackConfig");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        for (i = 0; i < appCntxt->pipelineDepth; i++ )
        {
            /* Radar Object data. */
            appCntxt->vxObjData[i] =
                vxCreateUserDataObject(appCntxt->vxContext,
                                       "PTK_Alg_RadarDetOutput",
                                       objDataSize,
                                       NULL);

            if (appCntxt->vxObjData[i] == NULL)
            {
                PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
                break;
            }
            else
            {
                vxSetReferenceName((vx_reference)appCntxt->vxObjData[i],
                                   "RadarObjectData");
            }

            /* Position and reference object. */
            appCntxt->vxPoseAndRef[i] =
                vxCreateUserDataObject(appCntxt->vxContext,
                                       "PTK_InsPoseAndRef",
                                       sizeof(PTK_InsPoseAndRef),
                                       NULL);

            if (appCntxt->vxPoseAndRef[i] == NULL)
            {
                PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
                break;
            }
            else
            {
                vxSetReferenceName((vx_reference)appCntxt->vxPoseAndRef[i],
                                   "PoseAndReference");
            }

            /* Sensor config input. */
            appCntxt->vxSensorCfg[i] =
                vxCreateUserDataObject(appCntxt->vxContext,
                                       "PTK_Alg_RadarSensorConfig",
                                       sensorCfgSize,
                                       NULL);

            if (appCntxt->vxSensorCfg[i] == NULL)
            {
                PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
                break;
            }
            else
            {
                vxSetReferenceName((vx_reference)appCntxt->vxSensorCfg[i],
                                   "SensorCfg");
            }

            /* Tracker output. */
            appCntxt->vxOutTrackInfo[i] =
                vxCreateUserDataObject(appCntxt->vxContext,
                                       "PTK_Alg_RadarGTrackTargetInfo",
                                       trackInfoSize,
                                       NULL);

            if (appCntxt->vxOutTrackInfo[i] == NULL)
            {
                PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
                break;
            }
            else
            {
                vxSetReferenceName((vx_reference)appCntxt->vxOutTrackInfo[i],
                                   "TrackTargetInfo");
            }

        } // for (i = 0; i < appCntxt->pipelineDepth; i++ )
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Create the nodes. */
        appCntxt->vxGTrackNode =
            tivxRadarGtrackNode(appCntxt->vxGraph,
                                appCntxt->vxTrackConfig,
                                appCntxt->vxSensorCfg[0],
                                appCntxt->vxObjData[0],
                                appCntxt->vxPoseAndRef[0],
                                appCntxt->vxOutTrackInfo[0]);

        if (appCntxt->vxGTrackNode == NULL)
        {
            PTK_printf("[%s:%d] tivxRadarGtrackNode() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxGTrackNode,
                               "RadarGTrack");

            vxStatus = vxSetNodeTarget(appCntxt->vxGTrackNode,
                                       VX_TARGET_STRING,
                                       appCntxt->gTrackNodeCore);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] vxSetNodeTarget() failed.\n",
                           __FUNCTION__, __LINE__);
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* set up the pipeline. */
        vxStatus = RADAR_GTRACK_APPLIB_setupPipeline(appCntxt);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] RADAR_GTRACK_APPLIB_setupPipeline() failed.\n",
                       __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxVerifyGraph(appCntxt->vxGraph);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] vxVerifyGraph() failed.\n",
                       __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appPerfPointSetName(&appCntxt->perf , "RADAR GTRACK");

        if (appCntxt->exportGraph == 1)
        {
            tivxExportGraphToDot(appCntxt->vxGraph, ".",
                                 "vx_applib_radar_gtrack");
        }

        if (appCntxt->rtLogEnable == 1)
        {
            tivxLogRtTraceEnable(appCntxt->vxGraph);
        }
    }

    return vxStatus;

} /* RADAR_GTRACK_APPLIB_createGraph */

static void RADAR_GTRACK_APPLIB_releaseGraph(RADAR_GTRACK_APPLIB_Handle handle)
{
    RADAR_GTRACK_APPLIB_Context    *appCntxt;
    uint32_t                        i;

    appCntxt = (RADAR_GTRACK_APPLIB_Context *)handle;

    /* Relase the node. */
    if (appCntxt->vxGTrackNode != NULL)
    {
        vxReleaseNode(&appCntxt->vxGTrackNode);
    }

    /* Release the parameter memory. */
    if (appCntxt->vxTrackConfig != NULL)
    {
        vxReleaseUserDataObject(&appCntxt->vxTrackConfig);
    }

    for (i = 0; i < appCntxt->pipelineDepth; i++ )
    {
        if (appCntxt->vxPoseAndRef[i] != NULL)
        {
            vxReleaseUserDataObject(&appCntxt->vxPoseAndRef[i]);
        }

        if (appCntxt->vxObjData[i] != NULL)
        {
            vxReleaseUserDataObject(&appCntxt->vxObjData[i]);
        }

        if (appCntxt->vxSensorCfg[i] != NULL)
        {
            vxReleaseUserDataObject(&appCntxt->vxSensorCfg[i]);
        }

        if (appCntxt->vxOutTrackInfo[i] != NULL)
        {
            vxReleaseUserDataObject(&appCntxt->vxOutTrackInfo[i]);
        }
    }

    if (appCntxt->rtLogEnable == 1)
    {
        tivxLogRtTraceDisable(appCntxt->vxGraph);
    }

    if (appCntxt->vxGraph != NULL)
    {
        /* Release the graph. */
        vxReleaseGraph(&appCntxt->vxGraph);
    }

    return;

} /* RADAR_GTRACK_APPLIB_releaseGraph */

RADAR_GTRACK_APPLIB_Handle RADAR_GTRACK_APPLIB_create(
        RADAR_GTRACK_APPLIB_createParams *createParams)
{
    RADAR_GTRACK_APPLIB_Context *appCntxt;
    RADAR_GTRACK_APPLIB_Handle   handle;
    vx_status                    vxStatus;

    handle   = new RADAR_GTRACK_APPLIB_Context();
    vxStatus = VX_SUCCESS;

    if (handle == NULL)
    {
        PTK_printf("[%s:%d] Could not allocate memory for internal context.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt = (RADAR_GTRACK_APPLIB_Context *)handle;
        appCntxt->gTrackParams = createParams->gTrackParams;
        appCntxt->ogConfig     = createParams->ogConfig;
        appCntxt->vxContext    = createParams->vxContext;

        appCntxt->gTrackNodeCore = createParams->gTrackNodeCore;

        if (appCntxt->gTrackNodeCore == NULL)
        {
            appCntxt->gTrackNodeCore = RADAR_GTRACK_APPLIB_DEFAULT_CORE_MAPPING;
        }

        /* Get the sensor count. */
        appCntxt->numSensors =
            PTK_Util_bitCnt(appCntxt->ogConfig.sensorMask);

        appCntxt->vxEvtAppValBase = createParams->vxEvtAppValBase;

        /* Set the pipeline depth. */
        if (!createParams->pipelineDepth ||
            (createParams->pipelineDepth > RADAR_GTRACK_APPLIB_PIPELINE_DEPTH))
        {
            PTK_printf("[%s:%d] Invalid pipeline depth value. "
                       "Passed %d. Allowed range [1..%d]\n",
                       __FUNCTION__,
                       __LINE__,
                       createParams->pipelineDepth,
                       RADAR_GTRACK_APPLIB_PIPELINE_DEPTH);

            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->pipelineDepth = createParams->pipelineDepth;
        appCntxt->exportGraph   = createParams->exportGraph;
        appCntxt->rtLogEnable   = createParams->rtLogEnable;

        /* Create Nodes. */
        vxStatus = RADAR_GTRACK_APPLIB_createGraph(handle);
    }

    if (vxStatus != VX_SUCCESS)
    {
        RADAR_GTRACK_APPLIB_delete(&handle);
    }

    return handle;

} /* RADAR_GTRACK_APPLIB_create */

vx_status RADAR_GTRACK_APPLIB_delete(
        RADAR_GTRACK_APPLIB_Handle *handle)
{
    /* Delete the nodes. */
    if ((handle != NULL) && (*handle != NULL))
    {
        RADAR_GTRACK_APPLIB_releaseGraph(*handle);

        delete *handle;
        *handle = NULL;
    }

    return VX_SUCCESS;

} /* RADAR_GTRACK_APPLIB_delete */

vx_status RADAR_GTRACK_APPLIB_setWorldReference(
        RADAR_GTRACK_APPLIB_Handle  handle,
        const PTK_RigidTransform_d *ecef_w)
{
    RADAR_GTRACK_APPLIB_Context *appCntxt;
    PTK_InsPoseAndRef           *poseAndRef;
    void                        *ptr;
    vx_status                    vxStatus;

    appCntxt = (RADAR_GTRACK_APPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        for (uint32_t i = 0; i < appCntxt->pipelineDepth; i++)
        {
            /* Get a reference to the Pose and reference buffer. */
            ptr = ptkdemo_getUserObjDataPayload(appCntxt->vxPoseAndRef[i]);

            if (ptr == NULL)
            {
                PTK_printf("[%s:%d] ptkdemo_getUserObjDataPayload() failed.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
                break;
            }

            poseAndRef         = (PTK_InsPoseAndRef*)ptr;
            poseAndRef->w2Ecef = *ecef_w;
        }
    }

    return vxStatus;

} /* RADAR_GTRACK_APPLIB_setWorldReference */

vx_status RADAR_GTRACK_APPLIB_process(
        RADAR_GTRACK_APPLIB_Handle  handle,
        uint8_t                    *data,
        uint32_t                    sensorId,
        uint64_t                    timestamp)
{
    RADAR_GTRACK_APPLIB_Context    *appCntxt;
    vx_user_data_object             obj[RADAR_GTRACK_APPLIB_NUM_GRAPH_PARAMS];
    RADAR_GTRACK_APPLIB_graphParams gpDesc;
    vx_status                       vxStatus;

    appCntxt = (RADAR_GTRACK_APPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = RADAR_GTRACK_APPLIB_getFreeParamRsrc(appCntxt, &gpDesc);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] RADAR_GTRACK_APPLIB_getFreeParamRsrc() "
                       "failed.\n", __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = RADAR_GTRACK_APPLIB_setupTrackerNode(appCntxt,
                                                        &gpDesc,
                                                        data,
                                                        sensorId,
                                                        timestamp);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] RADAR_GTRACK_APPLIB_setupTrackerNode() "
                       "failed.\n", __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        obj[0] = gpDesc.vxSensorCfg;
        obj[1] = gpDesc.vxObjData;
        obj[2] = gpDesc.vxPoseAndRef;
        obj[3] = gpDesc.vxOutTrackInfo;

        for (uint32_t i = 0; i < RADAR_GTRACK_APPLIB_NUM_GRAPH_PARAMS; i++)
        {
            vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxGraph,
                                                       i,
                                                       (vx_reference*)&obj[i],
                                                       1);
            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] vxGraphParameterEnqueueReadyRef(%d) "
                           "failed\n", __FUNCTION__, __LINE__, i);
                break;
            }
        }
    }

    return vxStatus;

} /* RADAR_GTRACK_APPLIB_process */

vx_status RADAR_GTRACK_APPLIB_getOutBuff(
        RADAR_GTRACK_APPLIB_Handle      handle,
        RADAR_GTRACK_APPLIB_OutputBuff *buff)
{
    RADAR_GTRACK_APPLIB_Context        *appCntxt;
    RADAR_GTRACK_APPLIB_graphParams    *desc;
    PTK_Alg_RadarGTrackTargetInfo      *trackInfo;
    PTK_InsPoseAndRef                  *poseAndRef;
    vx_status                           vxStatus;

    appCntxt = (RADAR_GTRACK_APPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        void                        *ptr;
        std::unique_lock<std::mutex> lock(appCntxt->paramRsrcMutex);

        if (appCntxt->outputQ.empty())
        {
            vxStatus = VX_FAILURE;
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            /* Get the descriptor. */
            desc = appCntxt->outputQ.front();

            /* Get a reference to the Pose and reference buffer. */
            ptr = ptkdemo_getUserObjDataPayload(desc->vxPoseAndRef);

            if (ptr == NULL)
            {
                PTK_printf("[%s:%d] ptkdemo_getUserObjDataPayload() failed.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }

            poseAndRef = (PTK_InsPoseAndRef*)ptr;
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            /* Get a reference to the Pose and reference buffer. */
            ptr = ptkdemo_getUserObjDataPayload(desc->vxOutTrackInfo);

            if (ptr == NULL)
            {
                PTK_printf("[%s:%d] ptkdemo_getUserObjDataPayload() failed.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }

            trackInfo = (PTK_Alg_RadarGTrackTargetInfo*)ptr;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        buff->ts        = poseAndRef->curInsRec.timestamp;
        buff->trackInfo = trackInfo;
    }

    return vxStatus;

} /* RADAR_GTRACK_APPLIB_getOutBuff */

vx_status RADAR_GTRACK_APPLIB_releaseOutBuff(
        RADAR_GTRACK_APPLIB_Handle  handle)
{
    RADAR_GTRACK_APPLIB_Context        *appCntxt;
    RADAR_GTRACK_APPLIB_graphParams    *desc;
    vx_status                           vxStatus;

    appCntxt = (RADAR_GTRACK_APPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        std::unique_lock<std::mutex> lock(appCntxt->paramRsrcMutex);

        if (appCntxt->outputQ.empty())
        {
            PTK_printf("[%s:%d] No output buffers available.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            desc = appCntxt->outputQ.front();
            appCntxt->outputQ.pop();

            /* Push the descriptor into the free queue. */
            appCntxt->freeQ.push(desc);
        }
    }

    return vxStatus;
}

vx_status RADAR_GTRACK_APPLIB_printStats(
        RADAR_GTRACK_APPLIB_Handle  handle)
{
    RADAR_GTRACK_APPLIB_Context    *appCntxt;
    vx_status                       vxStatus;

    appCntxt = (RADAR_GTRACK_APPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = tivx_utils_graph_perf_print(appCntxt->vxGraph);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] tivx_utils_graph_perf_print() failed\n",
                        __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appPerfPointPrint(&appCntxt->perf);
        appPerfPointPrintFPS(&appCntxt->perf);
    }

    return vxStatus;
}

vx_status RADAR_GTRACK_APPLIB_exportStats(
        RADAR_GTRACK_APPLIB_Handle  handle,
        FILE                       *fp,
        bool                        exportAll)
{
    RADAR_GTRACK_APPLIB_Context    *appCntxt;
    vx_status                       vxStatus;

    appCntxt = (RADAR_GTRACK_APPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = tivx_utils_graph_perf_export(fp, appCntxt->vxGraph);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] tivx_utils_graph_perf_export() failed\n",
                        __FUNCTION__, __LINE__);
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            if (exportAll == true)
            {
                app_perf_point_t *perfArr[1] = {&appCntxt->perf};
                appPerfStatsExportAll(fp, perfArr, 1);
            }
        }
    }

    return vxStatus;
}

vx_status RADAR_GTRACK_APPLIB_processEvent(
        RADAR_GTRACK_APPLIB_Handle  handle,
        vx_event_t                 *event)
{
    RADAR_GTRACK_APPLIB_Context    *appCntxt;
    vx_status                       vxStatus;

    appCntxt = (RADAR_GTRACK_APPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if ((vxStatus == (vx_status)VX_SUCCESS) &&
        (event->type == VX_EVENT_GRAPH_COMPLETED))
    {
        vx_reference    ref[RADAR_GTRACK_APPLIB_NUM_GRAPH_PARAMS];
        uint32_t        numRefs;
        uint32_t        index;
        uint32_t        appValue;
        uint32_t        i;

        appValue = appCntxt->vxEvtAppValBase +
                   RADAR_GTRACK_APPLIB_GRAPH_COMPLETE_EVENT;

        if (event->app_value != appValue)
        {
            /* Something wrong. We did not register for this event. */
            PTK_printf("[%s:%d] Unknown App Value [%d].\n",
                       __FUNCTION__, __LINE__, event->app_value);

            vxStatus = VX_FAILURE;
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            if (appCntxt->startPerfCapt == false)
            {
                appCntxt->startPerfCapt = true;
            }
            else
            {
                appPerfPointEnd(&appCntxt->perf);
            }

            appPerfPointBegin(&appCntxt->perf);

            /* Node execution is complete. Deque all the parameters. */
            for (i = 0; i < RADAR_GTRACK_APPLIB_NUM_GRAPH_PARAMS; i++)
            {
                vxStatus = vxGraphParameterDequeueDoneRef(appCntxt->vxGraph,
                                                          i,
                                                          &ref[i],
                                                          1,
                                                          &numRefs);
                if (vxStatus != VX_SUCCESS)
                {
                    PTK_printf("[%s:%d] vxGraphParameterDequeueDoneRef() failed\n",
                               __FUNCTION__, __LINE__);

                    break;
                }
            }
        }

        /* The first one to deque is the vxSensorCfg parameter. Search and
         * identify the resource index.
         */
        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            index = 255;
            for (i = 0; i < appCntxt->pipelineDepth; i++)
            {
                if (ref[0] == (vx_reference)appCntxt->vxSensorCfg[i])
                {
                    index = i;
                    break;
                }
            }

            if (index == 255)
            {
                PTK_printf("[%s:%d] Resource look up failed\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            /* Mark the dequeued resource as free. */
            vxStatus = RADAR_GTRACK_APPLIB_releaseParamRsrc(appCntxt, index);

            if (vxStatus != VX_SUCCESS)
            {
                PTK_printf("[%s:%d] RADAR_GTRACK_APPLIB_releaseParamRsrc() "
                           "failed.\n", __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }
        }
    }

    return vxStatus;

} /* RADAR_GTRACK_APPLIB_processEvent. */

vx_status RADAR_GTRACK_APPLIB_waitGraph(
        RADAR_GTRACK_APPLIB_Handle  handle)
{
    RADAR_GTRACK_APPLIB_Context    *appCntxt;
    vx_status                       vxStatus;

    appCntxt = (RADAR_GTRACK_APPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxWaitGraph(appCntxt->vxGraph);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] vxWaitGraph() failed\n",
                        __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Wait for the output queue to get flushed. */
        while (appCntxt->freeQ.size() != appCntxt->pipelineDepth)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    return vxStatus;
}

vx_status  RADAR_GTRACK_APPLIB_reset(
        RADAR_GTRACK_APPLIB_Handle  handle)
{
    RADAR_GTRACK_APPLIB_Context    *appCntxt;
    vx_status                       vxStatus;

    appCntxt = (RADAR_GTRACK_APPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }
    else
    {
        /* Reset the performance capture initialization flag. */
        appCntxt->startPerfCapt = false;
    }

    return vxStatus;
}

