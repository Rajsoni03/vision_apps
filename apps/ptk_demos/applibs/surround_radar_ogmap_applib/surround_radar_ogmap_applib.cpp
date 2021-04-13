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
#include "surround_radar_ogmap_applib_priv.h"

static vx_status RADAROGAPPLIB_setupOgNode(
       RADAROGAPPLIB_Context       *appCntxt,
       RADAROGAPPLIB_graphParams   *gpDesc,
       uint8_t                     *objData,
       uint32_t                     sensorId,
       uint64_t                     timestamp)
{
    void                       *ptr;
    PTK_Alg_RadarDetOutput     *gatedObjData;
    PTK_Alg_RadarSensorConfig  *sensorCfg;
    vx_user_data_object         vxObjData;
    vx_user_data_object         vxPoseAndRef;
    vx_user_data_object         vxSensorCfg;
    PTK_INS_RetCode             retCode;
    vx_status                   vxStatus;

    sensorCfg    = &appCntxt->ogPfsdCfg.ogConfig.cfg[sensorId];
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

}

static vx_status RADAROGAPPLIB_createGraph(
        RADAROGAPPLIB_Context  *appCntxt)
{
    PTK_Map                *ogMap;
    uint8_t                *ogMapMem;
    PTK_MapConfig           mapConfig[PTK_ALG_RADAR_OGMAP_NUM_OUT_MAPS];
    PTK_Api_MemoryReq       memReq;
    uint32_t                cfgDataSize;
    uint32_t                sensorCfgSize;
    uint32_t                objDataSize;
    uint32_t                i;
    vx_status               vxStatus;
    int32_t                 status;

    vxStatus      = VX_SUCCESS;
    cfgDataSize   = sizeof(tivx_radar_ogmap_pfsd_params_t);
    sensorCfgSize = sizeof(PTK_Alg_RadarSensorConfig);
    objDataSize   = sizeof(PTK_Alg_RadarDetOutput);
    ogMap         = NULL;
    ogMapMem      = NULL;

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
                           "Radar Occupancy Grid Graph");
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Configuration context. */
        appCntxt->vxOgConfig =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "tivx_radar_ogmap_pfsd_params_t",
                                   cfgDataSize,
                                   &appCntxt->ogPfsdCfg);

        if (appCntxt->vxOgConfig == NULL)
        {
            PTK_printf("[%s:%d] vxCreateUserDataObject() failed\n",
                        __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxOgConfig,
                               "RadarOgmapPfsdConfig");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Out Map */
        status = PTK_Alg_RadarOgmapConfig(&appCntxt->ogPfsdCfg.ogConfig,
                                          &memReq);

        if ((status != PTK_ALG_RET_SUCCESS) ||
            (memReq.numBlks != PTK_ALG_RADAR_OGMAP_NUM_MEM_REQ_BLKS))
        {
            PTK_printf("[%s:%d] PTK_Alg_RadarOgmapConfig() failed\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        memset(mapConfig, 0, sizeof(mapConfig));

        /* Dummy memory space to initialize maps */
        status = PTK_Alg_RadarOgmapGetMapConfig(&appCntxt->ogPfsdCfg.ogConfig,
                                                mapConfig);

        /* According to the RADAR OGMAP API guide:
         * - the entry with id * PTK_ALG_RADAR_OGMAP_MEM_BUFF_MAP_ACC_OCC gives
         *   the output accumulated map size.
         * - the entry with id * PTK_ALG_RADAR_OGMAP_MEM_BUFF_MAP_INST_DS gives
         *   the output instantaneous map size.
         */
        /* Accumulated output map. */
        if (status != PTK_ALG_RET_SUCCESS)
        {
            PTK_printf("[%s:%d] PTK_Alg_RadarOgmapGetMapConfig() failed\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            appCntxt->outAccMapSize =
                memReq.blks[PTK_ALG_RADAR_OGMAP_MEM_BUFF_MAP_ACC_OCC].size;

            ogMapMem = (uint8_t *)malloc(appCntxt->outAccMapSize);

            if (ogMapMem == NULL)
            {
                PTK_printf("[%s:%d] Memory allocation failed.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }
            else
            {
                memset(ogMapMem, 0, appCntxt->outAccMapSize);

                ogMap =
                PTK_Map_init(ogMapMem,
                             &mapConfig[PTK_ALG_RADAR_OGMAP_OUT_MAP_ACC_OCC]);

                if (ogMap == NULL)
                {
                    PTK_printf("[%s:%d] PTK_Map_init() failed.\n",
                               __FUNCTION__, __LINE__);

                    vxStatus = VX_FAILURE;
                }
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Output map object. */
        appCntxt->vxOutAccMap =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "AccMap",
                                   appCntxt->outAccMapSize,
                                   ogMap);

        if (appCntxt->vxOutAccMap == NULL)
        {
            PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxOutAccMap,
                               "RadarOutAccMap");
        }
    }

    if (ogMapMem != NULL)
    {
        free(ogMapMem);
        ogMapMem = NULL;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Instantaneous output map. */
        appCntxt->outInstMapSize =
            memReq.blks[PTK_ALG_RADAR_OGMAP_MEM_BUFF_MAP_INST_DS].size;

        ogMapMem = (uint8_t *)malloc(appCntxt->outInstMapSize);

        if (ogMapMem == NULL)
        {
            PTK_printf("[%s:%d] Memory allocation failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            memset(ogMapMem, 0, appCntxt->outInstMapSize);

            ogMap =
                PTK_Map_init(ogMapMem,
                             &mapConfig[PTK_ALG_RADAR_OGMAP_OUT_MAP_INST_DS]);

            if (ogMap == NULL)
            {
                PTK_printf("[%s:%d] PTK_Map_init() failed.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }
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

            /* Output instantaneous map. */
            appCntxt->vxOutInstMap[i] =
                vxCreateUserDataObject(appCntxt->vxContext,
                                       "InstMap",
                                       appCntxt->outInstMapSize,
                                       ogMap);

            if (appCntxt->vxOutInstMap[i] == NULL)
            {
                PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
                break;
            }
            else
            {
                vxSetReferenceName((vx_reference)appCntxt->vxOutInstMap[i],
                                   "RadarOutInstMap");
            }

            /* Output PFSD descriptor object. */
            appCntxt->vxPfsdOutDesc[i] =
                vxCreateUserDataObject(appCntxt->vxContext,
                                       "PTK_Alg_FsdPfsdPSDesc",
                                       sizeof(PTK_Alg_FsdPfsdPSDesc),
                                       NULL);

            if (appCntxt->vxPfsdOutDesc[i] == NULL)
            {
                PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
                break;
            }
            else
            {
                vxSetReferenceName((vx_reference)appCntxt->vxPfsdOutDesc[i],
                                   "PfsdOutDesc");
            }

        } // for (i = 0; i < appCntxt->pipelineDepth; i++ )
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Create the nodes. */
        appCntxt->vxOgmapNode =
            tivxRadarOgmapNode(appCntxt->vxGraph,
                               appCntxt->vxOgConfig,
                               appCntxt->vxSensorCfg[0],
                               appCntxt->vxObjData[0],
                               appCntxt->vxPoseAndRef[0],
                               appCntxt->vxPfsdOutDesc[0],
                               appCntxt->vxOutAccMap,
                               appCntxt->vxOutInstMap[0]);

        if (appCntxt->vxOgmapNode == NULL)
        {
            PTK_printf("[%s:%d] tivxRadarOgmapNode() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxOgmapNode,
                               "RadarOgmap");

            vxStatus = vxSetNodeTarget(appCntxt->vxOgmapNode,
                                       VX_TARGET_STRING,
                                       appCntxt->ogNodeCore);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] vxSetNodeTarget() failed.\n",
                           __FUNCTION__, __LINE__);
            }
        }
    }

    /* set up the pipeline. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = RADAROGAPPLIB_setupPipeline(appCntxt);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] RADAROGAPPLIB_setupPipeline() failed.\n",
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
        if (appCntxt->exportGraph == 1)
        {
            tivxExportGraphToDot(appCntxt->vxGraph, ".",
                                 "vx_applib_radar_ogmap");
        }

        if (appCntxt->rtLogEnable == 1)
        {
            tivxLogRtTraceEnable(appCntxt->vxGraph);
        }
    }

    /** Cache the reference to the accumulated map. This is persistent so it is
     * safe to cache it.
     */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appPerfPointSetName(&appCntxt->perf , "RADAR OGMAP");

        appCntxt->outAccMap =
            (PTK_Map*)ptkdemo_getUserObjDataPayload(appCntxt->vxOutAccMap);

        if (appCntxt->outAccMap == NULL)
        {
            PTK_printf("[%s:%d] ptkdemo_getUserObjDataPayload() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
    }

    if (ogMapMem != NULL)
    {
        free(ogMapMem);
    }

    return vxStatus;

}

static void RADAROGAPPLIB_releaseGraph(
        RADAROGAPPLIB_Handle    handle)
{
    RADAROGAPPLIB_Context  *appCntxt;
    uint32_t                i;

    appCntxt = (RADAROGAPPLIB_Context *)handle;

    if (appCntxt->vxOgmapNode != NULL)
    {
        /* Release the node. */
        vxReleaseNode(&appCntxt->vxOgmapNode);
    }

    /* Release the parameter memory. */
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

        if (appCntxt->vxPfsdOutDesc[i] != NULL)
        {
            vxReleaseUserDataObject(&appCntxt->vxPfsdOutDesc[i]);
        }

        if (appCntxt->vxOutInstMap[i] != NULL)
        {
            vxReleaseUserDataObject(&appCntxt->vxOutInstMap[i]);
        }
    }

    if (appCntxt->vxOgConfig != NULL)
    {
        vxReleaseUserDataObject(&appCntxt->vxOgConfig);
    }

    if (appCntxt->vxOutAccMap != NULL)
    {
        vxReleaseUserDataObject(&appCntxt->vxOutAccMap);
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

}

RADAROGAPPLIB_Handle RADAROGAPPLIB_create(
        RADAROGAPPLIB_createParams *createParams)
{
    RADAROGAPPLIB_Context *appCntxt;
    RADAROGAPPLIB_Handle   handle;
    vx_status              vxStatus;

    handle   = new RADAROGAPPLIB_Context();
    vxStatus = VX_SUCCESS;

    if (handle == NULL)
    {
        PTK_printf("[%s:%d] Could not allocate memory for internal context.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt = (RADAROGAPPLIB_Context *)handle;

        appCntxt->vxContext  = createParams->vxContext;
        appCntxt->ogPfsdCfg  = createParams->ogPfsdCfg;
        appCntxt->ogNodeCore = createParams->ogNodeCore;
        appCntxt->ogFlagEgo  = createParams->ogFlagEgo;

        if (appCntxt->ogNodeCore == NULL)
        {
            appCntxt->ogNodeCore = RADAROGAPPLIB_DEFAULT_CORE_MAPPING;
        }

        /* Get the sensor count. */
        appCntxt->numSensors =
            PTK_Util_bitCnt(appCntxt->ogPfsdCfg.ogConfig.sensorMask);

        appCntxt->vxEvtAppValBase = createParams->vxEvtAppValBase;

        /* Set the pipeline depth. */
        if (!createParams->pipelineDepth ||
            (createParams->pipelineDepth > RADAROGAPPLIB_PIPELINE_DEPTH))
        {
            PTK_printf("[%s:%d] Invalid pipeline depth value. "
                       "Passed %d. Allowed range [1..%d]\n",
                       __FUNCTION__, __LINE__,
                       createParams->pipelineDepth,
                       RADAROGAPPLIB_PIPELINE_DEPTH);

            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->pipelineDepth = createParams->pipelineDepth;
        appCntxt->exportGraph   = createParams->exportGraph;
        appCntxt->rtLogEnable   = createParams->rtLogEnable;

        /* Create Nodes. */
        vxStatus = RADAROGAPPLIB_createGraph(appCntxt);

        if (vxStatus != VX_SUCCESS)
        {
            PTK_printf("[%s:%d] RADAROGAPPLIB_createGraph() failed.\n",
                       __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus != VX_SUCCESS)
    {
        RADAROGAPPLIB_delete(&handle);
    }

    return handle;

}

vx_status RADAROGAPPLIB_delete(RADAROGAPPLIB_Handle *handle)
{
    if ((handle != NULL) && (*handle != NULL))
    {
        /* Delete the nodes. */
        RADAROGAPPLIB_releaseGraph(*handle);

        delete *handle;
        *handle = NULL;
    }

    return VX_SUCCESS;

}

vx_status RADAROGAPPLIB_setWorldReference(
        RADAROGAPPLIB_Handle        handle,
        const PTK_RigidTransform_d *ecef_w)
{
    RADAROGAPPLIB_Context *appCntxt;
    PTK_InsPoseAndRef     *poseAndRef;
    void                  *ptr;
    vx_status              vxStatus;

    appCntxt = (RADAROGAPPLIB_Context *)handle;
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

}

vx_status RADAROGAPPLIB_process(
        RADAROGAPPLIB_Handle    handle,
        uint8_t                *data,
        uint32_t                sensorId,
        uint64_t                timestamp)
{
    RADAROGAPPLIB_Context      *appCntxt;
    vx_user_data_object         obj[RADAROGAPPLIB_NUM_GRAPH_PARAMS];
    RADAROGAPPLIB_graphParams   gpDesc;
    vx_status                   vxStatus;

    appCntxt = (RADAROGAPPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = RADAROGAPPLIB_getFreeParamRsrc(appCntxt, &gpDesc);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] RADAROGAPPLIB_getFreeParamRsrc() failed.\n",
                       __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = RADAROGAPPLIB_setupOgNode(appCntxt,
                                             &gpDesc,
                                             data,
                                             sensorId,
                                             timestamp);
        
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] RADAROGAPPLIB_setupOgNode() failed.\n",
                       __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        obj[0] = gpDesc.vxSensorCfg;
        obj[1] = gpDesc.vxObjData;
        obj[2] = gpDesc.vxPoseAndRef;
        obj[3] = gpDesc.vxPfsdOutDesc;
        obj[4] = gpDesc.vxOutInstMap;

        for (uint32_t i = 0; i < RADAROGAPPLIB_NUM_GRAPH_PARAMS; i++)
        {
            vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxGraph,
                                                       i,
                                                       (vx_reference*)&obj[i],
                                                       1);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] vxGraphParameterEnqueueReadyRef(%d) "
                           "failed.\n", __FUNCTION__, __LINE__, i);
                break;
            }
        }
    }

    return vxStatus;

}

PTK_Map* RADAROGAPPLIB_getOutAccMap(
        RADAROGAPPLIB_Handle    handle)
{
    RADAROGAPPLIB_Context  *appCntxt;
    PTK_Map                *map;

    appCntxt = (RADAROGAPPLIB_Context*)handle;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        map = NULL;
    }
    else
    {
        map = appCntxt->outAccMap;
    }

    return map;
}

uint32_t RADAROGAPPLIB_getOutAccMapSize(RADAROGAPPLIB_Handle handle)
{
    RADAROGAPPLIB_Context  *appCntxt;
    uint32_t                size;

    appCntxt = (RADAROGAPPLIB_Context*)handle;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        size = 0;
    }
    else
    {
        size = appCntxt->outAccMapSize;
    }

    return size;
}

vx_status RADAROGAPPLIB_getOutBuff(
        RADAROGAPPLIB_Handle        handle,
        RADAROGAPPLIB_OutputBuff   *buff)
{
    RADAROGAPPLIB_Context      *appCntxt;
    RADAROGAPPLIB_graphParams  *desc;
    PTK_InsPoseAndRef          *poseAndRef;
    void                       *ptr;
    vx_status                   vxStatus;

    appCntxt = (RADAROGAPPLIB_Context*)handle;
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
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        poseAndRef          = (PTK_InsPoseAndRef*)ptr;
        buff->ts            = poseAndRef->curInsRec.timestamp;
        buff->outAccMap     = appCntxt->outAccMap;
        buff->vxOutInstMap  = desc->vxOutInstMap;
        buff->vxPfsdOutDesc = desc->vxPfsdOutDesc;
    }

    return vxStatus;
}

vx_status RADAROGAPPLIB_releaseOutBuff(
        RADAROGAPPLIB_Handle    handle)
{
    RADAROGAPPLIB_Context      *appCntxt;
    RADAROGAPPLIB_graphParams  *desc;
    vx_status                   vxStatus;

    appCntxt = (RADAROGAPPLIB_Context *)handle;
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

vx_status RADAROGAPPLIB_printStats(
        RADAROGAPPLIB_Handle    handle)
{
    RADAROGAPPLIB_Context  *appCntxt;
    vx_status               vxStatus;

    appCntxt = (RADAROGAPPLIB_Context *)handle;
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

vx_status RADAROGAPPLIB_exportStats(
        RADAROGAPPLIB_Handle    handle,
        FILE                   *fp,
        bool                    exportAll)
{
    RADAROGAPPLIB_Context  *appCntxt;
    vx_status               vxStatus;

    appCntxt = (RADAROGAPPLIB_Context *)handle;
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

vx_status RADAROGAPPLIB_waitGraph(
        RADAROGAPPLIB_Handle    handle)
{
    RADAROGAPPLIB_Context  *appCntxt;
    vx_status               vxStatus;

    appCntxt = (RADAROGAPPLIB_Context *)handle;
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

vx_status RADAROGAPPLIB_processEvent(
        RADAROGAPPLIB_Handle    handle,
        vx_event_t             *event)
{
    RADAROGAPPLIB_Context  *appCntxt;
    vx_status               vxStatus;

    appCntxt = (RADAROGAPPLIB_Context *)handle;
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
        vx_reference    ref[RADAROGAPPLIB_NUM_GRAPH_PARAMS];
        uint32_t        numRefs;
        uint32_t        index;
        uint32_t        appValue;
        uint32_t        i;

        appValue = appCntxt->vxEvtAppValBase +
                   RADAROGAPPLIB_GRAPH_COMPLETE_EVENT;

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
            for (i = 0; i < RADAROGAPPLIB_NUM_GRAPH_PARAMS; i++)
            {
                vxStatus = vxGraphParameterDequeueDoneRef(appCntxt->vxGraph,
                                                          i,
                                                          &ref[i],
                                                          1,
                                                          &numRefs);
                if (vxStatus != VX_SUCCESS)
                {
                    PTK_printf("[%s:%d] vxGraphParameterDequeueDoneRef() "
                               "failed\n", __FUNCTION__, __LINE__);

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
            vxStatus = RADAROGAPPLIB_releaseParamRsrc(appCntxt, index);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] RADAROGAPPLIB_releaseParamRsrc() failed.\n",
                           __FUNCTION__, __LINE__);
            }
        }
    }

    return vxStatus;

}

vx_status RADAROGAPPLIB_reset(
        RADAROGAPPLIB_Handle    handle)
{
    RADAROGAPPLIB_Context  *appCntxt;
    vx_status               vxStatus;

    appCntxt = (RADAROGAPPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }
    else
    {
        /* Reset the library. */
        vxStatus = tivxNodeSendCommand(appCntxt->vxOgmapNode,
                                       0,
                                       TIVX_KERNEL_RADAR_OGMAP_RESET,
                                       NULL,
                                       0);

        if (vxStatus != VX_SUCCESS)
        {
            PTK_printf("[%s:%d] tivxNodeSendCommand() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }

        /* Reset the accumulated map. */
        PTK_Map_clear(appCntxt->outAccMap);

        /* Reset the performance capture initialization flag. */
        appCntxt->startPerfCapt = false;
    }

    return vxStatus;
}

