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
#include <stdlib.h>
#include <mutex>

#include <TI/tivx_target_kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <tivx_utils_graph_perf.h>
#include <utils/perf_stats/include/app_perf_stats.h>

#ifdef __cplusplus
}
#endif

#include <app_ptk_demo_common.h>
#include "fused_ogmap_applib.h"

#define FUSEDOGAPPLIB_MAX_LINE_LEN   (256U)

typedef struct FUSEDOGAPPLIB_Context
{
    /* OpenVX references */
    vx_context                      vxContext;
    vx_graph                        vxGraph;

    /* Fused OGMAP Node. */
    vx_node                         fusedOgmapNode;
    vx_user_data_object             vxConfig;
    vx_user_data_object             vxPoseAndRef;
    vx_user_data_object             vxPfsdOutDesc;
    vx_user_data_object             vxRadarMap;
    vx_user_data_object             vxLidarMap;
    vx_user_data_object             vxCameraap;
    vx_user_data_object             vxFusedMap;

    vx_map_id                       pfsdOutDescMapId;
    vx_map_id                       outMapId;

    /* TIDL PSD node */
    vx_node                         tidlPsdNode;
    vx_user_data_object             vxTidlPsdGrid;
    vx_user_data_object             vxTidlPsdMap;
    vx_map_id                       tidlPsdMapId;

    /* OGMAP and PFSD configuration parameters. */
    tivx_fused_ogmap_pfsd_params_t  ogPfsdCfg;

    /** FSD enable flag. */
    uint32_t                        fsdEnable;

    /** TIDL PSD enable flag: DK */
    uint32_t                        tidlPsdEnable;

    /** Flag for indicating EGO position. */
    uint32_t                        ogFlagEgo;

    /** Grid configuration parameters. */
    uint32_t                        fusedMapSize;

    /** Map configuration. */
    PTK_MapConfig                   mapConfig;

    /** Fused grid Id. */
    uint32_t                        fusedGridId;

    /** TIDL PSD grid id: DK */
    uint32_t                        tidlPsdGridId;

    /** Fusion Node Core mapping. */
    const char                    * fusionNodeCore;

    /** FSD Node Core mapping. */
    const char                    * fsdNodeCore;

    /** PFSD Node Core mapping. */
    const char                    * pfsdNodeCore;

    /** Mutex for output buffer access. */
    std::mutex                      outBuffMutex;

    /** Mutex for PFSD output descriptor access. */
    std::mutex                      pfsdOutDescBuffMutex;

    /** Output map cache. */
    PTK_Map                        *outMap;

    /** Performance tracking context. */
    app_perf_point_t                perf;

    /** Flag to indicate if the graph should be exported
     * 0 - disable
     * 1 - enable
     */
    uint8_t                         exportGraph;

    /** Real-time logging enable.
     * 0 - disable
     * 1 - enable
     */
    uint8_t                         rtLogEnable;

} FUSEDOGAPPLIB_Context;

static vx_status FUSEDOGAPPLIB_createGraph(
        FUSEDOGAPPLIB_Context  *appCntxt)
{
    PTK_Map    *ogMap;
    uint8_t    *ogMapMem;
    uint32_t    cfgDataSize;
    vx_status   vxStatus;

    cfgDataSize = sizeof(tivx_fused_ogmap_pfsd_params_t);
    vxStatus    = VX_SUCCESS;
    ogMapMem    = NULL;

    /* Graph */
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
                           "Fused Occupancy Grid Graph");
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Dummy memory space to initialize maps */
        ogMapMem = new uint8_t[appCntxt->fusedMapSize];

        if (!ogMapMem)
        {
            printf("[%s:%d] Memory allocation failed for feedback ogmap\n",
                   __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            ogMap = PTK_Map_init(ogMapMem, &appCntxt->mapConfig);

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
        /* Configuration context. */
        appCntxt->vxConfig =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "tivx_fused_ogmap_pfsd_params_t",
                                   cfgDataSize,
                                   &appCntxt->ogPfsdCfg);

        if (appCntxt->vxConfig == NULL)
        {
            PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxConfig,
                               "FusedOgmapConfig");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Position and reference object. */
        appCntxt->vxPoseAndRef =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "PTK_InsPoseAndRef",
                                   sizeof(PTK_InsPoseAndRef),
                                   NULL);

        if (appCntxt->vxPoseAndRef == NULL)
        {
            PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxPoseAndRef,
                               "PoseAndReference");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Output PFSD descriptor object. */
        appCntxt->vxPfsdOutDesc =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "PTK_Alg_FsdPfsdPSDesc",
                                   sizeof(PTK_Alg_FsdPfsdPSDesc),
                                   NULL);

        if (appCntxt->vxPfsdOutDesc == NULL)
        {
            PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxPfsdOutDesc,
                               "PfsdOutDesc");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Output map object. */
        appCntxt->vxFusedMap =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "Output OGMAP",
                                   appCntxt->fusedMapSize,
                                   ogMap);

        if (appCntxt->vxFusedMap == NULL)
        {
            PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxFusedMap,
                               "FusedOgmapOut");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Create the nodes. */
        appCntxt->fusedOgmapNode =
            tivxFusedOgmapNode(appCntxt->vxGraph,
                               appCntxt->vxConfig,
                               NULL, // Camera
                               NULL, // Radar
                               NULL, // Lidar
                               appCntxt->vxPoseAndRef,
                               appCntxt->vxPfsdOutDesc,
                               appCntxt->vxFusedMap);

        if (appCntxt->fusedOgmapNode == NULL)
        {
            PTK_printf("[%s:%d] tivxRadarOgmapNode() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->fusedOgmapNode,
                               "FusedOgmap");

            vxStatus = vxSetNodeTarget(appCntxt->fusedOgmapNode,
                                       VX_TARGET_STRING,
                                       appCntxt->fusionNodeCore);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] vxSetNodeTarget() failed.\n",
                           __FUNCTION__, __LINE__);
            }
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
                                 "vx_applib_fused_ogmap");
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
        appPerfPointSetName(&appCntxt->perf, "FUSED OGMAP");

        appCntxt->outMap =
            (PTK_Map*)ptkdemo_getUserObjDataPayload(appCntxt->vxFusedMap);

        if (appCntxt->outMap == NULL)
        {
            PTK_printf("[%s:%d] ptkdemo_getUserObjDataPayload() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
    }

    if (ogMapMem != NULL)
    {
        delete [] ogMapMem;
    }

    return vxStatus;

}

static void FUSEDOGAPPLIB_releaseGraph(
        FUSEDOGAPPLIB_Handle    handle)
{
    FUSEDOGAPPLIB_Context *appCntxt;

    appCntxt = (FUSEDOGAPPLIB_Context *)handle;

    /* Release the node. */
    if (appCntxt->fusedOgmapNode != NULL)
    {
        vxReleaseNode(&appCntxt->fusedOgmapNode);
    }

    /* Release the parameter memory. */
    if (appCntxt->vxPfsdOutDesc != NULL)
    {
        vxReleaseUserDataObject(&appCntxt->vxPfsdOutDesc);
    }

    if (appCntxt->vxFusedMap != NULL)
    {
        vxReleaseUserDataObject(&appCntxt->vxFusedMap);
    }

    if (appCntxt->vxPoseAndRef != NULL)
    {
        vxReleaseUserDataObject(&appCntxt->vxPoseAndRef);
    }

    if (appCntxt->vxConfig != NULL)
    {
        vxReleaseUserDataObject(&appCntxt->vxConfig);
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

static vx_status FUSEDOGAPPLIB_setupOgNode(
     FUSEDOGAPPLIB_Context     *appCntxt,
     FUSEDOGAPPLIB_processReq  *req)
{
    PTK_Alg_FusedOgmapParams   *ogConfig;
    void                       *ptr;
    PTK_INS_RetCode             retCode;
    uint64_t                    ts = 0;
    vx_status                   vxStatus;

    /* Update the input parameters. */
    /* Configuration information. */
    ptr      = NULL;
    ogConfig = &appCntxt->ogPfsdCfg.ogConfig;
    ogConfig->sensorDataValidMask = req->sensorDataValidMask;

    vxStatus = vxCopyUserDataObject(appCntxt->vxConfig,
                                    0,
                                    sizeof(tivx_fused_ogmap_pfsd_params_t),
                                    &appCntxt->ogPfsdCfg,
                                    VX_WRITE_ONLY,
                                    VX_MEMORY_TYPE_HOST);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        PTK_printf("[%s:%d] vxCopyUserDataObject() failed.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Camera map. */
        if (req->sensorDataValidMask & PTK_SensorTypeMask_CAMERA)
        {
            if (req->cameraMap == NULL)
            {
                PTK_printf("[%s:%d] cameraMap NULL.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }
            else
            {
                ts = req->tsData.cameraFramsTs;

                vxStatus = vxSetParameterByIndex(appCntxt->fusedOgmapNode,
                                                 1,
                                                 (vx_reference)req->cameraMap);

                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    PTK_printf("[%s:%d] vxSetParameterByIndex() failed.\n",
                               __FUNCTION__, __LINE__);

                    vxStatus = VX_FAILURE;
                }
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Radar map. */
        if (req->sensorDataValidMask & PTK_SensorTypeMask_RADAR)
        {
            if (req->radarMap == NULL)
            {
                PTK_printf("[%s:%d] radarMap NULL.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }
            else
            {
                ts = req->tsData.radarFramsTs;

                vxStatus = vxSetParameterByIndex(appCntxt->fusedOgmapNode,
                                                 2,
                                                 (vx_reference)req->radarMap);

                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    PTK_printf("[%s:%d] vxSetParameterByIndex() failed.\n",
                               __FUNCTION__, __LINE__);

                    vxStatus = VX_FAILURE;
                }
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Lidar map. */
        if (req->sensorDataValidMask & PTK_SensorTypeMask_LIDAR)
        {
            if (req->lidarMap == NULL)
            {
                PTK_printf("[%s:%d] lidarMap NULL.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }
            else
            {
                ts = req->tsData.lidarFramsTs;

                vxStatus = vxSetParameterByIndex(appCntxt->fusedOgmapNode,
                                                 3,
                                                 (vx_reference)req->lidarMap);

                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    PTK_printf("[%s:%d] vxSetParameterByIndex() failed.\n",
                               __FUNCTION__, __LINE__);

                    vxStatus = VX_FAILURE;
                }
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Get a reference to the Pose and reference buffer. */
        ptr = ptkdemo_getUserObjDataPayload(appCntxt->vxPoseAndRef);

        if (ptr == NULL)
        {
            PTK_printf("[%s:%d] ptkdemo_getUserObjDataPayload() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        PTK_InsPoseAndRef  *poseAndRef;
        PTK_INS_Record     *curInsRec;
        PTK_INS_Record     *prevInsRec;

        poseAndRef = (PTK_InsPoseAndRef*)ptr;
        curInsRec  = &poseAndRef->curInsRec;
        prevInsRec = &poseAndRef->prevInsRec;

        /* Get the current INS record corresponding to the timestamp. */
        retCode = PTK_INS_getRecordLinearInterp(PTK_INS_RECORD_TYPE_INSPVA,
                                                ts,
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
            /* Get the previous INS record corresponding to the timestamp. */
            retCode = PTK_INS_getRecordBefore(PTK_INS_RECORD_TYPE_INSPVA,
                                              ts,
                                              prevInsRec);

            if ( retCode != PTK_INS_RETURN_CODE_OK )
            {
                PTK_printf("[%s:%d] PTK_INS_getRecordBefore() failed.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }
        }
    }

    return vxStatus;

}

FUSEDOGAPPLIB_Handle FUSEDOGAPPLIB_create(
        FUSEDOGAPPLIB_createParams *createParams)
{
    FUSEDOGAPPLIB_Context      *appCntxt;
    PTK_Alg_FusedOgmapParams   *ogConfig;
    FUSEDOGAPPLIB_Handle        handle;
    int32_t                     status;
    vx_status                   vxStatus;

    handle   = new FUSEDOGAPPLIB_Context();
    vxStatus = VX_SUCCESS;

    if (handle == NULL)
    {
        PTK_printf("[%s:%d] Could not allocate memory for internal context.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt = (FUSEDOGAPPLIB_Context *)handle;

        appCntxt->ogPfsdCfg = createParams->ogPfsdCfg;
        ogConfig            = &appCntxt->ogPfsdCfg.ogConfig;

        status = PTK_Alg_FusedOgmapGetMapConfig(ogConfig, &appCntxt->mapConfig);

        if (status != PTK_ALG_RET_SUCCESS)
        {
            PTK_printf("[%s:%d] PTK_Alg_FusedOgmapGetMapConfig() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->vxContext       = createParams->vxContext;
        appCntxt->exportGraph     = createParams->exportGraph;
        appCntxt->rtLogEnable     = createParams->rtLogEnable;
        appCntxt->fusedMapSize    = PTK_Map_getSize(&appCntxt->mapConfig);
        appCntxt->fusionNodeCore  = createParams->fusionNodeCore;

        if (appCntxt->fusionNodeCore == NULL)
        {
            appCntxt->fusionNodeCore = FUSEDOGAPPLIB_DEFAULT_CORE_MAPPING;
        }

        /* Create Nodes. */
        vxStatus = FUSEDOGAPPLIB_createGraph(appCntxt);

        if (vxStatus != VX_SUCCESS)
        {
            PTK_printf("[%s:%d] FUSEDOGAPPLIB_createGraph() failed.\n",
                       __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus != VX_SUCCESS)
    {
        FUSEDOGAPPLIB_delete(&handle);
    }

    return handle;

}

vx_status FUSEDOGAPPLIB_delete(
        FUSEDOGAPPLIB_Handle *handle)
{
    if ((handle != NULL) && (*handle != NULL))
    {
        /* Delete the nodes. */
        FUSEDOGAPPLIB_releaseGraph(*handle);

        delete *handle;
        *handle = NULL;
    }

    return VX_SUCCESS;

}

vx_status FUSEDOGAPPLIB_setWorldReference(
        FUSEDOGAPPLIB_Handle        handle,
        const PTK_RigidTransform_d *ecef_w)
{
    FUSEDOGAPPLIB_Context *appCntxt;
    PTK_InsPoseAndRef     *poseAndRef;
    void                  *ptr;
    vx_status              vxStatus;

    appCntxt = (FUSEDOGAPPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Get a reference to the Pose and reference buffer. */
        ptr = ptkdemo_getUserObjDataPayload(appCntxt->vxPoseAndRef);

        if (ptr == NULL)
        {
            PTK_printf("[%s:%d] ptkdemo_getUserObjDataPayload() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        poseAndRef         = (PTK_InsPoseAndRef*)ptr;
        poseAndRef->w2Ecef = *ecef_w;
    }

    return vxStatus;

}

vx_status FUSEDOGAPPLIB_process(
        FUSEDOGAPPLIB_Handle        handle,
        FUSEDOGAPPLIB_processReq   *req)
{
    FUSEDOGAPPLIB_Context  *appCntxt;
    vx_status               vxStatus;

    appCntxt = (FUSEDOGAPPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Setup the OGMAP node. */
        vxStatus = FUSEDOGAPPLIB_setupOgNode(appCntxt, req);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] FUSEDOGAPPLIB_setupOgNode() failed.\n",
                       __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appPerfPointBegin(&appCntxt->perf);

        vxStatus = vxScheduleGraph(appCntxt->vxGraph);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] vxScheduleGraph() failed.\n",
                       __FUNCTION__, __LINE__);
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            vxStatus = vxWaitGraph(appCntxt->vxGraph);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] vxWaitGraph() failed.\n",
                           __FUNCTION__, __LINE__);
            }
        }

        appPerfPointEnd(&appCntxt->perf);
    }

    return vxStatus;

}

PTK_Map* FUSEDOGAPPLIB_getOutMap(
        FUSEDOGAPPLIB_Handle    handle)
{
    FUSEDOGAPPLIB_Context  *appCntxt;
    PTK_Map                *map;

    appCntxt = (FUSEDOGAPPLIB_Context *)handle;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        map = NULL;
    }
    else
    {
        map = appCntxt->outMap;
    }

    return map;

}

uint32_t FUSEDOGAPPLIB_getOutMapSize(
        FUSEDOGAPPLIB_Handle    handle)
{
    FUSEDOGAPPLIB_Context *appCntxt;
    uint32_t               size;

    appCntxt = (FUSEDOGAPPLIB_Context *)handle;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        size = 0;
    }
    else
    {
        size = PTK_Map_getSize(&appCntxt->mapConfig);
    }

    return size;
}

vx_status FUSEDOGAPPLIB_printStats(
        FUSEDOGAPPLIB_Handle    handle)
{
    FUSEDOGAPPLIB_Context  *appCntxt;
    vx_status               vxStatus;

    appCntxt = (FUSEDOGAPPLIB_Context *)handle;
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

vx_status FUSEDOGAPPLIB_exportStats(
        FUSEDOGAPPLIB_Handle    handle,
        FILE                   *fp,
        bool                    exportAll)
{
    FUSEDOGAPPLIB_Context  *appCntxt;
    vx_status               vxStatus;

    appCntxt = (FUSEDOGAPPLIB_Context *)handle;
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

vx_status FUSEDOGAPPLIB_reset(
        FUSEDOGAPPLIB_Handle    handle)
{
    FUSEDOGAPPLIB_Context  *appCntxt;
    vx_status               vxStatus;

    appCntxt = (FUSEDOGAPPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Reset the accumulated map. */
        PTK_Map_clear(appCntxt->outMap);
    }

    return vxStatus;
}

