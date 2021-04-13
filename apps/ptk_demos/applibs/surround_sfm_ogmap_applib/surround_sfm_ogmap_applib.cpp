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

#include "surround_sfm_ogmap_priv.h"

static void SFMOGAPPLIB_dof2tracksLoadDof(SFMOGAPPLIB_Context *appCntxt, SFMOGAPPLIB_graphParams *gpDesc, void * dataPtr);


static int32_t SFMOGAPPLIB_setParams(SFMOGAPPLIB_Context       *appCntxt,
                                     SFMOGAPPLIB_createParams  *createParams)
{
    int32_t     status = 0;

    /* Set the pipeline depth. */
    if (!createParams->pipelineDepth ||
        (createParams->pipelineDepth > SFMOGAPPLIB_PIPELINE_DEPTH))
    {
        PTK_printf("[%s:%d] Invalid pipeline depth value. "
                   "Passed %d. Allowed range [1..%d]\n",
                   __FUNCTION__,
                   __LINE__,
                   createParams->pipelineDepth,
                   SFMOGAPPLIB_PIPELINE_DEPTH);

        status = -1;
    }

    if (status == 0)
    {
        appCntxt->pipelineDepth = createParams->pipelineDepth;
        appCntxt->exportGraph   = createParams->exportGraph;
        appCntxt->rtLogEnable   = createParams->rtLogEnable;

        appCntxt->vxContext  = createParams->vxContext;
        appCntxt->generatePC = createParams->generatePC;
        appCntxt->enableDof  = createParams->enableDof;
        appCntxt->ogPfsdCfg  = createParams->ogPfsdCfg;
        appCntxt->M_c_e      = createParams->M_c_e;

        appCntxt->vxEvtAppValBase = createParams->vxEvtAppValBase;

        // when we need to generate point clouds
        if (appCntxt->generatePC)
        {
            if (appCntxt->enableDof)
            {
                appCntxt->dofCfg = createParams->dofCfg;
            }

            appCntxt->dtNodeCfg   = createParams->dtNodeCfg;
            appCntxt->trNodeCfg   = createParams->trNodeCfg;
            appCntxt->dofLevels   = createParams->dofLevels;
            appCntxt->dofWidth    = createParams->dofWidth;
            appCntxt->dofHeight   = createParams->dofHeight;
        }

        /* maximum number of tracks */
        appCntxt->maxNumTracks = createParams->maxNumTracks;

        /* References frames */
        PTK_RigidTransform_invert(&appCntxt->M_e_c, &appCntxt->M_c_e);

        /* Note: Md_ecef_w set by calling application through
         * SFMOGAPPLIB_setWorldReference API */
        /* Note: M_w_e set at run-time per process call*/

        appCntxt->ogNodeCore = createParams->ogNodeCore;
        appCntxt->triNodeCore = createParams->triNodeCore;
        appCntxt->dofTrackNodeCore = createParams->dofTrackNodeCore;

        if (appCntxt->ogNodeCore == NULL)
        {
            appCntxt->ogNodeCore = SFMOGAPPLIB_DEFAULT_CORE_MAPPING;
        }

        if (appCntxt->triNodeCore == NULL)
        {
            appCntxt->triNodeCore = SFMOGAPPLIB_DEFAULT_CORE_MAPPING;
        }

        if (appCntxt->dofTrackNodeCore == NULL)
        {
            appCntxt->dofTrackNodeCore = SFMOGAPPLIB_DEFAULT_CORE_MAPPING;
        }
    }

    return status;
}

int32_t SFMOGAPPLIB_process(SFMOGAPPLIB_Handle handle,
                            void              *data,
                            uint32_t           sensorId,
                            uint64_t           timestamp)
{
    SFMOGAPPLIB_Context        *appCntxt;
    vx_user_data_object         obj[SFMOGAPPLIB_NUM_GRAPH_PARAMS];
    SFMOGAPPLIB_graphParams     gpDesc;
    vx_status                   vxStatus;
    int32_t                     status;
    uint32_t                    i;
    uint32_t                    count;
    static uint32_t             firstCall = 1;

    appCntxt = (SFMOGAPPLIB_Context *)handle;

    status = SFMOGAPPLIB_getFreeParamRsrc(appCntxt, &gpDesc);

    if (status < 0)
    {
        /* No free descriptors available. */
        PTK_printf("[%s:%d] SFMOGAPPLIB_getFreeParamRsrc() failed\n",
                   __FUNCTION__, __LINE__);

        return status;
    }

    if (status == 0)
    {
        status = SFMOGAPPLIB_setupNodes(appCntxt, &gpDesc, (uint8_t *)data, timestamp);
        if (status < 0)
        {
            PTK_printf("[%s:%d] SFMOGAPPLIB_setupNodes() failed.\n",
                       __FUNCTION__, __LINE__);

            return status;
        }
    }

    count = 0;
    if (appCntxt->generatePC)
    {
        if( appCntxt->enableDof)
        {
            vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxSfmGraph,
                                                       count,
                                                       (vx_reference*)&gpDesc.pyr_in_image,
                                                       1);
            PTK_assert(VX_SUCCESS == vxStatus);
            count++;
        }
        else
        {
            vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxSfmGraph,
                                                       count,
                                                       (vx_reference*)&gpDesc.dof2tracks_in_field,
                                                       1);
            PTK_assert(VX_SUCCESS == vxStatus);
            count++;
        }

        vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxSfmGraph,
                                                   count,
                                                   (vx_reference*)&gpDesc.triang_in_pose,
                                                   1);
        PTK_assert(VX_SUCCESS == vxStatus);
        count++;
    } else
    {
        vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxSfmGraph,
                                                   count,
                                                   (vx_reference*)&gpDesc.vxOgInPoints3d,
                                                   1);
        PTK_assert(VX_SUCCESS == vxStatus);
        count++;
    }

    obj[count]   = gpDesc.points3d_transform;
    obj[count+1] = gpDesc.vxOgInPose;
    obj[count+2] = gpDesc.vxPoseAndRef;
    obj[count+3] = gpDesc.vxPfsdOutDesc;
    obj[count+4] = gpDesc.vxOutInstMap;

    for (i = count; i < count+5; i++)
    {
        vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxSfmGraph,
                                                   i,
                                                   (vx_reference*)&obj[i],
                                                   1);
        PTK_assert(VX_SUCCESS == vxStatus);
    }

    if (firstCall)
    {
        firstCall = 0;
        appCntxt ->timestampPrv = timestamp;
        return status;
    }

    appCntxt->timestampPrv = timestamp;

    return status;

} /* SFMOGAPPLIB_process */

SFMOGAPPLIB_Handle SFMOGAPPLIB_create(SFMOGAPPLIB_createParams *createParams)
{
    SFMOGAPPLIB_Context    *appCntxt;
    SFMOGAPPLIB_Handle      handle;
    PTK_PointCloudConfig    cfg;
    vx_status               vxStatus;
    size_t                  pcSize;
    int32_t                 status;

    handle = new SFMOGAPPLIB_Context();

    if (handle == NULL)
    {
        PTK_printf("[%s:%d] Could not allocate memory for internal context.\n",
                   __FUNCTION__,
                   __LINE__);

        return NULL;
    }

    appCntxt = (SFMOGAPPLIB_Context *)handle;

    /* Set applib-level create parameters */
    status = SFMOGAPPLIB_setParams(appCntxt, createParams);

    if (status < 0)
    {
        PTK_printf("[%s:%d] SFMOGAPPLIB_setParams() failed.\n",
                   __FUNCTION__,
                   __LINE__);

        delete handle;
        return NULL;
    }

    /* create memory for point cloud process */
    cfg.maxPoints = appCntxt->maxNumTracks;
    pcSize        = PTK_PointCloud_getSize(&cfg);

    appCntxt->pcMemPtr = new uint8_t[pcSize];

    if (appCntxt->pcMemPtr  == NULL)
    {
        PTK_printf("[%s:%d] Memory allocation failed.\n",
                   __FUNCTION__,
                   __LINE__);

        vxStatus = VX_FAILURE;
    }

    /* Create nodes and graph */
    vxStatus = SFMOGAPPLIB_createGraph(appCntxt);

    if (vxStatus == VX_SUCCESS)
    {
        appCntxt = (SFMOGAPPLIB_Context *)handle;
    }

    if (vxStatus != VX_SUCCESS)
    {
        delete handle;
        handle = NULL;
    }

    return handle;

} /* SFMOGAPPLIB_create */

void SFMOGAPPLIB_delete(SFMOGAPPLIB_Handle *handle)
{
    if (*handle)
    {
        SFMOGAPPLIB_Context * appCntxt = (SFMOGAPPLIB_Context *)*handle;

        SFMOGAPPLIB_releaseGraph(appCntxt);

        /* delete memory for point cloud process */
        if (appCntxt->pcMemPtr  != NULL)
        {
            delete [] appCntxt->pcMemPtr;
        }

        delete *handle;
        *handle = NULL;
    }

    return;

} /* SFMOGAPPLIB_delete */

void SFMOGAPPLIB_setWorldReference(SFMOGAPPLIB_Handle          handle,
                                   const PTK_RigidTransform_d *ecef_w)
{
    SFMOGAPPLIB_Context *appCntxt;
    PTK_InsPoseAndRef   *poseAndRef;
    void                *ptr;

    appCntxt = (SFMOGAPPLIB_Context *)handle;

    for (uint32_t i = 0; i < appCntxt->pipelineDepth; i++)
    {
        /* Get a reference to the Pose and reference buffer. */
        ptr = ptkdemo_getUserObjDataPayload(appCntxt->vxPoseAndRef[i]);

        if (ptr == NULL)
        {
            PTK_printf("[%s:%d] ptkdemo_getUserObjDataPayload() failed.\n",
                       __FUNCTION__,
                       __LINE__);

            return;
        }

        poseAndRef          = (PTK_InsPoseAndRef*)ptr;
        poseAndRef->w2Ecef  = *ecef_w;
    }

    appCntxt->Md_ecef_w = *ecef_w;

    return;

} /* SFMOGAPPLIB_setWorldReference */


PTK_Map* SFMOGAPPLIB_getOutAccMap(SFMOGAPPLIB_Handle handle)
{
    SFMOGAPPLIB_Context  *appCntxt;

    appCntxt = (SFMOGAPPLIB_Context*)handle;

    return appCntxt->outAccMap;
}

uint32_t SFMOGAPPLIB_getOutAccMapSize(SFMOGAPPLIB_Handle handle)
{
    SFMOGAPPLIB_Context *appCntxt;

    appCntxt = (SFMOGAPPLIB_Context *)handle;

    return appCntxt->outAccMapSize;

} /* SFMOGAPPLIB_getOutAccMapSize */

uint32_t SFMOGAPPLIB_getOutInstMapSize(SFMOGAPPLIB_Handle handle)
{
    SFMOGAPPLIB_Context  *appCntxt;

    appCntxt = (SFMOGAPPLIB_Context*)handle;

    return appCntxt->outInstMapSize;
}

int32_t SFMOGAPPLIB_getOutBuff(SFMOGAPPLIB_Handle       handle,
                               SFMOGAPPLIB_OutputBuff  *buff)
{
    SFMOGAPPLIB_Context        *appCntxt;
    SFMOGAPPLIB_graphParams    *desc;
    PTK_InsPoseAndRef          *poseAndRef;
    void                       *ptr;

    appCntxt = (SFMOGAPPLIB_Context*)handle;
    std::unique_lock<std::mutex>    lock(appCntxt->paramRsrcMutex);

    if (appCntxt->outputQ.empty())
    {
        return -1;
    }

    /* Get the descriptor. */
    desc = appCntxt->outputQ.front();

    /* Get a reference to the Pose and reference buffer. */
    ptr = ptkdemo_getUserObjDataPayload(desc->vxPoseAndRef);

    if (ptr == NULL)
    {
        PTK_printf("[%s:%d] ptkdemo_getUserObjDataPayload() failed.\n",
                   __FUNCTION__,
                   __LINE__);

        return -1;
    }

    poseAndRef          = (PTK_InsPoseAndRef*)ptr;
    buff->ts            = poseAndRef->curInsRec.timestamp;
    buff->outAccMap     = appCntxt->outAccMap;
    buff->vxOutInstMap  = desc->vxOutInstMap;
    buff->vxPfsdOutDesc = desc->vxPfsdOutDesc;

    return 0;
}

void SFMOGAPPLIB_releaseOutBuff(SFMOGAPPLIB_Handle handle)
{
    SFMOGAPPLIB_Context        *appCntxt;
    SFMOGAPPLIB_graphParams    *desc;

    appCntxt = (SFMOGAPPLIB_Context *)handle;
    std::unique_lock<std::mutex>    lock(appCntxt->paramRsrcMutex);

    if (appCntxt->outputQ.empty())
    {
        PTK_printf("[%s:%d] No output buffers available.\n",
                   __FUNCTION__, __LINE__);

        return;
    }

    desc = appCntxt->outputQ.front();
    appCntxt->outputQ.pop();

    /* Push the descriptor into the free queue. */
    appCntxt->freeQ.push(desc);
}

int32_t SFMOGAPPLIB_set_ego_to_world_pose(SFMOGAPPLIB_Context    *appCntxt,
                                          SFMOGAPPLIB_graphParams *gpDesc,
                                          uint64_t timestamp)
{
    PTK_InsPoseAndRef      *poseAndRef;
    PTK_INS_Record         *curInsRec;
    PTK_INS_Record         *prevInsRec;
    void                   *ptr;
    PTK_RigidTransform_d    Md_enu_ecef;
    PTK_RigidTransform_d    Md_enu_w;
    PTK_RigidTransform      M_enu_w;
    PTK_RigidTransform      M_w_enu;
    PTK_RigidTransform      M_enu_e;
    PTK_INS_RetCode         retCode;

    /* Get a reference to the Pose and reference buffer. */
    ptr = ptkdemo_getUserObjDataPayload(gpDesc->vxPoseAndRef);

    if (ptr == NULL)
    {
        PTK_printf("[%s:%d] ptkdemo_getUserObjDataPayload() failed.\n",
                   __FUNCTION__,
                   __LINE__);

        return -1;
    }

    poseAndRef = (PTK_InsPoseAndRef*)ptr;
    curInsRec  = &poseAndRef->curInsRec;
    prevInsRec = &poseAndRef->prevInsRec;

    retCode = PTK_INS_getRecordLinearInterp(PTK_INS_RECORD_TYPE_INSPVA,
                                            timestamp,
                                            curInsRec);

    if (( retCode != PTK_INS_RETURN_CODE_OK ) &&
        ( retCode != PTK_INS_RETURN_CODE_RECORD_PARTIALLY_AVAIL ))
    {
        return -1;
    }

    retCode = PTK_INS_getRecordBefore(PTK_INS_RECORD_TYPE_INSPVA,
                                      timestamp,
                                      prevInsRec);

    if ( retCode != PTK_INS_RETURN_CODE_OK )
    {
        return -1;
    }

    PTK_Position_getECEFtoENU_d(&curInsRec->data.inspva.position, &Md_enu_ecef);
    PTK_RigidTransform_d_compose(&Md_enu_w, &Md_enu_ecef, &appCntxt->Md_ecef_w);
    PTK_RigidTransform_d_convertToSingle(&M_enu_w, &Md_enu_w);
    PTK_RigidTransform_invert(&M_w_enu, &M_enu_w);
    PTK_INS_getIMUtoENU(curInsRec, &M_enu_e);
    PTK_RigidTransform_compose(&appCntxt->M_w_e, &M_w_enu, &M_enu_e);
    PTK_RigidTransform_invert(&appCntxt->M_e_w, &appCntxt->M_w_e);

    return 0;
}

int32_t SFMOGAPPLIB_setupNodes(SFMOGAPPLIB_Context *appCntxt,
                               SFMOGAPPLIB_graphParams  *gpDesc,
                               uint8_t *data,
                               uint64_t timestamp)
{
    vx_status vxStatus;

    /* load ego pose */
    SFMOGAPPLIB_set_ego_to_world_pose(appCntxt, gpDesc, timestamp);

    if (appCntxt->generatePC)
    {
        if(appCntxt->enableDof)
        {
            vxStatus = tivx_utils_load_vximage_from_bmpfile(gpDesc->pyr_in_image, (char *)data, vx_true_e);
            if (vxStatus != VX_SUCCESS)
            {
                PTK_printf("[%s:%d] SFMOGAPPLIB_setupNodes() failed - Cannot read input image.\n",
                           __FUNCTION__, __LINE__);
                return -1;
            }
        } 
        else
        {
            /* load DOF file */
            SFMOGAPPLIB_dof2tracksLoadDof(appCntxt, gpDesc, (uint8_t *)data);
        }

        /* load triangulation poses */
        SFMOGAPPLIB_triangLoadCameraPoses(appCntxt, gpDesc, appCntxt->timestampPrv, timestamp);

    } else
    {
        PTK_PointCloud *pc = (PTK_PointCloud *)data;
        if (pc == NULL)
        {
            PTK_printf("[%s:%d] pc NULL.\n", __FUNCTION__, __LINE__);
            return -1;
        }

        SFMOGAPPLIB_load3DPoints(appCntxt, gpDesc, pc);
    }

    /* set 3D points transform matrix */
    SFMOGAPPLIB_setPoints3DTransform(appCntxt, gpDesc);

    /* load ego pose */
    SFMOGAPPLIB_loadEgoPose(appCntxt, gpDesc);


    return 0;
}

void SFMOGAPPLIB_dof2tracksLoadDof(SFMOGAPPLIB_Context *appCntxt, SFMOGAPPLIB_graphParams *gpDesc, void * dataPtr)
{
    vx_status status;
    vx_map_id map_id;
    void *ptr;
    vx_rectangle_t rect;
    vx_imagepatch_addressing_t addr = VX_IMAGEPATCH_ADDR_INIT;

    /* open OVX image for writing */
    rect.start_x = 0;
    rect.end_x = appCntxt->dofWidth;
    rect.start_y = 0;
    rect.end_y = appCntxt->dofHeight;

    status = vxMapImagePatch(gpDesc->dof2tracks_in_field, &rect, (vx_uint32)0, &map_id, &addr, (void **)&ptr,  VX_READ_AND_WRITE, VX_MEMORY_TYPE_HOST, VX_NOGAP_X);
    PTK_assert(VX_SUCCESS == status);

    status = vxCopyImagePatch(gpDesc->dof2tracks_in_field, &rect, (vx_uint32)0, &addr, dataPtr, VX_WRITE_ONLY,  VX_MEMORY_TYPE_HOST);
    PTK_assert(VX_SUCCESS == status);

    status = vxUnmapImagePatch(gpDesc->dof2tracks_in_field, map_id);
    PTK_assert(VX_SUCCESS == status);

    return;
}


void SFMOGAPPLIB_triangLoadCameraPoses(SFMOGAPPLIB_Context *appCntxt,
                                       SFMOGAPPLIB_graphParams *gpDesc,
                                       uint64_t prvTimestamp,
                                       uint64_t curTimestamp)
{
    vx_status status;
    tivx_triangulation_pose_t poses;

    //estimate ego pose (0=prv, 1=cur)
    #define TRIANGULATION_MAX_TRACKS_LENGTH (2) //doing only 2-view triangulation in this demo
    uint64_t timestamps[TRIANGULATION_MAX_TRACKS_LENGTH];
    PTK_RigidTransform M_en_e0[TRIANGULATION_MAX_TRACKS_LENGTH];
    uint32_t numViews = TRIANGULATION_MAX_TRACKS_LENGTH;
    timestamps[0] = prvTimestamp;
    timestamps[1] = curTimestamp;

    //PTK_INS_RetCode imu_status = PTK_INS_getIMUPosesFromVelAndAtt(timestamps, numViews, M_en_e0);
    PTK_INS_getIMUPosesFromVelAndAtt(timestamps, numViews, M_en_e0);

    //if (PTK_INS_RETURN_CODE_OK != imu_status)
    //{
    //    PTK_printf("WARNING: PTK INS return code (%d) NOT OK (getIMUPosesFromVelAndAtt at timestamp %" PRId64 ")\n", imu_status, timestamps[0]);
    //}

    //use calbration to convert ego-pose to camera poses 1=cur, 0=prv, ref=cur camera
    PTK_RigidTransform M_c1_ref;
    PTK_RigidTransform M_c0_ref;

    PTK_RigidTransform_makeIdentity(&M_c1_ref);
    PTK_RigidTransform M_e0_e1;
    PTK_RigidTransform_invert(&M_e0_e1, &M_en_e0[1]);
    PTK_RigidTransform M_c0_e1;
    PTK_RigidTransform_compose(&M_c0_e1, &appCntxt->M_c_e, &M_e0_e1);
    PTK_RigidTransform_compose(&M_c0_ref, &M_c0_e1, &appCntxt->M_e_c);

    poses.view[0] = M_c0_ref;
    poses.view[1] = M_c1_ref;

    status = vxCopyUserDataObject(gpDesc->triang_in_pose,
                                  0,
                                  sizeof(tivx_triangulation_pose_t),
                                  &poses,
                                  VX_WRITE_ONLY,
                                  VX_MEMORY_TYPE_HOST);

    PTK_assert(VX_SUCCESS == status);

    return;
}

void SFMOGAPPLIB_loadEgoPose(SFMOGAPPLIB_Context *appCntxt, SFMOGAPPLIB_graphParams *gpDesc)
{
    vx_status status;

    /* load poses */
    status = vxCopyUserDataObject(gpDesc->vxOgInPose,
                                  0,
                                  sizeof(PTK_RigidTransform),
                                  &appCntxt->M_w_e,
                                  VX_WRITE_ONLY,
                                  VX_MEMORY_TYPE_HOST);

    PTK_assert(VX_SUCCESS == status);

    return;
}

void SFMOGAPPLIB_setPoints3DTransform(SFMOGAPPLIB_Context *appCntxt, SFMOGAPPLIB_graphParams *gpDesc)
{
    vx_status status;

    PTK_RigidTransform M_w_c;
    PTK_RigidTransform_compose(&M_w_c, &appCntxt->M_w_e, &appCntxt->M_e_c);

    /* load 3D points transform */
    status = vxCopyUserDataObject(gpDesc->points3d_transform,
                                  0,
                                  sizeof(PTK_RigidTransform),
                                  &M_w_c,
                                  VX_WRITE_ONLY,
                                  VX_MEMORY_TYPE_HOST);
    PTK_assert(VX_SUCCESS == status);

    return;
}

void SFMOGAPPLIB_load3DPoints(SFMOGAPPLIB_Context *appCntxt, SFMOGAPPLIB_graphParams *gpDesc, PTK_PointCloud *pcPtr)
{
    vx_status status;

    status = vxTruncateArray(gpDesc->vxOgInPoints3d, 0);
    PTK_assert(VX_SUCCESS == status);

    status = vxAddArrayItems(gpDesc->vxOgInPoints3d,
                             pcPtr->numPoints,
                             (void *)PTK_PointCloud_getPoints(pcPtr),
                             sizeof(PTK_Point));

    PTK_assert(VX_SUCCESS == status);

    return;
}

vx_graph SFMOGAPPLIB_getSfmGraphHandle(SFMOGAPPLIB_Handle handle)
{
    SFMOGAPPLIB_Context *appCntxt;

    appCntxt = (SFMOGAPPLIB_Context *)handle;

    return appCntxt->vxSfmGraph;

}

void SFMOGAPPLIB_printStats(SFMOGAPPLIB_Handle handle)
{
    SFMOGAPPLIB_Context    *appCntxt;

    appCntxt = (SFMOGAPPLIB_Context *)handle;

    tivx_utils_graph_perf_print(appCntxt->vxSfmGraph);
    appPerfPointPrint(&appCntxt->perf);
    appPerfPointPrintFPS(&appCntxt->perf);

}

void SFMOGAPPLIB_exportStats(SFMOGAPPLIB_Handle handle, FILE *fp, bool exportAll)
{
    SFMOGAPPLIB_Context    *appCntxt;

    appCntxt = (SFMOGAPPLIB_Context *)handle;

    tivx_utils_graph_perf_export(fp, appCntxt->vxSfmGraph);

    if (exportAll == true)
    {
        app_perf_point_t *perfArr[1] = {&appCntxt->perf};
        appPerfStatsExportAll(fp, perfArr, 1);
    }
}

int32_t SFMOGAPPLIB_releaseParamRsrc(
        SFMOGAPPLIB_Context  *appCntxt,
        uint32_t              rsrcIndex)
{
    SFMOGAPPLIB_graphParams        *desc;
    std::unique_lock<std::mutex>   lock(appCntxt->paramRsrcMutex);

    desc = &appCntxt->paramDesc[rsrcIndex];

    appCntxt->outputQ.push(desc);

    return 0;
}

int32_t SFMOGAPPLIB_getFreeParamRsrc(
        SFMOGAPPLIB_Context      *appCntxt,
        SFMOGAPPLIB_graphParams  *gpDesc)
{
    std::unique_lock<std::mutex>    lock(appCntxt->paramRsrcMutex);

    /* Check if we have free og node descriptors available. */
    if (appCntxt->freeQ.empty())
    {
        return -1;
    }

    *gpDesc = *appCntxt->freeQ.front();
    appCntxt->freeQ.pop();

    return 0;
}

int32_t SFMOGAPPLIB_processEvent(SFMOGAPPLIB_Handle     handle,
                                 vx_event_t            *event)
{
    SFMOGAPPLIB_Context    *appCntxt;
    vx_reference            ref;
    uint32_t                numRefs;
    uint32_t                index;
    int32_t                 status;
    vx_status               vxStatus;

    appCntxt = (SFMOGAPPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if(event->type == VX_EVENT_GRAPH_COMPLETED)
    {
        uint32_t    i;

        if (appCntxt->startPerfCapt == false)
        {
            appCntxt->startPerfCapt = true;
        }
        else
        {
            appPerfPointEnd(&appCntxt->perf);
        }

        appPerfPointBegin(&appCntxt->perf);

        /* Node execution is complete. Deque all the parameters
         * for this node.
         */
        for (i = 0; i < appCntxt->effectiveNumParams; i++)
        {
            vxStatus = vxGraphParameterDequeueDoneRef(appCntxt->vxSfmGraph,
                                                      i,
                                                      &ref,
                                                      1,
                                                      &numRefs);
            if (vxStatus != VX_SUCCESS)
            {
                PTK_printf("[%s:%d] vxGraphParameterDequeueDoneRef() failed\n",
                           __FUNCTION__, __LINE__);

                return -1;
            }
        }

        /* The last one to deque is vxOutInstMap parameter. Search and
         * identify the resource index.
         */
        index = appCntxt->pipelineDepth;
        for (uint32_t i = 0; i < appCntxt->pipelineDepth; i++)
        {
            if (ref == (vx_reference)appCntxt->vxOutInstMap[i])
            {
                index = i;
                break;
            }
        }

        if (index == appCntxt->pipelineDepth)
        {
            PTK_printf("[%s:%d] Resource look up failed\n",
                       __FUNCTION__, __LINE__);

            return -1;
        }

        /* Mark the dequeued resource as free. */
        status = SFMOGAPPLIB_releaseParamRsrc(appCntxt, index);

        if (status < 0)
        {
            PTK_printf("[%s:%d] SFMOGAPPLIB_releaseParamRsrc() failed.\n",
                       __FUNCTION__,
                       __LINE__);

            return -1;
        }
    }

    return 0;

} /* SFMOGAPPLIB_processEvent. */

void SFMOGAPPLIB_waitGraph(SFMOGAPPLIB_Handle handle)
{
    SFMOGAPPLIB_Context *appCntxt;

    appCntxt = (SFMOGAPPLIB_Context *)handle;

    vxWaitGraph(appCntxt->vxSfmGraph);

    /* Wait for the output queue to get flushed. */
    while (appCntxt->freeQ.size() != appCntxt->pipelineDepth)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

