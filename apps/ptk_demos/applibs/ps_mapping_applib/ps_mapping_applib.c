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
#include <assert.h>

#include "TI/tivx_target_kernel.h"
#include "ps_mapping_applib.h"

#include "itidl_ti.h"

typedef struct
{
    vx_context                  vxContext;
    vx_graph                    vxGraph;
    vx_node                     vxPsMappingNode;
    vx_user_data_object         vxPsMappingConfig;
    vx_user_data_object         vxDataPtr;
    vx_user_data_object         vxPoses;
    vx_user_data_object         vxInMap;
    vx_user_data_object         vxOutMap;
    vx_map_id                   vxInMap_mapId;
    vx_map_id                   vxOutMap_mapId;
    vx_lut                      vxD2ULUT;
    vx_lut                      vxProjMatrix;

    /* Reference frame. ECEF-world */
    PTK_RigidTransform_d        Md_ecef_w;

    vx_uint32                   ogMapSize;
    /* input parking spots */
    vx_uint32                   inputSpotSize;
    tivx_ps_mapping_input_t     inputSpots;

    /* OG Map configuration */
    tivx_ps_mapping_config_t    psMappingNodeConfig;

    /* ego vehicle poses */
    tivx_ps_mapping_pose_t      ps_mapping_pose;

    /** camera projection matrix */
    double                    * projMatrix;

    /** camera projection matrix size */
    vx_uint32                   projMatrixSize;

    /** camera distortion correction table */
    float                     * d2uLUT;

    /** camera distortion correction table size */
    vx_uint32                   d2uLUTSize;

    /** PS Mapping Node Core mapping. */
    const char                * psNodeCore;

    /** Mutex for output buffer access. */
    tivx_mutex                  outBuffMutex;

} PSLIB_Context;


const char* label_id_name_map[3] = {"background","parking_empty","parking_occupied"};


static void PSLIB_setParams(PSLIB_Handle handle, PSLIB_createParams *createParams);
static vx_status PSLIB_createGraph(PSLIB_Handle handle);
static void PSLIB_releaseGraph(PSLIB_Handle handle);

static void PSLIB_set_ego_to_world_pose(PSLIB_Handle handle, uint64_t timestamp);

static void PSLIB_setupPsMappingNode(PSLIB_Handle handle, void *dataPtr);

static PTK_Map *PSLIB_mapInput(PSLIB_Handle handle);
static void     PSLIB_unmapInput(PSLIB_Handle handle);

void PSLIB_process(PSLIB_Handle handle, uint8_t* data, uint32_t sensorId, uint64_t timestamp)
{
    PSLIB_Context  *appCntxt = (PSLIB_Context *)handle;
    vx_status       vxStatus;

    PSLIB_set_ego_to_world_pose(handle, timestamp);

    /* set up nodes */
    PSLIB_setupPsMappingNode(handle, data);


    /*copy output map to input map (feedback loop) */
    PTK_Map *outMap = PSLIB_mapOutput(handle);
    PTK_Map *fdbMap = PSLIB_mapInput(handle);

    PTK_Map_copy(fdbMap, outMap);

    PSLIB_unmapOutput(handle);
    PSLIB_unmapInput(handle);

    /* process graph */
    vxStatus = vxProcessGraph(appCntxt->vxGraph);
    PTK_assert(VX_SUCCESS == vxStatus);

    return;

}

PSLIB_Handle PSLIB_create(PSLIB_createParams *createParams)
{
    PSLIB_Context    *appCntxt;
    PSLIB_Handle      handle;
    vx_status         vxStatus = VX_SUCCESS;

    handle = (PSLIB_Handle)malloc(sizeof(PSLIB_Context));

    if (handle == NULL)
    {
        PTK_printf("[%s:%d] Could not allocate memory for internal context.\n",
                   __FUNCTION__, __LINE__);

        return NULL;
    }

    /* Set applib-level create parameters */
    PSLIB_setParams(handle, createParams);

    /* Create nodes and graph */
    vxStatus = PSLIB_createGraph(handle);

    if (vxStatus == VX_SUCCESS)
    {
        appCntxt = (PSLIB_Context *)handle;
        vxStatus = tivxMutexCreate(&appCntxt->outBuffMutex);
    }

    if (vxStatus != VX_SUCCESS)
    {
        free(handle);
        handle = NULL;
    }

    return handle;

} /* PSLIB_create */


void PSLIB_releaseGraph(PSLIB_Handle handle)
{
    PSLIB_Context *appCntxt;

    appCntxt = (PSLIB_Context *)handle;

    vxReleaseNode(&appCntxt->vxPsMappingNode);

    vxReleaseUserDataObject(&appCntxt->vxDataPtr);
    vxReleaseUserDataObject(&appCntxt->vxPsMappingConfig);
    vxReleaseLUT(&appCntxt->vxD2ULUT);
    vxReleaseLUT(&appCntxt->vxProjMatrix);
    vxReleaseUserDataObject(&appCntxt->vxPoses);
    vxReleaseUserDataObject(&appCntxt->vxInMap);
    vxReleaseUserDataObject(&appCntxt->vxOutMap);

    vxReleaseGraph(&appCntxt->vxGraph);

    return;

} /* PSLIB_releaseGraph */


void PSLIB_delete(PSLIB_Handle *handle)
{
    PSLIB_Context *appCntxt;
    appCntxt = (PSLIB_Context *)*handle;

    if (appCntxt->inputSpots.imagePoints)
    {
        tivxMemFree(appCntxt->inputSpots.imagePoints, appCntxt->inputSpots.imagePoints_size, TIVX_MEM_INTERNAL_L2);
    }

    if (appCntxt->inputSpots.type)
    {
        tivxMemFree(appCntxt->inputSpots.type, sizeof(vx_uint16)*appCntxt->psMappingNodeConfig.ps_mapping_alg_params.maxSpotsFrame, TIVX_MEM_INTERNAL_L2);
    }

    if (appCntxt->inputSpots.prob)
    {
        tivxMemFree(appCntxt->inputSpots.prob, sizeof(vx_float32)*appCntxt->psMappingNodeConfig.ps_mapping_alg_params.maxSpotsFrame, TIVX_MEM_INTERNAL_L2);
    }


    PSLIB_releaseGraph(*handle);

    if (*handle)
    {
        free(*handle);
        *handle = NULL;
    }

    return;

} /* PSLIB_delete */

void PSLIB_setWorldReference(PSLIB_Handle handle,
                             const PTK_RigidTransform_d *ecef_w)
{
    PSLIB_Context *appCntxt;

    appCntxt = (PSLIB_Context *)handle;

    appCntxt->Md_ecef_w = *ecef_w;

    return;
}

PTK_Map* PSLIB_mapOutput(PSLIB_Handle handle)
{
    PSLIB_Context *appCntxt;
    PTK_Map       *map;
    vx_status      vxStatus;

    appCntxt = (PSLIB_Context *)handle;

    vxStatus = tivxMutexLock(appCntxt->outBuffMutex);
    PTK_assert(VX_SUCCESS == vxStatus);

    vxStatus = vxMapUserDataObject(appCntxt->vxOutMap,
                                   0,
                                   appCntxt->ogMapSize,
                                   &appCntxt->vxOutMap_mapId,
                                   (void **)&map,
                                   VX_READ_AND_WRITE,
                                   VX_MEMORY_TYPE_HOST,
                                   0);


    PTK_assert(VX_SUCCESS == vxStatus);
    PTK_assert(NULL != map);

    return map;

} /* PSLIB_mapOutput */

void PSLIB_unmapOutput(PSLIB_Handle handle)
{
    PSLIB_Context *appCntxt;
    vx_status      vxStatus;

    appCntxt = (PSLIB_Context *)handle;

    vxStatus = vxUnmapUserDataObject(appCntxt->vxOutMap,
                                     appCntxt->vxOutMap_mapId);

    PTK_assert(VX_SUCCESS == vxStatus);

    tivxMutexUnlock(appCntxt->outBuffMutex);
    PTK_assert(VX_SUCCESS == vxStatus);

    return;

} /* PSLIB_unmapOutput */

PTK_Map* PSLIB_mapInput(PSLIB_Handle handle)
{
    PTK_Map       *map = NULL;
    PSLIB_Context *appCntxt;

    vx_status      vxStatus;

    appCntxt = (PSLIB_Context *)handle;

    vxStatus = vxMapUserDataObject(appCntxt->vxInMap,
                                   0,
                                   appCntxt->ogMapSize,
                                   &appCntxt->vxInMap_mapId,
                                   (void **)&map,
                                   VX_READ_ONLY,
                                   VX_MEMORY_TYPE_HOST,
                                   0);

    PTK_assert(VX_SUCCESS == vxStatus);
    PTK_assert(NULL != map);

    return map;

} /* PSLIB_mapInput */

void PSLIB_unmapInput(PSLIB_Handle handle)
{
    PSLIB_Context *appCntxt;
    vx_status      vxStatus;

    appCntxt = (PSLIB_Context *)handle;

    vxStatus = vxUnmapUserDataObject(appCntxt->vxInMap,
                                     appCntxt->vxInMap_mapId);

    PTK_assert(VX_SUCCESS == vxStatus);

    return;

} /* PSLIB_unmapInput */

uint32_t PSLIB_getOutMapSize(PSLIB_Handle handle)
{
    uint32_t       size;
    PSLIB_Context *appCntxt;

    appCntxt = (PSLIB_Context *)handle;

    size  = tivxPSMappingNode_getMapSize(&appCntxt->psMappingNodeConfig.ps_mapping_ogmap_params);


    return size;
} /* PSLIB_getOutMapSize */

void PSLIB_setParams(PSLIB_Handle handle, PSLIB_createParams *createParams)
{
	vx_uint32 size;

    PSLIB_Context *appCntxt;
    appCntxt = (PSLIB_Context *)handle;

    appCntxt->vxContext = createParams->vxContext;

    /* PS Mapping OG mapping node config */
    /******************************/
    memset(&appCntxt->psMappingNodeConfig, 0, sizeof(tivx_ps_mapping_config_t));

    /* projection matrix and d2u table */
    appCntxt->projMatrix     = createParams->projMatrix;
    appCntxt->projMatrixSize = createParams->projMatrixSize;
    appCntxt->d2uLUT         = createParams->d2uLUT;
    appCntxt->d2uLUTSize     = createParams->d2uLUTSize;

    /*general map config*/
    appCntxt->psMappingNodeConfig.ps_mapping_ogmap_params.xCells = createParams->gridConfig.xCells;
    appCntxt->psMappingNodeConfig.ps_mapping_ogmap_params.yCells = createParams->gridConfig.yCells;
    appCntxt->psMappingNodeConfig.ps_mapping_ogmap_params.xCellSize = createParams->gridConfig.xCellSize;
    appCntxt->psMappingNodeConfig.ps_mapping_ogmap_params.yCellSize = createParams->gridConfig.yCellSize;
    appCntxt->psMappingNodeConfig.ps_mapping_ogmap_params.xMin = createParams->gridConfig.xMin;
    appCntxt->psMappingNodeConfig.ps_mapping_ogmap_params.yMin = createParams->gridConfig.yMin;
    appCntxt->psMappingNodeConfig.ps_mapping_ogmap_params.occupancyGridId = createParams->occupancyGridId;
    appCntxt->psMappingNodeConfig.ps_mapping_ogmap_params.ogFlagFree = createParams->ogFlagFree;
    appCntxt->psMappingNodeConfig.ps_mapping_ogmap_params.ogFlagOccupied = createParams->ogFlagOccupied;

    /* algorithm config */
    appCntxt->psMappingNodeConfig.ps_mapping_alg_params.maxSpotsFrame = createParams->psMappingAlgoParams.maxSpotsFrame;
    appCntxt->psMappingNodeConfig.ps_mapping_alg_params.maxTotalSpots = createParams->psMappingAlgoParams.maxTotalSpots;
    appCntxt->psMappingNodeConfig.ps_mapping_alg_params.numSpots = createParams->psMappingAlgoParams.numSpots;
    appCntxt->psMappingNodeConfig.ps_mapping_alg_params.minProb   = createParams->psMappingAlgoParams.minProb;
    appCntxt->psMappingNodeConfig.ps_mapping_alg_params.minHeight = createParams->psMappingAlgoParams.minHeight;
    appCntxt->psMappingNodeConfig.ps_mapping_alg_params.maxHeight = createParams->psMappingAlgoParams.maxHeight;
    appCntxt->psMappingNodeConfig.ps_mapping_alg_params.minWidth = createParams->psMappingAlgoParams.minWidth;
    appCntxt->psMappingNodeConfig.ps_mapping_alg_params.maxWidth = createParams->psMappingAlgoParams.maxWidth;
    appCntxt->psMappingNodeConfig.ps_mapping_alg_params.matchTh = createParams->psMappingAlgoParams.matchTh;
    appCntxt->psMappingNodeConfig.ps_mapping_alg_params.minCountPerPS = createParams->psMappingAlgoParams.minCountPerPS;
    appCntxt->psMappingNodeConfig.ps_mapping_alg_params.minImPosX =  createParams->psMappingAlgoParams.minImPosX;
    appCntxt->psMappingNodeConfig.ps_mapping_alg_params.maxImPosX =  createParams->psMappingAlgoParams.maxImPosX;
    appCntxt->psMappingNodeConfig.ps_mapping_alg_params.minImPosY =  createParams->psMappingAlgoParams.minImPosY;
    appCntxt->psMappingNodeConfig.ps_mapping_alg_params.maxImPosY =  createParams->psMappingAlgoParams.maxImPosY;

    appCntxt->psMappingNodeConfig.ps_mapping_cam_params.imageWidth = 1280;
    appCntxt->psMappingNodeConfig.ps_mapping_cam_params.imageHeight = 720;
    appCntxt->psMappingNodeConfig.ps_mapping_cam_params.focalLengthX = 311.8333f;
    appCntxt->psMappingNodeConfig.ps_mapping_cam_params.focalLengthY = 311.8333f;
    appCntxt->psMappingNodeConfig.ps_mapping_cam_params.principalPointX = 639.f;
    appCntxt->psMappingNodeConfig.ps_mapping_cam_params.principalPointY = 359.f;
    appCntxt->psMappingNodeConfig.ps_mapping_cam_params.d2uTableStep = 3.5889965e+02f;

    appCntxt->ps_mapping_pose.M_e_g = createParams->M_e_g;

    appCntxt->inputSpotSize = sizeof(appCntxt->inputSpots);

    size = sizeof(vx_uint16)*appCntxt->psMappingNodeConfig.ps_mapping_alg_params.maxSpotsFrame;
    appCntxt->inputSpots.type = malloc(size);

    if (appCntxt->inputSpots.type == NULL)
    {
        PTK_printf("[%s:%d] Memory allocation failed.\n",
                   __FUNCTION__, __LINE__);

        assert(0);
    }

    appCntxt->inputSpotSize += size;

    size = sizeof(vx_float32)*appCntxt->psMappingNodeConfig.ps_mapping_alg_params.maxSpotsFrame;
    appCntxt->inputSpots.prob = malloc(size);

    if (appCntxt->inputSpots.prob == NULL)
    {
        PTK_printf("[%s:%d] Memory allocation failed.\n",
                   __FUNCTION__, __LINE__);

        free(appCntxt->inputSpots.type);

        assert(0);
    }

    appCntxt->inputSpotSize += size;

    appCntxt->inputSpots.imagePoints_size = sizeof(vx_float32)*appCntxt->psMappingNodeConfig.ps_mapping_alg_params.maxSpotsFrame*4*2;
    appCntxt->inputSpots.imagePoints = malloc(appCntxt->inputSpots.imagePoints_size);

    if (appCntxt->inputSpots.imagePoints == NULL)
    {
        PTK_printf("[%s:%d] Memory allocation failed.\n",
                   __FUNCTION__, __LINE__);

        free(appCntxt->inputSpots.prob);
        free(appCntxt->inputSpots.type);

        assert(0);
    }

    appCntxt->inputSpotSize += appCntxt->inputSpots.imagePoints_size;

    appCntxt->psNodeCore = createParams->psNodeCore;

    if (appCntxt->psNodeCore == NULL)
    {
        appCntxt->psNodeCore = PSLIB_DEFAULT_CORE_MAPPING;
    }
}

vx_size tivxPSMappingNode_getMapSize(tivx_ps_mapping_ogmap_params_t *prms)
{
    PTK_MapConfig config;
    vx_status status;

    status = tivxPSMappingNode_getMapConfig(prms, &config);

    if (VX_SUCCESS == status)
    {
        return PTK_Map_getSize(&config);
    }
    else
    {
        return 0;
    }

}

vx_status tivxPSMappingNode_getMapConfig(tivx_ps_mapping_ogmap_params_t *prms, PTK_MapConfig *config)
{
    memset(config, 0, sizeof(PTK_MapConfig));

    config->xCells = prms->xCells;
    config->yCells = prms->yCells;
    config->zCells = 1;
    config->xCellSize = prms->xCellSize;
    config->yCellSize = prms->yCellSize;
    config->zCellSize = 0.f;
    config->xMin = prms->xMin;
    config->yMin = prms->yMin;
    config->zMin = 0.f;
    config->grid[0].id = prms->occupancyGridId;
    config->grid[0].type = PTK_GRID_TYPE_BITS;

    return VX_SUCCESS;
}

vx_status PSLIB_createGraph(PSLIB_Handle handle)
{
    PSLIB_Context *appCntxt;
    vx_status      status = VX_SUCCESS;

    appCntxt = (PSLIB_Context *)handle;

    /* Mapping Graph */
    appCntxt->vxGraph = vxCreateGraph(appCntxt->vxContext);
    PTK_assert(appCntxt->vxGraph);

    /* allocate maps */
    uint8_t       *ogMapMem;
    PTK_Map       *psdOgMap;
    PTK_MapConfig  ogMapConfig;

    appCntxt->ogMapSize  = tivxPSMappingNode_getMapSize(&appCntxt->psMappingNodeConfig.ps_mapping_ogmap_params);
    ogMapMem             = (uint8_t *)malloc(appCntxt->ogMapSize);

    if (!ogMapMem)
    {
        PTK_printf("[%s:%d] Memory allocation failed for TIDL PSD ogmap\n",
                   __FUNCTION__, __LINE__);

        return VX_FAILURE;
    }

    tivxPSMappingNode_getMapConfig(&appCntxt->psMappingNodeConfig.ps_mapping_ogmap_params, &ogMapConfig);
    psdOgMap = PTK_Map_init(ogMapMem, &ogMapConfig);


    // config params
    appCntxt->vxPsMappingConfig =
        vxCreateUserDataObject(appCntxt->vxContext,
                               "tivx_ps_mapping_config_t",
                               sizeof(tivx_ps_mapping_config_t),
                               &appCntxt->psMappingNodeConfig);

    PTK_assert(appCntxt->vxPsMappingConfig);

    vxSetReferenceName((vx_reference)appCntxt->vxPsMappingConfig, "PsMappingConfig");

    // dataPtr
    appCntxt->vxDataPtr =
        vxCreateUserDataObject(appCntxt->vxContext,
                               "tivx_ps_mapping_input_t",
                               appCntxt->inputSpotSize,
                               NULL);

    PTK_assert(appCntxt->vxDataPtr);

    vxSetReferenceName((vx_reference)appCntxt->vxDataPtr, "InputParkingSpots");

    // d2uLUT
    appCntxt->vxD2ULUT = vxCreateLUT(appCntxt->vxContext, VX_TYPE_FLOAT32, appCntxt->d2uLUTSize);
    PTK_assert(appCntxt->vxD2ULUT);
    vxSetReferenceName((vx_reference)appCntxt->vxD2ULUT, "DistortionCorrectionLUT");
    status = vxCopyLUT(appCntxt->vxD2ULUT, appCntxt->d2uLUT, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);
    PTK_assert(VX_SUCCESS == status);

    appCntxt->vxProjMatrix = vxCreateLUT(appCntxt->vxContext, VX_TYPE_FLOAT64, appCntxt->projMatrixSize);
    PTK_assert(appCntxt->vxProjMatrix);
    vxSetReferenceName((vx_reference)appCntxt->vxProjMatrix, "ProjectionMatrix");
    status = vxCopyLUT(appCntxt->vxProjMatrix, appCntxt->projMatrix, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);
    PTK_assert(VX_SUCCESS == status);

    // poses
    appCntxt->vxPoses =
        vxCreateUserDataObject(appCntxt->vxContext,
                               "tivx_ps_mapping_pose_t",
                               sizeof(tivx_ps_mapping_pose_t),
                               NULL);

    PTK_assert(appCntxt->vxPoses);

    vxSetReferenceName((vx_reference)appCntxt->vxPoses, "VehiclePose");


    //input map
    appCntxt->vxInMap =
        vxCreateUserDataObject(appCntxt->vxContext,
                               "InputMap",
                               appCntxt->ogMapSize,
                               psdOgMap);

    PTK_assert(appCntxt->vxInMap);

    vxSetReferenceName((vx_reference)appCntxt->vxInMap, "MapIn");

    //output map
    appCntxt->vxOutMap =
        vxCreateUserDataObject(appCntxt->vxContext,
                               "OutputMap",
                               appCntxt->ogMapSize,
                               psdOgMap);

    PTK_assert(appCntxt->vxOutMap);

    vxSetReferenceName((vx_reference)appCntxt->vxOutMap, "MapOut");

    appCntxt->vxPsMappingNode = tivxPsMappingNode(appCntxt->vxGraph,
                    appCntxt->vxPsMappingConfig,
                    appCntxt->vxDataPtr,
                    appCntxt->vxD2ULUT,
                    appCntxt->vxProjMatrix,
                    appCntxt->vxPoses,
                    appCntxt->vxInMap,
                    appCntxt->vxOutMap);

    PTK_assert(appCntxt->vxPsMappingNode);
    vxSetReferenceName((vx_reference)appCntxt->vxPsMappingNode, "PsMappingNode");

    status = vxSetNodeTarget(appCntxt->vxPsMappingNode,
                             VX_TARGET_STRING,
                             appCntxt->psNodeCore);

    PTK_assert(status==VX_SUCCESS);

    /* Verify graph */
    status = vxVerifyGraph(appCntxt->vxGraph);
    PTK_assert(VX_SUCCESS == status);

    free(ogMapMem);

#if 0
    tivxExportGraphToDot(appCntxt->vxGraph, ".", "vx_applib_ps_mapping");
#endif

    return status;

} /* PSLIB_createGraph */


void PSLIB_set_ego_to_world_pose(PSLIB_Handle handle, uint64_t timestamp)
{
    PSLIB_Context *appCntxt = (PSLIB_Context *)handle;
    PTK_INS_Record curInspva;
    PTK_INS_RetCode imu_status;

    imu_status = PTK_INS_getRecordLinearInterp(PTK_INS_RECORD_TYPE_INSPVA, timestamp, &curInspva);
    (void)imu_status;
    //if (PTK_INS_RETURN_CODE_OK != imu_status)
    //    PTK_printf("WARNING: PTK INS return code (%d) NOT OK (linear interp at timestamp %" PRId64 ")\n", imu_status, timestamp);

    PTK_RigidTransform_d Md_enu_ecef;
    PTK_Position_getECEFtoENU_d(&curInspva.data.inspva.position, &Md_enu_ecef);

    PTK_RigidTransform_d Md_enu_w;
    PTK_RigidTransform_d_compose(&Md_enu_w, &Md_enu_ecef, &appCntxt->Md_ecef_w);

    PTK_RigidTransform M_enu_w;
    PTK_RigidTransform_d_convertToSingle(&M_enu_w, &Md_enu_w);

    PTK_RigidTransform M_w_enu;
    PTK_RigidTransform_invert(&M_w_enu, &M_enu_w);

    PTK_RigidTransform M_enu_e;
    PTK_INS_getIMUtoENU(&curInspva, &M_enu_e);

    PTK_RigidTransform_compose(&appCntxt->ps_mapping_pose.M_w_e, &M_w_enu, &M_enu_e);
    //PTK_RigidTransform_invert(&appCntxt->M_e_w, &appCntxt->M_w_e);

    return;
}


void PSLIB_setupPsMappingNode(PSLIB_Handle handle, void *dataPtr)
{
    PSLIB_Context *appCntxt = (PSLIB_Context *)handle;
    vx_status status;

    // load parking spots data
    vx_int32  psIdx, psType;

    vx_float32 x[4], y[4], prob;
    vx_float32 minX = appCntxt->psMappingNodeConfig.ps_mapping_alg_params.minImPosX;
    vx_float32 maxX = appCntxt->psMappingNodeConfig.ps_mapping_alg_params.maxImPosX;
    vx_float32 minY = appCntxt->psMappingNodeConfig.ps_mapping_alg_params.minImPosY;
    vx_float32 maxY = appCntxt->psMappingNodeConfig.ps_mapping_alg_params.maxImPosY;

    vx_int16   imageWidth  = appCntxt->psMappingNodeConfig.ps_mapping_cam_params.imageWidth;
    vx_int16   imageHeight = appCntxt->psMappingNodeConfig.ps_mapping_cam_params.imageHeight;

#if 0
    appCntxt->inputSpots.numParkingSpots = 0;

    while (sscanf(datachar,"%s %*d %*d %*d %*f %*f %*f %*f %*d %*d %*d %*d %*d %*d %*d %f %f %f %f %f %f %f %f %f",
            strType, &prob, &x0, &y0, &x1, &y1, &x2, &y2, &x3, &y3) == 10)
    {
        datachar = strchr(datachar, '\n') +1;

        // ignore low probable spots
        if (prob <= appCntxt->psMappingNodeConfig.ps_mapping_alg_params.minProb)
        {
           continue;
        }

        if ((x0 < minX) || (x1 < minX) || (x2 < minX) || (x3 < minX) ||
            (x0 > maxX) || (x1 > maxX) || (x2 > maxX) || (x3 > maxX) ||
            (y0 < minY) || (y1 < minY) || (y2 < minY) || (y3 < minY) ||
            (y0 > maxY) || (y1 > maxY) || (y2 > maxY) || (y3 > maxY))
        {
            continue;
        }

        if (!strcmp(strType, label_id_name_map[1]))
        {
            psType = 1;
        }
        else
        {
            psType = 2;
        }

        psIdx = appCntxt->inputSpots.numParkingSpots;
        appCntxt->inputSpots.numParkingSpots++;

        appCntxt->inputSpots.type[psIdx] = psType;
        appCntxt->inputSpots.prob[psIdx] = prob;

        appCntxt->inputSpots.imagePoints[psIdx*8 + 0] = x0;
        appCntxt->inputSpots.imagePoints[psIdx*8 + 1] = y0;
        appCntxt->inputSpots.imagePoints[psIdx*8 + 2] = x1;
        appCntxt->inputSpots.imagePoints[psIdx*8 + 3] = y1;
        appCntxt->inputSpots.imagePoints[psIdx*8 + 4] = x2;
        appCntxt->inputSpots.imagePoints[psIdx*8 + 5] = y2;
        appCntxt->inputSpots.imagePoints[psIdx*8 + 6] = x3;
        appCntxt->inputSpots.imagePoints[psIdx*8 + 7] = y3;
    }
#else
    TIDL_ODLayerHeaderInfo * pHeader;
    TIDL_ODLayerObjInfo *    pSpot;



    vx_int16 i, j;
    vx_int16 numTotalSpots;
    vx_int16 hdrSize  = sizeof(TIDL_ODLayerHeaderInfo);
    vx_int16 spotSize = sizeof(TIDL_ODLayerObjInfo);

    appCntxt->inputSpots.numParkingSpots = 0;

    pHeader = (TIDL_ODLayerHeaderInfo *) dataPtr;
    numTotalSpots = (vx_int16) pHeader->numDetObjects;


    for (i=0; i < numTotalSpots; i++)
    {
        pSpot = (TIDL_ODLayerObjInfo *)((uint8_t *)dataPtr + hdrSize + i*spotSize);

        prob = (vx_float32) pSpot->score;

        // ignore low probable spots
        if (prob <= appCntxt->psMappingNodeConfig.ps_mapping_alg_params.minProb)
        {
           continue;
        }

        psType = (vx_int32) pSpot->label;
        if (psType != 1)
        {
            psType = 2;
        }

        for (j = 0; j < 4; j++)
        {
            x[j] = (vx_float32) (pSpot->keyPoints[j].x * imageWidth);
            y[j] = (vx_float32) (pSpot->keyPoints[j].y * imageHeight);
        }

        if ((x[0] < minX) || (x[1] < minX) || (x[2] < minX) || (x[3] < minX) ||
            (x[0] > maxX) || (x[1] > maxX) || (x[2] > maxX) || (x[3] > maxX) ||
            (y[0] < minY) || (y[1] < minY) || (y[2] < minY) || (y[3] < minY) ||
            (y[0] > maxY) || (y[1] > maxY) || (y[2] > maxY) || (y[3] > maxY))
        {
            continue;
        }

        psIdx = appCntxt->inputSpots.numParkingSpots;
        appCntxt->inputSpots.numParkingSpots++;

        appCntxt->inputSpots.type[psIdx] = psType;
        appCntxt->inputSpots.prob[psIdx] = prob;

        appCntxt->inputSpots.imagePoints[psIdx*8 + 0] = x[0];
        appCntxt->inputSpots.imagePoints[psIdx*8 + 1] = y[0];
        appCntxt->inputSpots.imagePoints[psIdx*8 + 2] = x[1];
        appCntxt->inputSpots.imagePoints[psIdx*8 + 3] = y[1];
        appCntxt->inputSpots.imagePoints[psIdx*8 + 4] = x[2];
        appCntxt->inputSpots.imagePoints[psIdx*8 + 5] = y[2];
        appCntxt->inputSpots.imagePoints[psIdx*8 + 6] = x[3];
        appCntxt->inputSpots.imagePoints[psIdx*8 + 7] = y[3];

    }

#endif

    status = vxCopyUserDataObject(appCntxt->vxDataPtr,
                                  0,
                                  appCntxt->inputSpotSize,
                                  &appCntxt->inputSpots,
                                  VX_WRITE_ONLY,
                                  VX_MEMORY_TYPE_HOST);
    PTK_assert(VX_SUCCESS == status);


    // load pose
    status = vxCopyUserDataObject(appCntxt->vxPoses,
                                  0,
                                  sizeof(tivx_ps_mapping_pose_t),
                                  &appCntxt->ps_mapping_pose,
                                  VX_WRITE_ONLY,
                                  VX_MEMORY_TYPE_HOST);
    PTK_assert(VX_SUCCESS == status);


    return;
}

vx_graph PSLIB_getGraphHandle(PSLIB_Handle handle)
{
    PSLIB_Context *appCntxt;

    appCntxt = (PSLIB_Context *)handle;

    return appCntxt->vxGraph;
}

