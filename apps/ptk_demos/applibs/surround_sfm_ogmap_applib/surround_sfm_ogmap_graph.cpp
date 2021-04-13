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

#include "surround_sfm_ogmap_priv.h"


#define DOF_TEMP_PRED    1

vx_status SFMOGAPPLIB_createPCNodesInGraph(SFMOGAPPLIB_Context  *appCntxt)
{
    uint32_t    i;
    vx_status  status;
    vx_pyramid pyr_ref;
    vx_pyramid pyr_cur;


    /*******************************************************/
    /* Point Cloud Related                                 */
    /*******************************************************/
    /* Pyramid - Data Objects */
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        if (appCntxt->enableDof)
        {
            // input image
            appCntxt->pyr_in_image[i] = vxCreateImage(appCntxt->vxContext, appCntxt->dofWidth, appCntxt->dofHeight, VX_DF_IMAGE_U8);
            PTK_assert(appCntxt->pyr_in_image[i]);
            vxSetReferenceName((vx_reference)appCntxt->pyr_in_image[i], "LDC_OutputImageU8");
        }
        else 
        {
            //DOF input
            appCntxt->dof2tracks_in_field[i] = vxCreateImage(appCntxt->vxContext,
                                                             appCntxt->dofWidth,
                                                             appCntxt->dofHeight,
                                                             VX_DF_IMAGE_U32);
        
            PTK_assert(appCntxt->dof2tracks_in_field[i]);
            vxSetReferenceName((vx_reference)appCntxt->dof2tracks_in_field[i],
                               "DenseOpticalFlowField");
        }
        // poses
        appCntxt->triang_in_pose[i] =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "tivx_triangulation_pose_t",
                                   sizeof(tivx_triangulation_pose_t),
                                   NULL);
        PTK_assert(appCntxt->triang_in_pose[i]);
        vxSetReferenceName((vx_reference)appCntxt->triang_in_pose[i], "CameraPoses");
    }

    /* dof2tracks - Data Objects */
    // config
    appCntxt->dof2tracks_in_config =
        vxCreateUserDataObject(appCntxt->vxContext,
                               "tivx_dof_to_tracks_params_t",
                               sizeof(tivx_dof_to_tracks_params_t),
                               &appCntxt->dtNodeCfg);
    PTK_assert(appCntxt->dof2tracks_in_config);
    vxSetReferenceName((vx_reference)appCntxt->dof2tracks_in_config,
                       "DOF2TracksConfig");

    // we read distortion corrected input,
    // so we don't need LUT for distortion correction anyway
    appCntxt->dof2tracks_in_lut = NULL;

    /* Triangulation - Data Objects */
    // config
    appCntxt->triang_in_config =
        vxCreateUserDataObject(appCntxt->vxContext,
                               "tivx_triangulation_params_t",
                               sizeof(tivx_triangulation_params_t),
                               &appCntxt->trNodeCfg);
    PTK_assert(appCntxt->triang_in_config);
    vxSetReferenceName((vx_reference)appCntxt->triang_in_config,
                       "TriangulationConfig");

    // tracks
    vx_enum tivx_triangulation_track_e =
        vxRegisterUserStruct(appCntxt->vxContext,
                             sizeof(tivx_triangulation_track_t));
    appCntxt->triang_in_tracks = vxCreateArray(appCntxt->vxContext,
                                          tivx_triangulation_track_e,
                                          appCntxt->maxNumTracks);
    PTK_assert(appCntxt->triang_in_tracks);
    vxSetReferenceName((vx_reference)appCntxt->triang_in_tracks, "Tracks2d");

    // points3d
    vx_enum tivx_triangulation_point3d_e =
        vxRegisterUserStruct(appCntxt->vxContext,
                             sizeof(PTK_Point));
    appCntxt->triang_out_points3d =
        vxCreateArray(appCntxt->vxContext,
                      tivx_triangulation_point3d_e,
                      appCntxt->maxNumTracks);
    PTK_assert(appCntxt->triang_out_points3d);
    vxSetReferenceName((vx_reference)appCntxt->triang_out_points3d, "Points3d");


    if (appCntxt->enableDof)
    {
        // delay
        vx_pyramid pyr_exemplar = vxCreatePyramid(appCntxt->vxContext,
               appCntxt->dofLevels, VX_SCALE_PYRAMID_HALF,
               appCntxt->dofWidth, appCntxt->dofHeight,
               VX_DF_IMAGE_U8);
        PTK_assert(pyr_exemplar);
        appCntxt->pyr_out_delay = vxCreateDelay(appCntxt->vxContext, (vx_reference)pyr_exemplar, 2);
        PTK_assert(appCntxt->pyr_out_delay);
        vxReleasePyramid(&pyr_exemplar);
    
        pyr_ref = (vx_pyramid)vxGetReferenceFromDelay(appCntxt->pyr_out_delay, -1);
        PTK_assert(pyr_ref);
        vxSetReferenceName((vx_reference)pyr_ref, "DOF_PyramidReference");
        pyr_cur = (vx_pyramid)vxGetReferenceFromDelay(appCntxt->pyr_out_delay,  0);
        PTK_assert(pyr_cur);
        vxSetReferenceName((vx_reference)pyr_cur, "DOF_PyramidCurrent");
    
        /* DOF - Data Objects */
        // config
        //tivx_dmpac_dof_params_init(&appCntxt->dofCfg);
#if !DOF_TEMP_PRED
        appCntxt->dofCfg.base_predictor[0] = TIVX_DMPAC_DOF_PREDICTOR_DELAY_LEFT;
        appCntxt->dofCfg.base_predictor[1] = TIVX_DMPAC_DOF_PREDICTOR_PYR_COLOCATED;
    
        appCntxt->dofCfg.inter_predictor[0] = TIVX_DMPAC_DOF_PREDICTOR_DELAY_LEFT;
        appCntxt->dofCfg.inter_predictor[1] = TIVX_DMPAC_DOF_PREDICTOR_PYR_COLOCATED;
#endif
    
        //appCntxt->dof_in_config = vxCreateUserDataObject(appCntxt->vxContext, "tivx_dmpac_dof_params_t", sizeof(tivx_dmpac_dof_params_t), NULL);
        appCntxt->dof_in_config = vxCreateUserDataObject(appCntxt->vxContext, "tivx_dmpac_dof_params_t", sizeof(tivx_dmpac_dof_params_t), &appCntxt->dofCfg);
        PTK_assert(appCntxt->dof_in_config);
        vxSetReferenceName((vx_reference)appCntxt->dof_in_config, "DOF_Config");

        status = vxCopyUserDataObject(appCntxt->dof_in_config, 0, sizeof(tivx_dmpac_dof_params_t), &appCntxt->dofCfg, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);
    
        // delay
#if DOF_TEMP_PRED
        
        vx_image  dof_field_exemplar = vxCreateImage(appCntxt->vxContext, appCntxt->dofWidth, appCntxt->dofHeight, VX_DF_IMAGE_U32);
        PTK_assert(dof_field_exemplar);
        appCntxt->dof_out_delay = vxCreateDelay(appCntxt->vxContext, (vx_reference)dof_field_exemplar, 2);
        PTK_assert(appCntxt->dof_out_delay);
        vxReleaseImage(&dof_field_exemplar);
    
        appCntxt->dof_in_field = (vx_image)vxGetReferenceFromDelay(appCntxt->dof_out_delay, -1);
        PTK_assert(appCntxt->dof_in_field);
        vxSetReferenceName((vx_reference)appCntxt->dof_in_field, "DOF_FlowVectorIn");
        appCntxt->dof_out_field = (vx_image)vxGetReferenceFromDelay(appCntxt->dof_out_delay,  0);
        PTK_assert(appCntxt->dof_out_field);
        vxSetReferenceName((vx_reference)appCntxt->dof_out_field, "DOF_FlowVectorOut");
#else
        appCntxt->dof_out_delay = NULL;
        appCntxt->dof_in_field  = NULL;
        
        appCntxt->dof_out_field = vxCreateImage(appCntxt->vxContext, appCntxt->dofWidth, appCntxt->dofHeight, VX_DF_IMAGE_U32);
        PTK_assert(appCntxt->dof_out_field);
        vxSetReferenceName((vx_reference)appCntxt->dof_out_field, "DOF_FlowVectorOut");
#endif

        /*******************************************************/
        /* Create Nodes                                        */
        /*******************************************************/
        /* Gaussian Pyramid Node */
        appCntxt->vxPyrNode = vxGaussianPyramidNode( 
                appCntxt->vxSfmGraph,
                appCntxt->pyr_in_image[0],
                pyr_ref);
        PTK_assert(appCntxt->vxPyrNode);
        status = vxSetNodeTarget(appCntxt->vxPyrNode, VX_TARGET_STRING, TIVX_TARGET_VPAC_MSC1);
        PTK_assert(status==VX_SUCCESS);
        vxSetReferenceName((vx_reference)appCntxt->vxPyrNode, "GaussianPyramid");
    
        /* DMPAC DOF Node */
        appCntxt->vxDofNode = tivxDmpacDofNode(
                appCntxt->vxSfmGraph,
                appCntxt->dof_in_config,
                NULL,
                NULL,
                pyr_cur,
                pyr_ref,
                appCntxt->dof_in_field,
                NULL,
                NULL,
                appCntxt->dof_out_field,
                NULL);
        PTK_assert(appCntxt->vxDofNode);
        status = vxSetNodeTarget(appCntxt->vxDofNode, VX_TARGET_STRING, TIVX_TARGET_DMPAC_DOF);
        PTK_assert(status==VX_SUCCESS);
        vxSetReferenceName((vx_reference)appCntxt->vxDofNode, "DOF_Processing");

        appCntxt->vxDof2tracksNode = tivxDofToTracksNode(
                appCntxt->vxSfmGraph,
                appCntxt->dof2tracks_in_config,
                appCntxt->dof_out_field,
                appCntxt->dof2tracks_in_lut,
                appCntxt->triang_in_tracks);
    } 
    else
    {
        appCntxt->vxDof2tracksNode = tivxDofToTracksNode(
                appCntxt->vxSfmGraph,
                appCntxt->dof2tracks_in_config,
                appCntxt->dof2tracks_in_field[0],
                appCntxt->dof2tracks_in_lut,
                appCntxt->triang_in_tracks);
    } 
    
    PTK_assert(appCntxt->vxDof2tracksNode);
    vxSetReferenceName((vx_reference)appCntxt->vxDof2tracksNode, "DOF2Tracks");

    status = vxSetNodeTarget(appCntxt->vxDof2tracksNode,
                             VX_TARGET_STRING,
                             appCntxt->dofTrackNodeCore);
    PTK_assert(status==VX_SUCCESS);

    appCntxt->vxTriangNode = tivxTriangulationNode(
            appCntxt->vxSfmGraph,
            appCntxt->triang_in_config,
            appCntxt->triang_in_tracks,
            appCntxt->triang_in_pose[0],
            appCntxt->triang_out_points3d);
    PTK_assert(appCntxt->vxTriangNode);
    vxSetReferenceName((vx_reference)appCntxt->vxTriangNode, "Triangulation");

    status = vxSetNodeTarget(appCntxt->vxTriangNode,
                             VX_TARGET_STRING,
                             appCntxt->triNodeCore);
    PTK_assert(status==VX_SUCCESS);

    /* Update the parameter count for the following:
     * - pyr_in_image
     * - triang_in_pose
     */
    appCntxt->effectiveNumParams += 2;

    return status;
}


vx_status SFMOGAPPLIB_createOGNodesInGraph(SFMOGAPPLIB_Context  *appCntxt)
{
    uint8_t           *ogMapMem;
    PTK_Map           *ogMap;
    PTK_MapConfig      mapConfig[PTK_ALG_SFM_OGMAP_NUM_OUT_MAPS];
    PTK_Api_MemoryReq  memReq;
    vx_status          status;
    uint32_t            i;

    /*******************************************************/
    /* OG Mapping related                                  */
    /*******************************************************/
    PTK_Alg_SfmOgmapConfig(&appCntxt->ogPfsdCfg.ogConfig, &memReq);

    if (memReq.numBlks != PTK_ALG_SFM_OGMAP_NUM_MEM_REQ_BLKS)
    {
        printf("[%s:%d] PTK_Alg_SfmOgmapConfig() failed\n",
               __FUNCTION__, __LINE__);

        return VX_FAILURE;
    }

    memset(mapConfig, 0, sizeof(mapConfig));

    /* Get the map configuration. */
    PTK_Alg_SfmOgmapGetMapConfig(&appCntxt->ogPfsdCfg.ogConfig, mapConfig);

    /* According to the SFM OGMAP API guide:
     * - the entry with id * PTK_ALG_SFM_OGMAP_MEM_BUFF_MAP_ACC_OCC gives the
     *   output accumulated map size.
     * - the entry with id * PTK_ALG_SFM_OGMAP_MEM_BUFF_MAP_INST_DS gives the
     *   output instantaneous map size.
     */
    /* Accumulated output map. */
    appCntxt->outAccMapSize =
        memReq.blks[PTK_ALG_SFM_OGMAP_MEM_BUFF_MAP_ACC_OCC].size;

    ogMapMem = (uint8_t *)malloc(appCntxt->outAccMapSize);

    if (!ogMapMem)
    {
        PTK_printf("[%s:%d] Memory allocation failed.\n",
                   __FUNCTION__,
                   __LINE__);

        return VX_FAILURE;
    }

    ogMap = PTK_Map_init(ogMapMem,
                         &mapConfig[PTK_ALG_SFM_OGMAP_OUT_MAP_ACC_OCC]);

    appCntxt->vxOutAccMap =
        vxCreateUserDataObject(appCntxt->vxContext,
                               "AccMap",
                               appCntxt->outAccMapSize,
                               ogMap);

    PTK_assert(appCntxt->vxOutAccMap);
    vxSetReferenceName((vx_reference)appCntxt->vxOutAccMap,
                       "SfmOutAccMap");

    free(ogMapMem);

    /* Instantaneous output map. */
    appCntxt->outInstMapSize =
        memReq.blks[PTK_ALG_SFM_OGMAP_MEM_BUFF_MAP_INST_DS].size;

    ogMapMem = (uint8_t *)malloc(appCntxt->outInstMapSize);

    if (!ogMapMem)
    {
        PTK_printf("[%s:%d] Memory allocation failed.\n",
                   __FUNCTION__,
                   __LINE__);

        return VX_FAILURE;
    }

    ogMap = PTK_Map_init(ogMapMem,
                         &mapConfig[PTK_ALG_SFM_OGMAP_OUT_MAP_INST_DS]);

    /* OG Mapping Node */
    /* config. */
    appCntxt->vxOgInConfig =
        vxCreateUserDataObject(appCntxt->vxContext,
                               "tivx_sfm_ogmap_pfsd_params_t",
                               sizeof(tivx_sfm_ogmap_pfsd_params_t),
                               &appCntxt->ogPfsdCfg);

    PTK_assert(appCntxt->vxOgInConfig);
    vxSetReferenceName((vx_reference)appCntxt->vxOgInConfig,
                       "SfmOgmapConfig");

    // if we read point cloud from a file, we need point cloud object
    if (appCntxt->generatePC == 0)
    {
        vx_enum tivx_triangulation_point3d_e;

        tivx_triangulation_point3d_e =
            vxRegisterUserStruct(appCntxt->vxContext,
                                 sizeof(PTK_Point));

        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            appCntxt->vxOgInPoints3d[i] =
                vxCreateArray(appCntxt->vxContext,
                              tivx_triangulation_point3d_e,
                              appCntxt->maxNumTracks);

            PTK_assert(appCntxt->vxOgInPoints3d[i]);
            vxSetReferenceName((vx_reference)appCntxt->vxOgInPoints3d[i],
                               "Points3d");
        }

    /* Update the parameter count for the following:
     * - vxOgInPoints3d
     */
        appCntxt->effectiveNumParams++;
    }

    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        // points3d transform, camera pose on WCS */
        appCntxt->points3d_transform[i] =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "PTK_RigidTransform",
                                   sizeof(PTK_RigidTransform),
                                   NULL);

        PTK_assert(appCntxt->points3d_transform[i]);
        vxSetReferenceName((vx_reference)appCntxt->points3d_transform[i],
                           "Points3DTransform");

        /* ego pose on WCS */
        appCntxt->vxOgInPose[i] =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "PTK_RigidTransform",
                                   sizeof(PTK_RigidTransform),
                                   NULL);

        PTK_assert(appCntxt->vxOgInPose[i]);
        vxSetReferenceName((vx_reference)appCntxt->vxOgInPose[i],
                           "VehiclePose");

        /* Position and reference object. */
        appCntxt->vxPoseAndRef[i] =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "PTK_InsPoseAndRef",
                                   sizeof(PTK_InsPoseAndRef),
                                   NULL);

        PTK_assert(appCntxt->vxPoseAndRef[i]);
        vxSetReferenceName((vx_reference)appCntxt->vxPoseAndRef[i],
                           "PoseAndReference");

        appCntxt->vxOutInstMap[i] =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "InstMap",
                                   appCntxt->outInstMapSize,
                                   ogMap);

        PTK_assert(appCntxt->vxOutInstMap[i]);
        vxSetReferenceName((vx_reference)appCntxt->vxOutInstMap[i],
                           "SfmOutInstMap");

        appCntxt->vxPfsdOutDesc[i] =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "PTK_Alg_FsdPfsdPSDesc",
                                   sizeof(PTK_Alg_FsdPfsdPSDesc),
                                   NULL);

        PTK_assert(appCntxt->vxPfsdOutDesc[i]);

        vxSetReferenceName((vx_reference)appCntxt->vxPfsdOutDesc[i],
                           "PfsdOutDesc");
    }

    /* Update the parameter count for the following:
     * - points3d_transform
     * - vxOgInPose
     * - vxPoseAndRef
     * - vxOutInstMap
     * - vxPfsdOutDesc
     */
    appCntxt->effectiveNumParams += 5;

    /*******************************************************/
    /* Create Nodes                                        */
    /*******************************************************/
    if (appCntxt->generatePC)
    {
        appCntxt->vxOgNode = tivxSfmOgmapNode(appCntxt->vxSfmGraph,
                                              appCntxt->vxOgInConfig,
                                              appCntxt->triang_out_points3d,
                                              appCntxt->points3d_transform[0],
                                              appCntxt->vxOgInPose[0],
                                              appCntxt->vxPoseAndRef[0],
                                              appCntxt->vxPfsdOutDesc[0],
                                              appCntxt->vxOutAccMap,
                                              appCntxt->vxOutInstMap[0]);
    }
    else
    {
        appCntxt->vxOgNode = tivxSfmOgmapNode(appCntxt->vxSfmGraph,
                                              appCntxt->vxOgInConfig,
                                              appCntxt->vxOgInPoints3d[0],
                                              appCntxt->points3d_transform[0],
                                              appCntxt->vxOgInPose[0],
                                              appCntxt->vxPoseAndRef[0],
                                              appCntxt->vxPfsdOutDesc[0],
                                              appCntxt->vxOutAccMap,
                                              appCntxt->vxOutInstMap[0]);
    }

    PTK_assert(appCntxt->vxOgNode);
    vxSetReferenceName((vx_reference)appCntxt->vxOgNode,
                       "SfmOgmap");

    status = vxSetNodeTarget(appCntxt->vxOgNode,
                             VX_TARGET_STRING,
                             appCntxt->ogNodeCore);

    PTK_assert(status==VX_SUCCESS);

    free(ogMapMem);

    return status;
}


static vx_status SFMOGAPPLIB_setupPipeline(SFMOGAPPLIB_Context  *appCntxt)
{
    SFMOGAPPLIB_graphParams            *paramDesc;
    vx_graph_parameter_queue_params_t   q[SFMOGAPPLIB_NUM_GRAPH_PARAMS];
    uint32_t                            cnt = 0;
    vx_status                           vxStatus;

    if (appCntxt->generatePC)
    {
        if (appCntxt->enableDof)
        {
            /* vxPyrNode Param 0 ==> graph param 0. */
            ptkdemo_addParamByNodeIndex(appCntxt->vxSfmGraph,
                                        appCntxt->vxPyrNode,
                                        0);
            q[cnt++].refs_list = (vx_reference*)appCntxt->pyr_in_image;
        } 
        else 
        {
            ptkdemo_addParamByNodeIndex(appCntxt->vxSfmGraph,
                                        appCntxt->vxDof2tracksNode,
                                        1);
            q[cnt++].refs_list = (vx_reference*)appCntxt->dof2tracks_in_field;
        }

        /* vxTriangNode Param 2 ==> graph param 1. */
        ptkdemo_addParamByNodeIndex(appCntxt->vxSfmGraph,
                                    appCntxt->vxTriangNode,
                                    2);
        q[cnt++].refs_list = (vx_reference*)appCntxt->triang_in_pose;
    }
    else
    {
       /* vxOgNode Param 1 ==> graph param 0. */
        ptkdemo_addParamByNodeIndex(appCntxt->vxSfmGraph,
                                    appCntxt->vxOgNode,
                                    1);
        q[cnt++].refs_list = (vx_reference*)appCntxt->vxOgInPoints3d;
    }

    /* vxOgNode Param 2 ==> graph param 2/1. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxSfmGraph,
                                appCntxt->vxOgNode,
                                2);

    q[cnt++].refs_list = (vx_reference*)appCntxt->points3d_transform;

    /* vxOgNode Param 3 ==> graph param 3/2. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxSfmGraph,
                                appCntxt->vxOgNode,
                                3);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxOgInPose;

    /* vxOgNode Param 4 ==> graph param 4/3. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxSfmGraph,
                                appCntxt->vxOgNode,
                                4);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxPoseAndRef;

    /* vxOgNode Param 5 ==> graph param 5/4. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxSfmGraph,
                                appCntxt->vxOgNode,
                                5);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxPfsdOutDesc;

    /* vxOgNode Param 4 ==> graph param 6/5. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxSfmGraph,
                                appCntxt->vxOgNode,
                                7);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxOutInstMap;

    for (uint32_t i = 0; i < cnt; i++)
    {
        q[i].graph_parameter_index = i;
        q[i].refs_list_size        = appCntxt->pipelineDepth;
    }

    for (uint32_t i = 0; i < appCntxt->pipelineDepth; i++)
    {
        paramDesc                      = &appCntxt->paramDesc[i];
        if (appCntxt->enableDof)
        {
            paramDesc->pyr_in_image        = appCntxt->pyr_in_image[i];
        } 
        else
        {
            paramDesc->dof2tracks_in_field = appCntxt->dof2tracks_in_field[i];
        }
        paramDesc->triang_in_pose      = appCntxt->triang_in_pose[i];
        paramDesc->vxOgInPoints3d      = appCntxt->vxOgInPoints3d[i];
        paramDesc->points3d_transform  = appCntxt->points3d_transform[i];
        paramDesc->vxOgInPose          = appCntxt->vxOgInPose[i];
        paramDesc->vxPoseAndRef        = appCntxt->vxPoseAndRef[i];
        paramDesc->vxPfsdOutDesc       = appCntxt->vxPfsdOutDesc[i];
        paramDesc->vxOutInstMap        = appCntxt->vxOutInstMap[i];
        appCntxt->freeQ.push(paramDesc);
    }

    vxStatus = vxSetGraphScheduleConfig(appCntxt->vxSfmGraph,
                                        VX_GRAPH_SCHEDULE_MODE_QUEUE_AUTO,
                                        cnt,
                                        q);
    if (vxStatus == VX_SUCCESS)
    {
        /* explicitly set graph pipeline depth */
        vxStatus = tivxSetGraphPipelineDepth(appCntxt->vxSfmGraph,
                                             appCntxt->pipelineDepth);

        if (vxStatus != VX_SUCCESS)
        {
            PTK_printf("[%s:%d] tivxSetGraphPipelineDepth() failed\n",
                       __FUNCTION__, __LINE__);
        }
    }
    else
    {
        PTK_printf("[%s:%d] vxSetGraphScheduleConfig() failed\n",
                   __FUNCTION__, __LINE__);
    }

    if (vxStatus == VX_SUCCESS)
    {
        uint32_t    appValue;

        appValue = appCntxt->vxEvtAppValBase +
                   SFMOGAPPLIB_GRAPH_COMPLETE_EVENT;

        vxStatus = vxRegisterEvent((vx_reference)appCntxt->vxSfmGraph,
                                   VX_EVENT_GRAPH_COMPLETED,
                                   0,
                                   appValue);

        if (vxStatus != VX_SUCCESS)
        {
            PTK_printf("[%s:%d] vxRegisterEvent() failed\n",
                       __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == VX_SUCCESS)
    {
        if (appCntxt->generatePC)
        {
            if (appCntxt->enableDof)
            {
                vxStatus = tivxSetNodeParameterNumBufByIndex(appCntxt->vxPyrNode,
                                                             1,
                                                             appCntxt->pipelineDepth);
                PTK_assert(vxStatus == VX_SUCCESS);
    
                vxStatus = tivxSetNodeParameterNumBufByIndex(appCntxt->vxDofNode,
                                                             8,
                                                             appCntxt->pipelineDepth);
                PTK_assert(vxStatus == VX_SUCCESS);
            }

            vxStatus = tivxSetNodeParameterNumBufByIndex(appCntxt->vxDof2tracksNode,
                                                         3,
                                                         appCntxt->pipelineDepth);
            PTK_assert(vxStatus == VX_SUCCESS);


            vxStatus = tivxSetNodeParameterNumBufByIndex(appCntxt->vxTriangNode,
                                                         3,
                                                         appCntxt->pipelineDepth);
            PTK_assert(vxStatus == VX_SUCCESS);
        }
    }

    return vxStatus;
}


vx_status SFMOGAPPLIB_createGraph(SFMOGAPPLIB_Context *appCntxt)
{
    vx_status               status = VX_SUCCESS;

    /*******************************************************/
    /* CREATE GRAPHS                                       */
    /*******************************************************/
    appCntxt->vxSfmGraph = vxCreateGraph(appCntxt->vxContext);
    PTK_assert(appCntxt->vxSfmGraph);
    vxSetReferenceName((vx_reference)appCntxt->vxSfmGraph,
                       "SFM OGMap Graph");


    if (appCntxt->generatePC)
    {
        status = SFMOGAPPLIB_createPCNodesInGraph(appCntxt);
        PTK_assert(status==VX_SUCCESS);
    }

    status = SFMOGAPPLIB_createOGNodesInGraph(appCntxt);
    PTK_assert(status==VX_SUCCESS);

    if (appCntxt->generatePC)
    {
        if (appCntxt->enableDof)
        {
            status = vxRegisterAutoAging(appCntxt->vxSfmGraph, appCntxt->pyr_out_delay);
            PTK_assert(status==VX_SUCCESS);
            #if DOF_TEMP_PRED
            status = vxRegisterAutoAging(appCntxt->vxSfmGraph, appCntxt->dof_out_delay);
            PTK_assert(status==VX_SUCCESS);
            #endif
        }
    }


    /* set up the pipeline. */
    status = SFMOGAPPLIB_setupPipeline(appCntxt);

    if (status == (vx_status)VX_SUCCESS)
    {
        status = vxVerifyGraph(appCntxt->vxSfmGraph);

        if (status != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] vxVerifyGraph() failed.\n");
        }
    }

    if (status == (vx_status)VX_SUCCESS)
    {
        if (appCntxt->exportGraph == 1)
        {
            tivxExportGraphToDot(appCntxt->vxSfmGraph, ".",
                                 "vx_app_dof_sfm");
        }

        if (appCntxt->rtLogEnable == 1)
        {
            tivxLogRtTraceEnable(appCntxt->vxSfmGraph);
        }
    }

    /** Cache the reference to the accumulated map. This is persistent so it is
     * safe to cache it.
     */
    if (status == (vx_status)VX_SUCCESS)
    {
        appPerfPointSetName(&appCntxt->perf , "SFM OGMAP");

        appCntxt->outAccMap =
            (PTK_Map*)ptkdemo_getUserObjDataPayload(appCntxt->vxOutAccMap);

        if (appCntxt->outAccMap == NULL)
        {
            PTK_printf("[%s:%d] ptkdemo_getUserObjDataPayload() failed.\n",
                       __FUNCTION__,
                       __LINE__);

            status = VX_FAILURE;
        }
    }

    return status;

} /* SFMOGAPPLIB_createGraph */

void SFMOGAPPLIB_releaseGraph(SFMOGAPPLIB_Context * appCntxt)
{
    uint32_t              i;

    if (appCntxt->generatePC)
    {
        if (appCntxt->enableDof)
        {
            vxReleaseNode(&appCntxt->vxPyrNode);
            vxReleaseNode(&appCntxt->vxDofNode);
        }
        vxReleaseNode(&appCntxt->vxTriangNode);
        vxReleaseNode(&appCntxt->vxDof2tracksNode);

        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            if (appCntxt->enableDof)
            {
                vxReleaseImage(&appCntxt->pyr_in_image[i]);
            } 
            else 
            {
                vxReleaseImage(&appCntxt->dof2tracks_in_field[i]);
            }

            vxReleaseUserDataObject(&appCntxt->triang_in_pose[i]);
        }


        if (appCntxt->enableDof)
        {
            vxReleaseDelay(&appCntxt->pyr_out_delay);
            vxReleaseUserDataObject(&appCntxt->dof_in_config);
            #if DOF_TEMP_PRED
            vxReleaseDelay(&appCntxt->dof_out_delay);
            #else
            vxReleaseImage(&appCntxt->dof_out_field);
            #endif
        }



        vxReleaseUserDataObject(&appCntxt->dof2tracks_in_config);

        if (NULL != appCntxt->dof2tracks_in_lut)
        {
            vxReleaseLUT(&appCntxt->dof2tracks_in_lut);
        }

        vxReleaseUserDataObject(&appCntxt->triang_in_config);
        vxReleaseArray(&appCntxt->triang_in_tracks);
        vxReleaseArray(&appCntxt->triang_out_points3d);
    }

    vxReleaseNode(&appCntxt->vxOgNode);
    vxReleaseUserDataObject(&appCntxt->vxOgInConfig);

    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        vxReleaseUserDataObject(&appCntxt->points3d_transform[i]);
        vxReleaseUserDataObject(&appCntxt->vxOgInPose[i]);
        vxReleaseUserDataObject(&appCntxt->vxPoseAndRef[i]);
        vxReleaseUserDataObject(&appCntxt->vxPfsdOutDesc[i]);
        vxReleaseUserDataObject(&appCntxt->vxOutInstMap[i]);
    }

    vxReleaseUserDataObject(&appCntxt->vxOutAccMap);

    /* if point cloud is read from a file */
    if (appCntxt->generatePC == 0)
    {
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            vxReleaseArray(&appCntxt->vxOgInPoints3d[i]);
        }
    }

    if (appCntxt->rtLogEnable == 1)
    {
        tivxLogRtTraceDisable(appCntxt->vxSfmGraph);
    }

    vxReleaseGraph(&appCntxt->vxSfmGraph);

    return;

} /* SFMOGAPPLIB_releaseGraph */

int32_t SFMOGAPPLIB_reset(SFMOGAPPLIB_Handle handle)
{
    SFMOGAPPLIB_Context  *appCntxt;
    vx_status               vxStatus;

    appCntxt = (SFMOGAPPLIB_Context *)handle;

    vxStatus = tivxNodeSendCommand(appCntxt->vxOgNode,
                                   0,
                                   TIVX_KERNEL_SFM_OGMAP_RESET,
                                   NULL,
                                   0);

    if (vxStatus != VX_SUCCESS)
    {
        PTK_printf("[%s:%d] tivxNodeSendCommand() failed.\n",
                   __FUNCTION__,
                   __LINE__);

        return -1;
    }

    /* Reset the accumulated map. */
    PTK_Map_clear(appCntxt->outAccMap);

    /* Reset the performance capture initialization flag. */
    appCntxt->startPerfCapt = false;

    return 0;
}

