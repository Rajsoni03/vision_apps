#include "lidar_ogmap_applib_priv.h"

static vx_status LIDAROGAPPLIB_updateVehiclePos(
        LIDAROGAPPLIB_Context  *appCntxt,
        vx_user_data_object     vxPoseAndRef)
{
    PTK_Alg_LidarOgmapParams   *ogConfig;
    PTK_Grid                   *occupancy;
    PTK_InsPoseAndRef          *poseAndRef;
    PTK_INS_Record             *curInsRec;
    void                       *ptr;
    PTK_Position                refPosition;
    PTK_RigidTransform          M_ref_enu;
    vx_status                   vxStatus;

    ogConfig = &appCntxt->params.ogPfsdCfg.ogConfig;
    vxStatus = VX_SUCCESS;

    /* Set car's position in OG map */
    occupancy = PTK_Map_get(appCntxt->outAccMap, ogConfig->accGridId);

    if (occupancy == NULL)
    {
        PTK_printf("[%s:%d] PTK_Map_get() failed.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Get a reference to the Pose and reference buffer. */
        ptr = ptkdemo_getUserObjDataPayload(vxPoseAndRef);

        if (ptr == NULL)
        {
            PTK_printf("[%s:%d] ptkdemo_getUserObjDataPayload() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        poseAndRef = (PTK_InsPoseAndRef*)ptr;
        curInsRec  = &poseAndRef->curInsRec;

        PTK_INS_getReferenceFrame(NULL, &refPosition);
        PTK_Position_getENUtoENU(&curInsRec->data.inspva.position,
                                 &refPosition,
                                 &M_ref_enu);

        PTK_Grid_setb2d(occupancy,
                        PTK_Grid_getXCell(occupancy, M_ref_enu.M[3]),
                        PTK_Grid_getYCell(occupancy, M_ref_enu.M[7]),
                        appCntxt->params.ogFlagEgo);
    }

    return vxStatus;
}

vx_status LIDAROGAPPLIB_releaseParamRsrc(
        LIDAROGAPPLIB_Context  *appCntxt,
        uint32_t                rsrcIndex)
{
    LIDAROGAPPLIB_graphParams  *desc;
    vx_status                   vxStatus;
    std::unique_lock<std::mutex> lock(appCntxt->paramRsrcMutex);

    desc = &appCntxt->paramDesc[rsrcIndex];

    vxStatus = LIDAROGAPPLIB_updateVehiclePos(appCntxt, desc->vxPoseAndRef);

    /* Move the descriptor to the output queue. */
    appCntxt->outputQ.push(desc);

    return vxStatus;
}

vx_status LIDAROGAPPLIB_setupPipeline(
        LIDAROGAPPLIB_Context  *appCntxt)
{
    vx_graph_parameter_queue_params_t   q[LIDAROGAPPLIB_NUM_GRAPH_PARAMS];
    uint32_t                            cnt = 0;
    uint32_t                            i;
    vx_status                           vxStatus;

    /* mdcNode Param 0 ==> Graph parameter 0. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph, appCntxt->mdcNode, 0);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxLidarPointCloud;

    /* gpcNode Param 2 ==> Graph parameter 1. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph, appCntxt->gpcNode, 2);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxLidarMeta;

    /* ogNode Param 2 ==> Graph parameter 2. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph, appCntxt->ogNode, 2);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxPoseAndRef;

    /* ogNode Param 5 ==> Graph parameter 3. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph, appCntxt->ogNode, 3);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxPfsdOutDesc;

    /* ogNode Param 5 ==> Graph parameter 4. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph, appCntxt->ogNode, 5);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxOutInstMap;

    for (i = 0; i < cnt; i++)
    {
        q[i].graph_parameter_index = i;
        q[i].refs_list_size        = appCntxt->pipelineDepth;
    }

    /* Create the OG node param descriptors and push them to the
     * free queue.
     */
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        LIDAROGAPPLIB_graphParams  *paramDesc;

        paramDesc                    = &appCntxt->paramDesc[i];
        paramDesc->vxLidarPointCloud = appCntxt->vxLidarPointCloud[i];
        paramDesc->vxLidarMeta       = appCntxt->vxLidarMeta[i];
        paramDesc->vxPoseAndRef      = appCntxt->vxPoseAndRef[i];
        paramDesc->vxPfsdOutDesc     = appCntxt->vxPfsdOutDesc[i];
        paramDesc->vxOutInstMap      = appCntxt->vxOutInstMap[i];
        appCntxt->freeQ.push(paramDesc);
    }

    vxStatus = vxSetGraphScheduleConfig(appCntxt->vxGraph,
                                        VX_GRAPH_SCHEDULE_MODE_QUEUE_AUTO,
                                        cnt,
                                        q);
    if (vxStatus == VX_SUCCESS)
    {
        /* explicitly set graph pipeline depth */
        vxStatus = tivxSetGraphPipelineDepth(appCntxt->vxGraph,
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

        appValue = appCntxt->params.vxEvtAppValBase +
                   LIDAROGAPPLIB_GRAPH_COMPLETE_EVENT;

        vxStatus = vxRegisterEvent((vx_reference)appCntxt->vxGraph,
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
        /* The 'gpcNode' and 'ogNode' can run on different cores so the
         * 'vxLidarGpcPointCloud' should be configured to have multiple
         * buffers for effective pipelining.
         */
        vxStatus =
        tivxSetNodeParameterNumBufByIndex(appCntxt->gpcNode,
                                          5,
                                          appCntxt->pipelineDepth);

        if (vxStatus != VX_SUCCESS)
        {
            PTK_printf("[%s:%d] tivxSetNodeParameterNumBufByIndex() failed\n",
                       __FUNCTION__, __LINE__);
        }
    }

    return vxStatus;
}

vx_status LIDAROGAPPLIB_getFreeParamRsrc(
        LIDAROGAPPLIB_Context      *appCntxt,
        LIDAROGAPPLIB_graphParams  *gpDesc)
{
    std::unique_lock<std::mutex> lock(appCntxt->paramRsrcMutex);
    vx_status                    vxStatus;

    vxStatus = VX_SUCCESS;

    /* Check if a free descriptor available. */
    if (appCntxt->freeQ.empty())
    {
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* We have all the resources we need. */
        *gpDesc = *appCntxt->freeQ.front();
        appCntxt->freeQ.pop();
    }

    return vxStatus;
}

