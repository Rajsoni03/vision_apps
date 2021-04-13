#include "surround_radar_ogmap_applib_priv.h"

static vx_status RADAROGAPPLIB_updateVehiclePos(
        RADAROGAPPLIB_Context  *appCntxt,
        vx_user_data_object     vxPoseAndRef)
{
    PTK_Alg_RadarOgmapParams   *ogConfig;
    PTK_Grid                   *occupancy;
    PTK_InsPoseAndRef          *poseAndRef;
    PTK_INS_Record             *curInsRec;
    void                       *ptr;
    PTK_Position                refPosition;
    PTK_RigidTransform          M_ref_enu;
    vx_status                   vxStatus;

    ogConfig = &appCntxt->ogPfsdCfg.ogConfig;
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
                        appCntxt->ogFlagEgo);
    }

    return vxStatus;
}

vx_status RADAROGAPPLIB_releaseParamRsrc(
        RADAROGAPPLIB_Context  *appCntxt,
        uint32_t                rsrcIndex)
{
    RADAROGAPPLIB_graphParams  *desc;
    vx_status                   vxStatus;
    std::unique_lock<std::mutex> lock(appCntxt->paramRsrcMutex);

    desc = &appCntxt->paramDesc[rsrcIndex];

    /* Update the vecicle position. */
    vxStatus = RADAROGAPPLIB_updateVehiclePos(appCntxt, desc->vxPoseAndRef);

    appCntxt->outputQ.push(desc);

    return vxStatus;
}

vx_status RADAROGAPPLIB_setupPipeline(
        RADAROGAPPLIB_Context  *appCntxt)
{
    vx_graph_parameter_queue_params_t   q[RADAROGAPPLIB_NUM_GRAPH_PARAMS];
    RADAROGAPPLIB_graphParams          *paramDesc;
    uint32_t                            cnt = 0;
    vx_status                           vxStatus;

    /* vxOgmapNode Param 1 ==> graph param 0. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph, appCntxt->vxOgmapNode, 1);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxSensorCfg;

    /* vxOgmapNode Param 2 ==> graph param 1. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph, appCntxt->vxOgmapNode, 2);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxObjData;

    /* vxOgmapNode Param 3 ==> graph param 2. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph, appCntxt->vxOgmapNode, 3);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxPoseAndRef;

    /* vxOgmapNode Param 4 ==> graph param 3. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph, appCntxt->vxOgmapNode, 4);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxPfsdOutDesc;

    /* vxOgmapNode Param 6 ==> graph param 4. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph, appCntxt->vxOgmapNode, 6);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxOutInstMap;

    for (uint32_t i = 0; i < cnt; i++)
    {
        q[i].graph_parameter_index = i;
        q[i].refs_list_size        = appCntxt->pipelineDepth;
    }

    for (uint32_t i = 0; i < appCntxt->pipelineDepth; i++)
    {
        paramDesc                = &appCntxt->paramDesc[i];
        paramDesc->vxSensorCfg   = appCntxt->vxSensorCfg[i];
        paramDesc->vxObjData     = appCntxt->vxObjData[i];
        paramDesc->vxPoseAndRef  = appCntxt->vxPoseAndRef[i];
        paramDesc->vxPfsdOutDesc = appCntxt->vxPfsdOutDesc[i];
        paramDesc->vxOutInstMap  = appCntxt->vxOutInstMap[i];
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

        appValue = appCntxt->vxEvtAppValBase +
                   RADAROGAPPLIB_GRAPH_COMPLETE_EVENT;

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

    return vxStatus;
}

vx_status RADAROGAPPLIB_getFreeParamRsrc(
        RADAROGAPPLIB_Context      *appCntxt,
        RADAROGAPPLIB_graphParams  *gpDesc)
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
