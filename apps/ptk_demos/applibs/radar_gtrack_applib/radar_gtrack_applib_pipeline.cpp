#include "radar_gtrack_applib_priv.h"

vx_status RADAR_GTRACK_APPLIB_releaseParamRsrc(
        RADAR_GTRACK_APPLIB_Context    *appCntxt,
        uint32_t                        rsrcIndex)
{
    std::unique_lock<std::mutex> lock(appCntxt->paramRsrcMutex);
    RADAR_GTRACK_APPLIB_graphParams    *desc;

    desc = &appCntxt->paramDesc[rsrcIndex];

    /* Move the descriptor to the output queue. */
    appCntxt->outputQ.push(desc);

    return VX_SUCCESS;
}

vx_status RADAR_GTRACK_APPLIB_setupPipeline(RADAR_GTRACK_APPLIB_Context  *appCntxt)
{
    vx_graph_parameter_queue_params_t   q[RADAR_GTRACK_APPLIB_NUM_GRAPH_PARAMS];
    RADAR_GTRACK_APPLIB_graphParams    *paramDesc;
    uint32_t                            cnt = 0;
    uint32_t                            i;
    vx_status                           vxStatus;

    /* vxGTrackNode Param 1 ==> graph param 0. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph, appCntxt->vxGTrackNode, 1);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxSensorCfg;

    /* vxGTrackNode Param 2 ==> graph param 1. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph, appCntxt->vxGTrackNode, 2);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxObjData;

    /* vxGTrackNode Param 3 ==> graph param 2. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph, appCntxt->vxGTrackNode, 3);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxPoseAndRef;

    /* vxGTrackNode Param 4 ==> graph param 3. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph, appCntxt->vxGTrackNode, 4);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxOutTrackInfo;

    for (i = 0; i < cnt; i++)
    {
        q[i].graph_parameter_index = i;
        q[i].refs_list_size        = appCntxt->pipelineDepth;
    }

    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        paramDesc                 = &appCntxt->paramDesc[i];
        paramDesc->vxSensorCfg    = appCntxt->vxSensorCfg[i];
        paramDesc->vxObjData      = appCntxt->vxObjData[i];
        paramDesc->vxPoseAndRef   = appCntxt->vxPoseAndRef[i];
        paramDesc->vxOutTrackInfo = appCntxt->vxOutTrackInfo[i];
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
                   RADAR_GTRACK_APPLIB_GRAPH_COMPLETE_EVENT;

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

vx_status RADAR_GTRACK_APPLIB_getFreeParamRsrc(
        RADAR_GTRACK_APPLIB_Context        *appCntxt,
        RADAR_GTRACK_APPLIB_graphParams    *gpDesc)
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
