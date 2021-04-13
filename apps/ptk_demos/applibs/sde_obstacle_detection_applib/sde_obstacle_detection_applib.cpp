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

#include "TI/tivx_target_kernel.h"
#include "sde_obstacle_detection_applib.h"

typedef struct SODAPPLIB_Context
{
    /** OVX Node References */
    vx_context                        vxContext;

    /** OpenVX graph */
    vx_graph                          vxGraph;

    /** Obstacle detection node */
    vx_node                           vxSodNode;

    /** Ground estimation node */
    vx_node                           vxGeNode;

    /** Handle to the data object holding the obstacle detection 
     *  configuration parameters 
     */
    vx_user_data_object               vxSodInConfig;

    /** Handle to the data object holding the ground estimation
     *  configuration parameters 
     */
    vx_user_data_object               vxGeInConfig;

    /** Right rectified image object */
    vx_image                          vxRightRectImage[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Raw disparity map object */
    vx_image                          vxSde16BitOutput[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Disparity map after discarding low-confidence ones*/
    vx_image                          vxPostDisparity;

    /** Estimated ground model */
    vx_user_data_object               vxGroundModel;

    /** Output obstacles' poses */
    vx_array                          vxObstaclesPose[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Output number of obstacles */
    vx_scalar                         vxNumObstacles[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Output free space */
    vx_array                          vxFreeSpaceBoundary[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Output drivable space */
    vx_user_data_object               vxDrivableSpace[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** SDE configuration parameters */
    tivx_dmpac_sde_params_t           sdeCfg;

    /** Ground estimation configuration parameters */
    tivx_ground_estimation_params_t   geCfg;

    /** Obstacled detection configuration parameters */
    tivx_obstacle_detection_params_t  sodCfg;

    /** Ground model parameters */
    tivx_ground_model_params_t        gmCfg;

    /** Input width */
    uint32_t                          width;

    /** Input height */
    uint32_t                          height;

    /** Input format,  0: obstacle detection, 1: free space detection */
    uint8_t                           inputFormat;

    /** GE Node Core mapping. */
    const char                      * geNodeCore;
    /** SOD Node Core mapping. */
    const char                      * sodNodeCore;
    /** FSD Node Core mapping. */
    const char                      * fsdNodeCore;

    /** pipeline depth */
    uint8_t                           pipelineDepth;
    
    /** Input pipeline depth */
    uint8_t                           inputPipelineDepth;

    /** flag indicating whether or not create input OpenVX object in applib */
    uint8_t                           createInputFlag;

    /** flag indicating whether or not create output OpenVX object in applib */
    uint8_t                           createOutputFlag;

} SODAPPLIB_Context;

static void      SODAPPLIB_setParams(SODAPPLIB_Handle        handle,
                                     SODAPPLIB_createParams *createParams);
static vx_status SODAPPLIB_createGraph(SODAPPLIB_Handle handle);
static void      SODAPPLIB_releaseGraph(SODAPPLIB_Handle handle);

SODAPPLIB_Handle SODAPPLIB_create(SODAPPLIB_createParams *createParams)
{
    SODAPPLIB_Handle      handle;
    vx_status             vxStatus;

    handle = new SODAPPLIB_Context();

    /* Set applib-level create parameters */
    SODAPPLIB_setParams(handle, createParams);

    /* Create nodes and graph */
    vxStatus = SODAPPLIB_createGraph(handle);

    if (vxStatus != VX_SUCCESS)
    {
        delete handle;
        handle = NULL;
    }

    return handle;

}


void SODAPPLIB_delete(SODAPPLIB_Handle *handle)
{
    if (*handle)
    {
        SODAPPLIB_releaseGraph(*handle);

        delete *handle;
        *handle = NULL;
    }

    return;

} /* SODAPPLIB_delete */


void SODAPPLIB_setParams(SODAPPLIB_Handle        handle,
                         SODAPPLIB_createParams *createParams)
{
    int32_t                i;
    vx_enum                config_type;
    SODAPPLIB_Context    * appCntxt;
    appCntxt = (SODAPPLIB_Context *)handle;

    appCntxt->vxContext          = createParams->vxContext;
    appCntxt->vxGraph            = createParams->vxGraph;
    appCntxt->pipelineDepth      = createParams->pipelineDepth;
    appCntxt->inputPipelineDepth = createParams->inputPipelineDepth;
    appCntxt->createInputFlag    = createParams->createInputFlag;
    appCntxt->createOutputFlag   = createParams->createOutputFlag;
    appCntxt->inputFormat        = createParams->inputFormat;
    
    appCntxt->width              = createParams->width;
    appCntxt->height             = createParams->height;
    appCntxt->sdeCfg             = createParams->sdeCfg;
    appCntxt->geCfg              = createParams->geCfg;
    appCntxt->sodCfg             = createParams->sodCfg;

    /* ground model params */
    appCntxt->gmCfg.numModels = NUM_GP_MODELS;

    for (int32_t i = 0; i < NUM_GP_MODELS; i++)
    {
        appCntxt->gmCfg.gmParams[i].validGM    = 0;
    }

    appCntxt->geNodeCore      = createParams->geNodeCore;
    appCntxt->sodNodeCore     = createParams->sodNodeCore;

    if (appCntxt->geNodeCore == NULL)
    {
        appCntxt->geNodeCore = SODAPPLIB_DEFAULT_CORE_MAPPING;
    }
    if (appCntxt->sodNodeCore == NULL)
    {
        appCntxt->sodNodeCore = SODAPPLIB_DEFAULT_CORE_MAPPING;
    }

    /*
     * set up input objects
     */
    if (appCntxt->createInputFlag == 1)
    {
        // input rectified right image
        if (appCntxt->inputFormat == PTK_IMG_FORMAT_Y)
        {
            appCntxt->vxRightRectImage[0] = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_U8);
        } else 
        {
            appCntxt->vxRightRectImage[0] = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_NV12);
        }
        if (appCntxt->vxRightRectImage[0] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);
        }
        vxSetReferenceName((vx_reference)appCntxt->vxRightRectImage[0], "Right_Rectified_Image");

        appCntxt->vxSde16BitOutput[0] = vxCreateImage(appCntxt->vxContext, appCntxt->geCfg.config.width, appCntxt->geCfg.config.height, VX_DF_IMAGE_S16);
        if (appCntxt->vxSde16BitOutput[0] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);
        }
        vxSetReferenceName((vx_reference)appCntxt->vxSde16BitOutput[0], "Stereo_OutputDisparityImageS16");
    } else 
    {
        for(i = 0; i < appCntxt->pipelineDepth; i++)
        {
            appCntxt->vxRightRectImage[i] = createParams->vxRightRectImage[i];
            appCntxt->vxSde16BitOutput[i] = createParams->vxSde16BitOutput[i];
        }
    }

    /*
     * set up output objects
     */
    if (appCntxt->createOutputFlag == 1)
    {
        config_type = vxRegisterUserStruct(appCntxt->vxContext, sizeof(tivx_obstacle_pos_t));
        appCntxt->vxObstaclesPose[0] = vxCreateArray(appCntxt->vxContext, config_type, MAX_DETECTIONS);
        if (appCntxt->vxObstaclesPose[0] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);
        }
        vxSetReferenceName((vx_reference)appCntxt->vxObstaclesPose[0], "ObstaclePoses");

        appCntxt->vxNumObstacles[0] = vxCreateScalar(appCntxt->vxContext, VX_TYPE_UINT32, NULL);
        if (appCntxt->vxNumObstacles[0] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);
        }
        vxSetReferenceName((vx_reference)appCntxt->vxNumObstacles[0], "OutputNumObstacles");

        appCntxt->vxFreeSpaceBoundary[0] = vxCreateArray(appCntxt->vxContext, VX_TYPE_INT32, appCntxt->sodCfg.copppParams.dsWidth);
        if (appCntxt->vxFreeSpaceBoundary[0] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);
        }
        vxSetReferenceName((vx_reference)appCntxt->vxFreeSpaceBoundary[0], "OutputFreeSpaceBoundary");

        appCntxt->vxDrivableSpace[0] =
                vxCreateUserDataObject(appCntxt->vxContext,
                                       "tivx_drivable_space_t",
                                       sizeof(tivx_drivable_space_t),
                                       NULL);
        if (appCntxt->vxDrivableSpace[0] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);
        }
        vxSetReferenceName((vx_reference)appCntxt->vxDrivableSpace[0], "OutputDrivableSpace");

    } else
    {
        for(i = 0; i < appCntxt->pipelineDepth; i++)
        {
            appCntxt->vxObstaclesPose[i]     = createParams->vxObstaclesPose[i];
            appCntxt->vxNumObstacles[i]      = createParams->vxNumObstacles[i];
            appCntxt->vxFreeSpaceBoundary[i] = createParams->vxFreeSpaceBoundary[i];
            appCntxt->vxDrivableSpace[i]     = createParams->vxDrivableSpace[i];
        }
    }
}

vx_status SODAPPLIB_createGraph(SODAPPLIB_Handle handle)
{
    SODAPPLIB_Context    *appCntxt;
    vx_status             status = VX_SUCCESS;
    
    appCntxt = (SODAPPLIB_Context *)handle;

    /* Mapping Graph */
    PTK_assert(appCntxt->vxGraph);

    /*********************************/
    /* Stereo ground estimation node */
    /*********************************/
    appCntxt->vxGeInConfig =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "tivx_ground_estimation_params_t",
                                   sizeof(tivx_ground_estimation_params_t),
                                   &appCntxt->geCfg);
    PTK_assert(appCntxt->vxGeInConfig);
    vxSetReferenceName((vx_reference)appCntxt->vxGeInConfig, "GEConfig");

    /* post-processed disparity (float) after discarding low-confidence ones*/
    appCntxt->vxPostDisparity = vxCreateImage(appCntxt->vxContext, appCntxt->geCfg.config.width, appCntxt->geCfg.config.height, VX_DF_IMAGE_S16);
    PTK_assert(appCntxt->vxPostDisparity);
    vxSetReferenceName((vx_reference)appCntxt->vxPostDisparity, "OutFilteredDisparity");

    appCntxt->vxGroundModel =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "tivx_ground_model_params_t",
                                   sizeof(tivx_ground_model_params_t),
                                   &appCntxt->gmCfg);
    PTK_assert(appCntxt->vxGroundModel);
    vxSetReferenceName((vx_reference)appCntxt->vxGroundModel, "InOutGroundModel");

    appCntxt->vxGeNode = tivxGroundEstimationNode(appCntxt->vxGraph,
                                                  appCntxt->vxGeInConfig,
                                                  appCntxt->vxRightRectImage[0],
                                                  appCntxt->vxSde16BitOutput[0],
                                                  appCntxt->vxPostDisparity,
                                                  appCntxt->vxGroundModel);

    PTK_assert(appCntxt->vxGeNode);
    vxSetReferenceName((vx_reference)appCntxt->vxGeNode, "StereoGroundEsimation");

    status = vxSetNodeTarget(appCntxt->vxGeNode,
                             VX_TARGET_STRING,
                             appCntxt->geNodeCore);
    
    PTK_assert(status==VX_SUCCESS);

    /**********************************/
    /* Stereo obstacle detection node */
    /**********************************/
    appCntxt->vxSodInConfig =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "tivx_obstacle_detection_params_t",
                                   sizeof(tivx_obstacle_detection_params_t),
                                   &appCntxt->sodCfg);
    PTK_assert(appCntxt->vxSodInConfig);
    vxSetReferenceName((vx_reference)appCntxt->vxSodInConfig, "SODConfig");

    // for obstacle detection output
    appCntxt->vxSodNode = tivxObstacleDetectionNode(appCntxt->vxGraph,
                                                    appCntxt->vxSodInConfig,
                                                    appCntxt->vxRightRectImage[0],
                                                    appCntxt->vxPostDisparity,
                                                    appCntxt->vxGroundModel,
                                                    appCntxt->vxObstaclesPose[0],
                                                    appCntxt->vxNumObstacles[0],
                                                    appCntxt->vxFreeSpaceBoundary[0],
                                                    appCntxt->vxDrivableSpace[0]);
    PTK_assert(appCntxt->vxSodNode);
    vxSetReferenceName((vx_reference)appCntxt->vxSodNode, "StereoObstacleDetection");

    status = vxSetNodeTarget(appCntxt->vxSodNode,
                             VX_TARGET_STRING,
                             appCntxt->sodNodeCore);

    PTK_assert(status==VX_SUCCESS);

    return status;

} /* SODAPPLIB_createGraph */


void SODAPPLIB_releaseGraph(SODAPPLIB_Handle handle)
{
    SODAPPLIB_Context *appCntxt = (SODAPPLIB_Context *)handle;

    if (appCntxt->createInputFlag == 1)
    {
        vxReleaseImage(&appCntxt->vxRightRectImage[0]);
        vxReleaseImage(&appCntxt->vxSde16BitOutput[0]);
    }

    /* release ground estimation node */
    vxReleaseNode(&appCntxt->vxGeNode);
    vxReleaseUserDataObject(&appCntxt->vxGeInConfig);
    vxReleaseUserDataObject(&appCntxt->vxGroundModel);
    vxReleaseImage(&appCntxt->vxPostDisparity);

    /* release obstacle detection node */
    vxReleaseNode(&appCntxt->vxSodNode);
    vxReleaseUserDataObject(&appCntxt->vxSodInConfig);

    if (appCntxt->createOutputFlag == 1)
    {
        // for obstacle detection output
        vxReleaseArray(&appCntxt->vxObstaclesPose[0]);
        vxReleaseScalar(&appCntxt->vxNumObstacles[0]);

        // for freespace detection output
        vxReleaseArray(&appCntxt->vxFreeSpaceBoundary[0]);
        vxReleaseUserDataObject(&appCntxt->vxDrivableSpace[0]);
    }

    return;

} /* SODAPPLIB_releaseGraph */

vx_node SODAPPLIB_getGENode(SODAPPLIB_Handle handle)
{
    SODAPPLIB_Context *appCntxt = (SODAPPLIB_Context *)handle;
    return appCntxt->vxGeNode;
}

vx_node SODAPPLIB_getSODNode(SODAPPLIB_Handle handle)
{
    SODAPPLIB_Context *appCntxt = (SODAPPLIB_Context *)handle;
    return appCntxt->vxSodNode;
}
