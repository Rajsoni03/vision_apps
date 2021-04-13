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
 * licensed and provided to you in appCntxtect code.
 *
 * If software source code is provided to you, modification and redistribution of the
 * source code are permitted provided that the following conditions are met:
 *
 * *       any redistribution and use of the source code, including any resulting derivative
 * works, are licensed by TI for use only with TI Devices.
 *
 * *       any redistribution and use of any appCntxtect code compiled from the source code
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

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include "perception/utils/sde_rd_wr.h"

#include <app_ptk_demo_common.h>

#include "app_sde_obstacle_detection_main.h"
#include "app_sde_obstacle_detection.h"

int32_t SODAPP_init_LDC(SODAPP_Context *appCntxt)
{
    SDELDCAPPLIB_createParams * createParams;
    int32_t                     i, status = 0;

    createParams            = &appCntxt->sdeLdcCreateParams;
    createParams->vxContext = appCntxt->vxContext;
    createParams->vxGraph   = appCntxt->vxGraph;

    /*
     * Create input image objects 
     */
    createParams->createInputFlag      = 0;
    createParams->inputPipelineDepth   = appCntxt->pipelineDepth;
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        // input left image
        if (appCntxt->inputFormat == PTK_IMG_FORMAT_Y)
        {
            appCntxt->vxInputLeftImage[i]  = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_U8);
        } else 
        {
            appCntxt->vxInputLeftImage[i] = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_UYVY);
        }

        if (appCntxt->vxInputLeftImage[i] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);
            status = -1;
            break;
        }
        vxSetReferenceName((vx_reference)appCntxt->vxInputLeftImage[i], "InputLeftImage");

        // input right image
        if (appCntxt->inputFormat == PTK_IMG_FORMAT_Y)
        {
            appCntxt->vxInputRightImage[i] = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_U8);
        } else 
        {
            appCntxt->vxInputRightImage[i] = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_UYVY);
        }

        if (appCntxt->vxInputRightImage[i] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);
            status = -1;
            break;
        }
        vxSetReferenceName((vx_reference)appCntxt->vxInputRightImage[i], "InputRightImage");

        // pass to LDC Applib createParams
        createParams->vxInputLeftImage[i]  = appCntxt->vxInputLeftImage[i];
        createParams->vxInputRightImage[i] = appCntxt->vxInputRightImage[i];
    }

    if (status >= 0)
    {
        /*
         * Create output image objects 
         */
        createParams->createOutputFlag      = 0;
        createParams->outputPipelineDepth   = 1;

        // output left image
        if (appCntxt->inputFormat == PTK_IMG_FORMAT_Y)
        {
            appCntxt->vxLeftRectImage[0]  = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_U8);
        } else 
        {
            appCntxt->vxLeftRectImage[0] = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_NV12);
        }

        if (appCntxt->vxLeftRectImage[0] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);
            status = -1;
        }
        vxSetReferenceName((vx_reference)appCntxt->vxLeftRectImage[0], "LeftRectifiedImage");

        createParams->vxOutputLeftImage[0]  = appCntxt->vxLeftRectImage[0];

        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            // output right image
            if (appCntxt->inputFormat == PTK_IMG_FORMAT_Y)
            {
                appCntxt->vxRightRectImage[i]  = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_U8);
            } else 
            {
                appCntxt->vxRightRectImage[i]  = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_NV12);
            }

            if (appCntxt->vxRightRectImage[i] == NULL)
            {
                PTK_printf("[%s:%d] vxCreateImage() failed\n",
                            __FUNCTION__, __LINE__);
                status = -1;
                break;
            }
            vxSetReferenceName((vx_reference)appCntxt->vxRightRectImage[i], "RightRectifiedImage");

            // pass to LDC Applib createParams
            createParams->vxOutputRightImage[i] = appCntxt->vxRightRectImage[i];
        }
    }

    if (status >= 0)
    {
        appCntxt->sdeLdcHdl = SDELDCAPPLIB_create(createParams);
        if (appCntxt->sdeLdcHdl == NULL)
        {
            PTK_printf("[%s:%d] SDELDCAPPLIB_create() failed\n",
                        __FUNCTION__, __LINE__);
            status = -1;
        }
    }

    return status;
}

int32_t SODAPP_init_SDE(SODAPP_Context *appCntxt)
{
    int32_t i, status = 0;

    if (appCntxt->sdeAlgoType == 0)
    {
        SL_SDEAPPLIB_createParams   * slSdeCreateParams;

        slSdeCreateParams            = &appCntxt->slSdeCreateParams;
        slSdeCreateParams->vxContext = appCntxt->vxContext;
        slSdeCreateParams->vxGraph   = appCntxt->vxGraph;

        /* 
         * Create input image objects for SL SDE
         */
        slSdeCreateParams->createInputFlag     = 0;
        slSdeCreateParams->inputPipelineDepth  = 1;
        slSdeCreateParams->vxLeftRectImage[0]  = appCntxt->vxLeftRectImage[0];
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            slSdeCreateParams->vxRightRectImage[i] = appCntxt->vxRightRectImage[i];
        }

        /* 
         * Create output image objects for SL SDE
         */
        slSdeCreateParams->createOutputFlag    = 0;
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            appCntxt->vxSde16BitOutput[i]  = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_S16);
            if (appCntxt->vxSde16BitOutput[i] == NULL)
            {
                PTK_printf("[%s:%d] vxCreateImage() failed\n",
                            __FUNCTION__, __LINE__);
                status = -1;
                break;
            }
            vxSetReferenceName((vx_reference)appCntxt->vxSde16BitOutput[i], "RawDisparityMap");

            slSdeCreateParams->vxSde16BitOutput[i]  = appCntxt->vxSde16BitOutput[i];
        }

        if (status >= 0)
        {
            appCntxt->slSdeHdl = SL_SDEAPPLIB_create(slSdeCreateParams);
            if (appCntxt->slSdeHdl == NULL)
            {
                PTK_printf("[%s:%d] SL_SDEAPPLIB_create() failed\n",
                            __FUNCTION__, __LINE__);

                status = -1;
            }
        }
    }
    else if (appCntxt->sdeAlgoType == 1)
    {
        ML_SDEAPPLIB_createParams   * mlSdeCreateParams;

        mlSdeCreateParams            = &appCntxt->mlSdeCreateParams;
        mlSdeCreateParams->vxContext = appCntxt->vxContext;
        mlSdeCreateParams->vxGraph   = appCntxt->vxGraph;

        /* 
         * Create input image objects for SL SDE
         */
        mlSdeCreateParams->createInputFlag    = 0;
        mlSdeCreateParams->inputPipelineDepth = 1;
        mlSdeCreateParams->vxLeftRectImageL0[0]  = appCntxt->vxLeftRectImage[0];
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            mlSdeCreateParams->vxRightRectImageL0[i] = appCntxt->vxRightRectImage[i];
        }

        /* 
         * Create output image objects for ML SDE
         */
        mlSdeCreateParams->createOutputFlag  = 0;
#if ENABLE_PP_MEDIAN_FILTER
        if (appCntxt->ppMedianFilterEnable)
        {
            for (i = 0; i < appCntxt->pipelineDepth; i++)
            {
                appCntxt->vxMedianFilteredDisparity[i] = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_S16);
                if (appCntxt->vxMedianFilteredDisparity[i] == NULL)
                {
                    PTK_printf("[%s:%d] vxCreateImage() failed\n",
                                __FUNCTION__, __LINE__);
                    status = -1;
                    break;
                }
                mlSdeCreateParams->vxMedianFilteredDisparity[i]  = appCntxt->vxMedianFilteredDisparity[i];
            }
        }
#endif

        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            appCntxt->vxMergeDisparityL0[i] = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_S16);
            if (appCntxt->vxMergeDisparityL0[i] == NULL)
            {
                PTK_printf("[%s:%d] vxCreateImage() failed\n",
                            __FUNCTION__, __LINE__);
                status = -1;
                break;
            }
            mlSdeCreateParams->vxMergeDisparityL0[i]  = appCntxt->vxMergeDisparityL0[i];
        }

        if (status >= 0)
        {
            appCntxt->mlSdeHdl = ML_SDEAPPLIB_create(mlSdeCreateParams);
            if (appCntxt->mlSdeHdl == NULL)
            {
                PTK_printf("[%s:%d] ML_SDEAPPLIB_create() failed\n",
                            __FUNCTION__, __LINE__);
                status = -1;
            }
        }
    }

    return status;
}

int32_t SODAPP_init_Detection(SODAPP_Context *appCntxt)
{
    int32_t                  i, status = 0;
    vx_enum                  config_type;
    SODAPPLIB_createParams * sodCreateParams;

    sodCreateParams            = &appCntxt->sodCreateParams;
    sodCreateParams->vxContext = appCntxt->vxContext;
    sodCreateParams->vxGraph   = appCntxt->vxGraph;

    /* 
     * Create input image objects for SL SDE
     */
    sodCreateParams->createInputFlag = 0;
    sodCreateParams->inputPipelineDepth  = appCntxt->pipelineDepth;
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        sodCreateParams->vxRightRectImage[i] = appCntxt->vxRightRectImage[i];
        if (appCntxt->sdeAlgoType == 0)
        {
            sodCreateParams->vxSde16BitOutput[i] = appCntxt->vxSde16BitOutput[i];
        } else
        {
#if ENABLE_PP_MEDIAN_FILTER
            if (appCntxt->ppMedianFilterEnable)
            {
                sodCreateParams->vxSde16BitOutput[i] = appCntxt->vxMedianFilteredDisparity[i];
            } else
#endif
            {
                sodCreateParams->vxSde16BitOutput[i] = appCntxt->vxMergeDisparityL0[i];
            }
        }
    }

    /* 
     * Create output objects for SL SDE
     */
    sodCreateParams->createOutputFlag  = 0;
    config_type = vxRegisterUserStruct(appCntxt->vxContext, sizeof(tivx_obstacle_pos_t));
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        appCntxt->vxObstaclesPose[i] = vxCreateArray(appCntxt->vxContext, config_type, MAX_DETECTIONS);
        if (appCntxt->vxObstaclesPose[i] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);
            status = -1;
            break;
        }
        vxSetReferenceName((vx_reference)appCntxt->vxObstaclesPose[i], "ObstaclePoses");
        sodCreateParams->vxObstaclesPose[i]  = appCntxt->vxObstaclesPose[i];
    }

    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        appCntxt->vxNumObstacles[i] = vxCreateScalar(appCntxt->vxContext, VX_TYPE_UINT32, NULL);
        if (appCntxt->vxNumObstacles[i] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);
            status = -1;
            break;
        }
        vxSetReferenceName((vx_reference)appCntxt->vxNumObstacles[i], "OutputNumObstacles");
        sodCreateParams->vxNumObstacles[i] = appCntxt->vxNumObstacles[i];
    }

    // for freespace detection output
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        appCntxt->vxFreeSpaceBoundary[i] = vxCreateArray(appCntxt->vxContext, VX_TYPE_INT32, appCntxt->sodCreateParams.sodCfg.copppParams.dsWidth);
        if (appCntxt->vxFreeSpaceBoundary[i] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);
            status = -1;
            break;
        }
        vxSetReferenceName((vx_reference)appCntxt->vxFreeSpaceBoundary[i], "OutputFreeSpaceBoundary");
        sodCreateParams->vxFreeSpaceBoundary[i] = appCntxt->vxFreeSpaceBoundary[i];
    }

    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {    
        appCntxt->vxDrivableSpace[i] =
                vxCreateUserDataObject(appCntxt->vxContext,
                                       "tivx_drivable_space_t",
                                       sizeof(tivx_drivable_space_t),
                                       NULL);
        if (appCntxt->vxDrivableSpace[i] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);
            status = -1;
            break;
        }
        vxSetReferenceName((vx_reference)appCntxt->vxDrivableSpace[i], "OutputDrivableSpace");
        sodCreateParams->vxDrivableSpace[i] = appCntxt->vxDrivableSpace[i];
    }

    if (status >= 0)
    {
        appCntxt->sodHdl = SODAPPLIB_create(sodCreateParams);
        if (appCntxt->sodHdl == NULL)
        {
            PTK_printf("[%s:%d] SODAPPLIB_create() failed\n",
                        __FUNCTION__, __LINE__);
            status = -1;
        }
    }

    return status;
}



vx_status  SODAPP_setupPipeline(SODAPP_Context * appCntxt)
{
    vx_status vxStatus = VX_SUCCESS;

    if (appCntxt->sdeAlgoType == 0)
    {
        vxStatus = SODAPP_setupPipeline_SL(appCntxt);
    } else
    {
        vxStatus = SODAPP_setupPipeline_ML(appCntxt);
    }

    if (vxStatus == VX_SUCCESS)
    {
        uint32_t   appValue;
        appValue = appCntxt->vxEvtAppValBase + SODAPP_GRAPH_COMPLETE_EVENT;

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

vx_status SODAPP_setupPipeline_SL(SODAPP_Context * appCntxt)
{
    SODAPP_graphParams                * paramDesc;
    vx_graph_parameter_queue_params_t   q[SODAPP_NUM_GRAPH_PARAMS];
    uint32_t                            i;
    uint32_t                            cnt = 0;
    vx_status                           vxStatus;

    appCntxt->numGraphParams = 8;

    vx_node leftLdcNode  = SDELCDAPPLIB_getLeftLDCNode(appCntxt->sdeLdcHdl);
    vx_node rightLdcNode = SDELCDAPPLIB_getRightLDCNode(appCntxt->sdeLdcHdl);
    vx_node sdeNode      = SL_SDEAPPLIB_getSDENode(appCntxt->slSdeHdl);
    vx_node geNode       = SODAPPLIB_getGENode(appCntxt->sodHdl);
    vx_node sodNode      = SODAPPLIB_getSODNode(appCntxt->sodHdl);

    /* LDC left node Param 6 ==> graph param 0. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                leftLdcNode,
                                6);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxInputLeftImage;

    /* LDC rigth node Param 6 ==> graph param 1. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                rightLdcNode,
                                6);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxInputRightImage;

    /* LDC right node Param 7 ==> graph param 3. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                rightLdcNode,
                                7);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxRightRectImage;


    /* vxDmpacSdeNode Param 3 ==> graph param 4 */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                sdeNode,
                                3);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxSde16BitOutput;

    /* vxSodNode Param 4 ==> graph param 5. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                sodNode,
                                4);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxObstaclesPose;

    /* vxSodNode Param 5 ==> graph param 6. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                sodNode,
                                5);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxNumObstacles;

    /* vxSodNode Param 6 ==> graph param 7. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                sodNode,
                                6);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxFreeSpaceBoundary;

    /* vxSodNode Param 7 ==> graph param 8. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                sodNode,
                                7);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxDrivableSpace;

    for (i = 0; i < cnt; i++)
    {
        q[i].graph_parameter_index = i;
        q[i].refs_list_size        = appCntxt->pipelineDepth;
    }

    // allocate free Q
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        paramDesc                      = &appCntxt->paramDesc[i];
        paramDesc->vxInputLeftImage    = appCntxt->vxInputLeftImage[i];
        paramDesc->vxInputRightImage   = appCntxt->vxInputRightImage[i];
        paramDesc->vxRightRectImage    = appCntxt->vxRightRectImage[i];
        paramDesc->vxSde16BitOutput    = appCntxt->vxSde16BitOutput[i];
        paramDesc->vxObstaclesPose     = appCntxt->vxObstaclesPose[i];
        paramDesc->vxNumObstacles      = appCntxt->vxNumObstacles[i];
        paramDesc->vxFreeSpaceBoundary = appCntxt->vxFreeSpaceBoundary[i];
        paramDesc->vxDrivableSpace     = appCntxt->vxDrivableSpace[i];

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
        /*  vxLeftRectImage */
        vxStatus = tivxSetNodeParameterNumBufByIndex(leftLdcNode,
                                                     7,
                                                     appCntxt->pipelineDepth);

        /* vxPostDisparity */
        vxStatus = tivxSetNodeParameterNumBufByIndex(geNode,
                                                     3,
                                                     appCntxt->pipelineDepth);
        PTK_assert(vxStatus == VX_SUCCESS);

        /* vxGroundModel */
        vxStatus = tivxSetNodeParameterNumBufByIndex(geNode,
                                                     4,
                                                     appCntxt->pipelineDepth);
        PTK_assert(vxStatus == VX_SUCCESS);             
    }

    return vxStatus;
}


vx_status SODAPP_setupPipeline_ML(SODAPP_Context * appCntxt)
{
    SODAPP_graphParams                * paramDesc;
    vx_graph_parameter_queue_params_t   q[SODAPP_NUM_GRAPH_PARAMS];
    uint32_t                            i;
    uint32_t                            cnt = 0;
    vx_status                           vxStatus;

    appCntxt->numGraphParams = 8;

    vx_node leftLdcNode    = SDELCDAPPLIB_getLeftLDCNode(appCntxt->sdeLdcHdl);
    vx_node rightLdcNode   = SDELCDAPPLIB_getRightLDCNode(appCntxt->sdeLdcHdl);

    vx_node sdeNodeL0      = ML_SDEAPPLIB_getSDENodeL0(appCntxt->mlSdeHdl);
    vx_node sdeNodeL1      = ML_SDEAPPLIB_getSDENodeL1(appCntxt->mlSdeHdl);
    vx_node sdeNodeL2      = ML_SDEAPPLIB_getSDENodeL2(appCntxt->mlSdeHdl);
#if ENABLE_PP_MEDIAN_FILTER
    vx_node medFilterNode  = ML_SDEAPPLIB_getMedFilterNode(appCntxt->mlSdeHdl);
#endif
    vx_node mergeNodeL1    = ML_SDEAPPLIB_getMergeNodeL1(appCntxt->mlSdeHdl);
    vx_node mergeNodeL2    = ML_SDEAPPLIB_getMergeNodeL2(appCntxt->mlSdeHdl);
    vx_node leftMscNodeL1  = ML_SDEAPPLIB_getLeftMSCNodeL1(appCntxt->mlSdeHdl);
    vx_node rightMscNodeL1 = ML_SDEAPPLIB_getRightMSCNodeL1(appCntxt->mlSdeHdl);
    vx_node leftMscNodeL2  = ML_SDEAPPLIB_getLeftMSCNodeL2(appCntxt->mlSdeHdl);
    vx_node rightMscNodeL2 = ML_SDEAPPLIB_getRightMSCNodeL2(appCntxt->mlSdeHdl);

    vx_node geNode       = SODAPPLIB_getGENode(appCntxt->sodHdl);
    vx_node sodNode      = SODAPPLIB_getSODNode(appCntxt->sodHdl);


    /* LDC left node Param 6 ==> graph param 0. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                leftLdcNode,
                                6);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxInputLeftImage;

    /* LDC rigth node Param 6 ==> graph param 1. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                rightLdcNode,
                                6);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxInputRightImage;

    /* LDC right node Param 7 ==> graph param 3. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                rightLdcNode,
                                7);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxRightRectImage;

    /* vxDisparityMergeNodeL1 Param 3 => graph param 4. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                mergeNodeL1,
                                3);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxMergeDisparityL0;

#if ENABLE_PP_MEDIAN_FILTER
    if (appCntxt->ppMedianFilterEnable)
    {
        appCntxt->numGraphParams += 1;

        /* vxMedianFilterNode Param 2 => graph param 5. */
        ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                    medFilterNode,
                                    2);
        q[cnt++].refs_list = (vx_reference*)appCntxt->vxMedianFilteredDisparity;
    }
#endif

    /* vxSodNode Param 4 ==> graph param 5/6. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                sodNode,
                                4);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxObstaclesPose;

    /* vxSodNode Param 5 ==> graph param 6/7. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                sodNode,
                                5);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxNumObstacles;

    /* vxSodNode Param 6 ==> graph param 7/8. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                sodNode,
                                6);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxFreeSpaceBoundary;

    /* vxSodNode Param 7 ==> graph param 8/9. */
    ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                sodNode,
                                7);
    q[cnt++].refs_list = (vx_reference*)appCntxt->vxDrivableSpace;


    for (i = 0; i < cnt; i++)
    {
        q[i].graph_parameter_index = i;
        q[i].refs_list_size        = appCntxt->pipelineDepth;
    }

    // allocate free Q
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        paramDesc                     = &appCntxt->paramDesc[i];
        paramDesc->vxInputLeftImage   = appCntxt->vxInputLeftImage[i];
        paramDesc->vxInputRightImage  = appCntxt->vxInputRightImage[i];
        paramDesc->vxRightRectImage   = appCntxt->vxRightRectImage[i];
        paramDesc->vxMergeDisparityL0 = appCntxt->vxMergeDisparityL0[i];

#if ENABLE_PP_MEDIAN_FILTER
        if (appCntxt->ppMedianFilterEnable)
        {
            paramDesc->vxMedianFilteredDisparity = appCntxt->vxMedianFilteredDisparity[i];
        }
#endif

        paramDesc->vxObstaclesPose     = appCntxt->vxObstaclesPose[i];
        paramDesc->vxNumObstacles      = appCntxt->vxNumObstacles[i];
        paramDesc->vxFreeSpaceBoundary = appCntxt->vxFreeSpaceBoundary[i];
        paramDesc->vxDrivableSpace     = appCntxt->vxDrivableSpace[i];

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
        /*  vxLeftRectImage */
        vxStatus = tivxSetNodeParameterNumBufByIndex(leftLdcNode,
                                                     7,
                                                     appCntxt->pipelineDepth);
        PTK_assert(vxStatus == VX_SUCCESS);

        /* vxSde16BitOutputL0 */
        vxStatus = tivxSetNodeParameterNumBufByIndex(sdeNodeL0,
                                                     3,
                                                     appCntxt->pipelineDepth);
        PTK_assert(vxStatus == VX_SUCCESS);

        if (appCntxt->numLayers > 1)
        {
            /* vxLeftImageL1 */
            vxStatus = tivxSetNodeParameterNumBufByIndex(leftMscNodeL1,
                                                         1,
                                                         appCntxt->pipelineDepth);
            PTK_assert(vxStatus == VX_SUCCESS);

            /* vxRightImageL1 */
            vxStatus = tivxSetNodeParameterNumBufByIndex(rightMscNodeL1,
                                                         1,
                                                         appCntxt->pipelineDepth);
            PTK_assert(vxStatus == VX_SUCCESS);

            /* vxSde16BitOutputL1 */
            vxStatus = tivxSetNodeParameterNumBufByIndex(sdeNodeL1,
                                                         3,
                                                         appCntxt->pipelineDepth);
            PTK_assert(vxStatus == VX_SUCCESS);
        }

        if (appCntxt->numLayers > 2)
        {
            /* vxLeftImageL2 */
            vxStatus = tivxSetNodeParameterNumBufByIndex(leftMscNodeL2,
                                                         1,
                                                         appCntxt->pipelineDepth);
            PTK_assert(vxStatus == VX_SUCCESS);

            /* vxRightImageL2 */
            vxStatus = tivxSetNodeParameterNumBufByIndex(rightMscNodeL2,
                                                         1,
                                                         appCntxt->pipelineDepth);
            PTK_assert(vxStatus == VX_SUCCESS);

            /* vxSde16BitOutputL2 */
            vxStatus = tivxSetNodeParameterNumBufByIndex(sdeNodeL2,
                                                         3,
                                                         appCntxt->pipelineDepth);
            PTK_assert(vxStatus == VX_SUCCESS);

            /* vxMergeDisparityL1 */
            vxStatus = tivxSetNodeParameterNumBufByIndex(mergeNodeL2,
                                                         3,
                                                         appCntxt->pipelineDepth);
            PTK_assert(vxStatus == VX_SUCCESS);
        }

        /* vxPostDisparity */
        vxStatus = tivxSetNodeParameterNumBufByIndex(geNode,
                                                     3,
                                                     appCntxt->pipelineDepth);
        PTK_assert(vxStatus == VX_SUCCESS);

        /* vxGroundModel */
        vxStatus = tivxSetNodeParameterNumBufByIndex(geNode,
                                                     4,
                                                     appCntxt->pipelineDepth);
        PTK_assert(vxStatus == VX_SUCCESS);
    }

    return vxStatus;
}

void SODAPP_printStats(SODAPP_Context * appCntxt)
{
    tivx_utils_graph_perf_print(appCntxt->vxGraph);
    appPerfPointPrint(&appCntxt->sodPerf);
    PTK_printf("\n");
    appPerfPointPrintFPS(&appCntxt->sodPerf);
    PTK_printf("\n");
}

void SODAPP_exportStats(SODAPP_Context * appCntxt)
{
    FILE *fp;
    app_perf_point_t *perf_arr[1];

    perf_arr[0] = &appCntxt->sodPerf;
    fp = appPerfStatsExportOpenFile(".", "SODAPP_datasheet");
    if (NULL != fp)
    {
        appPerfStatsExportAll(fp, perf_arr, 1);
        tivx_utils_graph_perf_export(fp, appCntxt->vxGraph);
        appPerfStatsExportCloseFile(fp);
        appPerfStatsResetAll();
    }
    else
    {
        printf("fp is null\n");
    }
}

void SODAPP_waitGraph(SODAPP_Context * appCntxt)
{
    vxWaitGraph(appCntxt->vxGraph);

    /* Wait for the output queue to get flushed. */
    while (appCntxt->freeQ.size() != appCntxt->pipelineDepth)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

vx_status SODAPP_getFreeParamRsrc(SODAPP_Context       *appCntxt,
                                  SODAPP_graphParams   **gpDesc)
{
    std::unique_lock<std::mutex>   lock(appCntxt->paramRsrcMutex);
    vx_status                      vxStatus = VX_SUCCESS;

    /* Check if we have free og node descriptors available. */
    if (appCntxt->freeQ.empty())
    {
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        *gpDesc = appCntxt->freeQ.front();
        appCntxt->freeQ.pop();
    }

    return 0;
}

vx_status SODAPP_process(SODAPP_Context * appCntxt,  SODAPP_graphParams * gpDesc)
{
    vx_status vxStatus = VX_SUCCESS;
    uint8_t   cnt = 0;

    vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxGraph,
                                               cnt++,
                                               (vx_reference*)&gpDesc->vxInputLeftImage,
                                               1);
    if(VX_SUCCESS != vxStatus)
    {
        return vxStatus;
    }

    vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxGraph,
                                               cnt++,
                                               (vx_reference*)&gpDesc->vxInputRightImage,
                                               1);
    if(VX_SUCCESS != vxStatus)
    {
        return vxStatus;
    }

    vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxGraph,
                                               cnt++,
                                               (vx_reference*)&gpDesc->vxRightRectImage,
                                               1);
    if(VX_SUCCESS != vxStatus)
    {
        return vxStatus;
    }

    if (appCntxt->sdeAlgoType == 0)
    {
        vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxGraph,
                                                   cnt++,
                                                   (vx_reference*)&gpDesc->vxSde16BitOutput,
                                                   1);
        if(VX_SUCCESS != vxStatus)
        {
            return vxStatus;
        }
    } else
    {
        vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxGraph,
                                                   cnt++,
                                                   (vx_reference*)&gpDesc->vxMergeDisparityL0,
                                                   1);
        if(VX_SUCCESS != vxStatus)
        {
            return vxStatus;
        }

#if ENABLE_PP_MEDIAN_FILTER
        if (appCntxt->ppMedianFilterEnable)
        {
            vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxGraph,
                                                       cnt++,
                                                       (vx_reference*)&gpDesc->vxMedianFilteredDisparity,
                                                       1);
            if(VX_SUCCESS != vxStatus)
            {
                return vxStatus;
            }
        }
#endif
    }

    vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxGraph,
                                               cnt++,
                                               (vx_reference*)&gpDesc->vxObstaclesPose,
                                               1);
    if(VX_SUCCESS != vxStatus)
    {
        return vxStatus;
    }

    vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxGraph,
                                               cnt++,
                                               (vx_reference*)&gpDesc->vxNumObstacles,
                                               1);
    if(VX_SUCCESS != vxStatus)
    {
        return vxStatus;
    }

    vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxGraph,
                                               cnt++,
                                               (vx_reference*)&gpDesc->vxFreeSpaceBoundary,
                                               1);
    if(VX_SUCCESS != vxStatus)
    {
        return vxStatus;
    }

    vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxGraph,
                                               cnt++,
                                              (vx_reference*)&gpDesc->vxDrivableSpace,
                                               1);
    if(VX_SUCCESS != vxStatus)
    {
        return vxStatus;
    }

    return vxStatus;
}


int32_t   SODAPP_processEvent(SODAPP_Context * appCntxt, vx_event_t * event)
{
    vx_reference            ref;
    uint32_t                numRefs;
    uint32_t                index;
    int32_t                 status;
    vx_status               vxStatus;

    ref      = NULL;
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
            appPerfPointEnd(&appCntxt->sodPerf);
        }

        appPerfPointBegin(&appCntxt->sodPerf);

        /* Node execution is complete. Deque all the parameters
         * for this node.
         */
        for (i = 0; i < appCntxt->numGraphParams; i++)
        {
            vxStatus = vxGraphParameterDequeueDoneRef(appCntxt->vxGraph,
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
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            if (ref == (vx_reference)appCntxt->vxDrivableSpace[i])
            {
                index = i;
                break;
            }
        }

        if (index >= appCntxt->pipelineDepth)
        {
            PTK_printf("[%s:%d] Resource look up failed\n",
                       __FUNCTION__, __LINE__);

            return -1;
        } else 
        {
            /* Mark the dequeued resource as free. */
            status = SODAPP_releaseParamRsrc(appCntxt, index);

            if (status < 0)
            {
                PTK_printf("[%s:%d] SODAPP_releaseParamRsrc() failed.\n",
                           __FUNCTION__,
                           __LINE__);

                return -1;
            }
        }
    }

    return 0;
}


int32_t SODAPP_releaseParamRsrc(SODAPP_Context  *appCntxt,
                                uint32_t         rsrcIndex)
{
    SODAPP_graphParams        * desc;
    std::unique_lock<std::mutex>   lock(appCntxt->paramRsrcMutex);

    desc = &appCntxt->paramDesc[rsrcIndex];
    appCntxt->outputQ.push(desc);

    return 0;
}


int32_t SODAPP_getOutBuff(SODAPP_Context *appCntxt, uint32_t * numObstacles, vx_image *rightRectImage, vx_image* disparity16,
                          tivx_obstacle_pos_t * obsBox, int32_t * fsBoundary, tivx_drivable_space_t * drivableSpace)
{
    vx_status                    vxStatus;
    vx_map_id                    map_id;
    vx_size                      stride;

    SODAPP_graphParams         * desc;
    std::unique_lock<std::mutex> lock(appCntxt->paramRsrcMutex);

    tivx_obstacle_pos_t        * curObsBox;

    int32_t                    * curFsBoundary;
    tivx_drivable_space_t      * curDrivableSpace;
    int32_t                      dsWidth = (appCntxt->sodCreateParams.sodCfg.odConfig.width / 
                                            appCntxt->sodCreateParams.sodCfg.copppParams.dsFactor);

    if (appCntxt->outputQ.empty())
    {
        return -1;
    }

    /* Get the descriptor. */
    desc              = appCntxt->outputQ.front();
    *rightRectImage = desc->vxRightRectImage;

    if (disparity16 != NULL)
    {
        // get SDE disparity output
        if (appCntxt->sdeAlgoType == 0)
        {
        
            *disparity16 = desc->vxSde16BitOutput;
        } 
#if ENABLE_PP_MEDIAN_FILTER
        else if (appCntxt->ppMedianFilterEnable)
        {
            *disparity16 = desc->vxMedianFilteredDisparity;
        }
#endif
        else
        {
            *disparity16 = desc->vxMergeDisparityL0;
        }
    }

    // for obstacle detection output
    // use gpDesc
    vxCopyScalar(desc->vxNumObstacles, numObstacles, VX_READ_ONLY, VX_MEMORY_TYPE_HOST);

    if (*numObstacles != 0)
    {
        vxStatus = vxMapArrayRange(desc->vxObstaclesPose,
                                   0,
                                   *numObstacles,
                                   &map_id,
                                   &stride,
                                   (void **)&curObsBox,
                                   VX_READ_ONLY,
                                   VX_MEMORY_TYPE_HOST,
                                   0);
        PTK_assert(VX_SUCCESS == vxStatus);
        PTK_assert(NULL != curObsBox);

        memcpy(obsBox, curObsBox, (*numObstacles) *sizeof(tivx_obstacle_pos_t));

        vxStatus = vxUnmapArrayRange(desc->vxObstaclesPose, map_id);
        PTK_assert(VX_SUCCESS == vxStatus);
    }

    // for freespace detection output
    vxStatus = vxMapArrayRange(desc->vxFreeSpaceBoundary,
                               0,
                               dsWidth,
                               &map_id,
                               &stride,
                               (void **)&curFsBoundary,
                               VX_READ_ONLY,
                               VX_MEMORY_TYPE_HOST,
                               0);
    PTK_assert(VX_SUCCESS == vxStatus);
    PTK_assert(NULL != curFsBoundary);

    memcpy(fsBoundary, curFsBoundary, dsWidth * sizeof(int32_t));

    vxStatus = vxUnmapArrayRange(desc->vxFreeSpaceBoundary, map_id);
    PTK_assert(VX_SUCCESS == vxStatus);

    vxStatus = vxMapUserDataObject(desc->vxDrivableSpace,
                                   0,
                                   sizeof(tivx_drivable_space_t),
                                   &map_id,
                                   (void **)&curDrivableSpace,
                                   VX_READ_ONLY,
                                   VX_MEMORY_TYPE_HOST,
                                   0);

    memcpy(drivableSpace, curDrivableSpace, sizeof(tivx_drivable_space_t));

    vxStatus = vxUnmapUserDataObject(desc->vxDrivableSpace, map_id);
    PTK_assert(VX_SUCCESS == vxStatus);

    return 0;
}


void SODAPP_releaseOutBuff(SODAPP_Context * appCntxt)
{
    SODAPP_graphParams    *desc;
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
