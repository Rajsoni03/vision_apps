/*
 *
 * Copyright (c) 2020 Texas Instruments Incorporated
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
#include "sde_singlelayer_applib.h"

typedef struct SL_SDEAPPLIB_Context
{
    /** OVX Node References */
    vx_context                        vxContext;

    /** OpenVX graph */
    vx_graph                          vxGraph;

    /** SDE Node */
    vx_node                           vxDmpacSdeNode;

    /** SDE visualization node */
    vx_node                           vxNodeDisparityVis;

    /** Handle to the data object holding the SDE configuration 
     *  parameters. */
    vx_user_data_object               vxSdeConfig;

    /** Handle to the data object holding the disparity Visualization 
     *  configuration parameters. 
     */
    vx_user_data_object               vxSdeVisConfig;

    /** Left rectified image object */
    vx_image                          vxLeftRectImage[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Right rectified image object */
    vx_image                          vxRightRectImage[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Raw disparity map object */
    vx_image                          vxSde16BitOutput[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Handle to the data object holding the disparity histogram */
    vx_distribution                   vxHistogram;

    /** input image width */
    uint16_t                          width;
    
    /** input image height */
    uint16_t                          height;

    /** input format, U8 or YUV_UYVY */
    uint8_t                           inputFormat;

    /** SDE config params */
    tivx_dmpac_sde_params_t           sdeCfg;

    /** Disparity visualization params */
    tivx_sde_disparity_vis_params_t   sdeVisCfg;

    /** pipeline depth */
    uint8_t                           pipelineDepth;

    /** Input pipeline depth */
    uint8_t                           inputPipelineDepth;

    /** flag indicating whether or not create input OpenVX object in applib */
    uint8_t                           createInputFlag;

    /** flag indicating whether or not create output OpenVX object in applib */
    uint8_t                           createOutputFlag;

} SL_SDEAPPLIB_Context;

static void      SL_SDEAPPLIB_setParams(SL_SDEAPPLIB_Handle        handle,
                                        SL_SDEAPPLIB_createParams *createParams);

static vx_status SL_SDEAPPLIB_createGraph(SL_SDEAPPLIB_Handle handle);

static void      SL_SDEAPPLIB_releaseGraph(SL_SDEAPPLIB_Handle handle);


SL_SDEAPPLIB_Handle SL_SDEAPPLIB_create(SL_SDEAPPLIB_createParams *createParams)
{
    SL_SDEAPPLIB_Handle      handle;
    vx_status                vxStatus;

    handle = new SL_SDEAPPLIB_Context();

    /* Set applib-level create parameters */
    SL_SDEAPPLIB_setParams(handle, createParams);

    /* Create nodes and graph */
    vxStatus = SL_SDEAPPLIB_createGraph(handle);

    if (vxStatus != VX_SUCCESS)
    {
        delete handle;
        handle = NULL;
    }

    return handle;

}


void SL_SDEAPPLIB_delete(SL_SDEAPPLIB_Handle *handle)
{
    if (*handle)
    {
        SL_SDEAPPLIB_releaseGraph(*handle);

        delete *handle;
        *handle = NULL;
    }

    return;

} /* SL_SDEAPPLIB_delete */


void SL_SDEAPPLIB_setParams(SL_SDEAPPLIB_Handle        handle,
                            SL_SDEAPPLIB_createParams *createParams)
{
    int32_t                  i;
    SL_SDEAPPLIB_Context   * appCntxt;
    appCntxt = (SL_SDEAPPLIB_Context *)handle;

    appCntxt->vxContext          = createParams->vxContext;
    appCntxt->vxGraph            = createParams->vxGraph;
    appCntxt->pipelineDepth      = createParams->pipelineDepth;
    appCntxt->inputPipelineDepth = createParams->inputPipelineDepth;
    appCntxt->createInputFlag    = createParams->createInputFlag;
    appCntxt->createOutputFlag   = createParams->createOutputFlag;

    appCntxt->sdeCfg             = createParams->sdeCfg;
    appCntxt->width              = createParams->width;
    appCntxt->height             = createParams->height;
    appCntxt->inputFormat        = createParams->inputFormat;

    /*
     * set up input objects
     */
    if (appCntxt->createInputFlag == 1)
    {
        // input left image
        if (appCntxt->inputFormat == PTK_IMG_FORMAT_Y)
        {
            appCntxt->vxLeftRectImage[0] = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_U8);
        } else 
        {
            appCntxt->vxLeftRectImage[0] = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_NV12);
        }
        if (appCntxt->vxLeftRectImage[0] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);
        }

        // input right image
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
    } else 
    {
        for(i = 0; i < appCntxt->inputPipelineDepth; i++)
        {
            appCntxt->vxLeftRectImage[i] = createParams->vxLeftRectImage[i];
        }
        for(i = 0; i < appCntxt->pipelineDepth; i++)
        {
            appCntxt->vxRightRectImage[i] = createParams->vxRightRectImage[i];
        }
    }

    /*
     * set up output objects
     */
    if (appCntxt->createOutputFlag == 1)
    {
        appCntxt->vxSde16BitOutput[0]  = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_S16);        
        if (appCntxt->vxSde16BitOutput[0] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);
        }
    } else 
    {
        for(i = 0; i < appCntxt->pipelineDepth; i++)
        {
            appCntxt->vxSde16BitOutput[i]  = createParams->vxSde16BitOutput[i];
        }
    }
}


vx_status SL_SDEAPPLIB_createGraph(SL_SDEAPPLIB_Handle handle)
{
    SL_SDEAPPLIB_Context       * appCntxt;
    vx_status                    status = VX_SUCCESS;

    appCntxt = (SL_SDEAPPLIB_Context *)handle;

    /* Mapping Graph */
    PTK_assert(appCntxt->vxGraph);

    /* DMPAC SDE node */
    appCntxt->vxSdeConfig = vxCreateUserDataObject(appCntxt->vxContext, "tivx_dmpac_sde_params_t",
                                                   sizeof(tivx_dmpac_sde_params_t), &appCntxt->sdeCfg);
    PTK_assert(appCntxt->vxSdeConfig);
    vxSetReferenceName((vx_reference)appCntxt->vxSdeConfig, "Stereo_Config");

    /* histogram output from SDE */
    appCntxt->vxHistogram = vxCreateDistribution(appCntxt->vxContext, 128, 0, 4096);
    PTK_assert(appCntxt->vxHistogram);
    vxSetReferenceName((vx_reference)appCntxt->vxHistogram, "Stereo_OutputHistogramDistribution_L0");

    /* crearte DMPAC SDE node */
    appCntxt->vxDmpacSdeNode = tivxDmpacSdeNode(appCntxt->vxGraph,
                                                appCntxt->vxSdeConfig,
                                                appCntxt->vxLeftRectImage[0],
                                                appCntxt->vxRightRectImage[0],
                                                appCntxt->vxSde16BitOutput[0],
                                                appCntxt->vxHistogram);


    PTK_assert(appCntxt->vxDmpacSdeNode);
    vxSetReferenceName((vx_reference)appCntxt->vxDmpacSdeNode, "DMPAC_SDE_Processing");

    status = vxSetNodeTarget(appCntxt->vxDmpacSdeNode, VX_TARGET_STRING, TIVX_TARGET_DMPAC_SDE);
    PTK_assert(status==VX_SUCCESS);

    return status;

} /* SL_SDEAPPLIB_createGraph */


void SL_SDEAPPLIB_releaseGraph(SL_SDEAPPLIB_Handle handle)
{
    SL_SDEAPPLIB_Context * appCntxt;

    appCntxt = (SL_SDEAPPLIB_Context *)handle;

    /* Layer 0 */
    /* release DMPAC SDE node */
    vxReleaseNode(&appCntxt->vxDmpacSdeNode);
    vxReleaseUserDataObject(&appCntxt->vxSdeConfig);

    if (appCntxt->createInputFlag == 1)
    {
        vxReleaseImage(&appCntxt->vxLeftRectImage[0]);
        vxReleaseImage(&appCntxt->vxRightRectImage[0]);
    } 

    if (appCntxt->createOutputFlag == 1)
    {
        vxReleaseImage(&appCntxt->vxSde16BitOutput[0]);
    }

    vxReleaseDistribution(&appCntxt->vxHistogram);

    return;
} /* SL_SDEAPPLIB_releaseGraph */


vx_node  SL_SDEAPPLIB_getSDENode(SL_SDEAPPLIB_Handle handle)
{
    SL_SDEAPPLIB_Context *appCntxt = (SL_SDEAPPLIB_Context *)handle;
    return appCntxt->vxDmpacSdeNode;
}

