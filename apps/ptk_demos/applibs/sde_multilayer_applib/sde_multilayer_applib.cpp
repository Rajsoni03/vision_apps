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
#include "sde_multilayer_applib.h"

typedef struct ML_SDEAPPLIB_Context
{
    /** OVX Node References */
    vx_context                        vxContext;

    /** OpenVX graph */
    vx_graph                          vxGraph;

    /** Scaler Node to create down-sampled layer-1 left image 
     *  from layer-0 left image
     */
    vx_node                           vxMscNodeL_L1;

    /** Scaler Node to create down-sampled layer-1 right image 
     *  from layer-0 right image
     */
    vx_node                           vxMscNodeR_L1;

    /** Scaler Node to create down-sampled layer-2 left image 
     *  from layer-1 left image
     */
    vx_node                           vxMscNodeL_L2; 

    /** Scaler Node to create down-sampled layer-2 right image 
     *  from layer-1 right image
     */
    vx_node                           vxMscNodeR_L2; 

    /** Layer-0 SDE Node */
    vx_node                           vxDmpacSdeNodeL0;

    /** Layer-1 SDE Node */
    vx_node                           vxDmpacSdeNodeL1;

    /** Layer-2 SDE Node */
    vx_node                           vxDmpacSdeNodeL2;

    /** Disparity Merge Node at layer 1 that combines disparity 
     * maps at layer 0 and layer 1 
     */
    vx_node                           vxDisparityMergeNodeL1;

    /** Disparity Merge Node at layer 2 that combines disparity 
     * maps at layer 1 and layer 2
     */
    vx_node                           vxDisparityMergeNodeL2;

    /** Post-processing hole Filling Node */
    vx_node                           vxHoleFillingNode;
        
    /** Post-processing Median Filter Node */
    vx_node                           vxMedianFilterNode;

    /** Handle to the data object holding the MSC Scaler coefficent */
    vx_user_data_object               vxMscCoeff;

    /** Handle to the data object holding the SDE configuration parameters */
    vx_user_data_object               vxSdeConfig;

    /** Handle to the data object holding the configuration parameters 
     *  for Layer-1 merge node 
     */
    vx_user_data_object               vxDisparityMergeConfigL1;

    /** Handle to the data object holding the configuration parameters 
     *  for Layer-2 merge node 
     */
    vx_user_data_object               vxDisparityMergeConfigL2;

    /** Handle to the data object holding the configuration parameters 
     *  for hole filling node
     */
    vx_user_data_object               vxHoleFillingConfig;

    /** Handle to the data object holding the configuration parameters 
     *  for median filter node
     */
    vx_user_data_object               vxMedianFilterConfig;

    /** Input (base layer) left image object */
    vx_image                          vxLeftImageL0[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Input (base layer) right image object */
    vx_image                          vxRightImageL0[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Base-layer (Layer 0) rectified left image object */
    vx_image                          vxLeftRectImageL0[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Base-layer (Layer 0) rectified right image object */
    vx_image                          vxRightRectImageL0[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Disparity object at Layer 0 */
    vx_image                          vxDisparityL0;

    /** Raw disparity map object at Layer 0 */
    vx_image                          vxSde16BitOutputL0;

    /** Merged disparity map object at Layer 0 */
    vx_image                          vxMergeDisparityL0[PTK_GRAPH_MAX_PIPELINE_DEPTH];
    vx_distribution                   vxHistogramL0;
    
    /** Layer-1 left image object by down-sampling layer-0 
     *  rectified left image */
    vx_image                          vxLeftImageL1;

    /** Layer-1 right image object by down-sampling layer-0 
     *  rectified right image */
    vx_image                          vxRightImageL1;

    /** Disparity object at Layer 1 */
    vx_image                          vxDisparityL1;

    /** Raw disparity map object at Layer 1 */
    vx_image                          vxSde16BitOutputL1;

    /** Merged disparity map object at Layer 1 */
    vx_image                          vxMergeDisparityL1;

    /** Disparity histogram map at Layer 1 */
    vx_distribution                   vxHistogramL1;

    /** Layer-2 left image object by down-sampling layer-1 
     *  rectified left image */
    vx_image                          vxLeftImageL2;

    /** Layer-2 right image object by down-sampling layer-1 
     *  rectified right image */
    vx_image                          vxRightImageL2;

    /** Disparity object at Layer 2 */
    vx_image                          vxDisparityL2;

    /** Raw disparity map object at Layer 2 */
    vx_image                          vxSde16BitOutputL2;

    /** Disparity histogram object at Layer 2 */
    vx_distribution                   vxHistogramL2;

    /** Disparity map object after median filter */
    vx_image                          vxMedianFilteredDisparity[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Input image width */
    uint16_t                          width;

    /** Input image height */
    uint16_t                          height;

    /** Number of layers */
    uint8_t                           numLayers;

    /** Input image format, U0 or YUV_UYVY */
    uint8_t                           inputFormat;

    /** Flag indicating whether median filter is enabled or not */
    uint8_t                           enableMedianFilter;

    /** SDE configuration parameters */
    tivx_dmpac_sde_params_t           sdeCfg;

    /** SDE visualization node configuration parameters */
    tivx_sde_disparity_vis_params_t   sdeVisCfg;

    /** Layer-1 disparity merge node configuration parameters */
    tivx_disparity_merge_params_t     dmCfgL1;

    /** Layer-2 disparity merge node configuration parameters */
    tivx_disparity_merge_params_t     dmCfgL2;

    /** Hole filling node configuration parameters */
    tivx_hole_filling_params_t        hfCfg;

    /** Median filter node configuration parameters */
    tivx_median_filter_params_t       mfCfg;

    /** Disparity Merge Node core mapping */
    const char                      * dispMergeNodeCore;

    /** Hole Filling Node core mapping */
    const char                      * holeFillingNodeCore;

    /** Median Filter Node core mapping */
    const char                      * medianFilterNodeCore;

    /** pipeline depth */
    uint8_t                           pipelineDepth;

    /** Input pipeline depth */
    uint8_t                           inputPipelineDepth;

    /** flag indicating whether or not create input OpenVX object in applib */
    uint8_t                           createInputFlag;

    /** flag indicating whether or not create output OpenVX object in applib */
    uint8_t                           createOutputFlag;

} ML_SDEAPPLIB_Context;

static void      ML_SDEAPPLIB_setParams(ML_SDEAPPLIB_Handle        handle,
                                        ML_SDEAPPLIB_createParams *createParams);

static vx_status ML_SDEAPPLIB_createGraph(ML_SDEAPPLIB_Handle handle);

static void      ML_SDEAPPLIB_releaseGraph(ML_SDEAPPLIB_Handle handle);


ML_SDEAPPLIB_Handle ML_SDEAPPLIB_create(ML_SDEAPPLIB_createParams *createParams)
{
    ML_SDEAPPLIB_Handle      handle;
    vx_status                vxStatus;

    handle = new ML_SDEAPPLIB_Context();

    /* Set applib-level create parameters */
    ML_SDEAPPLIB_setParams(handle, createParams);

    /* Create nodes and graph */
    vxStatus = ML_SDEAPPLIB_createGraph(handle);

    if (vxStatus != VX_SUCCESS)
    {
        delete handle;
        handle = NULL;
    }

    return handle;

}


void ML_SDEAPPLIB_delete(ML_SDEAPPLIB_Handle *handle)
{
    if (*handle)
    {
        ML_SDEAPPLIB_releaseGraph(*handle);

        delete *handle;
        *handle = NULL;
    }

    return;
} /* ML_SDEAPPLIB_delete */


void ML_SDEAPPLIB_setParams(ML_SDEAPPLIB_Handle        handle,
                            ML_SDEAPPLIB_createParams *createParams)
{
    int32_t                  i;
    ML_SDEAPPLIB_Context   * appCntxt;
    appCntxt = (ML_SDEAPPLIB_Context *)handle;

    appCntxt->vxContext          = createParams->vxContext;
    appCntxt->vxGraph            = createParams->vxGraph;
    appCntxt->pipelineDepth      = createParams->pipelineDepth;
    appCntxt->inputPipelineDepth = createParams->inputPipelineDepth;
    appCntxt->createInputFlag    = createParams->createInputFlag;
    appCntxt->createOutputFlag   = createParams->createOutputFlag;

    appCntxt->sdeCfg             = createParams->sdeCfg;
    appCntxt->numLayers          = createParams->numLayers;
    appCntxt->enableMedianFilter = createParams->enableMedianFilter;

    appCntxt->width              = createParams->width;
    appCntxt->height             = createParams->height;
    appCntxt->inputFormat        = createParams->inputFormat;

    /* set disparity merge config */
    if (appCntxt->numLayers > 1)
    {
       appCntxt->dmCfgL1.hiWidth        = appCntxt->width;
       appCntxt->dmCfgL1.hiHeight       = appCntxt->height;
       appCntxt->dmCfgL1.loWidth        = appCntxt->width >> 1;
       appCntxt->dmCfgL1.loHeight       = appCntxt->height >> 1;
       // low-resolution input has only disparity info without confidence if there is another layer below
       appCntxt->dmCfgL1.loDispOnlyFlag = (appCntxt->numLayers > 2) ? 1 : 0;
       appCntxt->dmCfgL1.diffDispTh     = 16 * 2;  // make it configurable
    }

    if (appCntxt->numLayers > 2)
    {
       appCntxt->dmCfgL2.hiWidth        = appCntxt->width >> 1;
       appCntxt->dmCfgL2.hiHeight       = appCntxt->height >> 1;
       appCntxt->dmCfgL2.loWidth        = appCntxt->width >> 2;
       appCntxt->dmCfgL2.loHeight       = appCntxt->height >> 2;
       // low-resolution input has only disparity info without confidence
       appCntxt->dmCfgL2.loDispOnlyFlag = 0;    
       appCntxt->dmCfgL2.diffDispTh     = 16 * 2;  // make it configurable
    }
    
    /* set hole filling config */
    appCntxt->hfCfg.width              = appCntxt->width;
    appCntxt->hfCfg.height             = appCntxt->height;
    appCntxt->hfCfg.minDisparity       = createParams->minDisparity;
    appCntxt->hfCfg.maxDisparity       = createParams->maxDisparity;
    appCntxt->hfCfg.deltaDispTh        = 16 * 16;  // make it configurable
    appCntxt->hfCfg.gapLengthTh        = appCntxt->width / 40; // make it configurable

    /* set median filter config */
    appCntxt->mfCfg.width              = appCntxt->width;
    appCntxt->mfCfg.height             = appCntxt->height;
    appCntxt->mfCfg.filterSize         = 3; // make it configurable up to 5x5

    appCntxt->dispMergeNodeCore        = createParams->dispMergeNodeCore;
    appCntxt->holeFillingNodeCore      = createParams->holeFillingNodeCore;
    appCntxt->medianFilterNodeCore     = createParams->medianFilterNodeCore;

    if (appCntxt->dispMergeNodeCore == NULL)
    {
        appCntxt->dispMergeNodeCore = ML_SDEAPPLIB_DEFAULT_CORE_MAPPING;
    }
    if (appCntxt->holeFillingNodeCore == NULL)
    {
        appCntxt->holeFillingNodeCore = ML_SDEAPPLIB_DEFAULT_CORE_MAPPING;
    }
    if (appCntxt->medianFilterNodeCore == NULL)
    {
        appCntxt->medianFilterNodeCore = ML_SDEAPPLIB_DEFAULT_CORE_MAPPING;
    }

    /*
     * set up input objects
     */
    if (appCntxt->createInputFlag == 1)
    {
        // input left image
        if (appCntxt->inputFormat == PTK_IMG_FORMAT_Y)
        {
            appCntxt->vxLeftRectImageL0[0] = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_U8);
        } else 
        {
            appCntxt->vxLeftRectImageL0[0] = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_NV12);
        }
        if (appCntxt->vxLeftRectImageL0[0] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);
        }

        // input right image
        if (appCntxt->inputFormat == PTK_IMG_FORMAT_Y)
        {
            appCntxt->vxRightRectImageL0[0] = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_U8);
        } else 
        {
            appCntxt->vxRightRectImageL0[0] = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_NV12);
        }
        if (appCntxt->vxRightRectImageL0[0] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);
        }
    } else 
    {
        for(i = 0; i < appCntxt->inputPipelineDepth; i++)
        {
            appCntxt->vxLeftRectImageL0[i]  = createParams->vxLeftRectImageL0[i];
        }
        for(i = 0; i < appCntxt->pipelineDepth; i++)
        {
            //appCntxt->vxLeftRectImageL0[i]  = createParams->vxLeftRectImageL0[i];
            appCntxt->vxRightRectImageL0[i] = createParams->vxRightRectImageL0[i];
        }
    } 

    /*
     * set up output objects
     */
    if (appCntxt->createOutputFlag == 1)
    {
        if (appCntxt->enableMedianFilter)
        {
            appCntxt->vxMedianFilteredDisparity[0] = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_S16);
            if (appCntxt->vxMedianFilteredDisparity[0] == NULL)
            {
                PTK_printf("[%s:%d] vxCreateImage() failed\n",
                           __FUNCTION__, __LINE__);
            }
        } 

        appCntxt->vxMergeDisparityL0[0] = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_S16);
        if (appCntxt->vxMergeDisparityL0[0] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateImage() failed\n",
                        __FUNCTION__, __LINE__);
        }
        createParams->vxMergeDisparityL0[0]  = appCntxt->vxMergeDisparityL0[0];
    } else 
    {
        if (appCntxt->enableMedianFilter)
        {
            for(i = 0; i < appCntxt->pipelineDepth; i++)
            {
                appCntxt->vxMedianFilteredDisparity[i]  = createParams->vxMedianFilteredDisparity[i];
            }
        } 

        for(i = 0; i < appCntxt->pipelineDepth; i++)
        {
            appCntxt->vxMergeDisparityL0[i] = createParams->vxMergeDisparityL0[i];
        }
    }
}


vx_status ML_SDEAPPLIB_createGraph(ML_SDEAPPLIB_Handle handle)
{
    ML_SDEAPPLIB_Context       * appCntxt = (ML_SDEAPPLIB_Context *)handle;
    tivx_vpac_msc_coefficients_t coeffs;

    vx_status                    status = VX_SUCCESS;

    /* Mapping Graph */
    PTK_assert(appCntxt->vxGraph);

    /*********************/
    /* For the 1st layer */
    /*********************/

    /* DMPAC SDE node */
    appCntxt->vxSdeConfig = vxCreateUserDataObject(appCntxt->vxContext, "tivx_dmpac_sde_params_t",
                                                   sizeof(tivx_dmpac_sde_params_t), &appCntxt->sdeCfg);
    PTK_assert(appCntxt->vxSdeConfig);
    vxSetReferenceName((vx_reference)appCntxt->vxSdeConfig, "Stereo_Config");

    /* 16-bit output from SDE */
    appCntxt->vxSde16BitOutputL0 = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_S16);
    PTK_assert(appCntxt->vxSde16BitOutputL0);
    vxSetReferenceName((vx_reference)appCntxt->vxSde16BitOutputL0, "Stereo_OutputDisparityImageS16_L0");

    /* histogram output from SDE */
    appCntxt->vxHistogramL0 = vxCreateDistribution(appCntxt->vxContext, 128, 0, 4096);
    PTK_assert(appCntxt->vxHistogramL0);
    vxSetReferenceName((vx_reference)appCntxt->vxHistogramL0, "Stereo_OutputHistogramDistribution_L0");

    appCntxt->vxDmpacSdeNodeL0 = tivxDmpacSdeNode(appCntxt->vxGraph,
                                                  appCntxt->vxSdeConfig,
                                                  appCntxt->vxLeftRectImageL0[0],
                                                  appCntxt->vxRightRectImageL0[0],
                                                  appCntxt->vxSde16BitOutputL0,
                                                  appCntxt->vxHistogramL0);
    PTK_assert(appCntxt->vxDmpacSdeNodeL0);
    vxSetReferenceName((vx_reference)appCntxt->vxDmpacSdeNodeL0, "DMPAC_SDE_Processing_L0");

    status = vxSetNodeTarget(appCntxt->vxDmpacSdeNodeL0, VX_TARGET_STRING, TIVX_TARGET_DMPAC_SDE);
    PTK_assert(status==VX_SUCCESS);


    if (appCntxt->numLayers > 1)
    {
        /* For the 2nd Layer */
        tivx_vpac_msc_coefficients_params_init(&coeffs, VX_INTERPOLATION_BILINEAR);

        /* Set Coefficients */
        appCntxt->vxMscCoeff = vxCreateUserDataObject(appCntxt->vxContext,
                                                      "tivx_vpac_msc_coefficients_t",
                                                      sizeof(tivx_vpac_msc_coefficients_t),
                                                      NULL);

        vxCopyUserDataObject(appCntxt->vxMscCoeff, 0,
                             sizeof(tivx_vpac_msc_coefficients_t),
                             &coeffs,
                             VX_WRITE_ONLY,
                             VX_MEMORY_TYPE_HOST);

        /* 2nd layer image */
        if (appCntxt->inputFormat == PTK_IMG_FORMAT_Y)
        {
            appCntxt->vxLeftImageL1 = vxCreateImage(appCntxt->vxContext, appCntxt->width >> 1, appCntxt->height >> 1, VX_DF_IMAGE_U8);
            PTK_assert(appCntxt->vxLeftImageL1);
            vxSetReferenceName((vx_reference)appCntxt->vxLeftImageL1, "InputLeftImageU8_L1");

            appCntxt->vxRightImageL1 = vxCreateImage(appCntxt->vxContext, appCntxt->width >> 1, appCntxt->height >> 1, VX_DF_IMAGE_U8);
            PTK_assert(appCntxt->vxRightImageL1);
            vxSetReferenceName((vx_reference)appCntxt->vxRightImageL1, "InputRightImageU8_L1");
        } else
        {
            appCntxt->vxLeftImageL1 = vxCreateImage(appCntxt->vxContext, appCntxt->width >> 1, appCntxt->height >> 1, VX_DF_IMAGE_NV12);
            PTK_assert(appCntxt->vxLeftImageL1);
            vxSetReferenceName((vx_reference)appCntxt->vxLeftImageL1, "InputLeftImage_NV12_L1");

            appCntxt->vxRightImageL1 = vxCreateImage(appCntxt->vxContext, appCntxt->width >> 1, appCntxt->height >> 1, VX_DF_IMAGE_NV12);
            PTK_assert(appCntxt->vxRightImageL1);
            vxSetReferenceName((vx_reference)appCntxt->vxRightImageL1, "InputRightImage_NV12_L1");
        }

        /* 16-bit output from the 2nd layer SDE */
        appCntxt->vxSde16BitOutputL1 = vxCreateImage(appCntxt->vxContext, appCntxt->width >> 1, appCntxt->height >> 1, VX_DF_IMAGE_S16);
        PTK_assert(appCntxt->vxSde16BitOutputL1);
        vxSetReferenceName((vx_reference)appCntxt->vxSde16BitOutputL1, "Stereo_OutputDisparityImageS16_L1");

        /* histogram output from SDE */
        appCntxt->vxHistogramL1 = vxCreateDistribution(appCntxt->vxContext, 128, 0, 4096);
        PTK_assert(appCntxt->vxHistogramL1);
        vxSetReferenceName((vx_reference)appCntxt->vxHistogramL1, "Stereo_OutputHistogramDistribution_L1");

        /* Scaler Nodes  */
        appCntxt-> vxMscNodeL_L1 = tivxVpacMscScaleNode(appCntxt->vxGraph, appCntxt->vxLeftRectImageL0[0], appCntxt->vxLeftImageL1, NULL, NULL, NULL, NULL);
        vxSetNodeTarget(appCntxt-> vxMscNodeL_L1, VX_TARGET_STRING, TIVX_TARGET_VPAC_MSC1);
        vxSetReferenceName((vx_reference)appCntxt-> vxMscNodeL_L1, "ScalerNode_Left_L1");

        appCntxt-> vxMscNodeR_L1 = tivxVpacMscScaleNode(appCntxt->vxGraph, appCntxt->vxRightRectImageL0[0], appCntxt->vxRightImageL1, NULL, NULL, NULL, NULL);
        vxSetNodeTarget(appCntxt-> vxMscNodeR_L1, VX_TARGET_STRING, TIVX_TARGET_VPAC_MSC1);
        vxSetReferenceName((vx_reference)appCntxt-> vxMscNodeR_L1, "ScalerNode_Right_L1");

        /* SDE Node */
        appCntxt->vxDmpacSdeNodeL1 = tivxDmpacSdeNode(appCntxt->vxGraph,
                                                      appCntxt->vxSdeConfig,
                                                      appCntxt->vxLeftImageL1,
                                                      appCntxt->vxRightImageL1,
                                                      appCntxt->vxSde16BitOutputL1,
                                                      appCntxt->vxHistogramL1);

        PTK_assert(appCntxt->vxDmpacSdeNodeL1);
        vxSetReferenceName((vx_reference)appCntxt->vxDmpacSdeNodeL1, "DMPAC_SDE_Processing_L1");
    
        status = vxSetNodeTarget(appCntxt->vxDmpacSdeNodeL1, VX_TARGET_STRING, TIVX_TARGET_DMPAC_SDE);
        PTK_assert(status==VX_SUCCESS);
    }

    if (appCntxt->numLayers > 2)
    {
        /* For the 3rd Layer */
        if (appCntxt->inputFormat == PTK_IMG_FORMAT_Y)
        {
            appCntxt->vxLeftImageL2 = vxCreateImage(appCntxt->vxContext, appCntxt->width >> 2, appCntxt->height >> 2, VX_DF_IMAGE_U8);
            PTK_assert(appCntxt->vxLeftImageL2);
            vxSetReferenceName((vx_reference)appCntxt->vxLeftImageL2, "InputLeftImageU8_L2");

            appCntxt->vxRightImageL2 = vxCreateImage(appCntxt->vxContext, appCntxt->width >> 2, appCntxt->height >> 2, VX_DF_IMAGE_U8);
            PTK_assert(appCntxt->vxRightImageL2);
            vxSetReferenceName((vx_reference)appCntxt->vxRightImageL2, "InputRightImageU8_L2");
        } else
        {
            appCntxt->vxLeftImageL2 = vxCreateImage(appCntxt->vxContext, appCntxt->width >> 2, appCntxt->height >> 2, VX_DF_IMAGE_NV12);
            PTK_assert(appCntxt->vxLeftImageL2);
            vxSetReferenceName((vx_reference)appCntxt->vxLeftImageL2, "InputLeftImageU8_NV12_L2");

            appCntxt->vxRightImageL2 = vxCreateImage(appCntxt->vxContext, appCntxt->width >> 2, appCntxt->height >> 2, VX_DF_IMAGE_NV12);
            PTK_assert(appCntxt->vxRightImageL2);
            vxSetReferenceName((vx_reference)appCntxt->vxRightImageL2, "InputRightImage_NV12_L2");
        }

        /* 16-bit output from the 3rd layer SDE */
        appCntxt->vxSde16BitOutputL2 = vxCreateImage(appCntxt->vxContext, appCntxt->width >> 2, appCntxt->height >>2, VX_DF_IMAGE_S16);
        PTK_assert(appCntxt->vxSde16BitOutputL2);
        vxSetReferenceName((vx_reference)appCntxt->vxSde16BitOutputL2, "Stereo_OutputDisparityImageS16_L2");

        /* histogram output from SDE */
        appCntxt->vxHistogramL2 = vxCreateDistribution(appCntxt->vxContext, 128, 0, 4096);
        PTK_assert(appCntxt->vxHistogramL2);
        vxSetReferenceName((vx_reference)appCntxt->vxHistogramL2, "Stereo_OutputHistogramDistribution_L2");

        /* Scaler Nodes   */
        appCntxt-> vxMscNodeL_L2 = tivxVpacMscScaleNode(appCntxt->vxGraph, appCntxt->vxLeftImageL1, appCntxt->vxLeftImageL2, NULL, NULL, NULL, NULL);
        vxSetNodeTarget(appCntxt-> vxMscNodeL_L2, VX_TARGET_STRING, TIVX_TARGET_VPAC_MSC1);
        vxSetReferenceName((vx_reference)appCntxt-> vxMscNodeL_L2, "ScalerNode_Left_L2");

        appCntxt-> vxMscNodeR_L2 = tivxVpacMscScaleNode(appCntxt->vxGraph, appCntxt->vxRightImageL1, appCntxt->vxRightImageL2, NULL, NULL, NULL, NULL);
        vxSetNodeTarget(appCntxt-> vxMscNodeR_L2, VX_TARGET_STRING, TIVX_TARGET_VPAC_MSC1);
        vxSetReferenceName((vx_reference)appCntxt-> vxMscNodeR_L2, "ScalerNode_Right_L2");


        /* SDE Node */
        appCntxt->vxDmpacSdeNodeL2 = tivxDmpacSdeNode(appCntxt->vxGraph,
                                                      appCntxt->vxSdeConfig,
                                                      appCntxt->vxLeftImageL2,
                                                      appCntxt->vxRightImageL2,
                                                      appCntxt->vxSde16BitOutputL2,
                                                      appCntxt->vxHistogramL2);

        PTK_assert(appCntxt->vxDmpacSdeNodeL2);
        vxSetReferenceName((vx_reference)appCntxt->vxDmpacSdeNodeL2, "DMPAC_SDE_Processing_L2");
    
        status = vxSetNodeTarget(appCntxt->vxDmpacSdeNodeL2, VX_TARGET_STRING, TIVX_TARGET_DMPAC_SDE);
        PTK_assert(status==VX_SUCCESS);
    }

    if (appCntxt->numLayers > 2)
    {
        /* Disparity Merge Node */
        appCntxt->vxDisparityMergeConfigL2 = vxCreateUserDataObject(appCntxt->vxContext, "tivx_disparity_merge_params_t",
                                                                    sizeof(tivx_disparity_merge_params_t), &appCntxt->dmCfgL2);
        PTK_assert(appCntxt->vxDisparityMergeConfigL2);
        vxSetReferenceName((vx_reference)appCntxt->vxDisparityMergeConfigL2, "Disparit_Merge_Config_L2");

        /* Merged disparity */
        appCntxt->vxMergeDisparityL1 = vxCreateImage(appCntxt->vxContext, appCntxt->width >> 1, appCntxt->height >> 1, VX_DF_IMAGE_S16);
        PTK_assert(appCntxt->vxMergeDisparityL1);
        vxSetReferenceName((vx_reference)appCntxt->vxMergeDisparityL1, "Stereo_MergedDisparityImageS16_L1");

        appCntxt->vxDisparityMergeNodeL2 = tivxDisparityMergeNode(appCntxt->vxGraph,
                                                                  appCntxt->vxDisparityMergeConfigL2,
                                                                  appCntxt->vxSde16BitOutputL2,
                                                                  appCntxt->vxSde16BitOutputL1,
                                                                  appCntxt->vxMergeDisparityL1);

        PTK_assert(appCntxt->vxDisparityMergeNodeL2);
        vxSetReferenceName((vx_reference)appCntxt->vxDisparityMergeNodeL2, "Disparity_Merge_L2");
    
        status = vxSetNodeTarget(appCntxt->vxDisparityMergeNodeL2, VX_TARGET_STRING, appCntxt->dispMergeNodeCore);
        PTK_assert(status==VX_SUCCESS);

    }

    if (appCntxt->numLayers > 1)
    {
        /* Disparity Merge Node */
        appCntxt->vxDisparityMergeConfigL1 = vxCreateUserDataObject(appCntxt->vxContext, "tivx_disparity_merge_params_t",
                                                                    sizeof(tivx_disparity_merge_params_t), &appCntxt->dmCfgL1);
        PTK_assert(appCntxt->vxDisparityMergeConfigL1);
        vxSetReferenceName((vx_reference)appCntxt->vxDisparityMergeConfigL1, "Disparit_Merge_Config_L1");

        if (appCntxt->numLayers > 2)
        {
            appCntxt->vxDisparityMergeNodeL1 = tivxDisparityMergeNode(appCntxt->vxGraph,
                                                                      appCntxt->vxDisparityMergeConfigL1,
                                                                      appCntxt->vxMergeDisparityL1,
                                                                      appCntxt->vxSde16BitOutputL0,
                                                                      appCntxt->vxMergeDisparityL0[0]);
        } else 
        {
            appCntxt->vxDisparityMergeNodeL1 = tivxDisparityMergeNode(appCntxt->vxGraph,
                                                                      appCntxt->vxDisparityMergeConfigL1,
                                                                      appCntxt->vxSde16BitOutputL1,
                                                                      appCntxt->vxSde16BitOutputL0,
                                                                      appCntxt->vxMergeDisparityL0[0]);
        }

        PTK_assert(appCntxt->vxDisparityMergeNodeL1);
        vxSetReferenceName((vx_reference)appCntxt->vxDisparityMergeNodeL1, "Disparity_Merge_L1");
    
        status = vxSetNodeTarget(appCntxt->vxDisparityMergeNodeL1, VX_TARGET_STRING, appCntxt->dispMergeNodeCore);
        PTK_assert(status==VX_SUCCESS);
    }


    /* Hole Fiilling Node */
    appCntxt->vxHoleFillingConfig = vxCreateUserDataObject(appCntxt->vxContext, "tivx_hole_filling_params_t",
                                                           sizeof(tivx_hole_filling_params_t), &appCntxt->hfCfg);
    PTK_assert(appCntxt->vxHoleFillingConfig);
    vxSetReferenceName((vx_reference)appCntxt->vxHoleFillingConfig, "Hole_Filling_Config");

    appCntxt->vxHoleFillingNode = tivxHoleFillingNode(appCntxt->vxGraph,
                                                      appCntxt->vxHoleFillingConfig,
                                                      appCntxt->vxMergeDisparityL0[0]);

    PTK_assert(appCntxt->vxHoleFillingNode);
    vxSetReferenceName((vx_reference)appCntxt->vxHoleFillingNode, "Hole_Filling");
    
    status = vxSetNodeTarget(appCntxt->vxHoleFillingNode, VX_TARGET_STRING, appCntxt->holeFillingNodeCore);
    PTK_assert(status==VX_SUCCESS);


    if (appCntxt->enableMedianFilter == 1)
    {
        /* Median filter Node */
        appCntxt->vxMedianFilterConfig = vxCreateUserDataObject(appCntxt->vxContext, "tivx_median_filter_params_t",
                                                               sizeof(tivx_median_filter_params_t), &appCntxt->mfCfg);
        PTK_assert(appCntxt->vxMedianFilterConfig);
        vxSetReferenceName((vx_reference)appCntxt->vxMedianFilterConfig, "Median_Filter_Config");
    
        appCntxt->vxMedianFilterNode = tivxMedianFilterNode(appCntxt->vxGraph,
                                                            appCntxt->vxMedianFilterConfig,
                                                            appCntxt->vxMergeDisparityL0[0],
                                                            appCntxt->vxMedianFilteredDisparity[0]);

        PTK_assert(appCntxt->vxMedianFilterNode);
        vxSetReferenceName((vx_reference)appCntxt->vxMedianFilterNode, "Median Filter");

        status = vxSetNodeTarget(appCntxt->vxMedianFilterNode, VX_TARGET_STRING, appCntxt->medianFilterNodeCore);
        PTK_assert(status==VX_SUCCESS);
    }

    return status;

} /* ML_SDEAPPLIB_createGraph */


vx_status ML_SDEAPPLIB_initScaler(ML_SDEAPPLIB_Handle handle)
{
    ML_SDEAPPLIB_Context       * appCntxt;

    vx_status                    vxStatus = VX_SUCCESS;
    vx_reference                 refs[1];

    appCntxt = (ML_SDEAPPLIB_Context *)handle;
    refs[0] = (vx_reference)appCntxt->vxMscCoeff;

    if (appCntxt->numLayers > 1)
    {
        vxStatus = tivxNodeSendCommand(appCntxt->vxMscNodeL_L1, 0u,
                                       TIVX_VPAC_MSC_CMD_SET_COEFF,
                                       refs, 1u);
        if(VX_SUCCESS != vxStatus)
        {
            printf("MSC: vxMscNodeL_L1 Node send command failed!\n");
        }
        
        vxStatus = tivxNodeSendCommand(appCntxt->vxMscNodeR_L1, 0u,
                                       TIVX_VPAC_MSC_CMD_SET_COEFF,
                                       refs, 1u);
        if(VX_SUCCESS != vxStatus)
        {
            printf("MSC: vxMscNodeR_L1 Node send command failed!\n");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    if (appCntxt->numLayers > 2)
    {
        vxStatus = tivxNodeSendCommand(appCntxt->vxMscNodeL_L2, 0u,
                                      TIVX_VPAC_MSC_CMD_SET_COEFF,
                                      refs, 1u);
        if(VX_SUCCESS != vxStatus)
        {
            printf("MSC: vxMscNodeL_L2 Node send command failed!\n");
        }
        
        vxStatus = tivxNodeSendCommand(appCntxt->vxMscNodeR_L2, 0u,
                                      TIVX_VPAC_MSC_CMD_SET_COEFF,
                                      refs, 1u);
        if(VX_SUCCESS != vxStatus)
        {
            printf("MSC: vxMscNodeR_L2 Node send command failed!\n");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return vxStatus;
}


void ML_SDEAPPLIB_releaseGraph(ML_SDEAPPLIB_Handle handle)
{
    ML_SDEAPPLIB_Context * appCntxt = (ML_SDEAPPLIB_Context *)handle;

    /* Layer 0 */
    /* release DMPAC SDE node */
    vxReleaseNode(&appCntxt->vxDmpacSdeNodeL0);
    vxReleaseUserDataObject(&appCntxt->vxSdeConfig);
    vxReleaseImage(&appCntxt->vxSde16BitOutputL0);

    if (appCntxt->createInputFlag == 1)
    {
        vxReleaseImage(&appCntxt->vxLeftRectImageL0[0]);
        vxReleaseImage(&appCntxt->vxRightRectImageL0[0]);
    }
    vxReleaseDistribution(&appCntxt->vxHistogramL0);

    /* Layer 1 */
    if (appCntxt->numLayers > 1)
    {
        vxReleaseNode(&appCntxt->vxMscNodeL_L1);
        vxReleaseNode(&appCntxt->vxMscNodeR_L1);
        vxReleaseNode(&appCntxt->vxDmpacSdeNodeL1);
    
        vxReleaseUserDataObject(&appCntxt->vxMscCoeff);
        vxReleaseImage(&appCntxt->vxLeftImageL1);
        vxReleaseImage(&appCntxt->vxRightImageL1);
        vxReleaseImage(&appCntxt->vxSde16BitOutputL1);

        vxReleaseDistribution(&appCntxt->vxHistogramL1);

        /* Disparity merge Node */
        vxReleaseNode(&appCntxt->vxDisparityMergeNodeL1);
        vxReleaseUserDataObject(&appCntxt->vxDisparityMergeConfigL1);

        if (appCntxt->createOutputFlag == 1)
        {
            vxReleaseImage(&appCntxt->vxMergeDisparityL0[0]);
        }
    }

    /* Layer2 */
    if (appCntxt->numLayers > 2)
    {
        vxReleaseNode(&appCntxt->vxMscNodeL_L2);
        vxReleaseNode(&appCntxt->vxMscNodeR_L2);
        vxReleaseNode(&appCntxt->vxDmpacSdeNodeL2);
        
        vxReleaseImage(&appCntxt->vxLeftImageL2);
        vxReleaseImage(&appCntxt->vxRightImageL2);
        vxReleaseImage(&appCntxt->vxSde16BitOutputL2);

        vxReleaseDistribution(&appCntxt->vxHistogramL2);

        /* Disparity merge Node */
        vxReleaseNode(&appCntxt->vxDisparityMergeNodeL2);
        vxReleaseUserDataObject(&appCntxt->vxDisparityMergeConfigL2);
        vxReleaseImage(&appCntxt->vxMergeDisparityL1);
    }

    /* Hole Filling Node */
    vxReleaseNode(&appCntxt->vxHoleFillingNode);
    vxReleaseUserDataObject(&appCntxt->vxHoleFillingConfig);

    if (appCntxt->enableMedianFilter)
    {
        /* Median Filter Node */
        vxReleaseNode(&appCntxt->vxMedianFilterNode);
        vxReleaseUserDataObject(&appCntxt->vxMedianFilterConfig);

        if (appCntxt->createOutputFlag == 1)
        {
            vxReleaseImage(&appCntxt->vxMedianFilteredDisparity[0]);
        }
    }

    return;

} /* ML_SDEAPPLIB_releaseGraph */


vx_node  ML_SDEAPPLIB_getSDENodeL0(ML_SDEAPPLIB_Handle handle)
{
    ML_SDEAPPLIB_Context *appCntxt = (ML_SDEAPPLIB_Context *)handle;
    return appCntxt->vxDmpacSdeNodeL0;
}

vx_node  ML_SDEAPPLIB_getSDENodeL1(ML_SDEAPPLIB_Handle handle)
{
    ML_SDEAPPLIB_Context *appCntxt = (ML_SDEAPPLIB_Context *)handle;
    return appCntxt->vxDmpacSdeNodeL1;
}

vx_node  ML_SDEAPPLIB_getSDENodeL2(ML_SDEAPPLIB_Handle handle)
{
    ML_SDEAPPLIB_Context *appCntxt = (ML_SDEAPPLIB_Context *)handle;
    return appCntxt->vxDmpacSdeNodeL2;
}

vx_node  ML_SDEAPPLIB_getMedFilterNode(ML_SDEAPPLIB_Handle handle)
{
    ML_SDEAPPLIB_Context *appCntxt = (ML_SDEAPPLIB_Context *)handle;
    return appCntxt->vxMedianFilterNode;
}

vx_node  ML_SDEAPPLIB_getMergeNodeL1(ML_SDEAPPLIB_Handle handle)
{
    ML_SDEAPPLIB_Context *appCntxt = (ML_SDEAPPLIB_Context *)handle;
    return appCntxt->vxDisparityMergeNodeL1;
}

vx_node  ML_SDEAPPLIB_getMergeNodeL2(ML_SDEAPPLIB_Handle handle)
{
    ML_SDEAPPLIB_Context *appCntxt = (ML_SDEAPPLIB_Context *)handle;
    return appCntxt->vxDisparityMergeNodeL2;
}

vx_node  ML_SDEAPPLIB_getLeftMSCNodeL1(ML_SDEAPPLIB_Handle handle)
{
    ML_SDEAPPLIB_Context *appCntxt = (ML_SDEAPPLIB_Context *)handle;
    return appCntxt->vxMscNodeL_L1;
}

vx_node  ML_SDEAPPLIB_getRightMSCNodeL1(ML_SDEAPPLIB_Handle handle)
{
    ML_SDEAPPLIB_Context *appCntxt = (ML_SDEAPPLIB_Context *)handle;
    return appCntxt->vxMscNodeR_L1;
}

vx_node  ML_SDEAPPLIB_getLeftMSCNodeL2(ML_SDEAPPLIB_Handle handle)
{
    ML_SDEAPPLIB_Context *appCntxt = (ML_SDEAPPLIB_Context *)handle;
    return appCntxt->vxMscNodeL_L2;
}

vx_node  ML_SDEAPPLIB_getRightMSCNodeL2(ML_SDEAPPLIB_Handle handle)
{
    ML_SDEAPPLIB_Context *appCntxt = (ML_SDEAPPLIB_Context *)handle;
    return appCntxt->vxMscNodeR_L2;
}

