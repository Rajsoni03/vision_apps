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
 * Redistributions must postserve existing copyright notices and reproduce this license
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
 * THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPOSTSS
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
#include <tivx_pixel_visualization_host.h>
#include <cm_postproc_node_cntxt.h>

/* Update the color map as required, the lenth of array should be less than
 * or equal to, TIVX_PIXEL_VIZ_MAX_CLASS
 * The format is R, G, B
 */
static const vx_uint8 color_map[20][3] =
{{128,  64, 128},{244,  35, 232},{ 70,  70,  70},{102, 102, 156},{190, 153, 153},
 {153, 153, 153},{250, 170,  30},{220, 220,   0},{107, 142,  35},{152, 251, 152},
 { 70, 130, 180},{220,  20,  60},{255,   0,   0},{  0,   0, 142},{  0,   0,  70},
 {  0,  60, 100},{  0,  80, 100},{  0,   0, 230},{119,  11,  32},{128, 128, 128}};


static vx_image **alloc2DImageArray(
        uint32_t    dim1,
        uint32_t    dim2,
        uint32_t   *memSize)
{
    vx_image  **out;
    vx_image   *tmp;
    uint8_t    *ptr;
    uint32_t    size;

    size = dim1 * (sizeof(vx_image **) + (sizeof(vx_image*) * dim2));
    ptr  = (uint8_t *)tivxMemAlloc(size, TIVX_MEM_EXTERNAL);

    if (ptr == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Failed to allocate output image array");

        out = NULL;
    }
    else
    {
        *memSize = size;
        out = (vx_image **)ptr;
        ptr += sizeof(vx_image **) * dim1;
        tmp = (vx_image *)ptr;

        for (uint32_t i = 0; i < dim1; i++, tmp++)
        {
            out[i] = tmp;
        }
    }

    return out;
}

static void CM_postProcNodeCntxtSetParams(
        CM_PostProcNodeCntxt           *postProcObj,
        const CM_PostProcCreateParams  *createParams)
{
    tivxPixelVizParams *vxVizParams;
    int32_t             i;

    postProcObj->outWidth      = createParams->outWidth;
    postProcObj->outHeight     = createParams->outHeight;
    postProcObj->pipelineDepth = createParams->pipelineDepth;

    vxVizParams = &postProcObj->vxVizParams;

    vxVizParams->ip_rgb_or_yuv = 1;
    vxVizParams->op_rgb_or_yuv = 1;

    vxVizParams->num_input_tensors  = 1;
    vxVizParams->num_output_tensors = 1;

    vxVizParams->tidl_8bit_16bit_flag = 0;

    vxVizParams->num_classes[0] = createParams->numClasses;
    vxVizParams->num_classes[1] = 0;
    vxVizParams->num_classes[2] = 0;

    memcpy(vxVizParams->color_map[0],
           color_map,
           vxVizParams->num_classes[0]*3*sizeof(vx_uint8));

    vxVizParams->max_value[0] = vxVizParams->num_classes[0] - 1;

    for (i = 0; i < (int32_t)TIVX_PIXEL_VIZ_MAX_TENSOR; i++)
    {
        vxVizParams->valid_region[i][0] = 0;
        vxVizParams->valid_region[i][1] = 0;
        vxVizParams->valid_region[i][2] = postProcObj->outWidth - 1;
        vxVizParams->valid_region[i][3] = postProcObj->outHeight - 1;
    }
}

vx_status CM_postProcNodeCntxtInit(
        CM_PostProcNodeCntxt           *postProcObj,
        vx_context                      context,
        const CM_PostProcCreateParams  *createParams)
{
    sTIDL_IOBufDesc_t  *ioBufDesc;
    tivxTIDLJ7Params   *tidlParams;
    tivxPixelVizParams *vxVizParams;
    vx_map_id           map_id_config;
    vx_int32            id;
    vx_status           vxStatus;

    vxStatus = VX_SUCCESS;

    if (postProcObj == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'postProcObj' NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (context == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'context' NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (createParams == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'createParams' NULL.");
        vxStatus = VX_FAILURE;
    }
    else if ((createParams->pipelineDepth == 0) ||
             (createParams->pipelineDepth > CM_MAX_PIPELINE_DEPTH))
    {
        VX_PRINT(VX_ZONE_ERROR, "Invalid pipeline depth.");
        vxStatus = VX_FAILURE;
    }
    else if (postProcObj->state != CM_POSTPROC_NODE_CNTXT_STATE_INVALID)
    {
        VX_PRINT(VX_ZONE_ERROR, "Invalid state.");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        CM_postProcNodeCntxtSetParams(postProcObj, createParams);

        vxStatus = vxMapUserDataObject(createParams->tidlCfg,
                                       0,
                                       sizeof(tivxTIDLJ7Params),
                                       &map_id_config,
                                       (void **)&tidlParams,
                                       VX_READ_ONLY,
                                       VX_MEMORY_TYPE_HOST,
                                       0);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "vxMapUserDataObject() failed.");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        ioBufDesc = (sTIDL_IOBufDesc_t *)&tidlParams->ioBufDesc;

        vxVizParams = &postProcObj->vxVizParams;

        for (id = 0; id < ioBufDesc->numOutputBuf; id++)
        {
            if (id < (int32_t)TIVX_PIXEL_VIZ_MAX_TENSOR)
            {
                vxVizParams->output_width[id] = ioBufDesc->outWidth[id];
                vxVizParams->output_height[id] = ioBufDesc->outHeight[id];
                vxVizParams->output_buffer_pitch[id] = ioBufDesc->outWidth[id] +
                                                      ioBufDesc->outPadL[id] +
                                                      ioBufDesc->outPadR[id];
                vxVizParams->output_buffer_offset[id] = ioBufDesc->outPadL[id] *
                    (vxVizParams->output_buffer_pitch[id] * ioBufDesc->outPadT[id]);
            }
        }

        if ((ioBufDesc->outElementType[0] == TIDL_UnsignedChar) ||
            (ioBufDesc->outElementType[0] == TIDL_SignedChar))
        {
            vxVizParams->tidl_8bit_16bit_flag = 0;
        }
        else if ((ioBufDesc->outElementType[0] == TIDL_UnsignedShort) ||
                 (ioBufDesc->outElementType[0] == TIDL_SignedShort))
        {
            vxVizParams->tidl_8bit_16bit_flag = 1;
        }

        vxStatus = vxUnmapUserDataObject(createParams->tidlCfg, map_id_config);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "vxUnmapUserDataObject() failed.");
        }

        /* Number of output tensors from the TIDL node should
         * equal the number of input tensors to the post-processing node.
         *
         * Number of output tensors will equal the number of input tensors. In
         * the end the number of output tensors should be the same as the
         * number of output tensors from the TIDL node.
         */
        postProcObj->numOutTensors = ioBufDesc->numOutputBuf;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /************************/
        /* Post processing node */
        /************************/
        postProcObj->vxVizConfig =
            vxCreateUserDataObject(context,
                                   "tivxPixelVizParams",
                                   sizeof(tivxPixelVizParams),
                                   NULL);

        if (postProcObj->vxVizConfig == NULL)
        {
            VX_PRINT(VX_ZONE_ERROR, "vxCreateUserDataObject() failed");
            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)postProcObj->vxVizConfig,
                               "VIZ_config");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxCopyUserDataObject(postProcObj->vxVizConfig,
                                        0,
                                        sizeof(tivxPixelVizParams),
                                        &postProcObj->vxVizParams,
                                        VX_WRITE_ONLY,
                                        VX_MEMORY_TYPE_HOST);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "vxCopyUserDataObject() failed");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        postProcObj->vxVizKernel =
            tivxAddKernelPixelViz(context,
                                  postProcObj->numOutTensors);

        if (postProcObj->vxVizKernel == NULL)
        {
            VX_PRINT(VX_ZONE_ERROR, "tivxAddKernelPixelViz() failed");
            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        postProcObj->vxOutputImage =
            alloc2DImageArray(postProcObj->pipelineDepth,
                              postProcObj->numOutTensors,
                              &postProcObj->vxOutImageMemSize);

        if (postProcObj->vxOutputImage == NULL)
        {
            VX_PRINT(VX_ZONE_ERROR, "alloc2DImageArray() failed.");
            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        postProcObj->vxOutputImageRefs =
            alloc2DImageArray(postProcObj->numOutTensors,
                              postProcObj->pipelineDepth,
                              &postProcObj->vxOutImageRefMemSize);

        if (postProcObj->vxOutputImageRefs == NULL)
        {
            VX_PRINT(VX_ZONE_ERROR, "alloc2DImageArray() failed.");
            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        for (uint32_t i = 0; i < postProcObj->pipelineDepth; i++)
        {
            for (uint32_t j = 0; j < postProcObj->numOutTensors; j++)
            {
                postProcObj->vxOutputImage[i][j] =
                        vxCreateImage(context,
                                      postProcObj->outWidth,
                                      postProcObj->outHeight,
                                      VX_DF_IMAGE_NV12);

                if (postProcObj->vxOutputImage[i][j] == NULL)
                {
                    VX_PRINT(VX_ZONE_ERROR, "vxCreateImage() failed");
                    vxStatus = VX_FAILURE;
                    break;
                }
                else
                {
                    postProcObj->vxOutputImageRefs[j][i] =
                                 postProcObj->vxOutputImage[i][j];

                    vxSetReferenceName((vx_reference)postProcObj->vxOutputImage[i][j],
                                       "PostProcOutputImage_NV12");
                }
            }

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                break;
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        postProcObj->outImageBaseParamIdx =
            TIVX_KERNEL_PIXEL_VISUALIZATION_OUTPUT_TENSOR_IDX +
            postProcObj->numOutTensors;

        postProcObj->state = CM_POSTPROC_NODE_CNTXT_STATE_INIT;
    }

    return vxStatus;

}

vx_status CM_postProcNodeCntxtSetup(
        CM_PostProcNodeCntxt   *postProcObj,
        vx_context              context,
        vx_graph                graph,
        vx_user_data_object     tidlOutArgs,
        vx_image                inputImage,
        vx_tensor              *detection)
{
    vx_status   vxStatus = VX_SUCCESS;

    if (postProcObj == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'postProcObj' NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (context == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'context' NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (graph == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'graph' NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (tidlOutArgs == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'tidlOutArgs' NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (inputImage == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'inputImage' NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (detection == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'detection' NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (postProcObj->state != CM_POSTPROC_NODE_CNTXT_STATE_INIT)
    {
        VX_PRINT(VX_ZONE_ERROR, "Invalid state.");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        postProcObj->vxNode =
            tivxPixelVizNode(graph,
                             postProcObj->vxVizKernel,
                             postProcObj->vxVizConfig,
                             tidlOutArgs,
                             inputImage,
                             postProcObj->numOutTensors,
                             detection,
                             postProcObj->vxOutputImage[0]);

        if (postProcObj->vxNode == NULL)
        {
            VX_PRINT(VX_ZONE_ERROR, "tivxPixelVizNode() failed");
            vxStatus = VX_FAILURE;
        }
        else
        {
            vxStatus = vxSetNodeTarget(postProcObj->vxNode,
                                       VX_TARGET_STRING,
                                       TIVX_TARGET_DSP2);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                VX_PRINT(VX_ZONE_ERROR, "vxSetNodeTarget() failed");
                vxStatus = VX_FAILURE;
            }
            else
            {
                vxSetReferenceName((vx_reference)postProcObj->vxNode,
                                   "PostProcNode");
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        postProcObj->state = CM_POSTPROC_NODE_CNTXT_STATE_SETUP;
    }

    return vxStatus;
}

vx_status CM_postProcNodeCntxtDeInit(
        CM_PostProcNodeCntxt   *postProcObj)
{
    vx_status   vxStatus = VX_SUCCESS;

    if (postProcObj == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'postProcObj' NULL.");
        vxStatus = VX_FAILURE;
    }
    else
    {
        if (postProcObj->state == CM_POSTPROC_NODE_CNTXT_STATE_INVALID)
        {
            VX_PRINT(VX_ZONE_ERROR, "Invalid state.");
            vxStatus = VX_FAILURE;
        }
        else
        {
            if (postProcObj->vxNode != NULL)
            {
                vxReleaseNode(&postProcObj->vxNode);
            }

            if (postProcObj->vxVizKernel != NULL)
            {
                /* Remove the kernel and not just release. */
                vxRemoveKernel(postProcObj->vxVizKernel);
            }

            if (postProcObj->vxVizConfig != NULL)
            {
                vxReleaseUserDataObject(&postProcObj->vxVizConfig);
            }

            /* Release the output image array. */
            if (postProcObj->vxOutputImage)
            {
                for (uint32_t i = 0; i < postProcObj->pipelineDepth; i++)
                {
                    for (uint32_t j = 0; j < postProcObj->numOutTensors; j++)
                    {
                        if (postProcObj->vxOutputImage[i][j] != NULL)
                        {
                            vxReleaseImage(&postProcObj->vxOutputImage[i][j]);
                        }
                    }
                }

                /* Release the output tensor array. */
                tivxMemFree(postProcObj->vxOutputImage,
                            postProcObj->vxOutImageMemSize,
                            TIVX_MEM_EXTERNAL);

                postProcObj->vxOutputImage = NULL;
            }

            /* Release the output image ref array. */
            if (postProcObj->vxOutputImageRefs)
            {
                /* Release the output tensor array. */
                tivxMemFree(postProcObj->vxOutputImageRefs,
                            postProcObj->vxOutImageRefMemSize,
                            TIVX_MEM_EXTERNAL);

                postProcObj->vxOutputImageRefs = NULL;
            }
        }

        postProcObj->state = CM_POSTPROC_NODE_CNTXT_STATE_INVALID;
    }

    return vxStatus;
}


