/*
*
* Copyright (c) 2021 Texas Instruments Incorporated
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

#include <TI/tivx.h>
#include <TI/j7.h>
#include <TI/tivx_img_proc.h>
#include <TI/tivx_target_kernel.h>
#include "tivx_kernels_target_utils.h"

#include <tivx_dl_pre_proc_host.h>

#include <utils/udma/include/app_udma.h>
#include <utils/mem/include/app_mem.h>

static tivx_target_kernel vx_dl_pre_proc_target_kernel = NULL;

static vx_status VX_CALLBACK tivxKernelDLPreProcCreate
(
    tivx_target_kernel_instance kernel,
    tivx_obj_desc_t *obj_desc[],
    vx_uint16 num_params,
    void *priv_arg
)
{
    vx_status status = VX_SUCCESS;
    int32_t i;

    for (i = 0U; i < num_params; i ++)
    {
        if (NULL == obj_desc[i])
        {
            status = VX_FAILURE;
            break;
        }
    }

    tivxDLPreProcParams * prms = NULL;

    prms = tivxMemAlloc(sizeof(tivxDLPreProcParams), TIVX_MEM_EXTERNAL);
    if (NULL == prms)
    {
        status = VX_FAILURE;
    }
    else
    {
        status = tivxSetTargetKernelInstanceContext(kernel, prms,  sizeof(tivxDLPreProcParams));
    }

    return (status);
}

static vx_status VX_CALLBACK tivxKernelDLPreProcDelete(
    tivx_target_kernel_instance kernel, tivx_obj_desc_t *obj_desc[],
    vx_uint16 num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    uint32_t i;

    for (i = 0U; i < num_params; i ++)
    {
        if (NULL == obj_desc[i])
        {
            status = VX_FAILURE;
            break;
        }
    }

    if (VX_SUCCESS == status)
    {
        uint32_t size;
        tivxDLPreProcParams *prms = NULL;

        status = tivxGetTargetKernelInstanceContext(kernel,
            (void **)&prms, &size);

        if (VX_SUCCESS == status)
        {
            tivxMemFree(prms, sizeof(tivxDLPreProcParams), TIVX_MEM_EXTERNAL);
        }
    }

    return (status);
}

static vx_status VX_CALLBACK tivxKernelDLPreProcProcess
(
    tivx_target_kernel_instance kernel,
    tivx_obj_desc_t *obj_desc[],
    vx_uint16 num_params,
    void *priv_arg
)
{
    vx_status status = VX_SUCCESS;

    tivxDLPreProcParams *prms = NULL;
    vx_int32 i;

    for (i = 0U; i < num_params; i ++)
    {
        if (NULL == obj_desc[i])
        {
            status = VX_FAILURE;
            break;
        }
    }

    if(status==VX_SUCCESS)
    {
        uint32_t size;

        status = tivxGetTargetKernelInstanceContext(kernel,
            (void **)&prms, &size);
        if ((VX_SUCCESS != status) || (NULL == prms) ||
            (sizeof(tivxDLPreProcParams) != size))
        {
            status = VX_FAILURE;
        }
    }

    if (VX_SUCCESS == status)
    {
        tivx_obj_desc_user_data_object_t* config_desc;
        void * config_target_ptr;

        tivx_obj_desc_image_t *in_img_desc;
        void* in_img_target_ptr[2];

        tivx_obj_desc_tensor_t *out_tensor_desc;
        void *out_tensor_target_ptr;

        config_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_DL_PRE_PROC_CONFIG_IDX];
        config_target_ptr = tivxMemShared2TargetPtr(&config_desc->mem_ptr);
        tivxMemBufferMap(config_target_ptr, config_desc->mem_size, VX_MEMORY_TYPE_HOST,VX_READ_ONLY);

        in_img_desc  = (tivx_obj_desc_image_t *)obj_desc[TIVX_KERNEL_DL_PRE_PROC_INPUT_IMAGE_IDX];
        in_img_target_ptr[0]  = tivxMemShared2TargetPtr(&in_img_desc->mem_ptr[0]);
        tivxMemBufferMap(in_img_target_ptr[0], in_img_desc->mem_size[0], VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
        in_img_target_ptr[1]  = NULL;
        if(in_img_desc->mem_ptr[1].shared_ptr != 0)
        {
            in_img_target_ptr[1]  = tivxMemShared2TargetPtr(&in_img_desc->mem_ptr[1]);
            tivxMemBufferMap(in_img_target_ptr[1], in_img_desc->mem_size[1], VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
        }

        out_tensor_desc = (tivx_obj_desc_tensor_t *)obj_desc[TIVX_KERNEL_DL_PRE_PROC_OUTPUT_TENSOR_IDX];
        out_tensor_target_ptr = tivxMemShared2TargetPtr(&out_tensor_desc->mem_ptr);
        tivxMemBufferMap(out_tensor_target_ptr, out_tensor_desc->mem_size, VX_MEMORY_TYPE_HOST, VX_WRITE_ONLY);

        tivxDLPreProcParams *dlParams = (tivxDLPreProcParams *)config_target_ptr;

        vx_df_image image_format = in_img_desc->format;
        uint32_t tensor_format = dlParams->tensor_format;
        uint32_t channel_order = dlParams->channel_order;
        uint32_t tensor_data_type = out_tensor_desc->data_type;

        if(image_format == VX_DF_IMAGE_RGB)
        {
            if(channel_order == TIVX_DL_PRE_PROC_CHANNEL_ORDER_NHWC)
            {
                if(tensor_format == TIVX_DL_PRE_PROC_TENSOR_FORMAT_RGB)
                {
                    /* Case 1 */
                    /* Input is RGB, Output is RGB (NHWC) bit-depth unsigned 8-bit */
                    if(tensor_data_type == VX_TYPE_UINT8)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t pos_x, pos_y;

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            /* In this case output is simply a copy of input, so use the input stride_y */
                            uint8_t *pOut = (uint8_t *)out_tensor_target_ptr + (pos_y * in_stride_y);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                uint8_t R = pIn[offset + 0];
                                uint8_t G = pIn[offset + 1];
                                uint8_t B = pIn[offset + 2];

                                pOut[offset + 0] = (uint8_t)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[offset + 1] = (uint8_t)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[offset + 2] = (uint8_t)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                    if(tensor_data_type == VX_TYPE_INT16)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t pos_x, pos_y;

                        VX_PRINT(VX_ZONE_ERROR, "dim_x = %d\n", in_img_desc->imagepatch_addr[0].dim_x);
                        VX_PRINT(VX_ZONE_ERROR, "dim_y = %d\n", in_img_desc->imagepatch_addr[0].dim_y);
                        VX_PRINT(VX_ZONE_ERROR, "stride_y = %d\n", in_img_desc->imagepatch_addr[0].stride_y);
                        VX_PRINT(VX_ZONE_ERROR, "stride_x = %d\n", in_img_desc->imagepatch_addr[0].stride_x);
                        VX_PRINT(VX_ZONE_ERROR, "step_x = %d\n", in_img_desc->imagepatch_addr[0].step_x);
                        VX_PRINT(VX_ZONE_ERROR, "step_y = %d\n", in_img_desc->imagepatch_addr[0].step_y);

                        VX_PRINT(VX_ZONE_ERROR, "stride[0] = %d\n", out_tensor_desc->stride[0]);
                        VX_PRINT(VX_ZONE_ERROR, "stride[1] = %d\n", out_tensor_desc->stride[1]);
                        VX_PRINT(VX_ZONE_ERROR, "stride[2] = %d\n", out_tensor_desc->stride[2]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[0] = %d\n", out_tensor_desc->dimensions[0]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[1] = %d\n", out_tensor_desc->dimensions[1]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[2] = %d\n", out_tensor_desc->dimensions[2]);

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            /* In this case output is simply a copy of input, so use the input stride_y */
                            int16_t *pOut = (int16_t *)out_tensor_target_ptr + (pos_y * in_stride_y);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                int16_t R = pIn[offset + 0];
                                int16_t G = pIn[offset + 1];
                                int16_t B = pIn[offset + 2];

                                pOut[offset + 0] = (int16_t)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[offset + 1] = (int16_t)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[offset + 2] = (int16_t)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                    if(tensor_data_type == VX_TYPE_UINT16)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t pos_x, pos_y;

                        VX_PRINT(VX_ZONE_ERROR, "dim_x = %d\n", in_img_desc->imagepatch_addr[0].dim_x);
                        VX_PRINT(VX_ZONE_ERROR, "dim_y = %d\n", in_img_desc->imagepatch_addr[0].dim_y);
                        VX_PRINT(VX_ZONE_ERROR, "stride_y = %d\n", in_img_desc->imagepatch_addr[0].stride_y);
                        VX_PRINT(VX_ZONE_ERROR, "stride_x = %d\n", in_img_desc->imagepatch_addr[0].stride_x);
                        VX_PRINT(VX_ZONE_ERROR, "step_x = %d\n", in_img_desc->imagepatch_addr[0].step_x);
                        VX_PRINT(VX_ZONE_ERROR, "step_y = %d\n", in_img_desc->imagepatch_addr[0].step_y);

                        VX_PRINT(VX_ZONE_ERROR, "stride[0] = %d\n", out_tensor_desc->stride[0]);
                        VX_PRINT(VX_ZONE_ERROR, "stride[1] = %d\n", out_tensor_desc->stride[1]);
                        VX_PRINT(VX_ZONE_ERROR, "stride[2] = %d\n", out_tensor_desc->stride[2]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[0] = %d\n", out_tensor_desc->dimensions[0]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[1] = %d\n", out_tensor_desc->dimensions[1]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[2] = %d\n", out_tensor_desc->dimensions[2]);

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            /* In this case output is simply a copy of input, so use the input stride_y */
                            uint16_t *pOut = (uint16_t *)out_tensor_target_ptr + (pos_y * in_stride_y);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                uint16_t R = pIn[offset + 0];
                                uint16_t G = pIn[offset + 1];
                                uint16_t B = pIn[offset + 2];

                                pOut[offset + 0] = (uint16_t)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[offset + 1] = (uint16_t)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[offset + 2] = (uint16_t)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                    if(tensor_data_type == VX_TYPE_INT32)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t pos_x, pos_y;

                        VX_PRINT(VX_ZONE_ERROR, "dim_x = %d\n", in_img_desc->imagepatch_addr[0].dim_x);
                        VX_PRINT(VX_ZONE_ERROR, "dim_y = %d\n", in_img_desc->imagepatch_addr[0].dim_y);
                        VX_PRINT(VX_ZONE_ERROR, "stride_y = %d\n", in_img_desc->imagepatch_addr[0].stride_y);
                        VX_PRINT(VX_ZONE_ERROR, "stride_x = %d\n", in_img_desc->imagepatch_addr[0].stride_x);
                        VX_PRINT(VX_ZONE_ERROR, "step_x = %d\n", in_img_desc->imagepatch_addr[0].step_x);
                        VX_PRINT(VX_ZONE_ERROR, "step_y = %d\n", in_img_desc->imagepatch_addr[0].step_y);

                        VX_PRINT(VX_ZONE_ERROR, "stride[0] = %d\n", out_tensor_desc->stride[0]);
                        VX_PRINT(VX_ZONE_ERROR, "stride[1] = %d\n", out_tensor_desc->stride[1]);
                        VX_PRINT(VX_ZONE_ERROR, "stride[2] = %d\n", out_tensor_desc->stride[2]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[0] = %d\n", out_tensor_desc->dimensions[0]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[1] = %d\n", out_tensor_desc->dimensions[1]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[2] = %d\n", out_tensor_desc->dimensions[2]);

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            /* In this case output is simply a copy of input, so use the input stride_y */
                            int32_t *pOut = (int32_t *)out_tensor_target_ptr + (pos_y * in_stride_y);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                int32_t R = pIn[offset + 0];
                                int32_t G = pIn[offset + 1];
                                int32_t B = pIn[offset + 2];

                                pOut[offset + 0] = (int32_t)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[offset + 1] = (int32_t)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[offset + 2] = (int32_t)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                    if(tensor_data_type == VX_TYPE_UINT32)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t pos_x, pos_y;

                        VX_PRINT(VX_ZONE_ERROR, "dim_x = %d\n", in_img_desc->imagepatch_addr[0].dim_x);
                        VX_PRINT(VX_ZONE_ERROR, "dim_y = %d\n", in_img_desc->imagepatch_addr[0].dim_y);
                        VX_PRINT(VX_ZONE_ERROR, "stride_y = %d\n", in_img_desc->imagepatch_addr[0].stride_y);
                        VX_PRINT(VX_ZONE_ERROR, "stride_x = %d\n", in_img_desc->imagepatch_addr[0].stride_x);
                        VX_PRINT(VX_ZONE_ERROR, "step_x = %d\n", in_img_desc->imagepatch_addr[0].step_x);
                        VX_PRINT(VX_ZONE_ERROR, "step_y = %d\n", in_img_desc->imagepatch_addr[0].step_y);

                        VX_PRINT(VX_ZONE_ERROR, "stride[0] = %d\n", out_tensor_desc->stride[0]);
                        VX_PRINT(VX_ZONE_ERROR, "stride[1] = %d\n", out_tensor_desc->stride[1]);
                        VX_PRINT(VX_ZONE_ERROR, "stride[2] = %d\n", out_tensor_desc->stride[2]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[0] = %d\n", out_tensor_desc->dimensions[0]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[1] = %d\n", out_tensor_desc->dimensions[1]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[2] = %d\n", out_tensor_desc->dimensions[2]);

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            /* In this case output is simply a copy of input, so use the input stride_y */
                            uint32_t *pOut = (uint32_t *)out_tensor_target_ptr + (pos_y * in_stride_y);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                uint32_t R = pIn[offset + 0];
                                uint32_t G = pIn[offset + 1];
                                uint32_t B = pIn[offset + 2];

                                pOut[offset + 0] = (uint32_t)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[offset + 1] = (uint32_t)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[offset + 2] = (uint32_t)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                    if(tensor_data_type == VX_TYPE_FLOAT32)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t pos_x, pos_y;

                        VX_PRINT(VX_ZONE_ERROR, "dim_x = %d\n", in_img_desc->imagepatch_addr[0].dim_x);
                        VX_PRINT(VX_ZONE_ERROR, "dim_y = %d\n", in_img_desc->imagepatch_addr[0].dim_y);
                        VX_PRINT(VX_ZONE_ERROR, "stride_y = %d\n", in_img_desc->imagepatch_addr[0].stride_y);
                        VX_PRINT(VX_ZONE_ERROR, "stride_x = %d\n", in_img_desc->imagepatch_addr[0].stride_x);
                        VX_PRINT(VX_ZONE_ERROR, "step_x = %d\n", in_img_desc->imagepatch_addr[0].step_x);
                        VX_PRINT(VX_ZONE_ERROR, "step_y = %d\n", in_img_desc->imagepatch_addr[0].step_y);

                        VX_PRINT(VX_ZONE_ERROR, "stride[0] = %d\n", out_tensor_desc->stride[0]);
                        VX_PRINT(VX_ZONE_ERROR, "stride[1] = %d\n", out_tensor_desc->stride[1]);
                        VX_PRINT(VX_ZONE_ERROR, "stride[2] = %d\n", out_tensor_desc->stride[2]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[0] = %d\n", out_tensor_desc->dimensions[0]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[1] = %d\n", out_tensor_desc->dimensions[1]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[2] = %d\n", out_tensor_desc->dimensions[2]);

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            /* In this case output is simply a copy of input, so use the input stride_y */
                            float *pOut = (float *)out_tensor_target_ptr + (pos_y * in_stride_y);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                float R = pIn[offset + 0] * 1.0f;
                                float G = pIn[offset + 1] * 1.0f;
                                float B = pIn[offset + 2] * 1.0f;

                                pOut[offset + 0] = (float)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[offset + 1] = (float)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[offset + 2] = (float)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                }
                if(tensor_format == TIVX_DL_PRE_PROC_TENSOR_FORMAT_BGR)
                {
                    /* Case 2 */
                    /* Input is RGB, Output is BGR (NHWC) */
                    if(tensor_data_type == VX_TYPE_UINT8)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t pos_x, pos_y;

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            /* In this case output is simply a copy of input, so use the input stride_y */
                            uint8_t *pOut = (uint8_t *)out_tensor_target_ptr + (pos_y * in_stride_y);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                uint8_t R = pIn[offset + 0];
                                uint8_t G = pIn[offset + 1];
                                uint8_t B = pIn[offset + 2];

                                pOut[offset + 2] = (uint8_t)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[offset + 1] = (uint8_t)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[offset + 0] = (uint8_t)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                    if(tensor_data_type == VX_TYPE_INT16)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t pos_x, pos_y;

                        VX_PRINT(VX_ZONE_ERROR, "dim_x = %d\n", in_img_desc->imagepatch_addr[0].dim_x);
                        VX_PRINT(VX_ZONE_ERROR, "dim_y = %d\n", in_img_desc->imagepatch_addr[0].dim_y);
                        VX_PRINT(VX_ZONE_ERROR, "stride_y = %d\n", in_img_desc->imagepatch_addr[0].stride_y);
                        VX_PRINT(VX_ZONE_ERROR, "stride_x = %d\n", in_img_desc->imagepatch_addr[0].stride_x);
                        VX_PRINT(VX_ZONE_ERROR, "step_x = %d\n", in_img_desc->imagepatch_addr[0].step_x);
                        VX_PRINT(VX_ZONE_ERROR, "step_y = %d\n", in_img_desc->imagepatch_addr[0].step_y);

                        VX_PRINT(VX_ZONE_ERROR, "stride[0] = %d\n", out_tensor_desc->stride[0]);
                        VX_PRINT(VX_ZONE_ERROR, "stride[1] = %d\n", out_tensor_desc->stride[1]);
                        VX_PRINT(VX_ZONE_ERROR, "stride[2] = %d\n", out_tensor_desc->stride[2]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[0] = %d\n", out_tensor_desc->dimensions[0]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[1] = %d\n", out_tensor_desc->dimensions[1]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[2] = %d\n", out_tensor_desc->dimensions[2]);

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            /* In this case output is simply a copy of input, so use the input stride_y */
                            int16_t *pOut = (int16_t *)out_tensor_target_ptr + (pos_y * in_stride_y);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                int16_t R = pIn[offset + 0];
                                int16_t G = pIn[offset + 1];
                                int16_t B = pIn[offset + 2];

                                pOut[offset + 2] = (int16_t)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[offset + 1] = (int16_t)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[offset + 0] = (int16_t)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                    if(tensor_data_type == VX_TYPE_UINT16)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t pos_x, pos_y;

                        VX_PRINT(VX_ZONE_ERROR, "dim_x = %d\n", in_img_desc->imagepatch_addr[0].dim_x);
                        VX_PRINT(VX_ZONE_ERROR, "dim_y = %d\n", in_img_desc->imagepatch_addr[0].dim_y);
                        VX_PRINT(VX_ZONE_ERROR, "stride_y = %d\n", in_img_desc->imagepatch_addr[0].stride_y);
                        VX_PRINT(VX_ZONE_ERROR, "stride_x = %d\n", in_img_desc->imagepatch_addr[0].stride_x);
                        VX_PRINT(VX_ZONE_ERROR, "step_x = %d\n", in_img_desc->imagepatch_addr[0].step_x);
                        VX_PRINT(VX_ZONE_ERROR, "step_y = %d\n", in_img_desc->imagepatch_addr[0].step_y);

                        VX_PRINT(VX_ZONE_ERROR, "stride[0] = %d\n", out_tensor_desc->stride[0]);
                        VX_PRINT(VX_ZONE_ERROR, "stride[1] = %d\n", out_tensor_desc->stride[1]);
                        VX_PRINT(VX_ZONE_ERROR, "stride[2] = %d\n", out_tensor_desc->stride[2]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[0] = %d\n", out_tensor_desc->dimensions[0]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[1] = %d\n", out_tensor_desc->dimensions[1]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[2] = %d\n", out_tensor_desc->dimensions[2]);

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            /* In this case output is simply a copy of input, so use the input stride_y */
                            uint16_t *pOut = (uint16_t *)out_tensor_target_ptr + (pos_y * in_stride_y);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                uint16_t R = pIn[offset + 0];
                                uint16_t G = pIn[offset + 1];
                                uint16_t B = pIn[offset + 2];

                                pOut[offset + 2] = (uint16_t)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[offset + 1] = (uint16_t)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[offset + 0] = (uint16_t)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                    if(tensor_data_type == VX_TYPE_INT32)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t pos_x, pos_y;

                        VX_PRINT(VX_ZONE_ERROR, "dim_x = %d\n", in_img_desc->imagepatch_addr[0].dim_x);
                        VX_PRINT(VX_ZONE_ERROR, "dim_y = %d\n", in_img_desc->imagepatch_addr[0].dim_y);
                        VX_PRINT(VX_ZONE_ERROR, "stride_y = %d\n", in_img_desc->imagepatch_addr[0].stride_y);
                        VX_PRINT(VX_ZONE_ERROR, "stride_x = %d\n", in_img_desc->imagepatch_addr[0].stride_x);
                        VX_PRINT(VX_ZONE_ERROR, "step_x = %d\n", in_img_desc->imagepatch_addr[0].step_x);
                        VX_PRINT(VX_ZONE_ERROR, "step_y = %d\n", in_img_desc->imagepatch_addr[0].step_y);

                        VX_PRINT(VX_ZONE_ERROR, "stride[0] = %d\n", out_tensor_desc->stride[0]);
                        VX_PRINT(VX_ZONE_ERROR, "stride[1] = %d\n", out_tensor_desc->stride[1]);
                        VX_PRINT(VX_ZONE_ERROR, "stride[2] = %d\n", out_tensor_desc->stride[2]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[0] = %d\n", out_tensor_desc->dimensions[0]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[1] = %d\n", out_tensor_desc->dimensions[1]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[2] = %d\n", out_tensor_desc->dimensions[2]);

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            /* In this case output is simply a copy of input, so use the input stride_y */
                            int32_t *pOut = (int32_t *)out_tensor_target_ptr + (pos_y * in_stride_y);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                int32_t R = pIn[offset + 0];
                                int32_t G = pIn[offset + 1];
                                int32_t B = pIn[offset + 2];

                                pOut[offset + 2] = (int32_t)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[offset + 1] = (int32_t)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[offset + 0] = (int32_t)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                    if(tensor_data_type == VX_TYPE_UINT32)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t pos_x, pos_y;

                        VX_PRINT(VX_ZONE_ERROR, "dim_x = %d\n", in_img_desc->imagepatch_addr[0].dim_x);
                        VX_PRINT(VX_ZONE_ERROR, "dim_y = %d\n", in_img_desc->imagepatch_addr[0].dim_y);
                        VX_PRINT(VX_ZONE_ERROR, "stride_y = %d\n", in_img_desc->imagepatch_addr[0].stride_y);
                        VX_PRINT(VX_ZONE_ERROR, "stride_x = %d\n", in_img_desc->imagepatch_addr[0].stride_x);
                        VX_PRINT(VX_ZONE_ERROR, "step_x = %d\n", in_img_desc->imagepatch_addr[0].step_x);
                        VX_PRINT(VX_ZONE_ERROR, "step_y = %d\n", in_img_desc->imagepatch_addr[0].step_y);

                        VX_PRINT(VX_ZONE_ERROR, "stride[0] = %d\n", out_tensor_desc->stride[0]);
                        VX_PRINT(VX_ZONE_ERROR, "stride[1] = %d\n", out_tensor_desc->stride[1]);
                        VX_PRINT(VX_ZONE_ERROR, "stride[2] = %d\n", out_tensor_desc->stride[2]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[0] = %d\n", out_tensor_desc->dimensions[0]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[1] = %d\n", out_tensor_desc->dimensions[1]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[2] = %d\n", out_tensor_desc->dimensions[2]);

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            /* In this case output is simply a copy of input, so use the input stride_y */
                            uint32_t *pOut = (uint32_t *)out_tensor_target_ptr + (pos_y * in_stride_y);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                uint32_t R = pIn[offset + 0];
                                uint32_t G = pIn[offset + 1];
                                uint32_t B = pIn[offset + 2];

                                pOut[offset + 2] = (uint32_t)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[offset + 1] = (uint32_t)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[offset + 0] = (uint32_t)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                    if(tensor_data_type == VX_TYPE_FLOAT32)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t pos_x, pos_y;

                        VX_PRINT(VX_ZONE_ERROR, "dim_x = %d\n", in_img_desc->imagepatch_addr[0].dim_x);
                        VX_PRINT(VX_ZONE_ERROR, "dim_y = %d\n", in_img_desc->imagepatch_addr[0].dim_y);
                        VX_PRINT(VX_ZONE_ERROR, "stride_y = %d\n", in_img_desc->imagepatch_addr[0].stride_y);
                        VX_PRINT(VX_ZONE_ERROR, "stride_x = %d\n", in_img_desc->imagepatch_addr[0].stride_x);
                        VX_PRINT(VX_ZONE_ERROR, "step_x = %d\n", in_img_desc->imagepatch_addr[0].step_x);
                        VX_PRINT(VX_ZONE_ERROR, "step_y = %d\n", in_img_desc->imagepatch_addr[0].step_y);

                        VX_PRINT(VX_ZONE_ERROR, "stride[0] = %d\n", out_tensor_desc->stride[0]);
                        VX_PRINT(VX_ZONE_ERROR, "stride[1] = %d\n", out_tensor_desc->stride[1]);
                        VX_PRINT(VX_ZONE_ERROR, "stride[2] = %d\n", out_tensor_desc->stride[2]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[0] = %d\n", out_tensor_desc->dimensions[0]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[1] = %d\n", out_tensor_desc->dimensions[1]);
                        VX_PRINT(VX_ZONE_ERROR, "dimensions[2] = %d\n", out_tensor_desc->dimensions[2]);

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            /* In this case output is simply a copy of input, so use the input stride_y */
                            float *pOut = (float *)out_tensor_target_ptr + (pos_y * in_stride_y);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                float R = pIn[offset + 0] * 1.0f;
                                float G = pIn[offset + 1] * 1.0f;
                                float B = pIn[offset + 2] * 1.0f;

                                pOut[offset + 2] = (float)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[offset + 1] = (float)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[offset + 0] = (float)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                }
            }
            if(channel_order == TIVX_DL_PRE_PROC_CHANNEL_ORDER_NCHW)
            {
                if(tensor_format == TIVX_DL_PRE_PROC_TENSOR_FORMAT_RGB)
                {
                    /* Case 3 */
                    /* Input is RGB Output is RGB (NCHW) */
                    if(tensor_data_type == VX_TYPE_UINT8)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t ch_offset = out_tensor_desc->dimensions[0] * out_tensor_desc->dimensions[1];
                        uint32_t pos_x, pos_y;

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            uint8_t *pOut = (uint8_t *)out_tensor_target_ptr + (pos_y * out_tensor_desc->dimensions[0]);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                uint8_t R = pIn[offset + 0];
                                uint8_t G = pIn[offset + 1];
                                uint8_t B = pIn[offset + 2];

                                pOut[(ch_offset * 0) + pos_x] = (uint8_t)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[(ch_offset * 1) + pos_x] = (uint8_t)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[(ch_offset * 2) + pos_x] = (uint8_t)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                    if(tensor_data_type == VX_TYPE_INT16)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t ch_offset = out_tensor_desc->dimensions[0] * out_tensor_desc->dimensions[1];
                        uint32_t pos_x, pos_y;

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            int16_t *pOut = (int16_t *)out_tensor_target_ptr + (pos_y * out_tensor_desc->dimensions[0]);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                int16_t R = pIn[offset + 0];
                                int16_t G = pIn[offset + 1];
                                int16_t B = pIn[offset + 2];

                                pOut[(ch_offset * 0) + pos_x] = (int16_t)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[(ch_offset * 1) + pos_x] = (int16_t)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[(ch_offset * 2) + pos_x] = (int16_t)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                    if(tensor_data_type == VX_TYPE_UINT16)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t ch_offset = out_tensor_desc->dimensions[0] * out_tensor_desc->dimensions[1];
                        uint32_t pos_x, pos_y;

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            uint16_t *pOut = (uint16_t *)out_tensor_target_ptr + (pos_y * out_tensor_desc->dimensions[0]);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                uint16_t R = pIn[offset + 0];
                                uint16_t G = pIn[offset + 1];
                                uint16_t B = pIn[offset + 2];

                                pOut[(ch_offset * 0) + pos_x] = (uint16_t)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[(ch_offset * 1) + pos_x] = (uint16_t)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[(ch_offset * 2) + pos_x] = (uint16_t)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                    if(tensor_data_type == VX_TYPE_INT32)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t ch_offset = out_tensor_desc->dimensions[0] * out_tensor_desc->dimensions[1];
                        uint32_t pos_x, pos_y;

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            int32_t *pOut = (int32_t *)out_tensor_target_ptr + (pos_y * out_tensor_desc->dimensions[0]);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                int32_t R = pIn[offset + 0];
                                int32_t G = pIn[offset + 1];
                                int32_t B = pIn[offset + 2];

                                pOut[(ch_offset * 0) + pos_x] = (int32_t)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[(ch_offset * 1) + pos_x] = (int32_t)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[(ch_offset * 2) + pos_x] = (int32_t)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                    if(tensor_data_type == VX_TYPE_UINT32)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t ch_offset = out_tensor_desc->dimensions[0] * out_tensor_desc->dimensions[1];
                        uint32_t pos_x, pos_y;

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            uint32_t *pOut = (uint32_t *)out_tensor_target_ptr + (pos_y * out_tensor_desc->dimensions[0]);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                uint32_t R = pIn[offset + 0];
                                uint32_t G = pIn[offset + 1];
                                uint32_t B = pIn[offset + 2];

                                pOut[(ch_offset * 0) + pos_x] = (uint32_t)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[(ch_offset * 1) + pos_x] = (uint32_t)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[(ch_offset * 2) + pos_x] = (uint32_t)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                    if(tensor_data_type == VX_TYPE_FLOAT32)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t ch_offset = out_tensor_desc->dimensions[0] * out_tensor_desc->dimensions[1];
                        uint32_t pos_x, pos_y;

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            float *pOut = (float *)out_tensor_target_ptr + (pos_y * out_tensor_desc->dimensions[0]);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                float R = pIn[offset + 0];
                                float G = pIn[offset + 1];
                                float B = pIn[offset + 2];

                                pOut[(ch_offset * 0) + pos_x] = (float)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[(ch_offset * 1) + pos_x] = (float)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[(ch_offset * 2) + pos_x] = (float)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                }
                if(tensor_format == TIVX_DL_PRE_PROC_TENSOR_FORMAT_BGR)
                {
                    /* Case 4 */
                    /* Input is RGB Output is BGR (NCHW) */
                    if(tensor_data_type == VX_TYPE_UINT8)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t ch_offset = out_tensor_desc->dimensions[0] * out_tensor_desc->dimensions[1];
                        uint32_t pos_x, pos_y;

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            uint8_t *pOut = (uint8_t *)out_tensor_target_ptr + (pos_y * out_tensor_desc->dimensions[0]);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                uint8_t R = pIn[offset + 0];
                                uint8_t G = pIn[offset + 1];
                                uint8_t B = pIn[offset + 2];

                                pOut[(ch_offset * 2) + pos_x] = (uint8_t)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[(ch_offset * 1) + pos_x] = (uint8_t)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[(ch_offset * 0) + pos_x] = (uint8_t)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                    if(tensor_data_type == VX_TYPE_INT16)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t ch_offset = out_tensor_desc->dimensions[0] * out_tensor_desc->dimensions[1];
                        uint32_t pos_x, pos_y;

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            int16_t *pOut = (int16_t *)out_tensor_target_ptr + (pos_y * out_tensor_desc->dimensions[0]);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                int16_t R = pIn[offset + 0];
                                int16_t G = pIn[offset + 1];
                                int16_t B = pIn[offset + 2];

                                pOut[(ch_offset * 2) + pos_x] = (int16_t)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[(ch_offset * 1) + pos_x] = (int16_t)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[(ch_offset * 0) + pos_x] = (int16_t)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                    if(tensor_data_type == VX_TYPE_UINT16)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t ch_offset = out_tensor_desc->dimensions[0] * out_tensor_desc->dimensions[1];
                        uint32_t pos_x, pos_y;

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            uint16_t *pOut = (uint16_t *)out_tensor_target_ptr + (pos_y * out_tensor_desc->dimensions[0]);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                uint16_t R = pIn[offset + 0];
                                uint16_t G = pIn[offset + 1];
                                uint16_t B = pIn[offset + 2];

                                pOut[(ch_offset * 2) + pos_x] = (uint16_t)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[(ch_offset * 1) + pos_x] = (uint16_t)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[(ch_offset * 0) + pos_x] = (uint16_t)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                    if(tensor_data_type == VX_TYPE_INT32)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t ch_offset = out_tensor_desc->dimensions[0] * out_tensor_desc->dimensions[1];
                        uint32_t pos_x, pos_y;

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            int32_t *pOut = (int32_t *)out_tensor_target_ptr + (pos_y * out_tensor_desc->dimensions[0]);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                int32_t R = pIn[offset + 0];
                                int32_t G = pIn[offset + 1];
                                int32_t B = pIn[offset + 2];

                                pOut[(ch_offset * 2) + pos_x] = (int32_t)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[(ch_offset * 1) + pos_x] = (int32_t)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[(ch_offset * 0) + pos_x] = (int32_t)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                    if(tensor_data_type == VX_TYPE_UINT32)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t ch_offset = out_tensor_desc->dimensions[0] * out_tensor_desc->dimensions[1];
                        uint32_t pos_x, pos_y;

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            uint32_t *pOut = (uint32_t *)out_tensor_target_ptr + (pos_y * out_tensor_desc->dimensions[0]);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                uint32_t R = pIn[offset + 0];
                                uint32_t G = pIn[offset + 1];
                                uint32_t B = pIn[offset + 2];

                                pOut[(ch_offset * 2) + pos_x] = (uint32_t)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[(ch_offset * 1) + pos_x] = (uint32_t)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[(ch_offset * 0) + pos_x] = (uint32_t)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                    if(tensor_data_type == VX_TYPE_FLOAT32)
                    {
                        uint32_t in_stride_y = in_img_desc->imagepatch_addr[0].stride_y;
                        uint32_t input_width = in_img_desc->imagepatch_addr[0].dim_x;
                        uint32_t input_height = in_img_desc->imagepatch_addr[0].dim_y;
                        uint32_t ch_offset = out_tensor_desc->dimensions[0] * out_tensor_desc->dimensions[1];
                        uint32_t pos_x, pos_y;

                        for(pos_y = 0; pos_y < input_height; pos_y++)
                        {
                            uint8_t *pIn  = (uint8_t *)in_img_target_ptr[0] + (pos_y * in_stride_y);
                            float *pOut = (float *)out_tensor_target_ptr + (pos_y * out_tensor_desc->dimensions[0]);
                            uint32_t offset;

                            offset = 0;
                            for(pos_x = 0; pos_x < input_width; pos_x++)
                            {
                                float R = pIn[offset + 0];
                                float G = pIn[offset + 1];
                                float B = pIn[offset + 2];

                                pOut[(ch_offset * 2) + pos_x] = (float)((R - dlParams->mean[0]) * dlParams->scale[0]);
                                pOut[(ch_offset * 1) + pos_x] = (float)((G - dlParams->mean[1]) * dlParams->scale[1]);
                                pOut[(ch_offset * 0) + pos_x] = (float)((B - dlParams->mean[2]) * dlParams->scale[2]);

                                offset += 3;
                            }
                        }
                    }
                }
            }
        }
        else if(image_format == VX_DF_IMAGE_NV12)
        {
            if(channel_order == TIVX_DL_PRE_PROC_CHANNEL_ORDER_NHWC)
            {
                if(tensor_format == TIVX_DL_PRE_PROC_TENSOR_FORMAT_RGB)
                {
                    /* Case 1 */
                    /* Input is NV12, Output is RGB (NHWC) */
                }
                if(tensor_format == TIVX_DL_PRE_PROC_TENSOR_FORMAT_BGR)
                {
                    /* Case 2 */
                    /* Input is NV12,  Output is BGR (NHWC) */
                }
            }
            if(channel_order == TIVX_DL_PRE_PROC_CHANNEL_ORDER_NCHW)
            {
                if(tensor_format == TIVX_DL_PRE_PROC_TENSOR_FORMAT_RGB)
                {
                    /* Case 3 */
                    /* Input is NV12, Output is RGB (NCHW) */
                }
                if(tensor_format == TIVX_DL_PRE_PROC_TENSOR_FORMAT_BGR)
                {
                    /* Case 4 */
                    /* Input is NV12, Output is BGR (NCHW) */
                }
            }
        }


        /* Write DL pre proc operation here */
        tivxMemBufferUnmap(out_tensor_target_ptr, out_tensor_desc->mem_size, VX_MEMORY_TYPE_HOST, VX_WRITE_ONLY);
        tivxMemBufferUnmap(in_img_target_ptr[0], in_img_desc->mem_size[0], VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
        if (in_img_target_ptr[1] != NULL)
        {
            tivxMemBufferUnmap(in_img_target_ptr[1], in_img_desc->mem_size[1], VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
        }
        tivxMemBufferUnmap(config_target_ptr, config_desc->mem_size, VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
    }

    return (status);
}

void tivxAddTargetKernelDLPreProc()
{
    char target_name[TIVX_TARGET_MAX_NAME];
    vx_enum self_cpu;

    self_cpu = tivxGetSelfCpuId();

    if ((self_cpu == TIVX_CPU_ID_DSP1) || (self_cpu == TIVX_CPU_ID_DSP2))
    {
        if (self_cpu == TIVX_CPU_ID_DSP1)
        {
            strncpy(target_name, TIVX_TARGET_DSP1, TIVX_TARGET_MAX_NAME);
        }
        else if (self_cpu == TIVX_CPU_ID_DSP2)
        {
            strncpy(target_name, TIVX_TARGET_DSP2, TIVX_TARGET_MAX_NAME);
        }

        vx_dl_pre_proc_target_kernel = tivxAddTargetKernelByName
                                        (
                                            TIVX_KERNEL_DL_PRE_PROC_NAME,
                                            target_name,
                                            tivxKernelDLPreProcProcess,
                                            tivxKernelDLPreProcCreate,
                                            tivxKernelDLPreProcDelete,
                                            NULL,
                                            NULL
                                        );
    }
}

void tivxRemoveTargetKernelDLPreProc()
{
    vx_status status = VX_SUCCESS;

    status = tivxRemoveTargetKernel(vx_dl_pre_proc_target_kernel);
    if (status == VX_SUCCESS)
    {
        vx_dl_pre_proc_target_kernel = NULL;
    }
}
