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

#include "TI/tivx.h"
#include <TI/j7.h>
#include <TI/tivx_img_proc.h>
#include "TI/tivx_target_kernel.h"
#include "tivx_kernels_host_utils.h"
#include "tivx_dl_draw_box_host.h"

static vx_status VX_CALLBACK tivxAddKernelDLDrawBoxValidate(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num,
            vx_meta_format metas[]);

static vx_status VX_CALLBACK tivxAddKernelDLDrawBoxValidate(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num,
            vx_meta_format metas[])
{
    vx_status status = VX_SUCCESS;
    vx_user_data_object config = NULL;
    vx_char config_name[VX_MAX_REFERENCE_NAME];
    vx_size config_size;

    vx_image  image_input = NULL;
    vx_uint32 width, height;
    vx_df_image image_input_fmt;

    vx_tensor tensor_input = NULL;
    vx_size out_tensor_dims;

    if (   (NULL == parameters[TIVX_DL_DRAW_BOX_CONFIG_IDX])
        || (NULL == parameters[TIVX_DL_DRAW_BOX_INPUT_IMAGE_IDX])
        || (NULL == parameters[TIVX_DL_DRAW_BOX_INPUT_TENSOR_IDX])
        || (NULL == parameters[TIVX_DL_DRAW_BOX_OUTPUT_START_IDX])
    )
    {
        status = VX_ERROR_INVALID_PARAMETERS;
        VX_PRINT(VX_ZONE_ERROR, "One or more REQUIRED parameters are set to NULL\n");
    }


    if (VX_SUCCESS == status)
    {
        config = (vx_user_data_object)parameters[TIVX_DL_DRAW_BOX_CONFIG_IDX];
        image_input = (vx_image)parameters[TIVX_DL_DRAW_BOX_INPUT_IMAGE_IDX];
        tensor_input = (vx_tensor)parameters[TIVX_DL_DRAW_BOX_INPUT_TENSOR_IDX];
    }

    /* PARAMETER ATTRIBUTE FETCH */

    if (VX_SUCCESS == status)
    {
        tivxCheckStatus(&status, vxQueryUserDataObject(config, VX_USER_DATA_OBJECT_NAME, &config_name, sizeof(config_name)));
        tivxCheckStatus(&status, vxQueryUserDataObject(config, VX_USER_DATA_OBJECT_SIZE, &config_size, sizeof(config_size)));

        tivxCheckStatus(&status, vxQueryImage(image_input, VX_IMAGE_WIDTH, &width, sizeof(width)));
        tivxCheckStatus(&status, vxQueryImage(image_input, VX_IMAGE_HEIGHT, &height, sizeof(height)));
        tivxCheckStatus(&status, vxQueryImage(image_input, VX_IMAGE_FORMAT, &image_input_fmt, sizeof(image_input_fmt)));

        tivxCheckStatus(&status, vxQueryTensor(tensor_input, VX_TENSOR_NUMBER_OF_DIMS, &out_tensor_dims, sizeof(out_tensor_dims)));
    }

    if (VX_SUCCESS == status)
    {
        if (config_size != sizeof(tivxDLDrawBoxParams))
        {
            status = VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'config' should be a user_data_object of type:\n tivxDLDrawBoxParams \n");
        }
    }

    if (VX_SUCCESS == status)
    {
        if ( !( (VX_DF_IMAGE_NV12 == image_input_fmt) ||
                (VX_DF_IMAGE_U8   == image_input_fmt) ||
                (VX_DF_IMAGE_S16  == image_input_fmt) ||
                (VX_DF_IMAGE_U16  == image_input_fmt) ) )
        {
            status = VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'output' should be an image of type:\n VX_DF_IMAGE_NV12 or \n VX_DF_IMAGE_U8 or \n VX_DF_IMAGE_S16 or \n VX_DF_IMAGE_U16\n");
        }
    }

    return status;
}

vx_kernel tivxAddKernelDLDrawBox(vx_context context, vx_int32 num_outputs)
{
    vx_kernel kernel;
    vx_status status;
    vx_uint32 index;
    vx_enum kernel_id;
    vx_int32 i;

    status = vxAllocateUserKernelId(context, &kernel_id);
    if(status != VX_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Unable to allocate user kernel ID\n");
    }

    if (status == VX_SUCCESS)
    {
        /* Number of parameters are base params + output list */
        uint32_t num_params = TIVX_DL_DRAW_BOX_BASE_PARAMS + num_outputs;
        kernel = vxAddUserKernel(
                    context,
                    TIVX_KERNEL_DL_DRAW_BOX_NAME,
                    kernel_id,
                    NULL,
                    num_params,
                    tivxAddKernelDLDrawBoxValidate,
                    NULL,
                    NULL);

        status = vxGetStatus((vx_reference)kernel);
    }

    index = 0;
    if (status == VX_SUCCESS)
    {
       status = vxAddParameterToKernel(kernel,
                        index,
                        VX_INPUT,
                        VX_TYPE_USER_DATA_OBJECT,
                        VX_PARAMETER_STATE_REQUIRED);
       index++;
    }
    if (status == VX_SUCCESS)
    {
       status = vxAddParameterToKernel(kernel,
                        index,
                        VX_INPUT,
                        VX_TYPE_IMAGE,
                        VX_PARAMETER_STATE_REQUIRED);
       index++;
    }
    if (status == VX_SUCCESS)
    {
       status = vxAddParameterToKernel(kernel,
                        index,
                        VX_INPUT,
                        VX_TYPE_TENSOR,
                        VX_PARAMETER_STATE_REQUIRED);
       index++;
    }
    for(i = 0; i < num_outputs; i++)
    {
        if (status == VX_SUCCESS)
        {
              status = vxAddParameterToKernel(kernel,
                          index,
                          VX_OUTPUT,
                          VX_TYPE_IMAGE,
                          VX_PARAMETER_STATE_REQUIRED);
              index++;
        }
    }
    if (status == VX_SUCCESS)
    {
        /* add supported target's */
        tivxAddKernelTarget(kernel, TIVX_TARGET_DSP1);
        tivxAddKernelTarget(kernel, TIVX_TARGET_DSP2);
    }
    if (status == VX_SUCCESS)
    {
        status = vxFinalizeKernel(kernel);
    }
    if (status != VX_SUCCESS)
    {
        vxReleaseKernel(&kernel);
        kernel = NULL;
    }

    return kernel;
}
