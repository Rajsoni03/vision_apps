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
#include "tivx_img_mosaic_host.h"

static vx_status VX_CALLBACK tivxAddKernelImgMosaicValidate(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num,
            vx_meta_format metas[]);

static vx_status VX_CALLBACK tivxAddKernelImgMosaicValidate(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num,
            vx_meta_format metas[])
{
    vx_status status = VX_SUCCESS;
    vx_user_data_object configuration = NULL;
    vx_char configuration_name[VX_MAX_REFERENCE_NAME];
    vx_size configuration_size;

    vx_image output = NULL;
    vx_uint32 width, height;
    vx_df_image output_fmt;

    if (   (NULL == parameters[TIVX_IMG_MOSAIC_HOST_CONFIG_IDX])
        || (NULL == parameters[TIVX_IMG_MOSAIC_HOST_OUTPUT_IMAGE_IDX])
        || (NULL == parameters[TIVX_IMG_MOSAIC_INPUT_START_IDX])
    )
    {
        status = VX_ERROR_INVALID_PARAMETERS;
        VX_PRINT(VX_ZONE_ERROR, "One or more REQUIRED parameters are set to NULL\n");
    }


    if (VX_SUCCESS == status)
    {
        configuration = (vx_user_data_object)parameters[TIVX_IMG_MOSAIC_HOST_CONFIG_IDX];
        output = (vx_image)parameters[TIVX_IMG_MOSAIC_HOST_OUTPUT_IMAGE_IDX];
    }

    /* PARAMETER ATTRIBUTE FETCH */

    if (VX_SUCCESS == status)
    {
        tivxCheckStatus(&status, vxQueryUserDataObject(configuration, VX_USER_DATA_OBJECT_NAME, &configuration_name, sizeof(configuration_name)));
        tivxCheckStatus(&status, vxQueryUserDataObject(configuration, VX_USER_DATA_OBJECT_SIZE, &configuration_size, sizeof(configuration_size)));

        tivxCheckStatus(&status, vxQueryImage(output, VX_IMAGE_WIDTH, &width, sizeof(width)));
        tivxCheckStatus(&status, vxQueryImage(output, VX_IMAGE_HEIGHT, &height, sizeof(height)));
        tivxCheckStatus(&status, vxQueryImage(output, VX_IMAGE_FORMAT, &output_fmt, sizeof(output_fmt)));
    }

    if (VX_SUCCESS == status)
    {
        if (configuration_size != sizeof(tivxImgMosaicParams))
        {
            status = VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'configuration' should be a user_data_object of type:\n tivxImgMosaicParams \n");
        }
    }

    if (VX_SUCCESS == status)
    {
        if ( !( (VX_DF_IMAGE_NV12 == output_fmt) ||
                (VX_DF_IMAGE_U8   == output_fmt) ||
                (VX_DF_IMAGE_U16  == output_fmt) ) )
        {
            status = VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'output' should be an image of type:\n VX_DF_IMAGE_NV12 or \n VX_DF_IMAGE_U8 or \n VX_DF_IMAGE_U16\n");
        }
    }

    return status;
}

vx_kernel tivxAddKernelImgMosaic(vx_context context, vx_int32 num_inputs)
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
        /* Number of parameters are config + output_image + coordinate list + input list */
        uint32_t num_params = 2 + num_inputs;
        kernel = vxAddUserKernel(
                    context,
                    TIVX_KERNEL_IMG_MOSAIC_NAME,
                    kernel_id,
                    NULL,
                    num_params,
                    tivxAddKernelImgMosaicValidate,
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
                        VX_OUTPUT,
                        VX_TYPE_IMAGE,
                        VX_PARAMETER_STATE_REQUIRED);
       index++;
    }
    for(i = 0; i < num_inputs; i++)
    {
        if (status == VX_SUCCESS)
        {
              status = vxAddParameterToKernel(kernel,
                          index,
                          VX_INPUT,
                          VX_TYPE_OBJECT_ARRAY,
                          VX_PARAMETER_STATE_REQUIRED);
              index++;
        }
    }
    if (status == VX_SUCCESS)
    {
        /* add supported target's */
        tivxAddKernelTarget(kernel, TIVX_TARGET_DSP1);
        tivxAddKernelTarget(kernel, TIVX_TARGET_DSP2);
        tivxAddKernelTarget(kernel, TIVX_TARGET_VPAC_MSC1);
        tivxAddKernelTarget(kernel, TIVX_TARGET_VPAC_MSC2);
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

void tivxImgMosaicParamsSetDefaults(tivxImgMosaicParams *prms)
{
    memset(prms, 0, sizeof(tivxImgMosaicParams));

    prms->enable_overlay    = 0;
    prms->num_msc_instances = 2;
    prms->msc_instance      = 0;
}
