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

#include "TI/tivx.h"
#include "TI/tivx_park_assist.h"
#include "tivx_park_assist_kernels_priv.h"
#include "tivx_kernel_dof_to_tracks.h"
#include "TI/tivx_target_kernel.h"

static vx_kernel vx_dof_to_tracks_kernel = NULL;

static vx_status VX_CALLBACK tivxAddKernelDofToTracksValidate(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num,
            vx_meta_format metas[]);
static vx_status VX_CALLBACK tivxAddKernelDofToTracksInitialize(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num_params);
vx_status tivxAddKernelDofToTracks(vx_context context);
vx_status tivxRemoveKernelDofToTracks(vx_context context);

static vx_status VX_CALLBACK tivxAddKernelDofToTracksValidate(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num,
            vx_meta_format metas[])
{
    vx_status status = VX_SUCCESS;

    vx_user_data_object configuration = NULL;
    vx_char configuration_name[VX_MAX_REFERENCE_NAME];
    vx_size configuration_size;

    vx_image input_dof_field = NULL;
    vx_df_image input_dof_field_fmt;
    vx_uint32 input_dof_field_w, input_dof_field_h;

    vx_lut input_d2u_lut = NULL;
    vx_enum input_d2u_lut_type;

    vx_array output_tracks = NULL;
    vx_size output_tracks_item_size;

    if ( (num != TIVX_KERNEL_DOF_TO_TRACKS_MAX_PARAMS)
        || (NULL == parameters[TIVX_KERNEL_DOF_TO_TRACKS_CONFIGURATION_IDX])
        || (NULL == parameters[TIVX_KERNEL_DOF_TO_TRACKS_INPUT_DOF_FIELD_IDX])
        || (NULL == parameters[TIVX_KERNEL_DOF_TO_TRACKS_OUTPUT_TRACKS_IDX])
    )
    {
        status = VX_ERROR_INVALID_PARAMETERS;
        VX_PRINT(VX_ZONE_ERROR, "One or more REQUIRED parameters are set to NULL\n");
    }

    if (VX_SUCCESS == status)
    {
        configuration = (vx_user_data_object)parameters[TIVX_KERNEL_DOF_TO_TRACKS_CONFIGURATION_IDX];
        input_dof_field = (vx_image)parameters[TIVX_KERNEL_DOF_TO_TRACKS_INPUT_DOF_FIELD_IDX];
        input_d2u_lut = (vx_lut)parameters[TIVX_KERNEL_DOF_TO_TRACKS_INPUT_D2U_LUT_IDX];
        output_tracks = (vx_array)parameters[TIVX_KERNEL_DOF_TO_TRACKS_OUTPUT_TRACKS_IDX];
    }


    /* PARAMETER ATTRIBUTE FETCH */

    if (VX_SUCCESS == status)
    {
        tivxCheckStatus(&status, vxQueryUserDataObject(configuration, VX_USER_DATA_OBJECT_NAME, &configuration_name, sizeof(configuration_name)));
        tivxCheckStatus(&status, vxQueryUserDataObject(configuration, VX_USER_DATA_OBJECT_SIZE, &configuration_size, sizeof(configuration_size)));

        tivxCheckStatus(&status, vxQueryImage(input_dof_field, VX_IMAGE_FORMAT, &input_dof_field_fmt, sizeof(input_dof_field_fmt)));
        tivxCheckStatus(&status, vxQueryImage(input_dof_field, VX_IMAGE_WIDTH, &input_dof_field_w, sizeof(input_dof_field_w)));
        tivxCheckStatus(&status, vxQueryImage(input_dof_field, VX_IMAGE_HEIGHT, &input_dof_field_h, sizeof(input_dof_field_h)));

        if (NULL != input_d2u_lut)
        {
            tivxCheckStatus(&status, vxQueryLUT(input_d2u_lut, VX_LUT_TYPE, &input_d2u_lut_type, sizeof(input_d2u_lut_type)));
        }

        tivxCheckStatus(&status, vxQueryArray(output_tracks, VX_ARRAY_ITEMSIZE, &output_tracks_item_size, sizeof(output_tracks_item_size)));
    }

    /* PARAMETER CHECKING */

    if (VX_SUCCESS == status)
    {
        if ((configuration_size != sizeof(tivx_dof_to_tracks_params_t)) ||
            (strncmp(configuration_name, "tivx_dof_to_tracks_params_t", sizeof(configuration_name)) != 0))
        {
            status = VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'configuration' should be a user_data_object of type:\n tivx_dof_to_tracks_params_t \n");
        }

        if (VX_DF_IMAGE_U32 != input_dof_field_fmt)
        {
            status = VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'input_dof_field' should be an image of type:\n VX_DF_IMAGE_U32 \n");
        }

        if (NULL != input_d2u_lut)
        {
            if (VX_TYPE_FLOAT32 != input_d2u_lut_type)
            {
                status = VX_ERROR_INVALID_PARAMETERS;
                VX_PRINT(VX_ZONE_ERROR, "'input_d2u_lut' should be a lut of type:\n VX_TYPE_FLOAT32 \n");
            }
        }

        if ( output_tracks_item_size != sizeof(tivx_triangulation_track_t))
        {
            status = VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'output_tracks' should be an array of type:\n tivx_triangulation_track_t \n");
        }
    }


    /* CUSTOM PARAMETER CHECKING */

    /* < DEVELOPER_TODO: (Optional) Add any custom parameter type or range checking not */
    /*                   covered by the code-generation script.) > */

    return status;
}

static vx_status VX_CALLBACK tivxAddKernelDofToTracksInitialize(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num_params)
{
    vx_status status = VX_SUCCESS;
    tivxKernelValidRectParams prms;

    if ( (num_params != TIVX_KERNEL_DOF_TO_TRACKS_MAX_PARAMS)
        || (NULL == parameters[TIVX_KERNEL_DOF_TO_TRACKS_CONFIGURATION_IDX])
        || (NULL == parameters[TIVX_KERNEL_DOF_TO_TRACKS_INPUT_DOF_FIELD_IDX])
        || (NULL == parameters[TIVX_KERNEL_DOF_TO_TRACKS_OUTPUT_TRACKS_IDX])
    )
    {
        status = VX_ERROR_INVALID_PARAMETERS;
        VX_PRINT(VX_ZONE_ERROR, "One or more REQUIRED parameters are set to NULL\n");
    }
    if (VX_SUCCESS == status)
    {
        tivxKernelValidRectParams_init(&prms);

        prms.in_img[0U] = (vx_image)parameters[TIVX_KERNEL_DOF_TO_TRACKS_INPUT_DOF_FIELD_IDX];

        prms.num_input_images = 1;
        prms.num_output_images = 0;

        status = tivxKernelConfigValidRect(&prms);
    }

    return status;
}

vx_status tivxAddKernelDofToTracks(vx_context context)
{
    vx_kernel kernel;
    vx_status status;
    uint32_t index;
    vx_enum kernel_id;

    status = vxAllocateUserKernelId(context, &kernel_id);
    if(status != VX_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Unable to allocate user kernel ID\n");
    }

    if (status == VX_SUCCESS)
    {
        kernel = vxAddUserKernel(
                    context,
                    TIVX_KERNEL_DOF_TO_TRACKS_NAME,
                    kernel_id,
                    NULL,
                    TIVX_KERNEL_DOF_TO_TRACKS_MAX_PARAMS,
                    tivxAddKernelDofToTracksValidate,
                    tivxAddKernelDofToTracksInitialize,
                    NULL);

        status = vxGetStatus((vx_reference)kernel);
    }
    if (status == VX_SUCCESS)
    {
        index = 0;

        {
            status = vxAddParameterToKernel(kernel,
                        index,
                        VX_INPUT,
                        VX_TYPE_USER_DATA_OBJECT,
                        VX_PARAMETER_STATE_REQUIRED
            );
            index++;
        }
        if (status == VX_SUCCESS)
        {
            status = vxAddParameterToKernel(kernel,
                        index,
                        VX_INPUT,
                        VX_TYPE_IMAGE,
                        VX_PARAMETER_STATE_REQUIRED
            );
            index++;
        }
        if (status == VX_SUCCESS)
        {
            status = vxAddParameterToKernel(kernel,
                        index,
                        VX_INPUT,
                        VX_TYPE_LUT,
                        VX_PARAMETER_STATE_OPTIONAL
            );
            index++;
        }
        if (status == VX_SUCCESS)
        {
            status = vxAddParameterToKernel(kernel,
                        index,
                        VX_OUTPUT,
                        VX_TYPE_ARRAY,
                        VX_PARAMETER_STATE_REQUIRED
            );
            index++;
        }
        if (status == VX_SUCCESS)
        {
            /* add supported target's */
            tivxAddKernelTarget(kernel, TIVX_TARGET_DSP1);
            tivxAddKernelTarget(kernel, TIVX_TARGET_DSP2);
            tivxAddKernelTarget(kernel, TIVX_TARGET_A72_0);
            tivxAddKernelTarget(kernel, TIVX_TARGET_A72_1);
            tivxAddKernelTarget(kernel, TIVX_TARGET_A72_2);
            tivxAddKernelTarget(kernel, TIVX_TARGET_A72_3);
            tivxAddKernelTarget(kernel, TIVX_TARGET_DSP_C7_1);
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
    }
    else
    {
        kernel = NULL;
    }
    vx_dof_to_tracks_kernel = kernel;

    return status;
}

vx_status tivxRemoveKernelDofToTracks(vx_context context)
{
    vx_status status;
    vx_kernel kernel = vx_dof_to_tracks_kernel;

    status = vxRemoveKernel(kernel);
    vx_dof_to_tracks_kernel = NULL;

    return status;
}


