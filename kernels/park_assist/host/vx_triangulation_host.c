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
#include "tivx_kernel_triangulation.h"
#include "TI/tivx_target_kernel.h"

static vx_kernel vx_triangulation_kernel = NULL;

static vx_status VX_CALLBACK tivxAddKernelTriangulationValidate(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num,
            vx_meta_format metas[]);
static vx_status VX_CALLBACK tivxAddKernelTriangulationInitialize(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num_params);
vx_status tivxAddKernelTriangulation(vx_context context);
vx_status tivxRemoveKernelTriangulation(vx_context context);

static vx_status VX_CALLBACK tivxAddKernelTriangulationValidate(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num,
            vx_meta_format metas[])
{
    vx_status status = VX_SUCCESS;

    vx_user_data_object configuration = NULL;
    vx_char configuration_name[VX_MAX_REFERENCE_NAME];
    vx_size configuration_size;

    vx_array input_track = NULL;
    vx_size input_track_item_size;

    vx_user_data_object input_pose = NULL;
    vx_char input_pose_name[VX_MAX_REFERENCE_NAME];
    vx_size input_pose_size;

    vx_array output_point = NULL;
    vx_size output_point_item_size;

    if ( (num != TIVX_KERNEL_TRIANGULATION_MAX_PARAMS)
        || (NULL == parameters[TIVX_KERNEL_TRIANGULATION_CONFIGURATION_IDX])
        || (NULL == parameters[TIVX_KERNEL_TRIANGULATION_INPUT_TRACK_IDX])
        || (NULL == parameters[TIVX_KERNEL_TRIANGULATION_INPUT_POSE_IDX])
        || (NULL == parameters[TIVX_KERNEL_TRIANGULATION_OUTPUT_POINT_IDX])
    )
    {
        status = VX_ERROR_INVALID_PARAMETERS;
        VX_PRINT(VX_ZONE_ERROR, "One or more REQUIRED parameters are set to NULL\n");
    }

    if (VX_SUCCESS == status)
    {
        configuration = (vx_user_data_object)parameters[TIVX_KERNEL_TRIANGULATION_CONFIGURATION_IDX];
        input_track = (vx_array)parameters[TIVX_KERNEL_TRIANGULATION_INPUT_TRACK_IDX];
        input_pose = (vx_user_data_object)parameters[TIVX_KERNEL_TRIANGULATION_INPUT_POSE_IDX];
        output_point = (vx_array)parameters[TIVX_KERNEL_TRIANGULATION_OUTPUT_POINT_IDX];
    }


    /* PARAMETER ATTRIBUTE FETCH */

    if (VX_SUCCESS == status)
    {
        tivxCheckStatus(&status, vxQueryUserDataObject(configuration, VX_USER_DATA_OBJECT_NAME, &configuration_name, sizeof(configuration_name)));
        tivxCheckStatus(&status, vxQueryUserDataObject(configuration, VX_USER_DATA_OBJECT_SIZE, &configuration_size, sizeof(configuration_size)));

        tivxCheckStatus(&status, vxQueryArray(input_track, VX_ARRAY_ITEMSIZE, &input_track_item_size, sizeof(input_track_item_size)));

        tivxCheckStatus(&status, vxQueryUserDataObject(input_pose, VX_USER_DATA_OBJECT_NAME, &input_pose_name, sizeof(input_pose_name)));
        tivxCheckStatus(&status, vxQueryUserDataObject(input_pose, VX_USER_DATA_OBJECT_SIZE, &input_pose_size, sizeof(input_pose_size)));

        tivxCheckStatus(&status, vxQueryArray(output_point, VX_ARRAY_ITEMSIZE, &output_point_item_size, sizeof(output_point_item_size)));
    }

    /* PARAMETER CHECKING */

    if (VX_SUCCESS == status)
    {
        if ((configuration_size != sizeof(tivx_triangulation_params_t)) ||
            (strncmp(configuration_name, "tivx_triangulation_params_t", sizeof(configuration_name)) != 0))
        {
            status = VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'configuration' should be a user_data_object of type:\n tivx_triangulation_params_t \n");
        }

        if ( input_track_item_size != sizeof(tivx_triangulation_track_t))
        {
            status = VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'input_track' should be an array of type:\n tivx_triangulation_track_t \n");
        }

        if ((input_pose_size != sizeof(tivx_triangulation_pose_t)) ||
            (strncmp(input_pose_name, "tivx_triangulation_pose_t", sizeof(input_pose_name)) != 0))
        {
            status = VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'input_pose' should be a user_data_object of type:\n tivx_triangulation_pose_t \n");
        }

        if ( output_point_item_size != sizeof(PTK_Point))
        {
            status = VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'output_point' should be an array of type:\n PTK_Point \n");
        }
    }


    /* CUSTOM PARAMETER CHECKING */

    /* < DEVELOPER_TODO: (Optional) Add any custom parameter type or range checking not */
    /*                   covered by the code-generation script.) > */

    return status;
}

static vx_status VX_CALLBACK tivxAddKernelTriangulationInitialize(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num_params)
{
    vx_status status = VX_SUCCESS;

    if ( (num_params != TIVX_KERNEL_TRIANGULATION_MAX_PARAMS)
        || (NULL == parameters[TIVX_KERNEL_TRIANGULATION_CONFIGURATION_IDX])
        || (NULL == parameters[TIVX_KERNEL_TRIANGULATION_INPUT_TRACK_IDX])
        || (NULL == parameters[TIVX_KERNEL_TRIANGULATION_INPUT_POSE_IDX])
        || (NULL == parameters[TIVX_KERNEL_TRIANGULATION_OUTPUT_POINT_IDX])
    )
    {
        status = VX_ERROR_INVALID_PARAMETERS;
        VX_PRINT(VX_ZONE_ERROR, "One or more REQUIRED parameters are set to NULL\n");
    }
    return status;
}

vx_status tivxAddKernelTriangulation(vx_context context)
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
                    TIVX_KERNEL_TRIANGULATION_NAME,
                    kernel_id,
                    NULL,
                    TIVX_KERNEL_TRIANGULATION_MAX_PARAMS,
                    tivxAddKernelTriangulationValidate,
                    tivxAddKernelTriangulationInitialize,
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
                        VX_TYPE_ARRAY,
                        VX_PARAMETER_STATE_REQUIRED
            );
            index++;
        }
        if (status == VX_SUCCESS)
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
    vx_triangulation_kernel = kernel;

    return status;
}

vx_status tivxRemoveKernelTriangulation(vx_context context)
{
    vx_status status;
    vx_kernel kernel = vx_triangulation_kernel;

    status = vxRemoveKernel(kernel);
    vx_triangulation_kernel = NULL;

    return status;
}


