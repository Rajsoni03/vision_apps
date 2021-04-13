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
#include "TI/tivx_lidar.h"
#include "tivx_lidar_kernels_priv.h"
#include "tivx_kernel_lidar_mdc.h"
#include "TI/tivx_target_kernel.h"

#include <perception/perception.h>

static vx_kernel vx_lidar_mdc_kernel = NULL;

static vx_status VX_CALLBACK tivxAddKernelLidarMdcValidate(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num,
            vx_meta_format metas[]);
static vx_status VX_CALLBACK tivxAddKernelLidarMdcInitialize(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num_params);
vx_status tivxAddKernelLidarMdc(vx_context context);
vx_status tivxRemoveKernelLidarMdc(vx_context context);

static vx_status VX_CALLBACK tivxAddKernelLidarMdcValidate(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num,
            vx_meta_format metas[])
{
    vx_status status = VX_SUCCESS;

    vx_user_data_object point_cloud = NULL;
    vx_char point_cloud_name[VX_MAX_REFERENCE_NAME];
    vx_size point_cloud_size;

    vx_user_data_object meta = NULL;
    vx_char meta_name[VX_MAX_REFERENCE_NAME];
    vx_size meta_size;

    vx_user_data_object m_ego_lidar = NULL;
    vx_char m_ego_lidar_name[VX_MAX_REFERENCE_NAME];
    vx_size m_ego_lidar_size;

    vx_user_data_object root_ecef = NULL;
    vx_char root_ecef_name[VX_MAX_REFERENCE_NAME];
    vx_size root_ecef_size;

    vx_user_data_object mdc_point_cloud = NULL;
    vx_char mdc_point_cloud_name[VX_MAX_REFERENCE_NAME];
    vx_size mdc_point_cloud_size;

    if ( (num != TIVX_KERNEL_LIDAR_MDC_MAX_PARAMS)
        || (NULL == parameters[TIVX_KERNEL_LIDAR_MDC_POINT_CLOUD_IDX])
        || (NULL == parameters[TIVX_KERNEL_LIDAR_MDC_META_IDX])
        || (NULL == parameters[TIVX_KERNEL_LIDAR_MDC_M_EGO_LIDAR_IDX])
        || (NULL == parameters[TIVX_KERNEL_LIDAR_MDC_ROOT_ECEF_IDX])
        || (NULL == parameters[TIVX_KERNEL_LIDAR_MDC_MDC_POINT_CLOUD_IDX])
    )
    {
        status = VX_ERROR_INVALID_PARAMETERS;
        VX_PRINT(VX_ZONE_ERROR, "One or more REQUIRED parameters are set to NULL\n");
    }

    if (VX_SUCCESS == status)
    {
        point_cloud = (vx_user_data_object)parameters[TIVX_KERNEL_LIDAR_MDC_POINT_CLOUD_IDX];
        meta = (vx_user_data_object)parameters[TIVX_KERNEL_LIDAR_MDC_META_IDX];
        m_ego_lidar = (vx_user_data_object)parameters[TIVX_KERNEL_LIDAR_MDC_M_EGO_LIDAR_IDX];
        root_ecef = (vx_user_data_object)parameters[TIVX_KERNEL_LIDAR_MDC_ROOT_ECEF_IDX];
        mdc_point_cloud = (vx_user_data_object)parameters[TIVX_KERNEL_LIDAR_MDC_MDC_POINT_CLOUD_IDX];
    }


    /* PARAMETER ATTRIBUTE FETCH */

    if (VX_SUCCESS == status)
    {
        tivxCheckStatus(&status, vxQueryUserDataObject(point_cloud, VX_USER_DATA_OBJECT_NAME, &point_cloud_name, sizeof(point_cloud_name)));
        tivxCheckStatus(&status, vxQueryUserDataObject(point_cloud, VX_USER_DATA_OBJECT_SIZE, &point_cloud_size, sizeof(point_cloud_size)));

        tivxCheckStatus(&status, vxQueryUserDataObject(meta, VX_USER_DATA_OBJECT_NAME, &meta_name, sizeof(meta_name)));
        tivxCheckStatus(&status, vxQueryUserDataObject(meta, VX_USER_DATA_OBJECT_SIZE, &meta_size, sizeof(meta_size)));

        tivxCheckStatus(&status, vxQueryUserDataObject(m_ego_lidar, VX_USER_DATA_OBJECT_NAME, &m_ego_lidar_name, sizeof(m_ego_lidar_name)));
        tivxCheckStatus(&status, vxQueryUserDataObject(m_ego_lidar, VX_USER_DATA_OBJECT_SIZE, &m_ego_lidar_size, sizeof(m_ego_lidar_size)));

        tivxCheckStatus(&status, vxQueryUserDataObject(root_ecef, VX_USER_DATA_OBJECT_NAME, &root_ecef_name, sizeof(root_ecef_name)));
        tivxCheckStatus(&status, vxQueryUserDataObject(root_ecef, VX_USER_DATA_OBJECT_SIZE, &root_ecef_size, sizeof(root_ecef_size)));

        tivxCheckStatus(&status, vxQueryUserDataObject(mdc_point_cloud, VX_USER_DATA_OBJECT_NAME, &mdc_point_cloud_name, sizeof(mdc_point_cloud_name)));
        tivxCheckStatus(&status, vxQueryUserDataObject(mdc_point_cloud, VX_USER_DATA_OBJECT_SIZE, &mdc_point_cloud_size, sizeof(mdc_point_cloud_size)));
    }

    /* PARAMETER CHECKING */

    if (VX_SUCCESS == status)
    {
        if ((m_ego_lidar_size != sizeof(PTK_RigidTransform)) ||
            (strncmp(m_ego_lidar_name, "PTK_RigidTransform", sizeof(m_ego_lidar_name)) != 0))
        {
            status = VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'m_ego_lidar' should be a user_data_object of type:\n PTK_RigidTransform \n");
        }

        if ((root_ecef_size != sizeof(PTK_Position)) ||
            (strncmp(root_ecef_name, "PTK_Position", sizeof(root_ecef_name)) != 0))
        {
            status = VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'root_ecef' should be a user_data_object of type:\n PTK_Position \n");
        }

       /* The following sizes cannot be statically determined. */
#if 0
        if ((meta_size != sizeof(PTK_LidarMeta)) ||
            (strncmp(meta_name, "PTK_LidarMeta", sizeof(meta_name)) != 0))
        {
            status = VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'meta' should be a user_data_object of type:\n PTK_LidarMeta \n");
        }

        if ((point_cloud_size != sizeof(PTK_PointCloud)) ||
            (strncmp(point_cloud_name, "PTK_PointCloud", sizeof(point_cloud_name)) != 0))
        {
            status = VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'point_cloud' should be a user_data_object of type:\n PTK_PointCloud \n");
        }

        if ((mdc_point_cloud_size != sizeof(PTK_PointCloud)) ||
            (strncmp(mdc_point_cloud_name, "PTK_PointCloud", sizeof(mdc_point_cloud_name)) != 0))
        {
            status = VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'mdc_point_cloud' should be a user_data_object of type:\n PTK_PointCloud \n");
        }
#endif
    }


    /* CUSTOM PARAMETER CHECKING */

    return status;
}

static vx_status VX_CALLBACK tivxAddKernelLidarMdcInitialize(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num_params)
{
    vx_status status = VX_SUCCESS;

    if ( (num_params != TIVX_KERNEL_LIDAR_MDC_MAX_PARAMS)
        || (NULL == parameters[TIVX_KERNEL_LIDAR_MDC_POINT_CLOUD_IDX])
        || (NULL == parameters[TIVX_KERNEL_LIDAR_MDC_META_IDX])
        || (NULL == parameters[TIVX_KERNEL_LIDAR_MDC_M_EGO_LIDAR_IDX])
        || (NULL == parameters[TIVX_KERNEL_LIDAR_MDC_ROOT_ECEF_IDX])
        || (NULL == parameters[TIVX_KERNEL_LIDAR_MDC_MDC_POINT_CLOUD_IDX])
    )
    {
        status = VX_ERROR_INVALID_PARAMETERS;
        VX_PRINT(VX_ZONE_ERROR, "One or more REQUIRED parameters are set to NULL\n");
    }
    return status;
}

vx_status tivxAddKernelLidarMdc(vx_context context)
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
                    TIVX_KERNEL_LIDAR_MDC_NAME,
                    kernel_id,
                    NULL,
                    TIVX_KERNEL_LIDAR_MDC_MAX_PARAMS,
                    tivxAddKernelLidarMdcValidate,
                    tivxAddKernelLidarMdcInitialize,
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
                        VX_TYPE_USER_DATA_OBJECT,
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
    vx_lidar_mdc_kernel = kernel;

    return status;
}

vx_status tivxRemoveKernelLidarMdc(vx_context context)
{
    vx_status status;
    vx_kernel kernel = vx_lidar_mdc_kernel;

    status = vxRemoveKernel(kernel);
    vx_lidar_mdc_kernel = NULL;

    return status;
}


