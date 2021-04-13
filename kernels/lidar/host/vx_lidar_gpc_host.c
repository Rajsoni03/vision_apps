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
#include "tivx_kernel_lidar_gpc.h"
#include "TI/tivx_target_kernel.h"

#include <perception/perception.h>

static vx_kernel vx_lidar_gpc_kernel = NULL;

static vx_status VX_CALLBACK tivxAddKernelLidarGpcValidate(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num,
            vx_meta_format metas[]);
static vx_status VX_CALLBACK tivxAddKernelLidarGpcInitialize(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num_params);
vx_status tivxAddKernelLidarGpc(vx_context context);
vx_status tivxRemoveKernelLidarGpc(vx_context context);

static vx_status VX_CALLBACK tivxAddKernelLidarGpcValidate(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num,
            vx_meta_format metas[])
{
    vx_status status = VX_SUCCESS;

    vx_user_data_object configuration = NULL;
    vx_char configuration_name[VX_MAX_REFERENCE_NAME];
    vx_size configuration_size;

    vx_user_data_object point_cloud_in = NULL;
    vx_char point_cloud_in_name[VX_MAX_REFERENCE_NAME];
    vx_size point_cloud_in_size;

    vx_user_data_object lidar_meta = NULL;
    vx_char lidar_meta_name[VX_MAX_REFERENCE_NAME];
    vx_size lidar_meta_size;

    vx_user_data_object m_ego_lidar = NULL;
    vx_char m_ego_lidar_name[VX_MAX_REFERENCE_NAME];
    vx_size m_ego_lidar_size;

    vx_user_data_object root_ecef = NULL;
    vx_char root_ecef_name[VX_MAX_REFERENCE_NAME];
    vx_size root_ecef_size;

    vx_user_data_object point_cloud_out = NULL;
    vx_char point_cloud_out_name[VX_MAX_REFERENCE_NAME];
    vx_size point_cloud_out_size;

    vx_user_data_object normal_cloud = NULL;
    vx_char normal_cloud_name[VX_MAX_REFERENCE_NAME];
    vx_size normal_cloud_size;

    if ( (num != TIVX_KERNEL_LIDAR_GPC_MAX_PARAMS)
        || (NULL == parameters[TIVX_KERNEL_LIDAR_GPC_CONFIGURATION_IDX])
        || (NULL == parameters[TIVX_KERNEL_LIDAR_GPC_POINT_CLOUD_IN_IDX])
        || (NULL == parameters[TIVX_KERNEL_LIDAR_GPC_LIDAR_META_IDX])
        || (NULL == parameters[TIVX_KERNEL_LIDAR_GPC_M_EGO_LIDAR_IDX])
        || (NULL == parameters[TIVX_KERNEL_LIDAR_GPC_ROOT_ECEF_IDX])
        || (NULL == parameters[TIVX_KERNEL_LIDAR_GPC_POINT_CLOUD_OUT_IDX])
        || (NULL == parameters[TIVX_KERNEL_LIDAR_GPC_NORMAL_CLOUD_IDX])
    )
    {
        status = VX_ERROR_INVALID_PARAMETERS;
        VX_PRINT(VX_ZONE_ERROR, "One or more REQUIRED parameters are set to NULL\n");
    }

    if (VX_SUCCESS == status)
    {
        configuration = (vx_user_data_object)parameters[TIVX_KERNEL_LIDAR_GPC_CONFIGURATION_IDX];
        point_cloud_in = (vx_user_data_object)parameters[TIVX_KERNEL_LIDAR_GPC_POINT_CLOUD_IN_IDX];
        lidar_meta = (vx_user_data_object)parameters[TIVX_KERNEL_LIDAR_GPC_LIDAR_META_IDX];
        m_ego_lidar = (vx_user_data_object)parameters[TIVX_KERNEL_LIDAR_GPC_M_EGO_LIDAR_IDX];
        root_ecef = (vx_user_data_object)parameters[TIVX_KERNEL_LIDAR_GPC_ROOT_ECEF_IDX];
        point_cloud_out = (vx_user_data_object)parameters[TIVX_KERNEL_LIDAR_GPC_POINT_CLOUD_OUT_IDX];
        normal_cloud = (vx_user_data_object)parameters[TIVX_KERNEL_LIDAR_GPC_NORMAL_CLOUD_IDX];
    }


    /* PARAMETER ATTRIBUTE FETCH */

    if (VX_SUCCESS == status)
    {
        tivxCheckStatus(&status, vxQueryUserDataObject(configuration, VX_USER_DATA_OBJECT_NAME, &configuration_name, sizeof(configuration_name)));
        tivxCheckStatus(&status, vxQueryUserDataObject(configuration, VX_USER_DATA_OBJECT_SIZE, &configuration_size, sizeof(configuration_size)));

        tivxCheckStatus(&status, vxQueryUserDataObject(point_cloud_in, VX_USER_DATA_OBJECT_NAME, &point_cloud_in_name, sizeof(point_cloud_in_name)));
        tivxCheckStatus(&status, vxQueryUserDataObject(point_cloud_in, VX_USER_DATA_OBJECT_SIZE, &point_cloud_in_size, sizeof(point_cloud_in_size)));

        tivxCheckStatus(&status, vxQueryUserDataObject(lidar_meta, VX_USER_DATA_OBJECT_NAME, &lidar_meta_name, sizeof(lidar_meta_name)));
        tivxCheckStatus(&status, vxQueryUserDataObject(lidar_meta, VX_USER_DATA_OBJECT_SIZE, &lidar_meta_size, sizeof(lidar_meta_size)));

        tivxCheckStatus(&status, vxQueryUserDataObject(m_ego_lidar, VX_USER_DATA_OBJECT_NAME, &m_ego_lidar_name, sizeof(m_ego_lidar_name)));
        tivxCheckStatus(&status, vxQueryUserDataObject(m_ego_lidar, VX_USER_DATA_OBJECT_SIZE, &m_ego_lidar_size, sizeof(m_ego_lidar_size)));

        tivxCheckStatus(&status, vxQueryUserDataObject(root_ecef, VX_USER_DATA_OBJECT_NAME, &root_ecef_name, sizeof(root_ecef_name)));
        tivxCheckStatus(&status, vxQueryUserDataObject(root_ecef, VX_USER_DATA_OBJECT_SIZE, &root_ecef_size, sizeof(root_ecef_size)));

        tivxCheckStatus(&status, vxQueryUserDataObject(point_cloud_out, VX_USER_DATA_OBJECT_NAME, &point_cloud_out_name, sizeof(point_cloud_out_name)));
        tivxCheckStatus(&status, vxQueryUserDataObject(point_cloud_out, VX_USER_DATA_OBJECT_SIZE, &point_cloud_out_size, sizeof(point_cloud_out_size)));

        tivxCheckStatus(&status, vxQueryUserDataObject(normal_cloud, VX_USER_DATA_OBJECT_NAME, &normal_cloud_name, sizeof(normal_cloud_name)));
        tivxCheckStatus(&status, vxQueryUserDataObject(normal_cloud, VX_USER_DATA_OBJECT_SIZE, &normal_cloud_size, sizeof(normal_cloud_size)));
    }

    /* PARAMETER CHECKING */

    if (VX_SUCCESS == status)
    {
        if ((configuration_size != sizeof(PTK_Lidar_GpcConfig)) ||
            (strncmp(configuration_name, "PTK_Lidar_GpcConfig", sizeof(configuration_name)) != 0))
        {
            status = VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'configuration' should be a user_data_object of type:\n PTK_Lidar_GpcConfig \n");
        }

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
        if ((lidar_meta_size != sizeof(PTK_LidarMeta)) ||
            (strncmp(lidar_meta_name, "PTK_LidarMeta", sizeof(lidar_meta_name)) != 0))
        {
            status = VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'lidar_meta' should be a user_data_object of type:\n PTK_LidarMeta \n");
        }

        if ((point_cloud_in_size != sizeof(PTK_PointCloud)) ||
            (strncmp(point_cloud_in_name, "PTK_PointCloud", sizeof(point_cloud_in_name)) != 0))
        {
            status = VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'point_cloud_in' should be a user_data_object of type:\n PTK_PointCloud \n");
        }

        if ((point_cloud_out_size != sizeof(PTK_PointCloud)) ||
            (strncmp(point_cloud_out_name, "PTK_PointCloud", sizeof(point_cloud_out_name)) != 0))
        {
            status = VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'point_cloud_out' should be a user_data_object of type:\n PTK_PointCloud \n");
        }

        if ((normal_cloud_size != sizeof(PTK_PointCloud)) ||
            (strncmp(normal_cloud_name, "PTK_PointCloud", sizeof(normal_cloud_name)) != 0))
        {
            status = VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'normal_cloud' should be a user_data_object of type:\n PTK_PointCloud \n");
        }
#endif
    }

    return status;
}

static vx_status VX_CALLBACK tivxAddKernelLidarGpcInitialize(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num_params)
{
    vx_status status = VX_SUCCESS;

    if ( (num_params != TIVX_KERNEL_LIDAR_GPC_MAX_PARAMS)
        || (NULL == parameters[TIVX_KERNEL_LIDAR_GPC_CONFIGURATION_IDX])
        || (NULL == parameters[TIVX_KERNEL_LIDAR_GPC_POINT_CLOUD_IN_IDX])
        || (NULL == parameters[TIVX_KERNEL_LIDAR_GPC_LIDAR_META_IDX])
        || (NULL == parameters[TIVX_KERNEL_LIDAR_GPC_M_EGO_LIDAR_IDX])
        || (NULL == parameters[TIVX_KERNEL_LIDAR_GPC_ROOT_ECEF_IDX])
        || (NULL == parameters[TIVX_KERNEL_LIDAR_GPC_POINT_CLOUD_OUT_IDX])
        || (NULL == parameters[TIVX_KERNEL_LIDAR_GPC_NORMAL_CLOUD_IDX])
    )
    {
        status = VX_ERROR_INVALID_PARAMETERS;
        VX_PRINT(VX_ZONE_ERROR, "One or more REQUIRED parameters are set to NULL\n");
    }
    return status;
}

vx_status tivxAddKernelLidarGpc(vx_context context)
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
                    TIVX_KERNEL_LIDAR_GPC_NAME,
                    kernel_id,
                    NULL,
                    TIVX_KERNEL_LIDAR_GPC_MAX_PARAMS,
                    tivxAddKernelLidarGpcValidate,
                    tivxAddKernelLidarGpcInitialize,
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
    vx_lidar_gpc_kernel = kernel;

    return status;
}

vx_status tivxRemoveKernelLidarGpc(vx_context context)
{
    vx_status status;
    vx_kernel kernel = vx_lidar_gpc_kernel;

    status = vxRemoveKernel(kernel);
    vx_lidar_gpc_kernel = NULL;

    return status;
}


