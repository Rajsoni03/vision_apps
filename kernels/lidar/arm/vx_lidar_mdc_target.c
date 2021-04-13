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
#include "VX/vx.h"
#include "tivx_lidar_kernels_priv.h"
#include "tivx_kernel_lidar_mdc.h"
#include "TI/tivx_target_kernel.h"
#include "tivx_kernels_target_utils.h"

static tivx_target_kernel vx_lidar_mdc_target_kernel = NULL;

static vx_status VX_CALLBACK tivxLidarMdcProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxLidarMdcCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxLidarMdcDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxLidarMdcControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);

static vx_status VX_CALLBACK tivxLidarMdcProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    tivx_obj_desc_user_data_object_t *point_cloud_desc;
    tivx_obj_desc_user_data_object_t *meta_desc;
    tivx_obj_desc_user_data_object_t *m_ego_lidar_desc;
    tivx_obj_desc_user_data_object_t *root_ecef_desc;
    tivx_obj_desc_user_data_object_t *mdc_point_cloud_desc;

    PTK_PointCloud *cloud;
    PTK_LidarMeta *meta;
    PTK_RigidTransform *M_ego_lidar;
    PTK_Position *root_ecef;
    PTK_PointCloud *mdc_cloud;

    if ( (num_params != TIVX_KERNEL_LIDAR_MDC_MAX_PARAMS)
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_MDC_POINT_CLOUD_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_MDC_META_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_MDC_M_EGO_LIDAR_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_MDC_ROOT_ECEF_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_MDC_MDC_POINT_CLOUD_IDX])
    )
    {
        status = VX_FAILURE;
    }
    else
    {
        void *point_cloud_target_ptr;
        void *meta_target_ptr;
        void *m_ego_lidar_target_ptr;
        void *root_ecef_target_ptr;
        void *mdc_point_cloud_target_ptr;

        point_cloud_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_LIDAR_MDC_POINT_CLOUD_IDX];
        meta_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_LIDAR_MDC_META_IDX];
        m_ego_lidar_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_LIDAR_MDC_M_EGO_LIDAR_IDX];
        root_ecef_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_LIDAR_MDC_ROOT_ECEF_IDX];
        mdc_point_cloud_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_LIDAR_MDC_MDC_POINT_CLOUD_IDX];

        point_cloud_target_ptr = tivxMemShared2TargetPtr(&point_cloud_desc->mem_ptr);
        tivxMemBufferMap(point_cloud_target_ptr,
           point_cloud_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_READ_ONLY);

        meta_target_ptr = tivxMemShared2TargetPtr(&meta_desc->mem_ptr);
        tivxMemBufferMap(meta_target_ptr,
           meta_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_READ_ONLY);

        m_ego_lidar_target_ptr = tivxMemShared2TargetPtr(&m_ego_lidar_desc->mem_ptr);
        tivxMemBufferMap(m_ego_lidar_target_ptr,
           m_ego_lidar_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_READ_ONLY);

        root_ecef_target_ptr = tivxMemShared2TargetPtr(&root_ecef_desc->mem_ptr);
        tivxMemBufferMap(root_ecef_target_ptr,
           root_ecef_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_READ_ONLY);

        mdc_point_cloud_target_ptr = tivxMemShared2TargetPtr(&mdc_point_cloud_desc->mem_ptr);
        tivxMemBufferMap(mdc_point_cloud_target_ptr,
           mdc_point_cloud_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_WRITE_ONLY);

        /* call kernel processing function */
        cloud = (PTK_PointCloud *) point_cloud_target_ptr;
        meta = (PTK_LidarMeta *) meta_target_ptr;
        M_ego_lidar = (PTK_RigidTransform *) m_ego_lidar_target_ptr;
        root_ecef = (PTK_Position *) root_ecef_target_ptr;
        mdc_cloud = (PTK_PointCloud *) mdc_point_cloud_target_ptr;

        PTK_PointCloud_clear(mdc_cloud);
        PTK_PointCloud_copy(mdc_cloud, cloud);

        status = PTK_Lidar_correctMotionDistortion_HDL32E(mdc_cloud,
                                                          meta,
                                                          M_ego_lidar,
                                                          root_ecef);

        if (status != PTK_ALG_RET_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR,
                     "PTK_Lidar_correctMotionDistortion_HDL32E() failed.\n");
            status = VX_FAILURE;
        }

        /* kernel processing function complete */

        tivxMemBufferUnmap(point_cloud_target_ptr,
           point_cloud_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);

        tivxMemBufferUnmap(meta_target_ptr,
           meta_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);

        tivxMemBufferUnmap(m_ego_lidar_target_ptr,
           m_ego_lidar_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);

        tivxMemBufferUnmap(root_ecef_target_ptr,
           root_ecef_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);

        tivxMemBufferUnmap(mdc_point_cloud_target_ptr,
           mdc_point_cloud_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_WRITE_ONLY);

    }

    return status;
}

static vx_status VX_CALLBACK tivxLidarMdcCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;

    /* < DEVELOPER_TODO: (Optional) Add any target kernel create code here (e.g. allocating */
    /*                   local memory buffers, one time initialization, etc) > */

    return status;
}

static vx_status VX_CALLBACK tivxLidarMdcDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;

    /* < DEVELOPER_TODO: (Optional) Add any target kernel delete code here (e.g. freeing */
    /*                   local memory buffers, etc) > */

    return status;
}

static vx_status VX_CALLBACK tivxLidarMdcControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;

    /* < DEVELOPER_TODO: (Optional) Add any target kernel control code here (e.g. commands */
    /*                   the user can call to modify the processing of the kernel at run-time) > */

    return status;
}

void tivxAddTargetKernelLidarMdc(void)
{
    vx_status status = VX_FAILURE;
    char target_name[4][TIVX_TARGET_MAX_NAME];
    uint32_t num_targets = 1;
    vx_enum self_cpu;

    self_cpu = tivxGetSelfCpuId();

    if ( self_cpu == TIVX_CPU_ID_DSP1 )
    {
        strncpy(target_name[0], TIVX_TARGET_DSP1, TIVX_TARGET_MAX_NAME);
        status = VX_SUCCESS;
    }
    else
    if ( self_cpu == TIVX_CPU_ID_DSP2 )
    {
        strncpy(target_name[0], TIVX_TARGET_DSP2, TIVX_TARGET_MAX_NAME);
        status = VX_SUCCESS;
    }
    else
    if ( self_cpu == TIVX_CPU_ID_A72_0 )
    {
        strncpy(target_name[0], TIVX_TARGET_A72_0, TIVX_TARGET_MAX_NAME);
        strncpy(target_name[1], TIVX_TARGET_A72_1, TIVX_TARGET_MAX_NAME);
        strncpy(target_name[2], TIVX_TARGET_A72_2, TIVX_TARGET_MAX_NAME);
        strncpy(target_name[3], TIVX_TARGET_A72_3, TIVX_TARGET_MAX_NAME);
        num_targets = 4;
        status = VX_SUCCESS;
    }
    else
    if ( self_cpu == TIVX_CPU_ID_DSP_C7_1 )
    {
        strncpy(target_name[0], TIVX_TARGET_DSP_C7_1, TIVX_TARGET_MAX_NAME);
        status = VX_SUCCESS;
    }
    else
    {
        status = VX_FAILURE;
    }

    if (status == VX_SUCCESS)
    {
        uint32_t    i;

        for (i = 0; i < num_targets; i++)
        {
            vx_lidar_mdc_target_kernel = tivxAddTargetKernelByName(
                                TIVX_KERNEL_LIDAR_MDC_NAME,
                                target_name[i],
                                tivxLidarMdcProcess,
                                tivxLidarMdcCreate,
                                tivxLidarMdcDelete,
                                tivxLidarMdcControl,
                                NULL);
        }
    }
}

void tivxRemoveTargetKernelLidarMdc(void)
{
    vx_status status = VX_SUCCESS;

    status = tivxRemoveTargetKernel(vx_lidar_mdc_target_kernel);
    if (status == VX_SUCCESS)
    {
        vx_lidar_mdc_target_kernel = NULL;
    }
}


