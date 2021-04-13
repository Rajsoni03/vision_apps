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

#include <math.h>

#include "TI/tivx.h"
#include "TI/tivx_lidar.h"
#include "VX/vx.h"
#include "tivx_lidar_kernels_priv.h"
#include "tivx_kernel_lidar_gpc.h"
#include "TI/tivx_target_kernel.h"
#include "tivx_kernels_target_utils.h"

static tivx_target_kernel vx_lidar_gpc_target_kernel = NULL;

static vx_status VX_CALLBACK tivxLidarGpcProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);

static vx_status VX_CALLBACK tivxLidarGpcCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);

static vx_status VX_CALLBACK tivxLidarGpcDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);

static vx_status VX_CALLBACK tivxLidarGpcControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);

static vx_status processGPC(
    void *config_target_ptr,
    void *pc_in_target_ptr,
    void *meta_target_ptr,
    void *m_ego_lidar_target_ptr,
    void *root_ecef_target_ptr,
    void *pc_out_target_ptr,
    void *normals_target_ptr,
    uint8_t *scratchMem);

static vx_status processGPC(
    void *config_target_ptr,
    void *pc_in_target_ptr,
    void *meta_target_ptr,
    void *m_ego_lidar_target_ptr,
    void *root_ecef_target_ptr,
    void *pc_out_target_ptr,
    void *normals_target_ptr,
    uint8_t *scratchMem
)
{
    const PTK_Lidar_GpcConfig  *config;
    PTK_PointCloud             *pc_in;
    PTK_PointCloud             *pc;
    PTK_PointCloud             *normals;
    PTK_LidarMeta              *meta;
    PTK_RigidTransform         *M_ego_lidar;
    PTK_Position               *root_ecef;
    PTK_Point                   translateENU;
    int32_t                     status;
    vx_status                   vxStatus;

    config      = (PTK_Lidar_GpcConfig *) config_target_ptr;
    pc_in       = (PTK_PointCloud *) pc_in_target_ptr;
    pc          = (PTK_PointCloud *) pc_out_target_ptr;
    normals     = (PTK_PointCloud *) normals_target_ptr;
    meta        = (PTK_LidarMeta *) meta_target_ptr;
    M_ego_lidar = (PTK_RigidTransform *) m_ego_lidar_target_ptr;
    root_ecef   = (PTK_Position *) root_ecef_target_ptr;
    vxStatus    = VX_SUCCESS;

    /* Copy input state to output state as is. */
    PTK_PointCloud_clear(pc);
    PTK_PointCloud_copy(pc, pc_in);

    /* Copy the meta data. */
    PTK_PointCloud_copyMetaData(normals, pc_in);

    status = PTK_Lidar_performGPC(pc, meta, config, scratchMem, normals);

    if (status == PTK_ALG_RET_SUCCESS)
    {
        /* This is a bit nasty to sneak in here, it might be more
         * appropriate as a separate node
         */
        PTK_Lidar_shiftToRootENU(pc, meta, M_ego_lidar,
                                 root_ecef, &translateENU);
    }
    else
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}

static vx_status VX_CALLBACK tivxLidarGpcProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    tivx_obj_desc_user_data_object_t *configuration_desc;
    tivx_obj_desc_user_data_object_t *point_cloud_in_desc;
    tivx_obj_desc_user_data_object_t *lidar_meta_desc;
    tivx_obj_desc_user_data_object_t *m_ego_lidar_desc;
    tivx_obj_desc_user_data_object_t *root_ecef_desc;
    tivx_obj_desc_user_data_object_t *point_cloud_out_desc;
    tivx_obj_desc_user_data_object_t *normal_cloud_desc;

    if ( (num_params != TIVX_KERNEL_LIDAR_GPC_MAX_PARAMS)
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_GPC_CONFIGURATION_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_GPC_POINT_CLOUD_IN_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_GPC_LIDAR_META_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_GPC_M_EGO_LIDAR_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_GPC_ROOT_ECEF_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_GPC_POINT_CLOUD_OUT_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_GPC_NORMAL_CLOUD_IDX])
    )
    {
        status = VX_FAILURE;
    }
    else
    {
        void       *configuration_target_ptr;
        void       *point_cloud_in_target_ptr;
        void       *lidar_meta_target_ptr;
        void       *m_ego_lidar_target_ptr;
        void       *root_ecef_target_ptr;
        void       *point_cloud_out_target_ptr;
        void       *normal_cloud_target_ptr;
        uint8_t    *scratchMem;
        uint32_t    size = 0;

        configuration_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_LIDAR_GPC_CONFIGURATION_IDX];
        point_cloud_in_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_LIDAR_GPC_POINT_CLOUD_IN_IDX];
        lidar_meta_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_LIDAR_GPC_LIDAR_META_IDX];
        m_ego_lidar_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_LIDAR_GPC_M_EGO_LIDAR_IDX];
        root_ecef_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_LIDAR_GPC_ROOT_ECEF_IDX];
        point_cloud_out_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_LIDAR_GPC_POINT_CLOUD_OUT_IDX];
        normal_cloud_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_LIDAR_GPC_NORMAL_CLOUD_IDX];

        configuration_target_ptr = tivxMemShared2TargetPtr(&configuration_desc->mem_ptr);
        tivxMemBufferMap(configuration_target_ptr,
           configuration_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_READ_ONLY);

        point_cloud_in_target_ptr = tivxMemShared2TargetPtr(&point_cloud_in_desc->mem_ptr);
        tivxMemBufferMap(point_cloud_in_target_ptr,
           point_cloud_in_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_READ_ONLY);

        lidar_meta_target_ptr = tivxMemShared2TargetPtr(&lidar_meta_desc->mem_ptr);
        tivxMemBufferMap(lidar_meta_target_ptr,
           lidar_meta_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_READ_ONLY);

        m_ego_lidar_target_ptr = tivxMemShared2TargetPtr(&m_ego_lidar_desc->mem_ptr);
        tivxMemBufferMap(m_ego_lidar_target_ptr,
           m_ego_lidar_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_READ_ONLY);

        root_ecef_target_ptr = tivxMemShared2TargetPtr(&root_ecef_desc->mem_ptr);
        tivxMemBufferMap(root_ecef_target_ptr,
           root_ecef_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_READ_ONLY);

        point_cloud_out_target_ptr = tivxMemShared2TargetPtr(&point_cloud_out_desc->mem_ptr);
        tivxMemBufferMap(point_cloud_out_target_ptr,
           point_cloud_out_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_WRITE_ONLY);

        normal_cloud_target_ptr = tivxMemShared2TargetPtr(&normal_cloud_desc->mem_ptr);
        tivxMemBufferMap(normal_cloud_target_ptr,
           normal_cloud_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_WRITE_ONLY);

        status = tivxGetTargetKernelInstanceContext(kernel,
                                                    (void **)&scratchMem,
                                                    &size);

        if ((VX_SUCCESS == status) && (NULL != scratchMem) && (size != 0))
        {
            /* call kernel processing function */
            status = processGPC(configuration_target_ptr,
                                point_cloud_in_target_ptr,
                                lidar_meta_target_ptr,
                                m_ego_lidar_target_ptr,
                                root_ecef_target_ptr,
                                point_cloud_out_target_ptr,
                                normal_cloud_target_ptr,
                                scratchMem);
        }

        /* kernel processing function complete */

        tivxMemBufferUnmap(configuration_target_ptr,
           configuration_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);

        tivxMemBufferUnmap(point_cloud_in_target_ptr,
           point_cloud_in_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);

        tivxMemBufferUnmap(lidar_meta_target_ptr,
           lidar_meta_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);

        tivxMemBufferUnmap(m_ego_lidar_target_ptr,
           m_ego_lidar_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);

        tivxMemBufferUnmap(root_ecef_target_ptr,
           root_ecef_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);

        tivxMemBufferUnmap(point_cloud_out_target_ptr,
           point_cloud_out_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_WRITE_ONLY);

        tivxMemBufferUnmap(normal_cloud_target_ptr,
           normal_cloud_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_WRITE_ONLY);

    }

    return status;
}

static vx_status VX_CALLBACK tivxLidarGpcCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    uint8_t        *scratchMem = NULL;
    uint32_t        memSize;
    vx_status       status = VX_SUCCESS;

    memSize = PTK_Lidar_getGpcScratchMemSize();

    if (memSize == 0)
    {
        VX_PRINT(VX_ZONE_ERROR, "Invalid scratch memory size.\n");
        status = VX_FAILURE;
    }

    if (status == (vx_status)VX_SUCCESS)
    {
        scratchMem = tivxMemAlloc(memSize, TIVX_MEM_EXTERNAL);

        if (NULL == scratchMem)
        {
            VX_PRINT(VX_ZONE_ERROR,
                     "Failed to allocate memory block of size %d bytes\n",
                     memSize);

            status = VX_FAILURE;
        }
    }

    if (status == (vx_status)VX_SUCCESS)
    {
        status = tivxSetTargetKernelInstanceContext(kernel, scratchMem, memSize);
    }

    return status;
}

static vx_status VX_CALLBACK tivxLidarGpcDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    uint8_t        *scratchMem;
    vx_status       status = VX_SUCCESS;
    uint32_t        size = 0;

    /*free all memory */
    if (status == (vx_status)VX_SUCCESS)
    {
        status = tivxGetTargetKernelInstanceContext(kernel,
                                                    (void **)&scratchMem,
                                                    &size);
    }

    if ((VX_SUCCESS == status) && (NULL != scratchMem) && (size != 0))
    {
        tivxMemFree(scratchMem, size, TIVX_MEM_EXTERNAL);
    }

    return status;
}

static vx_status VX_CALLBACK tivxLidarGpcControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;

    /* < DEVELOPER_TODO: (Optional) Add any target kernel control code here (e.g. commands */
    /*                   the user can call to modify the processing of the kernel at run-time) > */

    return status;
}

void tivxAddTargetKernelLidarGpc(void)
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
            vx_lidar_gpc_target_kernel = tivxAddTargetKernelByName(
                                TIVX_KERNEL_LIDAR_GPC_NAME,
                                target_name[i],
                                tivxLidarGpcProcess,
                                tivxLidarGpcCreate,
                                tivxLidarGpcDelete,
                                tivxLidarGpcControl,
                                NULL);
        }
    }
}

void tivxRemoveTargetKernelLidarGpc(void)
{
    vx_status status = VX_SUCCESS;

    status = tivxRemoveTargetKernel(vx_lidar_gpc_target_kernel);
    if (status == VX_SUCCESS)
    {
        vx_lidar_gpc_target_kernel = NULL;
    }
}


