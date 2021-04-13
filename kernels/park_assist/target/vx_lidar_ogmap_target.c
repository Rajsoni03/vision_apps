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

#include "VX/vx.h"

#include "TI/tivx.h"
#include "TI/tivx_park_assist.h"
#include "TI/tivx_lidar.h"
#include "TI/tivx_target_kernel.h"
#include "TI/tivx_park_assist_kernels.h"
#include "tivx_park_assist_kernels_priv.h"
#include "tivx_kernel_lidar_ogmap.h"
#include "tivx_kernels_target_utils.h"
#include <vx_ptk_alg_common.h>

#define TIVX_LIDAR_OGMAP_ALG_ID           (0U)
#define TIVX_FSD_PFSD_ALG_ID              (TIVX_LIDAR_OGMAP_ALG_ID + 1)
#define TIVX_MAX_LIDAR_OGMAP_FSD_PFSD_IDS (TIVX_FSD_PFSD_ALG_ID + 1)

static tivx_target_kernel vx_lidar_ogmap_target_kernel = NULL;

static vx_status VX_CALLBACK tivxLidarOgmapProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxLidarOgmapCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxLidarOgmapDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxLidarOgmapControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);

static vx_status VX_CALLBACK tivxLidarOgmapProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    tivx_obj_desc_user_data_object_t *configuration_desc;
    tivx_obj_desc_user_data_object_t *point_cloud_desc;
    tivx_obj_desc_user_data_object_t *reftr_and_pose_desc;
    tivx_obj_desc_user_data_object_t *pfsd_out_desc_desc;
    tivx_obj_desc_user_data_object_t *acc_output_map_desc;
    tivx_obj_desc_user_data_object_t *inst_output_map_desc;
    vx_status status = VX_SUCCESS;

    if ( (num_params != TIVX_KERNEL_LIDAR_OGMAP_MAX_PARAMS)
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_OGMAP_CONFIGURATION_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_OGMAP_POINT_CLOUD_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_OGMAP_REFTR_AND_POSE_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_OGMAP_PFSD_OUT_DESC_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_OGMAP_ACC_OUTPUT_MAP_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_OGMAP_INST_OUTPUT_MAP_IDX])
    )
    {
        status = VX_FAILURE;
    }
    else
    {
        void                  *configuration_target_ptr;
        void                  *point_cloud_target_ptr;
        void                  *reftr_and_pose_target_ptr;
        void                  *pfsd_out_desc_target_ptr;
        void                  *acc_output_map_target_ptr;
        void                  *inst_output_map_target_ptr;
        tivx_ptk_alg_if_cntxt *algCntxt;
        int32_t                algRetCode;
        uint32_t               size;

        configuration_desc = (tivx_obj_desc_user_data_object_t *)
            obj_desc[TIVX_KERNEL_LIDAR_OGMAP_CONFIGURATION_IDX];

        point_cloud_desc = (tivx_obj_desc_user_data_object_t *)
            obj_desc[TIVX_KERNEL_LIDAR_OGMAP_POINT_CLOUD_IDX];

        reftr_and_pose_desc = (tivx_obj_desc_user_data_object_t *)
            obj_desc[TIVX_KERNEL_LIDAR_OGMAP_REFTR_AND_POSE_IDX];

        pfsd_out_desc_desc = (tivx_obj_desc_user_data_object_t *)
            obj_desc[TIVX_KERNEL_LIDAR_OGMAP_PFSD_OUT_DESC_IDX];

        acc_output_map_desc = (tivx_obj_desc_user_data_object_t *)
            obj_desc[TIVX_KERNEL_LIDAR_OGMAP_ACC_OUTPUT_MAP_IDX];

        inst_output_map_desc = (tivx_obj_desc_user_data_object_t *)
            obj_desc[TIVX_KERNEL_LIDAR_OGMAP_INST_OUTPUT_MAP_IDX];

        configuration_target_ptr =
            tivxMemShared2TargetPtr(&configuration_desc->mem_ptr);

        tivxMemBufferMap(configuration_target_ptr,
                         configuration_desc->mem_size,
                         VX_MEMORY_TYPE_HOST,
                         VX_READ_ONLY);

        point_cloud_target_ptr =
            tivxMemShared2TargetPtr(&point_cloud_desc->mem_ptr);

        tivxMemBufferMap(point_cloud_target_ptr,
                         point_cloud_desc->mem_size,
                         VX_MEMORY_TYPE_HOST,
                         VX_READ_ONLY);

        reftr_and_pose_target_ptr =
            tivxMemShared2TargetPtr(&reftr_and_pose_desc->mem_ptr);

        tivxMemBufferMap(reftr_and_pose_target_ptr,
                         reftr_and_pose_desc->mem_size,
                         VX_MEMORY_TYPE_HOST,
                         VX_READ_ONLY);

        pfsd_out_desc_target_ptr =
            tivxMemShared2TargetPtr(&pfsd_out_desc_desc->mem_ptr);

        tivxMemBufferMap(pfsd_out_desc_target_ptr,
                         pfsd_out_desc_desc->mem_size,
                         VX_MEMORY_TYPE_HOST,
                         VX_WRITE_ONLY);

        acc_output_map_target_ptr =
            tivxMemShared2TargetPtr(&acc_output_map_desc->mem_ptr);

        tivxMemBufferMap(acc_output_map_target_ptr,
                         acc_output_map_desc->mem_size,
                         VX_MEMORY_TYPE_HOST,
                         VX_WRITE_ONLY);

        inst_output_map_target_ptr =
            tivxMemShared2TargetPtr(&inst_output_map_desc->mem_ptr);

        tivxMemBufferMap(inst_output_map_target_ptr,
                         inst_output_map_desc->mem_size,
                         VX_MEMORY_TYPE_HOST,
                         VX_WRITE_ONLY);

        /* call kernel processing function */
        algCntxt = NULL;
        status = tivxGetTargetKernelInstanceContext(kernel,
            (void **)&algCntxt, &size);

        if ((VX_SUCCESS != status) || (NULL == algCntxt) ||
            (sizeof(tivx_ptk_alg_if_cntxt) != size))
        {
            VX_PRINT(VX_ZONE_ERROR, "Failed to get algorithm handle!\n");
            status = VX_FAILURE;
        }
        else
        {
            PTK_PointCloud             * cloud;
            const PTK_InsPoseAndRef    * poseAndRef;
            PTK_Map                    * outAccMap;
            PTK_Map                    * outInstMap;
            PTK_Alg_FsdPfsdPSDesc      * pfsdDesc;
            PTK_AlgHandle                algHandle;

            cloud      = (PTK_PointCloud *)point_cloud_target_ptr;
            poseAndRef = (PTK_InsPoseAndRef *)reftr_and_pose_target_ptr;
            pfsdDesc   = (PTK_Alg_FsdPfsdPSDesc*)pfsd_out_desc_target_ptr;
            outAccMap  = (PTK_Map *)acc_output_map_target_ptr;
            outInstMap = (PTK_Map *)inst_output_map_target_ptr;

            algHandle = algCntxt->algHandle[TIVX_LIDAR_OGMAP_ALG_ID];
            algRetCode = PTK_Alg_LidarOgmapProcess(algHandle, cloud, outAccMap, outInstMap);

            if (algRetCode != PTK_ALG_RET_SUCCESS)
            {
                VX_PRINT(VX_ZONE_ERROR, "PTK_Alg_LidarOgmapProcess() failed.\n");
                status = VX_FAILURE;
            }

            algHandle = algCntxt->algHandle[TIVX_FSD_PFSD_ALG_ID];
            if ((status == VX_SUCCESS) && algHandle)
            {
                algRetCode = PTK_Alg_FsdPfsdProcess(algHandle,
                                                    poseAndRef,
                                                    outAccMap,
                                                    pfsdDesc);

                if (algRetCode != PTK_ALG_RET_SUCCESS)
                {
                    VX_PRINT(VX_ZONE_ERROR, "PTK_Alg_FsdPfsdProcess() failed.\n");
                    status = VX_FAILURE;
                }
            }
        }

        /* kernel processing function complete */

        tivxMemBufferUnmap(configuration_target_ptr,
                           configuration_desc->mem_size,
                           VX_MEMORY_TYPE_HOST,
                           VX_READ_ONLY);

        tivxMemBufferUnmap(point_cloud_target_ptr,
                           point_cloud_desc->mem_size,
                           VX_MEMORY_TYPE_HOST,
                           VX_READ_ONLY);

        tivxMemBufferUnmap(reftr_and_pose_target_ptr,
                           reftr_and_pose_desc->mem_size,
                           VX_MEMORY_TYPE_HOST,
                           VX_READ_ONLY);

        tivxMemBufferUnmap(pfsd_out_desc_target_ptr,
                           pfsd_out_desc_desc->mem_size,
                           VX_MEMORY_TYPE_HOST,
                           VX_WRITE_ONLY);

        tivxMemBufferUnmap(acc_output_map_target_ptr,
                           acc_output_map_desc->mem_size,
                           VX_MEMORY_TYPE_HOST,
                           VX_WRITE_ONLY);

        tivxMemBufferUnmap(inst_output_map_target_ptr,
                           inst_output_map_desc->mem_size,
                           VX_MEMORY_TYPE_HOST,
                           VX_WRITE_ONLY);

    }

    return status;
}

static vx_status VX_CALLBACK tivxLidarOgmapCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status                               status = VX_SUCCESS;

    if ( (num_params != TIVX_KERNEL_LIDAR_OGMAP_MAX_PARAMS)
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_OGMAP_CONFIGURATION_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_OGMAP_POINT_CLOUD_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_OGMAP_REFTR_AND_POSE_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_OGMAP_PFSD_OUT_DESC_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_OGMAP_ACC_OUTPUT_MAP_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_OGMAP_INST_OUTPUT_MAP_IDX])
    )
    {
        VX_PRINT(VX_ZONE_ERROR, "Interface parameter check failed.\n");

        status = VX_FAILURE;
    }
    else
    {
        tivx_ptk_alg_if_cntxt             * algCntxt;
        tivx_lidar_ogmap_pfsd_params_t    * cfgParams;
        const PTK_Alg_LidarOgmapParams    * ogCfg;
        const PTK_Alg_FsdPfsdParams       * fsdPfsdCfg;
        tivx_obj_desc_user_data_object_t  * cfgObjDesc;
        void                              * cfg_target_ptr;
        PTK_Api_MemoryReq                 * memReqOg;
        PTK_Api_MemoryReq                 * memReqFsd;
        PTK_Api_MemoryReq                   memReq[TIVX_MAX_LIDAR_OGMAP_FSD_PFSD_IDS];
        uint32_t                            numMemReq;
        int32_t                             algRetCode;

        cfgObjDesc = (tivx_obj_desc_user_data_object_t *)
                             obj_desc[TIVX_KERNEL_LIDAR_OGMAP_CONFIGURATION_IDX];

        cfg_target_ptr =
            tivxMemShared2TargetPtr(&cfgObjDesc->mem_ptr);

        tivxMemBufferMap(cfg_target_ptr,
                         cfgObjDesc->mem_size,
                         VX_MEMORY_TYPE_HOST,
                         VX_READ_ONLY);

        cfgParams  = (tivx_lidar_ogmap_pfsd_params_t *)cfg_target_ptr;
        ogCfg      = &cfgParams->ogConfig;
        fsdPfsdCfg = &cfgParams->fsdPfsdConfig;

        memReqOg  = &memReq[TIVX_LIDAR_OGMAP_ALG_ID];
        memReqFsd = &memReq[TIVX_FSD_PFSD_ALG_ID];
        numMemReq = 1;

        /* Get the memory requirements. */
        algRetCode = PTK_Alg_LidarOgmapConfig(ogCfg, memReqOg);

        /* Based on the API specification, PTK_ALG_LIDAR_OGMAP_NUM_MEM_REQ_BLKS
         * blocks are expected.
         */
        if ((algRetCode != PTK_ALG_RET_SUCCESS) ||
            (memReqOg->numBlks != PTK_ALG_LIDAR_OGMAP_NUM_MEM_REQ_BLKS))
        {
            VX_PRINT(VX_ZONE_ERROR, "PTK_Alg_LidarOgmapConfig() failed!\n");
            status = VX_FAILURE;
        }

        if ((status == VX_SUCCESS) && cfgParams->fsdEnable)
        {
            /* Get the memory requirements for FSD Alg context. */
            algRetCode = PTK_Alg_FsdPfsdConfig(fsdPfsdCfg, memReqFsd);

            /* Based on the API specification, 1 block is expected. */
            if ((algRetCode != PTK_ALG_RET_SUCCESS) ||
                (memReqFsd->numBlks != 1))
            {
                VX_PRINT(VX_ZONE_ERROR, "PTK_Alg_FsdPfsdConfig() failed!\n");
                status = VX_FAILURE;
            }
            else
            {
                numMemReq++;
            }
        }
        
        if (status == VX_SUCCESS)
        {
            /* Based on the Lidar OGMAP API documentation, only the first three
             * entries of the 'memReq' will need to allocated, the fourth and
             * final entry indicates the size of the output buffer which will
             * not be alloated here. It will be done at the Application level.
             */
            memReqOg->numBlks = PTK_ALG_LIDAR_OGMAP_NUM_MEM_RSP_BLKS;

            /* Create alg object */
            algCntxt = tivxPtkAlgCommonCreate(kernel, memReq, numMemReq);

            if (NULL == algCntxt)
            {
                VX_PRINT(VX_ZONE_ERROR, "tivxPtkAlgCommonCreate() failed!\n");
                status = VX_FAILURE;
            }
            else
            {
                /* Initialize the OGMAP library. */
                algCntxt->algHandle[TIVX_LIDAR_OGMAP_ALG_ID] =
                    PTK_Alg_LidarOgmapInit(ogCfg, &algCntxt->memRsp[TIVX_LIDAR_OGMAP_ALG_ID]);

                if (!algCntxt->algHandle[TIVX_LIDAR_OGMAP_ALG_ID])
                {
                    VX_PRINT(VX_ZONE_ERROR, "PTK_Alg_LidarOgmapInit() failed!\n");
                    status = VX_FAILURE;
                }

                if ((status == VX_SUCCESS) && cfgParams->fsdEnable)
                {
                    /* Initialize the FSD/PFSD library. */
                    algCntxt->algHandle[TIVX_FSD_PFSD_ALG_ID] =
                        PTK_Alg_FsdPfsdInit(fsdPfsdCfg, &algCntxt->memRsp[TIVX_FSD_PFSD_ALG_ID]);

                    if (!algCntxt->algHandle[TIVX_FSD_PFSD_ALG_ID])
                    {
                        VX_PRINT(VX_ZONE_ERROR, "PTK_Alg_FsdPfsdInit() failed!\n");
                        status = VX_FAILURE;
                    }
                }
                else
                {
                    algCntxt->algHandle[TIVX_FSD_PFSD_ALG_ID] = NULL;
                }
            }
        }

        tivxMemBufferUnmap(cfg_target_ptr,
                           cfgObjDesc->mem_size,
                           VX_MEMORY_TYPE_HOST,
                           VX_READ_ONLY);
    }

    return status;
}

static vx_status VX_CALLBACK tivxLidarOgmapDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;

    if ( (num_params != TIVX_KERNEL_LIDAR_OGMAP_MAX_PARAMS)
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_OGMAP_CONFIGURATION_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_OGMAP_POINT_CLOUD_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_OGMAP_REFTR_AND_POSE_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_OGMAP_PFSD_OUT_DESC_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_OGMAP_ACC_OUTPUT_MAP_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_LIDAR_OGMAP_INST_OUTPUT_MAP_IDX])
    )
    {
        VX_PRINT(VX_ZONE_ERROR, "Interface parameter check failed.\n");

        status = VX_FAILURE;
    }
    else
    {
        tivx_ptk_alg_if_cntxt * algCntxt;
        uint32_t                size;

        /* Get the kernel context. */
        status = tivxGetTargetKernelInstanceContext(kernel,
                                                    (void **)&algCntxt,
                                                    &size);

        if ((VX_SUCCESS != status) ||
            (NULL == algCntxt)     ||
            (sizeof(tivx_ptk_alg_if_cntxt) != size))
        {
            VX_PRINT(VX_ZONE_ERROR, "Failed to get algorithm handle!\n");
            status = VX_FAILURE;
        }
        else
        {
            if (algCntxt->algHandle[TIVX_FSD_PFSD_ALG_ID])
            {
                /* De-Initialize the FSD/PFSD library. */
                PTK_Alg_FsdPfsdDeInit(algCntxt->algHandle[TIVX_FSD_PFSD_ALG_ID]);
            }

            /* De-Initialize the OGMAP library. */
            PTK_Alg_LidarOgmapDeInit(algCntxt->algHandle[TIVX_LIDAR_OGMAP_ALG_ID]);
        }

        status = tivxPtkAlgCommonDelete(kernel);
    }

    return status;
}

static vx_status VX_CALLBACK tivxLidarOgmapControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    tivx_ptk_alg_if_cntxt * algCntxt;
    uint32_t                size;
    vx_status               status;

    /* Get the kernel context. */
    status = tivxGetTargetKernelInstanceContext(kernel,
                                                (void **)&algCntxt,
                                                &size);

    if ((VX_SUCCESS != status) ||
        (NULL == algCntxt)     ||
        (sizeof(tivx_ptk_alg_if_cntxt) != size))
    {
        VX_PRINT(VX_ZONE_ERROR, "Failed to get algorithm handle!\n");
        status = VX_FAILURE;
    }
    else
    {
        if (node_cmd_id == TIVX_KERNEL_LIDAR_OGMAP_RESET)
        {
            PTK_AlgHandle   algHandle;
            int32_t         algRetCode;

            algHandle = algCntxt->algHandle[TIVX_LIDAR_OGMAP_ALG_ID];

            /* Issue the reset command. */
            algRetCode = PTK_Alg_LidarOgmapReset(algHandle);

            if (algRetCode != PTK_ALG_RET_SUCCESS)
            {
                VX_PRINT(VX_ZONE_ERROR, "PTK_Alg_LidarOgmapReset() failed!\n");

                status = VX_FAILURE;
            }
        }
    }

    return status;
}

void tivxAddTargetKernelLidarOgmap(void)
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
            vx_lidar_ogmap_target_kernel = tivxAddTargetKernelByName(
                                TIVX_KERNEL_LIDAR_OGMAP_NAME,
                                target_name[i],
                                tivxLidarOgmapProcess,
                                tivxLidarOgmapCreate,
                                tivxLidarOgmapDelete,
                                tivxLidarOgmapControl,
                                NULL);
        }
    }
}

void tivxRemoveTargetKernelLidarOgmap(void)
{
    vx_status status = VX_SUCCESS;

    status = tivxRemoveTargetKernel(vx_lidar_ogmap_target_kernel);
    if (status == VX_SUCCESS)
    {
        vx_lidar_ogmap_target_kernel = NULL;
    }
}


