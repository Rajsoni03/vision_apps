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
#include "VX/vx.h"
#include "tivx_park_assist_kernels_priv.h"
#include "tivx_kernel_fused_ogmap.h"
#include "TI/tivx_target_kernel.h"
#include "tivx_kernels_target_utils.h"
#include <vx_ptk_alg_common.h>

#define TIVX_FUSED_OGMAP_ALG_ID           (0U)
#define TIVX_FSD_PFSD_ALG_ID              (TIVX_FUSED_OGMAP_ALG_ID + 1)
#define TIVX_MAX_FUSED_OGMAP_FSD_PFSD_IDS (TIVX_FSD_PFSD_ALG_ID + 1)

static tivx_target_kernel vx_fused_ogmap_target_kernel = NULL;

static vx_status VX_CALLBACK tivxFusedOgmapProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxFusedOgmapCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxFusedOgmapDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxFusedOgmapControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);


static vx_status tivxFusedOgmapProcessLocal(
    tivx_ptk_alg_if_cntxt *algCntxt,
    void *config_target_ptr,
    void *cameraMap_target_ptr,
    void *radarMap_target_ptr,
    void *lidarMap_target_ptr,
    void *reftr_and_pose_target_ptr,
    void *pfsd_out_desc_target_ptr,
    void *outMap_target_ptr
)
{
    tivx_fused_ogmap_pfsd_params_t *cfgParams;
    PTK_Alg_FusedOgmapParams       *ogCfg;
    PTK_Map                        *outMap;
    PTK_Map                        *cameraMap;
    PTK_Map                        *radarMap;
    PTK_Map                        *lidarMap;
    PTK_InsPoseAndRef              *posAndRef;
    PTK_Alg_FsdPfsdPSDesc          *pfsdDesc;
    PTK_INS_Record                 *curInsRec;
    PTK_RigidTransform_d           *Md_ecef_w;
    int32_t                         algRetCode;
    PTK_AlgHandle                   algHandle;
    vx_status                       vxStatus;

    vxStatus  = VX_SUCCESS;
    cfgParams = (tivx_fused_ogmap_pfsd_params_t *)config_target_ptr;
    cameraMap = (PTK_Map *)cameraMap_target_ptr;
    radarMap  = (PTK_Map *)radarMap_target_ptr;
    lidarMap  = (PTK_Map *)lidarMap_target_ptr;
    posAndRef = (PTK_InsPoseAndRef *)reftr_and_pose_target_ptr;
    pfsdDesc  = (PTK_Alg_FsdPfsdPSDesc*)pfsd_out_desc_target_ptr;
    outMap    = (PTK_Map *)outMap_target_ptr;

    curInsRec  = &posAndRef->curInsRec;
    Md_ecef_w  = &posAndRef->w2Ecef;

    ogCfg = &cfgParams->ogConfig;

    algHandle = algCntxt->algHandle[TIVX_FUSED_OGMAP_ALG_ID];

    algRetCode = PTK_Alg_FusedOgmapProcess(algHandle,
                                           ogCfg,
                                           cameraMap,
                                           radarMap,
                                           lidarMap,
                                           curInsRec,
                                           Md_ecef_w,
                                           outMap);

    if (algRetCode != PTK_ALG_RET_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "PTK_Alg_FusedOgmapProcess() failed.\n");
        vxStatus = VX_FAILURE;
    }

    algHandle = algCntxt->algHandle[TIVX_FSD_PFSD_ALG_ID];
    if ((vxStatus == VX_SUCCESS) && algHandle)
    {
        algRetCode = PTK_Alg_FsdPfsdProcess(algHandle,
                                            posAndRef,
                                            outMap,
                                            pfsdDesc);

        if (algRetCode != PTK_ALG_RET_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "PTK_Alg_FsdPfsdProcess() failed.\n");
            vxStatus = VX_FAILURE;
        }
    }

    return vxStatus;
}

static vx_status VX_CALLBACK tivxFusedOgmapProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    tivx_obj_desc_user_data_object_t *configuration_desc;
    tivx_obj_desc_user_data_object_t *ogmap_camera_desc;
    tivx_obj_desc_user_data_object_t *ogmap_radar_desc;
    tivx_obj_desc_user_data_object_t *ogmap_lidar_desc;
    tivx_obj_desc_user_data_object_t *reftr_and_pose_desc;
    tivx_obj_desc_user_data_object_t *pfsd_out_desc_desc;
    tivx_obj_desc_user_data_object_t *output_map_desc;

    if ( (num_params != TIVX_KERNEL_FUSED_OGMAP_MAX_PARAMS)
        || (NULL == obj_desc[TIVX_KERNEL_FUSED_OGMAP_CONFIGURATION_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_FUSED_OGMAP_REFTR_AND_POSE_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_FUSED_OGMAP_PFSD_OUT_DESC_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_FUSED_OGMAP_OUTPUT_MAP_IDX])
    )
    {
        status = VX_FAILURE;
    }
    else
    {
        void                  *configuration_target_ptr;
        void                  *ogmap_camera_target_ptr;
        void                  *ogmap_radar_target_ptr;
        void                  *ogmap_lidar_target_ptr;
        void                  *reftr_and_pose_target_ptr;
        void                  *pfsd_out_desc_target_ptr;
        void                  *output_map_target_ptr;
        tivx_ptk_alg_if_cntxt *algCntxt;
        uint32_t               size;

        configuration_desc = (tivx_obj_desc_user_data_object_t *)
            obj_desc[TIVX_KERNEL_FUSED_OGMAP_CONFIGURATION_IDX];

        ogmap_camera_desc = (tivx_obj_desc_user_data_object_t *)
            obj_desc[TIVX_KERNEL_FUSED_OGMAP_OGMAP_CAMERA_IDX];

        ogmap_radar_desc = (tivx_obj_desc_user_data_object_t *)
            obj_desc[TIVX_KERNEL_FUSED_OGMAP_OGMAP_RADAR_IDX];

        ogmap_lidar_desc = (tivx_obj_desc_user_data_object_t *)
            obj_desc[TIVX_KERNEL_FUSED_OGMAP_OGMAP_LIDAR_IDX];

        reftr_and_pose_desc = (tivx_obj_desc_user_data_object_t *)
            obj_desc[TIVX_KERNEL_FUSED_OGMAP_REFTR_AND_POSE_IDX];

        pfsd_out_desc_desc = (tivx_obj_desc_user_data_object_t *)
            obj_desc[TIVX_KERNEL_FUSED_OGMAP_PFSD_OUT_DESC_IDX];

        output_map_desc = (tivx_obj_desc_user_data_object_t *)
            obj_desc[TIVX_KERNEL_FUSED_OGMAP_OUTPUT_MAP_IDX];

        configuration_target_ptr =
            tivxMemShared2TargetPtr(&configuration_desc->mem_ptr);

        tivxMemBufferMap(configuration_target_ptr,
                         configuration_desc->mem_size,
                         VX_MEMORY_TYPE_HOST,
                         VX_READ_ONLY);

        ogmap_camera_target_ptr = NULL;
        ogmap_radar_target_ptr  = NULL;
        ogmap_lidar_target_ptr  = NULL;

        if (ogmap_camera_desc != NULL)
        {
            ogmap_camera_target_ptr =
                tivxMemShared2TargetPtr(&ogmap_camera_desc->mem_ptr);

            tivxMemBufferMap(ogmap_camera_target_ptr,
                             ogmap_camera_desc->mem_size,
                             VX_MEMORY_TYPE_HOST,
                             VX_READ_ONLY);
        }

        if (ogmap_radar_desc != NULL)
        {
            ogmap_radar_target_ptr =
                tivxMemShared2TargetPtr(&ogmap_radar_desc->mem_ptr);

            tivxMemBufferMap(ogmap_radar_target_ptr,
                             ogmap_radar_desc->mem_size,
                             VX_MEMORY_TYPE_HOST,
                             VX_READ_ONLY);
        }

        if (ogmap_lidar_desc != NULL)
        {
            ogmap_lidar_target_ptr =
                tivxMemShared2TargetPtr(&ogmap_lidar_desc->mem_ptr);

            tivxMemBufferMap(ogmap_lidar_target_ptr,
                             ogmap_lidar_desc->mem_size,
                             VX_MEMORY_TYPE_HOST,
                             VX_READ_ONLY);
        }

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

        output_map_target_ptr =
            tivxMemShared2TargetPtr(&output_map_desc->mem_ptr);

        tivxMemBufferMap(output_map_target_ptr,
                         output_map_desc->mem_size,
                         VX_MEMORY_TYPE_HOST,
                         VX_WRITE_ONLY);

        /* call kernel processing function */
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
            status = tivxFusedOgmapProcessLocal(algCntxt,
                                                configuration_target_ptr,
                                                ogmap_camera_target_ptr,
                                                ogmap_radar_target_ptr,
                                                ogmap_lidar_target_ptr,
                                                reftr_and_pose_target_ptr,
                                                pfsd_out_desc_target_ptr,
                                                output_map_target_ptr);
        }

        /* kernel processing function complete */

        tivxMemBufferUnmap(configuration_target_ptr,
                           configuration_desc->mem_size,
                           VX_MEMORY_TYPE_HOST,
                           VX_READ_ONLY);

        if (ogmap_camera_desc != NULL)
        {
            tivxMemBufferUnmap(ogmap_camera_target_ptr,
                               ogmap_camera_desc->mem_size,
                               VX_MEMORY_TYPE_HOST,
                               VX_READ_ONLY);
        }

        if (ogmap_radar_desc != NULL)
        {
            tivxMemBufferUnmap(ogmap_radar_target_ptr,
                               ogmap_radar_desc->mem_size,
                               VX_MEMORY_TYPE_HOST,
                               VX_READ_ONLY);
        }

        if (ogmap_lidar_desc != NULL)
        {
            tivxMemBufferUnmap(ogmap_lidar_target_ptr,
                               ogmap_lidar_desc->mem_size,
                               VX_MEMORY_TYPE_HOST,
                               VX_READ_ONLY);
        }

        tivxMemBufferUnmap(reftr_and_pose_target_ptr,
                           reftr_and_pose_desc->mem_size,
                           VX_MEMORY_TYPE_HOST,
                           VX_READ_ONLY);

        tivxMemBufferUnmap(pfsd_out_desc_target_ptr,
                           pfsd_out_desc_desc->mem_size,
                           VX_MEMORY_TYPE_HOST,
                           VX_WRITE_ONLY);

        tivxMemBufferUnmap(output_map_target_ptr,
                           output_map_desc->mem_size,
                           VX_MEMORY_TYPE_HOST,
                           VX_WRITE_ONLY);
    }

    return status;
}

static vx_status VX_CALLBACK tivxFusedOgmapCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;

    if ( (num_params != TIVX_KERNEL_FUSED_OGMAP_MAX_PARAMS)
        || (NULL == obj_desc[TIVX_KERNEL_FUSED_OGMAP_CONFIGURATION_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_FUSED_OGMAP_REFTR_AND_POSE_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_FUSED_OGMAP_PFSD_OUT_DESC_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_FUSED_OGMAP_OUTPUT_MAP_IDX])
    )
    {
        VX_PRINT(VX_ZONE_ERROR, "Interface parameter check failed.\n");
        status = VX_FAILURE;
    }

    if (VX_SUCCESS == status)
    {
        tivx_ptk_alg_if_cntxt             * algCntxt;
        tivx_fused_ogmap_pfsd_params_t    * cfgParams;
        const PTK_Alg_FusedOgmapParams    * ogCfg;
        const PTK_Alg_FsdPfsdParams       * fsdPfsdCfg;
        tivx_obj_desc_array_t             * configDesc;
        void                              * config_target_ptr;
        PTK_Api_MemoryReq                 * memReqOg;
        PTK_Api_MemoryReq                 * memReqFsd;
        PTK_Api_MemoryReq                   memReq[TIVX_MAX_FUSED_OGMAP_FSD_PFSD_IDS];
        uint32_t                            numMemReq;
        int32_t                             algRetCode;

        configDesc = (tivx_obj_desc_array_t *)
            obj_desc[TIVX_KERNEL_FUSED_OGMAP_CONFIGURATION_IDX];

        config_target_ptr =
            tivxMemShared2TargetPtr(&configDesc->mem_ptr);

        tivxMemBufferMap(config_target_ptr,
                         configDesc->mem_size,
                         VX_MEMORY_TYPE_HOST,
                         VX_READ_ONLY);

        cfgParams = (tivx_fused_ogmap_pfsd_params_t *)config_target_ptr;
        ogCfg      = &cfgParams->ogConfig;
        fsdPfsdCfg = &cfgParams->fsdPfsdConfig;

        memReqOg  = &memReq[TIVX_FUSED_OGMAP_ALG_ID];
        memReqFsd = &memReq[TIVX_FSD_PFSD_ALG_ID];
        numMemReq = 1;

        /* Get the memory requirements. */
        algRetCode = PTK_Alg_FusedOgmapConfig(ogCfg, memReqOg);

        /* Based on the API specification, PTK_ALG_FUSED_OGMAP_NUM_MEM_REQ_BLKS
         * blocks are expected.
         */
        if ((algRetCode != PTK_ALG_RET_SUCCESS) ||
            (memReqOg->numBlks !=PTK_ALG_FUSED_OGMAP_NUM_MEM_REQ_BLKS))
        {
            VX_PRINT(VX_ZONE_ERROR, "PTK_Alg_FusedOgmapConfig() failed!\n");
            status = VX_FAILURE;
        }

        if ((status == VX_SUCCESS) && cfgParams->fsdEnable)
        {
            /* Get the memory requirements for FSD Alg context. */
            algRetCode = PTK_Alg_FsdPfsdConfig(fsdPfsdCfg, memReqFsd);

            /* Based on the API specification, 3 blocks are expected. */
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
            memReqOg->numBlks = PTK_ALG_FUSED_OGMAP_NUM_MEM_RSP_BLKS;

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
                algCntxt->algHandle[TIVX_FUSED_OGMAP_ALG_ID] =
                    PTK_Alg_FusedOgmapInit(ogCfg, &algCntxt->memRsp[TIVX_FUSED_OGMAP_ALG_ID]);

                if (!algCntxt->algHandle[TIVX_FUSED_OGMAP_ALG_ID])
                {
                    VX_PRINT(VX_ZONE_ERROR, "PTK_Alg_RadarOgmapInit() failed!\n");
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

        tivxMemBufferUnmap(config_target_ptr,
                           configDesc->mem_size,
                           VX_MEMORY_TYPE_HOST,
                           VX_READ_ONLY);

    }

    return status;
}

static vx_status VX_CALLBACK tivxFusedOgmapDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;

    if ( (num_params != TIVX_KERNEL_FUSED_OGMAP_MAX_PARAMS)
        || (NULL == obj_desc[TIVX_KERNEL_FUSED_OGMAP_CONFIGURATION_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_FUSED_OGMAP_REFTR_AND_POSE_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_FUSED_OGMAP_PFSD_OUT_DESC_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_FUSED_OGMAP_OUTPUT_MAP_IDX])
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
            PTK_Alg_RadarOgmapDeInit(algCntxt->algHandle[TIVX_FUSED_OGMAP_ALG_ID]);
        }

        status = tivxPtkAlgCommonDelete(kernel);
    }

    return status;
}

static vx_status VX_CALLBACK tivxFusedOgmapControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;

    /* < DEVELOPER_TODO: (Optional) Add any target kernel control code here (e.g. commands */
    /*                   the user can call to modify the processing of the kernel at run-time) > */

    return status;
}

void tivxAddTargetKernelFusedOgmap(void)
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
            vx_fused_ogmap_target_kernel = tivxAddTargetKernelByName(
                                TIVX_KERNEL_FUSED_OGMAP_NAME,
                                target_name[i],
                                tivxFusedOgmapProcess,
                                tivxFusedOgmapCreate,
                                tivxFusedOgmapDelete,
                                tivxFusedOgmapControl,
                                NULL);
        }
    }
}

void tivxRemoveTargetKernelFusedOgmap(void)
{
    vx_status status = VX_SUCCESS;

    status = tivxRemoveTargetKernel(vx_fused_ogmap_target_kernel);
    if (status == VX_SUCCESS)
    {
        vx_fused_ogmap_target_kernel = NULL;
    }
}


