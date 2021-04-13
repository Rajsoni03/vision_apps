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
#include "tivx_kernel_radar_gtrack.h"
#include "TI/tivx_target_kernel.h"
#include "tivx_kernels_target_utils.h"
#include <vx_ptk_alg_common.h>

#define TIVX_RADAR_GRACK_ALG_ID           (0U)
#define TIVX_MAX_LIDAR_OGMAP_FSD_PFSD_IDS (TIVX_RADAR_GRACK_ALG_ID + 1)

static tivx_target_kernel vx_radar_gtrack_target_kernel = NULL;

static vx_status VX_CALLBACK tivxRadarGtrackProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxRadarGtrackCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxRadarGtrackDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxRadarGtrackControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);

static vx_status VX_CALLBACK tivxRadarGtrackProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;

    if ( (num_params != TIVX_KERNEL_RADAR_GTRACK_MAX_PARAMS)
        || (NULL == obj_desc[TIVX_KERNEL_RADAR_GTRACK_CONFIGURATION_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_RADAR_GTRACK_SENSOR_CONFIG_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_RADAR_GTRACK_RADAR_OBJ_DATA_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_RADAR_GTRACK_REFTR_AND_POSE_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_RADAR_GTRACK_TRACK_INFO_IDX])
    )
    {
        status = VX_FAILURE;
    }
    else
    {
        tivx_obj_desc_user_data_object_t *configuration_desc;
        tivx_obj_desc_user_data_object_t *sensor_config_desc;
        tivx_obj_desc_user_data_object_t *radar_obj_data_desc;
        tivx_obj_desc_user_data_object_t *reftr_and_pose_desc;
        tivx_obj_desc_user_data_object_t *track_info_desc;
        void                             *configuration_target_ptr;
        void                             *sensor_config_target_ptr;
        void                             *radar_obj_data_target_ptr;
        void                             *reftr_and_pose_target_ptr;
        void                             *track_info_target_ptr;
        tivx_ptk_alg_if_cntxt            *algCntxt;
        int32_t                           algRetCode;
        uint32_t                          size;

        configuration_desc = (tivx_obj_desc_user_data_object_t *)
            obj_desc[TIVX_KERNEL_RADAR_GTRACK_CONFIGURATION_IDX];

        sensor_config_desc = (tivx_obj_desc_user_data_object_t *)
            obj_desc[TIVX_KERNEL_RADAR_GTRACK_SENSOR_CONFIG_IDX];

        radar_obj_data_desc = (tivx_obj_desc_user_data_object_t *)
            obj_desc[TIVX_KERNEL_RADAR_GTRACK_RADAR_OBJ_DATA_IDX];

        reftr_and_pose_desc = (tivx_obj_desc_user_data_object_t *)
            obj_desc[TIVX_KERNEL_RADAR_GTRACK_REFTR_AND_POSE_IDX];

        track_info_desc = (tivx_obj_desc_user_data_object_t *)
            obj_desc[TIVX_KERNEL_RADAR_GTRACK_TRACK_INFO_IDX];

        configuration_target_ptr =
            tivxMemShared2TargetPtr(&configuration_desc->mem_ptr);

        tivxMemBufferMap(configuration_target_ptr,
                         configuration_desc->mem_size,
                         VX_MEMORY_TYPE_HOST,
                         VX_READ_ONLY);

        sensor_config_target_ptr = 
            tivxMemShared2TargetPtr(&sensor_config_desc->mem_ptr);

        tivxMemBufferMap(sensor_config_target_ptr,
                         sensor_config_desc->mem_size,
                         VX_MEMORY_TYPE_HOST,
                         VX_READ_ONLY);

        radar_obj_data_target_ptr =
            tivxMemShared2TargetPtr(&radar_obj_data_desc->mem_ptr);

        tivxMemBufferMap(radar_obj_data_target_ptr,
                         radar_obj_data_desc->mem_size,
                         VX_MEMORY_TYPE_HOST,
                         VX_READ_ONLY);

        reftr_and_pose_target_ptr =
            tivxMemShared2TargetPtr(&reftr_and_pose_desc->mem_ptr);

        tivxMemBufferMap(reftr_and_pose_target_ptr,
                         reftr_and_pose_desc->mem_size,
                         VX_MEMORY_TYPE_HOST,
                         VX_READ_ONLY);

        track_info_target_ptr =
            tivxMemShared2TargetPtr(&track_info_desc->mem_ptr);

        tivxMemBufferMap(track_info_target_ptr,
                         track_info_desc->mem_size,
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
            const PTK_Alg_RadarDetOutput    *doaData;
            const PTK_Alg_RadarSensorConfig *sensorCfg;
            const PTK_InsPoseAndRef         *poseAndRef;
            PTK_Alg_RadarGTrackTargetInfo   *targetInfo;
            PTK_AlgHandle                    algHandle;

            doaData = (PTK_Alg_RadarDetOutput*)
                            radar_obj_data_target_ptr;

            sensorCfg  = (PTK_Alg_RadarSensorConfig *) sensor_config_target_ptr;
            poseAndRef = (PTK_InsPoseAndRef *)reftr_and_pose_target_ptr;
            targetInfo = (PTK_Alg_RadarGTrackTargetInfo*) track_info_target_ptr;

            algHandle  = algCntxt->algHandle[TIVX_RADAR_GRACK_ALG_ID];
            algRetCode = PTK_Alg_RadarGTrackProcess(algHandle,
                                                    sensorCfg,
                                                    doaData,
                                                    poseAndRef,
                                                    targetInfo);


            if (algRetCode != PTK_ALG_RET_SUCCESS)
            {
                VX_PRINT(VX_ZONE_ERROR, "PTK_Alg_RadarGTrackProcess() failed.\n");
                status = VX_FAILURE;
            }
        }

        tivxMemBufferUnmap(configuration_target_ptr,
                           configuration_desc->mem_size,
                           VX_MEMORY_TYPE_HOST,
                           VX_READ_ONLY);

        tivxMemBufferUnmap(sensor_config_target_ptr,
                           sensor_config_desc->mem_size,
                           VX_MEMORY_TYPE_HOST,
                           VX_READ_ONLY);

        tivxMemBufferUnmap(radar_obj_data_target_ptr,
                           radar_obj_data_desc->mem_size,
                           VX_MEMORY_TYPE_HOST,
                           VX_READ_ONLY);

        tivxMemBufferUnmap(reftr_and_pose_target_ptr,
                           reftr_and_pose_desc->mem_size,
                           VX_MEMORY_TYPE_HOST,
                           VX_READ_ONLY);

        tivxMemBufferUnmap(track_info_target_ptr,
                           track_info_desc->mem_size,
                           VX_MEMORY_TYPE_HOST,
                           VX_WRITE_ONLY);
    }

    return status;
}

static vx_status VX_CALLBACK tivxRadarGtrackCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;

    if ( (num_params != TIVX_KERNEL_RADAR_GTRACK_MAX_PARAMS)
        || (NULL == obj_desc[TIVX_KERNEL_RADAR_GTRACK_CONFIGURATION_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_RADAR_GTRACK_SENSOR_CONFIG_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_RADAR_GTRACK_RADAR_OBJ_DATA_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_RADAR_GTRACK_REFTR_AND_POSE_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_RADAR_GTRACK_TRACK_INFO_IDX])
    )
    {
        status = VX_FAILURE;
    }
    else
    {
        tivx_ptk_alg_if_cntxt     * algCntxt;
        PTK_Alg_RadarGTrackParams * cfgParams;
        tivx_obj_desc_array_t     * cfgObjDesc;
        void                      * cfg_target_ptr;
        PTK_Api_MemoryReq           memReq;

        cfgObjDesc = (tivx_obj_desc_array_t *)
                obj_desc[TIVX_KERNEL_RADAR_GTRACK_CONFIGURATION_IDX];

        cfg_target_ptr =
            tivxMemShared2TargetPtr(&cfgObjDesc->mem_ptr);

        tivxMemBufferMap(cfg_target_ptr,
                         cfgObjDesc->mem_size,
                         VX_MEMORY_TYPE_HOST,
                         VX_READ_ONLY);

        cfgParams = (PTK_Alg_RadarGTrackParams *)cfg_target_ptr;

        /* No need for any memory allocation. */
        memReq.numBlks = 0;

        /* Create alg object */
        algCntxt = tivxPtkAlgCommonCreate(kernel, &memReq, 1);

        if (NULL == algCntxt)
        {
            VX_PRINT(VX_ZONE_ERROR, "tivxPtkAlgCommonCreate() failed!\n");
            status = VX_FAILURE;
        }
        else
        {
            /* Initialize the library. */
            algCntxt->algHandle[TIVX_RADAR_GRACK_ALG_ID] =
                PTK_Alg_RadarGTrackInit(cfgParams);

            if (!algCntxt->algHandle[TIVX_RADAR_GRACK_ALG_ID])
            {
                VX_PRINT(VX_ZONE_ERROR, "PTK_Alg_RadarGTrackInit() failed!\n");
                status = VX_FAILURE;
            }
        }

        tivxMemBufferUnmap(cfg_target_ptr,
                           cfgObjDesc->mem_size,
                           VX_MEMORY_TYPE_HOST,
                           VX_READ_ONLY);
    }

    return status;
}

static vx_status VX_CALLBACK tivxRadarGtrackDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;

    if ( (num_params != TIVX_KERNEL_RADAR_GTRACK_MAX_PARAMS)
        || (NULL == obj_desc[TIVX_KERNEL_RADAR_GTRACK_CONFIGURATION_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_RADAR_GTRACK_SENSOR_CONFIG_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_RADAR_GTRACK_RADAR_OBJ_DATA_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_RADAR_GTRACK_REFTR_AND_POSE_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_RADAR_GTRACK_TRACK_INFO_IDX])
    )
    {
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
            /* De-Initialize the OGMAP library. */
            PTK_Alg_RadarGTrackDeInit(algCntxt->algHandle[TIVX_RADAR_GRACK_ALG_ID]);
        }

        status = tivxPtkAlgCommonDelete(kernel);
    }

    return status;
}

static vx_status VX_CALLBACK tivxRadarGtrackControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;

    /* < DEVELOPER_TODO: (Optional) Add any target kernel control code here (e.g. commands */
    /*                   the user can call to modify the processing of the kernel at run-time) > */

    return status;
}

void tivxAddTargetKernelRadarGtrack(void)
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
            vx_radar_gtrack_target_kernel = tivxAddTargetKernelByName(
                                TIVX_KERNEL_RADAR_GTRACK_NAME,
                                target_name[i],
                                tivxRadarGtrackProcess,
                                tivxRadarGtrackCreate,
                                tivxRadarGtrackDelete,
                                tivxRadarGtrackControl,
                                NULL);
        }
    }
}

void tivxRemoveTargetKernelRadarGtrack(void)
{
    vx_status status = VX_SUCCESS;

    status = tivxRemoveTargetKernel(vx_radar_gtrack_target_kernel);
    if (status == VX_SUCCESS)
    {
        vx_radar_gtrack_target_kernel = NULL;
    }
}

/* The following functions are neede by the gtrack library. */
void *gtrack_alloc(uint32_t numElements, uint32_t sizeInBytes)
{
    uint32_t    size;
    void      * mem;

    size = numElements * sizeInBytes;

    mem = tivxMemAlloc(size, TIVX_MEM_EXTERNAL);

    return mem;
}

void gtrack_free(void *pFree, uint32_t sizeInBytes)
{
    tivxMemFree(pFree, sizeInBytes, TIVX_MEM_EXTERNAL);
}

void gtrack_log(GTRACK_VERBOSE_TYPE level, const char *format, ...)
{
}

