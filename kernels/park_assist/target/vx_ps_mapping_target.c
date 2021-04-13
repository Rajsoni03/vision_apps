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
#include "TI/tivx_park_assist.h"
#include "VX/vx.h"
#include "tivx_park_assist_kernels_priv.h"
#include "tivx_kernel_ps_mapping.h"
#include "TI/tivx_target_kernel.h"
#include "tivx_kernels_target_utils.h"
#include <perception/utils/pnpoly.h>

/* processed parking spot obj struct */
typedef struct
{
    uint8_t     type;
    vx_float32  prob;

    uint8_t     displayed; // 0: not displayed, 1: displayed
    int32_t     count;
    PTK_Point   point[4];

} tivxFilteredParkingSpotObj;


typedef struct
{
    tivx_ps_mapping_input_t  *inputSpots;

    vx_float32 *normalizedPoints;
    uint32_t    normalizedPoints_size;

    PTK_Point  *worldPoints;
    uint32_t    worldPoints_size;

} tivxFrameParkingSpotObj;


typedef struct
{
    tivx_ps_mapping_config_t     *prms;
    tivx_ps_mapping_pose_t       *poses;

    tivxFrameParkingSpotObj       framePSObj;
    tivxFilteredParkingSpotObj   *filteredPSObj;

    PTK_Map                      *map;
    vx_uint32                     numSpots;

} tivxPsMappingObj;

static tivx_target_kernel vx_ps_mapping_target_kernel = NULL;

static vx_status VX_CALLBACK tivxPsMappingProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxPsMappingCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxPsMappingDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxPsMappingControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);

static vx_status tivxPsMappingAllocMem(tivxPsMappingObj *obj);
static vx_status tivxPsMappingFreeMem(tivxPsMappingObj *obj);
static void      getPsUndistorted(tivxPsMappingObj *obj,
        vx_float32  d2uTable[],
        vx_uint32   d2uTableSize);
static void getPsWorldPoses(tivxPsMappingObj *obj,  vx_float64 projMat[]);

void checkFrameParkingSpotSize(tivxPsMappingObj *obj, vx_int32 psIdx);
void postProcessParkingSpots(tivxPsMappingObj *obj);
vx_int32 findSameParkingSpots(tivx_ps_mapping_alg_parmas_t *params,
        tivxFilteredParkingSpotObj *psObj, PTK_Point* newSpot, vx_uint16 type, vx_uint32 numSpots);
void calculateSpotDistance(tivx_ps_mapping_alg_parmas_t *params, PTK_Point * pt1, PTK_Point* pt2, vx_float32 *dist);

void alg_ps_mapping_update_og(tivxPsMappingObj *obj);

// based on VXLIB_distortedToNormalized_c32f_cn
static vx_uint32 distortedToNormalized(
        vx_float32     src[],
        vx_float32     dst[],
        vx_uint32      numPoints,
        vx_float32     d2uTable[],
        vx_uint32      d2uTableSize,
        vx_float32     d2uTableStep,
        vx_float32     focalLengthX,
        vx_float32     focalLengthY,
        vx_float32     principalPointX,
        vx_float32     principalPointY);

static vx_status VX_CALLBACK tivxPsMappingProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    tivx_obj_desc_user_data_object_t *configuration_desc;
    tivx_obj_desc_user_data_object_t *dataptr_desc;
    tivx_obj_desc_lut_t *d2u_lut_desc;
    tivx_obj_desc_lut_t *projmat_desc;
    tivx_obj_desc_user_data_object_t *input_pose_desc;
    tivx_obj_desc_user_data_object_t *input_map_desc;
    tivx_obj_desc_user_data_object_t *output_map_desc;

    if ( (num_params != TIVX_KERNEL_PS_MAPPING_MAX_PARAMS)
        || (NULL == obj_desc[TIVX_KERNEL_PS_MAPPING_CONFIGURATION_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_PS_MAPPING_DATAPTR_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_PS_MAPPING_D2U_LUT_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_PS_MAPPING_PROJMAT_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_PS_MAPPING_INPUT_POSE_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_PS_MAPPING_INPUT_MAP_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_PS_MAPPING_OUTPUT_MAP_IDX])
    )
    {
        status = VX_FAILURE;
    }

    if(VX_SUCCESS == status)
    {
        configuration_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_PS_MAPPING_CONFIGURATION_IDX];
        dataptr_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_PS_MAPPING_DATAPTR_IDX];
        d2u_lut_desc = (tivx_obj_desc_lut_t *)obj_desc[TIVX_KERNEL_PS_MAPPING_D2U_LUT_IDX];
        projmat_desc = (tivx_obj_desc_lut_t *)obj_desc[TIVX_KERNEL_PS_MAPPING_PROJMAT_IDX];
        input_pose_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_PS_MAPPING_INPUT_POSE_IDX];
        input_map_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_PS_MAPPING_INPUT_MAP_IDX];
        output_map_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_PS_MAPPING_OUTPUT_MAP_IDX];

    }

    if(VX_SUCCESS == status)
    {

        void *configuration_target_ptr;
        void *dataptr_target_ptr;
        void *d2u_lut_target_ptr;
        void *projmat_target_ptr;
        void *input_pose_target_ptr;
        void *input_map_target_ptr;
        void *output_map_target_ptr;

        configuration_target_ptr = tivxMemShared2TargetPtr(&configuration_desc->mem_ptr);
        tivxMemBufferMap(configuration_target_ptr,
           configuration_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_READ_ONLY);

        dataptr_target_ptr = tivxMemShared2TargetPtr(&dataptr_desc->mem_ptr);
        tivxMemBufferMap(dataptr_target_ptr,
           dataptr_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_READ_ONLY);

        d2u_lut_target_ptr = tivxMemShared2TargetPtr(&d2u_lut_desc->mem_ptr);
        tivxMemBufferMap(d2u_lut_target_ptr,
           d2u_lut_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_READ_ONLY);

        projmat_target_ptr = tivxMemShared2TargetPtr(&projmat_desc->mem_ptr);
        tivxMemBufferMap(projmat_target_ptr,
           projmat_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_READ_ONLY);

        input_pose_target_ptr = tivxMemShared2TargetPtr(&input_pose_desc->mem_ptr);
        tivxMemBufferMap(input_pose_target_ptr,
           input_pose_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_READ_ONLY);

        input_map_target_ptr = tivxMemShared2TargetPtr(&input_map_desc->mem_ptr);
        tivxMemBufferMap(input_map_target_ptr,
           input_map_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_READ_ONLY);

        output_map_target_ptr = tivxMemShared2TargetPtr(&output_map_desc->mem_ptr);
        tivxMemBufferMap(output_map_target_ptr,
           output_map_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_WRITE_ONLY);


        /* call kernel processing function */
        PTK_Map *pMapIn;
        PTK_Map *pMapOut;

        tivxPsMappingObj *obj;
        uint32_t size;

        /* get alg handle */
        status = tivxGetTargetKernelInstanceContext(kernel,
            (void **)&obj, &size);

        if ((VX_SUCCESS != status) || (NULL == obj) ||
            (sizeof(tivxPsMappingObj) != size))
        {
            VX_PRINT(VX_ZONE_ERROR, " Invalid internal kernel object!\n");
            return VX_FAILURE;
        }

        obj->prms = (tivx_ps_mapping_config_t *)configuration_target_ptr;
        obj->framePSObj.inputSpots = (tivx_ps_mapping_input_t *)dataptr_target_ptr;
        obj->poses = (tivx_ps_mapping_pose_t *)input_pose_target_ptr;
        pMapIn = (PTK_Map *)input_map_target_ptr;
        pMapOut = (PTK_Map *)output_map_target_ptr;

        /******************************/
        /* run parking spot detection */
        /******************************/
        getPsUndistorted(obj, d2u_lut_target_ptr, d2u_lut_desc->num_items);
        getPsWorldPoses(obj, projmat_target_ptr);
        postProcessParkingSpots(obj);

        PTK_Map_copy(pMapOut, pMapIn);
        obj->map = pMapOut;

        alg_ps_mapping_update_og(obj);


        /* kernel processing function complete */
        tivxMemBufferUnmap(configuration_target_ptr,
           configuration_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);

        tivxMemBufferUnmap(dataptr_target_ptr,
           dataptr_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);

        tivxMemBufferUnmap(d2u_lut_target_ptr,
           d2u_lut_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);

        tivxMemBufferUnmap(projmat_target_ptr,
           projmat_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);

        tivxMemBufferUnmap(input_pose_target_ptr,
            input_pose_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);

        tivxMemBufferUnmap(input_map_target_ptr,
            input_map_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);

        tivxMemBufferUnmap(output_map_target_ptr,
            output_map_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_WRITE_ONLY);
    }


    return status;
}

static vx_status VX_CALLBACK tivxPsMappingCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;

    tivxPsMappingObj *obj;

    tivx_obj_desc_array_t *configuration_desc;
    void *configuration_target_ptr;

    if ( num_params != TIVX_KERNEL_PS_MAPPING_MAX_PARAMS
            || (NULL == obj_desc[TIVX_KERNEL_PS_MAPPING_CONFIGURATION_IDX])
            || (NULL == obj_desc[TIVX_KERNEL_PS_MAPPING_DATAPTR_IDX])
            || (NULL == obj_desc[TIVX_KERNEL_PS_MAPPING_D2U_LUT_IDX])
            || (NULL == obj_desc[TIVX_KERNEL_PS_MAPPING_PROJMAT_IDX])
            || (NULL == obj_desc[TIVX_KERNEL_PS_MAPPING_INPUT_POSE_IDX])
            || (NULL == obj_desc[TIVX_KERNEL_PS_MAPPING_INPUT_MAP_IDX])
            || (NULL == obj_desc[TIVX_KERNEL_PS_MAPPING_OUTPUT_MAP_IDX])
       )
    {
        return VX_FAILURE;
    }

    /* create object */
    obj = tivxMemAlloc(sizeof(tivxPsMappingObj), TIVX_MEM_EXTERNAL);
    if (NULL == obj)
    {
        return VX_ERROR_NO_MEMORY;
    }

    // initialize
    obj->numSpots = 0;

    /* read parameters */
    configuration_desc = (tivx_obj_desc_array_t *)obj_desc[TIVX_KERNEL_PS_MAPPING_CONFIGURATION_IDX];
    configuration_target_ptr = tivxMemShared2TargetPtr(&configuration_desc->mem_ptr);
    tivxMemBufferMap(configuration_target_ptr,
            configuration_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);

    obj->prms = (tivx_ps_mapping_config_t *)configuration_target_ptr;

    status = tivxPsMappingAllocMem(obj);

    if (VX_SUCCESS == status)
    {
        tivxSetTargetKernelInstanceContext(kernel, obj, sizeof(tivxPsMappingObj));
    } else
    {
        if (NULL != obj)
        {
            tivxPsMappingFreeMem(obj);
            tivxMemFree(obj, sizeof(tivxPsMappingObj), TIVX_MEM_EXTERNAL);
        }
    }

    tivxMemBufferUnmap(configuration_target_ptr,
            configuration_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);

    return status;
}

static vx_status VX_CALLBACK tivxPsMappingDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;

    tivxPsMappingObj *obj = NULL;
    uint32_t size  =0;

    /*free all memory */
    status = tivxGetTargetKernelInstanceContext(kernel,
        (void **)&obj, &size);

    if ((VX_SUCCESS == status) && (NULL != obj) &&
        (sizeof(tivxPsMappingObj) == size))
    {
        tivxPsMappingFreeMem(obj);
        tivxMemFree(obj, sizeof(tivxPsMappingObj), TIVX_MEM_EXTERNAL);
    }
    else
    {
        status = VX_FAILURE;
    }

    return status;
}

static vx_status VX_CALLBACK tivxPsMappingControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;

    /* < DEVELOPER_TODO: (Optional) Add any target kernel control code here (e.g. commands */
    /*                   the user can call to modify the processing of the kernel at run-time) > */

    return status;
}

void tivxAddTargetKernelPsMapping(void)
{
    vx_status status = VX_FAILURE;
    char target_name[TIVX_TARGET_MAX_NAME];
    vx_enum self_cpu;

    self_cpu = tivxGetSelfCpuId();

    if ( self_cpu == TIVX_CPU_ID_DSP1 )
    {
        strncpy(target_name, TIVX_TARGET_DSP1, TIVX_TARGET_MAX_NAME);
        status = VX_SUCCESS;
    }
    else
    if ( self_cpu == TIVX_CPU_ID_DSP2 )
    {
        strncpy(target_name, TIVX_TARGET_DSP2, TIVX_TARGET_MAX_NAME);
        status = VX_SUCCESS;
    }
    else
    if ( self_cpu == TIVX_CPU_ID_A72_0 )
    {
        strncpy(target_name, TIVX_TARGET_A72_0, TIVX_TARGET_MAX_NAME);
        status = VX_SUCCESS;
    }
    else
    if ( self_cpu == TIVX_CPU_ID_DSP_C7_1 )
    {
        strncpy(target_name, TIVX_TARGET_DSP_C7_1, TIVX_TARGET_MAX_NAME);
        status = VX_SUCCESS;
    }
    else
    {
        status = VX_FAILURE;
    }

    if (status == VX_SUCCESS)
    {
        vx_ps_mapping_target_kernel = tivxAddTargetKernelByName(
                            TIVX_KERNEL_PS_MAPPING_NAME,
                            target_name,
                            tivxPsMappingProcess,
                            tivxPsMappingCreate,
                            tivxPsMappingDelete,
                            tivxPsMappingControl,
                            NULL);
    }
}

void tivxRemoveTargetKernelPsMapping(void)
{
    vx_status status = VX_SUCCESS;

    status = tivxRemoveTargetKernel(vx_ps_mapping_target_kernel);
    if (status == VX_SUCCESS)
    {
        vx_ps_mapping_target_kernel = NULL;
    }
}

static vx_status tivxPsMappingAllocMem(tivxPsMappingObj *obj)
{
    vx_status status = VX_SUCCESS;

    /*
    obj->framePSObj.type = tivxMemAlloc(sizeof(vx_uint16)*obj->prms.ps_mapping_alg_params.maxSpotsFrame, TIVX_MEM_INTERNAL_L2);
    obj->framePSObj.prob = tivxMemAlloc(sizeof(vx_float32)*obj->prms.ps_mapping_alg_params.maxSpotsFrame, TIVX_MEM_INTERNAL_L2);

    obj->framePSObj.imagePoints_size      = sizeof(vx_float32)*obj->prms.ps_mapping_alg_params.maxSpotsFrame*4*2;
    */
    obj->framePSObj.normalizedPoints_size = sizeof(vx_float32)*obj->prms->ps_mapping_alg_params.maxSpotsFrame*4*2;
    obj->framePSObj.worldPoints_size      = sizeof(PTK_Point)*obj->prms->ps_mapping_alg_params.maxSpotsFrame*4;

    /*
    obj->framePSObj.imagePoints      = tivxMemAlloc(obj->framePSObj.imagePoints_size, TIVX_MEM_INTERNAL_L2);
    */
    obj->framePSObj.normalizedPoints = tivxMemAlloc(obj->framePSObj.normalizedPoints_size, TIVX_MEM_INTERNAL_L2);
    obj->framePSObj.worldPoints      = tivxMemAlloc(obj->framePSObj.worldPoints_size, TIVX_MEM_INTERNAL_L2);

    if(NULL == obj->framePSObj.normalizedPoints || NULL == obj->framePSObj.worldPoints)
    {
        status = VX_ERROR_NO_MEMORY;
    }

    /* allocate filtered parking spot obj */
    obj->filteredPSObj = tivxMemAlloc(sizeof(tivxFilteredParkingSpotObj) * obj->prms->ps_mapping_alg_params.maxTotalSpots, TIVX_MEM_INTERNAL_L2);
    if (NULL == obj->filteredPSObj)
    {
        status = VX_ERROR_NO_MEMORY;
    }

    return status;
}

static vx_status tivxPsMappingFreeMem(tivxPsMappingObj *obj)
{
    vx_status status = VX_SUCCESS;

    if(obj->framePSObj.normalizedPoints)
    {
        tivxMemFree(obj->framePSObj.normalizedPoints, obj->framePSObj.normalizedPoints_size, TIVX_MEM_INTERNAL_L2);
    }
    if(obj->framePSObj.worldPoints)
    {
        tivxMemFree(obj->framePSObj.worldPoints, obj->framePSObj.worldPoints_size, TIVX_MEM_INTERNAL_L2);
    }

    if (obj->filteredPSObj)
    {
        tivxMemFree(obj->filteredPSObj, sizeof(tivxFilteredParkingSpotObj) * obj->prms->ps_mapping_alg_params.maxTotalSpots, TIVX_MEM_INTERNAL_L2);
    }

    return status;
}


static void getPsUndistorted(tivxPsMappingObj *obj,
        vx_float32    d2uTable[],
        vx_uint32     d2uTableSize)
{

    vx_int32 psnum = obj->framePSObj.inputSpots->numParkingSpots;

    // convert to undistorted normalized points
    distortedToNormalized(obj->framePSObj.inputSpots->imagePoints,
            obj->framePSObj.normalizedPoints,
            psnum*4,
            d2uTable,
            d2uTableSize,
            obj->prms->ps_mapping_cam_params.d2uTableStep,
            obj->prms->ps_mapping_cam_params.focalLengthX,
            obj->prms->ps_mapping_cam_params.focalLengthY,
            obj->prms->ps_mapping_cam_params.principalPointX,
            obj->prms->ps_mapping_cam_params.principalPointY);
}


static vx_uint32 distortedToNormalized(
        vx_float32     src[],
        vx_float32     dst[],
        vx_uint32      numPoints,
        vx_float32     d2uTable[],
        vx_uint32      d2uTableSize,
        vx_float32     d2uTableStep,
        vx_float32     focalLengthX,
        vx_float32     focalLengthY,
        vx_float32     principalPointX,
        vx_float32     principalPointY)
{
    uint32_t    pp;
    uint32_t    status = VX_SUCCESS;
    float       d2uTableStepInv = 1/d2uTableStep;
    float       indFMax = d2uTableSize-1.f;
    float       fXInv = 1.f/focalLengthX;
    float       fYInv = 1.f/focalLengthY;

    for (pp = 0; pp < numPoints; pp++)
    {
        float   rdSq, diffX, diffY, ruDivRd, indF, a;
        int32_t indI;
        diffX = src[2*pp    ] - principalPointX;
        diffY = src[2*pp + 1] - principalPointY;
        /* NOTE: Opportunity for optimization :
         * diffX and diffY and their squares are actually computed twice,
         * once here and once in the node to determine ROI.
         */
        rdSq = diffX*diffX + diffY*diffY;

        /* table look-up */
        indF = rdSq * d2uTableStepInv;
        if (indF >= indFMax)
        {
            //ruDivRd = NAN;
            ruDivRd = d2uTable[(int32_t)indFMax];
        }
        else
        {
            indI = (int32_t)indF;
            //printf("indI:%d, %.4f, %.4f, %.4f\n", indI, rdSq, src[2*pp],  src[2*pp+1]    );
            a = indF - (float)indI;
            ruDivRd = (1.f - a)*d2uTable[indI] + a * d2uTable[indI + 1];
        }

        /* normalized point */
        dst[2*pp    ] = fXInv * diffX * ruDivRd;
        dst[2*pp + 1] = fYInv * diffY * ruDivRd;

    }

    return (status);
}


static void getPsWorldPoses(tivxPsMappingObj *obj, vx_float64 projMat[])
{

    int32_t i, j;
    int32_t psnum = obj->framePSObj.inputSpots->numParkingSpots;


    PTK_Point  outPt, worldPt;
    PTK_Point_d outPt_d, inPt_d;

    PTK_RigidTransform_d M_g_c;

    vx_float32* normalizedPoints = obj->framePSObj.normalizedPoints;
    PTK_Point * worldPoints = obj->framePSObj.worldPoints;

    /*
    M_g_c.M[0] = -0.0370342771198098;
    M_g_c.M[1] =  0.851584581647000;
    M_g_c.M[2] =  1.04023819588674;
    M_g_c.M[4] = -0.370699435133620;
    M_g_c.M[5] =  1.38687207305566;
    M_g_c.M[6] =  0.989116628168284;
    M_g_c.M[8] = -0.00000624740230713446;
    M_g_c.M[9] =  0.000254219649153303;
    M_g_c.M[10] = 0.000182713676343543;
    M_g_c.M[3] =  0.0;
    M_g_c.M[7] =  0.0;
    M_g_c.M[11] =  0.0;
    */
    for (i = 0; i < 12; i++)
    {
        M_g_c.M[i] = projMat[i];
    }


    PTK_RigidTransform M_w_g;
    PTK_RigidTransform_compose(&M_w_g, &obj->poses->M_w_e, &obj->poses->M_e_g);


    for (i = 0; i < psnum; i++)
    {
        //printf("parking spot %d based on H\n", i);
        for (j = 0; j < 4; j++)
        {
            //orgPt.x = imagePoints[i*8 + 2*j];
            //orgPt.y = imagePoints[i*8 + 2*j + 1];

            inPt_d.x = (double)normalizedPoints[i*8 + 2*j];
            inPt_d.y = (double)normalizedPoints[i*8 + 2*j + 1];
            inPt_d.z = (double)1.0;

            PTK_Point_d_transform(&outPt_d, &inPt_d, &M_g_c);
            //PTK_Point_transform(&outPt, &inPt, &M_g_c);


            outPt_d.x = outPt_d.x/outPt_d.z;
            outPt_d.y = outPt_d.y/outPt_d.z;
            outPt_d.z = 0;

            outPt.x = (float)(outPt_d.x*0.001);  // M_g_c is in mm
            outPt.y = (float)(outPt_d.y*0.001);
            outPt.z = (float)(outPt_d.z*0.001);

            PTK_Point_transform(&worldPt, &outPt, &M_w_g);
            worldPoints[i * 4 + j] = worldPt;
        }

        checkFrameParkingSpotSize(obj, i);

    }
}


void checkFrameParkingSpotSize(tivxPsMappingObj *obj, vx_int32 psIdx)
{


    float minHsize = obj->prms->ps_mapping_alg_params.minWidth;
    float maxHsize = obj->prms->ps_mapping_alg_params.maxWidth;
    float minVsize = obj->prms->ps_mapping_alg_params.minHeight;
    float maxVsize = obj->prms->ps_mapping_alg_params.maxHeight;


    float width, height;

    PTK_Point *point = &obj->framePSObj.worldPoints[psIdx * 4];

    // 1st width between pt1 and pt2
    width = sqrt( pow(point[0].x - point[1].x, 2) + pow(point[0].y - point[1].y, 2));
    if (width < minHsize || width > maxHsize)
    {
        obj->framePSObj.inputSpots->prob[psIdx] = 0.0;
        return;
    }

    // 2nd width between pt3 and pt4
    width = sqrt( pow(point[2].x - point[3].x, 2) + pow(point[2].y - point[3].y, 2));
    if (width < minHsize || width > maxHsize)
    {
        obj->framePSObj.inputSpots->prob[psIdx] = 0.0;
        return;
    }

    // 1st height between pt2 and pt3
    height = sqrt(pow(point[1].x - point[2].x, 2) + pow(point[1].y - point[2].y, 2) );
    if (height < minVsize || height > maxVsize)
    {
        obj->framePSObj.inputSpots->prob[psIdx] = 0.0;
        return;
    }

    // 2nd height between pt1 and pt4
    height = sqrt( pow(point[0].x - point[3].x, 2) + pow(point[0].y - point[3].y, 2) );
    if (height < minVsize || height > maxVsize)
    {
        obj->framePSObj.inputSpots->prob[psIdx] = 0.0;
        return;
    }
}



void postProcessParkingSpots(tivxPsMappingObj *obj)
{

    int32_t matchPsIdx;
    int32_t count;
    int32_t i;
    int32_t psnum = obj->framePSObj.inputSpots->numParkingSpots;

    PTK_Point *worldPoints = obj->framePSObj.worldPoints;
    tivxFilteredParkingSpotObj *filteredPSObj = obj->filteredPSObj;

    for (i = 0; i < psnum; i++)
    {
        if (obj->framePSObj.inputSpots->prob[i] >
            obj->prms->ps_mapping_alg_params.minProb)
        {
            matchPsIdx = findSameParkingSpots(&obj->prms->ps_mapping_alg_params,
                                              filteredPSObj,
                                              &worldPoints[i*4],
                                              obj->framePSObj.inputSpots->type[i],
                                              obj->numSpots);

            if (matchPsIdx == -1)
            {
                // create a new
                filteredPSObj[obj->numSpots].type = obj->framePSObj.inputSpots->type[i];
                filteredPSObj[obj->numSpots].prob = 1.0;
                filteredPSObj[obj->numSpots].count = 1;
                filteredPSObj[obj->numSpots].displayed = 0;

                filteredPSObj[obj->numSpots].point[0].x = worldPoints[i*4 + 0].x;
                filteredPSObj[obj->numSpots].point[0].y = worldPoints[i*4 + 0].y;
                filteredPSObj[obj->numSpots].point[1].x = worldPoints[i*4 + 1].x;
                filteredPSObj[obj->numSpots].point[1].y = worldPoints[i*4 + 1].y;
                filteredPSObj[obj->numSpots].point[2].x = worldPoints[i*4 + 2].x;
                filteredPSObj[obj->numSpots].point[2].y = worldPoints[i*4 + 2].y;
                filteredPSObj[obj->numSpots].point[3].x = worldPoints[i*4 + 3].x;
                filteredPSObj[obj->numSpots].point[3].y = worldPoints[i*4 + 3].y;

                // increases numSpots
                obj->numSpots++;

            }
            else if (filteredPSObj[matchPsIdx].displayed == 0)
            {
                // calculate average
                filteredPSObj[matchPsIdx].count += 1;
                filteredPSObj[matchPsIdx].displayed = 0;

                count = filteredPSObj[matchPsIdx].count;

                filteredPSObj[matchPsIdx].point[0].x =
                    (filteredPSObj[matchPsIdx].point[0].x * (count-1) + worldPoints[i*4 + 0].x) / count;
                filteredPSObj[matchPsIdx].point[0].y =
                    (filteredPSObj[matchPsIdx].point[0].y * (count-1) + worldPoints[i*4 + 0].y) / count;
                filteredPSObj[matchPsIdx].point[1].x =
                    (filteredPSObj[matchPsIdx].point[1].x * (count-1) + worldPoints[i*4 + 1].x) / count;
                filteredPSObj[matchPsIdx].point[1].y =
                    (filteredPSObj[matchPsIdx].point[1].y * (count-1) + worldPoints[i*4 + 1].y) / count;
                filteredPSObj[matchPsIdx].point[2].x =
                    (filteredPSObj[matchPsIdx].point[2].x * (count-1) + worldPoints[i*4 + 2].x) / count;
                filteredPSObj[matchPsIdx].point[2].y =
                    (filteredPSObj[matchPsIdx].point[2].y * (count-1) + worldPoints[i*4 + 2].y) / count;
                filteredPSObj[matchPsIdx].point[3].x =
                    (filteredPSObj[matchPsIdx].point[3].x * (count-1) + worldPoints[i*4 + 3].x) / count;
                filteredPSObj[matchPsIdx].point[3].y =
                    (filteredPSObj[matchPsIdx].point[3].y * (count-1) + worldPoints[i*4 + 3].y) / count;
            }
        }
    }
}

vx_int32 findSameParkingSpots(tivx_ps_mapping_alg_parmas_t *params,
        tivxFilteredParkingSpotObj *psObj, PTK_Point* newSpot, vx_uint16 type, vx_uint32 numSpots)
{
    int32_t i, matchIdx  = -1;
    float dist, minDist;

    minDist = 10000.0f;

    for (i = 0; i < numSpots; i++)
    {
        if (psObj[i].type == type)
        {
            calculateSpotDistance(params, psObj[i].point, newSpot, &dist);

            if (dist < minDist)
            {
                matchIdx = i;
                minDist = dist;
            }
        }
    }

    return matchIdx;
}

void calculateSpotDistance(tivx_ps_mapping_alg_parmas_t *params, PTK_Point * pt1, PTK_Point* pt2, vx_float32 *dist)
{

    int32_t i;
    int32_t numMatchVertice = 0;
    float d;

    *dist = 0.0;
    for (i = 0; i < 4; i++)
    {
        d =  sqrt( pow(pt1[0].x - pt2[0].x, 2) + pow(pt1[0].y - pt2[0].y, 2));
        *dist += d;
        if (d < params->matchTh)
        {
            numMatchVertice++;
        }
    }

    if (numMatchVertice < 3)
        *dist = 20000.0f;
}

void alg_ps_mapping_update_og(tivxPsMappingObj *obj)
{
    PTK_Grid * ogOut;
    int32_t    mark;
    int32_t    ii;
    int32_t    jj;
    int32_t    x;
    int32_t    y;

    ogOut = PTK_Map_get(obj->map,
                        obj->prms->ps_mapping_ogmap_params.occupancyGridId);

    if (ogOut == NULL)
    {
        PTK_printf("[%s:%d] PTK_Map_get() failed\n", __FUNCTION__, __LINE__);
        return;
    }

    for (ii = 0; ii < obj->numSpots; ii++)
    {
        int x_min = ogOut->config.xCells;
        int x_max = 0;
        int y_min = ogOut->config.yCells;
        int y_max = 0;
        float ptx[4], pty[4];

        // check probabilities
        if (obj->filteredPSObj[ii].prob < obj->prms->ps_mapping_alg_params.minProb)
        {
            continue;
        }

        if (obj->filteredPSObj[ii].displayed == 1)
        {
            continue;
        }

        if (obj->filteredPSObj[ii].count < obj->prms->ps_mapping_alg_params.minCountPerPS)
        {
            continue;
        }

        // display parking spots observed more than N times
        obj->filteredPSObj[ii].displayed = 1;

        for (jj = 0; jj < 4; jj++)
        {
             ptx[jj]= floor((obj->filteredPSObj[ii].point[jj].x - ogOut->config.xMin) / ogOut->config.xCellSize);
             pty[jj]= floor((obj->filteredPSObj[ii].point[jj].y - ogOut->config.yMin) / ogOut->config.yCellSize);

             if (ptx[jj] < x_min)
             {
                 x_min = ptx[jj];
             }

             if (pty[jj] < y_min)
             {
                 y_min = pty[jj];
             }

             if (ptx[jj] > x_max)
             {
                 x_max = ptx[jj];
             }

             if (pty[jj] > y_max)
             {
                 y_max = pty[jj];
             }
        }

        for(x = x_min; x < x_max; x++)
        {
            for(y = y_min; y < y_max; y++)
            {
                mark = pnpoly(4, ptx, pty, x, y);

                if (mark)
                {
                    if (obj->filteredPSObj[ii].type == 1)
                    {
                        PTK_Grid_clearb2d(ogOut, x, y, obj->prms->ps_mapping_ogmap_params.ogFlagOccupied);
                        PTK_Grid_setb2d(ogOut, x, y, obj->prms->ps_mapping_ogmap_params.ogFlagFree);
                    }
                    else if (obj->filteredPSObj[ii].type == 2)
                    {
                        PTK_Grid_clearb2d(ogOut,x, y, obj->prms->ps_mapping_ogmap_params.ogFlagFree);
                        PTK_Grid_setb2d(ogOut, x, y, obj->prms->ps_mapping_ogmap_params.ogFlagOccupied);
                    }
                }
            }
        }

    } //for (int ii = 0; ii < obj->numSpots; ii++)

}
