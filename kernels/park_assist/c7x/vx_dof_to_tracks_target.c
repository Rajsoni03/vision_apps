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
#include "TI/tivx_debug.h"
#include "TI/tivx_park_assist.h"
#include "VX/vx.h"
#include "tivx_park_assist_kernels_priv.h"
#include "tivx_kernel_dof_to_tracks.h"
#include "TI/tivx_target_kernel.h"
#include "tivx_kernels_target_utils.h"
#include <math.h>

/*
 * processing of DOF field is done line by line
 * to avoid allocating worst case full
 * frame buffer for temporary storage
 */
typedef struct {

    /* points in image domain, to be normalized by VXLIB kernel*/
    vx_float32 *imagePoints;
    uint32_t imagePoints_size;

    /* points in normalized coordinates (output of VXLIB kernel)*/
    vx_float32 *normalizedPoints;
    uint32_t normalizedPoints_size;

} tivxDofToTracksObj;

static tivx_target_kernel vx_dof_to_tracks_target_kernel = NULL;

static vx_status VX_CALLBACK tivxDofToTracksProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxDofToTracksCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxDofToTracksDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxDofToTracksControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);

static vx_status tivxDofToTracksAllocMem(tivxDofToTracksObj *obj, uint32_t points_per_batch);
static vx_status tivxDofToTracksFreeMem(tivxDofToTracksObj *obj);

static vx_uint32 dof_get_confid(uint32_t dofValue);
static vx_float32 dof_get_u(uint32_t dofValue);
static vx_float32 dof_get_v(uint32_t dofValue);

static uint32_t VXLIB_distortedToNormalized_c32f_cn(
        float    src[],
        float    dst[],
        uint32_t numPoints,
        float    d2uTable[],
        uint32_t d2uTableSize,
        float    d2uTableStep,
        float    focalLengthX,
        float    focalLengthY,
        float    principalPointX,
        float    principalPointY);

static uint32_t VXLIB_undistortedToNormalized_c32f_cn(
        float    src[],
        float    dst[],
        uint32_t numPoints,
        float    focalLengthX,
        float    focalLengthY,
        float    principalPointX,
        float    principalPointY);

static vx_status VX_CALLBACK tivxDofToTracksProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    tivx_obj_desc_user_data_object_t *configuration_desc;
    tivx_obj_desc_image_t *input_dof_field_desc;
    tivx_obj_desc_lut_t *input_d2u_lut_desc;
    tivx_obj_desc_array_t *output_tracks_desc;

    uint32_t width, height, stride, outCapacity;
    uint32_t x, y, numKeep, i, totalPoints, keep, dofValue, size;
    uint8_t *dofPtr;
    tivx_triangulation_track_t *outPtr;
    tivx_dof_to_tracks_params_t *prms;
    tivxDofToTracksObj *obj;

    if ( (num_params != TIVX_KERNEL_DOF_TO_TRACKS_MAX_PARAMS)
        || (NULL == obj_desc[TIVX_KERNEL_DOF_TO_TRACKS_CONFIGURATION_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_DOF_TO_TRACKS_INPUT_DOF_FIELD_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_DOF_TO_TRACKS_OUTPUT_TRACKS_IDX])
    )
    {
        status = VX_FAILURE;
    }
    else
    {
        void *configuration_target_ptr;
        void *input_dof_field_target_ptr;
        void *input_d2u_lut_target_ptr = NULL;
        void *output_tracks_target_ptr;

        configuration_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_DOF_TO_TRACKS_CONFIGURATION_IDX];
        input_dof_field_desc = (tivx_obj_desc_image_t *)obj_desc[TIVX_KERNEL_DOF_TO_TRACKS_INPUT_DOF_FIELD_IDX];
        input_d2u_lut_desc = (tivx_obj_desc_lut_t *)obj_desc[TIVX_KERNEL_DOF_TO_TRACKS_INPUT_D2U_LUT_IDX];
        output_tracks_desc = (tivx_obj_desc_array_t *)obj_desc[TIVX_KERNEL_DOF_TO_TRACKS_OUTPUT_TRACKS_IDX];

        configuration_target_ptr = tivxMemShared2TargetPtr(&configuration_desc->mem_ptr);
        input_dof_field_target_ptr = tivxMemShared2TargetPtr(&input_dof_field_desc->mem_ptr[0]);
        if( input_d2u_lut_desc != NULL)
        {
            input_d2u_lut_target_ptr = tivxMemShared2TargetPtr(&input_d2u_lut_desc->mem_ptr);
        }
        output_tracks_target_ptr = tivxMemShared2TargetPtr(&output_tracks_desc->mem_ptr);

        tivxMemBufferMap(configuration_target_ptr,
           configuration_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);
        tivxMemBufferMap(input_dof_field_target_ptr,
           input_dof_field_desc->mem_size[0], VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);
        if( input_d2u_lut_desc != NULL)
        {
            tivxMemBufferMap(input_d2u_lut_target_ptr,
               input_d2u_lut_desc->mem_size, VX_MEMORY_TYPE_HOST,
                VX_READ_ONLY);
        }
        tivxMemBufferMap(output_tracks_target_ptr,
           output_tracks_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_WRITE_ONLY);

        status = tivxGetTargetKernelInstanceContext(kernel,
            (void **)&obj, &size);

        if ((VX_SUCCESS != status) || (NULL == obj) ||
            (sizeof(tivxDofToTracksObj) != size))
        {
            VX_PRINT(VX_ZONE_ERROR, "Invalid internal kernel object \n");
            status = VX_FAILURE;
        }

        if(status == VX_SUCCESS)
        {
            width = input_dof_field_desc->width;
            height = input_dof_field_desc->height;
            stride = input_dof_field_desc->imagepatch_addr[0].stride_y / sizeof(uint32_t);
            dofPtr = input_dof_field_target_ptr;

            prms = (tivx_dof_to_tracks_params_t*)configuration_target_ptr;

            outPtr = output_tracks_target_ptr;
            outCapacity = output_tracks_desc->capacity;

            /* checks */
            if ( NULL == outPtr || NULL == prms || NULL == dofPtr)
            {
                status = VX_FAILURE;
                VX_PRINT(VX_ZONE_ERROR, "Invalid memory pointer \n");
            }

            if (prms->roiType >= TIVX_DOF_TO_TRACKS_ROI_MAX)
            {
                status = VX_FAILURE;
                VX_PRINT(VX_ZONE_ERROR, "Invalid value (%d) for parameter roiType \n", prms->roiType);
            }
        }


        if(status==VX_SUCCESS)
        {
            totalPoints = 0;

            /* call kernel processing function */
            for(y=0; y<height; y+=prms->subsampleFactor)
            {
                numKeep = 0;
                for(x=0; x<width; x+= prms->subsampleFactor)
                {
                    float diffX = (float)x - prms->principalPointX;
                    float diffY = (float)y - prms->principalPointY;


                    dofValue = ((uint32_t *)dofPtr)[y*stride+x];

                    /* threshold on confidence */
                    keep = ( dof_get_confid(dofValue) >= prms->dofConfidThresh );

                    /* check if within ROI */
                    switch (prms->roiType)
                    {
                        case TIVX_DOF_TO_TRACKS_ROI_VERTICAL_CONE:
                            keep = keep & (diffY < (prms->roiPrm1*fabsf(diffX) + prms->roiPrm2));
                            break;
                        case TIVX_DOF_TO_TRACKS_ROI_ELLIPSE:
                            keep = keep & ((prms->roiPrm1 * diffX * diffX + diffY * diffY) < prms->roiPrm2);
                            break;
                        default:
                            keep = 0;
                    }

                    /* if all checks pass, add to list of tracks */
                    if (keep)
                    {
                        obj->imagePoints[4*numKeep+0] = (float)x;
                        obj->imagePoints[4*numKeep+1] = (float)y;
                        obj->imagePoints[4*numKeep+2] = (float)x+dof_get_u(dofValue);
                        obj->imagePoints[4*numKeep+3] = (float)y+dof_get_v(dofValue);
                        numKeep++;
                    }

                }

                /* undistort all points that passed ROI test */
                if (input_d2u_lut_desc != NULL)
                {
                    VXLIB_distortedToNormalized_c32f_cn(
                        obj->imagePoints,
                        obj->normalizedPoints,
                        2*numKeep, /* there are two xy pairs per point */
                        input_d2u_lut_target_ptr,
                        input_d2u_lut_desc->num_items,
                        prms->d2uTableStep,
                        prms->focalLength,
                        prms->focalLength,
                        prms->principalPointX,
                        prms->principalPointY);
                }
                else
                {
                    VXLIB_undistortedToNormalized_c32f_cn(
                             obj->imagePoints,
                             obj->normalizedPoints,
                             2*numKeep, /* there are two xy pairs per point */
                             prms->focalLength,
                             prms->focalLength,
                             prms->principalPointX,
                             prms->principalPointY);
                }

                /* make sure output buffer is not exceeded */
                if( (totalPoints+numKeep) <= outCapacity)
                {
                    tivx_triangulation_track_t *outTrack;

                    outTrack = &outPtr[totalPoints];

                    for(i=0; i<numKeep; i++)
                    {
                        outTrack[i].length = 2;
                        outTrack[i].view[0].x = obj->normalizedPoints[4*i + 0];
                        outTrack[i].view[0].y = obj->normalizedPoints[4*i + 1];
                        outTrack[i].view[1].x = obj->normalizedPoints[4*i + 2];
                        outTrack[i].view[1].y = obj->normalizedPoints[4*i + 3];
                    }
                }

                totalPoints += numKeep;
            }

            output_tracks_desc->num_items = totalPoints;
        }
        /* kernel processing function complete */
        if(status!=VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "processing error\n");
            output_tracks_desc->num_items = 0;
        }

        tivxMemBufferUnmap(configuration_target_ptr,
           configuration_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);
        tivxMemBufferUnmap(input_dof_field_target_ptr,
           input_dof_field_desc->mem_size[0], VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);
        if( input_d2u_lut_desc != NULL)
        {
            tivxMemBufferUnmap(input_d2u_lut_target_ptr,
               input_d2u_lut_desc->mem_size, VX_MEMORY_TYPE_HOST,
                VX_READ_ONLY);
        }
        tivxMemBufferUnmap(output_tracks_target_ptr,
           output_tracks_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_WRITE_ONLY);


    }

    return status;
}

static vx_status VX_CALLBACK tivxDofToTracksCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    tivxDofToTracksObj *obj = NULL;
    tivx_obj_desc_image_t *input_dof_field_desc;

    /*check input arguments*/
    if ( num_params != TIVX_KERNEL_DOF_TO_TRACKS_MAX_PARAMS
        || (NULL == obj_desc[TIVX_KERNEL_DOF_TO_TRACKS_CONFIGURATION_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_DOF_TO_TRACKS_INPUT_DOF_FIELD_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_DOF_TO_TRACKS_OUTPUT_TRACKS_IDX])
    )
    {
        status = VX_FAILURE;
    }

    if (VX_SUCCESS == status)
    {
        input_dof_field_desc = (tivx_obj_desc_image_t *)obj_desc[TIVX_KERNEL_DOF_TO_TRACKS_INPUT_DOF_FIELD_IDX];

        obj = tivxMemAlloc(sizeof(tivxDofToTracksObj), TIVX_MEM_EXTERNAL);
        /* allocate local memory */
        if (NULL != obj)
        {
            status = tivxDofToTracksAllocMem(obj, input_dof_field_desc->width);
        }
        else
        {
            status = VX_ERROR_NO_MEMORY;
        }


        if (VX_SUCCESS == status)
        {
            /* tell the OVX kernel object about the VXLIB configuration */
            tivxSetTargetKernelInstanceContext(kernel, obj,
                sizeof(tivxDofToTracksObj));
        }
        else
        {
            /* free all memory  */
            if (NULL != obj)
            {
                tivxDofToTracksFreeMem(obj);
                tivxMemFree(obj, sizeof(tivxDofToTracksObj), TIVX_MEM_EXTERNAL);
            }
        }
    }


    return status;
}

static vx_status VX_CALLBACK tivxDofToTracksDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    tivxDofToTracksObj *obj = NULL;
    uint32_t size;

    /* check parameters */
    if ( num_params != TIVX_KERNEL_DOF_TO_TRACKS_MAX_PARAMS
        || (NULL == obj_desc[TIVX_KERNEL_DOF_TO_TRACKS_CONFIGURATION_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_DOF_TO_TRACKS_INPUT_DOF_FIELD_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_DOF_TO_TRACKS_OUTPUT_TRACKS_IDX])
    )
    {
        status = VX_FAILURE;
    }

    /*free all memory */
    if (VX_SUCCESS == status)
    {
        status = tivxGetTargetKernelInstanceContext(kernel,
            (void **)&obj, &size);

        if ((VX_SUCCESS == status) && (NULL != obj) &&
            (sizeof(tivxDofToTracksObj) == size))
        {
            tivxDofToTracksFreeMem(obj);
            tivxMemFree(obj, sizeof(tivxDofToTracksObj), TIVX_MEM_EXTERNAL);
        }
        else
        {
            status = VX_FAILURE;
        }
    }

    return status;
}

static vx_status VX_CALLBACK tivxDofToTracksControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;

    /* < DEVELOPER_TODO: (Optional) Add any target kernel control code here (e.g. commands */
    /*                   the user can call to modify the processing of the kernel at run-time) > */

    return status;
}

void tivxAddTargetKernelDofToTracks(void)
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
        vx_dof_to_tracks_target_kernel = tivxAddTargetKernelByName(
                            TIVX_KERNEL_DOF_TO_TRACKS_NAME,
                            target_name,
                            tivxDofToTracksProcess,
                            tivxDofToTracksCreate,
                            tivxDofToTracksDelete,
                            tivxDofToTracksControl,
                            NULL);
    }
}

void tivxRemoveTargetKernelDofToTracks(void)
{
    vx_status status = VX_SUCCESS;

    status = tivxRemoveTargetKernel(vx_dof_to_tracks_target_kernel);
    if (status == VX_SUCCESS)
    {
        vx_dof_to_tracks_target_kernel = NULL;
    }
}

static vx_status tivxDofToTracksAllocMem(tivxDofToTracksObj *obj, uint32_t points_per_batch)
{
    vx_status status = VX_SUCCESS;

    obj->imagePoints_size = sizeof(vx_float32)*2*2*points_per_batch;
    obj->normalizedPoints_size = obj->imagePoints_size;

    obj->imagePoints = tivxMemAlloc(obj->imagePoints_size, TIVX_MEM_INTERNAL_L2);
    obj->normalizedPoints = tivxMemAlloc(obj->normalizedPoints_size, TIVX_MEM_INTERNAL_L2);
    if(obj->imagePoints==NULL||obj->normalizedPoints==NULL)
    {
        status = VX_ERROR_NO_MEMORY;
    }
    return status;
}

static vx_status tivxDofToTracksFreeMem(tivxDofToTracksObj *obj)
{
    vx_status status = VX_SUCCESS;

    if(obj->imagePoints)
    {
        tivxMemFree(obj->imagePoints, obj->imagePoints_size, TIVX_MEM_INTERNAL_L2);
    }
    if(obj->normalizedPoints)
    {
        tivxMemFree(obj->normalizedPoints, obj->normalizedPoints_size, TIVX_MEM_INTERNAL_L2);
    }
    return status;
}


static vx_uint32 dof_get_confid(uint32_t dofValue)
{
    /* confidence value from 0 (low confidence) to 15 (high confidence) */
    return (dofValue & 0xF);
}

static vx_float32 dof_get_u(uint32_t dofValue)
{
    int32_t value_i = ((int32_t)dofValue >> 16);
    vx_float32 value_f = ((vx_float32)value_i)/16.f;

    return value_f;
}

static vx_float32 dof_get_v(uint32_t dofValue)
{
    int32_t value_i = ((int32_t)dofValue << 16)>>20;
    vx_float32 value_f = ((vx_float32)value_i)/16.f;

    return value_f;
}

static uint32_t VXLIB_distortedToNormalized_c32f_cn(
        float    src[],
        float    dst[],
        uint32_t numPoints,
        float    d2uTable[],
        uint32_t d2uTableSize,
        float    d2uTableStep,
        float    focalLengthX,
        float    focalLengthY,
        float    principalPointX,
        float    principalPointY)
{
    uint32_t    pp;
    uint32_t    status = 0;
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
            ruDivRd = NAN;
        }
        else
        {
            indI = (int32_t)indF;
            a = indF - (float)indI;
            ruDivRd = (1.f - a)*d2uTable[indI] + a * d2uTable[indI + 1];
        }

        /* normalized point */
        dst[2*pp    ] = fXInv * diffX * ruDivRd;
        dst[2*pp + 1] = fYInv * diffY * ruDivRd;
    }

    return (status);
}

static uint32_t VXLIB_undistortedToNormalized_c32f_cn(
        float    src[],
        float    dst[],
        uint32_t numPoints,
        float    focalLengthX,
        float    focalLengthY,
        float    principalPointX,
        float    principalPointY)
{
    uint32_t    pp;
    uint32_t    status = 0;
    float       fXInv = 1.f/focalLengthX;
    float       fYInv = 1.f/focalLengthY;

    for (pp = 0; pp < numPoints; pp++)
    {
        float   diffX, diffY;
        /* normalized point */
        diffX = src[2*pp    ] - principalPointX;
        diffY = src[2*pp + 1] - principalPointY;
        dst[2*pp    ] = fXInv * diffX;
        dst[2*pp + 1] = fYInv * diffY;
    }

    return (status);
}
