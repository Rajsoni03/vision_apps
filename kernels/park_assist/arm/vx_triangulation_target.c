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
#include "tivx_kernel_triangulation.h"
#include "TI/tivx_target_kernel.h"


#include "VLIB_triangulatePoints_types.h"
#include "VLIB_triangulatePoints_cn.h"


#define TRIANGULATION_MEM_LOC TIVX_MEM_EXTERNAL // TIVX_MEM_INTERNAL_L2

#define VX_TRIANGULATION_TRACKS_PER_BATCH  (128u)

#define DBG_TRIANG_KERNEL_VERSION 1 //0-VXLIB 1-VLIB //@todo  remove VLIB option once VXLIB is tested a bit more

#if DBG_TRIANG_KERNEL_VERSION==0
#define TRIANG_CAMERA_EXTRENSIC_PARAM_ASIZE VXLIB_TRIANG_CAMERA_EXTRENSIC_PARAM_ASIZE
#define TRIANG_MAX_POINTS_IN_TRACK          VXLIB_TRIANG_MAX_POINTS_IN_TRACK
#else
#define TRIANG_CAMERA_EXTRENSIC_PARAM_ASIZE VLIB_TRIANG_CAMERA_EXTRENSIC_PARAM_ASIZE
#define TRIANG_MAX_POINTS_IN_TRACK          VLIB_TRIANG_MAX_POINTS_IN_TRACK
#endif

#if DBG_TRIANG_KERNEL_VERSION==1
typedef float VXLIB_F32;
#define VXLIB_min(x, y) (((x) < (y)) ? (x) : (y))
#endif

typedef struct
{
    /*parameters*/
    uint32_t    tracksPerBatch; /* number of tracks in one batch (batch = one VXLIB process call) */

    /* local input, output and scratch buffers */
    VXLIB_F32   *curTrack;      /*input (tracks) */
    uint32_t    curTrack_size;
    VXLIB_F32   *extPrms;       /*input (poses) */
    uint32_t    extPrms_size;
    uint8_t     *trackLength;   /*input (track lengths) */
    uint32_t    trackLength_size;
    VXLIB_F32   *Xcam;          /*output (3d points) */
    uint32_t    Xcam_size;
#if DBG_TRIANG_KERNEL_VERSION==0
    int32_t     *validOut;      /*output (validity of 3d point)*/
#else
    uint8_t     *validOut;      /*output (validity of 3d point)*/
#endif
    uint32_t    validOut_size;
    VXLIB_F32   *scratch;       /*scratch */
    uint32_t    scratch_size;

} tivxTriangulationVXLIBObj;

static void vx_triangulation_target_convertPose(PTK_RigidTransform *M, VXLIB_F32 *extPrm);

/*for memory allocation*/
static vx_status vx_triangulation_alloc_local_mem(tivxTriangulationVXLIBObj *obj);
static void vx_triangulation_free_local_mem(tivxTriangulationVXLIBObj *obj);
static uint32_t vx_triangulation_target_getSize_curTrack(uint32_t maxNumTracksPerVXLIBCall);
static uint32_t vx_triangulation_target_getSize_extPrms();
static uint32_t vx_triangulation_target_getSize_trackLength(uint32_t maxNumTracksPerVXLIBCall);
static uint32_t vx_triangulation_target_getSize_Xcam(uint32_t maxNumTracksPerVXLIBCall);
static uint32_t vx_triangulation_target_getSize_scratch(uint32_t maxNumTracksPerVXLIBCall);
static uint32_t vx_triangulation_target_getSize_validOut(uint32_t maxNumTracksPerVXLIBCall);

#if DBG_TRIANG_KERNEL_VERSION==0
static uint32_t vx_triangulation_target_getNumVectors(uint32_t numTracks);
#endif

#if TIVX_KERNEL_TRIANGULATION_MAX_VIEWS > TRIANG_MAX_POINTS_IN_TRACK
#error "VXLIB kernel does not support that many views"
#endif

static tivx_target_kernel vx_triangulation_target_kernel = NULL;

static vx_status VX_CALLBACK tivxTriangulationProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxTriangulationCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxTriangulationDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxTriangulationControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);

static vx_status VX_CALLBACK tivxTriangulationProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    tivx_obj_desc_user_data_object_t *configuration_desc;
    tivx_obj_desc_array_t *input_track_desc;
    tivx_obj_desc_user_data_object_t *input_pose_desc;
    tivx_obj_desc_array_t            *output_point_desc;

    uint8_t length;
    uint32_t size, ff, ii, ll, ind;
#if DBG_TRIANG_KERNEL_VERSION==0
    uint32_t offset, iimod, iidiv;
#endif
    tivx_triangulation_track_t *pTracks;
    tivx_triangulation_pose_t *pPoses;
    tivx_triangulation_params_t *pPrms;
    PTK_Point *pPoints;
    tivxTriangulationVXLIBObj *obj;
    const uint32_t maxTrackLength = TIVX_KERNEL_TRIANGULATION_MAX_VIEWS;
    uint32_t numTracksInCurrentBatch = 0; /*number of tracks in current batch processed by VXLIB*/
    uint32_t numTracksToBeProcessed = 0; /*number of tracks that are to be processed by VXLIB*/
    uint32_t numTracksAlreadyProcessed= 0; /*number of tracks that have already been processed by VXLIB*/

    if ( (num_params != TIVX_KERNEL_TRIANGULATION_MAX_PARAMS)
        || (NULL == obj_desc[TIVX_KERNEL_TRIANGULATION_CONFIGURATION_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_TRIANGULATION_INPUT_TRACK_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_TRIANGULATION_INPUT_POSE_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_TRIANGULATION_OUTPUT_POINT_IDX])
    )
    {
        VX_PRINT(VX_ZONE_ERROR, "A required parameter is NULL\n");
        status = VX_FAILURE;
    }
    else
    {
        void *configuration_target_ptr;
        void *input_track_target_ptr;
        void *input_pose_target_ptr;
        void *output_point_target_ptr;

        configuration_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_TRIANGULATION_CONFIGURATION_IDX];
        input_track_desc = (tivx_obj_desc_array_t *)obj_desc[TIVX_KERNEL_TRIANGULATION_INPUT_TRACK_IDX];
        input_pose_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_TRIANGULATION_INPUT_POSE_IDX];
        output_point_desc = (tivx_obj_desc_array_t *)obj_desc[TIVX_KERNEL_TRIANGULATION_OUTPUT_POINT_IDX];

        configuration_target_ptr = tivxMemShared2TargetPtr(&configuration_desc->mem_ptr);
        input_track_target_ptr = tivxMemShared2TargetPtr(&input_track_desc->mem_ptr);
        input_pose_target_ptr = tivxMemShared2TargetPtr(&input_pose_desc->mem_ptr);
        output_point_target_ptr = tivxMemShared2TargetPtr(&output_point_desc->mem_ptr);

        tivxMemBufferMap(configuration_target_ptr,
           configuration_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_READ_ONLY);
        tivxMemBufferMap(input_track_target_ptr,
           input_track_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_READ_ONLY);
        tivxMemBufferMap(input_pose_target_ptr,
           input_pose_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_READ_ONLY);
        tivxMemBufferMap(output_point_target_ptr,
           output_point_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_WRITE_ONLY);


        pPrms = (tivx_triangulation_params_t *)configuration_target_ptr;
        pTracks = (tivx_triangulation_track_t *)input_track_target_ptr;
        pPoses = (tivx_triangulation_pose_t *)input_pose_target_ptr;
        pPoints = (PTK_Point *)output_point_target_ptr;

        /* checks */
        if ( NULL == pTracks || NULL == pPoses || NULL == pPrms || NULL == pPoints)
        {
            status = VX_FAILURE;
            VX_PRINT(VX_ZONE_ERROR, "Invalid memory pointer \n");
        }

        if (input_track_desc->num_items > input_track_desc->capacity)
        {
            status = VX_FAILURE;
            VX_PRINT(VX_ZONE_ERROR, "number of input tracks > input tracks array capacity \n");
        }

        if (output_point_desc->capacity < input_track_desc->num_items)
        {
            status = VX_FAILURE;
            VX_PRINT(VX_ZONE_ERROR, "output point array capacity < number of input tracks \n");
        }

        if (VX_SUCCESS == status)
        {
            status = tivxGetTargetKernelInstanceContext(kernel,
                (void **)&obj, &size);

            if ((VX_SUCCESS != status) || (NULL == obj) ||
                (sizeof(tivxTriangulationVXLIBObj) != size))
            {
                VX_PRINT(VX_ZONE_ERROR, "Invalid internal kernel object \n");
                status = VX_FAILURE;
            }
        }

        /* call kernel processing function */
        /* For each batch:
         * 1) reshape tracks into local buffer to match VXLIB kernel requirements
         * 2) call VXLIB kernel
         * 3) write result from local buffer into destination
         */
        if (VX_SUCCESS == status)
        {
            /* Reshape poses input to VXLIB format*/
            for (ff = 0; ff < maxTrackLength; ff++)
            {
                vx_triangulation_target_convertPose(
                    &(pPoses->view[ff]),
                    &(obj->extPrms[(TRIANG_MAX_POINTS_IN_TRACK - ff - 1)*TRIANG_CAMERA_EXTRENSIC_PARAM_ASIZE]));
            }
        }


        if (VX_SUCCESS == status)
        {
            /*for each batch of tracks*/
            numTracksToBeProcessed = input_track_desc->num_items;

            memset(obj->trackLength, 0, sizeof(uint8_t)*obj->tracksPerBatch);

#if DBG_TRIANG_KERNEL_VERSION==0
            for (ll = 0; ll < (numTracksToBeProcessed + obj->tracksPerBatch - 1) /
                 obj->tracksPerBatch; ll++)
            {
                /* initially, no tracks to be triangulated */
                memset(obj->trackLength, 0, sizeof(uint8_t)*obj->tracksPerBatch);

                /* how many tracks are in current batch */
                numTracksInCurrentBatch = VXLIB_min((numTracksToBeProcessed - numTracksAlreadyProcessed), obj->tracksPerBatch);

                /*set up curTracks*/
                for (ii = 0; ii < numTracksInCurrentBatch; ii++)
                {
                    ind = numTracksAlreadyProcessed + ii;
                    iimod = (ii % VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);
                    iidiv = (ii / VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);
                    offset = 2 * TRIANG_MAX_POINTS_IN_TRACK * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR * iidiv + iimod;

                    for (ff = 0; ff < maxTrackLength; ff++)
                    {
                        /* outTrackId ii, view ff, x-coordinate */
                        obj->curTrack[offset + (2*ff*VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR)] =
                                pTracks[ind].view[TRIANG_MAX_POINTS_IN_TRACK-ff-1].x;

                        /* outTrackId ii, view ff, y-coordinate */
                        obj->curTrack[offset + (2*ff*VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR) + VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR] =
                                pTracks[ind].view[TRIANG_MAX_POINTS_IN_TRACK-ff-1].y;
                    }

                    /* outTrackId ii, length */
                    length = (uint8_t)pTracks[ind].length;
                    length = length < 2 ? 0 : length; //need at least two tracks
                    length = length >  maxTrackLength ? 0 : length; //cannot use more than maxTrackLength tracks
                    obj->trackLength[iidiv * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + iimod] = length;
                }

                /*kernel call*/
                VXLIB_triangulatePoints_i32f_o32f_cn(obj->curTrack,
                                       obj->extPrms,
                                       obj->trackLength,
                                       pPrms->maxIters,
                                       obj->scratch,
                                       obj->tracksPerBatch,
                                       pPrms->enableHighPrecision,
                                       obj->Xcam,
                                       obj->validOut
                                       );

                /*set outputs*/
                for (ii = 0; ii < numTracksInCurrentBatch; ii++)
                {
                    ind = numTracksAlreadyProcessed + ii;
                    iimod = (ii % VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);
                    iidiv = (ii / VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);
                    offset = 3 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR * iidiv + iimod;

                    pPoints[ind].x = obj->Xcam[offset];
                    pPoints[ind].y = obj->Xcam[offset +   VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR];
                    pPoints[ind].z = obj->Xcam[offset + 2*VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR];
                    pPoints[ind].meta.w = (float)obj->validOut[iidiv * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + iimod];
                }

                numTracksAlreadyProcessed += numTracksInCurrentBatch;
            }
#else
            for (ll = 0; ll < (numTracksToBeProcessed + obj->tracksPerBatch - 1) /
                 obj->tracksPerBatch; ll++)
            {
                numTracksInCurrentBatch = VXLIB_min((numTracksToBeProcessed - numTracksAlreadyProcessed), obj->tracksPerBatch);


                /*set up curTracks*/
                for (ii = 0; ii < numTracksInCurrentBatch / 2; ii++)
                {
                    for (ff = 0; ff < maxTrackLength; ff++)
                    {
                        obj->curTrack[(ii * 24) + (TRIANG_MAX_POINTS_IN_TRACK - ff - 1) * 4 + 1] =
                            pTracks[numTracksAlreadyProcessed+ ii].view[ff].x;
                        obj->curTrack[(ii * 24) + (TRIANG_MAX_POINTS_IN_TRACK - ff - 1) * 4 + 3] =
                            pTracks[numTracksAlreadyProcessed+ ii].view[ff].y;
                    }
                }

                for (ii = numTracksInCurrentBatch / 2; ii < numTracksInCurrentBatch; ii++)
                {
                    for (ff = 0; ff < maxTrackLength; ff++)
                    {
                        obj->curTrack[((ii - numTracksInCurrentBatch / 2) * 24) + (TRIANG_MAX_POINTS_IN_TRACK - ff - 1) * 4 + 0] =
                            pTracks[numTracksAlreadyProcessed+ ii].view[ff].x;
                        obj->curTrack[((ii - numTracksInCurrentBatch / 2) * 24) + (TRIANG_MAX_POINTS_IN_TRACK - ff - 1) * 4 + 2] =
                            pTracks[numTracksAlreadyProcessed+ ii].view[ff].y;
                    }
                }

                /*set up lengths*/
                for (ii = 0; ii < numTracksInCurrentBatch / 2; ii++)
                {
                    length = (uint8_t)pTracks[numTracksAlreadyProcessed+ ii].length;
                    length = length < 2 ? 0 : length; //need at least two tracks
                    length = length >  maxTrackLength ? 0 : length; //cannot use more than maxTrackLength tracks
                    obj->trackLength[2 * ii + 1] = length;
                }
                for (ii = numTracksInCurrentBatch / 2; ii < numTracksInCurrentBatch; ii++)
                {
                    length = (uint8_t)pTracks[numTracksAlreadyProcessed+ ii].length;
                    length = length < 2 ? 0 : length; //need at least two tracks
                    length = length >  maxTrackLength ? 0 : length; //cannot use more than maxTrackLength tracks
                    obj->trackLength[2 * (ii - numTracksInCurrentBatch / 2)] = length;
                }

                /*kernel call*/
                VLIB_triangulatePoints_cn(obj->curTrack,
                                          obj->extPrms,
                                          obj->trackLength,
                                          pPrms->maxIters,
                                          obj->Xcam,
                                          obj->scratch,
                                          (((numTracksInCurrentBatch + 1) / 2) * 2),
                                          obj->validOut,
                                          pPrms->enableHighPrecision);

                /*set outputs*/
                /*Xcam*/
                for (ii = 0; ii < numTracksInCurrentBatch / 2; ii++)
                {
                    ind = numTracksAlreadyProcessed+ ii;
                    pPoints[ind].x = obj->Xcam[ii * 6 + 1];
                    pPoints[ind].y = obj->Xcam[ii * 6 + 3];
                    pPoints[ind].z = obj->Xcam[ii * 6 + 5];
                }
                for (ii = numTracksInCurrentBatch / 2; ii < numTracksInCurrentBatch; ii++)
                {
                    ind = numTracksAlreadyProcessed+ ii;
                    pPoints[ind].x = obj->Xcam[(ii - numTracksInCurrentBatch / 2) * 6 + 0];
                    pPoints[ind].y = obj->Xcam[(ii - numTracksInCurrentBatch / 2) * 6 + 2];
                    pPoints[ind].z = obj->Xcam[(ii - numTracksInCurrentBatch / 2) * 6 + 4];
                }

                /*validOut*/
                for (ii = 0; ii < numTracksInCurrentBatch / 2; ii++)
                {
                    ind = numTracksAlreadyProcessed+ ii;
                    pPoints[ind].meta.w = (float)obj->validOut[ii * 2 + 1];
                }
                for (ii = numTracksInCurrentBatch / 2; ii < numTracksInCurrentBatch; ii++)
                {
                    ind = numTracksAlreadyProcessed+ ii;
                    pPoints[ind].meta.w = (float)obj->validOut[(ii - numTracksInCurrentBatch / 2) * 2];
                }

                numTracksAlreadyProcessed+= numTracksInCurrentBatch;
            }
#endif

            if (numTracksAlreadyProcessed != numTracksToBeProcessed)
            {
                VX_PRINT(VX_ZONE_ERROR, "vx_kernel_triangulation: numTracksAlreadyProcessed != numTracksToBeProcessed\n");
                status = VX_FAILURE;
                output_point_desc->num_items = 0;
            }
            else
            {
                output_point_desc->num_items = numTracksToBeProcessed;
            }
        }
        if(status!=VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "processing error\n");
            output_point_desc->num_items = 0;
        }

        /* kernel processing function complete */

        tivxMemBufferUnmap(configuration_target_ptr,
           configuration_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);
        tivxMemBufferUnmap(input_track_target_ptr,
           input_track_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);
        tivxMemBufferUnmap(input_pose_target_ptr,
           input_pose_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);
        tivxMemBufferUnmap(output_point_target_ptr,
           output_point_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_WRITE_ONLY);


    }

    return status;
}

static vx_status VX_CALLBACK tivxTriangulationCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    tivxTriangulationVXLIBObj *obj = NULL;
    uint32_t i;

    /*check input arguments*/
    if (num_params != TIVX_KERNEL_TRIANGULATION_MAX_PARAMS)
    {
        status = VX_FAILURE;
    }
    else
    {
        for (i = 0U; i < TIVX_KERNEL_TRIANGULATION_MAX_PARAMS; i ++)
        {
            if (NULL == obj_desc[i])
            {
                status = VX_FAILURE;
                break;
            }
        }
    }

    if (VX_SUCCESS == status)
    {
        obj = tivxMemAlloc(sizeof(tivxTriangulationVXLIBObj), TIVX_MEM_EXTERNAL);
        /* allocate local memory */
        if (NULL != obj)
        {
            status = vx_triangulation_alloc_local_mem(obj);
        }
        else
        {
            status = VX_ERROR_NO_MEMORY;
        }


        if (VX_SUCCESS == status)
        {
            /* tell the OVX kernel object about the VXLIB configuration */
            tivxSetTargetKernelInstanceContext(kernel, obj,
                sizeof(tivxTriangulationVXLIBObj));
        }
        else
        {
            /* free all memory  */
            if (NULL != obj)
            {
                vx_triangulation_free_local_mem(obj);
                tivxMemFree(obj, sizeof(tivxTriangulationVXLIBObj), TIVX_MEM_EXTERNAL);
            }
        }
    }


    return status;
}

static vx_status VX_CALLBACK tivxTriangulationDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    tivxTriangulationVXLIBObj *obj = NULL;
    uint32_t i;
    uint32_t size;

    /* check parameters */
    if (num_params != TIVX_KERNEL_TRIANGULATION_MAX_PARAMS)
    {
        status = VX_FAILURE;
    }
    else
    {
        for (i = 0U; i < TIVX_KERNEL_TRIANGULATION_MAX_PARAMS; i++)
        {
            if (NULL == obj_desc[i])
            {
                status = VX_FAILURE;
                break;
            }
        }
    }

    /*free all memory */
    if (VX_SUCCESS == status)
    {
        status = tivxGetTargetKernelInstanceContext(kernel,
            (void **)&obj, &size);

        if ((VX_SUCCESS == status) && (NULL != obj) &&
            (sizeof(tivxTriangulationVXLIBObj) == size))
        {
            vx_triangulation_free_local_mem(obj);
            tivxMemFree(obj, sizeof(tivxTriangulationVXLIBObj), TIVX_MEM_EXTERNAL);
        }
        else
        {
            status = VX_FAILURE;
        }
    }

    return status;
}

static vx_status VX_CALLBACK tivxTriangulationControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;

    /* < DEVELOPER_TODO: (Optional) Add any target kernel control code here (e.g. commands */
    /*                   the user can call to modify the processing of the kernel at run-time) > */

    return status;
}

void tivxAddTargetKernelTriangulation(void)
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
    {
        status = VX_FAILURE;
    }

    if (status == VX_SUCCESS)
    {
        uint32_t    i;

        for (i = 0; i < num_targets; i++)
        {
            vx_triangulation_target_kernel = tivxAddTargetKernelByName(
                                TIVX_KERNEL_TRIANGULATION_NAME,
                                target_name[i],
                                tivxTriangulationProcess,
                                tivxTriangulationCreate,
                                tivxTriangulationDelete,
                                tivxTriangulationControl,
                                NULL);
        }
    }
}

void tivxRemoveTargetKernelTriangulation(void)
{
    vx_status status = VX_SUCCESS;

    status = tivxRemoveTargetKernel(vx_triangulation_target_kernel);
    if (status == VX_SUCCESS)
    {
        vx_triangulation_target_kernel = NULL;
    }
}

vx_status vx_triangulation_alloc_local_mem(tivxTriangulationVXLIBObj *obj)
{
    vx_status status;
    memset(obj, 0, sizeof(tivxTriangulationVXLIBObj));
    obj->tracksPerBatch = VX_TRIANGULATION_TRACKS_PER_BATCH;
    obj->curTrack_size = vx_triangulation_target_getSize_curTrack(obj->tracksPerBatch);
    obj->curTrack = tivxMemAlloc(obj->curTrack_size, TRIANGULATION_MEM_LOC);
    obj->extPrms_size = vx_triangulation_target_getSize_extPrms();
    obj->extPrms = tivxMemAlloc(obj->extPrms_size, TRIANGULATION_MEM_LOC);
    obj->trackLength_size = vx_triangulation_target_getSize_trackLength(obj->tracksPerBatch);
    obj->trackLength = tivxMemAlloc(obj->trackLength_size, TRIANGULATION_MEM_LOC);
    obj->Xcam_size = vx_triangulation_target_getSize_Xcam(obj->tracksPerBatch);
    obj->Xcam = tivxMemAlloc(obj->Xcam_size, TRIANGULATION_MEM_LOC);
    obj->validOut_size = vx_triangulation_target_getSize_validOut(obj->tracksPerBatch);
    obj->validOut = tivxMemAlloc(obj->validOut_size, TRIANGULATION_MEM_LOC);
    obj->scratch_size = vx_triangulation_target_getSize_scratch(obj->tracksPerBatch);
    obj->scratch = tivxMemAlloc(obj->scratch_size, TRIANGULATION_MEM_LOC);
    if (NULL ==  obj->curTrack   || NULL == obj->extPrms ||
        NULL == obj->trackLength || NULL == obj->Xcam ||
        NULL == obj->validOut    || NULL == obj->scratch)
    {
        status = VX_ERROR_NO_MEMORY;
    }
    else
    {
        status = VX_SUCCESS;
    }

    return (status);
}

void vx_triangulation_free_local_mem(tivxTriangulationVXLIBObj *obj)
{
    if (NULL !=  obj->curTrack)
    {
        tivxMemFree(obj->curTrack, obj->curTrack_size, TRIANGULATION_MEM_LOC);
    }
    if (NULL !=  obj->extPrms)
    {
        tivxMemFree(obj->extPrms, obj->extPrms_size, TRIANGULATION_MEM_LOC);
    }
    if (NULL !=  obj->trackLength)
    {
        tivxMemFree(obj->trackLength, obj->trackLength_size, TRIANGULATION_MEM_LOC);
    }
    if (NULL !=  obj->Xcam)
    {
        tivxMemFree(obj->Xcam, obj->Xcam_size, TRIANGULATION_MEM_LOC);
    }
    if (NULL !=  obj->validOut)
    {
        tivxMemFree(obj->validOut, obj->validOut_size, TRIANGULATION_MEM_LOC);
    }
    if (NULL !=  obj->scratch)
    {
        tivxMemFree(obj->scratch, obj->scratch_size, TRIANGULATION_MEM_LOC);
    }

    return;
}

/*convert pose3d to format used in VXLIB_trinagulate kernel*/
void vx_triangulation_target_convertPose(
        PTK_RigidTransform *M,
        VXLIB_F32 *extPrm)
{
#if DBG_TRIANG_KERNEL_VERSION==0
    /*@todo remove comment once verified
    extPrm[0] = M->R[6];
    extPrm[1] = M->R[0];
    extPrm[2] = M->R[3];
    extPrm[3] = M->R[7];
    extPrm[4] = M->R[1];
    extPrm[5] = M->R[4];
    extPrm[6] = M->R[8];
    extPrm[7] = M->t[2];
    extPrm[8] = M->R[2];
    extPrm[9] = M->R[5];
    extPrm[10] = M->t[0];
    extPrm[11] = M->t[1];
    extPrm[12] = M->R[6];
    extPrm[13] = M->R[7];
    extPrm[14] = M->R[8];
    extPrm[15] = M->t[2];
    */
    extPrm[0] = M->M[8];
    extPrm[1] = M->M[0];
    extPrm[2] = M->M[4];
    extPrm[3] = M->M[9];
    extPrm[4] = M->M[1];
    extPrm[5] = M->M[5];
    extPrm[6] = M->M[10];
    extPrm[7] = M->M[11];
    extPrm[8] = M->M[2];
    extPrm[9] = M->M[6];
    extPrm[10] = M->M[3];
    extPrm[11] = M->M[7];
    extPrm[12] = M->M[8];
    extPrm[13] = M->M[9];
    extPrm[14] = M->M[10];
    extPrm[15] = M->M[11];
#else
    extPrm[0] = M->M[8];
    extPrm[1] = M->M[0];
    extPrm[2] = M->M[4];
    extPrm[3] = M->M[9];
    extPrm[4] = M->M[1];
    extPrm[5] = M->M[5];
    extPrm[6] = M->M[10];
    extPrm[7] = M->M[11];
    extPrm[8] = M->M[2];
    extPrm[9] = M->M[6];
    extPrm[10] = M->M[3];
    extPrm[11] = M->M[7];
    extPrm[12] = M->M[8];
    extPrm[13] = M->M[8];
    extPrm[14] = M->M[9];
    extPrm[15] = M->M[9];
    extPrm[16] = M->M[10];
    extPrm[17] = M->M[10];
    extPrm[18] = M->M[11];
    extPrm[19] = M->M[11];
#endif
}

// 4*2*128*6 = 6144 bytes
uint32_t vx_triangulation_target_getSize_curTrack(uint32_t maxNumTracksPerKernelCall)
{
#if DBG_TRIANG_KERNEL_VERSION==0
    uint32_t maxNumVectorsPerKernelCall = vx_triangulation_target_getNumVectors(maxNumTracksPerKernelCall);
    return (sizeof(VXLIB_F32) *
            2 *
            TRIANG_MAX_POINTS_IN_TRACK *
            maxNumVectorsPerKernelCall * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR
            );
#else
    return (sizeof(VXLIB_F32) *
            2 *
            maxNumTracksPerKernelCall *
            TRIANG_MAX_POINTS_IN_TRACK);
#endif
}

// 4 * 20 * 6 = 480 bytes
uint32_t vx_triangulation_target_getSize_extPrms()
{
    return (sizeof(VXLIB_F32) *
            TRIANG_CAMERA_EXTRENSIC_PARAM_ASIZE *
            TRIANG_MAX_POINTS_IN_TRACK);
}

// 128 bytes
uint32_t vx_triangulation_target_getSize_trackLength(uint32_t maxNumTracksPerKernelCall)
{
#if DBG_TRIANG_KERNEL_VERSION==0
    uint32_t maxNumVectorsPerKernelCall = vx_triangulation_target_getNumVectors(maxNumTracksPerKernelCall);
    return (sizeof(uint8_t) * maxNumVectorsPerKernelCall * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);
#else
    return (sizeof(uint8_t) * maxNumTracksPerKernelCall);
#endif
}

// 4 * 3 * 128 = 1536 bytes
uint32_t vx_triangulation_target_getSize_Xcam(uint32_t maxNumTracksPerKernelCall)
{
#if DBG_TRIANG_KERNEL_VERSION==0
    uint32_t maxNumVectorsPerKernelCall = vx_triangulation_target_getNumVectors(maxNumTracksPerKernelCall);
    return (sizeof(VXLIB_F32) * 3 * maxNumVectorsPerKernelCall * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);
#else
    return (sizeof(VXLIB_F32) * 3 * maxNumTracksPerKernelCall);
#endif
}

// 1540 * 128 = 197120 bytes
uint32_t vx_triangulation_target_getSize_scratch(uint32_t maxNumTracksPerKernelCall)
{
#if DBG_TRIANG_KERNEL_VERSION==0
    uint32_t maxNumVectorsPerKernelCall = vx_triangulation_target_getNumVectors(maxNumTracksPerKernelCall);
    return (sizeof(uint8_t) * 6528 * maxNumVectorsPerKernelCall);
#else
    return (sizeof(uint8_t) * 1540 * maxNumTracksPerKernelCall);
#endif
}

// 128 bytes
uint32_t vx_triangulation_target_getSize_validOut(uint32_t maxNumTracksPerKernelCall)
{
#if DBG_TRIANG_KERNEL_VERSION==0
    uint32_t maxNumVectorsPerKernelCall = vx_triangulation_target_getNumVectors(maxNumTracksPerKernelCall);
    return (sizeof(int32_t) * maxNumVectorsPerKernelCall * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);
#else
    return (sizeof(uint8_t) * maxNumTracksPerKernelCall);
#endif
}

#if DBG_TRIANG_KERNEL_VERSION==0
uint32_t vx_triangulation_target_getNumVectors(uint32_t numTracks)
{
    return ((numTracks + VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR - 1) / VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);
}
#endif

