/*
 *
 * Copyright (c) 2020 Texas Instruments Incorporated
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

#include "app_capture_module.h"
#include <iss_sensors.h>
#include <iss_sensor_if.h>

vx_status app_init_capture(vx_context context, CaptureObj *captureObj, SensorObj *sensorObj, char *objName, int32_t bufq_depth)
{
    vx_status status = VX_SUCCESS;
    tivx_raw_image raw_image;
    vx_uint32 num_capt_instances;
    vx_int32 id, lane, q, ch;

    IssSensor_CreateParams *sensorParams = &sensorObj->sensorParams;

    if(sensorObj->num_cameras_enabled > 4)
    {
        num_capt_instances= 2;
    }
    else
    {
        num_capt_instances= 1;
    }

    tivx_capture_params_init(&captureObj->params);

    if (captureObj->enable_error_detection || captureObj->test_mode)
    {
        captureObj->params.timeout        = 90;
        captureObj->params.timeoutInitial = 500;
    }
    captureObj->params.numInst  = num_capt_instances;
    captureObj->params.numCh    = sensorObj->num_cameras_enabled;

    ch = 0;
    for(id = 0; id < num_capt_instances; id++)
    {
        captureObj->params.instId[id]                       = id;
        captureObj->params.instCfg[id].enableCsiv2p0Support = (uint32_t)vx_true_e;
        captureObj->params.instCfg[id].numDataLanes         = 4U;
        APP_PRINTF("captureObj->params.numDataLanes = %d \n", captureObj->params.instCfg[id].numDataLanes);

        for (lane = 0; lane < captureObj->params.instCfg[id].numDataLanes; lane++)
        {
            captureObj->params.instCfg[id].dataLanesMap[lane] = lane + 1;
            APP_PRINTF("captureObj->params.dataLanesMap[%d] = %d \n",
                        lane,
                        captureObj->params.instCfg[id].dataLanesMap[lane]);
        }
        for (q = 0; q < NUM_CAPT_CHANNELS; q++)
        {
            captureObj->params.chVcNum[ch]   = q;
            captureObj->params.chInstMap[ch] = id;
            ch++;
        }
    }

    if(status == VX_SUCCESS)
    {
        captureObj->config = vxCreateUserDataObject(context, "tivx_capture_params_t", sizeof(tivx_capture_params_t), &captureObj->params);
        status = vxGetStatus((vx_reference)captureObj->config);
    }

    if(status == VX_SUCCESS)
    {
        raw_image = tivxCreateRawImage(context, &sensorParams->sensorInfo.raw_params);
        status = vxGetStatus((vx_reference)raw_image);
    }
    for(q = 0; q < bufq_depth; q++)
    {
        if(status == VX_SUCCESS)
        {
            captureObj->raw_image_arr[q] = vxCreateObjectArray(context, (vx_reference)raw_image, sensorObj->num_cameras_enabled);
            status = vxGetStatus((vx_reference)captureObj->raw_image_arr[q]);
        }
    }
    tivxReleaseRawImage(&raw_image);

    if((captureObj->en_out_capture_write == 1) && (status == VX_SUCCESS))
    {
        char file_path[TIVX_FILEIO_FILE_PATH_LENGTH];
        char file_prefix[TIVX_FILEIO_FILE_PREFIX_LENGTH];

        strcpy(file_path, captureObj->output_file_path);
        captureObj->file_path   = vxCreateArray(context, VX_TYPE_UINT8, TIVX_FILEIO_FILE_PATH_LENGTH);
        status = vxAddArrayItems(captureObj->file_path, TIVX_FILEIO_FILE_PATH_LENGTH, &file_path[0], 1);

        if(status == VX_SUCCESS)
        {
            strcpy(file_prefix, "csix_raw_output");
            captureObj->file_prefix = vxCreateArray(context, VX_TYPE_UINT8, TIVX_FILEIO_FILE_PREFIX_LENGTH);
            status = vxAddArrayItems(captureObj->file_prefix, TIVX_FILEIO_FILE_PREFIX_LENGTH, &file_prefix[0], 1);
        }
        if(status == VX_SUCCESS)
        {
            captureObj->write_cmd = vxCreateUserDataObject(context, "tivxFileIOWriteCmd", sizeof(tivxFileIOWriteCmd), NULL);
            status = vxGetStatus((vx_reference)captureObj->write_cmd);
        }
    }
    else
    {
        captureObj->file_path   = NULL;
        captureObj->file_prefix = NULL;
        captureObj->write_node  = NULL;
        captureObj->write_cmd   = NULL;
    }

    /* If in test mode, send the test frame */
    if((captureObj->test_mode) && (status == VX_SUCCESS))
    {
        char test_data_paths[2][255] = {"/opt/vision_apps/test_data/psdkra/app_single_cam/IMX390_001/input2.raw",
                                            "/opt/vision_apps/test_data/psdkra/app_single_cam/AR0233_001/input2.raw"};
        vx_int32 bytes_read = 0;
        captureObj->error_frame_raw_image = read_error_image_raw(context, &(sensorParams->sensorInfo),
                                                test_data_paths[sensorObj->sensor_index],
                                                &bytes_read);
        APP_PRINTF("%d bytes were read by read_error_image_raw()\n", bytes_read);
        if (captureObj->error_frame_raw_image == NULL)
        {
            printf("read_error_image_raw returned a null pointer - test case failed\n");
            status = VX_FAILURE;
        }
        if((bytes_read <= 0) && (status == VX_SUCCESS))
        {
            status = tivxReleaseRawImage(&captureObj->error_frame_raw_image);
            captureObj->error_frame_raw_image = NULL;
        }
    }
    /* If we are doing error detection, just send a black frame to the dead sensor */
    else
    if((captureObj->enable_error_detection) && (status == VX_SUCCESS))
    {
        vx_map_id map_id;
        vx_rectangle_t rect;
        vx_imagepatch_addressing_t image_addr;
        void * data_ptr;

        rect.start_x = 0;
        rect.start_y = 0;
        rect.end_x = sensorParams->sensorInfo.raw_params.width;
        rect.end_y = sensorParams->sensorInfo.raw_params.height;

        captureObj->error_frame_raw_image = tivxCreateRawImage(context, &(sensorParams->sensorInfo.raw_params));

        status = tivxMapRawImagePatch((tivx_raw_image)captureObj->error_frame_raw_image, &rect, 0, &map_id, &image_addr, &data_ptr, VX_READ_ONLY, VX_MEMORY_TYPE_HOST, TIVX_RAW_IMAGE_ALLOC_BUFFER);

        if (status == VX_SUCCESS)
        {
            memset(data_ptr, 0x00, image_addr.stride_y*(sensorParams->sensorInfo.raw_params.height+sensorParams->sensorInfo.raw_params.meta_height_before+sensorParams->sensorInfo.raw_params.meta_height_after));
            status = tivxUnmapRawImagePatch((tivx_raw_image)captureObj->error_frame_raw_image, map_id);
        }
    }
    else
    {
        captureObj->error_frame_raw_image = NULL;
    }
    return (status);
}
/* This function is largely taken from single_cam_app :
    this function takes a raw_image that is unpopulated
    and populates it with the path set below */
tivx_raw_image read_error_image_raw(vx_context context,
                             IssSensor_Info *sensorInfo, char raw_image_fname[],
                             vx_int32 *bytes_read)
{
    /* Taking the test data from the app_single_cam demo to save space in the repo */
    FILE * fp;
    vx_uint32 width, height, i;
    vx_imagepatch_addressing_t image_addr;
    vx_rectangle_t rect;
    vx_map_id map_id;
    void *data_ptr;
    vx_uint32 num_bytes_per_pixel = 2; /*Supports only RAW 12b Unpacked format*/
    vx_uint32 num_bytes_read_from_file;
    tivx_raw_image raw_image = NULL;
    tivx_raw_image_format_t format;
    vx_uint32 imgaddr_width, imgaddr_height, imgaddr_stride;
    /* Provision for injecting RAW frame into the sensor */
    /* rect defines the outer bounds of the raw_image frame
        which will be defined */
    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x = sensorInfo->raw_params.width;
    rect.end_y = sensorInfo->raw_params.height;

    /* Nothing is being populated here - just an empty frame */
    raw_image = tivxCreateRawImage(context, &(sensorInfo->raw_params));

    APP_PRINTF("Reading test RAW image %s \n", raw_image_fname);
    fp = fopen(raw_image_fname, "rb");
    if(!fp)
    {
        printf("read_test_image_raw : Unable to open file %s\n", raw_image_fname);
        return NULL;
    }
    tivxQueryRawImage(raw_image, TIVX_RAW_IMAGE_WIDTH, &width, sizeof(vx_uint32));
    tivxQueryRawImage(raw_image, TIVX_RAW_IMAGE_HEIGHT, &height, sizeof(vx_uint32));
    tivxQueryRawImage(raw_image, TIVX_RAW_IMAGE_FORMAT, &format, sizeof(format));

    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x = width;
    rect.end_y = height;

    tivxMapRawImagePatch(raw_image,
        &rect,
        0,
        &map_id,
        &image_addr,
        &data_ptr,
        VX_READ_AND_WRITE,
        VX_MEMORY_TYPE_HOST,
        TIVX_RAW_IMAGE_PIXEL_BUFFER
        );

    if(!data_ptr)
    {
        APP_PRINTF("data_ptr is NULL \n");
        fclose(fp);
        return NULL;
    }
    num_bytes_read_from_file = 0;

    imgaddr_width  = image_addr.dim_x;
    imgaddr_height = image_addr.dim_y;
    imgaddr_stride = image_addr.stride_y;

    for(i=0;i<imgaddr_height;i++)
    {
        num_bytes_read_from_file += fread(data_ptr, 1, imgaddr_width*num_bytes_per_pixel, fp);
        data_ptr += imgaddr_stride;
    }

    tivxUnmapRawImagePatch(raw_image, map_id);

    fclose(fp);
    APP_PRINTF("%d bytes read from %s\n", num_bytes_read_from_file, raw_image_fname);
    *bytes_read = num_bytes_read_from_file;
    return raw_image;
}


void app_deinit_capture(CaptureObj *captureObj, vx_int32 bufq_depth)
{
    vx_int32 i;

    vxReleaseUserDataObject(&captureObj->config);
    for(i = 0; i < bufq_depth; i++)
    {
        vxReleaseObjectArray(&captureObj->raw_image_arr[i]);
    }

    tivxReleaseRawImage(&captureObj->error_frame_raw_image);

    if(captureObj->en_out_capture_write == 1)
    {
        vxReleaseArray(&captureObj->file_path);
        vxReleaseArray(&captureObj->file_prefix);
        vxReleaseUserDataObject(&captureObj->write_cmd);
    }
}

void app_delete_capture(CaptureObj *captureObj)
{
    if(captureObj->node != NULL)
    {
        vxReleaseNode(&captureObj->node);
    }
    if(captureObj->write_node != NULL)
    {
        vxReleaseNode(&captureObj->write_node);
    }
}

vx_status app_create_graph_capture(vx_graph graph, CaptureObj *captureObj)
{
    vx_status status = VX_SUCCESS;

    captureObj->node = tivxCaptureNode(graph, captureObj->config, captureObj->raw_image_arr[0]);
    status = vxGetStatus((vx_reference)captureObj->node);
    vxSetNodeTarget(captureObj->node, VX_TARGET_STRING, TIVX_TARGET_CAPTURE1);
    vxSetReferenceName((vx_reference)captureObj->node, "capture_node");

    if(captureObj->en_out_capture_write == 1)
    {
        APP_PRINTF("Adding Capture write node on graph .. \n");
        status = app_create_graph_capture_write_output(graph, captureObj);
        APP_PRINTF("Capture write node added! \n");
    }

    return status;
}

vx_status app_create_graph_capture_write_output(vx_graph graph, CaptureObj *captureObj)
{
    vx_status status = VX_SUCCESS;

    tivx_raw_image raw_img = (tivx_raw_image)vxGetObjectArrayItem(captureObj->raw_image_arr[0], 0);

    captureObj->write_node = tivxWriteRawImageNode(graph, raw_img, captureObj->file_path, captureObj->file_prefix);
    status = vxGetStatus((vx_reference)captureObj->write_node);
    vxSetNodeTarget(captureObj->write_node, VX_TARGET_STRING, TIVX_TARGET_A72_0);
    vxSetReferenceName((vx_reference)captureObj->write_node, "capture_write_node");

    vx_bool replicate[] = { vx_true_e, vx_false_e, vx_false_e};
    vxReplicateNode(graph, captureObj->write_node, replicate, 3);

    tivxReleaseRawImage(&raw_img);

    return (status);
}

vx_status app_send_cmd_capture_node(CaptureObj *captureObj, vx_uint32 start_frame, vx_uint32 num_frames, vx_uint32 num_skip)
{
    vx_status status = VX_SUCCESS;

    tivxFileIOWriteCmd write_cmd;

    write_cmd.start_frame = start_frame;
    write_cmd.num_frames = num_frames;
    write_cmd.num_skip = num_skip;

    status = vxCopyUserDataObject(captureObj->write_cmd, 0, sizeof(tivxFileIOWriteCmd),\
                  &write_cmd, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);

    if(status == VX_SUCCESS)
    {
        vx_reference refs[2];

        refs[0] = (vx_reference)captureObj->write_cmd;

        status = tivxNodeSendCommand(captureObj->write_node, TIVX_CONTROL_CMD_SEND_TO_ALL_REPLICATED_NODES,
                                 TIVX_FILEIO_CMD_SET_FILE_WRITE,
                                 refs, 1u);

        if(VX_SUCCESS != status)
        {
            printf("Capture node send command failed!\n");
        }

        APP_PRINTF("Capture node send command success!\n");
    }

    return (status);
}

vx_status app_send_error_frame(vx_context context, CaptureObj *captureObj, SensorObj *sensorObj)
{
    vx_status status = VX_SUCCESS;
    IssSensor_CreateParams *sensorParams = &sensorObj->sensorParams;

    captureObj->error_frame_raw_image = tivxCreateRawImage(context, &(sensorParams->sensorInfo.raw_params));;

    status = tivxCaptureRegisterErrorFrame(captureObj->node, (vx_reference)captureObj->error_frame_raw_image);

    return status;
}
