/*
 *
 * Copyright (c) 2017 Texas Instruments Incorporated
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

#include "app_encode_module.h"



vx_status app_init_encode(vx_context context, EncodeObj *encodeObj, char *objName)
{
    uint32_t max_bitstream_size;

    vx_status status = VX_SUCCESS;

    /* Create object for encode parameters */
    tivx_video_encoder_params_init(&(encodeObj->encode_params));
    encodeObj->configuration_obj = vxCreateUserDataObject(context,
            "tivx_video_encoder_params_t",
            sizeof(tivx_video_encoder_params_t),
            NULL);

    if (vxGetStatus((vx_reference)encodeObj->configuration_obj) != VX_SUCCESS)
    {
        APP_PRINTF("configuration_obj create failed\n");
        return VX_FAILURE;
    }

    /* Set bitstream format  */
    encodeObj->encode_params.bitstream_format = TIVX_BITSTREAM_FORMAT_H264;

    /* Copy object back onto main graph object ??? */
    vxCopyUserDataObject(encodeObj->configuration_obj, 0,
            sizeof(tivx_video_encoder_params_t),
            &(encodeObj->encode_params),
            VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);

    max_bitstream_size = ((uint32_t)(encodeObj->inWidth / 16)
            * (uint32_t)(encodeObj->inHeight / 16) * WORST_QP_SIZE)
                                          + ((encodeObj->inHeight >> 4) * CODED_BUFFER_INFO_SECTION_SIZE);

    encodeObj->bitstream_obj = vxCreateUserDataObject(context,
            "tivx_video_bitstream_t",
            sizeof(uint8_t) * max_bitstream_size,
            NULL);
    status = vxGetStatus((vx_reference)encodeObj->bitstream_obj);

    if(status == VX_SUCCESS)
    {
        /* Open file for output */
        char * filename = "encode_output.h264";
        snprintf(encodeObj->output_file, FILENAME_SIZE_MAX, "%s/%s", encodeObj->output_file_path, filename);

        encodeObj->out_fp = fopen(encodeObj->output_file, "wb");
        if (NULL == encodeObj->out_fp)
        {
            printf("app_create_graph: encoder: %s: Output file not opened!!!\n", encodeObj->output_file);
            printf("app_create_graph: encoder: Bitstream write to media skipped!\n");
        }
    }
    return status;
}

void app_deinit_encode(EncodeObj *encodeObj)
{
    if(NULL != encodeObj)
    {
        vxReleaseNode(&encodeObj->node);
        APP_PRINTF("releasing encodeNode done\n");
        encodeObj->node = NULL;

        if(NULL != encodeObj->configuration_obj)
        {
            vxReleaseUserDataObject(&encodeObj->configuration_obj);
            APP_PRINTF("releasing encode configuration object done\n");
            encodeObj->configuration_obj = NULL;
        }

        if(NULL != encodeObj->bitstream_obj)
        {
            vxReleaseUserDataObject(&encodeObj->bitstream_obj);
            APP_PRINTF("releasing encode bitstream object done\n");
            encodeObj->bitstream_obj = NULL;
        }

        if (NULL != encodeObj->out_fp)
        {
            fclose(encodeObj->out_fp);
            encodeObj->out_fp = NULL;
        }
    }

    return;
}

void app_delete_encode(EncodeObj *encodeObj)
{
    if(encodeObj->node != NULL)
    {
        vxReleaseNode(&encodeObj->node);
    }
    if(encodeObj->kernel != NULL)
    {
        vxRemoveKernel(encodeObj->kernel);
    }
    return;
}

vx_status app_create_graph_encode(vx_graph graph, EncodeObj *encodeObj, vx_image *output_image)
{

    vx_status status = VX_SUCCESS;

    encodeObj->node = tivxVideoEncoderNode(graph,
            encodeObj->configuration_obj, /* Encode parameters          (0) */
            output_image[0],              /* From Mosaic                (1) */
            encodeObj->bitstream_obj);    /* Bitstream to write to file (2) */
    APP_ASSERT_VALID_REF(encodeObj->node);

    vxSetNodeTarget(encodeObj->node, VX_TARGET_STRING, TIVX_TARGET_VENC1);

    vxSetReferenceName((vx_reference)encodeObj->node, "Encode_node");
    status = vxGetStatus((vx_reference)encodeObj->node);

    return (status);

}

vx_status writeEncodeOutput(EncodeObj *encodeObj)
{
    vx_status status;
    vx_map_id map_id;
    vx_size   bitstream_size;

    size_t num_read;
    uint8_t *bitstream;

    status = vxQueryUserDataObject(encodeObj->encoded_image,
            TIVX_USER_DATA_OBJECT_VALID_SIZE,
            &(bitstream_size), sizeof(vx_size));
    APP_ASSERT(status==VX_SUCCESS);

    status = vxMapUserDataObject(encodeObj->encoded_image, 0,
            bitstream_size,
            &map_id, (void*) &bitstream,
            VX_READ_ONLY, VX_MEMORY_TYPE_HOST, 0);
    APP_ASSERT(status==VX_SUCCESS);

    if (NULL != encodeObj->out_fp)
    {
        num_read = fwrite(bitstream, sizeof(uint8_t), bitstream_size, encodeObj->out_fp);
        if (bitstream_size != num_read)
        {
            APP_PRINTF("app_create_graph: encoder: %s: Wrote less than expected (%d < %d)!!!\n", encodeObj->output_file, (uint32_t)num_read, (uint32_t)(bitstream_size));
        }
    }

    vxUnmapUserDataObject(encodeObj->encoded_image, map_id);

    return(status);
}
