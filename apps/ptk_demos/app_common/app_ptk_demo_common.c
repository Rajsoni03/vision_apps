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
#include <TI/tivx.h>
#if !defined(TARGET_X86_64)
#include <app_init.h>
#endif
#include "app_ptk_demo_common.h"

#define PTKDEMO_MAX_NUM_TIVX_OBJ_HANDLES    (4U)

int32_t ptkdemo_find_slash(char        *path,
                           uint32_t     maxPathLen)
{
    size_t   len = strlen(path);
    uint32_t  i;

    if ( len > maxPathLen )
    {
        VX_PRINT(VX_ZONE_ERROR, "[%s] File path length [%d] exceeds the maximum length "
                   "allowed [%d]\n", __FUNCTION__, len, maxPathLen);
        return -1;
    }

    for ( i = 0; i < len ; i++ )
    {
        char c = path[len - i - 1];
        if ((c == '\\') || (c == '/') )
        {
            return len - i - 1;
        }
    }

    return -1;
}

int32_t ptkdemo_get_file_path(char        **filePath,
                              char         *pValueStr,
                              char         *globalBasePath,
                              char         *localBasePath,
                              uint32_t      maxPathLen)
{
    int32_t pos;
    int32_t status;

    status = 0;
    pos    = ptkdemo_find_slash(pValueStr, maxPathLen);

    if (globalBasePath == NULL)
    {
        /* Case 1:
         * Path of the form 'a/b/c/d' has been specified. We assume that
         * this is relative to localBasePath.
         *
         * This case needs the globalBasePath to se to NULL.
         */
        snprintf(*filePath, maxPathLen,
                 "%s/%s", localBasePath, pValueStr);
    }
    else if (pos >= 0 && globalBasePath != NULL)
    {
        /* Case 2:
         * Path of the form 'a/b/c/d' has been specified. we assume that
         * this is relative to globalBasePath.
         */
        snprintf(*filePath, maxPathLen,
                 "%s/%s", globalBasePath, pValueStr);
    }
    else if (pos < 0)
    {
        /* Case 3:
         * File of the form 'abcd' has been specified. We assume that
         * this is relative to localBasePath.
         */
        snprintf(*filePath, maxPathLen,
                 "%s/%s", localBasePath, pValueStr);
    }

    return status;
}

const char *app_common_get_coreName(const char *appCoreName)
{
    if (!strcmp(appCoreName, "TIVX_TARGET_DSP1"))
    {
        return TIVX_TARGET_DSP1;
    }
    else if (!strcmp(appCoreName, "TIVX_TARGET_DSP2"))
    {
        return TIVX_TARGET_DSP2;
    }
    else if (!strcmp(appCoreName, "TIVX_TARGET_DSP_C7_1"))
    {
        return TIVX_TARGET_DSP_C7_1;
    }
    else if (!strcmp(appCoreName, "TIVX_TARGET_EVE2"))
    {
        return TIVX_TARGET_EVE2;
    }
    else if (!strcmp(appCoreName, "TIVX_TARGET_EVE3"))
    {
        return TIVX_TARGET_EVE3;
    }
    else if (!strcmp(appCoreName, "TIVX_TARGET_EVE4"))
    {
        return TIVX_TARGET_EVE4;
    }
    else if (!strcmp(appCoreName, "TIVX_TARGET_A72_0"))
    {
        return TIVX_TARGET_A72_0;
    }
    else if (!strcmp(appCoreName, "TIVX_TARGET_A72_1"))
    {
        return TIVX_TARGET_A72_1;
    }
    else if (!strcmp(appCoreName, "TIVX_TARGET_A72_2"))
    {
        return TIVX_TARGET_A72_2;
    }
    else if (!strcmp(appCoreName, "TIVX_TARGET_A72_3"))
    {
        return TIVX_TARGET_A72_3;
    }
    else if (!strcmp(appCoreName, "TIVX_TARGET_IPU1_0"))
    {
        return TIVX_TARGET_IPU1_0;
    }
    else if (!strcmp(appCoreName, "TIVX_TARGET_IPU1_1"))
    {
        return TIVX_TARGET_IPU1_1;
    }
    else if (!strcmp(appCoreName, "TIVX_TARGET_IPU2"))
    {
        return TIVX_TARGET_IPU2;
    }

    return NULL;
}

#if defined(TARGET_X86_64)
int32_t appCommonInit()
{
    return 0;
}

int32_t appCommonDeInit()
{
    return 0;
}

#endif // defined(TARGET_X86_64)

int32_t appInit()
{
    int32_t status = 0;

    status = appCommonInit();

    if(status==0)
    {
        tivxInit();

#if !defined(TARGET_X86_64)
        tivxHostInit();
#endif // !defined(TARGET_X86_64)
    }

    return status;
}

int32_t appDeInit()
{
    int32_t status = 0;

#if !defined(TARGET_X86_64)
    tivxHostDeInit();
#endif // !defined(TARGET_X86_64)

    tivxDeInit();
    appCommonDeInit();

    return status;
}

vx_status ptkdemo_addParamByNodeIndex(vx_graph  graph,
                                      vx_node   node,
                                      vx_uint32 nodeParamIndex)
{
    vx_parameter    param;
    vx_status       vxStatus;

    vxStatus = VX_SUCCESS;
    param = vxGetParameterByIndex(node, nodeParamIndex);

    if (param == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "[%s:%d] vxGetParameterByIndex() failed\n",
                    __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxAddParameterToGraph(graph, param);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "[%s:%d] vxAddParameterToGraph() failed\n",
                        __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxReleaseParameter(&param);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "[%s:%d] vxReleaseParameter() failed\n",
                        __FUNCTION__, __LINE__);
        }
    }

    return vxStatus;
}

void * ptkdemo_getUserObjDataPayload(vx_user_data_object   obj)
{
    void       *data;
    vx_map_id   mapId;
    vx_status   vxStatus;

    /* Map the user object. */
    vxStatus = vxMapUserDataObject(obj,
                                   0, // offset
                                   0, // size = 0 ==> entire buffer
                                   &mapId,
                                   &data,
                                   VX_READ_AND_WRITE,
                                   VX_MEMORY_TYPE_HOST,
                                   0);

    if ((vxStatus != (vx_status)VX_SUCCESS) || (data == NULL))
    {
        VX_PRINT(VX_ZONE_ERROR, "[%s:%d] vxMapUserDataObject() failed.\n",
                   __FUNCTION__, __LINE__);

        return NULL;
    }

    /* Unmap the user object. */
    vxStatus = vxUnmapUserDataObject(obj, mapId);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "[%s:%d] vxUnmapUserDataObject() failed.\n",
                   __FUNCTION__, __LINE__);

        return NULL;
    }

    return data;
}

vx_status ptkdemo_load_vximage_from_yuvfile(vx_image image, char *filename)
{
    vx_status  vxStatus = (vx_status)VX_SUCCESS;

    vx_rectangle_t             rect;
    vx_imagepatch_addressing_t image_addr;
    vx_map_id                  map_id;
    void                     * data_ptr;
    vx_uint32                  img_width;
    vx_uint32                  img_height;
    vx_df_image                img_format;
    vx_int32                   j;

    FILE *fp= fopen(filename, "rb");
    if(fp==NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "# ERROR: Unable to open input file [%s]\n", filename);
        return(VX_FAILURE);
    }

    vxQueryImage(image, VX_IMAGE_WIDTH, &img_width, sizeof(vx_uint32));
    vxQueryImage(image, VX_IMAGE_HEIGHT, &img_height, sizeof(vx_uint32));
    vxQueryImage(image, VX_IMAGE_FORMAT, &img_format, sizeof(vx_df_image));

    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x = img_width;
    rect.end_y = img_height;

    // Copy Luma or Luma+Chroma
    vxStatus = vxMapImagePatch(image,
                               &rect,
                               0,
                               &map_id,
                               &image_addr,
                               &data_ptr,
                               VX_WRITE_ONLY,
                               VX_MEMORY_TYPE_HOST,
                               VX_NOGAP_X);

    for (j = 0; j < image_addr.dim_y; j++)
    {
        fread(data_ptr, 1, image_addr.dim_x*image_addr.stride_x, fp);
        data_ptr += image_addr.stride_y;
    }
    vxUnmapImagePatch(image, map_id);


    // Copy Chroma for NV12
    if (img_format == VX_DF_IMAGE_NV12)
    {
        vxStatus = vxMapImagePatch(image,
                                   &rect,
                                   1,
                                   &map_id,
                                   &image_addr,
                                   &data_ptr,
                                   VX_WRITE_ONLY,
                                   VX_MEMORY_TYPE_HOST,
                                   VX_NOGAP_X);

        for (j = 0; j < img_height/2; j++)
        {
            fread(data_ptr, 1, image_addr.dim_x, fp);
            data_ptr += image_addr.stride_y;
        }

        vxUnmapImagePatch(image, map_id);
    }

    fclose(fp);
    return vxStatus;
}

vx_status ptkdemo_load_vximage_from_sdefile(vx_image image, char *filename)
{
    vx_status  vxStatus = VX_SUCCESS;

    vx_rectangle_t             rect;
    vx_imagepatch_addressing_t image_addr;
    vx_map_id                  map_id;
    uint8_t                  * data_ptr;
    vx_uint32                  img_width;
    vx_uint32                  img_height;
    vx_df_image                img_format;
    vx_int32                   j;

    FILE *fp= fopen(filename, "rb");
    if(fp==NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "# ERROR: Unable to open input file [%s]\n", filename);
        return(VX_FAILURE);
    }

    // skip SDE header
    fseek(fp, SDE_FILE_HEADER_LEN*4, SEEK_SET);

    vxQueryImage(image, VX_IMAGE_WIDTH, &img_width, sizeof(vx_uint32));
    vxQueryImage(image, VX_IMAGE_HEIGHT, &img_height, sizeof(vx_uint32));
    vxQueryImage(image, VX_IMAGE_FORMAT, &img_format, sizeof(vx_df_image));

    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x   = img_width;
    rect.end_y   = img_height;
    
    // raw disparity data
    vxStatus = vxMapImagePatch(image,
                               &rect,
                               0,
                               &map_id,
                               &image_addr,
                               (void **) &data_ptr,
                               VX_WRITE_ONLY,
                               VX_MEMORY_TYPE_HOST,
                               VX_NOGAP_X);

    for (j = 0; j < image_addr.dim_y; j++)
    {
        fread(data_ptr, 1, image_addr.dim_x*image_addr.stride_x, fp);
        data_ptr += image_addr.stride_y;
    }
    vxUnmapImagePatch(image, map_id);

    fclose(fp);

    return vxStatus;
}

vx_status ptkdemo_load_vxtensor_from_file(vx_tensor tensor, char *filename)
{
    vx_status vxStatus = VX_SUCCESS;

    uint8_t * output_buffer = NULL;

    vx_map_id map_id;
    vx_size   num_dims;
    vx_size   start[MAX_TENSOR_DIMS];
    vx_size   output_strides[MAX_TENSOR_DIMS];
    vx_size   output_sizes[MAX_TENSOR_DIMS];
    int32_t   padT, padB, padL, padR;
    int32_t   i;
    uint8_t * ssOut;
    FILE    * fp = fopen(filename, "rb");

    if (fp == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "# ERROR: Unable to open input file [%s]\n", filename);
        return(VX_FAILURE);
    }

    // read header
    fread(output_sizes, sizeof(vx_size), 4, fp);
    fread(output_strides, sizeof(vx_size), 4, fp);
    fread(&padT, sizeof(int32_t), 1, fp);
    fread(&padB, sizeof(int32_t), 1, fp);
    fread(&padL, sizeof(int32_t), 1, fp);
    fread(&padR, sizeof(int32_t), 1, fp);
    
    start[0] = start[1] = start[2] = 0;
    vxQueryTensor(tensor, VX_TENSOR_NUMBER_OF_DIMS, &num_dims, sizeof(vx_size));

    if (num_dims >= MAX_TENSOR_DIMS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Invalid number of dims read [%d]", num_dims);
        fclose(fp);
        return(VX_FAILURE);
    }

    vxStatus = tivxMapTensorPatch(tensor, num_dims, start, output_sizes,
                                  &map_id, output_strides,
                                  (void **)&output_buffer, VX_WRITE_ONLY,
                                  VX_MEMORY_TYPE_HOST);

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        // read tensor data (semantic segmentation map)
        ssOut = (uint8_t *)output_buffer;
        for (i = 0; i < output_sizes[1]; i++)
        {
            fread((void*)ssOut, 1,  output_sizes[0], fp);
            ssOut += output_sizes[0];
        }

        tivxUnmapTensorPatch(tensor, map_id);
    }

    fclose(fp);

    return vxStatus;
}

vx_status ptkdemo_save_vximage_to_yuvfile(vx_image image, char *filename)
{
    FILE                       *fp;
    void                       *data_ptr;
    vx_rectangle_t              rect;
    vx_imagepatch_addressing_t  image_addr;
    vx_map_id                   map_id;
    vx_uint32                   img_width;
    vx_uint32                   img_height;
    vx_df_image                 img_format;
    vx_int32                    j;
    vx_status                   vxStatus;

    fp= fopen(filename, "wb");

    if (fp == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "# ERROR: Unable to open input file [%s]\n", filename);
        return(VX_FAILURE);
    }

    vxQueryImage(image, VX_IMAGE_WIDTH, &img_width, sizeof(vx_uint32));
    vxQueryImage(image, VX_IMAGE_HEIGHT, &img_height, sizeof(vx_uint32));
    vxQueryImage(image, VX_IMAGE_FORMAT, &img_format, sizeof(vx_df_image));

    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x = img_width;
    rect.end_y = img_height;

    // Write Luma or Luma+Chroma
    vxStatus = vxMapImagePatch(image,
                               &rect,
                               0,
                               &map_id,
                               &image_addr,
                               &data_ptr,
                               VX_READ_ONLY,
                               VX_MEMORY_TYPE_HOST,
                               VX_NOGAP_X);

    for (j = 0; j < image_addr.dim_y; j++)
    {
        fwrite(data_ptr, 1, image_addr.dim_x*image_addr.stride_x, fp);
        data_ptr += image_addr.stride_y;
    }
    vxUnmapImagePatch(image, map_id);

    // Write Chroma for NV12
    if (img_format == VX_DF_IMAGE_NV12)
    {
        vxStatus = vxMapImagePatch(image,
                                   &rect,
                                   1,
                                   &map_id,
                                   &image_addr,
                                   &data_ptr,
                                   VX_READ_ONLY,
                                   VX_MEMORY_TYPE_HOST,
                                   VX_NOGAP_X);

        for (j = 0; j < img_height/2; j++)
        {
            fwrite(data_ptr, 1, image_addr.dim_x, fp);
            data_ptr += image_addr.stride_y;
        }
        vxUnmapImagePatch(image, map_id);
    }

    fclose(fp);

    return vxStatus;
}

vx_status ptkdemo_save_vximage_to_sdefile(vx_image image, char *filename)
{
    vx_uint32   i;
    vx_uint32   width;
    vx_uint32   height;
    vx_map_id   map_id;
    uint8_t   * data_ptr;

    vx_rectangle_t             rect;
    vx_imagepatch_addressing_t image_addr;

    vx_status vxStatus;

    vxStatus = vxGetStatus((vx_reference)image);

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxQueryImage(image, VX_IMAGE_WIDTH, &width, sizeof(vx_uint32));
        vxQueryImage(image, VX_IMAGE_HEIGHT, &height, sizeof(vx_uint32));

        rect.start_x = 0;
        rect.start_y = 0;
        rect.end_x = width;
        rect.end_y = height;

        vxStatus = vxMapImagePatch(image,
            &rect,
            0,
            &map_id,
            &image_addr,
            (void **)&data_ptr,
            VX_READ_ONLY,
            VX_MEMORY_TYPE_HOST,
            VX_NOGAP_X
            );

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            vx_int32 header[SDE_FILE_HEADER_LEN];
            memset(header, 0, SDE_FILE_HEADER_LEN * sizeof(vx_int32));

            // only write image width and height
            header[3] = (vx_uint32)width;
            header[4] = (vx_uint32)height;

            FILE *fp = fopen(filename,"wb");

            if(fp != NULL)
            {
                size_t ret;

                fwrite((void *)header, 4, SDE_FILE_HEADER_LEN, fp);

                ret = 0;
                for (i = 0; i < height; i++)
                {
                    ret += fwrite(data_ptr, image_addr.stride_x, width, fp);
                    data_ptr += image_addr.stride_y;
                }

                if(ret != width * height)
                {
                    VX_PRINT(VX_ZONE_ERROR, "# ERROR: Unable to write data to file [%s]\n", filename);
                }
                fclose(fp);
            }
            else
            {
                VX_PRINT(VX_ZONE_ERROR, "# ERROR: Unable to open file for writing [%s]\n", filename);
                vxStatus = VX_FAILURE;
            }

            vxUnmapImagePatch(image, map_id);
        }
    }

    return vxStatus;
}

vx_status ptkdemo_save_vxtensor_to_file(vx_tensor tensor, sTIDL_IOBufDesc_t * ioBufDesc, char *filename)
{
    uint8_t * ssOut;
    FILE *    fp = NULL;
    uint8_t * output_buffer = NULL;
    vx_map_id map_id;
    vx_size   num_dims;
    vx_size   start[MAX_TENSOR_DIMS];
    vx_size   output_strides[MAX_TENSOR_DIMS];
    vx_size   output_sizes[MAX_TENSOR_DIMS];
    int32_t   i;
    vx_status vxStatus = (vx_status)VX_SUCCESS;

    /* Output tensor */
    output_sizes[0] = (vx_size)(ioBufDesc->outWidth[0]  +
                                ioBufDesc->outPadL[0] +
                                 ioBufDesc->outPadR[0]);

    output_sizes[1] = ioBufDesc->outHeight[0] +
                      ioBufDesc->outPadT[0] +
                      ioBufDesc->outPadB[0];

    output_sizes[2] = ioBufDesc->outNumChannels[0];

    start[0] = start[1] = start[2] = 0;
    vxQueryTensor(tensor, VX_TENSOR_NUMBER_OF_DIMS, &num_dims, sizeof(vx_size));

    if (num_dims >= MAX_TENSOR_DIMS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Invalid number of dims read [%d]", num_dims);     
        return (vx_status)VX_FAILURE;
    }

    vxStatus = tivxMapTensorPatch(tensor, num_dims, start, output_sizes,
                                  &map_id, output_strides,
                                  (void **)&output_buffer, VX_READ_ONLY,
                                  VX_MEMORY_TYPE_HOST);

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        ssOut = (uint8_t *)output_buffer;

        fp = fopen(filename, "wb");
    
        if (fp == NULL)
        {
            tivxUnmapTensorPatch(tensor, map_id);
            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        // write output size
        fwrite((void *)output_sizes, sizeof(vx_size), 4, fp);

        // write stride
        fwrite((void *)output_strides, sizeof(vx_size), 4, fp);

        // write padT, padR, padL and padR
        fwrite((void *)&ioBufDesc->outPadT[0], sizeof(int32_t), 1, fp);
        fwrite((void *)&ioBufDesc->outPadB[0], sizeof(int32_t), 1, fp);
        fwrite((void *)&ioBufDesc->outPadL[0], sizeof(int32_t), 1, fp);
        fwrite((void *)&ioBufDesc->outPadR[0], sizeof(int32_t), 1, fp);

        for (i = 0; i < output_sizes[1]; i++)
        {
            fwrite((void*)ssOut, 1,  output_sizes[0], fp);
            ssOut += output_sizes[0];
        }

        tivxUnmapTensorPatch(tensor, map_id);
    }

    if (fp != NULL)
    {
        fclose(fp);
    }

    return vxStatus;
}

vx_status ptkdemo_copy_image_to_image(vx_image srcImage, vx_image dstImage)
{
    vx_map_id                  map_id;
    vx_rectangle_t             rect;
    vx_imagepatch_addressing_t  image_addr_1;
    vx_imagepatch_addressing_t  image_addr_2;
    vx_df_image                img_format;
    vx_uint32                  img_width;
    vx_uint32                  img_height;
    vx_status                  vxStatus;

    uint8_t                    *data_ptr_src_1;
    uint8_t                    *data_ptr_src_2;

    vxQueryImage(srcImage, VX_IMAGE_FORMAT, &img_format, sizeof(vx_df_image));
    vxQueryImage(srcImage, VX_IMAGE_WIDTH, &img_width, sizeof(vx_uint32));
    vxQueryImage(srcImage, VX_IMAGE_HEIGHT, &img_height, sizeof(vx_uint32));

    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x   = img_width;
    rect.end_y   = img_height;

    // get source pointer
    vxStatus = vxMapImagePatch(srcImage,
                               &rect,
                               0,
                               &map_id,
                               &image_addr_1,
                               (void **)&data_ptr_src_1,
                               VX_READ_ONLY,
                               VX_MEMORY_TYPE_HOST,
                               VX_NOGAP_X);
    PTK_assert(VX_SUCCESS == vxStatus);
    vxUnmapImagePatch(srcImage, map_id);

    vxCopyImagePatch(dstImage, &rect, 0, &image_addr_1, data_ptr_src_1, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);

    // chroma
    if (img_format == VX_DF_IMAGE_NV12)
    {
        // get source pointer
        vxStatus = vxMapImagePatch(srcImage,
                                   &rect,
                                   1,
                                   &map_id,
                                   &image_addr_2,
                                   (void **)&data_ptr_src_2,
                                   VX_READ_ONLY,
                                   VX_MEMORY_TYPE_HOST,
                                   VX_NOGAP_X);
        PTK_assert(VX_SUCCESS == vxStatus);
        vxUnmapImagePatch(srcImage, map_id);

        vxCopyImagePatch(dstImage, &rect, 1, &image_addr_2, data_ptr_src_2, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);
    }

    return vxStatus;

}

vx_status ptkdemo_copy_data_to_image(const uint8_t * data_ptr_src, vx_image dstImage)
{
    vx_map_id                  map_id;
    vx_uint32                  img_width;
    vx_uint32                  img_height;
    vx_df_image                img_format;
    vx_rectangle_t             rect;
    vx_imagepatch_addressing_t image_addr;
    int32_t                    j;
    int32_t                    widthInByte;
    uint8_t                  * data_ptr_dst;
    vx_status                  vxStatus;

    vxQueryImage(dstImage, VX_IMAGE_FORMAT, &img_format, sizeof(vx_df_image));
    vxQueryImage(dstImage, VX_IMAGE_WIDTH,  &img_width, sizeof(vx_uint32));
    vxQueryImage(dstImage, VX_IMAGE_HEIGHT, &img_height, sizeof(vx_uint32));

    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x   = img_width;
    rect.end_y   = img_height;

    // get destination pointer
    vxStatus = vxMapImagePatch(dstImage,
                               &rect,
                               0,
                               &map_id,
                               &image_addr,
                               (void **)&data_ptr_dst,
                               VX_WRITE_ONLY,
                               VX_MEMORY_TYPE_HOST,
                               VX_NOGAP_X);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "vxMapImagePatch() failed.");
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        widthInByte = image_addr.dim_x * image_addr.stride_x;

        for (j = 0; j < image_addr.dim_y; j++)
        {
            memcpy(data_ptr_dst, data_ptr_src, widthInByte);
            data_ptr_src += widthInByte;
            data_ptr_dst += image_addr.stride_y;
        }

        vxUnmapImagePatch(dstImage, map_id);
    }

    // chroma
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        if (img_format == VX_DF_IMAGE_NV12)
        {
            rect.start_x = 0;
            rect.start_y = 0;
            rect.end_x   = img_width;
            rect.end_y   = img_height/2;

            // get destination pointer
            vxStatus = vxMapImagePatch(dstImage,
                                       &rect,
                                       1,
                                       &map_id,
                                       &image_addr,
                                       (void **)&data_ptr_dst,
                                       VX_WRITE_ONLY,
                                       VX_MEMORY_TYPE_HOST,
                                       VX_NOGAP_X);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                VX_PRINT(VX_ZONE_ERROR, "vxMapImagePatch() failed.");
            }
            else
            {
                widthInByte = image_addr.dim_x;

                for (j = 0; j < image_addr.dim_y; j++)
                {
                    memcpy(data_ptr_dst, data_ptr_src, widthInByte);
                    data_ptr_src += widthInByte;
                    data_ptr_dst += image_addr.stride_y;
                }

                vxUnmapImagePatch(dstImage, map_id);
            }
        }
    }

    return vxStatus;
}

vx_status ptkdemo_swap_ref_handles(vx_reference ref1, vx_reference ref2)
{
    void       *ref1Addr[PTKDEMO_MAX_NUM_TIVX_OBJ_HANDLES];
    void       *ref2Addr[PTKDEMO_MAX_NUM_TIVX_OBJ_HANDLES];
    uint32_t    size[PTKDEMO_MAX_NUM_TIVX_OBJ_HANDLES];
    uint32_t    numHandles;
    vx_bool     metaFlag;
    vx_status   vxStatus;

    vxStatus = VX_SUCCESS;

    /* Validate that the references are of the same type. */
    metaFlag = tivxIsReferenceMetaFormatEqual(ref1, ref2);

    if (metaFlag != (vx_bool)vx_true_e)
    {
        VX_PRINT(VX_ZONE_ERROR, "tivxIsReferenceMetaFormatEqual() failed.");
        vxStatus = VX_FAILURE;
    }

    /* Export the handles from ref1. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = tivxReferenceExportHandle(ref1,
                                             ref1Addr,
                                             size,
                                             PTKDEMO_MAX_NUM_TIVX_OBJ_HANDLES,
                                             &numHandles);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "tivxReferenceExportHandle() failed.");
        }
    }

    /* Export the handles from ref2. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = tivxReferenceExportHandle(ref2,
                                             ref2Addr,
                                             size,
                                             PTKDEMO_MAX_NUM_TIVX_OBJ_HANDLES,
                                             &numHandles);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "tivxReferenceExportHandle() failed.");
        }
    }

    /* Import ref2 handles into ref1. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = tivxReferenceImportHandle(ref1,
                                             (const void **)ref2Addr,
                                             size,
                                             numHandles);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "tivxReferenceImportHandle() failed.");
        }
    }

    /* Import ref1 handles into ref2. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = tivxReferenceImportHandle(ref2,
                                             (const void **)ref1Addr,
                                             size,
                                             numHandles);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "tivxReferenceImportHandle() failed.");

            /* Backout of the ref1 handle import into ref2. */
            vxStatus = tivxReferenceImportHandle(ref1,
                                                 (const void **)ref1Addr,
                                                 size,
                                                 numHandles);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                VX_PRINT(VX_ZONE_ERROR,
                         "Could not backout fo the previous import operation.");
            }
        }
    }

    return vxStatus;
}

uint32_t ptkdemo_compute_checksum(const uint8_t *data, uint32_t numBytes)
{
    uint32_t    checksum = 0;
    vx_status   vxStatus = VX_SUCCESS;

    if (data == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'data' is NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (numBytes == 0)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'numBytes' is 0.");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        const uint32_t *p;
        uint32_t        numWords;
        uint32_t        bytesLeft;
        uint32_t        i;

        p         = (const uint32_t *)data;
        numWords  = numBytes >> 2;
        bytesLeft = numBytes & 0x3;

        for (i = 0; i < numWords; i++)
        {
            checksum += *p++;
        }

        if (bytesLeft != 0)
        {
            uint32_t bitshift = (4U - bytesLeft) * 8U;

            checksum += (*p << bitshift) >> bitshift;
        }
    }

    return checksum;
}

