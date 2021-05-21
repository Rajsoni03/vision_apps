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
#include <cm_common.h>

#define MAX_TENSOR_DIMS    (4u)
#define clip3(x, min, max) ( x > max? max : (x < min ? min : x))

vx_status CM_saveImage(const char *fname, vx_image image)
{
    FILE                       *fp;
    void                       *data_ptr;
    vx_rectangle_t              rect;
    vx_imagepatch_addressing_t  image_addr;
    vx_map_id                   map_id;
    vx_uint32                   img_width;
    vx_uint32                   img_height;
    vx_uint32                   num_bytes = 0;
    vx_status                   vxStatus;
    vx_int32                    j;

    vxStatus = VX_SUCCESS;

    fp = fopen(fname, "wb");

    if (fp == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Unable to open input file [%s]\n",
                    __FUNCTION__, __LINE__, fname);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxQueryImage(image, VX_IMAGE_WIDTH, &img_width, sizeof(vx_uint32));
        vxQueryImage(image, VX_IMAGE_HEIGHT, &img_height, sizeof(vx_uint32));

        rect.start_x = 0;
        rect.start_y = 0;
        rect.end_x = img_width;
        rect.end_y = img_height;
        vxStatus = vxMapImagePatch(image,
                                   &rect,
                                   0,
                                   &map_id,
                                   &image_addr,
                                   &data_ptr,
                                   VX_WRITE_ONLY,
                                   VX_MEMORY_TYPE_HOST,
                                   VX_NOGAP_X);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "vxMapImagePatch() failed\n",
                        __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Copy Luma */
        for (j = 0; j < img_height; j++)
        {
            num_bytes += fwrite(data_ptr, 1, img_width, fp);
            data_ptr += image_addr.stride_y;
        }

        if (num_bytes != (img_width*img_height))
        {
            VX_PRINT(VX_ZONE_ERROR, "Luma bytes written = %d, expected = %d\n",
                        __FUNCTION__, __LINE__,
                        num_bytes, img_width*img_height);

            vxStatus = VX_FAILURE;
        }

        vxUnmapImagePatch(image, map_id);
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        rect.start_x = 0;
        rect.start_y = 0;
        rect.end_x = img_width;
        rect.end_y = img_height / 2;
        vxStatus = vxMapImagePatch(image,
                                   &rect,
                                   1,
                                   &map_id,
                                   &image_addr,
                                   &data_ptr,
                                   VX_WRITE_ONLY,
                                   VX_MEMORY_TYPE_HOST,
                                   VX_NOGAP_X);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "vxMapImagePatch() failed\n",
                        __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Copy CbCr */
        num_bytes = 0;
        for (j = 0; j < img_height/2; j++)
        {
            num_bytes += fwrite(data_ptr, 1, img_width, fp);
            data_ptr += image_addr.stride_y;
        }

        if (num_bytes != (img_width*img_height/2))
        {
            VX_PRINT(VX_ZONE_ERROR,
                     "CbCr bytes written = %d, expected = %d\n",
                     img_width*img_height/2);

            vxStatus = VX_FAILURE;
        }

        vxUnmapImagePatch(image, map_id);
    }

    if (fp != NULL)
    {
        fclose(fp);
    }

    return vxStatus;

}

vx_status CM_convertYUV2RGB(uint8_t        *rgbImage,
                            const uint8_t  *yuvImage[2],
                            int32_t         width,
                            int32_t         height)
{
    const uint8_t  *srcPtrY;
    const uint8_t  *srcPtrUV;
    uint8_t        *dstPtrR;
    uint8_t        *dstPtrG;
    uint8_t        *dstPtrB;
    int32_t         i;
    int32_t         j;
    int32_t         y;
    int32_t         cb;
    int32_t         cr;
    int32_t         r;
    int32_t         g;
    int32_t         b;
    vx_status       vxStatus;

    vxStatus = VX_SUCCESS;

    if (rgbImage == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'rgbImage' is NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (yuvImage[0] == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'yuvImage[0]' is NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (yuvImage[1] == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'yuvImage[1]' is NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (width == 0)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'width' cannot be 0.");
        vxStatus = VX_FAILURE;
    }
    else if (height == 0)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'height' cannot be 0.");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        srcPtrY  = yuvImage[0];
        srcPtrUV = yuvImage[1];

        dstPtrR  = rgbImage;
        dstPtrG  = rgbImage + 1;
        dstPtrB  = rgbImage + 2;

        for (j = 0; j < height; j++)
        {
            for (i = 0; i < width; i++)
            {
                y  = srcPtrY[j * width + i];
                cb = srcPtrUV[(j >> 1)*width + (i>>1)*2];
                cr = srcPtrUV[(j >> 1)*width + (i>>1)*2 + 1];

                y  = y  - 16;
                cb = cb - 128;
                cr = cr - 128;

                r = clip3((298*y + 409*cr + 128) >> 8, 0, 255);
                g = clip3((298*y - 100*cb - 208*cr + 128) >> 8, 0, 255);
                b = clip3((298*y + 516*cb + 128) >> 8, 0, 255);

                *dstPtrR = (uint8_t)r; dstPtrR += 3;
                *dstPtrG = (uint8_t)g; dstPtrG += 3;
                *dstPtrB = (uint8_t)b; dstPtrB += 3;
            }
        }
    }

    return vxStatus;
}

vx_status CM_extractImageData(uint8_t         *outImageData,
                              const vx_image   yuvImage,
                              uint32_t         width,
                              uint32_t         height,
                              uint8_t          numPlanes,
                              uint8_t          rgbFlag)
{
    const uint8_t              *dataPtr[2];
    vx_imagepatch_addressing_t  addr;
    vx_df_image                 img_format;
    uint32_t                    i;
    vx_status                   vxStatus;

    vxStatus = VX_SUCCESS;

    if (outImageData == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'outImageData' is NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (yuvImage == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'yuvImage' is NULL.");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxQueryImage(yuvImage,
                                VX_IMAGE_FORMAT,
                                &img_format,
                                sizeof(vx_df_image));

        if ((img_format != VX_DF_IMAGE_NV12) &&
            (img_format != VX_DF_IMAGE_S16)  &&
            (img_format != VX_DF_IMAGE_RGB))
        {
            VX_PRINT(VX_ZONE_ERROR, "Image format is NOT supported.");
            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vx_rectangle_t  rect;
        vx_map_id       mapId;

        // Define the ROI.
        rect.start_x = 0;
        rect.start_y = 0;
        rect.end_x   = width;
        rect.end_y   = height;

        for (i = 0; i < numPlanes; i++)
        {
            vxStatus = vxMapImagePatch(yuvImage,
                                       &rect,
                                       i,
                                       &mapId,
                                       &addr,
                                       (void **)&dataPtr[i],
                                       VX_READ_ONLY,
                                       VX_MEMORY_TYPE_HOST,
                                       VX_NOGAP_X);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                VX_PRINT(VX_ZONE_ERROR, "vxMapImagePatch() failed.");
                break;
            }

            vxUnmapImagePatch(yuvImage, mapId);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        if (rgbFlag == 0)
        {
            uint8_t    *dstImgPtr;
            uint32_t    size = 0;

            /* Copy the YUV data as is, Y followed by UV. */
            dstImgPtr = outImageData;

            if (img_format == VX_DF_IMAGE_NV12)
            {
                size = width * height;
            }
            else if (img_format == VX_DF_IMAGE_S16)
            {
                size = 2 * width * height;
            }
            else if (img_format == VX_DF_IMAGE_RGB)
            {
                size = 3 * width * height;
            }

            for (i = 0; i < numPlanes; i++, size /= 2)
            {
                memcpy(dstImgPtr, dataPtr[i], size);
                dstImgPtr += size;
            }
        }
        else
        {
            /* Convert to RGB data format. */
            vxStatus = CM_convertYUV2RGB(outImageData,
                                         dataPtr,
                                         width,
                                         height);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                VX_PRINT(VX_ZONE_ERROR, "CM_convertYUV2RGB() failed.");
                vxStatus = VX_FAILURE;
            }
        }
    }

    return vxStatus;
}

vx_status CM_extractTensorData(uint8_t         *outTensorData,
                               const vx_tensor  tensor,
                               uint32_t         width,
                               uint32_t         height)
{
    uint8_t * output_buffer = NULL;
    vx_map_id map_id;
    vx_size   num_dims;
    vx_size   start[MAX_TENSOR_DIMS];
    vx_size   tensor_strides[MAX_TENSOR_DIMS];
    vx_size   tensor_sizes[MAX_TENSOR_DIMS];
    vx_status vxStatus = VX_SUCCESS;

    start[0] = start[1] = start[2] = 0;
    vxQueryTensor(tensor, VX_TENSOR_NUMBER_OF_DIMS, &num_dims, sizeof(vx_size));
    vxQueryTensor(tensor, VX_TENSOR_DIMS, tensor_sizes, 3 * sizeof(vx_size));

    if (num_dims >= MAX_TENSOR_DIMS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Invalid number of dims read [%d].", num_dims);
        vxStatus = VX_FAILURE;
    }
    else
    {
        vxStatus = tivxMapTensorPatch(tensor, num_dims, start, tensor_sizes,
                                      &map_id, tensor_strides,
                                      (void **)&output_buffer, VX_READ_ONLY,
                                      VX_MEMORY_TYPE_HOST);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "tivxMapTensorPatch() failed.");
            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        memcpy(outTensorData, output_buffer, width*height);
        vxStatus = tivxUnmapTensorPatch(tensor, map_id);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "tivxUnmapTensorPatch() failed.");
            vxStatus = VX_FAILURE;
        }
    }

    return vxStatus;
}



vx_status CM_extractPointCloudData(uint8_t                  *outPcData,
                                   const vx_user_data_object pointCloud,
                                   uint32_t                  pointSize,
                                   uint32_t                 *numPoints)
{
    PTK_PointCloud  * pc;
    PTK_Point       * points;
    vx_map_id         mapId;
    vx_status         vxStatus = (vx_status) VX_SUCCESS;

    uint32_t          i;
    uint32_t          rgb;

    if (outPcData == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'outPcData' is NULL.");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Map the user object. */
        vxStatus = vxMapUserDataObject(pointCloud,
                                       0, // offset
                                       0, // size = 0 ==> entire buffer
                                       &mapId,
                                       (void *)&pc,
                                       VX_READ_ONLY,
                                       VX_MEMORY_TYPE_HOST,
                                       0);

        if ((vxStatus != (vx_status)VX_SUCCESS) || (pc == NULL))
        {
            VX_PRINT(VX_ZONE_ERROR, "[%s:%d] vxMapUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Unmap the user object. */
        vxStatus = vxUnmapUserDataObject(pointCloud, mapId);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "[%s:%d] vxUnmapUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        *numPoints = pc->numPoints;
        points     = PTK_PointCloud_getPoints(pc);

        for (i = 0; i < pc->numPoints; i++)
        {
            rgb = (uint32_t) points[i].meta.w;

            memcpy(&outPcData[i*pointSize    ], &points[i].x, sizeof(float));
            memcpy(&outPcData[i*pointSize + 4], &points[i].y, sizeof(float));
            memcpy(&outPcData[i*pointSize + 8], &points[i].z, sizeof(float));
            memcpy(&outPcData[i*pointSize + 12], &rgb, sizeof(uint32_t));
        }
    }


    return vxStatus;
}
