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

#ifndef _CM_COMMON_CNTXT_H_
#define _CM_COMMON_CNTXT_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <TI/tivx.h>
#include <TI/j7.h>
#include <TI/j7_tidl.h>

#include <perception/perception.h>

/**
 * \defgroup group_applib_common Common APPLIB code.
 * \ingroup group_ptk_applib
 *
 */

/**
 * \brief Constant for use in handling strings. The specified size is in bytes.
 * \ingroup group_applib_common
 */
#define CM_MAX_LINE_LEN          (256U)

/**
 * \brief Constant for use in validating the pipeline depth.
 * \ingroup group_applib_common
 */
#define CM_MAX_PIPELINE_DEPTH    (8U)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Function for saving an image.
 *
 * \param [in] fname Name of the file for saving the image.
 *
 * \param [out] image OpenVX image object with image data to save.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_common
 */
vx_status CM_saveImage(const char *fname, vx_image image);

/**
 * \brief Function to convert a YUV420 image to planar RGB format.
 *
 * \param [out] rgbImage RGB image data.
 *
 * \param [in] yuvImage YUV raw image data. yuvImage[0] will point to the Y
 *                      plane date and yuvImage[1] will point to the UV plane
 *                      data.
 *
 * \param [in] width Width of the image in pixels.
 *
 * \param [in] height Height of the image in pixels.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_common
 */
vx_status CM_convertYUV2RGB(uint8_t        *rgbImage,
                            const uint8_t  *yuvImage[2],
                            int32_t         width,
                            int32_t         height);

/**
 * \brief Function to extract image data from an openVX YUV image object. The
 *        data can be extracted and converted to RGB format using 'rgbFlag'
 *        parameter.
 *
 * \param [out] outImageData Output image data. The size of this buffer needs
 *                           to be properly domensioned, as follows:
 *                           - size = 1.5 * width * height for YUV data, i.e. if
 *                             rgbFlag == 0
 *                           - size = 3 * width * height for RGB data, i.e. if
 *                             rgbFlag == 1
 *
 * \param [in] yuvImage OpenVX Image object carrying YUV image.
 *
 * \param [in] width Width of the image in pixels.
 *
 * \param [in] height Height of the image in pixels.
 *
 * \param [in] rgbFlag Flag to indicate if the image data needs to be converted to
 *                     RGB format.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_common
 */
vx_status CM_extractImageData(uint8_t         *outImageData,
                              const vx_image   yuvImage,
                              uint32_t         width,
                              uint32_t         height,
                              uint8_t          numPlanes,
                              uint8_t          rgbFlag);

/**
 * \brief Function to extract 8-bit raw tensor data from an openVX tensor object.
 *        Other data type is not supported yet.
 *
 * \param [out] outTensorData Output 8-bit tensor data. The size of this buffer is
 *                            1 * width * height.
 *
 * \param [in]  tensor OpenVX Tensor object
 *
 * \param [in]  width  Width of raw tensor data in pixels.
 *
 * \param [in]  height Height of raw tensor data in pixels.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_common
 */
vx_status CM_extractTensorData(uint8_t         *outTensorData,
                               const vx_tensor  tensor,
                               uint32_t         width,
                               uint32_t         height);

/**
 * \brief Function to extract point cloud (XYZRGB) data from an OpenVX user data object.
 *
 * \param [out]     outPcData Output point cloud data, which inclues X,Y,Z and R, G, B for each point. 
 *                            The size of each point is 15 bytes.
 *
 * \param [in]      pointCloud OpenVX user object for PTK_PointCloud 
 *
 * \param [in]      pointSize  size of one point in byte 
 * 
 * \param [in/out]  outPcSize  the number of points in pointCloud object. 
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_common
 */

vx_status CM_extractPointCloudData(uint8_t                  *outPcData,
                                   const vx_user_data_object pointCloud,
                                   uint32_t                  pointSize,
                                   uint32_t                 *numPoints);
#ifdef __cplusplus
}
#endif

#endif /* _CM_COMMON_CNTXT_H_ */

