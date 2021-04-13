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
#if !defined(_APP_PTK_DEMO_COMMON_H_)
#define _APP_PTK_DEMO_COMMON_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>
#include <assert.h>
#include <TI/j7.h>
#include <TI/j7_tidl.h>
#include <itidl_ti.h>

#include <perception/perception.h>

/* SDE Header size in byte */
#define SDE_FILE_HEADER_LEN 80

/* Max tensor dimension */
#define MAX_TENSOR_DIMS     4

/* input image format */
typedef enum {
    PTK_IMG_FORMAT_Y    = 0,
    PTK_IMG_FORMAT_UYVY = 1
} PTK_IMG_FORMAT;

#define PTK_GRAPH_MAX_PIPELINE_DEPTH 8

#ifdef __cplusplus
extern "C" {
#endif


/* return -1 if there is no / or \ in @path,
 * otherwise returns position of last such character in @path.
 */
int32_t ptkdemo_find_slash(char        *path,
                           uint32_t     maxPathLen);

/* if @pValueStr has / or \, prefix @pValueStr with @globalBasePath and write to @filePath.
 * Otherwise, prefix @pValueStr with @localBasePath and write to @filePath.
 */
int32_t ptkdemo_get_file_path(char        **filePath,
                              char         *pValueStr,
                              char         *globalBasePath,
                              char         *localBasePath,
                              uint32_t      maxPathLen);

const char *app_common_get_coreName(const char *appCoreName);
int32_t appInit();
int32_t appDeInit();

vx_status ptkdemo_addParamByNodeIndex(vx_graph  graph,
                                      vx_node   node,
                                      vx_uint32 nodeParamIndex);

/*
 * \brief Returns the payload pointer from vx_user_data_object.
 */
void * ptkdemo_getUserObjDataPayload(vx_user_data_object   obj);

/* Load yuv image into vx_image */
vx_status ptkdemo_load_vximage_from_yuvfile(vx_image image, char *filename);

/* Load SDE raw disparity map into vx_image */
vx_status ptkdemo_load_vximage_from_sdefile(vx_image image, char *filename);

/* Load binary tensor file into vx_tensor */
vx_status ptkdemo_load_vxtensor_from_file(vx_tensor tensor, char *filename);

/* Save vx_image into yuv file */
vx_status ptkdemo_save_vximage_to_yuvfile(vx_image image, char *filename);


/* Save vx_image into raw disparity map file format */
vx_status ptkdemo_save_vximage_to_sdefile(vx_image image, char *filename);

/* Save vx_tensor into binary file */
vx_status ptkdemo_save_vxtensor_to_file(vx_tensor tensor, sTIDL_IOBufDesc_t * ioBufDesc, char *filename);

/*
 * \brief Copy image data from one image object to another image object
 */
vx_status ptkdemo_copy_image_to_image(vx_image srcImage, vx_image dstImage);

/*
 * \brief Copy image data from one data pointer to another image object
 */
vx_status ptkdemo_copy_data_to_image(const uint8_t * data_ptr_src, vx_image dstImage);

/*
 * \brief Swaps the internal handles (memory buffers) between two references. The
 *        references must be of the same type.
 */
vx_status ptkdemo_swap_ref_handles(vx_reference ref1, vx_reference ref2);

/*
 * \brief Computes the checksum on the data buffer.
 */
uint32_t ptkdemo_compute_checksum(const uint8_t *data, uint32_t numBytes);

#ifdef __cplusplus
}
#endif

#endif /* _APP_PTK_DEMO_COMMON_H_ */

