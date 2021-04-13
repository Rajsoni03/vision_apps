/*
*
* Copyright (c) 2018 Texas Instruments Incorporated
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
#include <TI/j7.h>
#include <TI/tivx_img_proc.h>
#include <TI/tivx_target_kernel.h>
#include <tivx_kernels_target_utils.h>
#include <utils/udma/include/app_udma.h>
#include <utils/mem/include/app_mem.h>

#include <tivx_img_mosaic_host.h>
static tivx_target_kernel vx_imgMosaic_kernel = NULL;

#ifndef x86_64
#if CPU_COPY    /* Not enabled - change to 1 for doing CPU copy */
static void memcpyC66(uint8_t *restrict pOut, uint8_t *restrict pIn, int32_t size)
{
    int32_t remSize = size - ((size >> 3) << 3);
    int32_t i;

    for(i = 0; i < size; i+=8)
    {
        _mem8(&pOut[i]) = _mem8(&pIn[i]);
    }
    if(remSize > 0)
    {
        i-=8;
        for(; i < size; i++)
        {
            pOut[i] = pIn[i];
        }
    }
}
static void memcpyC662d(uint8_t *pOut, uint8_t *pIn, uint32_t width, uint32_t height, uint32_t src_pitch, uint32_t dest_pitch)
{
    uint32_t i;
    for(i = 0; i < height; i++)
    {
        memcpyC66(pOut, pIn, width);
        pIn += src_pitch;
        pOut += dest_pitch;
    }
}
#else
static void memcpyC662d(uint8_t *pOut, uint8_t *pIn, uint32_t width, uint32_t height, uint32_t src_pitch, uint32_t dest_pitch)
{
    app_udma_copy_2d_prms_t prms_2d;

    appUdmaCopy2DPrms_Init(&prms_2d);
    prms_2d.width        = width;
    prms_2d.height       = height;
    prms_2d.dest_pitch   = dest_pitch;
    prms_2d.src_pitch    = src_pitch;
    prms_2d.dest_addr    = appMemGetVirt2PhyBufPtr((uint64_t) pOut, APP_MEM_HEAP_DDR);
    prms_2d.src_addr     = appMemGetVirt2PhyBufPtr((uint64_t) pIn, APP_MEM_HEAP_DDR);
    appUdmaCopy2D(NULL, &prms_2d, 1);
}
#endif
#else
static void memcpyC662d(uint8_t *pOut, uint8_t *pIn, uint32_t width, uint32_t height, uint32_t src_pitch, uint32_t dest_pitch)
{
    uint32_t i;
    for(i = 0; i < height; i++)
    {
        memcpy(pOut, pIn, width);
        pIn += src_pitch;
        pOut += dest_pitch;
    }
}
#endif

static vx_status VX_CALLBACK tivxKernelImgMosaicCreate
(
    tivx_target_kernel_instance kernel,
    tivx_obj_desc_t *obj_desc[],
    vx_uint16 num_params,
    void *priv_arg
)
{
    vx_status status = VX_SUCCESS;
    uint32_t i;

    for (i = 0U; i < num_params; i ++)
    {
        if (NULL == obj_desc[i])
        {
            status = VX_FAILURE;
            break;
        }
    }

    if(VX_SUCCESS == status)
    {
        tivx_obj_desc_user_data_object_t* config_desc;
        void * config_target_ptr = NULL;

        tivx_obj_desc_image_t* output_image_desc;
        tivx_obj_desc_object_array_t *input_image_arr_desc;
        tivx_obj_desc_image_t *input_image_desc[TIVX_OBJECT_ARRAY_MAX_ITEMS];

        vx_int32 win;
        vx_int32 num_inputs;

        /* 0 - config, 1 - output image, 2 onwards is array of inputs */
        num_inputs = num_params - 2;

        config_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_IMG_MOSAIC_HOST_CONFIG_IDX];
        config_target_ptr = tivxMemShared2TargetPtr(&config_desc->mem_ptr);
        tivxMemBufferMap(config_target_ptr, config_desc->mem_size, VX_MEMORY_TYPE_HOST,VX_READ_ONLY);

        output_image_desc = (tivx_obj_desc_image_t *)obj_desc[TIVX_IMG_MOSAIC_HOST_OUTPUT_IMAGE_IDX];

        tivxImgMosaicParams *params  = (tivxImgMosaicParams *)config_target_ptr;

        if(params->num_windows > TIVX_IMG_MOSAIC_MAX_WINDOWS)
        {
            VX_PRINT(VX_ZONE_ERROR, "Num windows %d greater than supported max %d\n", params->num_windows, TIVX_IMG_MOSAIC_MAX_WINDOWS);
            status = VX_FAILURE;
        }

        if(VX_SUCCESS == status)
        {
            for(win = 0; win < params->num_windows; win++)
            {
                tivxImgMosaicWindow *window = (tivxImgMosaicWindow *)&params->windows[win];
                vx_int32 in = window->input_select;
                vx_int32 ch = window->channel_select;

                if(in > num_inputs)
                {
                    VX_PRINT(VX_ZONE_ERROR, "Input ID %d greater than num_inputs %d\n", in, num_inputs);
                    status = VX_FAILURE;
                    break;
                }

                input_image_arr_desc = (tivx_obj_desc_object_array_t *)obj_desc[TIVX_IMG_MOSAIC_INPUT_START_IDX + in];
                tivxGetObjDescList(input_image_arr_desc->obj_desc_id, (tivx_obj_desc_t**)input_image_desc, input_image_arr_desc->num_items);

                if(ch > input_image_arr_desc->num_items)
                {
                    VX_PRINT(VX_ZONE_ERROR, "Channel ID %d greater than num_items %d\n", ch, input_image_arr_desc->num_items);
                    status = VX_FAILURE;
                    break;
                }

                if(((window->startX + window->width) > output_image_desc->width) ||
                ((window->startY + window->height) > output_image_desc->height))
                {
                    VX_PRINT(VX_ZONE_ERROR, "Window %d, does not fit within output image!\n", win);
                    status = VX_FAILURE;
                }
            }
        }
        tivxMemBufferUnmap(config_target_ptr, config_desc->mem_size, VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
    }
    return (status);
}

static vx_status VX_CALLBACK tivxKernelImgMosaicDelete(
    tivx_target_kernel_instance kernel, tivx_obj_desc_t *obj_desc[],
    vx_uint16 num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    int32_t i;

    for (i = 0U; i < num_params; i ++)
    {
        if (NULL == obj_desc[i])
        {
            status = VX_FAILURE;
            break;
        }
    }

    return (status);
}

static vx_status VX_CALLBACK tivxKernelImgMosaicProcess
(
    tivx_target_kernel_instance kernel,
    tivx_obj_desc_t *obj_desc[],
    vx_uint16 num_params,
    void *priv_arg
)
{
    vx_status status = VX_SUCCESS;

    if(VX_SUCCESS == status)
    {
        tivx_obj_desc_user_data_object_t* config_desc;
        void * config_target_ptr = NULL;

        tivx_obj_desc_image_t* output_image_desc;
        void * output_image_target_ptr[2];

        tivx_obj_desc_object_array_t *input_image_arr_desc;
        tivx_obj_desc_image_t *input_image_desc[TIVX_OBJECT_ARRAY_MAX_ITEMS];
        void * input_image_target_ptr[2];

        vx_int32 win;
        vx_int32 num_inputs;

        output_image_target_ptr[0] = NULL;
        output_image_target_ptr[1] = NULL;

        input_image_target_ptr[0] = NULL;
        input_image_target_ptr[1] = NULL;

        config_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_IMG_MOSAIC_HOST_CONFIG_IDX];
        config_target_ptr = tivxMemShared2TargetPtr(&config_desc->mem_ptr);
        tivxMemBufferMap(config_target_ptr, config_desc->mem_size, VX_MEMORY_TYPE_HOST,VX_READ_ONLY);

        output_image_desc = (tivx_obj_desc_image_t *)obj_desc[TIVX_IMG_MOSAIC_HOST_OUTPUT_IMAGE_IDX];
        output_image_target_ptr[0] = tivxMemShared2TargetPtr(&output_image_desc->mem_ptr[0]);
        tivxMemBufferMap(output_image_target_ptr[0], output_image_desc->mem_size[0], VX_MEMORY_TYPE_HOST,VX_WRITE_ONLY);
        if(output_image_desc->mem_ptr[1].shared_ptr != 0)
        {
            output_image_target_ptr[1] = tivxMemShared2TargetPtr(&output_image_desc->mem_ptr[1]);
            tivxMemBufferMap(output_image_target_ptr[1], output_image_desc->mem_size[1], VX_MEMORY_TYPE_HOST,VX_WRITE_ONLY);
        }

        tivxImgMosaicParams *params  = (tivxImgMosaicParams *)config_target_ptr;

        if(params->clear_count > 0)
        {
            memset(output_image_target_ptr[0], 0, output_image_desc->mem_size[0]);
            if(output_image_target_ptr[1] != 0)
            {
                memset(output_image_target_ptr[1], 128, output_image_desc->mem_size[1]);
            }
            params->clear_count--;
        }

        /* 0 - config, 1 - output image, 2 onwards is array of inputs */
        num_inputs = num_params - 2;

        for(win = 0; win < params->num_windows; win++)
        {
            tivxImgMosaicWindow *window = (tivxImgMosaicWindow *)&params->windows[win];
            vx_int32 in = window->input_select;
            vx_int32 ch = window->channel_select;

            if(in < num_inputs)
            {
                input_image_arr_desc = (tivx_obj_desc_object_array_t *)obj_desc[TIVX_IMG_MOSAIC_INPUT_START_IDX + in];
                tivxGetObjDescList(input_image_arr_desc->obj_desc_id, (tivx_obj_desc_t**)input_image_desc, input_image_arr_desc->num_items);

                if(ch < input_image_arr_desc->num_items)
                {
                    input_image_target_ptr[0] = tivxMemShared2TargetPtr(&input_image_desc[ch]->mem_ptr[0]);
                    tivxMemBufferMap(input_image_target_ptr[0], input_image_desc[ch]->mem_size[0], VX_MEMORY_TYPE_HOST,VX_READ_ONLY);
                    if(input_image_desc[ch]->mem_ptr[1].shared_ptr != 0)
                    {
                        input_image_target_ptr[1] = tivxMemShared2TargetPtr(&input_image_desc[ch]->mem_ptr[1]);
                        tivxMemBufferMap(input_image_target_ptr[1], input_image_desc[ch]->mem_size[1], VX_MEMORY_TYPE_HOST,VX_READ_ONLY);
                    }

                    if(((window->startX + window->width) <= output_image_desc->width) &&
                        ((window->startY + window->height) <= output_image_desc->height))
                        {
                            vx_uint8 *pIn;
                            vx_uint8 *pOut;

                            pOut = (vx_uint8 *)output_image_target_ptr[0] + (((window->startY >> 1) << 1) * output_image_desc->width) + ((window->startX >> 1) << 1);
                            pIn  = (vx_uint8 *)input_image_target_ptr[0];
                            memcpyC662d(pOut, pIn, window->width, window->height, input_image_desc[ch]->imagepatch_addr[0U].stride_y, output_image_desc->width);

                            pOut = (vx_uint8 *)output_image_target_ptr[1] + ((window->startY >> 1) * output_image_desc->width) + ((window->startX >> 1) << 1);
                            pIn  = (vx_uint8 *)input_image_target_ptr[1];
                            memcpyC662d(pOut, pIn, window->width, window->height>>1, input_image_desc[ch]->imagepatch_addr[1U].stride_y, output_image_desc->width);
                        }

                    tivxMemBufferUnmap(input_image_target_ptr[0], input_image_desc[ch]->mem_size[0], VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
                    if(input_image_target_ptr[1] != NULL)
                    {
                        tivxMemBufferUnmap(input_image_target_ptr[1], input_image_desc[ch]->mem_size[1], VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
                    }
                }
            }
        }

        tivxMemBufferUnmap(config_target_ptr, config_desc->mem_size, VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
        tivxMemBufferUnmap(output_image_target_ptr[0], output_image_desc->mem_size[0], VX_MEMORY_TYPE_HOST, VX_WRITE_ONLY);
        if(output_image_target_ptr[1] != NULL)
        {
            tivxMemBufferUnmap(output_image_target_ptr[1], output_image_desc->mem_size[1], VX_MEMORY_TYPE_HOST, VX_WRITE_ONLY);
        }

    }

    return (status);
}

void tivxAddTargetKernelImgMosaic()
{
    char target_name[TIVX_TARGET_MAX_NAME];
    vx_enum self_cpu;

    self_cpu = tivxGetSelfCpuId();

    if ((self_cpu == TIVX_CPU_ID_DSP1) || (self_cpu == TIVX_CPU_ID_DSP2) || (self_cpu == TIVX_CPU_ID_A72_0))
    {
        if (self_cpu == TIVX_CPU_ID_DSP1)
        {
            strncpy(target_name, TIVX_TARGET_DSP1, TIVX_TARGET_MAX_NAME);
        }
        else if (self_cpu == TIVX_CPU_ID_DSP2)
        {
            strncpy(target_name, TIVX_TARGET_DSP2, TIVX_TARGET_MAX_NAME);
        }
        else if (self_cpu == TIVX_CPU_ID_A72_0)
        {
            strncpy(target_name, TIVX_TARGET_A72_0, TIVX_TARGET_MAX_NAME);
        }

        vx_imgMosaic_kernel = tivxAddTargetKernelByName
                              (
                                TIVX_KERNEL_IMG_MOSAIC_NAME,
                                target_name,
                                tivxKernelImgMosaicProcess,
                                tivxKernelImgMosaicCreate,
                                tivxKernelImgMosaicDelete,
                                NULL,
                                NULL
                              );
    }
}

void tivxRemoveTargetKernelImgMosaic()
{
    vx_status status = VX_SUCCESS;

    status = tivxRemoveTargetKernel(vx_imgMosaic_kernel);
    if (status == VX_SUCCESS)
    {
        vx_imgMosaic_kernel = NULL;
    }
}
