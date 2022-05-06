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

#include "TI/tivx.h"
#include "TI/tivx_sample.h"
#include "VX/vx.h"
#include "tivx_sample_kernels_priv.h"
#include "tivx_kernel_pcie_tx.h"
#include "TI/tivx_target_kernel.h"
#include "tivx_kernels_target_utils.h"

#include "TI/tivx_task.h"
#include <utils/udma/include/app_udma.h>
#include <utils/pcie/include/app_pcie_queue.h>

#define PCIE_TX_POLLING_INTERVAL    (1)

typedef struct
{
    uint32_t reserved;
} tivxPcieTxParams;

static tivx_target_kernel vx_pcie_tx_target_kernel = NULL;

static vx_status VX_CALLBACK tivxPcieTxProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxPcieTxCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxPcieTxDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxPcieTxControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);

static vx_status VX_CALLBACK tivxPcieTxProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    tivx_obj_desc_user_data_object_t *configuration_desc;
  
    tivx_obj_desc_image_t *img_input_desc[TIVX_OBJECT_ARRAY_MAX_ITEMS];
    tivxPcieTxParams *pcieTxParams = NULL;
    app_udma_copy_2d_prms_t prms_2d;
    vx_uint32 i;
    vx_uint32 j;
    vx_uint32 size;
    vx_uint64 src_addr;
    vx_uint64 dest_addr;
    uint32_t width;
    uint32_t width_in_bytes = 0 ;
    uint32_t height;
    uint32_t pitch;
    vx_uint32 channel;
    app_pcie_queue_request_descriptor_t req_desc;
    uint32_t num_items;

    if ( (num_params != TIVX_KERNEL_PCIE_TX_MAX_PARAMS)
        || (NULL == obj_desc[TIVX_KERNEL_PCIE_TX_CONFIGURATION_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_PCIE_TX_INPUT_IDX])
    )
    {
        status = VX_FAILURE;
    }

    if(VX_SUCCESS == status)
    {
        configuration_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_PCIE_TX_CONFIGURATION_IDX];
    }

    if(VX_SUCCESS == status)
    {
        void *configuration_target_ptr;
        void *input_target_ptr[TIVX_OBJECT_ARRAY_MAX_ITEMS][TIVX_IMAGE_MAX_PLANES] = {NULL};

        configuration_target_ptr = tivxMemShared2TargetPtr(&configuration_desc->mem_ptr);
        tivxMemBufferMap(configuration_target_ptr,
           configuration_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_READ_ONLY);

        /* Object Array Case*/
        if(TIVX_OBJ_DESC_OBJARRAY == obj_desc[TIVX_KERNEL_PCIE_TX_INPUT_IDX]->type)
        {
            tivx_obj_desc_object_array_t *input_desc;

            input_desc = (tivx_obj_desc_object_array_t *)obj_desc[TIVX_KERNEL_PCIE_TX_INPUT_IDX];
            num_items = input_desc->num_items;
            tivxGetObjDescList(input_desc->obj_desc_id, (tivx_obj_desc_t**)img_input_desc, num_items);
        }
        /* Get Object Array from Image Case*/
        else if ( (TIVX_OBJ_DESC_INVALID != obj_desc[TIVX_KERNEL_PCIE_TX_INPUT_IDX]->scope_obj_desc_id) 
                && (TIVX_OBJ_DESC_IMAGE == obj_desc[TIVX_KERNEL_PCIE_TX_INPUT_IDX]->type) ) 
        {
            tivx_obj_desc_object_array_t *parent_obj_desc;

            tivxGetObjDescList(
                &obj_desc[TIVX_KERNEL_PCIE_TX_INPUT_IDX]->scope_obj_desc_id,
                (tivx_obj_desc_t**)&parent_obj_desc, 1);

            num_items = parent_obj_desc->num_items;                    

            tivxGetObjDescList(
                parent_obj_desc->obj_desc_id,  // !!! CONFIRM THIS !!!
                (tivx_obj_desc_t**)img_input_desc, num_items);
        }
        /* Image Case*/
        else if (TIVX_OBJ_DESC_IMAGE == obj_desc[TIVX_KERNEL_PCIE_TX_INPUT_IDX]->type)
        {
            img_input_desc[0] = (tivx_obj_desc_image_t *)obj_desc[TIVX_KERNEL_PCIE_TX_INPUT_IDX];
            num_items = 1;
        }
        /* Unhandled Case*/
        else
        {
            status = VX_FAILURE;    
            num_items = 0;
        }
        

        for(i=0; i<num_items; i++)
        {
            for(j=0;j<img_input_desc[i]->planes;j++)
            {
                input_target_ptr[i][j] = tivxMemShared2TargetPtr(&img_input_desc[i]->mem_ptr[j]);
                tivxMemBufferMap(input_target_ptr[i][j],
                   img_input_desc[i]->mem_size[j], TIVX_MEMORY_TYPE_DMA,
                   VX_READ_ONLY);
            }
        }

        {

            /* call kernel processing function */
            if(VX_SUCCESS == status)
            {
                status = tivxGetTargetKernelInstanceContext(kernel,
                                                            (void *)&pcieTxParams,
                                                            &size);

                if((VX_SUCCESS != status) ||
                (NULL == pcieTxParams) ||
                (sizeof(tivxPcieTxParams) != size))
                {
                    status = VX_FAILURE;
                }
            }

            if(VX_SUCCESS == status && appPcieQueueCheckPcieLinkUpTx())
            {
                VX_PRINT(VX_ZONE_WARNING, "PCIE TX: PCIe link is not up!\r\n");
                status = VX_FAILURE;
            }

            if(VX_SUCCESS == status)
            {
                
                for(i=0; i<num_items; i++)
                {
                    channel = ((tivx_pcie_tx_params_t *)configuration_target_ptr)->pcie_queue_channel[i];

                    if(appPcieQueueCheckChannelLinkUpTx(channel))
                    {
                        VX_PRINT(VX_ZONE_WARNING, "PCIE TX: Channel %d is not up!\r\n", channel);
                        continue; // Is this correct?
                    }

                    //Get Params and check compatibility
                    appPcieQueueGetChannelParamsTx(channel, &width, &height, &pitch);

                    if( (width != img_input_desc[i]->width) || (height != img_input_desc[i]->height) )
                    {
                        VX_PRINT(VX_ZONE_WARNING, "PCIE TX: Height/Width mismatch at PCIE TX and PCIE RX for channel %d!\r\n", channel);
                        continue; // Is this correct?
                    }

                    if ( !(appPcieQueueDequeueFreeBufferTx(channel, &req_desc)) )
                    {
                        for(j=0;j<img_input_desc[i]->planes;j++)
                        {
                            //Get the address of the SRC_BUF (ie Buffer from Capture)
                            src_addr = (uint64_t)input_target_ptr[i][j];

                            //Get the address of the destination buffer
                            dest_addr = appPcieQueueTranslateToPcieAddressTx(req_desc.rx_buffer_address[j]);

                            if(VX_DF_IMAGE_NV12 == img_input_desc[i]->format)
                            {
                                width_in_bytes = width;

                                /* For 2nd plane, height is halved */
                                if(j == 1)
                                    height = (height / 2);
                            }
                            else if(VX_DF_IMAGE_U16 == img_input_desc[i]->format)
                            {
                                width_in_bytes = (width * 2);
                            }
                            else
                            {
                                /*TO-DO: Add other formats*/
                                VX_PRINT(VX_ZONE_WARNING, "PCIE TX: %d format is not supported!\r\n", img_input_desc[i]->format);
                            }

                            //Setup DMA
                            appUdmaCopy2DPrms_Init(&prms_2d);
                            prms_2d.width        = width_in_bytes;
                            prms_2d.height       = height;
                            prms_2d.dest_pitch   = pitch;
                            prms_2d.src_pitch    = img_input_desc[i]->imagepatch_addr[j].stride_y;
                            prms_2d.dest_addr    = dest_addr;
                            prms_2d.src_addr     = src_addr;
                            appUdmaCopy2D(NULL, &prms_2d, 1);

                            // VX_PRINT(VX_ZONE_WARNING, "0x%llx->0x%llx(0x%llx), w(b)=%4d, h=%4d, s_p=%4d, d_p=%4d\r\n", 
                            //         prms_2d.src_addr, 
                            //         req_desc.sink_buffer_address[j],
                            //         prms_2d.dest_addr,
                            //         prms_2d.width, prms_2d.height,
                            //         prms_2d.src_pitch, prms_2d.dest_pitch);
                        }
                        appPcieQueueEnqueueReadyBufferTx(channel, &req_desc);
                    }
                }
            }
            /* kernel processing function complete */

        }
        tivxMemBufferUnmap(configuration_target_ptr,
           configuration_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);


        for(i=0; i<num_items; i++)
        {
            for(j=0;j<img_input_desc[i]->planes;j++)
            {
                tivxMemBufferUnmap(input_target_ptr[i][j],
                    img_input_desc[i]->mem_size[j], TIVX_MEMORY_TYPE_DMA,
                    VX_READ_ONLY);
            }
        }

    }

    return status;
}

static vx_status VX_CALLBACK tivxPcieTxCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    tivxPcieTxParams *pcieTxParams = NULL;

    if ( (num_params != TIVX_KERNEL_PCIE_TX_MAX_PARAMS)
        || (NULL == obj_desc[TIVX_KERNEL_PCIE_TX_CONFIGURATION_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_PCIE_TX_INPUT_IDX])
    )
    {
        status = VX_FAILURE;
    }
    else
    {
        tivx_obj_desc_user_data_object_t *configuration_desc;

        configuration_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_PCIE_TX_CONFIGURATION_IDX];

        if (configuration_desc->mem_size != sizeof(tivx_pcie_tx_params_t))
        {
            VX_PRINT(VX_ZONE_ERROR, "User data object size on target does not match the size on host, possibly due to misalignment in data structure\r\n");
            status = VX_FAILURE;
        }

        pcieTxParams = tivxMemAlloc(sizeof(tivxPcieTxParams), TIVX_MEM_EXTERNAL);
        if(NULL != pcieTxParams)
        {
            memset(pcieTxParams, 0, sizeof(tivxPcieTxParams));
        }
        else
        {
            VX_PRINT(VX_ZONE_ERROR, "PCIE TX: ERROR: Couldn't allocate memory!\r\n");
            status = VX_ERROR_NO_MEMORY;
        }

        if(VX_SUCCESS == status)
        {
            tivxSetTargetKernelInstanceContext(kernel,
                                               pcieTxParams,
                                               sizeof(tivxPcieTxParams));
        }
        else
        {
            if(NULL != pcieTxParams)
            {
                tivxMemFree(pcieTxParams, sizeof(tivxPcieTxParams), TIVX_MEM_EXTERNAL);
            }
        }

    }

    return status;
}

static vx_status VX_CALLBACK tivxPcieTxDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    tivxPcieTxParams *pcieTxParams = NULL;
    vx_uint32 size;

    if ( (num_params != TIVX_KERNEL_PCIE_TX_MAX_PARAMS)
        || (NULL == obj_desc[TIVX_KERNEL_PCIE_TX_CONFIGURATION_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_PCIE_TX_INPUT_IDX])
    )
    {
        status = VX_FAILURE;
    }
    else
    {
        status = tivxGetTargetKernelInstanceContext(kernel,
                                                    (void *)&pcieTxParams,
                                                    &size);

        if(VX_SUCCESS != status)
        {
            VX_PRINT(VX_ZONE_ERROR, "PCIE TX: ERROR: Could not obtain Pcie Tx kernel instance context!\r\n");
        }
        
        if (NULL == pcieTxParams)
        {
            status = VX_FAILURE;
            VX_PRINT(VX_ZONE_ERROR, "PCIE TX: ERROR: Pcie Tx params is NULL!\r\n");
        }     

        if(sizeof(tivxPcieTxParams) == size)
        {
            tivxMemFree(pcieTxParams, sizeof(tivxPcieTxParams), TIVX_MEM_EXTERNAL);
        }                                               
    }

    return status;
}

static vx_status VX_CALLBACK tivxPcieTxControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;

    return status;
}

void tivxAddTargetKernelPcieTx(void)
{
    vx_status status = VX_FAILURE;
    char target_name[TIVX_TARGET_MAX_NAME];
    vx_enum self_cpu;

    self_cpu = tivxGetSelfCpuId();

    if ( self_cpu == TIVX_CPU_ID_MCU2_0 )
    {
        strncpy(target_name, TIVX_TARGET_MCU2_0, TIVX_TARGET_MAX_NAME);
        status = VX_SUCCESS;
    }
    else
    {
        status = VX_FAILURE;
    }

    if (status == VX_SUCCESS)
    {
        vx_pcie_tx_target_kernel = tivxAddTargetKernelByName(
                            TIVX_KERNEL_PCIE_TX_NAME,
                            target_name,
                            tivxPcieTxProcess,
                            tivxPcieTxCreate,
                            tivxPcieTxDelete,
                            tivxPcieTxControl,
                            NULL);
    }
}


void tivxRemoveTargetKernelPcieTx(void)
{
    vx_status status = VX_SUCCESS;

    status = tivxRemoveTargetKernel(vx_pcie_tx_target_kernel);
    if (status == VX_SUCCESS)
    {
        vx_pcie_tx_target_kernel = NULL;
    }
}


