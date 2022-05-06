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
#include "tivx_kernel_pcie_rx.h"
#include "TI/tivx_target_kernel.h"
#include "tivx_kernels_target_utils.h"

#include "TI/tivx_task.h"
#include <utils/pcie/include/app_pcie_queue.h>

#define PCIE_RX_POLLING_INTERVAL    (1)
#define PCIE_TX_NOT_INITIALIZED     (0U)   
#define PCIE_TX_INITIALIZED         (1U)
#define RAT_SETUP_NOT_DONE          (0U)
#define RAT_SETUP_DONE              (1U)

typedef struct
{
    uint32_t pcie_tx_initialized[TIVX_KERNEL_PCIE_RX_MAX_CHANNELS];
    uint32_t pcie_tx_initialized_all_channels;
    uint32_t rat_setup;
} tivxPcieRxParams;

static tivx_target_kernel vx_pcie_rx_target_kernel = NULL;

static vx_status VX_CALLBACK tivxPcieRxProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxPcieRxCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxPcieRxDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxPcieRxControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);

static vx_status tivxPcieRxEnqueueBufferToPcieTx(
       tivx_obj_desc_object_array_t *output_desc,
       tivx_obj_desc_image_t *img_output_desc[],
       void *output_target_ptr[TIVX_OBJECT_ARRAY_MAX_ITEMS][TIVX_IMAGE_MAX_PLANES],
        void *configuration_target_ptr,
       tivxPcieRxParams *pcieRxParams);

static vx_status tivxPcieRxEnqueueBufferToPcieTx(
       tivx_obj_desc_object_array_t *output_desc,
       tivx_obj_desc_image_t *img_output_desc[],
       void *output_target_ptr[TIVX_OBJECT_ARRAY_MAX_ITEMS][TIVX_IMAGE_MAX_PLANES],
       void *configuration_target_ptr,
       tivxPcieRxParams *pcieRxParams)
{
    vx_status status = VX_SUCCESS;
    vx_uint32 i;
    vx_uint32 j;
    vx_uint32 channel;
    app_pcie_queue_request_descriptor_t req_desc;

    for(i=0; i<output_desc->num_items; i++)
    {
        channel = ((tivx_pcie_rx_params_t *)configuration_target_ptr)->pcie_queue_channel[i];

        if( PCIE_TX_NOT_INITIALIZED == pcieRxParams->pcie_tx_initialized[i])
        {
            VX_PRINT(VX_ZONE_WARNING, "PCIE TX: PCIE RX for channel %d is not up!\r\n", channel);
            continue; // Is this correct?
        }

        for(j=0; j<img_output_desc[i]->planes; j++)
        {
            req_desc.rx_buffer_address[j] = (vx_uint64)output_target_ptr[i][j];
        }
        req_desc.token = (vx_uint64)output_desc;

        status = appPcieQueueEnqueueFreeBufferRx(channel, &req_desc);

        if (0 != status)
        {
            // status = VX_FAILURE; // Is this correct?
            VX_PRINT(VX_ZONE_ERROR, "PCIE RX: ERROR: Enqueue Frame to PCIe TX failed !!!\r\n");
        }
    }

    return status;
}

static vx_status VX_CALLBACK tivxPcieRxProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    tivx_obj_desc_user_data_object_t *configuration_desc;
    tivx_obj_desc_object_array_t *output_desc;
    tivx_obj_desc_image_t *img_output_desc[TIVX_OBJECT_ARRAY_MAX_ITEMS];
    vx_uint32 i;
    vx_uint32 j;
    vx_enum state;
    vx_uint32 channel;
    tivxPcieRxParams *pcieRxParams = NULL;
    vx_uint32 size;
    app_pcie_queue_request_descriptor_t req_desc;

    if ( (num_params != TIVX_KERNEL_PCIE_RX_MAX_PARAMS)
        || (NULL == obj_desc[TIVX_KERNEL_PCIE_RX_CONFIGURATION_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_PCIE_RX_OUTPUT_IDX])
    )
    {
        status = VX_FAILURE;
    }

    if(VX_SUCCESS == status)
    {
        configuration_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_PCIE_RX_CONFIGURATION_IDX];
        output_desc = (tivx_obj_desc_object_array_t *)obj_desc[TIVX_KERNEL_PCIE_RX_OUTPUT_IDX];

    }

    if(VX_SUCCESS == status)
    {

        void *configuration_target_ptr;
        void *output_target_ptr[TIVX_OBJECT_ARRAY_MAX_ITEMS][TIVX_IMAGE_MAX_PLANES] = {NULL};

        configuration_target_ptr = tivxMemShared2TargetPtr(&configuration_desc->mem_ptr);
        tivxMemBufferMap(configuration_target_ptr,
           configuration_desc->mem_size, VX_MEMORY_TYPE_HOST,
           VX_READ_ONLY);

        tivxGetObjDescList(output_desc->obj_desc_id, (tivx_obj_desc_t**)img_output_desc, output_desc->num_items);
        for(i=0; i<output_desc->num_items; i++)
        {
            for(j=0; j<img_output_desc[i]->planes; j++)
            {
                output_target_ptr[i][j] = tivxMemShared2TargetPtr(&img_output_desc[i]->mem_ptr[j]);
                tivxMemBufferMap(output_target_ptr[i][j],
                    img_output_desc[i]->mem_size[j], TIVX_MEMORY_TYPE_DMA,
                    VX_WRITE_ONLY);
            }
        }

        {

            /* call kernel processing function */
            if(VX_SUCCESS == status)
            {
                status = tivxGetTargetKernelInstanceContext(kernel,
                                                            (void *)&pcieRxParams,
                                                            &size);

                if((VX_SUCCESS != status) ||
                (NULL == pcieRxParams) ||
                (sizeof(tivxPcieRxParams) != size))
                {
                    status = VX_FAILURE;
                }
                else
                {
                    status = tivxGetTargetKernelInstanceState(kernel, &state);
                }
            }

            if(VX_SUCCESS == status)
            {
                while(appPcieQueueCheckPcieLinkUpRx())
                {
                    VX_PRINT(VX_ZONE_WARNING, "PCIE RX: PCIe link is not up!\r\n");
                    tivxTaskWaitMsecs(PCIE_RX_POLLING_INTERVAL);
                }
            }

            /* One-time RAT setup */
            if (VX_SUCCESS == status)
            {
                if ( RAT_SETUP_NOT_DONE == pcieRxParams->rat_setup )
                {
                    appPcieQueueSetupRatRx();
                    pcieRxParams->rat_setup = RAT_SETUP_DONE;
                }
            }

            /* Sends start control to PCIe Tx */
            if (VX_SUCCESS == status)
            {

                if( PCIE_TX_NOT_INITIALIZED == pcieRxParams->pcie_tx_initialized_all_channels)
                {
                    uint32_t all_channels_initialized = PCIE_TX_INITIALIZED;
                    for(i=0; i<output_desc->num_items; i++)
                    {
                        channel = ((tivx_pcie_rx_params_t *)configuration_target_ptr)->pcie_queue_channel[i];

                        if (PCIE_TX_NOT_INITIALIZED == pcieRxParams->pcie_tx_initialized[i])
                        {
                            VX_PRINT(VX_ZONE_WARNING, "PCIE RX: Setting up channel %d!\r\n", channel);
                            VX_PRINT(VX_ZONE_WARNING, "PCIE RX: width = %d!\r\n", img_output_desc[i]->width);
                            VX_PRINT(VX_ZONE_WARNING, "PCIE RX: height= %d!\r\n", img_output_desc[i]->height);
                            VX_PRINT(VX_ZONE_WARNING, "PCIE RX: pitch = %d!\r\n\n", img_output_desc[i]->imagepatch_addr[0].stride_y);

                            appPcieQueueSetChannelParamsRx(channel,
                            img_output_desc[i]->width, img_output_desc[i]->height,
                            img_output_desc[i]->imagepatch_addr[0].stride_y);

                            appPcieQueueSetChannelLinkUpRx(channel);

                            pcieRxParams->pcie_tx_initialized[i] = PCIE_TX_INITIALIZED;
                        }
                    }
                    pcieRxParams->pcie_tx_initialized_all_channels = all_channels_initialized;
                }
            }

            if(VX_SUCCESS == status)
            {
                /* Steady state: receives a buffer and returns a buffer */
                if (VX_NODE_STATE_STEADY == state)
                {
                    status = tivxPcieRxEnqueueBufferToPcieTx(output_desc, (tivx_obj_desc_image_t **)img_output_desc, output_target_ptr, configuration_target_ptr, pcieRxParams);

                    /* Pends until a frame is available then dequeue frames from PCIe Tx */
                    if (VX_SUCCESS == status)
                    {
                        for(i=0; i<output_desc->num_items; i++)
                        {
                            channel = ((tivx_pcie_rx_params_t *)configuration_target_ptr)->pcie_queue_channel[i];

                            while(1)
                            {
                                if ( !appPcieQueueDequeueReadyBufferRx(channel, &req_desc))
                                {
                                    obj_desc[TIVX_KERNEL_PCIE_RX_OUTPUT_IDX] = (tivx_obj_desc_t *)(uintptr_t)req_desc.token;
                                    break;
                                }
                                tivxTaskWaitMsecs(PCIE_RX_POLLING_INTERVAL);
                            }
                        }
                    }
                }
                /* Pipe-up state: only receives a buffer; does not return a buffer */
                else
                {
                    status = tivxPcieRxEnqueueBufferToPcieTx(output_desc, (tivx_obj_desc_image_t **)img_output_desc, output_target_ptr, configuration_target_ptr, pcieRxParams);
                }
            }
            /* kernel processing function complete */
        }
        tivxMemBufferUnmap(configuration_target_ptr,
           configuration_desc->mem_size, VX_MEMORY_TYPE_HOST,
            VX_READ_ONLY);

        for(i=0; i<output_desc->num_items; i++)
        {
            for(j=0; j<img_output_desc[i]->planes; j++)
            {
                tivxMemBufferUnmap(output_target_ptr[i][j],
                   img_output_desc[i]->mem_size[j], TIVX_MEMORY_TYPE_DMA,
                    VX_WRITE_ONLY);
            }
        }
    }

    return status;
}

static vx_status VX_CALLBACK tivxPcieRxCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    tivxPcieRxParams *pcieRxParams = NULL;
    vx_uint32 i;

    if ( (num_params != TIVX_KERNEL_PCIE_RX_MAX_PARAMS)
        || (NULL == obj_desc[TIVX_KERNEL_PCIE_RX_CONFIGURATION_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_PCIE_RX_OUTPUT_IDX])
    )
    {
        status = VX_FAILURE;
    }
    else
    {
        tivx_obj_desc_user_data_object_t *configuration_desc;

        configuration_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_PCIE_RX_CONFIGURATION_IDX];

        if (configuration_desc->mem_size != sizeof(tivx_pcie_rx_params_t))
        {
            VX_PRINT(VX_ZONE_ERROR, "User data object size on target does not match the size on host, possibly due to misalignment in data structure\r\n");
            status = VX_FAILURE;
        }

        pcieRxParams = tivxMemAlloc(sizeof(tivxPcieRxParams), TIVX_MEM_EXTERNAL);
        if(NULL != pcieRxParams)
        {
            memset(pcieRxParams, 0, sizeof(tivxPcieRxParams));
        }
        else
        {
            VX_PRINT(VX_ZONE_ERROR, "PCIE RX: ERROR: Couldn't allocate memory!\r\n");
            status = VX_ERROR_NO_MEMORY;
        }

        if(VX_SUCCESS == status)
        {
            for(i=0; i<TIVX_KERNEL_PCIE_RX_MAX_CHANNELS; i++)
            {
                pcieRxParams->pcie_tx_initialized[i] = PCIE_TX_NOT_INITIALIZED;
            }
            pcieRxParams->pcie_tx_initialized_all_channels = PCIE_TX_NOT_INITIALIZED;

            pcieRxParams->rat_setup = RAT_SETUP_NOT_DONE;
        }

        if(VX_SUCCESS == status)
        {
            tivxSetTargetKernelInstanceContext(kernel,
                                               pcieRxParams,
                                               sizeof(tivxPcieRxParams));
        }
        else
        {
            if(NULL != pcieRxParams)
            {
                tivxMemFree(pcieRxParams, sizeof(tivxPcieRxParams), TIVX_MEM_EXTERNAL);
            }
        }
    }

    return status;
}

static vx_status VX_CALLBACK tivxPcieRxDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;

    tivxPcieRxParams *pcieRxParams = NULL;
    vx_uint32 size;

    if ( (num_params != TIVX_KERNEL_PCIE_RX_MAX_PARAMS)
        || (NULL == obj_desc[TIVX_KERNEL_PCIE_RX_CONFIGURATION_IDX])
        || (NULL == obj_desc[TIVX_KERNEL_PCIE_RX_OUTPUT_IDX])
    )
    {
        status = VX_FAILURE;
    }
    else
    {
        status = tivxGetTargetKernelInstanceContext(kernel,
                                                    (void *)&pcieRxParams,
                                                    &size);

        if(VX_SUCCESS != status)
        {
            VX_PRINT(VX_ZONE_ERROR, "PCIE RX: ERROR: Could not obtain Pcie Rx kernel instance context!\r\n");
        }

        if (NULL == pcieRxParams)
        {
            status = VX_FAILURE;
            VX_PRINT(VX_ZONE_ERROR, "PCIE RX: ERROR: Pcie Rx params is NULL!\r\n");
        }

        if(sizeof(tivxPcieRxParams) == size)
        {
            tivxMemFree(pcieRxParams, sizeof(tivxPcieRxParams), TIVX_MEM_EXTERNAL);
        }
    }

    return status;
}

static vx_status VX_CALLBACK tivxPcieRxControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;

    return status;
}

void tivxAddTargetKernelPcieRx(void)
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
        vx_pcie_rx_target_kernel = tivxAddTargetKernelByName(
                            TIVX_KERNEL_PCIE_RX_NAME,
                            target_name,
                            tivxPcieRxProcess,
                            tivxPcieRxCreate,
                            tivxPcieRxDelete,
                            tivxPcieRxControl,
                            NULL);
    }
}

void tivxRemoveTargetKernelPcieRx(void)
{
    vx_status status = VX_SUCCESS;

    status = tivxRemoveTargetKernel(vx_pcie_rx_target_kernel);
    if (status == VX_SUCCESS)
    {
        vx_pcie_rx_target_kernel = NULL;
    }
}


