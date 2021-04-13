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

#include <stdint.h>
#include <stdio.h>

#include <ti/csl/csl_rat.h>

#include <utils/pcie/include/app_pcie_queue.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** The bit in the control variable set by Linux, when PCIe setup is done */
#define CONTROL_RTOS_LINUX_READY            (1 << 0)
/** The bit in the control variable set by application, when channel setup is done */
#define CONTROL_RTOS_PCIE_RX_CHANNEL_READY  (1 << 7)

/** Error code, when the free list is empty */
#define ERROR_FREE_LIST_EMPTY               (-1)
/** Error code, when the free list is full */
#define ERROR_FREE_LIST_FULL                (-2)
/** Error code, when the ready list is empty */
#define ERROR_READY_LIST_EMPTY              (-3)
/** Error code, when the ready list is full */
#define ERROR_READY_LIST_FULL               (-4)
/** Error code, when Linux is not ready */
#define ERROR_LINUX_NOT_READY               (-5)
/** Error code, when channel is not ready */
#define ERROR_CHANNEL_NOT_READY             (-6)

/** RAT register base address for MCU2_1 */
#define ARMSS_RAT_CFG                       (0x0FF90000)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    app_pcie_queue_init_prms_t init_params;
} app_pcie_queue_obj_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static volatile app_pcie_queue_shmem_t g_shmem __attribute__((section(".pcie_queue_shmem")));
static app_pcie_queue_obj_t g_app_pcie_queue_obj;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void appPcieQueueInitSetDefault(app_pcie_queue_init_prms_t *prm)
{
    prm->rat_region_index = APP_PCIE_QUEUE_DEFAULT_RAT_REGION_INDEX;
    prm->rat_mapped_base_address = 0;
    prm->rat_map_size = 0;
}

int32_t appPcieQueueInit(app_pcie_queue_init_prms_t *prm)
{
    int32_t status = 0;

    g_app_pcie_queue_obj.init_params.rat_region_index = prm->rat_region_index;

    if(prm->rat_mapped_base_address == 0)
    {
        status = -1;
        printf("PCIE_QUEUE: Base address for RAT mapping region can't be 0\n");
    }
    else
    {
        g_app_pcie_queue_obj.init_params.rat_mapped_base_address = prm->rat_mapped_base_address;
    }

    if(status == 0)
    {
        if(prm->rat_map_size == 0)
        {
            status = -1;
            printf("PCIE_QUEUE: Size for RAT mapping region can't be 0\n");
        }
        else
            g_app_pcie_queue_obj.init_params.rat_map_size = prm->rat_map_size;
    }

    return status;
}

int32_t appPcieQueueCheckPcieLinkUpRx()
{
    if( !(g_shmem.shmem_rx.control & CONTROL_RTOS_LINUX_READY) )
    {
        return ERROR_LINUX_NOT_READY;
    }

    return 0;
}

int32_t appPcieQueueSetChannelLinkUpRx(uint32_t channel_num)
{
    volatile app_pcie_queue_shmem_t *remoteShmem = (volatile app_pcie_queue_shmem_t *)(uintptr_t)(g_shmem.shmem_rx.remote_shmem_rat_mapped_base_address);

    remoteShmem->shmem_tx.shmem_tx_channel[channel_num].control = CONTROL_RTOS_PCIE_RX_CHANNEL_READY;

    return 0;
}

int32_t appPcieQueueEnqueueFreeBufferRx(uint32_t channel_num, app_pcie_queue_request_descriptor_t *req_desc)
{
    volatile app_pcie_queue_shmem_t *remoteShmem = (volatile app_pcie_queue_shmem_t *)(uintptr_t)(g_shmem.shmem_rx.remote_shmem_rat_mapped_base_address);
    uint64_t producerCount, consumerCount;
    uint32_t index;
    uint32_t i;

    producerCount = remoteShmem->shmem_tx.shmem_tx_channel[channel_num].free_list_producer_count;
    consumerCount = remoteShmem->shmem_tx.shmem_tx_channel[channel_num].free_list_consumer_count;

    if( (producerCount - consumerCount) == APP_PCIE_QUEUE_NUM_BUFFERS )
    {
        return ERROR_FREE_LIST_FULL;
    }

    index = producerCount % APP_PCIE_QUEUE_NUM_BUFFERS;

    for(i=0; i<APP_PCIE_QUEUE_MAX_PLANES; i++)
    {
        remoteShmem->shmem_tx.shmem_tx_channel[channel_num].free_list[index].rx_buffer_address[i] = req_desc->rx_buffer_address[i];
    }
    remoteShmem->shmem_tx.shmem_tx_channel[channel_num].free_list[index].token = req_desc->token;

    //NOTE: Wrap condition not handled
    producerCount++;

    remoteShmem->shmem_tx.shmem_tx_channel[channel_num].free_list_producer_count = producerCount;

    return 0;
}

int32_t appPcieQueueDequeueReadyBufferRx(uint32_t channel_num, app_pcie_queue_request_descriptor_t *req_desc)
{
    volatile app_pcie_queue_shmem_t *remoteShmem = (volatile app_pcie_queue_shmem_t *)(uintptr_t)(g_shmem.shmem_rx.remote_shmem_rat_mapped_base_address);
    uint64_t producerCount, consumerCount;
    uint32_t index;
    uint32_t i;

    producerCount = remoteShmem->shmem_tx.shmem_tx_channel[channel_num].ready_list_producer_count;
    consumerCount = remoteShmem->shmem_tx.shmem_tx_channel[channel_num].ready_list_consumer_count;

    if( (producerCount - consumerCount) == 0 )
    {
        return ERROR_READY_LIST_EMPTY;
    }

    index = consumerCount % APP_PCIE_QUEUE_NUM_BUFFERS;

    for(i=0; i<APP_PCIE_QUEUE_MAX_PLANES; i++)
    {
        req_desc->rx_buffer_address[i] = remoteShmem->shmem_tx.shmem_tx_channel[channel_num].free_list[index].rx_buffer_address[i];
    }
    req_desc->token = remoteShmem->shmem_tx.shmem_tx_channel[channel_num].free_list[index].token;

    //NOTE: Wrap condition not handled
    consumerCount++;

    remoteShmem->shmem_tx.shmem_tx_channel[channel_num].ready_list_consumer_count = consumerCount;
    return 0;
}

int32_t appPcieQueueSetChannelParamsRx(uint32_t channel_num, uint32_t width, uint32_t height,
                                uint32_t pitch)
{
    volatile app_pcie_queue_shmem_t *remoteShmem = (volatile app_pcie_queue_shmem_t *)(uintptr_t)(g_shmem.shmem_rx.remote_shmem_rat_mapped_base_address);
    remoteShmem->shmem_tx.shmem_tx_channel[channel_num].image_height = height;
    remoteShmem->shmem_tx.shmem_tx_channel[channel_num].image_width = width;
    remoteShmem->shmem_tx.shmem_tx_channel[channel_num].pitch = pitch;

    return 0;
}

int32_t appPcieQueueSetupRatRx()
{
    int32_t ret;
    CSL_RatTranslationCfgInfo TranslationCfg;
    TranslationCfg.sizeInBytes = (uint64_t)g_app_pcie_queue_obj.init_params.rat_map_size;
    TranslationCfg.baseAddress = (uint64_t)g_app_pcie_queue_obj.init_params.rat_mapped_base_address;
    TranslationCfg.translatedAddress = (uint64_t)g_shmem.shmem_rx.remote_shmem_pcie_base_address;

    ret = CSL_ratConfigRegionTranslation((CSL_ratRegs *)ARMSS_RAT_CFG, g_app_pcie_queue_obj.init_params.rat_region_index,
                    &TranslationCfg);

    if(ret == (bool)false)
    {
        printf("appPcieSetupRat(): Error in CSL_ratConfigRegionTranslation()\n");
    }

    g_shmem.shmem_rx.remote_shmem_rat_mapped_base_address = g_app_pcie_queue_obj.init_params.rat_mapped_base_address;
    return ret;
}

int32_t appPcieQueueCheckPcieLinkUpTx()
{
    if( !(g_shmem.shmem_tx.control & CONTROL_RTOS_LINUX_READY) )
    {
        return ERROR_LINUX_NOT_READY;
    }

    return 0;
}

int32_t appPcieQueueCheckChannelLinkUpTx(uint32_t channel_num)
{

    if( !(g_shmem.shmem_tx.shmem_tx_channel[channel_num].control & CONTROL_RTOS_PCIE_RX_CHANNEL_READY) )
    {
        return ERROR_CHANNEL_NOT_READY;
    }

    return 0;
}

int32_t appPcieQueueEnqueueReadyBufferTx(uint32_t channel_num, app_pcie_queue_request_descriptor_t *req_desc)
{
    volatile app_pcie_queue_shmem_tx_t *shmemTx = &g_shmem.shmem_tx;
    uint64_t producerCount, consumerCount;
    uint32_t index;
    uint32_t i;

    producerCount = shmemTx->shmem_tx_channel[channel_num].ready_list_producer_count;
    consumerCount = shmemTx->shmem_tx_channel[channel_num].ready_list_consumer_count;

    if( (producerCount - consumerCount) == APP_PCIE_QUEUE_NUM_BUFFERS )
    {
        return ERROR_READY_LIST_FULL;
    }

    index = producerCount % APP_PCIE_QUEUE_NUM_BUFFERS;

    for(i=0; i<APP_PCIE_QUEUE_MAX_PLANES; i++)
    {
        shmemTx->shmem_tx_channel[channel_num].ready_list[index].rx_buffer_address[i] = req_desc->rx_buffer_address[i];
    }
    shmemTx->shmem_tx_channel[channel_num].ready_list[index].token = req_desc->token;

    //NOTE: Wrap condition not handled
    producerCount++;
    shmemTx->shmem_tx_channel[channel_num].ready_list_producer_count = producerCount;

    return 0;
}

int32_t appPcieQueueDequeueFreeBufferTx(uint32_t channel_num, app_pcie_queue_request_descriptor_t *req_desc)
{
    volatile app_pcie_queue_shmem_tx_t *shmemTx = &g_shmem.shmem_tx;
    uint64_t producerCount, consumerCount;
    uint32_t index;
    uint32_t i;

    producerCount = shmemTx->shmem_tx_channel[channel_num].free_list_producer_count;
    consumerCount = shmemTx->shmem_tx_channel[channel_num].free_list_consumer_count;

    if( (producerCount - consumerCount) == 0 )
    {
        return ERROR_FREE_LIST_EMPTY;
    }

    index = consumerCount % APP_PCIE_QUEUE_NUM_BUFFERS;

    for(i=0; i<APP_PCIE_QUEUE_MAX_PLANES; i++)
    {
        req_desc->rx_buffer_address[i] = shmemTx->shmem_tx_channel[channel_num].free_list[index].rx_buffer_address[i];
    }
    req_desc->token = shmemTx->shmem_tx_channel[channel_num].free_list[index].token;

    //NOTE: Wrap condition not handled
    consumerCount++;
    shmemTx->shmem_tx_channel[channel_num].free_list_consumer_count = consumerCount;

    return 0;
}

int32_t appPcieQueueGetChannelParamsTx(uint32_t channel_num, uint32_t *width,
                                    uint32_t *height, uint32_t *pitch)
{
    volatile app_pcie_queue_shmem_tx_t *shmemTx = &g_shmem.shmem_tx;

    *width = shmemTx->shmem_tx_channel[channel_num].image_width;
    *height = shmemTx->shmem_tx_channel[channel_num].image_height;
    *pitch = shmemTx->shmem_tx_channel[channel_num].pitch;

    return 0;
}

uint64_t appPcieQueueTranslateToPcieAddressTx(uint64_t sink_buffer_address)
{
    volatile app_pcie_queue_shmem_tx_t *shmemTx = &g_shmem.shmem_tx;
    uint64_t address;

    address = shmemTx->remote_rx_buffer_space_pcie_base_address +
                (sink_buffer_address - shmemTx->remote_rx_buffer_space_local_base_address);

    return address;
}
