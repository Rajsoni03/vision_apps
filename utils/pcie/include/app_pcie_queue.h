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

#ifndef APP_PCIE_QUEUE_H_
#define APP_PCIE_QUEUE_H_

/**
 * \defgroup group_vision_apps_utils_pcie_queue PCIe Queue Utility APIs (TI-RTOS only)
 *
 * \brief These APIs allow users to send and receive messages over PCIe.
 *
 *        The APIs having 'Tx' as suffix should be called only from the sender
 *        side, and those with 'Rx' should be called only from the receiver side.
 *
 * \ingroup group_vision_apps_utils
 *
 * @{
 */

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Default region index for RAT mapping */
#define APP_PCIE_QUEUE_DEFAULT_RAT_REGION_INDEX         (0)

/** \brief Maximum number of planes in an image. Used for creating array of
*          addresses in a request descriptor
*/
#define APP_PCIE_QUEUE_MAX_PLANES                       (3)
/** \brief Maximum channels supported. One channel means one Tx-Rx pair */
#define APP_PCIE_QUEUE_MAX_CHANNELS                     (1)
/** \brief Queue depth  of Free and Ready Queue */
#define APP_PCIE_QUEUE_NUM_BUFFERS                      (5)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/** \brief Structure of parameters to use when initializing this utility using
*          appPcieQueueInitSetDefault() and appPcieQueueInit()
*/
typedef struct
{
    uint32_t rat_region_index;
    /**< Region index used for RAT mapping */
    uint32_t rat_mapped_base_address;
    /**< Translated address used for RAT mapping */
    uint32_t rat_map_size;
    /**< Translated region size used for RAT mapping */
} app_pcie_queue_init_prms_t;

/** \brief Request Descriptor. The free and ready queues are made of such
*          request descriptors.
*/
typedef struct
{
	uint64_t rx_buffer_address[APP_PCIE_QUEUE_MAX_PLANES];
    /**< Buffer address of each plane (Local address on the Rx side) */
	uint64_t token;
    /**< Token used for identifying the OpenVX object descriptor corresponding to buffers */
} app_pcie_queue_request_descriptor_t;

/** \brief Shared memory for Rx side
*/
typedef struct
{
    uint64_t remote_shmem_local_base_address;
    /**<  Address of remote peer's shared memory (Local address on the remote side) */
    uint64_t remote_shmem_pcie_base_address;
    /**<  PCIe address of remote peer's shared memory */
    uint32_t remote_shmem_size;
    /**<  Size of remote peer's shared memory */
    uint32_t control;
    /**<  Control variable, which will be set by Linux */
    uint32_t remote_shmem_rat_mapped_base_address;
    /**<  RAT mapped address of remote peer's shared memory */
    uint32_t reserved;
    /**<  Reserved variable for 64-bit alignment of this structure */
} app_pcie_queue_shmem_rx_t;

/** \brief Shared memory for each Tx channel
*/
typedef struct
{
    uint32_t control;
    /**<  Channel level control variable */
    uint32_t image_height;
    /**<  Image height set by Rx side for this channel */
    uint32_t image_width;
    /**<  Image width set by Rx side for this channel */
    uint32_t pitch;
    /**<  Pitch set by Rx side for this channel */
    uint64_t free_list_producer_count;
    /**<  Producer count of the free queue. Rx increases this on enqueue. */
    uint64_t free_list_consumer_count;
    /**<  Consumer count of the free queue. Tx increases this on dequeue. */
    uint64_t ready_list_producer_count;
    /**<  Producer count of the ready queue. Tx increases this on enqueue. */
    uint64_t ready_list_consumer_count;
    /**<  Consumer count of the ready queue. Rx increases this on dequeue. */
    app_pcie_queue_request_descriptor_t free_list[APP_PCIE_QUEUE_NUM_BUFFERS];
    /**<  Free queue, made up of requets descriptors*/
    app_pcie_queue_request_descriptor_t ready_list[APP_PCIE_QUEUE_NUM_BUFFERS];
    /**<  Ready queue, made up of requets descriptors*/
} app_pcie_queue_shmem_tx_per_channel_t;

/** \brief Shared memory for Tx side
*/
typedef struct
{
    uint64_t remote_rx_buffer_space_local_base_address;
    /**<  Base address of remote peer's buffer space (Local address on the remote side) */
    uint64_t remote_rx_buffer_space_pcie_base_address;
    /**<  PCIe address corresponding to base address of remote peer's buffer space */
    uint32_t remote_rx_buffer_space_size;
    /**< Size of remote peer's buffer space */
    uint32_t control;
    /**<  Control variable, which will be set by Linux */
    app_pcie_queue_shmem_tx_per_channel_t shmem_tx_channel[APP_PCIE_QUEUE_MAX_CHANNELS];
    /**<  Array of app_pcie_queue_shmem_tx_per_channel_t for each channel*/
} app_pcie_queue_shmem_tx_t;

/** \brief Structure comprehending the whole shared memory
*/
typedef struct
{
    app_pcie_queue_shmem_rx_t shmem_rx;
    /**< Shared memory for Rx */
    app_pcie_queue_shmem_tx_t shmem_tx;
    /**< Shared memory for Tx */
} app_pcie_queue_shmem_t;
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief Initialize prm with default values
 *
 * Users are recommended to call this API before calling appPcieQueueInit()
 *
 * \param prm [OUT] Pointer to initialization parameters
 *
 */
void appPcieQueueInitSetDefault(app_pcie_queue_init_prms_t *prm);

/**
 * \brief Initialize PCIe queue utility
 *
 * \param prm [IN] Pointer to Initialization parameters
 *
 * \return 0 on success, else failure
 */
int32_t appPcieQueueInit(app_pcie_queue_init_prms_t *prm);

/**
 * \brief Check if PCIe link is up on the Rx side
 *
 * \return 0 on success, else failure
 */
int32_t appPcieQueueCheckPcieLinkUpRx();

/**
 * \brief Set channel link up from the Rx side
 *
 * \param channel_num [IN] Channel number
 *
 * \return 0 on success, else failure
 */
int32_t appPcieQueueSetChannelLinkUpRx(uint32_t channel_num);

/**
 * \brief Enqueue request descriptor in the free queue from the Rx side
 *
 * \param channel_num [IN] Channel number
 *
 * \param req_desc [IN] Pointer to request descriptor
 *
 * \return 0 on success, else failure
 */
int32_t appPcieQueueEnqueueFreeBufferRx(
    uint32_t channel_num,
    app_pcie_queue_request_descriptor_t *req_desc);

/**
 * \brief Dequeue request descriptor from the ready queue from the Rx side
 *
 * \param channel_num [IN] Channel number
 *
 * \param req_desc [OUT] Pointer to request descriptor
 *
 * \return 0 on success, else failure
 */
int32_t appPcieQueueDequeueReadyBufferRx(
    uint32_t channel_num,
    app_pcie_queue_request_descriptor_t *req_desc);

/**
 * \brief Set channel specific parameters from the Rx side
 *
 * \param channel_num [IN] Channel number
 *
 * \param width [IN] Image width
 *
 * \param height [IN] Image height
 *
 * \param pitch [IN] Pitch
 *
 * \return 0 on success, else failure
 */
int32_t appPcieQueueSetChannelParamsRx(
    uint32_t channel_num,
    uint32_t width,
    uint32_t height,
    uint32_t pitch);

/**
 * \brief Setup RAT to access remote shared memory from the Rx side
 *
 * \return 0 on success, else failure
 */
int32_t appPcieQueueSetupRatRx();

/**
 * \brief Check if PCIe link is up on the Tx side
 *
 * \return 0 on success, else failure
 */
int32_t appPcieQueueCheckPcieLinkUpTx();

int32_t appPcieQueueCheckChannelLinkUpTx(uint32_t channel_num);

/**
 * \brief Enqueue request descriptor in the ready queue from the Tx side
 *
 * \param channel_num [IN] Channel number
 *
 * \param req_desc [IN] Pointer to request descriptor
 *
 * \return 0 on success, else failure
 */
int32_t appPcieQueueEnqueueReadyBufferTx(
    uint32_t channel_num,
    app_pcie_queue_request_descriptor_t *req_desc);

/**
 * \brief Dequeue request descriptor from the free queue from the Tx side
 *
 * \param channel_num [IN] Channel number
 *
 * \param req_desc [OUT] Pointer to request descriptor
 *
 * \return 0 on success, else failure
 */
int32_t appPcieQueueDequeueFreeBufferTx(
    uint32_t channel_num,
    app_pcie_queue_request_descriptor_t *req_desc);

/**
 * \brief Get channel specific parameters on the Tx side
 *
 * \param channel_num [IN] Channel number
 *
 * \param width [OUT] Image width
 *
 * \param height [OUT] Image height
 *
 * \param pitch [OUT] Pitch
 *
 * \return 0 on success, else failure
 */
int32_t appPcieQueueGetChannelParamsTx(
    uint32_t channel_num,
    uint32_t *width,
    uint32_t *height,
    uint32_t *pitch);

/**
 * \brief Get the PCIe address of the remote buffer using the buffer address
 *        local to remote side
 *
 * \param rx_buffer_address [IN] Buffer address (Local address on the remote side)
 *
 * \return Translated PCIe address
 */
uint64_t appPcieQueueTranslateToPcieAddressTx(uint64_t rx_buffer_address);


/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* @} */

#endif
