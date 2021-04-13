/*
 *
 * Copyright (c) 2021 Texas Instruments Incorporated
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
#if !defined(_CM_DLR_NODE_CNTXT_H_)
#define _CM_DLR_NODE_CNTXT_H_

#include <dlr.h>
#include <cm_common.h>

/**
 * \defgroup group_applib_common_dlr Common DLR node setup code.
 * \ingroup group_applib_common
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Constant for Maximum number of inputs supported.
 * \ingroup group_applib_common_dlr
 */
#define CM_DLR_MAX_NUM_INPUTS               (8)

/**
 * \brief Constant for Maximum number of outputs supported.
 * \ingroup group_applib_common_dlr
 */
#define CM_DLR_MAX_NUM_OUTPUTS              (8)

/**
 * \brief Constant for Maximum number of dimensions supported.
 * \ingroup group_applib_common_dlr
 */
#define CM_DLR_MAX_NUM_DIMS                 (4)

/**
 * \brief Constant for representing the dlr node context invalid state.
 * \ingroup group_applib_common_dlr
 */
#define CM_DLR_NODE_CNTXT_STATE_INVALID     (0U)

/**
 * \brief Constant for representing the dlr node context initialization
 *        state.
 * \ingroup group_applib_common_dlr
 */
#define CM_DLR_NODE_CNTXT_STATE_INIT        (1U)

/**
 * \brief Constant for representing the dlr node context setup state.
 * \ingroup group_applib_common_dlr
 */
#define CM_DLR_NODE_CNTXT_STATE_SETUP       (2U)

/**
 * \brief Constant for DLR device type CPU.
 * \ingroup group_applib_common_dlr
 */
#define DLR_DEVTYPE_CPU                     (1U)

/**
 * \brief Constant for DLR device type GPU.
 * \ingroup group_applib_common_dlr
 */
#define DLR_DEVTYPE_GPU                     (2U)

/**
 * \brief Constant for DLR device type OPENCL.
 * \ingroup group_applib_common_dlr
 */
#define DLR_DEVTYPE_OPENCL                  (3U)

/**
 * \brief DLR node create time parameters.
 *
 * \ingroup group_applib_common_dlr
 */
typedef struct
{
    /** Path to the network definition file. */
    char                   *modelPath;

    /** DLR device type.
     *  1 - CPU
     *  2 - GPU
     *  4 - OPENCL
     */
    int32_t                 devType;

    /** DLR device type. */
    int32_t                 devId;

} CM_DLRCreateParams;

/**
 * \brief DLR Interface context.
 *
 * \ingroup group_applib_common_dlr
 */
typedef struct
{
    /** Name. */
    const char             *name;

    /** Size. */
    int64_t                 size;

    /** Dimensions. */
    int32_t                 dim;

    /** Shape information. */
    int64_t                shape[CM_DLR_MAX_NUM_DIMS];

    /** Data buffer. */
    void                   *data;

} CM_DLRIfInfo;

/**
 * \brief DLR node process input parameters.
 *
 * \ingroup group_applib_common_dlr
 */
typedef struct
{
    /** Number of inputs. */
    int32_t                 numInfo;

    /** Input information. */
    CM_DLRIfInfo            info[CM_DLR_MAX_NUM_INPUTS];

} CM_DLRNodeInputInfo;

/**
 * \brief DLR node process output parameters.
 *
 * \ingroup group_applib_common_dlr
 */
typedef struct
{
    /** Number of outputs. */
    int32_t                 numInfo;

    /** Output information. */
    CM_DLRIfInfo            info[CM_DLR_MAX_NUM_OUTPUTS];

} CM_DLRNodeOutputInfo;

/**
 * \brief DLR node context.
 *
 * \ingroup group_applib_common_dlr
 */
typedef struct
{
    /** State variable. */
    uint32_t                state;

    /** DLR model. */
    DLRModelHandle          handle;

    /** Input information. */
    CM_DLRNodeInputInfo     input;

    /** Output information. */
    CM_DLRNodeOutputInfo    output;

    /** Create parameters. */
    CM_DLRCreateParams      params;

} CM_DLRNodeCntxt;

/**
 * \brief Function to initialize the DLR node context.
 *
 * \param [in,out] dlrObj DLR node context.
 *
 * \param [in] context The handle to the openVX implementation context.
 *
 * \param [in] createParams DLR node context create parameters.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_common_dlr
 */
int32_t CM_dlrNodeCntxtInit(
        CM_DLRNodeCntxt           *dlrObj,
        const CM_DLRCreateParams  *createParams);

/**
 * \brief Function to process the data.
 *
 * \param [in,out] dlrObj DLR node context.
 *
 * \param [in]     req Input parameters
 *
 * \param [out]    rsp Output result
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_common_dlr
 */
int32_t CM_dlrNodeCntxtProcess(
        CM_DLRNodeCntxt            *dlrObj,
        const CM_DLRNodeInputInfo  *input,
        CM_DLRNodeOutputInfo       *output);

/**
 * \brief Function to dump the information on the model
 *
 * \param [in,out] dlrObj DLR node context.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_common_dlr
 */
int32_t CM_dlrNodeCntxtDumpInfo(
        CM_DLRNodeCntxt  *dlrObj);

/**
 * \brief Function to de-initialize the DLR node context.
 *
 * \param [in,out] dlrObj DLR node context.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_common_dlr
 */
int32_t CM_dlrNodeCntxtDeInit(
        CM_DLRNodeCntxt  *dlrObj);

#ifdef __cplusplus
}
#endif

#endif /* _CM_DLR_NODE_CNTXT_H_ */

