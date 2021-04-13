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
#if !defined(_CM_TIDL_NODE_CNTXT_H_)
#define _CM_TIDL_NODE_CNTXT_H_

#include <cm_common.h>

/**
 * \defgroup group_applib_common_tidl Common TIDL node setup code.
 * \ingroup group_applib_common
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Constant for representing the tidl node context invalid state.
 * \ingroup group_applib_common_tidl
 */
#define CM_TIDL_NODE_CNTXT_STATE_INVALID    (0U)

/**
 * \brief Constant for representing the tidl node context initialization
 *        state.
 * \ingroup group_applib_common_tidl
 */
#define CM_TIDL_NODE_CNTXT_STATE_INIT       (1U)

/**
 * \brief Constant for representing the tidl node context setup state.
 * \ingroup group_applib_common_tidl
 */
#define CM_TIDL_NODE_CNTXT_STATE_SETUP      (2U)

/**
 * \brief TIDL node create time parameters.
 *
 * \ingroup group_applib_common_tidl
 */
typedef struct
{
    /** Pipeline depth. */
    uint32_t                pipelineDepth;

} CM_TIDLCreateParams;

/**
 * \brief TIDL node context.
 *
 * \ingroup group_applib_common_tidl
 */
typedef struct
{
    /** State variable. */
    uint32_t                state;

    /** Handle to the TIDL kernel. */
    vx_kernel               vxKernel;

    /** Handle to the TIDL node. */
    vx_node                 vxNode;

    /** Handle to the vx object for holding the TIDL configuration
     *  parameters.
     */
    vx_user_data_object     vxConfig;

    /** Handle to the vx object for holding the TIDL network
     *  parameters.
     */
    vx_user_data_object     vxNetwork;

    /** Handle to the vx object for holding the TIDL creation
     *  parameters.
     */
    vx_user_data_object     vxCreateParams;

    /** Handle to the vx object for holding the TIDL Input Args
     *  parameters.
     */
    vx_user_data_object     vxInArgs;

    /** Handle to the vx object for holding the TIDL Output Args
     *  parameters.
     */
    vx_user_data_object     vxOutArgs;

    /** Array of handles to the tensor objects. The number of handles equals
     *  to numOutTensors.
     */
    //vx_tensor              *vxOutputTensor;

    /** Number of input tensors. */
    uint8_t                 numInTensors;

    /** Number of output tensors. */
    uint8_t                 numOutTensors;

    /** Path to the configuration definition file.*/
    char                   *tidlCfgFilePath;

    /** Path to the network definition file. */
    char                   *tidlNwFilePath;

    /** Pipeline depth. */
    uint8_t                 pipelineDepth;

    /** Pre-computed index for identifying the index of the first output
     *  parameter. This is used when setting the depth of the pipeline
     *  for the output parameters.
     */
    uint8_t                 outTensorBaseParamIdx;

    /** Size of the memory allocated for holding a 2d-array of output tensors. 
     *  This is saved since the size is needed when releasing the memory
     *  during the de-init time.
     */
    uint32_t                vxOutTensorMemSize;

    /** Size of the memory allocated for holding a 2d-array of output tensors. 
     *  This is saved since the size is needed when releasing the memory
     *  during the de-init time.
     */
    uint32_t                vxOutTensorRefMemSize;
    /** 
     * A 2d-array of output tensors.
     * vxOutputTensor[pipelineDepth][numOutTensors]
     */
    vx_tensor             **vxOutputTensor;

    /** 
     * A 2d-array of output tensors.
     * vxOutputTensorRefs[numOutTensors][pipelineDepth]
     *
     * Need to give the list of buffers for each output so need this structure.
     * This array just points to the objects in vxOutputImage.
     */
    vx_tensor             **vxOutputTensorRefs;

} CM_TIDLNodeCntxt;

/**
 * \brief Function to initialize the TIDL node context.
 *
 * \param [in,out] tidlObj TIDL node context.
 *
 * \param [in] context The handle to the openVX implementation context.
 *
 * \param [in] createParams TIDL node context create parameters.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_common_tidl
 */
vx_status CM_tidlNodeCntxtInit(
        CM_TIDLNodeCntxt           *tidlObj,
        vx_context                  context,
        const CM_TIDLCreateParams  *createParams);

/**
 * \brief Function to create and setup the TIDL node.
 *        
 *        The node should have been initialized by calling
 *        CM_tidlNodeCntxtInit() API, prior to invoking this API.
 *
 * \param [in,out] tidlObj TIDL node context.
 *
 * \param [in] context The handle to the openVX implementation context.
 *
 * \param [in] graph The handle to the graph this node belongs to.
 *
 * \param [in] inputTensor Input tensor parameter to the node.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_common_tidl
 */
vx_status CM_tidlNodeCntxtSetup(
        CM_TIDLNodeCntxt   *tidlObj,
        vx_context          context,
        vx_graph            graph,
        vx_tensor          *inputTensor);

/**
 * \brief Function to create input tensors to the TIDL node. This is a helper
 *        function to be used by the source to the TIDL node.
 *
 * \param [in] context The handle to the openVX implementation context.
 *
 * \param [in] config Handle to the user data object carrying TIDL
 *                    configuration.
 *
 * \param [in] pad_in_tidl Flag to indicate if TIDL node will handle the
 *                         padding of the input.
 *
 * \param [out] tensors Array of tensor handles. These are valid only if the
 *                      return status if VX_SUCCESS.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_common_tidl
 */
vx_status CM_tidlNodeCnxtCreateInputTensors(
        vx_context                  context,
        const vx_user_data_object   config,
        vx_int32                    pad_in_tidl,
        vx_tensor                  *tensors);

/**
 * \brief Function to create output tensors of the TIDL node. This is a helper
 *        function to be used by the sink to the TIDL node.
 *
 * \param [in] context The handle to the openVX implementation context.
 *
 * \param [in] config Handle to the user data object carrying TIDL
 *                    configuration.
 *
 * \param [out] tensors Array of tensor handles. These are valid only if the
 *                      return status if VX_SUCCESS.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_common_tidl
 */
vx_status CM_tidlNodeCnxtCreateOutputTensors(
        vx_context                  context,
        const vx_user_data_object   config,
        vx_tensor                  *tensors);

/**
 * \brief Function to de-initialize the TIDL node context.
 *
 * \param [in,out] tidlObj TIDL node context.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_common_tidl
 */
vx_status CM_tidlNodeCntxtDeInit(
        CM_TIDLNodeCntxt  *tidlObj);

#ifdef __cplusplus
}
#endif

#endif /* _CM_TIDL_NODE_CNTXT_H_ */

