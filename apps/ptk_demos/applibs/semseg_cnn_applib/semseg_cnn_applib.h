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

#ifndef _SEMSEG_CNN_APPLIB_H_
#define _SEMSEG_CNN_APPLIB_H_

#include <TI/tivx.h>
#include <app_ptk_demo_common.h>
#include <cm_scaler_node_cntxt.h>
#include <cm_preproc_node_cntxt.h>
#include <cm_tidl_node_cntxt.h>
#include <cm_postproc_node_cntxt.h>
#include <cm_dlr_node_cntxt.h>

/**
 * \defgroup group_applib_semseg_cnn Semantic Segmentation APPLIB code.
 * \ingroup group_ptk_applib
 *
 */

/* Scaler node completion event. */
#define SEMSEG_CNN_APPLIB_SCALER_NODE_COMP_EVT  (0U)

/* A user event for the calling app to know that output data is available. */
#define SEMSEG_CNN_APPLIB_OUT_AVAIL_EVT         (SEMSEG_CNN_APPLIB_SCALER_NODE_COMP_EVT + 1)

/**
 * \brief Semantic Segmentation APPLIB create parameter context.
 *
 * \ingroup group_applib_semseg_cnn
 */
typedef struct
{
    /** OpenVX context. */
    vx_context          vxContext;

    /** Graph handle from the Application. If this is NULL, the the APPLIB will
     * create an internal graph and will manage it.
     *
     * In case the Application would like to manage the graph but want the
     * applib to create a sub-graph and attach it to the graph then this 
     * should be allocated by the Application and pass it in.
     */
    vx_graph            vxGraph;

    /** Flag to indicate if LDC node will need to be created. If true, then
     *  the LDC node will be the head node, otherwise the scaler node will
     *  be the head node.
     */
    uint8_t             enableLdcNode;

    /** Flag to indicate if PostProc node will need to be created. If true, then
     *  the TIDL node will be the tail node, otherwise the PostProc node will
     *  be the tail node.
     */
    uint8_t             enablePostProcNode;

    /** Input object creation control. */
    uint8_t             createInputFlag;

    /* Input Image. This needs to be provided if vxGraph is not NULL. */
    vx_image            vxInputImage[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Input image width in pixels. */
    int32_t             inputImageWidth;

    /** Input image height in pixels. */
    int32_t             inputImageHeight;

    /** LDC sub-sampling factor. Ignored if enableLdcNode is false. */
    uint32_t            ldcSsFactor;

    /** LDC block width. Ignored if enableLdcNode is false. */
    uint32_t            ldcBlockWidth;

    /** LDC block height. Ignored if enableLdcNode is false. */
    uint32_t            ldcBlockHeight;

    /** Pixed padding. Ignored if enableLdcNode is false. */
    uint32_t            ldcPixelPad;

    /** DL image width in pixels. */
    int32_t             tidlImageWidth;

    /** DL image height in pixels. */
    int32_t             tidlImageHeight;

    /** Output image width in pixels. */
    int32_t             outImageWidth;

    /** Output image height in pixels. */
    int32_t             outImageHeight;

    /** Number of classes to dectect */
    uint8_t             numClasses;

    /** Mean values to be used in pre-processing stage. */
    float               preProcMean[CM_PREPROC_MAX_IMG_CHANS];

    /** Scaling values to be used in pre-processing stage. */
    float               preProcScale[CM_PREPROC_MAX_IMG_CHANS];

    /** Flag to indicate if padding of the input image will be done inside the
     *  TIDL.
     *  0 - Padding will not be handled inside TIDL
     *  1 - Padding will be handled inside TIDL
     */
    uint8_t             padInTidl;

    /** Path to the LDC LUT definition file.*/
    char               *ldcLutFilePath;

    /** Path to the folder containing the model files or colon-separated list
     *  of folders (or files) if model files stored in different locations.
     */
    char               *dlrModelPath;

    /** DLR device type.
     *  1 - CPU
     *  2 - GPU
     *  4 - OPENCL
     */
    int32_t             dlrDevType;

    /** DLR device type. */
    int32_t             dlrDevId;

    /** DLR node context object. */
    CM_DLRNodeCntxt    *dlrObj;

    /** Base value to be used for any programmed VX events. */
    uint32_t            vxEvtAppValBase;

    /** pipeline depth */
    uint8_t             pipelineDepth;

    /** Flag to indicate if the graph should be exported
     * 0 - disable
     * 1 - enable
     */
    uint8_t             exportGraph;

    /** Real-time logging enable.
     * 0 - disable
     * 1 - enable
     */
    uint8_t             rtLogEnable;

} SEMSEG_CNN_APPLIB_createParams;

struct  SEMSEG_CNN_APPLIB_Context;
typedef SEMSEG_CNN_APPLIB_Context * SEMSEG_CNN_APPLIB_Handle;

/**
 * \brief Function to initialize the APPLIB.
 *
 * \param [in] createParams APPLIB create parameters.
 *
 * \return Valid handle on success. NULL otherwise.
 *
 * \ingroup group_applib_semseg_cnn
 */
SEMSEG_CNN_APPLIB_Handle SEMSEG_CNN_APPLIB_create(
        SEMSEG_CNN_APPLIB_createParams *createParams);

/**
 * \brief Function to process an input message. This function is non-blocking
 *        and returns success if the pipeline is not full.
 *
 * \param [in] handle APPLIB handle.
 *
 * \param [in] inputImg Input image to process.
 * 
 * \param [in] timestamp Timestamp of input image to process.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_semseg_cnn
 */
vx_status SEMSEG_CNN_APPLIB_process(
        SEMSEG_CNN_APPLIB_Handle    handle,
        vx_image                    inputImg,
        uint64_t                    timestamp);

/**
 * \brief Function to print the performance statistics to stdout.
 *
 * \param [in] handle APPLIB handle.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_semseg_cnn
 */
vx_status SEMSEG_CNN_APPLIB_printStats(
        SEMSEG_CNN_APPLIB_Handle    handle);

/**
 * \brief Function to export the performance statistics to a file.
 *
 * \param [in] handle APPLIB handle.
 *
 * \param [in,out] fp File handle.
 *
 * \param [in] exportAll Flag to indicate if the memory usage statistics need
 *                       to be exported as well. If this flag is false, then
 *                       only the graph level performance stats will be
 *                       exported.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_semseg_cnn
 */
vx_status SEMSEG_CNN_APPLIB_exportStats(
        SEMSEG_CNN_APPLIB_Handle    handle,
        FILE                       *fp,
        bool                        exportAll);

/**
 * \brief Function to release the output buffer at the head of the output queue.
 *        Once the graph execution is complete, the output buffer is stored in 
 *        a queue for the use by the calling application. The caller is 
 *        responsible for calling this function to release the output buffers
 *        in a timely manner. Failing to do will result in subsequent graph
 *        execution failures due to non-availability of the output buffers.
 *
 * \param [in] handle APPLIB handle.
 *
 * \ingroup group_applib_semseg_cnn
 */
vx_status SEMSEG_CNN_APPLIB_releaseOutBuff(
        SEMSEG_CNN_APPLIB_Handle    handle);

/**
 * \brief Function to returns references to the {input, output} images at the
 *        head of the output queue. The data is not popped out of the queue.
 *        If the caller calls this functions repeatedly without calling
 *        SEMSEG_CNN_APPLIB_releaseOutBuff() in between, then the same output
 *        is returned.
 *
 *        Once the graph execution is complete, the output buffer is stored in 
 *        a queue for the use by the calling application. The caller can use
 *        this API to get a reference to the input/output image pair for post
 *        processing. One the caller is done with the input/output pair, a call
 *        must be made to SEMSEG_CNN_APPLIB_releaseOutBuff()for releasing the
 *        buffered input/output pair. Failing to do will result in subsequent
 *        graph execution failures due to non-availability of the output
 *        buffers.
 *
 * \param [in] handle APPLIB handle.
 *
 * \param [out] inputImg Input image passed to the graph.
 *
 * \param [out] output Reference to the output object from the graph
 *              corresponding to the 'inputImage'. The reference could
 *              be either a tensor or an image. See following.
 *              - output refere to a tensor if enablePostProcNode is false
 *              - output refere to an image if enablePostProcNode is true
 * 
 * \param [out] timestamp Timestamp of inputImg
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_semseg_cnn
 */
vx_status SEMSEG_CNN_APPLIB_getOutBuff(
        SEMSEG_CNN_APPLIB_Handle    handle,
        vx_image                   *inputImage,
        vx_reference               *output,
        vx_uint64                  *timestamp);

/**
 * \brief Function to copies the {input, output} images at the head of the
 *        output queue. The data is not popped out of the queue. If the caller
 *        calls this functions repeatedly without calling 
 *        SEMSEG_CNN_APPLIB_releaseOutBuff() in between, then the same output
 *        is returned.
 *
 *        Once the graph execution is complete, the output buffer is stored in 
 *        a queue for the use by the calling application. The caller can use
 *        this API to get a reference to the input/output image pair for post
 *        processing. One the caller is done with the input/output pair, a call
 *        must be made to SEMSEG_CNN_APPLIB_releaseOutBuff()for releasing the
 *        buffered input/output pair. Failing to do will result in subsequent
 *        graph execution failures due to non-availability of the output
 *        buffers.
 *
 * \param [in] handle APPLIB handle.
 *
 * \param [out] inputImg Image to copy the input image passed to the graph
 *
 * \param [out] outputImage Output image from the graph corresponding to the
 *              'inputImage'.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_semseg_cnn
 */
vx_status SEMSEG_CNN_APPLIB_getOutBuff(
        SEMSEG_CNN_APPLIB_Handle    handle,
        vx_image                    inputImage,
        vx_image                    outputImage);

/**
 * \brief Function for processing the events programmed to be handled by the
 *        APPLIB. Currently only VX_EVENT_GRAPH_COMPLETED event is handled.
 *
 *        The caller must invoke this handler when it receives a graph
 *        completion event. The APPLIB needs to deueue the graph parameters
 *        and buffer the output for use by the caller and these tasks are
 *        performed inside this handler.
 *
 * \param [in] handle APPLIB handle.
 *
 * \param [in] event Event context.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_semseg_cnn
 */
vx_status SEMSEG_CNN_APPLIB_processEvent(
        SEMSEG_CNN_APPLIB_Handle    handle,
        vx_event_t                 *event);

/**
 * \brief Function to reset the performance and other APPLIB specific counters.
 *
 * \param [in] handle APPLIB handle.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_semseg_cnn
 */
vx_status SEMSEG_CNN_APPLIB_reset(
        SEMSEG_CNN_APPLIB_Handle   handle);

/**
 * \brief Function to wait for the pending graph execution completions. This is
 *        relavant only if the graph is created and managed by the APPLIB.
 *
 *        If the graph is created and manged by the APPLIB, then this function
 *        could potentially block until the current graph execution is
 *        is complete.
 *
 *        If the graph is created and managed by the caller, then this function
 *        always returns immediately with status VX_SUCCESS.
 *
 * \param [in] handle APPLIB handle.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_semseg_cnn
 */
vx_status SEMSEG_CNN_APPLIB_waitGraph(
        SEMSEG_CNN_APPLIB_Handle   handle);

/**
 * \brief Function to de-init the APPLIB and release the memory associated with
 *        the handle.
 *
 * \param [in,out] handle Reference to APPLIB handle.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_semseg_cnn
 */
vx_status SEMSEG_CNN_APPLIB_delete(
        SEMSEG_CNN_APPLIB_Handle   *handle);


/**
 * \brief Function to convert the scaler output to RGB
 *
 * \param [in] vxScalerOut Scaler (MSC) output image
 *
 * \param [out] dlrInputBuff RGB data from the scaler output image
 *
 * \param [in] createParams Applib create params
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_semseg_cnn
 */
vx_status SEMSEG_CNN_APPLIB_preProcess(vx_image                         vxScalerOut,
                                       float                           *dlrInputBuff,
                                       SEMSEG_CNN_APPLIB_createParams  *createParams);


/**
 * \brief Function to create color-coded semantic segmantaion map
 *
 * \param [in, out] vxScalerOut Scaler (MSC) output image
 *
 * \param [in] dlrOutputBuff DLR modle output that includes sematic segmentation classes
 *
 * \param [in] createParams Applib create params
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_semseg_cnn
 */
vx_status SEMSEG_CNN_APPLIB_postProcess(vx_image                        vxScalerOut,
                                        int32_t                        *dlrOutputBuff,
                                        SEMSEG_CNN_APPLIB_createParams *createParams);


/**
 * \brief Function to convert semantic sementation output to tensor
 *
 * \param [out] vxOutTensor tensor object created from dlrOutputBuff
 *
 * \param [in] dlrOutputBuff DLR modle output that includes sematic segmentation classes
 *
 * \param [in] createParams Applib create params
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_semseg_cnn
 */
vx_status SEMSEG_CNN_APPLIB_createOutTensor(vx_tensor                       vxOutTensor,
                                            int32_t                        *dlrOutputBuff,
                                            SEMSEG_CNN_APPLIB_createParams *createParams);

/**
 * \brief Function to get scaler node context
 *
 * \param [in,out] handle APPLIB handle.
 *
 * \return Scaler node context
 *
 * \ingroup group_applib_semseg_cnn
 */
CM_ScalerNodeCntxt * SEMSEG_CNN_APPLIB_getScalerNodeCntxt(
        SEMSEG_CNN_APPLIB_Handle handle);



/**
 * \brief Function to get reference to the output tensor array.
 *
 * \param [in] handle APPLIB handle.
 *
 * \return vx_tensor
 *
 * \ingroup group_applib_semseg_cnn
 */
vx_tensor * SEMSEG_CNN_APPLIB_getOutputTensor(
        SEMSEG_CNN_APPLIB_Handle handle);

/**
 * \brief Function to get reference to the DLR Node input info.
 *
 * \param [in] handle APPLIB handle.
 *
 * \return CM_DLRNodeInputInfo
 *
 * \ingroup group_applib_semseg_cnn
 */
CM_DLRNodeInputInfo * SEMSEG_CNN_APPLIB_getDLRNodeInputInfo(
        SEMSEG_CNN_APPLIB_Handle handle);

/**
 * \brief Function to get reference to the DLR Node output info.
 *
 * \param [in] handle APPLIB handle.
 *
 * \return CM_DLRNodeOutputInfo
 *
 * \ingroup group_applib_semseg_cnn
 */
CM_DLRNodeOutputInfo * SEMSEG_CNN_APPLIB_getDLRNodeOutputInfo(
        SEMSEG_CNN_APPLIB_Handle handle);


/**
 * \brief Function to get the DLR input buffer
 *
 * \param [in] handle APPLIB handle.
 *
 * \return DLR input buffer 
 *
 * \ingroup group_applib_semseg_cnn
 */
float ** SEMSEG_CNN_APPLIB_getDLRInputBuff(
        SEMSEG_CNN_APPLIB_Handle handle);

/**
 * \brief Function to get the DLR output buffer
 *
 * \param [in] handle APPLIB handle.
 *
 * \return DLR output buffer 
 *
 * \ingroup group_applib_semseg_cnn
 */

int32_t ** SEMSEG_CNN_APPLIB_getDLROutputBuff(
        SEMSEG_CNN_APPLIB_Handle handle);


#endif /* _SEMSEG_CNN_APPLIB_H_ */

