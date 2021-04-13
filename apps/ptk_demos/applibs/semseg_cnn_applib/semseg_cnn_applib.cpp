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

#include <string.h>
#include <thread>

#include <TI/tivx.h>

#include <TI/tivx_target_kernel.h>

#include <perception/utils/ptk_semaphore.h>
#include <app_ptk_demo_common.h>
#include <app_ptk_demo_profile.h>

#include <cm_ldc_node_cntxt.h>
#include <cm_scaler_node_cntxt.h>

#include <semseg_cnn_applib_priv.h>
#include <semseg_cnn_applib.h>

#define SEMSEG_CNN_APPLIB_NUM_GRAPH_PARAMS  (3U)
#define SEMSEG_CNN_MAX_OUT_TENSOR_DIMS      (3U)

#define semseg_clip3(x, min, max) ((x) > (max)?(max):((x) < (min)?(min):(x)))
#define semseg_normalize(x, mean, scale) (((x) - (mean))*(scale))

struct SEMSEG_CNN_APPLIB_graphParams
{
    /* Graph parameter 0 */
    vx_image                vxInputImage;

    /* Graph parameter 1 */
    vx_image                vxRectifiedImage;

    /* Graph parameter 1/2 */
    vx_image                vxScalerOut;

    /* Output Tensor after post-processing. */
    vx_tensor               vxOutTensor;

    /* Input to DLR node. */
    float                  *dlrInputBuff;

    /* Output from the DLR node. */
    int32_t                *dlrOutputBuff;

    /* timestamp - Not a Graph param */
    vx_uint64              *timestamp;
};

using SEMSEG_CNN_APPLIB_graphParamQ =
      std::queue<SEMSEG_CNN_APPLIB_graphParams*>;

struct SEMSEG_CNN_APPLIB_Queue
{
    /** Resource lock. Used get/put from/to freeQ. */
    std::mutex                      m_mutex;

    /** A queue for holding free descriptors. */
    SEMSEG_CNN_APPLIB_graphParamQ   m_q;

    SEMSEG_CNN_APPLIB_graphParams *peek()
    {
        std::unique_lock<std::mutex>    lock(m_mutex);
        SEMSEG_CNN_APPLIB_graphParams  *desc = nullptr;

        /* Check if we have descriptors available. */
        if (!m_q.empty())
        {
            desc = m_q.front();
        }

        return desc;
    }

    SEMSEG_CNN_APPLIB_graphParams *pop()
    {
        std::unique_lock<std::mutex>    lock(m_mutex);
        SEMSEG_CNN_APPLIB_graphParams  *desc = nullptr;

        /* Check if we have descriptors available. */
        if (!m_q.empty())
        {
            desc = m_q.front();
            m_q.pop();
        }

        return desc;
    }

    void push(SEMSEG_CNN_APPLIB_graphParams  *desc)
    {
        std::unique_lock<std::mutex>   lock(m_mutex);

        m_q.push(desc);
    }

    int32_t size()
    {
        return m_q.size();
    }
};

struct SEMSEG_CNN_APPLIB_Context
{
    /** openVX context handle. */
    vx_context                      vxContext{};

    /** Graph handle. */
    vx_graph                        vxGraph{};

    /** LDC node context object. */
    CM_LdcNodeCntxt                 ldcObj{};

    /** Scaler node context object. */
    CM_ScalerNodeCntxt              scalerObj{};

    /** DLR node context object. */
    CM_DLRNodeCntxt                *dlrObj;

    /** Input images handles. */
    vx_image                        vxInputImage[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Input images handles. */
    vx_image                        vxRectifiedImage[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /* Output Tensor after post-processing. */
    vx_tensor                       vxOutTensor[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /* Input timestamp */
    vx_uint64                       timestamp[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /* DLR input buffers. */
    float                          *dlrInputBuff[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /* DLR output buffer. */
    int32_t                        *dlrOutputBuff[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Flag to indicate if the APPLIB should create and manage the graph. */
    bool                            manageGraph{false};

    /** Create parameters. */
    SEMSEG_CNN_APPLIB_createParams  createParams;

    /** graph parameter tracking */
    SEMSEG_CNN_APPLIB_graphParams   paramDesc[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** DLR task queue. */
    SEMSEG_CNN_APPLIB_Queue         freeQ;

    /** Pre-process task queue. */
    SEMSEG_CNN_APPLIB_Queue         preProcQ;

    /** DLR task queue. */
    SEMSEG_CNN_APPLIB_Queue         dlrQ;

    /** Post-process task queue. */
    SEMSEG_CNN_APPLIB_Queue         postProcQ;

    /** Queue for output processing. */
    SEMSEG_CNN_APPLIB_Queue         outputQ;

    /** The number of pipelined params */
    uint8_t                         numPipeParams;

    /** Performance monitoring. */
    app_perf_point_t                semsegPerf;

    /** Flag to track if the performance counter has been initialized. */
    bool                            startPerfCapt;

    /** Flag to control DLR process exit. */
    bool                            exitPreprocThread;

    /** Flag to control DLR process exit. */
    bool                            exitDlrThread;

    /** Flag to control DLR process exit. */
    bool                            exitPostprocThread;

    /** pre-processing thread. */
    std::thread                     preProcThread;

    /** DLR processing thread. */
    std::thread                     dlrThread;

    /** post-processing thread. */
    std::thread                     postProcThread;

    /** Semaphore for controlling the pre-processing thread. */
    UTILS::Semaphore               *preProcSem;

    /** Semaphore for controlling the DLR thread. */
    UTILS::Semaphore               *dlrDataReadySem;

    /** Semaphore for controlling the post-processing thread. */
    UTILS::Semaphore               *postProcSem;

    /** For OpenVX graph profiling: graph start time */
    chrono::time_point<chrono::system_clock> start;

    /** For OpenVX graph profiling: graph end time */
    chrono::time_point<chrono::system_clock> end;
};

typedef struct
{
    uint8_t y;
    uint8_t u;
    uint8_t v;

} SemSegColor;

static SemSegColor yuvColMap[] =
{
    //       Y    U    V         R    G    B
    [ 0] = { 94, 147, 152}, // {128,  64, 128}
    [ 1] = {119, 183, 206}, // {244,  35, 232}
    [ 2] = { 76, 128, 128}, // { 70,  70,  70}
    [ 3] = {109, 152, 124}, // {102, 102, 156}
    [ 4] = {157, 123, 144}, // {190, 153, 153}
    [ 5] = {147, 128, 128}, // {153, 153, 153}
    [ 6] = {169,  55, 173}, // {250, 170,  30}
    [ 7] = {184,  32, 143}, // {220, 220,   0}
    [ 8] = {119,  86, 120}, // {107, 142,  35}
    [ 9] = {197,  99,  92}, // {152, 251, 152}
    [10] = {117, 159,  98}, // { 70, 130, 180}
    [11] = { 89, 116, 213}, // {220,  20,  60}
    [12] = { 82,  90, 240}, // {255,   0,   0}
    [13] = { 30, 190, 118}, // {  0,   0, 142}
    [14] = { 23, 159, 123}, // {  0,   0,  70}
    [15] = { 56, 154,  99}, // {  0,  60, 100}
    [16] = { 66, 149,  92}, // {  0,  80, 100}
    [17] = { 38, 229, 112}, // {  0,   0, 230}
    [18] = { 55, 121, 174}, // {119,  11,  32}
    [19] = {126, 128, 128}  // {128, 128, 128}
}; // yuvColMap

static vx_status SEMSEG_CNN_APPLIB_setParams(
        SEMSEG_CNN_APPLIB_Handle        handle,
        SEMSEG_CNN_APPLIB_createParams *createParams)
{
    SEMSEG_CNN_APPLIB_Context  *appCntxt;
    vx_status                   vxStatus;

    vxStatus = VX_SUCCESS;
    appCntxt = (SEMSEG_CNN_APPLIB_Context *)handle;

    if (createParams->vxGraph == NULL &&
        createParams->pipelineDepth > PTK_GRAPH_MAX_PIPELINE_DEPTH)
    {
        PTK_printf("[%s:%d] Invalid pipelineDepth. Max allowed value is [%d]\n",
                    __FUNCTION__, __LINE__,
                    PTK_GRAPH_MAX_PIPELINE_DEPTH);

        vxStatus = VX_FAILURE;
    }

    appCntxt->vxContext = createParams->vxContext;
    appCntxt->dlrObj    = createParams->dlrObj;

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        memcpy(&appCntxt->createParams,
               createParams,
               sizeof(SEMSEG_CNN_APPLIB_createParams));

        /* Check whether we need to create and manage the graph. */
        if (createParams->vxGraph != NULL)
        {
            /* A graph handle has been provided. This indicates that the
             * Application has created the graph and will manage it. We just
             * need to create the nodes and attach it to the graph.
             */
            appCntxt->manageGraph = false;
            appCntxt->vxGraph     = createParams->vxGraph;

            if (createParams->createInputFlag == 0)
            {
                appCntxt->vxInputImage[0] = createParams->vxInputImage[0];

                if (appCntxt->vxInputImage[0] == NULL)
                {
                    PTK_printf("[%s:%d] vxInputImage NULL.\n",
                                __FUNCTION__, __LINE__);

                    vxStatus = VX_FAILURE;
                }
            }
        }
        else
        {
            /* No graph handle has been provided. We need to create and manage
             * the graph ourselves.
             */
            appCntxt->manageGraph = true;
        }
    }

    return vxStatus;
}

static vx_status SEMSEG_CNN_APPLIB_popFreeInputDesc(
        SEMSEG_CNN_APPLIB_Context       *appCntxt,
        SEMSEG_CNN_APPLIB_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->freeQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}

static vx_status SEMSEG_CNN_APPLIB_popPreprocInputDesc(
        SEMSEG_CNN_APPLIB_Context       *appCntxt,
        SEMSEG_CNN_APPLIB_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->preProcQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}

static vx_status SEMSEG_CNN_APPLIB_popDLRInputDesc(
        SEMSEG_CNN_APPLIB_Context       *appCntxt,
        SEMSEG_CNN_APPLIB_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->dlrQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}

static vx_status SEMSEG_CNN_APPLIB_popPostprocInputDesc(
        SEMSEG_CNN_APPLIB_Context       *appCntxt,
        SEMSEG_CNN_APPLIB_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->postProcQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}

static vx_status SEMSEG_CNN_APPLIB_getOutputDesc(
        SEMSEG_CNN_APPLIB_Context       *appCntxt,
        SEMSEG_CNN_APPLIB_graphParams   *gpDesc)
{
    SEMSEG_CNN_APPLIB_graphParams  *desc;
    vx_status                       vxStatus = VX_SUCCESS;

    desc = appCntxt->outputQ.peek();

    if (desc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }
    else
    {
        *gpDesc = *desc;
    }


    return vxStatus;
}

static vx_status SEMSEG_CNN_APPLIB_popOutputDesc(
        SEMSEG_CNN_APPLIB_Context       *appCntxt,
        SEMSEG_CNN_APPLIB_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->outputQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}

static void SEMSEG_CNN_APPLIB_enqueInputDesc(
        SEMSEG_CNN_APPLIB_Context      *appCntxt,
        SEMSEG_CNN_APPLIB_graphParams  *desc)
{
    appCntxt->freeQ.push(desc);
}

static void SEMSEG_CNN_APPLIB_enquePreprocInputDesc(
        SEMSEG_CNN_APPLIB_Context      *appCntxt,
        SEMSEG_CNN_APPLIB_graphParams  *desc)
{
    appCntxt->preProcQ.push(desc);
}

static void SEMSEG_CNN_APPLIB_enqueDLRInputDesc(
        SEMSEG_CNN_APPLIB_Context      *appCntxt,
        SEMSEG_CNN_APPLIB_graphParams  *desc)
{
    appCntxt->dlrQ.push(desc);
}

static void SEMSEG_CNN_APPLIB_enquePostprocInputDesc(
        SEMSEG_CNN_APPLIB_Context      *appCntxt,
        SEMSEG_CNN_APPLIB_graphParams  *desc)
{
    appCntxt->postProcQ.push(desc);
}

static void SEMSEG_CNN_APPLIB_enqueOutputDesc(
        SEMSEG_CNN_APPLIB_Context      *appCntxt,
        SEMSEG_CNN_APPLIB_graphParams  *desc)
{
    appCntxt->outputQ.push(desc);
}

static int32_t SEMSEG_CNN_APPLIB_convertYUV2RGB(float          *rgbImage,
                                                const uint8_t  *yuvImage[2],
                                                const float     mean[3],
                                                const float     scale[3],
                                                int32_t         width,
                                                int32_t         height)
{
    const uint8_t  *srcPtrY;
    const uint8_t  *srcPtrUV;
    float          *dstPtrR;
    float          *dstPtrG;
    float          *dstPtrB;
    float           meanR;
    float           meanG;
    float           meanB;
    float           scaleR;
    float           scaleG;
    float           scaleB;
    float           r;
    float           g;
    float           b;
    int32_t         cb;
    int32_t         cr;
    int32_t         i;
    int32_t         j;
    int32_t         y;

    srcPtrY  = yuvImage[0];
    srcPtrUV = yuvImage[1];

    dstPtrR  = rgbImage;
    dstPtrG  = dstPtrR + (width * height);
    dstPtrB  = dstPtrG + (width * height);

    meanR    = mean[0];
    meanG    = mean[1];
    meanB    = mean[2];
    scaleR   = scale[0];
    scaleG   = scale[1];
    scaleB   = scale[2];

    for (j = 0; j < height; j++)
    {
        for (i = 0; i < width; i++)
        {
            y  = srcPtrY[j * width + i];
            cb = srcPtrUV[(j >> 1)*width + (i>>1)*2];
            cr = srcPtrUV[(j >> 1)*width + (i>>1)*2 + 1];

            y  = y  - 16;
            cb = cb - 128;
            cr = cr - 128;

            r = (float)semseg_clip3((298*y + 409*cr + 128) >> 8, 0, 255);
            g = (float)semseg_clip3((298*y - 100*cb - 208*cr + 128) >> 8, 0, 255);
            b = (float)semseg_clip3((298*y + 516*cb + 128) >> 8, 0, 255);

            *dstPtrR++ = semseg_normalize(r, meanR, scaleR);
            *dstPtrG++ = semseg_normalize(g, meanG, scaleG);
            *dstPtrB++ = semseg_normalize(b, meanB, scaleB);
        }
    }

    return VX_SUCCESS;
}


vx_status SEMSEG_CNN_APPLIB_createOutTensor(vx_tensor                       vxOutTensor,
                                            int32_t                        *dlrOutputBuff,
                                            SEMSEG_CNN_APPLIB_createParams *createParams)
{
    uint8_t    *dPtr;
    vx_size     start[SEMSEG_CNN_MAX_OUT_TENSOR_DIMS];
    vx_size     strides[SEMSEG_CNN_MAX_OUT_TENSOR_DIMS];
    vx_size     tensorDims[SEMSEG_CNN_MAX_OUT_TENSOR_DIMS];
    vx_map_id   mapId;
    vx_status   vxStatus;

    start[0] = 0;
    start[1] = 0;
    start[2] = 0;

    /* DIM 0 - Width. */
    tensorDims[0] = createParams->tidlImageWidth;

    /* DIM 1 - Height. */
    tensorDims[1] = createParams->tidlImageHeight;

    /* DIM 2 - Number of channels. */
    tensorDims[2] = 1;

    /* Strides */
    strides[0] = 1;
    strides[1] = tensorDims[0];
    strides[2] = strides[1]*tensorDims[1];

    vxStatus = tivxMapTensorPatch(vxOutTensor,
                                  SEMSEG_CNN_MAX_OUT_TENSOR_DIMS,
                                  start,
                                  tensorDims,
                                  &mapId,
                                  strides,
                                  (void **)&dPtr,
                                  VX_READ_AND_WRITE,
                                  VX_MEMORY_TYPE_HOST);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        PTK_printf("[%s:%d] tivxMapTensorPatch() failed.",
                   __FUNCTION__, __LINE__);
    }
    else
    {
        int32_t    *sPtr;
        uint32_t    size;

        sPtr = dlrOutputBuff;
        size = tensorDims[0] * tensorDims[1];

        for (uint32_t i = 0; i < size; i++)
        {
            *dPtr++ = (uint8_t)*sPtr++;
        }

        vxStatus = tivxUnmapTensorPatch(vxOutTensor, mapId);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] tivxUnmapTensorPatch() failed.",
                       __FUNCTION__, __LINE__);     
        }
    }

    return vxStatus;
}

vx_status SEMSEG_CNN_APPLIB_preProcess(vx_image                         vxScalerOut,
                                       float                           *dlrInputBuff,
                                       SEMSEG_CNN_APPLIB_createParams  *createParams)
{
    vx_rectangle_t              rect;
    vx_imagepatch_addressing_t  imgAddr;
    uint8_t                    *srcPtr[2];
    vx_map_id                   mapId[2];
    bool                        mapped[2] = {false, false};
    vx_status                   vxStatus;

    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x   = createParams->tidlImageWidth;
    rect.end_y   = createParams->tidlImageHeight;

    /* Map the image planes. */
    for (uint32_t i = 0; i < 2; i++)
    {
        /* Get the pointer to the YUV data plans. */
        vxStatus = vxMapImagePatch(vxScalerOut,
                                   &rect,
                                   i,
                                   &mapId[i],
                                   &imgAddr,
                                   (void **)&srcPtr[i],
                                   VX_WRITE_ONLY,
                                   VX_MEMORY_TYPE_HOST,
                                   VX_NOGAP_X);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] vxMapImagePatch() failed.",
                       __FUNCTION__, __LINE__);
        }
        else
        {
            mapped[i] = true;
        }

        /* UV plane has half the height. */
        rect.end_y /= 2;
    }

    /* Take thr output image from the scaler which is in NV12 format and do
     * the following:
     * - Color convert the image from YUV420 to BGR format
     * - Convert to float and write to the DLR input buffer
     */

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus =
            SEMSEG_CNN_APPLIB_convertYUV2RGB(dlrInputBuff,
                                             (const uint8_t**)srcPtr,
                                             createParams->preProcMean,
                                             createParams->preProcScale,
                                             createParams->tidlImageWidth,
                                             createParams->tidlImageHeight);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] SEMSEG_CNN_APPLIB_convertYUV2RGB() failed.",
                       __FUNCTION__, __LINE__);
        }
    }

    /* Unmap the image planes. */
    for (uint32_t i = 0; i < 2; i++)
    {
        if (mapped[i] == true)
        {
            vxUnmapImagePatch(vxScalerOut, mapId[i]);
        }
    }

    return vxStatus;
}


vx_status SEMSEG_CNN_APPLIB_postProcess(vx_image                        vxScalerOut,
                                        int32_t                        *dlrOutputBuff,
                                        SEMSEG_CNN_APPLIB_createParams *createParams)
{
    vx_rectangle_t              rect;
    vx_imagepatch_addressing_t  imgAddr;
    vx_map_id                   mapId;
    uint8_t                    *dPtr;
    vx_status                   vxStatus;

    /* Get the pointer to the UV plane in the output image. */
    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x   = createParams->tidlImageWidth;
    rect.end_y   = createParams->tidlImageHeight/2; // Chroma is UV interleaved

    /* Get the pointer to the UV data plans. */
    vxStatus = vxMapImagePatch(vxScalerOut,
                               &rect,
                               1, // UV plane
                               &mapId,
                               &imgAddr,
                               (void **)&dPtr,
                               VX_WRITE_ONLY,
                               VX_MEMORY_TYPE_HOST,
                               VX_NOGAP_X);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        PTK_printf("[%s:%d] vxMapImagePatch() failed.",
                   __FUNCTION__, __LINE__);     
    }
    else
    {
        int32_t    *classId;
        uint8_t    *p;
        float       alpha;
        float       alphaC;
        int32_t     widthInByte;

        alpha  = 0.5;
        alphaC = 1.0f - alpha;

        classId     = dlrOutputBuff;
        widthInByte = imgAddr.dim_x/2;

        /* Blend the input image data with the DLR output tensor and generate
         * the output image.
         */
        for (uint32_t i = 0; i < imgAddr.dim_y; i++)
        {
            p = dPtr;

            for (int32_t j = 0; j < widthInByte; j++)
            {
                uint8_t c = (uint8_t)*classId;

                p[0] = (uint8_t)((p[0] * alpha) + yuvColMap[c].u * alphaC);
                p[1] = (uint8_t)((p[1] * alpha) + yuvColMap[c].v * alphaC);

                /* Move to the next pain of UV pixels. */
                p += 2;

                /* We use one class Id for a pair of UV pixels, so skip one
                 * class value.
                 */
                classId += 2;
            }

            /* Move to the next line in the image. */
            dPtr += imgAddr.stride_y;

            /* Skip the next line in the tensor data. */
            classId += createParams->tidlImageWidth;
        }

        vxUnmapImagePatch(vxScalerOut, mapId);
    }

    return vxStatus;
}

static vx_status SEMSEG_CNN_APPLIB_setupPipeline(
        SEMSEG_CNN_APPLIB_Context * appCntxt)
{
    SEMSEG_CNN_APPLIB_createParams     *createParams;
    CM_LdcNodeCntxt                    *ldcObj;
    CM_ScalerNodeCntxt                 *scalerObj;
    vx_graph_parameter_queue_params_t   q[SEMSEG_CNN_APPLIB_NUM_GRAPH_PARAMS];
    uint32_t                            cnt;
    uint32_t                            i;
    vx_status                           vxStatus;

    createParams = &appCntxt->createParams;
    ldcObj       = &appCntxt->ldcObj;
    scalerObj    = &appCntxt->scalerObj;
    cnt          = 0;

    if (createParams->enableLdcNode)
    {
        /* LDC node Param 6 ==> graph param 0. */
        vxStatus = ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                               ldcObj->vxNode,
                                               6);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ptkdemo_addParamByNodeIndex() failed\n",
                        __FUNCTION__, __LINE__);
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxInputImage;

            /* LDC node Param 7 ==> graph param 1. */
            vxStatus = ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                                   ldcObj->vxNode,
                                                   7);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] ptkdemo_addParamByNodeIndex() failed\n",
                            __FUNCTION__, __LINE__);
            }
            else
            {
                q[cnt++].refs_list = (vx_reference*)appCntxt->vxRectifiedImage;
            }
        }
    }
    else
    {
        /* Scaler node Param 0 ==> graph param 0. */
        vxStatus = ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                               scalerObj->vxNode,
                                               0);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ptkdemo_addParamByNodeIndex() failed\n",
                        __FUNCTION__, __LINE__);
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxInputImage;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Scaler node Param 1 ==> graph param 1/2. */
        vxStatus = ptkdemo_addParamByNodeIndex(appCntxt->vxGraph,
                                               scalerObj->vxNode,
                                               1);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ptkdemo_addParamByNodeIndex() failed\n",
                        __FUNCTION__, __LINE__);
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)scalerObj->outImage;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        for (i = 0; i < cnt; i++)
        {
            q[i].graph_parameter_index = i;
            q[i].refs_list_size        = createParams->pipelineDepth;
        }

        vxStatus = vxSetGraphScheduleConfig(appCntxt->vxGraph,
                                            VX_GRAPH_SCHEDULE_MODE_QUEUE_AUTO,
                                            cnt,
                                            q);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] vxSetGraphScheduleConfig() failed\n",
                       __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Explicitly set graph pipeline depth */
        vxStatus = tivxSetGraphPipelineDepth(appCntxt->vxGraph,
                                             createParams->pipelineDepth);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] tivxSetGraphPipelineDepth() failed\n",
                       __FUNCTION__, __LINE__);
        }
    }

    /* Setup the parameter descriptors. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        for (i = 0; i < createParams->pipelineDepth; i++)
        {
            SEMSEG_CNN_APPLIB_graphParams      *paramDesc;

            paramDesc                = &appCntxt->paramDesc[i];
            paramDesc->vxInputImage  = appCntxt->vxInputImage[i];
            paramDesc->vxOutTensor   = appCntxt->vxOutTensor[i];
            paramDesc->timestamp     = &appCntxt->timestamp[i];
            paramDesc->dlrInputBuff  = appCntxt->dlrInputBuff[i];
            paramDesc->dlrOutputBuff = appCntxt->dlrOutputBuff[i];
            paramDesc->vxScalerOut   = scalerObj->outImage[i];

            if (createParams->enableLdcNode)
            {
                paramDesc->vxRectifiedImage = appCntxt->vxRectifiedImage[i];
            }

            SEMSEG_CNN_APPLIB_enqueInputDesc(appCntxt, paramDesc);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        uint32_t    appValue;

        appValue = createParams->vxEvtAppValBase +
                   SEMSEG_CNN_APPLIB_SCALER_NODE_COMP_EVT;

        vxStatus = vxRegisterEvent((vx_reference)scalerObj->vxNode,
                                   VX_EVENT_NODE_COMPLETED,
                                   0,
                                   appValue);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] vxRegisterEvent() failed\n",
                       __FUNCTION__, __LINE__);
        }
    }

    return vxStatus;
}

static vx_status SEMSEG_CNN_APPLIB_createGraph(
        SEMSEG_CNN_APPLIB_Context  *appCntxt)
{
    SEMSEG_CNN_APPLIB_createParams *createParams;
    CM_LdcNodeCntxt                *ldcObj;
    CM_ScalerNodeCntxt             *scalerObj;
    CM_DLRNodeCntxt                *dlrObj;
    vx_image                        scalerInput;
    vx_status                       vxStatus;
    int32_t                         i;

    vxStatus      = VX_SUCCESS;
    createParams  = &appCntxt->createParams;
    scalerObj     = &appCntxt->scalerObj;
    ldcObj        = &appCntxt->ldcObj;
    dlrObj        = appCntxt->dlrObj;

    if (appCntxt->manageGraph == true)
    {
        /* Create OpenVx Graph to run TIDL Network */
        appCntxt->vxGraph = vxCreateGraph(appCntxt->vxContext);

        if (appCntxt->vxGraph == NULL)
        {
            PTK_printf("[%s:%d] vxCreateGraph() failed\n",
                        __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxGraph,
                               "SDE CNN Post Processing Graph");
        }

        /************************/
        /*    Input data        */
        /************************/
        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            const char *str;
            int32_t     imageType;

            /* Input image */
            appCntxt->numPipeParams = 1;

            if (createParams->enableLdcNode)
            {
                imageType = VX_DF_IMAGE_UYVY;
                str = "InputImage_UYVY";
            }
            else
            {
                imageType = VX_DF_IMAGE_NV12;
                str = "InputImage_NV12";
            }

            for (i = 0; i < appCntxt->createParams.pipelineDepth; i++)
            {
                appCntxt->vxInputImage[i] =
                    vxCreateImage(appCntxt->vxContext,
                                  createParams->inputImageWidth,
                                  createParams->inputImageHeight,
                                  imageType);

                if (appCntxt->vxInputImage[i] == NULL)
                {
                    PTK_printf("[%s:%d] vxCreateImage() failed\n",
                                __FUNCTION__, __LINE__);

                    vxStatus = VX_FAILURE;
                    break;
                }

                vxSetReferenceName((vx_reference)appCntxt->vxInputImage[i], str);
            }
        }

    } // if (appCntxt->manageGraph == true)

    /* Create rectified image objects when LDC is enabled */
    if ((vxStatus == (vx_status)VX_SUCCESS) && createParams->enableLdcNode)
    {
        appCntxt->numPipeParams++;

        for (i = 0; i < appCntxt->createParams.pipelineDepth; i++)
        {
            appCntxt->vxRectifiedImage[i] =
                vxCreateImage(appCntxt->vxContext,
                              createParams->inputImageWidth,
                              createParams->inputImageHeight,
                              VX_DF_IMAGE_NV12);

            if (appCntxt->vxRectifiedImage[i] == NULL)
            {
                PTK_printf("[%s:%d] vxCreateImage() failed\n",
                            __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
                break;
            }

            vxSetReferenceName((vx_reference)appCntxt->vxRectifiedImage[i],
                               "RectifiedImage_NV12");
        }
    }


    if ((vxStatus == (vx_status)VX_SUCCESS) &&
        !createParams->enablePostProcNode)
    {
        vx_size tensorDims[SEMSEG_CNN_MAX_OUT_TENSOR_DIMS];

        /* DIM 0 - Width. */
        tensorDims[0] = createParams->tidlImageWidth;

        /* DIM 1 - Height. */
        tensorDims[1] = createParams->tidlImageHeight;

        /* DIM 2 - Number of channels. */
        tensorDims[2] = 1;

        for (i = 0; i < appCntxt->createParams.pipelineDepth; i++)
        {
            appCntxt->vxOutTensor[i] =
                vxCreateTensor(appCntxt->vxContext,
                               SEMSEG_CNN_MAX_OUT_TENSOR_DIMS,
                               tensorDims,
                               VX_TYPE_UINT8,
                               0);

            if (appCntxt->vxOutTensor[i] == NULL)
            {
                PTK_printf("[%s:%d] vxCreateTensor() failed\n",
                            __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
                break;
            }
        }
    }

    /* Setup the DLR node. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        int32_t numDims;

#if 0
        CM_DLRCreateParams  params;
        int32_t             status;

        params.modelPath = createParams->dlrModelPath;
        params.devType   = DLR_DEVTYPE_CPU;
        params.devId     = 0;

        status = CM_dlrNodeCntxtInit(dlrObj, &params);

        if (status < 0)
        {
            PTK_printf("[%s:%d] CM_dlrNodeCntxtInit() failed.\n",
                        __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
#endif
        numDims = dlrObj->input.info[0].dim - 1;
        if (dlrObj->input.numInfo != CM_PREPROC_NUM_OUTPUT_TENSORS)
        {
            PTK_printf("[%s:%d] Number of DLR inputs [%d] does not match the "
                       "number of tensors [%d] output by the pre-processing "
                       "node.\n",
                        __FUNCTION__,
                        __LINE__,
                        dlrObj->input.numInfo,
                        CM_PREPROC_NUM_OUTPUT_TENSORS);

            vxStatus = VX_FAILURE;
        }
        else if (numDims != CM_PREPROC_OUTPUT_TENSORS_DIMS)
        {
            PTK_printf("[%s:%d] Number of DLR input dims [%d] does not match the "
                       "number of tensor dims [%d] output by the pre-processing "
                       "node.\n",
                        __FUNCTION__,
                        __LINE__,
                        numDims,
                        CM_PREPROC_OUTPUT_TENSORS_DIMS);

            vxStatus = VX_FAILURE;
        }
        else
        {
            CM_DLRIfInfo   *info;

            /* Allocate DLR input buffers. Currenly, the input buffers are held
             * for each pipelined outputs so we need pipeline number of input
             * buffers.
             */
            info = &dlrObj->input.info[0];
            for (i = 0; i < createParams->pipelineDepth; i++)
            {
                /* Allocate the input buffer. */
                appCntxt->dlrInputBuff[i] = (float *)
                    tivxMemAlloc(info->size * sizeof(float), TIVX_MEM_EXTERNAL);

                if (appCntxt->dlrInputBuff[i] == NULL)
                {
                    PTK_printf("[%s:%d] Failed to allocate input (%d) "
                               "of size (%d) bytes.\n",
                                __FUNCTION__, __LINE__, i, info->size);
                    vxStatus = VX_FAILURE;
                    break;
                }
            }

            /* Allocate DLR input buffer. Curently, the output buffer is not
             * held across frames so just one istance is sufficient.
             */
            if (vxStatus == (vx_status)VX_SUCCESS)
            {
                /* Allocate the output buffer. */
                info = &dlrObj->output.info[0];
                for (i = 0; i < createParams->pipelineDepth; i++)
                {
                    /* Allocate the input buffer. */
                    appCntxt->dlrOutputBuff[i] = (int32_t *)
                        tivxMemAlloc(info->size * sizeof(int32_t), TIVX_MEM_EXTERNAL);

                    if (appCntxt->dlrOutputBuff[i] == NULL)
                    {
                        PTK_printf("[%s:%d] Failed to allocate output (%d) "
                                   "of size (%d) bytes.\n",
                                    __FUNCTION__, __LINE__, i, info->size);
                        vxStatus = VX_FAILURE;
                        break;
                    }
                }
            }
        }
    }

    /* Input image will be the scaler default input. */
    scalerInput = appCntxt->vxInputImage[0];

    /************************/
    /*     LDC node         */
    /************************/
    if ((createParams->enableLdcNode) && (vxStatus == (vx_status)VX_SUCCESS))
    {
        CM_LdcCreateParams  params;

        params.width       = createParams->inputImageWidth;
        params.height      = createParams->inputImageHeight;
        params.ssFactor    = createParams->ldcSsFactor;
        params.blockWidth  = createParams->ldcBlockWidth;
        params.blockHeight = createParams->ldcBlockHeight;
        params.pixelPad    = createParams->ldcPixelPad;
        params.lutFilePath = createParams->ldcLutFilePath;

        vxStatus = CM_ldcNodeCntxtInit(&appCntxt->ldcObj,
                                       appCntxt->vxContext,
                                       &params);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] CM_ldcNodeCntxtInit() failed\n",
                        __FUNCTION__, __LINE__);
        }
        else
        {
            vxStatus = CM_ldcNodeCntxtSetup(ldcObj,
                                            appCntxt->vxContext,
                                            appCntxt->vxGraph,
                                            appCntxt->vxInputImage[0],
                                            appCntxt->vxRectifiedImage[0]);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] CM_ldcNodeCntxtSetup() failed\n",
                            __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }
            else
            {
                /* LDC output will be the scaler input. */
                scalerInput = appCntxt->vxRectifiedImage[0];
            }
        }
    }

    /************************/
    /*    Scaler node       */
    /************************/
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        CM_ScalerCreateParams   params;

        params.width         = createParams->tidlImageWidth;
        params.height        = createParams->tidlImageHeight;
        params.imageType     = VX_DF_IMAGE_NV12;
        params.interpolation = VX_INTERPOLATION_BILINEAR;
        params.pipelineDepth = createParams->pipelineDepth;

        vxStatus = CM_scalerNodeCntxtInit(&appCntxt->scalerObj,
                                          appCntxt->vxContext,
                                          &params);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] CM_scalerNodeCntxtInit() failed\n",
                        __FUNCTION__, __LINE__);
        }
        else
        {
            vxStatus = CM_scalerNodeCntxtSetup(scalerObj,
                                               appCntxt->vxContext,
                                               appCntxt->vxGraph,
                                               scalerInput);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] CM_scalerNodeCntxtSetup() failed\n",
                            __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }
        }

        appCntxt->numPipeParams++;
    }

    if (appCntxt->manageGraph == true)
    {
        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            /* set up the pipeline. */
            vxStatus = SEMSEG_CNN_APPLIB_setupPipeline(appCntxt);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] SEMSEG_CNN_APPLIB_setupPipeline() failed\n",
                            __FUNCTION__, __LINE__);
            }
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            /* Verify the TIDL Graph */
            vxStatus = vxVerifyGraph(appCntxt->vxGraph);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] vxVerifyGraph() failed\n",
                            __FUNCTION__, __LINE__);
            }
        }

        /* Set the MSC coefficients. */
        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            vxStatus = CM_scalerNodeCntxtSetCoeff(scalerObj);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] CM_scalerNodeCntxtSetCoeff() failed\n",
                            __FUNCTION__, __LINE__);
            }
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            appPerfPointSetName(&appCntxt->semsegPerf,
                                "Semantic Segmentation GRAPH");

            if (createParams->exportGraph == 1)
            {
                tivxExportGraphToDot(appCntxt->vxGraph, ".",
                                     "vx_applib_semseg_cnn");
            }

            if (createParams->rtLogEnable == 1)
            {
                tivxLogRtTraceEnable(appCntxt->vxGraph);
            }
        }

    } // if (appCntxt->manageGraph == true)

    return vxStatus;

}

static void SEMSEG_CNN_APPLIB_releaseGraph(
        SEMSEG_CNN_APPLIB_Context  *appCntxt)
{
    SEMSEG_CNN_APPLIB_createParams *createParams;
    CM_DLRNodeCntxt                *dlrObj;
    CM_DLRIfInfo                   *info;
    int32_t                         i;

    createParams  = &appCntxt->createParams;
    dlrObj        = appCntxt->dlrObj;

    if (createParams->enableLdcNode)
    {
        for (i = 0; i < createParams->pipelineDepth; i++)
        {
            if (appCntxt->vxRectifiedImage[i] != NULL)
            {
                vxReleaseImage(&appCntxt->vxRectifiedImage[i]);
            }
        }

        CM_ldcNodeCntxtDeInit(&appCntxt->ldcObj);
    }

    /* Scaler node.  */
    CM_scalerNodeCntxtDeInit(&appCntxt->scalerObj);

    if (appCntxt->manageGraph == true)
    {
        for (i = 0; i < createParams->pipelineDepth; i++)
        {
            if (appCntxt->vxInputImage[i] != NULL)
            {
                vxReleaseImage(&appCntxt->vxInputImage[i]);
            }
        }

        if (appCntxt->vxGraph != NULL)
        {
            vxReleaseGraph(&appCntxt->vxGraph);
        }
    }

    if (!createParams->enablePostProcNode)
    {
        for (i = 0; i < createParams->pipelineDepth; i++)
        {
            if (appCntxt->vxOutTensor[i] != NULL)
            {
                vxReleaseTensor(&appCntxt->vxOutTensor[i]);
            }
        }
    }

    /* De-allocate the input buffer. */
    info = &dlrObj->input.info[0];
    for (i = 0; i < createParams->pipelineDepth; i++)
    {
        if (appCntxt->dlrInputBuff[i] != NULL)
        {
            tivxMemFree(appCntxt->dlrInputBuff[i],
                        info->size * sizeof(float),
                        TIVX_MEM_EXTERNAL);
        }
    }

    /* De-allocate the DLR output buffer. */
    info = &dlrObj->output.info[0];
    for (i = 0; i < createParams->pipelineDepth; i++)
    {
        if (appCntxt->dlrOutputBuff[i] != NULL)
        {
            tivxMemFree(appCntxt->dlrOutputBuff[i],
                        info->size * sizeof(int32_t),
                        TIVX_MEM_EXTERNAL);
        }
    }

    appCntxt->vxGraph = NULL;

}

static void SEMSEG_CNN_APPLIB_preProcThread(SEMSEG_CNN_APPLIB_Context  *appCntxt)
{
    SEMSEG_CNN_APPLIB_createParams *createParams;
 
    /* For profiling */
    chrono::time_point<chrono::system_clock> start, end;
    float diff;

    createParams = &appCntxt->createParams;

    PTK_printf("[%s] Launched.\n", __FUNCTION__);

    while (true)
    {
        SEMSEG_CNN_APPLIB_graphParams  *desc;
        vx_status                       vxStatus;

        /* Wait for the input buffer availability. */
        appCntxt->preProcSem->wait();

        /*  Check if we need to exit the thread. */
        if (appCntxt->exitPreprocThread == true)
        {
            break;
        }

        vxStatus = SEMSEG_CNN_APPLIB_popPreprocInputDesc(appCntxt, &desc);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            start = GET_TIME();
            vxStatus =
                SEMSEG_CNN_APPLIB_preProcess(desc->vxScalerOut,
                                             desc->dlrInputBuff,
                                             createParams);
            end   = GET_TIME();
            diff  = GET_DIFF(start, end);
            ptkdemo_report_proctime("preprocessing", diff);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] SEMSEG_CNN_APPLIB_preProcess() "
                           "failed.\n",
                            __FUNCTION__, __LINE__);
            }
            else
            {
                /* Push the descriptor to the DLR queue. */
                SEMSEG_CNN_APPLIB_enqueDLRInputDesc(appCntxt, desc);

                /* Wakeup the DLR thread. The DLR thread will process the
                 * descriptor at the head of the queue.
                 */
                appCntxt->dlrDataReadySem->notify();
            }

        } // if (vxStatus == (vx_status)VX_SUCCESS)

    } // while (true)

    PTK_printf("[%s] Exiting.\n", __FUNCTION__);
}

static void SEMSEG_CNN_APPLIB_dlrThread(SEMSEG_CNN_APPLIB_Context  *appCntxt)
{
    CM_DLRNodeCntxt        *dlrObj;
    CM_DLRNodeInputInfo    *dlrInput;
    CM_DLRNodeOutputInfo   *dlrOutput;

    /* For profiling */
    chrono::time_point<chrono::system_clock> start, end;
    float diff;

    dlrObj       = appCntxt->dlrObj;
    dlrInput     = &dlrObj->input;
    dlrOutput    = &dlrObj->output;

    PTK_printf("[%s] Launched.\n", __FUNCTION__);

    while (true)
    {
        SEMSEG_CNN_APPLIB_graphParams  *desc;
        vx_status                       vxStatus;

        /* Wait for the input buffer availability. */
        appCntxt->dlrDataReadySem->wait();

        /*  Check if we need to exit the thread. */
        if (appCntxt->exitDlrThread == true)
        {
            break;
        }

        vxStatus = SEMSEG_CNN_APPLIB_popDLRInputDesc(appCntxt, &desc);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            int32_t status;

            dlrInput->info[0].data  = desc->dlrInputBuff;
            dlrOutput->info[0].data = desc->dlrOutputBuff;

            start = GET_TIME();
            status = CM_dlrNodeCntxtProcess(dlrObj, dlrInput, dlrOutput);
            end   = GET_TIME();
            diff  = GET_DIFF(start, end);
            ptkdemo_report_proctime("dlr-node", diff);

            if (status < 0)
            {
                PTK_printf("[%s:%d] CM_dlrNodeCntxtProcess() failed.\n",
                            __FUNCTION__, __LINE__);
            }
            else
            {
                /* Push the descriptor to the DLR queue. */
                SEMSEG_CNN_APPLIB_enquePostprocInputDesc(appCntxt, desc);

                /* Wakeup the post-process thread. The post-process thread will
                 * process the descriptor at the head of the queue.
                 */
                appCntxt->postProcSem->notify();
            }

        } // if (vxStatus == (vx_status)VX_SUCCESS)

    } // while (true)

    PTK_printf("[%s] Exiting.\n", __FUNCTION__);
}

static void SEMSEG_CNN_APPLIB_postProcThread(SEMSEG_CNN_APPLIB_Context  *appCntxt)
{
    SEMSEG_CNN_APPLIB_createParams *createParams;

    /* For profiling */
    chrono::time_point<chrono::system_clock> start, end;
    float diff;

    createParams = &appCntxt->createParams;

    PTK_printf("[%s] Launched.\n", __FUNCTION__);

    while (true)
    {
        SEMSEG_CNN_APPLIB_graphParams  *desc;
        vx_status                       vxStatus;

        /* Wait for the input buffer availability. */
        appCntxt->postProcSem->wait();

        /*  Check if we need to exit the thread. */
        if (appCntxt->exitPostprocThread == true)
        {
            break;
        }

        vxStatus = SEMSEG_CNN_APPLIB_popPostprocInputDesc(appCntxt, &desc);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            if (createParams->enablePostProcNode)
            {
                /* Create the output object for display. */
                start = GET_TIME();
                vxStatus =
                    SEMSEG_CNN_APPLIB_postProcess(desc->vxScalerOut,
                                                  desc->dlrOutputBuff,
                                                  createParams);
                end   = GET_TIME();
                diff  = GET_DIFF(start, end);
                ptkdemo_report_proctime("postprocessing", diff);

                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    PTK_printf("[%s:%d] SEMSEG_CNN_APPLIB_postProcess() "
                               "failed.\n",
                                __FUNCTION__, __LINE__);
                }
            }
            else
            {
                /* Create an output tensor. */
                start = GET_TIME();
                vxStatus =
                    SEMSEG_CNN_APPLIB_createOutTensor(desc->vxOutTensor,
                                                      desc->dlrOutputBuff,
                                                      createParams);
                end   = GET_TIME();
                diff  = GET_DIFF(start, end);
                ptkdemo_report_proctime("output_tensor_creation", diff);

                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    PTK_printf("[%s:%d] SEMSEG_CNN_APPLIB_createOutTensor() "
                               "failed.\n",
                                __FUNCTION__, __LINE__);
                }
            }

            if (vxStatus == (vx_status)VX_SUCCESS)
            {
                /* Push the descriptor to the output queue. */
                SEMSEG_CNN_APPLIB_enqueOutputDesc(appCntxt, desc);
            }

            if (vxStatus == (vx_status)VX_SUCCESS)
            {
                /* Let the usr know that output is ready. */
                vxStatus =
                    vxSendUserEvent(appCntxt->vxContext,
                                    SEMSEG_CNN_APPLIB_OUT_AVAIL_EVT,
                                    NULL);

                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    PTK_printf("[%s:%d] vxSendUserEvent() failed.\n",
                                __FUNCTION__, __LINE__);
                }
            }

        } // if (vxStatus == (vx_status)VX_SUCCESS)

    } // while (true)

    PTK_printf("[%s] Exiting.\n", __FUNCTION__);
}

vx_status SEMSEG_CNN_APPLIB_process(
        SEMSEG_CNN_APPLIB_Handle    handle,
        vx_image                    inputImg,
        uint64_t                    timestamp)
{
    SEMSEG_CNN_APPLIB_Context      *appCntxt;
    SEMSEG_CNN_APPLIB_graphParams  *gpDesc;
    vx_status                       vxStatus;

    /* For profiling */
    chrono::time_point<chrono::system_clock> start, end;
    float diff;

    appCntxt = (SEMSEG_CNN_APPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt->manageGraph == false)
    {
        PTK_printf("[%s:%d] Graph not managed by APPLIB.\n",
                    __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = SEMSEG_CNN_APPLIB_popFreeInputDesc(appCntxt, &gpDesc);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] SEMSEG_CNN_APPLIB_popFreeInputDesc() failed\n",
                        __FUNCTION__, __LINE__);
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            start = GET_TIME();
            vxStatus = ptkdemo_copy_image_to_image(inputImg,
                                                   gpDesc->vxInputImage);
            end   = GET_TIME();
            diff  = GET_DIFF(start, end);
            ptkdemo_report_proctime("input_image_copy", diff);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                /* Release the descriptor back to the free descriptor queue. */
                SEMSEG_CNN_APPLIB_enqueInputDesc(appCntxt, gpDesc);

                PTK_printf("[%s:%d] ptkdemo_copy_image_to_image() failed\n",
                            __FUNCTION__, __LINE__);
            }
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            vx_reference    obj[3];
            uint32_t        i;
            uint32_t        sidx = 0;

            // set timestamp of input image
            *gpDesc->timestamp = timestamp;

            /* Graph param 0. */
            obj[sidx++] = (vx_reference)gpDesc->vxInputImage;

            if (appCntxt->createParams.enableLdcNode)
            {
                /* Graph param 1. */
                obj[sidx++] = (vx_reference)gpDesc->vxRectifiedImage;
            }

            /* Graph param 1/2. */
            obj[sidx++] = (vx_reference)gpDesc->vxScalerOut;

            for (i = 0; i < appCntxt->numPipeParams; i++)
            {
                vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxGraph,
                                                           i,
                                                           &obj[i],
                                                           1);
                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    /* This is fatal. */
                    PTK_printf("[%s:%d] vxGraphParameterEnqueueReadyRef(%d) "
                               "failed\n", __FUNCTION__, __LINE__, i);
                    break;
                }
            }

            /* Push the descriptor to the pre-processing input queue. */
            SEMSEG_CNN_APPLIB_enquePreprocInputDesc(appCntxt, gpDesc);
        }
    }

    return vxStatus;
}

SEMSEG_CNN_APPLIB_Handle SEMSEG_CNN_APPLIB_create(
        SEMSEG_CNN_APPLIB_createParams *createParams)
{
    SEMSEG_CNN_APPLIB_Context  *appCntxt;
    vx_status                   vxStatus;

    appCntxt = new SEMSEG_CNN_APPLIB_Context();
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Could not allocate memory for internal context.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Set applib-level create parameters */
        vxStatus = SEMSEG_CNN_APPLIB_setParams(appCntxt, createParams);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] SEMSEG_CNN_APPLIB_setParams() failed.\n",
                        __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Create nodes and graph */
        vxStatus = SEMSEG_CNN_APPLIB_createGraph(appCntxt);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] SEMSEG_CNN_APPLIB_createGraph() failed.\n",
                        __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS && appCntxt->manageGraph == true) 
    {
        /* Setup and launch pre-processing thread. */
        appCntxt->preProcSem          = new UTILS::Semaphore(0);
        appCntxt->exitPreprocThread   = false;
        appCntxt->preProcThread =
            std::thread(SEMSEG_CNN_APPLIB_preProcThread, appCntxt);

        /* Setup and launch DLR processing thread. */
        appCntxt->dlrDataReadySem = new UTILS::Semaphore(0);
        appCntxt->exitDlrThread   = false;
        appCntxt->dlrThread =
            std::thread(SEMSEG_CNN_APPLIB_dlrThread, appCntxt);

        /* Setup and launch post-processing thread. */
        appCntxt->postProcSem        = new UTILS::Semaphore(0);
        appCntxt->exitPostprocThread = false;
        appCntxt->postProcThread =
            std::thread(SEMSEG_CNN_APPLIB_postProcThread, appCntxt);
    }

    if (vxStatus != VX_SUCCESS)
    {
        SEMSEG_CNN_APPLIB_delete(&appCntxt);
    }

    return appCntxt;
}

vx_status SEMSEG_CNN_APPLIB_reset(
        SEMSEG_CNN_APPLIB_Handle   handle)
{
    SEMSEG_CNN_APPLIB_Context *appCntxt;

    appCntxt = (SEMSEG_CNN_APPLIB_Context *)handle;

    /* Reset the performance capture initialization flag. */
    appCntxt->startPerfCapt = false;

    return VX_SUCCESS;
}

vx_status SEMSEG_CNN_APPLIB_getOutBuff(
        SEMSEG_CNN_APPLIB_Handle    handle,
        vx_image                   *inputImage,
        vx_reference               *output,
        vx_uint64                  *timestamp)
{
    SEMSEG_CNN_APPLIB_Context  *appCntxt;
    vx_status                   vxStatus;

    appCntxt = (SEMSEG_CNN_APPLIB_Context*)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt->manageGraph == false)
    {
        PTK_printf("[%s:%d] Graph not managed by APPLIB.\n",
                    __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        SEMSEG_CNN_APPLIB_graphParams   desc;

        vxStatus = SEMSEG_CNN_APPLIB_getOutputDesc(appCntxt, &desc);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            if (appCntxt->createParams.enableLdcNode)
            {
                *inputImage = desc.vxRectifiedImage;
            }
            else
            {
                *inputImage = desc.vxInputImage;
            }


            if (appCntxt->createParams.enablePostProcNode)
            {
                *output = (vx_reference)desc.vxScalerOut;
            }
            else
            {
                *output = (vx_reference)desc.vxOutTensor;
            }

            *timestamp = *desc.timestamp;
        }
        else
        {
            PTK_printf("[%s:%d] SEMSEG_CNN_APPLIB_getOutputDesc() failed\n",
                        __FUNCTION__, __LINE__);
        }
    }

    return vxStatus;
}

vx_status SEMSEG_CNN_APPLIB_getOutBuff(
        SEMSEG_CNN_APPLIB_Handle    handle,
        vx_image                    inputImage,
        vx_image                    outputImage)
{
    vx_reference    outRef;
    vx_image        inImage;
    vx_status       vxStatus;

    vxStatus = SEMSEG_CNN_APPLIB_getOutBuff(handle, &inImage, &outRef, nullptr);

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = ptkdemo_copy_image_to_image(inImage, inputImage);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ptkdemo_copy_image_to_image() failed\n",
                        __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vx_image    image = (vx_image)outRef;

        vxStatus = ptkdemo_copy_image_to_image(image, outputImage);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] ptkdemo_copy_image_to_image() failed\n",
                        __FUNCTION__, __LINE__);
        }
    }

    return vxStatus;
}

vx_status SEMSEG_CNN_APPLIB_releaseOutBuff(
        SEMSEG_CNN_APPLIB_Handle    handle)
{
    SEMSEG_CNN_APPLIB_Context  *appCntxt;
    vx_status                   vxStatus;

    appCntxt = (SEMSEG_CNN_APPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt->manageGraph == false)
    {
        PTK_printf("[%s:%d] Graph not managed by APPLIB.\n",
                    __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        SEMSEG_CNN_APPLIB_graphParams  *desc;

        /* Pop the output descriptor. */
        vxStatus = SEMSEG_CNN_APPLIB_popOutputDesc(appCntxt, &desc);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            /* Enque the descriptor to the free descriptor queue. */
            SEMSEG_CNN_APPLIB_enqueInputDesc(appCntxt, desc);
        }
    }

    return vxStatus;
}

vx_status SEMSEG_CNN_APPLIB_processEvent(
        SEMSEG_CNN_APPLIB_Handle    handle,
        vx_event_t                 *event)
{
    SEMSEG_CNN_APPLIB_Context  *appCntxt;
    vx_status                   vxStatus;

    appCntxt = (SEMSEG_CNN_APPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt->manageGraph == false)
    {
        PTK_printf("[%s:%d] Graph not managed by APPLIB.\n",
                    __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if ((vxStatus == (vx_status)VX_SUCCESS) &&
        (event->type == VX_EVENT_NODE_COMPLETED))
    {
        vx_reference    ref;
        uint32_t        numRefs;
        uint32_t        appValue;
        int32_t         i;
        float           diff;

        appValue = appCntxt->createParams.vxEvtAppValBase +
                   SEMSEG_CNN_APPLIB_SCALER_NODE_COMP_EVT;

        if (event->app_value != appValue)
        {
            /* Something wrong. We did not register for this event. */
            PTK_printf("[%s:%d] Unknown App Value [%d].\n",
                       __FUNCTION__, __LINE__, event->app_value);

            vxStatus = VX_FAILURE;
        }

        // Only when graph is managed by Applib
        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            if (appCntxt->startPerfCapt == false)
            {
                appCntxt->startPerfCapt = true;
            }
            else
            {
                appPerfPointEnd(&appCntxt->semsegPerf);
                appCntxt->end = GET_TIME();
                diff  = GET_DIFF(appCntxt->start, appCntxt->end);
                ptkdemo_report_proctime("graph_processing", diff);
            }

            appPerfPointBegin(&appCntxt->semsegPerf);
            appCntxt->start = GET_TIME();

            /* Node execution is complete. Deque all the parameters
             * for this node.
             */
            for (i = 0; i < appCntxt->numPipeParams; i++)
            {
                vxStatus = vxGraphParameterDequeueDoneRef(appCntxt->vxGraph,
                                                          i,
                                                          &ref,
                                                          1,
                                                          &numRefs);
                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    PTK_printf("[%s:%d] vxGraphParameterDequeueDoneRef() "
                               "failed\n", __FUNCTION__, __LINE__);

                    break;
                }
            }
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            /* Wakeup the pre-process thread. The pre-process thread will
             * process the descriptor at the head of the queue.
             */
            appCntxt->preProcSem->notify();
        }
    }

    return vxStatus;
}

vx_status SEMSEG_CNN_APPLIB_waitGraph(
        SEMSEG_CNN_APPLIB_Handle   handle)
{
    SEMSEG_CNN_APPLIB_Context  *appCntxt;
    vx_status                   vxStatus;

    appCntxt = (SEMSEG_CNN_APPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt->manageGraph == true)
    {
        vxStatus = vxWaitGraph(appCntxt->vxGraph);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] vxWaitGraph() failed\n",
                        __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            /* Wait for the output queue to get flushed. */
            while (appCntxt->freeQ.size() !=
                   appCntxt->createParams.pipelineDepth)
            {
                std::this_thread::sleep_for (std::chrono::milliseconds(50));
            }
        }
    }

    return vxStatus;
}

vx_status SEMSEG_CNN_APPLIB_printStats(
        SEMSEG_CNN_APPLIB_Handle    handle)
{
    SEMSEG_CNN_APPLIB_Context  *appCntxt;
    vx_status                   vxStatus;

    appCntxt = (SEMSEG_CNN_APPLIB_Context *)handle;

    vxStatus = tivx_utils_graph_perf_print(appCntxt->vxGraph);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        PTK_printf("[%s:%d] tivx_utils_graph_perf_print() failed\n",
                    __FUNCTION__, __LINE__);
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appPerfPointPrint(&appCntxt->semsegPerf);
        appPerfPointPrintFPS(&appCntxt->semsegPerf);
    }

    return vxStatus;
}

vx_status SEMSEG_CNN_APPLIB_exportStats(
        SEMSEG_CNN_APPLIB_Handle    handle,
        FILE                       *fp,
        bool                        exportAll)
{
    SEMSEG_CNN_APPLIB_Context  *appCntxt;
    vx_status                   vxStatus;

    appCntxt = (SEMSEG_CNN_APPLIB_Context *)handle;

    vxStatus = tivx_utils_graph_perf_export(fp, appCntxt->vxGraph);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        PTK_printf("[%s:%d] tivx_utils_graph_perf_export() failed\n",
                    __FUNCTION__, __LINE__);
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        if (exportAll == true)
        {
            app_perf_point_t *perfArr[1] = {&appCntxt->semsegPerf};
            appPerfStatsExportAll(fp, perfArr, 1);
        }
    }

    return vxStatus;
}

vx_status SEMSEG_CNN_APPLIB_delete(
        SEMSEG_CNN_APPLIB_Handle   *handle)
{
    vx_status   vxStatus = VX_SUCCESS;

    if ((handle != NULL) && (*handle != NULL))
    {
        SEMSEG_CNN_APPLIB_Context  *appCntxt;
        //int32_t                     dlrStatus;

        appCntxt = (SEMSEG_CNN_APPLIB_Context *)*handle;

        SEMSEG_CNN_APPLIB_releaseGraph(appCntxt);

#if 0
        /* Delete the DLR Model handle. */
        dlrStatus = CM_dlrNodeCntxtDeInit(appCntxt->dlrObj);

        if (dlrStatus < 0)
        {
            PTK_printf("[%s:%d] CM_dlrNodeCntxtDeInit() failed.\n",
                        __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
#endif

        if (appCntxt->manageGraph)
        {
            /* Set the exit flag for the pre-process thread. */
            appCntxt->exitPreprocThread = true;

            /* Set the exit flag for the DLR thread. */
            appCntxt->exitDlrThread = true;

            /* Set the exit flag for the post-process thread. */
            appCntxt->exitPostprocThread = true;

            /* Wake-up the pre-process thread. */
            if (appCntxt->preProcSem)
            {
                appCntxt->preProcSem->notify();
            }

            /* Wake-up the DLR thread. */
            if (appCntxt->dlrDataReadySem)
            {
                appCntxt->dlrDataReadySem->notify();
            }

            /* Wake-up the post-process thread. */
            if (appCntxt->postProcSem)
            {
                appCntxt->postProcSem->notify();
            }

            /* Wait for the thread exit. */
            if (appCntxt->preProcThread.joinable())
            {
                appCntxt->preProcThread.join();
            }

            if (appCntxt->dlrThread.joinable())
            {
                appCntxt->dlrThread.join();
            }

            if (appCntxt->postProcThread.joinable())
            {
                appCntxt->postProcThread.join();
            }

            /* Delete the semaphores. */
            if (appCntxt->preProcSem)
            {
                delete appCntxt->preProcSem;
            }

            if (appCntxt->dlrDataReadySem)
            {
                delete appCntxt->dlrDataReadySem;
            }

            if (appCntxt->postProcSem)
            {
                delete appCntxt->postProcSem;
            }
        }

        delete *handle;
        *handle = NULL;
    }

    return vxStatus;
}

CM_ScalerNodeCntxt * SEMSEG_CNN_APPLIB_getScalerNodeCntxt(
        SEMSEG_CNN_APPLIB_Handle handle)
{
    SEMSEG_CNN_APPLIB_Context *appCntxt = (SEMSEG_CNN_APPLIB_Context *)handle;
    return &appCntxt->scalerObj;
}

vx_tensor * SEMSEG_CNN_APPLIB_getOutputTensor(
        SEMSEG_CNN_APPLIB_Handle handle)
{
    SEMSEG_CNN_APPLIB_Context *appCntxt = (SEMSEG_CNN_APPLIB_Context *)handle;
    return appCntxt->vxOutTensor;
}

CM_DLRNodeInputInfo * SEMSEG_CNN_APPLIB_getDLRNodeInputInfo(
        SEMSEG_CNN_APPLIB_Handle handle)
{
    SEMSEG_CNN_APPLIB_Context *appCntxt = (SEMSEG_CNN_APPLIB_Context *)handle;
    return &appCntxt->dlrObj->input;
}

CM_DLRNodeOutputInfo * SEMSEG_CNN_APPLIB_getDLRNodeOutputInfo(
        SEMSEG_CNN_APPLIB_Handle handle)
{
    SEMSEG_CNN_APPLIB_Context *appCntxt = (SEMSEG_CNN_APPLIB_Context *)handle;
    return &appCntxt->dlrObj->output;
}

float ** SEMSEG_CNN_APPLIB_getDLRInputBuff(
        SEMSEG_CNN_APPLIB_Handle handle)
{
    SEMSEG_CNN_APPLIB_Context *appCntxt = (SEMSEG_CNN_APPLIB_Context *)handle;
    return appCntxt->dlrInputBuff;
}

int32_t ** SEMSEG_CNN_APPLIB_getDLROutputBuff(
        SEMSEG_CNN_APPLIB_Handle handle)
{
    SEMSEG_CNN_APPLIB_Context *appCntxt = (SEMSEG_CNN_APPLIB_Context *)handle;
    return appCntxt->dlrOutputBuff;
}


