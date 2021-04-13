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
#include <app_ptk_demo_display.h>

extern "C"
{
#include <utils/mem/include/app_mem.h>
}

/* Creating display node */
static vx_status PTKDEMO_createDisplayGraph(PTKDEMO_DisplayContext *dispCntxt)
{
    tivx_display_params_t       dispParams;
    vx_uint32                   posX;
    vx_uint32                   posY;
    vx_uint32                   pipeId;
    vx_uint32                   width;
    vx_uint32                   height;
    vx_rectangle_t              rect;
    vx_map_id                   map_id;
    vx_status                   status;

    posX   = 0;
    posY   = 0;
    pipeId = 2;

    dispCntxt->vxGraph = vxCreateGraph(dispCntxt->vxContext);

    if (dispCntxt->vxGraph == NULL)
    {
        PTK_printf("[%s:%d] vxCreateGraph() failed.\n",
                   __FUNCTION__,
                   __LINE__);

        return VX_FAILURE;
    }

    vxSetReferenceName((vx_reference)dispCntxt->vxGraph, "Display");

    /* Display initialization */
    memset(&dispParams, 0, sizeof(tivx_display_params_t));

    dispParams.opMode    = TIVX_KERNEL_DISPLAY_ZERO_BUFFER_COPY_MODE;
    dispParams.pipeId    = pipeId;
    dispParams.outWidth  = dispCntxt->vxOutWidth;
    dispParams.outHeight = dispCntxt->vxOutHeight;
    dispParams.posX      = posX;
    dispParams.posY      = posY;

    dispCntxt->vxDispParam =
        vxCreateUserDataObject(dispCntxt->vxContext,
                               "tivx_display_params_t",
                               sizeof(tivx_display_params_t),
                               &dispParams);

    if (vxGetStatus((vx_reference)dispCntxt->vxDispParam) != VX_SUCCESS)
    {
        PTK_printf("[%s:%d] vxCreateUserDataObject() failed\n",
                   __FUNCTION__,
                   __LINE__);

        return VX_FAILURE;
    }

    /* Creating objects for OpenGL node */
    dispCntxt->vxOutImage = vxCreateImage(dispCntxt->vxContext,
                                          dispCntxt->vxOutWidth,
                                          dispCntxt->vxOutHeight,
                                          VX_DF_IMAGE_RGBX);

    if (dispCntxt->vxOutImage == NULL)
    {
        PTK_printf("[%s:%d] vxCreateImage() failed\n",
                   __FUNCTION__,
                   __LINE__);

        return VX_FAILURE;
    }

    dispCntxt->vxDispNode =
        tivxDisplayNode(dispCntxt->vxGraph,
                        dispCntxt->vxDispParam,
                        dispCntxt->vxOutImage);

    if (dispCntxt->vxDispNode == NULL)
    {
        PTK_printf("[%s:%d] tivxDisplayNode() failed\n",
                   __FUNCTION__,
                   __LINE__);

        return VX_FAILURE;
    }

    vxSetNodeTarget(dispCntxt->vxDispNode,
                    VX_TARGET_STRING,
                    TIVX_TARGET_DISPLAY1);

    vxSetReferenceName((vx_reference)dispCntxt->vxDispNode, "Display_node");

    /* Create User Data object for channel switching */
    dispCntxt->chanParams.active_channel_id = 0;
    dispCntxt->vxSwitchCh =
        vxCreateUserDataObject(dispCntxt->vxContext,
                               "tivx_display_select_channel_params_t",
                               sizeof(tivx_display_select_channel_params_t),
                               &dispCntxt->chanParams);

    if (dispCntxt->vxSwitchCh == NULL)
    {
        PTK_printf("[%s:%d] vxCreateUserDataObject() failed\n",
                   __FUNCTION__,
                   __LINE__);

        return VX_FAILURE;
    }

    dispCntxt->refs[0] = (vx_reference)dispCntxt->vxSwitchCh;

    /* Get file descriptor for vxCreateImage */
    vxQueryImage(dispCntxt->vxOutImage,
                 VX_IMAGE_WIDTH,
                 &width,
                 sizeof(vx_uint32));

    vxQueryImage(dispCntxt->vxOutImage,
                 VX_IMAGE_HEIGHT,
                 &height,
                 sizeof(vx_uint32));

    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x = width;
    rect.end_y = height;

    vxMapImagePatch(dispCntxt->vxOutImage,
                    &rect,
                    0,
                    &map_id,
                    &(dispCntxt->image_addr),
                    &dispCntxt->data_ptr,
                    VX_READ_AND_WRITE,
                    VX_MEMORY_TYPE_HOST,
                    VX_NOGAP_X);

    dispCntxt->dispDmaBuffId =
        appMemGetDmaBufFd(dispCntxt->data_ptr, &dispCntxt->dispDmaBuffFdOffset);

    status = vxVerifyGraph(dispCntxt->vxGraph);
    
    if (status != VX_SUCCESS)
    {
        PTK_printf("[%s:%d] Graph create failed.\n",
                   __FUNCTION__,
                   __LINE__);

        return status;
    }

    dispCntxt->displayReadySem->notify();

    return VX_SUCCESS;
}

static void PTKDEMO_releaseDisplayGraph(PTKDEMO_DisplayContext *dispCntxt)
{
    /* Release the node. */
    vxReleaseNode(&dispCntxt->vxDispNode);

    /* Release the parameter memory. */
    vxReleaseImage(&dispCntxt->vxOutImage);
    vxReleaseUserDataObject(&dispCntxt->vxDispParam);
    vxReleaseUserDataObject(&dispCntxt->vxSwitchCh);

    /* Release the graph. */
    vxReleaseGraph(&dispCntxt->vxGraph);

    return;
}

static vx_status PTKDEMO_displayThread(PTKDEMO_DisplayContext *dispCntxt)
{
    vx_status status;

    if (vx_true_e != tivxIsTargetEnabled(TIVX_TARGET_DISPLAY1))
    {
        PTK_printf("[%s:%d] Appropriate cores not enabled\n",
                   __FUNCTION__,
                   __LINE__);

        return VX_FAILURE;
    }

    status = PTKDEMO_createDisplayGraph(dispCntxt);

    if (status != VX_SUCCESS)
    {
        PTK_printf("[%s:%d] PTKDEMO_createDisplayGraph() failed.\n",
                   __FUNCTION__,
                   __LINE__);

        return status;
    }

    while (dispCntxt->exitFlag == false)
    {
        std::chrono::microseconds   t;

        vxProcessGraph(dispCntxt->vxGraph);

        t = std::chrono::microseconds(dispCntxt->dispPeriod);

        /* Delay for dispPeriod. */
        std::this_thread::sleep_for(t);
    }

    PTKDEMO_releaseDisplayGraph(dispCntxt);

    return VX_SUCCESS;
}

int32_t PTKDEMO_launchDisplayThread(PTKDEMO_DisplayContext *dispCntxt)
{
    dispCntxt->exitFlag = false;

    /* Launch the display graph processing thread. */
    dispCntxt->displayThread = std::thread(PTKDEMO_displayThread, dispCntxt);

    return 0;
}

void PTKDEMO_exitDisplayThread(PTKDEMO_DisplayContext *dispCntxt)
{
    dispCntxt->exitFlag = true;

    if (dispCntxt->displayThread.joinable())
    {
        dispCntxt->displayThread.join();
    }

    return;
}

