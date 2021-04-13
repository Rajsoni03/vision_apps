/*
*
* Copyright (c) 2019 Texas Instruments Incorporated
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

#include <utils/draw2d/include/draw2d.h>

void vx_img_mosaic_draw_overlay_avp2(void *addr[], uint32_t pitch[], uint32_t width, uint32_t height)
{
    Draw2D_Handle  handle;
    Draw2D_BufInfo draw2dBufInfo;

    Draw2D_FontPrm sHeading;
    Draw2D_FontPrm sLabel;
    Draw2D_BmpPrm  bmpPrm;

    int32_t status;

    draw2dBufInfo.bufAddr[0]  = addr[0];
    draw2dBufInfo.bufAddr[1]  = addr[1];
    draw2dBufInfo.bufWidth    = width;
    draw2dBufInfo.bufHeight   = height;
    draw2dBufInfo.bufPitch[0] = pitch[0];
    draw2dBufInfo.bufPitch[1] = pitch[1];
    draw2dBufInfo.dataFormat  = DRAW2D_DF_YUV420SP_UV;
    draw2dBufInfo.transperentColor = 0x00000000;
    draw2dBufInfo.transperentColorFormat = DRAW2D_DF_BGR16_565;

    status = Draw2D_create(&handle);

    if(status==0)
    {
        if (addr[0] != NULL && addr[1] != NULL)
        {
            Draw2D_setBufInfo(handle, &draw2dBufInfo);

            bmpPrm.bmpIdx = DRAW2D_BMP_IDX_TI_LOGO_2;

            Draw2D_drawBmp(handle, 20, 0, &bmpPrm);

            sHeading.fontIdx = 10;

            Draw2D_drawString(handle, 560, 5, "Analytics for Auto Valet Parking", &sHeading);

            sLabel.fontIdx = 12;
            Draw2D_drawString(handle, 340, 90, "Left  camera", &sLabel);
            Draw2D_drawString(handle, 920, 90, "Front camera", &sLabel);
            Draw2D_drawString(handle, 1520,90, "Right camera", &sLabel);

            sLabel.fontIdx = 13;
            Draw2D_drawString(handle, 20, 244, "Semantic", &sLabel);
            Draw2D_drawString(handle, 0 , 264, "Segmentation", &sLabel);
            Draw2D_drawString(handle, 0 , 284, "  (768x384) ", &sLabel);

            Draw2D_drawString(handle, 20, 526, "Parking", &sLabel);
            Draw2D_drawString(handle, 35, 546, "Spot", &sLabel);
            Draw2D_drawString(handle, 10, 566, "Detection", &sLabel);
            Draw2D_drawString(handle, 10, 586, "(768x384)", &sLabel);

            Draw2D_drawString(handle, 25, 828, "Vehicle", &sLabel);
            Draw2D_drawString(handle, 15, 848, "Detection", &sLabel);
            Draw2D_drawString(handle, 15, 868, "(768x384)", &sLabel);

            sLabel.fontIdx = 12;
            Draw2D_drawString(handle, 340, 1000, "Camera resolution : 1280 x 720 (1 MP) , Analytics resolution : 768 x 384 (0.3 MP)", &sLabel);

            sLabel.fontIdx = 12;
            Draw2D_drawString(handle, 596, 1040, "Analytics performance : 3 CH, 3 algorithm, 15 FPS", &sLabel);
        }

        Draw2D_delete(handle);
    }

    return;
}


