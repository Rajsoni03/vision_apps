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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include "lidar_ogmap_renderer.h"

typedef struct
{
    uint32_t    flag;
    float       r;
    float       g;
    float       b;

} LIDAR_GridEntry;

static LIDAR_GridEntry gGridEntryTbl[] =
{
  /* Ground. */
  { OG_FLAG_GROUND,
    0.0, 0.6, 0.0},

  { OG_FLAG_FST |
    OG_FLAG_GROUND,
    0.0, 0.6, 0.0 },

  /** Detected as PFSD. */
  { OG_FLAG_FSD |
    OG_FLAG_FST |
    OG_FLAG_PFSD,
    0.0, 0.4, 0.9 },

  { OG_FLAG_FSD |
    OG_FLAG_GROUND |
    OG_FLAG_FST |
    OG_FLAG_PFSD,
    0.0, 0.4, 0.9 },

  /** Detected FSD but not PFSD. */
  { OG_FLAG_FSD |
    OG_FLAG_FST,
    0.0, 0.7, 0.7 },

  { OG_FLAG_FSD |
    OG_FLAG_GROUND |
    OG_FLAG_FST,
    0.0, 0.7, 0.7 },

  /** Occupied flag is set then always occupied color.
  *   FSD or PFSD ignored if occupied set. */
  { OG_FLAG_OCCUPIED,
    0.6, 0.0, 0.0 },

  { OG_FLAG_OCCUPIED |
    OG_FLAG_FST,
    0.6, 0.0, 0.0 },

  { OG_FLAG_OCCUPIED |
    OG_FLAG_FSD ,
    0.6, 0.0, 0.0 },

  { OG_FLAG_OCCUPIED |
    OG_FLAG_FSD |
    OG_FLAG_FST,
    0.6, 0.0, 0.0 },

  { OG_FLAG_OCCUPIED |
    OG_FLAG_FST |
    OG_FLAG_PFSD,
    0.6, 0.0, 0.0 },

  { OG_FLAG_OCCUPIED |
    OG_FLAG_FSD |
    OG_FLAG_PFSD,
    0.6, 0.0, 0.0 },

  { OG_FLAG_EGO,
    1.0, 1.0, 0.0 },

  { OG_FLAG_EGO |
    OG_FLAG_GROUND,
    1.0, 1.0, 0.0 },

  { OG_FLAG_EGO |
    OG_FLAG_OCCUPIED,
    1.0, 1.0, 0.0 },

  { OG_FLAG_EGO |
    OG_FLAG_FSD |
    OG_FLAG_FST,
    1.0, 1.0, 0.0 },

  { OG_FLAG_EGO |
    OG_FLAG_FSD |
    OG_FLAG_FST |
    OG_FLAG_GROUND,
    1.0, 1.0, 0.0 },

  { OG_FLAG_EGO |
    OG_FLAG_FSD |
    OG_FLAG_FST |
    OG_FLAG_OCCUPIED,
    1.0, 1.0, 0.0 },

  { OG_FLAG_EGO |
    OG_FLAG_FSD |
    OG_FLAG_FST |
    OG_FLAG_OCCUPIED |
    OG_FLAG_PFSD,
    1.0, 1.0, 0.0 },

  { OG_FLAG_OCCUPIED |
    OG_FLAG_FSD |
    OG_FLAG_FST |
    OG_FLAG_PFSD,
    0.6, 0.0, 0.0 },

}; /* gGridEntryTbl[] */

static void LIDAR_renderInit(LIDAR_AppContext *appCntxt)
{
    if (appCntxt->visualize)
    {
        LIDAROGAPPLIB_CreateParams     *createParams;
        PTK_Alg_LidarOgmapParams       *ogConfig;
        Renderer                       *renderer;
        Renderable                     *r;
        PTK_VirtualCamera              *virtCamera;
        Dimensions                      dims;
        uint32_t                        numEntries;
        uint32_t                        i;

        createParams = &appCntxt->createParams;
        ogConfig     = &createParams->ogPfsdCfg.ogConfig;

        appCntxt->renderer = new Renderer();
        renderer      = appCntxt->renderer;

#ifdef PLATFORM_EGL
        appCntxt->dispCntxt.displayReadySem->wait();
        renderer->setEglFdandPitch(appCntxt->dispCntxt.dispDmaBuffId,
                       appCntxt->dispCntxt.dispDmaBuffFdOffset,
                       appCntxt->dispCntxt.image_addr.stride_y);
#endif

        renderer->createConsole();

        renderer->createWindow("TI OpenVX LIDAR Demo",
                               appCntxt->winWidth,
                               appCntxt->winHeight);

        appCntxt->mapRenderer = new MapRenderable(renderer);

        dims.x = 0.00f;
        dims.y = 0.00f;
        dims.w = 0.99f;
        dims.h = 0.99f;

        r = appCntxt->mapRenderer;
        renderer->add(r, dims);

        appCntxt->keyHandlerTbl = new KeyHandlerTable();
        renderer->setKeyHandlerTable(appCntxt->keyHandlerTbl);

        virtCamera = r->getVirtualCamera();
        addCameraControls(*appCntxt->keyHandlerTbl, appCntxt->renderer, virtCamera);

        numEntries = sizeof(gGridEntryTbl)/sizeof(LIDAR_GridEntry);

        appCntxt->gridFlagView = appCntxt->mapRenderer->createGridFlagView();

        for (i = 0; i < numEntries; i++ )
        {
            LIDAR_GridEntry *e = &gGridEntryTbl[i];

            appCntxt->gridFlagView->setFlag(e->flag);
            appCntxt->gridFlagView->setColor(glm::vec4(e->r, e->g, e->b, 1.0f));

            appCntxt->mapRenderer->add(appCntxt->gridFlagView,
                                       ogConfig->accGridId);
        }
    }
}

static void LIDAR_renderDeInit(LIDAR_AppContext *appCntxt)
{
    if (appCntxt->mapRenderer != NULL)
    {
        delete appCntxt->mapRenderer;
    }

    if (appCntxt->renderer != NULL)
    {
        delete appCntxt->renderer;
    }

    return;

} /* LIDAR_renderDeInit */

static void LIDAR_dumpOutData(LIDAR_AppContext  *appCntxt,
                              const PTK_Map     *map)
{
    char        charBuffer[DBCONFIG_MAX_PATH_LEN] = {0};
    char       *dstFilePath = charBuffer;

    if (!appCntxt->outputStrmHdl)
    {
        return;
    }

    PTK_printf("Saving Map for lidar frame # %d\n", appCntxt->framesProcessed);

    VirtualSensorCreator_add_record(appCntxt->outputStrmHdl,
                                    (char **)&dstFilePath,
                                    appCntxt->curSensorTs,
                                    (uint8_t *)map,
                                    appCntxt->outMapSize);

    return;

} /* LIDAR_dumpOutData */

int32_t LIDAR_renderThread(LIDAR_AppContext  * appCntxt)
{
    PTK_Map           * map;

    LIDAR_renderInit(appCntxt);

    PTK_printf("[%s] Waiting for output to render...\n", __FUNCTION__);

    map = LIDAROGAPPLIB_getOutAccMap(appCntxt->lidarHdl);
    PTK_assert(NULL != map);

    while (!appCntxt->exitRenderProcess)
    {
        if (appCntxt->visualize)
        {
            appCntxt->mapRenderer->setMap(map);
            appCntxt->renderer->step();
        }

        /* Save the output data, if configured. */
        if (appCntxt->dumpMap)
        {
            LIDAR_dumpOutData(appCntxt, map);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(appCntxt->renderPeriod));
    }

    LIDAR_renderDeInit(appCntxt);

    PTK_printf("[%s] Exiting...\n", __FUNCTION__);

    return 0;
}

int32_t LIDAR_launchRenderThread(LIDAR_AppContext   * appCntxt)
{
    appCntxt->exitRenderProcess = false;

    /* Spawn the capture thread. */
    appCntxt->renderThreadId = std::thread(LIDAR_renderThread, appCntxt);

    return 0;
}

void LIDAR_exitRenderThread(LIDAR_AppContext   * appCntxt)
{
    appCntxt->exitRenderProcess = true;

    if (appCntxt->renderThreadId.joinable())
    {
        appCntxt->renderThreadId.join();
    }

    return;
}
