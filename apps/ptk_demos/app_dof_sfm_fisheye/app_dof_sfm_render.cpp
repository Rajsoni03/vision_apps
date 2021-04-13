/*
 *
 * Copyright (c) 2017 Texas Instruments Incorporated
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

#include "app_dof_sfm_main.h"
//#include "/usr/include/SDL2/SDL.h"
//#include "/usr/include/GL/gl.h"

#define WINDOW_WIDTH    1920
#define WINDOW_HEIGHT   1080


typedef struct
{
    uint32_t    flag;
    float       r;
    float       g;
    float       b;

} DOFSFMAPP_GridEntry;

static DOFSFMAPP_GridEntry gGridEntryTbl[] =
{
      /* Ground. */
      { DOFSFMAPP_FLAG_GROUND,
        0.0, 0.6, 0.0},

      { DOFSFMAPP_FLAG_FST |
        DOFSFMAPP_FLAG_GROUND,
        0.0, 0.6, 0.0 },

      /** Detected as PFSD. */
      { DOFSFMAPP_FLAG_FSD |
        DOFSFMAPP_FLAG_FST |
        DOFSFMAPP_FLAG_PFSD,
        0.0, 0.4, 0.9 },

      { DOFSFMAPP_FLAG_FSD |
        DOFSFMAPP_FLAG_GROUND |
        DOFSFMAPP_FLAG_FST |
        DOFSFMAPP_FLAG_PFSD,
        0.0, 0.4, 0.9 },

      /** Detected FSD but not PFSD. */
      { DOFSFMAPP_FLAG_FSD |
        DOFSFMAPP_FLAG_FST,
        0.0, 0.7, 0.7 },

      { DOFSFMAPP_FLAG_FSD |
        DOFSFMAPP_FLAG_GROUND |
        DOFSFMAPP_FLAG_FST,
        0.0, 0.7, 0.7 },

      /** Occupied flag is set then always occupied color.
      *   FSD or PFSD ignored if occupied set. */
      { DOFSFMAPP_FLAG_OCCUPIED,
        0.6, 0.0, 0.0 },

      { DOFSFMAPP_FLAG_OCCUPIED |
        DOFSFMAPP_FLAG_FST,
        0.6, 0.0, 0.0 },

      { DOFSFMAPP_FLAG_OCCUPIED |
        DOFSFMAPP_FLAG_FSD ,
        0.6, 0.0, 0.0 },

      { DOFSFMAPP_FLAG_OCCUPIED |
        DOFSFMAPP_FLAG_FSD |
        DOFSFMAPP_FLAG_FST,
        0.6, 0.0, 0.0 },

      { DOFSFMAPP_FLAG_OCCUPIED |
        DOFSFMAPP_FLAG_FST |
        DOFSFMAPP_FLAG_PFSD,
        0.6, 0.0, 0.0 },

      { DOFSFMAPP_FLAG_OCCUPIED |
        DOFSFMAPP_FLAG_FSD |
        DOFSFMAPP_FLAG_PFSD,
        0.6, 0.0, 0.0 },

      { DOFSFMAPP_FLAG_OCCUPIED |
        DOFSFMAPP_FLAG_FSD |
        DOFSFMAPP_FLAG_FST |
        DOFSFMAPP_FLAG_PFSD,
        0.6, 0.0, 0.0 }

}; /* gGridEntryTbl[] */

static void DOFSFMAPP_renderCreateViewPort(MapRenderable   **mapRenderer,
                                           Renderer         *renderer,
                                           char             *titleStr,
                                           PTK_Dimensions   *dims,
                                           int32_t           font)
{
    Label           *label;
    String          *title;
    BoxedRenderable *mapBox;

    *mapRenderer = MapRenderable_createBMPMapRenderable(renderer);

    mapBox = BoxedRenderable_create(renderer);
    label  = Label_create(renderer);
    title  = Renderer_createString(renderer, font, titleStr);

    Label_addString(label, title);
    Label_setBackgroundColor(label, 0.2f, 0.2f, 0.2f, 0.6f);
    Label_setBorderColor(label, 0.2f, 0.2f, 0.2f, 0.8f);
    Label_setAlignment(label, PTK_ALIGNMENT_LEFT);
    Label_setPadding(label, 5);

    BoxedRenderable_setTitle(mapBox, label);
    BoxedRenderable_setContents(mapBox, MapRenderable_asRenderable(*mapRenderer));
    BoxedRenderable_setBorderColor(mapBox, 0.4f, 0.4f, 0.4f, 0.8f);
    BoxedRenderable_show(mapBox);

    Renderer_addRenderable(renderer, BoxedRenderable_asRenderable(mapBox), *dims);

    return;
}



static void DOFSFMAPP_renderAndSaveCamera(AppObj           *obj,
                                          uint32_t          renderFlag,
                                          uint64_t          curTimestamp)
{
    PTK_Map            *map;
    char                charBuffer[DBCONFIG_MAX_PATH_LEN] = {0};
    char               *dstFilePath = charBuffer;

    map = SFMOGAPPLIB_getOutAccMap(obj->cameraHdl);
    APP_ASSERT(NULL != map);

    if (renderFlag)
    {
        MapRenderable_setMap(obj->cameraMapRenderer, map);
    }

    if (!VirtualSensorCreator_is_disabled(obj->mapOut))
    {
        VirtualSensorCreator_add_record(obj->mapOut,
                                        (char **)&dstFilePath,
                                        curTimestamp,
                                        (uint8_t *)map,
                                        obj->cameraMapSize);
    }

    return;

} /* DOFSFMAPP_renderAndSaveCamera */


static void DOFSFMAPP_renderSetGridFlagView(AppObj            *obj,
                                            MapRenderable     *mapRenderer,
                                            //FUSEDOGAPPLIB_input_mask_e sensorType,
                                            uint32_t           gridId)
{
    uint32_t numEntries;
    uint32_t i;
    uint32_t id;

    id = 0;

    obj->gridFlagView =
        MapRenderable_createGridFlagView(mapRenderer);

    numEntries = sizeof(gGridEntryTbl)/sizeof(DOFSFMAPP_GridEntry);

    for (i = 0; i < numEntries; i++ )
    {
        DOFSFMAPP_GridEntry *e = &gGridEntryTbl[i];

        GridFlagView_setFlag(obj->gridFlagView, e->flag);
        GridFlagView_setColor(obj->gridFlagView, e->r, e->g, e->b);

        MapRenderable_add(mapRenderer,
                          (pGridView)obj->gridFlagView,
                          gridId);
    }
}


void DOFSFMAPP_renderInit(AppObj *obj)
{
    if (obj->visualize)
    {
        SFMOGAPPLIB_createParams *camParams;
        PTK_Alg_SfmOgmapParams   *ogConfig;
        PTK_VirtualCamera   *virtCamera;
        MapRenderable       *mapRenderer;
        DemoDashboard       *dash;
        DashboardRenderable *dashRenderer;
        PTK_Dimensions      dims;
        int32_t             font;
#if 0
    int32_t             smfont;
#endif
        uint32_t            mask;

        uint32_t            canWidth, canHeight;

        uint64_t lastTime = 0;
        uint64_t endTime = 0;

        SensorDataPlayerINS_check_sensorstreams(obj->dataPlayer, &lastTime);
        lastTime += 90000;

        SensorDataPlayerINS_getEndTime(obj->dataPlayer, DOFSFMAPP_SENSOR_INS, &endTime);

        obj->renderer = Renderer_createRenderer();

#ifdef PLATFORM_EGL
        obj->dispCntxt.displayReadySem->wait();
        obj->renderer->setEglFdandPitch(obj->dispCntxt.dispDmaBuffId,
                                obj->dispCntxt.dispDmaBuffFdOffset,
                                obj->dispCntxt.image_addr.stride_y);
#endif

        Renderer_createConsole(obj->renderer);
        Renderer_createWindow(obj->renderer,
                              "TI OpenVX SFM Demo", WINDOW_WIDTH, WINDOW_HEIGHT);

        font   = Renderer_loadFont(obj->renderer, obj->fontFile, 14);

        camParams = &obj->cameraCreateParams;
        ogConfig  = &camParams->ogPfsdCfg.ogConfig;

        /* Camera map view port. */
        dims.x = 0.0f;
        dims.y = 0.0f;
        dims.w = 1.0f;
        dims.h = 1.0f;

        DOFSFMAPP_renderCreateViewPort(&obj->cameraMapRenderer,
                                       obj->renderer,
                                       "STRUCTURE from MOTION (SfM) [Occupancy Grid Map]",
                                       &dims,
                                       font);

        mapRenderer = obj->cameraMapRenderer;

        if (!mapRenderer)
        {
            APP_ASSERT(0);
        }

        DOFSFMAPP_renderSetGridFlagView(obj,
                                        mapRenderer,
                                        ogConfig->accGridId);

        obj->keyHandlerTbl = Renderer_createKeyHandlerTable();
        Renderer_setKeyHandlerTable(obj->renderer, obj->keyHandlerTbl);

        virtCamera = Renderable_getVirtualCamera(MapRenderable_asRenderable(mapRenderer));

        Renderer_addCameraControls(obj->renderer,
                               obj->keyHandlerTbl,
                               virtCamera);
    }

    return;

} /* DOFSFMAPP_renderInit */


void DOFSFMAPP_renderAndSave(AppObj *obj)
{
    PTK_INS_Record insRec;

    DOFSFMAPP_renderAndSaveCamera(obj, obj->visualize, obj->curTimestamp);

    if (obj->visualize)
    {
        Renderer_step(obj->renderer);
    }
}

void DOFSFMAPP_renderDeinit(AppObj *obj)
{
    if (obj->visualize)
    {
        MapRenderable_delete(obj->cameraMapRenderer);
        Renderer_delete(obj->renderer);
    }

    return;
}

int32_t DOFSFMAPP_renderThread(AppObj  * obj)
{
    DOFSFMAPP_renderInit(obj);

    PTK_printf("[%s] Waiting for output to render...\n", __FUNCTION__);


    while (!obj->exitRenderProcess)
    {
        std::chrono::microseconds    microSec;

        DOFSFMAPP_renderAndSave(obj);

        microSec = std::chrono::microseconds(obj->renderPeriod);
        std::this_thread::sleep_for(microSec);
    }

    DOFSFMAPP_renderDeinit(obj);

    PTK_printf("[%s] Exiting...\n", __FUNCTION__);

    return 0;
}

int32_t DOFSFMAPP_launchRenderThread(AppObj   * obj)
{
    obj->exitRenderProcess = false;

    /* Spawn the capture thread. */
    obj->renderThreadId = std::thread(DOFSFMAPP_renderThread, obj);

    return 0;
}

void DOFSFMAPP_exitRenderThread(AppObj * obj)
{
    obj->exitRenderProcess = true;

    if (obj->renderThreadId.joinable())
    {
        obj->renderThreadId.join();
    }

    return;
}
