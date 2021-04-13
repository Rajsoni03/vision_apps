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
#include "app_valet_parking_priv.h"
//#include <EGL/egl.h>

typedef struct
{
    uint32_t    flag;
    float       r;
    float       g;
    float       b;

} VALETAPP_GridEntry;

static VALETAPP_GridEntry gGridEntryTbl[] =
{
    /* Ground. */
    { VALETAPP_FLAG_GROUND,
      0.0, 0.6, 0.0},

    { VALETAPP_FLAG_FST |
      VALETAPP_FLAG_GROUND,
      0.0, 0.6, 0.0 },

    /** Detected as PFSD. */
    { VALETAPP_FLAG_FSD |
      VALETAPP_FLAG_FST |
      VALETAPP_FLAG_PFSD,
      0.0, 0.4, 0.9 },

    { VALETAPP_FLAG_FSD |
      VALETAPP_FLAG_GROUND |
      VALETAPP_FLAG_FST |
      VALETAPP_FLAG_PFSD,
      0.0, 0.4, 0.9 },

    /** Detected FSD but not PFSD. */
    { VALETAPP_FLAG_FSD |
      VALETAPP_FLAG_FST,
      0.0, 0.7, 0.7 },

    { VALETAPP_FLAG_FSD |
      VALETAPP_FLAG_GROUND |
      VALETAPP_FLAG_FST,
      0.0, 0.7, 0.7 },

    /** Occupied flag is set then always occupied color.
    *   FSD or PFSD ignored if occupied set. */
    { VALETAPP_FLAG_OCCUPIED,
      0.6, 0.0, 0.0 },

    { VALETAPP_FLAG_OCCUPIED |
      VALETAPP_FLAG_FST,
      0.6, 0.0, 0.0 },

    { VALETAPP_FLAG_OCCUPIED |
      VALETAPP_FLAG_FSD ,
      0.6, 0.0, 0.0 },

    { VALETAPP_FLAG_OCCUPIED |
      VALETAPP_FLAG_FSD |
      VALETAPP_FLAG_FST,
      0.6, 0.0, 0.0 },

    { VALETAPP_FLAG_OCCUPIED |
      VALETAPP_FLAG_FST |
      VALETAPP_FLAG_PFSD,
      0.6, 0.0, 0.0 },

    { VALETAPP_FLAG_OCCUPIED |
      VALETAPP_FLAG_FSD |
      VALETAPP_FLAG_PFSD,
      0.6, 0.0, 0.0 },

    { VALETAPP_FLAG_OCCUPIED |
      VALETAPP_FLAG_FSD |
      VALETAPP_FLAG_FST |
      VALETAPP_FLAG_PFSD,
      0.6, 0.0, 0.0 },

    { VALETAPP_FLAG_EGO,
      1.0, 1.0, 0.0 },

    { VALETAPP_FLAG_EGO |
      VALETAPP_FLAG_GROUND,
      1.0, 1.0, 0.0 },

    { VALETAPP_FLAG_EGO |
      VALETAPP_FLAG_OCCUPIED,
      1.0, 1.0, 0.0 },

    { VALETAPP_FLAG_EGO |
      VALETAPP_FLAG_FSD |
      VALETAPP_FLAG_FST,
      1.0, 1.0, 0.0 },

    { VALETAPP_FLAG_EGO |
      VALETAPP_FLAG_FSD |
      VALETAPP_FLAG_FST |
      VALETAPP_FLAG_GROUND,
      1.0, 1.0, 0.0 },

    { VALETAPP_FLAG_EGO |
      VALETAPP_FLAG_FSD |
      VALETAPP_FLAG_FST |
      VALETAPP_FLAG_OCCUPIED,
      1.0, 1.0, 0.0 },

    { VALETAPP_FLAG_EGO |
      VALETAPP_FLAG_FSD |
      VALETAPP_FLAG_FST |
      VALETAPP_FLAG_OCCUPIED |
      VALETAPP_FLAG_PFSD,
      1.0, 1.0, 0.0 },
    { VALETAPP_FLAG_TIDL_PSD_FREE,
      0.0, 0.4, 0.9},

    { VALETAPP_FLAG_TIDL_PSD_OCCUPIED,
      0.9, 0.4, 0.0}

}; /* gGridEntryTbl[] */

static void VALETAPP_renderCreateViewPort(MapRenderable   **mapRenderer,
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
    BoxedRenderable_setBorderColor(mapBox, 0.4f, 0.4f, 0.4f, 1.0f);
    BoxedRenderable_show(mapBox);

    Renderer_addRenderable(renderer, BoxedRenderable_asRenderable(mapBox), *dims);

    return;
}


static void VALETAPP_renderCreateRadarViewPort(RadarRenderable   **radarRenderer,
                                               Renderer           *renderer,
                                               char               *titleStr,
                                               PTK_Dimensions     *dims,
                                               int32_t             font)
{
    Label           *label;
    String          *title;
    BoxedRenderable *mapBox;

    *radarRenderer = RadarRenderable_createRadarRenderable(renderer);

    mapBox = BoxedRenderable_create(renderer);
    label  = Label_create(renderer);
    title  = Renderer_createString(renderer, font, titleStr);

    Label_addString(label, title);
    Label_setBackgroundColor(label, 0.2f, 0.2f, 0.2f, 0.6f);
    Label_setBorderColor(label, 0.2f, 0.2f, 0.2f, 0.8f);
    Label_setAlignment(label, PTK_ALIGNMENT_LEFT);
    Label_setPadding(label, 5);

    BoxedRenderable_setTitle(mapBox, label);
    BoxedRenderable_setContents(mapBox, RadarRenderable_asRenderable(*radarRenderer));
    BoxedRenderable_setBorderColor(mapBox, 0.4f, 0.4f, 0.4f, 1.0f);
    BoxedRenderable_show(mapBox);

    Renderer_addRenderable(renderer, BoxedRenderable_asRenderable(mapBox), *dims);

    return;
}

static void VALETAPP_renderCreateImgViewPort(Image          **imgRenderer,
                                             Renderer         *renderer,
                                             int8_t            showFlag,
                                             const char       *imgPath,
                                             unsigned int     *texture,
                                             char             *titleStr,
                                             PTK_Dimensions   *dims,
                                             int32_t           font,
                                             int32_t           fitType)
{
    Label           *label;
    String          *title;
    BoxedRenderable *mapBox;

    // load the first image
    if (showFlag)
    {
        *texture = Renderer_loadImageFromPath(renderer, imgPath);
        *imgRenderer = Image_create(renderer, *texture, 1, (PTK_FitType)fitType); // Image::STRETCH = 0, Image::FIT = 3
        Image_show(*imgRenderer);
    }

    mapBox = BoxedRenderable_create(renderer);
    label  = Label_create(renderer);
    title  = Renderer_createString(renderer, font, titleStr);

    Label_addString(label, title);
    Label_setBackgroundColor(label, 0.2f, 0.2f, 0.2f, 0.6f);
    Label_setBorderColor(label, 0.2f, 0.2f, 0.2f, 0.8f);
    Label_setAlignment(label, PTK_ALIGNMENT_LEFT);
    Label_setPadding(label, 5);

    BoxedRenderable_setTitle(mapBox, label);
    BoxedRenderable_setContents(mapBox, (Renderable *)*imgRenderer);
    BoxedRenderable_setBorderColor(mapBox, 0.4f, 0.4f, 0.4f, 1.0f);
    BoxedRenderable_show(mapBox);

    Renderer_addRenderable(renderer, BoxedRenderable_asRenderable(mapBox), *dims);

    return;
}


void VALETAPP_renderSetGridFlagView(VALETAPP_Context  *appCntxt,
                                    MapRenderable     *mapRenderer,
                                    FUSEDOGAPPLIB_input_mask_e sensorType,
                                    uint32_t           gridId)
{
    uint32_t numEntries;
    uint32_t i;
    uint32_t id;

    id = 0;

    if (sensorType == FUSEDOGAPPLIB_INPUT_MASK_RADAR)
    {
        id = 0;
    }
    else if (sensorType == FUSEDOGAPPLIB_INPUT_MASK_LIDAR)
    {
        id = 1;
    }
    else if (sensorType == FUSEDOGAPPLIB_INPUT_MASK_SFM)
    {
        id = 2;
    }
    else if (sensorType == FUSEDOGAPPLIB_INPUT_MASK_FUSION) /* Used for Fusion output. */
    {
        id = 3;
    }
    else if (sensorType == FUSEDOGAPPLIB_INPUT_MASK_PSD) /* Used for Fusion output. */
    {
        id = 4;
    }
    else
    {
        PTK_assert(0);
    }

    appCntxt->gridFlagView[id] =
        MapRenderable_createGridFlagView(mapRenderer);

    numEntries = sizeof(gGridEntryTbl)/sizeof(VALETAPP_GridEntry);

    for (i = 0; i < numEntries; i++ )
    {
        VALETAPP_GridEntry *e = &gGridEntryTbl[i];

        GridFlagView_setFlag(appCntxt->gridFlagView[id], e->flag);
        GridFlagView_setColor(appCntxt->gridFlagView[id], e->r, e->g, e->b);

        MapRenderable_add(mapRenderer,
                          (pGridView)appCntxt->gridFlagView[id],
                          gridId);
    }
}

void VALETAPP_renderInit(VALETAPP_Context *appCntxt)
{
    PTK_VirtualCamera   *virtCamera;
    MapRenderable       *mapRenderer;
    DemoDashboard       *dash;
    DashboardRenderable *dashRenderer;
    SFMOGAPPLIB_createParams *camParams;
    PTK_Dimensions      dims;
    int32_t             font;
    int32_t             smfont;
    uint32_t            mask;
    uint32_t            canWidth;
    uint32_t            canHeight;
    uint64_t            lastTime = 0;
    uint64_t            endTime = 0;
    char                datasetPathName[VALETAPP_MAX_LINE_LEN];

    mask = appCntxt->visualize;

    if (!mask)
    {
        return;
    }

    camParams = &appCntxt->cameraCreateParams;

    SensorDataPlayerINS_check_sensorstreams(appCntxt->dataPlayer, &lastTime);
    lastTime += 90000;

    SensorDataPlayerINS_getEndTime(appCntxt->dataPlayer,
                                   VALETAPP_SENSOR_INS,
                                   &endTime);

    //printf("latTime:%ld, endTime:%ld\n", lastTime, endTime);

    appCntxt->renderer = Renderer_createRenderer();

#ifdef PLATFORM_EGL
    printf("Waiting on display FD...\n");
    appCntxt->dispCntxt.displayReadySem->wait();
    printf("Waiting on display FD...done!!!!\n");
    appCntxt->renderer->setEglFdandPitch(appCntxt->dispCntxt.dispDmaBuffId,
                                 appCntxt->dispCntxt.dispDmaBuffFdOffset,
                                 appCntxt->dispCntxt.image_addr.stride_y);
#endif
    Renderer_createConsole(appCntxt->renderer);
    Renderer_createWindow(appCntxt->renderer,
                          "Demo: Automatic Parking using PTK Mapping and Fusion with TI OpenVX Framework",
                          appCntxt->winWidth,
                          appCntxt->winHeight);

    font   = Renderer_loadFont(appCntxt->renderer, appCntxt->fontFile, 18);
    smfont = Renderer_loadFont(appCntxt->renderer, appCntxt->fontFile, 18);

    /* Demo dashboard header. */
    /*
    dash = DemoDashboard_create(appCntxt->renderer, appCntxt->logoPath, font);
    */
    snprintf(datasetPathName,
             VALETAPP_MAX_LINE_LEN,
             "%s/sequence%04d",
             appCntxt->dbConfig.databasePath,
             appCntxt->dbConfig.dataSeqId);

    dash = DemoDashboard_create(appCntxt->renderer,
                                datasetPathName,
                                appCntxt->logoPath,
                                appCntxt->bkgrndPath,
                                font,
                                smfont,
                                endTime);

    appCntxt->dash = dash;

    dims.x = 0.25f;
    dims.y = 0.666f;//0.75f;
    dims.w = 0.5f;
    dims.h = 0.333f; //0.25f;
    DemoDashboard_show(dash);
    Renderer_addRenderable(appCntxt->renderer,
                           DemoDashboard_asRenderable(dash),
                           dims);

    /* DashboardRenderer */
    dashRenderer = DashboardRenderable_create(appCntxt->renderer,
                                              appCntxt->winWidth,
                                              appCntxt->winHeight);

    appCntxt->dashRenderer = dashRenderer;

    canWidth  = Renderer_getWidth(appCntxt->renderer);
    canHeight = Renderer_getHeight(appCntxt->renderer);
    DashboardRenderable_update(dashRenderer,
                               canWidth,
                               canHeight,
                               endTime,
                               lastTime);

    Renderer_addRenderable(appCntxt->renderer,
                           DashboardRenderable_asRenderable(dashRenderer),
                           dims);

    /*
    Statistics * stats = Statistics_create(appCntxt->renderer, smfont);
    Statistics_show(stats);
    Statistics_showExtended(stats);
    Renderer_addRenderable_2(appCntxt->renderer,
                             Statistics_asRenderable(stats));
    */

    mapRenderer = NULL;

    if (mask & FUSEDOGAPPLIB_INPUT_MASK_RADAR)
    {
        RADAROGAPPLIB_createParams   *raParams;
        PTK_Alg_RadarOgmapParams     *ogConfig;

        raParams = &appCntxt->radarCreateParams;
        ogConfig = &raParams->ogPfsdCfg.ogConfig;

        /* Radar map view port. */
        dims.x = 0.75f;
        dims.y = 0.333f;
        dims.w = 0.25f;
        dims.h = 0.333f;

        VALETAPP_renderCreateViewPort(&appCntxt->radarMapRenderer,
                                      appCntxt->renderer,
                                      "RADAR Occupancy Grid Map",
                                      &dims,
                                      font);

        VALETAPP_renderSetGridFlagView(appCntxt,
                                       appCntxt->radarMapRenderer,
                                       FUSEDOGAPPLIB_INPUT_MASK_RADAR,
                                       ogConfig->accGridId);

        mapRenderer = appCntxt->radarMapRenderer;

        /* Radar point cloud view port. */
        dims.x = 0.00f;
        dims.y = 0.333f;
        dims.w = 0.25f;
        dims.h = 0.333f;

        VALETAPP_renderCreateRadarViewPort(&appCntxt->radarRenderer,
                                           appCntxt->renderer,
                                           "RADAR Object Point Cloud [AWR1843: right]",
                                           &dims,
                                           font);
    }

    if (mask & FUSEDOGAPPLIB_INPUT_MASK_LIDAR)
    {
        LIDAROGAPPLIB_CreateParams *liParams;
        PTK_Alg_LidarOgmapParams   *ogConfig;

        liParams = &appCntxt->lidarCreateParams;
        ogConfig = &liParams->ogPfsdCfg.ogConfig;

        /* Lidar map view port. */
        dims.x = 0.75f;
        dims.y = 0.0f;
        dims.w = 0.25f;
        dims.h = 0.333f;

        VALETAPP_renderCreateViewPort(&appCntxt->lidarMapRenderer,
                                      appCntxt->renderer,
                                      "LIDAR Occupancy Grid Map",
                                      &dims,
                                      font);

        VALETAPP_renderSetGridFlagView(appCntxt,
                                       appCntxt->lidarMapRenderer,
                                       FUSEDOGAPPLIB_INPUT_MASK_LIDAR,
                                       ogConfig->accGridId);

        mapRenderer = appCntxt->lidarMapRenderer;
    }

    if (mask & FUSEDOGAPPLIB_INPUT_MASK_SFM)
    {
        PTK_Alg_SfmOgmapParams   *ogConfig;

        ogConfig  = &camParams->ogPfsdCfg.ogConfig;

        /* Camera map view port. */
        dims.x = 0.75f;
        dims.y = 0.666f;
        dims.w = 0.25f;
        dims.h = 0.333f;

        VALETAPP_renderCreateViewPort(&appCntxt->cameraMapRenderer,
                                      appCntxt->renderer,
                                      "CAMERA (Structure from Motion) Occupancy Grid Map",
                                      &dims,
                                      font);

        VALETAPP_renderSetGridFlagView(appCntxt,
                                       appCntxt->cameraMapRenderer,
                                       FUSEDOGAPPLIB_INPUT_MASK_SFM,
                                       ogConfig->accGridId);

        mapRenderer = appCntxt->cameraMapRenderer;
    }

    if (mask & FUSEDOGAPPLIB_INPUT_MASK_PSD)
    {
        PSLIB_createParams *psParams;

        psParams = &appCntxt->psCreateParams;

        /* TIDL PSD map view port. */
        dims.x = 0.25f; //0.0f;
        dims.y = 0.0f;
        dims.w = 0.25f; //0.37f;
        dims.h = 0.666f;

        VALETAPP_renderCreateViewPort(&appCntxt->psdMapRenderer,
                                      appCntxt->renderer,
                                      "TIDL Parking Spot Detection Grid Map",
                                      &dims,
                                      font);

        VALETAPP_renderSetGridFlagView(appCntxt,
                                       appCntxt->psdMapRenderer,
                                       FUSEDOGAPPLIB_INPUT_MASK_PSD,
                                       psParams->occupancyGridId);

        mapRenderer = appCntxt->psdMapRenderer;
    }


    if (mask & FUSEDOGAPPLIB_INPUT_MASK_PSD_IMG)
    {
        PSIMGLIB_Context  * cntxt;
        int32_t             dbSensorId;
        char                imagePath[VALETAPP_MAX_LINE_LEN];

        cntxt = (PSIMGLIB_Context *)appCntxt->psImgHdl;
        dbSensorId = PTK_DBConfig_exist_sensor_apptag(&appCntxt->dbConfig,
                                                      "CAM_PSD_IMG");
        if (dbSensorId < 0)
        {
            PTK_printf("[%s:%d] Invalid sensor Id [%d].\n",
                       __FUNCTION__, __LINE__, dbSensorId);
            return;
        }

        snprintf(imagePath,
                 VALETAPP_MAX_LINE_LEN,
                 "%s/sequence%04d/%s/data/%010d.bmp",
                 appCntxt->dbConfig.databasePath,
                 appCntxt->dbConfig.dataSeqId,
                 appCntxt->dbConfig.sensors[dbSensorId].folder,
                 appCntxt->dbConfig.startFrame);

        dims.x = 0.0f;
        dims.y = 0.666f;
        dims.w = 0.25f;
        dims.h = 0.333f;

        VALETAPP_renderCreateImgViewPort(&appCntxt->psdImgRenderer,
                                         appCntxt->renderer,
                                         appCntxt->tidlPsdImgEnabled,
                                         imagePath,
                                         &cntxt->texture,
                                         "CAMERA [OV2311: right]",
                                         &dims,
                                         font,
                                         0);
    }

	/*  Left camera
	if (mask & FUSEDOGAPPLIB_INPUT_MASK_CAM_R_IMG)
	{
		PSIMGLIB_Context  * cntxt;
		int32_t             dbSensorId;
		char                imagePath[VALETAPP_MAX_LINE_LEN];

		cntxt = (PSIMGLIB_Context *)appCntxt->camRImgHdl;
		dbSensorId = PTK_DBConfig_exist_sensor_apptag(&appCntxt->dbConfig,
			"CAM_R_IMG");

        if (dbSensorId < 0)
        {
            PTK_printf("[%s:%d] Invalid sensor Id [%d].\n",
                       __FUNCTION__, __LINE__, dbSensorId);
            return;
        }

		snprintf(imagePath,
			VALETAPP_MAX_LINE_LEN,
			"%s/sequence%04d/%s/data/%010d.bmp",
			appCntxt->dbConfig.databasePath,
			appCntxt->dbConfig.dataSeqId,
			appCntxt->dbConfig.sensors[dbSensorId].folder,
			appCntxt->dbConfig.startFrame);

		dims.x = 0.0f;
		dims.y = 0.666f;
		dims.w = 0.25f;
		dims.h = 0.333f;

		VALETAPP_renderCreateImgViewPort(&appCntxt->psdImgRenderer,
			appCntxt->renderer,
			appCntxt->tidlPsdImgEnabled,
			imagePath,
			&cntxt->texture,
			"TIDL Parking Spot Detection on Fisheye Camera",
			&dims,
			font,
			0);
	}
*/
    // DoF image
    if (mask & FUSEDOGAPPLIB_INPUT_MASK_DOF_IMG)
    {
        DOFIMGLIB_Context * cntxt;
        int32_t             dbSensorId;
        char                imagePath[VALETAPP_MAX_LINE_LEN];

        cntxt = (DOFIMGLIB_Context *)appCntxt->dofImgHdl;
        dbSensorId = PTK_DBConfig_exist_sensor_apptag(&appCntxt->dbConfig,
                                                      "CAM_DOF_IMG");

        if (dbSensorId < 0)
        {
            PTK_printf("[%s:%d] Invalid sensor Id [%d].\n",
                       __FUNCTION__, __LINE__, dbSensorId);
            return;
        }

        snprintf(imagePath,
                 VALETAPP_MAX_LINE_LEN,
                 "%s/sequence%04d/%s/data/%010d.bmp",
                 appCntxt->dbConfig.databasePath,
                 appCntxt->dbConfig.dataSeqId,
                 appCntxt->dbConfig.sensors[dbSensorId].folder,
                 appCntxt->dbConfig.startFrame);

        dims.x = 0.0f;
        dims.y = 0.0f;
        dims.w = 0.25f;
        dims.h = 0.333f;

        VALETAPP_renderCreateImgViewPort(&appCntxt->dofImgRenderer,
                                         appCntxt->renderer,
                                         appCntxt->dofImgEnabled,
                                         imagePath,
                                         &cntxt->texture,
                                         "Dense Optical Flow Output",
                                         &dims,
                                         font,
                                         3);
    }

    /*  Print-out free-space dimensions as discovered
    if (appCntxt->fusionEnabled)
    {
        FUSEDOGAPPLIB_createParams *createParams;
        PTK_Alg_FusedOgmapParams   *ogConfig;

        createParams  = &appCntxt->fusedOgCreateParams;
        ogConfig      = &createParams->ogPfsdCfg.ogConfig;

        // Text map view port
        dims.x = 0.0f;
        dims.y = 0.0f;
        dims.w = 0.250f;
        dims.h = 0.333f;

        VALETAPP_renderCreateViewPort(&appCntxt->fusionMapRenderer,
                                      appCntxt->renderer,
                                      "FUSED Occupancy Grid Map [CAMERA (SfM) + RADAR + LIDAR]",
                                      &dims,
                                      font);

        VALETAPP_renderSetGridFlagView(appCntxt,
                                       appCntxt->fusionMapRenderer,
                                       FUSEDOGAPPLIB_INPUT_MASK_FUSION,
                                       ogConfig->outGridId);

        mapRenderer = appCntxt->fusionMapRenderer;
    } */

    if (appCntxt->fusionEnabled)
    {
        FUSEDOGAPPLIB_createParams *createParams;
        PTK_Alg_FusedOgmapParams   *ogConfig;

        createParams  = &appCntxt->fusedOgCreateParams;
        ogConfig      = &createParams->ogPfsdCfg.ogConfig;

        /* Fused map view port. */
        dims.x = 0.25f; //0.37f;
        dims.y = 0.0f;
        dims.w = 0.50f; // 0.38f;
        dims.h = 0.666f;

        VALETAPP_renderCreateViewPort(&appCntxt->fusionMapRenderer,
                                      appCntxt->renderer,
                                      "FUSED Occupancy Grid Map [CAMERA (SfM) + RADAR + LIDAR]",
                                      &dims,
                                      font);

        VALETAPP_renderSetGridFlagView(appCntxt,
                                       appCntxt->fusionMapRenderer,
                                       FUSEDOGAPPLIB_INPUT_MASK_FUSION,
                                       ogConfig->outGridId);

        mapRenderer = appCntxt->fusionMapRenderer;
    }

    if (!mapRenderer)
    {
        PTK_assert(0);
    }

    appCntxt->keyHandlerTbl = Renderer_createKeyHandlerTable();
    Renderer_setKeyHandlerTable(appCntxt->renderer, appCntxt->keyHandlerTbl);

    virtCamera =
        Renderable_getVirtualCamera(MapRenderable_asRenderable(mapRenderer));

    Renderer_addCameraControls(appCntxt->renderer,
                               appCntxt->keyHandlerTbl,
                               virtCamera);

    return;

} /* VALETAPP_renderInit */

static void VALETAPP_renderAndSaveCamera(VALETAPP_Context *appCntxt,
                                     uint32_t          renderFlag,
                                     uint32_t          dumpFlag)
{
    PTK_Map            *map;
    char                charBuffer[DBCONFIG_MAX_PATH_LEN] = {0};
    char               *dstFilePath = charBuffer;

    map = SFMOGAPPLIB_getOutAccMap(appCntxt->cameraHdl);
    PTK_assert(NULL != map);

    if (renderFlag)
    {
        MapRenderable_setMap(appCntxt->cameraMapRenderer, map);
    }

    if (dumpFlag)
    {
        VirtualSensorCreator_add_record(
                appCntxt->outputStrmHdl[VALETAPP_OUTPUT_SFM_MAP],
                (char **)&dstFilePath,
                appCntxt->curTimeStamp,
                (uint8_t *)map,
                appCntxt->cameraMapSize);
    }

    return;

} /* VALETAPP_renderAndSaveCamera */

static void VALETAPP_renderAndSavePSD(VALETAPP_Context *appCntxt,
                                      uint32_t          renderFlag,
                                      uint32_t          dumpFlag)
{
    PTK_Map            *map;

    map = PSLIB_mapOutput(appCntxt->psHdl);
    PTK_assert(NULL != map);

    if (renderFlag)
    {
        MapRenderable_setMap(appCntxt->psdMapRenderer, map);
    }

    PSLIB_unmapOutput(appCntxt->psHdl);

    return;

} /* VALETAPP_renderAndSavePSD */


static void VALETAPP_renderAndSavePSDImage(VALETAPP_Context *appCntxt,
                                           uint32_t          renderFlag,
                                           uint32_t          dumpFlag)
{
    PSIMGLIB_Context * cntxt = (PSIMGLIB_Context *)appCntxt->psImgHdl;

    if (renderFlag && cntxt->bInit == 1)
    {
        Renderer_loadImageFromPathWithTex(appCntxt->renderer,
                                          cntxt->imgFileName, cntxt->texture);
    }

    return;

} /* VALETAPP_renderAndSavePSDImage */

static void VALETAPP_renderAndSaveDOFImage(VALETAPP_Context *appCntxt,
                                           uint32_t          renderFlag,
                                           uint32_t          dumpFlag)
{
    DOFIMGLIB_Context * cntxt = (DOFIMGLIB_Context *)appCntxt->dofImgHdl;

    if (renderFlag && cntxt->bInit == 1)
    {
        Renderer_loadImageFromPathWithTex(appCntxt->renderer,
                                          cntxt->imgFileName, cntxt->texture);
    }

    return;

} /* VALETAPP_renderAndSaveDOFImage */


static void VALETAPP_renderAndSaveRadar(VALETAPP_Context *appCntxt,
                                        uint32_t          renderFlag,
                                        uint32_t          dumpFlag)
{
    PTK_Map            *map;
    char                charBuffer[DBCONFIG_MAX_PATH_LEN] = {0};
    char               *dstFilePath = charBuffer;

    map = RADAROGAPPLIB_getOutAccMap(appCntxt->radarHdl);
    PTK_assert(NULL != map);

    if (renderFlag)
    {
        MapRenderable_setMap(appCntxt->radarMapRenderer, map);
    }

    if (dumpFlag)
    {
        VirtualSensorCreator_add_record(
                appCntxt->outputStrmHdl[VALETAPP_OUTPUT_RADAR_MAP],
                (char **)&dstFilePath,
                appCntxt->curTimeStamp,
                (uint8_t *)map,
                appCntxt->radarMapSize);
    }

    // Radar point cloud
    if (renderFlag)
    {
        RADARLIB_Context * cntxt = (RADARLIB_Context *)appCntxt->radarPCHdl;

        if (cntxt->data != NULL)
        {
            RadarRenderable_setDetectionList(appCntxt->radarRenderer,
                                             (void *)cntxt->data);
        }
    }

    return;

} /* VALETAPP_renderAndSaveRadar */

static void VALETAPP_renderAndSaveLidar(VALETAPP_Context *appCntxt,
                                        uint32_t          renderFlag,
                                        uint32_t          dumpFlag)
{
    PTK_Map            *map;
    char                charBuffer[DBCONFIG_MAX_PATH_LEN] = {0};
    char               *dstFilePath = charBuffer;

    map = LIDAROGAPPLIB_getOutAccMap(appCntxt->lidarHdl);
    PTK_assert(NULL != map);

    if (renderFlag)
    {
        MapRenderable_setMap(appCntxt->lidarMapRenderer, map);
    }

    if (dumpFlag)
    {
        VirtualSensorCreator_add_record(
                appCntxt->outputStrmHdl[VALETAPP_OUTPUT_LIDAR_MAP],
                (char **)&dstFilePath,
                appCntxt->curTimeStamp,
                (uint8_t *)map,
                appCntxt->lidarMapSize);
    }

    return;
}

static void VALETAPP_renderAndSaveFusedOutput(VALETAPP_Context *appCntxt,
                                              uint32_t          renderFlag,
                                              uint32_t          dumpFlag)
{
    char        charBuffer[DBCONFIG_MAX_PATH_LEN] = {0};
    PTK_Map    *map;
    char       *dstFilePath = charBuffer;

    map = FUSEDOGAPPLIB_getOutMap(appCntxt->fusedOgHdl);
    PTK_assert(NULL != map);

    if (renderFlag)
    {
        MapRenderable_setMap(appCntxt->fusionMapRenderer, map);
    }

    if (dumpFlag)
    {
        VirtualSensorCreator_add_record(
                appCntxt->outputStrmHdl[VALETAPP_OUTPUT_FUSED_MAP],
                (char **)&dstFilePath,
                appCntxt->curTimeStamp,
                (uint8_t *)map,
                appCntxt->fusedMapSize);
    }

    return;
}

void VALETAPP_renderAndSave(VALETAPP_Context *appCntxt)
{
    uint32_t  renderFlag;
    int32_t  dumpFlag;

    if (appCntxt->cameraEnabled)
    {
        renderFlag = appCntxt->visualize & FUSEDOGAPPLIB_INPUT_MASK_SFM;
        dumpFlag = (appCntxt->outputStrmHdl[VALETAPP_OUTPUT_SFM_MAP] != NULL);

        if (renderFlag || dumpFlag)
        {
            VALETAPP_renderAndSaveCamera(appCntxt, renderFlag, dumpFlag);
        }
    }

    if (appCntxt->radarEnabled)
    {
        renderFlag = appCntxt->visualize & FUSEDOGAPPLIB_INPUT_MASK_RADAR;
        dumpFlag = (appCntxt->outputStrmHdl[VALETAPP_OUTPUT_RADAR_MAP] != NULL);

        if (renderFlag || dumpFlag)
        {
            VALETAPP_renderAndSaveRadar(appCntxt, renderFlag, dumpFlag);
        }
    }

    if (appCntxt->lidarEnabled)
    {
        renderFlag = appCntxt->visualize & FUSEDOGAPPLIB_INPUT_MASK_LIDAR;
        dumpFlag = (appCntxt->outputStrmHdl[VALETAPP_OUTPUT_LIDAR_MAP] != NULL);

        if (renderFlag || dumpFlag)
        {
            VALETAPP_renderAndSaveLidar(appCntxt, renderFlag, dumpFlag);
        }
    }

    if (appCntxt->tidlPsdEnabled)
    {
        renderFlag = appCntxt->visualize & FUSEDOGAPPLIB_INPUT_MASK_PSD;
        VALETAPP_renderAndSavePSD(appCntxt, renderFlag, 0);
    }

    if (appCntxt->tidlPsdImgEnabled)
    {
        renderFlag = appCntxt->visualize & FUSEDOGAPPLIB_INPUT_MASK_PSD_IMG;
        VALETAPP_renderAndSavePSDImage(appCntxt, renderFlag, 0);
    }

    if (appCntxt->dofImgEnabled)
    {
        renderFlag = appCntxt->visualize & FUSEDOGAPPLIB_INPUT_MASK_DOF_IMG;
        VALETAPP_renderAndSaveDOFImage(appCntxt, renderFlag, 0);
    }


    if (appCntxt->fusionEnabled)
    {
        renderFlag = appCntxt->visualize & FUSEDOGAPPLIB_INPUT_MASK_FUSION;
        dumpFlag = (appCntxt->outputStrmHdl[VALETAPP_OUTPUT_FUSED_MAP] != NULL);

        if (renderFlag || dumpFlag)
        {
            VALETAPP_renderAndSaveFusedOutput(appCntxt, renderFlag, dumpFlag);
        }
    }

    if (appCntxt->visualize)
    {
        PTK_INS_Record insRec;

        /* Get the latest IMU record. */
        PTK_INS_getCurrentRecord(PTK_INS_RECORD_TYPE_INSPVA, &insRec);

        DemoDashboard_setPositionInfo(appCntxt->dash, &insRec);

        DashboardRenderable_setPositionInfo(appCntxt->dashRenderer, &insRec);
        DashboardRenderable_setCurTime(appCntxt->dashRenderer,
                                       insRec.timestamp);

        Renderer_step(appCntxt->renderer);

        /* Screen capture */
#if 0
        static int frameNum = 0;
        char path[VALETAPP_MAX_LINE_LEN];

        int32_t scWidth  = Renderer_getWidth(appCntxt->renderer);
        int32_t scHeight = Renderer_getHeight(appCntxt->renderer);

        uint8_t *pixels = (uint8_t *) malloc(3*scWidth*scHeight);

        if (pixels == NULL)
        {
            PTK_printf("[%s:%d] Memory allocation failed.\n",
                       __FUNCTION__, __LINE__);

            return;
        }

        for (uint32_t y = 0; y < scHeight; ++y)
        {
            glReadPixels(0, y, scWidth, 1, GL_RGB, GL_UNSIGNED_BYTE,
                         pixels + 3*scWidth*(scHeight-1-y));
        }

        SDL_Surface *surface = SDL_CreateRGBSurfaceFrom(pixels,
                                                        scWidth,
                                                        scHeight,
                                                        24,
                                                        3*scWidth,
                                                        0xff,
                                                        0xff00,
                                                        0xff0000,
                                                        0);

        sprintf(path, "%s/sequence%04d/vizOut/%010d.bmp",
                appCntxt->dbConfig.databasePath,
                appCntxt->dbConfig.dataSeqId, frameNum);
        frameNum += 1;
        if (frameNum % 5 == 0)
        {
            SDL_SaveBMP(surface, path);
        }
        SDL_FreeSurface(surface);
        free(pixels);
#endif
    }

    return;

} /* VALETAPP_renderAndSave */

void VALETAPP_renderDeInit(VALETAPP_Context *appCntxt)
{
    if (appCntxt->visualize)
    {
        MapRenderable_delete(appCntxt->radarMapRenderer);
        Renderer_delete(appCntxt->renderer);
    }

    return;

} /* VALETAPP_renderDeInit */

int32_t VALETAPP_renderThread(VALETAPP_Context  * appCntxt)
{
    VALETAPP_renderInit(appCntxt);

    PTK_printf("[%s] Waiting for output to render...\n", __FUNCTION__);

    while (!appCntxt->exitRenderProcess)
    {
        std::chrono::microseconds    microSec;

        VALETAPP_renderAndSave(appCntxt);

        microSec = std::chrono::microseconds(appCntxt->renderPeriod);
        std::this_thread::sleep_for(microSec);
    }

    VALETAPP_renderDeInit(appCntxt);

    PTK_printf("[%s] Exiting...\n", __FUNCTION__);

    return 0;
}

int32_t VALETAPP_launchRenderThread(VALETAPP_Context   * appCntxt)
{
    appCntxt->exitRenderProcess = false;

    /* Spawn the capture thread. */
    appCntxt->renderThreadId = std::thread(VALETAPP_renderThread, appCntxt);

    return 0;
}

void VALETAPP_exitRenderThread(VALETAPP_Context    * appCntxt)
{
    appCntxt->exitRenderProcess = true;

    if (appCntxt->renderThreadId.joinable())
    {
        appCntxt->renderThreadId.join();
    }

    return;
}
