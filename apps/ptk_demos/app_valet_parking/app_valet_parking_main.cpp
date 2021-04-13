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

#include <signal.h>

#include <TI/tivx_park_assist_kernels.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <utils/perf_stats/include/app_perf_stats.h>

#ifdef __cplusplus
}
#endif

#include <perception/utils/bit_utils.h>
#include <perception/utils/calmat_utils.h>
#include <perception/utils/fsd_pfsd_parse_config.h>
#include <perception/utils/radar_ogmap_parse_config.h>
#include <perception/utils/lidar_ogmap_parse_config.h>
#include <perception/utils/lidar_gating_parse_config.h>
#include <perception/utils/sfm_ogmap_parse_config.h>
#include <perception/utils/fused_ogmap_parse_config.h>
#include <perception/utils/grid_parse_config.h>

#include <app_ptk_demo_common.h>

#include "app_valet_parking_priv.h"
#include "lens_lut.h"
#include "projection_mat.h"

static VALETAPP_Context  gTestAppCntxt{0};

static char sensorAppTags[VALETAPP_SENSOR_MAX][DBCONFIG_MAX_WORD_LEN] =
    {{"CAM_R"},{"RAD_0"},{"RAD_1"},{"RAD_2"},{"RAD_3"},{"LID"},{"INS"},{"CAM_PSD"},{"CAM_PSD_IMG"},{"CAM_DOF_IMG"}};

static char outputAppTags[VALETAPP_OUTPUT_MAX][DBCONFIG_MAX_WORD_LEN] =
    {{"MAP_SFM"},{"MAP_RADAR"},{"MAP_LIDAR"},{"MAP_FUSED"}};

static FUSEDOGAPPLIB_input_mask_e processRadar(VALETAPP_Context *appCntxt, FUSEDOGAPPLIB_processReq *req,
                         uint32_t s, uint8_t *data, size_t size, uint64_t ts);
static FUSEDOGAPPLIB_input_mask_e processLidar(VALETAPP_Context *appCntxt, FUSEDOGAPPLIB_processReq *req,
                         uint32_t s, uint8_t *data, size_t size, uint64_t ts);
static FUSEDOGAPPLIB_input_mask_e processCamera(VALETAPP_Context *appCntxt, FUSEDOGAPPLIB_processReq *req,
                         uint32_t s, uint8_t *data, size_t size, uint64_t ts);
static FUSEDOGAPPLIB_input_mask_e processPS(VALETAPP_Context *appCntxt, FUSEDOGAPPLIB_processReq *req,
                         uint32_t s, uint8_t *data, size_t size, uint64_t ts);
static FUSEDOGAPPLIB_input_mask_e processPSImg(VALETAPP_Context *appCntxt, FUSEDOGAPPLIB_processReq *req,
                         uint32_t s, uint8_t *data, size_t size, uint64_t ts);
static FUSEDOGAPPLIB_input_mask_e processDOFImg(VALETAPP_Context *appCntxt, FUSEDOGAPPLIB_processReq *req,
                         uint32_t s, uint8_t *data, size_t size, uint64_t ts);

FUSEDOGAPPLIB_input_mask_e   (*sensorProcessCalls[VALETAPP_SENSOR_MAX])(VALETAPP_Context *appCntxt,
                                                                        FUSEDOGAPPLIB_processReq *req,
                                                                        uint32_t                  s,
                                                                        uint8_t                  *data,
                                                                        size_t                   size,
                                                                        uint64_t                 ts) =
    {processCamera, processRadar, processRadar, processRadar, processRadar, processLidar, NULL, processPS, processPSImg, processDOFImg};

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

static PSIMGLIB_Handle PSIMGLIB_create()
{
    PSIMGLIB_Context    *appCntxt;
    PSIMGLIB_Handle      handle;

    handle = (PSIMGLIB_Handle)malloc(sizeof(PSIMGLIB_Context));

    if (handle == NULL)
    {
        PTK_printf("[%s:%d] Could not allocate memory for internal context.\n",
                   __FUNCTION__, __LINE__);

        return NULL;
    }

    appCntxt = (PSIMGLIB_Context*) handle;

    /* Set applib-level create parameters */
    appCntxt->bInit = 0;

    return handle;
}

/* de-initialize parking spot lib */
void VALETAPP_psLibDeInit(VALETAPP_Context *appCntxt)
{
    PSLIB_delete(&appCntxt->psHdl);
}

static void PSIMGLIB_delete(PSIMGLIB_Handle *handle)
{
    if (*handle)
    {
        free(*handle);
        *handle = NULL;
    }

    return;
}

static void PSIMGLIB_process(PSIMGLIB_Handle handle, uint8_t* data, uint64_t timestamp)
{
    PSIMGLIB_Context *appCntxt = (PSIMGLIB_Context *)handle;

    strcpy(appCntxt->imgFileName, (char*)data);

    appCntxt->bInit = 1;

    return;
}

static DOFIMGLIB_Handle DOFIMGLIB_create()
{
    DOFIMGLIB_Context    *appCntxt;
    DOFIMGLIB_Handle      handle;

    handle = (DOFIMGLIB_Handle)malloc(sizeof(DOFIMGLIB_Context));

    if (handle == NULL)
    {
        PTK_printf("[%s:%d] Could not allocate memory for internal context.\n",
                   __FUNCTION__, __LINE__);

        return NULL;
    }

    appCntxt = (DOFIMGLIB_Context*) handle;

    /* Set applib-level create parameters */
    appCntxt->bInit = 0;

    return handle;
}

static void DOFIMGLIB_delete(DOFIMGLIB_Handle *handle)
{
    if (*handle)
    {
        free(*handle);
        *handle = NULL;
    }

    return;
}

static void DOFIMGLIB_process(DOFIMGLIB_Handle handle, uint8_t* data, uint64_t timestamp)
{
    DOFIMGLIB_Context *appCntxt = (DOFIMGLIB_Context *)handle;

    strcpy(appCntxt->imgFileName, (char*)data);

    appCntxt->bInit = 1;

    return;
}

static RADARLIB_Handle RADARLIB_create()
{
    RADARLIB_Context *appCntxt;
    RADARLIB_Handle   handle;

    handle = (RADARLIB_Handle)malloc(sizeof(RADARLIB_Context));

    if (handle == NULL)
    {
        PTK_printf("[%s:%d] Could not allocate memory for internal context.\n",
                   __FUNCTION__, __LINE__);

        return NULL;
    }

    appCntxt = (RADARLIB_Context*) handle;
    appCntxt->data = (uint8_t*) calloc(8004, sizeof(uint8_t)); // 400 x sizeof(RADAR OBJECT) + 4 bytes

    if (appCntxt->data == NULL)
    {
        PTK_printf("[%s:%d] Memory allocation failed.\n",
                   __FUNCTION__, __LINE__);

        free(handle);
        handle = NULL;
    }

    return handle;
}

static void RADARLIB_delete(RADARLIB_Handle *handle)
{
    if (*handle)
    {
        RADARLIB_Context *appCntxt = (RADARLIB_Context*) *handle;
        free(appCntxt->data);

        free(*handle);
        *handle = NULL;
    }

    return;
}

static void RADARLIB_process(RADARLIB_Handle handle, size_t size, uint8_t* data)
{
    RADARLIB_Context *appCntxt = (RADARLIB_Context *)handle;
    memcpy(appCntxt->data, data, size);

    return;
}

static void VALETAPP_appShowUsage(char* argv[])
{
    PTK_printf("\n");
    PTK_printf(" Valet Parking Demo - (c) Texas Instruments 2018\n");
    PTK_printf(" ========================================================\n");
    PTK_printf("\n");
    PTK_printf("Please refer to demo guide for prerequisites before running this demo\n");
    PTK_printf("\n");
    PTK_printf(" Usage,\n");
    PTK_printf("  %s --cfg <config file>\n", argv[0]);
    PTK_printf("\n");

    return;

} /* VALETAPP_appShowUsage */

static void VALETAPP_printStats(VALETAPP_Context   *appCntxt,
                                uint32_t            id)
{
    VALETAPP_Stats *stats;
    char           *str;

    if (id == VALETAPP_STATS_RADAR_ID)
    {
        str = "RADAR FRAME STATS:";
    }
    else if (id == VALETAPP_STATS_LIDAR_ID)
    {
        str = "LIDAR FRAME STATS:";
    }
    else if (id == VALETAPP_STATS_SFM_ID)
    {
        str = "SFM FRAME STATS:";
    }
    else
    {
        return;
    }

    stats = &appCntxt->stats[id];
    PTK_printf("\n%s\n", str);
    PTK_printf("\tTotal Frame Count     = %d\n", stats->totalFrameCount);
    PTK_printf("\tProcessed Frame Count = %d\n", stats->framesProcessed);
    PTK_printf("\tDropped Frame Count   = %d\n", stats->droppedFrameCnt);
}

static void VALETAPP_parseCfgFile(VALETAPP_Context *appCntxt, char *cfgFileName)
{
    FILE                       *fptr;
    char                       *pParamStr;
    char                       *pValueStr;
    char                       *pSLine;
    char                       *globalBasePath;
    char                       *localBasePath;
    char                       *filePath;
    SFMOGAPPLIB_createParams   *cameraCreateParams;
    RADAROGAPPLIB_createParams *radarCreateParams;
    LIDAROGAPPLIB_CreateParams *lidarCreateParams;
    FUSEDOGAPPLIB_createParams *fusedOgCreateParams;
    PTK_Alg_FsdPfsdParams      *fsdPfsdConfig;
    PSLIB_createParams         *psCreateParams;
    PTK_LidarGatingParams      *lidarGatingParams;
    char                        paramSt[VALETAPP_MAX_LINE_LEN];
    char                        valueSt[VALETAPP_MAX_LINE_LEN];
    char                        sLine[VALETAPP_MAX_LINE_LEN];
    char                        localBasePathArr[VALETAPP_MAX_LINE_LEN] = {0};
    char                        filePathArr[VALETAPP_MAX_LINE_LEN];
    int32_t                     pos;

    /* initialize defaults */
    PTK_GridRoi_setDefault(&appCntxt->roiParams);

    pParamStr                = paramSt;
    pValueStr                = valueSt;
    pSLine                   = sLine;
    localBasePath            = localBasePathArr;
    filePath                 = filePathArr;
    globalBasePath           = getenv("APP_CONFIG_BASE_PATH");
    cameraCreateParams       = &appCntxt->cameraCreateParams;
    radarCreateParams        = &appCntxt->radarCreateParams;
    lidarCreateParams        = &appCntxt->lidarCreateParams;
    fusedOgCreateParams      = &appCntxt->fusedOgCreateParams;
    psCreateParams           = &appCntxt->psCreateParams;
    fsdPfsdConfig            = &appCntxt->fsdPfsdConfig;
    lidarGatingParams        = &lidarCreateParams->gatingParams;

    /* get local base path (directory of cfgFileName) */
    pos = ptkdemo_find_slash(cfgFileName, VALETAPP_MAX_LINE_LEN);

    if (pos >= 0)
    {
        strncpy(localBasePath, cfgFileName, pos + 1);
    }
    else
    {
        strcpy(localBasePath, "./");
    }

    fptr = fopen(cfgFileName, "r");

    if ( !fptr )
    {
        PTK_printf("Cannot open %s for reading.\n", cfgFileName);
        exit(-1);
    }

    /* set d2u table and projection matrix */
    psCreateParams->d2uLUT         = ValetApp_ldcLUT_D2U_table;
    psCreateParams->d2uLUTSize     = sizeof(ValetApp_ldcLUT_D2U_table) / sizeof(float);
    psCreateParams->projMatrix     = ValetApp_projection_matrix;
    psCreateParams->projMatrixSize = sizeof(ValetApp_projection_matrix) / sizeof(double);


    /* Set the default render periodicity to 50 milli-sec. */
    appCntxt->renderPeriod       = 50;
    appCntxt->inputRateControl   = false;
    appCntxt->sensorRateControl  = false;
    appCntxt->winWidth           = 1920;
    appCntxt->winHeight          = 1080;
    appCntxt->radarPipelineDepth = RADAROGAPPLIB_PIPELINE_DEPTH;
    appCntxt->lidarPipelineDepth = LIDAROGAPPLIB_PIPELINE_DEPTH;
    appCntxt->sfmPipelineDepth   = SFMOGAPPLIB_PIPELINE_DEPTH;
    appCntxt->rtLogEnable        = 0;

    memset(&cameraCreateParams->dtNodeCfg, 0,
           sizeof(tivx_dof_to_tracks_params_t));

    memset(&cameraCreateParams->trNodeCfg, 0,
           sizeof(tivx_triangulation_params_t));

    /* Default setting for DOF generation. */
    cameraCreateParams->generatePC                = 1;
    cameraCreateParams->enableDof                 = false;
    cameraCreateParams->maxNumTracks              = 20000;
    cameraCreateParams->dtNodeCfg.dofConfidThresh = 9;

    lidarGatingParams->valid = 0;

    while ( 1 )
    {
        pSLine = fgets(pSLine, VALETAPP_MAX_LINE_LEN, fptr);

        if ( pSLine == NULL )
        {
            break;
        }

        if (strchr(pSLine, '#'))
        {
            continue;
        }

        pParamStr[0] = '\0';
        pValueStr[0] = '\0';

        sscanf(pSLine,"%128s %128s", pParamStr, pValueStr);

        if (pParamStr[0] == '\0' || pValueStr[0] == '\0')
        {
            continue;
        }

        if (strcmp(pParamStr, "dump_output_map") == 0)
        {
            appCntxt->dumpMap = strtol(pValueStr, NULL, 0);
        }
        else if (strcmp(pParamStr, "visualize") == 0)
        {
            appCntxt->visualize = strtol(pValueStr, NULL, 0);
        }
        else if (strcmp(pParamStr, "verbose") == 0)
        {
            appCntxt->verbose = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "render_periodicity") == 0)
        {
            appCntxt->renderPeriod =
                static_cast<uint64_t>(atof(pValueStr) * 1000);
        }
        else if (strcmp(pParamStr, "winWidth") == 0)
        {
            appCntxt->winWidth = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "winHeight") == 0)
        {
            appCntxt->winHeight = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "max_num_tracks") == 0)
        {
            cameraCreateParams->maxNumTracks = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "dof_confidence_threshold") == 0)
        {
            cameraCreateParams->dtNodeCfg.dofConfidThresh = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "radar_fsd_enable") == 0)
        {
            radarCreateParams->ogPfsdCfg.fsdEnable = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "lidar_fsd_enable") == 0)
        {
            lidarCreateParams->ogPfsdCfg.fsdEnable = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "camera_fsd_enable") == 0)
        {
            cameraCreateParams->ogPfsdCfg.fsdEnable = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "ldc_input_width") == 0)
        {
        	cameraCreateParams->dofWidth = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "ldc_input_height") == 0)
        {
        	cameraCreateParams->dofHeight = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "generate_point_cloud") == 0)
        {
        	cameraCreateParams->generatePC = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "dof_enable") == 0)
        {
            cameraCreateParams->enableDof = (bool)atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "fusion_fsd_enable") == 0)
        {
            fusedOgCreateParams->ogPfsdCfg.fsdEnable = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "is_interactive") == 0)
        {
            appCntxt->is_interactive = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "sensor_rate_control") == 0)
        {
            appCntxt->sensorRateControl = (bool)atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "input_rate_control") == 0)
        {
            appCntxt->inputRateControl = (bool)atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "radar_pipeline_depth") == 0)
        {
            appCntxt->radarPipelineDepth = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "lidar_pipeline_depth") == 0)
        {
            appCntxt->lidarPipelineDepth = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "sfm_pipeline_depth") == 0)
        {
            appCntxt->sfmPipelineDepth = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "rtLogEnable") == 0)
        {
            appCntxt->rtLogEnable = (uint8_t)atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "roi_cfg_file") == 0)
        {
            ptkdemo_get_file_path(&filePath, pValueStr, globalBasePath,
                                  localBasePath, VALETAPP_MAX_LINE_LEN);
            PTK_Util_parseRoiConfig(filePath, &appCntxt->roiParams);
        }
        else if (strcmp(pParamStr, "dash_font_path") == 0)
        {
            snprintf(appCntxt->fontFile, VALETAPP_MAX_LINE_LEN,
                     "%s", pValueStr);
        }
        else if (strcmp(pParamStr, "dash_logo_path") == 0)
        {
            ptkdemo_get_file_path(&filePath, pValueStr, globalBasePath,
                                  localBasePath, VALETAPP_MAX_LINE_LEN);
            strcpy(appCntxt->logoPath, filePath);
        }
        else if (strcmp(pParamStr, "dash_bkgrnd_path") == 0)
        {
            ptkdemo_get_file_path(&filePath, pValueStr, globalBasePath,
                                  localBasePath, VALETAPP_MAX_LINE_LEN);
            strcpy(appCntxt->bkgrndPath, filePath);
        }
        else if (strcmp(pParamStr, "tiap_cfg_file") == 0)
        {
            ptkdemo_get_file_path(&filePath, pValueStr, globalBasePath,
                                  localBasePath, VALETAPP_MAX_LINE_LEN);
            PTK_DBConfig_parse(&appCntxt->dbConfig, filePath);
        }
        else if (strcmp(pParamStr, "grid_cfg_file") == 0)
        {
            ptkdemo_get_file_path(&filePath, pValueStr, globalBasePath,
                                  localBasePath, VALETAPP_MAX_LINE_LEN);
            PTK_Util_parseGridConfig(filePath, &appCntxt->gridConfig);
        }
        else if (strcmp(pParamStr, "radar_ogmap_cfg_file") == 0)
        {
            ptkdemo_get_file_path(&filePath, pValueStr, globalBasePath,
                                  localBasePath, VALETAPP_MAX_LINE_LEN);
            PTK_Util_RadarOgmapParseConfig(filePath,
                     &radarCreateParams->ogPfsdCfg.ogConfig);
        }
        else if (strcmp(pParamStr, "lidar_ogmap_cfg_file") == 0)
        {
            ptkdemo_get_file_path(&filePath, pValueStr, globalBasePath,
                                  localBasePath, VALETAPP_MAX_LINE_LEN);
            PTK_Util_LidarOgmapParseConfig(filePath,
                     &lidarCreateParams->ogPfsdCfg.ogConfig);

            /* We decided to add the gating parameters to the current OGMAP
             * config file to avoid adding additional config files. The
             * OGMAP config file will be parsed twice but it is OK since
             * we do this at initialization time.
             */
            PTK_Util_LidarGatingParseConfig(filePath, lidarGatingParams);
        }
        else if (strcmp(pParamStr, "sfm_ogmap_cfg_file") == 0)
        {
            ptkdemo_get_file_path(&filePath, pValueStr, globalBasePath,
                                  localBasePath, VALETAPP_MAX_LINE_LEN);
            PTK_Util_SfmOgmapParseConfig(filePath,
                     &cameraCreateParams->ogPfsdCfg.ogConfig);
        }
        else if (strcmp(pParamStr, "fsd_pfsd_cfg_file") == 0)
        {
            ptkdemo_get_file_path(&filePath, pValueStr, globalBasePath,
                                  localBasePath, VALETAPP_MAX_LINE_LEN);

            PTK_Util_FsdPfsdParseConfig(filePath, fsdPfsdConfig);
        }
        else if  (strcmp(pParamStr, "fused_ogmap_cfg_file") == 0)
        {
            ptkdemo_get_file_path(&filePath, pValueStr, globalBasePath,
                                  localBasePath, VALETAPP_MAX_LINE_LEN);
            PTK_Util_FusedOgmapParseConfig(filePath,
                    &fusedOgCreateParams->ogPfsdCfg.ogConfig);
        }
        else if  (strcmp(pParamStr, "fusion_enable") == 0)
        {
            appCntxt->fusionEnabled = atoi(pValueStr);
        }
        else if  (strcmp(pParamStr, "fusionMethod") == 0)
        {
            appCntxt->fusionMethod = atoi(pValueStr);
        }

        else if (strcmp(pParamStr, "radar_og_deploy_core") == 0)
        {
            radarCreateParams->ogNodeCore =
                app_common_get_coreName(pValueStr);
        }
        else if (strcmp(pParamStr, "lidar_og_deploy_core") == 0)
        {
            lidarCreateParams->ogNodeCore =
                app_common_get_coreName(pValueStr);
        }
        else if (strcmp(pParamStr, "lidar_gpc_deploy_core") == 0)
        {
            lidarCreateParams->gpcNodeCore =
                app_common_get_coreName(pValueStr);
        }
        else if (strcmp(pParamStr, "lidar_mdc_deploy_core") == 0)
        {
            lidarCreateParams->mdcNodeCore =
                app_common_get_coreName(pValueStr);
        }
        else if (strcmp(pParamStr, "sfm_og_deploy_core") == 0)
        {
            cameraCreateParams->ogNodeCore =
                app_common_get_coreName(pValueStr);
        }
        else if (strcmp(pParamStr, "sfm_tri_deploy_core") == 0)
        {
            cameraCreateParams->triNodeCore =
                app_common_get_coreName(pValueStr);
        }
        else if (strcmp(pParamStr, "sfm_dof_track_deploy_core") == 0)
        {
            cameraCreateParams->dofTrackNodeCore =
                app_common_get_coreName(pValueStr);
        }

        else if (strcmp(pParamStr, "fused_og_deploy_core") == 0)
        {
            fusedOgCreateParams->fusionNodeCore =
                app_common_get_coreName(pValueStr);
        }
        else if (strcmp(pParamStr, "ps_map_deploy_core") == 0)
        {
            psCreateParams->psNodeCore =
                app_common_get_coreName(pValueStr);
        }
    }

    fclose(fptr);

    return;

} /* VALETAPP_parseCfgFile */

static void VALETAPP_getExtrinsicCalibration(ssHandle               cam,
                                             PTK_RigidTransform   * M_c_e,
                                             PTK_RigidTransform   * M_e_g,
                                             uint32_t               verbose)
{
    /* M_e_c */
    float extCalib[12];

    //camera
    sensorstream_get_extrinsic_calibration(cam, extCalib);
    PTK_RigidTransform_setRotation(M_c_e, extCalib);
    PTK_RigidTransform_setTranslation(M_c_e, extCalib[9], extCalib[10], extCalib[11]);
    if (verbose)
    {
        PTK_printf("\nCamera Extrinsic Calibration (M_c_e): \n");
        PTK_RigidTransform_print(M_c_e);
    }

    /* M_e_g */
    uint32_t readFromDB = 0;
    if (readFromDB)
    {
        //@todo make DB API
    }
    else //@todo remove once readFromDB is enabled
    {
        FILE    *fp;
        char     calmatPath[VALETAPP_MAX_LINE_LEN];
        uint8_t  calmatheader[128];
        int32_t  calmat[4*12];
        uint32_t camId = 1; //position in calmat (0-front, 1-right, 2-rear, 3-left)
        float    scaleFactor = 0.001f; //calmat is in mm
        PTK_RigidTransform M_c_g;
        size_t   size;

        sensorstream_get_full_dir_path(cam, calmatPath);
        strcat(calmatPath, "/../calibration_data/extrinsic_calibration_srv/CALMAT.BIN");

        fp = fopen(calmatPath, "rb");
        assert(NULL!=fp);

        size = fread((void *)calmatheader, sizeof(uint8_t),128, fp);
        size = fread((void *)calmat, sizeof(int32_t), 4*12, fp);

        (void)size;

        PTK_Util_GroundToCamera(calmat, camId, &M_c_g);

        M_c_g.M[3] = M_c_g.M[3] * scaleFactor;
        M_c_g.M[7] = M_c_g.M[7] * scaleFactor;
        M_c_g.M[11] = M_c_g.M[11] * scaleFactor;

        PTK_RigidTransform M_e_c;
        PTK_RigidTransform_invert(&M_e_c, M_c_e);
        PTK_RigidTransform_compose(M_e_g, &M_e_c, &M_c_g);

        if (verbose)
        {
            PTK_printf("\nGround Transform (M_e_g): \n");
            PTK_RigidTransform_print(M_e_g);
        }

        fclose(fp);
    }
} /* SFMOGAPP_get_ground_calibration */

static void VALETAPP_getExtrinsicCalibrationPsd(ssHandle psd, PSLIB_createParams *prm, uint32_t verbose)
{
    /* M_e_c */
    float extCalib[12];

    //camera
    sensorstream_get_extrinsic_calibration(psd, extCalib);
    PTK_RigidTransform_setRotation(&prm->M_c_e, extCalib);
    PTK_RigidTransform_setTranslation(&prm->M_c_e, extCalib[9], extCalib[10], extCalib[11]);
    if (verbose)
    {
        PTK_printf("\nCamera Extrinsic Calibration (M_c_e): \n");
        PTK_RigidTransform_print(&prm->M_c_e);
    }

    /* M_e_g */
    uint32_t readFromDB = 0;
    if (readFromDB)
    {
        //@todo make DB API
    }
    else //@todo remove once readFromDB is enabled
    {
        FILE    *fp;
        char     calmatPath[VALETAPP_MAX_LINE_LEN];
        uint8_t  calmatheader[128];
        int32_t  calmat[4*12];
        uint32_t camId = 1; //position in calmat (0-front, 1-right, 2-rear, 3-left)
        float    scaleFactor = 0.001f; //calmat is in mm
        size_t   size;

        sensorstream_get_full_dir_path(psd, calmatPath);
        strcat(calmatPath, "/../calibration_data/extrinsic_calibration_srv/CALMAT.BIN");

        fp = fopen(calmatPath, "rb");
        assert(NULL!=fp);

        size = fread((void *)calmatheader, sizeof(uint8_t),128, fp);
        size = fread((void *)calmat, sizeof(int32_t), 4*12, fp);

        (void)size;

        PTK_RigidTransform M_c_g;

        PTK_Util_GroundToCamera(calmat, camId, &M_c_g);

        M_c_g.M[3] = M_c_g.M[3] * scaleFactor;
        M_c_g.M[7] = M_c_g.M[7] * scaleFactor;
        M_c_g.M[11] = M_c_g.M[11] * scaleFactor;

        PTK_RigidTransform_invert(&prm->M_g_c, &M_c_g);

        PTK_RigidTransform M_e_c;
        PTK_RigidTransform_invert(&M_e_c, &prm->M_c_e);
        PTK_RigidTransform_compose(&prm->M_e_g, &M_e_c, &M_c_g);

        if (verbose)
        {
            PTK_printf("\nGround Transform (M_e_g): \n");
            PTK_RigidTransform_print(&prm->M_e_g);
        }

        fclose(fp);
    }
} /* SFMOGAPP_get_ground_calibration */

static void VALETAPP_appLibsInit(VALETAPP_Context *appCntxt)
{
    PTK_Alg_FsdPfsdParams  *fsdPfsdConfig;
    uint32_t                sensorEnableMask;

    /* Set the parameters common across all the sensor processing. */
    fsdPfsdConfig               = &appCntxt->fsdPfsdConfig;
    fsdPfsdConfig->gridConfig   = appCntxt->gridConfig;
    fsdPfsdConfig->ogFlagFst    = VALETAPP_FLAG_FST;
    fsdPfsdConfig->ogFlagFsd    = VALETAPP_FLAG_FSD;
    fsdPfsdConfig->ogFlagPfsd   = VALETAPP_FLAG_PFSD;
    fsdPfsdConfig->newFSDCycle  = 0;
    sensorEnableMask            = 0;

    appCntxt->perfOutFile = "apps_valet_parking";

    /* Radar configuration. */
    if (appCntxt->radarEnabled)
    {
        RADAROGAPPLIB_createParams   *raParams;
        PTK_Alg_RadarOgmapParams     *ogConfig;

        raParams                      = &appCntxt->radarCreateParams;
        ogConfig                      = &raParams->ogPfsdCfg.ogConfig;
        fsdPfsdConfig                 = &raParams->ogPfsdCfg.fsdPfsdConfig;

        raParams->pipelineDepth       = appCntxt->radarPipelineDepth;
        raParams->vxEvtAppValBase     = VALETAPP_RADAR_OG_GRAPH_EVENT_BASE;
        raParams->ogFlagEgo           = VALETAPP_FLAG_EGO;
        ogConfig->gridConfig          = appCntxt->gridConfig;
        ogConfig->accGridId           = VALETAPP_GRID_ID_RADAR_ACCUMULATED;
        ogConfig->instOccGridId       = VALETAPP_GRID_ID_RADAR_INST_OCC;
        ogConfig->instDsGridId        = VALETAPP_GRID_ID_RADAR_INST_DS;
        ogConfig->ogFlagOccupied      = VALETAPP_FLAG_OCCUPIED;
        ogConfig->ogFlagFree          = VALETAPP_FLAG_GROUND;
        ogConfig->ogFlagChanged       = VALETAPP_FLAG_INST_CHANGED;

        *fsdPfsdConfig                = appCntxt->fsdPfsdConfig;
        fsdPfsdConfig->numBoxes       = fsdPfsdConfig->numBoxes_ogMap_2;
        fsdPfsdConfig->occGridId      = ogConfig->accGridId;
        fsdPfsdConfig->ogFlagOccupied = ogConfig->ogFlagOccupied;
        fsdPfsdConfig->ogFlagFree     = ogConfig->ogFlagFree;

        /* Always set to '1' since renderer shows binarized maps. */
        ogConfig->binarize = 1;

        raParams->vxContext   = appCntxt->vxContext;
        raParams->rtLogEnable = appCntxt->rtLogEnable;

        appCntxt->radarHdl = RADAROGAPPLIB_create(raParams);
        PTK_assert(appCntxt->radarHdl != NULL);

        appCntxt->radarMapSize =
                  RADAROGAPPLIB_getOutAccMapSize(appCntxt->radarHdl);
        appCntxt->radarSensorMask = ogConfig->sensorMask;
        appCntxt->numRadars       = PTK_Util_bitCnt(ogConfig->sensorMask);

        sensorEnableMask |= FUSEDOGAPPLIB_INPUT_MASK_RADAR;

        // Radar Renderer
        appCntxt->radarPCHdl = RADARLIB_create();

        if (ogConfig->mappingMethod ==
                PTK_ALG_RADAR_OGMAP_MAPPING_METHOD_DS_ONLY)
        {
            appCntxt->perfOutFile = "apps_valet_parking_ds";
        }

    }

    /* Lidar configuration. */
    if (appCntxt->lidarEnabled)
    {
        LIDAROGAPPLIB_CreateParams *liParams;
        PTK_Alg_LidarOgmapParams   *ogConfig;

        liParams                      = &appCntxt->lidarCreateParams;
        ogConfig                      = &liParams->ogPfsdCfg.ogConfig;
        fsdPfsdConfig                 = &liParams->ogPfsdCfg.fsdPfsdConfig;

        liParams->pipelineDepth       = appCntxt->lidarPipelineDepth;
        liParams->vxEvtAppValBase     = VALETAPP_LIDAR_OG_GRAPH_EVENT_BASE;
        liParams->ogFlagEgo           = VALETAPP_FLAG_EGO;
        ogConfig->gridConfig          = appCntxt->gridConfig;
        ogConfig->accGridId           = VALETAPP_GRID_ID_LIDAR_ACCUMULATED;
        ogConfig->instOccGridId       = VALETAPP_GRID_ID_LIDAR_INST_OCC;
        ogConfig->instDsGridId        = VALETAPP_GRID_ID_LIDAR_INST_DS;
        ogConfig->tagPcRemoved        = VALETAPP_POINT_TAG_REMOVED;
        ogConfig->tagPcGround         = VALETAPP_POINT_TAG_GROUND;
        ogConfig->tagOgGround         = VALETAPP_FLAG_GROUND;
        ogConfig->tagOgObstacle       = VALETAPP_FLAG_OCCUPIED;
        ogConfig->ogFlagChanged       = VALETAPP_FLAG_INST_CHANGED;

        *fsdPfsdConfig                = appCntxt->fsdPfsdConfig;
        fsdPfsdConfig->numBoxes       = fsdPfsdConfig->numBoxes_ogMap_3;
        fsdPfsdConfig->occGridId      = ogConfig->accGridId;
        fsdPfsdConfig->ogFlagOccupied = ogConfig->tagOgObstacle;
        fsdPfsdConfig->ogFlagFree     = ogConfig->tagOgGround;

        liParams->vxContext   = appCntxt->vxContext;
        liParams->rtLogEnable = appCntxt->rtLogEnable;

        appCntxt->lidarHdl = LIDAROGAPPLIB_create(liParams);
        PTK_assert(appCntxt->lidarHdl != NULL);

        appCntxt->lidarMapSize =
            LIDAROGAPPLIB_getOutAccMapSize(appCntxt->lidarHdl);

        sensorEnableMask |= FUSEDOGAPPLIB_INPUT_MASK_LIDAR;

        if (ogConfig->mappingMethod ==
                PTK_ALG_LIDAR_OGMAP_MAPPING_METHOD_CNT_AND_DS)
        {
            appCntxt->perfOutFile = "apps_valet_parking_ds";
        }
    }

    /* Camera configuration. */
    if (appCntxt->cameraEnabled)
    {
        SFMOGAPPLIB_createParams    *camParams;
        PTK_Alg_SfmOgmapParams      *ogConfig;
        tivx_dmpac_dof_params_t     *dofCfg;
        tivx_dof_to_tracks_params_t *dtNodeCfg;
        tivx_triangulation_params_t *trNodeCfg;
        ssHandle                     cam;
        int32_t                      i;

        camParams     = &appCntxt->cameraCreateParams;
        ogConfig      = &camParams->ogPfsdCfg.ogConfig;
        fsdPfsdConfig = &camParams->ogPfsdCfg.fsdPfsdConfig;
        dofCfg        = &camParams->dofCfg;
        dtNodeCfg     = &camParams->dtNodeCfg;
        trNodeCfg     = &camParams->trNodeCfg;

        camParams->pipelineDepth   = appCntxt->sfmPipelineDepth;
        camParams->vxEvtAppValBase = VALETAPP_SFM_OG_GRAPH_EVENT_BASE;

        if (camParams->generatePC == 1)
        {
            if (camParams->enableDof)
            {
                /* DOF node config */
                tivx_dmpac_dof_params_init(dofCfg);
    
                dofCfg->vertical_search_range[0] = 48;
                dofCfg->vertical_search_range[1] = 48;
                dofCfg->horizontal_search_range  = 191;
                dofCfg->median_filter_enable     = 1;
                dofCfg->motion_smoothness_factor = 24;
                dofCfg->motion_direction         = 0; /* 0: for side camera, neutral */
            } 
            
            camParams->dofLevels = 5;

            /* dof2tracks node config */

            dtNodeCfg->roiType         = TIVX_DOF_TO_TRACKS_ROI_VERTICAL_CONE;
            dtNodeCfg->roiPrm1         = -4.f;
            dtNodeCfg->roiPrm2         = 648.f;
            dtNodeCfg->subsampleFactor = 2;
            dtNodeCfg->focalLength     = 311.8333f;
            dtNodeCfg->principalPointX = 367.f; //640-1-tx
            dtNodeCfg->principalPointY = 479.f; //360-1-ty
            dtNodeCfg->d2uTableStep    = 0.f; //not used

            /* triangulation node config */
            trNodeCfg->maxIters = 5;
            trNodeCfg->enableHighPrecision = 1;
        }

        /*general map config*/
        ogConfig->method                = 0;
        ogConfig->gridConfig            = appCntxt->gridConfig;

        ogConfig->accGridId             = VALETAPP_GRID_ID_SFM_ACCUMULATED;
        ogConfig->instOccGridId         = VALETAPP_GRID_ID_SFM_INST_OCC;
        ogConfig->instDsGridId          = VALETAPP_GRID_ID_SFM_INST_DS;
        ogConfig->ogFlagFree            = VALETAPP_FLAG_GROUND;
        ogConfig->ogFlagOccupied        = VALETAPP_FLAG_OCCUPIED;
        ogConfig->ogFlagChanged         = VALETAPP_FLAG_INST_CHANGED;

        *fsdPfsdConfig                  = appCntxt->fsdPfsdConfig;
        fsdPfsdConfig->numBoxes         = fsdPfsdConfig->numBoxes_ogMap_1;
        fsdPfsdConfig->occGridId        = ogConfig->accGridId;
        fsdPfsdConfig->ogFlagOccupied   = ogConfig->ogFlagOccupied;
        fsdPfsdConfig->ogFlagFree       = ogConfig->ogFlagFree;

        camParams->vxContext   = appCntxt->vxContext;
        camParams->rtLogEnable = appCntxt->rtLogEnable;

        i = PTK_DBConfig_exist_sensor_apptag(&appCntxt->dbConfig, "CAM_R");
        PTK_assert(i >= 0);

        cam = SensorDataPlayerINS_get_sensorstream(appCntxt->dataPlayer,
                                                   VALETAPP_SENSOR_CAMERA_R);
        PTK_assert(cam != NULL);

        /* This populates M_e_g and M_c_e fields. */
        VALETAPP_getExtrinsicCalibration(cam,
                                         &camParams->M_c_e,
                                         &ogConfig->M_e_g,
                                         appCntxt->verbose);

        appCntxt->cameraHdl = SFMOGAPPLIB_create(&appCntxt->cameraCreateParams);
        PTK_assert(appCntxt->cameraHdl != NULL);

        appCntxt->cameraMapSize =
            SFMOGAPPLIB_getOutAccMapSize(appCntxt->cameraHdl);

        sensorEnableMask |= FUSEDOGAPPLIB_INPUT_MASK_SFM;
    }

    /* initialize parking spot lib */
    if (appCntxt->tidlPsdEnabled == 1)
    {
        appCntxt->psCreateParams.gridConfig =
                  appCntxt->gridConfig;

        appCntxt->psCreateParams.occupancyGridId =
                  VALETAPP_GRID_ID_TIDL_PSD_OCCUPANCY;

        appCntxt->psCreateParams.ogFlagFree =
                  VALETAPP_FLAG_TIDL_PSD_FREE;

        appCntxt->psCreateParams.ogFlagOccupied =
                  VALETAPP_FLAG_TIDL_PSD_OCCUPIED;

        // parking spot configs
        appCntxt->psCreateParams.psMappingAlgoParams.maxSpotsFrame = 50;
        appCntxt->psCreateParams.psMappingAlgoParams.maxTotalSpots = 300;
        appCntxt->psCreateParams.psMappingAlgoParams.numSpots = 0;
        appCntxt->psCreateParams.psMappingAlgoParams.minProb = 0.9;
        appCntxt->psCreateParams.psMappingAlgoParams.minHeight = 4.0;
        appCntxt->psCreateParams.psMappingAlgoParams.maxHeight = 6.5;
        appCntxt->psCreateParams.psMappingAlgoParams.minWidth = 1.2;
        appCntxt->psCreateParams.psMappingAlgoParams.maxWidth = 3.5;
        appCntxt->psCreateParams.psMappingAlgoParams.matchTh = 0.5;
        appCntxt->psCreateParams.psMappingAlgoParams.minCountPerPS = 4;

        appCntxt->psCreateParams.psMappingAlgoParams.minImPosX = 300;
        appCntxt->psCreateParams.psMappingAlgoParams.maxImPosX = 960;
        appCntxt->psCreateParams.psMappingAlgoParams.minImPosY = 200;
        appCntxt->psCreateParams.psMappingAlgoParams.maxImPosY = 540;

        appCntxt->psCreateParams.vxContext = appCntxt->vxContext;

        // copy transform matrix from SFMOGAPPLIB_createParams.
        // may need to calculate independently
        int32_t i = PTK_DBConfig_exist_sensor_apptag(&appCntxt->dbConfig,
                                                     "CAM_PSD");
        PTK_assert(i >= 0);

        ssHandle psd =
            SensorDataPlayerINS_get_sensorstream(appCntxt->dataPlayer,
                                                 VALETAPP_SENSOR_PSD);
        PTK_assert(psd != NULL);

        VALETAPP_getExtrinsicCalibrationPsd(psd,
                                            &appCntxt->psCreateParams,
                                            appCntxt->verbose);

        PTK_RigidTransform_invert(&appCntxt->psCreateParams.M_e_c,
                                  &appCntxt->psCreateParams.M_c_e);

        appCntxt->psHdl = PSLIB_create(&appCntxt->psCreateParams);
        PTK_assert(appCntxt->psHdl != NULL);

        appCntxt->tidlPsdMapSize = PSLIB_getOutMapSize(appCntxt->psHdl);

        sensorEnableMask |= FUSEDOGAPPLIB_INPUT_MASK_PSD;
    }

    if (appCntxt->tidlPsdImgEnabled == 1)
    {
        int32_t i = PTK_DBConfig_exist_sensor_apptag(&appCntxt->dbConfig,
                                                     "CAM_PSD_IMG");
        PTK_assert(i >= 0);

        appCntxt->psImgHdl = PSIMGLIB_create();
        PTK_assert(appCntxt->psImgHdl != NULL);

        sensorEnableMask |= FUSEDOGAPPLIB_INPUT_MASK_PSD_IMG;
    }

    // DoF Image
    if (appCntxt->dofImgEnabled == 1)
    {
        int32_t i = PTK_DBConfig_exist_sensor_apptag(&appCntxt->dbConfig,
                                                     "CAM_DOF_IMG");
        PTK_assert(i >= 0);

        appCntxt->dofImgHdl = DOFIMGLIB_create();
        PTK_assert(appCntxt->dofImgHdl != NULL);

        sensorEnableMask |= FUSEDOGAPPLIB_INPUT_MASK_DOF_IMG;
    }

    if (!sensorEnableMask)
    {
        PTK_assert(0);
    }

    /* Fusion node setup.
     * We need to setup parameters for the fusion node since the fusion APPLIB
     * needs to use basic information for setup. However, conditionally create
     * the fusion node.
     */
    if (appCntxt->fusionEnabled)
    {
        FUSEDOGAPPLIB_createParams *fusionParams;
        PTK_Alg_FusedOgmapParams   *ogConfig;

        fusionParams                    = &appCntxt->fusedOgCreateParams;
        ogConfig                        = &fusionParams->ogPfsdCfg.ogConfig;
        fsdPfsdConfig                   = &fusionParams->ogPfsdCfg.fsdPfsdConfig;

        ogConfig->gridConfig            = appCntxt->gridConfig;
        ogConfig->roiParams             = appCntxt->roiParams;
        ogConfig->sensorEnableMask      = sensorEnableMask;
        ogConfig->outGridId             = VALETAPP_GRID_ID_FUSED_OCCUPANCY;
        ogConfig->ogFlagOccupied        = VALETAPP_FLAG_OCCUPIED;
        ogConfig->ogFlagFree            = VALETAPP_FLAG_GROUND;
        ogConfig->fusedGridId           = VALETAPP_GRID_ID_FUSED_DS;
        ogConfig->fusionMethod          = appCntxt->fusionMethod;
        ogConfig->ogFlagChanged         = VALETAPP_FLAG_INST_CHANGED;

        *fsdPfsdConfig                  = appCntxt->fsdPfsdConfig;
        fsdPfsdConfig->numBoxes         = fsdPfsdConfig->numBoxes_ogMap_4;
        fsdPfsdConfig->occGridId        = ogConfig->outGridId;
        fsdPfsdConfig->ogFlagOccupied   = ogConfig->ogFlagOccupied;
        fsdPfsdConfig->ogFlagFree       = ogConfig->ogFlagFree;

        switch(appCntxt->fusionMethod)
        {
            case PTK_ALG_FUSED_OGMAP_METHOD_VOTING:
                ogConfig->radarGridId  = VALETAPP_GRID_ID_RADAR_INST_OCC;
                ogConfig->lidarGridId  = VALETAPP_GRID_ID_LIDAR_INST_OCC;
                ogConfig->cameraGridId = VALETAPP_GRID_ID_SFM_INST_OCC;
                break;

            case PTK_ALG_FUSED_OGMAP_METHOD_DS:
                ogConfig->radarGridId  = VALETAPP_GRID_ID_RADAR_INST_DS;
                ogConfig->lidarGridId  = VALETAPP_GRID_ID_LIDAR_INST_DS;
                ogConfig->cameraGridId = VALETAPP_GRID_ID_SFM_INST_DS;
                break;

            default:
                PTK_printf("[%s:%d] Invalid fusion method %d\n",
                           __FUNCTION__,
                           __LINE__,
                           appCntxt->fusionMethod);
                return;
        }

        fusionParams->vxContext        = appCntxt->vxContext;
        fusionParams->sensorEnableMask = sensorEnableMask;
        fusionParams->tidlPsdEnable    = appCntxt->tidlPsdEnabled;
        fusionParams->tidlPsdGridId    = VALETAPP_GRID_ID_TIDL_PSD_OCCUPANCY;
        fusionParams->rtLogEnable      = appCntxt->rtLogEnable;

        appCntxt->fusedOgHdl = FUSEDOGAPPLIB_create(fusionParams);
        PTK_assert(appCntxt->fusedOgHdl != NULL);

        appCntxt->fusedMapSize =
            FUSEDOGAPPLIB_getOutMapSize(appCntxt->fusedOgHdl);
    }
}

static void VALETAPP_appLibsDeInit(VALETAPP_Context *appCntxt)
{
    if (appCntxt->cameraHdl != NULL)
    {
        /* Release the Radar module context. */
        SFMOGAPPLIB_delete(&appCntxt->cameraHdl);
    }

    if (appCntxt->radarHdl != NULL)
    {
        /* Release the Radar module context. */
        RADAROGAPPLIB_delete(&appCntxt->radarHdl);
    }

    if (appCntxt->radarPCHdl != NULL)
    {
        RADARLIB_delete(&appCntxt->radarPCHdl);
    }

    if (appCntxt->lidarHdl != NULL)
    {
        /* Release the Lidar module context. */
        LIDAROGAPPLIB_delete(&appCntxt->lidarHdl);
    }

    if (appCntxt->psHdl != NULL)
    {
        PSLIB_delete(&appCntxt->psHdl);
    }

    if (appCntxt->psImgHdl != NULL)
    {
        PSIMGLIB_delete(&appCntxt->psImgHdl);
    }

    if (appCntxt->dofImgHdl != NULL)
    {
        /* Release the Radar module context. */
        DOFIMGLIB_delete(&appCntxt->dofImgHdl);
    }

    if (appCntxt->fusedOgHdl != NULL)
    {
        /* Release the Fusion module context. */
        FUSEDOGAPPLIB_delete(&appCntxt->fusedOgHdl);
    }
}

static void VALETAPP_parseCmdLineArgs(VALETAPP_Context *appCntxt, int argc, char *argv[])
{
    int i;

    if (argc==1)
    {
        VALETAPP_appShowUsage(argv);
        exit(0);
    }

    for(i=0; i<argc; i++)
    {
        if (strcmp(argv[i], "--cfg")==0)
        {
            i++;
            if (i>=argc)
            {
                VALETAPP_appShowUsage(argv);
            }

            VALETAPP_parseCfgFile(appCntxt, argv[i]);

            break;
        }
        else if (strcmp(argv[i], "--help")==0)
        {
            VALETAPP_appShowUsage(argv);
            exit(0);
        }
    }

    return;

} /* VALETAPP_parseCmdLineArgs */

static void VALETAPP_inputStreamInit(VALETAPP_Context *appCntxt)
{
    PTK_DBConfig *cfg = &appCntxt->dbConfig;

    /* Initialize data streams. */
    appCntxt->dataPlayer =
        SensorDataPlayerINS_create(cfg,
                                   sensorAppTags,
                                   VALETAPP_SENSOR_MAX,
                                   VALETAPP_SENSOR_INS,
                                   appCntxt->sensorRateControl);

    /* Check which sensors are enabled */
    if (SensorDataPlayerINS_get_sensorstream(appCntxt->dataPlayer, VALETAPP_SENSOR_CAMERA_R) != NULL)
    {
        appCntxt->cameraEnabled = 1;
    }

    if (SensorDataPlayerINS_get_sensorstream(appCntxt->dataPlayer, VALETAPP_SENSOR_RADAR_0) != NULL ||
        SensorDataPlayerINS_get_sensorstream(appCntxt->dataPlayer, VALETAPP_SENSOR_RADAR_1) != NULL ||
        SensorDataPlayerINS_get_sensorstream(appCntxt->dataPlayer, VALETAPP_SENSOR_RADAR_2) != NULL ||
        SensorDataPlayerINS_get_sensorstream(appCntxt->dataPlayer, VALETAPP_SENSOR_RADAR_3) != NULL)
    {
        appCntxt->radarEnabled = 1;
    }

    if (SensorDataPlayerINS_get_sensorstream(appCntxt->dataPlayer, VALETAPP_SENSOR_LIDAR) != NULL)
    {
        appCntxt->lidarEnabled = 1;
    }

    if (SensorDataPlayerINS_get_sensorstream(appCntxt->dataPlayer, VALETAPP_SENSOR_PSD) != NULL)
    {
        appCntxt->tidlPsdEnabled = 1;
    }

    if (SensorDataPlayerINS_get_sensorstream(appCntxt->dataPlayer, VALETAPP_SENSOR_PSD_IMG) != NULL)
    {
        appCntxt->tidlPsdImgEnabled = 1;
    }

    if (SensorDataPlayerINS_get_sensorstream(appCntxt->dataPlayer, VALETAPP_SENSOR_CAMERA_R_DOF) != NULL)
    {
        appCntxt->dofImgEnabled = 1;
    }

    if (!(appCntxt->cameraEnabled || appCntxt->radarEnabled || appCntxt->lidarEnabled))
    {
        PTK_printf("ERROR: No sensor (camera, radar, nor lidar) enabled!\n");
        PTK_exit(-1);
    }

    return;

} /* VALETAPP_inputStreamInit */

static void VALETAPP_outputStreamInit(VALETAPP_Context *appCntxt)
{
    PTK_DBConfig *cfg = &appCntxt->dbConfig;
    uint32_t            i; /*app index*/
    int32_t             j; /*dbconfig index*/

    if (!appCntxt->dumpMap)
    {
        return;
    }

    PTK_printf("Outputs to be saved to disk =");
    for (i = 0; i < VALETAPP_OUTPUT_MAX; i++)
    {
        j = PTK_DBConfig_exist_output_apptag(cfg, outputAppTags[i]);
        if (j>=0)
        {
            appCntxt->outputStrmHdl[i] =
                VirtualSensorCreator_create_by_dbconfig(cfg, outputAppTags[i]);

            PTK_printf(" %s", outputAppTags[i]);
        }
        else
        {
            appCntxt->outputStrmHdl[i] = NULL;
        }
    }
    PTK_printf("\n");

    return;

} /* VALETAPP_outputStreamInit */

static void VALETAPP_inputStreamDeInit(VALETAPP_Context *appCntxt)
{
    SensorDataPlayerINS_delete(appCntxt->dataPlayer);
    return;

} /* VALETAPP_inputStreamDeInit */

static void VALETAPP_outputStreamDeInit(VALETAPP_Context *appCntxt)
{
    uint32_t i;

    if (!appCntxt->dumpMap)
    {
        return;
    }

    for (i = 0; i < VALETAPP_OUTPUT_MAX; i++)
    {
        if (appCntxt->outputStrmHdl[i] != NULL)
            VirtualSensorCreator_delete(appCntxt->outputStrmHdl[i]);
    }

    return;

} /* VALETAPP_outputStreamDeInit */

static void VALETAPP_run(VALETAPP_Context *appCntxt)
{
    /* sensor data being read */
    uint8_t                    *sensorData;
    FUSEDOGAPPLIB_processReq   *processReq;
    size_t                      sensorDataSize;
    uint64_t                    sensorTs;
    uint32_t                    sensorId;
    PTK_RigidTransform_d        ecef_w;
    PTK_Position                ref;

    memset(appCntxt->stats, 0, sizeof(appCntxt->stats));
    appCntxt->psFrameCnt     = 0;
    appCntxt->psImgFrameCnt  = 0;
    appCntxt->dofImgFrameCnt = 0;
    processReq               = &appCntxt->processReq;
    sensorId                 = 0;

    /* Set up world reference frame
     * (SensorDataPlayerINS handles initializing this at create time) */
    PTK_INS_getReferenceFrame(&ecef_w, &ref);

    /* Set the reference frame in the sensor modules */
    if (appCntxt->cameraEnabled)
    {
        SFMOGAPPLIB_setWorldReference(appCntxt->cameraHdl, &ecef_w);
    }

    if (appCntxt->radarEnabled)
    {
        RADAROGAPPLIB_setWorldReference(appCntxt->radarHdl, &ecef_w);
    }

    if (appCntxt->lidarEnabled)
    {
        LIDAROGAPPLIB_setRootPosition(appCntxt->lidarHdl, &ecef_w, &ref);
    }

    if (appCntxt->fusionEnabled)
    {
        FUSEDOGAPPLIB_setWorldReference(appCntxt->fusedOgHdl, &ecef_w);
    }

    /* Set the reference for PS */
    if (appCntxt->tidlPsdEnabled)
    {
        PSLIB_setWorldReference(appCntxt->psHdl, &ecef_w);
    }

    processReq->tidlPsMap = NULL;

    /* main processing loop */
    while (1)
    {
        sensorId = SensorDataPlayerINS_get_next(appCntxt->dataPlayer,
                                                (void **)&sensorData,
                                                &sensorDataSize,
                                                &sensorTs);
        if (sensorId == UINT32_MAX)
        {
            break;
        }

        appCntxt->curTimeStamp = sensorTs;

        /* call single-sensor process chain */
        sensorProcessCalls[sensorId](appCntxt,
                                     processReq,
                                     sensorId,
                                     sensorData,
                                     sensorDataSize,
                                     sensorTs);

    } /* while (1) */

    if (appCntxt->cameraEnabled)
    {
        SFMOGAPPLIB_waitGraph(appCntxt->cameraHdl);
    }

    if (appCntxt->radarEnabled)
    {
        RADAROGAPPLIB_waitGraph(appCntxt->radarHdl);
    }

    if (appCntxt->lidarEnabled)
    {
        LIDAROGAPPLIB_waitGraph(appCntxt->lidarHdl);
    }

    return;

} /* VALETAPP_run */

static FUSEDOGAPPLIB_input_mask_e processRadar(VALETAPP_Context         *appCntxt,
                                               FUSEDOGAPPLIB_processReq *req,
                                               uint32_t                  s,
                                               uint8_t                  *data,
                                               size_t                    size,
                                               uint64_t                  ts)
{
    VALETAPP_Stats *stats;
    uint32_t        radarSensorId;
    vx_status       vxStatus;

    /* Wait for the data ready semaphore. */
    if (appCntxt->radarDataReadySem)
    {
        appCntxt->radarDataReadySem->wait();
    }

    /* map valet app sensor to radar og sensor id */
    radarSensorId = s - VALETAPP_SENSOR_RADAR_0;

    stats = &appCntxt->stats[VALETAPP_STATS_RADAR_ID];
    stats->totalFrameCount++;

    vxStatus = RADAROGAPPLIB_process(appCntxt->radarHdl,
                                     data,
                                     radarSensorId,
                                     ts);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        stats->droppedFrameCnt++;
    }
    else
    {
        stats->framesProcessed++;
    }

    if (radarSensorId == 0)
    {
        RADARLIB_process(appCntxt->radarPCHdl, size, data);
    }

    return FUSEDOGAPPLIB_INPUT_MASK_RADAR;
}

static FUSEDOGAPPLIB_input_mask_e processLidar(VALETAPP_Context         *appCntxt,
                                               FUSEDOGAPPLIB_processReq *req,
                                               uint32_t                  s,
                                               uint8_t                  *data,
                                               size_t                    size,
                                               uint64_t                  ts)
{
    VALETAPP_Stats *stats;
    vx_status       vxStatus;

    /* Wait for the data ready semaphore. */
    if (appCntxt->lidarDataReadySem)
    {
        appCntxt->lidarDataReadySem->wait();
    }

    stats = &appCntxt->stats[VALETAPP_STATS_LIDAR_ID];
    stats->totalFrameCount++;

    vxStatus = LIDAROGAPPLIB_process(appCntxt->lidarHdl, data);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        stats->droppedFrameCnt++;
    }
    else
    {
        stats->framesProcessed++;
    }

    return FUSEDOGAPPLIB_INPUT_MASK_LIDAR;
}

static FUSEDOGAPPLIB_input_mask_e processCamera(VALETAPP_Context         *appCntxt,
                                                FUSEDOGAPPLIB_processReq *req,
                                                uint32_t                  s,
                                                uint8_t                  *data,
                                                size_t                    size,
                                                uint64_t                  ts)
{
    VALETAPP_Stats *stats;
    int32_t         status;

    /* Wait for the data ready semaphore. */
    if (appCntxt->sfmDataReadySem)
    {
        appCntxt->sfmDataReadySem->wait();
    }

    stats = &appCntxt->stats[VALETAPP_STATS_SFM_ID];
    stats->totalFrameCount++;

    status = SFMOGAPPLIB_process(appCntxt->cameraHdl, data, 0, ts);

    if (status < 0)
    {
        stats->droppedFrameCnt++;
    }
    else
    {
        stats->framesProcessed++;
    }

    return FUSEDOGAPPLIB_INPUT_MASK_SFM;
}

static FUSEDOGAPPLIB_input_mask_e processPS(VALETAPP_Context         *appCntxt,
                                            FUSEDOGAPPLIB_processReq *req,
                                            uint32_t                  s,
                                            uint8_t                  *data,
                                            size_t                    size,
                                            uint64_t                  ts)
{
    PSLIB_process(appCntxt->psHdl,
                  data,
                  0, //@todo multi-camera not enabled at the moment
                  ts);

    req->tidlPsMap = PSLIB_mapOutput(appCntxt->psHdl);
    PSLIB_unmapOutput(appCntxt->psHdl);

    req->tsData.psFrameTs = ts;
    appCntxt->psFrameCnt++;

    return FUSEDOGAPPLIB_INPUT_MASK_PSD;
}

static FUSEDOGAPPLIB_input_mask_e processPSImg(VALETAPP_Context         *appCntxt,
                                               FUSEDOGAPPLIB_processReq *req,
                                               uint32_t                  s,
                                               uint8_t                  *data,
                                               size_t                    size,
                                               uint64_t                  ts)
{
    PSIMGLIB_process(appCntxt->psImgHdl, data, ts);

    req->tsData.psImgFrameTs = ts;
    appCntxt->psImgFrameCnt++;

    return FUSEDOGAPPLIB_INPUT_MASK_PSD_IMG;
}

static FUSEDOGAPPLIB_input_mask_e processDOFImg(VALETAPP_Context         *appCntxt,
                                                FUSEDOGAPPLIB_processReq *req,
                                                uint32_t                  s,
                                                uint8_t                  *data,
                                                size_t                    size,
                                                uint64_t                  ts)
{
    DOFIMGLIB_process(appCntxt->dofImgHdl,
                  data,
                  ts);

    //req->tsData.dofImgFrameTs = ts;
    appCntxt->dofImgFrameCnt++;

    return FUSEDOGAPPLIB_INPUT_MASK_DOF_IMG;
}

static void VALETAPP_init(VALETAPP_Context *appCntxt)
{
    int32_t     status;

    status = appInit();
    PTK_assert(status == 0);

    /* Load sensor processing kernels. */
    appCntxt->vxContext = vxCreateContext();
    APP_ASSERT_VALID_REF(appCntxt->vxContext);

    /* Load kernel modules */
    tivxParkAssistLoadKernels(appCntxt->vxContext);

    tivxLidarLoadKernels(appCntxt->vxContext);

    tivxHwaLoadKernels(appCntxt->vxContext);

    /* Initialize the INS context. */
    PTK_INS_initializeBuffers();

    /* Initialize the input streams and check validity of specified sensors. */
    VALETAPP_inputStreamInit(appCntxt);

    /* Initialize the save-to-file streams (virtual sensor creator objects). */
    VALETAPP_outputStreamInit(appCntxt);

    /* Initialize node parameters. */
    VALETAPP_appLibsInit(appCntxt);

#ifdef PLATFORM_EGL
    /* Create the display semaphore. */
    appCntxt->dispCntxt.displayReadySem = new UTILS::Semaphore();
#endif

    /* Create the semaphore. */
    if (appCntxt->inputRateControl)
    {
        /* Create a data control semaphores. */
        appCntxt->radarDataReadySem =
            new UTILS::Semaphore(appCntxt->radarPipelineDepth);

        appCntxt->lidarDataReadySem =
            new UTILS::Semaphore(appCntxt->lidarPipelineDepth);

        appCntxt->sfmDataReadySem =
            new UTILS::Semaphore(appCntxt->sfmPipelineDepth);
    }
    else
    {
        appCntxt->radarDataReadySem = nullptr;
        appCntxt->lidarDataReadySem = nullptr;
        appCntxt->sfmDataReadySem   = nullptr;
    }

    return;

} /* VALETAPP_init */

static void VALETAPP_deInit(VALETAPP_Context *appCntxt)
{
    int32_t     status;

    /* De-initialize app libs . */
    VALETAPP_appLibsDeInit(appCntxt);

    /* De-initialize the input streams. */
    VALETAPP_inputStreamDeInit(appCntxt);

    /* De-initialize the output streams. */
    VALETAPP_outputStreamDeInit(appCntxt);

    /* Unload kernel modules */
    tivxHwaUnLoadKernels(appCntxt->vxContext);

    tivxLidarUnLoadKernels(appCntxt->vxContext);

    tivxParkAssistUnLoadKernels(appCntxt->vxContext);

    /* Release the context. */
    vxReleaseContext(&appCntxt->vxContext);

    status = appDeInit();
    PTK_assert(status == 0);

    if (appCntxt->radarDataReadySem)
    {
        delete appCntxt->radarDataReadySem;
    }

    if (appCntxt->lidarDataReadySem)
    {
        delete appCntxt->lidarDataReadySem;
    }

    if (appCntxt->sfmDataReadySem)
    {
        delete appCntxt->sfmDataReadySem;
    }

#ifdef PLATFORM_EGL
    delete appCntxt->dispCntxt.displayReadySem;
#endif

    return;

} /* VALETAPP_deInit */

static void VALETAPP_exitProcThreads(VALETAPP_Context  * appCntxt,
                                     bool                detach)
{
    vx_status   vxStatus;

    appCntxt->exitGraphProcess = true;

    if (appCntxt->inputDataThread.joinable())
    {
        if (detach)
        {
            appCntxt->inputDataThread.detach();
        }
        else
        {
            appCntxt->inputDataThread.join();
        }
    }

    appCntxt->exitFusionProcess = true;

    /* Let the event handler thread exit. */
    vxStatus = vxSendUserEvent(appCntxt->vxContext,
                               VALETAPP_USER_EVT_EXIT,
                               NULL);

    if (vxStatus != VX_SUCCESS)
    {
        PTK_printf("[%s:%d] vxSendUserEvent() failed.\n");
    }

    if (appCntxt->evtHdlrThread.joinable())
    {
        appCntxt->evtHdlrThread.join();
    }

    if (appCntxt->fusionThread.joinable())
    {
        appCntxt->fusionThread.join();
    }

}

static void VALETAPP_dumpStats(VALETAPP_Context   *appCntxt)
{
    appPerfStatsPrintAll();

    if (appCntxt->radarHdl)
    {
        RADAROGAPPLIB_printStats(appCntxt->radarHdl);
    }

    if (appCntxt->lidarHdl)
    {
        LIDAROGAPPLIB_printStats(appCntxt->lidarHdl);
    }

    if (appCntxt->cameraHdl)
    {
        SFMOGAPPLIB_printStats(appCntxt->cameraHdl);
    }

    if (appCntxt->fusedOgHdl)
    {
        FUSEDOGAPPLIB_printStats(appCntxt->fusedOgHdl);
    }
}

static void VALETAPP_exportStats(VALETAPP_Context   *appCntxt)
{
    FILE *fp;

    fp = appPerfStatsExportOpenFile(".", appCntxt->perfOutFile);
    if (fp != NULL)
    {
        if (appCntxt->radarHdl)
        {
            RADAROGAPPLIB_exportStats(appCntxt->radarHdl, fp, true);
        }

        if (appCntxt->lidarHdl)
        {
            LIDAROGAPPLIB_exportStats(appCntxt->lidarHdl, fp, true);
        }

        if (appCntxt->cameraHdl)
        {
            SFMOGAPPLIB_exportStats(appCntxt->cameraHdl, fp, true);
        }

        if (appCntxt->fusedOgHdl)
        {
            FUSEDOGAPPLIB_exportStats(appCntxt->fusedOgHdl, fp, true);
        }

        appPerfStatsExportCloseFile(fp);
    }
    else
    {
        PTK_printf("Could not open [%s] for exporting "
                   "performance data\n", appCntxt->perfOutFile);
    }
}

static void VALETAPP_cleanupHdlr(VALETAPP_Context  * appCntxt)
{
    /* Wait for the threads to exit. */
    VALETAPP_exitProcThreads(appCntxt, false);

    PTK_printf("\nPausing to let user inspect the map. "
               "Press ENTER key to exit.\n");
    fflush(stdout);
    getchar();

    PTK_printf("========= BEGIN:PERFORMANCE STATS SUMMARY =========\n");

    VALETAPP_dumpStats(appCntxt);

    if (appCntxt->cameraEnabled)
    {
        VALETAPP_printStats(appCntxt, VALETAPP_STATS_SFM_ID);
    }

    if (appCntxt->radarEnabled)
    {
        VALETAPP_printStats(appCntxt, VALETAPP_STATS_RADAR_ID);
    }

    if (appCntxt->lidarEnabled)
    {
        VALETAPP_printStats(appCntxt, VALETAPP_STATS_LIDAR_ID);
    }

    PTK_printf("========= END:PERFORMANCE STATS SUMMARY ===========\n");

#ifdef PLATFORM_EGL
    PTKDEMO_exitDisplayThread(&appCntxt->dispCntxt);
#endif

    /* Exit the renderer thread. */
    VALETAPP_exitRenderThread(appCntxt);

    if (appCntxt->rtLogEnable == 1)
    {
        char name[256];

        snprintf(name, 255, "%s.bin", appCntxt->perfOutFile);
        tivxLogRtTraceExportToFile(name);
    }

    /* De-initialize the Application context. */
    VALETAPP_deInit(appCntxt);

    PTK_printf("[%s] Clean-up complete.\n", __FUNCTION__);
}

static void VALETAPP_resetOutMaps(VALETAPP_Context  * appCntxt)
{
    vx_status   vxStatus;

    if (appCntxt->cameraEnabled)
    {
        vxStatus = SFMOGAPPLIB_reset(appCntxt->cameraHdl);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] SFMOGAPPLIB_reset() failed.\n",
                       __FUNCTION__,
                       __LINE__);
        }
    }

    if (appCntxt->radarEnabled)
    {
        vxStatus = RADAROGAPPLIB_reset(appCntxt->radarHdl);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] RADAROGAPPLIB_reset() failed.\n",
                       __FUNCTION__,
                       __LINE__);
        }
    }

    if (appCntxt->lidarEnabled)
    {
        vxStatus = LIDAROGAPPLIB_reset(appCntxt->lidarHdl);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] LIDAROGAPPLIB_reset() failed.\n",
                       __FUNCTION__,
                       __LINE__);
        }
    }

    if (appCntxt->fusionEnabled)
    {
        vxStatus = FUSEDOGAPPLIB_reset(appCntxt->fusedOgHdl);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] FUSEDOGAPPLIB_reset() failed.\n",
                       __FUNCTION__,
                       __LINE__);
        }
    }
}

static void VALETAPP_fusionThread(VALETAPP_Context  * appCntxt)
{
    while (!appCntxt->exitFusionProcess)
    {
        FUSEDOGAPPLIB_processReq    req;
        vx_status                   vxStatus;

        req.sensorDataValidMask = 0;

        if (appCntxt->radarHdl)
        {
            RADAROGAPPLIB_OutputBuff    buff;

            vxStatus = RADAROGAPPLIB_getOutBuff(appCntxt->radarHdl, &buff);

            if (vxStatus == (vx_status)VX_SUCCESS)
            {
                req.sensorDataValidMask |= PTK_SensorTypeMask_RADAR;
                req.tsData.radarFramsTs = buff.ts;
                req.radarMap            = buff.vxOutInstMap;
            }
        }

        if (appCntxt->lidarHdl)
        {
            LIDAROGAPPLIB_OutputBuff    buff;

            vxStatus = LIDAROGAPPLIB_getOutBuff(appCntxt->lidarHdl, &buff);

            if (vxStatus == (vx_status)VX_SUCCESS)
            {
                req.sensorDataValidMask |= PTK_SensorTypeMask_LIDAR;
                req.tsData.lidarFramsTs = buff.ts;
                req.lidarMap            = buff.vxOutInstMap;
            }
        }

        if (appCntxt->cameraHdl)
        {
            SFMOGAPPLIB_OutputBuff  buff;

            vxStatus = SFMOGAPPLIB_getOutBuff(appCntxt->cameraHdl, &buff);

            if (vxStatus == (vx_status)VX_SUCCESS)
            {
                req.sensorDataValidMask  |= PTK_SensorTypeMask_CAMERA;
                req.tsData.cameraFramsTs = buff.ts;
                req.cameraMap            = buff.vxOutInstMap;
            }
        }

        if (req.sensorDataValidMask)
        {
            if (appCntxt->fusedOgHdl)
            {
                /* Invoke fusion process. */
                FUSEDOGAPPLIB_process(appCntxt->fusedOgHdl, &req);
            }

            /* Release the output buffers. */
            if (req.sensorDataValidMask & PTK_SensorTypeMask_CAMERA)
            {
                SFMOGAPPLIB_releaseOutBuff(appCntxt->cameraHdl);

                /* Wakeup the input data thread. */
                if (appCntxt->sfmDataReadySem)
                {
                    appCntxt->sfmDataReadySem->notify();
                }
            }

            if (req.sensorDataValidMask & PTK_SensorTypeMask_LIDAR)
            {
                LIDAROGAPPLIB_releaseOutBuff(appCntxt->lidarHdl);

                /* Wakeup the input data thread. */
                if (appCntxt->lidarDataReadySem)
                {
                    appCntxt->lidarDataReadySem->notify();
                }
            }

            if (req.sensorDataValidMask & PTK_SensorTypeMask_RADAR)
            {
                RADAROGAPPLIB_releaseOutBuff(appCntxt->radarHdl);

                /* Wakeup the input data thread. */
                if (appCntxt->radarDataReadySem)
                {
                    appCntxt->radarDataReadySem->notify();
                }
            }
        }
        else
        {
            std::chrono::microseconds   microSec;

            /* Nothing to process. Sleep for 10 ms. */
            microSec = std::chrono::microseconds(100);
            std::this_thread::sleep_for(microSec);
        }
    }
}

static void VALETAPP_evtHdlrThread(VALETAPP_Context *appCntxt)
{
    vx_event_t  evt;
    vx_status   vxStatus;

    vxStatus = VX_SUCCESS;

    PTK_printf("[%s] Launched.\n", __FUNCTION__);

    /* Clear any pending events. The third argument is do_not_block = true. */
    while (vxStatus == VX_SUCCESS)
    {
        vxStatus = vxWaitEvent(appCntxt->vxContext, &evt, vx_true_e);
    }

    while (true)
    {
        vxStatus = vxWaitEvent(appCntxt->vxContext, &evt, vx_false_e);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            if (evt.type == VX_EVENT_USER)
            {
                if (evt.app_value == VALETAPP_USER_EVT_EXIT)
                {
                    break;
                }
            }

            if ((evt.app_value >= VALETAPP_RADAR_OG_GRAPH_EVENT_BASE) &&
                (evt.app_value <  VALETAPP_LIDAR_OG_GRAPH_EVENT_BASE))
            {
                vxStatus = RADAROGAPPLIB_processEvent(appCntxt->radarHdl, &evt);

                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    PTK_printf("[%s:%d] RADAROGAPPLIB_processEvent() "
                               "failed.\n",
                               __FUNCTION__,
                               __LINE__);

                    continue;
                }
            }
            else if ((evt.app_value >= VALETAPP_LIDAR_OG_GRAPH_EVENT_BASE) &&
                     (evt.app_value <  VALETAPP_SFM_OG_GRAPH_EVENT_BASE))
            {
                vxStatus = LIDAROGAPPLIB_processEvent(appCntxt->lidarHdl, &evt);

                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    PTK_printf("[%s:%d] LIDAROGAPPLIB_processEvent() "
                               "failed.\n", __FUNCTION__, __LINE__);
                    continue;
                }
            }
            else if (evt.app_value >= VALETAPP_SFM_OG_GRAPH_EVENT_BASE)
            {
                vxStatus = SFMOGAPPLIB_processEvent(appCntxt->cameraHdl, &evt);

                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    PTK_printf("[%s:%d] SFMOGAPPLIB_processEvent() "
                               "failed.\n", __FUNCTION__, __LINE__);
                    continue;
                }
            }

        } // if (vxStatus == VX_SUCCESS)

    } // while (true)
}

static void VALETAPP_inputDataThread(VALETAPP_Context  * appCntxt)
{
    PTK_printf("[%s] Launched.\n", __FUNCTION__);

    while (true)
    {
        /* Reset the output map. */
        VALETAPP_resetOutMaps(appCntxt);

        /* Execute the graph. */
        VALETAPP_run(appCntxt);

        if (appCntxt->exitGraphProcess)
        {
            break;
        }

        /* De-initialize the input streams. */
        VALETAPP_inputStreamDeInit(appCntxt);

        /* De-initialize the output streams. */
        VALETAPP_outputStreamDeInit(appCntxt);

        /* Reset the INS context. */
        PTK_INS_resetBuffers();

        /* Initialize the input streams. */
        VALETAPP_inputStreamInit(appCntxt);

        /* Initialize the output streams. */
        VALETAPP_outputStreamInit(appCntxt);
    }
}

static void VALETAPP_launchProcThreads(VALETAPP_Context   * appCntxt)
{
    /* Launch the input data thread. */
    appCntxt->inputDataThread =
        std::thread(VALETAPP_inputDataThread, appCntxt);

    /* Launch the fusion processing thread. */
    appCntxt->fusionThread =
        std::thread(VALETAPP_fusionThread, appCntxt);

    /* Launch the event handler thread. */
    appCntxt->evtHdlrThread =
        std::thread(VALETAPP_evtHdlrThread, appCntxt);
}

static void VALETAPP_intSigHandler(int sig)
{
    VALETAPP_Context   *appCntxt;

    appCntxt = &gTestAppCntxt;

    /* Exit the renderer thread. */
    VALETAPP_exitRenderThread(appCntxt);

#ifdef PLATFORM_EGL
    PTKDEMO_exitDisplayThread(&appCntxt->dispCntxt);
#endif

    /* Wait for the threads to exit. */
    VALETAPP_exitProcThreads(appCntxt, true);

    exit(0);
}

static char menu[] = {
    "\n"
    "\n ================================="
    "\n Demo : Multi-sensor Valet Parking"
    "\n ================================="
    "\n"
    "\n p: Print performance statistics"
    "\n"
    "\n e: Export performance statistics"
    "\n"
    "\n x: Exit"
    "\n"
    "\n Enter Choice: "
};

int main(int argc, char* argv[])
{
    VALETAPP_Context *appCntxt;
    PTK_CRT           ptkConfig;

    /* Register the signal handler. */
    signal(SIGINT, VALETAPP_intSigHandler);

    appCntxt = &gTestAppCntxt;

    /* Initialize PTK library. */
    ptkConfig.exit   = exit;
    ptkConfig.printf = printf;
    ptkConfig.time   = NULL;

    PTK_init(&ptkConfig);

    VALETAPP_parseCmdLineArgs(appCntxt, argc, argv);

    /* Initialize the Application context. */
    VALETAPP_init(appCntxt);

    /* Launch the renderer thread. */
    VALETAPP_launchRenderThread(appCntxt);

#ifdef PLATFORM_EGL
    /* Launch display thread */
    appCntxt->dispCntxt.vxContext   = appCntxt->vxContext;
    appCntxt->dispCntxt.dispPeriod  = appCntxt->renderPeriod;
    appCntxt->dispCntxt.vxOutWidth  = appCntxt->winWidth;
    appCntxt->dispCntxt.vxOutHeight = appCntxt->winHeight;
    PTKDEMO_launchDisplayThread(&appCntxt->dispCntxt);
#endif

    /* Launch the processing threads. */
    VALETAPP_launchProcThreads(appCntxt);

    if (appCntxt->is_interactive)
    {
        uint32_t    done = 0;

        appPerfStatsResetAll();

        while (!done)
        {
            char    ch;

            printf(menu);
            ch = getchar();
            printf("\n");

            switch(ch)
            {
                case 'p':
                    VALETAPP_dumpStats(appCntxt);
                    break;

                case 'e':
                    VALETAPP_exportStats(appCntxt);
                    break;

                case 'x':
                    done = 1;
                    break;

            } // switch(ch)

            /* Consume the newline character. */
            if (ch != '\n')
            {
                getchar();
            }

        } // while (!done)

    } // if (appCntxt->is_interactive)

    VALETAPP_cleanupHdlr(appCntxt);

    return 0;

} /* main */
