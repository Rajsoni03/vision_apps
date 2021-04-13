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
#include <string>
#include <vector>
#include <getopt.h>

#include <perception/utils/fsd_pfsd_parse_config.h>
#include <perception/utils/grid_parse_config.h>

#include <app_ptk_demo_common.h>

#include "surround_radar_ogmap_renderer.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <utils/perf_stats/include/app_perf_stats.h>

#ifdef __cplusplus
}
#endif

#define RADAR_OGAPP_MAX_LINE_LEN   (256U)

/** sensor inputs */
typedef enum
{
    SUOGMAP_SENSOR_RADAR_0 = 0,
    SUOGMAP_SENSOR_RADAR_1,
    SUOGMAP_SENSOR_RADAR_2,
    SUOGMAP_SENSOR_RADAR_3,
    SUOGMAP_SENSOR_INS,
    SUOGMAP_SENSOR_MAX
} SUOGMAP_SensorType;

static SUOGMAP_Context  gTestAppCntxt{0};

static char
gSensorAppTags[SUOGMAP_SENSOR_MAX][DBCONFIG_MAX_WORD_LEN] =
{
    {"RAD_0"},
    {"RAD_1"},
    {"RAD_2"},
    {"RAD_3"},
    {"INS"  }
};

static char gOutAppTag[] = "MAP_RADAR";

static void SUOGMAP_appShowUsage(const char *name)
{
    PTK_printf("\n");
    PTK_printf("Surround Radar OGMAP Demo - (c) Texas Instruments 2020\n");
    PTK_printf("========================================================\n");
    PTK_printf("\n");
    PTK_printf("Please refer to demo guide for prerequisites before running this demo\n");
    PTK_printf("\n");
    PTK_printf("USAGE:\n");
    PTK_printf("# %s PARAMETERS [OPTIONAL PARAMETERS]\n", name);
    PTK_printf("\n");
    PTK_printf("# PARAMETERS:\n");
    PTK_printf("#  --cfg        |-c <config file>\n");
    PTK_printf("# \n");
    PTK_printf("# OPTIONAL PARAMETERS:\n");
    PTK_printf("#  [--checksum  |-k] Perform checksum validation at the end.");
    PTK_printf(" Specifying this will turn off display and interactive mode.\n");
    PTK_printf("#  [--help      |-h] Display help and exit.\n");
    PTK_printf("# \n");
    PTK_printf("# \n");
    PTK_printf("\n");

    exit(0);

} /* SUOGMAP_appShowUsage */

static int32_t SUOGMAP_parseCfgFile(SUOGMAP_Context *appCntxt, char *cfgFileName)
{
    RADAROGAPPLIB_createParams *createParams;
    PTK_Alg_RadarOgmapParams   *ogConfig;
    PTK_Alg_FsdPfsdParams      *fsdPfsdConfig;
    FILE                       *fptr;
    char                       *pParamStr;
    char                       *pValueStr;
    char                       *pSLine;
    char                       *globalBasePath;
    char                       *localBasePath;
    char                       *filePath;
    PTK_GridConfig              gridConfig;
    char                        paramSt[RADAR_OGAPP_MAX_LINE_LEN];
    char                        valueSt[RADAR_OGAPP_MAX_LINE_LEN];
    char                        sLine[RADAR_OGAPP_MAX_LINE_LEN];
    char                        filePathArr[RADAR_OGAPP_MAX_LINE_LEN];
    char                        localBasePathArr[RADAR_OGAPP_MAX_LINE_LEN] = {0};
    uint32_t                    fsdEnable;
    uint32_t                    radarTypeMask;
    int32_t                     pos;
    int32_t                     status;

    status         = 0;
    pParamStr      = paramSt;
    pValueStr      = valueSt;
    pSLine         = sLine;

    createParams   = &appCntxt->createParams;
    ogConfig       = &createParams->ogPfsdCfg.ogConfig;
    fsdPfsdConfig  = &createParams->ogPfsdCfg.fsdPfsdConfig;
    globalBasePath = getenv("APP_CONFIG_BASE_PATH");
    localBasePath  = localBasePathArr;
    filePath       = filePathArr;
    radarTypeMask  = 1 << dbconfig_sensor_type_RADAR;
    fsdEnable      = 0;

    if (globalBasePath == NULL)
    {
        PTK_printf("Please define APP_CONFIG_BASE_PATH environment variable.\n");
        status = -1;
    }

    if (status == 0)
    {
        /* get local base path (directory of cfgFileName) */
        pos = ptkdemo_find_slash(cfgFileName, RADAR_OGAPP_MAX_LINE_LEN);

        if (pos >= 0)
        {
            strncpy(localBasePath, cfgFileName, pos + 1);
        }
        else
        {
            strcpy(localBasePath, "./");
        }

        fptr = fopen(cfgFileName, "r");

        if (fptr == NULL )
        {
            PTK_printf("Cannot open file [%s] for reading.\n", cfgFileName);
            status = -1;
        }
    }

    if (status == 0)
    {
        /* Set the default render periodicity to 50 milli-sec. */
        appCntxt->renderPeriod      = 50;
        appCntxt->expectedChecksum  = 0;
        appCntxt->inputRateControl  = false;
        appCntxt->sensorRateControl = false;
        appCntxt->winWidth          = 1920;
        appCntxt->winHeight         = 1080;
        createParams->pipelineDepth = RADAROGAPPLIB_PIPELINE_DEPTH;
        createParams->exportGraph   = 0;
        createParams->rtLogEnable   = 0;

        while (1)
        {
            pSLine = fgets(pSLine, RADAR_OGAPP_MAX_LINE_LEN, fptr);

            if (pSLine == NULL)
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

            if ((pParamStr[0] == '\0') || (pValueStr[0] == '\0'))
            {
                continue;
            }

            if (strcmp(pParamStr, "dump_output_map") == 0)
            {
                appCntxt->dumpMap = strtol(pValueStr, NULL, 0);
            }
            else if (strcmp(pParamStr, "visualize") == 0)
            {
                appCntxt->visualize = strtol(pValueStr, NULL, 0) & radarTypeMask;
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
            else if (strcmp(pParamStr, "fsd_enable") == 0)
            {
                fsdEnable = atoi(pValueStr);
            }
            else if (strcmp(pParamStr, "is_interactive") == 0)
            {
                appCntxt->is_interactive = atoi(pValueStr);
            }
            else if (strcmp(pParamStr, "pipeline_depth") == 0)
            {
                createParams->pipelineDepth = atoi(pValueStr);
            }
            else if (strcmp(pParamStr, "rtLogEnable") == 0)
            {
                createParams->rtLogEnable = (uint8_t)atoi(pValueStr);
            }
            else if (strcmp(pParamStr, "exportGraph") == 0)
            {
                createParams->exportGraph = (uint8_t)atoi(pValueStr);
            }
            else if (strcmp(pParamStr, "sensor_rate_control") == 0)
            {
                appCntxt->sensorRateControl = (bool)atoi(pValueStr);
            }
            else if (strcmp(pParamStr, "input_rate_control") == 0)
            {
                appCntxt->inputRateControl = (bool)atoi(pValueStr);
            }
            else if (strcmp(pParamStr, "expected_checksum") == 0)
            {
                appCntxt->expectedChecksum = strtol(pValueStr, NULL, 0);
            }
            else if (strcmp(pParamStr, "og_deploy_core") == 0)
            {
                createParams->ogNodeCore =
                    app_common_get_coreName(pValueStr);
            }
            else if (strcmp(pParamStr, "tiap_cfg_file") == 0)
            {
                ptkdemo_get_file_path(&filePath, pValueStr, globalBasePath,
                                      localBasePath, RADAR_OGAPP_MAX_LINE_LEN);

                PTK_DBConfig_parse(&appCntxt->dbConfig, filePath);
            }
            else if (strcmp(pParamStr, "grid_cfg_file") == 0)
            {
                ptkdemo_get_file_path(&filePath, pValueStr, globalBasePath,
                                      localBasePath, RADAR_OGAPP_MAX_LINE_LEN);

                PTK_Util_parseGridConfig(filePath, &gridConfig);
            }
            else if (strcmp(pParamStr, "radar_cfg_file") == 0)
            {
                ptkdemo_get_file_path(&filePath, pValueStr, globalBasePath,
                                      localBasePath, RADAR_OGAPP_MAX_LINE_LEN);

                PTK_Util_RadarOgmapParseConfig(filePath, ogConfig);
            }
            else if (strcmp(pParamStr, "fsd_pfsd_cfg_file") == 0)
            {
                ptkdemo_get_file_path(&filePath, pValueStr, globalBasePath,
                                      localBasePath, RADAR_OGAPP_MAX_LINE_LEN);

                PTK_Util_FsdPfsdParseConfig(filePath, fsdPfsdConfig);
            }
        }

        createParams->vxEvtAppValBase     = SUOGMAP_EVENT_BASE;
        createParams->ogFlagEgo           = SUOGMAP_FLAG_EGO;
        createParams->ogPfsdCfg.fsdEnable = fsdEnable;
        ogConfig->gridConfig              = gridConfig;

        ogConfig->accGridId               = SUOGMAP_GRID_ID_ACCUMULATED;
        ogConfig->instOccGridId           = SUOGMAP_GRID_ID_INST_OCC;
        ogConfig->instDsGridId            = SUOGMAP_GRID_ID_INST_DS;
        ogConfig->ogFlagOccupied          = SUOGMAP_FLAG_OCCUPIED;
        ogConfig->ogFlagFree              = SUOGMAP_FLAG_GROUND;
        ogConfig->ogFlagChanged           = SUOGMAP_FLAG_INST_CHANGED;

        if (ogConfig->mappingMethod ==
                PTK_ALG_RADAR_OGMAP_MAPPING_METHOD_PROB_ONLY)
        {
            appCntxt->perfOutFile = "apps_radar_ogmap";
        }
        else
        {
            appCntxt->perfOutFile = "apps_radar_ogmap_ds";
        }

        if (createParams->ogPfsdCfg.fsdEnable)
        {
            fsdPfsdConfig->numBoxes         = fsdPfsdConfig->numBoxes_ogMap_1;
            fsdPfsdConfig->occGridId        = ogConfig->accGridId;
            fsdPfsdConfig->gridConfig       = ogConfig->gridConfig;
            fsdPfsdConfig->ogFlagOccupied   = ogConfig->ogFlagOccupied;
            fsdPfsdConfig->ogFlagFree       = SUOGMAP_FLAG_GROUND;
            fsdPfsdConfig->ogFlagFst        = SUOGMAP_FLAG_FST;
            fsdPfsdConfig->ogFlagFsd        = SUOGMAP_FLAG_FSD;
            fsdPfsdConfig->ogFlagPfsd       = SUOGMAP_FLAG_PFSD;
            fsdPfsdConfig->newFSDCycle      = 0;
        }

        fclose(fptr);

        appCntxt->sensorMask = ogConfig->sensorMask;

        /* If checksum is enabled then turn off the interactive mode. */
        if (appCntxt->doChecksum == true)
        {
            appCntxt->is_interactive = 0;
            appCntxt->visualize      = 0;
        }

        /* Validate the parameters. */
        if ((appCntxt->visualize != 0) && (appCntxt->renderPeriod == 0))
        {
            PTK_printf("[%s:%d] Render periodicity cannot be 0.\n",
                       __FUNCTION__, __LINE__);
            status = -1;
        }
        else if (appCntxt->sensorMask == 0)
        {
            PTK_printf("[%s:%d] Sensor Mask cannot be 0.\n",
                       __FUNCTION__, __LINE__);
            status = -1;
        }
    }

    return status;

} /* SUOGMAP_parseCfgFile */

static int32_t SUOGMAP_parseCmdLineArgs(SUOGMAP_Context    *appCntxt,
                                        int32_t             argc,
                                        char               *argv[])
{
    char    name[RADAR_OGAPP_MAX_LINE_LEN];
    int     longIndex;
    int     opt;
    int32_t status;
    static struct option long_options[] = {
        {"help",         no_argument,       0,  'h' },
        {"checksum",     no_argument,       0,  'k' },
        {"cfg",          required_argument, 0,  'c' },
        {0,              0,                 0,   0  }
    };

    appCntxt->doChecksum = false;
    name[0]              = '\0';
    status               = 0;

    while ((opt = getopt_long(argc, argv,"hkc:", 
                   long_options, &longIndex )) != -1)
    {
        switch (opt)
        {
            case 'k' :
                appCntxt->doChecksum = true;
                break;

            case 'c' :
                strncpy(name, optarg, RADAR_OGAPP_MAX_LINE_LEN-1);
                break;

            case 'h' :
            default:
                SUOGMAP_appShowUsage(argv[0]);
                break;
        }
    }

    if (name[0] == '\0')
    {
        PTK_printf("# ERROR: A valid configuration file MUST be specified.\n");
        SUOGMAP_appShowUsage(argv[0]);
        status = -1;
    }

    if (status == 0)
    {
        status = SUOGMAP_parseCfgFile(appCntxt, name);
    }

    return status;

} /* SUOGMAP_parseCmdLineArgs */

static void SUOGMAP_inputStreamInit(SUOGMAP_Context *appCntxt)
{
    PTK_DBConfig   *cfg = &appCntxt->dbConfig;
    vector<string>  appTagsVec;

    for (uint32_t i = 0; i < SUOGMAP_SENSOR_MAX; i++)
    {
        appTagsVec.push_back(string(gSensorAppTags[i]));
    }

    appCntxt->dataPlayer = new SensorDataPlayerINS(cfg,
                                                   appTagsVec,
                                                   SUOGMAP_SENSOR_INS,
                                                   appCntxt->sensorRateControl);

    return;

} /* SUOGMAP_inputStreamInit */

static void SUOGMAP_outputStreamInit(SUOGMAP_Context *appCntxt)
{
    appCntxt->outputStrmHdl = NULL;

    if (appCntxt->dumpMap)
    {
        PTK_DBConfig *cfg = &appCntxt->dbConfig;
        int32_t             j; /*dbconfig index*/

        j = PTK_DBConfig_exist_output_apptag(cfg, gOutAppTag);

        if (j >= 0)
        {
            appCntxt->outputStrmHdl =
                VirtualSensorCreator_create_by_dbconfig(cfg, gOutAppTag);

            PTK_printf("Outputs to be saved to disk = %s\n", gOutAppTag);
        }
    }

    return;

} /* SUOGMAP_outputStreamInit */

static void SUOGMAP_inputStreamDeInit(SUOGMAP_Context *appCntxt)
{
    delete appCntxt->dataPlayer;

    return;

} /* SUOGMAP_inputStreamDeInit */

static void SUOGMAP_outputStreamDeInit(SUOGMAP_Context *appCntxt)
{
    if (appCntxt->outputStrmHdl != NULL)
    {
        VirtualSensorCreator_delete(appCntxt->outputStrmHdl);
    }

    return;

} /* SUOGMAP_outputStreamDeInit */

static void SUOGMAP_run(SUOGMAP_Context *appCntxt)
{
    uint32_t    sensorId;

    /* Read Radar object data, extract the {sensorId and timestamp}
     * fields, strip the header and pass it as 'data'.
     *
     * Read the IMU data, strip the header, interpret the first 8-bytes as
     * timestamp, pass &imuBuff[8] as 'data'. Set sensorId as 0.
     */
    appCntxt->totalFrameCount = 0;
    appCntxt->droppedFrameCnt = 0;
    appCntxt->framesProcessed = 0;
    sensorId                  = 0;

    /* Setup the reference frame. */
    {
        PTK_RigidTransform_d ecef_w;
        PTK_Position         ref;

        /* Set up world reference frame
         * (SensorDataPlayerINS handles initializing this at create time) */
        PTK_INS_getReferenceFrame(&ecef_w, &ref);

        /* Set the reference in the radar module. */
        RADAROGAPPLIB_setWorldReference(appCntxt->radarHdl, &ecef_w);
    }

    while (true)
    {
        void      * sensorData;
        uint64_t    sensorTs;
        size_t      sensorDataSize;
        vx_status   vxStatus;

        sensorId = appCntxt->dataPlayer->get_next(sensorData,
                                                  sensorDataSize,
                                                  sensorTs);

        if (sensorId == UINT32_MAX)
        {
            break;
        }

        /* Wait for the data ready semaphore. */
        if (appCntxt->dataReadySem)
        {
            appCntxt->dataReadySem->wait();
        }

        appCntxt->totalFrameCount++;

        vxStatus = RADAROGAPPLIB_process(appCntxt->radarHdl,
                                         (uint8_t*)sensorData,
                                         sensorId,
                                         sensorTs);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            appCntxt->droppedFrameCnt++;
        }
        else
        {
            appCntxt->framesProcessed++;
        }

        appCntxt->curSensorTs = sensorTs;

    } // while (true)

    /* Wait for the graph to consume all input. */
    RADAROGAPPLIB_waitGraph(appCntxt->radarHdl);

    PTK_printf("\e[K[%d] Frames processed = %d/%d Frames dropped = %d/%d\n\e[A",
               appCntxt->runCtr,
               appCntxt->framesProcessed,
               appCntxt->totalFrameCount,
               appCntxt->droppedFrameCnt,
               appCntxt->totalFrameCount);

    /* Wait for 100 msec before resetting the state since we need to let
     * the graph events to be processed.
     */
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return;

} /* SUOGMAP_run */

static int32_t SUOGMAP_init(SUOGMAP_Context *appCntxt)
{
    int32_t status;

    status = appInit();

    if (status != 0)
    {
        PTK_printf("[%s:%d] appInit() failed.\n", __FUNCTION__, __LINE__);
    }

    if (status == 0)
    {
        appCntxt->vxContext = vxCreateContext();

        if (appCntxt->vxContext == NULL)
        {
            PTK_printf("[%s:%d] vxCreateContext() failed.\n",
                       __FUNCTION__, __LINE__);

            status = -1;
        }
    }

    if (status == 0)
    {
        tivxParkAssistLoadKernels(appCntxt->vxContext);

        tivxHwaLoadKernels(appCntxt->vxContext);

        /* Initialize the INS context. */
        PTK_INS_initializeBuffers();

        /* Initialize the input streams. */
        SUOGMAP_inputStreamInit(appCntxt);

        /* Initialize the output streams. */
        SUOGMAP_outputStreamInit(appCntxt);

        /* Create the semaphore. */
        if (appCntxt->inputRateControl)
        {
            appCntxt->dataReadySem =
                new UTILS::Semaphore(appCntxt->createParams.pipelineDepth);
        }
        else
        {
            appCntxt->dataReadySem = nullptr;
        }
    }

#ifdef PLATFORM_EGL
    if ((status == 0) && (appCntxt->visualize != 0))
    {
        /* Create the display semaphore. */
        appCntxt->dispCntxt.displayReadySem = new UTILS::Semaphore();
    }
#endif

    if (status == 0)
    {
        RADAROGAPPLIB_createParams *createParams;

        createParams            = &appCntxt->createParams;
        createParams->vxContext = appCntxt->vxContext;

        appCntxt->radarHdl = RADAROGAPPLIB_create(createParams);

        if (appCntxt->radarHdl == NULL)
        {
            PTK_printf("[%s:%d] RADAROGAPPLIB_create() failed.\n",
                       __FUNCTION__, __LINE__);

            status = -1;
        }
    }

    return status;

} /* SUOGMAP_init */

static int32_t SUOGMAP_deInit(SUOGMAP_Context *appCntxt)
{
    int32_t     status;

    /* De-initialize the input streams. */
    SUOGMAP_inputStreamDeInit(appCntxt);

    /* De-initialize the output streams. */
    SUOGMAP_outputStreamDeInit(appCntxt);

    tivxParkAssistUnLoadKernels(appCntxt->vxContext);

    tivxHwaUnLoadKernels(appCntxt->vxContext);

    /* Release the context. */
    vxReleaseContext(&appCntxt->vxContext);

    status = appDeInit();

    if (status != 0)
    {
        PTK_printf("[%s:%d] appDeInit() failed.\n", __FUNCTION__, __LINE__);
    }

    if (appCntxt->dataReadySem)
    {
        delete appCntxt->dataReadySem;
    }

#ifdef PLATFORM_EGL
    if (appCntxt->dispCntxt.displayReadySem != NULL)
    {
        delete appCntxt->dispCntxt.displayReadySem;
    }
#endif

    return status;

} /* SUOGMAP_deInit */

static void SUOGMAP_exitProcThreads(SUOGMAP_Context    * appCntxt,
                                    bool                 detach)
{
    vx_status   vxStatus;

    appCntxt->exitGraphProcess = true;

    if (appCntxt->inputDataThread.joinable())
    {
        if (detach)
        {
            /* Exiting under CTRL-C. Detach. */
            appCntxt->inputDataThread.detach();
        }
        else
        {
            /* Block on the input data thread exit. */
            appCntxt->inputDataThread.join();
        }
    }

    /* Let the event handler thread exit. */
    vxStatus = vxSendUserEvent(appCntxt->vxContext,
                               SUOGMAP_USER_EVT_EXIT,
                               NULL);

    if (vxStatus != VX_SUCCESS)
    {
        PTK_printf("[%s:%d] vxSendUserEvent() failed.\n");
    }

    if (appCntxt->evtHdlrThread.joinable())
    {
        appCntxt->evtHdlrThread.join();
    }
}

static int32_t SUOGMAP_cleanupHdlr(SUOGMAP_Context  * appCntxt)
{
    float   frac;
    int32_t status1;
    int32_t status = 0;

    /* Wait for the threads to exit. */
    SUOGMAP_exitProcThreads(appCntxt, false);

    if (appCntxt->doChecksum == false)
    {
        PTK_printf("\nPausing to let user inspect the map. "
                   "Press ENTER key to exit.\n");
        fflush(stdout);
        getchar();
    }

    PTK_printf("Total Number of RADAR frames     = %d\n",
               appCntxt->totalFrameCount);

    frac = 100.0/appCntxt->totalFrameCount;

    PTK_printf("Number of RADAR frames processed = %d (%.2f%%)\n",
               appCntxt->framesProcessed,
               appCntxt->framesProcessed * frac);

    PTK_printf("Number of RADAR frames dropped   = %d (%.2f%%)\n\n",
               appCntxt->droppedFrameCnt,
               appCntxt->droppedFrameCnt * frac);

    PTK_printf("========= BEGIN:PERFORMANCE STATS SUMMARY =========\n");
    appPerfStatsPrintAll();
    RADAROGAPPLIB_printStats(appCntxt->radarHdl);
    PTK_printf("========= END:PERFORMANCE STATS SUMMARY ===========\n");

    if (appCntxt->visualize != 0)
    {
#ifdef PLATFORM_EGL
        PTKDEMO_exitDisplayThread(&appCntxt->dispCntxt);
#endif

        /* Exit the renderer thread. */
        SUOGMAP_exitRenderThread(appCntxt);
    }

    if (appCntxt->doChecksum == true)
    {
        const uint8_t  *data;
        uint32_t        size;

        data = (const uint8_t *)RADAROGAPPLIB_getOutAccMap(appCntxt->radarHdl);

        size = appCntxt->outAccMapSize;

        appCntxt->computedChecksum = ptkdemo_compute_checksum(data, size);

        PTK_printf("EXPECTED_CHECKSUM: 0x%8.8X COMPUTED_CHECKSUM: 0x%8.8X\n",
                   appCntxt->expectedChecksum,
                   appCntxt->computedChecksum);

        if (appCntxt->expectedChecksum == appCntxt->computedChecksum)
        {
            PTK_printf("TEST_PASSED\n");
        }
        else
        {
            PTK_printf("TEST_FAILED\n");
            status = -1;
        }
    }

    if (appCntxt->createParams.rtLogEnable == 1)
    {
        char name[256];

        snprintf(name, 255, "%s.bin", appCntxt->perfOutFile);
        tivxLogRtTraceExportToFile(name);
    }

    /* Release the Application context. */
    RADAROGAPPLIB_delete(&appCntxt->radarHdl);

    /* De-initialize the Application context. */
    status1 = SUOGMAP_deInit(appCntxt);

    if ((status == 0) && (status1 != 0))
    {
        status = status1;
    }

    PTK_printf("[%s] Clean-up complete.\n", __FUNCTION__);

    return status;
}

static void SUOGMAP_reset(SUOGMAP_Context  * appCntxt)
{
    vx_status   vxStatus;

    vxStatus = RADAROGAPPLIB_reset(appCntxt->radarHdl);

    if (vxStatus != VX_SUCCESS)
    {
        PTK_printf("[%s:%d] RADAROGAPPLIB_reset() failed.\n",
                   __FUNCTION__,
                   __LINE__);
    }
}

static void SUOGMAP_evtHdlrThread(SUOGMAP_Context *appCntxt)
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

        if (vxStatus == VX_SUCCESS)
        {
            if (evt.type == VX_EVENT_USER)
            {
                if (evt.app_value == SUOGMAP_USER_EVT_EXIT)
                {
                    break;
                }
            }

            RADAROGAPPLIB_processEvent(appCntxt->radarHdl, &evt);

            if (evt.type == VX_EVENT_GRAPH_COMPLETED)
            {
                RADAROGAPPLIB_releaseOutBuff(appCntxt->radarHdl);

                /* Wakeup the input data thread. */
                if (appCntxt->dataReadySem)
                {
                    appCntxt->dataReadySem->notify();
                }
            }
        }

    } // while (true)
}

static void SUOGMAP_inputDataThread(SUOGMAP_Context  * appCntxt)
{
    PTK_printf("[%s] Launched.\n", __FUNCTION__);

    while (true)
    {
        appCntxt->runCtr++;

        /* Reset the output map. */
        SUOGMAP_reset(appCntxt);

        /* Execute the graph. */
        SUOGMAP_run(appCntxt);

        if (appCntxt->exitGraphProcess)
        {
            break;
        }

        /* De-initialize the input streams. */
        SUOGMAP_inputStreamDeInit(appCntxt);

        /* De-initialize the output streams. */
        SUOGMAP_outputStreamDeInit(appCntxt);

        /* Reset the INS context. */
        PTK_INS_resetBuffers();

        /* Initialize the input streams. */
        SUOGMAP_inputStreamInit(appCntxt);

        /* Initialize the output streams. */
        SUOGMAP_outputStreamInit(appCntxt);
    }

    PTK_printf("[%s] Exited.\n", __FUNCTION__);
}

static void SUOGMAP_launchProcThreads(SUOGMAP_Context   *appCntxt)
{
    /* Launch the graph processing thread. */
    appCntxt->inputDataThread =
        std::thread(SUOGMAP_inputDataThread, appCntxt);

    /* Launch the event handler thread. */
    appCntxt->evtHdlrThread =
        std::thread(SUOGMAP_evtHdlrThread, appCntxt);

}

static void SUOGMAP_intSigHandler(int sig)
{
    SUOGMAP_Context    *appCntxt;

    appCntxt = &gTestAppCntxt;

    /* Exit the renderer thread. */
    SUOGMAP_exitRenderThread(appCntxt);

#ifdef PLATFORM_EGL
    if (appCntxt->visualize != 0)
    {
        PTKDEMO_exitDisplayThread(&appCntxt->dispCntxt);
    }
#endif

    SUOGMAP_exitProcThreads(appCntxt, true);

    exit(0);
}

static char menu[] = {
    "\n"
    "\n ========================================"
    "\n Demo : Surround Radar Occupancy Grid Map"
    "\n ========================================"
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
    SUOGMAP_Context    *appCntxt;
    PTK_CRT             ptkConfig;
    int32_t             status;

    /* Register the signal handler. */
    signal(SIGINT, SUOGMAP_intSigHandler);

    /* Initialize PTK library. */
    ptkConfig.exit   = exit;
    ptkConfig.printf = printf;
    ptkConfig.time   = NULL;

    PTK_init(&ptkConfig);

    appCntxt = &gTestAppCntxt;

    status = SUOGMAP_parseCmdLineArgs(appCntxt, argc, argv);

    if (status == 0)
    {
        /* Initialize the Application context. */
        status = SUOGMAP_init(appCntxt);
    }

    if (status == 0)
    {
        /* Save the output map size. */
        appCntxt->outAccMapSize =
            RADAROGAPPLIB_getOutAccMapSize(appCntxt->radarHdl);

        if (appCntxt->visualize != 0)
        {
            /* Launch the renderer thread. */
            SUOGMAP_launchRenderThread(appCntxt);

#ifdef PLATFORM_EGL
            /* Launch display thread */
            appCntxt->dispCntxt.vxContext   = appCntxt->vxContext;
            appCntxt->dispCntxt.dispPeriod  = appCntxt->renderPeriod;
            appCntxt->dispCntxt.vxOutWidth  = appCntxt->winWidth;
            appCntxt->dispCntxt.vxOutHeight = appCntxt->winHeight;
            PTKDEMO_launchDisplayThread(&appCntxt->dispCntxt);
#endif
        }
    }

    if (status == 0)
    {
        /* Launch processing threads. */
        SUOGMAP_launchProcThreads(appCntxt);
    }

    if ((status == 0) && (appCntxt->is_interactive != 0))
    {
        uint32_t    done = 0;

        appPerfStatsResetAll();

        while (!done)
        {
            char    ch;

            PTK_printf(menu);
            ch = getchar();
            PTK_printf("\n");

            switch(ch)
            {
                case 'p':
                    appPerfStatsPrintAll();
                    RADAROGAPPLIB_printStats(appCntxt->radarHdl);
                    break;

                case 'e':
                    {
                        FILE *fp = NULL;

                        fp = appPerfStatsExportOpenFile(".",
                                                        appCntxt->perfOutFile);
                        if (fp != NULL)
                        {
                            RADAROGAPPLIB_exportStats(appCntxt->radarHdl, fp, true);
                            appPerfStatsExportCloseFile(fp);
                        }
                        else
                        {
                            PTK_printf("Could not open [%s] for exporting "
                                       "performance data\n",
                                       appCntxt->perfOutFile);
                        }
                    }
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

    if (status == 0)
    {
        status = SUOGMAP_cleanupHdlr(appCntxt);
    }

    return status;

} /* main */

