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
#include <getopt.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <utils/perf_stats/include/app_perf_stats.h>

#ifdef __cplusplus
}
#endif

#include <perception/utils/fsd_pfsd_parse_config.h>
#include <perception/utils/grid_parse_config.h>
#include <perception/utils/lidar_ogmap_parse_config.h>
#include <perception/utils/lidar_gating_parse_config.h>

#include <app_ptk_demo_common.h>

#include "lidar_ogmap_renderer.h"

#define LIDAR_MAX_LINE_LEN 256U

/** sensor inputs */
typedef enum
{
    LIOGMAP_SENSOR_LIDAR = 0,
    LIOGMAP_SENSOR_INS,
    LIOGMAP_SENSOR_MAX
} LIOGMAP_SensorType;

static char gSensorAppTags[LIOGMAP_SENSOR_MAX][DBCONFIG_MAX_WORD_LEN] =
    {{"LID"},{"INS"}};

static char gOutAppTag[] = "MAP_LIDAR";

static LIDAR_AppContext gLidarAppContext{0};

static void LIDAR_appShowUsage(char *name)
{
    PTK_printf("\n");
    PTK_printf("Lidar OGMAP Demo - (c) Texas Instruments 2020\n");
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
}

static int32_t LIDAR_parseCfgFile(LIDAR_AppContext *appCntxt, char *cfgFileName)
{
    FILE                       *fptr;
    char                       *pSLine;
    char                       *globalBasePath;
    char                       *localBasePath;
    char                       *filePath;
    LIDAROGAPPLIB_CreateParams *createParams;
    PTK_Alg_LidarOgmapParams   *ogConfig;
    PTK_Alg_FsdPfsdParams      *fsdPfsdConfig;
    PTK_LidarGatingParams      *gatingParams;
    PTK_GridConfig              gridConfig;
    char                        pLine[LIDAR_MAX_LINE_LEN];
    char                        pParamStr[LIDAR_MAX_LINE_LEN];
    char                        pValueStr[LIDAR_MAX_LINE_LEN];
    char                        filePathArr[LIDAR_MAX_LINE_LEN];
    char                        localBasePathArr[LIDAR_MAX_LINE_LEN] = {0};
    uint32_t                    fsdEnable;
    int32_t                     pos;
    int32_t                     status;

    status         = 0;
    globalBasePath = getenv("APP_CONFIG_BASE_PATH");
    localBasePath  = localBasePathArr;
    filePath       = filePathArr;
    createParams   = &appCntxt->createParams;
    ogConfig       = &createParams->ogPfsdCfg.ogConfig;
    fsdPfsdConfig  = &createParams->ogPfsdCfg.fsdPfsdConfig;
    gatingParams   = &createParams->gatingParams;

    if (globalBasePath == NULL)
    {
        PTK_printf("Please define APP_CONFIG_BASE_PATH environment variable.\n");
        status = -1;
    }

    if (status == 0)
    {
        /* get local base path (directory of cfgFileName) */
        pos = ptkdemo_find_slash(cfgFileName, LIDAR_MAX_LINE_LEN);

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
        createParams->pipelineDepth = LIDAROGAPPLIB_PIPELINE_DEPTH;
        createParams->exportGraph   = 0;
        createParams->rtLogEnable   = 0;
        gatingParams->valid         = 0;
        fsdEnable                   = 0;

        while (1)
        {
            pSLine = fgets(pLine, LIDAR_MAX_LINE_LEN, fptr);

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

            if (0 == strcmp(pParamStr, "tiap_cfg_file"))
            {
                ptkdemo_get_file_path(&filePath, pValueStr, globalBasePath,
                                      localBasePath, LIDAR_MAX_LINE_LEN);
                PTK_DBConfig_parse(&appCntxt->dbConfig, filePath);
            }
            else if (strcmp(pParamStr, "grid_cfg_file") == 0)
            {
                ptkdemo_get_file_path(&filePath, pValueStr, globalBasePath,
                                      localBasePath, LIDAR_MAX_LINE_LEN);
                PTK_Util_parseGridConfig(filePath, &gridConfig);
            }
            else if (strcmp(pParamStr, "dump_output_map") == 0)
            {
                appCntxt->dumpMap = strtol(pValueStr, NULL, 0);
            }
            else if (strcmp(pParamStr, "visualize") == 0)
            {
                appCntxt->visualize = strtol(pValueStr, NULL, 0) &
                                      PTK_SensorTypeMask_LIDAR;
            }
            else if (strcmp(pParamStr, "useTiap") == 0)
            {
                appCntxt->useTiap = strtol(pValueStr, NULL, 0);
            }
            else if (strcmp(pParamStr, "ins_ip_addr") == 0)
            {
                uint8_t size;

                /* IP v4 address will have a max of 15 characters
                 * aaa.bbb.ccc.ddd
                 *
                 * The last character of the string should be a null character.
                 */
                size = strnlen(pValueStr, INS_CAPTURE_DRV_IP_ADDR_LENGTH-1);
                memcpy(appCntxt->insDrvConfig.ipAddr, pValueStr, size);

                /* Need to do the following to keep the GCC compiler happy. */
                appCntxt->insDrvConfig.ipAddr[size] = '\0';
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
            else if (strcmp(pParamStr, "gpc_deploy_core") == 0)
            {
                createParams->gpcNodeCore =
                    app_common_get_coreName(pValueStr);
            }
            else if (strcmp(pParamStr, "mdc_deploy_core") == 0)
            {
                createParams->mdcNodeCore =
                    app_common_get_coreName(pValueStr);
            }
            else if (strcmp(pParamStr, "fsd_pfsd_cfg_file") == 0)
            {
                ptkdemo_get_file_path(&filePath, pValueStr, globalBasePath,
                                      localBasePath, LIDAR_MAX_LINE_LEN);

                PTK_Util_FsdPfsdParseConfig(filePath, fsdPfsdConfig);
            }
            else if (strcmp(pParamStr, "lidar_ogmap_cfg_file") == 0)
            {
                ptkdemo_get_file_path(&filePath, pValueStr, globalBasePath,
                                      localBasePath, LIDAR_MAX_LINE_LEN);

                PTK_Util_LidarOgmapParseConfig(filePath, ogConfig);

                /* We decided to add the gating parameters to the current OGMAP
                 * config file to avoid adding additional config files. The
                 * OGMAP config file will be parsed twice but it is OK since
                 * we do this at initialization time.
                 */
                PTK_Util_LidarGatingParseConfig(filePath, gatingParams);
            }
        }

        fclose(fptr);

        createParams->vxEvtAppValBase     = APP_LIDAR_EVENT_BASE;
        createParams->ogPfsdCfg.fsdEnable = fsdEnable;
        createParams->ogFlagEgo           = OG_FLAG_EGO;
        ogConfig->gridConfig              = gridConfig;

        ogConfig->accGridId               = LIDARAPP_GRID_ID_ACCUMULATED;
        ogConfig->instOccGridId           = LIDARAPP_GRID_ID_INST_OCC;
        ogConfig->instDsGridId            = LIDARAPP_GRID_ID_INST_DS;
        ogConfig->tagPcRemoved            = TAG_POINT_REMOVED;
        ogConfig->tagPcGround             = TAG_MAYBE_GROUND;
        ogConfig->tagOgGround             = OG_FLAG_GROUND;
        ogConfig->tagOgObstacle           = OG_FLAG_OCCUPIED;
        ogConfig->ogFlagChanged           = OG_FLAG_INST_CHANGED;

        if (ogConfig->mappingMethod ==
                PTK_ALG_LIDAR_OGMAP_MAPPING_METHOD_CNT_ONLY)
        {
            appCntxt->perfOutFile = "apps_lidar_ogmap";
        }
        else
        {
            appCntxt->perfOutFile = "apps_lidar_ogmap_ds";
        }

        if (createParams->ogPfsdCfg.fsdEnable)
        {
            fsdPfsdConfig->numBoxes       = fsdPfsdConfig->numBoxes_ogMap_1;
            fsdPfsdConfig->occGridId      = ogConfig->accGridId;
            fsdPfsdConfig->gridConfig     = ogConfig->gridConfig;
            fsdPfsdConfig->ogFlagOccupied = ogConfig->tagOgObstacle;
            fsdPfsdConfig->ogFlagFree     = ogConfig->tagOgGround;
            fsdPfsdConfig->ogFlagFst      = OG_FLAG_FST;
            fsdPfsdConfig->ogFlagFsd      = OG_FLAG_FSD;
            fsdPfsdConfig->ogFlagPfsd     = OG_FLAG_PFSD;
            fsdPfsdConfig->newFSDCycle    = 0;
        }

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
    }

    return status;
}

static int32_t LIDAR_parseCmdLineArgs(LIDAR_AppContext *appCntxt,
                                      int32_t           argc,
                                      char             *argv[])
{
    char    name[LIDAR_MAX_LINE_LEN];
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
                strncpy(name, optarg, LIDAR_MAX_LINE_LEN-1);
                break;

            case 'h' :
            default:
                LIDAR_appShowUsage(argv[0]);
                break;
        }
    }

    if (name[0] == '\0')
    {
        PTK_printf("# ERROR: A valid configuration file MUST be specified.\n");
        LIDAR_appShowUsage(argv[0]);
        status = -1;
    }

    if (status == 0)
    {
        status = LIDAR_parseCfgFile(appCntxt, name);
    }

    return status;
}

static void LIDAR_inputStreamInit(LIDAR_AppContext *appCntxt)
{
    PTK_DBConfig   *cfg = &appCntxt->dbConfig;
    vector<string>  appTagsVec;

    if (!appCntxt->useTiap)
    {
        return;
    }

    for (uint32_t i = 0; i < LIOGMAP_SENSOR_MAX; i++)
    {
        appTagsVec.push_back(string(gSensorAppTags[i]));
    }

    appCntxt->dataPlayer = new SensorDataPlayerINS(cfg,
                                                   appTagsVec,
                                                   LIOGMAP_SENSOR_INS,
                                                   appCntxt->sensorRateControl);

}

static void LIDAR_outputStreamInit(LIDAR_AppContext *appCntxt)
{
    PTK_DBConfig *cfg = &appCntxt->dbConfig;
    int32_t             j; /*dbconfig index*/

    if (!appCntxt->dumpMap)
    {
        return;
    }

    PTK_printf("Outputs to be saved to disk =");

    j = PTK_DBConfig_exist_output_apptag(cfg, gOutAppTag);

    if (j>=0)
    {
        appCntxt->outputStrmHdl =
            VirtualSensorCreator_create_by_dbconfig(cfg, gOutAppTag);

        PTK_printf(" %s\n", gOutAppTag);
    }
    else
    {
        appCntxt->outputStrmHdl = NULL;
    }

    return;

} /* LIDAR_outputStreamInit */

static int32_t LIDAR_init(LIDAR_AppContext *appCntxt)
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
        tivxLidarLoadKernels(appCntxt->vxContext);
        tivxParkAssistLoadKernels(appCntxt->vxContext);
        tivxHwaLoadKernels(appCntxt->vxContext);

        PTK_INS_initializeBuffers();
        appCntxt->firstInsPkt = 1;

        LIDAR_inputStreamInit(appCntxt);

        /* Initialize the output streams. */
        LIDAR_outputStreamInit(appCntxt);

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
        LIDAROGAPPLIB_CreateParams *createParams;

        createParams            = &appCntxt->createParams;
        createParams->vxContext = appCntxt->vxContext;

        appCntxt->lidarHdl = LIDAROGAPPLIB_create(createParams);

        if (appCntxt->lidarHdl == NULL)
        {
            PTK_printf("[%s:%d] LIDAROGAPPLIB_create() failed.\n",
                       __FUNCTION__, __LINE__);

            status = -1;
        }
    }

    return status;
}

static void LIDAR_run(LIDAR_AppContext *appCntxt)
{
    uint32_t                sensorId;
    PTK_RigidTransform_d    ecef_w;
    PTK_Position            ref;

    /* Set up world reference frame
     * (SensorDataPlayerINS handles initializing this at create time) */
    PTK_INS_getReferenceFrame(&ecef_w, &ref);

    /* Set the reference frame in the sensor module. */
    LIDAROGAPPLIB_setRootPosition(appCntxt->lidarHdl, &ecef_w, &ref);

    appCntxt->totalFrameCount = 0;
    appCntxt->droppedFrameCnt = 0;
    appCntxt->framesProcessed = 0;
    sensorId                  = 0;

    while (true)
    {
        void       *sensorData;
        size_t      sensorDataSize;
        uint64_t    sensorTs;
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

        vxStatus = LIDAROGAPPLIB_process(appCntxt->lidarHdl,
                                         (uint8_t*)sensorData);

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
    LIDAROGAPPLIB_waitGraph(appCntxt->lidarHdl);

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
}

static void LIDAR_inputStreamDeInit(LIDAR_AppContext *appCntxt)
{
    if (appCntxt->dataPlayer != NULL)
    {
        delete appCntxt->dataPlayer;
    }
}

static void LIDAR_outputStreamDeInit(LIDAR_AppContext *appCntxt)
{
    if (appCntxt->outputStrmHdl != NULL)
    {
        VirtualSensorCreator_delete(appCntxt->outputStrmHdl);
    }

    return;

} /* LIDAR_outputStreamDeInit */

static int32_t LIDAR_deInit(LIDAR_AppContext *appCntxt)
{
    int32_t     status;

    LIDAR_inputStreamDeInit(appCntxt);

    /* De-initialize the output streams. */
    LIDAR_outputStreamDeInit(appCntxt);

    tivxLidarUnLoadKernels(appCntxt->vxContext);
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
}

static void LIDAR_exitProcThreads(LIDAR_AppContext * appCntxt,
                                  bool               detach)
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
                               APP_LIDAR_USER_EVT_EXIT,
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

static int32_t LIDAR_cleanupHdlr(LIDAR_AppContext  * appCntxt)
{
    float   frac;
    int32_t status1;
    int32_t status = 0;

    /* Wait for the threads to exit. */
    LIDAR_exitProcThreads(appCntxt, false);

    if (appCntxt->doChecksum == false)
    {
        PTK_printf("\nPausing to let user inspect the map. "
                   "Press ENTER key to exit.\n");
        fflush(stdout);
        getchar();
    }

    PTK_printf("Total Number of LIDAR frames     = %d\n",
               appCntxt->totalFrameCount);

    frac = 100.0/appCntxt->totalFrameCount;

    PTK_printf("Number of LIDAR frames processed = %d (%.2f%%)\n",
               appCntxt->framesProcessed,
               appCntxt->framesProcessed * frac);

    PTK_printf("Number of LIDAR frames dropped   = %d (%.2f%%)\n\n",
               appCntxt->droppedFrameCnt,
               appCntxt->droppedFrameCnt * frac);

    PTK_printf("========= BEGIN:PERFORMANCE STATS SUMMARY =========\n");
    appPerfStatsPrintAll();
    LIDAROGAPPLIB_printStats(appCntxt->lidarHdl);
    PTK_printf("========= END:PERFORMANCE STATS SUMMARY ===========\n");

    if (appCntxt->visualize != 0)
    {
#ifdef PLATFORM_EGL
        PTKDEMO_exitDisplayThread(&appCntxt->dispCntxt);
#endif
        /* Exit the renderer thread. */
        LIDAR_exitRenderThread(appCntxt);
    }

    if (appCntxt->doChecksum == true)
    {
        const uint8_t  *data;
        uint32_t        size;

        data = (const uint8_t *)LIDAROGAPPLIB_getOutAccMap(appCntxt->lidarHdl);

        size = LIDAROGAPPLIB_getOutAccMapSize(appCntxt->lidarHdl);

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
    LIDAROGAPPLIB_delete(&appCntxt->lidarHdl);

    /* De-initialize the Application context. */
    status1 = LIDAR_deInit(appCntxt);

    if ((status == 0) && (status1 != 0))
    {
        status = status1;
    }

    PTK_printf("[%s] Clean-up complete.\n", __FUNCTION__);

    return status;
}

static void LIDAR_reset(LIDAR_AppContext  * appCntxt)
{
    vx_status   vxStatus;

    vxStatus = LIDAROGAPPLIB_reset(appCntxt->lidarHdl);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        PTK_printf("[%s:%d] LIDAROGAPPLIB_reset() failed.\n",
                   __FUNCTION__,
                   __LINE__);
    }
}

static void LIDAR_evtHdlrThread(LIDAR_AppContext *appCntxt)
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
                if (evt.app_value == APP_LIDAR_USER_EVT_EXIT)
                {
                    break;
                }
            }

            LIDAROGAPPLIB_processEvent(appCntxt->lidarHdl, &evt);

            if (evt.type == VX_EVENT_GRAPH_COMPLETED)
            {
                LIDAROGAPPLIB_releaseOutBuff(appCntxt->lidarHdl);

                /* Wakeup the input data thread. */
                if (appCntxt->dataReadySem)
                {
                    appCntxt->dataReadySem->notify();
                }
            }
        }

    } // while (true)
}

static void LIDAR_inputDataThread(LIDAR_AppContext  * appCntxt)
{
    PTK_printf("[%s] Launched.\n", __FUNCTION__);

    while (true)
    {
        appCntxt->runCtr++;

        /* Reset the output map. */
        LIDAR_reset(appCntxt);

        /* Execute the graph. */
        LIDAR_run(appCntxt);

        if (appCntxt->exitGraphProcess)
        {
            break;
        }

        /* De-initialize the input streams. */
        LIDAR_inputStreamDeInit(appCntxt);

        /* De-initialize the output streams. */
        LIDAR_outputStreamDeInit(appCntxt);

        /* Reset the INS context. */
        PTK_INS_resetBuffers();

        /* Initialize the input streams. */
        LIDAR_inputStreamInit(appCntxt);

        /* Initialize the output streams. */
        LIDAR_outputStreamInit(appCntxt);
    }
}

static void LIDAR_launchProcThreads(LIDAR_AppContext   * appCntxt)
{
    if (appCntxt->useTiap)
    {
        /* Launch the graph processing thread. */
        appCntxt->inputDataThread =
            std::thread(LIDAR_inputDataThread, appCntxt);
    }
    else
    {
        LIDAR_launchCaptureThreads(appCntxt);
    }

    appCntxt->evtHdlrThread =
        std::thread(LIDAR_evtHdlrThread, appCntxt);
}

static void LIDAR_intSigHandler(int sig)
{
    LIDAR_AppContext   *appCntxt;

    appCntxt = &gLidarAppContext;

    /* Exit the renderer thread. */
    LIDAR_exitRenderThread(appCntxt);

#ifdef PLATFORM_EGL
    if (appCntxt->visualize != 0)
    {
        PTKDEMO_exitDisplayThread(&appCntxt->dispCntxt);
    }
#endif

    /* Wait for the threads to exit. */
    LIDAR_exitProcThreads(appCntxt, true);

    exit(0);
}

static char menu[] = {
    "\n"
    "\n ==============================="
    "\n Demo : Lidar Occupancy Grid Map"
    "\n ==============================="
    "\n"
    "\n p: Print performance statistics"
    "\n"
    "\n e: Export performance statistics"
    "\n"
    "\n x: Exit"
    "\n"
    "\n Enter Choice: "
};

int main(int argc, char **argv)
{
    LIDAR_AppContext   *appCntxt;
    PTK_CRT             ptkConfig;
    int32_t             status;

    /* Initialize PTK library. */
    ptkConfig.exit   = exit;
    ptkConfig.printf = printf;
    ptkConfig.time   = NULL;

    PTK_init(&ptkConfig);

    appCntxt = &gLidarAppContext;

    /* Register the signal handler. */
    signal(SIGINT, LIDAR_intSigHandler);

    status = LIDAR_parseCmdLineArgs(appCntxt, argc, argv);

    if (status == 0)
    {
        status = LIDAR_init(appCntxt);
    }

    if ((status == 0) && (appCntxt->visualize != 0))
    {
        LIDAR_launchRenderThread(appCntxt);

#ifdef PLATFORM_EGL
        /* Launch display thread */
        appCntxt->dispCntxt.vxContext   = appCntxt->vxContext;
        appCntxt->dispCntxt.dispPeriod  = appCntxt->renderPeriod;
        appCntxt->dispCntxt.vxOutWidth  = appCntxt->winWidth;
        appCntxt->dispCntxt.vxOutHeight = appCntxt->winHeight;
        PTKDEMO_launchDisplayThread(&appCntxt->dispCntxt);
#endif
    }

    if (status == 0)
    {
        /* Launch the processing threads. */
        LIDAR_launchProcThreads(appCntxt);
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
                    LIDAROGAPPLIB_printStats(appCntxt->lidarHdl);
                    break;

                case 'e':
                    {
                        FILE *fp = NULL;

                        fp = appPerfStatsExportOpenFile(".",
                                                        appCntxt->perfOutFile);
                        if (fp != NULL)
                        {
                            LIDAROGAPPLIB_exportStats(appCntxt->lidarHdl, fp, true);
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

    }

    if (status == 0)
    {
        status = LIDAR_cleanupHdlr(appCntxt);
    }

    return status;
}
