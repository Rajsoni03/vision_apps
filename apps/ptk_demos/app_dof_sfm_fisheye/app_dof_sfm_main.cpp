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


/* brief: App takes sequence of images and perform DOF pipeline on them. Using DOF output,
 *        creates camera-normalized tracks and performs triangulation on the tracks using
 *        camera poses derived from INS measurements. Output is a sequence of 3D point clouds
 *        in camera coordinates.
 *
 *        The app has two modes:
 *              without LDC: it is assumed that DOF was computed on original
 *                        fisheye images. Track normalization includes fisheye
 *                        lens distortion correction and pinhole intrinsic parameters
 *              with LDC: it is assumed that DOF was computed on rectified images
 *                        which have principal point in the image center. The only
 *                        intrinsic parameter to be corrected for when constructing
 *                        normalized tracks is the focal length
 */

#include <signal.h>
#include <getopt.h>

#include <TI/tivx.h>
#include <TI/j7.h>
#include <TI/tivx_park_assist.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>
#include <assert.h>

#include <app_dof_sfm_main.h>

#include <perception/utils/calmat_utils.h>
#include <perception/utils/fsd_pfsd_parse_config.h>
#include <perception/utils/grid_parse_config.h>
#include <perception/utils/sfm_ogmap_parse_config.h>
#include <app_ptk_demo_common.h>

#define APP_SFM_PERF_OUT_FILE   "apps_sfm_ogmap"

static char sensorAppTags[DOFSFMAPP_SENSOR_MAX][DBCONFIG_MAX_WORD_LEN] = {{"CAM"},{"INS"}};

static AppObj    gAppObj{0};

static void DOFSFMAPP_showUsage(const char *name)
{
    PTK_printf("\n");
    PTK_printf("Dense Optical Flow with LDC + SFM Fisheye App - (c) Texas Instruments 2020\n");
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

static void DOFSFMAPP_getExtrinsicCalibration(ssHandle               cam,
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
        char     calmatPath[TIVX_DOFSFMAPP_MAX_LINE_LEN];
        uint8_t  calmatheader[128];
        int32_t  calmat[4*12];
        uint32_t camId = 1; //position in calmat (0-front, 1-right, 2-rear, 3-left)
        float    scaleFactor = 0.001f; //calmat is in mm
        size_t   size;

        sensorstream_get_full_dir_path(cam, calmatPath);
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

static int32_t DOFSFMAPP_parseCfgFile(AppObj *obj, char *cfg_file_name)
{
    FILE                       *fptr;
    char                       *pParamStr;
    char                       *pValueStr;
    char                       *pSLine;
    char                       *filePath;
    char                       *basePath;
    char                       *localBasePath;
    SFMOGAPPLIB_createParams   *createParams;
    PTK_Alg_FsdPfsdParams      *fsdPfsdConfig;
    char                        paramSt[APP_MAX_FILE_PATH]          = {'\0'};
    char                        valueSt[APP_MAX_FILE_PATH]          = {'\0'};
    char                        sLine[APP_MAX_FILE_PATH]            = {'\0'};
    char                        filePathArr[APP_MAX_FILE_PATH]      = {'\0'};
    char                        localBasePathArr[APP_MAX_FILE_PATH] = {'\0'};
    int32_t                     pos;
    int32_t                     status;

    status        = 0;
    pParamStr     = paramSt;
    pValueStr     = valueSt;
    pSLine        = sLine;
    filePath      = filePathArr;
    localBasePath = localBasePathArr;
    basePath      = getenv("APP_CONFIG_BASE_PATH");
    createParams  = &obj->cameraCreateParams;
    fsdPfsdConfig = &createParams->ogPfsdCfg.fsdPfsdConfig;

    if (basePath == NULL)
    {
        PTK_printf("Please define APP_CONFIG_BASE_PATH environment variable.\n");
        status = -1;
    }

    if (status == 0)
    {
        pos = ptkdemo_find_slash(cfg_file_name, APP_MAX_FILE_PATH);

        if (pos >= 0)
        {
            strncpy(localBasePath, cfg_file_name, pos + 1);
        }
        else
        {
            strcpy(localBasePath, "./");
        }

        fptr = fopen(cfg_file_name, "r");

        if ( !fptr )
        {
            PTK_printf("Cannot open file [%s] for reading.\n", cfg_file_name);
            status = -1;
        }
    }

    if (status == 0)
    {
        /* Set the default render periodicity to 50 milli-sec. */
        obj->renderPeriod           = 50;
        obj->expectedChecksum       = 0;
        obj->inputRateControl       = false;
        obj->sensorRateControl      = false;
        obj->winWidth               = 1920;
        obj->winHeight              = 1080;
        obj->maxNumTracks           = 100000;
        createParams->pipelineDepth = SFMOGAPPLIB_PIPELINE_DEPTH;
        createParams->exportGraph   = 0;
        createParams->rtLogEnable   = 0;
        createParams->enableDof     = false;

        while ( 1 )
        {
            pParamStr[0] = '\0';
            pValueStr[0] = '\0';
            pSLine = fgets(pSLine, APP_MAX_FILE_PATH, fptr);

            if( pSLine == NULL )
            {
                break;
            }

            if (strchr(pSLine, '#'))
            {
                continue;
            }

            sscanf(pSLine,"%128s %128s", pParamStr, pValueStr);

            if (strcmp(pParamStr, "tiap_config_file") == 0)
            {
                ptkdemo_get_file_path(&filePath, pValueStr, basePath,
                                      localBasePath, APP_MAX_FILE_PATH);
                PTK_DBConfig_parse(&obj->dbConfig, filePath);
            }
            else if (strcmp(pParamStr, "verbose") == 0)
            {
                obj->verbose = atoi(pValueStr);
            }
            else if (strcmp(pParamStr, "font_path") == 0)
            {
                snprintf(obj->fontFile, TIVX_DOFSFMAPP_MAX_LINE_LEN,
                         "%s", pValueStr);
            }
            else if (strcmp(pParamStr, "fsd_enable") == 0)
            {
                obj->cameraCreateParams.ogPfsdCfg.fsdEnable = atoi(pValueStr);
            }
            else if (strcmp(pParamStr, "ldc_input_width") == 0)
            {
                obj->pyramidInWidth = atoi(pValueStr);
            }
            else if (strcmp(pParamStr, "ldc_input_height") == 0)
            {
                obj->pyramidInHeight = atoi(pValueStr);
            }
            else if (strcmp(pParamStr, "grid_cfg_file") == 0)
            {
                ptkdemo_get_file_path(&filePath, pValueStr, basePath,
                                      localBasePath, APP_MAX_FILE_PATH);
                PTK_Util_parseGridConfig(filePath, &obj->gridConfig);
            }
            else if (strcmp(pParamStr, "visualize") == 0)
            {
                obj->visualize = strtol(pValueStr, NULL, 0) &
                                 PTK_SensorTypeMask_CAMERA;
            }
            else if (strcmp(pParamStr, "is_interactive") == 0)
            {
                obj->is_interactive = atoi(pValueStr);
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
                obj->sensorRateControl = (bool)atoi(pValueStr);
            }
            else if (strcmp(pParamStr, "input_rate_control") == 0)
            {
                obj->inputRateControl = (bool)atoi(pValueStr);
            }
            else if (strcmp(pParamStr, "expected_checksum") == 0)
            {
                obj->expectedChecksum = strtol(pValueStr, NULL, 0);
            }
            else if (strcmp(pParamStr, "render_periodicity") == 0)
            {
                obj->renderPeriod =
                    static_cast<uint64_t>(atof(pValueStr) * 1000);
            }
            else if (strcmp(pParamStr, "winWidth") == 0)
            {
                obj->winWidth = atoi(pValueStr);
            }
            else if (strcmp(pParamStr, "winHeight") == 0)
            {
                obj->winHeight = atoi(pValueStr);
            }
            else if (strcmp(pParamStr, "max_num_tracks") == 0)
            {
                obj->maxNumTracks = atoi(pValueStr);
            }
            else if (strcmp(pParamStr, "dof_confidence_threshold") == 0)
            {
                obj->dofConfidThresh = atoi(pValueStr);
            }
            else if (strcmp(pParamStr, "dof_enable") == 0)
            {
                createParams->enableDof = (bool)atoi(pValueStr);
            }
            else if (strcmp(pParamStr, "fsd_pfsd_cfg_file") == 0)
            {
                ptkdemo_get_file_path(&filePath, pValueStr, basePath,
                                      localBasePath, APP_MAX_FILE_PATH);

                PTK_Util_FsdPfsdParseConfig(filePath, fsdPfsdConfig);
            }
            else if (strcmp(pParamStr, "sfm_og_deploy_core") == 0)
            {
                obj->cameraCreateParams.ogNodeCore =
                    app_common_get_coreName(pValueStr);
            }
            else if (strcmp(pParamStr, "sfm_tri_deploy_core") == 0)
            {
                obj->cameraCreateParams.triNodeCore =
                    app_common_get_coreName(pValueStr);
            }
            else if (strcmp(pParamStr, "sfm_dof_track_deploy_core") == 0)
            {
                obj->cameraCreateParams.dofTrackNodeCore =
                    app_common_get_coreName(pValueStr);
            }
            else if (strcmp(pParamStr, "sfm_ogmap_cfg_file") == 0)
            {
                ptkdemo_get_file_path(&filePath, pValueStr, basePath,
                                      localBasePath, APP_MAX_FILE_PATH);
                PTK_Util_SfmOgmapParseConfig(filePath,
                         &obj->cameraCreateParams.ogPfsdCfg.ogConfig);
            }

        }

        fclose(fptr);

        /* If checksum is enabled then turn off the interactive mode. */
        if (obj->doChecksum == true)
        {
            obj->is_interactive = 0;
            obj->visualize      = 0;
        }

        /* Validate the parameters. */
        if ((obj->visualize != 0) && (obj->renderPeriod == 0))
        {
            PTK_printf("[%s:%d] Render periodicity cannot be 0.\n",
                       __FUNCTION__, __LINE__);
            status = -1;
        }
    }

    return status;
}

static int32_t DOFSFMAPP_parseCmdLineArgs(AppObj   *obj,
                                          int       argc,
                                          char     *argv[])
{
    char    name[TIVX_DOFSFMAPP_MAX_LINE_LEN];
    int     longIndex;
    int     opt;
    int32_t status;
    static struct option long_options[] = {
        {"help",         no_argument,       0,  'h' },
        {"checksum",     no_argument,       0,  'k' },
        {"cfg",          required_argument, 0,  'c' },
        {0,              0,                 0,   0  }
    };

    obj->doChecksum = false;
    name[0]         = '\0';
    status          = 0;

    while ((opt = getopt_long(argc, argv,"hkc:", 
                   long_options, &longIndex )) != -1)
    {
        switch (opt)
        {
            case 'k' :
                obj->doChecksum = true;
                break;

            case 'c' :
                strncpy(name, optarg, TIVX_DOFSFMAPP_MAX_LINE_LEN-1);
                break;

            case 'h' :
            default:
                DOFSFMAPP_showUsage(argv[0]);
                break;
        }
    }

    if (name[0] == '\0')
    {
        PTK_printf("# ERROR: A valid configuration file MUST be specified.\n");
        DOFSFMAPP_showUsage(argv[0]);
        status = -1;
    }

    if (status == 0)
    {
        status = DOFSFMAPP_parseCfgFile(obj, name);
    }

    return status;
}

static void DOFSFMAPP_dataStreamInit(AppObj *obj)
{
    obj->mapOut = VirtualSensorCreator_create_by_dbconfig(&obj->dbConfig, "MAP_SFM");

    /*inputs*/
    obj->dataPlayer = SensorDataPlayerINS_create(&obj->dbConfig,
                           sensorAppTags,
                           DOFSFMAPP_SENSOR_MAX,
                           DOFSFMAPP_SENSOR_INS,
                           obj->sensorRateControl);
}

static void DOFSFMAPP_dataStreamDeinit(AppObj *obj)
{
    VirtualSensorCreator_delete(obj->mapOut);

    SensorDataPlayerINS_delete(obj->dataPlayer);
}

static int32_t DOFSFMAPP_appLibInit(AppObj *obj)
{
    SFMOGAPPLIB_createParams    *camParams;
    PTK_Alg_SfmOgmapParams      *ogConfig;
    tivx_dmpac_dof_params_t     *dofCfg;
    tivx_dof_to_tracks_params_t *dtNodeCfg;
    tivx_triangulation_params_t *trNodeCfg;
    PTK_Alg_FsdPfsdParams       *fsdPfsdConfig;
    PTK_Map                     *map;
    PTK_Grid                    *grid;
    ssHandle                     cam;
    int32_t                      status;
    int32_t                      i;

    status        = 0;
    camParams     = &obj->cameraCreateParams;
    ogConfig      = &camParams->ogPfsdCfg.ogConfig;
    fsdPfsdConfig = &camParams->ogPfsdCfg.fsdPfsdConfig;
    dofCfg        = &camParams->dofCfg;
    dtNodeCfg     = &camParams->dtNodeCfg;
    trNodeCfg     = &camParams->trNodeCfg;

    camParams->generatePC      = 1;
    camParams->vxContext       = obj->context;
    camParams->vxEvtAppValBase = DOFSFMAPP_EVENT_BASE;

    if (camParams->generatePC == 1)
    {
        if (camParams->enableDof)
        {
            /* DOF node config */
            tivx_dmpac_dof_params_init(dofCfg);
    
            dofCfg->vertical_search_range[0] = 48;
            dofCfg->vertical_search_range[1] = 48;
            dofCfg->horizontal_search_range = 191;
            dofCfg->median_filter_enable = 1;
            dofCfg->motion_smoothness_factor = 24;
            dofCfg->motion_direction = 0; /* 0: for side camera, neutral */
        }

        /* dof2tracks node config */
        memset(dtNodeCfg, 0, sizeof(tivx_dof_to_tracks_params_t));

        dtNodeCfg->dofConfidThresh = obj->dofConfidThresh;
        dtNodeCfg->roiType         = TIVX_DOF_TO_TRACKS_ROI_VERTICAL_CONE;
        dtNodeCfg->roiPrm1         = -4.f;
        dtNodeCfg->roiPrm2         = 648.f;
        dtNodeCfg->subsampleFactor = 2;
        dtNodeCfg->focalLength     = 311.8333f;
        dtNodeCfg->principalPointX = 367.f; //640-1-tx
        dtNodeCfg->principalPointY = 479.f; //360-1-ty
        dtNodeCfg->d2uTableStep    = 0.f; //not used

        /* triangulation node config */
        memset(trNodeCfg, 0, sizeof(tivx_triangulation_params_t));
        trNodeCfg->maxIters            = 2;
        trNodeCfg->enableHighPrecision = 1;

        camParams->dofWidth     = obj->pyramidInWidth;
        camParams->dofHeight    = obj->pyramidInHeight;
        camParams->maxNumTracks = obj->maxNumTracks;
        camParams->dofLevels    = 5;
    }

    /*general map config*/
    ogConfig->method              = 0;
    ogConfig->gridConfig          = obj->gridConfig;

    ogConfig->accGridId           = DOFSFMAPP_GRID_ID_SFM_ACCUMULATED;
    ogConfig->instOccGridId       = DOFSFMAPP_GRID_ID_SFM_INST_OCC;
    ogConfig->instDsGridId        = DOFSFMAPP_GRID_ID_SFM_INST_DS;

    ogConfig->ogFlagFree          = DOFSFMAPP_FLAG_GROUND;
    ogConfig->ogFlagOccupied      = DOFSFMAPP_FLAG_OCCUPIED;
    ogConfig->ogFlagChanged       = DOFSFMAPP_FLAG_INST_CHANGED;

    fsdPfsdConfig->gridConfig     = obj->gridConfig;
    fsdPfsdConfig->numBoxes       = fsdPfsdConfig->numBoxes_ogMap_1;
    fsdPfsdConfig->occGridId      = ogConfig->accGridId;
    fsdPfsdConfig->ogFlagOccupied = ogConfig->ogFlagOccupied;
    fsdPfsdConfig->ogFlagFree     = DOFSFMAPP_FLAG_GROUND;
    fsdPfsdConfig->ogFlagFst      = DOFSFMAPP_FLAG_FST;
    fsdPfsdConfig->ogFlagFsd      = DOFSFMAPP_FLAG_FSD;
    fsdPfsdConfig->ogFlagPfsd     = DOFSFMAPP_FLAG_PFSD;
    fsdPfsdConfig->newFSDCycle    = 0;

    i = PTK_DBConfig_exist_sensor_apptag(&obj->dbConfig, "CAM");

    if (i < 0)
    {
        PTK_printf("[%s:%d] PTK_DBConfig_exist_sensor_apptag() failed.\n",
                   __FUNCTION__, __LINE__);

        status = -1;
    }

    if (status == 0)
    {
        cam = SensorDataPlayerINS_get_sensorstream(obj->dataPlayer,
                                                   DOFSFMAPP_SENSOR_CAMERA);

        if (cam == NULL)
        {
            PTK_printf("[%s:%d] SensorDataPlayerINS_get_sensorstream() "
                       "failed.\n", __FUNCTION__, __LINE__);

            status = -1;
        }
    }

    if (status == 0)
    {
        DOFSFMAPP_getExtrinsicCalibration(cam,
                                          &camParams->M_c_e,
                                          &ogConfig->M_e_g,
                                          obj->verbose);

        obj->cameraHdl = SFMOGAPPLIB_create(&obj->cameraCreateParams);

        if (obj->cameraHdl == NULL)
        {
            PTK_printf("[%s:%d] SFMOGAPPLIB_create() failed.\n",
                       __FUNCTION__, __LINE__);

            status = -1;
        }
    }

    if (status == 0)
    {
        obj->cameraMapSize = SFMOGAPPLIB_getOutAccMapSize(obj->cameraHdl);

        if (obj->inputRateControl)
        {
            obj->dataReadySem =
                new UTILS::Semaphore(obj->cameraCreateParams.pipelineDepth);
        }
        else
        {
            obj->dataReadySem = nullptr;
        }
    }

    return status;
}

static int32_t DOFSFMAPP_init(AppObj *obj)
{
    int32_t  status;

    status = appInit();
    
    if (status != 0)
    {
        PTK_printf("[%s:%d] appInit() failed.\n", __FUNCTION__, __LINE__);
    }

    if (status == 0)
    {
        obj->context = vxCreateContext();

        if (obj->context == NULL)
        {
            PTK_printf("[%s:%d] vxCreateContext() failed.\n",
                       __FUNCTION__, __LINE__);

            status = -1;
        }
    }

    if (status == 0)
    {
        /* Load sensor processing kernels. */
        tivxParkAssistLoadKernels(obj->context);

        tivxHwaLoadKernels(obj->context);

        /* initialize the INS context */
        PTK_INS_initializeBuffers();

        /* Initialize the input/output data streams and
         * check validity of specified sensors.
         */
        DOFSFMAPP_dataStreamInit(obj);

        obj->exitInputDataProcess = 0;
    }

    if (status == 0)
    {
        /* Initialize node parameters */
        status = DOFSFMAPP_appLibInit(obj);
    }

#ifdef PLATFORM_EGL
    if ((status == 0) && (obj->visualize != 0))
    {
        /* Create the display semaphore. */
        obj->dispCntxt.displayReadySem = new UTILS::Semaphore();
    }
#endif

    return status;

}

static void DOFSFMAPP_appLibDeinit(AppObj *obj)
{
    if (obj->dataReadySem)
    {
        delete obj->dataReadySem;
    }

    /* Release the Camera module context. */
    SFMOGAPPLIB_delete(&obj->cameraHdl);
}

static int32_t DOFSFMAPP_deinit(AppObj *obj)
{
    int32_t   status;

    /* de-initialize data streams */
    DOFSFMAPP_dataStreamDeinit(obj);

    /* de-initialize app libs */
    DOFSFMAPP_appLibDeinit(obj);

    tivxParkAssistUnLoadKernels(obj->context);
    tivxHwaUnLoadKernels(obj->context);

    vxReleaseContext(&obj->context);

    status = appDeInit();
    
    if (status != 0)
    {
        PTK_printf("[%s:%d] appDeInit() failed.\n", __FUNCTION__, __LINE__);
    }

#ifdef PLATFORM_EGL
    if (obj->dispCntxt.displayReadySem != NULL)
    {
        delete obj->dispCntxt.displayReadySem;
    }
#endif

    return status;
}

static void DOFSFMAPP_exitProcThreads(AppObj  * obj,
                                      bool      detach)
{
    vx_status   vxStatus;

    obj->exitInputDataProcess = true;

    if (obj->inputDataThread.joinable())
    {
        if (detach)
        {
            /* Exiting under CTRL-C. Detach. */
            obj->inputDataThread.detach();
        }
        else
        {
            /* Block on the input data thread exit. */
            obj->inputDataThread.join();
        }
    }

    /* Let the event handler thread exit. */
    vxStatus = vxSendUserEvent(obj->context,
                               DOFSFMAPP_USER_EVT_EXIT,
                               NULL);

    if (vxStatus != VX_SUCCESS)
    {
        PTK_printf("[%s:%d] vxSendUserEvent() failed.\n");
    }

    if (obj->evtHdlrThread.joinable())
    {
        obj->evtHdlrThread.join();
    }
}

static int32_t DOFSFMAPP_cleanupHdlr(AppObj  * obj)
{
    float   frac;
    int32_t status1;
    int32_t status = 0;

    /* Wait for the threads to exit. */
    DOFSFMAPP_exitProcThreads(obj, false);

    if (obj->doChecksum == false)
    {
        PTK_printf("\nPausing to let user inspect the map. "
                   "Press ENTER key to exit.\n");
        fflush(stdout);
        getchar();
    }

    PTK_printf("Total Number of CAMERA frames     = %d\n",
               obj->totalFrameCount);

    frac = 100.0/obj->totalFrameCount;

    PTK_printf("Number of CAMERA frames processed = %d (%.2f%%)\n",
               obj->framesProcessed,
               obj->framesProcessed * frac);

    PTK_printf("Number of CAMERA frames dropped   = %d (%.2f%%)\n\n",
               obj->droppedFrameCnt,
               obj->droppedFrameCnt * frac);

    PTK_printf("========= BEGIN:PERFORMANCE STATS SUMMARY =========\n");
    appPerfStatsPrintAll();
    SFMOGAPPLIB_printStats(obj->cameraHdl);
    PTK_printf("========= END:PERFORMANCE STATS SUMMARY ===========\n");

    if (obj->visualize != 0)
    {
#ifdef PLATFORM_EGL
        PTKDEMO_exitDisplayThread(&obj->dispCntxt);
#endif

        /* Exit the renderer thread. */
        DOFSFMAPP_exitRenderThread(obj);
    }

    if (obj->doChecksum == true)
    {
        const uint8_t  *data;
        uint32_t        size;

        data = (const uint8_t *)SFMOGAPPLIB_getOutAccMap(obj->cameraHdl);

        size = obj->cameraMapSize;

        obj->computedChecksum = ptkdemo_compute_checksum(data, size);

        PTK_printf("EXPECTED_CHECKSUM: 0x%8.8X COMPUTED_CHECKSUM: 0x%8.8X\n",
                   obj->expectedChecksum,
                   obj->computedChecksum);

        if (obj->expectedChecksum == obj->computedChecksum)
        {
            PTK_printf("TEST_PASSED\n");
        }
        else
        {
            PTK_printf("TEST_FAILED\n");
            status = -1;
        }
    }

    if (obj->cameraCreateParams.rtLogEnable == 1)
    {
        char name[256];

        snprintf(name, 255, "%s.bin", APP_SFM_PERF_OUT_FILE);
        tivxLogRtTraceExportToFile(name);
    }

    status1 = DOFSFMAPP_deinit(obj);

    if ((status == 0) && (status1 != 0))
    {
        status = status1;
    }

    PTK_printf("[%s] Clean-up complete.\n", __FUNCTION__);

    return status;
}

static void DOFSFMAPP_run(AppObj *obj)
{
    vx_status status;
    //int32_t ssStatus;
    void  *dataPtr;
    size_t dataSize;
    uint32_t curFrame;
    uint32_t sensorId;

    PTK_RigidTransform_d ecef_w;
    PTK_Position ref;

    //obj->cameraFrameCnt= 0;
    obj->totalFrameCount = 0;
    obj->droppedFrameCnt = 0;
    obj->framesProcessed = 0;

    /* Set up world reference frame
     * (SensorDataPlayerINS handles initializing this at create time) */
    PTK_INS_getReferenceFrame(&ecef_w, &ref);

    /* Set the reference frame in the sensor modules */
    SFMOGAPPLIB_setWorldReference(obj->cameraHdl, &ecef_w);

    /*******************************************/
    /* main loop (for each frame)              */
    /*******************************************/
    /*
    sensorId = SensorDataPlayerINS_get_next(obj->dataPlayer,
                                            &dataPtr,
                                            &dataSize,
                                            &obj->curTimestamp);
    */
    obj->map = NULL;

    //while (sensorId == DOFSFMAPP_SENSOR_CAMERA)
    while (true)
    {
        sensorId = SensorDataPlayerINS_get_next(obj->dataPlayer,
                                                &dataPtr,
                                                &dataSize,
                                                &obj->curTimestamp);


        if (sensorId == UINT32_MAX)
        {
            break;
        }

        /* Wait for the data ready semaphore. */
        if (obj->dataReadySem)
        {
            obj->dataReadySem->wait();
        }

        obj->totalFrameCount++;

        status = SFMOGAPPLIB_process(obj->cameraHdl,
                                    dataPtr,
                                    0, //@todo multi-camera not enabled at the moment
                                    obj->curTimestamp);

        if (status < 0)
        {
            obj->droppedFrameCnt++;
        }
        else
        {
            obj->framesProcessed++;
        }

        /*get next record */
        /*
        sensorId = SensorDataPlayerINS_get_next(obj->dataPlayer,
                                                &dataPtr,
                                                &dataSize,
                                                &obj->curTimestamp);
        */

    }

    PTK_printf("\e[K[%d] Frames processed = %d/%d Frames dropped = %d/%d\n\e[A",
               obj->runCtr,
               obj->framesProcessed,
               obj->totalFrameCount,
               obj->droppedFrameCnt,
               obj->totalFrameCount);

    /* Wait for the graph to consume all input. */
    SFMOGAPPLIB_waitGraph(obj->cameraHdl);
}

static void DOFSFMAPP_reset(AppObj  * obj)
{
    int32_t status;

    status = SFMOGAPPLIB_reset(obj->cameraHdl);

    if (status < 0)
    {
        PTK_printf("[%s:%d] SFMOGAPPLIB_reset() failed.\n",
                   __FUNCTION__,
                   __LINE__);
    }
}

static void DOFSFMAPP_evtHdlrThread(AppObj *obj)
{
    vx_event_t  evt;
    vx_status   vxStatus;

    vxStatus = VX_SUCCESS;

    PTK_printf("[%s] Launched.\n", __FUNCTION__);

    /* Clear any pending events. The third argument is do_not_block = true. */
    while (vxStatus == VX_SUCCESS)
    {
        vxStatus = vxWaitEvent(obj->context, &evt, vx_true_e);
    }

    while (true)
    {
        vxStatus = vxWaitEvent(obj->context, &evt, vx_false_e);

        if (vxStatus == VX_SUCCESS)
        {
            if(evt.type == VX_EVENT_USER)
            {
                if(evt.app_value == DOFSFMAPP_USER_EVT_EXIT)
                {
                    break;
                }
            }

            SFMOGAPPLIB_processEvent(obj->cameraHdl, &evt);

            if(evt.type == VX_EVENT_GRAPH_COMPLETED)
            {
                SFMOGAPPLIB_releaseOutBuff(obj->cameraHdl);

                /* Wakeup the input data thread. */
                if (obj->dataReadySem)
                {
                    obj->dataReadySem->notify();
                }
             }
        }

    } // while (true)
}


static void DOFSFMAPP_inputDataThread(AppObj  * obj)
{
    PTK_printf("[%s] Launched Graph processing thread.\n", __FUNCTION__);


    while (true)
    {
        obj->runCtr++;

        /* Reset the output map. */
        DOFSFMAPP_reset(obj);

        /* Execute the graph. */
        DOFSFMAPP_run(obj);

        if (obj->exitInputDataProcess)
        {
            break;
        }

        /* De-initialize the output streams. */
        DOFSFMAPP_dataStreamDeinit(obj);

        /* Reset the INS context. */
        PTK_INS_resetBuffers();

        /* Initialize the input streams. */
        DOFSFMAPP_dataStreamInit(obj);

    }
}

static void DOFSFMAPP_launchProcThreads(AppObj   * obj)
{
    /* Launch the graph processing thread. */
    obj->inputDataThread = std::thread(DOFSFMAPP_inputDataThread, obj);

    /* Launch the event handler thread. */
    obj->evtHdlrThread = std::thread(DOFSFMAPP_evtHdlrThread, obj);
}

static void DOFSFMAPP_intSigHandler(int sig)
{
    AppObj   *obj;

    obj = &gAppObj;

    /* Exit the renderer thread. */
    DOFSFMAPP_exitRenderThread(obj);

#ifdef PLATFORM_EGL
    if (obj->visualize != 0)
    {
        PTKDEMO_exitDisplayThread(&obj->dispCntxt);
    }
#endif

    /* Wait for the threads to exit. */
    DOFSFMAPP_exitProcThreads(obj, true);

    exit(0);
}

static char menu[] = {
    "\n"
    "\n ================================="
    "\n Demo : SFM OG MAP                "
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
    AppObj     *obj;
    PTK_CRT     ptkConfig;
    int32_t     status;

    /* Register the signal handler. */
    signal(SIGINT, DOFSFMAPP_intSigHandler);

    /* Initialize PTK library. */
    ptkConfig.exit   = exit;
    ptkConfig.printf = printf;
    ptkConfig.time   = NULL;

    PTK_init(&ptkConfig);

    obj = &gAppObj;

    /* Parse command line args */
    status = DOFSFMAPP_parseCmdLineArgs(obj, argc, argv);

    if (status == 0)
    {
        /* Initialize the application context */
        status = DOFSFMAPP_init(obj);
    }

    if ((status == 0) && (obj->visualize != 0))
    {
        /*******************************************/
        /* OVX nodes/graphs initialization         */
        /*******************************************/
        /* Launch the renderer thread */
        DOFSFMAPP_launchRenderThread(obj);

#ifdef PLATFORM_EGL
        /* Launch display thread */
        obj->dispCntxt.vxContext   = obj->context;
        obj->dispCntxt.dispPeriod  = obj->renderPeriod;
        obj->dispCntxt.vxOutWidth  = obj->winWidth;
        obj->dispCntxt.vxOutHeight = obj->winHeight;
        PTKDEMO_launchDisplayThread(&obj->dispCntxt);
#endif
    }

    if (status == 0)
    {
        /* Launch the graph processing thread */
        DOFSFMAPP_launchProcThreads(obj);
    }

    if ((status == 0) && (obj->is_interactive != 0))
    {
        uint32_t    done = 0;
        uint32_t    i;

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
                    SFMOGAPPLIB_printStats(obj->cameraHdl);
                    break;

                case 'e':
                    {
                        FILE *fp = NULL;
                        char *name = APP_SFM_PERF_OUT_FILE;

                        fp = appPerfStatsExportOpenFile(".", name);
                        if (fp != NULL)
                        {
                            SFMOGAPPLIB_exportStats(obj->cameraHdl, fp, true);
                            appPerfStatsExportCloseFile(fp);
                        }
                        else
                        {
                            PTK_printf("Could not open [%s] for exporting "
                                       "performance data\n", name);
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
        status = DOFSFMAPP_cleanupHdlr(obj);
    }

    return status;
}
