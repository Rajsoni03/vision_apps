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
 * *       any redistribution and use ar./apps/ptk_demos/app_dof_sfm_fisheye/config/app.cfge licensed by TI for use only with TI Devices.
 *
 * *       Nothing shall obligate TI to provide you with source code for the software
 * licensed and provided to you in appCntxtect code.
 *
 * If software source code is provided to you, modification and redistribution of the
 * source code are permitted provided that the following conditions are met:
 *
 * *       any redistribution and use of the source code, including any resulting derivative
 * works, are licensed by TI for use only with TI Devices.
 *
 * *       any redistribution and use of any appCntxtect code compiled from the source code
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
#include <stdio.h>
#include <stdlib.h>

#include "perception/utils/sde_rd_wr.h"
#include "app_sde_obstacle_detection_main.h"
#include "app_sde_obstacle_detection.h"

#include <app_ptk_demo_common.h>
#include <app_ptk_demo_disparity.h>

#define APP_SDE_OBSTACLE_DETECT_NAME    "apps_sde_obstacle_detection"


#if !defined(PC)
static void SODAPP_drawGraphics(Draw2D_Handle *handle, Draw2D_BufInfo *draw2dBufInfo, uint32_t update_type)
{
    appGrpxDrawDefault(handle, draw2dBufInfo, update_type);

    return;
}
#endif

void SODAPP_parseCfgFile(SODAPP_Context *appCntxt, const char *cfg_file_name)
{
    FILE  * fp = fopen(cfg_file_name, "r");
    char  * pParamStr;
    char  * pValueStr;
    char  * pSLine;
    char  * basePath;
    char    paramSt[SODAPP_MAX_LINE_LEN];
    char    valueSt[SODAPP_MAX_LINE_LEN];
    char    sLine[SODAPP_MAX_LINE_LEN];

    // set default parameters
    appCntxt->display_option       = 0;
    appCntxt->width                = 1280;
    appCntxt->height               = 720;
    appCntxt->confidence_threshold = 0;
    appCntxt->is_interactive       = 0;

    appCntxt->sdeAlgoType          = 0;
    appCntxt->numLayers            = 2;
    appCntxt->ppMedianFilterEnable = 0;

    appCntxt->enableLDC            = 0;
    appCntxt->inputFormat          = 0;

    appCntxt->rtLogEnable          = 0;
    appCntxt->exportGraph          = 0;

    appCntxt->sde_params.median_filter_enable        = 1;
    appCntxt->sde_params.reduced_range_search_enable = 0;
    appCntxt->sde_params.disparity_min               = 0;
    appCntxt->sde_params.disparity_max               = 1;
    appCntxt->sde_params.threshold_left_right        = 3;
    appCntxt->sde_params.texture_filter_enable       = 0;
    appCntxt->sde_params.threshold_texture           = 0;
    appCntxt->sde_params.aggregation_penalty_p1      = 32;
    appCntxt->sde_params.aggregation_penalty_p2      = 197;

    appCntxt->sde_params.confidence_score_map[0]     = 0;
    appCntxt->sde_params.confidence_score_map[1]     = 4;
    appCntxt->sde_params.confidence_score_map[2]     = 9;
    appCntxt->sde_params.confidence_score_map[3]     = 18;
    appCntxt->sde_params.confidence_score_map[4]     = 28;
    appCntxt->sde_params.confidence_score_map[5]     = 43;
    appCntxt->sde_params.confidence_score_map[6]     = 109;
    appCntxt->sde_params.confidence_score_map[7]     = 127;

    appCntxt->pipelineDepth                          = SODAPP_MAX_PIPELINE_DEPTH;
    
    if (fp == NULL)
    {
        PTK_printf("# ERROR: Unable to open config file [%s]\n", cfg_file_name);
        exit(0);
    }

    basePath = getenv("APP_STEREO_DATA_PATH");
    if (basePath == NULL)
    {
        PTK_printf("Please define APP_STEREO_DATA_PATH environment variable.\n");
        exit(-1);
    }

    pParamStr  = paramSt;
    pValueStr  = valueSt;
    pSLine     = sLine;

    //while (fgets(line_str, sizeof(line_str), fp) != NULL)
    while (1)
    {
        pSLine = fgets(pSLine, SODAPP_MAX_LINE_LEN, fp);

        if( pSLine == NULL )
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

        /*
        char s[] = " \t";

        if (strchr(line_str, '#'))
        {
            continue;
        }
        */

        /* get the first token */
        if (strcmp(pParamStr, "left_img_file_path") == 0)
        {
            snprintf(appCntxt->left_img_file_path, SODAPP_MAX_LINE_LEN,
                 "%s/%s", basePath, pValueStr);
        }
        if (strcmp(pParamStr, "right_img_file_path") == 0)
        {
            snprintf(appCntxt->right_img_file_path, SODAPP_MAX_LINE_LEN,
                 "%s/%s", basePath, pValueStr);
        }
        else if (strcmp(pParamStr, "output_file_path") == 0)
        {
            snprintf(appCntxt->output_file_path, SODAPP_MAX_LINE_LEN,
                 "%s/%s", basePath, pValueStr);
        }
        if (strcmp(pParamStr, "left_LUT_file_name") == 0)
        {
            snprintf(appCntxt->left_LUT_file_name, SODAPP_MAX_LINE_LEN,
                 "%s/%s", basePath, pValueStr);
        }
        if (strcmp(pParamStr, "right_LUT_file_name") == 0)
        {
            snprintf(appCntxt->right_LUT_file_name, SODAPP_MAX_LINE_LEN,
                 "%s/%s", basePath, pValueStr);
        }
        else if (strcmp(pParamStr, "input_format") == 0)
        {
            appCntxt->inputFormat = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "start_seq") == 0)
        {
            appCntxt->start_fileno = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "end_seq") == 0)
        {
            appCntxt->end_fileno = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "display_option") == 0)
        {
            appCntxt->display_option = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "width") == 0)
        {
            appCntxt->width = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "height") == 0)
        {
            appCntxt->height = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "is_interactive") == 0)
        {
            appCntxt->is_interactive = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "distortion_center_x") == 0)
        {
            appCntxt->distCenterX = atof(pValueStr);
        }
        else if (strcmp(pParamStr, "distortion_center_y") == 0)
        {
            appCntxt->distCenterY = atof(pValueStr);
        }
        else if (strcmp(pParamStr, "focal_length") == 0)
        {
            appCntxt->focalLength = atof(pValueStr);
        }
        else if (strcmp(pParamStr, "camera_roll") == 0)
        {
            appCntxt->camRoll = atof(pValueStr);
        }
        else if (strcmp(pParamStr, "camera_pitch") == 0)
        {
            appCntxt->camPitch = atof(pValueStr);
        }
        else if (strcmp(pParamStr, "camera_yaw") == 0)
        {
            appCntxt->camYaw = atof(pValueStr);
        }
        else if (strcmp(pParamStr, "camera_height") == 0)
        {
            appCntxt->camHeight = atof(pValueStr);
        }
        else if (strcmp(pParamStr, "stereo_baseline") == 0)
        {
            appCntxt->baseline = atof(pValueStr);
        }
        else if (strcmp(pParamStr, "use_road_params") == 0)
        {
            appCntxt->useRoadParams = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "road_top_y_pos") == 0)
        {
            appCntxt->roadTopYpos = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "road_base_size") == 0)
        {
            appCntxt->roadSizeBase = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "road_top_size") == 0)
        {
            appCntxt->roadSizeTop = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "ego_size") == 0)
        {
            appCntxt->egoSize = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "min_obstacle_disparity") == 0)
        {
            appCntxt->minObstacleDisparity = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "min_obstacle_probability") == 0)
        {
            appCntxt->minObstacleProbability = atof(pValueStr);
        }
        else if (strcmp(pParamStr, "min_consecutive_obstacles") == 0)
        {
            appCntxt->minConsecutiveObstacles = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "sde_algo_type") == 0)
        {
            appCntxt->sdeAlgoType = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "num_layers") == 0)
        {
            appCntxt->numLayers = atoi(pValueStr);
        }

#if ENABLE_PP_MEDIAN_FILTER
        else if (strcmp(pParamStr, "pp_median_filter_enable") == 0)
        {
            appCntxt->ppMedianFilterEnable = atoi(pValueStr);
        }
#endif
        else if (strcmp(pParamStr, "sde_confidence_threshold") == 0)
        {
            appCntxt->confidence_threshold = atoi(pValueStr);
        }
        else if(strcmp(pParamStr, "median_filter_enable")==0)
        {
            appCntxt->sde_params.median_filter_enable = atoi(pValueStr);
        }
        else if(strcmp(pParamStr, "reduced_range_search_enable")==0)
        {
            appCntxt->sde_params.reduced_range_search_enable = atoi(pValueStr);
        }        
        else if (strcmp(pParamStr, "disparity_min") == 0)
        {
            appCntxt->sde_params.disparity_min = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "disparity_max") == 0)
        {
            appCntxt->sde_params.disparity_max = atoi(pValueStr);
        }
        else if(strcmp(pParamStr, "threshold_left_right")==0)
        {
            appCntxt->sde_params.threshold_left_right = atoi(pValueStr);
        }
        else if(strcmp(pParamStr, "texture_filter_enable")==0)
        {
            appCntxt->sde_params.texture_filter_enable = atoi(pValueStr);
        }
        else if(strcmp(pParamStr, "threshold_texture")==0)
        {
            appCntxt->sde_params.threshold_texture = atoi(pValueStr);
        }
        else if(strcmp(pParamStr, "aggregation_penalty_p1")==0)
        {
            appCntxt->sde_params.aggregation_penalty_p1 = atoi(pValueStr);
        }
        else if(strcmp(pParamStr, "aggregation_penalty_p2")==0)
        {
            appCntxt->sde_params.aggregation_penalty_p2 = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "pipeline_depth") == 0)
        {
            appCntxt->pipelineDepth  = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "exportGraph") == 0)
        {
            appCntxt->exportGraph = (uint8_t)atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "rtLogEnable") == 0)
        {
            appCntxt->rtLogEnable = (uint8_t)atoi(pValueStr);
        }
        else if(strcmp(pParamStr, "enable_ldc")==0)
        {
            appCntxt->enableLDC = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "disp_merge_deploy_core") == 0)
        {
            appCntxt->mlSdeCreateParams.dispMergeNodeCore = app_common_get_coreName(pValueStr);
        }
        else if (strcmp(pParamStr, "hole_filling_deploy_core") == 0)
        {
            appCntxt->mlSdeCreateParams.holeFillingNodeCore = app_common_get_coreName(pValueStr);
        }
        else if (strcmp(pParamStr, "median_filter_deploy_core") == 0)
        {
            appCntxt->mlSdeCreateParams.medianFilterNodeCore = app_common_get_coreName(pValueStr);
        }
        else if (strcmp(pParamStr, "detection_pipeline_depth") == 0)
        {
            appCntxt->sodCreateParams.pipelineDepth = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "sod_deploy_core") == 0)
        {
            appCntxt->sodCreateParams.sodNodeCore = app_common_get_coreName(pValueStr);
        }
        else if (strcmp(pParamStr, "ge_deploy_core") == 0)
        {
            appCntxt->sodCreateParams.geNodeCore = app_common_get_coreName(pValueStr);
        }
    }

    fclose(fp);

    if (appCntxt->width < 128)
    {
        appCntxt->width = 128;
    }
    if (appCntxt->height < 128)
    {
        appCntxt->height = 128;
    }
    if (appCntxt->end_fileno < appCntxt->start_fileno)
    {
        appCntxt->end_fileno = appCntxt->start_fileno;
    }

    // when multi-layer SDE is used
    if (appCntxt->sdeAlgoType == 1)
    {
        int8_t  factor = (appCntxt->numLayers - 1) * 2;
        int32_t w, i;

        if (appCntxt->height % factor != 0 || appCntxt->width % factor != 0)
        {
            PTK_printf("Improper stereo image resolution...\n");
            exit(0);
        }

        for (i = 1; i < appCntxt->numLayers; i++)
        {
            w = appCntxt->width  / (2*i);

            if (w % 16 != 0)
            {
                PTK_printf("Improper image width is not multiple of 16...\n");
                exit(0);
            }
        }
    }

    appCntxt->renderPeriod = 0;

    return;

} /* SODAPP_parseCfgFile */

void SODAPP_setLDCCreateParams(SODAPP_Context *appCntxt)
{
    SDELDCAPPLIB_createParams * createParams = &appCntxt->sdeLdcCreateParams;

    createParams->leftLutFileName  = appCntxt->left_LUT_file_name;
    createParams->rightLutFileName = appCntxt->right_LUT_file_name;

    createParams->width            = appCntxt->width;
    createParams->height           = appCntxt->height;
    createParams->inputFormat      = appCntxt->inputFormat;
    createParams->pipelineDepth    = appCntxt->pipelineDepth;
}

void SODAPP_setSLSdeCreateParams(SODAPP_Context *appCntxt)
{
    SL_SDEAPPLIB_createParams * createParams = &appCntxt->slSdeCreateParams;

    createParams->sdeCfg = appCntxt->sde_params;

     if (appCntxt->sde_params.disparity_min == 0)
    {
        createParams->minDisparity = 0;
    }
    else if (appCntxt->sde_params.disparity_min == 1)
    {
        createParams->minDisparity = -3;
    }

    if (appCntxt->sde_params.disparity_max == 0)
    {
        createParams->maxDisparity = 63;
    }
    else if (appCntxt->sde_params.disparity_max == 1)
    {
        createParams->maxDisparity = 127;
    }
    else if (appCntxt->sde_params.disparity_max == 2)
    {
        createParams->maxDisparity = 191;
    }

    createParams->width              = appCntxt->width;
    createParams->height             = appCntxt->height;
    createParams->inputFormat        = appCntxt->inputFormat;
    createParams->pipelineDepth      = appCntxt->pipelineDepth;
}

void SODAPP_setMLSdeCreateParams(SODAPP_Context *appCntxt)
{
    ML_SDEAPPLIB_createParams * createParams = &appCntxt->mlSdeCreateParams;\

    createParams->sdeCfg = appCntxt->sde_params;

    if (appCntxt->sde_params.disparity_min == 0)
    {
        createParams->minDisparity = 0;
    }
    else if (appCntxt->sde_params.disparity_min == 1)
    {
        createParams->minDisparity = -3;
    }

    if (appCntxt->sde_params.disparity_max == 0)
    {
        createParams->maxDisparity = 63;
    }
    else if (appCntxt->sde_params.disparity_max == 1)
    {
        createParams->maxDisparity = 127;
    }
    else if (appCntxt->sde_params.disparity_max == 2)
    {
        createParams->maxDisparity = 191;
    }

    createParams->inputFormat        = appCntxt->inputFormat;
    createParams->numLayers          = appCntxt->numLayers;
    createParams->enableMedianFilter = appCntxt->ppMedianFilterEnable;
    createParams->width              = appCntxt->width;
    createParams->height             = appCntxt->height;
    createParams->pipelineDepth      = appCntxt->pipelineDepth;
}


void SODAPP_setGeParams(SODAPP_Context *appCntxt)
{
    float invF;

    /* config parameters */
    appCntxt->sodCreateParams.geCfg.config.width  = appCntxt->width;
    appCntxt->sodCreateParams.geCfg.config.height = appCntxt->height;

    if (appCntxt->sde_params.disparity_min == 0)
    {
        appCntxt->sodCreateParams.geCfg.config.minDisparity = 0;
    }
    else if (appCntxt->sde_params.disparity_min == 1)
    {
        appCntxt->sodCreateParams.geCfg.config.minDisparity = -3;
    }

    if (appCntxt->sde_params.disparity_max == 0)
    {
        appCntxt->sodCreateParams.geCfg.config.maxDisparity = 63;
    }
    else if (appCntxt->sde_params.disparity_max == 1)
    {
        appCntxt->sodCreateParams.geCfg.config.maxDisparity = 127;
    }
    else if (appCntxt->sde_params.disparity_max == 2)
    {
        appCntxt->sodCreateParams.geCfg.config.maxDisparity = 191;
    }

    appCntxt->sodCreateParams.geCfg.config.sde_confidence_threshold = appCntxt->confidence_threshold;

    appCntxt->sodCreateParams.geCfg.config.distCenterX = appCntxt->distCenterX;
    appCntxt->sodCreateParams.geCfg.config.distCenterY = appCntxt->distCenterY;
    appCntxt->sodCreateParams.geCfg.config.focalLength = appCntxt->focalLength;

    /* initialize camera parameters */
    appCntxt->sodCreateParams.geCfg.camParams.width = appCntxt->sodCreateParams.geCfg.config.width;
    appCntxt->sodCreateParams.geCfg.camParams.height = appCntxt->sodCreateParams.geCfg.config.height;

    // for KITTI Sequences
    // To change for TI stereo camera
    appCntxt->sodCreateParams.geCfg.camParams.camHeight = appCntxt->camHeight;//1690; // 1.65m
    appCntxt->sodCreateParams.geCfg.camParams.camRoll   = appCntxt->camRoll;
    appCntxt->sodCreateParams.geCfg.camParams.camPitch  = appCntxt->camPitch;
    appCntxt->sodCreateParams.geCfg.camParams.camYaw    = appCntxt->camYaw;
    appCntxt->sodCreateParams.geCfg.camParams.sinPitch  = sin(appCntxt->camPitch);
    appCntxt->sodCreateParams.geCfg.camParams.cosPitch  = cos(appCntxt->camPitch);
    appCntxt->sodCreateParams.geCfg.camParams.baseline  = appCntxt->baseline;  //537.1058;
    appCntxt->sodCreateParams.geCfg.camParams.dcx = appCntxt->distCenterX;  //609.5593;
    appCntxt->sodCreateParams.geCfg.camParams.dcy = appCntxt->distCenterY; //172.8540;
    appCntxt->sodCreateParams.geCfg.camParams.f = appCntxt->focalLength; //721.5377;


    appCntxt->sodCreateParams.geCfg.camParams.K.M[0] = appCntxt->sodCreateParams.geCfg.camParams.f;
    appCntxt->sodCreateParams.geCfg.camParams.K.M[1] = 0;
    appCntxt->sodCreateParams.geCfg.camParams.K.M[2] = appCntxt->sodCreateParams.geCfg.camParams.dcx;
    appCntxt->sodCreateParams.geCfg.camParams.K.M[3] = 0;
    appCntxt->sodCreateParams.geCfg.camParams.K.M[4] = appCntxt->sodCreateParams.geCfg.camParams.f;
    appCntxt->sodCreateParams.geCfg.camParams.K.M[5] = appCntxt->sodCreateParams.geCfg.camParams.dcy;
    appCntxt->sodCreateParams.geCfg.camParams.K.M[6] = 0;
    appCntxt->sodCreateParams.geCfg.camParams.K.M[7] = 0;
    appCntxt->sodCreateParams.geCfg.camParams.K.M[8] = 1;

    invF = 1.0f / appCntxt->sodCreateParams.geCfg.camParams.f;
    appCntxt->sodCreateParams.geCfg.camParams.invK.M[0] = invF;
    appCntxt->sodCreateParams.geCfg.camParams.invK.M[1] = 0;
    appCntxt->sodCreateParams.geCfg.camParams.invK.M[2] = -invF * appCntxt->sodCreateParams.geCfg.camParams.dcx;
    appCntxt->sodCreateParams.geCfg.camParams.invK.M[3] = 0;
    appCntxt->sodCreateParams.geCfg.camParams.invK.M[4] = invF;
    appCntxt->sodCreateParams.geCfg.camParams.invK.M[5] = -invF * appCntxt->sodCreateParams.geCfg.camParams.dcy;
    appCntxt->sodCreateParams.geCfg.camParams.invK.M[6] = 0;
    appCntxt->sodCreateParams.geCfg.camParams.invK.M[7] = 0;
    appCntxt->sodCreateParams.geCfg.camParams.invK.M[8] = 1;

    /* initialize road params */
    appCntxt->sodCreateParams.geCfg.roadParams.roadCenterBase[0] = (appCntxt->sodCreateParams.geCfg.config.width >> 1) - 40;
    appCntxt->sodCreateParams.geCfg.roadParams.roadCenterBase[1] = appCntxt->sodCreateParams.geCfg.config.height - 1;
    appCntxt->sodCreateParams.geCfg.roadParams.roadCenterTop[0] = (appCntxt->sodCreateParams.geCfg.config.width >> 1) - 40;
    appCntxt->sodCreateParams.geCfg.roadParams.roadCenterTop[1] = appCntxt->roadTopYpos; 

    appCntxt->sodCreateParams.geCfg.roadParams.roadSizeBase = appCntxt->roadSizeBase;
    appCntxt->sodCreateParams.geCfg.roadParams.roadSizeTop = appCntxt->roadSizeTop;

    appCntxt->sodCreateParams.geCfg.roadParams.maxDepth = 100000; // 100 m
    appCntxt->sodCreateParams.geCfg.roadParams.egoSize = appCntxt->egoSize;    // 2.5 m

    /* initialize disparity params */
    appCntxt->sodCreateParams.geCfg.dispParams.confTh = appCntxt->confidence_threshold;
    appCntxt->sodCreateParams.geCfg.dispParams.existConf    = 1;
    appCntxt->sodCreateParams.geCfg.dispParams.minDisparity = appCntxt->sodCreateParams.geCfg.config.minDisparity;
    appCntxt->sodCreateParams.geCfg.dispParams.maxDisparity = appCntxt->sodCreateParams.geCfg.config.maxDisparity;
    appCntxt->sodCreateParams.geCfg.dispParams.numDisparity = appCntxt->sodCreateParams.geCfg.dispParams.maxDisparity - appCntxt->sodCreateParams.geCfg.dispParams.minDisparity + 1;
    appCntxt->sodCreateParams.geCfg.dispParams.fracBits = 4;
    appCntxt->sodCreateParams.geCfg.dispParams.farZ = appCntxt->sodCreateParams.geCfg.roadParams.maxDepth;

    /* initialize uv-Disparity params */
    appCntxt->sodCreateParams.geCfg.uvDispParams.dsFactor = 1; // recommend to set to 1
    appCntxt->sodCreateParams.geCfg.uvDispParams.useRoadParams = appCntxt->useRoadParams;
    appCntxt->sodCreateParams.geCfg.uvDispParams.adaptiveGroundError = 1;
    appCntxt->sodCreateParams.geCfg.uvDispParams.roadWidth = 500;
    appCntxt->sodCreateParams.geCfg.uvDispParams.ransacIter = 1000;
    appCntxt->sodCreateParams.geCfg.uvDispParams.ransacErrTh = 1;
    appCntxt->sodCreateParams.geCfg.uvDispParams.vdWidth = appCntxt->sodCreateParams.geCfg.dispParams.numDisparity;
    appCntxt->sodCreateParams.geCfg.uvDispParams.vdHeight = appCntxt->sodCreateParams.geCfg.config.height;

    if (appCntxt->sodCreateParams.geCfg.uvDispParams.useRoadParams == 1)
    {
        // startX and lastX are refined later
        appCntxt->sodCreateParams.geCfg.uvDispParams.startX = 0;
        appCntxt->sodCreateParams.geCfg.uvDispParams.lastX = appCntxt->sodCreateParams.geCfg.config.width;

        appCntxt->sodCreateParams.geCfg.uvDispParams.startY = appCntxt->sodCreateParams.geCfg.roadParams.roadCenterTop[1];
        appCntxt->sodCreateParams.geCfg.uvDispParams.lastY = appCntxt->sodCreateParams.geCfg.roadParams.roadCenterBase[1];
    }
    else
    {
        appCntxt->sodCreateParams.geCfg.uvDispParams.startX = 0;
        appCntxt->sodCreateParams.geCfg.uvDispParams.lastX = appCntxt->sodCreateParams.geCfg.config.width;

        appCntxt->sodCreateParams.geCfg.uvDispParams.startY = 0;
        appCntxt->sodCreateParams.geCfg.uvDispParams.lastY = appCntxt->sodCreateParams.geCfg.config.height;
    }
}

void SODAPP_setSodParams(SODAPP_Context *appCntxt)
{
    float invF;

    /* config parameters */
    appCntxt->sodCreateParams.sodCfg.odConfig.width = appCntxt->width;
    appCntxt->sodCreateParams.sodCfg.odConfig.height = appCntxt->height;

    if (appCntxt->sde_params.disparity_min == 0)
    {
        appCntxt->sodCreateParams.sodCfg.odConfig.minDisparity = 0;
    }
    else if (appCntxt->sde_params.disparity_min == 1)
    {
        appCntxt->sodCreateParams.sodCfg.odConfig.minDisparity = -3;
    }

    if (appCntxt->sde_params.disparity_max == 0)
    {
        appCntxt->sodCreateParams.sodCfg.odConfig.maxDisparity = 63;
    }
    else if (appCntxt->sde_params.disparity_max == 1)
    {
        appCntxt->sodCreateParams.sodCfg.odConfig.maxDisparity = 127;
    }
    else if (appCntxt->sde_params.disparity_max == 2)
    {
        appCntxt->sodCreateParams.sodCfg.odConfig.maxDisparity = 191;
    }

    appCntxt->sodCreateParams.sodCfg.odConfig.sde_confidence_threshold = appCntxt->confidence_threshold;

    appCntxt->sodCreateParams.sodCfg.odConfig.distCenterX = appCntxt->distCenterX;
    appCntxt->sodCreateParams.sodCfg.odConfig.distCenterY = appCntxt->distCenterY;
    appCntxt->sodCreateParams.sodCfg.odConfig.focalLength = appCntxt->focalLength;

    /* initialize camera parameters */
    appCntxt->sodCreateParams.sodCfg.camParams.width = appCntxt->sodCreateParams.sodCfg.odConfig.width;
    appCntxt->sodCreateParams.sodCfg.camParams.height = appCntxt->sodCreateParams.sodCfg.odConfig.height;

    // for KITTI Sequences
    // To change for TI stereo camera
    appCntxt->sodCreateParams.sodCfg.camParams.camHeight = appCntxt->camHeight; //1690; // 1.65m
    appCntxt->sodCreateParams.sodCfg.camParams.camRoll   = appCntxt->camRoll;
    appCntxt->sodCreateParams.sodCfg.camParams.camPitch  = appCntxt->camPitch;
    appCntxt->sodCreateParams.sodCfg.camParams.camYaw    = appCntxt->camYaw;
    appCntxt->sodCreateParams.sodCfg.camParams.sinPitch  = sin(appCntxt->camPitch);
    appCntxt->sodCreateParams.sodCfg.camParams.cosPitch  = cos(appCntxt->camPitch);
    appCntxt->sodCreateParams.sodCfg.camParams.baseline  = appCntxt->baseline; //537.1058;
    appCntxt->sodCreateParams.sodCfg.camParams.dcx = appCntxt->distCenterX;
    appCntxt->sodCreateParams.sodCfg.camParams.dcy = appCntxt->distCenterY;
    appCntxt->sodCreateParams.sodCfg.camParams.f = appCntxt->focalLength;

    appCntxt->sodCreateParams.sodCfg.camParams.K.M[0] = appCntxt->sodCreateParams.sodCfg.camParams.f;
    appCntxt->sodCreateParams.sodCfg.camParams.K.M[1] = 0;
    appCntxt->sodCreateParams.sodCfg.camParams.K.M[2] = appCntxt->sodCreateParams.sodCfg.camParams.dcx;
    appCntxt->sodCreateParams.sodCfg.camParams.K.M[3] = 0;
    appCntxt->sodCreateParams.sodCfg.camParams.K.M[4] = appCntxt->sodCreateParams.sodCfg.camParams.f;
    appCntxt->sodCreateParams.sodCfg.camParams.K.M[5] = appCntxt->sodCreateParams.sodCfg.camParams.dcy;
    appCntxt->sodCreateParams.sodCfg.camParams.K.M[6] = 0;
    appCntxt->sodCreateParams.sodCfg.camParams.K.M[7] = 0;
    appCntxt->sodCreateParams.sodCfg.camParams.K.M[8] = 1;

    invF = 1.0f / appCntxt->sodCreateParams.sodCfg.camParams.f;
    appCntxt->sodCreateParams.sodCfg.camParams.invK.M[0] = invF;
    appCntxt->sodCreateParams.sodCfg.camParams.invK.M[1] = 0;
    appCntxt->sodCreateParams.sodCfg.camParams.invK.M[2] = -invF * appCntxt->sodCreateParams.sodCfg.camParams.dcx;
    appCntxt->sodCreateParams.sodCfg.camParams.invK.M[3] = 0;
    appCntxt->sodCreateParams.sodCfg.camParams.invK.M[4] = invF;
    appCntxt->sodCreateParams.sodCfg.camParams.invK.M[5] = -invF * appCntxt->sodCreateParams.sodCfg.camParams.dcy;
    appCntxt->sodCreateParams.sodCfg.camParams.invK.M[6] = 0;
    appCntxt->sodCreateParams.sodCfg.camParams.invK.M[7] = 0;
    appCntxt->sodCreateParams.sodCfg.camParams.invK.M[8] = 1;

    /* initialize road params */    
    appCntxt->sodCreateParams.sodCfg.roadParams.roadCenterBase[0] = (appCntxt->sodCreateParams.sodCfg.odConfig.width >> 1) - 40;
    appCntxt->sodCreateParams.sodCfg.roadParams.roadCenterBase[1] = appCntxt->sodCreateParams.sodCfg.odConfig.height;
    appCntxt->sodCreateParams.sodCfg.roadParams.roadCenterTop[0] = (appCntxt->sodCreateParams.sodCfg.odConfig.width >> 1) - 40;
    appCntxt->sodCreateParams.sodCfg.roadParams.roadCenterTop[1] = appCntxt->roadTopYpos;

    appCntxt->sodCreateParams.sodCfg.roadParams.roadSizeBase = appCntxt->roadSizeBase;
    appCntxt->sodCreateParams.sodCfg.roadParams.roadSizeTop = appCntxt->roadSizeTop;

    appCntxt->sodCreateParams.sodCfg.roadParams.maxDepth = 100000; // 100 m
    appCntxt->sodCreateParams.sodCfg.roadParams.egoSize = appCntxt->egoSize;    // 2.5 m

    /* initialize disparity params */
    appCntxt->sodCreateParams.sodCfg.dispParams.confTh = appCntxt->confidence_threshold;
    appCntxt->sodCreateParams.sodCfg.dispParams.minDisparity = appCntxt->sodCreateParams.sodCfg.odConfig.minDisparity;
    appCntxt->sodCreateParams.sodCfg.dispParams.maxDisparity = appCntxt->sodCreateParams.sodCfg.odConfig.maxDisparity;
    appCntxt->sodCreateParams.sodCfg.dispParams.numDisparity = appCntxt->sodCreateParams.sodCfg.dispParams.maxDisparity - appCntxt->sodCreateParams.sodCfg.dispParams.minDisparity + 1;
    appCntxt->sodCreateParams.sodCfg.dispParams.fracBits = 4;
    appCntxt->sodCreateParams.sodCfg.dispParams.farZ = appCntxt->sodCreateParams.sodCfg.roadParams.maxDepth;

    /* compute obstacle probability per pixel params */
    appCntxt->sodCreateParams.sodCfg.copppParams.dsFactor = 4;
    appCntxt->sodCreateParams.sodCfg.copppParams.dsWidth = floor(appCntxt->sodCreateParams.sodCfg.odConfig.width / appCntxt->sodCreateParams.sodCfg.copppParams.dsFactor);
    appCntxt->sodCreateParams.sodCfg.copppParams.dsHeight = floor(appCntxt->sodCreateParams.sodCfg.odConfig.height / appCntxt->sodCreateParams.sodCfg.copppParams.dsFactor);

    appCntxt->sodCreateParams.sodCfg.copppParams.obsProbWindowHeight = 1500; // 2m
    appCntxt->sodCreateParams.sodCfg.copppParams.obsProbWindowWidth = 500;   // 0.5m
    appCntxt->sodCreateParams.sodCfg.copppParams.minSearchBoxSize = 5;
    appCntxt->sodCreateParams.sodCfg.copppParams.pixelAboveHorizonToCheck = 0;
    appCntxt->sodCreateParams.sodCfg.copppParams.minDisparityDifferenceFromModel = 0.0;

    /* find bottom of obstacles params */
    /* These config are set smaller then ones for freespace detection */
    appCntxt->sodCreateParams.sodCfg.fbooParams.minProbability = appCntxt->minObstacleProbability;
    appCntxt->sodCreateParams.sodCfg.fbooParams.minConsecutiveVertical = appCntxt->minConsecutiveObstacles;

    /* find closest obstacles params */
    appCntxt->sodCreateParams.sodCfg.fcdParams.statisticsBoxHeight = 1500; // 1.5m
    appCntxt->sodCreateParams.sodCfg.fcdParams.statisticsNearestPercent = 0.2;
    appCntxt->sodCreateParams.sodCfg.fcdParams.minSearchBoxSize = appCntxt->sodCreateParams.sodCfg.copppParams.minSearchBoxSize;

    /* detect obstacles params */
    appCntxt->sodCreateParams.sodCfg.doParams.maxHorizontalToSkip = 8;
    appCntxt->sodCreateParams.sodCfg.doParams.minHorizontalLength = 32;

    appCntxt->sodCreateParams.sodCfg.doParams.windowHeightRatioRange[0] = 0.333;
    appCntxt->sodCreateParams.sodCfg.doParams.windowHeightRatioRange[1] = 3.0;
    appCntxt->sodCreateParams.sodCfg.doParams.adaptiveTerminationPercentage = 0.99;
    appCntxt->sodCreateParams.sodCfg.doParams.maxDisparityDifference = 3.0;
    appCntxt->sodCreateParams.sodCfg.doParams.obsProbabilityMinDisparityToAccept = (float)appCntxt->minObstacleDisparity;

    /* merge obstacles params */
    appCntxt->sodCreateParams.sodCfg.moParams.depthThreshold = 1000; // 1m
    appCntxt->sodCreateParams.sodCfg.moParams.maxHorizontalSkip = 32;
    appCntxt->sodCreateParams.sodCfg.moParams.tiltThreshold =  0.05;
    appCntxt->sodCreateParams.sodCfg.moParams.adaptiveTerminationPercentage = appCntxt->sodCreateParams.sodCfg.doParams.adaptiveTerminationPercentage;

    /* split obstacles params */
    appCntxt->sodCreateParams.sodCfg.soParams.minSplitComponentWidth = 8;
    appCntxt->sodCreateParams.sodCfg.soParams.splitInlierThreshold = 1.0;
    appCntxt->sodCreateParams.sodCfg.soParams.adaptiveTerminationPercentage = appCntxt->sodCreateParams.sodCfg.moParams.adaptiveTerminationPercentage;
}


void SODAPP_setAllParams(SODAPP_Context *appCntxt)
{
    appCntxt->vxEvtAppValBase = 0;

    /* LDC params */
    SODAPP_setLDCCreateParams(appCntxt);

    /* SDE params */
    if (appCntxt->sdeAlgoType == 0)
    {
        SODAPP_setSLSdeCreateParams(appCntxt);
    }
    else if (appCntxt->sdeAlgoType == 1)
    {
        SODAPP_setMLSdeCreateParams(appCntxt);
    }

    /* Obstacle detection params */
    appCntxt->sodCreateParams.sdeCfg          = appCntxt->sde_params;
    appCntxt->sodCreateParams.inputFormat     = appCntxt->inputFormat;
    appCntxt->sodCreateParams.pipelineDepth   = appCntxt->pipelineDepth;

    SODAPP_setGeParams(appCntxt);
    SODAPP_setSodParams(appCntxt);
} /* SODAPP_setAllParams */


int32_t SODAPP_createDraw(SODAPP_Context *appCntxt)
{
    int32_t status;
    Draw2D_BufInfo sBufInfo;

    appCntxt->pDisplayBuf565 = (uint16_t *)tivxMemAlloc(appCntxt->width * appCntxt->height * sizeof(uint16_t), TIVX_MEM_EXTERNAL);
    PTK_assert(NULL != appCntxt->pDisplayBuf565);

    Draw2D_create(&appCntxt->pHndl);

    sBufInfo.bufWidth = appCntxt->width;
    sBufInfo.bufHeight = appCntxt->height;
    sBufInfo.bufPitch[0] = appCntxt->width * 2;
    sBufInfo.dataFormat = DRAW2D_DF_BGR16_565;
    sBufInfo.transperentColor = 1;
    sBufInfo.transperentColorFormat = DRAW2D_DF_BGR16_565;
    sBufInfo.bufAddr[0] = (uint8_t *)appCntxt->pDisplayBuf565;

    status = Draw2D_setBufInfo(appCntxt->pHndl, &sBufInfo);

    return status;
}

void SODAPP_init(SODAPP_Context *appCntxt)
{
    int32_t                     status;
    vx_status                   vxStatus;

    // bounding boxes for obstacles
    appCntxt->obsBox            = (tivx_obstacle_pos_t *)malloc(MAX_DETECTIONS * sizeof(tivx_obstacle_pos_t));

    // free space boundaries
    appCntxt->freespaceBoundary = (int32_t *)malloc((MAX_FRAME_WIDTH / 4) * sizeof(int32_t));

    status = appInit();
    PTK_assert(status == 0);

    // OpenVX initialization
    appCntxt->vxContext = vxCreateContext();
    PTK_assert(appCntxt->vxContext);

    // Create graph
    appCntxt->vxGraph = vxCreateGraph(appCntxt->vxContext);
    if (appCntxt->vxGraph == NULL)
    {
        PTK_printf("[%s:%d] vxCreateGraph() failed\n",
                    __FUNCTION__, __LINE__);
        status = -1;
    }

    if (status >= 0)
    {
        vxSetReferenceName((vx_reference)appCntxt->vxGraph, "SDE Obstacle Detection Graph");
    }

    if (status >= 0)
    {
        tivxStereoLoadKernels(appCntxt->vxContext);
        tivxHwaLoadKernels(appCntxt->vxContext);
        /*
         * 1 Setup Stereo LDC nodes
         */
        status = SODAPP_init_LDC(appCntxt);
    }

    /*
     * 2 Setup SDE nodes
     */
    if (status >= 0)
    {
        status = SODAPP_init_SDE(appCntxt);
    }

    /*
     * 3. Setup Obstacle Detection node
     */
    if (status >= 0)
    {
        status = SODAPP_init_Detection(appCntxt);
    }

    if (status >= 0)
    {
        /*
         * set up the pipeline. 
         */
        vxStatus = SODAPP_setupPipeline(appCntxt);

        /* Verify graph */
        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            vxStatus = vxVerifyGraph(appCntxt->vxGraph);
            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] vxVerifyGraph() failed\n",
                            __FUNCTION__, __LINE__);
                status = -1;
            }
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            appPerfPointSetName(&appCntxt->sodPerf , "Stereo Obstacle Detetion GRAPH");
        }
    }

    if (status >= 0)
    {
        // init sacler for ML SDE
        if (appCntxt->sdeAlgoType == 1)
        {
            ML_SDEAPPLIB_initScaler(appCntxt->mlSdeHdl);
        }

        if (appCntxt->exportGraph == 1)
        {
            tivxExportGraphToDot(appCntxt->vxGraph, ".", "vx_app_sde_obstacle_detection");
        }

        if (appCntxt->rtLogEnable == 1)
        {
            tivxLogRtTraceEnable(appCntxt->vxGraph);
        }

        appCntxt->exitInputDataProcess = false;
        appCntxt->dataReadySem   = new Semaphore(appCntxt->pipelineDepth);

        if (1 == appCntxt->display_option)
        {
#if !defined(PC)
            app_grpx_init_prms_t grpx_prms;
            appGrpxInitParamsInit(&grpx_prms, appCntxt->vxContext);
            grpx_prms.draw_callback = SODAPP_drawGraphics;
            appGrpxInit(&grpx_prms);
#endif
            SODAPP_createDraw(appCntxt);
        }
    }
}



/*
 * Create color coded disparity imag
 * the RGB image for display
 */
void SODAPP_createDisparityCCImage(SODAPP_Context *appCntxt)
{
    vx_status status;
    vx_rectangle_t rect;
    vx_imagepatch_addressing_t src_image_addr;
    vx_imagepatch_addressing_t dst_image_addr;
    vx_map_id map_src_id;
    vx_map_id map_dst_id;
    uint16_t *src_ptr;
    uint8_t  *dst_ptr;

    uint16_t width  = appCntxt->width;
    uint16_t height = appCntxt->height;

    tivx_sde_disparity_vis_params_t sdeVizParams;


    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x = width;
    rect.end_y = height;

    // UYUV and RGB both have 1 plane
    status = vxMapImagePatch(appCntxt->vxOutDisparity16,
                             &rect,
                             0,
                             &map_src_id,
                             &src_image_addr,
                             (void **)&src_ptr,
                             VX_READ_ONLY,
                             VX_MEMORY_TYPE_HOST,
                             VX_NOGAP_X);
    if(status != VX_SUCCESS)
    {
        PTK_printf("[%s:%d] vxMapImagePatch() failed.\n", __FUNCTION__,  __LINE__);
    }

    status = vxMapImagePatch(appCntxt->vxDisparityCC,
                             &rect,
                             0,
                             &map_dst_id,
                             &dst_image_addr,
                             (void **)&dst_ptr,
                             VX_WRITE_ONLY,
                             VX_MEMORY_TYPE_HOST,
                             VX_NOGAP_X);
    if(status != VX_SUCCESS)
    {
        PTK_printf("[%s:%d] vxMapImagePatch() failed.\n", __FUNCTION__,  __LINE__);
    }

    sdeVizParams.disparity_min  = appCntxt->sde_params.disparity_min;
    sdeVizParams.disparity_max  = appCntxt->sde_params.disparity_max;
    sdeVizParams.disparity_only = 0;
    if (appCntxt->sdeAlgoType == 0)
    {
        sdeVizParams.vis_confidence = appCntxt->confidence_threshold;
    } else
    {
        sdeVizParams.vis_confidence = 0;
    }

    ptkdemo_visualizeSdeDisparity(
        &sdeVizParams,
        (int16_t *)src_ptr,
        (uint8_t *)dst_ptr,
        src_image_addr.dim_x,
        src_image_addr.dim_y,
        src_image_addr.stride_y/src_image_addr.stride_x,
        dst_image_addr.stride_y
    );


    vxUnmapImagePatch(appCntxt->vxOutDisparity16, map_src_id);
    vxUnmapImagePatch(appCntxt->vxDisparityCC, map_dst_id);
}


void SODAPP_overlayBBAndFreeSpace(SODAPP_Context *appCntxt)
{
    uint32_t                   i, j, value;
    vx_status                  status;
    vx_rectangle_t             rect;
    vx_imagepatch_addressing_t image_addr;
    vx_map_id                  map_id;
    vx_df_image                img_format;
    uint8_t                  * data_ptr;

    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x = appCntxt->width;
    rect.end_y = appCntxt->height;

    vxQueryImage(appCntxt->vxDispRightImage, VX_IMAGE_FORMAT, &img_format, sizeof(vx_df_image));

    status = vxMapImagePatch(appCntxt->vxDispRightImage,
                             &rect,
                             0,
                             &map_id,
                             &image_addr,
                             (void **)&data_ptr,
                             VX_READ_ONLY,
                             VX_MEMORY_TYPE_HOST,
                             VX_NOGAP_X);

    vxUnmapImagePatch(appCntxt->vxDispRightImage, map_id);

    if (status == VX_SUCCESS)
    {
        Draw2D_LinePrm sLinePrm;
        Draw2D_FontPrm sDistance;

        char     strDistance[100];
        uint16_t RGB_565_val;
        int16_t  x, y, midX, midY;

        if (img_format == VX_DF_IMAGE_UYVY)
        {
            for (j = 0; j < image_addr.dim_y; j++)
            {
                for (i = 0; i < appCntxt->width; i++)
                {
                    value = data_ptr[j * image_addr.stride_y + i*2 + 1];
                    appCntxt->pDisplayBuf565[j * appCntxt->width + i] = (uint16_t)RGB888_TO_RGB565(value, value, value);
                }
            }
        } else
        {
            for (j = 0; j < image_addr.dim_y; j++)
            {
                for (i = 0; i < appCntxt->width; i++)
                {
                    value = data_ptr[j * image_addr.stride_y + i];
                    appCntxt->pDisplayBuf565[j * appCntxt->width + i] = (uint16_t)RGB888_TO_RGB565(value, value, value);
                }
            }
        }

        // Draw freespace
        sLinePrm.lineColor = RGB888_TO_RGB565(0, 255, 0);
        sLinePrm.lineSize = 2;
        sLinePrm.lineColorFormat = DRAW2D_DF_BGR16_565;

        for (i = 0; i < appCntxt->width / 4; i++)
        {
            if (appCntxt->freespaceBoundary[i] != -1)
            {
                x = i * 4;
                y = appCntxt->freespaceBoundary[i] * 4;

                Draw2D_drawLine(appCntxt->pHndl,
                                x,
                                y,
                                x,
                                appCntxt->height - 1,
                                &sLinePrm);
            }
        }

        // Draw drivable space
        sLinePrm.lineColor = RGB888_TO_RGB565(255, 0, 0);
        sLinePrm.lineSize = 3;
        sLinePrm.lineColorFormat = DRAW2D_DF_BGR16_565;

        // P3 P4
        // P1 P2
        Draw2D_drawLine(appCntxt->pHndl,
                        appCntxt->drivableSpace.p1x,
                        appCntxt->drivableSpace.p1y,
                        appCntxt->drivableSpace.p2x,
                        appCntxt->drivableSpace.p2y,
                        &sLinePrm);

        Draw2D_drawLine(appCntxt->pHndl,
                        appCntxt->drivableSpace.p2x,
                        appCntxt->drivableSpace.p2y,
                        appCntxt->drivableSpace.p4x,
                        appCntxt->drivableSpace.p4y,
                        &sLinePrm);

        Draw2D_drawLine(appCntxt->pHndl,
                        appCntxt->drivableSpace.p4x,
                        appCntxt->drivableSpace.p4y,
                        appCntxt->drivableSpace.p3x,
                        appCntxt->drivableSpace.p3y,
                        &sLinePrm);

        Draw2D_drawLine(appCntxt->pHndl,
                        appCntxt->drivableSpace.p3x,
                        appCntxt->drivableSpace.p3y,
                        appCntxt->drivableSpace.p1x,
                        appCntxt->drivableSpace.p1y,
                        &sLinePrm);

        sDistance.fontIdx = 3;
        sprintf(strDistance, "%.1f m", float(appCntxt->drivableSpace.drivableDepth / 1000));
        Draw2D_drawString(appCntxt->pHndl, (appCntxt->drivableSpace.p3x), appCntxt->drivableSpace.p4y, (char *)strDistance, &sDistance);

        // draw bounding boxes
        sLinePrm.lineColor = RGB888_TO_RGB565(0, 0, 255);
        sLinePrm.lineSize = 3;
        sLinePrm.lineColorFormat = DRAW2D_DF_BGR16_565;

        sDistance.fontIdx = 3;

        for (i = 0; i < appCntxt->numObstacles; i++)
        {
            // P3 P4
            // P1 P2
            Draw2D_drawLine(appCntxt->pHndl,
                            appCntxt->obsBox[i].p1x,
                            appCntxt->obsBox[i].p1y,
                            appCntxt->obsBox[i].p2x,
                            appCntxt->obsBox[i].p2y,
                            &sLinePrm);

            Draw2D_drawLine(appCntxt->pHndl,
                            appCntxt->obsBox[i].p2x,
                            appCntxt->obsBox[i].p2y,
                            appCntxt->obsBox[i].p4x,
                            appCntxt->obsBox[i].p4y,
                            &sLinePrm);

            Draw2D_drawLine(appCntxt->pHndl,
                            appCntxt->obsBox[i].p4x,
                            appCntxt->obsBox[i].p4y,
                            appCntxt->obsBox[i].p3x,
                            appCntxt->obsBox[i].p3y,
                            &sLinePrm);

            Draw2D_drawLine(appCntxt->pHndl,
                            appCntxt->obsBox[i].p3x,
                            appCntxt->obsBox[i].p3y,
                            appCntxt->obsBox[i].p1x,
                            appCntxt->obsBox[i].p1y,
                            &sLinePrm);

            sprintf(strDistance, "%.1f m", float(appCntxt->obsBox[i].depth / 1000));
            midX = (appCntxt->obsBox[i].p1x + appCntxt->obsBox[i].p2x) / 2;
            midY = (appCntxt->obsBox[i].p1y + appCntxt->obsBox[i].p2y) / 2;
            Draw2D_drawString(appCntxt->pHndl,midX-15, midY-15, (char *)strDistance, &sDistance);
        }

        // get properties of vxInputBBImage
        status = vxMapImagePatch(appCntxt->vxInputBBImage,
                             &rect,
                             0,
                             &map_id,
                             &image_addr,
                             (void **)&data_ptr,
                             VX_WRITE_ONLY,
                             VX_MEMORY_TYPE_HOST,
                             VX_NOGAP_X);


        // copy RGB565 to RGB888
        for (j = 0; j < appCntxt->height; j++)
        {
            for (i = 0; i < appCntxt->width; i++)
            {
                RGB_565_val = appCntxt->pDisplayBuf565[j * appCntxt->width + i];

                data_ptr[0] = (RGB_565_val & 0x1F) << 3;
                data_ptr[1] = ((RGB_565_val >> 5) & 0x3F) << 2;
                data_ptr[2] = ((RGB_565_val >> 11) & 0x1F) << 3;

                data_ptr += 3;
            }

            data_ptr += (image_addr.stride_y - appCntxt->width*3);
        }

        vxUnmapImagePatch(appCntxt->vxInputBBImage, map_id);
    }
}


void SODAPP_run(SODAPP_Context *appCntxt)
{
    uint32_t  i;
    vx_status vxStatus = VX_SUCCESS;
    char      temp[SODAPP_MAX_LINE_LEN];

    SODAPP_graphParams* gpDesc;

    appCntxt->frameCnt = 0;
    for (i = appCntxt->start_fileno; i <= appCntxt->end_fileno; i++)
    {
        /* Wait for the data ready semaphore. */
        if (appCntxt->dataReadySem)
        {
            appCntxt->dataReadySem->wait();
        }

#if defined(PC) 
        PTK_printf("Processing frame %d...\n", i);
#endif

        vxStatus = SODAPP_getFreeParamRsrc(appCntxt, &gpDesc);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] SODAPP_getFreeParamRsrc() failed\n",
                        __FUNCTION__, __LINE__);
        }

        // left image
        strcpy(appCntxt->left_img_file_name, appCntxt->left_img_file_path);
        if (appCntxt->inputFormat == PTK_IMG_FORMAT_Y)
        {
            sprintf(temp, "/%010d.bmp", i);
            if (strlen(appCntxt->left_img_file_name) + strlen(temp) >= SODAPP_MAX_LINE_LEN)
            {
                vxStatus = (vx_status)VX_FAILURE;
            } else
            {
                strcat(appCntxt->left_img_file_name, temp);
                vxStatus = tivx_utils_load_vximage_from_bmpfile(gpDesc->vxInputLeftImage, appCntxt->left_img_file_name, vx_true_e);
            }

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] tivx_utils_load_vximage_from_bmpfile() failed\n",
                            __FUNCTION__, __LINE__);
            }
        } else
        {
            sprintf(temp, "/%010d.yuv", i);
            if (strlen(appCntxt->left_img_file_name) + strlen(temp) >= SODAPP_MAX_LINE_LEN)
            {
                vxStatus = (vx_status)VX_FAILURE;
            } else
            {
                strcat(appCntxt->left_img_file_name, temp);
                vxStatus = ptkdemo_load_vximage_from_yuvfile(gpDesc->vxInputLeftImage, appCntxt->left_img_file_name);
            }

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] ptkdemo_load_vximage_from_yuvfile() failed\n",
                            __FUNCTION__, __LINE__);
            }
        }

        // rigth image
        strcpy(appCntxt->right_img_file_name, appCntxt->right_img_file_path);
        if (appCntxt->inputFormat == PTK_IMG_FORMAT_Y)
        {
            sprintf(temp, "/%010d.bmp", i);
            if (strlen(appCntxt->right_img_file_name) + strlen(temp) >= SODAPP_MAX_LINE_LEN)
            {
                vxStatus = (vx_status)VX_FAILURE;
            } else
            {
                strcat(appCntxt->right_img_file_name, temp);
                vxStatus = tivx_utils_load_vximage_from_bmpfile(gpDesc->vxInputRightImage, appCntxt->right_img_file_name, vx_true_e);
            }

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] tivx_utils_load_vximage_from_bmpfile() failed\n",
                            __FUNCTION__, __LINE__);
            }
        } else
        {
            sprintf(temp, "/%010d.yuv", i);
            if (strlen(appCntxt->right_img_file_name) + strlen(temp) >= SODAPP_MAX_LINE_LEN)
            {
                vxStatus = (vx_status)VX_FAILURE;
            } else
            {
                strcat(appCntxt->right_img_file_name, temp);
                vxStatus = ptkdemo_load_vximage_from_yuvfile(gpDesc->vxInputRightImage, appCntxt->right_img_file_name);
            }

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] ptkdemo_load_vximage_from_yuvfile() failed\n",
                            __FUNCTION__, __LINE__);
            }
        }

        // read input images and disparity maps,
        // then run SDE applib
        vxStatus = SODAPP_process(appCntxt, gpDesc);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] SODAPP_process() failed\n",
                        __FUNCTION__, __LINE__);
        }

        appCntxt->frameCnt++;
    }

    if (vxStatus == (vx_status)VX_SUCCESS) 
    {
        /* Wait for the graph to consume all input. */
        SODAPP_waitGraph(appCntxt);
    }

    PTK_printf("\e[KProcessed %d frames.\n\e[A", appCntxt->frameCnt);
}

void SODAPP_deInit(SODAPP_Context *appCntxt)
{
    int32_t i, status;

    // free memories
    if (NULL != appCntxt->obsBox)
    {
        free(appCntxt->obsBox);
    }

    if (NULL != appCntxt->freespaceBoundary)
    {
        free(appCntxt->freespaceBoundary);
    }

    if (appCntxt->dataReadySem)
    {
        delete appCntxt->dataReadySem;
    }

    // release input image object
    vxReleaseImage(&appCntxt->vxLeftRectImage[0]);
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        vxReleaseImage(&appCntxt->vxInputLeftImage[i]);
        vxReleaseImage(&appCntxt->vxInputRightImage[i]);
        vxReleaseImage(&appCntxt->vxRightRectImage[i]);
    }

    // release dispairty object
    if (appCntxt->sdeAlgoType == 0)
    {
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            vxReleaseImage(&appCntxt->vxSde16BitOutput[i]);
        }
    } else
    {
#if ENABLE_PP_MEDIAN_FILTER
        if (appCntxt->ppMedianFilterEnable)
        {
            for (i = 0; i < appCntxt->pipelineDepth; i++)
            {
                vxReleaseImage(&appCntxt->vxMedianFilteredDisparity[i]);
            }
        }
#endif

        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            vxReleaseImage(&appCntxt->vxMergeDisparityL0[i]);
        }
    }

    // release detection outputs
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        vxReleaseArray(&appCntxt->vxObstaclesPose[i]);
        vxReleaseScalar(&appCntxt->vxNumObstacles[i]);
        vxReleaseArray(&appCntxt->vxFreeSpaceBoundary[i]);
        vxReleaseUserDataObject(&appCntxt->vxDrivableSpace[i]);
    }

    vxReleaseGraph(&appCntxt->vxGraph);
    tivxStereoUnLoadKernels(appCntxt->vxContext);
    tivxHwaUnLoadKernels(appCntxt->vxContext);

    if (1 == appCntxt->display_option)
    {
        tivxMemFree(appCntxt->pDisplayBuf565, appCntxt->width * appCntxt->height * sizeof(uint16_t), TIVX_MEM_EXTERNAL);
        Draw2D_delete(appCntxt->pHndl);
#if !defined(PC)
        appGrpxDeInit();
#endif
    }

    /* Release the context. */
    vxReleaseContext(&appCntxt->vxContext);

    status = appDeInit();
    PTK_assert(status == 0);
}

void SODAPP_createDispalyGraph(SODAPP_Context *appCntxt)
{
    int32_t status;

    // create graph
    appCntxt->vxDispGraph = vxCreateGraph(appCntxt->vxContext);
    APP_ASSERT_VALID_REF(appCntxt->vxDispGraph);
    vxSetReferenceName((vx_reference)appCntxt->vxDispGraph, "Display");

    // create objects
    appCntxt->vxInputBBImage = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_RGB);
    APP_ASSERT_VALID_REF(appCntxt->vxInputBBImage);
    vxSetReferenceName((vx_reference)appCntxt->vxInputBBImage, "InputImage_BoundingBox_U8");

    appCntxt->vxDisparityCC = vxCreateImage(appCntxt->vxContext, appCntxt->width, appCntxt->height, VX_DF_IMAGE_RGB);
    APP_ASSERT_VALID_REF(appCntxt->vxDisparityCC);
    vxSetReferenceName((vx_reference)appCntxt->vxDisparityCC, "Stereo_Disparity_CC");


    if ((vx_true_e == tivxIsTargetEnabled(TIVX_TARGET_DISPLAY1)) && (1 == appCntxt->display_option))
    {
#if !defined(PC)
        memset(&appCntxt->disparity_display_params, 0, sizeof(tivx_display_params_t));
        appCntxt->disparity_display_config = vxCreateUserDataObject(appCntxt->vxContext, "tivx_display_params_t",
                                                                    sizeof(tivx_display_params_t), NULL);
        APP_ASSERT_VALID_REF(appCntxt->disparity_display_config);

        vxSetReferenceName((vx_reference)appCntxt->disparity_display_config, "DisparityDisplayConfiguration");

        appCntxt->disparity_display_params.opMode = TIVX_KERNEL_DISPLAY_ZERO_BUFFER_COPY_MODE;
        appCntxt->disparity_display_params.pipeId = 2;
        appCntxt->disparity_display_params.outWidth  = OUTPUT_DISPLAY_WIDTH;
        appCntxt->disparity_display_params.outHeight = OUTPUT_DISPLAY_HEIGHT;
        appCntxt->disparity_display_params.posX = 960;
        appCntxt->disparity_display_params.posY = 300;

        status = vxCopyUserDataObject(appCntxt->disparity_display_config, 0, sizeof(tivx_display_params_t), &appCntxt->disparity_display_params, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);
        APP_ASSERT(status == VX_SUCCESS);

        // create disparity display node
        appCntxt->node_disparity_display = tivxDisplayNode(
            appCntxt->vxDispGraph,
            appCntxt->disparity_display_config,
            appCntxt->vxDisparityCC);
        status = vxGetStatus((vx_reference)appCntxt->node_disparity_display);
        if (VX_SUCCESS != status)
        {
            PTK_printf("# ERROR: Display is not enabled on this platform, please disable it in config \n");
        }
        APP_ASSERT(VX_SUCCESS == status);
        status = vxSetNodeTarget(appCntxt->node_disparity_display, VX_TARGET_STRING, TIVX_TARGET_DISPLAY1);
        APP_ASSERT(status == VX_SUCCESS);
        vxSetReferenceName((vx_reference)appCntxt->node_disparity_display, "DisparityDisplay");
#endif
    }

    if ((vx_true_e == tivxIsTargetEnabled(TIVX_TARGET_DISPLAY2)) && (1 == appCntxt->display_option))
    {
#if !defined(PC)
        memset(&appCntxt->image_display_params, 0, sizeof(tivx_display_params_t));
        appCntxt->image_display_config = vxCreateUserDataObject(appCntxt->vxContext, "tivx_display_params_t",
                                                                sizeof(tivx_display_params_t), NULL);
        APP_ASSERT_VALID_REF(appCntxt->image_display_config);

        vxSetReferenceName((vx_reference)appCntxt->image_display_config, "ImageDisplayConfiguration");

        appCntxt->image_display_params.opMode = TIVX_KERNEL_DISPLAY_ZERO_BUFFER_COPY_MODE;
        appCntxt->image_display_params.pipeId = 0;
        appCntxt->image_display_params.outWidth = INPUT_DISPLAY_WIDTH;
        appCntxt->image_display_params.outHeight = INPUT_DISPLAY_HEIGHT;
        appCntxt->image_display_params.posX = 0;
        appCntxt->image_display_params.posY = 300;

        status = vxCopyUserDataObject(appCntxt->image_display_config, 0, sizeof(tivx_display_params_t), &appCntxt->image_display_params, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);
        APP_ASSERT(status == VX_SUCCESS);

        // create disparity display node
        appCntxt->node_image_display = tivxDisplayNode(
            appCntxt->vxDispGraph,
            appCntxt->image_display_config,
            appCntxt->vxInputBBImage);
        status = vxGetStatus((vx_reference)appCntxt->node_image_display);
        if (VX_SUCCESS != status)
        {
            PTK_printf("# ERROR: Display is not enabled on this platform, please disable it in config \n");
        }
        APP_ASSERT(VX_SUCCESS == status);
        status = vxSetNodeTarget(appCntxt->node_image_display, VX_TARGET_STRING, TIVX_TARGET_DISPLAY2);
        APP_ASSERT(status == VX_SUCCESS);
        vxSetReferenceName((vx_reference)appCntxt->node_image_display, "ImageDisplay");
#endif
    }

    status = vxVerifyGraph(appCntxt->vxDispGraph);
    APP_ASSERT(status == VX_SUCCESS);
}

void SODAPP_deleteDisplayGraph(SODAPP_Context *appCntxt)
{
    vxReleaseImage(&appCntxt->vxInputBBImage);
    vxReleaseImage(&appCntxt->vxDisparityCC);

#if !defined(PC)
    if ((vx_true_e == tivxIsTargetEnabled(TIVX_TARGET_DISPLAY1)) && (1 == appCntxt->display_option))
    {
        vxReleaseUserDataObject(&appCntxt->disparity_display_config);
    }
    if ((vx_true_e == tivxIsTargetEnabled(TIVX_TARGET_DISPLAY2)) && (1 == appCntxt->display_option))
    {
        vxReleaseUserDataObject(&appCntxt->image_display_config);
    }
#endif

    vxReleaseGraph(&appCntxt->vxDispGraph);
}

static void SODAPP_exitProcThreads(SODAPP_Context *appCntxt,
                                   bool            detach)
{
    vx_status vxStatus;

    appCntxt->exitInputDataProcess = true;

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
                               SODAPP_USER_EVT_EXIT,
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

void SODAPP_cleanupHdlr(SODAPP_Context *appCntxt)
{
    /* Wait for the threads to exit. */
    SODAPP_exitProcThreads(appCntxt, false);

    PTK_printf("\nPress ENTER key to exit.\n");
    fflush(stdout);
    getchar();

    if (appCntxt->rtLogEnable == 1)
    {
        char name[256];

        snprintf(name, 255, "%s.bin", APP_SDE_OBSTACLE_DETECT_NAME);
        tivxLogRtTraceExportToFile(name);
    }

    /* Release the objects. */
    SDELDCAPPLIB_delete(&appCntxt->sdeLdcHdl);

    if (appCntxt->sdeAlgoType == 0)
    {
        SL_SDEAPPLIB_delete(&appCntxt->slSdeHdl);
    }
    else if (appCntxt->sdeAlgoType == 1)
    {
        ML_SDEAPPLIB_delete(&appCntxt->mlSdeHdl);
    }

    SODAPPLIB_delete(&appCntxt->sodHdl);

    /* De-initialize the Application context. */
    SODAPP_deleteDisplayGraph(appCntxt);

    SODAPP_deInit(appCntxt);

    PTK_printf("[%s] Clean-up complete.\n", __FUNCTION__);
}

static void SODAPP_reset(SODAPP_Context * appCntxt)
{
    vx_status vxStatus;

    vx_node geNode = SODAPPLIB_getGENode(appCntxt->sodHdl);
    vxStatus = tivxNodeSendCommand(geNode,
                                   0,
                                   TIVX_KERNEL_GROUND_ESTIMATION_RESET,
                                   NULL,
                                   0);
    if (vxStatus != VX_SUCCESS)
    {
        PTK_printf("[%s:%d] tivxNodeSendCommand() failed.\n",
                   __FUNCTION__,
                   __LINE__);

    }

    /* Reset the performance capture initialization flag. */
    appCntxt->startPerfCapt = false; 
}

static void SODAPP_inputDataThread(SODAPP_Context *appCntxt)
{
    PTK_printf("[%s] Launched input data processing thread.\n", __FUNCTION__);

    // init display frame no
    appCntxt->displayFrmNo    = appCntxt->start_fileno;
    appCntxt->processFinished = false;

    while (true)
    {
        /* Reset ground model */
        SODAPP_reset(appCntxt);

        /* Execute the graph. */
        SODAPP_run(appCntxt);

        if (appCntxt->exitInputDataProcess)
        {
            break;
        }

        // reinit frame no to be displayed
        appCntxt->displayFrmNo = appCntxt->start_fileno;
    }

    appCntxt->processFinished = true;
}


static void SODAPP_evtHdlrThread(SODAPP_Context *appCntxt)
{
    vx_event_t evt;
    vx_status vxStatus;
    
    int32_t status;

#if defined(PC)
    char output_file_name_disparity_img[SODAPP_MAX_LINE_LEN];
    char output_file_name_input_img[SODAPP_MAX_LINE_LEN];
    char output_file_name_bb_img[SODAPP_MAX_LINE_LEN]; 
#endif

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
                if (evt.app_value == SODAPP_USER_EVT_EXIT)
                {
                    break;
                }
            }

            SODAPP_processEvent(appCntxt, &evt);

            if (evt.type == VX_EVENT_GRAPH_COMPLETED)
            {
                SODAPP_getOutBuff(appCntxt, &appCntxt->numObstacles, &appCntxt->vxDispRightImage, &appCntxt->vxOutDisparity16,
                                  appCntxt->obsBox, appCntxt->freespaceBoundary, &appCntxt->drivableSpace);

                if (1 == appCntxt->display_option)
                {
                    SODAPP_createDisparityCCImage(appCntxt);
                    SODAPP_overlayBBAndFreeSpace(appCntxt);
                }

#if !defined(PC)
                status = vxScheduleGraph(appCntxt->vxDispGraph);
                PTK_assert(VX_SUCCESS == status);
                status = vxWaitGraph(appCntxt->vxDispGraph);
#endif

                SODAPP_releaseOutBuff(appCntxt);

                /* Wakeup the input data thread. */
                if (appCntxt->dataReadySem)
                {
                    appCntxt->dataReadySem->notify();
                }


#if defined(PC)
                if (1 == appCntxt->display_option)
                {
                    // save output
                    if (appCntxt->inputFormat == PTK_IMG_FORMAT_Y)
                    {
                        snprintf(output_file_name_input_img, SODAPP_MAX_LINE_LEN, "%s/input_%05d.bmp",
                                 appCntxt->output_file_path,
                                 appCntxt->displayFrmNo);
                        tivx_utils_save_vximage_to_bmpfile(output_file_name_input_img, appCntxt->vxDispRightImage);
                    } else
                    {
                        snprintf(output_file_name_input_img, SODAPP_MAX_LINE_LEN, "%s/input_%05d.yuv",
                                 appCntxt->output_file_path,
                                 appCntxt->displayFrmNo);
                        ptkdemo_save_vximage_to_yuvfile(appCntxt->vxDispRightImage, output_file_name_input_img);
                    }

                    snprintf(output_file_name_disparity_img, SODAPP_MAX_LINE_LEN, "%s/disp_%05d.bmp",
                             appCntxt->output_file_path,
                             appCntxt->displayFrmNo);
    
                    snprintf(output_file_name_bb_img, SODAPP_MAX_LINE_LEN, "%s/bb_%05d.bmp",
                             appCntxt->output_file_path,
                             appCntxt->displayFrmNo);

                    tivx_utils_save_vximage_to_bmpfile(output_file_name_disparity_img, appCntxt->vxDisparityCC);
                    tivx_utils_save_vximage_to_bmpfile(output_file_name_bb_img, appCntxt->vxInputBBImage);
                }
#endif
                // In interaction mode, sometimes diplayFramNo is updated after it was reset to
                // the starting frame in input data thread (if writing output images takes too long).
                // In this case, the first frame number is "starting frame no + 1"
                // It only affects the saved file name index.
                appCntxt->displayFrmNo++;
            }
        }

    } // while (true)
}

void SODAPP_launchProcThreads(SODAPP_Context *appCntxt)
{
    /* Launch the input data thread. */
    appCntxt->inputDataThread = std::thread(SODAPP_inputDataThread, appCntxt);

    /* Launch the event handler thread. */
    appCntxt->evtHdlrThread = std::thread(SODAPP_evtHdlrThread, appCntxt);
}

void SODAPP_intSigHandler(SODAPP_Context *appCntxt, int sig)
{
    // wait for frame processing to finish
    appCntxt->end_fileno = 0;
    appCntxt->exitInputDataProcess = true;

    while(!appCntxt->processFinished)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    /* Wait for the threads to exit. */
    SODAPP_exitProcThreads(appCntxt, true);

    /* Release objects. */
    SDELDCAPPLIB_delete(&appCntxt->sdeLdcHdl);

    if (appCntxt->sdeAlgoType == 0)
    {
        SL_SDEAPPLIB_delete(&appCntxt->slSdeHdl);
    }
    else if (appCntxt->sdeAlgoType == 1)
    {
        ML_SDEAPPLIB_delete(&appCntxt->mlSdeHdl);
    }

    /* Release the Application context. */
    SODAPPLIB_delete(&appCntxt->sodHdl);

    /* Delete display graph */
    SODAPP_deleteDisplayGraph(appCntxt);

    /* De-initialize the Application context. */
    SODAPP_deInit(appCntxt);

    PTK_printf("[%s] Clean-up complete.\n", __FUNCTION__);

    exit(0);
}
