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

#include <app_ptk_demo_common.h>
#include <app_ptk_demo_disparity.h>

#include "app_sde_main.h"
#include "app_sde.h"

#define APP_SDE_NAME    "apps_sde"


#if !defined(PC)
static void SDEAPP_drawGraphics(Draw2D_Handle *handle, Draw2D_BufInfo *draw2dBufInfo, uint32_t update_type)
{
    appGrpxDrawDefault(handle, draw2dBufInfo, update_type);

    return;
}
#endif

void SDEAPP_parseCfgFile(SDEAPP_Context *appCntxt, const char *cfg_file_name)
{
    FILE      * fp = fopen(cfg_file_name, "r");
    char      * pParamStr;
    char      * pValueStr;
    char      * pSLine;
    char      * basePath;
    char        paramSt[SDEAPP_MAX_LINE_LEN];
    char        valueSt[SDEAPP_MAX_LINE_LEN];
    char        sLine[SDEAPP_MAX_LINE_LEN];

    // set default parameters
    appCntxt->display_option       = 0;
    appCntxt->width[0]             = 1280;
    appCntxt->height[0]            = 720;
    appCntxt->inputFormat          = 0;
    appCntxt->is_interactive       = 0;

    appCntxt->sdeAlgoType          = 0;
    appCntxt->numLayers            = 2;
    appCntxt->ppMedianFilterEnable = 0;
    appCntxt->confidence_threshold = 0;

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

    appCntxt->pipelineDepth                          = SDEAPP_MAX_PIPELINE_DEPTH;
    appCntxt->exportGraph                            = 0;
    appCntxt->rtLogEnable                            = 0;

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

    while (1)
    {
        pSLine = fgets(pSLine, SDEAPP_MAX_LINE_LEN, fp);

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

        if (strcmp(pParamStr, "left_img_file_path") == 0)
        {
            snprintf(appCntxt->left_img_file_path, SDEAPP_MAX_LINE_LEN,
                 "%s/%s", basePath, pValueStr);
        }
        if (strcmp(pParamStr, "right_img_file_path") == 0)
        {
            snprintf(appCntxt->right_img_file_path, SDEAPP_MAX_LINE_LEN,
                 "%s/%s", basePath, pValueStr);
        }
        else if (strcmp(pParamStr, "output_file_path") == 0)
        {
            snprintf(appCntxt->output_file_path, SDEAPP_MAX_LINE_LEN,
                 "%s/%s", basePath, pValueStr);
        }
        if (strcmp(pParamStr, "left_LUT_file_name") == 0)
        {
            snprintf(appCntxt->left_LUT_file_name, SDEAPP_MAX_LINE_LEN,
                 "%s/%s", basePath, pValueStr);
        }
        if (strcmp(pParamStr, "right_LUT_file_name") == 0)
        {
            snprintf(appCntxt->right_LUT_file_name, SDEAPP_MAX_LINE_LEN,
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
        else if (strcmp(pParamStr, "width") == 0)
        {
            appCntxt->width[0] = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "height") == 0)
        {
            appCntxt->height[0] = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "sde_algo_type") == 0)
        {
            appCntxt->sdeAlgoType = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "num_layers") == 0)
        {
            appCntxt->numLayers = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "pp_median_filter_enable") == 0)
        {
            appCntxt->ppMedianFilterEnable = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "sde_confidence_threshold") == 0)
        {
            appCntxt->confidence_threshold = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "display_option") == 0)
        {
            appCntxt->display_option = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "is_interactive") == 0)
        {
            appCntxt->is_interactive = atoi(pValueStr);
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
            appCntxt->pipelineDepth = atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "exportGraph") == 0)
        {
            appCntxt->exportGraph = (uint8_t)atoi(pValueStr);
        }
        else if (strcmp(pParamStr, "rtLogEnable") == 0)
        {
            appCntxt->rtLogEnable = (uint8_t)atoi(pValueStr);
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
    }

    fclose(fp);

    if (appCntxt->width[0] < 128)
    {
        appCntxt->width[0] = 128;
    }
    if (appCntxt->height[0] < 128)
    {
        appCntxt->height[0] = 128;
    }
    if (appCntxt->end_fileno < appCntxt->start_fileno)
    {
        appCntxt->end_fileno = appCntxt->start_fileno;
    }

    // when multi-layer SDE is used
    if (appCntxt->sdeAlgoType == 1)
    {
        if (appCntxt->numLayers < 2 || appCntxt->numLayers > 3)
        {
            PTK_printf("The number of layer should be 2 or 3. Set the number of layers to 2...\n");
            appCntxt->numLayers = 2;
        }

        int8_t  i;
        int8_t  factor = (appCntxt->numLayers - 1) * 2;

        if (appCntxt->height[0] % factor != 0 || appCntxt->width[0] % factor != 0)
        {
            PTK_printf("Improper stereo image resolution...\n");
            exit(0);
        }

        for (i = 1; i < appCntxt->numLayers; i++)
        {
            appCntxt->width[i]  = appCntxt->width[i-1]/2;
            appCntxt->height[i] = appCntxt->height[i-1]/2;

            if (appCntxt->width[i] % 16 != 0)
            {
                PTK_printf("Improper image width is not multiple of 16...\n");
                exit(0);
            }
        }
    }

    appCntxt->renderPeriod = 0;

    return;

} /* SDEAPP_parseCfgFile */


void SDEAPP_setLDCCreateParams(SDEAPP_Context *appCntxt)
{
    SDELDCAPPLIB_createParams * createParams = &appCntxt->sdeLdcCreateParams;

    createParams->leftLutFileName  = appCntxt->left_LUT_file_name;
    createParams->rightLutFileName = appCntxt->right_LUT_file_name;

    createParams->width            = appCntxt->width[0];
    createParams->height           = appCntxt->height[0];
    createParams->inputFormat      = appCntxt->inputFormat;
    createParams->pipelineDepth    = appCntxt->pipelineDepth;
}


void SDEAPP_setSLSdeCreateParams(SDEAPP_Context *appCntxt)
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

    createParams->width            = appCntxt->width[0];
    createParams->height           = appCntxt->height[0];
    createParams->inputFormat      = appCntxt->inputFormat;
    createParams->vis_confidence   = appCntxt->confidence_threshold;
    createParams->pipelineDepth    = appCntxt->pipelineDepth;
}


void SDEAPP_setMLSdeCreateParams(SDEAPP_Context *appCntxt)
{
    ML_SDEAPPLIB_createParams * createParams = &appCntxt->mlSdeCreateParams;

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
    createParams->width              = appCntxt->width[0];
    createParams->height             = appCntxt->height[0];
    createParams->pipelineDepth      = appCntxt->pipelineDepth;
}

void SDEAPP_setAllParams(SDEAPP_Context *appCntxt)
{
    appCntxt->vxEvtAppValBase = 0;

    SDEAPP_setLDCCreateParams(appCntxt);

    /* sinle-layer SDE */
    if (appCntxt->sdeAlgoType == 0)
    {
        SDEAPP_setSLSdeCreateParams(appCntxt);
    }
    else if (appCntxt->sdeAlgoType == 1)
    {
        SDEAPP_setMLSdeCreateParams(appCntxt);
    }
} /* SDEAPP_setAllParams */


int32_t SDEAPP_createDraw(SDEAPP_Context *appCntxt)
{
    int32_t status = 0;

    appCntxt->pDisplayBuf565 = (uint16_t *)tivxMemAlloc(appCntxt->width[0] * appCntxt->height[0] * sizeof(uint16_t), TIVX_MEM_EXTERNAL);
    PTK_assert(NULL != appCntxt->pDisplayBuf565);

    return status;
}


int32_t SDEAPP_init(SDEAPP_Context *appCntxt)
{
    int32_t                     status;
    vx_status                   vxStatus;

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
        vxSetReferenceName((vx_reference)appCntxt->vxGraph, "SDE Graph");
    }

    if (status >= 0)
    {
        tivxStereoLoadKernels(appCntxt->vxContext);
        tivxHwaLoadKernels(appCntxt->vxContext);

        /*
         * 1 Setup Stereo LDC nodes
         */
        status = SDEAPP_init_LDC(appCntxt);
    }

    /*
     * 2 Setup SDE nodes
     */
    if (status >= 0)
    {
        status = SDEAPP_init_SDE(appCntxt);
    }

    if (status >= 0)
    {
        appPerfPointSetName(&appCntxt->sdePerf , "Stereo GRAPH");

        /*
         * set up the pipeline. 
         */
        vxStatus = SDEAPP_setupPipeline(appCntxt);

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
    }

    if (status >= 0)
    {
        // init scaler for ML SDE
        if (appCntxt->sdeAlgoType == 1)
        {
            ML_SDEAPPLIB_initScaler(appCntxt->mlSdeHdl);
        }

        if (appCntxt->exportGraph == 1)
        {
            tivxExportGraphToDot(appCntxt->vxGraph, ".", "vx_app_sde");
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
            grpx_prms.draw_callback = SDEAPP_drawGraphics;
            appGrpxInit(&grpx_prms);
#endif
            SDEAPP_createDraw(appCntxt);
        }
    }

    return status;
}


void SDEAPP_run(SDEAPP_Context *appCntxt)
{
    uint32_t  i;
    vx_status vxStatus = VX_SUCCESS;
    char      temp[SDEAPP_MAX_LINE_LEN];

    SDEAPP_graphParams* gpDesc;

    appCntxt->frameCnt = 0;

    /* Process all frames */
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

        vxStatus = SDEAPP_getFreeParamRsrc(appCntxt, &gpDesc);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] SDEAPP_getFreeParamRsrc() failed\n",
                        __FUNCTION__, __LINE__);
        }

        // left image
        strcpy(appCntxt->left_img_file_name, appCntxt->left_img_file_path);
        if (appCntxt->inputFormat == PTK_IMG_FORMAT_Y)
        {
            sprintf(temp, "/%010d.bmp", i);
            if (strlen(appCntxt->left_img_file_name) + strlen(temp) >= SDEAPP_MAX_LINE_LEN)
            {
                vxStatus = (vx_status)VX_FAILURE;
            } else
            {
                strcat(appCntxt->left_img_file_name, temp);
                vxStatus = tivx_utils_load_vximage_from_bmpfile(gpDesc->vxInputLeftImage,  appCntxt->left_img_file_name, vx_true_e);
            }

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] tivx_utils_load_vximage_from_bmpfile() failed\n",
                            __FUNCTION__, __LINE__);
            }
        } else
        {
            sprintf(temp, "/%010d.yuv", i);
            if (strlen(appCntxt->left_img_file_name) + strlen(temp) >= SDEAPP_MAX_LINE_LEN)
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
            if (strlen(appCntxt->right_img_file_name) + strlen(temp) >= SDEAPP_MAX_LINE_LEN)
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
            if (strlen(appCntxt->right_img_file_name) + strlen(temp) >= SDEAPP_MAX_LINE_LEN)
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

        // run the app
        vxStatus = SDEAPP_process(appCntxt, gpDesc);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] SDEAPP_process() failed\n",
                        __FUNCTION__, __LINE__);
        }

        appCntxt->frameCnt++;
    }

    if (vxStatus == (vx_status)VX_SUCCESS) 
    {
        /* Wait for the graph to consume all input. */
        SDEAPP_waitGraph(appCntxt);
    }

    PTK_printf("\e[KProcessed %d frames.\n\e[A", appCntxt->frameCnt);
}

void SDEAPP_deInit(SDEAPP_Context *appCntxt)
{
    int32_t  i, status;

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
        if (appCntxt->ppMedianFilterEnable)
        {
            for (i = 0; i < appCntxt->pipelineDepth; i++)
            {
                vxReleaseImage(&appCntxt->vxMedianFilteredDisparity[i]);
            }
        }

        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            vxReleaseImage(&appCntxt->vxMergeDisparityL0[i]);
        }
    }

    if (appCntxt->rtLogEnable == 1)
    {
        tivxLogRtTraceDisable(appCntxt->vxGraph);
    }

    vxReleaseGraph(&appCntxt->vxGraph);
    tivxStereoUnLoadKernels(appCntxt->vxContext);
    tivxHwaUnLoadKernels(appCntxt->vxContext);

    if (1 == appCntxt->display_option)
    {
        tivxMemFree(appCntxt->pDisplayBuf565, appCntxt->width[0] * appCntxt->height[0] * sizeof(uint16_t), TIVX_MEM_EXTERNAL);
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

void SDEAPP_createDispalyGraph(SDEAPP_Context *appCntxt)
{
    int32_t status;

    // create graph
    appCntxt->vxDispGraph = vxCreateGraph(appCntxt->vxContext);
    APP_ASSERT_VALID_REF(appCntxt->vxDispGraph);
    vxSetReferenceName((vx_reference)appCntxt->vxDispGraph, "Display");

    // create objects
    if (appCntxt->inputFormat == PTK_IMG_FORMAT_Y)
    {
        appCntxt->vxInputDisplayImage = vxCreateImage(appCntxt->vxContext, appCntxt->width[0], appCntxt->height[0], VX_DF_IMAGE_U8);
    } else
    {
        appCntxt->vxInputDisplayImage = vxCreateImage(appCntxt->vxContext, appCntxt->width[0], appCntxt->height[0], VX_DF_IMAGE_NV12);
    }
    APP_ASSERT_VALID_REF(appCntxt->vxInputDisplayImage);
    vxSetReferenceName((vx_reference)appCntxt->vxInputDisplayImage, "InputImage_ToDisplay");

    appCntxt->vxDisparityCC = vxCreateImage(appCntxt->vxContext, appCntxt->width[0], appCntxt->height[0], VX_DF_IMAGE_RGB);
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
        appCntxt->disparity_display_params.outWidth = OUTPUT_DISPLAY_WIDTH;
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
            appCntxt->vxInputDisplayImage);
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

#if 0
    status = tivxExportGraphToDot(appCntxt->vxDispGraph,".", "vx_app_ml_sde");
    APP_ASSERT(status==VX_SUCCESS);
#endif

}


/* 
 * Create color coded disparity imag
 * the RGB image for display
 */
void SDEAPP_createDisparityCCImage(SDEAPP_Context *appCntxt)
{
    vx_status status;
    vx_rectangle_t rect;
    vx_imagepatch_addressing_t src_image_addr;
    vx_imagepatch_addressing_t dst_image_addr;
    vx_map_id map_src_id;
    vx_map_id map_dst_id;
    uint16_t *src_ptr;
    uint8_t  *dst_ptr;

    uint16_t width  = appCntxt->width[0];
    uint16_t height = appCntxt->height[0];

    tivx_sde_disparity_vis_params_t sdeVizParams;


    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x = width;
    rect.end_y = height;

    // UYUV and RGB both have 1 plane
    status = vxMapImagePatch(appCntxt->vxDisparity16,
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


    vxUnmapImagePatch(appCntxt->vxDisparity16, map_src_id);
    vxUnmapImagePatch(appCntxt->vxDisparityCC, map_dst_id);
}


/* It coule be used to overlay something on an image. 
 * But it does not overlay anything for now. It simply creates
 * the RGB image for display
 */
void SDEAPP_createDisplayImage(SDEAPP_Context *appCntxt)
{
    vx_status                  status;
    vx_rectangle_t             rect;
    vx_imagepatch_addressing_t image_addr;
    vx_map_id                  map_id;
    vx_df_image                img_format;
    uint16_t                 * data_ptr;

    uint16_t width  = appCntxt->width[0];
    uint16_t height = appCntxt->height[0];

    vxQueryImage(appCntxt->vxInputImage, VX_IMAGE_FORMAT, &img_format, sizeof(vx_df_image));

    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x = width;
    rect.end_y = height;

    // UYUV and RGB both have 1 plane
    status = vxMapImagePatch(appCntxt->vxInputImage,
                             &rect,
                             0,
                             &map_id,
                             &image_addr,
                             (void **)&data_ptr,
                             VX_READ_ONLY,
                             VX_MEMORY_TYPE_HOST,
                             VX_NOGAP_X);
    if(status != VX_SUCCESS)
    {
        PTK_printf("[%s:%d] vxMapImagePatch() failed.\n", __FUNCTION__,  __LINE__);
    }

    if (status == VX_SUCCESS)
    {
        vxUnmapImagePatch(appCntxt->vxInputImage, map_id);

        /* copy to image to be display */
        status = vxCopyImagePatch(appCntxt->vxInputDisplayImage, &rect, 0, &image_addr, data_ptr, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);
        if(status != VX_SUCCESS)
        {
            PTK_printf("[%s:%d] vxCopyImagePatch() failed.\n", __FUNCTION__,  __LINE__);
        }
    }

    if (status == VX_SUCCESS && img_format == VX_DF_IMAGE_NV12)
    {
        status = vxMapImagePatch(appCntxt->vxInputImage,
                                 &rect,
                                 1,
                                 &map_id,
                                 &image_addr,
                                 (void **)&data_ptr,
                                 VX_READ_ONLY,
                                 VX_MEMORY_TYPE_HOST,
                                 VX_NOGAP_X);
        if(status != VX_SUCCESS)
        {
            PTK_printf("[%s:%d] vxMapImagePatch() failed.\n", __FUNCTION__,  __LINE__);
        }

        if (status == VX_SUCCESS)
        {
            vxUnmapImagePatch(appCntxt->vxInputImage, map_id);
            status = vxCopyImagePatch(appCntxt->vxInputDisplayImage, &rect, 1, &image_addr, data_ptr, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);
            if(status != VX_SUCCESS)
            {
                PTK_printf("[%s:%d] vxCopyImagePatch() failed.\n", __FUNCTION__,  __LINE__);
            }
        }
    }
}


void SDEAPP_deleteDisplayGraph(SDEAPP_Context *appCntxt)
{
    vxReleaseImage(&appCntxt->vxInputDisplayImage);
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

static void SDEAPP_exitProcThreads(SDEAPP_Context *appCntxt,
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
                               SDEAPP_USER_EVT_EXIT,
                               NULL);

    if (vxStatus != VX_SUCCESS)
    {
        PTK_printf("[%s:%d] vxSendUserEvent() failed.\n", __FUNCTION__,  __LINE__);
    }

    if (appCntxt->evtHdlrThread.joinable())
    {
        appCntxt->evtHdlrThread.join();
    }
}

void SDEAPP_cleanupHdlr(SDEAPP_Context *appCntxt)
{
    /* Wait for the threads to exit. */
    SDEAPP_exitProcThreads(appCntxt, false);

    PTK_printf("\nPress ENTER key to exit.\n");
    fflush(stdout);
    getchar();

    if (appCntxt->rtLogEnable == 1)
    {
        char name[256];

        snprintf(name, 255, "%s.bin", APP_SDE_NAME);
        tivxLogRtTraceExportToFile(name);
    }

    /* Release the Application context. */
    SDELDCAPPLIB_delete(&appCntxt->sdeLdcHdl);
    if (appCntxt->sdeAlgoType == 0)
    {
        SL_SDEAPPLIB_delete(&appCntxt->slSdeHdl);
    }
    else if (appCntxt->sdeAlgoType == 1)
    {
        ML_SDEAPPLIB_delete(&appCntxt->mlSdeHdl);
    }

    /* De-initialize the Application context. */
    SDEAPP_deleteDisplayGraph(appCntxt);

    SDEAPP_deInit(appCntxt);

    PTK_printf("[%s] Clean-up complete.\n", __FUNCTION__);
}

void SDEAPP_reset(SDEAPP_Context * appCntxt)
{
    /* Reset the performance capture initialization flag. */
    appCntxt->startPerfCapt = false;
}

static void SDEAPP_inputDataThread(SDEAPP_Context *appCntxt)
{
    PTK_printf("[%s] Launched Graph processing thread.\n", __FUNCTION__);

    // init display frame no
    appCntxt->displayFrmNo    = appCntxt->start_fileno;
    appCntxt->processFinished = false;

    while (true)
    {
        /* Reset ground model */
        SDEAPP_reset(appCntxt);

        /* Execute the graph. */
        SDEAPP_run(appCntxt);

        if (appCntxt->exitInputDataProcess)
        {
            break;
        }

        // reinit frame no to be displayed
        appCntxt->displayFrmNo = appCntxt->start_fileno;
    }

    appCntxt->processFinished = true;
}

static void SDEAPP_evtHdlrThread(SDEAPP_Context *appCntxt)
{
    vx_event_t evt;
    vx_status vxStatus;

    int32_t   status;

#if defined(PC)
    char output_file_name_disparity_img[SDEAPP_MAX_LINE_LEN];
    char output_file_name_input_img[SDEAPP_MAX_LINE_LEN];
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
                if (evt.app_value == SDEAPP_USER_EVT_EXIT)
                {
                    break;
                }
            }

            SDEAPP_processEvent(appCntxt, &evt);

            if (evt.type == VX_EVENT_GRAPH_COMPLETED)
            {
                SDEAPP_getOutBuff(appCntxt, &appCntxt->vxInputImage, &appCntxt->vxDisparity16);

                if (1 == appCntxt->display_option)
                {
                    SDEAPP_createDisplayImage(appCntxt);
                    SDEAPP_createDisparityCCImage(appCntxt);
                }

#if !defined(PC)
                status = vxScheduleGraph(appCntxt->vxDispGraph);
                PTK_assert(VX_SUCCESS == status);
                status = vxWaitGraph(appCntxt->vxDispGraph);
#endif

                SDEAPP_releaseOutBuff(appCntxt);

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
                        snprintf(output_file_name_input_img, SDEAPP_MAX_LINE_LEN, "%s/input_%05d.bmp",
                                 appCntxt->output_file_path,
                                 appCntxt->displayFrmNo);
                        tivx_utils_save_vximage_to_bmpfile(output_file_name_input_img, appCntxt->vxInputDisplayImage);
                    } else
                    {
                        snprintf(output_file_name_input_img, SDEAPP_MAX_LINE_LEN, "%s/input_%05d.yuv",
                                 appCntxt->output_file_path,
                                 appCntxt->displayFrmNo);
                        ptkdemo_save_vximage_to_yuvfile(appCntxt->vxInputDisplayImage, output_file_name_input_img);
                    }

                    snprintf(output_file_name_disparity_img, SDEAPP_MAX_LINE_LEN, "%s/disp_%05d.bmp",
                             appCntxt->output_file_path,
                             appCntxt->displayFrmNo);
                    tivx_utils_save_vximage_to_bmpfile(output_file_name_disparity_img, appCntxt->vxDisparityCC);
                }
#endif
                appCntxt->displayFrmNo++;
            }
        }

    } // while (true)
}


void SDEAPP_launchProcThreads(SDEAPP_Context *appCntxt)
{
    /* Launch the graph processing thread. */
    appCntxt->inputDataThread = std::thread(SDEAPP_inputDataThread, appCntxt);

    /* Launch the event handler thread. */
    appCntxt->evtHdlrThread = std::thread(SDEAPP_evtHdlrThread, appCntxt);
}

void SDEAPP_intSigHandler(SDEAPP_Context *appCntxt, int sig)
{
    // wait for frame processing to finish
    appCntxt->end_fileno = 0;
    appCntxt->exitInputDataProcess = true;

    while(!appCntxt->processFinished)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    /* Wait for the threads to exit. */
    SDEAPP_exitProcThreads(appCntxt, true);

    /* Release the Application context. */
    SDELDCAPPLIB_delete(&appCntxt->sdeLdcHdl);

    if (appCntxt->sdeAlgoType == 0)
    {
        SL_SDEAPPLIB_delete(&appCntxt->slSdeHdl);
    }
    else if (appCntxt->sdeAlgoType == 1)
    {
        ML_SDEAPPLIB_delete(&appCntxt->mlSdeHdl);
    }

    /* Delete display graph */
    SDEAPP_deleteDisplayGraph(appCntxt);

    /* De-initialize the Application context. */
    SDEAPP_deInit(appCntxt);

    PTK_printf("[%s] Clean-up complete.\n", __FUNCTION__);

    exit(0);
}





