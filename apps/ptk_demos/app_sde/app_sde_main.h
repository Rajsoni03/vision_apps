 /*
 *******************************************************************************
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
 *******************************************************************************
 */

#ifndef _APP_SDE_MAIN_H_
#define _APP_SDE_MAIN_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include <TI/tivx.h>
#include <TI/tivx_debug.h>
#include <TI/j7.h>
#include <TI/tivx_stereo.h>

#include <perception/perception.h>
#include <perception/utils/ipc_chan.h>
#include <perception/utils/ptk_semaphore.h>

#include <sde_ldc_applib.h>
#include <sde_multilayer_applib.h>
#include <sde_singlelayer_applib.h>


#ifdef __cplusplus
extern "C" {
#endif

#include <utils/draw2d/include/draw2d.h>
#include <utils/grpx/include/app_grpx.h>


#define APP_ASSERT(x)               PTK_assert((x))
#define APP_ASSERT_VALID_REF(ref)   APP_ASSERT(vxGetStatus((vx_reference)(ref))==VX_SUCCESS)

#ifdef __cplusplus
}
#endif

#define SDEAPP_MAX_NUM_LAYERS    (3U)

#define SDEAPP_MAX_LINE_LEN      (512U)
#define SDEAPP_NUM_BUFF_DESC     (1U)

#define OUTPUT_DISPLAY_WIDTH        960
#define OUTPUT_DISPLAY_HEIGHT       480

#define INPUT_DISPLAY_WIDTH         960
#define INPUT_DISPLAY_HEIGHT        480

#define SDEAPP_MAX_PIPELINE_DEPTH   (4U)
#define SDEAPP_NUM_GRAPH_PARAMS     (6U)
#define SDEAPP_GRAPH_COMPLETE_EVENT (0U)

#define SDEAPP_USER_EVT_EXIT     (1U)


using namespace std;


typedef struct
{
    /* Graph parameter 0 */
    vx_image                vxInputLeftImage;

    /* Graph parameter 1 */
    vx_image                vxInputRightImage;

    /* Graph parameter 2 */
    vx_image                vxRightRectImage;

    /* Graph parameter 3 */
    vx_image                vxSde16BitOutput;

    /* Graph parameter 4 */
    vx_image                vxMergeDisparityL0;

    /* Graph parameter 5 */
    vx_image                vxMedianFilteredDisparity;

} SDEAPP_graphParams;

using SDEAPP_graphParamQ = std::queue<SDEAPP_graphParams*>;


typedef struct
{
    /** Live online visualization (1=on,0=off) */
    uint32_t                               display_option;

    /** OpenVX references */
    vx_context                             vxContext;

    /** OpenVX SDE graph */
    vx_graph                               vxGraph;

    /** OpenVX Display graph */
    vx_graph                               vxDispGraph;

    /** Disparity display node in display graph */
    vx_node                                node_disparity_display;

    /** Handle to the disparity display configuration parameters */
    vx_user_data_object                    disparity_display_config;

    /** Disparity display configuration parameters */
    tivx_display_params_t                  disparity_display_params;

    /** Input image display node in display graph */
    vx_node                                node_image_display;

    /** Handle to the input image display configuration parameters */
    vx_user_data_object                    image_display_config;

    /** Input image display configuration parameters */
    tivx_display_params_t                  image_display_params;

    /** graph parameter tracking */
    SDEAPP_graphParams                     paramDesc[SDEAPP_MAX_PIPELINE_DEPTH];

    /** A queue for holding free descriptors. */
    SDEAPP_graphParamQ                     freeQ;

    /** Queue for output processing. */
    SDEAPP_graphParamQ                     outputQ;

    /** Input left image object */
    vx_image                               vxInputLeftImage[SDEAPP_MAX_PIPELINE_DEPTH];

    /** Input right image object */
    vx_image                               vxInputRightImage[SDEAPP_MAX_PIPELINE_DEPTH];

    /** Input rectified left image object */
    vx_image                               vxLeftRectImage[SDEAPP_MAX_PIPELINE_DEPTH];

    /** Input rectified right image object */
    vx_image                               vxRightRectImage[SDEAPP_MAX_PIPELINE_DEPTH];

    /** Raw disparity map object */
    vx_image                               vxSde16BitOutput[SDEAPP_MAX_PIPELINE_DEPTH];

    /** Merged disparity map object at base layer */
    vx_image                               vxMergeDisparityL0[SDEAPP_MAX_PIPELINE_DEPTH];

    /** Median filtered disparity map object */
    vx_image                               vxMedianFilteredDisparity[SDEAPP_MAX_PIPELINE_DEPTH];

    /** Input image object to get rectfied right image from paramDesc  */
    vx_image                               vxInputImage;

    /** Input image object to be displayed by the display graph */
    vx_image                               vxInputDisplayImage;

    /** Color-coded disparity map object */
    vx_image                               vxDisparityCC;

    /** Raw disparity map object */
    vx_image                               vxDisparity16;

    /** SDE configuration parameters */
    tivx_dmpac_sde_params_t                sde_params;

    /** Image width */
    uint16_t                               width[SDEAPP_MAX_NUM_LAYERS];

    /** Image height */
    uint16_t                               height[SDEAPP_MAX_NUM_LAYERS];

    /** SDE algorithm type, 0: SDE, 1: Multi-layer SDE */
    uint8_t                                sdeAlgoType;

    /** Number of layers in multi-layer SDE */
    uint8_t                                numLayers;

    /** Flag whether median filter is enabled */
    uint8_t                                ppMedianFilterEnable;

    /** Disparity confidence threshold */
    uint8_t                                confidence_threshold;

    /** Pipeline depth */
    uint8_t                                pipelineDepth;

    /** Left input image file path */
    char                                   left_img_file_path[SDEAPP_MAX_LINE_LEN];

    /** Right input image file path */
    char                                   right_img_file_path[SDEAPP_MAX_LINE_LEN];

    /** Output file path */
    char                                   output_file_path[SDEAPP_MAX_LINE_LEN];
  
    /** Left input image file name */
    char                                   left_img_file_name[SDEAPP_MAX_LINE_LEN];

    /** Right input image file name */
    char                                   right_img_file_name[SDEAPP_MAX_LINE_LEN];

    /** Left rectification LUT file name */
    char                                   left_LUT_file_name[SDEAPP_MAX_LINE_LEN];

    /** Right rectification LUT file name */
    char                                   right_LUT_file_name[SDEAPP_MAX_LINE_LEN];

    /** Start frame number to process */
    uint32_t                               start_fileno;

    /** Last frame number to process */
    uint32_t                               end_fileno;

    /** Dispaly frame number */
    uint32_t                               displayFrmNo;

    /** Input format: U8 or YUV_UYVY */
    uint8_t                                inputFormat;

    /** Application interactive status, 0=non-interactive, 1=interactive. */
    uint32_t                               is_interactive;

    /** Total frames processed */
    uint32_t                               frameCnt;

    /** LDC APPLIB create params */
    SDELDCAPPLIB_createParams              sdeLdcCreateParams;

    /** Handle to LDC APPLIB context */
    SDELDCAPPLIB_Handle                    sdeLdcHdl;

    /** Multi-layer SDE APPLIB create params */
    ML_SDEAPPLIB_createParams              mlSdeCreateParams;

    /** Handle to multi-layer SDE APPLIB context */
    ML_SDEAPPLIB_Handle                    mlSdeHdl;

    /** Single-layer SDE APPLIB create params */
    SL_SDEAPPLIB_createParams              slSdeCreateParams;

    /** Handle to single-layer SDE APPLIB context */
    SL_SDEAPPLIB_Handle                    slSdeHdl;

    /** Number of graph params */
    uint8_t                                numGraphParams;

    /** Render periodicity in milli-sec. */
    uint64_t                               renderPeriod;

    /** Flag to indicate that the graph processing thread should exit. */
    bool                                   exitInputDataProcess;

    /** Flag to indicate that the graph processing thread has finished */
    bool                                   processFinished;

    /** Graph processing thread. */
    std::thread                            inputDataThread;

    /** Event handler thread. */
    std::thread                            evtHdlrThread;

    /** Semaphore to synchronize the input and graph processing threads. */
    UTILS::Semaphore                      *dataReadySem;
    
    /** Handle to 2D draw */
    Draw2D_Handle                          pHndl;

    /** Display buffer */
    uint16_t                             * pDisplayBuf565;

    /** Flag to indicate if the graph should be exported
     * 0 - disable
     * 1 - enable
     */
    uint8_t                                exportGraph;

    /** Real-time logging enable.
     * 0 - disable
     * 1 - enable
     */
    uint8_t                                rtLogEnable;

    /** Base value to be used for any programmed VX events. */
    uint32_t                               vxEvtAppValBase;

    /** Performance monitoring. */
    app_perf_point_t                       sdePerf;

    /** Flag to track if the performance counter has been initialized. */
    bool                                   startPerfCapt;

    /** Resource lock. */
    std::mutex                             paramRsrcMutex;

} SDEAPP_Context;

#endif /* _APP_SDE_MAIN_H_ */

