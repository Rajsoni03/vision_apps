 /*
 *******************************************************************************
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
 *******************************************************************************
 */

#ifndef _APP_SDE_OBSTACLE_DETECTION_MAIN_H_
#define _APP_SDE_OBSTACLE_DETECTION_MAIN_H_

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
#include <sde_obstacle_detection_applib.h>
#include <sde_singlelayer_applib.h>
#include <sde_multilayer_applib.h>


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

// disable post-processing median filter for now because it is too slow
// After optmization, it could be enabled
#define ENABLE_PP_MEDIAN_FILTER     0

#define SODAPP_MAX_LINE_LEN         (512U)
#define SODAPP_NUM_BUFF_DESC        (1U)

#define OUTPUT_DISPLAY_WIDTH        960
#define OUTPUT_DISPLAY_HEIGHT       480

#define INPUT_DISPLAY_WIDTH         960
#define INPUT_DISPLAY_HEIGHT        480

#define SODAPP_USER_EVT_EXIT        (1U)

#define SODAPP_MAX_PIPELINE_DEPTH   (6U)
#define SODAPP_NUM_GRAPH_PARAMS     (8U)
#define SODAPP_GRAPH_COMPLETE_EVENT (0U)

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

    /* Graph parameter 4 */
    vx_image                vxMedianFilteredDisparity;

    /* Graph parameter 4 */
    vx_array                vxObstaclesPose;

    /* Graph parameter 5 */
    vx_scalar               vxNumObstacles;

    /* Graph parameter 6 */
    vx_array                vxFreeSpaceBoundary;

    /* Graph parameter 7 */
    vx_user_data_object     vxDrivableSpace;

} SODAPP_graphParams;

using SODAPP_graphParamQ = std::queue<SODAPP_graphParams*>;


typedef struct
{
    /** Live online visualization (1=on,0=off) */
    uint32_t                               display_option;

    /** OpenVX references */
    vx_context                             vxContext;

    /** OpenVX SDE obstacle detection graph */
    vx_graph                               vxGraph;

    /** OpenVX Display graph */
    vx_graph                               vxDispGraph;

    /** Disparity display node in display graph */
    vx_node                                node_disparity_display;

    /** Handle to the disparity display configuration parameters */
    vx_user_data_object                    disparity_display_config;

    /** Disparity display configuration parameters */
    tivx_display_params_t                  disparity_display_params;

    /** Bounding box image display node in display graph */
    vx_node                                node_image_display;

    /** Handle to the bounding box image display configuration parameters */
    vx_user_data_object                    image_display_config;

    /** Bounding box image display configuration parameters */
    tivx_display_params_t                  image_display_params;

    /** Input left image object */
    vx_image                               vxInputLeftImage[SODAPP_MAX_PIPELINE_DEPTH];

    /** Input right image object */
    vx_image                               vxInputRightImage[SODAPP_MAX_PIPELINE_DEPTH];

    /** Input rectified left image object */
    vx_image                               vxLeftRectImage[SODAPP_MAX_PIPELINE_DEPTH];

    /** Input rectified right image object */
    vx_image                               vxRightRectImage[SODAPP_MAX_PIPELINE_DEPTH];

    /** Raw disparity map object */
    vx_image                               vxSde16BitOutput[SODAPP_MAX_PIPELINE_DEPTH];

    /** Merged disparity map object at base layer */
    vx_image                               vxMergeDisparityL0[SODAPP_MAX_PIPELINE_DEPTH];

    /** Median filtered disparity map object */
    vx_image                               vxMedianFilteredDisparity[SODAPP_MAX_PIPELINE_DEPTH];

    /** Handle to output obstacles' positions */
    vx_array                               vxObstaclesPose[SODAPP_MAX_PIPELINE_DEPTH];

    /** Handle to output number of obstacles */
    vx_scalar                              vxNumObstacles[SODAPP_MAX_PIPELINE_DEPTH];

    /** Handle to output free-space boundary */
    vx_array                               vxFreeSpaceBoundary[SODAPP_MAX_PIPELINE_DEPTH];

    /** Handle to drivable space */
    vx_user_data_object                    vxDrivableSpace[SODAPP_MAX_PIPELINE_DEPTH];

    /** Input image object to get rectfied right image from paramDesc  */
    vx_image                               vxDispRightImage;

    /** Bounding box image to be displayed by the display graph */
    vx_image                               vxInputBBImage;

    /** Color-coded disparity map object */
    vx_image                               vxDisparityCC;

    /** Raw disparity object to get the output SDE raw disparity from paramDesc  */
    vx_image                               vxOutDisparity16;

    /** SDE algorithm type, 0: SDE, 1: Multi-layer SDE */
    uint8_t                                sdeAlgoType;

    /** Number of layers in multi-layer SDE */
    uint8_t                                numLayers;

    /** Flag whether median filter is enabled */
    uint8_t                                ppMedianFilterEnable;

    /** SDE configuration parameters */
    tivx_dmpac_sde_params_t                sde_params;

    /** Image width */
    uint32_t                               width;

    /** Image height */
    uint32_t                               height;

    /** Input format: U8 or YUV_UYVY */
    uint8_t                                inputFormat;

    /** Disparity confidence threshold */
    uint8_t                                confidence_threshold;

    /** Flag whether LDC rectification is enabled */
    uint8_t                                enableLDC;

    /** Pipeline depth */
    uint8_t                                pipelineDepth;

    /** Camera param: horizontal distotion center */
    float                                  distCenterX;

    /** Camera param: vertical distotion center */
    float                                  distCenterY;

    /** Camera param: focal length */
    float                                  focalLength;

    /** Camera param: roll angle */
    float                                  camRoll;

    /** Camera param: pitch angle */
    float                                  camPitch;

    /** Camera param: yaw angle */
    float                                  camYaw;

    /** Camera param: camera height */
    float                                  camHeight;

    /** Camera param: baseline */
    float                                  baseline;

    /** Flag whether Road params will be used for ground plane estimation */
    uint8_t                                useRoadParams;

    /** Horizon position on image */
    int32_t                                roadTopYpos;

    /** Road width at horizon */
    int32_t                                roadSizeBase;

    /** Road width at horizon */
    int32_t                                roadSizeTop;

    /** Ego vehicle's size (width) */
    int32_t                                egoSize;

    /** Minimum obstacle's disparity, which is obstacle's maximum depth */
    int32_t                                minObstacleDisparity;

    /** Algorithm param: percentage of obstacle pixels in a region */
    float                                  minObstacleProbability;

    /** Algorithm param: minimum number of consecutive obstacle pixel 
     * for a region to be declared as an obstacle
     */
    uint8_t                                minConsecutiveObstacles;

    /* graph parameter tracking */
    SODAPP_graphParams                     paramDesc[SODAPP_MAX_PIPELINE_DEPTH];

    /** A queue for holding free descriptors. */
    SODAPP_graphParamQ                     freeQ;

    /** Queue for output processing. */
    SODAPP_graphParamQ                     outputQ;

    /** Left input image file path */
    char                                   left_img_file_path[SODAPP_MAX_LINE_LEN];

    /** Right input image file path */
    char                                   right_img_file_path[SODAPP_MAX_LINE_LEN];

    /** Output file path */
    char                                   output_file_path[SODAPP_MAX_LINE_LEN];

    /** Left input image file name */
    char                                   left_img_file_name[SODAPP_MAX_LINE_LEN];

    /** Right input image file name */
    char                                   right_img_file_name[SODAPP_MAX_LINE_LEN];

    /** Left rectification LUT file name */
    char                                   left_LUT_file_name[SODAPP_MAX_LINE_LEN];

    /** Right rectification LUT file name */
    char                                   right_LUT_file_name[SODAPP_MAX_LINE_LEN];

    /** Start frame number to process */
    uint32_t                               start_fileno;

    /** Last frame number to process */
    uint32_t                               end_fileno;

    /** Dispaly frame number */
    uint32_t                               displayFrmNo;

    /** Application interactive status, 0=non-interactive, 1=interactive. */
    uint32_t                               is_interactive;

    /** Total radar frames processed */
    uint32_t                               frameCnt;

    /** LDC APPLIB create params */
    SDELDCAPPLIB_createParams              sdeLdcCreateParams;

    /** Handle to LDC APPLIB context */
    SDELDCAPPLIB_Handle                    sdeLdcHdl;

    /** Single-layer SDE APPLIB create params */
    SL_SDEAPPLIB_createParams              slSdeCreateParams;

    /** Handle to single-layer SDE APPLIB context */
    SL_SDEAPPLIB_Handle                    slSdeHdl;

    /** Multi-layer SDE APPLIB create params */
    ML_SDEAPPLIB_createParams              mlSdeCreateParams;

    /** Handle to multi-layer SDE APPLIB context */
    ML_SDEAPPLIB_Handle                    mlSdeHdl;

    /** Stereo based obstacle detection APPLIB create params */
    SODAPPLIB_createParams                 sodCreateParams;

    /** Handle to Stereo based obstacle detection APPLIB context */
    SODAPPLIB_Handle                       sodHdl;

    /** Number of graph params */
    uint8_t                                numGraphParams;

    /** Render periodicity in milli-sec. */
    uint64_t                               renderPeriod;

    /** Number of detected obstacles */
    uint32_t                               numObstacles;

    /** Detected obstacles positions */
    tivx_obstacle_pos_t                  * obsBox;

    /** Detected free-space boundary */
    int32_t                              * freespaceBoundary;

    /** Detected drivable space */
    tivx_drivable_space_t                  drivableSpace;

    /** Flag to indicate that the input data processing thread should exit. */
    bool                                   exitInputDataProcess;

    /** Flag to indicate that the input data processing has finished. */
    bool                                   processFinished;

    /** Input data thread. */
    std::thread                            inputDataThread;

    /** Event handler thread. */
    std::thread                            evtHdlrThread;

    /** Semaphore to synchronize the input and input data processing threads. */
    UTILS::Semaphore                     * dataReadySem;

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
    app_perf_point_t                       sodPerf;

    /** Flag to track if the performance counter has been initialized. */
    bool                                   startPerfCapt;

    /** Resource lock. */
    std::mutex                             paramRsrcMutex;

} SODAPP_Context;

#endif /* _APP_SDE_OBSTACLE_DETECTION_MAIN_H_ */

