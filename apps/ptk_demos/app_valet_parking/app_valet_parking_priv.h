 /*
 *******************************************************************************
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
 *******************************************************************************
 */

#ifndef _APP_VALET_PARKING_PRIV_H_
#define _APP_VALET_PARKING_PRIV_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include <TI/tivx.h>
#include <TI/tivx_debug.h>
#include <TI/j7.h>

#include <perception/perception.h>
#include <perception/gui.h>
#include <perception/gui/c/gui.h>
#include <perception/dbtools.h>
#include <perception/dbtools/c/virtual_sensor_creator.h>
#include <perception/utils/ptk_semaphore.h>

#include <app_ptk_demo_display.h>

#include <surround_radar_ogmap_applib.h>
#include <surround_sfm_ogmap_applib.h>
#include <lidar_ogmap_applib.h>
#include <fused_ogmap_applib.h>
#include <ps_mapping_applib.h>



using namespace std;
using namespace ptk;

#ifdef __cplusplus
extern "C" {
#endif

#define APP_ASSERT(x)               assert((x))
#define APP_ASSERT_VALID_REF(ref)   PTK_assert(vxGetStatus((vx_reference)(ref))==VX_SUCCESS)

#ifdef __cplusplus
}
#endif

#define VALETAPP_MAX_LINE_LEN            (256U)

#define VALETAPP_GRID_ID_SFM_ACCUMULATED      (1U)
#define VALETAPP_GRID_ID_SFM_INST_OCC         (2U)
#define VALETAPP_GRID_ID_SFM_INST_DS          (3U)

#define VALETAPP_GRID_ID_RADAR_ACCUMULATED    (4U)
#define VALETAPP_GRID_ID_RADAR_INST_OCC       (5U)
#define VALETAPP_GRID_ID_RADAR_INST_DS        (6U)

#define VALETAPP_GRID_ID_LIDAR_ACCUMULATED    (7U)
#define VALETAPP_GRID_ID_LIDAR_INST_OCC       (8U)
#define VALETAPP_GRID_ID_LIDAR_INST_DS        (9U)

#define VALETAPP_GRID_ID_FUSED_OCCUPANCY      (10U)
#define VALETAPP_GRID_ID_FUSED_DS             (11U)
#define VALETAPP_GRID_ID_TIDL_PSD_OCCUPANCY   (12U)

// Flag bits for the occupancy grid
/** Contains at least one non-ground point. */
#define VALETAPP_FLAG_OCCUPIED                (0x00000001)

/** Contains a mixture of ground and drivable points. */
#define VALETAPP_FLAG_GROUND                  (0x00000002) /* Free */

/** A square we are or were present in. */
#define VALETAPP_FLAG_EGO                     (0x00000004)

/** A square we want to consider for parking space search. */
#define VALETAPP_FLAG_FST                     (0x00000008)

/** A square search candidate to test. */
#define VALETAPP_FLAG_FSD                     (0x00000010)

/** A square that has a circle of appropriate radius free of obstacles. */
#define VALETAPP_FLAG_PFSD                    (0x00000020)

/** Flag to indicate cell was updated. */
#define VALETAPP_FLAG_INST_CHANGED            (0x00000040)

/* Do-Kyoung */
/** Free space by TIDL PSD */
#define VALETAPP_FLAG_TIDL_PSD_FREE           (0x00000080)

/** Occupied space by TIDL PSD */
#define VALETAPP_FLAG_TIDL_PSD_OCCUPIED       (0x00000100)

/** Flag bits for the lidar point cloud */
/** Point is 'removed' and should be ignored by algorithms */
#define VALETAPP_POINT_TAG_REMOVED            (TAG_POINT_REMOVED)

/** Point is tagged as a ground point by algorithm */
#define VALETAPP_POINT_TAG_GROUND             (TAG_MAYBE_GROUND)

/** sensor inputs */
enum valetapp_sensors_e
{
    VALETAPP_SENSOR_CAMERA_R = 0,
    VALETAPP_SENSOR_RADAR_0,
    VALETAPP_SENSOR_RADAR_1,
    VALETAPP_SENSOR_RADAR_2,
    VALETAPP_SENSOR_RADAR_3,
    VALETAPP_SENSOR_LIDAR,
    VALETAPP_SENSOR_INS,
    VALETAPP_SENSOR_PSD,
    VALETAPP_SENSOR_PSD_IMG,
    VALETAPP_SENSOR_CAMERA_R_DOF,
    VALETAPP_SENSOR_MAX
};

/** output options */
enum valetapp_outputs_e
{
    VALETAPP_OUTPUT_SFM_MAP = 0,
    VALETAPP_OUTPUT_RADAR_MAP,
    VALETAPP_OUTPUT_LIDAR_MAP,
    VALETAPP_OUTPUT_FUSED_MAP,
    VALETAPP_OUTPUT_MAX
};

typedef struct
{
    uint32_t texture;
    int8_t   bInit;
    char imgFileName[VALETAPP_MAX_LINE_LEN];

} PSIMGLIB_Context;

typedef PSIMGLIB_Context * PSIMGLIB_Handle;

typedef struct
{
    uint32_t texture;
    int8_t   bInit;
    char imgFileName[VALETAPP_MAX_LINE_LEN];

} DOFIMGLIB_Context;

typedef DOFIMGLIB_Context * DOFIMGLIB_Handle;

typedef struct
{
    uint8_t* data;

} RADARLIB_Context;

typedef RADARLIB_Context * RADARLIB_Handle;

using namespace std;
using namespace ptk;

/** The following are the graphs we track.
 * - RADAR OGMAP
 * - LIDAR OGMAP
 * - CAMERA SFM OGMAP
 * - CAMERA POINT CLOUD
 * - FUSION OGMAP
 */
#define VALETAPP_MAX_NUM_GRAPHS     (5U)

/* The following user event is used to set the exit condition. */
#define VALETAPP_EVENT_BASE                 (0U)
#define VALETAPP_RADAR_GRAPH_COMP_EVENT     (VALETAPP_EVENT_BASE + 1U)
#define VALETAPP_LIDAR_GRAPH_COMP_EVENT     (VALETAPP_EVENT_BASE + 2U)
#define VALETAPP_SFM_GRAPH_COMP_EVENT       (VALETAPP_EVENT_BASE + 3U)
#define VALETAPP_USER_EVT_EXIT              (VALETAPP_EVENT_BASE + 4U)

#define VALETAPP_RADAR_OG_GRAPH_EVENT_BASE  (VALETAPP_EVENT_BASE + 20U)
#define VALETAPP_LIDAR_OG_GRAPH_EVENT_BASE  (VALETAPP_RADAR_OG_GRAPH_EVENT_BASE + 20U)
#define VALETAPP_SFM_OG_GRAPH_EVENT_BASE    (VALETAPP_LIDAR_OG_GRAPH_EVENT_BASE + 20U)

#define VALETAPP_STATS_RADAR_ID             (0U)
#define VALETAPP_STATS_LIDAR_ID             (VALETAPP_STATS_RADAR_ID+1U)
#define VALETAPP_STATS_SFM_ID               (VALETAPP_STATS_LIDAR_ID+1U)
#define VALETAPP_STATS_MAX                  (VALETAPP_STATS_SFM_ID+1U)

typedef struct
{
    /** Total radar frames processed. */
    uint32_t                    totalFrameCount;

    /** Total radar frames processed. */
    uint32_t                    framesProcessed;

    /** Total radar frames dropped. */
    uint32_t                    droppedFrameCnt;

} VALETAPP_Stats;

typedef struct
{
    /** database config file (defines sensor inputs and outputs to
     *  be saved as virtual sensor) */
    PTK_DBConfig                        dbConfig;

    /** Global flag to disable dumping outmaps to virtual sensor folder. */
    uint32_t                            dumpMap;

    /** Live online visualization bit mask (1=on,0=off) with bits as
     * 1 = radar-only map, 2 = lidar-only map, 3 = SfM-only map */
    uint32_t                            visualize;

    /** Verbosity, 0=no prints, 1=info prints, >1=debug prints. */
    uint32_t                            verbose;

    /** Application interactive status, 0=non-interactive, 1=interactive. */
    uint32_t                            is_interactive;

    /** Sensor rate control flag. If set to true then the inter-frame delay
     * based on the timestamps associated with the data frames will be used.
     */
    bool                                sensorRateControl;

    /** Input rate control flag. If set to true then a semaphore will be
     * used to control the synchronization between the input data and the
     * graph processing threads.
     */
    bool                                inputRateControl;

    /* Absolute Path of the font file. */
    char                                fontFile[VALETAPP_MAX_LINE_LEN];

    /* Relative Path for the logo file. The path is relative to
     * APP_CONFIG_BASE_PATH environment variable.
     */
    char                                logoPath[VALETAPP_MAX_LINE_LEN];

    /* Relative Path for the background file. The path is relative to
     * APP_CONFIG_BASE_PATH environment variable.
     */
    char                                bkgrndPath[VALETAPP_MAX_LINE_LEN];

    /** Grid parameters. */
    PTK_GridConfig                      gridConfig;

    /** ROI parameters. */
    PTK_GridRoi                         roiParams;

    /** FSD/PFSD configuration. */
    PTK_Alg_FsdPfsdParams               fsdPfsdConfig;

    /** FSD Match radius. */
    float                               fsdMatchRadius;

    /** FSD enable flag. */
    uint32_t                            cameraFsdEnable;

    /** Stream handle for each sensor.
     *  Handle is null if sensor is not enabled*/
    sensorDataPlayerInsHandle           dataPlayer;

    /** Stream handle for each sensor.
     *  Handle is null if output is not enabled */
    vscHandle                           outputStrmHdl[VALETAPP_OUTPUT_MAX];

    /** Render Objects */
    DemoDashboard                      *dash;
    DashboardRenderable                *dashRenderer;
    Renderer                           *renderer;
    MapRenderable                      *radarMapRenderer;
    MapRenderable                      *lidarMapRenderer;
    MapRenderable                      *cameraMapRenderer;
    MapRenderable                      *fusionMapRenderer;

    RadarRenderable                    *radarRenderer;

    MapRenderable                      *psdMapRenderer;
    Image                              *psdImgRenderer;
    Image                              *dofImgRenderer;
    GridIntensityView                  *gridIntView;
    /* One each for Camera, Radar, Lidar, and Fusion output. */
    GridFlagView                       *gridFlagView[4];
    KeyHandlerTable                    *keyHandlerTbl;

    /** OpenVX references */
    vx_context                          vxContext;

    /** SfM Applib Create Parameters. */
    SFMOGAPPLIB_createParams            cameraCreateParams;

    /** Radar Applib Create Parameters. */
    RADAROGAPPLIB_createParams          radarCreateParams;

    /** Lidar Applib Create Parameters. */
    LIDAROGAPPLIB_CreateParams          lidarCreateParams;

    /** Fusion Applib Create Parameters. */
    FUSEDOGAPPLIB_createParams          fusedOgCreateParams;

    PSLIB_createParams                  psCreateParams;

    /** SfM Applib handle. */
    SFMOGAPPLIB_Handle                  cameraHdl;

    /** Radar Applib handle. */
    RADAROGAPPLIB_Handle                radarHdl;

    /** Lidar Applib handle. */
    LIDAROGAPPLIB_Handle                lidarHdl;

    /** Fusion Applib handle. */
    FUSEDOGAPPLIB_Handle                fusedOgHdl;

    /** Parking Space graph handle. */
    PSLIB_Handle                        psHdl;

    /** Parking Space Image graph handle. */
    PSIMGLIB_Handle                     psImgHdl;

    /** DOF Image graph handle. */
    DOFIMGLIB_Handle                    dofImgHdl;

    /** Radar point cloud graph handle. */
    RADARLIB_Handle                     radarPCHdl;

    /** Mask indicating which sensors have been enabled. */
    uint32_t                            radarSensorMask;

    /** Number of radar sensors. */
    uint32_t                            numRadars;

    /** >0 if SfM Applib should be included */
    uint8_t                             cameraEnabled;

    /** >0 if Radar Applib should be included */
    uint8_t                             radarEnabled;

    /** >0 if Lidar Applib should be included */
    uint8_t                             lidarEnabled;

    /** >0 if Fusion Applib should be included */
    uint8_t                             fusionEnabled;

    /** >0 if TIDL PSD should be included */
    uint8_t                             tidlPsdEnabled;

    /** >0 if TIDL PSD Image should be included */
    uint8_t                             tidlPsdImgEnabled;

    /** >0 if DoF Image should be included */
    uint8_t                             dofImgEnabled;

    /** SfM Applib output map size. */
    uint32_t                            cameraMapSize;

    /** Radar Applib output map size. */
    uint32_t                            radarMapSize;

    /** Lidar Applib output map size. */
    uint32_t                            lidarMapSize;

    /** Fused OG Applib output map size. */
    uint32_t                            fusedMapSize;

    /** TIDL PSD output map size */
    uint32_t                            tidlPsdMapSize;

    /** Number of radar records processed */
    VALETAPP_Stats                      stats[VALETAPP_STATS_MAX];

    /* Do-Kyoung */
    /** Number of parking spot records processed */
    uint32_t                            psFrameCnt;

    /** Number of parking spot records processed */
    uint32_t                            psImgFrameCnt;

    /** Number of DOF records processed */
    uint32_t                            dofImgFrameCnt;

    /** Current timestamp */
    uint64_t                            curTimeStamp;

    /** Fusion method. */
    uint32_t                            fusionMethod;

    /** API parameter for fusion node processing. */
    FUSEDOGAPPLIB_processReq            processReq;

    /** Flag to indicate that the graph processing thread should exit. */
    bool                                exitGraphProcess;

    /** Flag to indicate that the fusion processing thread should exit. */
    bool                                exitFusionProcess;

    /** Flag to indicate the render thread should exit.. */
    bool                                exitRenderProcess;

    /** Render periodicity in milli-sec. */
    uint64_t                            renderPeriod;

    /** Input data processing thread. */
    std::thread                         inputDataThread;

    /** Fusion processing thread. */
    std::thread                         fusionThread;

    /** Event handler thread. */
    std::thread                         evtHdlrThread;

    /** Render thread. */
    std::thread                         renderThreadId;

    /** Semaphore for rate synchronizing the input data and
     * graph processing threads for radar sensor.
     *
     * This is used only if 'inputRateControl' flag is set.
     *
     */
    UTILS::Semaphore                   *radarDataReadySem;

    /** Semaphore for rate synchronizing the input data and
     * graph processing threads for lidar sensor.
     *
     * This is used only if 'inputRateControl' flag is set.
     *
     */
    UTILS::Semaphore                   *lidarDataReadySem;

    /** Semaphore for rate synchronizing the input data and
     * graph processing threads for camera sensor.
     *
     * This is used only if 'inputRateControl' flag is set.
     *
     */
    UTILS::Semaphore                   *sfmDataReadySem;

    /** Width of the display window. */
    uint32_t                            winWidth;

    /** Height of the display window. */
    uint32_t                            winHeight;

#ifdef PLATFORM_EGL
    PTKDEMO_DisplayContext              dispCntxt;
#endif

    /** File name for exporting performance data. */
    char                               *perfOutFile;

    /* Radar graph pipeline depth. */
    uint8_t                             radarPipelineDepth;

    /* Lidar graph pipeline depth. */
    uint8_t                             lidarPipelineDepth;

    /* SFM graph pipeline depth. */
    uint8_t                             sfmPipelineDepth;

    /** Real-time logging enable.
     * 0 - disable
     * 1 - enable
     */
    uint8_t                             rtLogEnable;

} VALETAPP_Context;

int32_t VALETAPP_launchRenderThread(VALETAPP_Context   * appCntxt);
void VALETAPP_exitRenderThread(VALETAPP_Context    * appCntxt);

#endif /* _APP_VALET_PARKING_PRIV_H_ */
