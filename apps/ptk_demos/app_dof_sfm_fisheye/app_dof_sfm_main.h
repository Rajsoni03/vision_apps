#ifndef _APP_DOF_SFM_OGMAP_RENDERER_H_
#define _APP_DOF_SFM_OGMAP_RENDERER_H_

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include <TI/tivx.h>
#include <TI/tivx_debug.h>
#include <TI/j7.h>

#include <perception/perception.h>
#include <perception/gui.h>
#include <perception/gui/c/gui.h>
#include <perception/dbtools/c/dbtools.h>
#include <perception/utils/ptk_semaphore.h>

#include <app_ptk_demo_display.h>
#include <surround_sfm_ogmap_applib.h>

using namespace std;
using namespace ptk;

#ifdef __cplusplus
extern "C" {
#endif

#include <tivx_utils_file_rd_wr.h>
#include <tivx_utils_graph_perf.h>
#include <utils/perf_stats/include/app_perf_stats.h>

#define APP_MAX_FILE_PATH           (1024U)
#define APP_ASSERT(x)               PTK_assert((x))
#define APP_ASSERT_VALID_REF(ref)   APP_ASSERT(vxGetStatus((vx_reference)(ref))==VX_SUCCESS)

#ifdef __cplusplus
}
#endif

/** sensor inputs */
enum sensors_e
{
    DOFSFMAPP_SENSOR_CAMERA = 0,
    DOFSFMAPP_SENSOR_INS,
    DOFSFMAPP_SENSOR_MAX
};

/*
typedef enum app_mode_e
{
    WITHOUT_LDC = 0,  //!< Do not use LDC remapping. Graph looks like Images->Pyr->DOF->DOF_vis
    WITH_LDC          //!< Include LDC remapping.    Graph looks like Images->LDC->Pyr->DOF->DOF_vis
} AppMode;
*/

#define TIVX_DOFSFMAPP_MAX_LINE_LEN                     (256U)

#define DOFSFMAPP_MAX_NUM_GRAPHS                        (2U)
#define DOFSFMAPP_NUM_BUFF_DESC                         (1U)

/* The following user event is used to set the exit condition. */
#define DOFSFMAPP_EVENT_BASE                        25U
#define DOFSFMAPP_USER_EVT_EXIT                     (DOFSFMAPP_EVENT_BASE + 1U)
#define DOFSFMAPP_SFM_GRAPH_COMP_EVENT              (DOFSFMAPP_EVENT_BASE + 2U)

#define DOFSFMAPP_GRID_ID_SFM_ACCUMULATED               (1U)
#define DOFSFMAPP_GRID_ID_SFM_INST_OCC                  (2U)
#define DOFSFMAPP_GRID_ID_SFM_INST_DS                   (3U)

// Flag bits for the occupancy grid
/** Contains at least one non-ground point. */
#define DOFSFMAPP_FLAG_OCCUPIED                        (0x00000001)

/** Contains a mixture of ground and drivable points. */
#define DOFSFMAPP_FLAG_GROUND                          (0x00000002)

/** A square we are or were present in. */
#define DOFSFMAPP_FLAG_US                              (0x00000004)

/** A square we want to consider for parking space search. */
#define DOFSFMAPP_FLAG_FST                             (0x00000008)

/** A square search candidate to test. */
#define DOFSFMAPP_FLAG_FSD                             (0x00000010)

/** A square that has a circle of appropriate radius free of obstacles. */
#define DOFSFMAPP_FLAG_PFSD                            (0x00000020)

/** Flag to indicate cell was updated. */
#define DOFSFMAPP_FLAG_INST_CHANGED                    (0x00000040)

typedef struct app_obj_t
{
    /* Input/Output data configuration */
    PTK_DBConfig                dbConfig;   //!< database config file (defining inputs and outputs)

    /* input data streams */
	sensorDataPlayerInsHandle   dataPlayer;   // from sfm

	/* Absolute Path of the font file. */
    char                        fontFile[TIVX_DOFSFMAPP_MAX_LINE_LEN];

    /* Relative Path for the logo file. The path is relative to
	 * APP_CONFIG_BASE_PATH environment variable.
	 */
	char                        logoPath[TIVX_DOFSFMAPP_MAX_LINE_LEN];

	/* Relative Path for the background file. The path is relative to
	 * APP_CONFIG_BASE_PATH environment variable.
	 */
	char                        bkgrndPath[TIVX_DOFSFMAPP_MAX_LINE_LEN];

	/* Live online visualization bit mask (1=on,0=off) with bits */
	uint32_t                    visualize;

    /** Application interactive status, 0=non-interactive, 1=interactive. */
    uint32_t                    is_interactive;

    /** Sensor rate control flag. If set to true then the inter-frame delay
     * based on the timestamps associated with the data frames will be used.
     */
    bool                        sensorRateControl;

    /** Input rate control flag. If set to true then a semaphore will be
     * used to control the synchronization between the input data and the
     * graph processing threads.
     */
    bool                        inputRateControl;

    Renderer                   *renderer;
    MapRenderable              *cameraMapRenderer;
    KeyHandlerTable            *keyHandlerTbl;
    /* OGMap Grid */
    GridFlagView               *gridFlagView;

    /** SfM Applib Create Parameters. */
    SFMOGAPPLIB_createParams    cameraCreateParams;

    /** SfM Applib handle. */
    SFMOGAPPLIB_Handle          cameraHdl;

    /** Grid parameters. */
    PTK_GridConfig              gridConfig;

    /** SfM Applib output map size. */
    uint32_t                    cameraMapSize;

    uint32_t                    totalFrameCount;

    uint32_t                    droppedFrameCnt;

    uint32_t                    framesProcessed;

    uint32_t                    maxNumTracks;

    uint32_t                    dofConfidThresh;

    uint8_t                     enableDof;

    PTK_Map                    *map;

    /* output data streams */
	vscHandle                   mapOut;      //<! output OG map

    /* App Control */
    uint32_t                    verbose;    //<! verbosity, 0=no prints, 1=info prints, >1=debug prints

    /*Internal Variables */
    uint32_t                    pyramidInWidth;  //!< width of input image to Pyramid node
    uint32_t                    pyramidInHeight; //!< height of input image to Pyramid node

    /* OVX Node References */
    vx_context                  context;

    /** Flag to indicate that the graph processing thread should exit. */
    bool                        exitInputDataProcess;

    /** Flag to indicate the render thread should exit.. */
    bool                        exitRenderProcess;

    app_perf_point_t            graph_perf[DOFSFMAPP_MAX_NUM_GRAPHS];

    /** Render periodicity in milli-sec. */
    uint64_t                    renderPeriod;

    /** current timestamp */
    uint64_t                    curTimestamp;

    /** Graph processing thread. */
    std::thread                 inputDataThread;

    /** Event handler thread. */
    std::thread                 evtHdlrThread;

    /** Render thread. */
    std::thread                 renderThreadId;

    /** Semaphore to synchronize the input and graph processing threads. */
    UTILS::Semaphore            *dataReadySem;

    /** Width of the display window. */
    uint32_t                    winWidth;

    /** Height of the display window. */
    uint32_t                    winHeight;

#ifdef PLATFORM_EGL
    /** Dispaly context. */
    PTKDEMO_DisplayContext      dispCntxt;
#endif

    /** Counter for tracking number of times the data has been played.*/
    uint32_t                    runCtr;

    /** Flag to indicate if checksum needs to be performed on the final
     *  output accumulated map.
     */
    bool                        doChecksum;

    /** Expected value of checksum. This is looked at only if doChecksum
     *  is true.
     */
    uint32_t                    expectedChecksum;

    /** Computed checksum. */
    uint32_t                    computedChecksum;

} AppObj;

int32_t DOFSFMAPP_launchRenderThread(AppObj   * obj);
void DOFSFMAPP_exitRenderThread(AppObj * obj);

void DOFSFMAPP_renderInit(AppObj *obj);
void DOFSFMAPP_renderDeinit(AppObj *obj);
void DOFSFMAPP_renderAndSave(AppObj *obj, uint64_t curTimestamp);


#endif // _APP_DOF_SFM_OGMAP_RENDERER_H_

