 /*
 *******************************************************************************
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
 *******************************************************************************
 */

#ifndef _SDE_OBSTACLE_DETECTION_APPLIB_H_
#define _SDE_OBSTACLE_DETECTION_APPLIB_H_

#include <stdio.h>
#include <stdlib.h>

#include <TI/tivx.h>
#include <TI/tivx_debug.h>
#include <TI/tivx_mutex.h>
#include <TI/j7.h>
#include <TI/tivx_stereo.h>
#include <app_ptk_demo_common.h>


#include <queue>
#include <thread>
#include <mutex>

/**
 * \defgroup group_applib_sde_od Stereo based obstacle detection APPLIB code.
 * \ingroup group_ptk_applib
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <tivx_utils_graph_perf.h>
#include <utils/perf_stats/include/app_perf_stats.h>
#include <tivx_utils_file_rd_wr.h>

#define SODAPPLIB_DEFAULT_CORE_MAPPING  (TIVX_TARGET_DSP1)
#define SODAPPLIB_MAX_LINE_LEN          (256U)

/**
 * \brief Stereo based obstacle detection APPLIB create parameter context.
 *
 * \ingroup group_applib_sde_od
 */
typedef struct
{
    /** OpenVX references */
    vx_context                        vxContext;

    /** OpenVX graph */
    vx_graph                          vxGraph;

    /** SDE configuration parameters */
    tivx_dmpac_sde_params_t           sdeCfg;

    /** Stereo ground estimation configuration parameters. */
    tivx_ground_estimation_params_t   geCfg;

    /** Stereo obstacle detection configuration parameters. */
    tivx_obstacle_detection_params_t  sodCfg;

    /** Input width */
    uint32_t                          width;

    /** Input height */
    uint32_t                          height;

    /** Input format */
    uint8_t                           inputFormat;

    /** Pipeline depth, 0, 1, ..., SODAPPLIB_MAX_PIPELINE_DEPTH */
    uint8_t                           pipelineDepth;

    /** Input object pipeline depth, 
     *  which is the number of each input OpenVX object
     */
    uint8_t                           inputPipelineDepth;

    /** Flag indicating whether or not create input OpenVX object in applib */
    uint8_t                           createInputFlag;

    /** Flag indicating whether or not create output OpenVX object in applib */
    uint8_t                           createOutputFlag;

    /** Input recitifed left image object */
    vx_image                          vxRightRectImage[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Input raw disparity image object */
    vx_image                          vxSde16BitOutput[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Output obstacles' poses */
    vx_array                          vxObstaclesPose[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Output number of obstacles */
    vx_scalar                         vxNumObstacles[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Output free space */
    vx_array                          vxFreeSpaceBoundary[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Output drivable space */
    vx_user_data_object               vxDrivableSpace[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Disparity Extraction Core mapping. */
    const char                      * dispExtNodeCore;

    /** Disparity Viz Node Core mapping. */
    const char                      * dispVisNodeCore;

    /** GE Node Core mapping. */
    const char                      * geNodeCore;

    /** SOD Node Core mapping. */
    const char                      * sodNodeCore;

    /** FSD Node Core mapping. */
    const char                      * fsdNodeCore;

} SODAPPLIB_createParams;

struct SODAPPLIB_Context;
typedef SODAPPLIB_Context * SODAPPLIB_Handle;


/**
 * \brief Function to initialize the APPLIB.
 *
 * \param [in] createParams APPLIB create parameters.
 *
 * \return Valid handle on success. NULL otherwise.
 *
 * \ingroup group_applib_sde_od
 */
SODAPPLIB_Handle SODAPPLIB_create(SODAPPLIB_createParams *createParams);

/**
 * \brief Function to de-init the APPLIB and release the memory associated with
 *        the handle.
 *
 * \param [in,out] handle Reference to APPLIB handle.
 *
 * \ingroup group_applib_sde_od
 */
void             SODAPPLIB_delete(SODAPPLIB_Handle *handle);

/**
 * \brief Function to get the ground estimation node handle. This is needed 
 *        when a graph is created outside APPLIB. The caller calls this function 
 *        to put this node into the graph. 
 *
 * \param [in] handle Reference to APPLIB handle.
 *
 * \return Ground estimation node
 *
 * \ingroup group_applib_sde_od
 */
vx_node          SODAPPLIB_getGENode(SODAPPLIB_Handle handle);

/**
 * \brief Function to get the obstacle detection node handle. This is needed 
 *        when a graph is created outside APPLIB. The caller calls this function  
 *        to put this node into the graph. 
 *
 * \param [in] handle Reference to APPLIB handle.
 *
 * \return Obstacle detection node
 *
 * \ingroup group_applib_sde_od
 */
vx_node          SODAPPLIB_getSODNode(SODAPPLIB_Handle handle);


#ifdef __cplusplus
}
#endif

#endif /* _SDE_OBSTACLE_DETECTION_APPLIB_H_ */

