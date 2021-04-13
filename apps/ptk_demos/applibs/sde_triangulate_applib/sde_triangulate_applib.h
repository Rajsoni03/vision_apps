 /*
 *******************************************************************************
 *
 * Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
 *******************************************************************************
 */

#ifndef _SDE_TRIANGULATE_APPLIB_H_
#define _SDE_TRIANGULATE_APPLIB_H_

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
 * \defgroup group_applib_sde_triangulate SDE Triangulate APPLIB code.
 * \ingroup  group_ptk_applib
 *
 */


#ifdef __cplusplus
extern "C" {
#endif

#include <tivx_utils_graph_perf.h>
#include <utils/perf_stats/include/app_perf_stats.h>
#include <tivx_utils_file_rd_wr.h>

#define SDE_TRIANG_APPLIB_DEFAULT_CORE_MAPPING  (TIVX_TARGET_DSP1)
#define SDE_TRIANG_APPLIB_MAX_LINE_LEN          (256U)

/**
 * \brief  SDE Triangulate APPLIB create parameter context.
 *
 * \ingroup group_applib_sde_triangulate
 */
typedef struct
{
    /** OpenVX references */
    vx_context                        vxContext;

    /** OpenVX graph */
    vx_graph                          vxGraph;

    /** Input width */
    uint16_t                          width;

    /** Input height */
    uint16_t                          height;

    /** Stereo camera params */
    tivx_stereo_cam_params_t          stereoCamCfg;

    /** Stereo pointcolud params */
    tivx_stereo_pointcloud_params_t   stereoPcCfg;
    
    /** Pipeline depth, 0, 1, ..., SDE_TRIANG_APPLIB_MAX_PIPELINE_DEPTH */
    uint8_t                           pipelineDepth;

    /** Input object pipeline depth, 
     *  which is the number of each input OpenVX object
     */
    uint8_t                           inputPipelineDepth;

    /** Flag indicating whether or not create input OpenVX object in applib */
    uint8_t                           createInputFlag;

    /** Flag indicating whether or not create output OpenVX object in applib */
    uint8_t                           createOutputFlag;

    /** Input recitifed image object */
    vx_image                          vxInputRectImage[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Input SDE disaprity */
    vx_image                          vxInputSde16Bit[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Output point cloud */
    vx_user_data_object               vxOutputTriangPC[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Color Conversion Node Core mapping. */
    const char                      * ccNodeCore;

    /** Triangulation Node Core mapping. */
    const char                      * triangNodeCore;

} SDE_TRIANG_APPLIB_createParams;

struct  SDE_TRIANG_APPLIB_Context;
typedef SDE_TRIANG_APPLIB_Context * SDE_TRIANG_APPLIB_Handle;


/**
 * \brief Function to initialize the APPLIB.
 *
 * \param [in] createParams APPLIB create parameters.
 *
 * \return Valid handle on success. NULL otherwise.
 *
 * \ingroup group_applib_sde_triangulate
 */
SDE_TRIANG_APPLIB_Handle SDE_TRIANG_APPLIB_create(SDE_TRIANG_APPLIB_createParams *createParams);

/**
 * \brief Function to de-init the APPLIB and release the memory associated with
 *        the handle.
 *
 * \param [in,out] handle Reference to APPLIB handle.
 *
 * \ingroup group_applib_sde_triangulate
 */
void                      SDE_TRIANG_APPLIB_delete(SDE_TRIANG_APPLIB_Handle *handle);



/**
 * \brief Function to get the SDE Triangulate node handle. This is needed when a graph 
 *        is created outside APPLIB. The caller calls this function to put this node 
 *        into the graph. 
 *
 * \param [in] handle Reference to APPLIB handle.
 *
 * \return Image Color Conversion node
 *
 * \ingroup group_applib_sde_triangulate
 */
vx_node                   SDE_TRIANG_APPLIB_getColorConvNode(SDE_TRIANG_APPLIB_Handle handle);


/**
 * \brief Function to get the SDE Triangulate node handle. This is needed when a graph 
 *        is created outside APPLIB. The caller calls this function to put this node 
 *        into the graph. 
 *
 * \param [in] handle Reference to APPLIB handle.
 *
 * \return SDE Triangulate node
 *
 * \ingroup group_applib_sde_triangulate
 */
vx_node                   SDE_TRIANG_APPLIB_getTriangNode(SDE_TRIANG_APPLIB_Handle handle);

#ifdef __cplusplus
}
#endif

#endif /* _SDE_TRIANGULATE_APPLIB_H_ */

