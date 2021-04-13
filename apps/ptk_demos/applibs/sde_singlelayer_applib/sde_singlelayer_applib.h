 /*
 *******************************************************************************
 *
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
 *******************************************************************************
 */

#ifndef _SDE_SINGLELAYER_APPLIB_H_
#define _SDE_SINGLELAYER_APPLIB_H_

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
 * \defgroup group_applib_sde_singlelayer Single-layer SDE APPLIB code.
 * \ingroup group_ptk_applib
 *
 */


#ifdef __cplusplus
extern "C" {
#endif

#include <tivx_utils_graph_perf.h>
#include <utils/perf_stats/include/app_perf_stats.h>
#include <tivx_utils_file_rd_wr.h>

#define SL_SDEAPPLIB_DEFAULT_CORE_MAPPING  (TIVX_TARGET_DSP1)
#define SL_SDEAPPLIB_MAX_LINE_LEN          (256U)

/**
 * \brief Single-layer SDE APPLIB create parameter context.
 *
 * \ingroup group_applib_sde_singlelayer
 */
typedef struct
{
    /** OpenVX references */
    vx_context                        vxContext;

    /** OpenVX graph */
    vx_graph                          vxGraph;

    /** SDE params */
    tivx_dmpac_sde_params_t           sdeCfg;

    /** Input width */
    uint16_t                          width;

    /** Input height */
    uint16_t                          height;

    /** Input format, U8 or YUV_UYVY */
    uint8_t                           inputFormat;

    /** disparity map confidence threshold for visualization */
    uint8_t                           vis_confidence;

    /** minimum Disparity */
    uint16_t                          minDisparity;

    /** maximum Disparity */
    uint16_t                          maxDisparity;

    /** Pipeline depth, 0, 1, ..., SL_SDEAPPLIB_MAX_PIPELINE_DEPTH */
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
    vx_image                          vxLeftRectImage[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Input recitifed right image object */
    vx_image                          vxRightRectImage[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Output image object */
    vx_image                          vxSde16BitOutput[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Disparity Viz Node Core mapping. */
    const char                      * dispVisNodeCore;

} SL_SDEAPPLIB_createParams;

struct  SL_SDEAPPLIB_Context;
typedef SL_SDEAPPLIB_Context * SL_SDEAPPLIB_Handle;


/**
 * \brief Function to initialize the APPLIB.
 *
 * \param [in] createParams APPLIB create parameters.
 *
 * \return Valid handle on success. NULL otherwise.
 *
 * \ingroup group_applib_sde_singlelayer
 */
SL_SDEAPPLIB_Handle SL_SDEAPPLIB_create(SL_SDEAPPLIB_createParams *createParams);

/**
 * \brief Function to de-init the APPLIB and release the memory associated with
 *        the handle.
 *
 * \param [in,out] handle Reference to APPLIB handle.
 *
 * \ingroup group_applib_sde_singlelayer
 */
void                SL_SDEAPPLIB_delete(SL_SDEAPPLIB_Handle *handle);

/**
 * \brief Function to get the SDE node handle. This is needed when a graph 
 *        is created outside APPLIB. The caller calls this function to put this node 
 *        into the graph. 
 *
 * \param [in] handle Reference to APPLIB handle.
 *
 * \return LDC node
 *
 * \ingroup group_applib_sde_singlelayer
 */
vx_node             SL_SDEAPPLIB_getSDENode(SL_SDEAPPLIB_Handle handle);

#ifdef __cplusplus
}
#endif

#endif /* _SDE_SINGLELAYER_APPLIB_H_ */

