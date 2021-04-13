 /*
 *******************************************************************************
 *
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
 *******************************************************************************
 */

#ifndef _SDE_MULTILAYER_APPLIB_H_
#define _SDE_MULTILAYER_APPLIB_H_

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
 * \defgroup group_applib_sde_multilayer Multi-layer SDE APPLIB code.
 * \ingroup group_ptk_applib
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <tivx_utils_graph_perf.h>
#include <utils/perf_stats/include/app_perf_stats.h>
#include <tivx_utils_file_rd_wr.h>

#define ML_SDEAPPLIB_DEFAULT_CORE_MAPPING  (TIVX_TARGET_DSP1)
#define ML_SDEAPPLIB_MAX_LINE_LEN          (256U)

/**
 * \brief Multi-layer SDE APPLIB create parameter context.
 *
 * \ingroup group_applib_sde_multilayer
 */
typedef struct
{
    /** OpenVX references */
    vx_context                        vxContext;

    /** OpenVX graph */
    vx_graph                          vxGraph;

    /** SDE params */
    tivx_dmpac_sde_params_t           sdeCfg;

    /** number of layers */
    uint8_t                           numLayers;

    /** input format, U8 or YUV_UYVY  */
    uint8_t                           inputFormat;

    /** median filtering flag */
    uint8_t                           enableMedianFilter;

    /** Input width */
    uint16_t                          width;

    /** Input height */
    uint16_t                          height;

    /** minimum Disparity */
    uint16_t                          minDisparity;

    /** maximum Disparity */
    uint16_t                          maxDisparity;

    /** Pipeline depth, 0, 1, ..., ML_SDEAPPLIB_MAX_PIPELINE_DEPTH */
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
    vx_image                          vxLeftRectImageL0[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Input recitifed right image object */
    vx_image                          vxRightRectImageL0[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Output image object */
    vx_image                          vxMergeDisparityL0[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Output image object */
    vx_image                          vxMedianFilteredDisparity[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Disparity Merge Node core mapping */
    const char                      * dispMergeNodeCore;

    /** Hole Filling Node core mapping */
    const char                      * holeFillingNodeCore;

    /** Median Filter Node core mapping */
    const char                      * medianFilterNodeCore;

    /** Disparity Viz Node Core mapping. */
    const char                      * dispVisNodeCore;

} ML_SDEAPPLIB_createParams;

struct ML_SDEAPPLIB_Context;
typedef ML_SDEAPPLIB_Context * ML_SDEAPPLIB_Handle;

/**
 * \brief Function to initialize the APPLIB.
 *
 * \param [in] createParams APPLIB create parameters.
 *
 * \return Valid handle on success. NULL otherwise.
 *
 * \ingroup group_applib_sde_multilayer
 */
ML_SDEAPPLIB_Handle ML_SDEAPPLIB_create(ML_SDEAPPLIB_createParams *createParams);

/**
 * \brief Function to de-init the APPLIB and release the memory associated with
 *        the handle.
 *
 * \param [in,out] handle Reference to APPLIB handle.
 *
 * \ingroup group_applib_sde_multilayer
 */
void                ML_SDEAPPLIB_delete(ML_SDEAPPLIB_Handle *handle);

/**
 * \brief Function to initialize Scalers' coefficients
 *
 * \param [in,out] handle Reference to APPLIB handle.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_sde_multilayer
 */
vx_status           ML_SDEAPPLIB_initScaler(ML_SDEAPPLIB_Handle handle);

/**
 * \brief Function to get the base-layer (layer 0) SDE node handle.
 *        This is needed when a graph is created outside APPLIB. 
 *        The caller calls this function to put this node into the graph.
 *
 * \param [in] handle Reference to APPLIB handle.
 *
 * \return SDE node
 *
 * \ingroup group_applib_sde_multilayer
 */
vx_node             ML_SDEAPPLIB_getSDENodeL0(ML_SDEAPPLIB_Handle handle);

/**
 * \brief Function to get the second-layer (layer 1) SDE node handle
 *        This is needed when a graph is created outside APPLIB. 
 *        The caller calls this function to put this node into the graph.
 *
 * \param [in] handle Reference to APPLIB handle.
 *
 * \return SDE node
 *
 * \ingroup group_applib_sde_multilayer
 */
vx_node             ML_SDEAPPLIB_getSDENodeL1(ML_SDEAPPLIB_Handle handle);

/**
 * \brief Function to get the third-layer (layer 2) SDE node handle. 
 *        This is needed when a graph is created outside APPLIB. 
 *        The caller calls this function to put this node into the graph.
 *
 * \param [in] handle Reference to APPLIB handle.
 *
 * \return SDE node
 *
 * \ingroup group_applib_sde_multilayer
 */
vx_node             ML_SDEAPPLIB_getSDENodeL2(ML_SDEAPPLIB_Handle handle);

/**
 * \brief Function to get the post-processing median fitler node.
 *        This is needed when a graph is created outside APPLIB. 
 *        The caller calls this function to put this node into the graph.
 * 
 * \param [in] handle Reference to APPLIB handle.
 *
 * \return Median filter node
 *
 * \ingroup group_applib_sde_multilayer
 */
vx_node             ML_SDEAPPLIB_getMedFilterNode(ML_SDEAPPLIB_Handle handle);

/**
 * \brief Function to get the merge node that combine disparity maps at
 *        layer 0 and layer 1. This is needed when a graph is
 *        created outside APPLIB. The caller calls this function  
 *        to put this node into the graph.
 * 
 * \param [in] handle Reference to APPLIB handle.
 *
 * \return Merge node
 *
 * \ingroup group_applib_sde_multilayer
 */
vx_node             ML_SDEAPPLIB_getMergeNodeL1(ML_SDEAPPLIB_Handle handle);

/**
 * \brief Function to get the merge node that combine disparity maps at
 *        layer 1 and layer 2. This is needed when a graph is created outside APPLIB.
 *        The caller calls this function to put this node into the graph.
  * 
 * \param [in] handle Reference to APPLIB handle.
 *
 * \return Merge node
 *
 * \ingroup group_applib_sde_multilayer
 */
vx_node             ML_SDEAPPLIB_getMergeNodeL2(ML_SDEAPPLIB_Handle handle);

/**
 * \brief Function to get the Scaler node that create down-sampled layer-1 
 *        left image from layer-0 left image. This is needed when a graph is
 *        created outside APPLIB. The caller calls this function  
 *        to put this node into the graph. 
 * 
 * \param [in] handle Reference to APPLIB handle.
 *
 * \return Scaler node
 *
 * \ingroup group_applib_sde_multilayer
 */
vx_node             ML_SDEAPPLIB_getLeftMSCNodeL1(ML_SDEAPPLIB_Handle handle);

/**
 * \brief Function to get the Scaler node that create down-sampled layer-1 
 *        right image from layer-0 right image. This is needed 
 *        when a graph is created outside APPLIB. The caller calls this function  
 *        to put this node into the graph. 
 * 
 * \param [in] handle Reference to APPLIB handle.
 *
 * \return Scaler node
 *
 * \ingroup group_applib_sde_multilayer
 */
vx_node             ML_SDEAPPLIB_getRightMSCNodeL1(ML_SDEAPPLIB_Handle handle);

/**
 * \brief Function to get the Scaler node that create down-sampled layer-2 
 *        left image from layer-1 left image. This is needed 
 *        when a graph is created outside APPLIB. The caller calls this function  
 *        to put this node into the graph.
 * 
 * \param [in] handle Reference to APPLIB handle.
 *
 * \return Scaler node
 *
 * \ingroup group_applib_sde_multilayer
 */
vx_node             ML_SDEAPPLIB_getLeftMSCNodeL2(ML_SDEAPPLIB_Handle handle);

/**
 * \brief Function to get the Scaler node that create down-sampled layer-2 
 *        right image from layer-1 right image. This is needed 
 *        when a graph is created outside APPLIB. The caller calls this function 
 *        to put this node into the graph. 
 * 
 * \param [in] handle Reference to APPLIB handle.
 *
 * \return Scaler node
 *
 * \ingroup group_applib_sde_multilayer
 */
vx_node             ML_SDEAPPLIB_getRightMSCNodeL2(ML_SDEAPPLIB_Handle handle);

#ifdef __cplusplus
}
#endif

#endif /* _SDE_MULTILAYER_APPLIB_H_ */

