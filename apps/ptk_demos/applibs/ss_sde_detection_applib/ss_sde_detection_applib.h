 /*
 *******************************************************************************
 *
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
 *******************************************************************************
 */

#ifndef _SS_SDE_DETECTION_APPLIB_H_
#define _SS_SDE_DETECTION_APPLIB_H_

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
 * \defgroup group_applib_sssde_od Stereo and semantic segmentation based obstacle detection APPLIB code.
 * \ingroup group_ptk_applib
 *
 */


#ifdef __cplusplus
extern "C" {
#endif

#include <tivx_utils_graph_perf.h>
#include <utils/perf_stats/include/app_perf_stats.h>
#include <tivx_utils_file_rd_wr.h>

#define SS_DETECT_APPLIB_DEFAULT_CORE_MAPPING  (TIVX_TARGET_DSP1)

#define SS_DETECT_APPLIB_MAX_PIPELINE_DEPTH    (2U)
#define SS_DETECT_APPLIB_NUM_GRAPH_PARAMS      (4U)

#define SS_DETECT_APPLIB_GRAPH_COMPLETE_EVENT  (0U)

#define SS_DETECT_APPLIB_MAX_LINE_LEN          (256U)

typedef struct
{
    /* Graph parameter 0 */
    vx_image                vxRightRectImage;

    /* Graph parameter 1 */
    vx_image                vxSde16BitOutput;

    /* Graph parameter 2 */
    vx_tensor               vxSSMapTensor;

    /* Graph parameter 3 */
    vx_user_data_object     vx3DBoundBox;

} SS_DETECT_APPLIB_graphParams;

using SS_DETECT_APPLIB_graphParamQ = std::queue<SS_DETECT_APPLIB_graphParams*>;


/**
 * \brief Stereo and semantic segmentation based obstacle detection APPLIB create parameter context.
 *
 * \ingroup group_applib_sssde_od
 */
typedef struct
{
    /** OpenVX references */
    vx_context                        vxContext;

    /** Graph handle from the Application. If this is NULL, the the APPLIB will
     * create an internal graph and will manage it.
     *
     * In case the Application would like to manage the graph but want the
     * applib to create a sub-graph and attach it to the graph then this 
     * should be allocated by the Application and pass it in.
     */
    vx_graph                          vxGraph;

    /** Input object pipeline depth */
    uint8_t                           createInputFlag;

    /** Input object pipeline depth */
    uint8_t                           createOutputFlag;

    /** input image/disparity width */
    int16_t                           width;

    /** input image/disparity height */
    int16_t                           height;

    /** input tensor (ss map) width */
    int16_t                           tensorWidth;

    /** input tensor (ss map) height */
    int16_t                           tensorHeight;

    /** Point cloud createion parameters. */
    tivx_ss_sde_point_cloud_params_t  pcCfg;

    /** Occupancy Grid based detection configuration parameters. */
    tivx_ss_sde_og_detection_params_t ogCfg;

    /** Base value to be used for any programmed VX events. */
    uint32_t                          vxEvtAppValBase;

    /** Pipeline depth, 0, 1, ..., SS_DETECT_APPLIB_MAX_PIPELINE_DEPTH */
    uint8_t                           pipelineDepth;

    /** input image format: 0: bmp (U8), 1: yuv420 */
    uint8_t                           inputFormat;

    /** Input image */
    vx_image                          vxRightRectImage[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Input raw disparity */
    vx_image                          vxSde16BitOutput[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Input semantic segmentation tensor */
    vx_tensor                         vxSSMapTensor[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Is it from ROS or not */
    uint8_t                           isROSInterface;

    /** Flag to indicate if the graph should be exported
     * 0 - disable
     * 1 - enable
     */
    uint8_t                           exportGraph;

    /** Real-time logging enable.
     * 0 - disable
     * 1 - enable
     */
    uint8_t                           rtLogEnable;

    /** Point Clound Node Core mapping. */
    const char                      * pcNodeCore;

    /** Occupancy Grid Core mapping. */
    const char                      * ogNodeCore;

}  SS_DETECT_APPLIB_createParams;

struct  SS_DETECT_APPLIB_Context;
typedef SS_DETECT_APPLIB_Context * SS_DETECT_APPLIB_Handle;


/**
 * \brief Function to initialize the APPLIB.
 *
 * \param [in] createParams APPLIB create parameters.
 *
 * \return Valid handle on success. NULL otherwise.
 *
 * \ingroup group_applib_sssde_od
 */
SS_DETECT_APPLIB_Handle SS_DETECT_APPLIB_create(SS_DETECT_APPLIB_createParams *createParams);

/**
 * \brief Function to process inputs. This function is non-blocking
 *        and returns success if the pipeline is not full.
 *
 * \param [in] handle APPLIB handle.
 *
 * \param [in] rightImage right image
 *
 * \param [in] disparity16 raw disparity map
 * 
 * \param [in] ssMap semantic segmentation map
 * 
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_sssde_od
 */
int32_t                 SS_DETECT_APPLIB_process(SS_DETECT_APPLIB_Handle handle, vx_image rightImage, 
                                                 vx_image disparity16, vx_tensor ssMap);

/**
 * \brief Function to de-init the APPLIB and release the memory associated with
 *        the handle.
 *
 * \param [in,out] handle Reference to APPLIB handle.
 *
 * \ingroup group_applib_sde_od
 */
void                    SS_DETECT_APPLIB_delete(SS_DETECT_APPLIB_Handle *handle);

/**
 * \brief Function to print the performance statistics to stdout.
 *
 * \param [in] handle APPLIB handle.
 *
 * \ingroup group_applib_sde_od
 */
void                    SS_DETECT_APPLIB_printStats(SS_DETECT_APPLIB_Handle handle);

/**
 * \brief Function to export the performance statistics to a file.
 *
 * \param [in] handle APPLIB handle.
 *
 * \ingroup group_applib_sde_od
 */
void                    SS_DETECT_APPLIB_exportStats(SS_DETECT_APPLIB_Handle handle);

/**
 * \brief Function to release the output buffer at the head of the output queue.
 *        Once the graph execution is complete, the output buffer is stored in 
 *        a queue for the use by the calling application. The caller is 
 *        responsible for calling this function to release the output buffers
 *        in a timely manner. Failing to do will result in subsequent graph
 *        execution failures due to non-availability of the output buffers.
 *
 * \param [in] handle APPLIB handle.
 *
 * \ingroup group_applib_sde_od
 */
void                    SS_DETECT_APPLIB_releaseOutBuff(SS_DETECT_APPLIB_Handle handle);

/**
 * \brief Function to returns references to the input recifitied image, output
 *        disparity and obstacle bounding box at the head of the output queue. 
 *        The data is not popped out of the queue.
 *        If the caller calls this functions repeatedly without calling
 *        SS_DETECT_APPLIB_releaseOutBuff() in between, then the same output
 *        is returned.
 *
 *        Once the graph execution is complete, the output buffer is stored in 
 *        a queue for the use by the calling application. The caller can use
 *        this API to get a reference to the input/output image pair for post
 *        processing. One the caller is done with the input/output pair, a call
 *        must be made to SS_DETECT_APPLIB_releaseOutBuff()for releasing the
 *        buffered input/output pair. Failing to do will result in subsequent
 *        graph execution failures due to non-availability of the output
 *        buffers.
 *
 * \param [in] handle APPLIB handle.
 *
 * \param [out] inputImg Input image passed to the graph.
 *
 * \param [out] output Reference to the output object from the graph
 *              corresponding to the 'inputImage'. The reference could
 *              be either a tensor or an image. See following.
 *              - output refere to a tensor if enablePostProcNode is false
 *              - output refere to an image if enablePostProcNode is true
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_sde_od
 */
int32_t                 SS_DETECT_APPLIB_getOutBuff(SS_DETECT_APPLIB_Handle handle, vx_image *rightRectImage, 
                                                    vx_image *disparity16, vx_user_data_object * obsBB);

/**
 * \brief Function for processing the events programmed to be handled by the
 *        APPLIB. Currently only VX_EVENT_GRAPH_COMPLETED event is handled.
 *
 *        The caller must invoke this handler when it receives a graph
 *        completion event. The APPLIB needs to deueue the graph parameters
 *        and buffer the output for use by the caller and these tasks are
 *        performed inside this handler.
 *
 * \param [in] handle APPLIB handle.
 *
 * \param [in] event Event context.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_sde_od
 */
int32_t                 SS_DETECT_APPLIB_processEvent(SS_DETECT_APPLIB_Handle handle, vx_event_t * event);

/**
 * \brief Function to reset the performance and other APPLIB specific counters.
 *
 * \param [in] handle APPLIB handle.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_sde_od
 */
int32_t                 SS_DETECT_APPLIB_reset(SS_DETECT_APPLIB_Handle handle);

/**
 * \brief Function to wait for the pending graph execution completions. This is
 *        relavant only if the graph is created and managed by the APPLIB.
 *
 *        If the graph is created and manged by the APPLIB, then this function
 *        could potentially block until the current graph execution is
 *        is complete.
 *
 *        If the graph is created and managed by the caller, then this function
 *        always returns immediately.
 *
 * \param [in] handle APPLIB handle.
 *
 * \ingroup group_applib_sde_od
 */
void                    SS_DETECT_APPLIB_waitGraph(SS_DETECT_APPLIB_Handle handle);

/**
 * \brief Function to get 3D Bounding Box object. This is needed when this object  
 *        is passed to a connected graph to build a bigger graph by a caller.
 *
 * \param [in] handle APPLIB handle.
 *
 *
 * \ingroup group_applib_sde_od
 */
vx_user_data_object     SS_DETECT_APPLIB_get3DBBObject(SS_DETECT_APPLIB_Handle handle, int16_t index);


/**
 * \brief Function to get point cloud creation node. This is needed when a graph 
 *        is created outside APPLIB. The caller calls this function to put this node 
 *        into the graph. 
 *
 * \param [in] handle APPLIB handle.
 *
 * \return VX_SUCCESS on success
 * 
 * \ingroup group_applib_sde_od
 */
vx_node                 SS_DETECT_APPLIB_getPCNode(SS_DETECT_APPLIB_Handle handle);

/**
 * \brief Function to get occupancy grid creation node. This is needed when a graph 
 *        is created outside APPLIB. The caller calls this function to put this node 
 *        into the graph. 
 *
 * \param [in] handle APPLIB handle.
 *
 *
 * \ingroup group_applib_sde_od
 */
vx_node                 SS_DETECT_APPLIB_getOGNode(SS_DETECT_APPLIB_Handle handle);

#ifdef __cplusplus
}
#endif

#endif /* _SS_SDE_DETECTION_APPLIB_H_ */

