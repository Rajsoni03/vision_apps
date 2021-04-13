 /*
 *******************************************************************************
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
 *******************************************************************************
 */

#ifndef _APP_SDE_OBSTACLE_DETECTION_H_
#define _APP_SDE_OBSTACLE_DETECTION_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include <app_sde_obstacle_detection_main.h>


#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Parse application config parameter
 *
 * \param [in] appCntxt APP context
 * 
 * \param [in] cfg_file_name configuration file name
 * 
 */
void      SODAPP_parseCfgFile(SODAPP_Context *appCntxt, const char *cfg_file_name);

/**
 * \brief Set LDC, SDE and Obstacle Detection create parameters
 *
 * \param [in] appCntxt APP context
 * 
 */
void      SODAPP_setAllParams(SODAPP_Context *appCntxt);

/**
 * \brief Initialize app, e.g. create graph, load kernels, 
 *        initialize LDC node, SDE node, Obstacle Detection node and 
 *        setup pipeline, etc.
 *
 * \param [in] appCntxt APP context
 * 
 */
void      SODAPP_init(SODAPP_Context *appCntxt);


/**
 * \brief Lauch graph processing thread and event handling thread
 *
 * \param [in] appCntxt APP context
 * 
 */
void      SODAPP_launchProcThreads(SODAPP_Context *appCntxt);

/**
 * \brief Handle intercept signal
 *
 * \param [in] appCntxt APP context
 * 
 * \param [in] sig signal
 * 
 */
void      SODAPP_intSigHandler(SODAPP_Context *appCntxt, int sig);

/**
 * \brief Clean-up application, e.g. delete APPLIBs, graph and app.
 *
 * \param [in] appCntxt APP context
 * 
 */
void      SODAPP_cleanupHdlr(SODAPP_Context *appCntxt);

/**
 * \brief Delete display graph
 *
 * \param [in] appCntxt APP context
 * 
 */
void      SODAPP_createDispalyGraph(SODAPP_Context *appCntxt);

/**
 * \brief Function to overlay obstacles' bounding box and free space
 *        on the rectified image
 *
 * \param [in] appCntxt APP context
 * 
 */
void      SODAPP_overlayBBAndFreeSpace(SODAPP_Context *appCntxt);

/**
 * \brief Initialized LDC Applib
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
int32_t   SODAPP_init_LDC(SODAPP_Context *appCntxt);


/**
 * \brief Initialized SDE Applib
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
int32_t   SODAPP_init_SDE(SODAPP_Context *appCntxt);

/**
 * \brief Initialized Obstacle Detection Applib
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
int32_t   SODAPP_init_Detection(SODAPP_Context *appCntxt);

/**
 * \brief Setup graph pipeline
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
vx_status SODAPP_setupPipeline(SODAPP_Context * appCntxt);

/**
 * \brief Setup graph pipeline when single-layer SDE is employed
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
vx_status SODAPP_setupPipeline_SL(SODAPP_Context * appCntxt);

/**
 * \brief Setup graph pipeline when multi-layer SDE is employed
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
vx_status SODAPP_setupPipeline_ML(SODAPP_Context * appCntxt);

/**
 * \brief Function to print the performance statistics to stdout.
 *
 * \param [in] appCntxt APP context
 *
 */
void      SODAPP_printStats(SODAPP_Context * appCntxt);

/**
 * \brief Function to export the performance statistics to a file.
 *
 * \param [in] appCntxt APP context
 *
 */
void      SODAPP_exportStats(SODAPP_Context * appCntxt);

/**
 * \brief Function to wait for the pending graph execution completions.
 *
 * \param [in] appCntxt APP context
 *
 */
void      SODAPP_waitGraph(SODAPP_Context * appCntxt);

/**
 * \brief Function to get free resource for the free queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 */
vx_status SODAPP_getFreeParamRsrc(SODAPP_Context       *appCntxt,
                                  SODAPP_graphParams   **gpDesc);

/**
 * \brief Function to process input images. This function is non-blocking
 *        and returns success if the pipeline is not full.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 */
vx_status SODAPP_process(SODAPP_Context * appCntxt,  SODAPP_graphParams * gpDesc);

/**
 * \brief Function to process the events programmed to be handled by the
 *        APPLIB. Currently only VX_EVENT_GRAPH_COMPLETED event is handled.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 */
int32_t   SODAPP_processEvent(SODAPP_Context * appCntxt, vx_event_t * event);

/**
 * \brief Function to mark the dequeued resource as free by moving dequeued 
 *        resource to the output queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] rsrcIndex index of paramDesc that will be moved to the output queue
 *
 * \return VX_SUCCESS on success
 *
 */
int32_t   SODAPP_releaseParamRsrc(SODAPP_Context  *appCntxt, uint32_t rsrcIndex);

/**
 * \brief Function to returns references to the input and output at the
 *        head of the output queue. The data is not popped out of the queue.
 *        If the caller calls this functions repeatedly without calling
 *        SODAPP_releaseOutBuff() in between, then the same output
 *        is returned.
 *
 *        Once the graph execution is complete, the output buffer is stored in 
 *        a queue for the use by the calling application. The caller can use
 *        this API to get a reference to the input/output image pair for post
 *        processing. One the caller is done with the input/output pair, a call
 *        must be made to SODAPP_releaseOutBuff()for releasing the
 *        buffered input/output pair. Failing to do will result in subsequent
 *        graph execution failures due to non-availability of the output
 *        buffers.
 *
 * \param [in] appCntxt APP context
 *
 * \param [out] rightRectImage right rectified image passed from a graph
 * 
 * \param [out] disparity16 output raw disparity map passed from a graph
 *
 * \return VX_SUCCESS on success
 * 
 */
int32_t   SODAPP_getOutBuff(SODAPP_Context *appCntxt, uint32_t * numObstacles, vx_image *rightRectImage, vx_image* disparity16,
                            tivx_obstacle_pos_t * obsBox, int32_t * fsBoundary, tivx_drivable_space_t * drivableSpace);


/**
 * \brief Function to release buffer in the output queue by moving 
 *        the buffer to the free queue
 *
 * \param [in] appCntxt APP context
 *
 */
void      SODAPP_releaseOutBuff(SODAPP_Context * appCntxt);


#ifdef __cplusplus
}
#endif

#endif /* _APP_SDE_OBSTACLE_DETECTION_H_ */

