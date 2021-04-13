 /*
 *******************************************************************************
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
 *******************************************************************************
 */

#ifndef _APP_SDE_H_
#define _APP_SDE_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include <app_sde_main.h>


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
void      SDEAPP_parseCfgFile(SDEAPP_Context *appCntxt, const char *cfg_file_name);

/**
 * \brief Set LDC and SDE create parameters
 *
 * \param [in] appCntxt APP context
 * 
 */
void      SDEAPP_setAllParams(SDEAPP_Context *appCntxt);

/**
 * \brief Initialize app, e.g. create graph, load kernels, 
 *        initialize LDC node, SDE node, setup pipeline, etc.
 *
 * \param [in] appCntxt APP context
 * 
 */
int32_t   SDEAPP_init(SDEAPP_Context *appCntxt);

/**
 * \brief Lauch graph processing thread and event handling thread
 *
 * \param [in] appCntxt APP context
 * 
 */
void      SDEAPP_launchProcThreads(SDEAPP_Context *appCntxt);

/**
 * \brief Handle intercept signal
 *
 * \param [in] appCntxt APP context
 * 
 * \param [in] sig signal
 * 
 */
void      SDEAPP_intSigHandler(SDEAPP_Context *appCntxt, int sig);

/**
 * \brief Clean-up application, e.g. delete APPLIBs, graph and app.
 *
 * \param [in] appCntxt APP context
 * 
 */
void      SDEAPP_cleanupHdlr(SDEAPP_Context *appCntxt);

/**
 * \brief Delete display graph
 *
 * \param [in] appCntxt APP context
 * 
 */
void      SDEAPP_createDispalyGraph(SDEAPP_Context *appCntxt);

/**
 * \brief Initialized LDC Applib
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
int32_t   SDEAPP_init_LDC(SDEAPP_Context *appCntxt);

/**
 * \brief Initialized SDE Applib
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
int32_t   SDEAPP_init_SDE(SDEAPP_Context *appCntxt);

/**
 * \brief Setup graph pipeline
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
vx_status SDEAPP_setupPipeline(SDEAPP_Context * appCntxt);

/**
 * \brief Setup graph pipeline when single-layer SDE is employed
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
vx_status SDEAPP_setupPipeline_SL(SDEAPP_Context * appCntxt);

/**
 * \brief Setup graph pipeline when multi-layer SDE is employed
 *
 * \param [in] appCntxt APP context
 * 
 * \return VX_SUCCESS on success
 * 
 */
vx_status SDEAPP_setupPipeline_ML(SDEAPP_Context * appCntxt);

/**
 * \brief Function to print the performance statistics to stdout.
 *
 * \param [in] appCntxt APP context
 *
 */
void      SDEAPP_printStats(SDEAPP_Context * appCntxt);

/**
 * \brief Function to export the performance statistics to a file.
 *
 * \param [in] appCntxt APP context
 *
 */
void      SDEAPP_exportStats(SDEAPP_Context * appCntxt);

/**
 * \brief Function to wait for the pending graph execution completions.
 *
 * \param [in] appCntxt APP context
 *
 */
void      SDEAPP_waitGraph(SDEAPP_Context * appCntxt);

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
vx_status SDEAPP_getFreeParamRsrc(SDEAPP_Context       *appCntxt,
                                  SDEAPP_graphParams   **gpDesc);

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
vx_status SDEAPP_process(SDEAPP_Context * appCntxt,  SDEAPP_graphParams * gpDesc);

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
int32_t   SDEAPP_processEvent(SDEAPP_Context * appCntxt, vx_event_t * event);


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
int32_t   SDEAPP_releaseParamRsrc(SDEAPP_Context  *appCntxt, uint32_t rsrcIndex);

/**
 * \brief Function to returns references to the input and output at the
 *        head of the output queue. The data is not popped out of the queue.
 *        If the caller calls this functions repeatedly without calling
 *        SDEAPP_releaseOutBuff() in between, then the same output
 *        is returned.
 *
 *        Once the graph execution is complete, the output buffer is stored in 
 *        a queue for the use by the calling application. The caller can use
 *        this API to get a reference to the input/output image pair for post
 *        processing. One the caller is done with the input/output pair, a call
 *        must be made to SDEAPP_releaseOutBuff()for releasing the
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
int32_t   SDEAPP_getOutBuff(SDEAPP_Context *appCntxt, vx_image *rightRectImage, vx_image *disparity16);


/**
 * \brief Function to release buffer in the output queue by moving 
 *        the buffer to the free queue
 *
 * \param [in] appCntxt APP context
 *
 */
void      SDEAPP_releaseOutBuff(SDEAPP_Context * appCntxt);

#ifdef __cplusplus
}
#endif


#endif /* _APP_SDE_H_ */

