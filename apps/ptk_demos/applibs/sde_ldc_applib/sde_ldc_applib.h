 /*
 *******************************************************************************
 *
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
 *******************************************************************************
 */

#ifndef _SDELDC_APPLIB_H_
#define _SDELDC_APPLIB_H_

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
 * \defgroup group_applib_sde_ldc Stereo camera LDC APPLIB code.
 * \ingroup group_ptk_applib
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <tivx_utils_graph_perf.h>
#include <utils/perf_stats/include/app_perf_stats.h>
#include <tivx_utils_file_rd_wr.h>

#define SDELDCAPPLIB_DEFAULT_CORE_MAPPING  (TIVX_TARGET_DSP1)
#define SDELDCAPPLIB_MAX_LINE_LEN          (256U)


/**
 * \brief SDE LDC APPLIB create parameter context.
 *
 * \ingroup group_applib_sde_ldc
 */
typedef struct
{
    /** OpenVX references */
    vx_context                        vxContext;

    /** Graph handle from the Application */
    vx_graph                          vxGraph;

    /** Input image width */
    uint16_t                          width;

    /** Input image height */
    uint16_t                          height;

    /** Input iamge format: U8 or YUV_UYVY */
    uint16_t                          inputFormat;

    /** pipeline depth */
    uint8_t                           pipelineDepth;

    /** Input object pipeline depth, 
     *  which is the number of each input OpenVX object
     */
    uint8_t                           inputPipelineDepth;

    /** Output object pipeline depth, 
     *  which is the number of each output openVX object
     */
    uint8_t                           outputPipelineDepth;

    /** Flag indicating whether or not create input OpenVX object in applib */
    uint8_t                           createInputFlag;

    /** Flag indicating whether or not create output OpenVX object in applib */
    uint8_t                           createOutputFlag;

    /** Input left image object to a graph*/
    vx_image                          vxInputLeftImage[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Input right image object to a graph */
    vx_image                          vxInputRightImage[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Output left image object from a graph */
    vx_image                          vxOutputLeftImage[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** Output right image object from a graph */
    vx_image                          vxOutputRightImage[PTK_GRAPH_MAX_PIPELINE_DEPTH];

    /** left LUT file name */
    char *                            leftLutFileName;

    /** right LUT file name */
    char *                            rightLutFileName;

} SDELDCAPPLIB_createParams;

struct  SDELDCAPPLIB_Context;
typedef SDELDCAPPLIB_Context * SDELDCAPPLIB_Handle;

/**
 * \brief Function to initialize the APPLIB.
 *
 * \param [in] createParams APPLIB create parameters.
 *
 * \return Valid handle on success. NULL otherwise.
 *
 * \ingroup group_applib_sde_ldc
 */
SDELDCAPPLIB_Handle  SDELDCAPPLIB_create(SDELDCAPPLIB_createParams *createParams);

/**
 * \brief Function to de-init the APPLIB and release the memory associated with
 *        the handle.
 *
 * \param [in,out] handle Reference to APPLIB handle.
 *
 * \ingroup group_applib_sde_ldc
 */
void                 SDELDCAPPLIB_delete(SDELDCAPPLIB_Handle *handle);

/**
 * \brief Function to get the LDC node handle for left input image.
 *        This is needed when a graph is created outside APPLIB. 
 *        The caller calls this function to put this node into the graph.
 *
 * \param [in] handle Reference to APPLIB handle.
 *
 * \return LDC node 
 *
 * \ingroup group_applib_sde_ldc
 */
vx_node              SDELCDAPPLIB_getLeftLDCNode(SDELDCAPPLIB_Handle handle);

/**
 * \brief Function to get the LDC node handle for right input image.
 *        This is needed when a graph is created outside APPLIB. 
 *        The caller calls this function to put this node into the graph.
 * 
 * \param [in] handle Reference to APPLIB handle.
 *
 * \return LDC node 
 *
 * \ingroup group_applib_sde_ldc
 */
vx_node              SDELCDAPPLIB_getRightLDCNode(SDELDCAPPLIB_Handle handle);

/**
 * \brief Function to get the output left image object from a graph.
 *        This is needed when this object is passed to a connected graph 
 *        to build a bigger graph by a caller.
 *
 * \param [in] handle Reference to APPLIB handle.
 *
 * \return image object 
 *
 * \ingroup group_applib_sde_ldc
 */
vx_image             SDELDCAPPLIB_getOutputLeftImage(SDELDCAPPLIB_Handle handle);

/**
 * \brief Function to get the output right image object from a graph.
 *        This is needed when this object is passed to a connected graph 
 *        to build a bigger graph by a caller.
 *
 * \param [in] handle Reference to APPLIB handle.
 *
 * \return image object 
 *
 * \ingroup group_applib_sde_ldc
 */
vx_image             SDELDCAPPLIB_getOutputRightImage(SDELDCAPPLIB_Handle handle);



#ifdef __cplusplus
}
#endif

#endif /* _SDELDC_APPLIB_H_ */

