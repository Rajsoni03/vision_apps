/*
 *
 * Copyright (c) 2019 Texas Instruments Incorporated
 *
 * All rights reserved not granted herein.
 *
 * Limited License.
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive
 * license under copyrights and patents it now or hereafter owns or controls to make,
 * have made, use, import, offer to sell and sell ("Utilize") this software subject to the
 * terms herein.  With respect to the foregoing patent license, such license is granted
 * solely to the extent that any such patent is necessary to Utilize the software alone.
 * The patent license shall not apply to any combinations which include this software,
 * other than combinations with devices manufactured by or for TI ("TI Devices").
 * No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce this license
 * (including the above copyright notice and the disclaimer and (if applicable) source
 * code license limitations below) in the documentation and/or other materials provided
 * with the distribution
 *
 * Redistribution and use in binary form, without modification, are permitted provided
 * that the following conditions are met:
 *
 * *       No reverse engineering, decompilation, or disassembly of this software is
 * permitted with respect to any software provided in binary form.
 *
 * *       any redistribution and use are licensed by TI for use only with TI Devices.
 *
 * *       Nothing shall obligate TI to provide you with source code for the software
 * licensed and provided to you in object code.
 *
 * If software source code is provided to you, modification and redistribution of the
 * source code are permitted provided that the following conditions are met:
 *
 * *       any redistribution and use of the source code, including any resulting derivative
 * works, are licensed by TI for use only with TI Devices.
 *
 * *       any redistribution and use of any object code compiled from the source code
 * and any resulting derivative works, are licensed by TI for use only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its suppliers
 *
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef _RADAR_GTRACK_APPLIB_PRIV_H_
#define _RADAR_GTRACK_APPLIB_PRIV_H_

#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <mutex>
#include <queue>

#include "TI/tivx_target_kernel.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <tivx_utils_graph_perf.h>
#include <utils/perf_stats/include/app_perf_stats.h>    

#ifdef __cplusplus
}
#endif

#include <perception/perception.h>
#include <perception/utils/bit_utils.h>

#include <app_ptk_demo_common.h>
#include <radar_gtrack_applib.h>

/** Number of graph parameters:
 * - Object data input
 * - Sensor configuration
 * - Position information
 * - Out track information
 */
#define RADAR_GTRACK_APPLIB_NUM_GRAPH_PARAMS      (4U)

/* The following node parameter completion event is used to manage
 * the parameter buffer recycling.
 */
#define RADAR_GTRACK_APPLIB_GRAPH_COMPLETE_EVENT  (0U)

typedef struct
{
    /* Graph parameter 0. */
    vx_user_data_object     vxSensorCfg;

    /* Graph parameter 1. */
    vx_user_data_object     vxObjData;

    /* Graph parameter 2 */
    vx_user_data_object     vxPoseAndRef;

    /* Graph parameter 3 */
    vx_user_data_object     vxOutTrackInfo;

} RADAR_GTRACK_APPLIB_graphParams;

using RADAR_GTRACK_APPLIB_graphParamQ = std::queue<RADAR_GTRACK_APPLIB_graphParams*>;

typedef struct RADAR_GTRACK_APPLIB_Context
{
    /* OpenVX references */
    vx_context                      vxContext;
    vx_graph                        vxGraph;
    vx_node                         vxGTrackNode;
    vx_user_data_object             vxTrackConfig;
    vx_user_data_object             vxObjData[RADAR_GTRACK_APPLIB_PIPELINE_DEPTH];
    vx_user_data_object             vxPoseAndRef[RADAR_GTRACK_APPLIB_PIPELINE_DEPTH];
    vx_user_data_object             vxOutTrackInfo[RADAR_GTRACK_APPLIB_PIPELINE_DEPTH];
    vx_user_data_object             vxSensorCfg[RADAR_GTRACK_APPLIB_PIPELINE_DEPTH];

    vx_map_id                       vxOutInfoLockId;

    /* OGMAP configuration. */
    PTK_Alg_RadarGTrackParams       gTrackParams;

    /* Radar sensor configuration. */
    PTK_Alg_RadarOgmapParams        ogConfig;

    PTK_Alg_RadarDetOutput          parsedObjData;

    /** GTRACK Node Core mapping. */
    const char                     *gTrackNodeCore;

    /** Number of sensors enabled. */
    uint32_t                        numSensors;

    /** Graph processing thread. */
    std::thread                     evtHdlrThread;

    /** Resource lock. */
    std::mutex                      paramRsrcMutex;

    /** Descriptor pool. */
    RADAR_GTRACK_APPLIB_graphParams paramDesc[RADAR_GTRACK_APPLIB_PIPELINE_DEPTH];

    /** Queue for output processing. */
    RADAR_GTRACK_APPLIB_graphParamQ freeQ;

    /** Queue for output processing. */
    RADAR_GTRACK_APPLIB_graphParamQ outputQ;

    /** Performance tracking context. */
    app_perf_point_t                perf;

    /** Flag to track if the performance counter has been initialized. */
    bool                            startPerfCapt;

    /** Base value to be used for any programmed VX events. */
    uint8_t                         vxEvtAppValBase;

    /** Pipeline depth. */
    uint8_t                         pipelineDepth;

    /** Flag to indicate if the graph should be exported
     * 0 - disable
     * 1 - enable
     */
    uint8_t                         exportGraph;

    /** Real-time logging enable.
     * 0 - disable
     * 1 - enable
     */
    uint8_t                         rtLogEnable;

} RADAR_GTRACK_APPLIB_Context;

vx_status RADAR_GTRACK_APPLIB_getFreeParamRsrc(
        RADAR_GTRACK_APPLIB_Context        *appCntxt,
        RADAR_GTRACK_APPLIB_graphParams    *gpDesc);

vx_status RADAR_GTRACK_APPLIB_setupPipeline(
        RADAR_GTRACK_APPLIB_Context    *appCntxt);

vx_status RADAR_GTRACK_APPLIB_releaseParamRsrc(
        RADAR_GTRACK_APPLIB_Context    *appCntxt,
        uint32_t                        rsrcIndex);

#endif /* _RADAR_GTRACK_APPLIB_PRIV_H_ */

