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
#ifndef _LIDAR_OGMAP_APPLIB_PRIV_H_
#define _LIDAR_OGMAP_APPLIB_PRIV_H_

#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <mutex>
#include <queue>

#include <perception/perception.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <tivx_utils_graph_perf.h>
#include <utils/perf_stats/include/app_perf_stats.h>    

#ifdef __cplusplus
}
#endif

#include "TI/tivx_target_kernel.h"

#include <app_ptk_demo_common.h>
#include <lidar_ogmap_applib.h>

/** Number of graph parameters:
 * - Lidar Point cloud input to GPC node
 * - Lidar Meta data input to GPC and MDC nodes
 * - Position information input to OG node
 * - PFSD output descriptor
 * - Output instantaneous map
 */
#define LIDAROGAPPLIB_GRAPH_PARAM_PC        (0U)
#define LIDAROGAPPLIB_GRAPH_PARAM_META      (LIDAROGAPPLIB_GRAPH_PARAM_PC+1U)
#define LIDAROGAPPLIB_GRAPH_PARAM_POSREF    (LIDAROGAPPLIB_GRAPH_PARAM_META+1U)
#define LIDAROGAPPLIB_GRAPH_PARAM_PFSD_DESC (LIDAROGAPPLIB_GRAPH_PARAM_POSREF+1U)
#define LIDAROGAPPLIB_GRAPH_PARAM_INST_MAP  (LIDAROGAPPLIB_GRAPH_PARAM_PFSD_DESC+1U)
#define LIDAROGAPPLIB_NUM_GRAPH_PARAMS      (LIDAROGAPPLIB_GRAPH_PARAM_INST_MAP+1U)

#define LIDAROGAPPLIB_GRAPH_COMPLETE_EVENT  (0U)

typedef struct
{
    /* Graph parameter 0. */
    vx_user_data_object     vxLidarPointCloud;

    /* Graph parameter 1. */
    vx_user_data_object     vxLidarMeta;

    /* Graph parameter 2 */
    vx_user_data_object     vxPoseAndRef;

    /* Graph parameter 3 */
    vx_user_data_object     vxPfsdOutDesc;

    /* Graph parameter 4 */
    vx_user_data_object     vxOutInstMap;

} LIDAROGAPPLIB_graphParams;

using LIDAROGAPPLIB_ogNodeParamQ = std::queue<LIDAROGAPPLIB_graphParams*>;

typedef struct LIDAROGAPPLIB_Context
{
    LIDAROGAPPLIB_CreateParams  params;

    vx_context                  vxContext;
    vx_graph                    vxGraph;
    vx_node                     mdcNode;
    vx_node                     gpcNode;
    vx_node                     ogNode;

    vx_user_data_object         vxLidarPointCloud[LIDAROGAPPLIB_PIPELINE_DEPTH];
    vx_user_data_object         vxPoseAndRef[LIDAROGAPPLIB_PIPELINE_DEPTH];
    vx_user_data_object         vxLidarMeta[LIDAROGAPPLIB_PIPELINE_DEPTH];
    vx_user_data_object         vxPfsdOutDesc[LIDAROGAPPLIB_PIPELINE_DEPTH];
    vx_user_data_object         vxOutInstMap[LIDAROGAPPLIB_PIPELINE_DEPTH];
    vx_user_data_object         vxM_ego_lidar;
    vx_user_data_object         vxInsData;
    vx_user_data_object         vxPrevInsData;
    vx_user_data_object         vxRootECEF;
    vx_user_data_object         vxGpcConfig;
    vx_user_data_object         vxLidarNormalCloud;
    vx_user_data_object         vxLidarMdcPointCloud;
    vx_user_data_object         vxLidarGpcPointCloud;
    vx_user_data_object         vxOgConfig;
    vx_user_data_object         vxOutAccMap;
    vx_user_data_object         vxPositionTime;

    /** Lidar point cloud size. */
    uint32_t                    pcSize;

    /** Lidar Metadata size. */
    uint32_t                    lidarMetaSize;

    /** Output Accumulated map size. */
    uint32_t                    outAccMapSize;

    /** Output Instantaneous map size. */
    uint32_t                    outInstMapSize;

    /** GPC configuration. */
    PTK_Lidar_GpcConfig         gpcConfig;

    /** MDC Node Core mapping. */
    const char                * mdcNodeCore;

    /** GPC Node Core mapping. */
    const char                * gpcNodeCore;

    /** OG Node Core mapping. */
    const char                * ogNodeCore;

    /** Resource lock. */
    std::mutex                  paramRsrcMutex;

    /** OG node specific graph parameter tracking. */
    LIDAROGAPPLIB_graphParams   paramDesc[LIDAROGAPPLIB_PIPELINE_DEPTH];

    /** A queue for holding free descriptors. */
    LIDAROGAPPLIB_ogNodeParamQ  freeQ;

    /** Queue for output processing. */
    LIDAROGAPPLIB_ogNodeParamQ  outputQ;

    /** Pipeline depth. */
    uint32_t                    pipelineDepth;

    /** Output Accumulated map cache. */
    PTK_Map                    *outAccMap;

    /** Performance tracking context. */
    app_perf_point_t            perf;

    /** Flag to track if the performance counter has been initialized. */
    bool                        startPerfCapt;

} LIDAROGAPPLIB_Context;

vx_status LIDAROGAPPLIB_getFreeParamRsrc(
        LIDAROGAPPLIB_Context      *appCntxt,
        LIDAROGAPPLIB_graphParams  *gpDesc);

vx_status LIDAROGAPPLIB_setupPipeline(
        LIDAROGAPPLIB_Context  *appCntxt);

vx_status LIDAROGAPPLIB_releaseParamRsrc(
        LIDAROGAPPLIB_Context  *appCntxt,
        uint32_t                rsrcIndex);

#endif /* _LIDAR_OGMAP_APPLIB_PRIV_H_ */

