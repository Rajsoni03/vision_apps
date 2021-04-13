 /*
 *******************************************************************************
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
 *******************************************************************************
 */


#ifndef _SURROUND_SFM_OGMAP_GRAPH_H_
#define _SURROUND_SFM_OGMAP_GRAPH_H_

#include <thread>
#include <stack>
#include <mutex>
#include <queue>

#include <TI/tivx_target_kernel.h>
#include <perception/perception.h>
#include <app_ptk_demo_common.h>

#include "surround_sfm_ogmap_applib.h"


#ifdef __cplusplus
extern "C" {
#endif

#include <TI/tivx_mutex.h>
#include <tivx_utils_graph_perf.h>
#include <tivx_utils_file_rd_wr.h>
#include <utils/perf_stats/include/app_perf_stats.h>

#define SFMOGAPPLIB_GRAPH_PARAM_PYR_IMG     (0U)
#define SFMOGAPPLIB_GRAPH_PARAM_TRIANG_POSE (SFMOGAPPLIB_GRAPH_PARAM_PYR_IMG+1U)
#define SFMOGAPPLIB_GRAPH_PARAM_POINTS_3D   (SFMOGAPPLIB_GRAPH_PARAM_TRIANG_POSE+1U)
#define SFMOGAPPLIB_GRAPH_PARAM_3D_TRANS    (SFMOGAPPLIB_GRAPH_PARAM_POINTS_3D+1U)
#define SFMOGAPPLIB_GRAPH_PARAM_OG_IN_POSE  (SFMOGAPPLIB_GRAPH_PARAM_3D_TRANS+1U)
#define SFMOGAPPLIB_GRAPH_PARAM_POSEREF     (SFMOGAPPLIB_GRAPH_PARAM_OG_IN_POSE+1U)
#define SFMOGAPPLIB_GRAPH_PARAM_PFSD_DESC   (SFMOGAPPLIB_GRAPH_PARAM_POSEREF+1U)
#define SFMOGAPPLIB_GRAPH_PARAM_INST_MAP    (SFMOGAPPLIB_GRAPH_PARAM_PFSD_DESC+1U)
#define SFMOGAPPLIB_NUM_GRAPH_PARAMS        (SFMOGAPPLIB_GRAPH_PARAM_INST_MAP+1U)

#define SFMOGAPPLIB_GRAPH_COMPLETE_EVENT    (0U)


typedef struct
{
    /* Graph parameter 0. */
    /* With DOF    */
    vx_image                pyr_in_image;
    /***************/

    /* Without DOF */
    vx_image                dof2tracks_in_field;
    /***************/

    /* Graph parameter 1. */
    vx_user_data_object     triang_in_pose;

    /* Graph parameter 2. */
    vx_array                vxOgInPoints3d;

    /* Graph parameter 3. */
    vx_user_data_object     points3d_transform;

    /* Graph parameter 4. */
    vx_user_data_object     vxOgInPose;

    /* Graph parameter 5. */
    vx_user_data_object     vxPoseAndRef;

    /* Graph parameter 6. */
    vx_user_data_object     vxPfsdOutDesc;

    /* Graph parameter 7. */
    vx_user_data_object     vxOutInstMap;

} SFMOGAPPLIB_graphParams;

using SFMOGAPPLIB_graphParamQ = std::queue<SFMOGAPPLIB_graphParams*>;

typedef struct SFMOGAPPLIB_Context
{
    /* Parameters*/
    /* maximum number of tracks that can be loaded from file */
    uint32_t                        maxNumTracks;

    /* OVX Node References */
    vx_context                      vxContext;

    vx_graph                        vxSfmGraph;


    /* whether or not creating point cloud using DOF + triangulation */
    uint8_t                         generatePC;

    bool                            enableDof;

    /* Point cloud */
    vx_uint32                       dofLevels;

    /* With DOF    */
    vx_node                         vxPyrNode;
    vx_node                         vxDofNode;
    /***************/
    vx_node                         vxDof2tracksNode;
    vx_node                         vxTriangNode;

    /* With DOF    */
    vx_image                        pyr_in_image[SFMOGAPPLIB_PIPELINE_DEPTH];
    vx_delay                        pyr_out_delay;
    vx_delay                        dof_out_delay;
    vx_user_data_object             dof_in_config;

    vx_image                        dof_in_field;
    vx_image                        dof_out_field;
    /***************/

    /* Without DOF */
    vx_image                        dof2tracks_in_field[SFMOGAPPLIB_PIPELINE_DEPTH];
    /***************/


    vx_user_data_object             dof2tracks_in_config;
    vx_lut                          dof2tracks_in_lut;
    vx_user_data_object             triang_in_config;
    vx_array                        triang_in_tracks;
    vx_user_data_object             triang_in_pose[SFMOGAPPLIB_PIPELINE_DEPTH];
    vx_array                        triang_out_points3d;
    vx_user_data_object             points3d_transform[SFMOGAPPLIB_PIPELINE_DEPTH];

    /* SfM Ogmap node. */
    vx_node                         vxOgNode;
    vx_array                        vxOgInPoints3d[SFMOGAPPLIB_PIPELINE_DEPTH];
    vx_user_data_object             vxOgInConfig;
    vx_user_data_object             vxOgInPose[SFMOGAPPLIB_PIPELINE_DEPTH];
    vx_user_data_object             vxPoseAndRef[SFMOGAPPLIB_PIPELINE_DEPTH];
    vx_user_data_object             vxPfsdOutDesc[SFMOGAPPLIB_PIPELINE_DEPTH];
    vx_user_data_object             vxOutInstMap[SFMOGAPPLIB_PIPELINE_DEPTH];
    vx_user_data_object             vxOutAccMap;
    vx_map_id                       vxOutMapLockId;

    /** Output Accumulated map size. */
    uint32_t                        outAccMapSize;

    /** Output Instantaneous map size. */
    uint32_t                        outInstMapSize;

    /* OGMAP and PFSD configuration parameters. */
    tivx_sfm_ogmap_pfsd_params_t    ogPfsdCfg;

    /* to create point clouds */
    tivx_dmpac_dof_params_t         dofCfg;

    tivx_dof_to_tracks_params_t     dtNodeCfg;

    tivx_triangulation_params_t     trNodeCfg;

    /* width of input image to Pyramid node */
    uint32_t                        dofWidth;

    /* height of input image to Pyramid node */
    uint32_t                        dofHeight;

    /* Reference Frames */
    //!< ego-camera (extrinsic calibration)
    PTK_RigidTransform              M_c_e, M_e_c;

    //!< ECEF-world
    PTK_RigidTransform_d            Md_ecef_w;

    //!< current ego-world (time-dependent)
    PTK_RigidTransform              M_w_e, M_e_w;

    /* Control */
    /* Timestamp from previous process call */
    uint64_t                        timestampPrv;

    /** OG Node Core mapping. */
    const char                    * ogNodeCore;

    /** Triangulation Node Core mapping. */
    const char                    * triNodeCore;

    /** DOF to Track Node Core mapping. */
    const char                    * dofTrackNodeCore;

    /** Resource lock. */
    std::mutex                      paramRsrcMutex;

    /** OG node specific graph parameter tracking. */
    SFMOGAPPLIB_graphParams         paramDesc[SFMOGAPPLIB_PIPELINE_DEPTH];

    /** A queue for holding free descriptors. */
    SFMOGAPPLIB_graphParamQ         freeQ;

    /** Queue for output processing. */
    SFMOGAPPLIB_graphParamQ         outputQ;

    /** Graph processing thread. */
    std::thread                     graphProcThread;

    /* the number of params that are actually used */
    uint32_t                        effectiveNumParams;

    /** Base value to be used for any programmed VX events. */
    uint32_t                        vxEvtAppValBase;

    uint8_t                       * pcMemPtr;

    /** Output Accumulated map cache. */
    PTK_Map                        *outAccMap;

    /** Performance tracking context. */
    app_perf_point_t                perf;

    /** Flag to track if the performance counter has been initialized. */
    bool                            startPerfCapt;

    /** pipeline depth */
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

} SFMOGAPPLIB_Context;

PTK_INS_RetCode PTK_INS_getIMUPosesFromVelAndAtt(uint64_t *timestamps, uint32_t num, PTK_RigidTransform *M_in_i0);

vx_status SFMOGAPPLIB_createPCNodesInGraph(SFMOGAPPLIB_Context  *appCntxt);
vx_status SFMOGAPPLIB_createOGNodesInGraph(SFMOGAPPLIB_Context  *appCntxt);

vx_status SFMOGAPPLIB_createGraph(SFMOGAPPLIB_Context *appCntxt);
void      SFMOGAPPLIB_releaseGraph(SFMOGAPPLIB_Context * appCntxt);

int32_t   SFMOGAPPLIB_set_ego_to_world_pose(SFMOGAPPLIB_Context *appCntxt,
                                            SFMOGAPPLIB_graphParams *gpDesc,
                                            uint64_t           timestamp);

int32_t   SFMOGAPPLIB_setupNodes(SFMOGAPPLIB_Context *appCntxt,
                                 SFMOGAPPLIB_graphParams *gpDesc,
                                 uint8_t *data,
                                 uint64_t timestamp);

void      SFMOGAPPLIB_triangLoadCameraPoses(SFMOGAPPLIB_Context *appCntxt,
                                            SFMOGAPPLIB_graphParams *gpDesc,
                                            uint64_t prvTimestamp,
                                            uint64_t curTimestamp);
void      SFMOGAPPLIB_load3DPoints(SFMOGAPPLIB_Context *appCntxt,
                                   SFMOGAPPLIB_graphParams *gpDesc,
                                   PTK_PointCloud *pcPtr);
void      SFMOGAPPLIB_setPoints3DTransform(SFMOGAPPLIB_Context *appCntxt,
                                           SFMOGAPPLIB_graphParams *gpDesc);
void      SFMOGAPPLIB_loadEgoPose(SFMOGAPPLIB_Context *appCntxt,
                                  SFMOGAPPLIB_graphParams *gpDesc);

int32_t   SFMOGAPPLIB_getFreeParamRsrc(SFMOGAPPLIB_Context       *appCntxt,
                                       SFMOGAPPLIB_graphParams   *gpDesc);

int32_t   SFMOGAPPLIB_releaseParamRsrc(SFMOGAPPLIB_Context  *appCntxt,
                                       uint32_t              rsrcIndex);

#ifdef __cplusplus
}
#endif

#endif /* _SURROUND_SFM_OGMAP_GRAPH_H_ */

