 /*
 *******************************************************************************
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
 *******************************************************************************
 */

/* IMPORTANT NOTES
 * 1. currently, this applib supports only 1 camera (IMI fisheye camera, right surround view)
 * 2. Most parameters are set internally, @todo move them out, ideally in config files (run time)
 * 3. Depending on SFMOGAPPLIB_mode, the SFMOGAPPLIB_process function expects different type of data
 * 4. applib does not do FSD (sufficient free space detection) at the moment
 */

#ifndef _SURROUND_SFM_OGMAP_APPLIB_H_
#define _SURROUND_SFM_OGMAP_APPLIB_H_

#include <TI/tivx.h>
#include <TI/tivx_debug.h>
#include <TI/tivx_mutex.h>
#include <TI/j7.h>
#include <TI/tivx_park_assist.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SFMOGAPPLIB_PIPELINE_DEPTH        (5U)
#define SFMOGAPPLIB_DEFAULT_CORE_MAPPING  (TIVX_TARGET_A72_0)

/* Forward declaration. */
struct SFMOGAPPLIB_Context;

typedef struct SFMOGAPPLIB_Context * SFMOGAPPLIB_Handle;

#define SFMOGAPPLIB_MAX_LINE_LEN   (256U)

typedef struct
{
    /* OpenVX references */
    vx_context                      vxContext;

    /* whether or not creating point cloud using DOF + triangulation */
    uint8_t                         generatePC;

    /* whether or not using DOF */
    bool                            enableDof;

    /* OGMAP and PFSD configuration parameters. */
    tivx_sfm_ogmap_pfsd_params_t    ogPfsdCfg;

    /* Camera to ego transform (extrinsic calibration) */
    PTK_RigidTransform              M_c_e;

    /* to create point clouds */
    tivx_dmpac_dof_params_t         dofCfg;

    tivx_dof_to_tracks_params_t     dtNodeCfg;

    tivx_triangulation_params_t     trNodeCfg;

    vx_uint32                       dofLevels;

    /* width of input image to Pyramid node */
    uint32_t                        dofWidth;

    /* height of input image to Pyramid node */
    uint32_t                        dofHeight;

    /* maximum number of tracks from DOF */
    uint32_t                        maxNumTracks;

    /** OG Node Core mapping. */
    const char                    * ogNodeCore;

    /** Triangulation Node Core mapping. */
    const char                    * triNodeCore;

    /** DOF to Track Node Core mapping. */
    const char                    * dofTrackNodeCore;

    /** Base value to be used for any programmed VX events. */
    uint32_t                        vxEvtAppValBase;

    /* pipeline depth */
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

} SFMOGAPPLIB_createParams;

typedef struct
{
    /** Timestamp corresponding to the input that generated this output. */
    uint64_t                ts;

    /** Output Accumulated map. */
    PTK_Map                *outAccMap;

    /** PFSD output descriptor. */
    vx_user_data_object     vxPfsdOutDesc;

    /** Output instantaneous map object. */
    vx_user_data_object     vxOutInstMap;

} SFMOGAPPLIB_OutputBuff;

SFMOGAPPLIB_Handle SFMOGAPPLIB_create(SFMOGAPPLIB_createParams *createParams);

void              SFMOGAPPLIB_setWorldReference(SFMOGAPPLIB_Handle           handle,
                                               const PTK_RigidTransform_d *ecef_w);

void              SFMOGAPPLIB_parse_config(char *filePath, SFMOGAPPLIB_createParams * createParams);

int32_t           SFMOGAPPLIB_process(SFMOGAPPLIB_Handle handle,
                                      void              *data,
                                      uint32_t           sensorId,
                                      uint64_t           timestamp);

void              SFMOGAPPLIB_delete(SFMOGAPPLIB_Handle *handle);

PTK_Map*          SFMOGAPPLIB_getOutAccMap(SFMOGAPPLIB_Handle handle);

int32_t           SFMOGAPPLIB_reset(SFMOGAPPLIB_Handle handle);

int32_t           SFMOGAPPLIB_getOutBuff(SFMOGAPPLIB_Handle handle,
                                         SFMOGAPPLIB_OutputBuff *buff);

void              SFMOGAPPLIB_releaseOutBuff(SFMOGAPPLIB_Handle handle);

uint32_t          SFMOGAPPLIB_getOutAccMapSize(SFMOGAPPLIB_Handle handle);

uint32_t          SFMOGAPPLIB_getOutInstMapSize(SFMOGAPPLIB_Handle handle);

vx_graph          SFMOGAPPLIB_getSfmGraphHandle(SFMOGAPPLIB_Handle handle);

void              SFMOGAPPLIB_printStats(SFMOGAPPLIB_Handle handle);

void              SFMOGAPPLIB_exportStats(SFMOGAPPLIB_Handle handle, FILE *fp, bool exportAll);

void              SFMOGAPPLIB_waitGraph(SFMOGAPPLIB_Handle handle);

int32_t           SFMOGAPPLIB_processEvent(SFMOGAPPLIB_Handle     handle,
                                           vx_event_t            *event);

#ifdef __cplusplus
}
#endif

#endif /* _SURROUND_SFM_OGMAP_APPLIB_H_ */

