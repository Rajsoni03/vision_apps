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

#ifndef _PS_MAPPING_APPLIB_H_
#define _PS_MAPPING_APPLIB_H_

#include <TI/tivx.h>
#include <TI/tivx_debug.h>
#include <TI/tivx_mutex.h>
#include <TI/j7.h>
#include <TI/tivx_park_assist.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PSLIB_DEFAULT_CORE_MAPPING  (TIVX_TARGET_A72_0)

/* Forward declaration. */
struct PSLIB_Context;

typedef struct PSLIB_Context * PSLIB_Handle;

//#define SFMOGAPPLIB_MAX_LINE_LEN   (256U)

typedef struct
{
    /** OpenVX references */
    vx_context                      vxContext;

    /** Configuration parameters. */
    tivx_ps_mapping_alg_parmas_t    psMappingAlgoParams;

    /** Grid parameters for the map. */
    PTK_GridConfig                  gridConfig;

    /** id to be used for "occupancy (bit flag)" grid in map structure */
    uint32_t                        occupancyGridId;

    /** bit flag used in "occupancy" grid to indicate free */
    uint32_t                        ogFlagFree;

    /** bit flag used in "occupancy" grid to indicate occupied */
    uint32_t                        ogFlagOccupied;

    /** camera projection matrix */
    double                        * projMatrix;

    /** camera projection matrix */
    uint32_t                        projMatrixSize;

    /** camera distortion correction table */
    float                         * d2uLUT;

    /** camera distortion correction table size */
    uint32_t                        d2uLUTSize;

    /* Estimate for static ground to ego transform */
    PTK_RigidTransform              M_e_g;

    /* Ego to camera transform (extrinsic calibration) */
    PTK_RigidTransform              M_c_e;

    /* Camera to ego transform */
    PTK_RigidTransform              M_e_c;

    /* Camera to ground transform */
    PTK_RigidTransform              M_g_c;

    /** PS Mapping Node Core mapping. */
    const char                    * psNodeCore;

} PSLIB_createParams;

PSLIB_Handle PSLIB_create(PSLIB_createParams *createParams);

void         PSLIB_setWorldReference(PSLIB_Handle handle,
                             const PTK_RigidTransform_d *ecef_w);

void         PSLIB_process(PSLIB_Handle handle, uint8_t* data,
                           uint32_t sensorId,  uint64_t timestamp);

void         PSLIB_delete(PSLIB_Handle *handle);

PTK_Map     *PSLIB_mapOutput(PSLIB_Handle handle);
void         PSLIB_unmapOutput(PSLIB_Handle handle);

uint32_t     PSLIB_getOutMapSize(PSLIB_Handle handle);

vx_graph     PSLIB_getGraphHandle(PSLIB_Handle handle);

#ifdef __cplusplus
}
#endif

#endif /* _PS_MAPPING_APPLIB_H_ */

