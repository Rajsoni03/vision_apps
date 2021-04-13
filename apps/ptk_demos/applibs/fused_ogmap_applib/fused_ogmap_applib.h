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

#ifndef _FUSED_OGAPP_APPLIB_H_
#define _FUSED_OGAPP_APPLIB_H_

#include <TI/tivx.h>
#include <TI/tivx_debug.h>
#include <TI/tivx_mutex.h>
#include <TI/j7.h>
#include <TI/tivx_park_assist.h>

#include <perception/dbtools/c/dbtools.h>

/**
 * \defgroup group_applib_fused_ogmap Fusion OGMAP APPLIB code.
 * \ingroup group_ptk_applib
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Constant for default core mapping for the nodes.
 * \ingroup group_applib_fused_ogmap
 */
#define FUSEDOGAPPLIB_DEFAULT_CORE_MAPPING  (TIVX_TARGET_A72_0)

typedef struct FUSEDOGAPPLIB_Context * FUSEDOGAPPLIB_Handle;

/**
 * \brief Mask definitions for identifying the source of the input to
 *        the fusion processing.
 *
 * \ingroup group_applib_fused_ogmap
 */
typedef enum
{
    /** Radar. */
    FUSEDOGAPPLIB_INPUT_MASK_RADAR   = 1 << dbconfig_sensor_type_RADAR,

    /** Lidar. */
    FUSEDOGAPPLIB_INPUT_MASK_LIDAR   = 1 << dbconfig_sensor_type_LIDAR,

    /** Camera SFM. */
    FUSEDOGAPPLIB_INPUT_MASK_SFM     = 1 << dbconfig_sensor_type_CAMERA,

    /** Fusion. */
    FUSEDOGAPPLIB_INPUT_MASK_FUSION  = 0x10,

    /** Parking Spot Detection. */
    FUSEDOGAPPLIB_INPUT_MASK_PSD     = 0x20,

    /** PSD output. */
    FUSEDOGAPPLIB_INPUT_MASK_PSD_IMG = 0x40,

    /** DOF output. */
    FUSEDOGAPPLIB_INPUT_MASK_DOF_IMG = 0x80

} FUSEDOGAPPLIB_input_mask_e;

/**
 * \brief Fused OGMAP APPLIB create parameter context.
 *
 * \ingroup group_applib_fused_ogmap
 */
typedef struct
{
    /** OpenVX references */
    vx_context                      vxContext;

    /* OGMAP and PFSD configuration parameters. */
    tivx_fused_ogmap_pfsd_params_t  ogPfsdCfg;

    /** TIDL PSD enable flag */
    uint32_t                        tidlPsdEnable;

    /** TIDL PSD grid id */
    uint32_t                        tidlPsdGridId;

    /** Fusion grid Id. */
    uint32_t                        fusionGridId;

    /** Sensor enable mask */
    uint32_t                        sensorEnableMask;

    /** Fusion Node Core mapping. */
    const char                    * fusionNodeCore;

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

} FUSEDOGAPPLIB_createParams;

/**
 * \brief Context carrying the inputs to the fusion processing block.
 *
 * \ingroup group_applib_fused_ogmap
 */
typedef struct
{
	/** Mask indicating which sensor data is valid. */
	uint32_t                sensorDataValidMask;

    /** SfM map (input). */
    vx_user_data_object     cameraMap;

    /** Radar map (input).*/
    vx_user_data_object     radarMap;

    /** Lidar map (input). */
    vx_user_data_object     lidarMap;

    /** TIDL PSD map (input) */
    PTK_Map                *tidlPsMap;

    /** Timestamp information. */
    PTK_Alg_FusedOgmapTime  tsData;

} FUSEDOGAPPLIB_processReq;

/**
 * \brief Function to initialize the APPLIB.
 *
 * \param [in] createParams APPLIB create parameters.
 *
 * \return Valid handle on success. NULL otherwise.
 *
 * \ingroup group_applib_fused_ogmap
 */
FUSEDOGAPPLIB_Handle FUSEDOGAPPLIB_create(
        FUSEDOGAPPLIB_createParams *createParams);

/**
 * \brief Function to process a given sensor frame.
 *
 * \param [in,out] handle APPLIB handle.
 *
 * \param [in] req Sensor data input.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_fused_ogmap
 */
vx_status FUSEDOGAPPLIB_process(
        FUSEDOGAPPLIB_Handle        handle,
        FUSEDOGAPPLIB_processReq   *req);

/**
 * \brief Function to de-init the APPLIB and release the memory associated with
 *        the handle.
 *
 * \param [in,out] handle Reference to APPLIB handle.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_fused_ogmap
 */
vx_status FUSEDOGAPPLIB_delete(
        FUSEDOGAPPLIB_Handle *handle);

/**
 * \brief Function to set the reference frame for mapping.
 *
 * \param [in,out] handle APPLIB handle.
 *
 * \param [in] ecef_w Transform to map to world frame.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_fused_ogmap
 */
vx_status FUSEDOGAPPLIB_setWorldReference(
        FUSEDOGAPPLIB_Handle        handle,
        const PTK_RigidTransform_d *ecef_w);

/**
 * \brief Function to reset the performance and other APPLIB specific counters.
 *
 * \param [in] handle APPLIB handle.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_fused_ogmap
 */
vx_status FUSEDOGAPPLIB_reset(
        FUSEDOGAPPLIB_Handle    handle);

/**
 * \brief Function to return a pointer to the output OGMAP.
 *
 * \param [in] handle APPLIB handle.
 *
 * \return A valid map on success. NULL, otherwise.
 *
 * \ingroup group_applib_fused_ogmap
 */
PTK_Map* FUSEDOGAPPLIB_getOutMap(
        FUSEDOGAPPLIB_Handle    handle);

/**
 * \brief Function to return the size of the output OGMAP in bytes.
 *
 * \param [in] handle APPLIB handle.
 *
 * \return Size of the map in bytes, if successful. 0, otherwise.
 *
 * \ingroup group_applib_fused_ogmap
 */
uint32_t FUSEDOGAPPLIB_getOutMapSize(
        FUSEDOGAPPLIB_Handle    handle);

/**
 * \brief Function to print the performance statistics to stdout.
 *
 * \param [in] handle APPLIB handle.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_fused_ogmap
 */
vx_status FUSEDOGAPPLIB_printStats(
        FUSEDOGAPPLIB_Handle    handle);

/**
 * \brief Function to export the performance statistics to a file.
 *
 * \param [in] handle APPLIB handle.
 *
 * \param [in,out] fp File handle.
 *
 * \param [in] exportAll Flag to indicate if the memory usage statistics need
 *                       to be exported as well. If this flag is false, then
 *                       only the graph level performance stats will be
 *                       exported.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_applib_fused_ogmap
 */
vx_status FUSEDOGAPPLIB_exportStats(
        FUSEDOGAPPLIB_Handle    handle,
        FILE                   *fp,
        bool                    exportAll);

#ifdef __cplusplus
}
#endif

#endif /* _FUSED_OGAPP_APPLIB_H_ */

