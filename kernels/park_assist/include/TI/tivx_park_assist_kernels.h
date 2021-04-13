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

#ifndef TIVX_PARK_ASSIST_KERNELS_H_
#define TIVX_PARK_ASSIST_KERNELS_H_

#include <VX/vx.h>
#include <VX/vx_kernels.h>

/* algorithms */
#include <perception/common/common_types.h>
#include <perception/algos.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_TIME_TO_SECONDS_CONVERSION_CONSTANT (1000000.0f)

/**
 * \defgroup group_vision_apps_kernels_park_assist TIVX Kernels for Park Assist
 *
 * \brief This section documents the kernels defined for Park Assist applications.
 *
 * \ingroup group_vision_apps_kernels
 */
/*!
 * \file
 * \brief The list of supported kernels in this kernel extension.
 */

/*! \brief OpenVX module name
 * \ingroup group_vision_apps_kernels_park_assist
 */
#define TIVX_MODULE_NAME_PARK_ASSIST    "park_assist"


/*! \brief Kernel Name: Triangulation
 *  \see group_vision_apps_kernels_park_assist
 */
#define TIVX_KERNEL_TRIANGULATION_NAME     "com.ti.park_assist.triangulation"

/*! \brief Kernel Name: DOF flow vectors to tracks
 *  \see group_vision_apps_kernels_park_assist
 */
#define TIVX_KERNEL_DOF_TO_TRACKS_NAME     "com.ti.park_assist.dof_to_tracks"

/*! \brief Kernel Name: SFM based OGMAP
 *  \see group_vision_apps_kernels_park_assist
 */
#define TIVX_KERNEL_SFM_OGMAP_NAME     "com.ti.park_assist.sfm_ogmap"

/*! \brief Kernel Name: Lidar based OGMAP
 *  \see group_vision_apps_kernels_park_assist
 */
#define TIVX_KERNEL_LIDAR_OGMAP_NAME     "com.ti.park_assist.lidar_ogmap"

/*! \brief Kernel Name: Radar based OGMAP
 *  \see group_vision_apps_kernels_park_assist
 */
#define TIVX_KERNEL_RADAR_OGMAP_NAME     "com.ti.park_assist.radar_ogmap"

/*! \brief Kernel Name: Fused OGMAP
 *  \see group_vision_apps_kernels_park_assist
 */
#define TIVX_KERNEL_FUSED_OGMAP_NAME     "com.ti.park_assist.fused_ogmap"


/*! \brief Kernel Name: TIDL Parking Spot Mapping
 *  \see group_vision_apps_kernels_park_assist
 */
#define TIVX_KERNEL_PS_MAPPING_NAME     "com.ti.park_assist.ps_mapping"

/*! \brief radar_gtrack kernel name
 *  \see group_vision_function_park_assist
 */
#define TIVX_KERNEL_RADAR_GTRACK_NAME     "com.ti.park_assist.radar_gtrack"

/* Control commands */
/*! \brief Control command for resetting the radar ogmap library.
 *  \see group_vision_apps_kernels_park_assist
 */
#define TIVX_KERNEL_RADAR_OGMAP_RESET       (0U)

/*! \brief Control command for resetting the lidar ogmap library.
 *  \see group_vision_apps_kernels_park_assist
 */
#define TIVX_KERNEL_LIDAR_OGMAP_RESET       (0U)

/*! \brief Control command for resetting the SFM ogmap library.
 *  \see group_vision_apps_kernels_park_assist
 */
#define TIVX_KERNEL_SFM_OGMAP_RESET         (0U)

/*********************************
 *      TRIANGULATE STRUCTURES
 *********************************/

/*! \brief Max number of view to track
 * \ingroup group_vision_apps_kernels_park_assist
 */
#define TIVX_KERNEL_TRIANGULATION_MAX_VIEWS (2u)

/*!
 * \brief tivxTriangulationNode parameters
 * \ingroup group_vision_apps_kernels_park_assist
 */
typedef struct {
    uint32_t maxIters; /**< maximum number of iterations for weighted iterative triangulation algorithm (default: 5) */
    uint32_t enableHighPrecision; /**< 0: normal precision, 1: high precision (default: 0) */
} tivx_triangulation_params_t;

/*!
 * \brief Triangulation track
 *
 * Structure describing one track of 2d coordinates to be triangulated by
 * triangulation kernel.
 *
 * The 2d coordinates must be in camera-normalized coordinates
 * (pixel coordinates pre-multiplied by inverse of intrinsic camera matrix K).
 *
 * \ingroup group_vision_apps_kernels_park_assist
 */
typedef struct {

    uint32_t     length;   /**< length (number views) for this this track.
                            * If <2 or >TIVX_KERNEL_TRIANGULATION_MAX_VIEWS, result will be invalid.
                            * xy field must be filled starting at view 0.
                            * NOTE: User is responsible to ensure that all required tracks and camera poses are available
                            *
                            * view0 is the latest view
                            */
    uint32_t     reserved; /**< padding for 8-byte alignment */
    struct pixel
    {
        vx_float32 x;
        vx_float32 y;
    } view[TIVX_KERNEL_TRIANGULATION_MAX_VIEWS];
} tivx_triangulation_track_t;

/*!
 * \brief Triangulation camera pose
 *
 * Structure describing the camera poses for the camera views used in triangulation kernel.
 *
 * The pose for view i will be interpreted as the transform from a fixed reference
 * coordinate frame "ref" chosen by the user to the camera reference frame "c_i".
 * For cameras, the standard coordinate system x=image right, y=image down, z=forward (into the scene) is used.
 *
 * \ingroup group_vision_apps_kernels_park_assist
 */
typedef struct
{
    PTK_RigidTransform view[TIVX_KERNEL_TRIANGULATION_MAX_VIEWS]; /**< pose of the different views */
} tivx_triangulation_pose_t;

/*********************************
 *   DOF TO TRACKS STRUCTURES
 *********************************/

/*!
 * \brief Defines the region of interest (ROI) types supported by tivxDofToTracksNode.
 *        ROIs are defined in the input image (DOF field) pixel domain.
 *        The parameters roiPrm1,roiPrm2,etc. below are defined by user in tivx_dof_to_tracks_params_t.
 *        diffX is defined as (x - prinicipalPointX) where x is the horizontal coordinate of the pixel
 *        diffY is defined as (y - prinicipalPointX) where y is the vertical coordinate of the pixel
 * \ingroup group_vision_apps_kernels_park_assist
 */
typedef enum {
    TIVX_DOF_TO_TRACKS_ROI_VERTICAL_CONE = 0,  /**< in ROI if (diffY < roiPrm1 * fabsf(diffX) + roiPrm2) */
    TIVX_DOF_TO_TRACKS_ROI_ELLIPSE,            /**< in ROI if (roiPrm1 * diffX * diffX + diffY * diffY) < roiPrm2) */
    TIVX_DOF_TO_TRACKS_ROI_MAX                 /**< number of ROI types supported */
} tivx_dof_to_tracks_roi_type_e;

/*!
 * \brief DOF flow vectors to track parameters
 * \ingroup group_vision_apps_kernels_park_assist
 */
typedef struct {
    vx_uint32                     dofConfidThresh; /**< max DOF confidence to be considered, valid range 0 (low confidence) to 15 (high confidence) */
    tivx_dof_to_tracks_roi_type_e roiType;         /**< the ROI type used */
    vx_float32                    roiPrm1;         /**< ROI param 1 (see tivx_dof_to_tracks_roi_type) */
    vx_float32                    roiPrm2;         /**< ROI param 2 (see tivx_dof_to_tracks_roi_type) */
    uint32_t                      subsampleFactor; /**< sub sample factor for picking flow vectors from DOF */
    vx_float32                    focalLength;     /**< focal length of camera lens [pixels] */
    vx_float32                    principalPointX; /**< x-coordinate of camera's principal point [pixels] */
    vx_float32                    principalPointY; /**< y-coordinate of camera's principal point [pixels] */
    vx_float32                    d2uTableStep;    /**< Distorted to Undistorted Table step (needs to correspond to node's lut input*/
} tivx_dof_to_tracks_params_t;

typedef struct
{
    PTK_RigidTransform M_e_g;
    PTK_RigidTransform M_w_e;
} tivx_ps_mapping_pose_t;

typedef struct {

    vx_int16   imageWidth;
    vx_int16   imageHeight;
    vx_float32 focalLengthX;
    vx_float32 focalLengthY;
    vx_float32 principalPointX;
    vx_float32 principalPointY;
    vx_float32 d2uTableStep;

} tivx_ps_mapping_cam_params_t;

typedef struct
{
    /** Flag to indicate if FSD is enabled. */
    uint32_t                    fsdEnable;

    /** OGMAP Configuration. */
    PTK_Alg_LidarOgmapParams    ogConfig;

    /** FSD-PFSD configuraion Parameters. */
    PTK_Alg_FsdPfsdParams       fsdPfsdConfig;

} tivx_lidar_ogmap_pfsd_params_t;

typedef struct
{
    /** Flag to indicate if FSD is enabled. */
    uint32_t                    fsdEnable;

    /** OGMAP Configuration. */
    PTK_Alg_RadarOgmapParams    ogConfig;

    /** FSD-PFSD configuraion Parameters. */
    PTK_Alg_FsdPfsdParams       fsdPfsdConfig;

} tivx_radar_ogmap_pfsd_params_t;

typedef struct
{
    /** Flag to indicate if FSD is enabled. */
    uint32_t                    fsdEnable;

    /** OGMAP Configuration. */
    PTK_Alg_FusedOgmapParams    ogConfig;

    /** FSD-PFSD configuraion Parameters. */
    PTK_Alg_FsdPfsdParams       fsdPfsdConfig;

} tivx_fused_ogmap_pfsd_params_t;

typedef struct
{
    /** Flag to indicate if FSD is enabled. */
    uint32_t                    fsdEnable;

    /** OGMAP Configuration. */
    PTK_Alg_SfmOgmapParams      ogConfig;

    /** FSD-PFSD configuraion Parameters. */
    PTK_Alg_FsdPfsdParams       fsdPfsdConfig;

} tivx_sfm_ogmap_pfsd_params_t;

typedef struct {

    /*configuration of PTK_Map */
    vx_uint32    xCells;       /*! [STATIC]Number of cells along x axis */
    vx_uint32    yCells;       /*! [STATIC]Number of cells along y axis */
    vx_float32   xCellSize;    /*! [STATIC]Physical length of cell along x */
    vx_float32   yCellSize;    /*! [STATIC]Physical length of cell along y */
    vx_float32   xMin;         /*! [STATIC]Physical x coordinate matching cell 0 */
    vx_float32   yMin;         /*! [STATIC]Physical y coordinate matching cell 0 */
    vx_uint32    occupancyGridId;      /*! [STATIC] id to be used for "occupancy (bit flag)" grid in map structure */

    vx_uint32    ogFlagFree;           /*! [STATIC] bit flag used in "occupancy" grid to indicate free */
    vx_uint32    ogFlagOccupied;       /*! [STATIC] bit flag used in "occupancy" grid to indicate occupied */

} tivx_ps_mapping_ogmap_params_t;

/*!
 * \brief
 * \ingroup group_vision_apps_kernels_park_assist
 */
typedef struct {
    vx_uint32   numParkingSpots;

    vx_uint16  *type;
    vx_float32 *prob;

    vx_float32 *imagePoints;
    uint32_t    imagePoints_size;
} tivx_ps_mapping_input_t ;

typedef struct {
    vx_uint32  maxSpotsFrame;
    vx_uint32  maxTotalSpots;
    vx_uint32  numSpots;

    vx_float32 minProb;     // min probability for valid detection
    vx_float32 minHeight;   // min height of a parking spot
    vx_float32 maxHeight;   // max height of a parking spot
    vx_float32 minWidth;    // min width of parking spot
    vx_float32 maxWidth;    // max width of parking spot

    vx_float32 matchTh;     // matching threshold between two spots

    vx_int8    minCountPerPS;   // min # of counts of a parking spot

    vx_float32 minImPosX;
    vx_float32 maxImPosX;
    vx_float32 minImPosY;
    vx_float32 maxImPosY;
} tivx_ps_mapping_alg_parmas_t;

typedef struct {
    tivx_ps_mapping_cam_params_t   ps_mapping_cam_params;
    tivx_ps_mapping_ogmap_params_t ps_mapping_ogmap_params;
    tivx_ps_mapping_alg_parmas_t   ps_mapping_alg_params;

} tivx_ps_mapping_config_t;


/*!
 * \brief Used for the application to determine the size requirement of the PTK_Map output
 * \ingroup group_vision_apps_kernels_park_assist
 * \retval vx_size, required memory size for map output
 */
vx_size tivxPSMappingNode_getMapSize(tivx_ps_mapping_ogmap_params_t *prms);

/*!
 * \brief Given tivx_ps_mapping_ogmap_params_t, write PTK_MapConfig into config,
 * which is needed to create a PTK_Map object.
 * \retval vx_status
 * \ingroup group_vision_apps_kernels_park_assist
 */
vx_status tivxPSMappingNode_getMapConfig(tivx_ps_mapping_ogmap_params_t *prms, PTK_MapConfig *config);


/*********************************
 *      Functions
 *********************************/

/*!
 * \brief Used for the Application to load the park_assist kernels into the context.
 * \ingroup group_vision_apps_kernels_park_assist
 */
void tivxParkAssistLoadKernels(vx_context context);

/*!
 * \brief Used for the Application to unload the park_assist kernels from the context.
 * \ingroup group_vision_apps_kernels_park_assist
 */
void tivxParkAssistUnLoadKernels(vx_context context);

#if 1
/*!
 * \brief Function to register PARK_ASSIST Kernels on the c6x Target
 * \ingroup group_tivx_ext
 */
void tivxRegisterParkAssistTargetC6XKernels(void);

/*!
 * \brief Function to un-register PARK_ASSIST Kernels on the c6x Target
 * \ingroup group_tivx_ext
 */
void tivxUnRegisterParkAssistTargetC6XKernels(void);


/*!
 * \brief Function to register PARK_ASSIST Kernels on the Arm Target
 * \ingroup group_tivx_ext
 */
void tivxRegisterParkAssistTargetArmKernels(void);

/*!
 * \brief Function to un-register PARK_ASSIST Kernels on the Arm Target
 * \ingroup group_tivx_ext
 */
void tivxUnRegisterParkAssistTargetArmKernels(void);

/*!
 * \brief Function to register PARK_ASSIST Kernels on the target
 * \ingroup group_tivx_ext
 */
void tivxRegisterParkAssistTargetKernels(void);

/*!
 * \brief Function to un-register PARK_ASSIST Kernels on the target
 * \ingroup group_tivx_ext
 */
void tivxUnRegisterParkAssistTargetKernels(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* TIVX_PARK_ASSIST_KERNELS_H_ */


