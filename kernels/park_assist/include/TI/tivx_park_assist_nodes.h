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

#ifndef TIVX_PARK_ASSIST_NODES_H_
#define TIVX_PARK_ASSIST_NODES_H_

#include <VX/vx.h>

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief Creates a Triangulation Node.
 *
 * Node generates 3d points from 2d tracks and known camera poses for the view points (views).
 * A 2d track are the pixel locations of a 3d point in multiple different views.
 * The 2d tracks must be normalized by camera's intrinsic matrix.
 *
 * \param [in] graph The reference to the graph.
 * \param [in] configuration   Array element type tivx_triangulation_params_t, max number of elements = 1
 * \param [in] input_track     Array element type tivx_triangulation_track_t, max number of elements = max number of tracks
 * \param [in] input_pose      Array element type tivx_triangulation_pose_t, max number of elements = 1
 * \param [out] output_point   Array element type PTK_Point, max number of elements = max number of tracks.
 *                             The w-coordinate of PTK_Point is 1.f if the triangulation
 *                             result is valid, and 0.f otherwise.
 *
 * \ingroup group_vision_apps_kernels_park_assist
 *
 * \retval vx_node A node reference. Any possible errors preventing a successful creation should be checked using <tt>vxGetStatus</tt>
 */
VX_API_ENTRY vx_node VX_API_CALL tivxTriangulationNode(vx_graph graph,
                                      vx_user_data_object  configuration,
                                      vx_array             input_track,
                                      vx_user_data_object  input_pose,
                                      vx_array             output_point);

/*! \brief Creates a DOF flow vectors to tracks Node
 *
 * Node creates tracks of length 2 (previous and current frame) from a dense optical flow (DOF) field,
 * by selecting pixels based on sub-sampling, DOF confidence and user-defined ROI. The selected pixels
 * are camera-normalized by correcting for lens distortions and applying the inverse intrinsic camera matrix.
 *
 * \param [in]  graph                The reference to the graph.
 * \param [in]  configuration        Array element type tivx_dof_to_tracks_params_t, max number of elements = 1
 * \param [in]  input_dof_field      vx_image of dataformat VX_DF_IMAGE_U32, the DOF field to be converted to tracks
 * \param [in]  input_d2u_lut (optional) vx_lut of type vx_float32, holds an distorted2undistorted LUT
 *                                   If a NULL pointer is passed, lens disortion correction is not performed.
 * \param [out] output_tracks        Array element type, tivx_triangulation_track_t
 *
 * \ingroup group_vision_apps_kernels_park_assist
 *
 * \retval vx_node A node reference. Any possible errors preventing a successful creation should be checked using <tt>vxGetStatus</tt>
 */
VX_API_ENTRY vx_node VX_API_CALL tivxDofToTracksNode(vx_graph graph,
                                      vx_user_data_object  configuration,
                                      vx_image             input_dof_field,
                                      vx_lut               input_d2u_lut,
                                      vx_array             output_tracks);

/*! \brief Creates a SFM based OGMAP Node.
 *
 * Node updates an occupancy grid map given a 3d point cloud in map coordinates and
 * classifies cells as free or occupied. The algorithm is was designed to work with
 * point clouds from SfM, but point clouds from other sources may be passed as well.
 * The input map is updated to produce the output map.
 *
 * Algorithm steps:
 * 1. For each input point within an ROI near the vehicle, update the corresponding
 *    "average height" cell if points
 * 2. Estimate local ground plane by fitting a plane to the "average height" grid
 *    using RANSAC
 * 3. For each input point within an ROI near the vehicle, compute distance to local
 *     ground plane
 * 4. For each input point within an ROI near the vehicle, increase "on ground" counter
 *    or "above ground" counter cell based on distance threshold to local ground plane.
 *    A threshold that increases linearly with distance from the car is used.
 * 5. If a cell contains a certaub minimum number of observations (points added to
 *    "on ground" or "above ground" counter), the cell is marked as occupied if
 *    "on ground" counter < "above ground" counter, and as free otherwise.
 *
 * \param [in] graph         The reference to the graph.
 * \param [in] configuration  Array element type tivx_parking_space_mapping_params_t, max number of elements = 1
 * \param [in] input_point    Array of length 1 containing a complete PTK_PointCloud object
 * \param [in] input_point_transform
 * \param [in] input_pose     Array element type PTK_RigidTransform of length 1. The ego to world transform (M_w_e) associated with the 3d points.
 * \param [in] reftr_and_pose Current and previous INS records and world to ecef transform
 * \param [out] pfsd_out_desc PFSD output descriptor
 * \param [out] acc_output_map    Array of length 1 containing a complete PTK_Map object
 * \param [out] inst_output_map
 *
 * \ingroup group_vision_apps_kernels_park_assist
 *
 * \retval vx_node A node reference. Any possible errors preventing a successful creation should be checked using <tt>vxGetStatus</tt>
 */
VX_API_ENTRY vx_node VX_API_CALL tivxSfmOgmapNode(vx_graph graph,
                                      vx_user_data_object  configuration,
                                      vx_array             input_point,
                                      vx_user_data_object  input_point_transform,
                                      vx_user_data_object  input_pose,
                                      vx_user_data_object  reftr_and_pose,
                                      vx_user_data_object  pfsd_out_desc,
                                      vx_user_data_object  acc_output_map,
                                      vx_user_data_object  inst_output_map);

/*! \brief Creates a Lidar based OGMAP Node.
 *
 * Node updates an occupancy grid map given a 3d point cloud with each point tagged as
 * "on ground" or "above ground" and provided in map coordinates.
 * Node then tags cells as free or occupied.
 * The algorithm is was designed to work with point clouds from Lidar, but point clouds
 * from other sources may be passed as well.
 * The map is updated accumulatively in place.
 *
 * \param [in] graph            The reference to the graph.
 * \param [in] configuration  Array element type tivx_lidar_og_config_t, max number of elements = 1
 * \param [in] point_cloud    Array of length 1 containing a complete PTK_PointCloud object
 * \param [in] reftr_and_pose Current and previous INS records and world to ecef transform
 * \param [out] pfsd_out_desc PFSD output descriptor
 * \param [out] acc_output_map    Array of length 1 containing a complete PTK_Map object
 * \param [out] inst_output_map
 *
 * \ingroup group_vision_apps_kernels_park_assist
 *
 * \retval vx_node A node reference. Any possible errors preventing a successful creation should be checked using <tt>vxGetStatus</tt>
 */
VX_API_ENTRY vx_node VX_API_CALL tivxLidarOgmapNode(vx_graph graph,
                                      vx_user_data_object  configuration,
                                      vx_user_data_object  point_cloud,
                                      vx_user_data_object  reftr_and_pose,
                                      vx_user_data_object  pfsd_out_desc,
                                      vx_user_data_object  acc_output_map,
                                      vx_user_data_object  inst_output_map);



/*! \brief Creates a Radar based OGMAP Node.
 *
 * Node updates an occupancy grid map based on object-level measurements
 * from multiple radars.
 * "imu_data" and "ecef_w" are needed to determine the vehicle's position and orientation
 * in the map.
 *
 * PTK_INS module needs to be enabled in the system.
 *
 * \param [in] graph            The reference to the graph.
 * \param [in] configuration  Array element type tivx_radar_ogmap_config_t, max number of elements = 1
 * \param [in] sensor_config  Sensor configuration parameters.
 * \param [in] radar_obj_data Radar object data
 * \param [in] reftr_and_pose Current and previous INS records and world to ecef transform
 * \param [out] pfsd_out_desc PFSD output descriptor
 * \param [out] acc_output_map    Output map
 * \param [out] inst_output_map
 *
 * \ingroup group_vision_apps_kernels_park_assist
 *
 * \retval vx_node A node reference. Any possible errors preventing a successful creation should be checked using <tt>vxGetStatus</tt>
 */
VX_API_ENTRY vx_node VX_API_CALL tivxRadarOgmapNode(vx_graph graph,
                                      vx_user_data_object  configuration,
                                      vx_user_data_object  sensor_config,
                                      vx_user_data_object  radar_obj_data,
                                      vx_user_data_object  reftr_and_pose,
                                      vx_user_data_object  pfsd_out_desc,
                                      vx_user_data_object  acc_output_map,
                                      vx_user_data_object  inst_output_map);

/*! \brief Creates a Fused OGMAP Node.
 *
 * Node combines a maximum of three aligned occupancy grids (from camera, radar and/or lidar) to
 * produce a fused occupancy grid based on cell-wise majority voting. The logic table for voting is configurable.
 * "timestamps" input is required to identify vehicle's position in the map to create
 * a region of interest where voting scheme should be applied.
 * The map "fused_map_feedback" is first copied to "fused_map", then "fused_map" is updated as described.
 *
 * PTK_INS module needs to be enabled in the system.
 *
 * \param [in] graph                The reference to the graph.
 * \param [in] configuration  Fusion node configuration
 * \param [in] ogmap_camera   Ogmap grid from camera sensor
 * \param [in] ogmap_radar    Ogmap grid from radar sensor
 * \param [in] ogmap_lidar    Ogmap grid from lidar sensor
 * \param [in] reftr_and_pose Current and previous INS records and world to ecef transform
 * \param [out] pfsd_out_desc PFSD output descriptor
 * \param [out] output_map    Fusion output map
 *
 * \ingroup group_vision_apps_kernels_park_assist
 *
 * \retval vx_node A node reference. Any possible errors preventing a successful creation should be checked using <tt>vxGetStatus</tt>
 */
VX_API_ENTRY vx_node VX_API_CALL tivxFusedOgmapNode(vx_graph graph,
                                      vx_user_data_object  configuration,
                                      vx_user_data_object  ogmap_camera,
                                      vx_user_data_object  ogmap_radar,
                                      vx_user_data_object  ogmap_lidar,
                                      vx_user_data_object  reftr_and_pose,
                                      vx_user_data_object  pfsd_out_desc,
                                      vx_user_data_object  output_map);

/*! \brief [Graph] Creates a PS_MAPPING Node.
 * \param [in] graph The reference to the graph.
 * \param [in] configuration
 * \param [in] dataptr
 * \param [in] d2u_lut
 * \param [in] projmat
 * \param [in] input_pose
 * \param [in] input_map
 * \param [out] output_map
 * \see <tt>TIVX_KERNEL_PS_MAPPING_NAME</tt>
 * \ingroup group_vision_function_ps_mapping
 * \return <tt> vx_node</tt>.
 * \retval vx_node A node reference. Any possible errors preventing a successful creation should be checked using <tt> vxGetStatus</tt>
 */
VX_API_ENTRY vx_node VX_API_CALL tivxPsMappingNode(vx_graph graph,
                                      vx_user_data_object  configuration,
                                      vx_user_data_object  dataptr,
                                      vx_lut               d2u_lut,
                                      vx_lut               projmat,
                                      vx_user_data_object  input_pose,
                                      vx_user_data_object  input_map,
                                      vx_user_data_object  output_map);

/*! \brief [Graph] Creates a RADAR_GTRACK Node.
 * \param [in] graph The reference to the graph.
 * \param [in] configuration
 * \param [in] sensor_config
 * \param [in] radar_obj_data
 * \param [in] reftr_and_pose
 * \param [out] track_info
 * \see <tt>TIVX_KERNEL_RADAR_GTRACK_NAME</tt>
 * \ingroup group_vision_function_radar_gtrack
 * \return <tt> vx_node</tt>.
 * \retval vx_node A node reference. Any possible errors preventing a successful creation should be checked using <tt> vxGetStatus</tt>
 */
VX_API_ENTRY vx_node VX_API_CALL tivxRadarGtrackNode(vx_graph graph,
                                      vx_user_data_object  configuration,
                                      vx_user_data_object  sensor_config,
                                      vx_user_data_object  radar_obj_data,
                                      vx_user_data_object  reftr_and_pose,
                                      vx_user_data_object  track_info);

#ifdef __cplusplus
}
#endif

#endif /* TIVX_PARK_ASSIST_NODES_H_ */


