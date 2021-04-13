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

#include <TI/tivx.h>
#include <TI/tivx_park_assist.h>

VX_API_ENTRY vx_node VX_API_CALL tivxTriangulationNode(vx_graph graph,
                                      vx_user_data_object  configuration,
                                      vx_array             input_track,
                                      vx_user_data_object  input_pose,
                                      vx_array             output_point)
{
    vx_reference prms[] = {
            (vx_reference)configuration,
            (vx_reference)input_track,
            (vx_reference)input_pose,
            (vx_reference)output_point
    };
    vx_node node = tivxCreateNodeByKernelName(graph,
                                           TIVX_KERNEL_TRIANGULATION_NAME,
                                           prms,
                                           dimof(prms));
    return node;
}

VX_API_ENTRY vx_node VX_API_CALL tivxDofToTracksNode(vx_graph graph,
                                      vx_user_data_object  configuration,
                                      vx_image             input_dof_field,
                                      vx_lut               input_d2u_lut,
                                      vx_array             output_tracks)
{
    vx_reference prms[] = {
            (vx_reference)configuration,
            (vx_reference)input_dof_field,
            (vx_reference)input_d2u_lut,
            (vx_reference)output_tracks
    };
    vx_node node = tivxCreateNodeByKernelName(graph,
                                           TIVX_KERNEL_DOF_TO_TRACKS_NAME,
                                           prms,
                                           dimof(prms));
    return node;
}

VX_API_ENTRY vx_node VX_API_CALL tivxSfmOgmapNode(vx_graph graph,
                                      vx_user_data_object  configuration,
                                      vx_array             input_point,
                                      vx_user_data_object  input_point_transform,
                                      vx_user_data_object  input_pose,
                                      vx_user_data_object  reftr_and_pose,
                                      vx_user_data_object  pfsd_out_desc,
                                      vx_user_data_object  acc_output_map,
                                      vx_user_data_object  inst_output_map)
{
    vx_reference prms[] = {
            (vx_reference)configuration,
            (vx_reference)input_point,
            (vx_reference)input_point_transform,
            (vx_reference)input_pose,
            (vx_reference)reftr_and_pose,
            (vx_reference)pfsd_out_desc,
            (vx_reference)acc_output_map,
            (vx_reference)inst_output_map
    };
    vx_node node = tivxCreateNodeByKernelName(graph,
                                           TIVX_KERNEL_SFM_OGMAP_NAME,
                                           prms,
                                           dimof(prms));
    return node;
}

VX_API_ENTRY vx_node VX_API_CALL tivxLidarOgmapNode(vx_graph graph,
                                      vx_user_data_object  configuration,
                                      vx_user_data_object  point_cloud,
                                      vx_user_data_object  reftr_and_pose,
                                      vx_user_data_object  pfsd_out_desc,
                                      vx_user_data_object  acc_output_map,
                                      vx_user_data_object  inst_output_map)
{
    vx_reference prms[] = {
            (vx_reference)configuration,
            (vx_reference)point_cloud,
            (vx_reference)reftr_and_pose,
            (vx_reference)pfsd_out_desc,
            (vx_reference)acc_output_map,
            (vx_reference)inst_output_map
    };
    vx_node node = tivxCreateNodeByKernelName(graph,
                                           TIVX_KERNEL_LIDAR_OGMAP_NAME,
                                           prms,
                                           dimof(prms));
    return node;
}

VX_API_ENTRY vx_node VX_API_CALL tivxRadarOgmapNode(vx_graph graph,
                                      vx_user_data_object  configuration,
                                      vx_user_data_object  sensor_config,
                                      vx_user_data_object  radar_obj_data,
                                      vx_user_data_object  reftr_and_pose,
                                      vx_user_data_object  pfsd_out_desc,
                                      vx_user_data_object  acc_output_map,
                                      vx_user_data_object  inst_output_map)
{
    vx_reference prms[] = {
            (vx_reference)configuration,
            (vx_reference)sensor_config,
            (vx_reference)radar_obj_data,
            (vx_reference)reftr_and_pose,
            (vx_reference)pfsd_out_desc,
            (vx_reference)acc_output_map,
            (vx_reference)inst_output_map
    };
    vx_node node = tivxCreateNodeByKernelName(graph,
                                           TIVX_KERNEL_RADAR_OGMAP_NAME,
                                           prms,
                                           dimof(prms));
    return node;
}

VX_API_ENTRY vx_node VX_API_CALL tivxFusedOgmapNode(vx_graph graph,
                                      vx_user_data_object  configuration,
                                      vx_user_data_object  ogmap_camera,
                                      vx_user_data_object  ogmap_radar,
                                      vx_user_data_object  ogmap_lidar,
                                      vx_user_data_object  reftr_and_pose,
                                      vx_user_data_object  pfsd_out_desc,
                                      vx_user_data_object  output_map)
{
    vx_reference prms[] = {
            (vx_reference)configuration,
            (vx_reference)ogmap_camera,
            (vx_reference)ogmap_radar,
            (vx_reference)ogmap_lidar,
            (vx_reference)reftr_and_pose,
            (vx_reference)pfsd_out_desc,
            (vx_reference)output_map
    };
    vx_node node = tivxCreateNodeByKernelName(graph,
                                           TIVX_KERNEL_FUSED_OGMAP_NAME,
                                           prms,
                                           dimof(prms));
    return node;
}

VX_API_ENTRY vx_node VX_API_CALL tivxPsMappingNode(vx_graph graph,
                                      vx_user_data_object  configuration,
                                      vx_user_data_object  dataptr,
                                      vx_lut               d2u_lut,
                                      vx_lut               projmat,
                                      vx_user_data_object  input_pose,
                                      vx_user_data_object  input_map,
                                      vx_user_data_object  output_map)
{
    vx_reference prms[] = {
            (vx_reference)configuration,
            (vx_reference)dataptr,
            (vx_reference)d2u_lut,
            (vx_reference)projmat,
            (vx_reference)input_pose,
            (vx_reference)input_map,
            (vx_reference)output_map
    };
    vx_node node = tivxCreateNodeByKernelName(graph,
                                           TIVX_KERNEL_PS_MAPPING_NAME,
                                           prms,
                                           dimof(prms));
    return node;
}

VX_API_ENTRY vx_node VX_API_CALL tivxRadarGtrackNode(vx_graph graph,
                                      vx_user_data_object  configuration,
                                      vx_user_data_object  sensor_config,
                                      vx_user_data_object  radar_obj_data,
                                      vx_user_data_object  reftr_and_pose,
                                      vx_user_data_object  track_info)
{
    vx_reference prms[] = {
            (vx_reference)configuration,
            (vx_reference)sensor_config,
            (vx_reference)radar_obj_data,
            (vx_reference)reftr_and_pose,
            (vx_reference)track_info
    };
    vx_node node = tivxCreateNodeByKernelName(graph,
                                           TIVX_KERNEL_RADAR_GTRACK_NAME,
                                           prms,
                                           dimof(prms));
    return node;
}

