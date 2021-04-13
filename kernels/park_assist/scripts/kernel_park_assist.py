'''
* Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
* ALL RIGHTS RESERVED
'''

from tiovx import *

code = KernelExportCode(Module.PARK_ASSIST, "c7x", "CUSTOM_APPLICATION_PATH")

# Triangulation Node
code.setCoreDirectory("c7x")
kernel = Kernel("triangulation")

kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "configuration", ['tivx_triangulation_params_t'])
kernel.setParameter(Type.ARRAY,            Direction.INPUT,  ParamState.REQUIRED, "input_track",   ['tivx_triangulation_track_t'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "input_pose",    ['tivx_triangulation_pose_t'])
kernel.setParameter(Type.ARRAY,            Direction.OUTPUT, ParamState.REQUIRED, "output_point",  ['PTK_Point'])

kernel.setTarget(Target.DSP1)
kernel.setTarget(Target.DSP2)
kernel.setTarget(Target.A72_0)
kernel.setTarget(Target.DSP_C7_1)
code.export(kernel)

# DOF to Tracke Node
code.setCoreDirectory("c7x")
kernel = Kernel("dof_to_tracks")

kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "configuration",   ['tivx_dof_to_tracks_params_t'])
kernel.setParameter(Type.IMAGE,            Direction.INPUT,  ParamState.REQUIRED, "input_dof_field", ['VX_DF_IMAGE_U32'])
kernel.setParameter(Type.LUT,              Direction.INPUT,  ParamState.OPTIONAL, "input_d2u_lut",   ['VX_TYPE_FLOAT32'])
kernel.setParameter(Type.ARRAY,            Direction.OUTPUT, ParamState.REQUIRED, "output_tracks",   ['tivx_triangulation_track_t'])

kernel.setTarget(Target.DSP1)
kernel.setTarget(Target.DSP2)
kernel.setTarget(Target.A72_0)
kernel.setTarget(Target.DSP_C7_1)
code.export(kernel)

# SFM OGMAP Node
code.setCoreDirectory("target")
kernel = Kernel("sfm_ogmap")

kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "configuration",         ['tivx_sfm_ogmap_pfsd_params_t'])
kernel.setParameter(Type.ARRAY,            Direction.INPUT,  ParamState.REQUIRED, "input_point",           ['PTK_Point'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "input_point_transform", ['PTK_RigidTransform'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "input_pose",            ['PTK_RigidTransform'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "reftr_and_pose",        ['PTK_InsPoseAndRef'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.OUTPUT, ParamState.REQUIRED, "pfsd_out_desc",         ['PTK_Alg_FsdPfsdPSDesc'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.OUTPUT, ParamState.REQUIRED, "acc_output_map",        ['PTK_Map'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.OUTPUT, ParamState.REQUIRED, "inst_output_map",       ['PTK_Map'])

kernel.setTarget(Target.DSP1)
kernel.setTarget(Target.DSP2)
kernel.setTarget(Target.A72_0)
kernel.setTarget(Target.DSP_C7_1)
code.export(kernel)

# Lidar OGMAP Node
code.setCoreDirectory("target")
kernel = Kernel("lidar_ogmap")

kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "configuration",   ['tivx_lidar_ogmap_pfsd_params_t'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "point_cloud",     ['PTK_PointCloud'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "reftr_and_pose",  ['PTK_InsPoseAndRef'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.OUTPUT, ParamState.REQUIRED, "pfsd_out_desc",   ['PTK_Alg_FsdPfsdPSDesc'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.OUTPUT, ParamState.REQUIRED, "acc_output_map",  ['PTK_Map'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.OUTPUT, ParamState.REQUIRED, "inst_output_map", ['PTK_Map'])

kernel.setTarget(Target.DSP1)
kernel.setTarget(Target.DSP2)
kernel.setTarget(Target.A72_0)
kernel.setTarget(Target.DSP_C7_1)
code.export(kernel)

# Radar OGMAP Node
code.setCoreDirectory("target")
kernel = Kernel("radar_ogmap")

kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "configuration",   ['tivx_radar_ogmap_pfsd_params_t'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "sensor_config",   ['PTK_Alg_RadarSensorConfig'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "radar_obj_data",  ['PTK_Alg_RadarDetOutput'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "reftr_and_pose",  ['PTK_InsPoseAndRef'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.OUTPUT, ParamState.REQUIRED, "pfsd_out_desc",   ['PTK_Alg_FsdPfsdPSDesc'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.OUTPUT, ParamState.REQUIRED, "acc_output_map",  ['PTK_Map'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.OUTPUT, ParamState.REQUIRED, "inst_output_map", ['PTK_Map'])

kernel.setTarget(Target.DSP1)
kernel.setTarget(Target.DSP2)
kernel.setTarget(Target.A72_0)
kernel.setTarget(Target.DSP_C7_1)
code.export(kernel)

# Fused OGMAP Node
code.setCoreDirectory("target")
kernel = Kernel("fused_ogmap")

kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "configuration",  ['tivx_fused_ogmap_pfsd_params_t'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.OPTIONAL, "ogmap_camera",   ['PTK_Map'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.OPTIONAL, "ogmap_radar",    ['PTK_Map'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.OPTIONAL, "ogmap_lidar",    ['PTK_Map'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "reftr_and_pose", ['PTK_InsPoseAndRef'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.OUTPUT, ParamState.REQUIRED, "pfsd_out_desc",  ['PTK_Alg_FsdPfsdPSDesc'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.OUTPUT, ParamState.REQUIRED, "output_map",     ['PTK_Map'])

kernel.setTarget(Target.DSP1)
kernel.setTarget(Target.DSP2)
kernel.setTarget(Target.A72_0)
kernel.setTarget(Target.DSP_C7_1)
code.export(kernel)

# Parking Spot Mapping Node
code.setCoreDirectory("target")
kernel = Kernel("ps_mapping")

kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "configuration", ['tivx_ps_mapping_config_t'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "dataptr",       ['tivx_ps_mapping_input_t'])
kernel.setParameter(Type.LUT,              Direction.INPUT,  ParamState.REQUIRED, "d2u_lut",       ['VX_TYPE_FLOAT32'])
kernel.setParameter(Type.LUT,              Direction.INPUT,  ParamState.REQUIRED, "projmat",       ['VX_TYPE_FLOAT64'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "input_pose",    ['tivx_ps_mapping_pose_t'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "input_map",     ['PTK_Map'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.OUTPUT, ParamState.REQUIRED, "output_map",    ['PTK_Map'])

kernel.setTarget(Target.DSP1)
kernel.setTarget(Target.DSP2)
kernel.setTarget(Target.A72_0)
kernel.setTarget(Target.DSP_C7_1)
code.export(kernel)

# Radar Tracker Node
code.setCoreDirectory("target")
kernel = Kernel("radar_gtrack")

kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "configuration",  ['PTK_Alg_RadarGTrackParams'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "sensor_config",  ['PTK_Alg_RadarSensorConfig'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "radar_obj_data", ['PTK_Alg_RadarDetOutput'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "reftr_and_pose", ['PTK_InsPoseAndRef'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.OUTPUT, ParamState.REQUIRED, "track_info",     ['PTK_Alg_RadarGTrackTargetInfo'])

kernel.setTarget(Target.DSP1)
kernel.setTarget(Target.DSP2)
kernel.setTarget(Target.A72_0)
kernel.setTarget(Target.DSP_C7_1)
code.export(kernel)

