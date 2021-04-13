'''
* Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
* ALL RIGHTS RESERVED
'''

from tiovx import *

code = KernelExportCode(Module.LIDAR,
                        Core.ARM,
                        env_var="CUSTOM_APPLICATION_PATH")

# Lidar GPC Node
code.setCoreDirectory("arm")
kernel = Kernel("lidar_gpc")

kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "configuration",   ['PTK_Lidar_GpcConfig'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "point_cloud_in",  ['PTK_PointCloud'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "lidar_meta",      ['PTK_LidarMeta'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "M_ego_lidar",     ['PTK_RigidTransform'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "root_ecef",       ['PTK_Position'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.OUTPUT, ParamState.REQUIRED, "point_cloud_out", ['PTK_PointCloud'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.OUTPUT, ParamState.REQUIRED, "normal_cloud",    ['PTK_PointCloud'])

kernel.setTarget(Target.A72_0)
code.export(kernel)

# Lidar MPC Node
code.setCoreDirectory("arm")
kernel = Kernel("lidar_mdc")

kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "point_cloud",     ['PTK_PointCloud'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "meta",            ['PTK_LidarMeta'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "M_ego_lidar",     ['PTK_RigidTransform'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT,  ParamState.REQUIRED, "root_ecef",       ['PTK_Position'])
kernel.setParameter(Type.USER_DATA_OBJECT, Direction.OUTPUT, ParamState.REQUIRED, "mdc_point_cloud", ['PTK_PointCloud'])

kernel.setTarget(Target.A72_0)
code.export(kernel)

