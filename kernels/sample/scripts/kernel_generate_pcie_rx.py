'''
* Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
* ALL RIGHTS RESERVED
'''

from tiovx import *

code = KernelExportCode("sample", "r5f", "VISION_APPS_PATH")

kernel = Kernel("pcie_rx")

kernel.setParameter(Type.USER_DATA_OBJECT, Direction.INPUT, ParamState.REQUIRED, "CONFIGURATION", ['tivx_pcie_rx_params_t'])
kernel.setParameter(Type.OBJECT_ARRAY, Direction.OUTPUT, ParamState.REQUIRED, "OUTPUT", ['VX_DF_IMAGE_NV12'])

kernel.setTarget(Target.MCU2_0)

code.export(kernel)
code.exportDiagram(kernel)