ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), X86 x86_64 C66 C71 A72))

include $(PRELUDE)
TARGET      := vx_target_kernels_park_assist
TARGETTYPE  := library
CSOURCES    := $(call all-c-files)

IDIRS       += $(VISION_APPS_PATH)/kernels/park_assist/include
IDIRS       += $(VISION_APPS_PATH)/kernels/lidar/include
IDIRS       += $(VISION_APPS_PATH)/kernels/park_assist/host
IDIRS       += $(VISION_APPS_PATH)/kernels/common/target
IDIRS       += $(VISION_APPS_PATH)/utils/perception
IDIRS       += $(PTK_PATH)/include
IDIRS       += $(TIADALG_PATH)/include
IDIRS       += $(IVISION_PATH)
IDIRS       += $(TIDL_PATH)/inc
IDIRS       += $(TIOVX_PATH)/kernels/ivision/include

#ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), X86 x86_64 C71))
#IDIRS       += $(J7_C_MODELS_PATH)/include
#IDIRS       += ../c7x
#CSOURCES    += ../c7x/vx_dof_to_tracks_target.c
#CSOURCES    += ../c7x/vx_kernels_park_assist_target.c
#CSOURCES    += ../c7x/vx_triangulation_target.c
#endif

#ifneq ($(TARGET_CPU),C71)
#IDIRS       += $(VXLIB_PATH)/packages
#endif

#ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), C71))
#CSOURCES    += ../c7x/VXLIB_triangulatePoints_i32f_o32f_cn.c
#endif

ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), X86 x86_64 C66))
IDIRS       += ../c6x
CSOURCES    += ../c6x/vx_dof_to_tracks_target.c
CSOURCES    += ../c6x/vx_kernels_park_assist_target.c
CSOURCES    += ../c6x/vx_triangulation_target.c
CSOURCES    += ../c6x/VLIB_triangulatePoints.c
endif

ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), A72))
CSOURCES    += ../arm/vx_dof_to_tracks_target.c
CSOURCES    += ../arm/vx_kernels_park_assist_target.c
CSOURCES    += ../arm/vx_triangulation_target.c
CSOURCES    += ../arm/VLIB_triangulatePoints_cn.c
endif

ifeq ($(TARGET_CPU),C66)
DEFS += CORE_DSP
endif

ifeq ($(BUILD_BAM),yes)
DEFS += BUILD_BAM
endif

ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), X86 x86_64))
DEFS += _HOST_BUILD _TMS320C6600 TMS320C66X HOST_EMULATION

# Added the following for c6xsim headers
IDIRS += $(VXLIB_PATH)/packages/ti/vxlib/src/common
endif

include $(FINALE)

endif
