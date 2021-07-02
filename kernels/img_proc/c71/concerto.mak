
ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), x86_64 C71 ))

include $(PRELUDE)
TARGET      := vx_target_kernels_img_proc_c71
TARGETTYPE  := library

CSOURCES    := vx_kernels_img_proc_target.c
CSOURCES    += vx_sfm_target.c

IDIRS       += $(VISION_APPS_PATH)/kernels/img_proc/include
IDIRS       += $(VISION_APPS_PATH)/kernels/img_proc/host
IDIRS       += $(TIADALG_PATH)/include
IDIRS       += $(IVISION_PATH)
IDIRS       += $(TIOVX_PATH)/source/include
IDIRS       += $(TIOVX_PATH)/kernels/ivision/include
IDIRS       += $(VXLIB_PATH)/packages
IDIRS       += $(PDK_PATH)/packages

ifeq ($(TARGET_CPU),C71)
DEFS += CORE_DSP
endif

DEFS += SOC_J721E

ifeq ($(TARGET_CPU), x86_64)
DEFS += _HOST_BUILD _TMS320C6600 TMS320C66X HOST_EMULATION __aarch64__
endif

include $(FINALE)

endif
