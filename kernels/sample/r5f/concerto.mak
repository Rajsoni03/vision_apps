
ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), R5F ))

include $(PRELUDE)
TARGET      := vx_target_kernels_sample_r5f
TARGETTYPE  := library
CSOURCES    := $(call all-c-files)
IDIRS       += $(VISION_APPS_PATH)/kernels/sample/include
IDIRS       += $(VISION_APPS_PATH)/kernels/sample/host
IDIRS       += $(TIOVX_PATH)/kernels/include
IDIRS       += $(VXLIB_PATH)/packages

include $(FINALE)
endif

