
ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), X86 x86_64 C66 C71 A72))

include $(PRELUDE)
TARGET      := vx_target_kernels_stereo
TARGETTYPE  := library
CSOURCES    := $(call all-c-files)
IDIRS       += $(VISION_APPS_PATH)/kernels/stereo/include
IDIRS       += $(VISION_APPS_PATH)/kernels/stereo/host
IDIRS       += $(VISION_APPS_PATH)/kernels/common/target
IDIRS       += $(HOST_ROOT)/kernels/include
IDIRS       += $(VXLIB_PATH)/packages
IDIRS       += $(PTK_PATH)/include
# < DEVELOPER_TODO: Add any custom include paths using 'IDIRS' >
# < DEVELOPER_TODO: Add any custom preprocessor defines or build options needed using
#                   'CFLAGS'. >
# < DEVELOPER_TODO: Adjust which cores this library gets built on using 'SKIPBUILD'. >

ifeq ($(TARGET_CPU),C66)
DEFS += CORE_DSP
endif

ifeq ($(BUILD_BAM),yes)
DEFS += BUILD_BAM
endif

ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), X86 x86_64))
DEFS += _HOST_BUILD _TMS320C6600 TMS320C66X HOST_EMULATION
endif

include $(FINALE)
endif

