
ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), X86 x86_64 A15 M4 A72 R5F))

include $(PRELUDE)
TARGET      := vx_kernels_lidar
TARGETTYPE  := library
CSOURCES    := $(call all-c-files)

IDIRS       += $(VISION_APPS_PATH)/kernels/lidar/include
IDIRS       += $(PTK_PATH)/include

include $(FINALE)

endif
