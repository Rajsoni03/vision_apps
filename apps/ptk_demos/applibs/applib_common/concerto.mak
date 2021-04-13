ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), x86_64 A72))
ifeq ($(TARGET_OS), $(filter $(TARGET_OS), LINUX))

include $(PRELUDE)
TARGET      := vx_applib_common
TARGETTYPE  := library
CSOURCES    := $(call all-c-files)
CPPSOURCES  :=
CPPFLAGS    := --std=c++11

IDIRS       += $(VISION_APPS_PATH)/kernels/img_proc/include
IDIRS       += $(VISION_APPS_PATH)/kernels/img_proc/host
IDIRS       += $(VISION_APPS_KERNELS_IDIRS)
IDIRS       += $(PTK_PATH)/include

ifeq ($(TARGET_CPU),A72)
CPPSOURCES  += cm_dlr_node_cntxt.cpp

IDIRS       += $(LINUX_FS_PATH)/usr/include
endif # ($(TARGET_CPU),A72)

include $(FINALE)

endif
endif
