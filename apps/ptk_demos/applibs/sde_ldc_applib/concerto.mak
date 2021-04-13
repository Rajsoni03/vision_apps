ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), x86_64 A72))
ifeq ($(TARGET_OS), $(filter $(TARGET_OS), LINUX))

include $(PRELUDE)
TARGET      := vx_applib_sde_ldc
TARGETTYPE  := library
CPPSOURCES  := $(call all-cpp-files)
CPPFLAGS    := --std=c++11

IDIRS       += $(VISION_APPS_PATH)/apps/ptk_demos/app_common
IDIRS       += $(VISION_APPS_PATH)/kernels/stereo/include
IDIRS       += $(PTK_PATH)/include

include $(FINALE)

endif
endif
