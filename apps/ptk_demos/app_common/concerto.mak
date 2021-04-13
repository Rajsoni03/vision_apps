ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), x86_64 C66 C71 A72))

include $(PRELUDE)
TARGET      := vx_app_ptk_demo_common
TARGETTYPE  := library
CSOURCES    := ptk_extdep_impl.c
CPPSOURCES  := 
CPPFLAGS    := --std=c++11

ifeq ($(TARGET_CPU),x86_64)
include $(VISION_APPS_PATH)/apps/concerto_x86_64_inc.mak
CFLAGS          += -DTARGET_X86_64
CSOURCES        += app_ptk_demo_common.c
CPPSOURCES      += app_ptk_demo_profile.cpp app_ptk_demo_disparity.cpp
endif #ifeq ($(TARGET_CPU),x86_64)

ifeq ($(TARGET_CPU),A72)
ifeq ($(TARGET_OS), $(filter $(TARGET_OS), LINUX))
include $(VISION_APPS_PATH)/apps/concerto_a72_inc.mak
CPPFLAGS        += -DPLATFORM_EGL
CSOURCES        += app_ptk_demo_common.c
CPPSOURCES      += app_ptk_demo_profile.cpp app_ptk_demo_display.cpp app_ptk_demo_disparity.cpp
endif #ifeq ($(TARGET_OS), $(filter $(TARGET_OS), LINUX))
endif #ifeq ($(TARGET_CPU),A72)

IDIRS       += $(PTK_PATH)/include
IDIRS       += $(VISION_APPS_PATH)/kernels/stereo/include

include $(FINALE)

endif # ifeq ($(TARGET_CPU), ...

