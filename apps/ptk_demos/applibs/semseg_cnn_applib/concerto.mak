ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), A72))
ifeq ($(TARGET_OS), $(filter $(TARGET_OS), LINUX))

include $(PRELUDE)
TARGET      := vx_applib_semseg_cnn
TARGETTYPE  := library
CSOURCES    := $(call all-c-files)
CPPSOURCES  := $(call all-cpp-files)
CPPFLAGS    := --std=c++11

IDIRS       += $(VISION_APPS_PATH)/apps/ptk_demos/app_common
IDIRS       += $(VISION_APPS_PATH)/apps/ptk_demos/applibs/applib_common
IDIRS       += $(VISION_APPS_PATH)/kernels/img_proc/include
IDIRS       += $(VISION_APPS_PATH)/kernels/img_proc/host
IDIRS       += $(PTK_PATH)/include

IDIRS       += $(VISION_APPS_KERNELS_IDIRS)

STATIC_LIBS += $(VISION_APPS_KERNELS_LIBS)
STATIC_LIBS += $(TIADALG_LIBS)

include $(FINALE)

endif
endif
