ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), A72))
ifeq ($(TARGET_OS), $(filter $(TARGET_OS), LINUX))

include $(PRELUDE)
TARGET      := vx_app_dlrt_test
TARGETTYPE  := exe
CPPSOURCES  := $(call all-cpp-files)

include $(VISION_APPS_PATH)/apps/ptk_demos/concerto_inc.mak

STATIC_LIBS += $(VISION_APPS_KERNELS_LIBS)
STATIC_LIBS += $(TIADALG_LIBS)

SKIPBUILD=1

include $(FINALE)

endif #ifeq ($(TARGET_OS), $(filter $(TARGET_OS), LINUX))
endif #ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), x86_64 A72))
