ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), x86_64 A72))
ifeq ($(TARGET_OS), $(filter $(TARGET_OS), LINUX))

include $(PRELUDE)
TARGET      := vx_app_valet_parking
TARGETTYPE  := exe
CPPSOURCES  := $(call all-cpp-files)
CSOURCES    := $(call all-c-files)

IDIRS       += $(VISION_APPS_PATH)/apps/ptk_demos/app_valet_parking/config_data

include $(VISION_APPS_PATH)/apps/ptk_demos/concerto_inc.mak

include $(FINALE)

endif #ifeq ($(TARGET_OS), $(filter $(TARGET_OS), LINUX))
endif #ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), x86_64 A72))
