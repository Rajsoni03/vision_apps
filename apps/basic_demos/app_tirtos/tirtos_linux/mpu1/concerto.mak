ifeq ($(BUILD_CPU_MPU1),yes)
ifeq ($(TARGET_CPU),A72)
ifeq ($(TARGET_OS),LINUX)

include $(PRELUDE)

TARGET      := app_tirtos_linux_mpu1_common
TARGETTYPE  := library
CSOURCES    := $(call all-c-files)

IDIRS+=$(VISION_APPS_PATH)/apps/basic_demos/app_tirtos/tirtos_linux
IDIRS+=$(VISION_APPS_PATH)/apps/basic_demos/app_tirtos/common

include $(FINALE)

endif
endif
endif
                                                   
