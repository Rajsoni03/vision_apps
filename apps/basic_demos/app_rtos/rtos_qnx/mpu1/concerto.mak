ifeq ($(BUILD_CPU_MPU1),yes)
ifeq ($(TARGET_CPU),A72)
ifeq ($(TARGET_OS),$(filter $(TARGET_OS), QNX))

include $(PRELUDE)
TARGET      := app_rtos_qnx_mpu1_common
TARGETTYPE  := library
CSOURCES    := $(call all-c-files)

IDIRS+=$(VISION_APPS_PATH)/apps/basic_demos/app_rtos/rtos_qnx
IDIRS+=$(VISION_APPS_PATH)/apps/basic_demos/app_rtos/common

include $(FINALE)

endif
endif
endif
                                                   
