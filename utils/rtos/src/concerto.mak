ifeq ($(TARGET_OS), $(filter $(TARGET_OS), QNX FREERTOS SAFERTOS))

include $(PRELUDE)

TARGET      := app_utils_rtos
TARGETTYPE  := library

ifeq ($(RTOS_SDK),pdk)
CSOURCES    := app_rtos_pdk.c
else
CSOURCES    := app_rtos_mcu_plus_sdk.c
endif
IDIRS       += $(VISION_APPS_PATH)/

include $(FINALE)

endif
