ifeq ($(TARGET_OS), $(filter $(TARGET_OS), QNX FREERTOS SAFERTOS))

include $(PRELUDE)

TARGET      := app_utils_rtos
TARGETTYPE  := library

CSOURCES    := app_rtos_pdk.c
IDIRS       += $(VISION_APPS_PATH)/

include $(FINALE)

endif
