ifeq ($(TARGET_OS),$(filter $(TARGET_OS),SYSBIOS FREERTOS))
ifeq ($(TARGET_CPU),R5F)
ifeq ($(BUILD_CPU_MCU2_0),yes)

include $(PRELUDE)

IDIRS       += $(ETHFW_PATH)
IDIRS       += $(VISION_APPS_PATH)

TARGET      := app_utils_ethfw
TARGETTYPE  := library

ifeq ($(TARGET_OS),SYSBIOS)

CSOURCES    := app_ethfw_tirtos.c

else ifeq ($(TARGET_OS),FREERTOS)

IDIRS += $(PDK_PATH)/packages/ti/transport/lwip/lwip-stack/src/include
IDIRS += $(PDK_PATH)/packages/ti/drv/enet/lwipif/ports/freertos/include

CSOURCES    := app_ethfw_freertos.c

endif

include $(FINALE)

endif
endif
endif
