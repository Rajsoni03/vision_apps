ifeq ($(TARGET_PLATFORM),J7)
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
IDIRS += $(PDK_PATH)/packages/ti/transport/lwip/lwip-port/freertos/include

CSOURCES    := app_ethfw_freertos.c

endif

ifeq ($(TARGET_OS),FREERTOS)
  ifeq ($(ETHFW_INTERCORE_ETH_SUPPORT),yes)
    DEFS += ETHAPP_ENABLE_INTERCORE_ETH
  endif
  DEFS += ENABLE_QSGMII_PORTS
endif

include $(FINALE)

endif
endif
endif
endif
