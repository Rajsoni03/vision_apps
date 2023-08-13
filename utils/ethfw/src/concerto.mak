ifeq ($(BUILD_ENABLE_ETHFW),yes)
ifeq ($(TARGET_PLATFORM), $(filter $(TARGET_PLATFORM), J721E J784S4))
ifeq ($(TARGET_OS),$(filter $(TARGET_OS),SYSBIOS FREERTOS SAFERTOS))
ifeq ($(TARGET_CPU),R5F)
ifeq ($(BUILD_CPU_MCU2_0),yes)

include $(PRELUDE)

IDIRS       += $(ETHFW_PATH)
IDIRS       += $(VISION_APPS_PATH)

TARGET      := app_utils_ethfw
TARGETTYPE  := library

ifeq ($(TARGET_OS),FREERTOS)

IDIRS += $(PDK_PATH)/packages/ti/transport/lwip/lwip-stack/src/include
IDIRS += $(PDK_PATH)/packages/ti/transport/lwip/lwip-port/freertos/include
IDIRS += $(PDK_PATH)/packages/ti/transport/lwip/lwip-contrib
IDIRS += $(PDK_PATH)/packages/ti/kernel/freertos/portable/TI_CGT/r5f
IDIRS += $(PDK_PATH)/packages/ti/kernel/freertos/config/$(SOC)/r5f
IDIRS += $(PDK_PATH)/packages/ti/kernel/freertos/FreeRTOS-LTS/FreeRTOS-Kernel/include
ifeq ($(ETHFW_GPTP_BUILD_SUPPORT),yes)
    IDIRS += $(PDK_PATH)/packages/ti/transport/tsn/tsn-stack
endif

CSOURCES    := app_ethfw_freertos.c

else ifeq ($(TARGET_OS),SAFERTOS)

IDIRS += $(PDK_PATH)/packages/ti/transport/lwip/lwip-stack/src/include
IDIRS += $(PDK_PATH)/packages/ti/transport/lwip/lwip-port/safertos/include
IDIRS += $(PDK_PATH)/packages/ti/transport/lwip/lwip-contrib
IDIRS += $(SAFERTOS_KERNEL_INSTALL_PATH_r5f)/source_code_and_projects/SafeRTOS/api/$(SAFERTOS_ISA_EXT_r5f)
IDIRS += $(SAFERTOS_KERNEL_INSTALL_PATH_r5f)/source_code_and_projects/SafeRTOS/api/PrivWrapperStd
IDIRS += $(SAFERTOS_KERNEL_INSTALL_PATH_r5f)/source_code_and_projects/SafeRTOS/config
IDIRS += $(SAFERTOS_KERNEL_INSTALL_PATH_r5f)/source_code_and_projects/SafeRTOS/kernel/include_api
IDIRS += $(SAFERTOS_KERNEL_INSTALL_PATH_r5f)/source_code_and_projects/SafeRTOS/kernel/include_prv
IDIRS += $(SAFERTOS_KERNEL_INSTALL_PATH_r5f)/source_code_and_projects/SafeRTOS/portable/$(SAFERTOS_ISA_EXT_r5f)
IDIRS += $(SAFERTOS_KERNEL_INSTALL_PATH_r5f)/source_code_and_projects/SafeRTOS/portable/$(SAFERTOS_ISA_EXT_r5f)/$(SAFERTOS_COMPILER_EXT_r5f)

CSOURCES    := app_ethfw_freertos.c

endif

ifeq ($(TARGET_OS),$(filter $(TARGET_OS),FREERTOS SAFERTOS))
  ifeq ($(ETHFW_INTERCORE_ETH_SUPPORT),yes)
    DEFS += ETHAPP_ENABLE_INTERCORE_ETH
  endif
  DEFS += ENABLE_QSGMII_PORTS
endif

# iperf server support
ifeq ($(ETHFW_IPERF_SERVER_SUPPORT),yes)
  DEFS += ETHAPP_ENABLE_IPERF_SERVER
endif

# ETHFW gPTP stack - for now, supported in FreeRTOS only
ifeq ($(ETHFW_GPTP_SUPPORT),yes)
  ifeq ($(TARGET_OS),FREERTOS)
    DEFS += ETHFW_GPTP_SUPPORT
  endif
endif

# Ethfw Intervlan demo
ifeq ($(ETHFW_DEMO_SUPPORT),yes)
  DEFS += ETHFW_DEMO_SUPPORT
endif

include $(FINALE)

endif
endif
endif
endif
endif