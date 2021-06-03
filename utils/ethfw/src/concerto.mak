ifeq ($(TARGET_PLATFORM),J7)
ifeq ($(TARGET_OS),SYSBIOS)
ifeq ($(TARGET_CPU),R5F)
ifeq ($(BUILD_CPU_MCU2_0),yes)

include $(PRELUDE)

IDIRS       += $(ETHFW_PATH)
IDIRS       += $(VISION_APPS_PATH)

TARGET      := app_utils_ethfw
TARGETTYPE  := library

CSOURCES    := app_ethfw.c

ifeq ($(SOC),j721e)
DEFS+=SOC_J721E
endif

include $(FINALE)

endif
endif
endif
endif
