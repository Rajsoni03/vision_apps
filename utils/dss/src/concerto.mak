ifeq ($(TARGET_PLATFORM),J7)
ifeq ($(TARGET_OS),$(filter $(TARGET_OS),SYSBIOS FREERTOS))
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)
TARGET      := app_utils_dss
TARGETTYPE  := library

CSOURCES    := app_dss.c app_dss_j721e.c app_dctrl.c app_dss_defaults.c app_dss_dual_display_defaults.c

ifeq ($(SOC),j721e)
DEFS+=SOC_J721E
DEFS+=j721e_evm
endif

ifeq ($(BUILD_ENABLE_ETHFW),yes)
DEFS+=ENABLE_ETHFW
endif

include $(FINALE)

endif
endif
endif

