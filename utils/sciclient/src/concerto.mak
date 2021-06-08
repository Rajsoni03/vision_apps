ifeq ($(TARGET_PLATFORM),J7)
ifeq ($(TARGET_OS),$(filter $(TARGET_OS),SYSBIOS FREERTOS))

include $(PRELUDE)
TARGET      := app_utils_sciclient
TARGETTYPE  := library

CSOURCES    := app_sciclient.c

ifeq ($(SOC),j721e)
DEFS+=SOC_J721E
endif

include $(FINALE)

endif
endif
