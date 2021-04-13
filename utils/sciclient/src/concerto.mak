ifeq ($(TARGET_PLATFORM),J7)
ifeq ($(TARGET_OS),SYSBIOS)


include $(PRELUDE)
TARGET      := app_utils_sciclient
TARGETTYPE  := library

CSOURCES    := app_sciclient.c

DEFS+=SOC_J721E

include $(FINALE)

endif
endif
