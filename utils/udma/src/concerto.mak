ifeq ($(TARGET_PLATFORM),J7)
ifeq ($(TARGET_OS),$(filter $(TARGET_OS),SYSBIOS FREERTOS))

include $(PRELUDE)
TARGET      := app_utils_udma
TARGETTYPE  := library

CSOURCES    := app_udma.c app_udma_test.c

ifeq ($(SOC),j721e)
DEFS+=SOC_J721E
endif

include $(FINALE)

endif
endif
