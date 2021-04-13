ifeq ($(TARGET_PLATFORM),J7)
ifeq ($(TARGET_OS),SYSBIOS)


include $(PRELUDE)
TARGET      := app_utils_udma
TARGETTYPE  := library

CSOURCES    := app_udma.c app_udma_test.c

DEFS+=SOC_J721E

include $(FINALE)

endif
endif
