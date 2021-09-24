ifneq ($(TARGET_PLATFORM),PC)
ifeq ($(TARGET_OS),$(filter $(TARGET_OS),SYSBIOS FREERTOS))

include $(PRELUDE)
TARGET      := app_utils_udma
TARGETTYPE  := library

CSOURCES    := app_udma.c app_udma_test.c

include $(FINALE)

endif
endif
