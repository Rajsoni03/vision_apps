ifeq ($(TARGET_OS),$(filter $(TARGET_OS),SYSBIOS FREERTOS))
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)
TARGET      := app_utils_pcie_queue
TARGETTYPE  := library

CSOURCES    := app_pcie_queue.c

include $(FINALE)

endif
endif
