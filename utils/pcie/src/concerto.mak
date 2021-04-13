ifeq ($(TARGET_PLATFORM),J7)
ifeq ($(TARGET_OS),SYSBIOS)
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)
TARGET      := app_utils_pcie_queue
TARGETTYPE  := library

CSOURCES    := app_pcie_queue.c

DEFS+=SOC_J721E

include $(FINALE)

endif
endif
endif
