ifeq ($(TARGET_OS),$(filter $(TARGET_OS),SYSBIOS FREERTOS SAFERTOS))
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)
TARGET      := app_utils_pcie_queue
TARGETTYPE  := library

CSOURCES    := app_pcie_queue.c

ifeq ($(TARGET_PLATFORM),J721S2)
SKIPBUILD=1
endif

include $(FINALE)

endif
endif
