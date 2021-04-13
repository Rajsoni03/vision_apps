ifeq ($(TARGET_PLATFORM),J7)
ifeq ($(TARGET_OS),SYSBIOS)
ifeq ($(TARGET_CPU),R5F)
ifeq ($(BUILD_CPU_MCU1_0),yes)

include $(PRELUDE)
TARGET      := app_utils_sciserver
TARGETTYPE  := library

CSOURCES    := app_sciserver.c

DEFS+=SOC_J721E
DEFS+=BUILD_MCU1_0
DEFS+=BUILD_MCU

include $(FINALE)

endif
endif
endif
endif