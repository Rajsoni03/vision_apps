ifeq ($(TARGET_PLATFORM),J7)
ifeq ($(TARGET_OS),SYSBIOS)
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)
TARGET      := app_utils_sensors
TARGETTYPE  := library

CSOURCES    := app_sensors.c

DEFS+=SOC_J721E

include $(FINALE)

endif
endif
endif

