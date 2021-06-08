ifeq ($(TARGET_PLATFORM),J7)
ifeq ($(TARGET_OS),$(filter $(TARGET_OS),SYSBIOS FREERTOS))
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)
TARGET      := app_utils_sensors
TARGETTYPE  := library

CSOURCES    := app_sensors.c

ifeq ($(SOC),j721e)
DEFS+=SOC_J721E
endif

include $(FINALE)

endif
endif
endif

