
include $(PRELUDE)

TARGET      := app_utils_iss
TARGETTYPE  := library
IMAGING_IDIRS += $(IMAGING_PATH)/sensor_drv/include
IDIRS += $(IMAGING_IDIRS)

ifeq ($(TARGET_PLATFORM),PC)
CSOURCES    := app_iss_x86.c app_iss_common.c
endif

ifeq ($(TARGET_PLATFORM),J7)

ifeq ($(SOC),j721e)
DEFS=SOC_J721E
endif

ifeq ($(TARGET_CPU),$(filter $(TARGET_CPU), A72))
CSOURCES    := app_iss_common.c app_iss.c
endif

ifeq ($(TARGET_CPU),$(filter $(TARGET_CPU), R5F))
CSOURCES    := app_iss.c
endif

endif

include $(FINALE)
