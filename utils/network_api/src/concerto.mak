ifeq ($(TARGET_PLATFORM),J7)

include $(PRELUDE)
TARGET      := app_utils_network_api
TARGETTYPE  := library

ifeq ($(TARGET_OS),LINUX)
CSOURCES    := network_api.c

endif

ifeq ($(SOC),j721e)
DEFS=SOC_J721E
endif

IDIRS += $(VISION_APPS_PATH)
IDIRS += $(VISION_APPS_PATH)/utils/itt_server/include
IDIRS += $(VISION_APPS_PATH)/utils/network_api/include
IDIRS += $(IMAGING_PATH)/sensor_drv/include

include $(FINALE)

endif
