ifeq ($(TARGET_PLATFORM),J7)
ifeq ($(TARGET_OS),SYSBIOS)


include $(PRELUDE)
TARGET      := app_tirtos_qnx
TARGETTYPE  := library

CSOURCES    := app_common.c

IDIRS+=$(VISION_APPS_PATH)/apps/basic_demos/app_tirtos/common

include $(FINALE)

endif
endif
