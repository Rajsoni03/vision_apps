ifeq ($(TARGET_PLATFORM),J7)
ifeq ($(TARGET_OS), $(filter $(TARGET_OS), SYSBIOS FREERTOS))

include $(PRELUDE)
TARGET      := app_rtos_linux
TARGETTYPE  := library

CSOURCES    := app_common.c

ifeq ($(TARGET_OS),FREERTOS)
CSOURCES    += ipc_trace.c
endif

IDIRS+=$(VISION_APPS_PATH)/apps/basic_demos/app_rtos/common

include $(FINALE)

endif
endif
