ifneq ($(TARGET_PLATFORM),PC)

ifeq ($(TARGET_OS),$(filter $(TARGET_OS), QNX))
ifeq ($(TARGET_PLATFORM), $(filter $(TARGET_PLATFORM), J7))

include $(PRELUDE)

TARGET      := app_utils_omax_wrapper
TARGETTYPE  := library

CSOURCES    := $(call all-c-files)

IDIRS += $(PSDK_QNX_PATH)/qnx/codec/img/qnx/OpenMAXIL/khronos/openmaxil/
IDIRS += $(PSDK_QNX_PATH)/qnx/codec/img/qnx/OpenMAXIL/core/public/khronos/openmaxil

include $(FINALE)

endif
endif

endif
