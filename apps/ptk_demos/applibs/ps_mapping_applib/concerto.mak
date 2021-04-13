ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), x86_64 A72))

include $(PRELUDE)
TARGET      := vx_applib_ps_mapping
TARGETTYPE  := library
CSOURCES    := $(call all-c-files)

IDIRS       += $(VISION_APPS_PATH)/kernels/park_assist/include
IDIRS       += $(PTK_PATH)/include
IDIRS       += $(IVISION_PATH)
IDIRS       += $(TIDL_PATH)/inc

include $(FINALE)

endif
