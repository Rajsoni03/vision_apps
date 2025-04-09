ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), A72 A53))

include $(PRELUDE)

CSOURCES    := app_heterogeneous.c
TARGET      := vx_app_heterogeneous
TARGETTYPE  := exe

include $(VISION_APPS_PATH)/apps/concerto_mpu_inc.mak

ifeq ($(TARGET_OS), $(filter $(TARGET_OS), LINUX QNX))
STATIC_LIBS += $(IMAGING_LIBS)
endif

IDIRS += $(IMAGING_IDIRS)

include $(FINALE)

endif