ifeq ($(TARGET_CPU),A72)
ifeq ($(TARGET_OS), $(filter $(TARGET_OS), LINUX QNX))

include $(PRELUDE)

TARGET      := tivision_apps
TARGETTYPE  := dsmo
VERSION     := $(PSDK_VERSION)

ifeq ($(TARGET_OS), $(filter $(TARGET_OS), LINUX))
# ptk_demos/concerto_inc.mak includes concerto_a72_inc.mak also
ifeq ($(SOC), am62a)
# PTK is not supported on AM62A
include $(VISION_APPS_PATH)/apps/concerto_a72_inc.mak
else
include $(VISION_APPS_PATH)/apps/ptk_demos/concerto_inc.mak
endif
else
# PTK is not supported on QNX
include $(VISION_APPS_PATH)/apps/concerto_a72_inc.mak
endif

STATIC_LIBS     += $(IMAGING_LIBS)
STATIC_LIBS     += $(VISION_APPS_KERNELS_LIBS)
STATIC_LIBS     += $(VISION_APPS_MODULES_LIBS)
STATIC_LIBS     += $(TEST_LIBS)

ifneq ($(SOC), am62a)
STATIC_LIBS     += $(VISION_APPS_SRV_LIBS)

# Following is needed since PTK has a copy of this library (PTK is only Linux)
ifeq ($(TARGET_OS), LINUX)
STATIC_LIBS := $(filter-out app_utils_opengl, $(STATIC_LIBS))
endif

SHARED_LIBS += EGL
SHARED_LIBS += GLESv2
endif

ifeq ($(TARGET_OS),QNX)
SYS_SHARED_LIBS += screen
endif

include $(FINALE)

endif
endif
