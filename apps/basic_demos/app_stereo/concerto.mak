ifeq ($(TARGET_CPU),$(filter $(TARGET_CPU), x86_64 A72))

include $(PRELUDE)

TARGET      := vx_app_stereo_depth

CSOURCES    := main.c

include $(VISION_APPS_PATH)/apps/ptk_demos/concerto_inc.mak

ifeq ($(TARGET_CPU),x86_64)

TARGETTYPE  := exe
CSOURCES    += main_x86.c

SYSLDIRS += /usr/lib64
SYS_SHARED_LIBS += $(x86_64_OPENGL_SYS_SHARED_LIBS)

endif

ifeq ($(TARGET_CPU),A72)
ifeq ($(TARGET_OS),$(filter $(TARGET_OS), LINUX QNX))

TARGETTYPE  := exe
CSOURCES    += main_linux_arm.c

include $(VISION_APPS_PATH)/apps/concerto_a72_inc.mak

IDIRS += $(VISION_APPS_PATH)/kernels/stereo/include
IDIRS += $(PTK_PATH)/include

STATIC_LIBS += vx_kernels_stereo

endif
endif

ifeq ($(TARGET_CPU),A72)
ifeq ($(TARGET_OS),SYSBIOS)

TARGETTYPE  := library

include $(VISION_APPS_PATH)/apps/concerto_a72_inc.mak

endif
endif

IDIRS += $(VISION_APPS_KERNELS_IDIRS)

STATIC_LIBS += $(VISION_APPS_KERNELS_LIBS)

include $(FINALE)

endif
