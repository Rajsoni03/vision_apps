ifneq ($(TARGET_PLATFORM), PC)

ifeq ($(TARGET_OS),$(filter $(TARGET_OS), LINUX QNX))

include $(PRELUDE)

TARGET      := app_utils_codec_wrapper
TARGETTYPE  := library


ifeq ($(TARGET_OS),LINUX)
CSOURCES    := codec_wrapper_linux.c

IDIRS += $(LINUX_FS_PATH)/usr/include/gstreamer-1.0/
IDIRS += $(LINUX_FS_PATH)/usr/include/glib-2.0/
IDIRS += $(LINUX_FS_PATH)/usr/lib/glib-2.0/include/

STATIC_LIBS += app_utils_gst_wrapper
endif

ifeq ($(TARGET_OS),QNX)
CSOURCES    := codec_wrapper_qnx.c

IDIRS += $(PSDK_QNX_PATH)/qnx/codec/img/qnx/OpenMAXIL/khronos/openmaxil/

STATIC_LIBS += app_utils_omax_wrapper
endif

include $(FINALE)

endif

endif
