ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), x86_64 A72))

include $(PRELUDE)

TARGET      := vx_app_multi_cam_encode
TARGETTYPE  := exe

CSOURCES    := main.c multi_cam_encode_ldc_module.c multi_cam_encode_scaler_module.c tiovx_img_mosaic_module.c app_common.c

ifeq ($(TARGET_CPU),x86_64)
include $(VISION_APPS_PATH)/apps/concerto_x86_64_inc.mak
CSOURCES    += main_x86.c
# Not building for PC
SKIPBUILD=1
endif

ifeq ($(TARGET_CPU),A72)
ifeq ($(TARGET_OS), $(filter $(TARGET_OS), LINUX QNX))
include $(VISION_APPS_PATH)/apps/concerto_a72_inc.mak
CSOURCES    += main_linux_arm.c
endif
endif

IDIRS += $(IMAGING_IDIRS)
IDIRS += $(VISION_APPS_KERNELS_IDIRS)
IDIRS += $(VISION_APPS_MODULES_IDIRS)
IDIRS += $(LINUX_FS_PATH)/usr/include/gstreamer-1.0/
IDIRS += $(LINUX_FS_PATH)/usr/include/glib-2.0/
IDIRS += $(LINUX_FS_PATH)/usr/lib/glib-2.0/include/

STATIC_LIBS += $(IMAGING_LIBS)
STATIC_LIBS += $(VISION_APPS_KERNELS_LIBS)
STATIC_LIBS += $(VISION_APPS_MODULES_LIBS)
ifeq ($(TARGET_OS), LINUX)
STATIC_LIBS += app_utils_gst_codec
endif

SHARED_LIBS += gstreamer-1.0
SHARED_LIBS += gstapp-1.0
SHARED_LIBS += gstbase-1.0
SHARED_LIBS += gobject-2.0
SHARED_LIBS += glib-2.0

include $(FINALE)

endif
