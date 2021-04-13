ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), A72))

include $(PRELUDE)

TARGET      := vx_app_encode
TARGETTYPE  := exe

CSOURCES    := main.c app_sensor_module.c app_capture_module.c app_viss_module.c app_aewb_module.c app_ldc_module.c app_img_mosaic_module.c app_encode_module.c
CSOURCES    += main_hlos_arm.c

include $(VISION_APPS_PATH)/apps/concerto_a72_inc.mak

IDIRS += $(IMAGING_IDIRS)
IDIRS += $(VISION_APPS_KERNELS_IDIRS)

STATIC_LIBS += $(IMAGING_LIBS)
STATIC_LIBS += $(VISION_APPS_KERNELS_LIBS)

include $(FINALE)

endif
