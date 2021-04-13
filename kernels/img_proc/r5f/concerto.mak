
ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), R5F))

include $(PRELUDE)
TARGET      := vx_target_kernels_img_proc_r5f
TARGETTYPE  := library

CSOURCES    := vx_kernels_img_proc_target.c
CSOURCES    += vx_img_mosaic_msc_target.c
CSOURCES    += vx_img_mosaic_draw_overlay_avp2.c

IDIRS       += $(VISION_APPS_PATH)/kernels/img_proc/include
IDIRS       += $(VISION_APPS_PATH)/kernels/img_proc/host
IDIRS       += $(TIADALG_PATH)/include
IDIRS       += $(IVISION_PATH)
IDIRS       += $(TIDL_PATH)/inc
IDIRS       += $(TIOVX_PATH)/kernels/ivision/include
IDIRS       += $(VXLIB_PATH)/packages

DEFS        += SOC_J721E

include $(FINALE)

endif
