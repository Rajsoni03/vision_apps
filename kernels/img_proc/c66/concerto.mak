
ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), x86_64 C66 ))

include $(PRELUDE)
TARGET      := vx_target_kernels_img_proc_c66
TARGETTYPE  := library

CSOURCES    := vx_kernels_img_proc_target.c
CSOURCES    += vx_dof_plane_seperation_target.c
CSOURCES    += vx_image_preprocessing_target.c
CSOURCES    += vx_oc_pre_proc_target.c
CSOURCES    += vx_oc_post_proc_target.c
CSOURCES    += vx_od_postprocessing_target.c
CSOURCES    += vx_pixel_visualization_target.c
CSOURCES    += vx_pose_visualization_target.c
CSOURCES    += vx_visual_localization_target.c
CSOURCES    += vx_img_mosaic_target.c
CSOURCES    += vx_draw_keypoint_detections_target.c
CSOURCES    += vx_draw_box_detections_target.c

IDIRS       += $(VISION_APPS_PATH)/kernels/img_proc/include
IDIRS       += $(VISION_APPS_PATH)/kernels/img_proc/host
IDIRS       += $(TIADALG_PATH)/include
IDIRS       += $(IVISION_PATH)
IDIRS       += $(TIDL_PATH)/inc
IDIRS       += $(TIOVX_PATH)/kernels/ivision/include
IDIRS       += $(VXLIB_PATH)/packages

ifeq ($(TARGET_CPU),C66)
DEFS += CORE_DSP
endif

ifeq ($(BUILD_BAM),yes)
DEFS += BUILD_BAM
endif

ifeq ($(TARGET_CPU), x86_64)
DEFS += _HOST_BUILD _TMS320C6600 TMS320C66X HOST_EMULATION
endif

include $(FINALE)

endif
