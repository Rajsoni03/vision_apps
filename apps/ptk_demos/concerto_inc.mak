ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), x86_64 A72))
ifeq ($(TARGET_OS), $(filter $(TARGET_OS), LINUX))

CPPFLAGS        := --std=c++11

IDIRS           += $(VISION_APPS_PATH)/kernels/lidar/include
IDIRS           += $(VISION_APPS_PATH)/kernels/park_assist/include
IDIRS           += $(VISION_APPS_PATH)/kernels/stereo/include
IDIRS           += $(VISION_APPS_PATH)/utils/perception
IDIRS           += $(VISION_APPS_PATH)/apps/ptk_demos/applibs/surround_radar_ogmap_applib
IDIRS           += $(VISION_APPS_PATH)/apps/ptk_demos/applibs/fused_ogmap_applib
IDIRS           += $(VISION_APPS_PATH)/apps/ptk_demos/applibs/surround_sfm_ogmap_applib
IDIRS           += $(VISION_APPS_PATH)/apps/ptk_demos/applibs/lidar_ogmap_applib
IDIRS           += $(VISION_APPS_PATH)/apps/ptk_demos/applibs/ps_mapping_applib
IDIRS           += $(VISION_APPS_PATH)/apps/ptk_demos/applibs/radar_gtrack_applib
IDIRS           += $(VISION_APPS_PATH)/apps/ptk_demos/applibs/sde_obstacle_detection_applib
IDIRS           += $(VISION_APPS_PATH)/apps/ptk_demos/applibs/sde_singlelayer_applib
IDIRS           += $(VISION_APPS_PATH)/apps/ptk_demos/applibs/sde_multilayer_applib
IDIRS           += $(VISION_APPS_PATH)/apps/ptk_demos/applibs/sde_ldc_applib
IDIRS           += $(VISION_APPS_PATH)/apps/ptk_demos/applibs/sde_triangulate_applib
IDIRS           += $(VISION_APPS_PATH)/apps/ptk_demos/applibs/ss_sde_detection_applib
IDIRS           += $(VISION_APPS_PATH)/apps/ptk_demos/applibs/semseg_cnn_applib
IDIRS           += $(VISION_APPS_PATH)/apps/ptk_demos/applibs/applib_common

IDIRS           += $(VISION_APPS_PATH)/apps/ptk_demos/app_common
IDIRS           += $(PTK_PATH)/include

LDIRS           += $(TIADALG_PATH)/lib/$(TARGET_CPU)/$(TARGET_BUILD)
LDIRS           += $(PTK_PATH)/lib/$(TARGET_PLATFORM)/$(TARGET_CPU)/$(TARGET_OS)/$(TARGET_BUILD)

ifeq ($(TARGET_CPU),x86_64)
include $(VISION_APPS_PATH)/apps/concerto_x86_64_inc.mak
CFLAGS          += -DPLATFORM_SDL
CPPFLAGS        += -DPLATFORM_SDL
IDIRS           += /usr/include/freetype2
IDIRS           += $(GLM_PATH)/
SYS_SHARED_LIBS += stdc++ SDL2 SDL2_image m GL GLEW EGL tiff
endif

ifeq ($(TARGET_CPU),A72)
include $(VISION_APPS_PATH)/apps/concerto_a72_inc.mak
CFLAGS          += -DPLATFORM_EGL -DEGL_NO_X11
CPPFLAGS        += -DPLATFORM_EGL
IDIRS           += $(LINUX_FS_PATH)/usr/include/freetype2
LDIRS           += $(LINUX_FS_PATH)/lib
LDIRS           += $(LINUX_FS_PATH)/usr/lib/python3.8/site-packages/dlr

SYS_SHARED_LIBS += pthread
SYS_SHARED_LIBS += rt GLESv2 EGL gbm
SYS_SHARED_LIBS += dl dlr
endif

# This section is for apps to link against static libs instead of tivision_apps library
# Also used to create tivision_apps library (so we can maintain lib list in one place)
LINK_STATIC_LIBS := yes

ifeq ($(LINK_SHARED_OBJ)$(TARGETTYPE),yesexe)
LINK_STATIC_LIBS := no
endif

# Currently doesn't support PC shared object linking yet.
ifeq ($(TARGET_CPU),x86_64)
LINK_STATIC_LIBS := yes
endif

ifeq ($(LINK_STATIC_LIBS),yes)

STATIC_LIBS     += ptk_base ptk_algos ptk_dbtools ptk_utils ptk_net ptk_drv ptk_gui
SYS_SHARED_LIBS += wayland-client wayland-server wayland-egl freetype IL ILU
STATIC_LIBS     += vx_app_ptk_demo_common
STATIC_LIBS     += vx_applib_fused_ogmap
STATIC_LIBS     += vx_applib_lidar_ogmap
STATIC_LIBS     += vx_applib_surround_radar_ogmap
STATIC_LIBS     += vx_applib_surround_sfm_ogmap
STATIC_LIBS     += vx_applib_sde_obstacle_detection
STATIC_LIBS     += vx_applib_sde_multilayer
STATIC_LIBS     += vx_applib_sde_singlelayer
STATIC_LIBS     += vx_applib_sde_ldc
STATIC_LIBS     += vx_applib_sde_triangulate
STATIC_LIBS     += vx_applib_ps_mapping
STATIC_LIBS     += vx_applib_radar_gtrack
STATIC_LIBS     += vx_applib_ss_sde_detection

STATIC_LIBS     += vx_kernels_common
STATIC_LIBS     += vx_kernels_lidar vx_target_kernels_lidar_arm
STATIC_LIBS     += vx_kernels_stereo vx_target_kernels_stereo
STATIC_LIBS     += vx_kernels_park_assist vx_target_kernels_park_assist
STATIC_LIBS     += vx_applib_common
ifeq ($(TARGET_CPU),A72)
STATIC_LIBS     += vx_applib_semseg_cnn
endif

endif  # ifeq ($(LINK_STATIC_LIBS),yes)

endif #ifeq ($(TARGET_OS), $(filter $(TARGET_OS), LINUX))
endif #ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), x86_64 A72))
