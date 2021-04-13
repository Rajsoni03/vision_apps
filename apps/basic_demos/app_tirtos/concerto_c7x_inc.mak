ifeq ($(TARGET_CPU),C71)

IDIRS+=$(VISION_APPS_PATH)/apps/basic_demos/app_tirtos/common
IDIRS+=$(VISION_APPS_PATH)/kernels/img_proc/include
IDIRS+=$(VISION_APPS_PATH)/kernels/fileio/include
IDIRS+=$(VISION_APPS_PATH)/kernels/srv/include
IDIRS+=$(VISION_APPS_PATH)/kernels/park_assist/include
IDIRS+=$(PTK_PATH)/include
IDIRS+=$(VISION_APPS_PATH)/kernels/stereo/include
IDIRS+=$(IMAGING_PATH)/kernels/include
IDIRS+=$(TIADALG_PATH)/include

LDIRS += $(PDK_PATH)/packages/ti/osal/lib/tirtos/j721e/c7x/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/csl/lib/j721e/c7x/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/ipc/lib/j721e/c7x_1/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/udma/lib/j721e/c7x_1/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/sciclient/lib/j721e/c7x_1/$(TARGET_BUILD)/
LDIRS += $(TIOVX_PATH)/lib/$(TARGET_PLATFORM)/$(TARGET_CPU)/$(TARGET_OS)/$(TARGET_BUILD)
LDIRS += $(PTK_PATH)/lib/$(TARGET_PLATFORM)/$(TARGET_CPU)/$(TARGET_OS)/$(TARGET_BUILD)
LDIRS += $(VISION_APPS_PATH)/lib/$(TARGET_PLATFORM)/$(TARGET_CPU)/$(TARGET_OS)/$(TARGET_BUILD)
LDIRS += $(MMALIB_PATH)/lib/release
LDIRS += $(TIDL_PATH)/lib/dsp/algo/release
LDIRS += $(TIADALG_PATH)/lib/$(TARGET_CPU)/$(TARGET_BUILD)

STATIC_LIBS += app_utils_mem
STATIC_LIBS += app_utils_console_io
STATIC_LIBS += app_utils_ipc
STATIC_LIBS += vx_app_c7x_target_kernel
STATIC_LIBS += app_utils_remote_service
STATIC_LIBS += app_utils_udma
STATIC_LIBS += app_utils_sciclient
STATIC_LIBS += app_utils_misc
STATIC_LIBS += app_utils_perf_stats
STATIC_LIBS += vx_target_kernels_park_assist
STATIC_LIBS += vx_target_kernels_stereo
STATIC_LIBS += vx_app_ptk_demo_common
STATIC_LIBS += vx_kernels_common
STATIC_LIBS += vx_target_kernels_img_proc_c71

PTK_LIBS =
PTK_LIBS += ptk_algos
PTK_LIBS += ptk_utils
PTK_LIBS += ptk_base

SYS_STATIC_LIBS += $(PTK_LIBS)

TIOVX_LIBS =
TIOVX_LIBS += vx_target_kernels_tidl
TIOVX_LIBS += vx_target_kernels_ivision_common
TIOVX_LIBS += vx_framework vx_platform_psdk_j7_bios vx_kernels_target_utils

TIDL_LIBS =
TIDL_LIBS += common_C7100
TIDL_LIBS += mmalib_C7100
TIDL_LIBS += mmalib_cn_C7100
TIDL_LIBS += tidl_algo
TIDL_LIBS += tidl_priv_algo
TIDL_LIBS += tidl_obj_algo
TIDL_LIBS += tidl_custom

SYS_STATIC_LIBS += $(TIOVX_LIBS) $(TIDL_LIBS)

ADDITIONAL_STATIC_LIBS += ti.osal.ae71
ADDITIONAL_STATIC_LIBS += ipc.ae71
ADDITIONAL_STATIC_LIBS += ti.csl.ae71
ADDITIONAL_STATIC_LIBS += udma.ae71
ADDITIONAL_STATIC_LIBS += dmautils.ae71
ADDITIONAL_STATIC_LIBS += sciclient.ae71
ADDITIONAL_STATIC_LIBS += libc.a
ADDITIONAL_STATIC_LIBS += libtiadalg_structure_from_motion.a
endif
