
ifeq ($(TARGET_CPU),A72)

TEST_IDIRS =
TEST_IDIRS += $(TIOVX_PATH)/conformance_tests

IMAGING_IDIRS  =
IMAGING_IDIRS += $(IMAGING_PATH)/kernels/include
IMAGING_IDIRS += $(IMAGING_PATH)/sensor_drv/include
IMAGING_IDIRS += $(VISION_APPS_PATH)/utils/itt_server/include
IMAGING_IDIRS += $(VISION_APPS_PATH)/utils/network_api/include

TIADALG_IDIRS = 
TIADALG_IDIRS = $(TIADALG_PATH)/include

VISION_APPS_KERNELS_IDIRS =
VISION_APPS_KERNELS_IDIRS += $(VISION_APPS_PATH)/kernels
VISION_APPS_KERNELS_IDIRS += $(VISION_APPS_PATH)/kernels/img_proc/include
VISION_APPS_KERNELS_IDIRS += $(VISION_APPS_PATH)/kernels/fileio/include

VISION_APPS_MODULES_IDIRS =
VISION_APPS_MODULES_IDIRS += $(VISION_APPS_PATH)/modules/include

VISION_APPS_SRV_IDIRS =
VISION_APPS_SRV_IDIRS += $(VISION_APPS_PATH)/kernels/srv/include
VISION_APPS_SRV_IDIRS += $(VISION_APPS_PATH)/kernels/srv/c66
VISION_APPS_SRV_IDIRS += $(VISION_APPS_PATH)/kernels/srv/gpu/3dsrv
VISION_APPS_SRV_IDIRS += $(VISION_APPS_PATH)/kernels/sample/include
VISION_APPS_SRV_IDIRS += $(VISION_APPS_PATH)/kernels/sample/host

VISION_APPS_APPLIBS_IDIRS =
VISION_APPS_APPLIBS_IDIRS += $(VISION_APPS_PATH)/applibs

ifeq ($(TARGET_OS),LINUX)
IDIRS       += $(VISION_APPS_PATH)/apps/basic_demos/app_tirtos/tirtos_linux/mpu1
IDIRS       += $(VISION_APPS_PATH)/apps/basic_demos/app_tirtos/tirtos_linux
endif
ifeq ($(TARGET_OS),QNX)
IDIRS       += $(VISION_APPS_PATH)/apps/basic_demos/app_tirtos/tirtos_qnx/mpu1
IDIRS       += $(VISION_APPS_PATH)/apps/basic_demos/app_tirtos/tirtos_qnx
endif
IDIRS       += $(VISION_APPS_PATH)/apps/basic_demos/app_tirtos/common

# These rpath-link linker options are to provide directories for
# secondary *.so file lookup
ifeq ($(TARGET_OS),LINUX)
$(_MODULE)_LOPT += -rpath-link=$(LINUX_FS_PATH)/usr/lib
$(_MODULE)_LOPT += -rpath-link=$(LINUX_FS_PATH)/lib
$(_MODULE)_LOPT += -rpath-link=$(LINUX_FS_PATH)/usr/lib/python3.8/site-packages/dlr
endif
ifeq ($(TARGET_OS),QNX)
$(_MODULE)_LOPT += -rpath-link=$(QNX_TARGET)/usr/lib
$(_MODULE)_LOPT += -rpath-link=$(QNX_TARGET)/lib
endif

CFLAGS+=-Wno-format-truncation


# This section is for apps to link against tivision_apps library instead of static libs
ifeq ($(LINK_SHARED_OBJ)$(TARGETTYPE),yesexe)

$(info $(TARGET) links against libtivision_apps.so)

SHARED_LIBS += tivision_apps
$(_MODULE)_LOPT += -rpath=/usr/lib

IMAGING_LIBS =
VISION_APPS_SRV_LIBS  =
VISION_APPS_KERNELS_LIBS  =
VISION_APPS_MODULES_LIBS  =
TEST_LIBS =


# This section is for apps to link against static libs instead of tivision_apps library
# Also used to create tivision_apps library (so we can maintain lib list in one place
else   # ifeq ($(LINK_SHARED_OBJ),yes)

LDIRS       += $(VISION_APPS_PATH)/out/J7/$(TARGET_CPU)/$(TARGET_OS)/$(TARGET_BUILD)
LDIRS       += $(TIOVX_PATH)/lib/J7/$(TARGET_CPU)/$(TARGET_OS)/$(TARGET_BUILD)
LDIRS       += $(IMAGING_PATH)/lib/J7/$(TARGET_CPU)/$(TARGET_OS)/$(TARGET_BUILD)
LDIRS       += $(ETHFW_PATH)/lib/J7/$(TARGET_CPU)/$(TARGET_OS)/$(TARGET_BUILD)
ifeq ($(TARGET_OS), LINUX)
LDIRS       += $(LINUX_FS_PATH)/usr/lib
endif
ifeq ($(TARGET_OS), QNX)
LDIRS       += $(QNX_HOST)/usr/lib
LDIRS       += $(PDK_QNX_PATH)/packages/ti/csl/lib/j721e/qnx_a72/release
LDIRS       += $(PDK_QNX_PATH)/packages/ti/osal/lib/qnx/j721e/qnx_a72/release
LDIRS       += $(PDK_QNX_PATH)/packages/ti/drv/sciclient/lib/j721e/qnx_mpu1_0/release
LDIRS       += $(PDK_QNX_PATH)/packages/ti/drv/udma/lib/j721e/qnx_mpu1_0/release
LDIRS       += $(PSDK_QNX_PATH)/qnx/sharedmemallocator/usr/aarch64/a.le
LDIRS       += $(PSDK_QNX_PATH)/qnx/resmgr/ipc_qnx_rsmgr/usr/aarch64/a.le/
LDIRS       += $(PSDK_QNX_PATH)/qnx/resmgr/udma_qnx_rsmgr/usr/aarch64/a.le/
endif

TIOVX_LIBS  =
TIOVX_LIBS += vx_vxu vx_framework
TIOVX_LIBS += vx_kernels_host_utils vx_kernels_target_utils
ifeq ($(TARGET_OS), LINUX)
TIOVX_LIBS += vx_platform_psdk_j7_linux
endif
ifeq ($(TARGET_OS), QNX)
TIOVX_LIBS += vx_platform_psdk_j7_qnx
endif
TIOVX_LIBS += vx_kernels_openvx_core
TIOVX_LIBS += vx_utils
TIOVX_LIBS += vx_kernels_hwa vx_kernels_tidl
TIOVX_LIBS += vx_tutorial

IMAGING_LIBS  = vx_kernels_imaging
IMAGING_LIBS += app_utils_itt_server
IMAGING_LIBS += app_utils_network_api
IMAGING_LIBS += app_utils_iss

VISION_APPS_UTILS_LIBS  =
VISION_APPS_UTILS_LIBS += app_utils_draw2d
VISION_APPS_UTILS_LIBS += app_utils_mem
VISION_APPS_UTILS_LIBS += app_utils_ipc
VISION_APPS_UTILS_LIBS += app_utils_console_io
VISION_APPS_UTILS_LIBS += app_utils_remote_service
VISION_APPS_UTILS_LIBS += app_utils_perf_stats
VISION_APPS_UTILS_LIBS += app_utils_grpx

VISION_APPS_SRV_LIBS  =
VISION_APPS_SRV_LIBS  += vx_kernels_sample vx_target_kernels_sample_a72
VISION_APPS_SRV_LIBS  += vx_kernels_srv vx_target_kernels_srv_gpu
VISION_APPS_SRV_LIBS  += vx_applib_srv_bowl_lut_gen
VISION_APPS_SRV_LIBS  += vx_applib_srv_calibration
VISION_APPS_SRV_LIBS  += vx_srv_render_utils
VISION_APPS_SRV_LIBS  += vx_srv_render_utils_tools
VISION_APPS_SRV_LIBS  += app_utils_opengl

VISION_APPS_KERNELS_LIBS  =
VISION_APPS_KERNELS_LIBS += vx_kernels_img_proc
VISION_APPS_KERNELS_LIBS += vx_target_kernels_img_proc_a72
VISION_APPS_KERNELS_LIBS += vx_kernels_fileio
VISION_APPS_KERNELS_LIBS += vx_target_kernels_fileio

VISION_APPS_MODULES_LIBS  = 
VISION_APPS_MODULES_LIBS += vx_app_modules

TEST_LIBS =
TEST_LIBS += vx_tiovx_tests vx_conformance_tests vx_conformance_engine vx_conformance_tests_testmodule
TEST_LIBS += vx_kernels_hwa_tests vx_tiovx_tidl_tests
TEST_LIBS += vx_kernels_test_kernels_tests vx_kernels_test_kernels
TEST_LIBS += vx_target_kernels_source_sink
TEST_LIBS += vx_kernels_srv_tests
TEST_LIBS += vx_applib_tests

STATIC_LIBS += $(TIOVX_LIBS)
STATIC_LIBS += $(VISION_APPS_UTILS_LIBS)
ifeq ($(TARGET_OS),LINUX)
STATIC_LIBS += app_tirtos_linux_mpu1_common
endif
ifeq ($(TARGET_OS),QNX)
STATIC_LIBS += app_tirtos_qnx_mpu1_common
endif

ifeq ($(TARGET_OS),LINUX)
SYS_SHARED_LIBS += stdc++ m rt pthread ti_rpmsg_char
endif
ifeq ($(TARGET_OS),QNX)
SYS_SHARED_LIBS +=
STATIC_LIBS += c++ sharedmemallocator tiipc-usr tiudma-usr
ADDITIONAL_STATIC_LIBS += ti.osal.aa72fg
ADDITIONAL_STATIC_LIBS += ti.csl.aa72fg
ADDITIONAL_STATIC_LIBS += udma.aa72fg
ADDITIONAL_STATIC_LIBS += udma_apputils.aa72fg
ADDITIONAL_STATIC_LIBS += sciclient.aa72fg
endif

endif  # ifeq ($(LINK_SHARED_OBJ),yes)

endif  # ifeq ($(TARGET_CPU),A72)
