
PSDK_PATH = $(abspath ..)
TIOVX_PATH = $(PSDK_PATH)/tiovx
MCUSW_PATH=$(PSDK_PATH)/mcusw

# paths for components shared between tiovx and vision_apps are specified in below
# file in tiovx, ex, bios, tidl, pdk, cgtools, ...
include $(TIOVX_PATH)/psdkra_tools_path.mak

LINUX_FS_PATH ?= $(PSDK_PATH)/targetfs/
LINUX_FS_BOOT_PATH ?= $(PSDK_PATH)/bootfs/
LINUX_SD_FS_ROOT_PATH ?= /media/$(USER)/rootfs
LINUX_SD_FS_BOOT_PATH ?= /media/$(USER)/BOOT

TIOVX_CUSTOM_KERNEL_PATH ?= $(CUSTOM_KERNEL_PATH)
PTK_PATH           ?= $(PSDK_PATH)/perception
REMOTE_DEVICE_PATH ?= $(PSDK_PATH)/remote_device
TIADALG_PATH       ?= $(PSDK_PATH)/tiadalg
GLM_PATH           ?= $(PSDK_PATH)/glm
NDK_PATH           ?= $(PSDK_PATH)/ndk_3_80_00_19
NS_PATH            ?= $(PSDK_PATH)/ns_2_80_00_17
ETHFW_PATH         ?= $(PSDK_PATH)/ethfw

BUILD_OS ?= Linux

# QNX Paths
export PSDK_QNX_PATH ?= $(PSDK_PATH)/psdkqa
ifeq ($(SOC),j721e)
  export PSDK_LINUX_PATH ?= /home/$(USER)/ti-processor-sdk-linux-j7-evm-08_01_00_03
endif
export QNX_SDP_VERSION ?= 710
ifeq ($(QNX_SDP_VERSION),700)
  export QNX_BASE ?= /home/$(USER)/qnx700
  export QNX_CROSS_COMPILER_TOOL ?= aarch64-unknown-nto-qnx7.0.0-
else
  export QNX_BASE ?= /home/$(USER)/qnx710
  export QNX_CROSS_COMPILER_TOOL ?= aarch64-unknown-nto-qnx7.1.0-
  # Adding this path for QNX SDP 7.1 which has a need to set the path
  # variable for the g++ tool to properly invloke the ld tool
  PATH := $(QNX_BASE)/host/linux/x86_64/usr/bin:$(PATH)
endif
export QNX_HOST ?= $(QNX_BASE)/host/linux/x86_64
export QNX_TARGET ?= $(QNX_BASE)/target/qnx7
export GCC_QNX_ROOT ?= $(QNX_HOST)/usr/bin
export GCC_QNX_ARM_ROOT ?= $(QNX_HOST)/usr/bin
export GCC_QNX_ARM ?= $(QNX_HOST)/usr/bin
QNX_FS_PATH ?= $(PSDK_QNX_PATH)/bootfs/
QNX_SD_FS_ROOT_PATH ?= /media/$(USER)/rootfs
QNX_SD_FS_BOOT_PATH ?= /media/$(USER)/boot

ifeq ($(BUILD_OS),Linux)
GCC_LINUX_ROOT ?= /usr/
endif
