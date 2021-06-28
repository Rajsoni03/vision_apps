ifeq ($(BUILD_APP_RTOS_QNX),yes)
ifeq ($(BUILD_CPU_MCU2_0),yes)
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)

DEFS+=CPU_mcu2_0

TARGET      := vx_app_rtos_qnx_mcu2_0
TARGETTYPE  := exe
CSOURCES    := main.c

ifeq ($(RTOS),SYSBIOS)
	XDC_BLD_FILE = $($(_MODULE)_SDIR)/../../bios_cfg/config_r5f.bld
	XDC_IDIRS    = $($(_MODULE)_SDIR)/../../bios_cfg/;$(NDK_PATH)/packages;$(BIOS_PATH)/packages/ti/posix/ccs
	XDC_CFG_FILE = $($(_MODULE)_SDIR)/mcu2_0.cfg
	XDC_PLATFORM = "ti.platforms.cortexR:J7ES_MAIN"
	LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker_mem_map.cmd
	LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker.cmd
endif
ifeq ($(RTOS),FREERTOS)
	CSOURCES += ../../common/mpu_cfg/$(SOC)_mpu_cfg.c
	LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/$(SOC)_linker_mem_map_freertos.cmd
	LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/$(SOC)_linker_freertos.cmd
endif

IDIRS+=$(VISION_APPS_PATH)/apps/basic_demos/app_rtos/rtos_qnx
IDIRS+=$(NDK_PATH)/packages
IDIRS+=$(REMOTE_DEVICE_PATH)
IDIRS+=$(ETHFW_PATH)

LDIRS += $(PDK_PATH)/packages/ti/drv/ipc/lib/$(SOC)/mcu2_0/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/udma/lib/$(SOC)/mcu2_0/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/sciclient/lib/$(SOC)/mcu2_0/$(TARGET_BUILD)/

LDIRS += $(PDK_PATH)/packages/ti/drv/enet/lib/$(SOC)/mcu2_0/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/enet/lib/$(SOC)_evm/mcu2_0/$(TARGET_BUILD)/

LDIRS += $(PDK_PATH)/packages/ti/drv/csirx/lib/$(SOC)/mcu2_0/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/csitx/lib/$(SOC)/mcu2_0/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/dss/lib/$(SOC)/mcu2_0/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/vhwa/lib/$(SOC)/mcu2_0/$(TARGET_BUILD)/

LDIRS += $(ETHFW_PATH)/out/J721E/R5Ft/$(TARGET_OS)/$(TARGET_BUILD)
LDIRS += $(REMOTE_DEVICE_PATH)/lib/J721E/$(TARGET_CPU)/$(TARGET_OS)/$(TARGET_BUILD)

ifeq ($(RTOS),FREERTOS)
    LDIRS += $(PDK_PATH)/packages/ti/kernel/lib/$(SOC)/mcu2_0/$(TARGET_BUILD)/
    LDIRS += $(PDK_PATH)/packages/ti/transport/lwip/lwip-stack/lib/freertos/$(SOC)/r5f/$(TARGET_BUILD)/
    LDIRS += $(PDK_PATH)/packages/ti/transport/lwip/lwip-contrib/lib/freertos/$(SOC)/r5f/$(TARGET_BUILD)/
    LDIRS += $(PDK_PATH)/packages/ti/drv/enet/lib/freertos/j721e/r5f/$(TARGET_BUILD)/
endif

include $($(_MODULE)_SDIR)/../../concerto_r5f_inc.mak

ifeq ($(BUILD_ENABLE_ETHFW),yes)
DEFS+=ENABLE_ETHFW
endif

# CPU instance specific libraries
STATIC_LIBS += app_rtos_common_mcu2_0
STATIC_LIBS += app_rtos_qnx

STATIC_LIBS += app_utils_hwa
STATIC_LIBS += app_utils_dss
STATIC_LIBS += app_utils_pcie_queue
STATIC_LIBS += vx_target_kernels_sample_r5f
STATIC_LIBS += app_utils_sciclient
STATIC_LIBS += app_utils_ethfw

ETHFW_LIBS = ethfw
ETHFW_LIBS += ethfw_callbacks
ETHFW_LIBS += eth_intervlan
ETHFW_LIBS += lib_remoteswitchcfg_server
ifeq ($(RTOS),FREERTOS)
	ETHFW_LIBS += ethfw_lwip
endif

REMOTE_DEVICE_LIBS = lib_remote_device

SYS_STATIC_LIBS += $(ETHFW_LIBS)
SYS_STATIC_LIBS += $(REMOTE_DEVICE_LIBS)

ADDITIONAL_STATIC_LIBS += csirx.aer5f
ADDITIONAL_STATIC_LIBS += csitx.aer5f
ADDITIONAL_STATIC_LIBS += dss.aer5f
ADDITIONAL_STATIC_LIBS += vhwa.aer5f

ADDITIONAL_STATIC_LIBS += enetsoc.aer5f
ADDITIONAL_STATIC_LIBS += enet.aer5f
ADDITIONAL_STATIC_LIBS += enetphy.aer5f
ADDITIONAL_STATIC_LIBS += enet_cfgserver.aer5f
ADDITIONAL_STATIC_LIBS += pm_lib.aer5f
ADDITIONAL_STATIC_LIBS += ti.timesync.hal.aer5f
ADDITIONAL_STATIC_LIBS += ti.timesync.ptp.aer5f
ADDITIONAL_STATIC_LIBS += sciclient.aer5f

ifeq ($(RTOS),SYSBIOS)
	ADDITIONAL_STATIC_LIBS += nimuenet.aer5f
	ADDITIONAL_STATIC_LIBS += enet_example_utils_tirtos.aer5f
endif

ifeq ($(RTOS),FREERTOS)
	ADDITIONAL_STATIC_LIBS += lwipstack_freertos.aer5f
	ADDITIONAL_STATIC_LIBS += lwipcontrib_freertos.aer5f
	ADDITIONAL_STATIC_LIBS += lwipif_freertos.aer5f
	ADDITIONAL_STATIC_LIBS += enet_example_utils_freertos.aer5f
endif

DEFS        += $(RTOS)

include $(FINALE)

endif
endif
endif
