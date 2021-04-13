ifeq ($(BUILD_APP_TIRTOS_QNX),yes)
ifeq ($(BUILD_CPU_MCU3_1),yes)
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)

DEFS+=CPU_mcu3_1

TARGET      := vx_app_tirtos_qnx_mcu3_1
TARGETTYPE  := exe
CSOURCES    := $(call all-c-files)

XDC_BLD_FILE = $($(_MODULE)_SDIR)/../../bios_cfg/config_r5f.bld
XDC_IDIRS    = $($(_MODULE)_SDIR)/../../bios_cfg/
XDC_CFG_FILE = $($(_MODULE)_SDIR)/mcu3_1.cfg
XDC_PLATFORM = "ti.platforms.cortexR:J7ES_MAIN"

IDIRS+=$(VISION_APPS_PATH)/apps/basic_demos/app_tirtos/tirtos_qnx

LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker_mem_map.cmd
LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker.cmd

LDIRS += $(PDK_PATH)/packages/ti/drv/ipc/lib/j721e/mcu3_1/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/udma/lib/j721e/mcu3_1/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/sciclient/lib/j721e/mcu3_1/$(TARGET_BUILD)/

include $($(_MODULE)_SDIR)/../../concerto_r5f_inc.mak

# CPU instance specific libraries
STATIC_LIBS += app_tirtos_common_mcu3_1
STATIC_LIBS += app_tirtos_qnx
STATIC_LIBS += app_utils_sciclient

ADDITIONAL_STATIC_LIBS += sciclient.aer5f

include $(FINALE)

endif
endif
endif
