ifeq ($(BUILD_APP_TIRTOS_LINUX),yes)
ifeq ($(BUILD_CPU_MCU1_0),yes)
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)

DEFS+=CPU_mcu1_0
DEFS+=BUILD_MCU1_0
DEFS+=BUILD_MCU

# This enables ARM Thumb mode which reduces firmware size and enables faster boot
COPT +=--code_state=16

TARGET      := vx_app_tirtos_linux_mcu1_0
TARGETTYPE  := exe
CSOURCES    := $(call all-c-files)
ASSEMBLY    := mcuCopyVecs2Exc.asm

XDC_BLD_FILE = $($(_MODULE)_SDIR)/../../bios_cfg/config_r5f.bld
XDC_IDIRS    = $($(_MODULE)_SDIR)/../../bios_cfg/
XDC_CFG_FILE = $($(_MODULE)_SDIR)/mcu1_0.cfg
XDC_PLATFORM = "ti.platforms.cortexR:J7ES_MCU"

IDIRS+=$(VISION_APPS_PATH)/apps/basic_demos/app_tirtos/tirtos_linux

LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker_mem_map.cmd
LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker.cmd

SYS_STATIC_LIBS += rtsv7R4_T_le_v3D16_eabi

LDIRS += $(PDK_PATH)/packages/ti/drv/ipc/lib/j721e/mcu1_0/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/udma/lib/j721e/mcu1_0/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/sciclient/lib/j721e/mcu1_0/$(TARGET_BUILD)/

include $($(_MODULE)_SDIR)/../../concerto_r5f_inc.mak

# CPU instance specific libraries
STATIC_LIBS += app_tirtos_common_mcu1_0
STATIC_LIBS += app_tirtos_linux
STATIC_LIBS += app_utils_sciserver

ADDITIONAL_STATIC_LIBS += sciclient_direct.aer5f
ADDITIONAL_STATIC_LIBS += sciserver_tirtos.aer5f
ADDITIONAL_STATIC_LIBS += rm_pm_hal.aer5f

include $(FINALE)

endif
endif
endif
