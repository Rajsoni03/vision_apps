ifeq ($(BUILD_APP_RTOS_LINUX),yes)
ifeq ($(BUILD_CPU_MCU1_0),yes)
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)

DEFS+=CPU_mcu1_0
DEFS+=BUILD_MCU1_0
DEFS+=BUILD_MCU

# This enables ARM Thumb mode which reduces firmware size and enables faster boot
COPT +=--code_state=16

TARGET      := vx_app_rtos_linux_mcu1_0
TARGETTYPE  := exe
CSOURCES    := main.c
ASSEMBLY    := mcuCopyVecs2Exc.asm

ifeq ($(RTOS),SYSBIOS)
	XDC_BLD_FILE = $($(_MODULE)_SDIR)/../../bios_cfg/config_r5f.bld
	XDC_IDIRS    = $($(_MODULE)_SDIR)/../../bios_cfg/
	XDC_CFG_FILE = $($(_MODULE)_SDIR)/mcu1_0.cfg
	XDC_PLATFORM = "ti.platforms.cortexR:J7ES_MCU"
	LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker_mem_map.cmd
	LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker.cmd
endif
ifeq ($(RTOS),FREERTOS)
	CSOURCES += ../../common/mpu_cfg/$(SOC)_mpu_cfg.c
	LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/$(SOC)_linker_mem_map_freertos.cmd
	LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/$(SOC)_linker_freertos.cmd
endif

IDIRS+=$(VISION_APPS_PATH)/apps/basic_demos/app_rtos/rtos_linux

ifeq ($(RTOS),FREERTOS)
	LDIRS += $(PDK_PATH)/packages/ti/kernel/lib/$(SOC)/mcu1_0/$(TARGET_BUILD)/
endif

LDIRS += $(PDK_PATH)/packages/ti/drv/ipc/lib/$(SOC)/mcu1_0/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/udma/lib/$(SOC)/mcu1_0/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/sciclient/lib/$(SOC)/mcu1_0/$(TARGET_BUILD)/

include $($(_MODULE)_SDIR)/../../concerto_r5f_inc.mak

# CPU instance specific libraries
STATIC_LIBS += app_rtos_common_mcu1_0
STATIC_LIBS += app_rtos_linux
STATIC_LIBS += app_utils_sciserver

ADDITIONAL_STATIC_LIBS += sciclient_direct.aer5f
ADDITIONAL_STATIC_LIBS += sciserver_tirtos.aer5f
ADDITIONAL_STATIC_LIBS += rm_pm_hal.aer5f

DEFS        += $(RTOS)

include $(FINALE)

endif
endif
endif
