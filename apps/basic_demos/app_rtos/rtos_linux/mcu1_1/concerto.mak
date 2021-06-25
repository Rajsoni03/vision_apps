ifeq ($(BUILD_APP_RTOS_LINUX),yes)
ifeq ($(BUILD_CPU_MCU1_1),yes)
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)

DEFS+=CPU_mcu1_1

TARGET      := vx_app_rtos_linux_mcu1_1
TARGETTYPE  := exe
CSOURCES    := main.c

ifeq ($(RTOS),SYSBIOS)
	XDC_BLD_FILE = $($(_MODULE)_SDIR)/../../bios_cfg/config_r5f.bld
	XDC_IDIRS    = $($(_MODULE)_SDIR)/../../bios_cfg/
	XDC_CFG_FILE = $($(_MODULE)_SDIR)/mcu1_1.cfg
	XDC_PLATFORM = "ti.platforms.cortexR:J7ES_MCU"
	LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker_mem_map.cmd
	LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker.cmd
endif
ifeq ($(RTOS),FREERTOS)
	CSOURCES += ../../common/mpu_cfg/$(SOC)_mpu_cfg.c
	LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/$(SOC)_linker_mem_map_freertos.cmd
	LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/$(SOC)_linker_freertos.cmd
endif

ifeq ($(RTOS),FREERTOS)
	LDIRS += $(PDK_PATH)/packages/ti/kernel/lib/$(SOC)/mcu1_1/$(TARGET_BUILD)/
endif

DEFS        += $(RTOS)

include $(FINALE)

endif
endif
endif
