ifeq ($(BUILD_APP_RTOS_QNX),yes)
ifeq ($(BUILD_CPU_MCU1_1),yes)
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)

DEFS+=CPU_mcu1_1

TARGET      := vx_app_rtos_qnx_mcu1_1
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
	CSOURCES += $(SOC)_mcu1_1_mpu_cfg.c
	LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/$(SOC)_mcu1_1_freertos.lds
endif


SYS_STATIC_LIBS += rtsv7R4_T_le_v3D16_eabi

ifeq ($(RTOS),SYSBIOS)
	LDIRS += $(PDK_PATH)/packages/ti/osal/lib/tirtos/$(SOC)/r5f/$(TARGET_BUILD)/
endif
ifeq ($(RTOS),FREERTOS)
	LDIRS += $(PDK_PATH)/packages/ti/kernel/lib/$(SOC)/mcu1_1/$(TARGET_BUILD)/
	LDIRS += $(PDK_PATH)/packages/ti/osal/lib/freertos/$(SOC)/r5f/$(TARGET_BUILD)/
endif

ifeq ($(RTOS),FREERTOS)
	ADDITIONAL_STATIC_LIBS += ti.csl.init.aer5f
	ADDITIONAL_STATIC_LIBS += ti.kernel.freertos.aer5f
endif

DEFS        += $(RTOS)

include $(FINALE)

endif
endif
endif
