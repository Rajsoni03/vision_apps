ifeq ($(BUILD_APP_TIRTOS_LINUX),yes)
ifeq ($(BUILD_CPU_MCU3_1),yes)
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)

DEFS+=CPU_mcu3_1

TARGET      := vx_app_tirtos_linux_mcu3_1
TARGETTYPE  := exe
CSOURCES    := main.c

ifeq ($(RTOS),SYSBIOS)
	XDC_BLD_FILE = $($(_MODULE)_SDIR)/../../bios_cfg/config_r5f.bld
	XDC_IDIRS    = $($(_MODULE)_SDIR)/../../bios_cfg/
	XDC_CFG_FILE = $($(_MODULE)_SDIR)/mcu3_1.cfg
	XDC_PLATFORM = "ti.platforms.cortexR:J7ES_MAIN"
	LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker_mem_map.cmd
	LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker.cmd
endif
ifeq ($(RTOS),FREERTOS)
	CSOURCES += $(SOC)_mcu3_1_mpu_cfg.c
	LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/$(SOC)_mcu3_1_freertos.lds
endif

IDIRS+=$(VISION_APPS_PATH)/apps/basic_demos/app_tirtos/tirtos_linux

LDIRS += $(PDK_PATH)/packages/ti/drv/ipc/lib/$(SOC)/mcu3_1/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/udma/lib/$(SOC)/mcu3_1/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/sciclient/lib/$(SOC)/mcu3_1/$(TARGET_BUILD)/

ifeq ($(RTOS),SYSBIOS)
	LDIRS += $(PDK_PATH)/packages/ti/osal/lib/tirtos/$(SOC)/r5f/$(TARGET_BUILD)/
endif
ifeq ($(RTOS),FREERTOS)
	LDIRS += $(PDK_PATH)/packages/ti/kernel/lib/$(SOC)/mcu3_1/$(TARGET_BUILD)/
	LDIRS += $(PDK_PATH)/packages/ti/osal/lib/freertos/$(SOC)/r5f/$(TARGET_BUILD)/
endif

include $($(_MODULE)_SDIR)/../../concerto_r5f_inc.mak

# CPU instance specific libraries
STATIC_LIBS += app_tirtos_common_mcu3_1
STATIC_LIBS += app_tirtos_linux
STATIC_LIBS += app_utils_sciclient

ifeq ($(RTOS),FREERTOS)
	ADDITIONAL_STATIC_LIBS += ti.csl.init.aer5f
	ADDITIONAL_STATIC_LIBS += ti.kernel.freertos.aer5f
endif

ADDITIONAL_STATIC_LIBS += sciclient.aer5f

DEFS        += $(RTOS)

include $(FINALE)

endif
endif
endif
