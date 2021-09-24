ifeq ($(RTOS),SYSBIOS)
	XDC_BLD_FILE = $($(_MODULE)_SDIR)/../bios_cfg/config_c66.bld
	XDC_IDIRS    = $($(_MODULE)_SDIR)/../bios_cfg/
	XDC_CFG_FILE = $($(_MODULE)_SDIR)/c66x_1.cfg
	XDC_PLATFORM = "ti.platforms.c6x:J7ES"
	LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker_mem_map.cmd
	LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker.cmd
endif
ifeq ($(RTOS),FREERTOS)
	LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/$(SOC)_linker_mem_map_freertos.cmd
	LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/$(SOC)_linker_freertos.cmd
endif

IDIRS+=$(VISION_APPS_PATH)/platform/$(SOC)/rtos

ifeq ($(RTOS),FREERTOS)
	LDIRS += $(PDK_PATH)/packages/ti/kernel/lib/$(SOC)/c66xdsp_1/$(TARGET_BUILD)/
endif

LDIRS += $(PDK_PATH)/packages/ti/drv/ipc/lib/$(SOC)/c66xdsp_1/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/sciclient/lib/$(SOC)/c66xdsp_1/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/udma/lib/$(SOC)/c66xdsp_1/$(TARGET_BUILD)/

include $($(_MODULE)_SDIR)/../concerto_c6x_inc.mak

# CPU instance specific libraries
STATIC_LIBS += app_rtos_common_c6x_1
ifeq ($(RTOS),FREERTOS)
	STATIC_LIBS += app_rtos
endif

ifeq ($(SOC),j721e)
DEFS+=SOC_J721E
endif

DEFS        += $(RTOS)
