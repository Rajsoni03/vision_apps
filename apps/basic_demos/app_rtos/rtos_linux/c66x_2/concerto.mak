ifeq ($(BUILD_APP_RTOS_LINUX),yes)
ifeq ($(BUILD_CPU_C6x_2),yes)
ifeq ($(TARGET_CPU),C66)

include $(PRELUDE)

DEFS+=CPU_c6x_2

TARGET      := vx_app_rtos_linux_c6x_2
TARGETTYPE  := exe
CSOURCES    := $(call all-c-files)

XDC_BLD_FILE = $($(_MODULE)_SDIR)/../../bios_cfg/config_c66.bld
XDC_IDIRS    = $($(_MODULE)_SDIR)/../../bios_cfg/
XDC_CFG_FILE = $($(_MODULE)_SDIR)/c66x_2.cfg
XDC_PLATFORM = "ti.platforms.c6x:J7ES"

IDIRS+=$(VISION_APPS_PATH)/apps/basic_demos/app_rtos/rtos_linux

LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker_mem_map.cmd
LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker.cmd

LDIRS += $(PDK_PATH)/packages/ti/drv/ipc/lib/$(SOC)/c66xdsp_2/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/sciclient/lib/$(SOC)/c66xdsp_2/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/udma/lib/$(SOC)/c66xdsp_2/$(TARGET_BUILD)/

include $($(_MODULE)_SDIR)/../../concerto_c6x_inc.mak

# CPU instance specific libraries
STATIC_LIBS += app_rtos_common_c6x_2
STATIC_LIBS += app_rtos_linux

ifeq ($(SOC),j721e)
DEFS+=SOC_J721E
endif

include $(FINALE)

endif
endif
endif
