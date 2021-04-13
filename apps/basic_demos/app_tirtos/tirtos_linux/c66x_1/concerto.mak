ifeq ($(BUILD_APP_TIRTOS_LINUX),yes)
ifeq ($(BUILD_CPU_C6x_1),yes)
ifeq ($(TARGET_CPU),C66)

include $(PRELUDE)

DEFS+=CPU_c6x_1

TARGET      := vx_app_tirtos_linux_c6x_1
TARGETTYPE  := exe
CSOURCES    := $(call all-c-files)

XDC_BLD_FILE = $($(_MODULE)_SDIR)/../../bios_cfg/config_c66.bld
XDC_IDIRS    = $($(_MODULE)_SDIR)/../../bios_cfg/
XDC_CFG_FILE = $($(_MODULE)_SDIR)/c66x_1.cfg
XDC_PLATFORM = "ti.platforms.c6x:J7ES"

LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker_mem_map.cmd
LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker.cmd

IDIRS+=$(VISION_APPS_PATH)/apps/basic_demos/app_tirtos/tirtos_linux

LDIRS += $(PDK_PATH)/packages/ti/drv/ipc/lib/j721e/c66xdsp_1/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/sciclient/lib/j721e/c66xdsp_1/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/udma/lib/j721e/c66xdsp_1/$(TARGET_BUILD)/

include $($(_MODULE)_SDIR)/../../concerto_c6x_inc.mak

# CPU instance specific libraries
STATIC_LIBS += app_tirtos_common_c6x_1
STATIC_LIBS += app_tirtos_linux

DEFS+=SOC_J721E

include $(FINALE)

endif
endif
endif
