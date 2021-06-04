ifeq ($(BUILD_APP_RTOS_QNX),yes)
ifeq ($(BUILD_CPU_C7x_1),yes)
ifeq ($(TARGET_CPU),C71)

include $(PRELUDE)

DEFS+=CPU_c7x_1

TARGET      := vx_app_rtos_qnx_c7x_1
TARGETTYPE  := exe
CSOURCES    := $(call all-c-files)

XDC_BLD_FILE = $($(_MODULE)_SDIR)/../../bios_cfg/config_c71.bld
XDC_IDIRS    = $($(_MODULE)_SDIR)/../../bios_cfg/
XDC_CFG_FILE = $($(_MODULE)_SDIR)/c7x_1.cfg
XDC_PLATFORM = "ti.platforms.tms320C7x:J7ES"

IDIRS+=$(VISION_APPS_PATH)/apps/basic_demos/app_rtos/rtos_qnx

LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker_mem_map.cmd
LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker.cmd


include $($(_MODULE)_SDIR)/../../concerto_c7x_inc.mak

# CPU instance specific libraries
STATIC_LIBS += app_rtos_common_c7x_1
STATIC_LIBS += app_rtos_qnx

DEFS+=SOC_J721E

#
# Suppress this warning, 10063-D: entry-point symbol other than "_c_int00" specified
# c7x boots in secure mode and to switch to non-secure mode we need to start at a special entry point '_c_int00_secure'
# and later after switching to non-secure mode, sysbios jumps to usual entry point of _c_int00
# Hence we need to suppress this warning
CFLAGS+=--diag_suppress=10063

include $(FINALE)

endif
endif
endif
