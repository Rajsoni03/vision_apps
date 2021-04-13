ifeq ($(BUILD_APP_TIRTOS_LINUX),yes)
ifeq ($(BUILD_CPU_MCU1_1),yes)
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)

DEFS+=CPU_mcu1_1

TARGET      := vx_app_tirtos_linux_mcu1_1
TARGETTYPE  := exe
CSOURCES    := $(call all-c-files)

XDC_BLD_FILE = $($(_MODULE)_SDIR)/../../bios_cfg/config_r5f.bld
XDC_IDIRS    = $($(_MODULE)_SDIR)/../../bios_cfg/
XDC_CFG_FILE = $($(_MODULE)_SDIR)/mcu1_1.cfg
XDC_PLATFORM = "ti.platforms.cortexR:J7ES_MCU"

LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker_mem_map.cmd
LINKER_CMD_FILES +=  $($(_MODULE)_SDIR)/linker.cmd

SYS_STATIC_LIBS += rtsv7R4_T_le_v3D16_eabi

include $(FINALE)

endif
endif
endif
