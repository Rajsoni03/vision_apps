ifeq ($(TARGET_CPU),A72)

IDIRS=$($(_MODULE)_SDIR)/../common

LDIRS += $(PDK_PATH)/packages/ti/osal/lib/tirtos/$(SOC)/a72/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/csl/lib/$(SOC)/a72/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/board/lib/$(BUILD_PDK_BOARD)/a72/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/uart/lib/$(SOC)/a72/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/mmcsd/lib/$(SOC)/a72/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/fs/fatfs/lib/a72/$(TARGET_BUILD)/
LDIRS += $(PDK_PATH)/packages/ti/drv/i2c/lib/$(SOC)/a72/$(TARGET_BUILD)/

STATIC_LIBS += app_utils_mem
STATIC_LIBS += app_utils_console_io
STATIC_LIBS += app_utils_mmc_sd
STATIC_LIBS += app_utils_perf_stats
STATIC_LIBS += app_utils_remote_service
STATIC_LIBS += app_utils_hwa

SYS_STATIC_LIBS += stdc++ gcc m c nosys

ADDITIONAL_STATIC_LIBS += ti.osal.aa72fg
ADDITIONAL_STATIC_LIBS += ti.csl.aa72fg
ADDITIONAL_STATIC_LIBS += ti.board.aa72fg
ADDITIONAL_STATIC_LIBS += ti.drv.uart.aa72fg
ADDITIONAL_STATIC_LIBS += ti.drv.mmcsd.aa72fg
ADDITIONAL_STATIC_LIBS += ti.fs.fatfs.aa72fg
ADDITIONAL_STATIC_LIBS += ti.drv.i2c.aa72fg

endif
