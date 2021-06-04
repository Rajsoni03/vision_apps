ifeq ($(BUILD_APP_RTOS_FILEIO),yes)

########################################################################

ifeq ($(BUILD_CPU_MPU1),yes)
ifeq ($(TARGET_CPU),A72)
ifeq ($(TARGET_OS),SYSBIOS)

# CPU_ID must be set before include $(PRELUDE)
CPU_ID=mpu1

_MODULE=$(CPU_ID)
include $(PRELUDE)

TARGET      := app_rtos_fileio_common_$(CPU_ID)
TARGETTYPE  := library
CSOURCES    := $(call all-c-files)

DEFS+=APP_CFG_FILE=\"app_cfg_$(CPU_ID).h\"
DEFS+=CPU_$(CPU_ID)


include $(FINALE)

endif
endif
endif

########################################################################

endif
