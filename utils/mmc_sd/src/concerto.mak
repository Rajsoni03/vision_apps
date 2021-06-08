ifeq ($(TARGET_PLATFORM),J7)
ifeq ($(TARGET_OS),$(filter $(TARGET_OS),SYSBIOS FREERTOS))
ifeq ($(TARGET_CPU),A72)

include $(PRELUDE)
TARGET      := app_utils_mmc_sd
TARGETTYPE  := library

CSOURCES    := app_mmc_sd.c app_fatfs_ffcio_gnu.c app_nosys_funcs.c

ifeq ($(SOC),j721e)
DEFS+=SOC_J721E
endif

include $(FINALE)

endif
endif
endif
