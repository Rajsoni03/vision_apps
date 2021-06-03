ifeq ($(TARGET_PLATFORM),J7)
ifeq ($(TARGET_OS),SYSBIOS)
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)

TARGET      := app_utils_hwa
TARGETTYPE  := library

CSOURCES    := app_hwa.c

ifeq ($(SOC),j721e)
DEFS+=SOC_J721E
endif

IDIRS       += $(VIDEO_CODEC_PATH)/ti-img-encode-decode/timmlib/include

include $(FINALE)

endif
endif
endif
