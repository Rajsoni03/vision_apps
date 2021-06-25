ifeq ($(TARGET_CPU),$(filter $(TARGET_CPU), x86_64))

include $(PRELUDE)

TARGET      := app_utils_perf_stats
TARGETTYPE  := library

CSOURCES += app_perf_stats_api_x86.c

include $(FINALE)

endif

ifeq ($(TARGET_PLATFORM),J7)

include $(PRELUDE)
TARGET      := app_utils_perf_stats
TARGETTYPE  := library

ifeq ($(TARGET_OS),SYSBIOS)
CSOURCES    := app_perf_stats_tirtos.c
endif

ifeq ($(TARGET_OS),FREERTOS)
CSOURCES    := app_perf_stats_freertos.c
endif

ifeq ($(TARGET_OS),$(filter $(TARGET_OS), LINUX QNX))
CSOURCES    := app_perf_stats_linux.c
endif

CSOURCES += app_perf_stats_api.c

ifeq ($(SOC),j721e)
DEFS=SOC_J721E
endif

include $(FINALE)


endif

