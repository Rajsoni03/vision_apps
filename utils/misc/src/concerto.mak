ifeq ($(TARGET_PLATFORM),J7)
ifeq ($(TARGET_OS),SYSBIOS)

include $(PRELUDE)

TARGET      := app_utils_misc
TARGETTYPE  := library

ifeq ($(TARGET_CPU),C71)
ASSEMBLY += app_c7x_init_asm.asm
CSOURCES += app_c7x_init.c
endif

ifeq ($(TARGET_CPU),R5F)
CSOURCES += app_r5f_init.c

ifeq ($(BUILD_PDK_BOARD), j721e_evm)
CSOURCES += app_pinmux.c
DEFS+=j721e_evm
endif

endif

ifeq ($(SOC),j721e)
DEFS+=SOC_J721E
endif

include $(FINALE)

endif
endif
