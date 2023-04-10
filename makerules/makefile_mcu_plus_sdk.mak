#
# Utility makefile to build MCU_PLUS_SDK libaries and related components
#
# Edit this file to suit your specific build needs
#

include $(MCU_PLUS_SDK_PATH)/imports.mak

mcu_plus_sdk_build:
	$(MAKE) -C $(MCU_PLUS_SDK_PATH) libs DEVICE=$(BUILD_MCU_PLUS_SDK_DEVICE) PROFILE=release -s
	$(MAKE) -C $(MCU_PLUS_SDK_PATH) libs DEVICE=$(BUILD_MCU_PLUS_SDK_DEVICE) PROFILE=debug -s
	@echo Generating SysConfig files for vision_apps
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(MCU_PLUS_SDK_PATH)/.metadata/product.json --context r5fss0-0 --part Default --package AMB --output platform/$(SOC)/rtos/mcu1_0/generated platform/$(SOC)/rtos/mcu1_0/example.syscfg

mcu_plus_sdk: mcu_plus_sdk_emu
ifeq ($(BUILD_TARGET_MODE),yes)
	$(MAKE) -C $(MCU_PLUS_SDK_PATH) libs DEVICE=$(BUILD_MCU_PLUS_SDK_DEVICE) PROFILE=release -s
	$(MAKE) -C $(MCU_PLUS_SDK_PATH) libs DEVICE=$(BUILD_MCU_PLUS_SDK_DEVICE) PROFILE=debug -s
	@echo Generating SysConfig files for vision_apps
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(MCU_PLUS_SDK_PATH)/.metadata/product.json --context r5fss0-0 --part Default --package AMB --output platform/$(SOC)/rtos/mcu1_0/generated platform/$(SOC)/rtos/mcu1_0/example.syscfg
endif

mcu_plus_sdk_emu:
ifeq ($(BUILD_EMULATION_MODE),yes)
	$(MAKE) -C $(MCU_PLUS_SDK_PATH) -f makefile.am62ax host-emu DEVICE=$(BUILD_MCU_PLUS_SDK_DEVICE) PROFILE=release -s
	$(MAKE) -C $(MCU_PLUS_SDK_PATH) -f makefile.am62ax host-emu DEVICE=$(BUILD_MCU_PLUS_SDK_DEVICE) PROFILE=debug -s
endif

mcu_plus_sdk_clean:
	$(MAKE) -C $(MCU_PLUS_SDK_PATH) libs-clean DEVICE=$(BUILD_MCU_PLUS_SDK_DEVICE) PROFILE=release -s
	$(MAKE) -C $(MCU_PLUS_SDK_PATH) libs-clean DEVICE=$(BUILD_MCU_PLUS_SDK_DEVICE) PROFILE=debug -s	

mcu_plus_sdk_scrub:
	$(MAKE) -C $(MCU_PLUS_SDK_PATH) libs-scrub DEVICE=$(BUILD_MCU_PLUS_SDK_DEVICE) -s

.PHONY: mcu_plus_sdk mcu_plus_sdk_clean mcu_plus_sdk_build
