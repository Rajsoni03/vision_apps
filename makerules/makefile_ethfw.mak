#
# Utility makefile to build ethfw libraries
#
# Edit this file to suit your specific build needs
#

ethfw: remote_device
ifeq ($(BUILD_TARGET_MODE),yes)
	$(MAKE) -C ${REMOTE_DEVICE_PATH} cp_to_lib BUILD_SOC_LIST=J721E PROFILE=$(PROFILE) -s
	$(MAKE) -C $(ETHFW_PATH) ethfw ethfw_callbacks eth_intervlan lib_remoteswitchcfg_server BUILD_CPU_MCU2_1=no BUILD_SOC_LIST=J721E PROFILE=$(PROFILE) -s
endif

ethfw_clean:
ifeq ($(BUILD_TARGET_MODE),yes)
	$(MAKE) -C ${REMOTE_DEVICE_PATH} clean -s
	$(MAKE) -C $(ETHFW_PATH) clean -s
endif

ethfw_scrub:
ifeq ($(BUILD_TARGET_MODE),yes)
	rm -rf $(REMOTE_DEVICE_PATH)/out
	rm -rf $(ETHFW_PATH)/out
endif

.PHONY: ethfw ethfw_clean ethfw_scrub
