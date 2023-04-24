#
# Utility makefile to build TIDL libaries and related components
#
# Edit this file to suit your specific build needs
#

export MMALIB_PATH

mmalib:
ifeq ($(BUILD_TARGET_MODE),yes)
	$(MAKE) -C $(MMALIB_PATH) mmalib mmalib_cn common SRC_DIR=cnn TARGET_BUILD=release
endif
ifeq ($(BUILD_EMULATION_MODE),yes)
	$(MAKE) -C $(MMALIB_PATH) mmalib mmalib_cn common SRC_DIR=cnn TARGET_CPU=x86_64 TARGET_SCPU=$(C7X_TARGET) TARGET_PLATFORM=PC TARGET_BUILD=release
endif

mmalib_clean:
	rm -rf $(MMALIB_PATH)/out

tidl:
ifeq ($(BUILD_LINUX_MPU),yes)
ifeq ($(BUILD_EMULATION_MODE),yes)
	$(foreach current_profile, $(PDK_BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH) tidl_lib PSDK_INSTALL_PATH=$(PSDK_PATH) DSP_TOOLS=$(CGT7X_ROOT) TARGET_PLATFORM=PC TARGET_SOC=$(SOC) TARGET_BUILD=$(current_profile); \
    )
	$(foreach current_profile, $(PDK_BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH) tidl_tiovx_kernels PSDK_INSTALL_PATH=$(PSDK_PATH) TARGET_PLATFORM=PC TARGET_SOC=$(SOC) TARGET_BUILD=$(current_profile); \
    )
endif
endif
ifeq ($(BUILD_TARGET_MODE),yes)
	$(foreach current_profile, $(PDK_BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH) tidl_lib PSDK_INSTALL_PATH=$(PSDK_PATH) DSP_TOOLS=$(CGT7X_ROOT) TARGET_PLATFORM=TI_DEVICE TARGET_SOC=$(SOC) TARGET_BUILD=$(current_profile); \
    )
	$(foreach current_profile, $(PDK_BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH) tidl_tiovx_kernels PSDK_INSTALL_PATH=$(PSDK_PATH) TARGET_SOC=$(SOC) TARGET_BUILD=$(current_profile); \
    )
endif

tidl_scrub tidl_clean:
ifeq ($(BUILD_LINUX_MPU),yes)
	rm -rf $(TIDL_PATH)/out
endif

.PHONY: tidl tidl_clean mmalib mmalib_clean tidl_tiovx_kernels tidl_tiovx_kernels_clean tidl_tiovx_kernels_scrub
