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

mmalib_clean mmalib_scrub:
	rm -rf $(MMALIB_PATH)/out

tidl_lib:
ifeq ($(BUILD_EMULATION_MODE),yes)
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH) tidl_lib DSP_TOOLS=$(CGT7X_ROOT) TARGET_PLATFORM=PC TARGET_BUILD=$(current_profile); \
    )
endif
ifeq ($(BUILD_TARGET_MODE),yes)
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH) tidl_lib DSP_TOOLS=$(CGT7X_ROOT) TARGET_PLATFORM=TI_DEVICE TARGET_BUILD=$(current_profile); \
    )
endif

tidl_tiovx_kernels:
ifeq ($(BUILD_EMULATION_MODE),yes)
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH)/arm-tidl tidl_tiovx_kernels TARGET_PLATFORM=PC TARGET_BUILD=$(current_profile); \
    )
endif
ifeq ($(BUILD_TARGET_MODE),yes)
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH)/arm-tidl tidl_tiovx_kernels TARGET_BUILD=$(current_profile); \
    )
endif

tidl_rt:
ifeq ($(BUILD_EMULATION_MODE),yes)
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH)/arm-tidl all TARGET_PLATFORM=PC TARGET_BUILD=$(current_profile); \
    )
endif
ifeq ($(BUILD_TARGET_MODE),yes)
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH)/arm-tidl all TARGET_BUILD=$(current_profile); \
    )
endif

tidl_lib_scrub tidl_lib_clean:
ifeq ($(BUILD_EMULATION_MODE),yes)
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH) tidl_lib_clean DSP_TOOLS=$(CGT7X_ROOT) TARGET_PLATFORM=PC TARGET_BUILD=$(current_profile); \
    )
endif
ifeq ($(BUILD_TARGET_MODE),yes)
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH) tidl_lib_clean DSP_TOOLS=$(CGT7X_ROOT) TARGET_PLATFORM=TI_DEVICE TARGET_BUILD=$(current_profile); \
    )
endif

tidl_tiovx_kernels_scrub tidl_tiovx_kernels_clean:
ifeq ($(BUILD_EMULATION_MODE),yes)
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH)/arm-tidl tidl_tiovx_kernels_clean TARGET_BUILD=$(current_profile); \
    )
endif
ifeq ($(BUILD_TARGET_MODE),yes)
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH)/arm-tidl tidl_tiovx_kernels_clean TARGET_BUILD=$(current_profile); \
    )
endif

tidl_rt_clean tidl_rt_scrub:
ifeq ($(BUILD_EMULATION_MODE),yes)
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH)/arm-tidl clean TARGET_PLATFORM=PC TARGET_BUILD=$(current_profile); \
    )
endif
ifeq ($(BUILD_TARGET_MODE),yes)
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH)/arm-tidl clean TARGET_BUILD=$(current_profile); \
    )
endif

.PHONY: tidl_lib tidl_lib_clean tidl_lib_scrub tidl_tiovx_kernels tidl_tiovx_kernels_clean tidl_tiovx_kernels_scrub tidl_rt tidl_rt_clean tidl_rt_scrub mmalib mmalib_clean mmalib_scrub
