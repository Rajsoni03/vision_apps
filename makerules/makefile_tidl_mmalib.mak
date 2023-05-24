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

dsp_tidl:
ifeq ($(BUILD_LINUX_MPU),yes)
ifeq ($(BUILD_EMULATION_MODE),yes)
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH) tidl_lib DSP_TOOLS=$(CGT7X_ROOT) TARGET_PLATFORM=PC TARGET_BUILD=$(current_profile); \
    )
endif
endif
ifeq ($(BUILD_TARGET_MODE),yes)
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH) tidl_lib DSP_TOOLS=$(CGT7X_ROOT) TARGET_PLATFORM=TI_DEVICE TARGET_BUILD=$(current_profile); \
    )
endif

arm_tidl:
ifeq ($(BUILD_LINUX_MPU),yes)
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
endif
ifeq ($(BUILD_QNX_MPU),yes)
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH)/arm-tidl tidl_tiovx_kernels TARGET_BUILD=$(current_profile); \
    )
endif

dsp_tidl_scrub dsp_tidl_clean:
ifeq ($(BUILD_LINUX_MPU),yes)
ifeq ($(BUILD_EMULATION_MODE),yes)
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH) tidl_lib_clean DSP_TOOLS=$(CGT7X_ROOT) TARGET_PLATFORM=PC TARGET_BUILD=$(current_profile); \
    )
endif
endif
ifeq ($(BUILD_TARGET_MODE),yes)
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH) tidl_lib_clean DSP_TOOLS=$(CGT7X_ROOT) TARGET_BUILD=$(current_profile); \
    )
endif

arm_tidl_scrub arm_tidl_clean:
ifeq ($(BUILD_LINUX_MPU),yes)
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
endif
ifeq ($(BUILD_QNX_MPU),yes)
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
		$(MAKE) -C $(TIDL_PATH)/arm-tidl tidl_tiovx_kernels_clean TARGET_BUILD=$(current_profile); \
    )
endif

tidl: dsp_tidl arm_tidl

tidl_clean: dsp_tidl_clean arm_tidl_clean

tidl_scrub: dsp_tidl_scrub arm_tidl_scrub

.PHONY: tidl tidl_clean tidl_scrub dsp_tidl dsp_tidl_clean dsp_tidl_scrub arm_tidl arm_tidl_clean arm_tidl_scrub mmalib mmalib_clean mmalib_scrub
