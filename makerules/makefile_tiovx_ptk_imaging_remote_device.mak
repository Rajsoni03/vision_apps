#
# Utility makefile to build TIOVX, PTK and imaging libaries and related components
#
# Edit this file to suit your specific build needs
#

#Check if PTK makefile is present and based on that skip build
PTK_MAKEFILE_PATH=$(PTK_PATH)/Makefile
ifneq ("$(wildcard $(PTK_MAKEFILE_PATH))","")
	BUILD_PTK?=yes
else
	BUILD_PTK=no
endif

tiovx:
	$(MAKE) -C $(TIOVX_PATH) -f tiovx_dev/Makefile

tiovx_clean:
	$(MAKE) -C $(TIOVX_PATH) -f tiovx_dev/Makefile clean 

tiovx_scrub:
	rm -rf $(TIOVX_PATH)/out

tiovx_docs:
	$(MAKE) -C $(TIOVX_PATH) -f tiovx_dev/Makefile doxy_docs

ptk:
ifeq ($(BUILD_PTK),yes)
ifeq ($(BUILD_EMULATION_MODE),yes)
	-$(MAKE) -C $(PTK_PATH) BUILD_TARGET_MODE=no
	-$(MAKE) -C $(PTK_PATH) release BUILD_TARGET_MODE=no
endif
ifeq ($(BUILD_TARGET_MODE),yes)
	-$(MAKE) -C $(PTK_PATH) BUILD_TARGET_MODE=yes
	-$(MAKE) -C $(PTK_PATH) release BUILD_TARGET_MODE=yes
endif
endif

ptk_clean:
ifeq ($(BUILD_PTK),yes)
	-$(MAKE) -C $(PTK_PATH) clean
endif

ptk_docs:
ifeq ($(BUILD_PTK),yes)
	$(MAKE) -C $(PTK_PATH) doxy_docs
endif

ptk_scrub:
ifeq ($(BUILD_PTK),yes)
	rm -rf $(PTK_PATH)/out
endif

imaging:
	$(MAKE) -C $(IMAGING_PATH)

imaging_clean:
	$(MAKE) -C $(IMAGING_PATH) clean

imaging_scrub:
	rm -rf $(IMAGING_PATH)/out

remote_device:
ifeq ($(BUILD_TARGET_MODE),yes)
	$(MAKE) -C $(REMOTE_DEVICE_PATH) RTOS=$(RTOS) lib_remote_device_display lib_remote_device
endif

remote_device_clean:
	$(MAKE) -C $(REMOTE_DEVICE_PATH) RTOS=$(RTOS) clean

remote_device_scrub:
	rm -rf $(REMOTE_DEVICE_PATH)/out

.PHONY: tiovx tiovx_clean ptk ptk_clean imaging imaging_clean remote_device remote_device_clean
