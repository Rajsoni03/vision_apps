#
# Utility makefile to build codec libraries
#
# Edit this file to suit your specific build needs
#

codec:
ifeq ($(BUILD_TARGET_MODE),yes)
	$(MAKE) -C $(VIDEO_CODEC_PATH) codec_libs -s
endif

codec_docs:
	$(MAKE) -C $(VIDEO_CODEC_PATH) doxy_docs -s

codec_clean:
ifeq ($(BUILD_TARGET_MODE),yes)
	$(MAKE) -C $(VIDEO_CODEC_PATH) codec_apps_clean -s
endif

codec_scrub:
ifeq ($(BUILD_TARGET_MODE),yes)
	rm -rf $(VIDEO_CODEC_PATH)/out
endif

.PHONY: codec codec_docs codec_clean codec_scrub
