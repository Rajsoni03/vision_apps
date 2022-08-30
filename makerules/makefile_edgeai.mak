#
# Utility makefile to build edgeai components
#
# 1. edgeai-tiovx-modules
# 2. edgeai-gst-plugins
#

EDGEAI_MODULES_PATH   ?= $(PSDK_PATH)/edgeai-tiovx-modules/
EDGEAI_PLUGINS_PATH   ?= $(PSDK_PATH)/edgeai-gst-plugins/
CROSS_COMPILER_PATH    = $(PSDK_PATH)/gcc-arm-9.2-2019.12-x86_64-aarch64-none-linux-gnu
CROSS_COMPILER_PREFIX  = aarch64-none-linux-gnu-
TARGET_FS              = $(PSDK_PATH)/targetfs
EDGEAI_INSTALL_PATH   ?= $(TARGET_FS)

export CROSS_COMPILER_PATH
export CROSS_COMPILER_PREFIX
export TARGET_FS

edgeai:
ifeq ($(SOC), $(filter $(SOC), j721e j721s2))
	@echo "Building EdgeAI Components"
	$(MAKE) edgeai_check_paths
	$(MAKE) linux_fs_install
	$(MAKE) edgeai_modules
	$(MAKE) edgeai_plugins
else ifeq ($(SOC),j784s4)
	@echo "Building EdgeAI Components not Supported"
endif

edgeai_modules:
	@echo "Building EdgeAI Modules"
	cd $(EDGEAI_MODULES_PATH); \
	mkdir build; \
	cd build; \
	cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/cross_compile_aarch64.cmake ..; \
	$(MAKE) install DESTDIR=$(TARGET_FS)

edgeai_plugins:
	@echo "Building EdgeAI Plugins"
	cd $(EDGEAI_PLUGINS_PATH); \
	PKG_CONFIG_PATH='' crossbuild/environment $(PSDK_PATH) > aarch64-none-linux-gnu.ini; \
	PKG_CONFIG_PATH='' meson build --cross-file aarch64-none-linux-gnu.ini --cross-file crossbuild/crosscompile.ini; \
	DESTDIR=$(TARGET_FS) ninja -C build install

edgeai_install:
	@echo "Install EdgeAI Modules and Plugins to EDGEAI_INSTALL_PATH"
	cd $(EDGEAI_MODULES_PATH); \
	$(MAKE) install DESTDIR=$(EDGEAI_INSTALL_PATH) -C build
	cd $(EDGEAI_PLUGINS_PATH); \
	DESTDIR=$(EDGEAI_INSTALL_PATH) ninja -C build install; \
	sync

edgeai_scrub:
	@echo "EdgeAI Scrub"
	cd $(EDGEAI_MODULES_PATH); \
	rm -rf build bin lib
	cd $(EDGEAI_PLUGINS_PATH); \
	rm -rf build
