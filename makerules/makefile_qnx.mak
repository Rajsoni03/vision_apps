#
# Utility makefile to build QNX libaries and related components
#
# Edit this file to suit your specific build needs
#

QNX_AUX_FS_PATH=$(PSDK_QNX_PATH)/rootfs

ifeq ($(PROFILE), $(filter $(PROFILE),debug all))
QNX_APP_PROFILE=debug
endif
ifeq ($(PROFILE), $(filter $(PROFILE),release all))
QNX_APP_PROFILE=release
endif

qnx:
ifeq ($(BUILD_QNX_A72),yes)
	$(MAKE) -C $(PSDK_QNX_PATH)/qnx all        VISION_APPS_BUILD_FLAGS_MAK=$(VISION_APPS_BUILD_FLAGS_MAK) QNX_BASE=$(QNX_BASE) PROFILE=$(PROFILE) BOARD=$(BOARD) -s
endif

qnx_clean:
ifeq ($(BUILD_QNX_A72),yes)
	$(MAKE) -C $(PSDK_QNX_PATH)/qnx clean      QNX_BASE=$(QNX_BASE) PROFILE=$(PROFILE) BOARD=$(BOARD)
endif

qnx_scrub:
ifeq ($(BUILD_QNX_A72),yes)
	$(MAKE) -C $(PSDK_QNX_PATH)/qnx scrub      QNX_BASE=$(QNX_BASE) PROFILE=$(PROFILE) BOARD=$(BOARD)
endif

qnx_fs_create:
ifeq ($(BUILD_QNX_A72),yes)
	$(MAKE) -C $(PSDK_QNX_PATH)/qnx qnx_fs_clean       QNX_BASE=$(QNX_BASE) PROFILE=$(PROFILE) BOARD=$(BOARD)
	$(MAKE) -C $(PSDK_QNX_PATH)/qnx qnx_fs_create      QNX_BASE=$(QNX_BASE) PROFILE=$(PROFILE) BOARD=$(BOARD)
endif

qnx_fs_copy_spl_uboot:
ifeq ($(BUILD_QNX_A72),yes)
	$(MAKE) -C $(PSDK_QNX_PATH)/qnx qnx_fs_copy_spl_uboot      QNX_BASE=$(QNX_BASE) PROFILE=$(PROFILE) BOARD=$(BOARD)
endif

qnx_fs_install:
ifeq ($(BUILD_CPU_MPU1),yes)
	# copy application binaries and scripts
	mkdir -p $(QNX_FS_PATH)/vision_apps
	cp $(VISION_APPS_PATH)/out/J7/A72/QNX/$(QNX_APP_PROFILE)/*.out $(QNX_FS_PATH)/vision_apps
	# app_linux_fs_files are not very OS specific. Only input file paths change
	cp -r $(VISION_APPS_PATH)/apps/basic_demos/app_linux_fs_files/* $(QNX_FS_PATH)/vision_apps
	# Rename file paths in app cfg files
	sed -i 's/\/opt\//\/ti_fs\//g' $(QNX_FS_PATH)/vision_apps/*.cfg
	sed -i 's/\/opt\//\/ti_fs\//g' $(QNX_FS_PATH)/vision_apps/app_srv_avp_cfg/*.cfg
	sed -i 's/\/opt\//\/ti_fs\//g' $(QNX_FS_PATH)/vision_apps/*.sh
	# Remove files not needed for QNX
	rm -rf $(QNX_FS_PATH)/vision_apps/limits.conf

	chmod +x $(QNX_FS_PATH)/vision_apps/*.sh
endif

	# remove old remote firmware files from filesystem
	mkdir -p $(QNX_AUX_FS_PATH)/lib/firmware
	-rm $(QNX_AUX_FS_PATH)/lib/firmware/j7-*-fw -f

ifeq ($(BUILD_CPU_MCU1_0),yes)
	# copy remote firmware files for mcu1_0
	cp $(VISION_APPS_PATH)/out/J7/R5F/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_mcu1_0.out $(QNX_AUX_FS_PATH)/lib/firmware/j7-mcu-r5f0_0-fw
	$(TIARMCGT_ROOT)/bin/armstrip -p $(QNX_AUX_FS_PATH)/lib/firmware/j7-mcu-r5f0_0-fw
endif

ifeq ($(BUILD_CPU_MCU2_0),yes)
	# copy remote firmware files for mcu2_0
	cp $(VISION_APPS_PATH)/out/J7/R5F/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_mcu2_0.out $(QNX_AUX_FS_PATH)/lib/firmware/j7-main-r5f0_0-fw
	$(TIARMCGT_ROOT)/bin/armstrip -p $(QNX_AUX_FS_PATH)/lib/firmware/j7-main-r5f0_0-fw
endif
ifeq ($(BUILD_CPU_MCU2_1),yes)
	# copy remote firmware files for mcu2_1
	cp $(VISION_APPS_PATH)/out/J7/R5F/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_mcu2_1.out $(QNX_AUX_FS_PATH)/lib/firmware/j7-main-r5f0_1-fw
	$(TIARMCGT_ROOT)/bin/armstrip -p $(QNX_AUX_FS_PATH)/lib/firmware/j7-main-r5f0_1-fw
endif
ifeq ($(BUILD_CPU_MCU3_0),yes)
	# copy remote firmware files for mcu3_0
	cp $(VISION_APPS_PATH)/out/J7/R5F/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_mcu3_0.out $(QNX_AUX_FS_PATH)/lib/firmware/j7-main-r5f1_0-fw
	$(TIARMCGT_ROOT)/bin/armstrip -p $(QNX_AUX_FS_PATH)/lib/firmware/j7-main-r5f1_0-fw
endif
ifeq ($(BUILD_CPU_MCU3_1),yes)
	# copy remote firmware files for mcu3_1
	cp $(VISION_APPS_PATH)/out/J7/R5F/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_mcu3_1.out $(QNX_AUX_FS_PATH)/lib/firmware/j7-main-r5f1_1-fw
	$(TIARMCGT_ROOT)/bin/armstrip -p $(QNX_AUX_FS_PATH)/lib/firmware/j7-main-r5f1_1-fw
endif
ifeq ($(BUILD_CPU_C6x_1),yes)
	# copy remote firmware files for c6x_1
	cp $(VISION_APPS_PATH)/out/J7/C66/SYSBIOS/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_c6x_1.out $(QNX_AUX_FS_PATH)/lib/firmware/j7-c66_0-fw
	$(CGT6X_ROOT)/bin/strip6x -p $(QNX_AUX_FS_PATH)/lib/firmware/j7-c66_0-fw
endif
ifeq ($(BUILD_CPU_C6x_2),yes)
	# copy remote firmware files for c6x_2
	cp $(VISION_APPS_PATH)/out/J7/C66/SYSBIOS/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_c6x_2.out $(QNX_AUX_FS_PATH)/lib/firmware/j7-c66_1-fw
	$(CGT6X_ROOT)/bin/strip6x -p $(QNX_AUX_FS_PATH)/lib/firmware/j7-c66_1-fw
endif
ifeq ($(BUILD_CPU_C7x_1),yes)
	# copy remote firmware files for c7x_1
	cp $(VISION_APPS_PATH)/out/J7/C71/SYSBIOS/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_c7x_1.out $(QNX_AUX_FS_PATH)/lib/firmware/j7-c71_0-fw
	$(CGT7X_ROOT)/bin/strip7x -p $(QNX_AUX_FS_PATH)/lib/firmware/j7-c71_0-fw
endif
	sync


# MODIFY_QNX_SD_FS macro for making PSDK RTOS modifications to the QNX to file system
define MODIFY_QNX_SD_FS_WITH_ARG =
	# copy vision apps binaries
	mkdir -p $(1)/vision_apps
	# copy tisci-mgr
	cp $(PSDK_QNX_PATH)/qnx/resmgr/sciclient_qnx_rsmgr/aarch64/o.le/tisci-mgr $(1)/tibin/
	# copy tiudma-mgr
	cp $(PSDK_QNX_PATH)/qnx/resmgr/udma_qnx_rsmgr/resmgr/aarch64/o.le/tiudma-mgr $(1)/tibin/
	# copy sharedmemallocator
	cp $(PSDK_QNX_PATH)/qnx/sharedmemallocator/resmgr/aarch64/o.le/shmemallocator $(1)/tibin/
	# copy tiipc-mgr
	cp $(PSDK_QNX_PATH)/qnx/resmgr/ipc_qnx_rsmgr/resmgr/aarch64/o.le/tiipc-mgr $(1)/tibin/
	sync
endef

define MODIFY_QNX_SD_FS =
	# copy all staged files
	cp -rfv $(QNX_FS_PATH)/* $(QNX_SD_FS_BOOT_PATH)/
	# copy vision apps binaries
	mkdir -p $(QNX_SD_FS_BOOT_PATH)/vision_apps
	cp -r $(QNX_FS_PATH)/vision_apps $(QNX_SD_FS_BOOT_PATH)/
	# copy tisci-mgr
	cp $(PSDK_QNX_PATH)/qnx/resmgr/sciclient_qnx_rsmgr/aarch64/o.le/tisci-mgr $(QNX_SD_FS_BOOT_PATH)/tibin/
	# copy tiudma-mgr
	cp $(PSDK_QNX_PATH)/qnx/resmgr/udma_qnx_rsmgr/resmgr/aarch64/o.le/tiudma-mgr $(QNX_SD_FS_BOOT_PATH)/tibin/
	# copy sharedmemallocator
	cp $(PSDK_QNX_PATH)/qnx/sharedmemallocator/resmgr/aarch64/o.le/shmemallocator $(QNX_SD_FS_BOOT_PATH)/tibin/
	# copy tiipc-mgr
	cp $(PSDK_QNX_PATH)/qnx/resmgr/ipc_qnx_rsmgr/resmgr/aarch64/o.le/tiipc-mgr $(QNX_SD_FS_BOOT_PATH)/tibin/
	sync
endef

qnx_fs_create_sd: qnx_fs_create
	sudo chmod 777 -R $(QNX_SD_FS_ROOT_PATH)
	rm -rf $(QNX_SD_FS_BOOT_PATH)/*
	rm -rf $(QNX_SD_FS_ROOT_PATH)/*
	cp -rfv $(QNX_FS_PATH)/* $(QNX_SD_FS_BOOT_PATH)/

qnx_fs_install_firmware:
	# copy remote core firmware's
	mkdir -p $(QNX_SD_FS_ROOT_PATH)/lib/firmware
	rm $(QNX_SD_FS_ROOT_PATH)/lib/firmware/j7-*-fw -f
	cp -v $(QNX_AUX_FS_PATH)/lib/firmware/j7-*-fw $(QNX_SD_FS_ROOT_PATH)/lib/firmware/
	sync

qnx_fs_install_sd: qnx_fs_copy_spl_uboot qnx_fs_install qnx_fs_install_firmware
	$(call MODIFY_QNX_SD_FS)
ifeq ($(BUILD_CPU_MCU1_0),yes)
	$(MAKE) uboot_qnx_install_sd
endif

qnx_fs_install_nfs: qnx_fs_copy_spl_uboot qnx_fs_install
	# QNX_FS_INSTALL_NFS with QNX_FS_PATH ${QNX_FS_PATH}
	$(call MODIFY_QNX_SD_FS_WITH_ARG,${QNX_FS_PATH})
	cp $(PSDK_QNX_PATH)/qnx/bsp/images/ifs-j721e-evm-ti-spl-nfs.raw $(QNX_AUX_FS_PATH)/qnx-ifs-spl-nfs
	sync

qnx_fs_install_nfs_test_data:
	$(call INSTALL_TEST_DATA,$(QNX_FS_PATH),vision_apps)
	sync

qnx_fs_install_tar: qnx_fs_install_nfs qnx_fs_install_nfs_test_data
	# Creating bootfs tar
	cd $(QNX_FS_PATH) && tar czf $(VISION_APPS_PATH)/bootfs.tar.gz .
	# Creating rootfs tar
	cp -r $(QNX_FS_PATH) $(QNX_AUX_FS_PATH)/
	cd  $(QNX_AUX_FS_PATH) && sudo tar cpzf $(VISION_APPS_PATH)/rootfs.tar.xz .

qnx_fs_install_sd_sbl: qnx_fs_install sbl_bootimage_install_sd
	$(call MODIFY_QNX_SD_FS)

qnx_fs_install_sd_sbl_hs: qnx_fs_install sbl_bootimage_hs_install_sd
	$(call MODIFY_QNX_SD_FS)

qnx_fs_install_ospi: qnx_fs_install sbl_bootimage_install_ospi
	$(call MODIFY_QNX_SD_FS)

qnx_fs_install_sd_test_data:
	$(call INSTALL_TEST_DATA,$(QNX_SD_FS_BOOT_PATH),vision_apps)

.PHONY: qnx qnx_clean qnx_scrub
