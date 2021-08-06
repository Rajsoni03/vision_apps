#
# Utility makefile for SBL boot support
#
# Edit this file to suit your build needs
#

#OSPI DATA
OSPI_LOCATION_SBL_IMAGE=0
OSPI_LOCATION_SYSFW_BIN=80000
OSPI_LOCATION_BOOT_APP=100000
OSPI_LOCATION_MULTI_CORE_IMAGE_1=1FC0000
OSPI_LOCATION_MULTI_CORE_IMAGE_2=27C0000
OSPI_LOCATION_ATF_IMAGE=1C0000
OSPI_LOCATION_LINUX_DTB=1EC0000
OSPI_LOCATION_HLOS_KERNEL_IMAGE=7C0000
OSPI_PATTERN=3FE0000
OSPI_BINARIES_PATH=$(VISION_APPS_PATH)/out/sbl_bootfiles
OSPI_PATTERN_FILE=$(PDK_PATH)/packages/ti/board/src/flash/nor/ospi/nor_spi_patterns.bin

#UNIFLASH INFO
UNIFLASH_VERSION=uniflash_6.0.0
UNIFLASH_DIR=${HOME}/ti/$(UNIFLASH_VERSION)
UNIFLASH_COM_PORT=/dev/ttyUSB5
UNIFLASH_SCRIPT=$(UNIFLASH_DIR)/dslite.sh

ifneq ("$(wildcard $(UNIFLASH_SCRIPT))","")
	UNIFLASH_FOUND=yes
else
	UNIFLASH_FOUND=no
endif

#SBL INFO
SBL_CORE=mcu1_0
BOARD=j721e_evm
ATF_OPTEE_PATH=$(VISION_APPS_PATH)/out/sbl_bootfiles/atf_optee_dir
SBL_REPO_PATH=$(PDK_PATH)/packages/ti/boot/sbl
MULTICORE_APPIMAGE_GEN_TOOL_PATH=$(SBL_REPO_PATH)/tools/multicoreImageGen/bin
SBL_OUT2RPRC_GEN_TOOL_PATH=$(SBL_REPO_PATH)/tools/out2rprc/bin
DEV_ID=55

REMOTE_CORE_LIST_LATEAPP1=
REMOTE_CORE_LIST_LATEAPP2=

CERT_SCRIPT=$(PDK_PATH)/packages/ti/build/makerules/x509CertificateGen.sh

ifeq ($(BUILD_CPU_MCU2_0),yes)
	REMOTE_CORE_LIST_LATEAPP1+=6 $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/vx_app_rtos_qnx_mcu2_0.out.rprc
endif
ifeq ($(BUILD_CPU_MCU2_1),yes)
	REMOTE_CORE_LIST_LATEAPP1+=7 $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/vx_app_rtos_qnx_mcu2_1.out.rprc
endif
ifeq ($(BUILD_CPU_MCU3_0),yes)
	REMOTE_CORE_LIST_LATEAPP2+=8 $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/vx_app_rtos_qnx_mcu3_0.out.rprc
endif
ifeq ($(BUILD_CPU_MCU3_1),yes)
	REMOTE_CORE_LIST_LATEAPP2+=9 $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/vx_app_rtos_qnx_mcu3_1.out.rprc
endif
ifeq ($(BUILD_CPU_C6x_1),yes)
	REMOTE_CORE_LIST_LATEAPP2+=10 $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/vx_app_rtos_qnx_c6x_1.out.rprc
endif
ifeq ($(BUILD_CPU_C6x_2),yes)
	REMOTE_CORE_LIST_LATEAPP2+=11 $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/vx_app_rtos_qnx_c6x_2.out.rprc
endif
ifeq ($(BUILD_CPU_C7x_1),yes)
	REMOTE_CORE_LIST_LATEAPP2+=12 $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/vx_app_rtos_qnx_c7x_1.out.rprc
endif

ifeq ($(BUILD_LINUX_A72), yes)
	SBL_SD_FS_PATH=$(LINUX_SD_FS_BOOT_PATH)
	LDS_PATH=$(VISION_APPS_PATH)/apps/basic_demos/app_rtos/rtos_linux/linux_lds
endif
ifeq ($(BUILD_QNX_A72), yes)
	SBL_SD_FS_PATH=$(QNX_SD_FS_BOOT_PATH)
	LDS_PATH=$(VISION_APPS_PATH)/apps/basic_demos/app_rtos/rtos_qnx/qnx_lds
endif

sbl_atf_optee:
ifeq ($(BUILD_QNX_A72), yes)
	# For ATF, setting HANDLE_EA_EL3_FIRST=0 for QNX so that the all runtime exception to be routed to current exception level (or in EL1 if the current exception level is EL0)
	$(MAKE) -C $(VISION_APPS_PATH)/../arm-trusted-firmware -s -j32 CROSS_COMPILE=$(GCC_LINUX_ARM_ROOT)/bin/aarch64-none-linux-gnu- PLAT=k3 TARGET_BOARD=generic SPD=opteed  HANDLE_EA_EL3_FIRST=0
endif
ifeq ($(BUILD_LINUX_A72), yes)
	$(MAKE) -C $(VISION_APPS_PATH)/../arm-trusted-firmware -s -j32 CROSS_COMPILE=$(GCC_LINUX_ARM_ROOT)/bin/aarch64-none-linux-gnu- PLAT=k3 TARGET_BOARD=generic SPD=opteed
endif

	$(MAKE) -C $(VISION_APPS_PATH)/../ti-optee-os -s -j32 CROSS_COMPILE_core=$(GCC_LINUX_ARM_ROOT)/bin/aarch64-none-linux-gnu- CROSS_COMPILE_ta_arm32=$(GCC_LINUX_ARM_ROOT)/bin/aarch64-none-linux-gnu- CROSS_COMPILE_ta_arm64=$(GCC_LINUX_ARM_ROOT)/bin/aarch64-none-linux-gnu- NOWERROR=1 CFG_TEE_TA_LOG_LEVEL=0 CFG_TEE_CORE_LOG_LEVEL=2 CFG_ARM64_core=y ta-targets=ta_arm64 PLATFORM=k3 PLATFORM_FLAVOR=j7
	mkdir -p $(ATF_OPTEE_PATH)
	cp $(VISION_APPS_PATH)/../arm-trusted-firmware/build/k3/generic/release/bl31.bin $(ATF_OPTEE_PATH)/bl31.bin
	cp $(VISION_APPS_PATH)/../ti-optee-os/out/arm-plat-k3/core/tee-pager_v2.bin $(ATF_OPTEE_PATH)/bl32.bin


sbl_atf_optee_scrub:
	$(MAKE) -C $(VISION_APPS_PATH)/../arm-trusted-firmware clean
	$(MAKE) -C $(VISION_APPS_PATH)/../ti-optee-os clean CFG_ARM64_core=y PLATFORM=k3 PLATFORM_FLAVOR=j7
	rm -rf $(VISION_APPS_PATH)/../arm-trusted-firmware/build/k3
	rm -rf $(VISION_APPS_PATH)/../ti-optee-os/out/arm-plat-k3

sbl_pdk_sd:
	$(MAKE) -C $(PDK_PATH)/packages/ti/build sbl_mmcsd_img DISABLE_RECURSE_DEPS=no BOARD=$(BOARD) CORE=$(SBL_CORE) -s
	mkdir -p $(VISION_APPS_PATH)/out/sbl_bootfiles
	cp $(PDK_PATH)/packages/ti/boot/sbl/binary/$(BOARD)/mmcsd/bin/sbl_mmcsd_img_$(SBL_CORE)_release.tiimage $(VISION_APPS_PATH)/out/sbl_bootfiles/tiboot3.bin
	cp $(PDK_PATH)/packages/ti/drv/sciclient/soc/V1/tifs.bin $(VISION_APPS_PATH)/out/sbl_bootfiles/tifs.bin

sbl_pdk_sd_hs:
	$(MAKE) -C $(PDK_PATH)/packages/ti/build sbl_mmcsd_img_hs DISABLE_RECURSE_DEPS=no BOARD=$(BOARD) CORE=$(SBL_CORE) -s
	mkdir -p $(VISION_APPS_PATH)/out/sbl_bootfiles
	cp $(PDK_PATH)/packages/ti/boot/sbl/binary/$(BOARD)_hs/mmcsd/bin/sbl_mmcsd_img_$(SBL_CORE)_release.tiimage $(VISION_APPS_PATH)/out/sbl_bootfiles/tiboot3.bin.signed
ifeq ($(J7ES_SR), 1_1)
	cp $(PDK_PATH)/packages/ti/drv/sciclient/soc/V1/tifs_sr1.1-hs-enc.bin $(VISION_APPS_PATH)/out/sbl_bootfiles/tifs.bin.signed
else
	cp $(PDK_PATH)/packages/ti/drv/sciclient/soc/V1/tifs-hs-enc.bin $(VISION_APPS_PATH)/out/sbl_bootfiles/tifs.bin.signed
endif

sbl_pdk_ospi:
	$(MAKE) -C $(PDK_PATH)/packages/ti/build sbl_cust_img DISABLE_RECURSE_DEPS=no BOARD=$(BOARD) CORE=$(SBL_CORE) -s
	mkdir -p $(VISION_APPS_PATH)/out/sbl_bootfiles
	cp $(PDK_PATH)/packages/ti/boot/sbl/binary/$(BOARD)/cust/bin/sbl_cust_img_$(SBL_CORE)_release.tiimage $(VISION_APPS_PATH)/out/sbl_bootfiles/

sbl_pdk_ospi_hs:
	$(MAKE) -C $(PDK_PATH)/packages/ti/build sbl_cust_img_hs DISABLE_RECURSE_DEPS=no BOARD=$(BOARD) CORE=$(SBL_CORE) -s
	mkdir -p $(VISION_APPS_PATH)/out/sbl_bootfiles
	cp $(PDK_PATH)/packages/ti/boot/sbl/binary/$(BOARD)_hs/cust/bin/sbl_cust_img_$(SBL_CORE)_release.tiimage $(VISION_APPS_PATH)/out/sbl_bootfiles/sbl_cust_img_$(SBL_CORE)_release.tiimage.signed

sbl_mcusw_bootimage_touch:
	touch $(MCUSW_PATH)/mcuss_demos/boot_app_mcu_rtos/boot.c
	touch $(MCUSW_PATH)/mcuss_demos/boot_app_mcu_rtos/main_tirtos.c
	touch $(MCUSW_PATH)/mcuss_demos/boot_app_mcu_rtos/soc/j721e/boot_core_defs.c

sbl_mcusw_bootimage_sd:
	$(MAKE) sbl_mcusw_bootimage_touch
ifeq ($(BUILD_QNX_A72), yes)
ifeq ($(RTOS),SYSBIOS)
	$(MAKE) -C $(MCUSW_PATH)/build can_boot_app_mcu_rtos -s HLOSBOOT=qnx BOOTMODE=mmcsd BOARD=$(BOARD) CORE=$(SBL_CORE) BUILD_OS_TYPE=tirtos CANFUNC=none
endif
ifeq ($(RTOS),FREERTOS)
	$(MAKE) -C $(MCUSW_PATH)/build can_boot_app_mcu_rtos -s HLOSBOOT=qnx BOOTMODE=mmcsd BOARD=$(BOARD) CORE=$(SBL_CORE) BUILD_OS_TYPE=freertos CANFUNC=none
endif
endif
ifeq ($(BUILD_LINUX_A72), yes)
ifeq ($(RTOS),SYSBIOS)
	$(MAKE) -C $(MCUSW_PATH)/build can_boot_app_mcu_rtos -s HLOSBOOT=linux BOOTMODE=mmcsd BOARD=$(BOARD) CORE=$(SBL_CORE) BUILD_OS_TYPE=tirtos CANFUNC=none
endif
ifeq ($(RTOS),FREERTOS)
	$(MAKE) -C $(MCUSW_PATH)/build can_boot_app_mcu_rtos -s HLOSBOOT=linux BOOTMODE=mmcsd BOARD=$(BOARD) CORE=$(SBL_CORE) BUILD_OS_TYPE=freertos CANFUNC=none
endif
endif
	mkdir -p $(VISION_APPS_PATH)/out/sbl_bootfiles
	cp $(MCUSW_PATH)/binary/can_boot_app_mcu_rtos/bin/$(BOARD)/can_boot_app_mcu_rtos_$(SBL_CORE)_release.appimage $(VISION_APPS_PATH)/out/sbl_bootfiles/app

sbl_mcusw_bootimage_sd_hs:
	$(MAKE) sbl_mcusw_bootimage_sd
	$(CERT_SCRIPT)  -b $(VISION_APPS_PATH)/out/sbl_bootfiles/app -o $(VISION_APPS_PATH)/out/sbl_bootfiles/app.signed -c R5 -l 0x0 -k $(PDK_PATH)/packages/ti/build/makerules/k3_dev_mpk.pem

sbl_mcusw_bootimage_ospi:
	$(MAKE) sbl_mcusw_bootimage_touch
ifeq ($(BUILD_QNX_A72), yes)
ifeq ($(RTOS),SYSBIOS)
	$(MAKE) -C $(MCUSW_PATH)/build can_boot_app_mcu_rtos -s HLOSBOOT=qnx BOOTMODE=ospi BOARD=$(BOARD) CORE=$(SBL_CORE) BUILD_OS_TYPE=tirtos CANFUNC=none
endif
ifeq ($(RTOS),FREERTOS)
	$(MAKE) -C $(MCUSW_PATH)/build can_boot_app_mcu_rtos -s HLOSBOOT=qnx BOOTMODE=ospi BOARD=$(BOARD) CORE=$(SBL_CORE) BUILD_OS_TYPE=freertos CANFUNC=none
endif
endif
ifeq ($(BUILD_LINUX_A72), yes)
ifeq ($(RTOS),SYSBIOS)
	$(MAKE) -C $(MCUSW_PATH)/build can_boot_app_mcu_rtos -s HLOSBOOT=linux BOOTMODE=ospi BOARD=$(BOARD) CORE=$(SBL_CORE) BUILD_OS_TYPE=tirtos CANFUNC=none
endif
ifeq ($(RTOS),FREERTOS)
	$(MAKE) -C $(MCUSW_PATH)/build can_boot_app_mcu_rtos -s HLOSBOOT=linux BOOTMODE=ospi BOARD=$(BOARD) CORE=$(SBL_CORE) BUILD_OS_TYPE=freertos CANFUNC=none
endif
endif
	mkdir -p $(VISION_APPS_PATH)/out/sbl_bootfiles
	cp $(MCUSW_PATH)/binary/can_boot_app_mcu_rtos/bin/$(BOARD)/can_boot_app_mcu_rtos_$(SBL_CORE)_release.appimage $(VISION_APPS_PATH)/out/sbl_bootfiles/can_boot_app_mcu_rtos_$(SBL_CORE)_release_ospi.appimage

sbl_mcusw_bootimage_ospi_hs:
	$(MAKE) sbl_mcusw_bootimage_ospi
	$(CERT_SCRIPT)  -b $(VISION_APPS_PATH)/out/sbl_bootfiles/can_boot_app_mcu_rtos_$(SBL_CORE)_release_ospi.appimage -o $(VISION_APPS_PATH)/out/sbl_bootfiles/can_boot_app_mcu_rtos_$(SBL_CORE)_release_ospi.appimage.signed -c R5 -l 0x0 -k $(PDK_PATH)/packages/ti/build/makerules/k3_dev_mpk.pem

sbl_vision_apps_bootimage_1:
	mkdir -p $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs
ifeq ($(BUILD_CPU_MCU2_0),yes)
	$(SBL_OUT2RPRC_GEN_TOOL_PATH)/out2rprc.exe $(VISION_APPS_PATH)/out/J7/R5F/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_mcu2_0.out $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/vx_app_rtos_qnx_mcu2_0.out.rprc
endif
ifeq ($(BUILD_CPU_MCU2_1),yes)
	$(SBL_OUT2RPRC_GEN_TOOL_PATH)/out2rprc.exe $(VISION_APPS_PATH)/out/J7/R5F/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_mcu2_1.out $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/vx_app_rtos_qnx_mcu2_1.out.rprc
endif
	$(MULTICORE_APPIMAGE_GEN_TOOL_PATH)/MulticoreImageGen LE $(DEV_ID) $(VISION_APPS_PATH)/out/sbl_bootfiles/lateapp1 $(REMOTE_CORE_LIST_LATEAPP1)

sbl_vision_apps_bootimage_hs_1:
	$(MAKE) sbl_vision_apps_bootimage_1
	$(CERT_SCRIPT) -b $(VISION_APPS_PATH)/out/sbl_bootfiles/lateapp1 -o $(VISION_APPS_PATH)/out/sbl_bootfiles/lateapp1.signed -c R5 -l 0x0 -k $(PDK_PATH)/packages/ti/build/makerules/k3_dev_mpk.pem

sbl_vision_apps_bootimage_2:
	mkdir -p $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs
ifeq ($(BUILD_CPU_MCU3_0),yes)
	$(SBL_OUT2RPRC_GEN_TOOL_PATH)/out2rprc.exe $(VISION_APPS_PATH)/out/J7/R5F/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_mcu3_0.out $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/vx_app_rtos_qnx_mcu3_0.out.rprc
endif
ifeq ($(BUILD_CPU_MCU3_1),yes)
	$(SBL_OUT2RPRC_GEN_TOOL_PATH)/out2rprc.exe $(VISION_APPS_PATH)/out/J7/R5F/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_mcu3_1.out $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/vx_app_rtos_qnx_mcu3_1.out.rprc
endif
ifeq ($(BUILD_CPU_C6x_1),yes)
	$(SBL_OUT2RPRC_GEN_TOOL_PATH)/out2rprc.exe $(VISION_APPS_PATH)/out/J7/C66/SYSBIOS/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_c6x_1.out $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/vx_app_rtos_qnx_c6x_1.out.rprc
endif
ifeq ($(BUILD_CPU_C6x_2),yes)
	$(SBL_OUT2RPRC_GEN_TOOL_PATH)/out2rprc.exe $(VISION_APPS_PATH)/out/J7/C66/SYSBIOS/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_c6x_2.out $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/vx_app_rtos_qnx_c6x_2.out.rprc
endif
ifeq ($(BUILD_CPU_C7x_1),yes)
	$(SBL_OUT2RPRC_GEN_TOOL_PATH)/out2rprc.exe $(VISION_APPS_PATH)/out/J7/C71/SYSBIOS/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_c7x_1.out $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/vx_app_rtos_qnx_c7x_1.out.rprc
endif
	$(MULTICORE_APPIMAGE_GEN_TOOL_PATH)/MulticoreImageGen LE $(DEV_ID) $(VISION_APPS_PATH)/out/sbl_bootfiles/lateapp2 $(REMOTE_CORE_LIST_LATEAPP2)

sbl_vision_apps_bootimage_hs_2:
	$(MAKE) sbl_vision_apps_bootimage_2
	$(CERT_SCRIPT) -b $(VISION_APPS_PATH)/out/sbl_bootfiles/lateapp2 -o $(VISION_APPS_PATH)/out/sbl_bootfiles/lateapp2.signed -c R5 -l 0x0 -k $(PDK_PATH)/packages/ti/build/makerules/k3_dev_mpk.pem

sbl_vision_apps_bootimage: sbl_vision_apps_bootimage_1 sbl_vision_apps_bootimage_2

sbl_vision_apps_bootimage_hs:sbl_vision_apps_bootimage_hs_1 sbl_vision_apps_bootimage_hs_2

sbl_qnx_bootimage:
ifeq ($(BUILD_QNX_A72), yes)
ifeq ("$(wildcard $(QNX_FS_PATH)/qnx-ifs)","")
	$(error qnx-ifs is still not built!)
endif
	mkdir -p $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs
	curr_dir=$(PWD)
	cd $(ATF_OPTEE_PATH) && \
	$(QNX_BASE)/host/linux/x86_64/usr/bin/$(QNX_CROSS_COMPILER_TOOL)ld -T $(LDS_PATH)/atf_optee.lds -o $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/atf_optee.elf && \
	cd $(QNX_FS_PATH) && \
	$(QNX_BASE)/host/linux/x86_64/usr/bin/$(QNX_CROSS_COMPILER_TOOL)ld -T $(LDS_PATH)/ifs_qnx.lds -o $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/ifs_qnx.elf && \
	cd $(curr_dir)
	$(SBL_OUT2RPRC_GEN_TOOL_PATH)/out2rprc.exe $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/atf_optee.elf $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/atf_optee.rprc
	$(SBL_OUT2RPRC_GEN_TOOL_PATH)/out2rprc.exe $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/ifs_qnx.elf $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/ifs_qnx.rprc
	$(MULTICORE_APPIMAGE_GEN_TOOL_PATH)/MulticoreImageGen LE $(DEV_ID) $(VISION_APPS_PATH)/out/sbl_bootfiles/atf_optee.appimage 0 $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/atf_optee.rprc
	$(MULTICORE_APPIMAGE_GEN_TOOL_PATH)/MulticoreImageGen LE $(DEV_ID) $(VISION_APPS_PATH)/out/sbl_bootfiles/ifs_qnx.appimage 0 $(VISION_APPS_PATH)/out/sbl_bootfiles/rprcs/ifs_qnx.rprc
endif

sbl_qnx_bootimage_hs:
ifeq ($(BUILD_QNX_A72), yes)
	$(MAKE) sbl_qnx_bootimage
	$(CERT_SCRIPT) -b $(VISION_APPS_PATH)/out/sbl_bootfiles/atf_optee.appimage -o $(VISION_APPS_PATH)/out/sbl_bootfiles/atf_optee.appimage.signed -c R5 -l 0x0 -k $(PDK_PATH)/packages/ti/build/makerules/k3_dev_mpk.pem
	$(CERT_SCRIPT) -b $(VISION_APPS_PATH)/out/sbl_bootfiles/ifs_qnx.appimage -o $(VISION_APPS_PATH)/out/sbl_bootfiles/ifs_qnx.appimage.signed -c R5 -l 0x0 -k $(PDK_PATH)/packages/ti/build/makerules/k3_dev_mpk.pem
endif

sbl_linux_bootimage:

sbl_bootimage_sd: sbl_pdk_sd sbl_mcusw_bootimage_sd
sbl_bootimage_ospi: sbl_pdk_ospi sbl_mcusw_bootimage_ospi

sbl_bootimage_sd_hs: sbl_pdk_sd_hs sbl_mcusw_bootimage_sd_hs
sbl_bootimage_ospi_hs: sbl_pdk_ospi_hs sbl_mcusw_bootimage_ospi_hs

sbl_bootimage: sbl_bootimage_sd sbl_bootimage_ospi sbl_atf_optee sbl_vision_apps_bootimage sbl_qnx_bootimage sbl_linux_bootimage

sbl_bootimage_hs: sbl_bootimage_sd_hs sbl_bootimage_ospi_hs sbl_atf_optee sbl_vision_apps_bootimage_hs sbl_qnx_bootimage_hs

sbl_bootimage_flash_uniflash_programmer:
ifeq ($(UNIFLASH_FOUND), yes)
	$(UNIFLASH_SCRIPT) --mode processors -c $(UNIFLASH_COM_PORT) -f $(UNIFLASH_DIR)/processors/FlashWriter/j721e_evm/uart_j721e_evm_flash_programmer_release.tiimage -i 0 || true
else
	echo "Uniflash not found! Please install uniflash or update the uniflash path in makefile"
endif

sbl_bootimage_flash_sbl_sysfw_bootapp:
ifeq ($(UNIFLASH_FOUND), yes)
	#SBL Image
	$(UNIFLASH_SCRIPT) --mode processors -c $(UNIFLASH_COM_PORT) -f $(OSPI_BINARIES_PATH)/sbl_cust_img_mcu1_0_release.tiimage -d 3 -o $(OSPI_LOCATION_SBL_IMAGE) || true
	#OSPI Patten file
	$(UNIFLASH_SCRIPT) --mode processors -c $(UNIFLASH_COM_PORT) -f $(OSPI_PATTERN_FILE) -d 3 -o $(OSPI_PATTERN) || true
	#SYSFW BIN
	$(UNIFLASH_SCRIPT) --mode processors -c $(UNIFLASH_COM_PORT) -f $(OSPI_BINARIES_PATH)/tifs.bin -d 3 -o $(OSPI_LOCATION_SYSFW_BIN) || true
	#Boot App
	$(UNIFLASH_SCRIPT) --mode processors -c $(UNIFLASH_COM_PORT) -f $(OSPI_BINARIES_PATH)/can_boot_app_mcu_rtos_mcu1_0_release_ospi.appimage -d 3 -o $(OSPI_LOCATION_BOOT_APP) || true
else
	echo "Uniflash not found! Please install uniflash or update the uniflash path in makefile"
endif

sbl_bootimage_flash_hlos:
ifeq ($(UNIFLASH_FOUND), yes)
	#ATF Image
	$(UNIFLASH_SCRIPT) --mode processors -c $(UNIFLASH_COM_PORT) -f $(OSPI_BINARIES_PATH)/atf_optee.appimage -d 3 -o $(OSPI_LOCATION_ATF_IMAGE) || true
	#HLOS Image
	$(UNIFLASH_SCRIPT) --mode processors -c $(UNIFLASH_COM_PORT) -f $(OSPI_BINARIES_PATH)/ifs_qnx.appimage -d 3 -o $(OSPI_LOCATION_HLOS_KERNEL_IMAGE) || true
else
	echo "Uniflash not found! Please install uniflash or update the uniflash path in makefile"
endif

sbl_bootimage_flash_rtosapp:
ifeq ($(UNIFLASH_FOUND), yes)
	#Multicore Image 1
	$(UNIFLASH_SCRIPT) --mode processors -c $(UNIFLASH_COM_PORT) -f $(OSPI_BINARIES_PATH)/lateapp1 -d 3 -o $(OSPI_LOCATION_MULTI_CORE_IMAGE_1) || true
	#Multicore Image 2
	$(UNIFLASH_SCRIPT) --mode processors -c $(UNIFLASH_COM_PORT) -f $(OSPI_BINARIES_PATH)/lateapp2 -d 3 -o $(OSPI_LOCATION_MULTI_CORE_IMAGE_2) || true
else
	echo "Uniflash not found! Please install uniflash or update the uniflash path in makefile"
endif

sbl_bootimage_install_sd: sbl_vision_apps_bootimage sbl_qnx_bootimage sbl_linux_bootimage
	cp $(VISION_APPS_PATH)/out/sbl_bootfiles/tiboot3.bin $(SBL_SD_FS_PATH)
	cp $(VISION_APPS_PATH)/out/sbl_bootfiles/tifs.bin $(SBL_SD_FS_PATH)
	cp $(VISION_APPS_PATH)/out/sbl_bootfiles/app $(SBL_SD_FS_PATH)
	cp $(VISION_APPS_PATH)/out/sbl_bootfiles/lateapp* $(SBL_SD_FS_PATH)
ifeq ($(BUILD_QNX_A72), yes)
	cp $(VISION_APPS_PATH)/out/sbl_bootfiles/atf_optee.appimage $(SBL_SD_FS_PATH)
	cp $(VISION_APPS_PATH)/out/sbl_bootfiles/ifs_qnx.appimage $(SBL_SD_FS_PATH)
endif
	sync

sbl_bootimage_hs_install_sd: sbl_vision_apps_bootimage_hs sbl_qnx_bootimage_hs
	cp $(VISION_APPS_PATH)/out/sbl_bootfiles/tiboot3.bin.signed $(SBL_SD_FS_PATH)/tiboot3.bin
	cp $(VISION_APPS_PATH)/out/sbl_bootfiles/tifs.bin.signed $(SBL_SD_FS_PATH)/tifs.bin
	cp $(VISION_APPS_PATH)/out/sbl_bootfiles/app.signed $(SBL_SD_FS_PATH)/app
	cp $(VISION_APPS_PATH)/out/sbl_bootfiles/lateapp1.signed $(SBL_SD_FS_PATH)/lateapp1
	cp $(VISION_APPS_PATH)/out/sbl_bootfiles/lateapp2.signed $(SBL_SD_FS_PATH)/lateapp2
ifeq ($(BUILD_QNX_A72), yes)
	cp $(VISION_APPS_PATH)/out/sbl_bootfiles/atf_optee.appimage.signed $(SBL_SD_FS_PATH)/atf_optee.appimage
	cp $(VISION_APPS_PATH)/out/sbl_bootfiles/ifs_qnx.appimage.signed $(SBL_SD_FS_PATH)/ifs_qnx.appimage
endif
	sync

sbl_bootimage_install_ospi: sbl_vision_apps_bootimage sbl_qnx_bootimage sbl_linux_bootimage
	$(MAKE) sbl_bootimage_flash_uniflash_programmer
	$(MAKE) sbl_bootimage_flash_sbl_sysfw_bootapp
	$(MAKE) sbl_bootimage_flash_rtosapp
	$(MAKE) sbl_bootimage_flash_hlos

sbl_bootimage_scrub: sbl_atf_optee_scrub
	rm -rf $(PDK_PATH)/packages/ti/binary/sbl_*
	rm -rf $(PDK_PATH)/packages/ti/binary/ti/boot/
	rm -rf $(PDK_PATH)/packages/ti/boot/sbl/binary
	rm -rf $(MCUSW_PATH)/binary
	rm -rf $(VISION_APPS_PATH)/out/sbl_bootfiles/

sbl_bootimage_clean: sbl_bootimage_scrub
