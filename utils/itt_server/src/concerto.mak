ifneq ($(TARGET_PLATFORM),PC)

include $(PRELUDE)
TARGET      := app_utils_itt_server
TARGETTYPE  := library

ifeq ($(TARGET_OS),LINUX)
CSOURCES    := itt_server_main.c itt_ctrl_handle_echo.c itt_ctrl_handle_2a.c itt_ctrl_handle_image_save.c itt_ctrl_handle_dcc_send.c itt_ctrl_handle_sensor.c itt_ctrl_handle_dev_ctrl.c

endif

IDIRS += $(IMAGING_PATH)/kernels/include/
IDIRS += $(VISION_APPS_PATH)
IDIRS += $(VISION_APPS_PATH)/utils/itt_server/include
IDIRS += $(VISION_APPS_PATH)/utils/network_api/include

include $(FINALE)

endif
