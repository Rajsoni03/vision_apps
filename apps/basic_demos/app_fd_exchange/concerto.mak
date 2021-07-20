ifeq ($(TARGET_CPU),$(filter $(TARGET_CPU), A72))
ifeq ($(TARGET_OS),$(filter $(TARGET_OS), LINUX QNX))

_MODULE=producer
# Producer executable
include $(PRELUDE)

TARGET      := vx_app_arm_fd_exchange_producer
TARGETTYPE  := exe
CSOURCES    := main_producer.c apputils_net.c app_common.c

ifeq ($(TARGET_OS),$(filter $(TARGET_OS), QNX))
SYS_SHARED_LIBS += socket
endif

include $(VISION_APPS_PATH)/apps/concerto_a72_inc.mak

include $(FINALE)

_MODULE=consumer
# Consumer executable
include $(PRELUDE)

TARGET      := vx_app_arm_fd_exchange_consumer
TARGETTYPE  := exe
CSOURCES    := main_consumer.c apputils_net.c app_common.c

ifeq ($(TARGET_OS),$(filter $(TARGET_OS), QNX))
SYS_SHARED_LIBS += socket
endif

include $(VISION_APPS_PATH)/apps/concerto_a72_inc.mak

include $(FINALE)

endif
endif
