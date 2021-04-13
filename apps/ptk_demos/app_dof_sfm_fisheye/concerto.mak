ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), x86_64 A72))
ifeq ($(TARGET_OS), LINUX)

include $(PRELUDE)
TARGET      := vx_app_dof_sfm_fisheye
TARGETTYPE  := exe
CPPSOURCES    := app_dof_sfm_main.cpp
CPPSOURCES    += app_dof_sfm_render.cpp

include $(VISION_APPS_PATH)/apps/ptk_demos/concerto_inc.mak

CFLAGS += -Wno-unused-but-set-variable
CFLAGS += -Wno-unused-variable
CFLAGS += -Wno-unused-result
CFLAGS += -Wno-maybe-uninitialized


include $(FINALE)

endif
endif
