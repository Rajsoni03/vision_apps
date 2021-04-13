ifeq ($(TARGET_CPU),x86_64)

include $(PRELUDE)
TARGET      := app_utils_opengl
TARGETTYPE  := library
IDIRS       += $(VISION_APPS_PATH)
IDIRS       += $(GLM_PATH)/
CFLAGS      += --std=c++14 -D_HOST_EMULATION -pedantic -fPIC -w -c -g
CFLAGS      += -Wno-sign-compare

CPPSOURCES  := app_gl_egl_utils_pc.cpp

SKIPBUILD=0

include $(FINALE)

endif

