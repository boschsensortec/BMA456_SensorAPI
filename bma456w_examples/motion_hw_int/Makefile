COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= motion_hw_int.c

API_LOCATION ?= ../..

COMMON_LOCATION ?= ..

C_SRCS += \
$(API_LOCATION)/bma4.c \
$(API_LOCATION)/bma456w.c \
$(COMMON_LOCATION)/common/common.c

INCLUDEPATHS += \
$(API_LOCATION) \
$(COMMON_LOCATION)/common

TARGET = MCU_APP30

include $(COINES_INSTALL_PATH)/coines.mk