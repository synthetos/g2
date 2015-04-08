# ----------------------------------------------------------------------------
# This file is part of the Motate project.

ifneq ("$(PLATFORM)","")
    $(warning Using PLATFORM value of $(PLATFORM) as BOARD.)
    $(warning Please swich to using BOARD on the command line.)
    BOARD = $(PLATFORM)
endif

BOARD ?= gShield


ifeq ("$(BOARD)","gShield")
    # This is a due with a Synthetos gShield. We'll use the Due platform, but set defines
    # for the code to get the pinout right.

    # Note: we call it "tinyg-due" instead of "due" since the Motate built-in provides
    # a "due" BASE_BOARD.
    BASE_BOARD = tinyg-due
    DEVICE_DEFINES += MOTATE_BOARD="gShield"
    DEVICE_DEFINES += MOTATE_CONFIG_HAS_USBSERIAL=1
    DEVICE_DEFINES += SETTINGS_FILE=${SETTINGS_FILE}
endif

ifeq ("$(BOARD)","shopbotShield")
    # This is a due with a shopbot shield. We'll use the Due platform, but set defines
    # for the code to get the pinout right.

    BASE_BOARD = tinyg-due
    DEVICE_DEFINES += MOTATE_BOARD="shopbotShield"
    DEVICE_DEFINES += MOTATE_CONFIG_HAS_USBSERIAL=1
    DEVICE_DEFINES += SETTINGS_FILE=${SETTINGS_FILE}
endif




ifeq ("$(BASE_BOARD)","tinyg-due")
    _BOARD_FOUND = 1

    FIRST_LINK_SOURCES += $(wildcard ${MOTATE_PATH}/Atmel_sam3xa/*.cpp)

    CHIP = SAM3X8E
    export CHIP
    CHIP_LOWERCASE = sam3x8e

    # Note: we call it "tinyg-due" instead of "due" since the Motate built-in provides
    # a "due" BASE_BOARD.
    BOARD_PATH = ./board/ArduinoDue
    DEVICE_INCLUDE_DIRS += $(BOARD_PATH)

    PLATFORM_BASE = ${MOTATE_PATH}/platform/atmel_sam

    include $(PLATFORM_BASE).mk
endif
