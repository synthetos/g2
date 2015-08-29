# ----------------------------------------------------------------------------
# This file is part of the Motate project.

ifeq ("$(BOARD)","g2ref-a")
    BASE_BOARD=g2-g2ref
    DEVICE_DEFINES += MOTATE_BOARD="g2ref-a"
    DEVICE_DEFINES += MOTATE_CONFIG_HAS_USBSERIAL=1
endif

ifeq ("$(BOARD)","PrintrbotPlus")
    BASE_BOARD=g2-g2ref
    SETTINGS_FILE="settings_Printrbot_Plus.h"
    DEVICE_DEFINES += MOTATE_BOARD="g2ref-a" SETTINGS_FILE=${SETTINGS_FILE}
endif

ifeq ("$(BOARD)","PrintrbotSimple")
    BASE_BOARD=g2-g2ref
    SETTINGS_FILE="settings_Printrbot_Simple.h"
    DEVICE_DEFINES += MOTATE_BOARD="g2ref-a" SETTINGS_FILE=${SETTINGS_FILE}
endif

# We call it g2-g2ref here to distinguish between the "g2Ref" in Motate
ifeq ("$(BASE_BOARD)","g2-g2ref")
    _BOARD_FOUND = 1

    FIRST_LINK_SOURCES += $(wildcard ${MOTATE_PATH}/Atmel_sam3xa/*.cpp)

    CHIP = SAM3X8C
    export CHIP
    CHIP_LOWERCASE = sam3x8c

    BOARD_PATH = ./board/g2ref
    DEVICE_INCLUDE_DIRS += ${BOARD_PATH}

    PLATFORM_BASE = ${MOTATE_PATH}/platform/atmel_sam
    include $(PLATFORM_BASE).mk
endif
