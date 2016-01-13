# ----------------------------------------------------------------------------
# This file is part of the Motate project.

# These two Printrbot default BOARD to pBoard.

ifeq ("$(CONFIG)","PrintrbotPlus")
    ifeq ("$(BOARD)","NONE")
        BOARD=$(CONFIG)
        BASE_BOARD=pboard-a
    endif
    SETTINGS_FILE="settings_Printrbot_Plus.h"
    DEVICE_DEFINES += SETTINGS_FILE=${SETTINGS_FILE}
endif

ifeq ("$(CONFIG)","PrintrbotSimple")
    ifeq ("$(BOARD)","NONE")
        BOARD=$(CONFIG)
        BASE_BOARD=pboard-a
    endif
    SETTINGS_FILE="settings_Printrbot_Simple.h"
    DEVICE_DEFINES += SETTINGS_FILE=${SETTINGS_FILE}
endif


ifeq ("$(BOARD)","pboard-a")
    BASE_BOARD=pboard-a
endif


ifeq ("$(BASE_BOARD)","pboard-a")
    DEVICE_DEFINES += MOTATE_BOARD="pboard-a"
    DEVICE_DEFINES += MOTATE_CONFIG_HAS_USBSERIAL=1
    _BOARD_FOUND = 1

    FIRST_LINK_SOURCES += $(wildcard ${MOTATE_PATH}/Atmel_sam3xa/*.cpp)

    CHIP = SAM3X8C
    export CHIP
    CHIP_LOWERCASE = sam3x8c

    BOARD_PATH = ./board/pboard
    DEVICE_INCLUDE_DIRS += ${BOARD_PATH}

    PLATFORM_BASE = ${MOTATE_PATH}/platform/atmel_sam
    include $(PLATFORM_BASE).mk
endif
