# ----------------------------------------------------------------------------
# This file is part of the Synthetos G2 project.


# To compile:
#   make BOARD=gquintic-a

# You can also choose a CONFIG from boards.mk:
#   make CONFIG=PrintrbotPlus BOARD=gquintic-a


##########
# BOARDs for use directly from the make command line (with default settings) or by CONFIGs.

ifeq ("$(BOARD)","gquintic-a")
    BASE_BOARD=gquintic
    DEVICE_DEFINES += MOTATE_BOARD="gquintic-a"
    DEVICE_DEFINES += SETTINGS_FILE=${SETTINGS_FILE}
endif


##########
# The general pboard-a BASE_BOARD.

ifeq ("$(BASE_BOARD)","gquintic")
    _BOARD_FOUND = 1

    DEVICE_DEFINES += MOTATE_CONFIG_HAS_USBSERIAL=1

    FIRST_LINK_SOURCES += $(wildcard ${MOTATE_PATH}/Atmel_sam_common/*.cpp) $(wildcard ${MOTATE_PATH}/Atmel_sam4e/*.cpp)

    CHIP = SAM4E8C
    export CHIP
    CHIP_LOWERCASE = sam4e8c

    BOARD_PATH = ./board/gquintic
    SOURCE_DIRS += ${BOARD_PATH} device/trinamic

    PLATFORM_BASE = ${MOTATE_PATH}/platform/atmel_sam
    include $(PLATFORM_BASE).mk
endif
