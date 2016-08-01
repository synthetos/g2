# ----------------------------------------------------------------------------
# This file is part of the Synthetos G2 project.


# To compile:
#   make BOARD=sbv300

# You can also choose a CONFIG from boards.mk:
#   make CONFIG=sbv300



# Backward compatibility with old projects that use PLATFORM instead, but with a warning:
ifneq ("$(PLATFORM)","")
    $(warning Using PLATFORM value of $(PLATFORM) as BOARD.)
    $(warning Please swich to using BOARD on the command line.)
    BOARD = $(PLATFORM)
endif


##########
# BOARDs for use directly from the make command line (with default settings) or by CONFIGs.

ifeq ("$(BOARD)","sbv300")
    BASE_BOARD = sbv300
    DEVICE_DEFINES += MOTATE_BOARD="sbv300"
    DEVICE_DEFINES += SETTINGS_FILE=${SETTINGS_FILE}
endif



##########
# The general tinyg-due BASE_BOARD.

ifeq ("$(BASE_BOARD)","sbv300")
    _BOARD_FOUND = 1

    DEVICE_DEFINES += MOTATE_CONFIG_HAS_USBSERIAL=1

    FIRST_LINK_SOURCES += $(wildcard ${MOTATE_PATH}/Atmel_sam_common/*.cpp) $(wildcard ${MOTATE_PATH}/Atmel_sam3x/*.cpp)

    CHIP = SAM3X8E
    export CHIP
    CHIP_LOWERCASE = sam3x8e

    BOARD_PATH = ./board/sbv300
    SOURCE_DIRS += ${BOARD_PATH}

    PLATFORM_BASE = ${MOTATE_PATH}/platform/atmel_sam

    include $(PLATFORM_BASE).mk
endif
