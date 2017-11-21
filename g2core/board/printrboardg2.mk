# ----------------------------------------------------------------------------
# This file is part of the Synthetos g2core project


# To compile:
#   make BOARD=printrboardG2v3

# You can also choose a CONFIG from boards.mk:
#   make CONFIG=PrintrbotPlus BOARD=printrboardG2v3


##########
# BOARDs for use directly from the make command line (with default settings) or by CONFIGs.

ifeq ("$(BOARD)","printrboardG2v3")
    BASE_BOARD=printrboardg2
    DEVICE_DEFINES += MOTATE_BOARD="printrboardG2v3"
    DEVICE_DEFINES += SETTINGS_FILE=${SETTINGS_FILE}
endif

##########
# The general pboard-a BASE_BOARD.

ifeq ("$(BASE_BOARD)","printrboardg2")
    _BOARD_FOUND = 1

    DEVICE_DEFINES += MOTATE_CONFIG_HAS_USBSERIAL=1

    FIRST_LINK_SOURCES += $(sort $(wildcard ${MOTATE_PATH}/Atmel_sam_common/*.cpp)) $(sort $(wildcard ${MOTATE_PATH}/Atmel_sam3x/*.cpp))

    CHIP = SAM3X8C
    export CHIP
    CHIP_LOWERCASE = sam3x8c

    BOARD_PATH = ./board/printrboardg2
    SOURCE_DIRS += ${BOARD_PATH} device/step_dir_driver device/neopixel

    PLATFORM_BASE = ${MOTATE_PATH}/platform/atmel_sam
    include $(PLATFORM_BASE).mk
endif
