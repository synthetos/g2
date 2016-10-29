# ----------------------------------------------------------------------------
# This file is part of the Synthetos g2core project


# To compile:
#   make BOARD=g2v9k

# You can also choose a CONFIG from boards.mk:
#   make CONFIG=ShapeokoDualY BOARD=g2v9k



##########
# BOARDs for use directly from the make command line (with default settings) or by the CONFIGS above.

ifeq ("$(BOARD)","g2v9i")
    BASE_BOARD=g2v9
    DEVICE_DEFINES += MOTATE_BOARD="G2v9i"
    DEVICE_DEFINES += SETTINGS_FILE=${SETTINGS_FILE}
endif

ifeq ("$(BOARD)","g2v9k")
    BASE_BOARD=g2v9
    DEVICE_DEFINES += MOTATE_BOARD="G2v9k"
    DEVICE_DEFINES += SETTINGS_FILE=${SETTINGS_FILE}
endif


##########
# The general g2v9 BASE_BOARD.

ifeq ("$(BASE_BOARD)","g2v9")
    _BOARD_FOUND = 1

    FIRST_LINK_SOURCES += $(sort $(wildcard ${MOTATE_PATH}/Atmel_sam_common/*.cpp)) $(sort $(wildcard ${MOTATE_PATH}/Atmel_sam3x/*.cpp))

    # Set CHIP and export it for GDB to see
    CHIP = SAM3X8C
    export CHIP
    CHIP_LOWERCASE = sam3x8c

    BOARD_PATH = ./board/G2v9
    SOURCE_DIRS += ${BOARD_PATH} device/step_dir_driver

    PLATFORM_BASE = ${MOTATE_PATH}/platform/atmel_sam

    include $(PLATFORM_BASE).mk
endif
