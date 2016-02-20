# ----------------------------------------------------------------------------
# This file is part of the Synthetos G2 project.


# To compile:
#   make BOARD=g2v9k

# You can also choose a CONFIG from g2-configs.mk:
#   make CONFIG=ShapeokoDualY BOARD=G2v9k



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

    FIRST_LINK_SOURCES += $(wildcard ${MOTATE_PATH}/Atmel_sam3xa/*.cpp)

    # Set CHIP and export it for GDB to see
    CHIP = SAM3X8C
    export CHIP
    CHIP_LOWERCASE = sam3x8c

    BOARD_PATH = ./board/g2v9
    DEVICE_INCLUDE_DIRS += $(BOARD_PATH)

    PLATFORM_BASE = ${MOTATE_PATH}/platform/atmel_sam

    include $(PLATFORM_BASE).mk
endif
