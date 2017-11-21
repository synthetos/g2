# ----------------------------------------------------------------------------
# This file is part of the Synthetos g2core project


# To compile:
#   make BOARD=gquintic-a

# You can also choose a CONFIG from boards.mk:
#   make CONFIG=PrintrbotPlus BOARD=gquintic-b


##########
# BOARDs for use directly from the make command line (with default settings) or by CONFIGs.

ifeq ("$(BOARD)","gquintic-b")
    BASE_BOARD=gquintic
    DEVICE_DEFINES += MOTATE_BOARD="gquintic-b"
    DEVICE_DEFINES += SETTINGS_FILE=${SETTINGS_FILE}
endif


##########
# The general gquintic BASE_BOARD.
# NOTE: rev a is no longer supported!

ifeq ("$(BASE_BOARD)","gquintic")
    _BOARD_FOUND = 1

    DEVICE_DEFINES += MOTATE_CONFIG_HAS_USBSERIAL=1

    FIRST_LINK_SOURCES += $(sort $(wildcard ${MOTATE_PATH}/Atmel_sam_common/*.cpp)) $(sort $(wildcard ${MOTATE_PATH}/Atmel_sams70/*.cpp))

    CHIP = SAMS70N19
    export CHIP
    CHIP_LOWERCASE = sams70n19

    BOARD_PATH = ./board/gquintic
    SOURCE_DIRS += ${BOARD_PATH} device/trinamic

    PLATFORM_BASE = ${MOTATE_PATH}/platform/atmel_sam
    include $(PLATFORM_BASE).mk
endif
