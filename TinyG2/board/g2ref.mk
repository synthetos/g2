# ----------------------------------------------------------------------------
# This file is part of the Motate project.

ifeq ("$(BOARD)","g2ref-a")
    BASE_BOARD=g2-g2ref
    DEVICE_DEFINES += MOTATE_BOARD="g2ref-a"
    DEVICE_DEFINES += MOTATE_CONFIG_HAS_USBSERIAL=1
endif

# MOVED: PrintrbotPlus and PrintrbotSimple are defined in pBoard.mk now,
#        and have changed from BOARD= to CONFIG=
#
# To compile for example: make CONFIG=PrintrbotPlus BOARD=g2ref-a

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
