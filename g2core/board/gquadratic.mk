# ----------------------------------------------------------------------------
# This file is part of the Synthetos g2core project


# To compile:
#   make BOARD=gquadratic-a

# You can also choose a CONFIG from boards.mk:
#   make CONFIG=PrintrbotPlus BOARD=gquadratic-a


##########
# BOARDs for use directly from the make command line (with default settings) or by CONFIGs.

ifeq ("$(BOARD)","gquadratic-b")
    BASE_BOARD=gquadratic
    DEVICE_DEFINES += MOTATE_BOARD="gquadratic-b"
    DEVICE_DEFINES += SETTINGS_FILE=${SETTINGS_FILE}
endif

ifeq ("$(BOARD)","gquadratic-c")
    BASE_BOARD=gquadratic
    DEVICE_DEFINES += MOTATE_BOARD="gquadratic-c"
    DEVICE_DEFINES += SETTINGS_FILE=${SETTINGS_FILE}
endif


##########
# The general gquadratic BASE_BOARD.

ifeq ("$(BASE_BOARD)","gquadratic")
    _BOARD_FOUND = 1

    DEVICE_DEFINES += MOTATE_CONFIG_HAS_USBSERIAL=1 ENABLE_TCM=1

    FIRST_LINK_SOURCES += $(sort $(wildcard ${MOTATE_PATH}/Atmel_sam_common/*.cpp)) $(sort $(wildcard ${MOTATE_PATH}/Atmel_sams70/*.cpp) $(wildcard ${BOARD_PATH}/*.cpp))

    CHIP = SAMS70N19
    export CHIP
    CHIP_LOWERCASE = sams70n19

    BOARD_PATH = ./board/gquadratic
    SOURCE_DIRS += ${BOARD_PATH} device/trinamic device/step_dir_hobbyservo device/max31865 device/neopixel device/esc_spindle device/laser_toolhead

    PLATFORM_BASE = ${MOTATE_PATH}/platform/atmel_sam
    include $(PLATFORM_BASE).mk
endif
