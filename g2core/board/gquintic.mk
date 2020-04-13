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

ifeq ("$(BOARD)","gquintic-c")
    BASE_BOARD=gquintic
    DEVICE_DEFINES += MOTATE_BOARD="gquintic-c"
    DEVICE_DEFINES += SETTINGS_FILE=${SETTINGS_FILE}
endif

ifeq ("$(BOARD)","gquintic-d")
BASE_BOARD=gquintic
DEVICE_DEFINES += MOTATE_BOARD="gquintic-d"
DEVICE_DEFINES += SETTINGS_FILE=${SETTINGS_FILE}
endif

ifeq ("$(BOARD)","gquintic-g")
BASE_BOARD=gquintic
QUINTIC_N20=1
DEVICE_DEFINES += MOTATE_BOARD="gquintic-d"
DEVICE_DEFINES += SETTINGS_FILE=${SETTINGS_FILE}
endif

##########
# The general gquintic BASE_BOARD.
# NOTE: rev a is no longer supported!

ifeq ("$(BASE_BOARD)","gquintic")
    _BOARD_FOUND = 1

    DEVICE_DEFINES += MOTATE_CONFIG_HAS_USBSERIAL=1 ENABLE_TCM=1

    FIRST_LINK_SOURCES += $(sort $(wildcard ${MOTATE_PATH}/Atmel_sam_common/*.cpp)) $(sort $(wildcard ${MOTATE_PATH}/Atmel_sams70/*.cpp) $(wildcard ${BOARD_PATH}/*.cpp))

    ifeq ("$(QUINTIC_N20)","1")
        CHIP = SAMS70N20
        export CHIP
        CHIP_LOWERCASE = sams70n20
        DEVICE_DEFINES += QUINTIC_N20=1
    else
        CHIP = SAMS70N19
        export CHIP
        CHIP_LOWERCASE = sams70n19
    endif

    BOARD_PATH = ./board/gquintic
    SOURCE_DIRS += ${BOARD_PATH} device/trinamic device/step_dir_hobbyservo device/max31865 device/i2c_eeprom device/i2c_multiplexer device/i2c_as5601 device/esc_spindle device/laser_toolhead

    PLATFORM_BASE = ${MOTATE_PATH}/platform/atmel_sam
    include $(PLATFORM_BASE).mk
endif
