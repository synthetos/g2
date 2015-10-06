# ----------------------------------------------------------------------------
# This file is part of the Motate project.

ifeq ("$(BOARD)","G2v9i")

    BASE_BOARD=G2v9
    DEVICE_DEFINES += MOTATE_BOARD="G2v9i" SETTINGS_FILE=${SETTINGS_FILE}

endif

ifeq ("$(BOARD)","G2v9k")

    BASE_BOARD=G2v9
    DEVICE_DEFINES += MOTATE_BOARD="G2v9k" SETTINGS_FILE=${SETTINGS_FILE}

endif

#### Example config-specific additions:

ifeq ("$(BOARD)","UltimakerTests")

    BASE_BOARD=G2v9
    SETTINGS_FILE="settings_Ultimaker_Rob_v9h.h"
    DEVICE_DEFINES += MOTATE_BOARD="G2v9k" SETTINGS_FILE=${SETTINGS_FILE}

endif

ifeq ("$(BOARD)","UltimakerV9h")

    BASE_BOARD=G2v9
    SETTINGS_FILE="settings_Ultimaker.h"
    DEVICE_DEFINES += MOTATE_BOARD="G2v9k" SETTINGS_FILE=${SETTINGS_FILE}

endif

ifeq ("$(BOARD)","ShapeokoDualY")

    BASE_BOARD=G2v9
    SETTINGS_FILE="settings_shapeoko2.h"
    DEVICE_DEFINES += MOTATE_BOARD="G2v9k" SETTINGS_FILE=${SETTINGS_FILE}

endif

ifeq ("$(BOARD)","Othermill")

    BASE_BOARD=G2v9
    SETTINGS_FILE="settings_othermill.h"
    DEVICE_DEFINES += MOTATE_BOARD="G2v9k" SETTINGS_FILE=${SETTINGS_FILE}

endif

ifeq ("$(BOARD)","ProbotixV90")

    BASE_BOARD=G2v9
    SETTINGS_FILE="settings_probotixV90.h"
    DEVICE_DEFINES += MOTATE_BOARD="G2v9k" SETTINGS_FILE=${SETTINGS_FILE}

endif


##########

ifeq ("$(BASE_BOARD)","G2v9")
    _BOARD_FOUND = 1

    FIRST_LINK_SOURCES += $(wildcard ${MOTATE_PATH}/Atmel_sam3xa/*.cpp)

    # Set CHIP and export it for GDB to see
    CHIP = SAM3X8C
    export CHIP
    CHIP_LOWERCASE = sam3x8c

    BOARD_PATH = ./board/G2v9
    DEVICE_INCLUDE_DIRS += $(BOARD_PATH)

    PLATFORM_BASE = ${MOTATE_PATH}/platform/atmel_sam

    include $(PLATFORM_BASE).mk
endif
