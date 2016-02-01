# ----------------------------------------------------------------------------
# This file is part of the Synthetos G2 project.

# To compile a specific CONFIG:
#   make CONFIG=PrintrbotPlus

# Note how the BOARD is defaulted if not provided.
# To choose a CONFIG but apply it to a different BOARD:
#   make CONFIG=PrintrbotPlus BOARD=g2ref-a


##########
# V9-based configs:

ifeq ("$(CONFIG)","ShapeokoDualY")
    ifeq ("$(BOARD)","NONE")
        BOARD=G2v9k
    endif
    SETTINGS_FILE="settings_shapeoko2.h"
endif

ifeq ("$(CONFIG)","Othermill")
    ifeq ("$(BOARD)","NONE")
        BOARD=G2v9k
    endif
    SETTINGS_FILE="settings_othermill.h"
endif

ifeq ("$(CONFIG)","ProbotixV90")
    ifeq ("$(BOARD)","NONE")
        BOARD=G2v9k
    endif
    SETTINGS_FILE="settings_probotixV90.h"
endif


##########
# PrintrBot configs:

ifeq ("$(CONFIG)","PrintrbotPlus")
    ifeq ("$(BOARD)","NONE")
        BOARD=pboard-a
    endif
    SETTINGS_FILE="settings_Printrbot_Plus.h"
endif

ifeq ("$(CONFIG)","PrintrbotSimple")
    ifeq ("$(BOARD)","NONE")
        BOARD=pboard-a
    endif
    SETTINGS_FILE="settings_Printrbot_Simple.h"
endif


##########
# Ultimaker configs:

ifeq ("$(CONFIG)","UltimakerTests")
    ifeq ("$(BOARD)","NONE")
        BOARD=g2ref-a
    endif
    SETTINGS_FILE="settings_Ultimaker_Rob_v9h.h"
endif

ifeq ("$(CONFIG)","UltimakerV9h")
    ifeq ("$(BOARD)","NONE")
        BOARD=g2ref-a
    endif
    SETTINGS_FILE="settings_Ultimaker.h"
endif
