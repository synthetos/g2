# ----------------------------------------------------------------------------
# This file is part of the Synthetos G2 project.
#
# Copyright (c) 2016 Robert Giseburt
# Copyright (c) 2016 Alden S. Hart Jr.
#
# This file ("the software") is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License, version 2 as published by the
# Free Software Foundation. You should have received a copy of the GNU General Public
# License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
#
# THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
# WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
# OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
# SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
# OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#

# To compile a specific CONFIG:
#   make CONFIG=PrintrbotPlus

# Note how the BOARD is defaulted if not provided.
# To choose a CONFIG but apply it to a different BOARD:
#   make CONFIG=PrintrbotPlus BOARD=g2ref-a


##########
# V9-based configs:

ifeq ("$(CONFIG)","ShapeokoDualY")
    ifeq ("$(BOARD)","NONE")
        BOARD=g2v9k
    endif
    SETTINGS_FILE="settings_shapeoko2.h"
endif

ifeq ("$(CONFIG)","Othermill")
    ifeq ("$(BOARD)","NONE")
        BOARD=g2v9k
    endif
#    SETTINGS_FILE="settings_othermill.h"
    SETTINGS_FILE="settings_othermill_test.h"
endif

ifeq ("$(CONFIG)","ProbotixV90")
    ifeq ("$(BOARD)","NONE")
        BOARD=g2v9k
    endif
    SETTINGS_FILE="settings_probotixV90.h"
endif

ifeq ("$(CONFIG)","Zen7x12")
    ifeq ("$(BOARD)","NONE")
        BOARD=g2v9k
    endif
    SETTINGS_FILE="settings_zen7x12.h"
endif

ifeq ("$(CONFIG)","Makeblock")
    ifeq ("$(BOARD)","NONE")
        BOARD=g2v9k
    endif
    SETTINGS_FILE="settings_makeblock.h"
endif

ifeq ("$(CONFIG)","TestV9")
    ifeq ("$(BOARD)","NONE")
        BOARD=g2v9k
    endif
    SETTINGS_FILE="settings_test.h"
endif

ifeq ("$(CONFIG)","TestQuintic")
    ifeq ("$(BOARD)","NONE")
        BOARD=gquintic-b
    endif
    SETTINGS_FILE="settings_test.h"
endif


ifeq ("$(CONFIG)","TestQuadratic")
    ifeq ("$(BOARD)","NONE")
        BOARD=gquadratic-b
    endif
    SETTINGS_FILE="settings_test.h"
endif

##########
# Shopbot configs:

ifeq ("$(CONFIG)","sbv300")
    ifeq ("$(BOARD)","NONE")
        BOARD=sbv300
    endif
    SETTINGS_FILE="settings_shopbot_sbv300.h"
endif


##########
# PrintrBot configs:

ifeq ("$(CONFIG)","PrintrbotPlus")
    ifeq ("$(BOARD)","NONE")
        BOARD=printrboardG2v3
    endif
    SETTINGS_FILE="settings_Printrbot_Plus.h"
endif

ifeq ("$(CONFIG)","PrintrbotSimple1403")
    ifeq ("$(BOARD)","NONE")
        BOARD=printrboardG2v3
    endif
    SETTINGS_FILE="settings_Printrbot_Simple_1403.h"
endif

ifeq ("$(CONFIG)","PrintrbotSimple1608")
    ifeq ("$(BOARD)","NONE")
        BOARD=printrboardG2v3
    endif
    SETTINGS_FILE="settings_Printrbot_Simple_1608.h"
endif

ifeq ("$(CONFIG)","PrintrbotPlay")
    ifeq ("$(BOARD)","NONE")
        BOARD=printrboardG2v3
    endif
    SETTINGS_FILE="settings_Printrbot_Play.h"
endif

##########
# Ultimaker configs:

ifeq ("$(CONFIG)","Ultimakerv9k")
    ifeq ("$(BOARD)","NONE")
        BOARD=g2v9k
    endif
    SETTINGS_FILE="settings_ultimaker.h"
endif

##########
# EMSL configs:

ifeq ("$(CONFIG)","WaterColorBotv2")
    ifeq ("$(BOARD)","NONE")
        BOARD=gquadratic-b
    endif
    SETTINGS_FILE="settings_watercolorbot_v2.h"
endif

ifeq ("$(CONFIG)","EggBot")
    ifeq ("$(BOARD)","NONE")
        BOARD=gquadratic-b
    endif
    SETTINGS_FILE="settings_eggbot.h"
endif


include $(wildcard ./board/$(STAR).mk)

