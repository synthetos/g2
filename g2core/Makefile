#
# Makefile
#
# Copyright (c) 2012 - 2016 Robert Giseburt
# Copyright (c) 2013 - 2016 Alden S. Hart Jr.
#
# This file is part of the g2core project.
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

##############################################################################################
# Start of default section
#

PROJECT = g2core

MOTATE_PATH ?= ../Motate/MotateProject/motate

# List *project-specific* files that must be linked to first in FIRST_LINK_SOURCES.
# For device-specific FIRST_LINK_SOURCES, use DEVICE_FIRST_LINK_SOURCES (in the
# device-specific section, below.)
#
# DO include a ./ at the beginning of paths, or they won't match the glob
# results and will cause linkers errors.
FIRST_LINK_SOURCES = ./main.cpp

# Define optimisation level here
#OPTIMIZATION ?= 0
OPTIMIZATION ?= s
#OPTIMIZATION ?= 3

DEBUG ?= 0
#DEBUG ?= 1 # Use this to turn on some debugging functions
#DEBUG ?= 2 # Use this for DEBUG=1 + some debug traps that need a HW debugger

# SETTINGS_FILE may get overriden by the BOARD settings in the appropriate board/*.mk files
SETTINGS_FILE ?= settings_default.h

NEEDS_PRINTF_FLOAT=1

# Now invoke the Motate compile system
include $(MOTATE_PATH)/Motate.mk

ifeq ($(DEBUG),0)
	DEVICE_DEFINES += DEBUG=0 IN_DEBUGGER=0
endif
ifeq ($(DEBUG),1)
	DEVICE_DEFINES += DEBUG=1 IN_DEBUGGER=0
endif
ifeq ($(DEBUG),2)
	DEVICE_DEFINES += DEBUG=1 IN_DEBUGGER=1
endif
ifeq ($(DEBUG),3)
	DEVICE_DEFINES += DEBUG=1 IN_DEBUGGER=1 DEBUG_SEMIHOSTING=1
endif

# *** EOF ***
