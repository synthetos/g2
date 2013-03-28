# 
# Makefile
# 
# Copyright (c) 2012 Robert Giseburt
# 
# This file is part of the TinyG3 project.
# 
# This file is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This file is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License
# along with the Motate Library.  If not, see <http://www.gnu.org/licenses/>.
# 

##############################################################################################
# Start of default section
#

PROJECT  = TinyG3

CHIP = SAM3X8E

SOURCE_DIRS = system/source \
              arduino \
              variants \
              cmsis/src \
              .

SOURCES     := $(foreach dir, $(SOURCE_DIRS), $(wildcard $(dir)/*.c) )
SOURCES_CXX := $(foreach dir, $(SOURCE_DIRS), $(wildcard $(dir)/*.cpp) )

# List ASM source files here
ASRC =

ARDUINO_SYSTEM_ROOT := ../../Arduino/hardware/arduino/sam/system
LIBSAM_ROOT = $(ARDUINO_SYSTEM_ROOT)/libsam
CMSIS_ROOT = $(ARDUINO_SYSTEM_ROOT)/CMSIS
LINKER_SCRIPT_flash = $(ARDUINO_SYSTEM_ROOT)/../variants/arduino_due_x/linker_scripts/gcc/flash.ld
LINKER_SCRIPT_sram = $(ARDUINO_SYSTEM_ROOT)/../variants/arduino_due_x/linker_scripts/gcc/sram.ld

# List all user directories here
UINCDIR = motate\
					./system/include \
					$(LIBSAM_ROOT) \
					$(SOURCE_DIRS)

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS = 

# Define optimisation level here
#OPTIMIZATION = -O0
OPTIMIZATION = -Os

CFLAGS :=
CPPFLAGS :=

INCLUDES  = $(patsubst %,-I%,$(DINCDIR) $(UINCDIR))
LIBDIR  = $(patsubst %,-L%,$(DLIBDIR) $(ULIBDIR))



#
# End of default section
##############################################################################################

##############################################################################################


include cmsis_sam.mk


# *** EOF ***