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

SOURCE_DIRS = . arduino variants
FIRST_LINK_SOURCES := arduino/syscalls_sam3.c

# List all user directories here
USER_INCLUDE_DIRS := motate $(SOURCE_DIRS)

# List the user directory to look for the libraries here
USER_LIB_DIRS =

# List all user libraries here
USER_LIBS =

# Define optimisation level here
#OPTIMIZATION = -Os
OPTIMIZATION = -O0

CFLAGS   :=
CPPFLAGS :=

# List ASM source files here
ASRC =

# ---------------------------------------------------------------------------------------
# C Flags (NOT CPP Flags)

# Configure warnings:
CFLAGS += -Wall -Wchar-subscripts -Wcomment -Wformat=2 -Wimplicit-int
CFLAGS += -Werror-implicit-function-declaration -Wmain -Wparentheses
CFLAGS += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
CFLAGS += -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef
CFLAGS += -Wshadow -Wpointer-arith -Wbad-function-cast -Wwrite-strings
CFLAGS += -Wsign-compare -Waggregate-return
#CFLAGS += -Wstrict-prototypes
#CFLAGS += -Wmissing-prototypes -Wmissing-declarations
CFLAGS += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
CFLAGS += -Wpacked -Wredundant-decls -Wnested-externs -Winline -Wlong-long
CFLAGS += -Wunreachable-code
#CFLAGS += -Wcast-align
#CFLAGS += -Wmissing-noreturn
#CFLAGS += -Wconversion

# To reduce application size use only integer printf function.
CFLAGS += -Dprintf=iprintf

# ---------------------------------------------------------------------------------------
# CPP Flags

CPPFLAGS += -Wall -Wchar-subscripts -Wcomment -Wformat=2
CPPFLAGS += -Wmain -Wparentheses -Wunreachable-code
#CPPFLAGS += -Wcast-align
CPPFLAGS += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
CPPFLAGS += -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef
CPPFLAGS += -Wshadow -Wpointer-arith -Wwrite-strings
#CPPFLAGS += -Wsign-compare -Waggregate-return -Wmissing-declarations
CPPFLAGS += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
CPPFLAGS += -Wpacked -Wredundant-decls -Winline -Wlong-long
#CPPFLAGS += -Wmissing-noreturn
#CPPFLAGS += -Wconversion

# To reduce application size use only integer printf function.
CPPFLAGS += -Dprintf=iprintf

# Default is to compile more quielty.
# To show actual commands: make [options] VERBOSE=1
VERBOSE ?= 0
ifeq ($(VERBOSE),0)
QUIET := @
else
QUIET :=
endif

#
# End of default section
##############################################################################################

##############################################################################################
# Start of platforms section
#

PLATFORM ?= due

ifeq ("$(PLATFORM)","due")
    CHIP = SAM3X8E

    PLATFORM_BASE = platform/atmel_sam

    CMSIS_ROOT = CMSIS
    SOURCE_DIRS += $(PLATFORM_BASE)/libsam/source
    DEVICE_INCLUDE_DIRS += $(PLATFORM_BASE)/libsam \
                           $(PLATFORM_BASE)/libsam/include

#    LINKER_SCRIPT_flash = $(PLATFORM_BASE)/linker_scripts/gcc/flash.ld
#    LINKER_SCRIPT_sram  = $(PLATFORM_BASE)/linker_scripts/gcc/sram.ld

    include $(PLATFORM_BASE).mk
endif

#
# End of platforms section
##############################################################################################

##############################################################################################
# Actual compilation
#

# Output directories
BIN = bin/$(CHIP)
OBJ = build/$(CHIP)
DEPDIR = $(OBJ)/dep

# Output file basename
OUTPUT_BIN = $(BIN)/$(PROJECT)_$(CHIP)

#-------------------------------------------------------------------------------
#		Tools
#-------------------------------------------------------------------------------

# Compilation tools
CC      = $(CROSS_COMPILE)gcc
CXX     = $(CROSS_COMPILE)g++
LD      = $(CROSS_COMPILE)ld
AR      = $(CROSS_COMPILE)ar
SIZE    = $(CROSS_COMPILE)size
STRIP   = $(CROSS_COMPILE)strip
OBJCOPY = $(CROSS_COMPILE)objcopy
GDB     = $(CROSS_COMPILE)gdb
NM      = $(CROSS_COMPILE)nm
RM      = rm


# ---------------------------------------------------------------------------------------
# C Flags (NOT CPP flags)

CFLAGS += --param max-inline-insns-single=500 -mcpu=cortex-m3 -mthumb -mlong-calls -ffunction-sections -fdata-sections -nostdlib -std=gnu99


# ---------------------------------------------------------------------------------------
# CPP Flags

CPPFLAGS += --param max-inline-insns-single=500 -mcpu=cortex-m3 -mthumb -mlong-calls -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions


# ---------------------------------------------------------------------------------------
# ASM Flags

ASFLAGS =


# ---------------------------------------------------------------------------------------
# Linker Flags

LDFLAGS = $(LIBS) $(USER_LIBS) -mthumb -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--entry=Reset_Handler -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align -Wl,--warn-unresolved-symbols
LDFLAGS += -nostartfiles
#LD_OPTIONAL=-Wl,--print-gc-sections -Wl,--stats


#-------------------------------------------------------------------------------
#		Files
#-------------------------------------------------------------------------------

# Directories where source files can be found

SOURCES     = $(foreach dir, $(SOURCE_DIRS), $(wildcard $(dir)/*.c) )
SOURCES_CXX = $(foreach dir, $(SOURCE_DIRS), $(wildcard $(dir)/*.cpp) )

OBJECTS     := $(addsuffix .o,$(basename $(SOURCES)))
CXX_OBJECTS := $(addsuffix .o,$(basename $(SOURCES_CXX)))
FIRST_LINK_OBJECTS = $(addsuffix .o,$(basename $(FIRST_LINK_SOURCES)))

# Append OBJ and BIN directories to output filename
OUTPUT := $(BIN)/$(OUTPUT_BIN)

INCLUDES = $(patsubst %,-I%,$(DEVICE_INCLUDE_DIRS) $(USER_INCLUDE_DIRS))
LIBDIR   = $(patsubst %,-L%,$(DEVICE_LIB_DIRS) $(USER_LIB_DIRS))

CFLAGS   += $(OPTIMIZATION) $(INCLUDES)
CPPFLAGS += $(OPTIMIZATION) $(INCLUDES)
ASFLAGS  += -g $(OPTIMIZATION) $(INCLUDES) -D__ASSEMBLY__

#-------------------------------------------------------------------------------
#		Rules
#-------------------------------------------------------------------------------


all: $(BIN) $(OBJ) $(MEMORIES)

REQUIRED_DIRS := $(BIN) $(OBJ) $(DEPDIR)

MK_DIRS =   $(shell                              \
              for d in $(REQUIRED_DIRS);         \
              do                                 \
                [[ -d $$d ]] || mkdir -p $$d;    \
              done)

define RULES

OUTDIR = $(OBJ)/$(1)
REQUIRED_DIRS += $(OUTDIR)
OBJECTS_$(1) := $(addprefix $$(OUTDIR)/, $(OBJECTS))
CXX_OBJECTS_$(1) := $(addprefix $$(OUTDIR)/, $(CXX_OBJECTS))
ASM_OBJECTS_$(1) := $(addprefix $$(OUTDIR)/, $(ASM_OBJECTS))
LINKER_SCRIPT_$(1) ?= $(PLATFORM_BASE)/linker_scripts/gcc/$(1).ld
ABS_LINKER_SCRIPT_$(1) = $(abspath $$(LINKER_SCRIPT_$(1)))
FIRST_LINK_OBJECTS_$(1) := $(addprefix $$(OUTDIR)/, $(FIRST_LINK_OBJECTS))

# Generate dependency information
DEPFLAGS_$(1) = -MMD -MF $(OBJ)/dep/$$(@F)_$(1).d -MT $(subst $(OUTDIR),$(OBJ)/$(1),$$@)

$(1): $$(OUTDIR)/core.a | $(MK_DIRS)
	@echo "Linking ($(1)) $$<"
	@echo "Using linker script: $$(ABS_LINKER_SCRIPT_$(1))"
	$(QUIET)$(CXX) $(LIB_PATH) -T"$$(ABS_LINKER_SCRIPT_$(1))" -Wl,-Map,"$(OUTPUT_BIN)_$$@.map" -o "$(OUTPUT_BIN)_$$@.elf" $(LDFLAGS) $(LD_OPTIONAL) $(LIBS) -Wl,--start-group $$(FIRST_LINK_OBJECTS_$(1)) $$< -Wl,--end-group
	@echo "Exporting symbols $(OUTPUT_BIN)_$$@.elf.txt"
	$(QUIET)$(NM) "$(OUTPUT_BIN)_$$@.elf" >"$(OUTPUT_BIN)_$$@.elf.txt"
	@echo "Making binary $(OUTPUT_BIN)_$$@.bin"
	$(QUIET)$(OBJCOPY) -O binary "$(OUTPUT_BIN)_$$@.elf" "$(OUTPUT_BIN)_$$@.bin"

	# @echo "--- SIZE INFO ---"
	# $(QUIET)$(SIZE) $$^ "$(OUTPUT_BIN)_$$@.elf"


$$(OUTDIR)/core.a: $$(OUTDIR)/core.a( $$(ASM_OBJECTS_$(1)) $$(CXX_OBJECTS_$(1)) $$(OBJECTS_$(1)) ) | $(BIN) $(OBJ)

$$(CXX_OBJECTS_$(1)): $$(OUTDIR)/%.o: %.cpp | $(MK_DIRS)
	$(QUIET)if [[ ! -d `dirname $$@` ]]; then mkdir -p `dirname $$@`; fi
	@echo "Compiling cpp ($(1)) $$< -> $$@"
	$(QUIET)$(CXX) $(CPPFLAGS) $$(DEPFLAGS_$(1)) -D$(1) -xc++ -c -o $$@ $$<

$$(OBJECTS_$(1)): $$(OUTDIR)/%.o: %.c | $(MK_DIRS)
	$(QUIET)if [[ ! -d `dirname $$@` ]]; then mkdir -p `dirname $$@`; fi
	@echo "Compiling c ($(1)) $$< -> $$@"
	$(QUIET)$(CC) $(CFLAGS) $$(DEPFLAGS_$(1)) -D$(1) -c -o $$@ $$<

$$(ASM_OBJECTS_$(1)): $$(OUTDIR)/%.o: %.S | $(MK_DIRS)
	$(QUIET)if [[ ! -d `dirname $$@` ]]; then mkdir -p `dirname $$@`; fi
	@echo "Compiling ($(1)) $$< -> $$@"
	$(QUIET)$(CC) $(ASFLAGS) $$(DEPFLAGS_$(1)) -D$(1) -c -o $$@ $$<

debug_$(1): $(1)
	$(GDB) -x "$(CHIP)_$(1).gdb" -ex "reset" -readnow -se "$(OUTPUT_BIN)_$(1).elf"

# 
# Include the dependency files, should be the last of the makefile
#
-include $(shell mkdir $(OBJ)/dep 2>/dev/null) $(wildcard $(OBJ)/dep/*.d)

endef

$(foreach MEMORY, $(MEMORIES), $(eval $(call RULES,$(MEMORY))))

clean:
	-$(RM) -fR $(OBJ) $(BIN)

# *** EOF ***