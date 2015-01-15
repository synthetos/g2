#
# Makefile
#
# Copyright (c) 2012 - 2013 Robert Giseburt
# Copyright (c) 2013 Alden S. Hart Jr.
#
# This file is part of the TinyG2 project.
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

PROJECT  = TinyG2


# Include all of the *directories* containing files to compile to SOURCE_DIRS.
# *EXCEPT* motate paths and processor-specific directories!
SOURCE_DIRS = .

# List *project-specific* files that must be linked to first in FIRST_LINK_SOURCES.
FIRST_LINK_SOURCES =

# List all user directories here that aren't in SOURCE_DIRS
USER_INCLUDE_DIRS =

# List the user directory to look for the libraries here
USER_LIB_DIRS =

# List all user libraries here
USER_LIBS =

# Define optimisation level here
#OPTIMIZATION ?= 0
OPTIMIZATION ?= s
#OPTIMIZATION ?= 3

# SETTINGS_FILE may get overriden by the PLATFORM setings below
SETTINGS_FILE ?= settings_default.h

CFLAGS   :=
CPPFLAGS :=

# List ASM source files here
ASRC =

MOTATE_PATH := motate

SOURCE_DIRS += $(MOTATE_PATH)
USER_INCLUDE_DIRS += $(SOURCE_DIRS)

# ---------------------------------------------------------------------------------------
# C Flags (NOT CPP Flags)

# Configure warnings:
CFLAGS += -Wall -Wchar-subscripts -Wcomment -Wformat=2 -Wimplicit-int
CFLAGS += -Werror-implicit-function-declaration -Wmain -Wparentheses
CFLAGS += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
CFLAGS += -Wuninitialized -Wno-unknown-pragmas -Wfloat-equal -Wundef
CFLAGS += -Wshadow -Wpointer-arith -Wbad-function-cast -Wwrite-strings
CFLAGS += -Wsign-compare -Waggregate-return
#CFLAGS += -Wstrict-prototypes
#CFLAGS += -Wmissing-prototypes -Wmissing-declarations
CFLAGS += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
CFLAGS += -Wredundant-decls -Wnested-externs -Wlong-long
CFLAGS += -Wunreachable-code
#CFLAGS += -Wcast-align -Wpacked
#CFLAGS += -Wmissing-noreturn
#CFLAGS += -Wconversion


# These are useful, but chatty:
#CFLAGS += -Winline


# Turn off printf() format strings. We use late bound FLASH and RAM format strings and this causes warnings
CFLAGS += -Wno-format-nonliteral -Wno-format-security

# To reduce application size use only integer printf function.
#CFLAGS += -Dprintf=iprintf

# ---------------------------------------------------------------------------------------
# CPP Flags

CPPFLAGS += -Wall -Wchar-subscripts -Wcomment -Wformat=2
CPPFLAGS += -Wmain -Wparentheses -Wunreachable-code
#CPPFLAGS += -Wcast-align
CPPFLAGS += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
CPPFLAGS += -Wuninitialized -Wno-unknown-pragmas -Wfloat-equal -Wundef
CPPFLAGS += -Wshadow -Wpointer-arith -Wwrite-strings
#CPPFLAGS += -Wsign-compare -Waggregate-return -Wmissing-declarations
CPPFLAGS += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
CPPFLAGS += -Wredundant-decls -Wlong-long
#CPPFLAGS += -Wmissing-noreturn -Wpacked
#CPPFLAGS += -Wconversion


# These are useful, but chatty:
#CFLAGS += -Winline


# Turn off printf() format strings. We use late bound FLASH and RAM format strings and this causes warnings
CPPFLAGS += -Wno-format-nonliteral -Wno-format-security

# To reduce application size use only integer printf function.
#CPPFLAGS += -Dprintf=iprintf

# Default is to compile more quielty.
# To show actual commands: make [options] VERBOSE=1
VERBOSE ?= 0
COLOR ?= 1
ifeq ($(VERBOSE),0)
QUIET := @
else
QUIET :=
endif

ifeq ($(COLOR),1)
START_BOLD=`tput bold`
END_BOLD=`tput sgr0`
NO_COLOR=`tput setaf 0`
OK_COLOR=`tput setaf 2`
ERROR_COLOR=`tput setaf 1`
WARN_COLOR=`tput setaf 3`
endif


#
# End of default section
##############################################################################################

##############################################################################################
# Output directories (must be before platforms section)
#

BIN = bin/$(PLATFORM)
OBJ = build/$(PLATFORM)
DEPDIR = $(OBJ)/dep

#
# End of Output directories
##############################################################################################

##############################################################################################
# Start of platforms section
#

PLATFORM ?= gShield

ifeq ("$(PLATFORM)","gShield")

	# This is a due with a Synthetos gShield. We'll use the Due platform, but set defines
	# for the code to get the pinout right.

	BASE_PLATFORM=due
	DEVICE_DEFINES += MOTATE_BOARD="gShield" SETTINGS_FILE=${SETTINGS_FILE}

endif

ifeq ("$(PLATFORM)","shopbotShield")

	# This is a due with a Synthetos gShield. We'll use the Due platform, but set defines
	# for the code to get the pinout right.

	BASE_PLATFORM=due
	DEVICE_DEFINES += MOTATE_BOARD="shopbotShield"

endif
ifeq ("$(PLATFORM)","G2v9d")

	BASE_PLATFORM=v9_3x8c
	DEVICE_DEFINES += MOTATE_BOARD="G2v9d" SETTINGS_FILE=${SETTINGS_FILE}

endif

ifeq ("$(PLATFORM)","G2v9f")

	BASE_PLATFORM=v9_3x8c
	DEVICE_DEFINES += MOTATE_BOARD="G2v9f" SETTINGS_FILE=${SETTINGS_FILE}

endif

ifeq ("$(PLATFORM)","G2v9g")

	BASE_PLATFORM=v9_3x8c
	DEVICE_DEFINES += MOTATE_BOARD="G2v9g" SETTINGS_FILE=${SETTINGS_FILE}

endif

ifeq ("$(PLATFORM)","G2v9i")

	BASE_PLATFORM=v9_3x8c
	DEVICE_DEFINES += MOTATE_BOARD="G2v9i" SETTINGS_FILE=${SETTINGS_FILE}

endif

ifeq ("$(PLATFORM)","UltimakerTests")

	BASE_PLATFORM=v9_3x8c
	SETTINGS_FILE="settings_Ultimaker_Rob_v9h.h"
	DEVICE_DEFINES += MOTATE_BOARD="G2v9i" SETTINGS_FILE=${SETTINGS_FILE}

endif

#ifeq ("$(PLATFORM)","G2v9j")
#
#BASE_PLATFORM=v9_3x8c
#DEVICE_DEFINES += MOTATE_BOARD="G2v9j"
#
#endif

ifeq ("$(BASE_PLATFORM)","")
	BASE_PLATFORM = $(PLATFORM)
endif

ifeq ("$(BASE_PLATFORM)","due")
	_PLATFORM_FOUND = 1
	FIRST_LINK_SOURCES += motate/SamTimers.cpp motate/SamUSB.cpp motate/SamPins.cpp

# Set CHIP and export it for GDB to see
	export CHIP = SAM3X8E
	PLATFORM_BASE = platform/atmel_sam
	CMSIS_ROOT = CMSIS

	DEVICE_INCLUDE_DIRS += $(PLATFORM_BASE)/board/due


	include $(PLATFORM_BASE).mk
endif

ifeq ("$(BASE_PLATFORM)","v9_3x8c")
	_PLATFORM_FOUND = 1
	FIRST_LINK_SOURCES += motate/SamTimers.cpp motate/SamUSB.cpp motate/SamPins.cpp

# Set CHIP and export it for GDB to see
	export CHIP = SAM3X8C
	PLATFORM_BASE = platform/atmel_sam
	CMSIS_ROOT = CMSIS

	DEVICE_INCLUDE_DIRS += $(PLATFORM_BASE)/board/v9_3x8c

	include $(PLATFORM_BASE).mk

endif

ifeq ("$(_PLATFORM_FOUND)", "0")
# errors cannot be indented
$(error Unknown platform "$(PLATFORM)")
endif

#
# End of platforms section
##############################################################################################

##############################################################################################
# Actual compilation
#


# Output file basename
OUTPUT_BIN = $(BIN)/$(PLATFORM)

#-------------------------------------------------------------------------------
#    Tools
#-------------------------------------------------------------------------------

# Compilation tools
CC      = $(CROSS_COMPILE)-gcc
CXX     = $(CROSS_COMPILE)-g++
LD      = $(CROSS_COMPILE)-ld
AR      = $(CROSS_COMPILE)-ar
SIZE    = $(CROSS_COMPILE)-size
STRIP   = $(CROSS_COMPILE)-strip
OBJCOPY = $(CROSS_COMPILE)-objcopy
GDB     = $(CROSS_COMPILE)-gdb
NM      = $(CROSS_COMPILE)-nm
RM      = rm
CP      = cp
CKSUM	= cksum
DD		= dd
# NOTE: Atmel Studio 6.1 will default to the wrong mkdir that doesn't understand -p.
#       So, in AS6.1, we have to pass MKDIR=gmkdir in the command line.
MKDIR   ?= mkdir

SHELL = bash
PATH := $(PATH):../Tools/gcc-$(CROSS_COMPILE)/bin


# ---------------------------------------------------------------------------------------
# C Flags (NOT CPP flags)

#CFLAGS +=  $(DEVICE_CFLAGS)
CFLAGS +=  $(DEVICE_CFLAGS) -DPLATFORM=$(PLATFORM)

# ---------------------------------------------------------------------------------------
# CPP Flags

#CPPFLAGS +=  $(DEVICE_CPPFLAGS)
CPPFLAGS +=  $(DEVICE_CPPFLAGS) -DPLATFORM=$(PLATFORM)

# ---------------------------------------------------------------------------------------
# ASM Flags

ASFLAGS = $(DEVICE_ASFLAGS)


# ---------------------------------------------------------------------------------------
# Linker Flags

LDFLAGS += $(LIBS) $(USER_LIBS) -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--entry=Reset_Handler -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align -Wl,--warn-unresolved-symbols $(DEVICE_LDFLAGS)


#-------------------------------------------------------------------------------
#    Files
#-------------------------------------------------------------------------------

# Directories where source files can be found

C_SOURCES   = $(foreach dir, $(SOURCE_DIRS), $(wildcard $(dir)/*.c) )
CXX_SOURCES = $(foreach dir, $(SOURCE_DIRS), $(wildcard $(dir)/*.cpp) )

C_OBJECTS   := $(addsuffix .o,$(basename $(C_SOURCES)))
CXX_OBJECTS := $(addsuffix .o,$(basename $(CXX_SOURCES)))
FIRST_LINK_OBJECTS = $(addsuffix .o,$(basename $(FIRST_LINK_SOURCES)))

# Append OBJ and BIN directories to output filename
OUTPUT := $(BIN)/$(OUTPUT_BIN)

INCLUDES = $(patsubst %,-I%,$(DEVICE_INCLUDE_DIRS) $(USER_INCLUDE_DIRS))
LIBS     = $(patsubst %,-l%,$(DEVICE_LIBS) $(USER_LIBS))
LIBDIR   = $(patsubst %,-L%,$(DEVICE_LIB_DIRS) $(USER_LIB_DIRS))

CFLAGS   += -g3 -O$(OPTIMIZATION) $(INCLUDES) $(patsubst %,-D%,$(DEVICE_DEFINES) $(USER_DEFINES))
CPPFLAGS += -g3 -O$(OPTIMIZATION) $(INCLUDES) $(patsubst %,-D%,$(DEVICE_DEFINES) $(USER_DEFINES))
ASFLAGS  += -g3 -O$(OPTIMIZATION) $(INCLUDES) -D__ASSEMBLY__ $(patsubst %,-D%,$(DEVICE_DEFINES) $(USER_DEFINES))


ifeq ($(VERBOSE),2)
define NEWLINE_ONLY


endef
define NEWLINE_TAB

	${EMPTY}
endef

$(info DEVICE_INCLUDE_DIRS+USER_INCLUDE_DIRS:$(patsubst %,$(NEWLINE_TAB)%,$(DEVICE_INCLUDE_DIRS) $(USER_INCLUDE_DIRS)))
$(info $(NEWLINE_ONLY)DEVICE_LIB_DIRS+USER_LIB_DIRS: $(patsubst %,$(NEWLINE_TAB)%,$(DEVICE_LIB_DIRS) $(USER_LIB_DIRS)))
$(info $(NEWLINE_ONLY)SOURCE_DIRS: $(patsubst %,$(NEWLINE_TAB)%,$(SOURCE_DIRS)))
$(info $(NEWLINE_ONLY)C_SOURCES: $(patsubst %,$(NEWLINE_TAB)%,$(C_SOURCES)))
$(info $(NEWLINE_ONLY)SAM_SOURCE_DIRS: $(patsubst %,$(NEWLINE_TAB)%,$(SAM_SOURCE_DIRS)))
$(info $(NEWLINE_ONLY)CXX_SOURCES: $(patsubst %,$(NEWLINE_TAB)%,$(CXX_SOURCES)))
$(info $(NEWLINE_ONLY)FIRST_LINK_SOURCES: $(patsubst %,$(NEWLINE_TAB)%,$(FIRST_LINK_SOURCES)))
endif


#-------------------------------------------------------------------------------
#    Rules
#-------------------------------------------------------------------------------


all: $(OUTPUT_BIN).elf

REQUIRED_DIRS := $(BIN) $(OBJ) $(DEPDIR)

MK_DIRS =   $(shell                              \
		for d in $(REQUIRED_DIRS);         \
		do                                 \
		[[ -d $$d ]] || mkdir -p $$d;    \
		done)

$(eval $(DEVICE_RULES))

.PHONY : MKTOOLS

MKTOOLS:
	@echo Looking for tools...
	cd ../Tools && make "ARCH=gcc-${CROSS_COMPILE}"

OUTDIR = $(OBJ)
REQUIRED_DIRS += $(OUTDIR)


FIRST_LINK_OBJECTS_PATHS := $(addprefix $(OUTDIR)/,$(subst $(MOTATE_PATH),motate,$(FIRST_LINK_OBJECTS)))

MOTATE_C_OBJECTS   := $(addprefix $(OUTDIR)/,$(subst $(MOTATE_PATH),motate,$(filter $(MOTATE_PATH)/%,$(C_OBJECTS))))
MOTATE_CXX_OBJECTS := $(addprefix $(OUTDIR)/,$(subst $(MOTATE_PATH),motate,$(filter $(MOTATE_PATH)/%,$(CXX_OBJECTS))))
MOTATE_ASM_OBJECTS := $(addprefix $(OUTDIR)/,$(subst $(MOTATE_PATH),motate,$(filter $(MOTATE_PATH)/%,$(ASM_OBJECTS))))

ALL_OTHER_C_OBJECTS   := $(addprefix $(OUTDIR)/,$(filter-out $(MOTATE_PATH)/%,$(C_OBJECTS)))
ALL_OTHER_CXX_OBJECTS := $(addprefix $(OUTDIR)/,$(filter-out $(MOTATE_PATH)/%,$(CXX_OBJECTS)))
ALL_OTHER_ASM_OBJECTS := $(addprefix $(OUTDIR)/,$(filter-out $(MOTATE_PATH)/%,$(ASM_OBJECTS)))

ALL_C_OBJECTS   := $(MOTATE_C_OBJECTS) $(ALL_OTHER_C_OBJECTS)
ALL_CXX_OBJECTS := $(MOTATE_CXX_OBJECTS) $(ALL_OTHER_CXX_OBJECTS)
ALL_ASM_OBJECTS := $(MOTATE_ASM_OBJECTS) $(ALL_OTHER_ASM_OBJECTS)


#
CFLAGS   += -g3 -O$(OPTIMIZATION) $(INCLUDES) $(patsubst %,-D%,$(DEVICE_DEFINES) $(USER_DEFINES))
CPPFLAGS += -g3 -O$(OPTIMIZATION) $(INCLUDES) $(patsubst %,-D%,$(DEVICE_DEFINES) $(USER_DEFINES))
ASFLAGS  += -g3 -O$(OPTIMIZATION) $(INCLUDES) -D__ASSEMBLY__ $(patsubst %,-D%,$(DEVICE_DEFINES) $(USER_DEFINES))

LINKER_SCRIPT ?= $(PLATFORM_BASE)/gcc_flash.ld
#LINKER_SCRIPT := $(DEVICE_LINKER_SCRIPT)
ABS_LINKER_SCRIPT = $(abspath $(LINKER_SCRIPT))

# Generate dependency information
DEPFLAGS = -MMD -MF $(OBJ)/dep/$(@F).d -MT $(subst $(OUTDIR),$(OBJ),$@)

$(OUTPUT_BIN).elf: $(ALL_C_OBJECTS) $(ALL_CXX_OBJECTS) $(ALL_ASM_OBJECTS) $(ABS_LINKER_SCRIPT)
	@echo $(START_BOLD)"Linking $(OUTPUT_BIN).elf" $(END_BOLD)
	@echo $(START_BOLD)"Using linker script: $(ABS_LINKER_SCRIPT)" $(END_BOLD)
	$(QUIET)$(CXX) $(LIB_PATH) -T"$(ABS_LINKER_SCRIPT)" -Wl,-Map,"$(OUTPUT_BIN).map" -o ${filter-out MKTOOLS,$@} $(LDFLAGS) $(LD_OPTIONAL) $(LIBS) -Wl,--start-group $(FIRST_LINK_OBJECTS_PATHS) $(filter-out $(FIRST_LINK_OBJECTS_PATHS) $(ABS_LINKER_SCRIPT) MKTOOLS,$+) -Wl,--end-group
	@echo $(START_BOLD)"Exporting symbols $(OUTPUT_BIN).elf.txt" $(END_BOLD)
	$(QUIET)$(NM) "$(OUTPUT_BIN).elf" >"$(OUTPUT_BIN).elf.txt"
	@echo $(START_BOLD)"Making binary $(OUTPUT_BIN).bin" $(END_BOLD)
	$(QUIET)$(OBJCOPY) -O binary "$(OUTPUT_BIN).elf" "$(OUTPUT_BIN).bin"
	$(QUIET)printf "%08x" `cksum $(OUTPUT_BIN).bin | cut -d' ' -f1` | xxd -r -p | dd of=$(OUTPUT_BIN).bin bs=1 count=4 seek=0x100 conv=notrunc
	$(QUIET)printf "%08x" `ls -nl "$(OUTPUT_BIN).bin" | cut -d' ' -f8` | xxd -r -p | dd of=$(OUTPUT_BIN).bin bs=1 count=4 seek=0x104 conv=notrunc
	@echo "--- SIZE INFO ---"
	$(QUIET)$(SIZE) "$(OUTPUT_BIN).elf"

## Note: The motate paths are seperated do to MOTATE_PATH having multple ../ in it.

$(MOTATE_CXX_OBJECTS): $(OUTDIR)/motate/%.o: $(MOTATE_PATH)/%.cpp
	$(QUIET)$(MKDIR) -p "$(@D)" "$(DEPDIR)" "$(BIN)"
	@echo $(START_BOLD)"Compiling cpp $<"; echo "    -> $@" $(END_BOLD)
	$(QUIET)$(CXX) $(CPPFLAGS) $(DEPFLAGS) -xc++ -c -o $@ $<

$(ALL_OTHER_CXX_OBJECTS): $(OUTDIR)/%.o: %.cpp
	$(QUIET)$(MKDIR) -p "$(@D)" "$(DEPDIR)" "$(BIN)"
	@echo $(START_BOLD)"Compiling cpp $<"; echo "    -> $@" $(END_BOLD)
	$(QUIET)$(CXX) $(CPPFLAGS) $(DEPFLAGS) -xc++ -c -o $@ $<

$(MOTATE_C_OBJECTS): $(OUTDIR)/motate/%.o: $(MOTATE_PATH)/%.c
	$(QUIET)$(MKDIR) -p "$(@D)" "$(DEPDIR)" "$(BIN)"
	@echo $(START_BOLD)"Compiling c $<"; echo "    -> $@" $(END_BOLD)
	$(QUIET)$(CC) $(CFLAGS) $(DEPFLAGS) -c -o $@ $<

$(ALL_OTHER_C_OBJECTS): $(OUTDIR)/%.o: %.c
	$(QUIET)$(MKDIR) -p "$(@D)" "$(DEPDIR)" "$(BIN)"
	@echo $(START_BOLD)"Compiling c $<"; echo "    -> $@" $(END_BOLD)
	$(QUIET)$(CC) $(CFLAGS) $(DEPFLAGS) -c -o $@ $<

$(MOTATE_ASM_OBJECTS): $(OUTDIR)/motate/%.o: $(MOTATE_PATH)/%.S
	$(QUIET)$(MKDIR) -p "$(@D)" "$(DEPDIR)" "$(BIN)"
	@echo $(START_BOLD)"Compiling $<"; echo "    -> $@"  $(END_BOLD)
	$(QUIET)$(CC) $(ASFLAGS) $(DEPFLAGS) -c -o $@ $<

$(ALL_OTHER_ASM_OBJECTS): $(OUTDIR)/%.o: %.S
	$(QUIET)$(MKDIR) -p "$(@D)" "$(DEPDIR)" "$(BIN)"
	@echo $(START_BOLD)"Compiling $<"; echo "    -> $@"  $(END_BOLD)
	$(QUIET)$(CC) $(ASFLAGS) $(DEPFLAGS) -c -o $@ $<

debug: $(OUTPUT_BIN).elf openocd.cfg
	$(GDB) -x "${PLATFORM_BASE}.gdb" -readnow -se "$(OUTPUT_BIN).elf"

flash: $(OUTPUT_BIN).elf
	"${PLATFORM_BASE}-flash.sh" "$(OUTPUT_BIN).elf"

openocd.cfg:
	$(error Please copy openocd.cfg.example to openocd.cfg and modify it as necessary for your configuration.)

#-------------------------------------------------------------------------------
#    Default $(PROJECT).elf, and clean
#-------------------------------------------------------------------------------

$(PROJECT).elf: $(OUTPUT_BIN).elf
	$(CP) $< $@

$(PROJECT).map: $(OUTPUT_BIN).map
	$(CP) $< $@

$(PROJECT).hex: $(OUTPUT_BIN).elf
	$(QUIET)$(OBJCOPY) -O ihex $< $@

$(PROJECT).bin: $(OUTPUT_BIN).elf
	$(QUIET)$(OBJCOPY) -O binary $< $@

clean:
	-$(RM) -fR $(OBJ) $(BIN) $(BOARD).elf $(BOARD).map $(BOARD).hex $(BOARD).bin


#
# Include the dependency files, should be the last of the makefile
#
-include $(shell mkdir $(OBJ)/dep 2>/dev/null) $(wildcard $(OBJ)/dep/*.d)


# *** EOF ***
