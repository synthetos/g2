# ----------------------------------------------------------------------------
#         ATMEL Microcontroller Software Support
# ----------------------------------------------------------------------------
# Copyright (c) 2010, Atmel Corporation
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following condition is met:
#
# - Redistributions of source code must retain the above copyright notice,
# this list of conditions and the disclaimer below.
#
# Atmel's name may not be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
# DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ----------------------------------------------------------------------------

#-------------------------------------------------------------------------------
#        User-modifiable options
#-------------------------------------------------------------------------------

ifeq ('$(CHIP)','')
$(error CHIP not defined)
endif

VERBOSE ?= 0

include sam_series.mk

# fill the needed variables
ifeq ($(CHIP),$(findstring $(CHIP), $(SAM3N)))

BOARD:=SAM3N_EK
SERIES:=sam3n

else ifeq ($(CHIP),$(findstring $(CHIP), $(SAM3S)))

BOARD:=SAM3S_EK
SERIES:=sam3s

else ifeq ($(CHIP),$(findstring $(CHIP), $(SAM3SD8)))

BOARD:=SAM3S_EK2
SERIES:=sam3sd8

else ifeq ($(CHIP),$(findstring $(CHIP), $(SAM3U)))

BOARD:=SAM3U_EK
SERIES:=sam3u

else ifeq ($(CHIP),$(findstring $(CHIP), $(SAM3XA)))

BOARD:=SAM3X_EK
SERIES:=sam3xa

else ifeq ($(CHIP),$(findstring $(CHIP), $(SAM4S)))

BOARD:=SAM4S_EK
SERIES:=sam4s

endif


# Defines which are the available memory targets for the device.
MEMORIES = sram flash

# Optimization level, put in comment for debugging
OPTIMIZATION = $(OPT)

# Output directories
BIN = $(CHIP)_bin
OBJ = $(CHIP)_obj
DEPDIR = $(OBJ)/dep

# Output file basename
OUTPUT_BIN = $(BIN)/$(PROJECT)_$(BOARD)_$(CHIP)

# GCC toolchain provider
GCC_TOOLCHAIN=gcc_atmel

#-------------------------------------------------------------------------------
#		Tools
#-------------------------------------------------------------------------------

# Toolchain prefix when cross-compiling
CROSS_COMPILE = arm-none-eabi-

# CMSIS_ROOT:=../../Arduino/hardware/arduino/sam/system
CMSIS_PATH=$(CMSIS_ROOT)/CMSIS/Include
SAM_PATH=$(CMSIS_ROOT)/Device/ATMEL
DEVICE_PATH=$(SAM_PATH)/$(SERIES)/source

LIBS = -lgcc -lc

LIB_PATH+=-L=/lib/thumb2
LIB_PATH+=-L"$(realpath $(DEVICE_PATH)/$(GCC_TOOLCHAIN))"

VARIANT=arduino_due_x

# Compilation tools
CC = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++
LD = $(CROSS_COMPILE)ld
AR = $(CROSS_COMPILE)ar
SIZE = $(CROSS_COMPILE)size
STRIP = $(CROSS_COMPILE)strip
OBJCOPY = $(CROSS_COMPILE)objcopy
GDB = $(CROSS_COMPILE)gdb
NM = $(CROSS_COMPILE)nm
RM = rm

# Flags
INCLUDES += -I"$(CMSIS_PATH)"
INCLUDES += -I"$(SAM_PATH)"
INCLUDES += -I"$(SAM_PATH)/$(SERIES)/include"

CFLAGS += -Wall -Wchar-subscripts -Wcomment -Wformat=2 -Wimplicit-int
CFLAGS += -Werror-implicit-function-declaration -Wmain -Wparentheses
CFLAGS += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
CFLAGS += -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef
CFLAGS += -Wshadow -Wpointer-arith -Wbad-function-cast -Wwrite-strings
CFLAGS += -Wsign-compare -Waggregate-return -Wstrict-prototypes
CFLAGS += -Wmissing-prototypes -Wmissing-declarations
CFLAGS += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
CFLAGS += -Wpacked -Wredundant-decls -Wnested-externs -Winline -Wlong-long
CFLAGS += -Wunreachable-code
CFLAGS += -Wcast-align
#CFLAGS += -Wmissing-noreturn
#CFLAGS += -Wconversion

CFLAGS += --param max-inline-insns-single=500 -mcpu=cortex-m3 -mthumb -mlong-calls -ffunction-sections -fdata-sections -nostdlib -std=gnu99
CFLAGS += $(OPTIMIZATION) $(INCLUDES) -D__$(CHIP)__ -D$(VARIANT)

# To reduce application size use only integer printf function.
CFLAGS += -Dprintf=iprintf

# ---------------------------------------------------------------------------------------
# CPP Flags

CPPFLAGS += -Wall -Wchar-subscripts -Wcomment -Wformat=2
CPPFLAGS += -Wmain -Wparentheses -Wcast-align -Wunreachable-code
CPPFLAGS += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
CPPFLAGS += -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef
CPPFLAGS += -Wshadow -Wpointer-arith -Wwrite-strings
CPPFLAGS += -Wsign-compare -Waggregate-return -Wmissing-declarations
CPPFLAGS += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
CPPFLAGS += -Wpacked -Wredundant-decls -Winline -Wlong-long
#CPPFLAGS += -Wmissing-noreturn
#CPPFLAGS += -Wconversion

CPPFLAGS += --param max-inline-insns-single=500 -mcpu=cortex-m3 -mthumb -mlong-calls -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions
CPPFLAGS += $(OPTIMIZATION) $(INCLUDES) -D__$(CHIP)__

# To reduce application size use only integer printf function.
CPPFLAGS += -Dprintf=iprintf

# ---------------------------------------------------------------------------------------
# ASM Flags

ASFLAGS = -mcpu=cortex-m3 -mthumb -Wall -g $(OPTIMIZATION) $(INCLUDES) -D__$(CHIP)__ -D__ASSEMBLY__

LDFLAGS = $(LIBS) -mthumb -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--entry=Reset_Handler -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align -Wl,--warn-unresolved-symbols
LDFLAGS += -nostartfiles
#LD_OPTIONAL=-Wl,--print-gc-sections -Wl,--stats

#-------------------------------------------------------------------------------
#		Files
#-------------------------------------------------------------------------------

# Directories where source files can be found

VPATH += ..
VPATH += $(DEVICE_PATH)
VPATH += $(DEVICE_PATH)/$(GCC_TOOLCHAIN)

# Objects built from C source files
# SOURCES += main.o
# SOURCES += startup_$(SERIES).o
# SOURCES += system_$(SERIES).o

CXX_OBJECTS = $(addsuffix .o,$(basename $(SOURCES_CXX)))
OBJECTS = $(addsuffix .o,$(basename $(SOURCES) ))


# Append OBJ and BIN directories to output filename
OUTPUT := $(BIN)/$(OUTPUT_BIN)

#-------------------------------------------------------------------------------
#		Rules
#-------------------------------------------------------------------------------

all: $(BIN) $(OBJ) $(MEMORIES)

$(BIN) $(OBJ) $(DEPDIR):
	-@mkdir $@

define RULES
OUTDIR = $(OBJ)/$(1)_
OBJECTS_$(1) = $(addprefix $$(OUTDIR)/, $(OBJECTS))
CXX_OBJECTS_$(1) = $(addprefix $$(OUTDIR)/, $(CXX_OBJECTS))
ASM_OBJECTS_$(1) = $(addprefix $$(OUTDIR)/, $(ASM_OBJECTS))
LINKER_SCRIPT_$(1) ?= "$(DEVICE_PATH)/$(GCC_TOOLCHAIN)/$(CHIP)_$$@.ld"
ABS_LINKER_SCRIPT_$(1) = $(abspath $$(LINKER_SCRIPT_$(1)))

# Generate dependency information
DEPFLAGS = -MMD -MF $(OBJ)/dep/$$(@F).d -MT $$(subst $$(OUTDIR),$$(OBJ)/\*_,$$@)

# 
# Include the dependency files, should be the last of the makefile
#

$(1): $(OBJ)/$(1)_/core.a
	@echo "Linking ($(1)) $$@"
	@if [[ ! -d `dirname $$@` ]]; then mkdir -p `dirname $$@`; fi
	@echo "Using linker script: $$(ABS_LINKER_SCRIPT_$(1))"
	@if [[ $(VERBOSE) == 1 ]]; then {\
		echo $(CXX) $(LIB_PATH) -T"$$(ABS_LINKER_SCRIPT_$(1))" -Wl,-Map,"$(OUTPUT_BIN)_$$@.map" -o "$(OUTPUT_BIN)_$$@.elf" $(LDFLAGS) $(LD_OPTIONAL) -Wl,--start-group $(LIBS) $$^ -Wl,--end-group;\
	}; fi
	@$(CXX) $(LIB_PATH) -T"$$(ABS_LINKER_SCRIPT_$(1))" -Wl,-Map,"$(OUTPUT_BIN)_$$@.map" -o "$(OUTPUT_BIN)_$$@.elf" $(LDFLAGS) $(LD_OPTIONAL) -Wl,--start-group $(LIBS) $$^ -Wl,--end-group
	@echo "Exporting symbols $(OUTPUT_BIN)_$$@.elf.txt"
	@if [[ $(VERBOSE) == 1 ]]; then {\
		echo $(NM) "$(OUTPUT_BIN)_$$@.elf" >"$(OUTPUT_BIN)_$$@.elf.txt";\
	}; fi
	@$(NM) "$(OUTPUT_BIN)_$$@.elf" >"$(OUTPUT_BIN)_$$@.elf.txt"
	@echo "Making binary $(OUTPUT_BIN)_$$@.bin"
	@if [[ $(VERBOSE) == 1 ]]; then {\
		echo $(OBJCOPY) -O binary "$(OUTPUT_BIN)_$$@.elf" "$(OUTPUT_BIN)_$$@.bin";\
	}; fi
	@$(OBJCOPY) -O binary "$(OUTPUT_BIN)_$$@.elf" "$(OUTPUT_BIN)_$$@.bin"
	# @echo "--- SIZE INFO ---"
	# @$(SIZE) $$^ "$(OUTPUT_BIN)_$$@.elf"


$$(OUTDIR)/core.a: $$(OUTDIR)/core.a( $$(ASM_OBJECTS_$(1)) $$(CXX_OBJECTS_$(1)) $$(OBJECTS_$(1)) ) | $(BIN) $(OBJ)

$$(CXX_OBJECTS_$(1)): $$(OUTDIR)/%.o: %.cpp | $(BIN) $(OBJ) $(DEPDIR)
	@if [[ ! -d `dirname $$@` ]]; then mkdir -p `dirname $$@`; fi
	@echo "Compiling cpp ($(1)) $$< -> $$@"
	@if [[ $(VERBOSE) == 1 ]]; then echo $(CXX) $(CPPFLAGS) $$(DEPFLAGS) -D$(1) -c -o $$@ $$<; fi
	@$(CXX) $(CPPFLAGS) $$(DEPFLAGS) -D$(1) -xc++ -c -o $$@ $$<

$$(OBJECTS_$(1)): $$(OUTDIR)/%.o: %.c | $(BIN) $(OBJ) $(DEPDIR)
	@if [[ ! -d `dirname $$@` ]]; then mkdir -p `dirname $$@`; fi
	@echo "Compiling c ($(1)) $$< -> $$@"
	@if [[ $(VERBOSE) == 1 ]]; then echo $(CC) $(CFLAGS) $$(DEPFLAGS) -D$(1) -c -o $$@ $$<; fi
	@$(CC) $(CFLAGS) $$(DEPFLAGS) -D$(1) -c -o $$@ $$<

$$(ASM_OBJECTS_$(1)): $$(OUTDIR)/%.o: %.S | $(BIN) $(OBJ) $(DEPDIR)
	@if [[ ! -d `dirname $$@` ]]; then mkdir -p `dirname $$@`; fi
	@echo "Compiling ($(1)) $$< -> $$@"
	@if [[ $(VERBOSE) == 1 ]]; then echo $(CC) $(ASFLAGS) $$(DEPFLAGS) -D$(1) -c -o $$@ $$<; fi
	@$(CC) $(ASFLAGS) $$(DEPFLAGS) -D$(1) -c -o $$@ $$<

debug_$(1): $(1)
	$(GDB) -x "$(CHIP)_$(1).gdb" -ex "reset" -readnow -se "$(OUTPUT_BIN)_$(1).elf"

-include $(shell mkdir $(OBJ)/dep 2>/dev/null) $(wildcard $(OBJ)/dep/*.d)

endef

$(foreach MEMORY, $(MEMORIES), $(eval $(call RULES,$(MEMORY))))

clean:
	-$(RM) -fR $(OBJ) $(BIN)

