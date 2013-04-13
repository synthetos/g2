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

include platform/atmel_sam/atmel_sam_series.mk
include platform/make_utilities.mk

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

# GCC toolchain provider
GCC_TOOLCHAIN = gcc_atmel

# Toolchain prefix when cross-compiling
CROSS_COMPILE = arm-none-eabi-

# Defines which are the available memory targets for the device.
MEMORIES = sram flash

CMSIS_PATH  = $(CMSIS_ROOT)/CMSIS/Include
SAM_PATH    = $(CMSIS_ROOT)/Device/ATMEL
DEVICE_PATH = $(SAM_PATH)/$(SERIES)/source

SAM_SOURCE_DIRS += $(DEVICE_PATH)
SAM_SOURCE_DIRS += $(DEVICE_PATH)/$(GCC_TOOLCHAIN)

$(eval $(call CREATE_DEVICE_LIBRARY,SAM,cmsis_sam))

# Flags
DEVICE_INCLUDE_DIRS += "$(CMSIS_PATH)"
DEVICE_INCLUDE_DIRS += "$(SAM_PATH)"
DEVICE_INCLUDE_DIRS += "$(SAM_PATH)/$(SERIES)/include"

LIBS     += -lgcc -lc

LIB_PATH += -L=/lib/thumb2
#LIB_PATH += -L"$(realpath $(DEVICE_PATH)/$(GCC_TOOLCHAIN))"

# FIXME: Assumes all sams are Dues
VARIANT=arduino_due_x
CFLAGS   += -D__$(CHIP)__ -D$(VARIANT)
CPPFLAGS += -D__$(CHIP)__ -D$(VARIANT)

ASFLAGS  += -mcpu=cortex-m3 -mthumb 
