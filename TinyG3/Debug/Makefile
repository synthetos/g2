################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS :=  \
../cmsis/ \
../cmsis/linkerScripts/ \
../cmsis/src/


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../cmsis/src/startup_sam3xa.c \
../cmsis/src/system_sam3xa.c \
../TinyG3.cpp


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
cmsis/src/startup_sam3xa.o \
cmsis/src/system_sam3xa.o \
TinyG3.o


OBJS_AS_ARGS +=  \
cmsis/src/startup_sam3xa.o \
cmsis/src/system_sam3xa.o \
TinyG3.o


C_DEPS +=  \
cmsis/src/startup_sam3xa.d \
cmsis/src/system_sam3xa.d \
TinyG3.d


C_DEPS_AS_ARGS +=  \
cmsis/src/startup_sam3xa.d \
cmsis/src/system_sam3xa.d \
TinyG3.d


OUTPUT_FILE_PATH +=TinyG3.elf

OUTPUT_FILE_PATH_AS_ARGS +=TinyG3.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

# AVR32/GNU C Compiler







cmsis/src/%.o: ../cmsis/src/%.c
	@echo Building file: $<
	@echo Invoking: ARM/GNU C Compiler : (crosstool-NG 1.15.3 - Atmel build: 59) 4.7.0
	$(QUOTE)C:\Program Files\Atmel\Atmel Studio 6.0\extensions\Atmel\ARMGCC\3.3.1.128\ARMGCCToolchain\bin\arm-none-eabi-gcc.exe$(QUOTE) -mthumb -D__SAM3X8E__ -I"C:\Program Files\Atmel\Atmel Studio 6.0\extensions\Atmel\ARMGCC\3.3.1.128\ARMSupportFiles" -I"C:\PROGRA~1\Atmel\ATMELS~1.0\EXTENS~1\Atmel\ARMGCC\331~1.128\ARMSUP~1\CMSIS\Include" -I"C:\PROGRA~1\Atmel\ATMELS~1.0\EXTENS~1\Atmel\ARMGCC\331~1.128\ARMSUP~1\Device\ATMEL" -I"C:\PROGRA~1\Atmel\ATMELS~1.0\EXTENS~1\Atmel\ARMGCC\331~1.128\ARMSUP~1\Device\ATMEL\sam3xa\include"  -O1 -ffunction-sections -mlong-calls -g3 -Wall -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -mcpu=cortex-m3   -o"$@" "$<" 
	@echo Finished building: $<

./%.o: .././%.cpp
	@echo Building file: $<
	@echo Invoking: ARM/GNU C Compiler : (crosstool-NG 1.15.3 - Atmel build: 59) 4.7.0
	$(QUOTE)C:\Program Files\Atmel\Atmel Studio 6.0\extensions\Atmel\ARMGCC\3.3.1.128\ARMGCCToolchain\bin\arm-none-eabi-g++.exe$(QUOTE) -mthumb -D__SAM3X8E__ -I"C:\Program Files\Atmel\Atmel Studio 6.0\extensions\Atmel\ARMGCC\3.3.1.128\ARMSupportFiles" -I"C:\PROGRA~1\Atmel\ATMELS~1.0\EXTENS~1\Atmel\ARMGCC\331~1.128\ARMSUP~1\CMSIS\Include" -I"C:\PROGRA~1\Atmel\ATMELS~1.0\EXTENS~1\Atmel\ARMGCC\331~1.128\ARMSUP~1\Device\ATMEL" -I"C:\PROGRA~1\Atmel\ATMELS~1.0\EXTENS~1\Atmel\ARMGCC\331~1.128\ARMSUP~1\Device\ATMEL\sam3xa\include"  -O1 -ffunction-sections -fno-rtti -fno-exceptions -mlong-calls -g3 -Wall -c -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"  -mcpu=cortex-m3   -o"$@" "$<" 
	@echo Finished building: $<



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP)
	@echo Building target: $@
	@echo Invoking: ARM/GNU Linker : (crosstool-NG 1.15.3 - Atmel build: 59) 4.7.0
	$(QUOTE)C:\Program Files\Atmel\Atmel Studio 6.0\extensions\Atmel\ARMGCC\3.3.1.128\ARMGCCToolchain\bin\arm-none-eabi-g++.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="TinyG3.map" -Wl,--start-group -lm  -Wl,--end-group -L"../cmsis/linkerScripts"  -Wl,--gc-sections -Tsam3x8e_flash.ld  -mcpu=cortex-m3 
	@echo Finished building target: $@
	"C:\Program Files\Atmel\Atmel Studio 6.0\extensions\Atmel\ARMGCC\3.3.1.128\ARMGCCToolchain\bin\arm-none-eabi-objcopy.exe" -O binary "TinyG3.elf" "TinyG3.bin"
	"C:\Program Files\Atmel\Atmel Studio 6.0\extensions\Atmel\ARMGCC\3.3.1.128\ARMGCCToolchain\bin\arm-none-eabi-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature  "TinyG3.elf" "TinyG3.hex"
	"C:\Program Files\Atmel\Atmel Studio 6.0\extensions\Atmel\ARMGCC\3.3.1.128\ARMGCCToolchain\bin\arm-none-eabi-objcopy.exe" -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings -O binary "TinyG3.elf" "TinyG3.eep" || exit 0
	"C:\Program Files\Atmel\Atmel Studio 6.0\extensions\Atmel\ARMGCC\3.3.1.128\ARMGCCToolchain\bin\arm-none-eabi-objdump.exe" -h -S "TinyG3.elf" > "TinyG3.lss"
	"C:\Program Files\Atmel\Atmel Studio 6.0\extensions\Atmel\ARMGCC\3.3.1.128\ARMGCCToolchain\bin\arm-none-eabi-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature  "TinyG3.elf" "TinyG3.srec"
	"C:\Program Files\Atmel\Atmel Studio 6.0\extensions\Atmel\ARMGCC\3.3.1.128\ARMGCCToolchain\bin\arm-none-eabi-size.exe" "TinyG3.elf"
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS)$(C_DEPS_AS_ARGS) $(EXECUTABLES) 
	rm -rf "TinyG3.elf" "TinyG3.a" "TinyG3.hex" "TinyG3.bin" "TinyG3.lss" "TinyG3.eep" "TinyG3.map" "TinyG3.srec"
	