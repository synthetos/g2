/*
 * printrbaordG2v3-pinout.h - board pinout specification
 * This file is part of the g2core project
 *
 * Copyright (c) 2013 - 2016 Robert Giseburt
 * Copyright (c) 2013 - 2016 Alden S. Hart Jr.
 *
 * This file is part of the Motate Library.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software. If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef printrboardG2_v3_pinout_h
#define printrboardG2_v3_pinout_h

/*
 * USAGE NOTES
 *
 * Read this first:
 * https://github.com/synthetos/g2/wiki/Adding-a-new-G2-board-(or-revision)-to-G2#making-a-new-pin-assignment
 *
 *  USAGE:
 *
 *  This file is lays out all the pin capabilities of the SAM3X8C organized by pin number.
 *  Each pin has its associated functions listed at the bottom of the file, and is essentially
 *  immutable for each processor.
 *
 *  To use, assign Motate pin numbers to the first value in the _MAKE_MOTATE_PIN() macro.
 *  ALL PINS MUST BE ASSIGNED A NUMBER, even if they are not used. There will NOT be a
 *  code-size or speed penalty for unused pins, but the WILL be a compiler-failure for
 *  unassigned pins. This new restriction allows for simplification of linkages deep in
 *  Motate.
 */
/*  See motate_pin_assignments.h for pin names to be used int he rest of the G2 code.
 *  EXAMPLES:
 *
 *  *** Vanilla pin example ***
 *
 *      _MAKE_MOTATE_PIN(4, A, 'A', 27);	// SPI0_SCKPinNumber
 *
 *      This assigns Motate pin 4 to Port A, pin 27 (A27)
 *      Look in motate_pin_assignments.h to see that this is kSPI_SCKPinNumber
 *
 *  ** Other pin functions ***
 *
 *      Please look in <Motate>/platform/atmel_sam/motate_chip_pin_functions.h
 */
/* NOTES:

    - Using kSocket1_InterruptPinNumber for INTERRUPT_OUT. Enabled in pin_assignments.h
    - kI2C1 reserved for filament sensor

 */

#include <MotateTimers.h>

// We don't have all of the inputs, so we have to indicate as much:
// BE AWARE: All Inputs set to 1 MUST have a corresponding kInput statement or it won't compile.
#define INPUT1_AVAILABLE 1
#define INPUT2_AVAILABLE 0
#define INPUT3_AVAILABLE 0
#define INPUT4_AVAILABLE 1
#define INPUT5_AVAILABLE 1
#define INPUT6_AVAILABLE 0
#define INPUT7_AVAILABLE 0
#define INPUT8_AVAILABLE 0
#define INPUT9_AVAILABLE 0
#define INPUT10_AVAILABLE 0
#define INPUT11_AVAILABLE 0
#define INPUT12_AVAILABLE 0
#define INPUT13_AVAILABLE 0

// BE AWARE: All ADCs set to 1 MUST have a corresponding kADC statement
#define ADC0_AVAILABLE 1
#define ADC1_AVAILABLE 1
#define ADC2_AVAILABLE 0
#define ADC3_AVAILABLE 0

#define XIO_HAS_USB 1
#define XIO_HAS_UART 1
#define XIO_HAS_SPI 0
#define XIO_HAS_I2C 0

#define TEMPERATURE_OUTPUT_ON 1

// Some pins, if the PWM capability is turned on, it will cause timer conflicts.
// So we have to explicitly enable them as PWM pins.
// Generated with: perl -e 'for($i=1;$i<14;$i++) { print "#define OUTPUT${i}_PWM 0\n";}'
#define OUTPUT1_PWM 1
#define OUTPUT2_PWM 0  // Unused
#define OUTPUT3_PWM 1
#define OUTPUT4_PWM 1
#define OUTPUT5_PWM 0   // Unused
#define OUTPUT6_PWM 0   // Unused
#define OUTPUT7_PWM 0   // Unused
#define OUTPUT8_PWM 0   // Unused
#define OUTPUT9_PWM 0   // Unused
#define OUTPUT10_PWM 0  // Unused
#define OUTPUT11_PWM 0  // Can't PWM anyway
#define OUTPUT12_PWM 0  // Unused
#define OUTPUT13_PWM 0  // Unused

namespace Motate {

// Arduino pin name & function
_MAKE_MOTATE_PIN(kUnassigned1, 'A', 0);                    // nc
_MAKE_MOTATE_PIN(kUnassigned2, 'A', 1);                    // nc
_MAKE_MOTATE_PIN(kSocket4_VrefPinNumber, 'A', 2);          // M4_Vref
_MAKE_MOTATE_PIN(kUnassigned3, 'A', 3);                    // nc
_MAKE_MOTATE_PIN(kADC0_PinNumber, 'A', 4);                 // BED_ADC
_MAKE_MOTATE_PIN(kOutput1_PinNumber, 'A', 5);              // DO_1 (Extruder1_PWM)
_MAKE_MOTATE_PIN(kOutput3_PinNumber, 'A', 6);              // DO_3 (Fan1B_PWM)
_MAKE_MOTATE_PIN(kOutputSAFE_PinNumber, 'A', 7);           // DO_9 (SAFE_PULSES - output from MCU)
_MAKE_MOTATE_PIN(kSerial_RXPinNumber, 'A', 8);                      // UART_RX
_MAKE_MOTATE_PIN(kSerial_TXPinNumber, 'A', 9);                      // UART_TX
_MAKE_MOTATE_PIN(kInput5_PinNumber, 'A', 10);              // DI_5 (ZMin)
_MAKE_MOTATE_PIN(kInput4_PinNumber, 'A', 11);              // DI_4 (YMax)
_MAKE_MOTATE_PIN(kUnassigned4, 'A', 12);                   // USART_RX (not used)
_MAKE_MOTATE_PIN(kUnassigned5, 'A', 13);                   // USART_TX (not used)
_MAKE_MOTATE_PIN(kSerial_RTSPinNumber, 'A', 14);                    // UART_RTS
_MAKE_MOTATE_PIN(kSerial_CTSPinNumber, 'A', 15);                    // UART_CTS
_MAKE_MOTATE_PIN(kSocket1_DirPinNumber, 'A', 16);          // M1_DIR
_MAKE_MOTATE_PIN(kUnassigned6, 'A', 17);                   // nc
_MAKE_MOTATE_PIN(kSocket1_InterruptPinNumber, 'A', 18);    // INTERRUPT_OUT
_MAKE_MOTATE_PIN(kSocket1_Microstep_2PinNumber, 'A', 19);  // M1_MS2
_MAKE_MOTATE_PIN(kSocket1_Microstep_0PinNumber, 'A', 20);  // M1_MS0 (M1_MS1 is slaved to this signal also)
_MAKE_MOTATE_PIN(kSocket2_StepPinNumber, 'A', 21);         // M2_STEP
_MAKE_MOTATE_PIN(kUnassigned7, 'A', 22);                   // <reserved for filament sensor digital in or ADC>
_MAKE_MOTATE_PIN(kADC1_PinNumber, 'A', 23);                // EX1_ADC (Extruder1 ADC)
_MAKE_MOTATE_PIN(kUnassigned8, 'A', 24);                   // nc
_MAKE_MOTATE_PIN(kSocket2_EnablePinNumber, 'A', 25);       // M2_ENABLE
_MAKE_MOTATE_PIN(kSocket2_DirPinNumber, 'A', 26);          // M2_DIR
_MAKE_MOTATE_PIN(kSocket3_Microstep_2PinNumber, 'A', 27);  // M3_MS2
_MAKE_MOTATE_PIN(kSocket3_Microstep_0PinNumber, 'A', 28);  // M3_MS0 (M3_MS1 is slaved to this signal also)
_MAKE_MOTATE_PIN(kSocket3_StepPinNumber, 'A', 29);         // M3_STEP

_MAKE_MOTATE_PIN(kSocket3_EnablePinNumber, 'B', 0);        // M3_ENABLE
_MAKE_MOTATE_PIN(kSocket3_DirPinNumber, 'B', 1);           // M3_DIR
_MAKE_MOTATE_PIN(kUnassigned9, 'B', 2);                    // nc
_MAKE_MOTATE_PIN(kOutput12_PinNumber, 'B', 3);             // DO_12 (INDICATOR_LED)
_MAKE_MOTATE_PIN(kUnassigned10, 'B', 4);                   // nc
_MAKE_MOTATE_PIN(kUnassigned11, 'B', 5);                   // nc
_MAKE_MOTATE_PIN(kUnassigned12, 'B', 6);                   // nc
_MAKE_MOTATE_PIN(kUnassigned13, 'B', 7);                   // nc
_MAKE_MOTATE_PIN(kUnassigned14, 'B', 8);                   // nc
_MAKE_MOTATE_PIN(kUnassigned15, 'B', 9);                   // nc
_MAKE_MOTATE_PIN(kSocket4_Microstep_2PinNumber, 'B', 10);  // M4_MS2
_MAKE_MOTATE_PIN(kSocket4_Microstep_0PinNumber, 'B', 11);  // M4_MS0 (M4_MS1 is slaved to this signal also)
_MAKE_MOTATE_PIN(kLED_RGBWPixelPinNumber, 'B', 12);        // <reserved for filament sensor digital in or I2C>
_MAKE_MOTATE_PIN(kUnassigned16, 'B', 13);                  // <reserved for filament sensor digital in or I2C>
_MAKE_MOTATE_PIN(kSocket4_StepPinNumber, 'B', 14);         // M4_STEP
_MAKE_MOTATE_PIN(kSocket1_StepPinNumber, 'B', 15);         // M1_STEP
_MAKE_MOTATE_PIN(kSocket1_EnablePinNumber, 'B', 16);       // M1_ENABLE
_MAKE_MOTATE_PIN(kSocket1_VrefPinNumber, 'B', 17);         // M1_Vref
_MAKE_MOTATE_PIN(kSocket2_VrefPinNumber, 'B', 18);         // M2_Vref
_MAKE_MOTATE_PIN(kSocket3_VrefPinNumber, 'B', 19);         // M3_Vref
_MAKE_MOTATE_PIN(kSocket2_Microstep_2PinNumber, 'B', 20);  // M2_MS2
_MAKE_MOTATE_PIN(kSocket2_Microstep_0PinNumber, 'B', 21);  // M2_MS0 (M2_MS1 is slaved to this signal also)
_MAKE_MOTATE_PIN(kSocket4_EnablePinNumber, 'B', 22);       // M4_ENABLE
_MAKE_MOTATE_PIN(kSocket4_DirPinNumber, 'B', 23);          // M4_DIR
_MAKE_MOTATE_PIN(kOutput11_PinNumber, 'B', 24);            // DO_11 (Header Bed FET)
_MAKE_MOTATE_PIN(kOutput5_PinNumber, 'B', 25);             // DO_5 (Fan2A_PWM)
_MAKE_MOTATE_PIN(kInput1_PinNumber, 'B', 26);              // DI_1 (XMin)
_MAKE_MOTATE_PIN(kOutput4_PinNumber, 'B', 27);             // DO_4 (Fan1A_PWM)
_MAKE_MOTATE_PIN(kUnassigned17, 'B', 28);                  // JTAG_CLK / SWD_CLK (unassigned)
_MAKE_MOTATE_PIN(kUnassigned18, 'B', 29);                  // JTAG_TDI (unassigned)
_MAKE_MOTATE_PIN(kUnassigned19, 'B', 30);                  // JTAG_TDO (unassigned)
_MAKE_MOTATE_PIN(kUnassigned20, 'B', 31);                  // JTAG_TMS / SWD_DIO (unassigned)

}  // namespace Motate

// We then allow each chip-type to have it's own function definitions
// that will refer to these pin assignments.
#include "motate_chip_pin_functions.h"

#endif
// printrboardG2_v3_pinout_h
