/*
 * g2v9k-pinout.h - board pinout specification
 * This file is part of the g2core project
 *
 * Copyright (c) 2013 - 2016 Robert Giseburt
 * Copyright (c) 2013 - 2016 Alden S. Hart Jr.
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

#ifndef g2v9_pinout_h
#define g2v9_pinout_h

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


#include <MotatePins.h>

// We don't have all of the inputs, so we don't define them.
#define INPUT1_AVAILABLE 1
#define INPUT2_AVAILABLE 1
#define INPUT3_AVAILABLE 1
#define INPUT4_AVAILABLE 1
#define INPUT5_AVAILABLE 1
#define INPUT6_AVAILABLE 1
#define INPUT7_AVAILABLE 1
#define INPUT8_AVAILABLE 1
#define INPUT9_AVAILABLE 0
#define INPUT10_AVAILABLE 0
#define INPUT11_AVAILABLE 0
#define INPUT12_AVAILABLE 0
#define INPUT13_AVAILABLE 0

#define ADC0_AVAILABLE 1
#define ADC1_AVAILABLE 1
#define ADC2_AVAILABLE 0
#define ADC3_AVAILABLE 0

#define XIO_HAS_USB 1
#define XIO_HAS_UART 0
#define XIO_HAS_SPI 0
#define XIO_HAS_I2C 0

#define TEMPERATURE_OUTPUT_ON 0

// Some pins, if the PWM capability is turned on, it will cause timer conflicts.
// So we have to explicity enable them as PWM pins.
// Generated with:
// perl -e 'for($i=1;$i<14;$i++) { print "#define OUTPUT${i}_PWM 0\n";}'
#define OUTPUT1_PWM 0
#define OUTPUT2_PWM 0
#define OUTPUT3_PWM 0
#define OUTPUT4_PWM 0
#define OUTPUT5_PWM 0
#define OUTPUT6_PWM 0
#define OUTPUT7_PWM 0
#define OUTPUT8_PWM 0
#define OUTPUT9_PWM 0
#define OUTPUT10_PWM 0
#define OUTPUT11_PWM 0
#define OUTPUT12_PWM 0
#define OUTPUT13_PWM 0

namespace Motate {

// Pin name and function
_MAKE_MOTATE_PIN(kUnassigned1, 'A', 0);                       // unassigned and disconnected
_MAKE_MOTATE_PIN(kCoolant_EnablePinNumber, 'A', 1);           // Coolant_EnablePinNumber
_MAKE_MOTATE_PIN(kSocket4_VrefPinNumber, 'A', 2);             // Socket4_VrefPinNumber
_MAKE_MOTATE_PIN(kSocket5_VrefPinNumber, 'A', 3);             // Socket5_VrefPinNumber
_MAKE_MOTATE_PIN(kSocket2_Microstep_2PinNumber, 'A', 4);      // Socket2_Microstep_2PinNumber
_MAKE_MOTATE_PIN(kSpindle_EnablePinNumber, 'A', 5);           // Spindle_EnablePinNumber
_MAKE_MOTATE_PIN(kSocket1_DirPinNumber, 'A', 6);              // Socket1_DirPinNumber
_MAKE_MOTATE_PIN(kSpindle_DirPinNumber, 'A', 7);              // Spindle_DirPinNumber
_MAKE_MOTATE_PIN(kSpindle_PwmPinNumber, 'A', 8);              // Spindle_PwmPinNumber
_MAKE_MOTATE_PIN(kInput2_PinNumber, 'A', 9);                  // XAxis_MaxPinNumber
_MAKE_MOTATE_PIN(kInput3_PinNumber, 'A', 10);                 // YAxis_MinPinNumber
_MAKE_MOTATE_PIN(kInput4_PinNumber, 'A', 11);                 // YAxis_MaxPinNumber
_MAKE_MOTATE_PIN(kInput5_PinNumber, 'A', 12);                 // ZAxis_MinPinNumber
_MAKE_MOTATE_PIN(kInput6_PinNumber, 'A', 13);                 // ZAxis_MaxPinNumber
_MAKE_MOTATE_PIN(kInput7_PinNumber, 'A', 14);                 // AAxis_MinPinNumber
_MAKE_MOTATE_PIN(kInput8_PinNumber, 'A', 15);                 // AAxis_MaxPinNumber
_MAKE_MOTATE_PIN(kSocket1_Microstep_1PinNumber, 'A', 16);     // Socket1_Microstep_1PinNumber
_MAKE_MOTATE_PIN(kSD_ChipSelectPinNumber, 'A', 17);           // Interlock_In
_MAKE_MOTATE_PIN(kLED_USBRXPinNumber, 'A', 18);               // LED_USBRXPinNumber
_MAKE_MOTATE_PIN(kLED_USBTXPinNumber, 'A', 19);               // LED_USBTXPinNumber
_MAKE_MOTATE_PIN(kUnassigned2, 'A', 20);                      // unassigned and disconnected
_MAKE_MOTATE_PIN(kSocket2_Microstep_1PinNumber, 'A', 21);     // Socket2_Microstep_1PinNumber
_MAKE_MOTATE_PIN(kSocket1_EnablePinNumber, 'A', 22);          // Socket1_EnablePinNumber
_MAKE_MOTATE_PIN(kSocket1_StepPinNumber, 'A', 23);            // Socket1_StepPinNumber
_MAKE_MOTATE_PIN(kSocket1_Microstep_0PinNumber, 'A', 24);     // Socket1_Microstep_0PinNumber
_MAKE_MOTATE_PIN(kSPI0_MISOPinNumber, 'A', 25);               // SPI0_MISOPinNumber
_MAKE_MOTATE_PIN(kSPI0_MOSIPinNumber, 'A', 26);               // SPI0_MOSIPinNumber
_MAKE_MOTATE_PIN(kSPI0_SCKPinNumber, 'A', 27);                // SPI0_SCKPinNumber
_MAKE_MOTATE_PIN(kSocket1_SPISlaveSelectPinNumber, 'A', 28);  // Socket1_SPISlaveSelectPinNumber
_MAKE_MOTATE_PIN(kSocket2_Microstep_0PinNumber, 'A', 29);     // Socket2_Microstep_0PinNumber

_MAKE_MOTATE_PIN(kSocket2_StepPinNumber, 'B', 0);             // Socket2_StepPinNumber
_MAKE_MOTATE_PIN(kSocket2_EnablePinNumber, 'B', 1);           // Socket2_EnablePinNumber
_MAKE_MOTATE_PIN(kSocket2_DirPinNumber, 'B', 2);              // Socket2_DirPinNumber
_MAKE_MOTATE_PIN(kSocket3_Microstep_2PinNumber, 'B', 3);      // Socket3_Microstep_2PinNumber
_MAKE_MOTATE_PIN(kSocket3_Microstep_1PinNumber, 'B', 4);      // Socket3_Microstep_1PinNumber
_MAKE_MOTATE_PIN(kSocket3_Microstep_0PinNumber, 'B', 5);      // Socket3_Microstep_0PinNumber
_MAKE_MOTATE_PIN(kSocket3_StepPinNumber, 'B', 6);             // Socket3_StepPinNumber
_MAKE_MOTATE_PIN(kSocket3_EnablePinNumber, 'B', 7);           // Socket3_EnablePinNumber
_MAKE_MOTATE_PIN(kSocket3_DirPinNumber, 'B', 8);              // Socket3_DirPinNumber
_MAKE_MOTATE_PIN(kSocket4_Microstep_2PinNumber, 'B', 9);      // Socket4_Microstep_2PinNumber
_MAKE_MOTATE_PIN(kSocket4_Microstep_1PinNumber, 'B', 10);     // Socket4_Microstep_1PinNumber
_MAKE_MOTATE_PIN(kSocket4_Microstep_0PinNumber, 'B', 11);     // Socket4_Microstep_0PinNumber
_MAKE_MOTATE_PIN(kUnassigned3, 'B', 12);                      // unassigned and disconnected
_MAKE_MOTATE_PIN(kUnassigned4, 'B', 13);                      // unassigned and disconnected
_MAKE_MOTATE_PIN(kSocket4_StepPinNumber, 'B', 14);            // Socket4_StepPinNumber
_MAKE_MOTATE_PIN(kSD_CardDetectPinNumber, 'B', 15);           // SD_CardDetect
_MAKE_MOTATE_PIN(kSocket1_Microstep_2PinNumber, 'B', 16);     // Socket1_Microstep_2PinNumber
_MAKE_MOTATE_PIN(kSocket1_VrefPinNumber, 'B', 17);            // Socket1_VrefPinNumber
_MAKE_MOTATE_PIN(kSocket2_VrefPinNumber, 'B', 18);            // Socket2_VrefPinNumber
_MAKE_MOTATE_PIN(kSocket3_VrefPinNumber, 'B', 19);            // Socket3_VrefPinNumber
_MAKE_MOTATE_PIN(kADC0_PinNumber, 'B', 20);                   // Socket2_SPISlaveSelectPinNumber
_MAKE_MOTATE_PIN(kADC1_PinNumber, 'B', 21);                   // Socket3_SPISlaveSelectPinNumber
_MAKE_MOTATE_PIN(kSocket4_EnablePinNumber, 'B', 22);          // Socket4_EnablePinNumber
_MAKE_MOTATE_PIN(kSocket4_SPISlaveSelectPinNumber, 'B', 23);  // Socket4_SPISlaveSelectPinNumber
_MAKE_MOTATE_PIN(kSocket4_DirPinNumber, 'B', 24);             // Socket4_DirPinNumber
_MAKE_MOTATE_PIN(kUnassigned5, 'B', 25);                      // unassigned and disconnected
_MAKE_MOTATE_PIN(kInput1_PinNumber, 'B', 26);                 // XAxis_MinPinNumber
_MAKE_MOTATE_PIN(kUnassigned6, 'B', 27);                      // unassigned and disconnected
_MAKE_MOTATE_PIN(kUnassigned7, 'B', 28);                      // JTAG_CLK / SWD_CLK (unassigned)
_MAKE_MOTATE_PIN(kUnassigned8, 'B', 29);                      // JTAG_TDI (unassigned)
_MAKE_MOTATE_PIN(kUnassigned9, 'B', 30);                      // JTAG_TDO (unassigned)
_MAKE_MOTATE_PIN(kUnassigned10, 'B', 31);                     // JTAG_TMS / SWD_DIO (unassigned)

}  // namespace Motate


// We then allow each chip-type to have it's onw function definitions
// that will refer to these pin assignments.
#include "motate_chip_pin_functions.h"

#endif
