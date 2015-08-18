/*
 * g2ref_revA-pinout.h - tinyg2 board pinout specification
 * This file is part of the TinyG project
 *
 * Copyright (c) 2013-2015 Robert Giseburt
 * Copyright (c) 2013-2015 Alden S. Hart Jr.
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

#ifndef g2ref_pinout_h
#define g2ref_pinout_h

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
 *
 *  See motate_pin_assignments.h for pin names to be used int he rest of the G2 code.
 *  See Endnotes in this file for examples
 */

#include <MotateTimers.h>

namespace Motate {

    // Arduino pin name & function
    _MAKE_MOTATE_PIN(113,   A, 'A',  0);    // kSpindle_DirPinNumber
    _MAKE_MOTATE_PIN(116,   A, 'A',  1);    // kCoolant_EnablePinNumber
    _MAKE_MOTATE_PIN( 48,   A, 'A',  2);    // Socket4_VrefPinNumber
    _MAKE_MOTATE_PIN( 58,   A, 'A',  3);    // Socket5_VrefPinNumber
    _MAKE_MOTATE_PIN(150,   A, 'A',  4);    // ADC2: kADC2_PinNumber (Heated bed thermistor ADC)
    _MAKE_MOTATE_PIN(130,   A, 'A',  5);    // D0_1: kOutput6_PinNumber (Extruder1_PWM)
    _MAKE_MOTATE_PIN(133,   A, 'A',  6);    // D0_4: kOutput4_PinNumber (Fan1B_PWM)
    _MAKE_MOTATE_PIN(138,   A, 'A',  7);    // <unassigned, available out>
    _MAKE_MOTATE_PIN(131,   A, 'A',  8);    // D0_2: kOutput2_PinNumber (Extruder1_PWM)
    _MAKE_MOTATE_PIN(101,   A, 'A',  9);    // DI_2: kInput2_PinNumber
    _MAKE_MOTATE_PIN(102,   A, 'A', 10);    // DI_3: kInput3_PinNumber
    _MAKE_MOTATE_PIN(103,   A, 'A', 11);    // DI_4: kInput4_PinNumber
    _MAKE_MOTATE_PIN(  0,   A, 'A', 12);    // kSerial_RX
    _MAKE_MOTATE_PIN(  1,   A, 'A', 13);    // kSerial_TX
    _MAKE_MOTATE_PIN(  8,   A, 'A', 14);    // kSerial_RTS
    _MAKE_MOTATE_PIN(  9,   A, 'A', 15);    // kSerial_CTS
    _MAKE_MOTATE_PIN( 14,   A, 'A', 16);    // Socket1_EnablePinNumber
    _MAKE_MOTATE_PIN(112,   A, 'A', 17);    // kSpindle_EnablePinNumber
    _MAKE_MOTATE_PIN(104,   A, 'A', 18);    // DI_5: kInput5_PinNumber
    _MAKE_MOTATE_PIN(105,   A, 'A', 19);    // DI_6: kInput6_PinNumber
    _MAKE_MOTATE_PIN(106,   A, 'A', 20);    // DI_7: kInput7_PinNumber
    _MAKE_MOTATE_PIN(107,   A, 'A', 21);    // DI_8: kInput8_PinNumber
    _MAKE_MOTATE_PIN(152,   A, 'A', 22);    // kADC4_PinNumber (Extruder2_ADC)
    _MAKE_MOTATE_PIN(151,   A, 'A', 23);    // kADC5_PinNumber (Extruder1_ADC)
    _MAKE_MOTATE_PIN( 13,   A, 'A', 24);    // Socket1_DirPinNumber
    _MAKE_MOTATE_PIN(  5,   A, 'A', 25);    // SPI0_MISOPinNumber
    _MAKE_MOTATE_PIN(  6,   A, 'A', 26);    // SPI0_MOSIPinNumber
    _MAKE_MOTATE_PIN(  4,   A, 'A', 27);    // SPI0_SCKPinNumber
    _MAKE_MOTATE_PIN( 10,   A, 'A', 28);    // Socket1_SPISlaveSelectPinNumber
    _MAKE_MOTATE_PIN( 50,   A, 'A', 29);    // Socket1_SPISlaveSelectPinNumber

    _MAKE_MOTATE_PIN( 22, B, 'B',  0);  // Socket2_StepPinNumber
    _MAKE_MOTATE_PIN( 24, B, 'B',  1);  // Socket2_EnablePinNumber
    _MAKE_MOTATE_PIN( 23, B, 'B',  2);  // Socket2_DirPinNumber
    _MAKE_MOTATE_PIN( 32, B, 'B',  3);  // Socket3_StepPinNumber
    _MAKE_MOTATE_PIN( 34, B, 'B',  4);  // Socket3_EnablePinNumber
    _MAKE_MOTATE_PIN( 33, B, 'B',  5);  // Socket3_DirPinNumber
    _MAKE_MOTATE_PIN( 42, B, 'B',  6);  // Socket4_StepPinNumber
    _MAKE_MOTATE_PIN( 44, B, 'B',  7);  // Socket4_EnablePinNumber
    _MAKE_MOTATE_PIN( 43, B, 'B',  8);  // Socket4_DirPinNumber
    _MAKE_MOTATE_PIN( 52, B, 'B',  9);  // Socket5_StepPinNumber
    _MAKE_MOTATE_PIN( 54, B, 'B', 10);  // Socket5_EnablePinNumber
    _MAKE_MOTATE_PIN( 53, B, 'B', 11);  // Socket5_DirPinNumber
    _MAKE_MOTATE_PIN(163, B, 'B', 12);  // NOT PINNED OUT PHYSICALLY - defined to avoid compilation errors
    _MAKE_MOTATE_PIN(164, B, 'B', 13);  // NOT PINNED OUT PHYSICALLY - defined to avoid compilation errors
    _MAKE_MOTATE_PIN(141, B, 'B', 14);  // DO_12: kOutput12_PinNumber (Indicator_LED)
    _MAKE_MOTATE_PIN(119, B, 'B', 15);  // SD_CardDetect
    _MAKE_MOTATE_PIN( 12, B, 'B', 16);  // Socket1_StepPinNumber
    _MAKE_MOTATE_PIN( 18, B, 'B', 17);  // Socket1_VrefPinNumber
    _MAKE_MOTATE_PIN( 28, B, 'B', 18);  // Socket2_VrefPinNumber
    _MAKE_MOTATE_PIN( 38, B, 'B', 19);  // Socket3_VrefPinNumber
    _MAKE_MOTATE_PIN( 20, B, 'B', 20);  // Socket2_SPISlaveSelectPinNumber
    _MAKE_MOTATE_PIN( 30, B, 'B', 21);  // Socket3_SPISlaveSelectPinNumber
    _MAKE_MOTATE_PIN(139, B, 'B', 22);  // DO_10: Socket4_SPISlaveSelectPinNumber
    _MAKE_MOTATE_PIN( 40, B, 'B', 23);  // Socket4_SPISlaveSelectPinNumber
    _MAKE_MOTATE_PIN(140, B, 'B', 24);  // DO_11: kOutput11_PinNumber (Header Bed FET)
    _MAKE_MOTATE_PIN(134, B, 'B', 25);  // DO_5: kOutput5_PinNumber (Fan2A_PWM)
    _MAKE_MOTATE_PIN(100, B, 'B', 26);  // DI_1: kInput1_PinNumber
    _MAKE_MOTATE_PIN(132, B, 'B', 27);  // DO_3: kOutput3_PinNumber (Fan1A_PWM)


    // We are using 200+ to mean "unused"
    _MAKE_MOTATE_PIN(200, B, 'B', 28);  // JTAG_CLK / SWD_CLK
    _MAKE_MOTATE_PIN(201, B, 'B', 29);  // JTAG_TDI
    _MAKE_MOTATE_PIN(202, B, 'B', 30);  // JTAG_TDO
    _MAKE_MOTATE_PIN(203, B, 'B', 31);  // JTAG_TMS / SWD_DIO

} // namespace Motate

// We then allow each chip-type to have it's onw function definitions
// that will refer to these pin assignments.
#include "motate_chip_pin_functions.h"

/*  END NOTES - EXAMPLES:
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
#endif

// g2ref_pinout_h

