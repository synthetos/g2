/*
 * g2ref_revA-pinout.h - tinyg2 board pinout specification
 * This file is part of the TinyG project
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

#ifndef pboard_a_pinout_h
#define pboard_a_pinout_h

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


#include <MotateTimers.h>

// We don't have all of the inputs, so we have to indicate as much:
#define INPUT1_AVAILABLE 1
#define INPUT2_AVAILABLE 0
#define INPUT3_AVAILABLE 0
#define INPUT4_AVAILABLE 1
#define INPUT5_AVAILABLE 1
#define INPUT6_AVAILABLE 0
#define INPUT7_AVAILABLE 0
#define INPUT8_AVAILABLE 1
#define INPUT9_AVAILABLE 0
#define INPUT10_AVAILABLE 0
#define INPUT11_AVAILABLE 0
#define INPUT12_AVAILABLE 0
#define INPUT13_AVAILABLE 0


#define ADC0_AVAILABLE 1
#define ADC1_AVAILABLE 1
#define ADC2_AVAILABLE 1
#define ADC3_AVAILABLE 1

#define TEMPERATURE_OUTPUT_ON 0


namespace Motate {

    // Arduino pin name & function
    _MAKE_MOTATE_PIN(kOutput7_PinNumber                 , A, 'A',  0); // Conditioned digital output, 3.3v, 5v
    _MAKE_MOTATE_PIN(kOutput8_PinNumber                 , A, 'A',  1); // Conditioned digital output, 3.3v, 5v
    _MAKE_MOTATE_PIN(kSocket4_VrefPinNumber             , A, 'A',  2);
    _MAKE_MOTATE_PIN(kSocket5_VrefPinNumber             , A, 'A',  3);
    _MAKE_MOTATE_PIN(kADC0_PinNumber                    , A, 'A',  4); // Heated bed thermistor ADC
    _MAKE_MOTATE_PIN(kOutput1_PinNumber                 , A, 'A',  5); // DO_1: Extruder1_PWM
    _MAKE_MOTATE_PIN(kOutput4_PinNumber                 , A, 'A',  6); // DO_4: Fan1B_PWM
    _MAKE_MOTATE_PIN(kOutput9_PinNumber                 , A, 'A',  7); // DO_9: SAFEin (Output) signal
    _MAKE_MOTATE_PIN(kOutput2_PinNumber                 , A, 'A',  8); // DO_2: Extruder2_PWM
    _MAKE_MOTATE_PIN(kInput4_PinNumber                  , A, 'A',  9); // DI_4: YMax
    _MAKE_MOTATE_PIN(kInput5_PinNumber                  , A, 'A', 10); // DI_5: ZMin
    _MAKE_MOTATE_PIN(kUnassigned1                       , A, 'A', 11); // ???
    _MAKE_MOTATE_PIN(kSerial_RX                         , A, 'A', 12); // kSerial_RX
    _MAKE_MOTATE_PIN(kSerial_TX                         , A, 'A', 13); // kSerial_TX
    _MAKE_MOTATE_PIN(kSerial_RTS                        , A, 'A', 14); // kSerial_RTS
    _MAKE_MOTATE_PIN(kSerial_CTS                        , A, 'A', 15); // kSerial_CTS
    _MAKE_MOTATE_PIN(kSocket1_DirPinNumber              , A, 'A', 16); //
    _MAKE_MOTATE_PIN(kOutput6_PinNumber                 , A, 'A', 17); // Conditioned digital output, 3.3v, 5v
    _MAKE_MOTATE_PIN(kUnassigned2                       , A, 'A', 18); // ???
    _MAKE_MOTATE_PIN(kUnassigned3                       , A, 'A', 19); // ???
    _MAKE_MOTATE_PIN(kSD_CardDetectPinNumber            , A, 'A', 20); //
    _MAKE_MOTATE_PIN(kInput8_PinNumber                  , A, 'A', 21); // DI_8: kInput8_PinNumber
    _MAKE_MOTATE_PIN(kADC2_PinNumber                    , A, 'A', 22); // Extruder2_ADC
    _MAKE_MOTATE_PIN(kADC1_PinNumber                    , A, 'A', 23); // Extruder1_ADC
    _MAKE_MOTATE_PIN(kADC3_PinNumber                    , A, 'A', 24); // Aux ADC
    _MAKE_MOTATE_PIN(kSPI0_MISOPinNumber                , A, 'A', 25); // SPI0_MISOPinNumber
    _MAKE_MOTATE_PIN(kSPI0_MOSIPinNumber                , A, 'A', 26); // SPI0_MOSIPinNumber
    _MAKE_MOTATE_PIN(kSPI0_SCKPinNumber                 , A, 'A', 27); // SPI0_SCKPinNumber
    _MAKE_MOTATE_PIN(kSD_ChipSelectPinNumber            , A, 'A', 28); // SPI-CS0 (SD card)
    _MAKE_MOTATE_PIN(kSocket4_SPISlaveSelectPinNumber   , A, 'A', 29); // <reserved - SPI-CS4>

    _MAKE_MOTATE_PIN(kSocket2_StepPinNumber             , B, 'B',  0);  // Socket2_StepPinNumber
    _MAKE_MOTATE_PIN(kSocket2_EnablePinNumber           , B, 'B',  1);  // Socket2_EnablePinNumber
    _MAKE_MOTATE_PIN(kSocket2_DirPinNumber              , B, 'B',  2);  // Socket2_DirPinNumber
    _MAKE_MOTATE_PIN(kSocket3_StepPinNumber             , B, 'B',  3);  // Socket3_StepPinNumber
    _MAKE_MOTATE_PIN(kSocket3_EnablePinNumber           , B, 'B',  4);  // Socket3_EnablePinNumber
    _MAKE_MOTATE_PIN(kSocket3_DirPinNumber              , B, 'B',  5);  // Socket3_DirPinNumber
    _MAKE_MOTATE_PIN(kSocket4_StepPinNumber             , B, 'B',  6);  // Socket4_StepPinNumber
    _MAKE_MOTATE_PIN(kSocket4_EnablePinNumber           , B, 'B',  7);  // Socket4_EnablePinNumber
    _MAKE_MOTATE_PIN(kSocket4_DirPinNumber              , B, 'B',  8);  // Socket4_DirPinNumber
    _MAKE_MOTATE_PIN(kSocket5_StepPinNumber             , B, 'B',  9);  // Socket5_StepPinNumber
    _MAKE_MOTATE_PIN(kSocket5_EnablePinNumber           , B, 'B', 10);  // Socket5_EnablePinNumber
    _MAKE_MOTATE_PIN(kSocket5_DirPinNumber              , B, 'B', 11);  // Socket5_DirPinNumber
    _MAKE_MOTATE_PIN(kI2C1_SDAPinNumber                 , B, 'B', 12);  // TWD1
    _MAKE_MOTATE_PIN(kI2C1_SCLPinNumber                 , B, 'B', 13);  // TWCK1
    _MAKE_MOTATE_PIN(kOutput12_PinNumber                , B, 'B', 14);  // DO_12: kOutput12_PinNumber (Indicator_LED)
    _MAKE_MOTATE_PIN(kSocket1_StepPinNumber             , B, 'B', 15);  //
    _MAKE_MOTATE_PIN(kSocket1_EnablePinNumber           , B, 'B', 16);  //
    _MAKE_MOTATE_PIN(kSocket1_VrefPinNumber             , B, 'B', 17);  // Socket1_VrefPinNumber
    _MAKE_MOTATE_PIN(kSocket2_VrefPinNumber             , B, 'B', 18);  // Socket2_VrefPinNumber
    _MAKE_MOTATE_PIN(kSocket3_VrefPinNumber             , B, 'B', 19);  // Socket3_VrefPinNumber
    _MAKE_MOTATE_PIN(kSocket1_SPISlaveSelectPinNumber   , B, 'B', 20);  //
    _MAKE_MOTATE_PIN(kSocket2_SPISlaveSelectPinNumber   , B, 'B', 21);  //
    _MAKE_MOTATE_PIN(kOutput10_PinNumber                , B, 'B', 22);  // DO_10: Socket4_SPISlaveSelectPinNumber
    _MAKE_MOTATE_PIN(kSocket3_SPISlaveSelectPinNumber   , B, 'B', 23);  //
    _MAKE_MOTATE_PIN(kOutput11_PinNumber                , B, 'B', 24);  // DO_11: (Header Bed FET)
    _MAKE_MOTATE_PIN(kOutput5_PinNumber                 , B, 'B', 25);  // DO_5: (Fan2A_PWM)
    _MAKE_MOTATE_PIN(kInput1_PinNumber                  , B, 'B', 26);  // DI_1: XMin
    _MAKE_MOTATE_PIN(kOutput3_PinNumber                 , B, 'B', 27);  // DO_3: kOutput3_PinNumber (Fan1A_PWM)
    _MAKE_MOTATE_PIN(kUnassigned4                       , B, 'B', 28);	// JTAG_CLK / SWD_CLK (unassigned)
    _MAKE_MOTATE_PIN(kUnassigned5                       , B, 'B', 29);	// JTAG_TDI (unassigned)
    _MAKE_MOTATE_PIN(kUnassigned6                       , B, 'B', 30);	// JTAG_TDO (unassigned)
    _MAKE_MOTATE_PIN(kUnassigned7                       , B, 'B', 31);	// JTAG_TMS / SWD_DIO (unassigned)

} // namespace Motate

// We then allow each chip-type to have it's onw function definitions
// that will refer to these pin assignments.
#include "motate_chip_pin_functions.h"

#endif

// pboard_a_pinout_h

