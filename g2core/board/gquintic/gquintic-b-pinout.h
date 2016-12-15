/*
 * gquintic-a-pinout.h - board pinout specification
 * This file is part of the g2core project
 *
 * Copyright (c) 2016 Robert Giseburt
 * Copyright (c) 2016 Alden S. Hart Jr.
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

#ifndef gquintic_a_pinout_h
#define gquintic_a_pinout_h

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


#define ADC0_AVAILABLE 0
#define ADC1_AVAILABLE 0
#define ADC2_AVAILABLE 0
#define ADC3_AVAILABLE 0

#define XIO_HAS_USB 1
#define XIO_HAS_UART 1
#define XIO_HAS_SPI 0
#define XIO_HAS_I2C 0

#define TEMPERATURE_OUTPUT_ON 0  // NO ADC yet

// Some pins, if the PWM capability is turned on, it will cause timer conflicts.
// So we have to explicitly enable them as PWM pins.
// Generated with:
// perl -e 'for($i=1;$i<14;$i++) { print "#define OUTPUT${i}_PWM 0\n";}'
#define OUTPUT1_PWM 1   // TC0,1 - Fet 1
#define OUTPUT2_PWM 1   // PWM1 - Fet 2
#define OUTPUT3_PWM 1   // TC1,0 - Fan 1
#define OUTPUT4_PWM 1   // TC1,1 - Fan 2
#define OUTPUT5_PWM 1   // TC2,0 - Fan 3
#define OUTPUT6_PWM 1   // PWM8+0
#define OUTPUT7_PWM 1   // PWM3
#define OUTPUT8_PWM 1   // PWM2
#define OUTPUT9_PWM 0   // PWM2
#define OUTPUT10_PWM 1  // PWM8+2
#define OUTPUT11_PWM 1  // PWM8+3 - Fet 3
#define OUTPUT12_PWM 0  // Unused
#define OUTPUT13_PWM 0  // Unused

namespace Motate {

// Unused:
//
//
//
//
//

_MAKE_MOTATE_PIN(kLED_RGBWPixelPinNumber, 'A', 0);                 //
_MAKE_MOTATE_PIN(kOutput1_PinNumber, 'A', 1);                 //
_MAKE_MOTATE_PIN(kOutput2_PinNumber, 'A', 2);           //
_MAKE_MOTATE_PIN(kI2C1_SDAPinNumber, 'A', 3);                 //
_MAKE_MOTATE_PIN(kI2C1_SCLPinNumber, 'A', 4);                 //
_MAKE_MOTATE_PIN(kOutput11_PinNumber, 'A', 5);                       //
_MAKE_MOTATE_PIN(kExternalClock1_PinNumber, 'A', 6);          // CPU_CLK
_MAKE_MOTATE_PIN(kOutput7_PinNumber, 'A', 7);                        //
_MAKE_MOTATE_PIN(kSerial_RTSPinNumber, 'A', 8);                       //
_MAKE_MOTATE_PIN(kSerial_RXPinNumber, 'A', 9);                         //
_MAKE_MOTATE_PIN(kSerial_TXPinNumber, 'A', 10);                        //
_MAKE_MOTATE_PIN(kSocket2_EnablePinNumber, 'A', 11);  //
_MAKE_MOTATE_PIN(kOutput6_PinNumber, 'A', 12);               //
_MAKE_MOTATE_PIN(kOutput8_PinNumber, 'A', 13);               //
_MAKE_MOTATE_PIN(kSocket1_StepPinNumber, 'A', 14);                //
_MAKE_MOTATE_PIN(kOutput3_PinNumber, 'A', 15);                //
_MAKE_MOTATE_PIN(kOutput4_PinNumber, 'A', 16);                //
_MAKE_MOTATE_PIN(kADC3_PinNumber, 'A', 17);                   //
_MAKE_MOTATE_PIN(kADC2_PinNumber, 'A', 18);                   //
_MAKE_MOTATE_PIN(kADC1_PinNumber, 'A', 19);                   //
_MAKE_MOTATE_PIN(kADC0_PinNumber, 'A', 20);                   //
_MAKE_MOTATE_PIN(kSerial_CTSPinNumber, 'A', 21);                      //
_MAKE_MOTATE_PIN(kSocket1_EnablePinNumber, 'A', 22);  //
_MAKE_MOTATE_PIN(kOutput10_PinNumber, 'A', 23);                //
_MAKE_MOTATE_PIN(kSocket3_EnablePinNumber, 'A', 24);               //
_MAKE_MOTATE_PIN(kSocket2_DirPinNumber, 'A', 25);                //
_MAKE_MOTATE_PIN(kOutput5_PinNumber, 'A', 26);                // On Timer 2!
_MAKE_MOTATE_PIN(kSocket3_StepPinNumber, 'A', 27);                // On Timer 2!
_MAKE_MOTATE_PIN(kUnassigned1, 'A', 28);                     // DIAG1
_MAKE_MOTATE_PIN(kUnassigned2, 'A', 29);                 //
_MAKE_MOTATE_PIN(kInput1_PinNumber, 'A', 30);                 //
_MAKE_MOTATE_PIN(kInput4_PinNumber, 'A', 31);                 //

_MAKE_MOTATE_PIN(kInput12_PinNumber, 'B', 0);                //
_MAKE_MOTATE_PIN(kInput11_PinNumber, 'B', 1);                //
_MAKE_MOTATE_PIN(kSocket1_SPISlaveSelectPinNumber, 'B', 2);  //
_MAKE_MOTATE_PIN(kOutputSAFE_PinNumber, 'B', 3);                       //
//_MAKE_MOTATE_PIN(                                 , 'B',  4); // TDI
//_MAKE_MOTATE_PIN(                                 , 'B',  5); // TRACESDO
//_MAKE_MOTATE_PIN(                                 , 'B',  6); // SWDIO
//_MAKE_MOTATE_PIN(                                 , 'B',  7); // SWDCLK
//_MAKE_MOTATE_PIN(                                 , 'B',  8); // XOUT
//_MAKE_MOTATE_PIN(                                 , 'B',  9); // XIN
//_MAKE_MOTATE_PIN(                                 , 'B', 10); // USB_D-
//_MAKE_MOTATE_PIN(                                 , 'B', 11); // USB_D+
//_MAKE_MOTATE_PIN(                                 , 'B', 12); // ERASE
_MAKE_MOTATE_PIN(kLED_USBRXPinNumber, 'B', 13);               // LED_1 (Heartbeat) - PWM2
_MAKE_MOTATE_PIN(kSocket4_SPISlaveSelectPinNumber, 'B', 14);  // NOT CONNECTED


//_MAKE_MOTATE_PIN(                                 , 'D',  0); // USB_VBUS
_MAKE_MOTATE_PIN(kInput9_PinNumber, 'D', 1);         //
_MAKE_MOTATE_PIN(kInput10_PinNumber, 'D', 2);          //
_MAKE_MOTATE_PIN(kInput8_PinNumber, 'D', 3);          //
_MAKE_MOTATE_PIN(kInput7_PinNumber, 'D', 4);          //
_MAKE_MOTATE_PIN(kInput6_PinNumber, 'D', 5);          //
_MAKE_MOTATE_PIN(kInput5_PinNumber, 'D', 6);          //
_MAKE_MOTATE_PIN(kInput3_PinNumber, 'D', 7);          //
_MAKE_MOTATE_PIN(kInput2_PinNumber, 'D', 8);              // ]
_MAKE_MOTATE_PIN(kUnassigned3, 'D', 9);      // DIAG0
_MAKE_MOTATE_PIN(kUnassigned4, 'D', 10);    //
_MAKE_MOTATE_PIN(kSocket5_StepPinNumber, 'D', 11);     //
_MAKE_MOTATE_PIN(kSocket3_SPISlaveSelectPinNumber, 'D', 12);    //
_MAKE_MOTATE_PIN(kSocket5_DirPinNumber, 'D', 13);              //
_MAKE_MOTATE_PIN(kSocket5_EnablePinNumber, 'D', 14);     //
_MAKE_MOTATE_PIN(kUnassigned5, 'D', 15);              //
_MAKE_MOTATE_PIN(kSocket4_StepPinNumber, 'D', 16);  //
_MAKE_MOTATE_PIN(kSocket4_DirPinNumber, 'D', 17);     //
_MAKE_MOTATE_PIN(kSocket3_DirPinNumber, 'D', 18);              //
_MAKE_MOTATE_PIN(kUnassigned6, 'D', 19);  //
_MAKE_MOTATE_PIN(kSPI0_MISOPinNumber, 'D', 20);    //
_MAKE_MOTATE_PIN(kSPI0_MOSIPinNumber, 'D', 21);     //
_MAKE_MOTATE_PIN(kSPI0_SCKPinNumber, 'D', 22);              //
_MAKE_MOTATE_PIN(kUnassigned7, 'D', 23);        //
_MAKE_MOTATE_PIN(kSocket2_StepPinNumber, 'D', 24);  //
_MAKE_MOTATE_PIN(kSocket2_SPISlaveSelectPinNumber, 'D', 25);     //
_MAKE_MOTATE_PIN(kOutput9_PinNumber, 'D', 26);    //
_MAKE_MOTATE_PIN(kSocket1_DirPinNumber, 'D', 27);  //
_MAKE_MOTATE_PIN(kSocket4_EnablePinNumber, 'D', 28);    //
_MAKE_MOTATE_PIN(kUnassigned8, 'D', 29);       //
_MAKE_MOTATE_PIN(kUnassigned9, 'D', 30);              // INTERRUPT_OUT
_MAKE_MOTATE_PIN(kUnassigned10, 'D', 31);             //

}  // namespace Motate

// We then allow each chip-type to have it's onw function definitions
// that will refer to these pin assignments.
#include "motate_chip_pin_functions.h"

#endif

// gquintic_a_pinout_h
