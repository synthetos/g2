/*
 * motate_pin_assignments.h - pin assignments for the gQuadratic boards
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

#ifndef motate_pin_assignments_h
#define motate_pin_assignments_h

// Board pinout is pulled in after naming, so we can use the naming there.

namespace Motate {

// NOT ALL OF THESE PINS ARE ON ALL PLATFORMS
// Undefined pins will be equivalent to Motate::NullPin, and return 1 for Pin<>::isNull();


pin_number kSerial_RXPinNumber  = 0;
pin_number kSerial_TXPinNumber  = 1;
pin_number kSerial_RTSPinNumber = 2;  // added later
pin_number kSerial_CTSPinNumber = 3;  // added later

pin_number kSerial0_RXPinNumber  = 0;
pin_number kSerial0_TXPinNumber  = 1;
pin_number kSerial0_RTSPinNumber = 2;  // added later
pin_number kSerial0_CTSPinNumber = 3;  // added later

pin_number kI2C_SDAPinNumber = 5;
pin_number kI2C_SCLPinNumber = 6;

pin_number kI2C0_SDAPinNumber = -1;  // not pinned out
pin_number kI2C0_SCLPinNumber = -1;  // not pinned out

pin_number kI2C1_SDAPinNumber = 5;
pin_number kI2C1_SCLPinNumber = 6;

pin_number kSPI_SCKPinNumber  = 7;
pin_number kSPI_MISOPinNumber = 8;
pin_number kSPI_MOSIPinNumber = 9;

pin_number kSPI0_SCKPinNumber  = 7;
pin_number kSPI0_MISOPinNumber = 8;
pin_number kSPI0_MOSIPinNumber = 9;

pin_number kKinen_SyncPinNumber = -1;  // not pinned out

pin_number kSocket1_SPISlaveSelectPinNumber = 10;
pin_number kSocket1_InterruptPinNumber      = 11;  // 11;
pin_number kSocket1_StepPinNumber           = 12;
pin_number kSocket1_DirPinNumber            = 13;
pin_number kSocket1_EnablePinNumber         = 14;
pin_number kSocket1_Microstep_0PinNumber    = 15;  // 15;
pin_number kSocket1_Microstep_1PinNumber    = 16;  // 16;
pin_number kSocket1_Microstep_2PinNumber    = 17;  // 17;
pin_number kSocket1_VrefPinNumber           = 18;

pin_number kSocket2_SPISlaveSelectPinNumber = 20;
pin_number kSocket2_InterruptPinNumber      = -1;  // 21;
pin_number kSocket2_StepPinNumber           = 22;
pin_number kSocket2_DirPinNumber            = 23;
pin_number kSocket2_EnablePinNumber         = 24;
pin_number kSocket2_Microstep_0PinNumber    = 25;  // 25;
pin_number kSocket2_Microstep_1PinNumber    = 26;  // 26;
pin_number kSocket2_Microstep_2PinNumber    = 27;  // 27;
pin_number kSocket2_VrefPinNumber           = 28;

pin_number kSocket3_SPISlaveSelectPinNumber = 30;
pin_number kSocket3_InterruptPinNumber      = -1;  // 31;
pin_number kSocket3_StepPinNumber           = 32;
pin_number kSocket3_DirPinNumber            = 33;
pin_number kSocket3_EnablePinNumber         = 34;
pin_number kSocket3_Microstep_0PinNumber    = 35;  // 35;
pin_number kSocket3_Microstep_1PinNumber    = 36;  // 36;
pin_number kSocket3_Microstep_2PinNumber    = 37;  // 37;
pin_number kSocket3_VrefPinNumber           = 38;

pin_number kSocket4_SPISlaveSelectPinNumber = 40;
pin_number kSocket4_InterruptPinNumber      = -1;  // 41;
pin_number kSocket4_StepPinNumber           = 42;
pin_number kSocket4_DirPinNumber            = 43;
pin_number kSocket4_EnablePinNumber         = 44;
pin_number kSocket4_Microstep_0PinNumber    = 45;  // 45;
pin_number kSocket4_Microstep_1PinNumber    = 46;  // 46;
pin_number kSocket4_Microstep_2PinNumber    = 47;  // 47;
pin_number kSocket4_VrefPinNumber           = 48;

pin_number kSocket5_SPISlaveSelectPinNumber = 50;
pin_number kSocket5_InterruptPinNumber      = -1;  // 51;
pin_number kSocket5_StepPinNumber           = 52;
pin_number kSocket5_DirPinNumber            = 53;
pin_number kSocket5_EnablePinNumber         = 54;
pin_number kSocket5_Microstep_0PinNumber    = 55;  // 55;
pin_number kSocket5_Microstep_1PinNumber    = 56;  // 56;
pin_number kSocket5_Microstep_2PinNumber    = 57;  // 57;
pin_number kSocket5_VrefPinNumber           = 58;

pin_number kSocket6_SPISlaveSelectPinNumber = -1;  // 60;
pin_number kSocket6_InterruptPinNumber      = -1;  // 61;
pin_number kSocket6_StepPinNumber           = -1;  // 62;
pin_number kSocket6_DirPinNumber            = -1;  // 63;
pin_number kSocket6_EnablePinNumber         = -1;  // 64;
pin_number kSocket6_Microstep_0PinNumber    = -1;  // 65;
pin_number kSocket6_Microstep_1PinNumber    = -1;  // 66;
pin_number kSocket6_Microstep_2PinNumber    = -1;  // 67;
pin_number kSocket6_VrefPinNumber           = -1;  // 68;

// We also have to define INPUTx_AVAILABLE so we know if we can make the interrupts
pin_number kInput1_PinNumber = 100;
pin_number kInput2_PinNumber = 101;
pin_number kInput3_PinNumber = 102;
pin_number kInput4_PinNumber = 103;
pin_number kInput5_PinNumber = 104;
pin_number kInput6_PinNumber = 105;

pin_number kInput7_PinNumber  = 106;
pin_number kInput8_PinNumber  = 107;
pin_number kInput9_PinNumber  = 108;
pin_number kInput10_PinNumber = 109;
pin_number kInput11_PinNumber = 110;
pin_number kInput12_PinNumber = 111;

// START DEBUG PINS - Convenient pins to hijack for hardware debugging
// To reuse a pin for debug change the original pin number to -1
// and uncomment the corresponding debug pin
pin_number kSpindle_EnablePinNumber = -1;  // 112       // Not implemented Spindle ON implemented in g2ref
pin_number kSpindle_DirPinNumber    = -1;  // 113       // Not implemented Spindle ON implemented in g2ref
pin_number kSpindle_PwmPinNumber    = -1;  // 114       // Not implemented (superseded by Extruder1_PWM)
pin_number kSpindle_Pwm2PinNumber   = -1;  // 115       // Not implemented (superseded by Fan1B_PWM)
pin_number kCoolant_EnablePinNumber = -1;  // 116       // Not implemented Spindle ON implemented in g2ref

pin_number kDebug1_PinNumber = -1;  // 112;
pin_number kDebug2_PinNumber = -1;  // 113;
pin_number kDebug3_PinNumber = -1;  // 116; //e Not the out-of-order numbering & 115 missing
pin_number kDebug4_PinNumber = -1;  // 114;
// END DEBUG PINS

pin_number kLED_USBRXPinNumber        = 117;
pin_number kLED_USBTXPinNumber        = 118;
pin_number kSD_CardDetectPinNumber    = 119;
pin_number kSD_ChipSelectPinNumber    = 120;
pin_number kInterlock_InPinNumber     = 121;
pin_number kOutputSAFE_PinNumber      = 122;  // SAFE signal
pin_number kLEDPWM_PinNumber          = 123;
pin_number kOutputInterrupt_PinNumber = 124;  // to-host interrupt signal
pin_number kLED_RGBWPixelPinNumber    = 125;  // 117;

// GRBL / gShield compatibility pins -- Due board ONLY

pin_number kGRBL_ResetPinNumber        = -1;
pin_number kGRBL_FeedHoldPinNumber     = -1;
pin_number kGRBL_CycleStartPinNumber   = -1;
pin_number kGRBL_CommonEnablePinNumber = -1;

// g2ref extensions
// These first 5 may replace the Spindle and Coolant pins, above
pin_number kOutput1_PinNumber = 130;  // DO_1: Extruder1_PWM
pin_number kOutput2_PinNumber = 131;  // DO_2: Extruder2_PWM
pin_number kOutput3_PinNumber = 132;  // DO_3: Fan1A_PWM
pin_number kOutput4_PinNumber = 133;  // DO_4: Fan1B_PWM
pin_number kOutput5_PinNumber = 134;  // DO_5: Fan2A_PWM

pin_number kOutput6_PinNumber  = 135;  // See Spindle Enable
pin_number kOutput7_PinNumber  = 136;  // See Spindle Direction
pin_number kOutput8_PinNumber  = 137;  // See Coolant Enable
pin_number kOutput9_PinNumber  = 138;  // SAFE signal
pin_number kOutput10_PinNumber = 139;  // DO_10: Fan2B_PWM

pin_number kOutput11_PinNumber = 140;  // DO_11: Heated Bed FET
pin_number kOutput12_PinNumber = 141;  // DO_12: Indicator_LED
pin_number kOutput13_PinNumber = -1;   // 142;
pin_number kOutput14_PinNumber = -1;   // 143;
pin_number kOutput15_PinNumber = -1;   // 144;
pin_number kOutput16_PinNumber = -1;   // 145;

pin_number kADC0_PinNumber  = 150;  // Heated bed thermistor ADC
pin_number kADC1_PinNumber  = 151;  // Extruder1_ADC
pin_number kADC2_PinNumber  = 152;  // Extruder2_ADC
pin_number kADC3_PinNumber  = 153;  // Aux ADC
pin_number kADC4_PinNumber  = 154;  // Not physically pinned out
pin_number kADC5_PinNumber  = 155;  // Not physically pinned out
pin_number kADC6_PinNumber  = 156;  // Not physically pinned out
pin_number kADC7_PinNumber  = 157;  // Not physically pinned out
pin_number kADC8_PinNumber  = 158;  // Not physically pinned out
pin_number kADC9_PinNumber  = 159;  // Not physically pinned out
pin_number kADC10_PinNumber = 160;  // Not physically pinned out
pin_number kADC11_PinNumber = 161;  // Not physically pinned out
pin_number kADC12_PinNumber = 162;  // Not physically pinned out
pin_number kADC13_PinNumber = 163;  // Not physically pinned out
pin_number kADC14_PinNumber = 164;  // Not physically pinned out

pin_number kExternalClock1_PinNumber = 170;  // External pins for exporting a clock signal (for Trinamics)

pin_number kServo1_PinNumber = 171;  //
pin_number kServo2_PinNumber = 172;  //
pin_number kServo3_PinNumber = 173;  //


// start next sequence at 174

// blank spots for unassigned pins - all unassigned pins need a unique number (do not re-use numbers)

pin_number kUnassigned38 = 217;
pin_number kUnassigned37 = 218;
pin_number kUnassigned36 = 219;
pin_number kUnassigned35 = 220;
pin_number kUnassigned34 = 221;
pin_number kUnassigned33 = 222;
pin_number kUnassigned32 = 223;
pin_number kUnassigned31 = 224;
pin_number kUnassigned30 = 225;
pin_number kUnassigned29 = 226;
pin_number kUnassigned28 = 227;
pin_number kUnassigned27 = 228;
pin_number kUnassigned26 = 229;
pin_number kUnassigned25 = 230;
pin_number kUnassigned24 = 231;
pin_number kUnassigned23 = 232;
pin_number kUnassigned22 = 233;
pin_number kUnassigned21 = 234;
pin_number kUnassigned20 = 235;
pin_number kUnassigned19 = 236;
pin_number kUnassigned18 = 237;
pin_number kUnassigned17 = 238;
pin_number kUnassigned16 = 239;
pin_number kUnassigned15 = 240;
pin_number kUnassigned14 = 241;
pin_number kUnassigned13 = 242;
pin_number kUnassigned12 = 243;
pin_number kUnassigned11 = 244;
pin_number kUnassigned10 = 245;
pin_number kUnassigned9  = 246;
pin_number kUnassigned8  = 247;
pin_number kUnassigned7  = 248;
pin_number kUnassigned6  = 249;
pin_number kUnassigned5  = 250;
pin_number kUnassigned4  = 251;
pin_number kUnassigned3  = 252;
pin_number kUnassigned2  = 253;
pin_number kUnassigned1  = 254;  // 254 is the max.. Do not exceed this number

/** NOTE: When adding pin definitions here, they must be
 *        added to ALL board pin assignment files, even if
 *        they are defined as -1.
 **/

}  // namespace Motate

// For the SAM3X8C boards, we actually use the same NUMBERING, but have different number to pin linkages

// We're putting this in to make the autocomplete work for XCode,
// since it doesn't understand the special syntax coming up.
#ifdef XCODE_INDEX
#include <gquadratic-a-pinout.h>
#endif

#ifdef MOTATE_BOARD
#define MOTATE_BOARD_PINOUT < MOTATE_BOARD-pinout.h >
#include MOTATE_BOARD_PINOUT
#else
#error Unknown board layout $(MOTATE_BOARD)
#endif

#endif

// motate_pin_assignments_h
