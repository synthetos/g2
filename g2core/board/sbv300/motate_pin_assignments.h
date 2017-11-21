/*
 * motate_pin_assignments.h - pin assignments for the shopbot sbv300 boards
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

#ifndef motate_pin_assignments_h
#define motate_pin_assignments_h

#include <MotateTimers.h>

namespace Motate {

// NOT ALL OF THESE PINS ARE ON ALL PLATFORMS
// Undefined pins will be equivalent to Motate::NullPin, and return 1 for Pin<>::isNull();

pin_number kSerial_RXPinNumber  = -1;
pin_number kSerial_TXPinNumber  = -1;
pin_number kSerial_RTSPinNumber = -1;  // added later
pin_number kSerial_CTSPinNumber = -1;  // added later

pin_number kSerial0_RX  = -1;
pin_number kSerial0_TX  = -1;
pin_number kSerial0_RTS = -1;  // added later
pin_number kSerial0_CTS = -1;  // added later

pin_number kI2C_SDAPinNumber = -1;
pin_number kI2C_SCLPinNumber = -1;

pin_number kI2C0_SDAPinNumber = -1;
pin_number kI2C0_SCLPinNumber = -1;

pin_number kSPI_SCKPinNumber  = -1;
pin_number kSPI_MISOPinNumber = -1;
pin_number kSPI_MOSIPinNumber = -1;

pin_number kSPI0_SCKPinNumber  = -1;
pin_number kSPI0_MISOPinNumber = -1;
pin_number kSPI0_MOSIPinNumber = -1;

pin_number kDebug1_PinNumber = 49;
pin_number kDebug2_PinNumber = 50;
pin_number kDebug3_PinNumber = 51;
pin_number kDebug4_PinNumber = 52;

pin_number kKinen_SyncPinNumber = -1;

pin_number kSocket1_SPISlaveSelectPinNumber = -1;
pin_number kSocket1_InterruptPinNumber      = -1;
pin_number kSocket1_StepPinNumber           = 0;
pin_number kSocket1_DirPinNumber            = 6;
pin_number kSocket1_EnablePinNumber         = -1;  // TODO Global Enable
pin_number kSocket1_Microstep_0PinNumber    = -1;
pin_number kSocket1_Microstep_1PinNumber    = -1;
pin_number kSocket1_Microstep_2PinNumber    = -1;
pin_number kSocket1_VrefPinNumber           = -1;

pin_number kSocket2_SPISlaveSelectPinNumber = -1;
pin_number kSocket2_InterruptPinNumber      = -1;
pin_number kSocket2_StepPinNumber           = 1;
pin_number kSocket2_DirPinNumber            = 7;
pin_number kSocket2_EnablePinNumber         = -1;  // TODO Global Enable
pin_number kSocket2_Microstep_0PinNumber    = -1;
pin_number kSocket2_Microstep_1PinNumber    = -1;
pin_number kSocket2_Microstep_2PinNumber    = -1;
pin_number kSocket2_VrefPinNumber           = -1;

pin_number kSocket3_SPISlaveSelectPinNumber = -1;
pin_number kSocket3_InterruptPinNumber      = -1;
pin_number kSocket3_StepPinNumber           = 2;
pin_number kSocket3_DirPinNumber            = 8;
pin_number kSocket3_EnablePinNumber         = -1;  // TODO Global Enable
pin_number kSocket3_Microstep_0PinNumber    = -1;
pin_number kSocket3_Microstep_1PinNumber    = -1;
pin_number kSocket3_Microstep_2PinNumber    = -1;
pin_number kSocket3_VrefPinNumber           = -1;

pin_number kSocket4_SPISlaveSelectPinNumber = -1;
pin_number kSocket4_InterruptPinNumber      = -1;
pin_number kSocket4_StepPinNumber           = 3;
pin_number kSocket4_DirPinNumber            = 9;
pin_number kSocket4_EnablePinNumber         = -1;
pin_number kSocket4_Microstep_0PinNumber    = -1;
pin_number kSocket4_Microstep_1PinNumber    = -1;
pin_number kSocket4_Microstep_2PinNumber    = -1;
pin_number kSocket4_VrefPinNumber           = -1;

pin_number kSocket5_SPISlaveSelectPinNumber = -1;
pin_number kSocket5_InterruptPinNumber      = -1;
pin_number kSocket5_StepPinNumber           = 4;
pin_number kSocket5_DirPinNumber            = 10;
pin_number kSocket5_EnablePinNumber         = -1;
pin_number kSocket5_Microstep_0PinNumber    = -1;
pin_number kSocket5_Microstep_1PinNumber    = -1;
pin_number kSocket5_Microstep_2PinNumber    = -1;
pin_number kSocket5_VrefPinNumber           = -1;

pin_number kSocket6_SPISlaveSelectPinNumber = -1;
pin_number kSocket6_InterruptPinNumber      = -1;
pin_number kSocket6_StepPinNumber           = 5;
pin_number kSocket6_DirPinNumber            = 11;
pin_number kSocket6_EnablePinNumber         = -1;
pin_number kSocket6_Microstep_0PinNumber    = -1;
pin_number kSocket6_Microstep_1PinNumber    = -1;
pin_number kSocket6_Microstep_2PinNumber    = -1;
pin_number kSocket6_VrefPinNumber           = -1;

pin_number kInput1_PinNumber = 100;  // X-Min
pin_number kInput2_PinNumber = 101;  // X-Max
pin_number kInput3_PinNumber = 102;  // Y-Min
pin_number kInput4_PinNumber = 103;  // Y-Max
pin_number kInput5_PinNumber = 104;  // Z-Min
pin_number kInput6_PinNumber = 105;  // Z-Max

pin_number kInput7_PinNumber  = 106;
pin_number kInput8_PinNumber  = 107;
pin_number kInput9_PinNumber  = 108;
pin_number kInput10_PinNumber = 109;
pin_number kInput11_PinNumber = 110;
pin_number kInput12_PinNumber = 111;
pin_number kInput13_PinNumber = 112;
pin_number kInput14_PinNumber = 113;

pin_number kSpindle_EnablePinNumber = 124;  // TODO enable spindle
pin_number kSpindle_DirPinNumber    = -1;

pin_number kCoolant_EnablePinNumber = -1;

pin_number kSpindle_PwmPinNumber  = -1;
pin_number kSpindle_Pwm2PinNumber = -1;

pin_number kSD_CardDetect = -1;
pin_number kInterlock_In  = -1;

pin_number kLED_USBRXPinNumber     = 12;
pin_number kLED_USBTXPinNumber     = 13;
pin_number kSD_CardDetectPinNumber = 119;
pin_number kSD_ChipSelectPinNumber = 120;
pin_number kInterlock_InPinNumber  = 121;
pin_number kOutputSAFE_PinNumber   = 122;  // SAFE signal
pin_number kLEDPWM_PinNumber       = 123;

// GRBL / gShield compatibility pins -- Due board ONLY

pin_number kGRBL_ResetPinNumber        = -1;
pin_number kGRBL_FeedHoldPinNumber     = -1;
pin_number kGRBL_CycleStartPinNumber   = -1;
pin_number kGRBL_CommonEnablePinNumber = -1;

// g2ref extensions
// These first 5 may replace the Spindle and Coolant pins, above
pin_number kOutput1_PinNumber = 124;  
pin_number kOutput2_PinNumber = 125;  
pin_number kOutput3_PinNumber = 126;  
pin_number kOutput4_PinNumber = 127;  
pin_number kOutput5_PinNumber = 128;  

pin_number kOutput6_PinNumber  = 129;  
pin_number kOutput7_PinNumber  = 130;  
pin_number kOutput8_PinNumber  = 131;  
pin_number kOutput9_PinNumber  = 132;  
pin_number kOutput10_PinNumber = 133;  

pin_number kOutput11_PinNumber = 134;  
pin_number kOutput12_PinNumber = 135;  
pin_number kOutput13_PinNumber = -1;  
pin_number kOutput14_PinNumber = -1;  
pin_number kOutput15_PinNumber = -1;  
pin_number kOutput16_PinNumber = -1;  

pin_number kADC0_PinNumber  = -1;  // Heated bed thermistor ADC
pin_number kADC1_PinNumber  = -1;  // Extruder1_ADC
pin_number kADC2_PinNumber  = -1;  // Extruder2_ADC
pin_number kADC3_PinNumber  = -1;  // 153;
pin_number kADC4_PinNumber  = -1;  // 154;
pin_number kADC5_PinNumber  = -1;  // 155;
pin_number kADC6_PinNumber  = -1;  // 156;
pin_number kADC7_PinNumber  = -1;  // 157;
pin_number kADC8_PinNumber  = -1;  // 158;
pin_number kADC9_PinNumber  = -1;  // 159;
pin_number kADC10_PinNumber = -1;  // 160;
pin_number kADC11_PinNumber = -1;  // 161;
pin_number kADC12_PinNumber = -1;  // 162;
pin_number kADC13_PinNumber = -1;  // Not physially pinned out
pin_number kADC14_PinNumber = -1;  // Not physially pinned out

// blank spots for unassigned pins - all unassigned pins need a unique number (do not re-use numbers)

pin_number kUnassigned90 = 165;
pin_number kUnassigned89 = 166;
pin_number kUnassigned88 = 167;
pin_number kUnassigned87 = 168;
pin_number kUnassigned86 = 169;
pin_number kUnassigned85 = 170;
pin_number kUnassigned84 = 171;
pin_number kUnassigned83 = 172;
pin_number kUnassigned82 = 173;
pin_number kUnassigned81 = 174;
pin_number kUnassigned80 = 175;
pin_number kUnassigned79 = 176;
pin_number kUnassigned78 = 177;
pin_number kUnassigned77 = 178;
pin_number kUnassigned76 = 179;
pin_number kUnassigned75 = 180;
pin_number kUnassigned74 = 181;
pin_number kUnassigned73 = 182;
pin_number kUnassigned72 = 183;
pin_number kUnassigned71 = 184;
pin_number kUnassigned70 = 185;
pin_number kUnassigned69 = 186;
pin_number kUnassigned68 = 187;
pin_number kUnassigned67 = 188;
pin_number kUnassigned66 = 189;
pin_number kUnassigned65 = 190;
pin_number kUnassigned64 = 191;
pin_number kUnassigned63 = 192;
pin_number kUnassigned62 = 193;
pin_number kUnassigned61 = 194;
pin_number kUnassigned60 = 195;
pin_number kUnassigned59 = 196;
pin_number kUnassigned58 = 197;
pin_number kUnassigned57 = 198;
pin_number kUnassigned56 = 199;
pin_number kUnassigned55 = 200;
pin_number kUnassigned54 = 201;
pin_number kUnassigned53 = 202;
pin_number kUnassigned52 = 203;
pin_number kUnassigned51 = 204;
pin_number kUnassigned50 = 205;
pin_number kUnassigned49 = 206;
pin_number kUnassigned48 = 207;
pin_number kUnassigned47 = 208;
pin_number kUnassigned46 = 209;
pin_number kUnassigned45 = 210;
pin_number kUnassigned44 = 211;
pin_number kUnassigned43 = 212;
pin_number kUnassigned42 = 213;
pin_number kUnassigned41 = 214;
pin_number kUnassigned40 = 215;
pin_number kUnassigned39 = 216;
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

// We're putting this in to make the autocomplete work for XCode,
// since it doesn't understand the special syntax coming up.
#ifdef XCODE_INDEX
#include <sbv300-pinout.h>
#endif

#ifdef MOTATE_BOARD
#define MOTATE_BOARD_PINOUT < MOTATE_BOARD-pinout.h >
#include MOTATE_BOARD_PINOUT
#else
#error Unknown board layout
#endif

#endif

// motate_pin_assignments_h
