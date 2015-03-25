/*
 * http://tinkerin.gs/
 *
 * Copyright (c) 2013 - 2014 Robert Giseburt
 * Copyright (c) 2013 - 2014 Alden S. Hart Jr.
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

// For the SAM3X8C boars, we actually use the same NUMBERING, but have different number to pin linkages

// We're putting this in to make the autocomplete work for XCode,
// since it doesn't understand the special syntax coming up.
#ifdef XCODE_INDEX
#include <G2v9d-pinout.h>
//#include <G2v9f-pinout.h>
#endif

#ifdef MOTATE_BOARD
#define MOTATE_BOARD_PINOUT < MOTATE_BOARD-pinout.h >
#include MOTATE_BOARD_PINOUT
#else
#error Unknown board layout $(MOTATE_BOARD)
#endif


namespace Motate {

	// NOT ALL OF THESE PINS ARE ON ALL PLATFORMS
	// Undefined pins will be equivalent to Motate::NullPin, and return 1 for Pin<>::isNull();

	pin_number kSerial_RX                       =  0;
	pin_number kSerial_TX                       =  1;

	pin_number kSerial0_RX                      =  0;
	pin_number kSerial0_TX                      =  1;

	pin_number kI2C_SDAPinNumber                =  2;
	pin_number kI2C_SCLPinNumber                =  3;

	pin_number kI2C0_SDAPinNumber               =  2;
	pin_number kI2C0_SCLPinNumber               =  3;

	pin_number kSPI_SCKPinNumber                =  4;
	pin_number kSPI_MISOPinNumber               =  5;
	pin_number kSPI_MOSIPinNumber               =  6;

	pin_number kSPI0_SCKPinNumber               =  4;
	pin_number kSPI0_MISOPinNumber              =  5;
	pin_number kSPI0_MOSIPinNumber              =  6;

	pin_number kKinen_SyncPinNumber             =  7;

	pin_number kSocket1_SPISlaveSelectPinNumber = 10;
	pin_number kSocket1_InterruptPinNumber      = 11;
	pin_number kSocket1_StepPinNumber           = 12;
	pin_number kSocket1_DirPinNumber            = 13;
	pin_number kSocket1_EnablePinNumber         = 14;
	pin_number kSocket1_Microstep_0PinNumber    = 15;
	pin_number kSocket1_Microstep_1PinNumber    = 16;
	pin_number kSocket1_Microstep_2PinNumber    = 17;
	pin_number kSocket1_VrefPinNumber           = 18;

	pin_number kSocket2_SPISlaveSelectPinNumber = 20;
	pin_number kSocket2_InterruptPinNumber      = 21;
	pin_number kSocket2_StepPinNumber           = 22;
	pin_number kSocket2_DirPinNumber            = 23;
	pin_number kSocket2_EnablePinNumber         = 24;
	pin_number kSocket2_Microstep_0PinNumber    = 25;
	pin_number kSocket2_Microstep_1PinNumber    = 26;
	pin_number kSocket2_Microstep_2PinNumber    = 27;
	pin_number kSocket2_VrefPinNumber           = 28;

	pin_number kSocket3_SPISlaveSelectPinNumber = 30;
	pin_number kSocket3_InterruptPinNumber      = 31;
	pin_number kSocket3_StepPinNumber           = 32;
	pin_number kSocket3_DirPinNumber            = 33;
	pin_number kSocket3_EnablePinNumber         = 34;
	pin_number kSocket3_Microstep_0PinNumber    = 35;
	pin_number kSocket3_Microstep_1PinNumber    = 36;
	pin_number kSocket3_Microstep_2PinNumber    = 37;
	pin_number kSocket3_VrefPinNumber           = 38;

	pin_number kSocket4_SPISlaveSelectPinNumber = 40;
	pin_number kSocket4_InterruptPinNumber      = 41;
	pin_number kSocket4_StepPinNumber           = 42;
	pin_number kSocket4_DirPinNumber            = 43;
	pin_number kSocket4_EnablePinNumber         = 44;
	pin_number kSocket4_Microstep_0PinNumber    = 45;
	pin_number kSocket4_Microstep_1PinNumber    = 46;
	pin_number kSocket4_Microstep_2PinNumber    = 47;
	pin_number kSocket4_VrefPinNumber           = 48;

	pin_number kSocket5_SPISlaveSelectPinNumber = 50;
	pin_number kSocket5_InterruptPinNumber      = 51;
	pin_number kSocket5_StepPinNumber           = 52;
	pin_number kSocket5_DirPinNumber            = 53;
	pin_number kSocket5_EnablePinNumber         = 54;
	pin_number kSocket5_Microstep_0PinNumber    = 55;
	pin_number kSocket5_Microstep_1PinNumber    = 56;
	pin_number kSocket5_Microstep_2PinNumber    = 57;
	pin_number kSocket5_VrefPinNumber           = 58;

	pin_number kSocket6_SPISlaveSelectPinNumber = 60;
	pin_number kSocket6_InterruptPinNumber      = 61;
	pin_number kSocket6_StepPinNumber           = 62;
	pin_number kSocket6_DirPinNumber            = 63;
	pin_number kSocket6_EnablePinNumber         = 64;
	pin_number kSocket6_Microstep_0PinNumber    = 65;
	pin_number kSocket6_Microstep_1PinNumber    = 66;
	pin_number kSocket6_Microstep_2PinNumber    = 67;
	pin_number kSocket6_VrefPinNumber           = 68;


	pin_number kInput1_PinNumber              = 100;
	pin_number kInput2_PinNumber              = 101;
	pin_number kInput3_PinNumber              = 102;
	pin_number kInput4_PinNumber              = 103;
	pin_number kInput5_PinNumber              = 104;
	pin_number kInput6_PinNumber              = 105;

	pin_number kInput7_PinNumber              = 106;
	pin_number kInput8_PinNumber              = 107;
	pin_number kInput9_PinNumber              = 108;
	pin_number kInput10_PinNumber              = 109;
	pin_number kInput11_PinNumber              = 110;
	pin_number kInput12_PinNumber              = 111;

	pin_number kSpindle_EnablePinNumber         = 112;
	pin_number kSpindle_DirPinNumber            = 113;
	pin_number kSpindle_PwmPinNumber            = 114;
	pin_number kSpindle_Pwm2PinNumber           = 115;
	pin_number kCoolant_EnablePinNumber         = 116;  // Convenient pin to hijack for debugging
//	pin_number kCoolant_EnablePinNumber         = -1;

	pin_number kLED_USBRXPinNumber              = 117;
	pin_number kLED_USBTXPinNumber              = 118;

    pin_number kSD_CardDetectPinNumber          = 119;
    pin_number kInterlock_InPinNumber           = 120;
//    pin_number kLEDPWM_PinNumber                = 121;
//    pin_number kNeopixel_DataPinNumber		= 121;

    pin_number kDebug1_PinNumber                =  -1;  // change to 116, for example
    pin_number kDebug2_PinNumber                =  -1;
    pin_number kDebug3_PinNumber                =  -1;
    pin_number kDebug4_PinNumber                =  -1;

	// GRBL / gShield compatibility pins -- Due board ONLY

	pin_number kGRBL_ResetPinNumber             =  -1;
	pin_number kGRBL_FeedHoldPinNumber          =  -1;
	pin_number kGRBL_CycleStartPinNumber        =  -1;

	pin_number kGRBL_CommonEnablePinNumber      =  -1;


	/** NOTE: When adding pin definitions here, they must be
     *        added to ALL board pin assignment files, even if
     *        they are defined as -1.
     **/

} // namespace Motate


#endif

// motate_pin_assignments_h
