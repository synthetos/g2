/*
 * http://tinkerin.gs/
 *
 * Copyright (c) 2013 Robert Giseburt
 * Copyright (c) 2013 Alden S. Hart Jr.
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

	// v9_3x8c - The SAM3X8C is a 100-pin sister to the SAM3X8E that is on the Due
	// The SAM3X8C is missing port C and D.

	_MAKE_MOTATE_PORT32(A, 'A');
	_MAKE_MOTATE_PORT32(B, 'B');

	/*** Pins that are signals and are *not* fin-specific:

	 * 0 - Serial RX0 (not on a fin)
	 * 1 - Serial TX0 (not on a fin)

	 * 2 - I2C SDA
	 * 3 - I2C SCL
	 
	 * 4 - SPI SCK
	 * 5 - SPI MISO
	 * 6 - SPI MOSI

	 * 7 - ~Sync
	 
	 * (8-9 reserved)

	 ***/


	/*** Pins that *are* a kinen fin:
	 
	 * (Number ralative to 10*x)

	 * (smart header)
	 * +0 - Sx_SS
	 * +1 - Sx_Interrupt
	 
	 * (dumb header)
	 * +2 - Sx_Step        Sx_D0
	 * +3 - Sx_Direction   Sx_D1
	 * +4 - Sx_Enable      Sx_D2
	 * +5 - Sx_MS0         Sx_D3
	 * +6 - Sx_MS1         Sx_D4
	 * +7 - Sx_VREF        Sx_A0
	 
	 * (8-9 reserved)

	 ***/


	 /*** Pins 100+ are board specific functions.
	  * Second (+) SPI or I2C would be here.
	  * Special non-kinen devices, LEDs, etc.
	  **/


	// First we define Motate::Pin<> object templates,
	// then we define the pin_number aliases.


	// All-Fin pins

	// Pin 0 - Serial RX - missing!
	// Pin 1 - Serial TX - missing!

	_MAKE_MOTATE_PIN( 2, B, 'B', 12);	// TWD1
	_MAKE_MOTATE_PWM_PIN( 2, Motate::PWMTimer<0>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!

	_MAKE_MOTATE_PIN( 3, B, 'B', 13);	// TWCK1
	_MAKE_MOTATE_PWM_PIN( 3, Motate::PWMTimer<1>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!

	_MAKE_MOTATE_PIN( 4, A, 'A', 27);	// SPI-SCK
	_MAKE_MOTATE_PIN( 5, A, 'A', 25);	// SPI-MISO
	_MAKE_MOTATE_PIN( 6, A, 'A', 26);	// SPI-MOSI

	_MAKE_MOTATE_PIN( 7, B, 'B', 15);	// ~Sync, DAC0
	_MAKE_MOTATE_PWM_PIN( 7, Motate::PWMTimer<3>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!


	// Per-Fin pins

	// Socket 1

	// Pin 10 - ~SS -- not on this board

	_MAKE_MOTATE_PIN(11, B, 'B', 16);	// S1_Interrupt - DAC1
	_MAKE_MOTATE_PWM_PIN(11, Motate::PWMTimer<0>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(12, A, 'A', 12);	// S1_Step, S1_D0, RX2
	_MAKE_MOTATE_PWM_PIN(12, Motate::PWMTimer<1>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(13, A, 'A', 10);	// S1_Direction, S1_D1, RX1

	_MAKE_MOTATE_PIN(14, A, 'A', 11);	// S1_Enable, S1_D2, TX1

	_MAKE_MOTATE_PIN(15, B, 'B', 26);	// S1_MS0, S1_D3

	_MAKE_MOTATE_PIN(16, A, 'A',  9);	// S1_MS1, S1_D4, TX0
	_MAKE_MOTATE_PWM_PIN(16, Motate::PWMTimer<3>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!

	_MAKE_MOTATE_PIN(17, A, 'A',  8);	// S1_VREF, S1_A0, RX0
	_MAKE_MOTATE_PWM_PIN(17, Motate::PWMTimer<0>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!


	// Socket 2

	_MAKE_MOTATE_PIN(20, A, 'A', 21);	// S2_SS, SPI0_CS3
	_MAKE_MOTATE_PWM_PIN(20, Motate::PWMTimer<0>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(21, B, 'B', 11);	// S2_Interrupt, UOTGID

	_MAKE_MOTATE_PIN(22, B, 'B', 25);	// S2_Step, S2_D0, PWM
	_MAKE_MOTATE_PWM_PIN(22, Motate::Timer<0>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(23, B, 'B', 22);	// S2_Direction, S2_D1

	_MAKE_MOTATE_PIN(24, B, 'B', 24);	// S2_Enable, S2_D2

//	_MAKE_MOTATE_PIN(25, C, 'C',  0);	// S2_MS0, S2_D3, ERASE -- not connected

	_MAKE_MOTATE_PIN(26, B, 'B', 14);	// S2_MS1, S2_D4
	_MAKE_MOTATE_PWM_PIN(26, Motate::PWMTimer<2>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!

	_MAKE_MOTATE_PIN(27, B, 'B', 17);	// S2_VREF, S2_A0
	_MAKE_MOTATE_PWM_PIN(27, Motate::PWMTimer<1>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);


	// Socket 3

	_MAKE_MOTATE_PIN(30, B, 'B', 23);	// S3_SS, SPI0_CS3

	_MAKE_MOTATE_PIN(31, B, 'B',  5);	// S3_Interrupt

	_MAKE_MOTATE_PIN(32, B, 'B', 10);	// S3_Step, S3_D0, UOTGVBOF

	_MAKE_MOTATE_PIN(33, B, 'B',  8);	// S3_Direction, S3_D1

	_MAKE_MOTATE_PIN(34, B, 'B',  9);	// S3_Enable, S3_D2

	_MAKE_MOTATE_PIN(35, B, 'B',  6);	// S3_MS0, S3_D3

	_MAKE_MOTATE_PIN(36, B, 'B',  7);	// S3_MS1, S3_D4

	_MAKE_MOTATE_PIN(37, B, 'B', 18);	// S3_VREF, S3_A0
	_MAKE_MOTATE_PWM_PIN(37, Motate::PWMTimer<2>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);


	// Socket 4

	_MAKE_MOTATE_PIN(40, A, 'A', 28);	// S4_SS, SPI0_CS0

	_MAKE_MOTATE_PIN(41, A, 'A',  2);	// S4_Interrupt
	_MAKE_MOTATE_PWM_PIN(41, Motate::Timer<1>, /*Channel:*/ A, /*Peripheral:*/ A, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(42, B, 'B',  4);	// S4_Step, S4_D0

	_MAKE_MOTATE_PIN(43, B, 'B',  3);	// S4_Direction, S4_D1

	_MAKE_MOTATE_PIN(44, B, 'B',  2);	// S4_Enable, S4_D2

	_MAKE_MOTATE_PIN(45, B, 'B',  1);	// S4_MS0, S4_D3

	_MAKE_MOTATE_PIN(46, B, 'B', 20);	// S4_MS1, S4_D4

	_MAKE_MOTATE_PIN(47, B, 'B', 19);	// S4_VREF, S4_A0
	_MAKE_MOTATE_PWM_PIN(47, Motate::PWMTimer<3>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);


	// Socket 5

	_MAKE_MOTATE_PIN(50, A, 'A', 29);	// S5_SS, SPI0_CS1

	_MAKE_MOTATE_PIN(51, A, 'A', 24);	// S5_Interrupt

	_MAKE_MOTATE_PIN(52, A, 'A', 23);	// S5_Step, S5_D0

	_MAKE_MOTATE_PIN(53, A, 'A', 22);	// S5_Direction, S5_D1

	_MAKE_MOTATE_PIN(54, A, 'A',  6);	// S5_Enable, S5_D2
	_MAKE_MOTATE_PWM_PIN(54, Motate::Timer<2>, /*Channel:*/ B, /*Peripheral:*/ A, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(55, A, 'A',  4);	// S5_MS0, S5_D3

	_MAKE_MOTATE_PIN(56, A, 'A',  3);	// S5_MS1, S5_D4
	_MAKE_MOTATE_PWM_PIN(56, Motate::Timer<1>, /*Channel:*/ B, /*Peripheral:*/ A, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(57, B, 'B',  0);	// S5_VREF, S5_A0
										// NO PWM!!


	// Socket 6 (smart half ONLY)

	_MAKE_MOTATE_PIN(60, B, 'B', 21);	// S6_SS, SPI0_CS2

	_MAKE_MOTATE_PIN(61, A, 'A', 16);	// S6_Interrupt


	// Limit switches
	_MAKE_MOTATE_PIN(100, A, 'A', 13);	// X_MIN_LIMIT
	_MAKE_MOTATE_PWM_PIN(100, Motate::PWMTimer<2>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!

	_MAKE_MOTATE_PIN(101, A, 'A', 14);	// X_MAX_LIMIT

	_MAKE_MOTATE_PIN(102, A, 'A', 15);	// Y_MIN_LIMIT

	_MAKE_MOTATE_PIN(103, A, 'A', 17);	// Y_MAX_LIMIT, TWD0

	_MAKE_MOTATE_PIN(104, A, 'A',  0);	// Z_MIN_LIMIT, CANTX
	_MAKE_MOTATE_PWM_PIN(104, Motate::PWMTimer<3>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(105, A, 'A',  1);	// Z_MAX_LIMIT, CANRX

	// 106-111 are A-C Min/Max

	_MAKE_MOTATE_PIN(112, A, 'A', 18);	// SPINDLE_ON, TWCK0

	_MAKE_MOTATE_PIN(113, A, 'A', 19);	// SPINDLE_DIR
	_MAKE_MOTATE_PWM_PIN(113, Motate::PWMTimer<1>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!

	_MAKE_MOTATE_PIN(114, B, 'B', 27);	// SPINDLE_PWM
	_MAKE_MOTATE_PWM_PIN(114, Motate::Timer<0>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(116, A, 'A', 20);	// Coolant_ON
	_MAKE_MOTATE_PWM_PIN(115, Motate::PWMTimer<2>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(117, A, 'A',  5);	// USB_RX_LED

	_MAKE_MOTATE_PIN(118, A, 'A',  7);	// USB_TX_LED




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
	pin_number kSocket1_VrefPinNumber           = 17;

	pin_number kSocket2_SPISlaveSelectPinNumber = 20;
	pin_number kSocket2_InterruptPinNumber      = 21;
	pin_number kSocket2_StepPinNumber           = 22;
	pin_number kSocket2_DirPinNumber            = 23;
	pin_number kSocket2_EnablePinNumber         = 24;
	pin_number kSocket2_Microstep_0PinNumber    = 25;
	pin_number kSocket2_Microstep_1PinNumber    = 26;
	pin_number kSocket2_VrefPinNumber           = 27;

	pin_number kSocket3_SPISlaveSelectPinNumber = 30;
	pin_number kSocket3_InterruptPinNumber      = 31;
	pin_number kSocket3_StepPinNumber           = 32;
	pin_number kSocket3_DirPinNumber            = 33;
	pin_number kSocket3_EnablePinNumber         = 34;
	pin_number kSocket3_Microstep_0PinNumber    = 35;
	pin_number kSocket3_Microstep_1PinNumber    = 36;
	pin_number kSocket3_VrefPinNumber           = 37;

	pin_number kSocket4_SPISlaveSelectPinNumber = 40;
	pin_number kSocket4_InterruptPinNumber      = 41;
	pin_number kSocket4_StepPinNumber           = 42;
	pin_number kSocket4_DirPinNumber            = 43;
	pin_number kSocket4_EnablePinNumber         = 44;
	pin_number kSocket4_Microstep_0PinNumber    = 45;
	pin_number kSocket4_Microstep_1PinNumber    = 46;
	pin_number kSocket4_VrefPinNumber           = 47;

	pin_number kSocket5_SPISlaveSelectPinNumber = 50;
	pin_number kSocket5_InterruptPinNumber      = 51;
	pin_number kSocket5_StepPinNumber           = 52;
	pin_number kSocket5_DirPinNumber            = 53;
	pin_number kSocket5_EnablePinNumber         = 54;
	pin_number kSocket5_Microstep_0PinNumber    = 55;
	pin_number kSocket5_Microstep_1PinNumber    = 56;
	pin_number kSocket5_VrefPinNumber           = 57;

	pin_number kSocket6_SPISlaveSelectPinNumber = 60;
	pin_number kSocket6_InterruptPinNumber      = 61;
	pin_number kSocket6_StepPinNumber           = 62;
	pin_number kSocket6_DirPinNumber            = 63;
	pin_number kSocket6_EnablePinNumber         = 64;
	pin_number kSocket6_Microstep_0PinNumber    = 65;
	pin_number kSocket6_Microstep_1PinNumber    = 66;
	pin_number kSocket6_VrefPinNumber           = 67;


	pin_number kXAxis_MinPinNumber              = 100;
	pin_number kXAxis_MaxPinNumber              = 101;
	pin_number kYAxis_MinPinNumber              = 102;
	pin_number kYAxis_MaxPinNumber              = 103;
	pin_number kZAxis_MinPinNumber              = 104;
	pin_number kZAxis_MaxPinNumber              = 105;

	pin_number kAAxis_MinPinNumber              = 106;
	pin_number kAAxis_MaxPinNumber              = 107;
	pin_number kBAxis_MinPinNumber              = 108;
	pin_number kBAxis_MaxPinNumber              = 109;
	pin_number kCAxis_MinPinNumber              = 110;
	pin_number kCAxis_MaxPinNumber              = 111;

	pin_number kSpindle_EnablePinNumber         = 112;
	pin_number kSpindle_DirPinNumber            = 113;
	pin_number kSpindle_PwmPinNumber            = 114;
	pin_number kSpindle_Pwm2PinNumber           = 115;
	pin_number kCoolant_EnablePinNumber         = 116;

	pin_number kLED_USBRXPinNumber              = 117;
	pin_number kLED_USBTXPinNumber              = 118;


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
