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

	// DUE
	_MAKE_MOTATE_PORT32(A, 'A');
	_MAKE_MOTATE_PORT32(B, 'B');
	_MAKE_MOTATE_PORT32(C, 'C');
	_MAKE_MOTATE_PORT32(D, 'D');
	
	// Arduino pin name & function
	_MAKE_MOTATE_PIN( 0, A, 'A',  8);	// D0, RX0
		_MAKE_MOTATE_PWM_PIN(0, Motate::PWMTimer<0>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!

	_MAKE_MOTATE_PIN( 1, A, 'A',  9);	// D1, TX0
		_MAKE_MOTATE_PWM_PIN(1, Motate::PWMTimer<3>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!

	_MAKE_MOTATE_PIN( 2, B, 'B', 25);	// D2, PWM2
		_MAKE_MOTATE_PWM_PIN(2, Motate::Timer<0>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN( 3, C, 'C', 28);	// D3, PWM3
		_MAKE_MOTATE_PWM_PIN(3, Motate::Timer<7>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN( 4, C, 'C', 26);	// D4, PWM4
		_MAKE_MOTATE_PWM_PIN(4, Motate::Timer<6>, /*Channel:*/ B, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN( 5, C, 'C', 25);	// D5, PWM5
		_MAKE_MOTATE_PWM_PIN(5, Motate::Timer<6>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN( 6, C, 'C', 24);	// D6, PWM6
		_MAKE_MOTATE_PWM_PIN(6, Motate::PWMTimer<7>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN( 7, C, 'C', 23);	// D7, PWM7
		_MAKE_MOTATE_PWM_PIN(7, Motate::PWMTimer<6>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN( 8, C, 'C', 22);	// D8, PWM8
		_MAKE_MOTATE_PWM_PIN(8, Motate::PWMTimer<5>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN( 9, C, 'C', 21);	// D9, PWM9
		_MAKE_MOTATE_PWM_PIN(9, Motate::PWMTimer<4>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(10, C, 'C', 29);	// D10, PWM10
		_MAKE_MOTATE_PWM_PIN(10, Motate::Timer<7>, /*Channel:*/ B, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(11, D, 'D',  7);	// D11, PWM11
		_MAKE_MOTATE_PWM_PIN(11, Motate::Timer<8>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(12, D, 'D',  8);	// D12, PWM12
		_MAKE_MOTATE_PWM_PIN(12, Motate::Timer<8>, /*Channel:*/ B, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(13, B, 'B', 27);	// D13, PWM13, LED Indicator
		_MAKE_MOTATE_PWM_PIN(13, Motate::Timer<0>, /*Channel:*/ B, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(14, D, 'D',  4);	// D14, TX3
	_MAKE_MOTATE_PIN(15, D, 'D',  5);	// D15, RX3
	_MAKE_MOTATE_PIN(16, A, 'A', 13);	// D16, TX2
		_MAKE_MOTATE_PWM_PIN(16, Motate::PWMTimer<2>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!

	_MAKE_MOTATE_PIN(17, A, 'A', 12);	// D17, RX2
		_MAKE_MOTATE_PWM_PIN(17, Motate::PWMTimer<1>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(18, A, 'A', 11);	// D18, TX1
	_MAKE_MOTATE_PIN(19, A, 'A', 10);	// D19, RX1
	_MAKE_MOTATE_PIN(20, B, 'B', 12);	// D20, SDA
		_MAKE_MOTATE_PWM_PIN(20, Motate::PWMTimer<0>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!

	_MAKE_MOTATE_PIN(21, B, 'B', 13);	// D21, SCL
		_MAKE_MOTATE_PWM_PIN(21, Motate::PWMTimer<1>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!


	_MAKE_MOTATE_PIN(22, B, 'B', 26);	// D22	- begins dual inline header
	_MAKE_MOTATE_PIN(23, A, 'A', 14);	// D23
	_MAKE_MOTATE_PIN(24, A, 'A', 15);	// D24
	_MAKE_MOTATE_PIN(25, D, 'D',  0);	// D25
	_MAKE_MOTATE_PIN(26, D, 'D',  1);	// D26
	_MAKE_MOTATE_PIN(27, D, 'D',  2);	// D27
	_MAKE_MOTATE_PIN(28, D, 'D',  3);	// D28
	_MAKE_MOTATE_PIN(29, D, 'D',  6);	// D29
	_MAKE_MOTATE_PIN(30, D, 'D',  9);	// D30
	_MAKE_MOTATE_PIN(31, A, 'A',  7);	// D31
	_MAKE_MOTATE_PIN(32, D, 'D', 10);	// D32
	_MAKE_MOTATE_PIN(33, C, 'C',  1);	// D33
	_MAKE_MOTATE_PIN(34, C, 'C',  2);	// D34
		_MAKE_MOTATE_PWM_PIN(34, Motate::PWMTimer<0>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(35, C, 'C',  3);	// D35
		_MAKE_MOTATE_PWM_PIN(35, Motate::PWMTimer<0>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!

	_MAKE_MOTATE_PIN(36, C, 'C',  4);	// D36
		_MAKE_MOTATE_PWM_PIN(36, Motate::PWMTimer<1>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(37, C, 'C',  5);	// D37
		_MAKE_MOTATE_PWM_PIN(37, Motate::PWMTimer<1>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!

	_MAKE_MOTATE_PIN(38, C, 'C',  6);	// D38
		_MAKE_MOTATE_PWM_PIN(38, Motate::PWMTimer<2>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(39, C, 'C',  7);	// D39
		_MAKE_MOTATE_PWM_PIN(39, Motate::PWMTimer<2>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!

	_MAKE_MOTATE_PIN(40, C, 'C',  8);	// D40
		_MAKE_MOTATE_PWM_PIN(40, Motate::PWMTimer<3>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(41, C, 'C',  9);	// D41
		_MAKE_MOTATE_PWM_PIN(41, Motate::PWMTimer<3>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!

	_MAKE_MOTATE_PIN(42, A, 'A', 19);	// D42
		_MAKE_MOTATE_PWM_PIN(42, Motate::PWMTimer<1>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!

	_MAKE_MOTATE_PIN(43, A, 'A', 20);	// D43
		_MAKE_MOTATE_PWM_PIN(43, Motate::PWMTimer<2>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(44, C, 'C', 19);	// D44
		_MAKE_MOTATE_PWM_PIN(44, Motate::PWMTimer<5>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!

	_MAKE_MOTATE_PIN(45, C, 'C', 18);	// D45
		_MAKE_MOTATE_PWM_PIN(45, Motate::PWMTimer<6>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!

	_MAKE_MOTATE_PIN(46, C, 'C', 17);	// D46
	_MAKE_MOTATE_PIN(47, C, 'C', 16);	// D47
	_MAKE_MOTATE_PIN(48, C, 'C', 15);	// D48
	_MAKE_MOTATE_PIN(49, C, 'C', 14);	// D49
	_MAKE_MOTATE_PIN(50, C, 'C', 13);	// D50
	_MAKE_MOTATE_PIN(51, C, 'C', 12);	// D51
	_MAKE_MOTATE_PIN(52, B, 'B', 21);	// D52
        _MAKE_MOTATE_SPI_CS_PIN(52, B, 2);

	_MAKE_MOTATE_PIN(53, B, 'B', 14);	// D53
		_MAKE_MOTATE_PWM_PIN(53, Motate::PWMTimer<2>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!


	_MAKE_MOTATE_PIN(54, A, 'A', 16);	// A0 - begins analog headers
	_MAKE_MOTATE_PIN(55, A, 'A', 24);	// A1
	_MAKE_MOTATE_PIN(56, A, 'A', 23);	// A2
	_MAKE_MOTATE_PIN(57, A, 'A', 22);	// A3
	_MAKE_MOTATE_PIN(58, A, 'A',  6);	// A4
		_MAKE_MOTATE_PWM_PIN(58, Motate::Timer<2>, /*Channel:*/ B, /*Peripheral:*/ A, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(59, A, 'A',  4);	// A5
	_MAKE_MOTATE_PIN(60, A, 'A',  3);	// A6
		_MAKE_MOTATE_PWM_PIN(60, Motate::Timer<1>, /*Channel:*/ B, /*Peripheral:*/ A, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(61, A, 'A',  2);	// A7
		_MAKE_MOTATE_PWM_PIN(61, Motate::Timer<1>, /*Channel:*/ A, /*Peripheral:*/ A, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(62, B, 'B', 17);	// A8
		_MAKE_MOTATE_PWM_PIN(62, Motate::PWMTimer<1>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(63, B, 'B', 18);	// A9
		_MAKE_MOTATE_PWM_PIN(63, Motate::PWMTimer<2>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(64, B, 'B', 19);	// A10
		_MAKE_MOTATE_PWM_PIN(64, Motate::PWMTimer<3>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(65, B, 'B', 20);	// A11
        _MAKE_MOTATE_SPI_CS_PIN(65, B, 1);

	_MAKE_MOTATE_PIN(66, B, 'B', 15);	// DAC0
		_MAKE_MOTATE_PWM_PIN(66, Motate::PWMTimer<3>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ true); // INVERTED!

	_MAKE_MOTATE_PIN(67, B, 'B', 16);	// DAC1
		_MAKE_MOTATE_PWM_PIN(67, Motate::PWMTimer<0>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(68, A, 'A',  1);	// CANRX
	_MAKE_MOTATE_PIN(69, A, 'A',  0);	// CANTX
		_MAKE_MOTATE_PWM_PIN(69, Motate::PWMTimer<3>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);


	_MAKE_MOTATE_PIN(70, A, 'A', 17);	// SDA1
	_MAKE_MOTATE_PIN(71, A, 'A', 18);	// SCL1
	_MAKE_MOTATE_PIN(72, C, 'C', 30);	// RXL, LED Indicator
	_MAKE_MOTATE_PIN(73, A, 'A', 21);	// TXL, LED Indicator
		_MAKE_MOTATE_PWM_PIN(73, Motate::PWMTimer<0>, /*Channel:*/ A, /*Peripheral:*/ B, /*Inverted:*/ false);

	_MAKE_MOTATE_PIN(74, A, 'A', 25);	// SPI-MISO (on ICSP header only)
    _MAKE_MOTATE_SPI_OTHER_PIN(74, A);
    
    _MAKE_MOTATE_PIN(75, A, 'A', 26);	// SPI-MOSI (on ICSP header only)
	_MAKE_MOTATE_SPI_OTHER_PIN(75, A);
    
    _MAKE_MOTATE_PIN(76, A, 'A', 27);	// SPI-SCK  (on ICSP header only)
    _MAKE_MOTATE_SPI_OTHER_PIN(76, A);
    
	_MAKE_MOTATE_PIN(77, A, 'A', 28);	// SS0
        _MAKE_MOTATE_SPI_CS_PIN(77, A, 0);

	_MAKE_MOTATE_PIN(78, B, 'B', 23);	// SS1
        _MAKE_MOTATE_SPI_CS_PIN(78, B, 3);

    
} // namespace Motate


// We're putting this in to make the autocomplete work for XCode,
// since it doesn't understand the special syntax coming up.
#ifdef XCODE_INDEX
#include <gShield-pinout.h>
#endif

#ifdef MOTATE_BOARD
#define MOTATE_BOARD_PINOUT < MOTATE_BOARD-pinout.h >
#include MOTATE_BOARD_PINOUT
#else
#error Unknown board layout
#endif

#endif

// motate_pin_assignments_h
