/*
 * motate_pin_assignments.h - pin assignments
 * For: /board/ArduinoDue
 * This file is part of the g2core project
 *
 * Copyright (c) 2013-2016 Robert Giseburt
 * Copyright (c) 2013-2016 Alden S. Hart Jr.
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

// NOTE: This is a terrible example of a motate_pin_assignments.h file!
// This one is assigned backward in order to match the numbering on the Due.
// When making your own board, please use one of the other boards as an example
// of how to assign names to the pins.

namespace Motate {

// Arduino pin name & function
_MAKE_MOTATE_PIN(0, 'A', 8);  // D0, RX0

_MAKE_MOTATE_PIN(1, 'A', 9);  // D1, TX0

_MAKE_MOTATE_PIN(2, 'B', 25);  // D2, PWM2

_MAKE_MOTATE_PIN(3, 'C', 28);  // D3, PWM3

_MAKE_MOTATE_PIN(4, 'C', 26);  // D4, PWM4

_MAKE_MOTATE_PIN(5, 'C', 25);  // D5, PWM5

_MAKE_MOTATE_PIN(6, 'C', 24);  // D6, PWM6

_MAKE_MOTATE_PIN(7, 'C', 23);  // D7, PWM7

_MAKE_MOTATE_PIN(8, 'C', 22);  // D8, PWM8

_MAKE_MOTATE_PIN(9, 'C', 21);  // D9, PWM9

_MAKE_MOTATE_PIN(10, 'C', 29);  // D10, PWM10

_MAKE_MOTATE_PIN(11, 'D', 7);  // D11, PWM11

_MAKE_MOTATE_PIN(12, 'D', 8);  // D12, PWM12

_MAKE_MOTATE_PIN(13, 'B', 27);  // D13, PWM13, LED Indicator

_MAKE_MOTATE_PIN(14, 'D', 4);   // D14, TX3
_MAKE_MOTATE_PIN(15, 'D', 5);   // D15, RX3
_MAKE_MOTATE_PIN(16, 'A', 13);  // D16, TX2

_MAKE_MOTATE_PIN(17, 'A', 12);  // D17, RX2

_MAKE_MOTATE_PIN(18, 'A', 11);  // D18, TX1
_MAKE_MOTATE_PIN(19, 'A', 10);  // D19, RX1
_MAKE_MOTATE_PIN(20, 'B', 12);  // D20, SDA

_MAKE_MOTATE_PIN(21, 'B', 13);  // D21, SCL


_MAKE_MOTATE_PIN(22, 'B', 26);  // D22	- begins dual inline header
_MAKE_MOTATE_PIN(23, 'A', 14);  // D23
_MAKE_MOTATE_PIN(24, 'A', 15);  // D24
_MAKE_MOTATE_PIN(25, 'D', 0);   // D25
_MAKE_MOTATE_PIN(26, 'D', 1);   // D26
_MAKE_MOTATE_PIN(27, 'D', 2);   // D27
_MAKE_MOTATE_PIN(28, 'D', 3);   // D28
_MAKE_MOTATE_PIN(29, 'D', 6);   // D29
_MAKE_MOTATE_PIN(30, 'D', 9);   // D30
_MAKE_MOTATE_PIN(31, 'A', 7);   // D31
_MAKE_MOTATE_PIN(32, 'D', 10);  // D32
_MAKE_MOTATE_PIN(33, 'C', 1);   // D33
_MAKE_MOTATE_PIN(34, 'C', 2);   // D34

_MAKE_MOTATE_PIN(35, 'C', 3);  // D35

_MAKE_MOTATE_PIN(36, 'C', 4);  // D36

_MAKE_MOTATE_PIN(37, 'C', 5);  // D37

_MAKE_MOTATE_PIN(38, 'C', 6);  // D38

_MAKE_MOTATE_PIN(39, 'C', 7);  // D39

_MAKE_MOTATE_PIN(40, 'C', 8);  // D40

_MAKE_MOTATE_PIN(41, 'C', 9);  // D41

_MAKE_MOTATE_PIN(42, 'A', 19);  // D42

_MAKE_MOTATE_PIN(43, 'A', 20);  // D43

_MAKE_MOTATE_PIN(44, 'C', 19);  // D44

_MAKE_MOTATE_PIN(45, 'C', 18);  // D45

_MAKE_MOTATE_PIN(46, 'C', 17);  // D46
_MAKE_MOTATE_PIN(47, 'C', 16);  // D47
_MAKE_MOTATE_PIN(48, 'C', 15);  // D48
_MAKE_MOTATE_PIN(49, 'C', 14);  // D49
_MAKE_MOTATE_PIN(50, 'C', 13);  // D50
_MAKE_MOTATE_PIN(51, 'C', 12);  // D51
_MAKE_MOTATE_PIN(52, 'B', 21);  // D52

_MAKE_MOTATE_PIN(53, 'B', 14);  // D53


_MAKE_MOTATE_PIN(54, 'A', 16);  // A0 - begins analog headers
_MAKE_MOTATE_PIN(55, 'A', 24);  // A1
_MAKE_MOTATE_PIN(56, 'A', 23);  // A2
_MAKE_MOTATE_PIN(57, 'A', 22);  // A3
_MAKE_MOTATE_PIN(58, 'A', 6);   // A4

_MAKE_MOTATE_PIN(59, 'A', 4);  // A5
_MAKE_MOTATE_PIN(60, 'A', 3);  // A6

_MAKE_MOTATE_PIN(61, 'A', 2);  // A7

_MAKE_MOTATE_PIN(62, 'B', 17);  // A8

_MAKE_MOTATE_PIN(63, 'B', 18);  // A9

_MAKE_MOTATE_PIN(64, 'B', 19);  // A10

_MAKE_MOTATE_PIN(65, 'B', 20);  // A11

_MAKE_MOTATE_PIN(66, 'B', 15);  // DAC0

_MAKE_MOTATE_PIN(67, 'B', 16);  // DAC1

_MAKE_MOTATE_PIN(68, 'A', 1);  // CANRX
_MAKE_MOTATE_PIN(69, 'A', 0);  // CANTX


_MAKE_MOTATE_PIN(70, 'A', 17);  // SDA1
_MAKE_MOTATE_PIN(71, 'A', 18);  // SCL1
_MAKE_MOTATE_PIN(72, 'C', 30);  // RXL, LED Indicator
_MAKE_MOTATE_PIN(73, 'A', 21);  // TXL, LED Indicator

_MAKE_MOTATE_PIN(74, 'A', 25);  // SPI-MISO (on ICSP header only)

_MAKE_MOTATE_PIN(75, 'A', 26);  // SPI-MOSI (on ICSP header only)

_MAKE_MOTATE_PIN(76, 'A', 27);  // SPI-SCK  (on ICSP header only)

_MAKE_MOTATE_PIN(77, 'A', 28);  // SS0

_MAKE_MOTATE_PIN(78, 'B', 23);  // SS1

}  // namespace Motate

// We then allow each chip-type to have it's own function definitions
// that will refer to these pin assignments.
#include "motate_chip_pin_functions.h"


// We're putting this in to make the autocomplete work for XCode,
// since it doesn't understand the special syntax coming up.
#ifdef XCODE_INDEX
#include <Due-pinout.h>
#endif

#ifdef MOTATE_BOARD
#define MOTATE_BOARD_PINOUT < MOTATE_BOARD-pinout.h >
#include MOTATE_BOARD_PINOUT
#else
#error Unknown board layout
#endif

#endif

// motate_pin_assignments_h
