/*
 * sbv300-pinout.h - board pinout specification
 * For: /board/sbv300
 * This file is part of the g2core project
 *
 * Copyright (c) 2013 - 2018 Robert Giseburt
 * Copyright (c) 2013 - 2018 Alden S. Hart Jr.
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

#ifndef sbv300_pinout_h
#define sbv300_pinout_h

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
#define INPUT9_AVAILABLE 1
#define INPUT10_AVAILABLE 1
#define INPUT11_AVAILABLE 1
#define INPUT12_AVAILABLE 1
#define INPUT13_AVAILABLE 1

#define ADC0_AVAILABLE 0
#define ADC1_AVAILABLE 0
#define ADC2_AVAILABLE 0
#define ADC3_AVAILABLE 0

#define XIO_HAS_USB 1
#define XIO_HAS_UART 0
#define XIO_HAS_SPI 0
#define XIO_HAS_I2C 0

#define TEMPERATURE_OUTPUT_ON 0

// Some pins, if the PWM capability is turned on, it will cause timer conflicts.
// So we have to explicity enable them as PWM pins.
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
#define OUTPUT11_PWM 1
#define OUTPUT12_PWM 1
#define OUTPUT13_PWM 0

namespace Motate {

    _MAKE_MOTATE_PIN(kOutput1_PinNumber         , 'A',  0);
    _MAKE_MOTATE_PIN(kUnassigned2               , 'A',  2);
    _MAKE_MOTATE_PIN(kUnassigned3               , 'A',  3);
    _MAKE_MOTATE_PIN(kUnassigned4               , 'A',  4);
    _MAKE_MOTATE_PIN(kUnassigned80        		, 'A',  5);
    _MAKE_MOTATE_PIN(kUnassigned6               , 'A',  6);
    _MAKE_MOTATE_PIN(kOutput10_PinNumber        , 'A',  7);
    _MAKE_MOTATE_PIN(kUnassigned81        		, 'A',  8);
    _MAKE_MOTATE_PIN(kInput10_PinNumber         , 'A',  9);
    _MAKE_MOTATE_PIN(kUnassigned10              , 'A', 10);
    _MAKE_MOTATE_PIN(kUnassigned11              , 'A', 11);

    _MAKE_MOTATE_PIN(kOutput11_PinNumber        , 'A',  12);
    _MAKE_MOTATE_PIN(kOutput12_PinNumber        , 'A',  13);

    _MAKE_MOTATE_PIN(kInput14_PinNumber         , 'A', 14);
    _MAKE_MOTATE_PIN(kInput13_PinNumber         , 'A', 15);
    _MAKE_MOTATE_PIN(kUnassigned16              , 'A', 16);
    _MAKE_MOTATE_PIN(kInput12_PinNumber         , 'A', 17);
    _MAKE_MOTATE_PIN(kUnassigned18              , 'A', 18);
    _MAKE_MOTATE_PIN(kUnassigned19              , 'A', 19);
    _MAKE_MOTATE_PIN(kUnassigned20              , 'A', 20);
    _MAKE_MOTATE_PIN(kUnassigned21              , 'A', 21);
    _MAKE_MOTATE_PIN(kUnassigned22              , 'A', 22);
    _MAKE_MOTATE_PIN(kUnassigned23              , 'A', 23);
    _MAKE_MOTATE_PIN(kUnassigned24              , 'A', 24);
    _MAKE_MOTATE_PIN(kUnassigned25              , 'A', 25);
    _MAKE_MOTATE_PIN(kUnassigned26              , 'A', 26);
    _MAKE_MOTATE_PIN(kUnassigned27              , 'A', 27);
    _MAKE_MOTATE_PIN(kUnassigned28              , 'A', 28);
    _MAKE_MOTATE_PIN(kUnassigned29              , 'A', 29);

    _MAKE_MOTATE_PIN(kSocket6_DirPinNumber      , 'B',  0);
    _MAKE_MOTATE_PIN(kSocket6_StepPinNumber     , 'B',  1);
    _MAKE_MOTATE_PIN(kUnassigned30              , 'B',  2);
    _MAKE_MOTATE_PIN(kInput4_PinNumber          , 'B',  3);
    _MAKE_MOTATE_PIN(kInput1_PinNumber          , 'B',  4);
    _MAKE_MOTATE_PIN(kInput2_PinNumber          , 'B',  5);
    _MAKE_MOTATE_PIN(kInput7_PinNumber          , 'B',  6);
    _MAKE_MOTATE_PIN(kInput8_PinNumber          , 'B',  7);
    _MAKE_MOTATE_PIN(kInput5_PinNumber          , 'B',  8);
    _MAKE_MOTATE_PIN(kInput6_PinNumber          , 'B',  9);
    _MAKE_MOTATE_PIN(kUnassigned38              , 'B', 10);
    _MAKE_MOTATE_PIN(kUnassigned39              , 'B', 11);
    _MAKE_MOTATE_PIN(kUnassigned40              , 'B', 12);
    _MAKE_MOTATE_PIN(kUnassigned41              , 'B', 13);
    _MAKE_MOTATE_PIN(kSocket3_DirPinNumber      , 'B', 14);
    _MAKE_MOTATE_PIN(kUnassigned42              , 'B', 15);
    _MAKE_MOTATE_PIN(kUnassigned43              , 'B', 16);
    _MAKE_MOTATE_PIN(kUnassigned44              , 'B', 17);
    _MAKE_MOTATE_PIN(kUnassigned45              , 'B', 18);
    _MAKE_MOTATE_PIN(kUnassigned46              , 'B', 19);
    _MAKE_MOTATE_PIN(kUnassigned47              , 'B', 20);
    _MAKE_MOTATE_PIN(kUnassigned48              , 'B', 21);
    _MAKE_MOTATE_PIN(kSocket4_DirPinNumber      , 'B', 22);
    _MAKE_MOTATE_PIN(kSocket1_DirPinNumber      , 'B', 23);
    _MAKE_MOTATE_PIN(kSocket2_DirPinNumber      , 'B', 24);
    _MAKE_MOTATE_PIN(kUnassigned49              , 'B', 25);
    _MAKE_MOTATE_PIN(kUnassigned50              , 'B', 26);
    _MAKE_MOTATE_PIN(kLED_USBTXPinNumber        , 'B', 27);
    _MAKE_MOTATE_PIN(kUnassigned51              , 'B', 28);
    _MAKE_MOTATE_PIN(kUnassigned52              , 'B', 29);
    _MAKE_MOTATE_PIN(kUnassigned53              , 'B', 30);
    _MAKE_MOTATE_PIN(kUnassigned54              , 'B', 31);


    _MAKE_MOTATE_PIN(kLED_USBRXPinNumber        , 'C',  0);
    _MAKE_MOTATE_PIN(kUnassigned55              , 'C',  1);
    _MAKE_MOTATE_PIN(kUnassigned56              , 'C',  2);
    _MAKE_MOTATE_PIN(kUnassigned57              , 'C',  3);
    _MAKE_MOTATE_PIN(kUnassigned58              , 'C',  4);
    _MAKE_MOTATE_PIN(kUnassigned59              , 'C',  5);
    _MAKE_MOTATE_PIN(kUnassigned60              , 'C',  6);
    _MAKE_MOTATE_PIN(kUnassigned61              , 'C',  7);
    _MAKE_MOTATE_PIN(kUnassigned62              , 'C',  8);
    _MAKE_MOTATE_PIN(kLEDPWM_PinNumber          , 'C',  9); // Heartbeat
    _MAKE_MOTATE_PIN(kInput3_PinNumber          , 'C', 10);
    _MAKE_MOTATE_PIN(kUnassigned64              , 'C', 11);
    _MAKE_MOTATE_PIN(kUnassigned65              , 'C', 12);
    _MAKE_MOTATE_PIN(kUnassigned66              , 'C', 13);
    _MAKE_MOTATE_PIN(kUnassigned67              , 'C', 14);
    _MAKE_MOTATE_PIN(kUnassigned68              , 'C', 15);
    _MAKE_MOTATE_PIN(kUnassigned69              , 'C', 16);
    _MAKE_MOTATE_PIN(kUnassigned70              , 'C', 17);
    _MAKE_MOTATE_PIN(kUnassigned71              , 'C', 18);
    _MAKE_MOTATE_PIN(kUnassigned72              , 'C', 19);
    _MAKE_MOTATE_PIN(kUnassigned73              , 'C', 20);
    _MAKE_MOTATE_PIN(kUnassigned74              , 'C', 21);
    _MAKE_MOTATE_PIN(kUnassigned75              , 'C', 22);
    _MAKE_MOTATE_PIN(kSocket4_StepPinNumber     , 'C', 23);
    _MAKE_MOTATE_PIN(kSocket5_StepPinNumber     , 'C', 24);
    _MAKE_MOTATE_PIN(kSocket2_StepPinNumber     , 'C', 25);
    _MAKE_MOTATE_PIN(kSocket3_StepPinNumber     , 'C', 26);
    _MAKE_MOTATE_PIN(kSocket5_DirPinNumber      , 'C', 27);
    _MAKE_MOTATE_PIN(kSocket1_StepPinNumber     , 'C', 28);
    _MAKE_MOTATE_PIN(kUnassigned76              , 'C', 29);
    _MAKE_MOTATE_PIN(kUnassigned77              , 'C', 30);
    _MAKE_MOTATE_PIN(kUnassigned78              , 'C', 31);

    _MAKE_MOTATE_PIN(kInput11_PinNumber         , 'D',  1);
    _MAKE_MOTATE_PIN(kInput9_PinNumber          , 'D',  2);

    _MAKE_MOTATE_PIN(kOutput8_PinNumber         , 'D',  3);
    _MAKE_MOTATE_PIN(kOutput7_PinNumber         , 'D',  4);
    _MAKE_MOTATE_PIN(kOutput6_PinNumber         , 'D',  5);
    _MAKE_MOTATE_PIN(kOutput5_PinNumber         , 'D',  6);
    _MAKE_MOTATE_PIN(kOutput4_PinNumber         , 'D',  7);

    _MAKE_MOTATE_PIN(kOutput3_PinNumber         , 'D',  8);
    _MAKE_MOTATE_PIN(kOutput2_PinNumber         , 'D',  9);
    _MAKE_MOTATE_PIN(kOutput9_PinNumber         , 'D',  10);

}  // namespace Motate

// We then allow each chip-type to have it's own function definitions
// that will refer to these pin assignments.
#include "motate_chip_pin_functions.h"

#endif
