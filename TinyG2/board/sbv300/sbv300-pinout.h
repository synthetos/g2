/*
 * http://tinkerin.gs/
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

#ifndef sbv300_pinout_h
#define sbv300_pinout_h

#include <MotatePins.h>

// We don't have all of the inputs, so we don't define them.
#define INPUT1_AVAILABLE 1
#define INPUT2_AVAILABLE 0
#define INPUT3_AVAILABLE 0
#define INPUT4_AVAILABLE 0
#define INPUT5_AVAILABLE 0
#define INPUT6_AVAILABLE 0
#define INPUT7_AVAILABLE 0
#define INPUT8_AVAILABLE 0
#define INPUT9_AVAILABLE 0
#define INPUT10_AVAILABLE 0
#define INPUT11_AVAILABLE 0
#define INPUT12_AVAILABLE 0
#define INPUT13_AVAILABLE 0

#define ADC0_AVAILABLE 0
#define ADC1_AVAILABLE 0
#define ADC2_AVAILABLE 0
#define ADC3_AVAILABLE 0

#define TEMPERATURE_OUTPUT_ON 0

namespace Motate {

    // NEED ASSIGNED: kSpindle_PwmPinNumber

    _MAKE_MOTATE_PIN(kSpindle_EnablePinNumber   , A, 'A',  0);
    _MAKE_MOTATE_PIN(kUnassigned1               , A, 'A',  1);
    _MAKE_MOTATE_PIN(kUnassigned2               , A, 'A',  2);
    _MAKE_MOTATE_PIN(kUnassigned3               , A, 'A',  3);
    _MAKE_MOTATE_PIN(kUnassigned4               , A, 'A',  4);
    _MAKE_MOTATE_PIN(kUnassigned5               , A, 'A',  5);
    _MAKE_MOTATE_PIN(kUnassigned6               , A, 'A',  6);
    _MAKE_MOTATE_PIN(kUnassigned7               , A, 'A',  7);
    _MAKE_MOTATE_PIN(kUnassigned8               , A, 'A',  8);
    _MAKE_MOTATE_PIN(kUnassigned9               , A, 'A',  9);
    _MAKE_MOTATE_PIN(kUnassigned10              , A, 'A', 10);
    _MAKE_MOTATE_PIN(kUnassigned11              , A, 'A', 11);
    _MAKE_MOTATE_PIN(kUnassigned12              , A, 'A', 12);
    _MAKE_MOTATE_PIN(kUnassigned13              , A, 'A', 13);
    _MAKE_MOTATE_PIN(kUnassigned14              , A, 'A', 14);
    _MAKE_MOTATE_PIN(kUnassigned15              , A, 'A', 15);
    _MAKE_MOTATE_PIN(kUnassigned16              , A, 'A', 16);
    _MAKE_MOTATE_PIN(kUnassigned17              , A, 'A', 17);
    _MAKE_MOTATE_PIN(kUnassigned18              , A, 'A', 18);
    _MAKE_MOTATE_PIN(kUnassigned19              , A, 'A', 19);
    _MAKE_MOTATE_PIN(kUnassigned20              , A, 'A', 20);
    _MAKE_MOTATE_PIN(kUnassigned21              , A, 'A', 21);
    _MAKE_MOTATE_PIN(kUnassigned22              , A, 'A', 22);
    _MAKE_MOTATE_PIN(kUnassigned23              , A, 'A', 23);
    _MAKE_MOTATE_PIN(kUnassigned24              , A, 'A', 24);
    _MAKE_MOTATE_PIN(kUnassigned25              , A, 'A', 25);
    _MAKE_MOTATE_PIN(kUnassigned26              , A, 'A', 26);
    _MAKE_MOTATE_PIN(kUnassigned27              , A, 'A', 27);
    _MAKE_MOTATE_PIN(kUnassigned28              , A, 'A', 28);
    _MAKE_MOTATE_PIN(kUnassigned29              , A, 'A', 29);

    _MAKE_MOTATE_PIN(kSocket6_DirPinNumber      , B, 'B',  0);
    _MAKE_MOTATE_PIN(kSocket6_StepPinNumber     , B, 'B',  1);
    _MAKE_MOTATE_PIN(kUnassigned30              , B, 'B',  2);
    _MAKE_MOTATE_PIN(kUnassigned31              , B, 'B',  3);
    _MAKE_MOTATE_PIN(kUnassigned32              , B, 'B',  4);
    _MAKE_MOTATE_PIN(kUnassigned33              , B, 'B',  5);
    _MAKE_MOTATE_PIN(kUnassigned34              , B, 'B',  6);
    _MAKE_MOTATE_PIN(kUnassigned35              , B, 'B',  7);
    _MAKE_MOTATE_PIN(kUnassigned36              , B, 'B',  8);
    _MAKE_MOTATE_PIN(kUnassigned37              , B, 'B',  9);
    _MAKE_MOTATE_PIN(kUnassigned38              , B, 'B', 10);
    _MAKE_MOTATE_PIN(kUnassigned39              , B, 'B', 11);
    _MAKE_MOTATE_PIN(kUnassigned40              , B, 'B', 12);
    _MAKE_MOTATE_PIN(kUnassigned41              , B, 'B', 13);
    _MAKE_MOTATE_PIN(kSocket3_DirPinNumber      , B, 'B', 14);
    _MAKE_MOTATE_PIN(kUnassigned42              , B, 'B', 15);
    _MAKE_MOTATE_PIN(kUnassigned43              , B, 'B', 16);
    _MAKE_MOTATE_PIN(kUnassigned44              , B, 'B', 17);
    _MAKE_MOTATE_PIN(kUnassigned45              , B, 'B', 18);
    _MAKE_MOTATE_PIN(kUnassigned46              , B, 'B', 19);
    _MAKE_MOTATE_PIN(kUnassigned47              , B, 'B', 20);
    _MAKE_MOTATE_PIN(kUnassigned48              , B, 'B', 21);
    _MAKE_MOTATE_PIN(kSocket4_DirPinNumber      , B, 'B', 22);
    _MAKE_MOTATE_PIN(kSocket1_DirPinNumber      , B, 'B', 23);
    _MAKE_MOTATE_PIN(kSocket2_DirPinNumber      , B, 'B', 24);
    _MAKE_MOTATE_PIN(kUnassigned49              , B, 'B', 25);
    _MAKE_MOTATE_PIN(kUnassigned50              , B, 'B', 26);
    _MAKE_MOTATE_PIN(kLED_USBTXPinNumber        , B, 'B', 27);
    _MAKE_MOTATE_PIN(kUnassigned51              , B, 'B', 28);
    _MAKE_MOTATE_PIN(kUnassigned52              , B, 'B', 29);
    _MAKE_MOTATE_PIN(kUnassigned53              , B, 'B', 30);
    _MAKE_MOTATE_PIN(kUnassigned54              , B, 'B', 31);


    _MAKE_MOTATE_PIN(kLED_USBRXPinNumber        , C, 'C',  0);
    _MAKE_MOTATE_PIN(kUnassigned55              , C, 'C',  1);
    _MAKE_MOTATE_PIN(kUnassigned56              , C, 'C',  2);
    _MAKE_MOTATE_PIN(kUnassigned57              , C, 'C',  3);
    _MAKE_MOTATE_PIN(kUnassigned58              , C, 'C',  4);
    _MAKE_MOTATE_PIN(kUnassigned59              , C, 'C',  5);
    _MAKE_MOTATE_PIN(kUnassigned60              , C, 'C',  6);
    _MAKE_MOTATE_PIN(kUnassigned61              , C, 'C',  7);
    _MAKE_MOTATE_PIN(kUnassigned62              , C, 'C',  8);
    _MAKE_MOTATE_PIN(kUnassigned63              , C, 'C',  9);
    _MAKE_MOTATE_PIN(kInput1_PinNumber          , C, 'C', 10);
    _MAKE_MOTATE_PIN(kUnassigned64              , C, 'C', 11);
    _MAKE_MOTATE_PIN(kUnassigned65              , C, 'C', 12);
    _MAKE_MOTATE_PIN(kUnassigned66              , C, 'C', 13);
    _MAKE_MOTATE_PIN(kUnassigned67              , C, 'C', 14);
    _MAKE_MOTATE_PIN(kUnassigned68              , C, 'C', 15);
    _MAKE_MOTATE_PIN(kUnassigned69              , C, 'C', 16);
    _MAKE_MOTATE_PIN(kUnassigned70              , C, 'C', 17);
    _MAKE_MOTATE_PIN(kUnassigned71              , C, 'C', 18);
    _MAKE_MOTATE_PIN(kUnassigned72              , C, 'C', 19);
    _MAKE_MOTATE_PIN(kUnassigned73              , C, 'C', 20);
    _MAKE_MOTATE_PIN(kUnassigned74              , C, 'C', 21);
    _MAKE_MOTATE_PIN(kUnassigned75              , C, 'C', 22);
    _MAKE_MOTATE_PIN(kSocket4_StepPinNumber     , C, 'C', 23);
    _MAKE_MOTATE_PIN(kSocket5_StepPinNumber     , C, 'C', 24);
    _MAKE_MOTATE_PIN(kSocket2_StepPinNumber     , C, 'C', 25);
    _MAKE_MOTATE_PIN(kSocket3_StepPinNumber     , C, 'C', 26);
    _MAKE_MOTATE_PIN(kSocket5_DirPinNumber      , C, 'C', 27);
    _MAKE_MOTATE_PIN(kSocket1_StepPinNumber     , C, 'C', 28);
    _MAKE_MOTATE_PIN(kUnassigned76              , C, 'C', 29);
    _MAKE_MOTATE_PIN(kUnassigned77              , C, 'C', 30);
    _MAKE_MOTATE_PIN(kUnassigned78              , C, 'C', 31);

    _MAKE_MOTATE_PIN(kSpindle_DirPinNumber      , D, 'D',  7);
    _MAKE_MOTATE_PIN(kUnassigned79              , D, 'D',  8);

} // namespace Motate

// We then allow each chip-type to have it's own function definitions
// that will refer to these pin assignments.
#include "motate_chip_pin_functions.h"

#endif
