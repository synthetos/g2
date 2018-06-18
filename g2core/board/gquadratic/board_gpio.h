/*
 * gpio.h - Digital IO  handling functions
 * This file is part of the g2core project
 *
 * Copyright (c) 2015 - 2017 Alden S. Hart, Jr.
 * Copyright (c) 2015 - 2017 Robert Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
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
 */
#ifndef BOARD_GPIO_H_ONCE
#define BOARD_GPIO_H_ONCE

// this file is included from the bottom of gpio.h, but we do this for completeness
#include "../../gpio.h"

/*
 * GPIO defines
 */
//--- change as required for board and switch hardware ---//

#define D_IN_CHANNELS       4           // number of digital inputs supported
#define D_OUT_CHANNELS      2           // number of digital outputs supported
#define A_IN_CHANNELS	    0           // number of analog inputs supported
#define A_OUT_CHANNELS	    0           // number of analog outputs supported

#define INPUT_LOCKOUT_MS    10          // milliseconds to go dead after input firing

/*
 * The GPIO objects themselves - this must match up with board_gpio.cpp!
 */

extern gpioDigitalInput*   const d_in[D_IN_CHANNELS];
extern gpioDigitalOutput*  const d_out[D_OUT_CHANNELS];
// extern gpioAnalogInput*    a_in[A_IN_CHANNELS];
// extern gpioAnalogOutput*   a_out[A_OUT_CHANNELS];

// prepare the objects as externs (for config_app to not bloat)
using Motate::IRQPin;
using Motate::PWMOutputPin;
using Motate::PWMLikeOutputPin;
template<bool can_pwm, pin_number... V>
using OutputType = typename std::conditional<can_pwm, PWMOutputPin<V...>, PWMLikeOutputPin<V...>>::type;

extern gpioDigitalInputPin<IRQPin<Motate::kInput1_PinNumber>>  din1;
extern gpioDigitalInputPin<IRQPin<Motate::kInput2_PinNumber>>  din2;
extern gpioDigitalInputPin<IRQPin<Motate::kInput3_PinNumber>>  din3;
extern gpioDigitalInputPin<IRQPin<Motate::kInput4_PinNumber>>  din4;

extern gpioDigitalOutputPin<OutputType<OUTPUT1_PWM,  Motate::kOutput1_PinNumber>>  dout1;
extern gpioDigitalOutputPin<OutputType<OUTPUT2_PWM,  Motate::kOutput2_PinNumber>>  dout2;


#endif // End of include guard: BOARD_GPIO_H_ONCE
