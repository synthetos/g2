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
#include "gpio.h"
#include "hardware.h"

/*
 * GPIO defines
 */
//--- change as required for board and switch hardware ---//

#define INPUT_LOCKOUT_MS    10          // milliseconds to go dead after input firing

/*
 * The GPIO objects themselves - this must match up with board_gpio.cpp!
 */

// prepare the objects as externs (for config_app to not bloat)
using Motate::IRQPin;
using Motate::PWMOutputPin;
using Motate::PWMLikeOutputPin;
using Motate::ADCPin;
using Motate::ADCDifferentialPair;
template<bool can_pwm, Motate::pin_number... V>
using OutputType = typename std::conditional<can_pwm, PWMOutputPin<V...>, PWMLikeOutputPin<V...>>::type;

#define D_IN_CHANNELS       10          // number of digital inputs supported

extern gpioDigitalInputPin<IRQPin<Motate::kInput1_PinNumber>>  din1;
extern gpioDigitalInputPin<IRQPin<Motate::kInput2_PinNumber>>  din2;
extern gpioDigitalInputPin<IRQPin<Motate::kInput3_PinNumber>>  din3;
extern gpioDigitalInputPin<IRQPin<Motate::kInput4_PinNumber>>  din4;
extern gpioDigitalInputPin<IRQPin<Motate::kInput5_PinNumber>>  din5;
extern gpioDigitalInputPin<IRQPin<Motate::kInput6_PinNumber>>  din6;
extern gpioDigitalInputPin<IRQPin<Motate::kInput7_PinNumber>>  din7;
extern gpioDigitalInputPin<IRQPin<Motate::kInput8_PinNumber>>  din8;
extern gpioDigitalInputPin<IRQPin<Motate::kInput9_PinNumber>>  din9;
extern gpioDigitalInputPin<IRQPin<Motate::kInput10_PinNumber>> din10;
// extern gpioDigitalInputPin<IRQPin<Motate::kInput11_PinNumber>> din11;
// extern gpioDigitalInputPin<IRQPin<Motate::kInput12_PinNumber>> din12;

extern gpioDigitalInput*   const d_in[D_IN_CHANNELS];


#define D_OUT_CHANNELS     13           // number of digital outputs supported

extern gpioDigitalOutputPin<OutputType<OUTPUT1_PWM,  Motate::kOutput1_PinNumber>>  dout1;
extern gpioDigitalOutputPin<OutputType<OUTPUT2_PWM,  Motate::kOutput2_PinNumber>>  dout2;
extern gpioDigitalOutputPin<OutputType<OUTPUT3_PWM,  Motate::kOutput3_PinNumber>>  dout3;
extern gpioDigitalOutputPin<OutputType<OUTPUT4_PWM,  Motate::kOutput4_PinNumber>>  dout4;
extern gpioDigitalOutputPin<OutputType<OUTPUT5_PWM,  Motate::kOutput5_PinNumber>>  dout5;
extern gpioDigitalOutputPin<OutputType<OUTPUT6_PWM,  Motate::kOutput6_PinNumber>>  dout6;
extern gpioDigitalOutputPin<OutputType<OUTPUT7_PWM,  Motate::kOutput7_PinNumber>>  dout7;
extern gpioDigitalOutputPin<OutputType<OUTPUT8_PWM,  Motate::kOutput8_PinNumber>>  dout8;
extern gpioDigitalOutputPin<OutputType<OUTPUT9_PWM,  Motate::kOutput9_PinNumber>>  dout9;
extern gpioDigitalOutputPin<OutputType<OUTPUT10_PWM, Motate::kOutput10_PinNumber>> dout10;
extern gpioDigitalOutputPin<OutputType<OUTPUT11_PWM, Motate::kOutput11_PinNumber>> dout11;
extern gpioDigitalOutputPin<OutputType<OUTPUT12_PWM, Motate::kOutput12_PinNumber>> dout12;
extern gpioDigitalOutputPin<OutputType<OUTPUT13_PWM, Motate::kOutput13_PinNumber>> dout13;

extern gpioDigitalOutput*  const d_out[D_OUT_CHANNELS];


#if QUINTIC_REVISION == 'C'

#define A_IN_CHANNELS	 4           // number of analog inputs supported

#include "device/max31865/max31865.h"
#define USING_A_MAX31865 1

#ifndef AI1_ENABLED
#define AI1_ENABLED IO_ENABLED
#endif
#ifndef AI1_TYPE
#define AI1_TYPE gpioAnalogInput::AIN_TYPE_EXTERNAL
#endif
#ifndef AI1_CIRCUIT
#define AI1_CIRCUIT gpioAnalogInput::AIN_CIRCUIT_EXTERNAL
#endif
extern gpioAnalogInputPin<MAX31865<SPIBus_used_t::SPIBusDevice>> ain1;

#ifndef AI2_ENABLED
#define AI2_ENABLED IO_ENABLED
#endif
#ifndef AI2_TYPE
#define AI2_TYPE gpioAnalogInput::AIN_TYPE_EXTERNAL
#endif
#ifndef AI2_CIRCUIT
#define AI2_CIRCUIT gpioAnalogInput::AIN_CIRCUIT_EXTERNAL
#endif
extern gpioAnalogInputPin<MAX31865<SPIBus_used_t::SPIBusDevice>> ain2;

#ifndef AI3_ENABLED
#define AI3_ENABLED IO_ENABLED
#endif
#ifndef AI3_TYPE
#define AI3_TYPE gpioAnalogInput::AIN_TYPE_INTERNAL
#endif
#ifndef AI3_CIRCUIT
#define AI3_CIRCUIT gpioAnalogInput::AIN_CIRCUIT_PULLUP
#endif
extern gpioAnalogInputPin<ADCDifferentialPair<Motate::kADC1_Neg_PinNumber, Motate::kADC1_Pos_PinNumber>> ain3;

#ifndef AI4_ENABLED
#define AI4_ENABLED IO_ENABLED
#endif
#ifndef AI4_TYPE
#define AI4_TYPE gpioAnalogInput::AIN_TYPE_INTERNAL
#endif
#ifndef AI4_CIRCUIT
#define AI4_CIRCUIT gpioAnalogInput::AIN_CIRCUIT_PULLUP
#endif
extern gpioAnalogInputPin<ADCDifferentialPair<Motate::kADC2_Neg_PinNumber, Motate::kADC2_Pos_PinNumber>> ain4;

#endif // 'C'

#if QUINTIC_REVISION == 'D'

#define A_IN_CHANNELS	 4           // number of analog inputs supported

#ifndef AI1_ENABLED
#define AI1_ENABLED IO_ENABLED
#endif
#ifndef AI1_TYPE
#define AI1_TYPE gpioAnalogInput::AIN_TYPE_INTERNAL
#endif
#ifndef AI1_CIRCUIT
#define AI1_CIRCUIT gpioAnalogInput::AIN_CIRCUIT_CC_INV_OPAMP
#endif
extern gpioAnalogInputPin<ADCPin<Motate::kADC1_PinNumber>> ai1;

#ifndef AI2_ENABLED
#define AI2_ENABLED IO_ENABLED
#endif
#ifndef AI2_TYPE
#define AI2_TYPE gpioAnalogInput::AIN_TYPE_INTERNAL
#endif
#ifndef AI2_CIRCUIT
#define AI2_CIRCUIT gpioAnalogInput::AIN_CIRCUIT_CC_INV_OPAMP
#endif
extern gpioAnalogInputPin<ADCPin<Motate::kADC2_PinNumber>> ai2;

#ifndef AI3_ENABLED
#define AI3_ENABLED IO_ENABLED
#endif
#ifndef AI3_TYPE
#define AI3_TYPE gpioAnalogInput::AIN_TYPE_INTERNAL
#endif
#ifndef AI3_CIRCUIT
#define AI3_CIRCUIT gpioAnalogInput::AIN_CIRCUIT_INV_OPAMP
#endif
extern gpioAnalogInputPin<ADCPin<Motate::kADC3_PinNumber>> ai3;

#ifndef AI4_ENABLED
#define AI4_ENABLED IO_ENABLED
#endif
#ifndef AI4_TYPE
#define AI4_TYPE gpioAnalogInput::AIN_TYPE_INTERNAL
#endif
#ifndef AI4_CIRCUIT
#define AI4_CIRCUIT gpioAnalogInput::AIN_CIRCUIT_INV_OPAMP
#endif
extern gpioAnalogInputPin<ADCPin<Motate::kADC4_PinNumber>> ai4;

#endif // 'D'

extern gpioAnalogInput*    const a_in[A_IN_CHANNELS];

#endif // End of include guard: BOARD_GPIO_H_ONCE
