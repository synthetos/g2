/*
 * gpio.cpp - digital IO handling functions
 * This file is part of the g2core project
 *
 * Copyright (c) 2015 - 2107 Alden S. Hart, Jr.
 * Copyright (c) 2015 - 2017 Robert Giseburt
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
 */
/* Switch Modes
 *
 *  The switches are considered to be homing switches when cycle_state is
 *  CYCLE_HOMING. At all other times they are treated as limit switches:
 *    - Hitting a homing switch puts the current move into feedhold
 *    - Hitting a limit switch causes the machine to shut down and go into lockdown until reset
 *
 *  The normally open switch modes (NO) trigger an interrupt on the falling edge
 *  and lockout subsequent interrupts for the defined lockout period. This approach
 *  beats doing debouncing as an integration as switches fire immediately.
 *
 *  The normally closed switch modes (NC) trigger an interrupt on the rising edge
 *  and lockout subsequent interrupts for the defined lockout period. Ditto on the method.
 */

#include "../../g2core.h"  // #1
#include "config.h"  // #2
#include "gpio.h"
#include "hardware.h"
#include "canonical_machine.h"

#include "text_parser.h"
#include "controller.h"
#include "util.h"
#include "report.h"
#include "xio.h"

#include "MotateTimers.h"

/**** Setup Actual Objects ****/

gpioDigitalInputPin<IRQPin<Motate::kInput1_PinNumber>>  din1  {DI1_ENABLED,  DI1_POLARITY,  1, DI1_EXTERNAL_NUMBER};
gpioDigitalInputPin<IRQPin<Motate::kInput2_PinNumber>>  din2  {DI2_ENABLED,  DI2_POLARITY,  2, DI2_EXTERNAL_NUMBER};
gpioDigitalInputPin<IRQPin<Motate::kInput3_PinNumber>>  din3  {DI3_ENABLED,  DI3_POLARITY,  3, DI3_EXTERNAL_NUMBER};
gpioDigitalInputPin<IRQPin<Motate::kInput4_PinNumber>>  din4  {DI4_ENABLED,  DI4_POLARITY,  4, DI4_EXTERNAL_NUMBER};

gpioDigitalOutputPin<OutputType<OUTPUT1_PWM,  Motate::kOutput1_PinNumber>>  dout1  { DO1_ENABLED,  DO1_POLARITY, DO1_EXTERNAL_NUMBER,  (uint32_t)200000 };
gpioDigitalOutputPin<OutputType<OUTPUT2_PWM,  Motate::kOutput2_PinNumber>>  dout2  { DO2_ENABLED,  DO2_POLARITY, DO2_EXTERNAL_NUMBER,  (uint32_t)200000 };

/**** Setup Arrays - these are extern and MUST match the board_gpio.h ****/

gpioDigitalInput*  const d_in[] = {&din1, &din2, &din3, &din4};
gpioDigitalOutput* const d_out[] = {&dout1, &dout2};
// not yet used
// gpioAnalogInput*    a_in[A_IN_CHANNELS];
// gpioAnalogOutput*   a_out[A_OUT_CHANNELS];

/************************************************************************************
 **** CODE **************************************************************************
 ************************************************************************************/
/*
 * gpio_reset() - reset inputs and outputs (no initialization)
 */


void outputs_reset(void) {
    // nothing to do
}

void inputs_reset(void) {
    // nothing to do
}
