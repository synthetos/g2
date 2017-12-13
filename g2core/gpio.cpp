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

#include "g2core.h"  // #1
#include "config.h"  // #2
#include "gpio.h"
#include "stepper.h"
#include "encoder.h"
#include "hardware.h"
#include "canonical_machine.h"

#include "text_parser.h"
#include "controller.h"
#include "util.h"
#include "report.h"
#include "xio.h"

#include "MotateTimers.h"
using namespace Motate;

/****
 *
 * Important: Do not directly allocate or access pins here!
 * Do that in board_gpio.cpp!
 *
 ****/


/************************************************************************************
 **** CODE **************************************************************************
 ************************************************************************************/
/*
 * gpio_init() - initialize inputs and outputs
 * gpio_reset() - reset inputs and outputs (no initialization)
 */

void gpio_init(void)
{
    return(gpio_reset());
}

// implement void outputs_reset(void) in board_gpio.cpp
// implement void inputs_reset(void) in board_gpio.cpp

void gpio_reset(void)
{
    inputs_reset();
    outputs_reset();
}

/********************************************
 **** Digital Input Supporting Functions ****
 ********************************************/

/*
 * gpio_set_homing_mode()   - set/clear input to homing mode
 * gpio_set_probing_mode()  - set/clear input to probing mode
 * gpio_get_probing_input() - get probing input
 * gpio_read_input()        - read conditioned input
 *
 * Note: input_num_ext means EXTERNAL input number -- 1-based
 * TODO: lookup the external number to find the internal input, don't assume
 */
void  gpio_set_homing_mode(const uint8_t input_num_ext, const bool is_homing)
{
    if (input_num_ext == 0) {
        return;
    }
    d_in[input_num_ext-1]->setIsHoming(is_homing);
}

void  gpio_set_probing_mode(const uint8_t input_num_ext, const bool is_probing)
{
    if (input_num_ext == 0) {
        return;
    }
    d_in[input_num_ext-1]->setIsProbing(is_probing);
}

int8_t gpio_get_probing_input(void)
{
    for (uint8_t i = 0; i <= D_IN_CHANNELS; i++) {
        if (d_in[i-1]->getFunction() == INPUT_FUNCTION_PROBE) {
            return (i);
        }
    }
    return (-1);
}

bool gpio_read_input(const uint8_t input_num_ext)
{
    if (input_num_ext == 0) {
        return false;
    }
    return (d_in[input_num_ext-1]->getState());
}


/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 * These functions are not part of the NIST defined functions
 ***********************************************************************************/

// internal helpers to get input and output object from the cfgArray target
// specified by an nv pointer

template <typename type>
type* _io(const nvObj_t *nv) {
 return reinterpret_cast<type*>(cfgArray[nv->index].target);
};

gpioDigitalInput* _i(const nvObj_t *nv) { return _io<gpioDigitalInput>(nv); }
gpioDigitalOutput* _o(const nvObj_t *nv) { return _io<gpioDigitalOutput>(nv); }


// static stat_t _input_set_helper(nvObj_t *nv, const int8_t lower_bound, const int8_t upper_bound)
// {
//     if ((nv->value < lower_bound) || (nv->value >= upper_bound)) {
//         return (STAT_INPUT_VALUE_RANGE_ERROR);
//     }
//     set_ui8(nv);        // will this work in -1 is a valid value?
//     if (cm_get_machine_state() != MACHINE_INITIALIZING) {
//         inputs_reset();
//     }
//     return (STAT_OK);
// }
//
// static stat_t _output_set_helper(nvObj_t *nv, const int8_t lower_bound, const int8_t upper_bound)
// {
//     if ((nv->value < lower_bound) || (nv->value >= upper_bound)) {
//         return (STAT_INPUT_VALUE_RANGE_ERROR);
//     }
//     set_ui8(nv);        // will this work in -1 is a valid value?
//     if (cm_get_machine_state() != MACHINE_INITIALIZING) {
//         outputs_reset();
//     }
//     return (STAT_OK);
// }

/*
 *  Get/set input mode
 */
stat_t din_get_mo(nvObj_t *nv)
{
    return _i(nv)->getMode(nv);
}
stat_t din_set_mo(nvObj_t *nv)
{
    return _i(nv)->setMode(nv);
}

/*
 *  Get/set input action
 */
stat_t din_get_ac(nvObj_t *nv)
{
    return _i(nv)->getAction(nv);
}
stat_t din_set_ac(nvObj_t *nv)
{
    return _i(nv)->setAction(nv);
}

/*
 *  Get/set input function
 */
stat_t din_get_fn(nvObj_t *nv)
{
    return _i(nv)->getFunction(nv);
}
stat_t din_set_fn(nvObj_t *nv)
{
    return _i(nv)->setFunction(nv);
}

/*
 *  io_get_input() - return input state given an nv object
 *  Note: if the input is disabled, it returns NULL.
 */
stat_t din_get_input(nvObj_t *nv)
{
    return _i(nv)->getState(nv);
}


stat_t dout_get_mo(nvObj_t *nv)
{
    return _o(nv)->getMode(nv);
}
stat_t dout_set_mo(nvObj_t *nv)
{
    return _o(nv)->setMode(nv);
}

/*
 *  io_get_output()/io_set_output() - get/set output state given an nv object
 */
stat_t dout_get_output(nvObj_t *nv)
{
    return _o(nv)->getValue(nv);
}
stat_t dout_set_output(nvObj_t *nv)
{
    return _o(nv)->setValue(nv);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

    static const char fmt_gpio_mo[] = "[%smo] input mode%17d [0=active-low,1=active-hi,2=disabled]\n";
    static const char fmt_gpio_ac[] = "[%sac] input action%15d [0=none,1=stop,2=fast_stop,3=halt,4=alarm,5=shutdown,6=panic,7=reset]\n";
    static const char fmt_gpio_fn[] = "[%sfn] input function%13d [0=none,1=limit,2=interlock,3=shutdown,4=probe]\n";
    static const char fmt_gpio_in[] = "Input %s state: %5d\n";

    static const char fmt_gpio_domode[] = "[%smo] output mode%16d [0=active low,1=active high,2=disabled]\n";
    static const char fmt_gpio_out[] = "Output %s state: %5d\n";

    static void _print_di(nvObj_t *nv, const char *format)
    {
        sprintf(cs.out_buf, format, nv->group, (int)nv->value);
        xio_writeline(cs.out_buf);
    }
    void din_print_mo(nvObj_t *nv) {_print_di(nv, fmt_gpio_mo);}
    void din_print_ac(nvObj_t *nv) {_print_di(nv, fmt_gpio_ac);}
    void din_print_fn(nvObj_t *nv) {_print_di(nv, fmt_gpio_fn);}
    void din_print_in(nvObj_t *nv) {
        sprintf(cs.out_buf, fmt_gpio_in, nv->token, (int)nv->value);
        xio_writeline(cs.out_buf);
    }

    void dout_print_mo(nvObj_t *nv) {_print_di(nv, fmt_gpio_domode);}
    void dout_print_out(nvObj_t *nv) {
        sprintf(cs.out_buf, fmt_gpio_out, nv->token, (int)nv->value);
        xio_writeline(cs.out_buf);
    }
#endif
