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

/**** Allocate structures ****/

d_in_t   d_in[D_IN_CHANNELS];
d_out_t  d_out[D_OUT_CHANNELS];
a_in_t   a_in[A_IN_CHANNELS];
a_out_t  a_out[A_OUT_CHANNELS];

/**** Extended DI structure ****/

// To be merged with ioDigitalInput later.
// For now, we use a pointer to the correct d_in, since the old code did.
// input_pin_num is the Motate pin number.
// ext_pin_number is the JSON ("external") pin number, as in "di1".
template <pin_number input_pin_num, uint8_t ext_pin_number>
struct ioDigitalInputExt {
    IRQPin<input_pin_num>  input_pin;

    /* Priority only needs set once in the system during startup.
     * However, if we wish to switch the interrupt trigger, here are other options:
     *  kPinInterruptOnRisingEdge
     *  kPinInterruptOnFallingEdge
     *
     * To change the trigger or priority provide a third paramater intValue that will
     * be called as pin.setInterrupts(intValue), or call pin.setInterrupts at any point.
     * Note that it may cause an interrupt to fire *immediately*!
     * intValue defaults to kPinInterruptOnChange|kPinInterruptPriorityMedium if not specified.
     */
    ioDigitalInputExt() : input_pin {kPullUp|kDebounce, [&]{this->pin_changed();}} {
    };

    ioDigitalInputExt(const ioDigitalInputExt&) = delete; // delete copy
    ioDigitalInputExt(ioDigitalInputExt&&) = delete;      // delete move

    void reset() {
        if (D_IN_CHANNELS < ext_pin_number) { return; }

        d_in_t *in = &d_in[ext_pin_number-1];

        if (in->mode == IO_MODE_DISABLED) {
            in->state = INPUT_DISABLED;
            return;
        }

        bool pin_value = (bool)input_pin;
        int8_t pin_value_corrected = (pin_value ^ ((int)in->mode ^ 1));    // correct for NO or NC mode
        in->state = (ioState)pin_value_corrected;
    }

    void pin_changed() {
        if (D_IN_CHANNELS < ext_pin_number) { return; }

        d_in_t *in = &d_in[ext_pin_number-1];

        // return if input is disabled (not supposed to happen)
        if (in->mode == IO_MODE_DISABLED) {
            in->state = INPUT_DISABLED;
            return;
        }

        // return if the input is in lockout period (take no action)
        if (in->lockout_timer.isSet() && !in->lockout_timer.isPast()) {
            return;
        }

        // return if no change in state
        bool pin_value = (bool)input_pin;
        int8_t pin_value_corrected = (pin_value ^ ((int)in->mode ^ 1));    // correct for NO or NC mode
        if (in->state == (ioState)pin_value_corrected) {
            return;
        }

        // lockout the pin for lockout_ms
        in->lockout_timer.set(in->lockout_ms);

        // record the changed state
        in->state = (ioState)pin_value_corrected;
        if (pin_value_corrected == INPUT_ACTIVE) {
            in->edge = INPUT_EDGE_LEADING;
        } else {
            in->edge = INPUT_EDGE_TRAILING;
        }

        // perform homing operations if in homing mode
        if (in->homing_mode) {
            if (in->edge == INPUT_EDGE_LEADING) {   // we only want the leading edge to fire
                en_take_encoder_snapshot();
                cm_start_hold();
            }
            return;
        }

        // perform probing operations if in probing mode
        if (in->probing_mode) {
            // We want to capture either way.
            // Probing tests the start condition for the correct direction ahead of time.
            // If we see any edge, it's the right one.
            en_take_encoder_snapshot();
            cm_start_hold();
            return;
        }

        // *** NOTE: From this point on all conditionals assume we are NOT in homing or probe mode ***

        // trigger the action on leading edges
        if (in->edge == INPUT_EDGE_LEADING) {
            if (in->action == INPUT_ACTION_STOP) {
                cm_start_hold();
            }
            if (in->action == INPUT_ACTION_FAST_STOP) {
                cm_start_hold();                        // for now is same as STOP
            }
            if (in->action == INPUT_ACTION_HALT) {
                cm_halt_all();                            // hard stop, including spindle and coolant
            }
            if (in->action == INPUT_ACTION_ALARM) {
                char msg[10];
                sprintf(msg, "input %d", ext_pin_number);
                cm_alarm(STAT_ALARM, msg);
            }
            if (in->action == INPUT_ACTION_SHUTDOWN) {
                char msg[10];
                sprintf(msg, "input %d", ext_pin_number);
                cm_shutdown(STAT_SHUTDOWN, msg);
            }
            if (in->action == INPUT_ACTION_PANIC) {
                char msg[10];
                sprintf(msg, "input %d", ext_pin_number);
                cm_panic(STAT_PANIC, msg);
            }
            if (in->action == INPUT_ACTION_RESET) {
                hw_hard_reset();
            }
        }

        // these functions trigger on the leading edge
        if (in->edge == INPUT_EDGE_LEADING) {
            if (in->function == INPUT_FUNCTION_LIMIT) {
                cm.limit_requested = ext_pin_number;

            } else if (in->function == INPUT_FUNCTION_SHUTDOWN) {
                cm.shutdown_requested = ext_pin_number;

            } else if (in->function == INPUT_FUNCTION_INTERLOCK) {
                cm.safety_interlock_disengaged = ext_pin_number;
            }
        }

        // trigger interlock release on trailing edge
        if (in->edge == INPUT_EDGE_TRAILING) {
            if (in->function == INPUT_FUNCTION_INTERLOCK) {
                cm.safety_interlock_reengaged = ext_pin_number;
            }
        }

        sr_request_status_report(SR_REQUEST_TIMED);   //+++++ Put this one back in.
    };
};

/**** Setup Low Level Stuff ****/

ioDigitalInputExt<kInput1_PinNumber  ,  1> _din1;
ioDigitalInputExt<kInput2_PinNumber  ,  2> _din2;
ioDigitalInputExt<kInput3_PinNumber  ,  3> _din3;
ioDigitalInputExt<kInput4_PinNumber  ,  4> _din4;
ioDigitalInputExt<kInput5_PinNumber  ,  5> _din5;
ioDigitalInputExt<kInput6_PinNumber  ,  6> _din6;
ioDigitalInputExt<kInput7_PinNumber  ,  7> _din7;
ioDigitalInputExt<kInput8_PinNumber  ,  8> _din8;
ioDigitalInputExt<kInput9_PinNumber  ,  9> _din9;
ioDigitalInputExt<kInput10_PinNumber , 10> _din10;
ioDigitalInputExt<kInput11_PinNumber , 11> _din11;
ioDigitalInputExt<kInput12_PinNumber , 12> _din12;

// Generated with:
// perl -e 'for($i=1;$i<14;$i++) { print "#if OUTPUT${i}_PWM == 1\nstatic PWMOutputPin<kOutput${i}_PinNumber>  output_${i}_pin;\n#else\nstatic PWMLikeOutputPin<kOutput${i}_PinNumber>  output_${i}_pin;\n#endif\n";}'
// BEGIN generated
#if OUTPUT1_PWM == 1
static PWMOutputPin<kOutput1_PinNumber>  output_1_pin {kNormal, 200000};
#else
static PWMLikeOutputPin<kOutput1_PinNumber>  output_1_pin;
#endif
#if OUTPUT2_PWM == 1
static PWMOutputPin<kOutput2_PinNumber>  output_2_pin {kNormal, 200000};
#else
static PWMLikeOutputPin<kOutput2_PinNumber>  output_2_pin;
#endif
#if OUTPUT3_PWM == 1
static PWMOutputPin<kOutput3_PinNumber>  output_3_pin {kNormal, 200000};
#else
static PWMLikeOutputPin<kOutput3_PinNumber>  output_3_pin;
#endif
#if OUTPUT4_PWM == 1
static PWMOutputPin<kOutput4_PinNumber>  output_4_pin {kNormal, 200000};
#else
static PWMLikeOutputPin<kOutput4_PinNumber>  output_4_pin;
#endif
#if OUTPUT5_PWM == 1
static PWMOutputPin<kOutput5_PinNumber>  output_5_pin {kNormal, 200000};
#else
static PWMLikeOutputPin<kOutput5_PinNumber>  output_5_pin;
#endif
#if OUTPUT6_PWM == 1
static PWMOutputPin<kOutput6_PinNumber>  output_6_pin {kNormal, 200000};
#else
static PWMLikeOutputPin<kOutput6_PinNumber>  output_6_pin;
#endif
#if OUTPUT7_PWM == 1
static PWMOutputPin<kOutput7_PinNumber>  output_7_pin {kNormal, 200000};
#else
static PWMLikeOutputPin<kOutput7_PinNumber>  output_7_pin;
#endif
#if OUTPUT8_PWM == 1
static PWMOutputPin<kOutput8_PinNumber>  output_8_pin {kNormal, 200000};
#else
static PWMLikeOutputPin<kOutput8_PinNumber>  output_8_pin;
#endif
#if OUTPUT9_PWM == 1
static PWMOutputPin<kOutput9_PinNumber>  output_9_pin {kNormal, 200000};
#else
static PWMLikeOutputPin<kOutput9_PinNumber>  output_9_pin;
#endif
#if OUTPUT10_PWM == 1
static PWMOutputPin<kOutput10_PinNumber>  output_10_pin {kNormal, 200000};
#else
static PWMLikeOutputPin<kOutput10_PinNumber>  output_10_pin;
#endif
#if OUTPUT11_PWM == 1
static PWMOutputPin<kOutput11_PinNumber>  output_11_pin {kNormal, 200000};
#else
static PWMLikeOutputPin<kOutput11_PinNumber>  output_11_pin;
#endif
#if OUTPUT12_PWM == 1
static PWMOutputPin<kOutput12_PinNumber>  output_12_pin {kNormal, 200000};
#else
static PWMLikeOutputPin<kOutput12_PinNumber>  output_12_pin;
#endif
#if OUTPUT13_PWM == 1
static PWMOutputPin<kOutput13_PinNumber>  output_13_pin {kNormal, 200000};
#else
static PWMLikeOutputPin<kOutput13_PinNumber>  output_13_pin;
#endif
// END generated

/************************************************************************************
 **** CODE **************************************************************************
 ************************************************************************************/
/*
 * gpio_init() - initialize inputs and outputs
 * gpio_reset() - reset inputs and outputs (no initialization)
 */

void gpio_init(void)
{
    // These are here due to an unfound glitch where for Timer-based pins the frequency isn't getting set.

    // Generated with:
    // perl -e 'for($i=1;$i<14;$i++) { print "output_${i}_pin.setFrequency(200000);\n";}'
    // BEGIN generated
    output_1_pin.setFrequency(200000);
    output_2_pin.setFrequency(200000);
    output_3_pin.setFrequency(200000);
    output_4_pin.setFrequency(200000);
    output_5_pin.setFrequency(200000);
    output_6_pin.setFrequency(200000);
    output_7_pin.setFrequency(200000);
    output_8_pin.setFrequency(200000);
    output_9_pin.setFrequency(200000);
    output_10_pin.setFrequency(200000);
    output_11_pin.setFrequency(200000);
    output_12_pin.setFrequency(200000);
    output_13_pin.setFrequency(200000);
    // END generated

    return(gpio_reset());
}

void outputs_reset(void) {
    // If the output is ACTIVE_LOW set it to 1. ACTIVE_HIGH gets set to 0.
#if D_OUT_CHANNELS >= 1
    if (d_out[1-1].mode  != IO_MODE_DISABLED) { (output_1_pin    = (d_out[1-1].mode  == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
#endif
#if D_OUT_CHANNELS >= 2
    if (d_out[2-1].mode  != IO_MODE_DISABLED) { (output_2_pin    = (d_out[2-1].mode  == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
#endif
#if D_OUT_CHANNELS >= 3
    if (d_out[3-1].mode  != IO_MODE_DISABLED) { (output_3_pin    = (d_out[3-1].mode  == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
#endif
#if D_OUT_CHANNELS >= 4
    if (d_out[4-1].mode  != IO_MODE_DISABLED) { (output_4_pin    = (d_out[4-1].mode  == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
#endif
#if D_OUT_CHANNELS >= 5
    if (d_out[5-1].mode  != IO_MODE_DISABLED) { (output_5_pin    = (d_out[5-1].mode  == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
#endif
#if D_OUT_CHANNELS >= 6
    if (d_out[6-1].mode  != IO_MODE_DISABLED) { (output_6_pin    = (d_out[6-1].mode  == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
#endif
#if D_OUT_CHANNELS >= 7
    if (d_out[7-1].mode  != IO_MODE_DISABLED) { (output_7_pin    = (d_out[7-1].mode  == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
#endif
#if D_OUT_CHANNELS >= 8
    if (d_out[8-1].mode  != IO_MODE_DISABLED) { (output_8_pin    = (d_out[8-1].mode  == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
#endif
#if D_OUT_CHANNELS >= 9
    if (d_out[9-1].mode  != IO_MODE_DISABLED) { (output_9_pin    = (d_out[9-1].mode  == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
#endif
#if D_OUT_CHANNELS >= 10
    if (d_out[10-1].mode != IO_MODE_DISABLED) { (output_10_pin   = (d_out[10-1].mode == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
#endif
#if D_OUT_CHANNELS >= 11
    if (d_out[11-1].mode != IO_MODE_DISABLED) { (output_11_pin   = (d_out[11-1].mode == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
#endif
#if D_OUT_CHANNELS >= 12
    if (d_out[12-1].mode != IO_MODE_DISABLED) { (output_12_pin   = (d_out[12-1].mode == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
#endif
#if D_OUT_CHANNELS >= 13
    if (d_out[13-1].mode != IO_MODE_DISABLED) { (output_13_pin   = (d_out[13-1].mode == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
#endif
}

void inputs_reset(void) {
    d_in_t *in;

    for (uint8_t i=0; i<D_IN_CHANNELS; i++) {
        in = &d_in[i];
        if (in->mode == IO_MODE_DISABLED) {
            in->state = INPUT_DISABLED;
            continue;
        }
        in->lockout_ms = INPUT_LOCKOUT_MS;
        in->lockout_timer.clear();
    }

  _din1.reset();
  _din2.reset();
  _din3.reset();
  _din4.reset();
  _din5.reset();
  _din6.reset();
  _din7.reset();
  _din8.reset();
  _din9.reset();
  _din10.reset();
  _din11.reset();
  _din12.reset();

}

void gpio_reset(void)
{
    inputs_reset();
    outputs_reset();
}

/******************************
 * Interrupt Service Routines *
 ******************************/
/*
 * ARM pin change interrupts are setup above when defining the IRQPins (inside the ioDigitalInputExt).
 */

/********************************************
 **** Digital Input Supporting Functions ****
 ********************************************/
/*
 * switch_rtc_callback() - called from RTC for each RTC tick.
 *
 *  Each switch has a counter which is initially set to negative SW_DEGLITCH_TICKS.
 *  When a switch closure is DETECTED the count increments for each RTC tick.
 *  When the count reaches zero the switch is tripped and action occurs.
 *  The counter continues to increment positive until the lockout is exceeded.
 */

/*
 * gpio_set_homing_mode()   - set/clear input to homing mode
 * gpio_set_probing_mode()  - set/clear input to probing mode
 * gpio_get_probing_input() - get probing input
 * gpio_read_input()        - read conditioned input
 *
 (* Note: input_num_ext means EXTERNAL input number -- 1-based
 */
void  gpio_set_homing_mode(const uint8_t input_num_ext, const bool is_homing)
{
    if (input_num_ext == 0) {
        return;
    }
    d_in[input_num_ext-1].homing_mode = is_homing;
}

void  gpio_set_probing_mode(const uint8_t input_num_ext, const bool is_probing)
{
    if (input_num_ext == 0) {
        return;
    }
    d_in[input_num_ext-1].probing_mode = is_probing;
}

int8_t gpio_get_probing_input(void) 
{
    for (uint8_t i = 0; i <= D_IN_CHANNELS; i++) {
        if (d_in[i-1].function == INPUT_FUNCTION_PROBE) {
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
    return (d_in[input_num_ext-1].state);
}


/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 * These functions are not part of the NIST defined functions
 ***********************************************************************************/

static stat_t _input_set_helper(nvObj_t *nv, const int8_t lower_bound, const int8_t upper_bound)
{
    if ((nv->value < lower_bound) || (nv->value >= upper_bound)) {
        return (STAT_INPUT_VALUE_RANGE_ERROR);
    }
    set_ui8(nv);        // will this work in -1 is a valid value?
    if (cm_get_machine_state() != MACHINE_INITIALIZING) {
        inputs_reset();
    }
    return (STAT_OK);
}

static stat_t _output_set_helper(nvObj_t *nv, const int8_t lower_bound, const int8_t upper_bound)
{
    if ((nv->value < lower_bound) || (nv->value >= upper_bound)) {
        return (STAT_INPUT_VALUE_RANGE_ERROR);
    }
    set_ui8(nv);        // will this work in -1 is a valid value?
    if (cm_get_machine_state() != MACHINE_INITIALIZING) {
        outputs_reset();
    }
    return (STAT_OK);
}

stat_t io_set_mo(nvObj_t *nv)            // input type or disabled
{
//    return (_io_set_helper(nv, IO_MODE_DISABLED, IO_MODE_MAX));
    return (_input_set_helper(nv, IO_ACTIVE_LOW, IO_MODE_MAX));
}

stat_t io_set_ac(nvObj_t *nv)            // input action
{
    return (_input_set_helper(nv, INPUT_ACTION_NONE, INPUT_ACTION_MAX));
}

stat_t io_set_fn(nvObj_t *nv)            // input function
{
    return (_input_set_helper(nv, INPUT_FUNCTION_NONE, INPUT_FUNCTION_MAX));
}

/*
 *  io_get_input() - return input state given an nv object
 */
stat_t io_get_input(nvObj_t *nv)
{
    char *num_start = nv->token;
    if (*(nv->group) == 0) {
        // if we don't have a group, then the group name is in the token
        // skip over "in"
        num_start+=2;
    }
    nv->value = d_in[strtol(num_start, NULL, 10)-1].state;

    if (nv->value > 1.1) {
        nv->valuetype = TYPE_NULL;
    } else {
        nv->valuetype = TYPE_BOOL;
    }
    return (STAT_OK);
}

stat_t io_set_domode(nvObj_t *nv)            // output function
{
    char *num_start = nv->token;
    if (*(nv->group) == 0) {
        // if we don't have a group, then the group name is in the token
        // skip over "out"
        num_start+=3;
    }
    // the token has been stripped down to an ASCII digit string - use it as an index
    uint8_t output_num = strtol(num_start, NULL, 10);

    if (output_num > D_OUT_CHANNELS) {
        nv->valuetype = TYPE_NULL;
        return(STAT_NO_GPIO);
    }

    // Force pins that aren't available to be "disabled"
    switch (output_num) {
        case 1:  if (output_1_pin.isNull())  { nv->value = IO_MODE_DISABLED; } break;
        case 2:  if (output_2_pin.isNull())  { nv->value = IO_MODE_DISABLED; } break;
        case 3:  if (output_3_pin.isNull())  { nv->value = IO_MODE_DISABLED; } break;
        case 4:  if (output_4_pin.isNull())  { nv->value = IO_MODE_DISABLED; } break;
        case 5:  if (output_5_pin.isNull())  { nv->value = IO_MODE_DISABLED; } break;
        case 6:  if (output_6_pin.isNull())  { nv->value = IO_MODE_DISABLED; } break;
        case 7:  if (output_7_pin.isNull())  { nv->value = IO_MODE_DISABLED; } break;
        case 8:  if (output_8_pin.isNull())  { nv->value = IO_MODE_DISABLED; } break;
        case 9:  if (output_9_pin.isNull())  { nv->value = IO_MODE_DISABLED; } break;
        case 10: if (output_10_pin.isNull()) { nv->value = IO_MODE_DISABLED; } break;
        case 11: if (output_11_pin.isNull()) { nv->value = IO_MODE_DISABLED; } break;
        case 12: if (output_12_pin.isNull()) { nv->value = IO_MODE_DISABLED; } break;
        case 13: if (output_13_pin.isNull()) { nv->value = IO_MODE_DISABLED; } break;

        default:
            break;
    }

    return (_output_set_helper(nv, IO_ACTIVE_LOW, IO_MODE_MAX));
}

/*
 *  io_get_output() - return output state given an nv object
 */
stat_t io_get_output(nvObj_t *nv)
{
    char *num_start = nv->token;
    if (*(nv->group) == 0) {
        // if we don't have a group, then the group name is in the token
        // skip over "out"
        num_start+=3;
    }
    // the token has been stripped down to an ASCII digit string - use it as an index
    uint8_t output_num = strtol(num_start, NULL, 10);

    if (output_num > D_OUT_CHANNELS) {
        nv->valuetype = TYPE_NULL;
        return(STAT_NO_GPIO);
    }

    ioMode outMode = d_out[output_num-1].mode;
    if (outMode == IO_MODE_DISABLED) {
//        nv->value = 0;
        nv->valuetype = TYPE_NULL;   // reports back as NULL
    } else {
        nv->valuetype = TYPE_FLOAT;
        nv->precision = 2;
        bool invert = (outMode == 0);
        // Note: !! forces a value to boolean 0 or 1
        switch (output_num) {
            case 1:  { nv->value = (float)output_1_pin; } break;
            case 2:  { nv->value = (float)output_2_pin; } break;
            case 3:  { nv->value = (float)output_3_pin; } break;
            case 4:  { nv->value = (float)output_4_pin; } break;
            case 5:  { nv->value = (float)output_5_pin; } break;
            case 6:  { nv->value = (float)output_6_pin; } break;
            case 7:  { nv->value = (float)output_7_pin; } break;
            case 8:  { nv->value = (float)output_8_pin; } break;
            case 9:  { nv->value = (float)output_9_pin; } break;
            case 10: { nv->value = (float)output_10_pin; } break;
            case 11: { nv->value = (float)output_11_pin; } break;
            case 12: { nv->value = (float)output_12_pin; } break;
            case 13: { nv->value = (float)output_13_pin; } break;

            default:
                {
//                  nv->value = 0;              // inactive
                    nv->valuetype = TYPE_NULL;  // reports back as NULL
                }
        }
        if (invert) {
            nv->value = 1.0 - nv->value;
        }
    }
    return (STAT_OK);
}

/*
 *  io_set_output() - return input state given an nv object
 */
stat_t io_set_output(nvObj_t *nv)
{
    char *num_start = nv->token;
    if (*(nv->group) == 0) {
        // if we don't have a group, then the group name is in the token
        // skip over "out"
        num_start+=3;
    }
    // the token has been stripped down to an ASCII digit string - use it as an index
    uint8_t output_num = strtol(num_start, NULL, 10);

    ioMode outMode = d_out[output_num-1].mode;
    if (outMode == IO_MODE_DISABLED) {
        nv->value = 0; // Inactive?
    } else {
        bool invert = (outMode == 0);
        float value = nv->value;
        if (invert) {
            value = 1.0 - value;
        }
        switch (output_num) {
            // Generated with:
            // perl -e 'for($i=1;$i<14;$i++) { print "case ${i}:  { output_${i}_pin = value; } break;\n";}'
            // BEGIN generated
            case 1:  { output_1_pin = value; } break;
            case 2:  { output_2_pin = value; } break;
            case 3:  { output_3_pin = value; } break;
            case 4:  { output_4_pin = value; } break;
            case 5:  { output_5_pin = value; } break;
            case 6:  { output_6_pin = value; } break;
            case 7:  { output_7_pin = value; } break;
            case 8:  { output_8_pin = value; } break;
            case 9:  { output_9_pin = value; } break;
            case 10:  { output_10_pin = value; } break;
            case 11:  { output_11_pin = value; } break;
            case 12:  { output_12_pin = value; } break;
            case 13:  { output_13_pin = value; } break;
            // END generated
            default: { nv->value = 0; } // inactive
        }
    }
    return (STAT_OK);
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
    void io_print_mo(nvObj_t *nv) {_print_di(nv, fmt_gpio_mo);}
    void io_print_ac(nvObj_t *nv) {_print_di(nv, fmt_gpio_ac);}
    void io_print_fn(nvObj_t *nv) {_print_di(nv, fmt_gpio_fn);}
    void io_print_in(nvObj_t *nv) {
        sprintf(cs.out_buf, fmt_gpio_in, nv->token, (int)nv->value);
        xio_writeline(cs.out_buf);
    }

    void io_print_domode(nvObj_t *nv) {_print_di(nv, fmt_gpio_domode);}
    void io_print_out(nvObj_t *nv) {
        sprintf(cs.out_buf, fmt_gpio_out, nv->token, (int)nv->value);
        xio_writeline(cs.out_buf);
    }
#endif
