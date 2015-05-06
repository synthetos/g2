/*
 * gpio.cpp - digital IO handling functions
 * This file is part of the TinyG project
 *
 * Copyright (c) 2015 Alden S. Hart, Jr.
 * Copyright (c) 2015 Robert Giseburt
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
 *	The switches are considered to be homing switches when cycle_state is
 *	CYCLE_HOMING. At all other times they are treated as limit switches:
 *	  - Hitting a homing switch puts the current move into feedhold
 *	  - Hitting a limit switch causes the machine to shut down and go into lockdown until reset
 *
 * 	The normally open switch modes (NO) trigger an interrupt on the falling edge
 *	and lockout subsequent interrupts for the defined lockout period. This approach
 *	beats doing debouncing as an integration as switches fire immediately.
 *
 * 	The normally closed switch modes (NC) trigger an interrupt on the rising edge
 *	and lockout subsequent interrupts for the defined lockout period. Ditto on the method.
 */

#include "tinyg2.h"
#include "config.h"
#include "gpio.h"
#include "stepper.h"
#include "encoder.h"
#include "hardware.h"
#include "canonical_machine.h"
#include "report.h"

#ifdef __AVR
#include <avr/interrupt.h>
#else
#include "MotateTimers.h"
using Motate::SysTickTimer;
#endif

// Allocate IO array structures
io_t io;

void static _handle_pin_changed(const uint8_t input_num, const int8_t pin_value);

static InputPin<kInput1_PinNumber> input_1_pin(kPullUp);
static InputPin<kInput2_PinNumber> input_2_pin(kPullUp);
static InputPin<kInput3_PinNumber> input_3_pin(kPullUp);
static InputPin<kInput4_PinNumber> input_4_pin(kPullUp);
static InputPin<kInput5_PinNumber> input_5_pin(kPullUp);
static InputPin<kInput6_PinNumber> input_6_pin(kPullUp);
static InputPin<kInput7_PinNumber> input_7_pin(kPullUp);
static InputPin<kInput8_PinNumber> input_8_pin(kPullUp);
static InputPin<kInput9_PinNumber> input_9_pin(kPullUp);
static InputPin<kInput10_PinNumber> input_10_pin(kPullUp);
static InputPin<kInput11_PinNumber> input_11_pin(kPullUp);
static InputPin<kInput12_PinNumber> input_12_pin(kPullUp);

// WARNING: this returns raw pin values, NOT corrected for NO/NC Active high/low
// Also, this takes EXTERNAL pin numbers -- 1-based
bool _read_input_pin(const uint8_t input_num_ext) {
    switch(input_num_ext) {
        case 1: { return (input_1_pin.get() != 0); }
        case 2: { return (input_2_pin.get() != 0); }
        case 3: { return (input_3_pin.get() != 0); }
        case 4: { return (input_4_pin.get() != 0); }
        case 5: { return (input_5_pin.get() != 0); }
        case 6: { return (input_6_pin.get() != 0); }
        case 7: { return (input_7_pin.get() != 0); }
        case 8: { return (input_8_pin.get() != 0); }
        case 9: { return (input_9_pin.get() != 0); }
        case 10: { return (input_10_pin.get() != 0); }
        case 11: { return (input_11_pin.get() != 0); }
        case 12: { return (input_12_pin.get() != 0); }
        default: { return false; } // ERROR?
    }
}

/*
 * gpio_init() - initialize inputs and outputs
 * gpio_reset() - reset inputs and outputs (no initialization)
 */

void gpio_init(void)
{
    /* Priority only needs set once in the system during startup.
     * However, if we wish to switch the interrupt trigger, here are other options:
     *  kPinInterruptOnRisingEdge
     *  kPinInterruptOnFallingEdge
     *
     * To change the trigger, just call pin.setInterrupts(value) at any point.
     *
     * Note that it may cause an interrupt to fire *immediately*!
     *
     */

    input_1_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    input_2_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    input_3_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    input_4_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    input_5_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    input_6_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    input_7_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    input_8_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
/*
    input_9_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    input_10_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    input_11_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    input_12_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
*/
	return(gpio_reset());
}

void gpio_reset(void)
{
	for (uint8_t i=0; i<DI_CHANNELS; i++) {
        if (io.in[i].mode == INPUT_MODE_DISABLED) {
            io.in[i].state = INPUT_DISABLED;
            continue;
        }
        int8_t pin_value_corrected = (_read_input_pin(i+1) ^ (io.in[i].mode ^ 1));	// correct for NO or NC mode
		io.in[i].state = pin_value_corrected;
        io.in[i].lockout_ms = INPUT_LOCKOUT_MS;
		io.in[i].lockout_timer = SysTickTimer.getValue();
	}
}

/*
 * gpio_set_homing_mode()   - set/clear input to homing mode
 * gpio_set_probing_mode()  - set/clear input to probing mode
 * gpio_read_input()        - read conditioned input
 *
 (* Note: input_num_ext means EXTERNAL input number -- 1-based
 */
void  gpio_set_homing_mode(const uint8_t input_num_ext, const bool is_homing)
{
    if (input_num_ext == 0) {
        return;
    }
    io.in[input_num_ext-1].homing_mode = is_homing;
}

void  gpio_set_probing_mode(const uint8_t input_num_ext, const bool is_probing)
{
    if (input_num_ext == 0) {
        return;
    }
    io.in[input_num_ext-1].probing_mode = is_probing;
}

bool gpio_read_input(const uint8_t input_num_ext)
{
    if (input_num_ext == 0) {
        return false;
    }
    return (io.in[input_num_ext-1].state);
}

/*
 * pin change ISRs - ISR entry point for input pin changes
 *
 * NOTE: InputPin<>.get() returns a uint32_t, and will NOT necessarily be 1 for true.
 * The actual values will be the pin's port mask or 0, so you must check for non-zero.
 */

MOTATE_PIN_INTERRUPT(kInput1_PinNumber) { _handle_pin_changed(1, (input_1_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kInput2_PinNumber) { _handle_pin_changed(2, (input_2_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kInput3_PinNumber) { _handle_pin_changed(3, (input_3_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kInput4_PinNumber) { _handle_pin_changed(4, (input_4_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kInput5_PinNumber) { _handle_pin_changed(5, (input_5_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kInput6_PinNumber) { _handle_pin_changed(6, (input_6_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kInput7_PinNumber) { _handle_pin_changed(7, (input_7_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kInput8_PinNumber) { _handle_pin_changed(8, (input_8_pin.get() != 0)); }
/*
MOTATE_PIN_INTERRUPT(kInput9_PinNumber) { _handle_pin_changed(9, (input_9_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kInput10_PinNumber) { _handle_pin_changed(9, (input_10_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kInput11_PinNumber) { _handle_pin_changed(10, (input_11_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kInput12_PinNumber) { _handle_pin_changed(11, (input_12_pin.get() != 0)); }
*/

/*
 * _handle_pin_changed() - ISR helper
 *
 * Since we set the interrupt to kPinInterruptOnChange _handle_pin_changed() should
 * only be called when the pin *changes* values, so we can assume that the current
 * pin value is not the same as the previous value. Note that the value may have
 * changed rapidly, and may even have changed again since the interrupt was triggered.
 * In this case a second interrupt will likely follow this one immediately after exiting.
 *
 *  input_num is the input channel, 1 - N
 *  pin_value = 1 if pin is set, 0 otherwise
 */

void static _handle_pin_changed(const uint8_t input_num_ext, const int8_t pin_value)
{
    io_di_t *in = &io.in[input_num_ext-1];  // array index is one less than input number

    // return if input is disabled (not supposed to happen)
	if (in->mode == INPUT_MODE_DISABLED) {
    	in->state = INPUT_DISABLED;
        return;
    }

    // return if the input is in lockout period (take no action)
    if (SysTickTimer.getValue() < in->lockout_timer) {
        return;
    }

	// return if no change in state
	int8_t pin_value_corrected = (pin_value ^ ((int)in->mode ^ 1));	// correct for NO or NC mode
	if ( in->state == pin_value_corrected ) {
//    	in->edge = INPUT_EDGE_NONE;        // edge should only be reset by function or opposite edge
    	return;
	}

	// record the changed state
    in->state = pin_value_corrected;
	in->lockout_timer = SysTickTimer.getValue() + in->lockout_ms;
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
        if (in->edge == INPUT_EDGE_LEADING) {   // we only want the leading edge to fire
            en_take_encoder_snapshot();
            cm_start_hold();
        }
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
	        cm_halt_all();					        // hard stop, including spindle and coolant
        }
        if (in->action == INPUT_ACTION_PANIC) {
	        char msg[10];
	        sprintf_P(msg, PSTR("input %d"), input_num_ext);
	        cm_panic(STAT_PANIC, msg);
        }
        if (in->action == INPUT_ACTION_RESET) {
            hw_hard_reset();
        }
    }

	// these functions trigger on the leading edge
    if (in->edge == INPUT_EDGE_LEADING) {
		if (in->function == INPUT_FUNCTION_LIMIT) {
			cm.limit_requested = input_num_ext;

		} else if (in->function == INPUT_FUNCTION_SHUTDOWN) {
			cm.shutdown_requested = input_num_ext;

		} else if (in->function == INPUT_FUNCTION_INTERLOCK) {
		    cm.safety_interlock_disengaged = input_num_ext;
		}
    }

    // trigger interlock release on trailing edge
    if (in->edge == INPUT_EDGE_TRAILING) {
        if (in->function == INPUT_FUNCTION_INTERLOCK) {
		    cm.safety_interlock_reengaged = input_num_ext;
        }
    }
    sr_request_status_report(SR_REQUEST_TIMED);
}

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 * These functions are not part of the NIST defined functions
 ***********************************************************************************/

static stat_t _io_set_helper(nvObj_t *nv, const int8_t lower_bound, const int8_t upper_bound)
{
	if ((nv->value < lower_bound) || (nv->value >= upper_bound)) {
		return (STAT_INPUT_VALUE_UNSUPPORTED);
	}
	set_ui8(nv);		// will this work in -1 is a valid value?
	gpio_reset();
	return (STAT_OK);
}

stat_t io_set_mo(nvObj_t *nv)			// input type or disabled
{
	if ((nv->value < INPUT_MODE_DISABLED) || (nv->value >= INPUT_MODE_MAX)) {
		return (STAT_INPUT_VALUE_UNSUPPORTED);
	}
	set_int8(nv);
	gpio_reset();
	return (STAT_OK);
}

stat_t io_set_ac(nvObj_t *nv)			// input action
{
	return (_io_set_helper(nv, INPUT_ACTION_NONE, INPUT_ACTION_MAX));
}

stat_t io_set_fn(nvObj_t *nv)			// input function
{
	return (_io_set_helper(nv, INPUT_FUNCTION_NONE, INPUT_FUNCTION_MAX));
}

/*
 *  io_get_input() - return input state given an nv object
 */
stat_t io_get_input(nvObj_t *nv)
{
    // the token has been stripped down to an ASCII digit string - use it as an index
    nv->value = io.in[strtol(nv->token, NULL, 10)-1].state;
    nv->valuetype = TYPE_INT;
    return (STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

	static const char fmt_gpio_mo[] PROGMEM = "[%smo] input mode%15d [-1=disabled, 0=NO,1=NC]\n";
	static const char fmt_gpio_ac[] PROGMEM = "[%sac] input action%13d [0=none,1=stop,2=halt,3=stop_steps,4=panic,5=reset]\n";
	static const char fmt_gpio_fn[] PROGMEM = "[%sfn] input function%11d [0=none,1=limit,2=interlock,3=shutdown]\n";
	static const char fmt_gpio_in[] PROGMEM = "Input %s state: %5d\n";

    static void _print_di(nvObj_t *nv, const char *format)
    {
        fprintf_P(stderr, format, nv->group, (int)nv->value);
    }
	void io_print_mo(nvObj_t *nv) {_print_di(nv, fmt_gpio_mo);}
	void io_print_ac(nvObj_t *nv) {_print_di(nv, fmt_gpio_ac);}
	void io_print_fn(nvObj_t *nv) {_print_di(nv, fmt_gpio_fn);}
	void io_print_in(nvObj_t *nv) {
        fprintf_P(stderr, fmt_gpio_in, nv->token, (int)nv->value);
    }
#endif
