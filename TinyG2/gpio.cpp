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
#include "hardware.h"
#include "canonical_machine.h"

#ifdef __AVR
#include <avr/interrupt.h>
#else
#include "MotateTimers.h"
using Motate::SysTickTimer;
#endif

// Allocate IO array structures
io_t io;

void static _handle_pin_changed(const uint8_t input_num, const int8_t pin_value);

static InputPin<kXAxis_MinPinNumber> axis_X_min_pin(kPullUp);
static InputPin<kXAxis_MaxPinNumber> axis_X_max_pin(kPullUp);
static InputPin<kYAxis_MinPinNumber> axis_Y_min_pin(kPullUp);
static InputPin<kYAxis_MaxPinNumber> axis_Y_max_pin(kPullUp);
static InputPin<kZAxis_MinPinNumber> axis_Z_min_pin(kPullUp);
static InputPin<kZAxis_MaxPinNumber> axis_Z_max_pin(kPullUp);
static InputPin<kAAxis_MinPinNumber> axis_A_min_pin(kPullUp);
static InputPin<kAAxis_MaxPinNumber> axis_A_max_pin(kPullUp);
static InputPin<kBAxis_MinPinNumber> axis_B_min_pin(kPullUp);
static InputPin<kBAxis_MaxPinNumber> axis_B_max_pin(kPullUp);
static InputPin<kCAxis_MinPinNumber> axis_C_min_pin(kPullUp);
static InputPin<kCAxis_MaxPinNumber> axis_C_max_pin(kPullUp);

/*
 * gpio_init() - initialize inputs and outputs
 * gpio_reset() - reset inputs and outputs (no initialization)
 */

void gpio_init(void)
{
    axis_X_min_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    axis_X_max_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    axis_Y_min_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    axis_Y_max_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    axis_Z_min_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    axis_Z_max_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    axis_A_min_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    axis_A_max_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
/*
    axis_B_min_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    axis_B_max_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    axis_C_min_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
    axis_C_max_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
*/
	return(gpio_reset());
}

void gpio_reset(void)
{
	for (uint8_t i=0; i<DI_CHANNELS; i++) {
		io.in[i].state = IO_INACTIVE;
        io.in[i].lockout_ms = IO_LOCKOUT_MS;
		io.in[i].lockout_timer = SysTickTimer.getValue();
	}
}

/*
 * pin change ISRs - ISR entry point for input pin changes
 *
 * NOTE: InpuTPin<>.get() returns a uint32_t, and will NOT necessarily be 1 for true.
 * The actual values will be the pin's port mask or 0, so you must check for non-zero.
 */

MOTATE_PIN_INTERRUPT(kXAxis_MinPinNumber) { _handle_pin_changed(1, (axis_X_min_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kXAxis_MaxPinNumber) { _handle_pin_changed(2, (axis_X_max_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kYAxis_MinPinNumber) { _handle_pin_changed(3, (axis_Y_min_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kYAxis_MaxPinNumber) { _handle_pin_changed(4, (axis_Y_max_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kZAxis_MinPinNumber) { _handle_pin_changed(5, (axis_Z_min_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kZAxis_MaxPinNumber) { _handle_pin_changed(6, (axis_Z_max_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kAAxis_MinPinNumber) { _handle_pin_changed(7, (axis_A_min_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kAAxis_MaxPinNumber) { _handle_pin_changed(8, (axis_A_max_pin.get() != 0)); }
/*
MOTATE_PIN_INTERRUPT(kBAxis_MinPinNumber) { _handle_pin_changed(9, (axis_B_min_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kBAxis_MaxPinNumber) { _handle_pin_changed(9, (axis_B_max_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kCAxis_MinPinNumber) { _handle_pin_changed(10, (axis_C_min_pin.get() != 0)); }
MOTATE_PIN_INTERRUPT(kCAxis_MaxPinNumber) { _handle_pin_changed(11, (axis_C_max_pin.get() != 0)); }
*/

/*
 * _handle_pin_changed() - ISR helper
 *
 * Since we set the interrupt to kPinInterruptOnChange _handle_pin_changed() should
 * only be called when the pin *changes* values, so we can assume that the current
 * pin value is not the same as the previous value. Note that The value may have
 * changed rapidly, and may even have changed again since the interrupt was triggered.
 * In this case a second interrupt will likely follow this one immediately after exiting.
 *
 *  input_num is the input channel, 1 - N
 *  pin_value = 1 if pin is set, 0 otherwise
 */

void static _handle_pin_changed(const uint8_t input_num, const int8_t pin_value)
{
    io_di_t *in = &io.in[input_num-1];  // array index is one less than input number

    printf("input num %d\n", input_num);

    // return if input is disabled (not supposed to happen)
	if (in->mode == IO_MODE_DISABLED) {
    	in->state = IO_DISABLED;
        return;
    }

    // return if the input is in lockout period (take no action)
    if (SysTickTimer.getValue() < in->lockout_timer) {
        return;
    }

	// return if no change in state
	int8_t pin_value_corrected = (pin_value ^ (in->mode ^ 1));	// correct for NO or NC mode
	if ( in->state == pin_value_corrected ) {
//    	in->edge = IO_EDGE_NONE;        // edge should only be reset by function or opposite edge
    	return;
	}

	// record the changed state
    in->state = pin_value_corrected;
	in->lockout_timer = SysTickTimer.getValue() + in->lockout_ms;
    if (pin_value_corrected == IO_ACTIVE) {
        in->edge = IO_EDGE_LEADING;
    } else {
        in->edge = IO_EDGE_TRAILING;
    }

    // perform homing operations if in homing mode
    if (in->homing_mode) {
		if (in->edge == IO_EDGE_LEADING) {
		//  cm_request_feedhold();
			cm_start_hold();
		}
        return;
    }

	// NOTE: From this point on all conditionals assume we are NOT in homing mode

    // trigger the action on leading edges
    // *** for now all the actions do the same thing ***
    if (in->edge == IO_EDGE_LEADING) {
        if (in->action == IO_ACTION_STOP) {
//			cm_request_feedhold();
			cm_start_hold();
        }
        if (in->action == IO_ACTION_FAST_STOP) {
		//	cm_request_feedhold();
			cm_start_hold();
        }
        if (in->action == IO_ACTION_HALT) {
		//	cm_request_feedhold();
//			cm_start_hold();
	        stepper_init();					// hard stop
        }
        if (in->action == IO_ACTION_RESET) {
		//	cm_request_feedhold();
		//	cm_start_hold();
            hw_hard_reset();
        }
    }

    // trigger interlock function on either leading and trailing edge
	if (in->function == IO_FUNCTION_INTERLOCK) {
		cm.interlock_requested = in->edge;
		return;
	}

	// the remainder of the functions only trigger on the leading edge
    if (in->edge == IO_EDGE_LEADING) {
		if (in->function == IO_FUNCTION_LIMIT) {
			cm.limit_requested = input_num;

		} else if (in->function == IO_FUNCTION_SHUTDOWN) {
			cm.shutdown_requested = input_num;
		}
    }
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
//	return (_io_set_helper(nv, IO_DISABLED, IO_MODE_MAX));
	if ((nv->value < IO_DISABLED) || (nv->value >= IO_MODE_MAX)) {
		return (STAT_INPUT_VALUE_UNSUPPORTED);
	}
	set_int8(nv);
	gpio_reset();
	return (STAT_OK);
}

stat_t io_set_ac(nvObj_t *nv)			// input action
{
	return (_io_set_helper(nv, IO_ACTION_NONE, IO_ACTION_MAX));
//	if ((nv->value < IO_ACTION_NONE) || (nv->value >= IO_ACTION_MAX)) {
}

stat_t io_set_fn(nvObj_t *nv)			// input function
{
	return (_io_set_helper(nv, IO_FUNCTION_NONE, IO_FUNCTION_MAX));
//	if ((nv->value < IO_FUNCTION_NONE) || (nv->value >= IO_FUNCTION_MAX)) {
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
	static const char fmt_gpio_mo[] PROGMEM = "[%smo] input mode%15.0f [-1=disabled, 0=NO,1=NC]\n";
	static const char fmt_gpio_ac[] PROGMEM = "[%sac] input action%13.0f [0=none,1=stop,2=halt,3=stop_steps,4=reset]\n";
	static const char fmt_gpio_fn[] PROGMEM = "[%sfn] input function%11.0f [0=none,1=limit,2=interlock,3=shutdown]\n";
	static const char fmt_gpio_in[] PROGMEM = "Input %s state: %5.0f\n";

	void io_print_mo(nvObj_t *nv)
	{
		fprintf(stderr, fmt_gpio_mo, nv->group, nv->value);
	}

	void io_print_ac(nvObj_t *nv)
	{
		fprintf(stderr, fmt_gpio_ac, nv->group, nv->value);
	}

	void io_print_fn(nvObj_t *nv)
	{
		fprintf(stderr, fmt_gpio_fn, nv->group, nv->value);
	}

	void io_print_in(nvObj_t *nv)
	{
    	fprintf(stderr, fmt_gpio_in, nv->token, nv->value);
	}
#endif
