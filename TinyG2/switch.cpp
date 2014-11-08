/*
 * switch.cpp - switch handling functions
 * This file is part of the TinyG project
 *
 * Copyright (c) 2013 - 2014 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2014 Robert Giseburt
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
#include "settings.h"
#include "switch.h"
#include "hardware.h"
#include "canonical_machine.h"
#include "text_parser.h"

#ifdef __AVR
#include <avr/interrupt.h>
#else
#include "MotateTimers.h"
using Motate::SysTickTimer;
#endif

// Allocate switch array structure
switches_t sw;

//static void _no_action(switch_t *s);
//static void _led_on(switch_t *s);
//static void _led_off(switch_t *s);
static void _trigger_feedhold(switch_t *s);
static void _trigger_cycle_start(switch_t *s);
static void _trigger_alarm(switch_t *s);

static void _no_action(switch_t *s) { return; }
//static void _led_on(switch_t *s) { IndicatorLed.clear(); }
//static void _led_off(switch_t *s) { IndicatorLed.set(); }


/*
 * switch_init() - initialize homing/limit switches
 * switch_reset() - reset homing/limit switches (no initialization)
 *
 *	This function assumes all Motate pins have been set up and that
 *	SW_PAIRS and SW_POSITIONS is accurate
 *
 *	Note: `type` and `mode` are not initialized as they should be set from configuration
 */

void switch_init(void)
{
	return(switch_reset());
}

void switch_reset(void)
{
	switch_t *s;	// shorthand
	for (uint8_t axis=0; axis<SW_PAIRS; axis++) {
		for (uint8_t position=0; position<SW_POSITIONS; position++) {
			s = &sw.s[axis][position];
			s->state = false;
			s->edge = SW_NO_EDGE;
			s->debounce_ticks = SW_LOCKOUT_TICKS;
			s->debounce_timeout = 0;

			// functions bound to each switch
			s->when_open = _no_action;
			s->when_closed = _no_action;
            if(s->mode & SW_LIMIT_BIT) {
                s->on_leading = _trigger_alarm;
            }
            else
            {
                s->on_leading = _no_action;
            }
			s->on_trailing = _no_action;
		}
	}
	// bind functions to individual switches
	// <none>
	// sw.s[AXIS_X][SW_MIN].when_open = _led_off;
	// sw.s[AXIS_X][SW_MIN].when_closed = _led_on;
}

/*
 * poll_switches() - run a polling cycle on all switches
 */
#ifndef __POCKETNC
stat_t poll_switches()
{
    poll_switch(&sw.s[AXIS_X][SW_MIN], (bool)axis_X_min_pin);
    poll_switch(&sw.s[AXIS_X][SW_MAX], (bool)axis_X_max_pin);
    poll_switch(&sw.s[AXIS_Y][SW_MIN], (bool)axis_Y_min_pin);
    poll_switch(&sw.s[AXIS_Y][SW_MAX], (bool)axis_Y_max_pin);
    poll_switch(&sw.s[AXIS_Z][SW_MIN], (bool)axis_Z_min_pin);
    poll_switch(&sw.s[AXIS_Z][SW_MAX], (bool)axis_Z_max_pin);
#if (HOMING_AXES >= 4)
    poll_switch(&sw.s[AXIS_A][SW_MIN], (bool)axis_A_min_pin);
    poll_switch(&sw.s[AXIS_A][SW_MAX], (bool)axis_A_max_pin);
#endif
#if (HOMING_AXES >= 5)
    poll_switch(&sw.s[AXIS_B][SW_MIN], (bool)axis_B_min_pin);
    poll_switch(&sw.s[AXIS_B][SW_MAX], (bool)axis_B_max_pin);
#endif
#if (HOMING_AXES >= 6)
    poll_switch(&sw.s[AXIS_C][SW_MIN], (bool)axis_C_min_pin);
    poll_switch(&sw.s[AXIS_C][SW_MAX], (bool)axis_C_max_pin);
#endif
    return (STAT_OK);

#else	// __POCKETNC
    // Pocket NC remaps Xmin to Amax and Ymin to Bmax
    stat_t poll_switches()
    {
	poll_switch(&sw.s[AXIS_X][SW_MIN], (bool)axis_X_min_pin);
	poll_switch(&sw.s[AXIS_X][SW_MAX], (bool)axis_X_max_pin);
	poll_switch(&sw.s[AXIS_Y][SW_MIN], (bool)axis_Y_min_pin);
	poll_switch(&sw.s[AXIS_Y][SW_MAX], (bool)axis_Y_max_pin);
	poll_switch(&sw.s[AXIS_Z][SW_MIN], (bool)axis_Z_min_pin);
	poll_switch(&sw.s[AXIS_Z][SW_MAX], (bool)axis_Z_max_pin);
	poll_switch(&sw.s[AXIS_A][SW_MIN], (bool)axis_A_min_pin);
	poll_switch(&sw.s[AXIS_A][SW_MAX], (bool)axis_X_min_pin);
	poll_switch(&sw.s[AXIS_B][SW_MIN], (bool)axis_B_min_pin);
	poll_switch(&sw.s[AXIS_B][SW_MAX], (bool)axis_Y_min_pin);
	return (STAT_OK);
#endif
}

/*
 * poll_switch() - read switch with NO/NC, debouncing and edge detection
 *
 *	Returns true if switch state changed - e.g. leading or falling edge detected.
 *	Assumes pin_value **input** = 1 means open, 0 is closed.
 *	Pin sense is adjusted to mean:
 *
 *	  0 = open for both NO and NC switches
 *	  1 = closed for both NO and NC switches
 *	 -1 = switch disabled
 *
 *	Also sets disabled switches to switch state -1;
 */
int8_t poll_switch(switch_t *s, uint8_t pin_value)
{
	// instant return conditions: switch disabled or in a lockout period
	if (s->mode == SW_MODE_DISABLED) {
		s->state = SW_DISABLED;
		return (false);
	}
	if (s->debounce_timeout > SysTickTimer.getValue()) {
		return (false);
	}
	// return if no change in state
	uint8_t pin_sense_corrected = (pin_value ^ (s->type ^ 1));	// correct for NO or NC mode
	if ( s->state == pin_sense_corrected ) {
		s->edge = SW_NO_EDGE;
		if (s->state == SW_OPEN) {
			s->when_open(s);
		} else {
			s->when_closed(s);
		}
		return (false);
	}
	// the switch legitimately changed state - process edges
	if ((s->state = pin_sense_corrected) == SW_OPEN) {
		s->edge = SW_TRAILING;
		s->on_trailing(s);
	} else {
		s->edge = SW_LEADING;
		s->on_leading(s);
	}
	s->debounce_timeout = (SysTickTimer.getValue() + s->debounce_ticks);
	return (true);
}

static void _trigger_feedhold(switch_t *s)
{
    //	IndicatorLed.toggle();
	cm_request_feedhold();
/*
	if (cm.cycle_state == CYCLE_HOMING) {		// regardless of switch type
		cm_request_feedhold();
	} else if (s->mode & SW_LIMIT_BIT) {		// set flag if it's a limit switch
		cm.limit_tripped_flag = true;
	}
*/
}

static void _trigger_cycle_start(switch_t *s)
{
//	IndicatorLed.toggle();
	cm_request_end_hold();
}

static void _trigger_alarm(switch_t *s)
{
    s->limit_switch_thrown = true;
}

uint8_t get_limit_switch_thrown(void)
{
    for (uint8_t axis=0; axis<SW_PAIRS; axis++) {
		for (uint8_t position=0; position<SW_POSITIONS; position++) {
			if(sw.s[axis][position].limit_switch_thrown) {
                return true;
            }
        }
    }
    return false;
}

void reset_limit_switches(void)
{
    for (uint8_t axis=0; axis<SW_PAIRS; axis++) {
		for (uint8_t position=0; position<SW_POSITIONS; position++) {
			sw.s[axis][position].limit_switch_thrown = false;
        }
    }
}

/*
 * get_switch_mode() - return switch mode setting
 * get_switch_type() - return switch type setting
 */

uint8_t get_switch_mode(uint8_t axis, uint8_t position)
{
	return (sw.s[axis][position].mode);
}

uint8_t get_switch_type(uint8_t axis, uint8_t position)
{
	return (sw.s[axis][position].type);
}

/*
 * read_switch() - read switch state from the switch structure
 *				   NOTE: This does NOT read the pin itself. See poll_switch
 */
int8_t read_switch(uint8_t axis, uint8_t position)
{
//	if (axis >= AXES) return (SW_DISABLED);
//	if (axis > SW_MAX) return (SW_DISABLED);
	return (sw.s[axis][position].state);
}

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 * These functions are not part of the NIST defined functions
 ***********************************************************************************/

stat_t sw_set_st(nvObj_t *nv)			// switch type (global)
{
	set_01(nv);
	switch_reset();
	return (STAT_OK);
}

stat_t sw_set_sw(nvObj_t *nv)			// switch setting
{
	if (nv->value > SW_MODE_MAX_VALUE) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	set_ui8(nv);
	switch_reset();
	return (STAT_OK);
}

/*
 *  sw_get_ss() - get switch state
 *
 *	Switches map to:
 *	  0 = Xmin, 1= Xmax
 *	  2 = Ymin, 3= Ymax
 *	  4 = Zmin, 5= Zmax
 *	  6 = Amin, 7= Amax
 */

stat_t sw_get_ss(nvObj_t *nv)			// switch number (0-7)
{
	if (nv->value >= (SW_PAIRS * SW_POSITIONS)) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	uint8_t number = ((uint8_t)nv->token[0] & 0x0F);	// change from ASCII to a number 0-9 (A-F, too)
	nv->value = (float) read_switch( number/2, number&0x01 );
	nv->valuetype = TYPE_FLOAT;
	return (STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

	static const char fmt_st[] PROGMEM = "[st]  switch type%18.0f [0=NO,1=NC]\n";
	void sw_print_st(nvObj_t *nv) { text_print_flt(nv, fmt_st);}

	static const char fmt_ss[] PROGMEM = "Switch ss%s state:     %1.0f\n";
	void sw_print_ss(nvObj_t *nv) { fprintf(stderr, fmt_ss, nv->token, nv->value);}

/*
	static const char msg_sw0[] PROGMEM = "Disabled";
	static const char msg_sw1[] PROGMEM = "NO homing";
	static const char msg_sw2[] PROGMEM = "NO homing & limit";
	static const char msg_sw3[] PROGMEM = "NC homing";
	static const char msg_sw4[] PROGMEM = "NC homing & limit";
	static const char *const msg_sw[] PROGMEM = { msg_sw0, msg_sw1, msg_sw2, msg_sw3, msg_sw4 };
*/

#endif
