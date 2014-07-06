/*
 * switch.h - switch handling functions
 * This file is part of the TinyG project
 *
 * Copyright (c) 2013 - 2014 Alden S. Hart, Jr.
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
/* Switch processing functions under Motate
 *
 *	Switch processing turns pin transitions into reliable switch states.
 *	There are 2 main operations:
 *
 *	  - read pin		get raw data from a pin
 *	  - read switch		return processed switch closures
 *
 *	Read pin may be a polled operation or an interrupt on pin change. If interrupts
 *	are used they must be provided for both leading and trailing edge transitions.
 *
 *	Read switch contains the results of read pin and manages edges and debouncing.
 */
#ifndef SWITCH_H_ONCE
#define SWITCH_H_ONCE

/*
 * Generic variables and settings
 */

// switch array configuration / sizing
#define SW_PAIRS				HOMING_AXES	// number of axes that can have switches
#define SW_POSITIONS			2			// swPosition is either SW_MIN or SW)MAX

// switch modes
#define SW_HOMING_BIT			0x01
#define SW_LIMIT_BIT			0x02

#define SW_MODE_DISABLED 		0							 // disabled for all operations
#define SW_MODE_HOMING 			SW_HOMING_BIT				 // enable switch for homing only
#define SW_MODE_LIMIT 			SW_LIMIT_BIT				 // enable switch for limits only
#define SW_MODE_HOMING_LIMIT   (SW_HOMING_BIT | SW_LIMIT_BIT)// homing and limits
#define SW_MODE_MAX_VALUE 		SW_MODE_HOMING_LIMIT

enum swType {
	SW_TYPE_NORMALLY_OPEN = 0,
	SW_TYPE_NORMALLY_CLOSED
};

enum swState {
	SW_DISABLED = -1,
	SW_OPEN = 0,					// also read as 'false'
	SW_CLOSED = 1					// also read as 'true'
};

enum swPosition {
	SW_MIN = 0,
	SW_MAX
};

enum swEdge {
	SW_NO_EDGE = 0,
	SW_LEADING,
	SW_TRAILING,
};

#define SW_LOCKOUT_TICKS 50			// milliseconds to go dead after switch firing

// DEPRECATED STUFF FROM OLD SWITCH CODE
//#define SW_DEGLITCH_TICKS 30		// in milliseconds

// macros for finding the index into the switch table give the axis number
//#define MIN_SWITCH(axis) (axis*2)
//#define MAX_SWITCH(axis) (axis*2+1)

/*
enum swNums {	 					// indexes into switch arrays
	SW_MIN_X = 0,
	SW_MAX_X,
	SW_MIN_Y,
	SW_MAX_Y,
	SW_MIN_Z,
	SW_MAX_Z,
	SW_MIN_A,
	SW_MAX_A,
	NUM_SWITCHES 			// must be last one. Used for array sizing and for loops
};
#define SW_OFFSET SW_MAX_X	// offset between MIN and MAX switches
#define NUM_SWITCH_PAIRS (NUM_SWITCHES/2)
*/

/*
 * Switch control structures
 */
typedef struct swSwitch {						// one struct per switch
	// public
	uint8_t type;								// swType: 0=NO, 1=NC
	uint8_t mode;								// 0=disabled, 1=homing, 2=limit, 3=homing+limit
	uint8_t state;								// set true if switch is closed

	// private
	uint8_t edge;								// keeps a transient record of edges for immediate inquiry
	uint16_t debounce_ticks;					// number of millisecond ticks for debounce lockout
	uint32_t debounce_timeout;					// time to expire current debounce lockout, or 0 if no lockout
	void (*when_open)(struct swSwitch *s);		// callback to action function when sw is open - passes *s, returns void
	void (*when_closed)(struct swSwitch *s);	// callback to action function when closed
	void (*on_leading)(struct swSwitch *s);		// callback to action function for leading edge onset
	void (*on_trailing)(struct swSwitch *s);	// callback to action function for trailing edge
} switch_t;
typedef void (*sw_callback)(switch_t *s);		// typedef for switch action callback

typedef struct swSwitchArray {					// array of switches
	uint8_t type;								// switch type for entire array (default)
	switch_t s[SW_PAIRS][SW_POSITIONS];
} switches_t;
extern switches_t sw;

/*
 * Function prototypes
 */
void switch_init(void);
stat_t poll_switches(void);
uint8_t poll_switch(switch_t *s, uint8_t pin_value);
uint8_t get_switch_mode(uint8_t axis, uint8_t position);
uint8_t get_switch_type(uint8_t axis, uint8_t position);
uint8_t read_switch(uint8_t axis, uint8_t position);

/*
 * Switch config accessors and text functions
 */
stat_t sw_set_st(nvObj_t *nv);
stat_t sw_set_sw(nvObj_t *nv);

#ifdef __TEXT_MODE
	void sw_print_st(nvObj_t *nv);
#else
	#define sw_print_st tx_print_stub
#endif // __TEXT_MODE

#endif // End of include guard: SWITCH_H_ONCE
