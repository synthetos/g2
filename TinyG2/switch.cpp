/*
 * switch.cpp - switch handling functions
 * This file is part of the TinyG project
 *
 * Copyright (c) 2013 Alden S. Hart Jr.
 * Copyright (c) 2013 Robert Giseburt
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
/* Switch Modes
 *
 *	The switches are considered to be homing switches when machine_state is
 *	MACHINE_HOMING. At all other times they are treated as limit switches:
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
#include "switch.h"
#include "hardware.h"

// Allocate switch array structure
switches_t sw;

/*
 * switch_init() - initialize homing/limit switches
 *
 *	This function assumes all Motate pins have been set up and that 
 *	SW_PAIRS and SW_POSITIONS is accurate
 *
 *	Note: `type` and `mode` are not initialized as they should be set from configuration
 */

void switch_init(void)
{
//	sw.type = SW_NORMALLY_OPEN;						// set from config
	sw.edge_flag = 0;
	sw.edge_pair = 0;
	sw.edge_position = 0;

	for (uint8_t i=0; i<SW_PAIRS; i++) {
		for (uint8_t j=0; j<SW_POSITIONS; j++) {
			sw.s[i][j].type = sw.type;				// propagate type from global type
//			sw.s[i][j].mode = SW_MODE_DISABLED;		// set from config			
			sw.s[i][j].state = false;
			sw.s[i][j].debounce_ticks = SW_LOCKOUT_TICKS;
			sw.s[i][j].debounce_timeout = 0;
		}		
	}
//	sw_reset_switches();
}

/*
 * read_switch() - read switch with NO/NC, debouncing and edge detection
 */
uint8_t read_switch(switch_t *s, uint8_t pin_value, uint8_t *edge)
{
	*edge = false;										// initial setting for edge flag
	uint8_t pin_sense_corrected = (pin_value ^ s->type);// correct for NO or NC mode

	// return false if switch is not enabled
	if (s->mode == SW_MODE_DISABLED) { return (false);}

	// no change in pin value
  	if (pin_sense_corrected == s->state) { return (s->state);}

	// switch is in debounce lockout interval
	if ((s->debounce_timeout != 0) && (s->debounce_timeout < GetTickCount())) { return (s->state);} 

	*edge = true;
	s->state = pin_sense_corrected;
	s->debounce_timeout = (GetTickCount() + s->debounce_ticks);
	return (s->state);
}

uint8_t read_switches()
{
	uint8_t edge;
	uint8_t state;
	
	state = read_switch(&sw.s[AXIS_X][SW_MIN], axis_X_min_pin, &edge);	
	return (false);
}	




/*
 * Switch closure processing routines
 *
 * ISRs 				- switch interrupt handler vectors
 * _isr_helper()		- common code for all switch ISRs
 * switch_rtc_callback()	- called from RTC for each RTC tick.
 *
 *	These functions interact with each other to process switch closures and firing.
 *	Each switch has a counter which is initially set to negative SW_DEGLITCH_TICKS.
 *	When a switch closure is DETECTED the count increments for each RTC tick.
 *	When the count reaches zero the switch is tripped and action occurs.
 *	The counter continues to increment positive until the lockout is exceeded.
 */
/*
ISR(X_MIN_ISR_vect)	{ _isr_helper(SW_MIN_X);}
ISR(Y_MIN_ISR_vect)	{ _isr_helper(SW_MIN_Y);}
ISR(Z_MIN_ISR_vect)	{ _isr_helper(SW_MIN_Z);}
ISR(A_MIN_ISR_vect)	{ _isr_helper(SW_MIN_A);}
ISR(X_MAX_ISR_vect)	{ _isr_helper(SW_MAX_X);}
ISR(Y_MAX_ISR_vect)	{ _isr_helper(SW_MAX_Y);}
ISR(Z_MAX_ISR_vect)	{ _isr_helper(SW_MAX_Z);}
ISR(A_MAX_ISR_vect)	{ _isr_helper(SW_MAX_A);}
*/
/*
static void _isr_helper(uint8_t sw_num)
{
	if (sw.mode[sw_num] == SW_MODE_DISABLED) return;	// this is never supposed to happen
	if (sw.state[sw_num] == SW_LOCKOUT) return;			// exit if switch is in lockout
	sw.state[sw_num] = SW_DEGLITCHING;					// either transitions state from IDLE or overwrites it
	sw.count[sw_num] = -SW_DEGLITCH_TICKS;				// reset deglitch count regardless of entry state
}
*/
void switch_rtc_callback(void)
{
/*
	for (uint8_t i=0; i < NUM_SWITCHES; i++) { 
		if (sw.state[i] == SW_IDLE) continue;
		if (++sw.count[i] == SW_LOCKOUT_TICKS) {		// state is either lockout or deglitching
			sw.state[i] = SW_IDLE; continue;
		}
		if (sw.count[i] == 0) {							// trigger point
			sw.sw_num_thrown = i;						// record number of thrown switch
			sw.state[i] = SW_LOCKOUT;
//			sw_show_switch();							// only called if __DEBUG enabled
			if (cm.cycle_state == CYCLE_HOMING) {		// regardless of switch type
				sig_feedhold();
			} else if (sw.mode[i] & SW_LIMIT) {			// should be a limit switch, so fire it.
				sw.limit_flag = true;					// triggers an emergency shutdown
			}
		}
	}
*/
}

/*
 * switch_get_switch_mode() 	- return switch mode setting
 * switch_get_limit_thrown()  - return true if a limit was tripped
 * switch_get_sw_num()  		- return switch number most recently thrown
 */

uint8_t get_switch_mode(uint8_t sw_num) { return (0);}	// ++++
//uint8_t switch_get_switch_mode(uint8_t sw_num) { return (sw.mode[sw_num]);}
//uint8_t switch_get_limit_thrown(void) { return(sw.limit_flag);}
//uint8_t switch_get_sw_thrown(void) { return(sw.sw_num_thrown);}

/*
 * switch_reset_switches() - reset all switches and reset limit flag
 */

void switch_reset_switches() 
{
/*
	for (uint8_t i=0; i < NUM_SWITCHES; i++) {
		sw.state[i] = SW_IDLE;
//		sw.count[i] = -SW_DEGLITCH_TICKS;
	}
	sw.limit_flag = false;
*/
}

/*
 * switch_read_switch() - read a switch directly with no interrupts or deglitching
 */
uint8_t switch_read_switch(uint8_t sw_num)
{
/*
	if ((sw_num < 0) || (sw_num >= NUM_SWITCHES)) return (SW_DISABLED);

	uint8_t read = 0;
	switch (sw_num) {
		case SW_MIN_X: { read = device.sw_port[X]->IN & SW_MIN_BIT_bm; break;}
		case SW_MAX_X: { read = device.sw_port[X]->IN & SW_MAX_BIT_bm; break;}
		case SW_MIN_Y: { read = device.sw_port[Y]->IN & SW_MIN_BIT_bm; break;}
		case SW_MAX_Y: { read = device.sw_port[Y]->IN & SW_MAX_BIT_bm; break;}
		case SW_MIN_Z: { read = device.sw_port[Z]->IN & SW_MIN_BIT_bm; break;}
		case SW_MAX_Z: { read = device.sw_port[Z]->IN & SW_MAX_BIT_bm; break;}
		case SW_MIN_A: { read = device.sw_port[A]->IN & SW_MIN_BIT_bm; break;}
		case SW_MAX_A: { read = device.sw_port[A]->IN & SW_MAX_BIT_bm; break;}
	}
	if (sw.switch_type == SW_TYPE_NORMALLY_OPEN) {
		return ((read == 0) ? SW_CLOSED : SW_OPEN);		// confusing. An NO switch drives the pin LO when thrown
	} else {
		return ((read != 0) ? SW_CLOSED : SW_OPEN);
	}
*/
	return (0);
}

/*
 * switch_led_on() - turn led on - assumes TinyG LED mapping
 * switch_led_off() - turn led on - assumes TinyG LED mapping
 * switch_led_toggle()
 */

void switch_led_on(uint8_t led)
{
//	if (led == 0) return (switch_set_bit_on(0x08));
//	if (led == 1) return (switch_set_bit_on(0x04));
//	if (led == 2) return (switch_set_bit_on(0x02));
//	if (led == 3) return (switch_set_bit_on(0x01));
/*
	if (led == 0) switch_set_bit_on(0x08); else 
	if (led == 1) switch_set_bit_on(0x04); else 
	if (led == 2) switch_set_bit_on(0x02); else 
	if (led == 3) switch_set_bit_on(0x01);
*/
}

void switch_led_off(uint8_t led)
{
//	if (led == 0) return (switch_set_bit_off(0x08));
//	if (led == 1) return (switch_set_bit_off(0x04));
//	if (led == 2) return (switch_set_bit_off(0x02));
//	if (led == 3) return (switch_set_bit_off(0x01));
/*
	if (led == 0) switch_set_bit_off(0x08); else 
	if (led == 1) switch_set_bit_off(0x04); else 
	if (led == 2) switch_set_bit_off(0x02); else 
	if (led == 3) switch_set_bit_off(0x01);
*/
}

void switch_led_toggle(uint8_t led)
{
/*
	if (led == 0) {
		if (switch_read_bit(0x08)) {
			switch_set_bit_off(0x08);
		} else {
			switch_set_bit_on(0x08);
		}
	} else if (led == 1) {
		if (switch_read_bit(0x04)) {
			switch_set_bit_off(0x04);
		} else {
			switch_set_bit_on(0x04);
		}
	} else if (led == 2) {
		if (switch_read_bit(0x02)) {
			switch_set_bit_off(0x02);
		} else {
			switch_set_bit_on(0x02);
		}
	} else if (led == 3) {
		if (switch_read_bit(0x08)) {
			switch_set_bit_off(0x08);
		} else {
			switch_set_bit_on(0x08);
		}
	}
*/
}

/*
 * switch_read_bit() - return true if bit is on, false if off
 * switch_set_bit_on() - turn bit on
 * switch_set_bit_off() - turn bit on
 *
 *	These functions have an inner remap depending on what hardware is running
 */

uint8_t switch_read_bit(uint8_t b)
{
/*
	if (b & 0x08) { return (device.out_port[0]->IN & GPIO1_OUT_BIT_bm); }
	if (b & 0x04) { return (device.out_port[1]->IN & GPIO1_OUT_BIT_bm); }
	if (b & 0x02) { return (device.out_port[2]->IN & GPIO1_OUT_BIT_bm); }
	if (b & 0x01) { return (device.out_port[3]->IN & GPIO1_OUT_BIT_bm); }
*/
	return (0);
}

void switch_set_bit_on(uint8_t b)
{
/*
	if (b & 0x08) { device.out_port[0]->OUTSET = GPIO1_OUT_BIT_bm; }
	if (b & 0x04) { device.out_port[1]->OUTSET = GPIO1_OUT_BIT_bm; }
	if (b & 0x02) { device.out_port[2]->OUTSET = GPIO1_OUT_BIT_bm; }
	if (b & 0x01) { device.out_port[3]->OUTSET = GPIO1_OUT_BIT_bm; }
*/
}

void switch_set_bit_off(uint8_t b)
{
/*
	if (b & 0x08) { device.out_port[0]->OUTCLR = GPIO1_OUT_BIT_bm; }
	if (b & 0x04) { device.out_port[1]->OUTCLR = GPIO1_OUT_BIT_bm; }
	if (b & 0x02) { device.out_port[2]->OUTCLR = GPIO1_OUT_BIT_bm; }
	if (b & 0x01) { device.out_port[3]->OUTCLR = GPIO1_OUT_BIT_bm; }
*/
}

//###########################################################################
//##### UNIT TESTS ##########################################################
//###########################################################################

#ifdef __UNIT_TESTS
#ifdef __UNIT_TEST_GPIO

void switch_unit_tests()
{
//	_isr_helper(SW_MIN_X, X);
	while (true) {
		switch_led_toggle(1);
	}
}

#endif // __UNIT_TEST_GPIO
#endif
