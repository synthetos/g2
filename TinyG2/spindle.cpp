/*
 * spindle.cpp - canonical machine spindle driver
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
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

#include "tinyg2.h"             // #1 dependency order
#include "config.h"             // #2
#include "canonical_machine.h"  // #3
#include "text_parser.h"        // #4

#include "spindle.h"
//#include "gpio.h"
#include "planner.h"
#include "hardware.h"
#include "pwm.h"
#include "util.h"

/**** Allocate structures ****/

spSpindleSingleton_t spindle;

/**** Static functions ****/

static void _exec_spindle_speed(float *value, float *flag);
static void _exec_spindle_control(float *value, float *flag);
static float _get_spindle_pwm (uint8_t spindle_state);

/*
 * cm_spindle_init()
 */
void cm_spindle_init()
{
	if( pwm.c[PWM_1].frequency < 0 )
		pwm.c[PWM_1].frequency = 0;

    pwm_set_freq(PWM_1, pwm.c[PWM_1].frequency);
    pwm_set_duty(PWM_1, pwm.c[PWM_1].phase_off);
}

/*
 * cm_get_spindle_state()
 *
 * cm_set_spindle_state()
 * cm_set_spindle_pause()
 * cm_set_spindle_speed_parameter()
 */
/*
uint8_t cm_get_spindle_state(GCodeState_t *gcode_state) { return gcode_state->spindle_state;}
uint8_t cm_get_spindle_pause(GCodeState_t *gcode_state) { return gcode_state->spindle_pause;}
void cm_set_spindle_state(GCodeState_t *gcode_state, uint8_t spindle_state) { gcode_state->spindle_state = spindle_state;}
void cm_set_spindle_pause(GCodeState_t *gcode_state, uint8_t spindle_pause) { gcode_state->spindle_pause = spindle_pause;}
void cm_set_spindle_speed_parameter(GCodeState_t *gcode_state, float speed) { gcode_state->spindle_speed = speed;}
*/

uint8_t cm_get_spindle_state() { return spindle.state;}
//uint8_t cm_get_spindle_pause(GCodeState_t *gcode_state) { return spindle.state;}
//void cm_set_spindle_state(GCodeState_t *gcode_state, uint8_t spindle_state) { spindle.state = (spSpindleState)spindle_state;}
//void cm_set_spindle_pause(GCodeState_t *gcode_state, uint8_t spindle_pause) { spindle.pause = (spSpindlePause)spindle_pause;}
//void cm_set_spindle_speed_parameter(GCodeState_t *gcode_state, float speed) { spindle.speed = speed;}

/*
 * cm_set_spindle_speed() - queue the S parameter to the planner buffer
 * _exec_spindle_speed() - spindle speed callback from planner queue
 */

stat_t cm_set_spindle_speed(float speed)
{
//	if (speed > cfg.max_spindle speed) { return (STAT_MAX_SPINDLE_SPEED_EXCEEDED);}

    float value[AXES] = { speed, 0,0,0,0,0 };
    mp_queue_command(_exec_spindle_speed, value, value);
    return (STAT_OK);
}

static void _exec_spindle_speed(float *value, float *flag)
{
    spindle.speed = value[0];
	pwm_set_duty(PWM_1, _get_spindle_pwm(spindle.state) ); // update spindle speed if we're running

/* OMC code
	uint8_t spindle_state = cm.gm.spindle_state & (~SPINDLE_PAUSED);
	bool paused = cm.gm.spindle_state & SPINDLE_PAUSED;
	if(cm.estop_state != 0 || cm.interlock_state != 0 || paused) {

	if (cm.safety_interlock_state != SAFETY_INTERLOCK_ENGAGED || paused) {
		spindle_state = SPINDLE_OFF;
   }
	pwm_set_duty(PWM_1, cm_get_spindle_pwm(spindle_state) ); // update spindle speed if we're running
*/
}

/*
 * cm_spindle_control() - queue the spindle command to the planner buffer. Observe PAUSE
 * cm_spindle_control_immediate() - turn on/off spindle w/o planning
 * _exec_spindle_control() - execute the spindle command (called from planner)
 *
 * This is kind of tricky...  If we're in interlock but still moving around, and we get an
 * M3, we just start a feedhold...  Usually, before we call cm_start_hold we check if there's
 * anything in the buffer to actually process the feedhold. Here, we're just about to add
 * something to the buffer, so we skip the check.
 */

stat_t cm_spindle_control(uint8_t spindle_state)
{
/* OMC code
	if (cm.gm.spindle_state & SPINDLE_PAUSED)
		spindle_state |= SPINDLE_PAUSED;

	if (cm.safety_interlock_state != SAFETY_INTERLOCK_ENGAGED && 
      !(spindle_state & SPINDLE_PAUSED) && spindle_state != SPINDLE_OFF) {
		cm_start_hold();
    }
*/
	float value[AXES] = { (float)spindle_state, 0,0,0,0,0 };
	mp_queue_command(_exec_spindle_control, value, value);
	return(STAT_OK);
}

void cm_spindle_control_immediate(spSpindleState spindle_state)
{
/* OMC code
    spindle_state &= ~SPINDLE_PAUSED;            // remove the pause bit
    if (spindle_state != cm.gm.spindle_state) {   // if it's already there, skip it
        float value[AXES] = { (float)spindle_state, 0,0,0,0,0 };
        _exec_spindle_control(value, value);
    }
*/
    // cancel PAUSE if turning off spindle
    if (spindle_state == SPINDLE_OFF) {
        spindle.pause = SPINDLE_NORMAL;
    }
    
    // turn it off
    float value[AXES] = { (float)spindle_state, 0,0,0,0,0 };
    _exec_spindle_control(value, value);
}

static void _exec_spindle_control(float *value, float *flag)
{
    spindle.state = (spSpindleState)value[0];

#ifdef __AVR
	if (spindle.state == SPINDLE_CW) {
		gpio_set_bit_on(SPINDLE_BIT);
		gpio_set_bit_off(SPINDLE_DIR);
	} else if (spindle.state == SPINDLE_CCW) {
		gpio_set_bit_on(SPINDLE_BIT);
		gpio_set_bit_on(SPINDLE_DIR);
	} else {
		gpio_set_bit_off(SPINDLE_BIT);	// failsafe: any error causes stop
	}
#endif // __AVR
#ifdef __ARM
	if (spindle.state == SPINDLE_CW) {
		spindle_enable_pin.set();
		spindle_dir_pin.clear();
	} else if (spindle.state == SPINDLE_CCW) {
		spindle_enable_pin.set();
		spindle_dir_pin.set();
	} else {
		spindle_enable_pin.clear();	// failsafe: any error causes stop
	}
#endif // __ARM
	pwm_set_duty(PWM_1, _get_spindle_pwm(spindle.state));

/* OMC code
	uint8_t spindle_state = (uint8_t)value[0];
	bool paused = spindle_state & SPINDLE_PAUSED;
	uint8_t raw_spindle_state = spindle_state & (~SPINDLE_PAUSED);

	if(cm.estop_state != 0) { // In E-stop, don't process any spindle commands
		spindle_state = raw_spindle_state = SPINDLE_OFF;
    } else
    // If we're paused or in interlock, send the spindle an "OFF" command (invisible to cm.gm)
    //++++ THE LOGIC HERE MIGHT BE WRONG - safety_interlock_requested versus safety_interlock_state
    if (paused || cm.safety_interlock_requested  != 0) {
		raw_spindle_state = SPINDLE_OFF;
    }
	//FIXME: else if(we just rebooted the ESC)... delay the pwm command...
	cm_set_spindle_state (MODEL, spindle_state);
    
    #ifdef __AVR
    if (raw_spindle_state == SPINDLE_CW) {
        gpio_set_bit_on(SPINDLE_BIT);
        gpio_set_bit_off(SPINDLE_DIR);
        } else if (raw_spindle_state == SPINDLE_CCW) {
        gpio_set_bit_on(SPINDLE_BIT);
        gpio_set_bit_on(SPINDLE_DIR);
        } else {
        gpio_set_bit_off(SPINDLE_BIT);	// failsafe: any error causes stop
    }
    #endif // __AVR
    #ifdef __ARM
    if (raw_spindle_state == SPINDLE_CW) {
        spindle_enable_pin.set();
        spindle_dir_pin.clear();
        } else if (raw_spindle_state == SPINDLE_CCW) {
        spindle_enable_pin.set();
        spindle_dir_pin.set();
        } else {
        spindle_enable_pin.clear();	// failsafe: any error causes stop
    }
    #endif // __ARM
	pwm_set_duty(PWM_1, _get_spindle_pwm(raw_spindle_state));
*/
}

/*
 * cm_spindle_conditional_pause() - pause spindle based on system flags selected
 * cm_spindle_conditional_resume() - restart a paused spindle with an optional dwell
 *
 *  Stops and Pauses are always immediate. Resumes may have an optional dwell
 *
 *  Usage: // conditionally shut down spindle on alarm (called from inside cm_alarm())
 *            cm_spindle_conditional_stop(SPINDLE_PAUSE_ON_ALARM); 
 */
void cm_spindle_conditional_pause()
{
    if (spindle.state != SPINDLE_OFF) {
        spSpindleState state = spindle.state;   // local copy
        spindle.pause = SPINDLE_PAUSED;
        cm_spindle_control_immediate(SPINDLE_OFF);
        spindle.state = state;                  // restore
    }
}

void cm_spindle_conditional_resume(float dwell_seconds)
{
    if(spindle.pause == SPINDLE_PAUSED) {
        mp_request_out_of_band_dwell(dwell_seconds);
        cm_spindle_control_immediate(spindle.state);
    }
    spindle.pause = SPINDLE_NORMAL;
    
/* OMC code - taken from other parts of the system, as this function was not here.
    if((cm.gm.spindle_state & (~SPINDLE_PAUSED)) != SPINDLE_OFF) {
        mp_request_out_of_band_dwell(dwell_seconds);
    }
    cm_spindle_control_immediate((cm.gm.spindle_state & (~SPINDLE_PAUSED)));
*/
}

/*
 * _get_spindle_pwm() - return PWM phase (duty cycle) for dir and speed
 */
static float _get_spindle_pwm (uint8_t spindle_state)
{
	float speed_lo=0, speed_hi=0, phase_lo=0, phase_hi=0;
	if (spindle_state == SPINDLE_CW ) {
		speed_lo = pwm.c[PWM_1].cw_speed_lo;
		speed_hi = pwm.c[PWM_1].cw_speed_hi;
		phase_lo = pwm.c[PWM_1].cw_phase_lo;
		phase_hi = pwm.c[PWM_1].cw_phase_hi;
	} else if (spindle_state == SPINDLE_CCW ) {
		speed_lo = pwm.c[PWM_1].ccw_speed_lo;
		speed_hi = pwm.c[PWM_1].ccw_speed_hi;
		phase_lo = pwm.c[PWM_1].ccw_phase_lo;
		phase_hi = pwm.c[PWM_1].ccw_phase_hi;
	}

	if (spindle_state==SPINDLE_CW || spindle_state==SPINDLE_CCW ) {
		// clamp spindle speed to lo/hi range
		if (cm.gm.spindle_speed < speed_lo) {
            cm.gm.spindle_speed = speed_lo;
        }        
		if (cm.gm.spindle_speed > speed_hi) {
            cm.gm.spindle_speed = speed_hi;
        }        
		// normalize speed to [0..1]
		float speed = (cm.gm.spindle_speed - speed_lo) / (speed_hi - speed_lo);
		return (speed * (phase_hi - phase_lo)) + phase_lo;
	} else {
		return pwm.c[PWM_1].phase_off;
	}
}

/*
 * cm_spindle_override_enable()
 * cm_spindle_override_factor()
 */

stat_t cm_spindle_override_enable(uint8_t flag)		// M51.1
{
    if (fp_TRUE(cm.gf.parameter) && fp_ZERO(cm.gn.parameter)) {
        spindle.override_enable = false;
    } else {
        spindle.override_enable = true;
    }
    return (STAT_OK);
}

stat_t cm_spindle_override_factor(uint8_t flag)		// M50.1
{
    spindle.override_enable = flag;
    spindle.override_factor = cm.gn.parameter;
//	change spindle speed
    return (STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

const char fmt_spo[] PROGMEM = "[spo] spindle options%14d [0=none,1=pause_on_hold]\n";
const char fmt_spd[] PROGMEM = "[spd] spindle auto-dwell time%6.1f seconds\n";
const char fmt_spc[] PROGMEM = "Spindle Control:%6d [0=OFF,1=CW,2=CCW]\n";
const char fmt_sps[] PROGMEM = "Spindle Speed: %8.0f rpm\n";

void cm_print_spo(nvObj_t *nv) { text_print_int(nv, fmt_spo);}
void cm_print_spd(nvObj_t *nv) { text_print_flt(nv, fmt_spd);}
void cm_print_spc(nvObj_t *nv) { text_print_int(nv, fmt_spc);}
void cm_print_sps(nvObj_t *nv) { text_print_flt(nv, fmt_sps);}

#endif // __TEXT_MODE