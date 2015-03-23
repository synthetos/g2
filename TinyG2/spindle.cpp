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

cmSpindleton_t spindle;

/**** Static functions ****/

static void _exec_spindle_speed(float *value, float *flag);
static void _exec_spindle_control(float *value, float *flag);
static float _get_spindle_pwm (cmSpindleState spindle_state);

/*
 * spindle_init()
 * spindle_reset()
 */
void spindle_init()
{
	if( pwm.c[PWM_1].frequency < 0 )
		pwm.c[PWM_1].frequency = 0;

    pwm_set_freq(PWM_1, pwm.c[PWM_1].frequency);
    pwm_set_duty(PWM_1, pwm.c[PWM_1].phase_off);
}

void spindle_reset()
{
    float value[AXES] = { 0,0,0,0,0,0 };        // set spindle speed to zero
    _exec_spindle_speed(value, value);
    cm_spindle_control_immediate(SPINDLE_OFF);  // turn spindle off
}

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
 * cm_spindle_optional_pause() - pause spindle immediately if option is true
 * cm_spindle_resume() - restart a paused spindle with an optional dwell
 */
void cm_spindle_optional_pause(bool option)
{
    if (option && (spindle.state != SPINDLE_OFF)) {
        cmSpindleState state = spindle.state;       // local copy
        cm_spindle_control_immediate(SPINDLE_OFF);  // changes spindle state
        spindle.pause = SPINDLE_PAUSED;             // mark as paused
        spindle.state = state;                      // restore previous spindle state
    }
}

void cm_spindle_resume(float dwell_seconds)
{
    if(spindle.pause == SPINDLE_PAUSED) {
        mp_request_out_of_band_dwell(dwell_seconds);
        cm_spindle_control_immediate(spindle.state);
    }
    spindle.pause = SPINDLE_NORMAL;
}

/*
 * cm_spindle_control() - queue the spindle command to the planner buffer. Observe PAUSE
 * cm_spindle_control_immediate() - turn on/off spindle w/o planning
 * _exec_spindle_control() - execute the spindle command (called from planner)
 */

stat_t cm_spindle_control(uint8_t spindle_state)
{
/* OMC code
	if (cm.gm.spindle_state & SPINDLE_PAUSED)
		spindle_state |= SPINDLE_PAUSED;

    // This is kind of tricky...  If we're in interlock but still moving around, and we get an
    // M3, we just start a feedhold...  Usually, before we call cm_start_hold we check if there's
    // anything in the buffer to actually process the feedhold. Here, we're just about to add
    // something to the buffer, so we skip the check.
	if (cm.safety_interlock_state != SAFETY_INTERLOCK_ENGAGED &&
      !(spindle_state & SPINDLE_PAUSED) && spindle_state != SPINDLE_OFF) {
		cm_start_hold();
    }
*/
	float value[AXES] = { (float)spindle_state, 0,0,0,0,0 };
	mp_queue_command(_exec_spindle_control, value, value);
	return(STAT_OK);
}

void cm_spindle_control_immediate(cmSpindleState spindle_state)
{
/* OMC code
    spindle_state &= ~SPINDLE_PAUSED;           // remove the pause bit
    if (spindle_state != cm.gm.spindle_state) { // if it's already there, skip it
        float value[AXES] = { (float)spindle_state, 0,0,0,0,0 };
        _exec_spindle_control(value, value);
    }
*/
    if (spindle_state == SPINDLE_OFF) {         // cancel PAUSE if turning off spindle
        spindle.pause = SPINDLE_NORMAL;
    }
    float value[AXES] = { (float)spindle_state, 0,0,0,0,0 };
    _exec_spindle_control(value, value);
}

#ifdef __ARM
#define _set_spindle_enable_bit_hi() spindle_enable_pin.set()
#define _set_spindle_enable_bit_lo() spindle_enable_pin.clear()
#define _set_spindle_direction_bit_hi() spindle_dir_pin.set()
#define _set_spindle_direction_bit_lo() spindle_dir_pin.clear()
#endif
#ifdef __AVR
#define _set_spindle_enable_bit_hi() gpio_set_bit_on(SPINDLE_BIT)
#define _set_spindle_enable_bit_lo() gpio_set_bit_off(SPINDLE_BIT)
#define _set_spindle_direction_bit_hi() gpio_set_bit_on(SPINDLE_DIR)
#define _set_spindle_direction_bit_lo() gpio_set_bit_off(SPINDLE_DIR)
#endif

static void _exec_spindle_control(float *value, float *flag)
{
    spindle.state = (cmSpindleState)value[0];               // set spindle state
    bool enable = spindle.state ^ ~spindle.polarity_enable; // enable is the polarity to set if turning ON

	if ((spindle.state == SPINDLE_CW) || (spindle.state == SPINDLE_CCW)) {

        // set the direction first
        if ((spindle.state & 0x02) ^ spindle.polarity_dir) { // xor the CCW bit to get direction
            _set_spindle_direction_bit_hi();
        } else {
            _set_spindle_direction_bit_lo();
        }

        // then run the enable
        if (enable) {
            _set_spindle_enable_bit_hi();
        } else {
            _set_spindle_enable_bit_lo();
        }

    } else { // SPINDLE_OFF (for safety - any value other than CW or CCW causes stop)
        if (enable) {
            _set_spindle_enable_bit_lo();
        } else {
            _set_spindle_enable_bit_hi();
        }
	}
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

    //// the rest is the AVR/ARM bit twiddling..., and the pwm_set_duty()
*/
}

/*
 * _get_spindle_pwm() - return PWM phase (duty cycle) for dir and speed
 */
static float _get_spindle_pwm (cmSpindleState spindle_state)
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
/*
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
*/
/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

const char fmt_spo[] PROGMEM = "[spo] spindle polarity on%10d [0=low is enabled,1=high is enabled]\n";
const char fmt_spd[] PROGMEM = "[spd] spindle polarity direction%3d [0=low is clockwise,1=high is clockwise]\n";
const char fmt_sph[] PROGMEM = "[sph] spindle pause on hold%8d [0=no,1=pause_on_hold]\n";
const char fmt_sdw[] PROGMEM = "[sdw] spindle auto-dwell time%8.1f seconds\n";
const char fmt_spc[] PROGMEM = "Spindle Control:%6d [0=OFF,1=CW,2=CCW]\n";
const char fmt_sps[] PROGMEM = "Spindle Speed: %8.0f rpm\n";

void cm_print_spo(nvObj_t *nv) { text_print(nv, fmt_spo);}  // TYPE_INT
void cm_print_spd(nvObj_t *nv) { text_print(nv, fmt_spd);}  // TYPE_INT
void cm_print_sph(nvObj_t *nv) { text_print(nv, fmt_sph);}  // TYPE_INT
void cm_print_sdw(nvObj_t *nv) { text_print(nv, fmt_sdw);}  // TYPE_FLOAT
void cm_print_spc(nvObj_t *nv) { text_print(nv, fmt_spc);}  // TYPE_INT
void cm_print_sps(nvObj_t *nv) { text_print(nv, fmt_sps);}  // TYPE_FLOAT

#endif // __TEXT_MODE