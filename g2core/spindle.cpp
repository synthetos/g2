/*
 * spindle.cpp - canonical machine spindle driver
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2017 Alden S. Hart, Jr.
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

#include "g2core.h"             // #1 dependency order
#include "config.h"             // #2
#include "canonical_machine.h"  // #3
#include "text_parser.h"        // #4

#include "spindle.h"
#include "planner.h"
#include "hardware.h"
#include "pwm.h"
#include "util.h"

/**** Allocate structures ****/

cmSpindleton_t spindle;

/**** Static functions ****/

static void _exec_spindle_speed(float *value, bool *flag);
static void _exec_spindle_control(float *value, bool *flag);
static float _get_spindle_pwm (cmSpindleEnable enable, cmSpindleDir direction);

/*
 * spindle_init()
 * spindle_reset() - stop spindle, set speed to zero, and reset values
 */
void spindle_init()
{
    if( pwm.c[PWM_1].frequency < 0 ) {
        pwm.c[PWM_1].frequency = 0;
    }
    pwm_set_freq(PWM_1, pwm.c[PWM_1].frequency);
    pwm_set_duty(PWM_1, pwm.c[PWM_1].phase_off);
}

void spindle_reset()
{
    float value[AXES] = { 0,0,0,0,0,0 };        // set spindle speed to zero
    bool flags[] = { 1,0,0,0,0,0 };
   _exec_spindle_speed(value, flags);
    cm_spindle_off_immediate();                 // turn spindle off
}

/*
 * cm_set_spindle_speed() - queue the S parameter to the planner buffer
 * _exec_spindle_speed() - spindle speed callback from planner queue
 */

stat_t cm_set_spindle_speed(float speed)
{
//    if (speed > cfg.max_spindle speed) { return (STAT_MAX_SPINDLE_SPEED_EXCEEDED);}

    float value[AXES] = { speed, 0,0,0,0,0 };
    bool flags[] = { 1,0,0,0,0,0 };
    mp_queue_command(_exec_spindle_speed, value, flags);
    return (STAT_OK);
}

static void _exec_spindle_speed(float *value, bool *flag)
{
    if (flag[0]) {
        spindle.speed = value[0];
    }

    if (flag[1]) {
        spindle.direction = (cmSpindleDir)value[1];
    }

    // update spindle speed if we're running
    pwm_set_duty(PWM_1, _get_spindle_pwm(spindle.enable, spindle.direction));
}

/*
 * cm_spindle_off_immediate() - turn on/off spindle w/o planning
 * cm_spindle_optional_pause() - pause spindle immediately if option is true
 * cm_spindle_resume() - restart a paused spindle with an optional dwell
 */

void cm_spindle_off_immediate()
{
    spindle.enable = SPINDLE_OFF;
    float value[] = { (float)SPINDLE_OFF, 0,0,0,0,0 };
    bool flags[] =  { 1,0,0,0,0,0 };
    _exec_spindle_control(value, flags);
}

void cm_spindle_optional_pause(bool option)
{
    if (option && spindle.enable == SPINDLE_ON) {
        cm_spindle_off_immediate();
        spindle.enable = SPINDLE_PAUSE;
    }
}

void cm_spindle_resume(float dwell_seconds)
{
    if(spindle.enable == SPINDLE_PAUSE) {
        spindle.enable = SPINDLE_ON;
        mp_request_out_of_band_dwell(dwell_seconds);
        float value[] = { (float)SPINDLE_ON, (float)spindle.direction, 0,0,0,0 };
        bool flags[] =  { 1,0,0,0,0,0 };
        _exec_spindle_control(value, flags);
    }
}

/*
 * cm_spindle_control() - queue the spindle command to the planner buffer. Observe PAUSE
 * _exec_spindle_control() - actually execute the spindle command
 */

stat_t cm_spindle_control(uint8_t control)  // requires SPINDLE_CONTROL_xxx style args
{
    if (control == SPINDLE_CONTROL_OFF) {
        spindle.enable = SPINDLE_OFF;
    } else {
        spindle.enable = SPINDLE_ON;
        if (control == SPINDLE_CONTROL_CW) {
            spindle.direction = SPINDLE_CW;
        } else {
            spindle.direction = SPINDLE_CCW;
        }
    }

    float value[] = { (float)spindle.enable, (float)spindle.direction, 0,0,0,0 };
    bool flags[] =  { 1,1,0,0,0,0 };
    mp_queue_command(_exec_spindle_control, value, flags);
    return(STAT_OK);
}

    #define _set_spindle_enable_bit_hi() spindle_enable_pin.set()
    #define _set_spindle_enable_bit_lo() spindle_enable_pin.clear()
    #define _set_spindle_direction_bit_hi() spindle_dir_pin.set()
    #define _set_spindle_direction_bit_lo() spindle_dir_pin.clear()

static void _exec_spindle_control(float *value, bool *flag)
{
    // set the direction first
    if (flag[1])
    {
        spindle.direction = (cmSpindleDir)value[1];             // record spindle direction in the struct
        if (spindle.direction ^ spindle.dir_polarity) {
            _set_spindle_direction_bit_hi();
        } else {
            _set_spindle_direction_bit_lo();
        }
    }

    if (flag[0])
    {
        // set on/off. Mask out PAUSE and consider it OFF
        spindle.enable = (cmSpindleEnable)value[0];             // record spindle enable in the struct
        if ((spindle.enable & 0x01) ^ spindle.enable_polarity) {
            _set_spindle_enable_bit_lo();
        } else {
            _set_spindle_enable_bit_hi();
        }
    }

    pwm_set_duty(PWM_1, _get_spindle_pwm(spindle.enable, spindle.direction));
}

/*
 * _get_spindle_pwm() - return PWM phase (duty cycle) for dir and speed
 */

static float _get_spindle_pwm (cmSpindleEnable enable, cmSpindleDir direction)
{
    float speed_lo=0, speed_hi=0, phase_lo=0, phase_hi=0;
    if (direction == SPINDLE_CW ) {
        speed_lo = pwm.c[PWM_1].cw_speed_lo;
        speed_hi = pwm.c[PWM_1].cw_speed_hi;
        phase_lo = pwm.c[PWM_1].cw_phase_lo;
        phase_hi = pwm.c[PWM_1].cw_phase_hi;
    } else { // if (direction == SPINDLE_CCW ) {
        speed_lo = pwm.c[PWM_1].ccw_speed_lo;
        speed_hi = pwm.c[PWM_1].ccw_speed_hi;
        phase_lo = pwm.c[PWM_1].ccw_phase_lo;
        phase_hi = pwm.c[PWM_1].ccw_phase_hi;
    }

    if (enable == SPINDLE_ON) {
        // clamp spindle speed to lo/hi range
        if (spindle.speed < speed_lo) {
            spindle.speed = speed_lo;
        }
        if (spindle.speed > speed_hi) {
            spindle.speed = speed_hi;
        }
        // normalize speed to [0..1]
        float speed = (spindle.speed - speed_lo) / (speed_hi - speed_lo);
        return (speed * (phase_hi - phase_lo)) + phase_lo;
    } else {
        return pwm.c[PWM_1].phase_off;
    }
}


/*
 * cm_spindle_override_control()
 * cm_start_spindle_override()
 * cm_end_spindle_override()
 */

stat_t cm_sso_control(const float P_word, const bool P_flag) // M51
{
    bool new_enable = true;
    bool new_override = false;
    if (P_flag) {                           // if parameter is present in Gcode block
        if (fp_ZERO(P_word)) {
            new_enable = false;             // P0 disables override
        } else {
            if (P_word < SPINDLE_OVERRIDE_MIN) {
                return (STAT_INPUT_LESS_THAN_MIN_VALUE);
            }
            if (P_word > SPINDLE_OVERRIDE_MAX) {
                return (STAT_INPUT_EXCEEDS_MAX_VALUE);
            }
            spindle.sso_factor = P_word;    // P word is valid, store it.
            new_override = true;
        }
    }
    if (cm.gmx.m48_enable) {               // if master enable is ON
        if (new_enable && (new_override || !spindle.sso_enable)) {   // 3 cases to start a ramp
            cm_start_spindle_override(SPINDLE_OVERRIDE_RAMP_TIME, spindle.sso_factor);
        } else if (spindle.sso_enable && !new_enable) {              // case to turn off the ramp
            cm_end_spindle_override(SPINDLE_OVERRIDE_RAMP_TIME);
        }
    }
    spindle.sso_enable = new_enable;        // always update the enable state
    return (STAT_OK);
}

void cm_start_spindle_override(const float ramp_time, const float override_factor)
{
    return;
}

void cm_end_spindle_override(const float ramp_time)
{
    return;
}

/****************************
 * END OF SPINDLE FUNCTIONS *
 ****************************/

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/*
 * cm_set_dir() - a cheat to set direction w/o using the M commands
 *
 * This is provided as a way to set and clear spindle direction without using M commands
 * It's here because disabling a spindle (M5) does not change the direction, only the enable.
 */

stat_t cm_set_dir(nvObj_t *nv)
{
    set_01(nv);
    float value[] = { (float)spindle.enable, (float)spindle.direction, 0,0,0,0 };
    bool flags[] =  { 1,1,0,0,0,0 };
    _exec_spindle_control(value, flags);
    return (STAT_OK);
}


/*
 * cm_set_sso() - set spindle speed feedrate override factor
 */

stat_t cm_set_sso(nvObj_t *nv)
{
    if (nv->value < SPINDLE_OVERRIDE_MIN) {
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value > SPINDLE_OVERRIDE_MAX) {
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    set_flt(nv);
    return(STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

const char fmt_spep[] = "[spep] spindle enable polarity%5d [0=active_low,1=active_high]\n";
const char fmt_spdp[] = "[spdp] spindle direction polarity%2d [0=CW_low,1=CW_high]\n";
const char fmt_spph[] = "[spph] spindle pause on hold%7d [0=no,1=pause_on_hold]\n";
const char fmt_spdw[] = "[spdw] spindle dwell time%12.1f seconds\n";
const char fmt_ssoe[] ="[ssoe] spindle speed override ena%2d [0=disable,1=enable]\n";
const char fmt_sso[] ="[sso] spindle speed override%11.3f [0.050 < sso < 2.000]\n";
const char fmt_spe[] = "Spindle Enable:%7d [0=OFF,1=ON,2=PAUSE]\n";
const char fmt_spd[] = "Spindle Direction:%4d [0=CW,1=CCW]\n";
const char fmt_sps[] = "Spindle Speed: %7.0f rpm\n";

void cm_print_spep(nvObj_t *nv) { text_print(nv, fmt_spep);}    // TYPE_INT
void cm_print_spdp(nvObj_t *nv) { text_print(nv, fmt_spdp);}    // TYPE_INT
void cm_print_spph(nvObj_t *nv) { text_print(nv, fmt_spph);}    // TYPE_INT
void cm_print_spdw(nvObj_t *nv) { text_print(nv, fmt_spdw);}    // TYPE_FLOAT
void cm_print_ssoe(nvObj_t *nv) { text_print(nv, fmt_ssoe);}    // TYPE INT
void cm_print_sso(nvObj_t *nv)  { text_print(nv, fmt_sso);}     // TYPE FLOAT
void cm_print_spe(nvObj_t *nv)  { text_print(nv, fmt_spe);}     // TYPE_INT
void cm_print_spd(nvObj_t *nv)  { text_print(nv, fmt_spd);}     // TYPE_INT
void cm_print_sps(nvObj_t *nv)  { text_print(nv, fmt_sps);}     // TYPE_FLOAT

#endif // __TEXT_MODE
