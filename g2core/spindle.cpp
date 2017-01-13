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
#include "settings.h"
#include "pwm.h"
#include "util.h"

/**** Allocate structures ****/

spSpindle_t spindle;

/**** Static functions ****/

static void _exec_spindle_speed(float *value, bool *flag);
static void _exec_spindle_control(float *value, bool *flag);
static float _get_spindle_pwm (spState state, spControl direction);

/***********************************************************************************
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
    spindle_control_immediate(SPINDLE_CONTROL_OFF, false); // turn spindle off
}

/***********************************************************************************
 * spindle_pause()             - pause spindle immediately if option is true
 * spindle_resume()            - restart a paused spindle with an optional dwell
 * spindle_control_immediate() - execute spindle control immediately
 * spindle_control_sync()      - queue a spindle control to the planner buffer
 * _exec_spindle_control()     - actually execute the spindle command
 */

void spindle_pause()
{
    if (spindle.state == SPINDLE_ON) {
        spindle_control_sync(SPINDLE_CONTROL_OFF, true);
    }
}

void spindle_resume()
{
    if (spindle.state == SPINDLE_PAUSE) {
        spindle_control_sync(SPINDLE_CONTROL_CW, false);
    }
}

stat_t spindle_control_immediate(uint8_t control, bool pause)
{
    float value[] = { (float)SPINDLE_OFF, 0,0,0,0,0 };
    bool flags[] =  { 1,0, pause, 0,0,0 };

    if (control != SPINDLE_CONTROL_OFF) {
        value[0] = SPINDLE_ON;
        value[1] = control;
        flags[1] = true;
    }
    _exec_spindle_control(value, flags);
    return(STAT_OK);
}

stat_t spindle_control_sync(uint8_t control, bool pause)  // uses spControl arg: OFF, CW, CCW
{
    float value[] = { (float)SPINDLE_OFF, 0,0,0,0,0 };
    bool flags[] =  { 1,0, pause, 0,0,0 };

    if (control != SPINDLE_CONTROL_OFF) {
        value[0] = SPINDLE_ON;
        value[1] = control;
        flags[1] = true;
    }
    mp_queue_command(_exec_spindle_control, value, flags);
    return(STAT_OK);
}

#define _set_spindle_enable_bit_hi() spindle_enable_pin.set()
#define _set_spindle_enable_bit_lo() spindle_enable_pin.clear()
#define _set_spindle_direction_bit_hi() spindle_dir_pin.set()
#define _set_spindle_direction_bit_lo() spindle_dir_pin.clear()

static void _exec_spindle_control(float *value, bool *flag)
{
    if (flag[1]) {      // set the direction first
        spindle.direction = (spControl)value[1];                // record spindle direction in the struct
        if ((spindle.direction-1) ^ spindle.dir_polarity) {     // take CW/CCW down to 0 or 1
            _set_spindle_direction_bit_hi();
        } else {
            _set_spindle_direction_bit_lo();
        }
    }
    if (flag[0]) {      // set on/off
        spindle.state = (spState)value[0];                      // record spindle state in the struct
        if ((spindle.state & 0x01) ^ spindle.enable_polarity) { // mask out PAUSE bit and consider it to be OFF
            _set_spindle_enable_bit_lo();
        } else {
            _set_spindle_enable_bit_hi();
        }
    }
    if (flag[2]) {      // set pause
        spindle.state = SPINDLE_PAUSE;
    }    
    pwm_set_duty(PWM_1, _get_spindle_pwm(spindle.state, spindle.direction));
}

/***********************************************************************************
 * spindle_queue_speed() - queue the S parameter to the planner buffer
 * _exec_spindle_speed() - spindle speed callback from planner queue
 */

stat_t spindle_speed_immediate(float speed)
{
    if (speed < spindle.speed_min) {
        return (STAT_SPINDLE_SPEED_BELOW_MINIMUM);
    }
    if (speed > spindle.speed_max) {
        return (STAT_SPINDLE_SPEED_MAX_EXCEEDED);
    }
    float value[AXES] = { speed, 0,0,0,0,0 };
//    bool flags[]      = { 1,0,0,0,0,0 };
    _exec_spindle_speed(value, nullptr);
    return (STAT_OK);
}

stat_t spindle_speed_sync(float speed)
{
    if (speed < spindle.speed_min) {
        return (STAT_SPINDLE_SPEED_BELOW_MINIMUM);
    }
    if (speed > spindle.speed_max) { 
        return (STAT_SPINDLE_SPEED_MAX_EXCEEDED);
    }
    float value[AXES] = { speed, 0,0,0,0,0 };
//    bool flags[]      = { 1,0,0,0,0,0 };
    mp_queue_command(_exec_spindle_speed, value, nullptr);
    return (STAT_OK);
}

static void _exec_spindle_speed(float *value, bool *flag)
{
    spindle.speed = value[0];
/*
    if (flag[1]) {
        spindle.direction = (spDir)value[1];
    }
*/
    // update spindle speed if we're running
    pwm_set_duty(PWM_1, _get_spindle_pwm(spindle.state, spindle.direction));
}

/***********************************************************************************
 * _get_spindle_pwm() - return PWM phase (duty cycle) for dir and speed
 */

static float _get_spindle_pwm (spState state, spControl direction)
{
    float speed_lo=0, speed_hi=0, phase_lo=0, phase_hi=0;
    if (direction == SPINDLE_CONTROL_CW ) {
        speed_lo = pwm.c[PWM_1].cw_speed_lo;
        speed_hi = pwm.c[PWM_1].cw_speed_hi;
        phase_lo = pwm.c[PWM_1].cw_phase_lo;
        phase_hi = pwm.c[PWM_1].cw_phase_hi;
    } else { // if (direction == SPINDLE_CONTROL_CCW ) {
        speed_lo = pwm.c[PWM_1].ccw_speed_lo;
        speed_hi = pwm.c[PWM_1].ccw_speed_hi;
        phase_lo = pwm.c[PWM_1].ccw_phase_lo;
        phase_hi = pwm.c[PWM_1].ccw_phase_hi;
    }

    if (state == SPINDLE_ON) {
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

/***********************************************************************************
 * spindle_override_control()
 * spindle_start_override()
 * spindle_end_override()
 */

stat_t spindle_override_control(const float P_word, const bool P_flag) // M51
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
    if (cm->gmx.m48_enable) {               // if master enable is ON
        if (new_enable && (new_override || !spindle.sso_enable)) {   // 3 cases to start a ramp
            spindle_start_override(SPINDLE_OVERRIDE_RAMP_TIME, spindle.sso_factor);
        } else if (spindle.sso_enable && !new_enable) {              // case to turn off the ramp
            spindle_end_override(SPINDLE_OVERRIDE_RAMP_TIME);
        }
    }
    spindle.sso_enable = new_enable;        // always update the enable state
    return (STAT_OK);
}

void spindle_start_override(const float ramp_time, const float override_factor)
{
    return;
}

void spindle_end_override(const float ramp_time)
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

/***********************************************************************************
 **** Spindle Settings
 ***********************************************************************************/

stat_t sp_get_spmo(nvObj_t *nv) { return(get_int(nv, spindle.mode)); }
stat_t sp_set_spmo(nvObj_t *nv) { return(set_int(nv, (uint8_t &)spindle.mode, SPINDLE_DISABLED, SPINDLE_MODE_MAX)); }

stat_t sp_get_spep(nvObj_t *nv) { return(get_int(nv, spindle.enable_polarity)); }
stat_t sp_set_spep(nvObj_t *nv) { return(set_int(nv, (uint8_t &)spindle.enable_polarity, 0, 1)); }
stat_t sp_get_spdp(nvObj_t *nv) { return(get_int(nv, spindle.dir_polarity)); }
stat_t sp_set_spdp(nvObj_t *nv) { return(set_int(nv, (uint8_t &)spindle.dir_polarity, 0, 1)); }
stat_t sp_get_spph(nvObj_t *nv) { return(get_int(nv, spindle.dir_polarity)); }
stat_t sp_set_spph(nvObj_t *nv) { return(set_int(nv, (uint8_t &)spindle.dir_polarity, 0, 1)); }

stat_t sp_get_spdw(nvObj_t *nv) { return(get_float(nv, spindle.dwell_seconds)); }
stat_t sp_set_spdw(nvObj_t *nv) { return(set_float_range(nv, spindle.dwell_seconds, 0, SPINDLE_DWELL_MAX)); }
stat_t sp_get_spsn(nvObj_t *nv) { return(get_float(nv, spindle.speed_min)); }
stat_t sp_set_spsn(nvObj_t *nv) { return(set_float_range(nv, spindle.speed_min, SPINDLE_SPEED_MIN, SPINDLE_SPEED_MAX)); }
stat_t sp_get_spsm(nvObj_t *nv) { return(get_float(nv, spindle.speed_max)); }
stat_t sp_set_spsm(nvObj_t *nv) { return(set_float_range(nv, spindle.speed_max, SPINDLE_SPEED_MIN, SPINDLE_SPEED_MAX)); }

stat_t sp_get_ssoe(nvObj_t *nv) { return(get_int(nv, spindle.sso_enable)); }
stat_t sp_set_ssoe(nvObj_t *nv) { return(set_int(nv, (uint8_t &)spindle.sso_enable, 0, 1)); }
stat_t sp_get_sso(nvObj_t *nv) { return(get_float(nv, spindle.sso_factor)); }
stat_t sp_set_sso(nvObj_t *nv) { return(set_float_range(nv, spindle.sso_factor, SPINDLE_OVERRIDE_MIN, SPINDLE_OVERRIDE_MAX)); }

/* These are provided as a way to set and clear spindle states without using M commands
 * SPD is useful because disabling a spindle (M5) does not change the direction, only the enable.
 */

stat_t sp_get_sps(nvObj_t *nv) { return(get_float(nv, spindle.speed)); }
stat_t sp_set_sps(nvObj_t *nv)
{
    set_float_range(nv, spindle.speed, spindle.speed_min, spindle.speed_max);
    float value[] = { spindle.speed, 0,0,0,0,0 };
    bool flags[] =  { 1,0,0,0,0,0 };
    _exec_spindle_speed(value, flags);
    return (STAT_OK);
}

stat_t sp_get_spe(nvObj_t *nv) { return(get_int(nv, spindle.state)); }
/*
stat_t sp_set_spe(nvObj_t *nv) 
{
//    set_int(nv, (uint8_t &)spindle.state, 0, 1);
    float value[] = { (float)spindle.state, (float)spindle.direction, 0,0,0,0 };
    bool flags[]  = { 1,1,0,0,0,0 };
    _exec_spindle_control(value, flags);
    return (STAT_OK);
}
*/

stat_t sp_get_spd(nvObj_t *nv) { return(get_int(nv, spindle.direction)); }
stat_t sp_set_spd(nvObj_t *nv)
{
    set_int(nv, (uint8_t &)spindle.direction, 0, 1);
    float value[] = { (float)spindle.state, (float)spindle.direction, 0,0,0,0 };
    bool flags[]  = { 1,1,0,0,0,0 };
    _exec_spindle_control(value, flags);
    return (STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

const char fmt_spmo[] = "[spmo] spindle mode%16d [0=disabled,1=plan-to-stop,2=continuous]\n";
const char fmt_spep[] = "[spep] spindle enable polarity%5d [0=active_low,1=active_high]\n";
const char fmt_spdp[] = "[spdp] spindle direction polarity%2d [0=CW_low,1=CW_high]\n";
const char fmt_spph[] = "[spph] spindle pause on hold%7d [0=no,1=pause_on_hold]\n";
const char fmt_spdw[] = "[spdw] spindle dwell time%12.1f seconds\n";
const char fmt_spsn[] = "[spsn] spindle speed min%14.2f rpm\n";
const char fmt_spsm[] = "[spsm] spindle speed max%14.2f rpm\n";
const char fmt_ssoe[] = "[ssoe] spindle speed override ena%2d [0=disable,1=enable]\n";
const char fmt_sso[]  = "[sso]  spindle speed override%10.3f [0.050 < sso < 2.000]\n";
const char fmt_sps[]  = "Spindle Speed: %7.0f rpm\n";
const char fmt_spe[]  = "Spindle Enable:%7d [0=OFF,1=ON,2=PAUSE]\n";
const char fmt_spd[]  = "Spindle Direction:%4d [0=CW,1=CCW]\n";

void sp_print_spmo(nvObj_t *nv) { text_print(nv, fmt_spmo);}    // TYPE_INT
void sp_print_spep(nvObj_t *nv) { text_print(nv, fmt_spep);}    // TYPE_INT
void sp_print_spdp(nvObj_t *nv) { text_print(nv, fmt_spdp);}    // TYPE_INT
void sp_print_spph(nvObj_t *nv) { text_print(nv, fmt_spph);}    // TYPE_INT
void sp_print_spdw(nvObj_t *nv) { text_print(nv, fmt_spdw);}    // TYPE_FLOAT
void sp_print_spsn(nvObj_t *nv) { text_print(nv, fmt_spsn);}    // TYPE_FLOAT
void sp_print_spsm(nvObj_t *nv) { text_print(nv, fmt_spsm);}    // TYPE_FLOAT
void sp_print_ssoe(nvObj_t *nv) { text_print(nv, fmt_ssoe);}    // TYPE INT
void sp_print_sso(nvObj_t *nv)  { text_print(nv, fmt_sso);}     // TYPE FLOAT
void sp_print_spe(nvObj_t *nv)  { text_print(nv, fmt_spe);}     // TYPE_INT
void sp_print_spd(nvObj_t *nv)  { text_print(nv, fmt_spd);}     // TYPE_INT
void sp_print_sps(nvObj_t *nv)  { text_print(nv, fmt_sps);}     // TYPE_FLOAT

#endif // __TEXT_MODE
