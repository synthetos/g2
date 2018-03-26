/*
 * spindle.cpp - canonical machine spindle driver
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2018 Alden S. Hart, Jr.
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

static float _get_spindle_pwm (spSpindle_t &_spindle, pwmControl_t &_pwm);

#define SPINDLE_DIRECTION_ASSERT \
    if ((spindle.direction < SPINDLE_CW) || (spindle.direction > SPINDLE_CCW)) { \
         spindle.direction = SPINDLE_CW; \
    }

/****************************************************************************************
 * spindle_init()
 * spindle_reset() - stop spindle, set speed to zero, and reset values
 */
void spindle_init()
{
    SPINDLE_DIRECTION_ASSERT        // spindle needs an initial direction
    
    if( pwm.c[PWM_1].frequency < 0 ) {
        pwm.c[PWM_1].frequency = 0;
    }
    pwm_set_freq(PWM_1, pwm.c[PWM_1].frequency);
    pwm_set_duty(PWM_1, pwm.c[PWM_1].phase_off);
}

void spindle_reset()
{
    spindle_speed_immediate(0);
    spindle_control_immediate(SPINDLE_OFF);
}

/****************************************************************************************
 * _exec_spindle_control()     - actually execute the spindle command
 * spindle_control_immediate() - execute spindle control immediately
 * spindle_control_sync()      - queue a spindle control to the planner buffer
 *
 *  Basic operation: Spindle function is executed by _exec_spindle_control().
 *  Spindle_control_immediate() performs the control as soon as it's received. 
 *  Spindle_control_sync() inserts spindle move into the planner, and handles spinups.
 *
 *  Valid inputs to Spindle_control_immediate() and Spindle_control_sync() are:
 *
 *    - SPINDLE_OFF turns off spindle and sets spindle state to SPINDLE_OFF.
 *      This will also re-load enable and direction polarity to the pins if they have changed.
 *      The spindle.direction value is not affected (although this doesn't really matter).
 *
 *    - SPINDLE_CW or SPINDLE_CCW turns sets direction accordingly and spindle on.
 *      In spindle_control_sync() a non-zero spinup delay runs a dwell immediately 
 *      following the spindle change, but only if the planner had planned the spindle 
 *      operation to zero. (I.e. if the spindle controls / S words do not plan to zero 
 *      the delay is not run). Spindle_control_immediate() has no spinup delay or 
 *      dwell behavior.
 *
 *    - SPINDLE_PAUSE is only applicable to CW and CCW states. It forces the spindle OFF and 
 *      sets spindle.state to PAUSE. A PAUSE received when not in CW or CCW state is ignored.
 *
 *    - SPINDLE_RESUME, if in a PAUSE state, reverts to previous SPINDLE_CW or SPINDLE_CCW.
 *      The SPEED is not changed, and if it were changed in the interim the "new" speed 
 *      is used. If RESUME is received from spindle_control_sync() the usual spinup delay 
 *      behavior occurs. If RESUME is received when not in a PAUSED state it is ignored. 
 *      This recognizes that the main reason an immediate command would be issued - either 
 *      manually by the user or by an alarm or some other program function - is to stop 
 *      a spindle. So the Resume should be ignored for safety.
 */
/*  Notes:
 *    - Since it's possible to queue a sync'd control, and then set any spindle state 
 *      with an immediate() before the queued command is reached, _exec_spindle_control()
 *      must gracefully handle any arbitrary state transition (not just the "legal" ones).
 *
 *    - The spinup and spindown rows are present, but are not implemented unless we 
 *      find we need them. It's easy enough to set these flags using the bit vector 
 *      passed from sync(),but unsetting them once the delay is complete would take 
 *      some more work.
 *  
 *    Q: Do we need a spin-down for direction reversal?
 *    Q: Should the JSON be able to pause and resume? For test purposes only?
 */
/*  State/Control matrix. Read "If you are in state X and get control Y do action Z"
 
    Control: OFF         CW          CCW        PAUSE      RESUME     
 State: |-----------|-----------|-----------|-----------|-----------|
    OFF |    OFF    |    CW     |    CCW    |    NOP    |    NOP    |
        |-----------|-----------|-----------|-----------|-----------|
     CW |    OFF    |    NOP    |  REVERSE  |   PAUSE   |    NOP    |
        |-----------|-----------|-----------|-----------|-----------|
    CCW |    OFF    |  REVERSE  |    NOP    |   PAUSE   |    NOP    |
        |-----------|-----------|-----------|-----------|-----------|
  PAUSE |    OFF    |    CW     |    CCW    |    NOP    |   RESUME  |
        |-----------|-----------|-----------|-----------|-----------|
 RESUME |  invalid  |  invalid  |  invalid  |  invalid  |  invalid  |
        |-----------|-----------|-----------|-----------|-----------|

 Actions:
    - OFF       Turn spindle off. Even if it's already off (reloads polarities)
    - CW        Turn spindle on clockwise
    - CCW       Turn spindle on counterclockwise
    - PAUSE     Turn off spindle, enter PAUSE state    
    - RESUME    Turn spindle on CW or CCW as before
    - NOP       No operation, ignore
    - REVERSE   Reverse spindle direction (Q: need a cycle to spin down then back up again?)
 */

static void _exec_spindle_control(float *value, bool *flag)
{
    spControl control = (spControl)value[0];
    if (control > SPINDLE_ACTION_MAX) {
        return;
    }
    spControl state = spindle.state;
    if (state >= SPINDLE_ACTION_MAX) {
//        rpt_exception(STAT_SPINDLE_ASSERTION_FAILURE, "illegal spindle state");        
        return;
    }
    spControl matrix[20] = {
        SPINDLE_OFF,  SPINDLE_CW,    SPINDLE_CCW,    SPINDLE_NOP,   SPINDLE_NOP,
        SPINDLE_OFF,  SPINDLE_NOP,   SPINDLE_REV,    SPINDLE_PAUSE, SPINDLE_NOP,
        SPINDLE_OFF,  SPINDLE_REV,   SPINDLE_NOP,    SPINDLE_PAUSE, SPINDLE_NOP,
        SPINDLE_OFF,  SPINDLE_CW,    SPINDLE_CCW,    SPINDLE_NOP,   SPINDLE_RESUME
    };
    spControl action = matrix[(state*5)+control];

    SPINDLE_DIRECTION_ASSERT;               // ensure that the spindle direction is sane
    int8_t enable_bit = 0;                  // default to 0=off
    int8_t dir_bit = -1;                    // -1 will skip setting the direction. 0 & 1 are valid values
    bool spinup_delay = false;

    switch (action) {
        case SPINDLE_NOP: { return; }
            
        case SPINDLE_OFF: {                 // enable_bit already set for this case
            dir_bit = spindle.direction-1;  // spindle direction was stored as '1' & '2'
            spindle.state = SPINDLE_OFF;    // the control might have been something other than SPINDLE_OFF
            break;
        }
        case SPINDLE_CW: case SPINDLE_CCW: case SPINDLE_REV: {  // REV is handled same as CW or CCW for now         
            enable_bit = 1;
            dir_bit = control-1;            // adjust direction to be used as a bitmask
            spindle.direction = control;
            spindle.state = control;
            spinup_delay = true;
            break; 
        }
        case SPINDLE_PAUSE : { 
            spindle.state = SPINDLE_PAUSE;
            break;                          // enable bit is already set up to stop the move
        }
        case SPINDLE_RESUME: { 
            enable_bit = 1;
            dir_bit = spindle.direction-1;  // spindle direction was stored as '1' & '2'
            spindle.state = spindle.direction;
            spinup_delay = true;
            break; 
        }
        default: {}                         // reversals not handled yet
    }

    // Apply the enable and direction bits and adjust the PWM as required

    // set the direction first
    if (dir_bit >= 0) {
        if (dir_bit ^ spindle.dir_polarity) {
            spindle_dir_pin.set();          // drive pin HI
        } else {
            spindle_dir_pin.clear();        // drive pin LO
        }
    }

    // set spindle enable
    if (enable_bit ^ spindle.enable_polarity) {
        spindle_enable_pin.clear();         // drive pin LO
    } else {
        spindle_enable_pin.set();           // drive pin HI
    }
    pwm_set_duty(PWM_1, _get_spindle_pwm(spindle, pwm));

    if (spinup_delay) {
        mp_request_out_of_band_dwell(spindle.spinup_delay);
    }
}

/*
 * spindle_control_immediate() - execute spindle control immediately
 * spindle_control_sync()      - queue a spindle control to the planner buffer
 */

stat_t spindle_control_immediate(spControl control)
{
    float value[] = { (float)control };
    _exec_spindle_control(value, nullptr);
    return(STAT_OK);
}

stat_t spindle_control_sync(spControl control)  // uses spControl arg: OFF, CW, CCW
{
    // skip the PAUSE operation if pause-enable is not enabled (pause-on-hold)
    if ((control == SPINDLE_PAUSE) && (!spindle.pause_enable)) {
        return (STAT_OK);
    }
    
    // queue the spindle control
    float value[] = { (float)control };
    mp_queue_command(_exec_spindle_control, value, nullptr);
    return(STAT_OK);
}

/****************************************************************************************
 * _exec_spindle_speed()     - actually execute the spindle speed command
 * spindle_speed_immediate() - execute spindle speed change immediately
 * spindle_speed_sync()      - queue a spindle speed change to the planner buffer
 *
 *  Setting S0 is considered as turning spindle off. Setting S to non-zero from S0
 *  will enable a spinup delay if spinups are npn-zero.
 */

static void _exec_spindle_speed(float *value, bool *flag)
{
    float previous_speed = spindle.speed;

    spindle.speed = value[0];
    pwm_set_duty(PWM_1, _get_spindle_pwm(spindle, pwm));

    if (fp_ZERO(previous_speed)) {
        mp_request_out_of_band_dwell(spindle.spinup_delay);
    }
}

static stat_t _casey_jones(float speed)
{
    if (speed < spindle.speed_min) { return (STAT_SPINDLE_SPEED_BELOW_MINIMUM); }
    if (speed > spindle.speed_max) { return (STAT_SPINDLE_SPEED_MAX_EXCEEDED); }
    return (STAT_OK);    
}

stat_t spindle_speed_immediate(float speed)
{
    ritorno(_casey_jones(speed));
    float value[] = { speed };
    _exec_spindle_speed(value, nullptr);
    return (STAT_OK);
}

stat_t spindle_speed_sync(float speed)
{
    ritorno(_casey_jones(speed));
    float value[] = { speed };
    mp_queue_command(_exec_spindle_speed, value, nullptr);
    return (STAT_OK);
}

/****************************************************************************************
 * _get_spindle_pwm() - return PWM phase (duty cycle) for dir and speed
 */

static float _get_spindle_pwm (spSpindle_t &_spindle, pwmControl_t &_pwm)
{
    float speed_lo, speed_hi, phase_lo, phase_hi;
    if (_spindle.direction == SPINDLE_CW ) {
        speed_lo = _pwm.c[PWM_1].cw_speed_lo;
        speed_hi = _pwm.c[PWM_1].cw_speed_hi;
        phase_lo = _pwm.c[PWM_1].cw_phase_lo;
        phase_hi = _pwm.c[PWM_1].cw_phase_hi;
    } else { // if (direction == SPINDLE_CCW ) {
        speed_lo = _pwm.c[PWM_1].ccw_speed_lo;
        speed_hi = _pwm.c[PWM_1].ccw_speed_hi;
        phase_lo = _pwm.c[PWM_1].ccw_phase_lo;
        phase_hi = _pwm.c[PWM_1].ccw_phase_hi;
    }

    if ((_spindle.state == SPINDLE_CW) || (_spindle.state == SPINDLE_CCW)) {
        // clamp spindle speed to lo/hi range
        if (_spindle.speed < speed_lo) {
            _spindle.speed = speed_lo;
        }
        if (_spindle.speed > speed_hi) {
            _spindle.speed = speed_hi;
        }
        // normalize speed to [0..1]
        float speed = (_spindle.speed - speed_lo) / (speed_hi - speed_lo);
        return ((speed * (phase_hi - phase_lo)) + phase_lo);
    } else {
        return (_pwm.c[PWM_1].phase_off);
    }
}

/****************************************************************************************
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
            spindle.override_factor = P_word;    // P word is valid, store it.
            new_override = true;
        }
    }
    if (cm->gmx.m48_enable) {               // if master enable is ON
        if (new_enable && (new_override || !spindle.override_enable)) {   // 3 cases to start a ramp
            spindle_start_override(SPINDLE_OVERRIDE_RAMP_TIME, spindle.override_factor);
        } else if (spindle.override_enable && !new_enable) {              // case to turn off the ramp
            spindle_end_override(SPINDLE_OVERRIDE_RAMP_TIME);
        }
    }
    spindle.override_enable = new_enable;        // always update the enable state
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

/****************************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ****************************************************************************************/

/****************************************************************************************
 **** Spindle Settings ******************************************************************
 ****************************************************************************************/

stat_t sp_get_spmo(nvObj_t *nv) { return(get_integer(nv, spindle.mode)); }
stat_t sp_set_spmo(nvObj_t *nv) { return(set_integer(nv, (uint8_t &)spindle.mode, SPINDLE_DISABLED, SPINDLE_MODE_MAX)); }

stat_t sp_get_spep(nvObj_t *nv) { return(get_integer(nv, spindle.enable_polarity)); }
stat_t sp_set_spep(nvObj_t *nv) { 
    stat_t status = set_integer(nv, (uint8_t &)spindle.enable_polarity, 0, 1); 
    spindle_control_immediate(SPINDLE_OFF); // stop spindle and apply new settings
    return (status);
}

stat_t sp_get_spdp(nvObj_t *nv) { return(get_integer(nv, spindle.dir_polarity)); }
stat_t sp_set_spdp(nvObj_t *nv) { 
    stat_t status = set_integer(nv, (uint8_t &)spindle.dir_polarity, 0, 1); 
    spindle_control_immediate(SPINDLE_OFF); // stop spindle and apply new settings
    return (status);
}

stat_t sp_get_spph(nvObj_t *nv) { return(get_integer(nv, spindle.pause_enable)); }
stat_t sp_set_spph(nvObj_t *nv) { return(set_integer(nv, (uint8_t &)spindle.pause_enable, 0, 1)); }
stat_t sp_get_spde(nvObj_t *nv) { return(get_float(nv, spindle.spinup_delay)); }
stat_t sp_set_spde(nvObj_t *nv) { return(set_float_range(nv, spindle.spinup_delay, 0, SPINDLE_DWELL_MAX)); }

stat_t sp_get_spsn(nvObj_t *nv) { return(get_float(nv, spindle.speed_min)); }
stat_t sp_set_spsn(nvObj_t *nv) { return(set_float_range(nv, spindle.speed_min, SPINDLE_SPEED_MIN, SPINDLE_SPEED_MAX)); }
stat_t sp_get_spsm(nvObj_t *nv) { return(get_float(nv, spindle.speed_max)); }
stat_t sp_set_spsm(nvObj_t *nv) { return(set_float_range(nv, spindle.speed_max, SPINDLE_SPEED_MIN, SPINDLE_SPEED_MAX)); }

stat_t sp_get_spoe(nvObj_t *nv) { return(get_integer(nv, spindle.override_enable)); }
stat_t sp_set_spoe(nvObj_t *nv) { return(set_integer(nv, (uint8_t &)spindle.override_enable, 0, 1)); }
stat_t sp_get_spo(nvObj_t *nv) { return(get_float(nv, spindle.override_factor)); }
stat_t sp_set_spo(nvObj_t *nv) { return(set_float_range(nv, spindle.override_factor, SPINDLE_OVERRIDE_MIN, SPINDLE_OVERRIDE_MAX)); }

// These are provided as a way to view and control spindles without using M commands
stat_t sp_get_spc(nvObj_t *nv) { return(get_integer(nv, spindle.state)); }
stat_t sp_set_spc(nvObj_t *nv) { return(spindle_control_immediate((spControl)nv->value_int)); }
stat_t sp_get_sps(nvObj_t *nv) { return(get_float(nv, spindle.speed)); }
stat_t sp_set_sps(nvObj_t *nv) { return(spindle_speed_immediate(nv->value_flt)); }

/****************************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ****************************************************************************************/

#ifdef __TEXT_MODE

const char fmt_spc[]  = "[spc]  spindle control:%12d [0=OFF,1=CW,2=CCW]\n";
const char fmt_sps[]  = "[sps]  spindle speed:%14.0f rpm\n";
const char fmt_spmo[] = "[spmo] spindle mode%16d [0=disabled,1=plan-to-stop,2=continuous]\n";
const char fmt_spep[] = "[spep] spindle enable polarity%5d [0=active_low,1=active_high]\n";
const char fmt_spdp[] = "[spdp] spindle direction polarity%2d [0=CW_low,1=CW_high]\n";
const char fmt_spph[] = "[spph] spindle pause on hold%7d [0=no,1=pause_on_hold]\n";
const char fmt_spde[] = "[spde] spindle spinup delay%10.1f seconds\n";
const char fmt_spsn[] = "[spsn] spindle speed min%14.2f rpm\n";
const char fmt_spsm[] = "[spsm] spindle speed max%14.2f rpm\n";
const char fmt_spoe[] = "[spoe] spindle speed override ena%2d [0=disable,1=enable]\n";
const char fmt_spo[]  = "[spo]  spindle speed override%10.3f [0.050 < spo < 2.000]\n";

void sp_print_spc(nvObj_t *nv)  { text_print(nv, fmt_spc);}     // TYPE_INT
void sp_print_sps(nvObj_t *nv)  { text_print(nv, fmt_sps);}     // TYPE_FLOAT
void sp_print_spmo(nvObj_t *nv) { text_print(nv, fmt_spmo);}    // TYPE_INT
void sp_print_spep(nvObj_t *nv) { text_print(nv, fmt_spep);}    // TYPE_INT
void sp_print_spdp(nvObj_t *nv) { text_print(nv, fmt_spdp);}    // TYPE_INT
void sp_print_spph(nvObj_t *nv) { text_print(nv, fmt_spph);}    // TYPE_INT
void sp_print_spde(nvObj_t *nv) { text_print(nv, fmt_spde);}    // TYPE_FLOAT
void sp_print_spsn(nvObj_t *nv) { text_print(nv, fmt_spsn);}    // TYPE_FLOAT
void sp_print_spsm(nvObj_t *nv) { text_print(nv, fmt_spsm);}    // TYPE_FLOAT
void sp_print_spoe(nvObj_t *nv) { text_print(nv, fmt_spoe);}    // TYPE INT
void sp_print_spo(nvObj_t *nv)  { text_print(nv, fmt_spo);}     // TYPE FLOAT

#endif // __TEXT_MODE
