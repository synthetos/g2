/*
 * spindle.cpp - canonical machine spindle driver
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2019 Alden S. Hart, Jr.
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

#include "planner.h"            // for mp_queue_command()
#include "spindle.h"

ToolHead *active_toolhead = nullptr;
bool spindle_pause_enabled = true;

/****************************************************************************************
 * toolhead_for_tool(uint8_t tool) - return the correct toolhead for the tool number
 * spindle_init() - init the spindle / toolhead subsystem
 * spindle_set_toolhead(ToolHead *toolhead) - set the active toolhead
 * spindle_reset() - stop spindle, set speed to zero, and reset values
 */

void spindle_init() {
    // active_toolhead->init();
}

void spindle_set_toolhead(ToolHead *toolhead) {
    active_toolhead = toolhead;
    active_toolhead->reset();
}

void spindle_reset() {
    active_toolhead->reset();
}

/*
 * spindle_stop();
 * spindle_pause();
 * spindle_resume();
 * spindle_set_speed(float speed);                // S parameter - returns STAT_EAGAIN if a command should be queued
 * float  spindle_get_speed();                    // return current speed - in the same units as the S parameter
 * spindle_set_direction(spDirection direction);  // M3/M4/M5 - - returns STAT_EAGAIN if a command should be queued
 * spDirection spindle_get_direction();           // return if any fo M3/M4/M5 are active (actual, not gcode model)
 * bool is_spindle_ready_to_resume(); // if the spindle can resume at this time, return true
 * bool is_spindle_on_or_paused(); // returns if the spindle is on or paused - IOW would it try to resume from feedhold
*/

void spindle_stop() {
    cm->gm.spindle_direction = SPINDLE_OFF;
    cm->gm.spindle_speed = 0;
    if (active_toolhead) {
        active_toolhead->stop();
    }
}
void spindle_pause() {
    if (spindle_pause_enabled && active_toolhead) {
        active_toolhead->pause();
    }
}
void spindle_resume() {
    if (spindle_pause_enabled && active_toolhead) {
        active_toolhead->resume();
    }
}

// A command for placing in the queue, which forces a PTS (plan-to-stop) as well as calls active_toolhead->engage()
static void _exec_spindle_control(float *, bool *) {
    // not really anything to do here - engage() should have just been called
}

stat_t spindle_set_speed(float speed) {
    cm->gm.spindle_speed = speed;

    if (active_toolhead && active_toolhead->set_speed(speed) == true) {
        mp_queue_command(_exec_spindle_control, nullptr, nullptr);
    }

    return (STAT_OK);
}
float spindle_get_speed() {
    if (active_toolhead) { return active_toolhead->get_speed(); }
    return cm->gm.spindle_speed; // if there's not active toolhead, return what the gcode model has
}

stat_t spindle_set_direction(spDirection direction)
{
    cm->gm.spindle_direction = direction;

    if (active_toolhead && active_toolhead->set_direction(direction) == true) {
        mp_queue_command(_exec_spindle_control, nullptr, nullptr);
    }

    return (STAT_OK);
}
spDirection spindle_get_direction() {
    if (active_toolhead) { return active_toolhead->get_direction(); }
    return cm->gm.spindle_direction; // if there's not active toolhead, return what the gcode model has
}

void spindle_engage(const GCodeState_t &gm) {
    if (active_toolhead) { active_toolhead->engage(gm); }
}

bool is_spindle_ready_to_resume() {
    if (active_toolhead) { return active_toolhead->ready_to_resume(); }
    return true;
}
bool is_spindle_on_or_paused() {
    if (active_toolhead) { return active_toolhead->is_on(); }
    return cm->gm.spindle_direction != SPINDLE_OFF;
}
bool is_a_toolhead_busy() {
    // TODO: look at more than just one toolhead
    if (active_toolhead) {
        return active_toolhead->busy();
    }
    return false;
}

/****************************************************************************************
 * spindle_override_control()
 * spindle_start_override()
 * spindle_end_override()
 */

// stat_t spindle_override_control(const float P_word, const bool P_flag) // M51
// {
//     bool new_enable = true;
//     bool new_override = false;
//     if (P_flag) {                           // if parameter is present in Gcode block
//         if (fp_ZERO(P_word)) {
//             new_enable = false;             // P0 disables override
//         } else {
//             if (P_word < SPINDLE_OVERRIDE_MIN) {
//                 return (STAT_INPUT_LESS_THAN_MIN_VALUE);
//             }
//             if (P_word > SPINDLE_OVERRIDE_MAX) {
//                 return (STAT_INPUT_EXCEEDS_MAX_VALUE);
//             }
//             spindle.override_factor = P_word;    // P word is valid, store it.
//             new_override = true;
//         }
//     }
//     if (cm->gmx.m48_enable) {               // if master enable is ON
//         if (new_enable && (new_override || !spindle.override_enable)) {   // 3 cases to start a ramp
//             spindle_start_override(SPINDLE_OVERRIDE_RAMP_TIME, spindle.override_factor);
//         } else if (spindle.override_enable && !new_enable) {              // case to turn off the ramp
//             spindle_end_override(SPINDLE_OVERRIDE_RAMP_TIME);
//         }
//     }
//     spindle.override_enable = new_enable;        // always update the enable state
//     return (STAT_OK);
// }

// void spindle_start_override(const float ramp_time, const float override_factor)
// {
//     return;
// }

// void spindle_end_override(const float ramp_time)
// {
//     return;
// }

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

stat_t sp_get_spep(nvObj_t *nv) { return(get_integer(nv, -1)); } // moved to gpio controls
stat_t sp_set_spep(nvObj_t *nv) { // moved to gpio controls
    // stat_t status = set_integer(nv, (uint8_t &)spindle.enable_polarity, 0, 1);
    // spindle_enable_output->setPolarity((ioPolarity)spindle.enable_polarity);
    // spindle_stop(); // stop spindle and apply new settings
    return (STAT_OK);
}

stat_t sp_get_spdp(nvObj_t *nv) { return(get_integer(nv, -1)); } // moved to gpio controls
stat_t sp_set_spdp(nvObj_t *nv) { // moved to gpio controls
    // stat_t status = set_integer(nv, (uint8_t &)spindle.dir_polarity, 0, 1);
    // spindle_direction_output->setPolarity((ioPolarity)spindle.dir_polarity);
    // spindle_stop(); // stop spindle and apply new settings
    return (STAT_OK);
}

stat_t sp_get_spph(nvObj_t *nv) { return(get_integer(nv, spindle_pause_enabled)); }
stat_t sp_set_spph(nvObj_t *nv) { return(set_integer(nv, (uint8_t &)spindle_pause_enabled, 0, 1)); }

/*
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
*/

stat_t sp_get_spde(nvObj_t *nv) { return(get_float(nv, 0)); }
stat_t sp_set_spde(nvObj_t *nv) { return(STAT_OK); }

stat_t sp_get_spsn(nvObj_t *nv) { return(get_float(nv, SPINDLE_SPEED_MIN)); }
stat_t sp_set_spsn(nvObj_t *nv) { return(STAT_OK); }
stat_t sp_get_spsm(nvObj_t *nv) { return(get_float(nv, SPINDLE_SPEED_MAX)); }
stat_t sp_set_spsm(nvObj_t *nv) { return(STAT_OK); }

stat_t sp_get_spoe(nvObj_t *nv) { return(get_integer(nv, 0)); }
stat_t sp_set_spoe(nvObj_t *nv) { return(STAT_OK); }
stat_t sp_get_spo(nvObj_t *nv) { return(get_float(nv, 1.0)); }
stat_t sp_set_spo(nvObj_t *nv) { return(STAT_OK); }

// These are provided as a way to view and control spindles without using M commands
stat_t sp_get_spc(nvObj_t *nv) { return(get_integer(nv, spindle_get_direction())); }
stat_t sp_set_spc(nvObj_t *nv) { return(spindle_set_direction((spDirection)nv->value_int)); }
stat_t sp_get_sps(nvObj_t *nv) { return(get_float(nv, spindle_get_speed())); }
stat_t sp_set_sps(nvObj_t *nv) { return(spindle_set_speed(nv->value_flt)); }


/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/
/*
 * pwm_get_*() - get generic PWM parameter and reset PWM channels
 * pwm_set_*() - set generic PWM parameter and reset PWM channels
 *
 */

stat_t pwm_get_p1frq(nvObj_t *nv) {
    if (active_toolhead) {
        nv->value_flt = active_toolhead->get_frequency();
        nv->valuetype = TYPE_FLOAT;
    }
    else {
        nv->valuetype = TYPE_NULL;
    }
    return (STAT_OK);
}
stat_t pwm_set_p1frq(nvObj_t *nv) {
    if (active_toolhead) { active_toolhead->set_frequency(nv->value_flt); }
    return (STAT_OK);
}
stat_t pwm_get_p1csl(nvObj_t *nv) {
    if (active_toolhead) {
        nv->value_flt = active_toolhead->get_cw_speed_lo();
        nv->valuetype = TYPE_FLOAT;
    }
    else {
        nv->valuetype = TYPE_NULL;
    }
    return (STAT_OK);
}
stat_t pwm_set_p1csl(nvObj_t *nv) {
    if (active_toolhead) { active_toolhead->set_cw_speed_lo(nv->value_flt); }
    return (STAT_OK);
}
stat_t pwm_get_p1csh(nvObj_t *nv) {
    if (active_toolhead) {
        nv->value_flt = active_toolhead->get_cw_speed_hi();
        nv->valuetype = TYPE_FLOAT;
    }
    else {
        nv->valuetype = TYPE_NULL;
    }
    return (STAT_OK);
}
stat_t pwm_set_p1csh(nvObj_t *nv) {
    if (active_toolhead) { active_toolhead->set_cw_speed_hi(nv->value_flt); }
    return (STAT_OK);
}
stat_t pwm_get_p1cpl(nvObj_t *nv) {
    if (active_toolhead) {
        nv->value_flt = active_toolhead->get_cw_phase_lo();
        nv->valuetype = TYPE_FLOAT;
    }
    else {
        nv->valuetype = TYPE_NULL;
    }
    return (STAT_OK);
}
stat_t pwm_set_p1cpl(nvObj_t *nv) {
    if (active_toolhead) { active_toolhead->set_cw_phase_lo(nv->value_flt); }
    return (STAT_OK);
}
stat_t pwm_get_p1cph(nvObj_t *nv) {
    if (active_toolhead) {
        nv->value_flt = active_toolhead->get_cw_phase_hi();
        nv->valuetype = TYPE_FLOAT;
    }
    else {
        nv->valuetype = TYPE_NULL;
    }
    return (STAT_OK);
}
stat_t pwm_set_p1cph(nvObj_t *nv) {
    if (active_toolhead) { active_toolhead->set_cw_phase_hi(nv->value_flt); }
    return (STAT_OK);
}
stat_t pwm_get_p1wsl(nvObj_t *nv) {
    if (active_toolhead) {
        nv->value_flt = active_toolhead->get_ccw_speed_lo();
        nv->valuetype = TYPE_FLOAT;
    }
    else {
        nv->valuetype = TYPE_NULL;
    }
    return (STAT_OK);
}
stat_t pwm_set_p1wsl(nvObj_t *nv) {
    if (active_toolhead) { active_toolhead->set_ccw_speed_lo(nv->value_flt); }
    return (STAT_OK);
}
stat_t pwm_get_p1wsh(nvObj_t *nv) {
    if (active_toolhead) {
        nv->value_flt = active_toolhead->get_ccw_speed_hi();
        nv->valuetype = TYPE_FLOAT;
    }
    else {
        nv->valuetype = TYPE_NULL;
    }
    return (STAT_OK);
}
stat_t pwm_set_p1wsh(nvObj_t *nv) {
    if (active_toolhead) { active_toolhead->set_ccw_speed_hi(nv->value_flt); }
    return (STAT_OK);
}
stat_t pwm_get_p1wpl(nvObj_t *nv) {
    if (active_toolhead) {
        nv->value_flt = active_toolhead->get_ccw_phase_lo();
        nv->valuetype = TYPE_FLOAT;
    }
    else {
        nv->valuetype = TYPE_NULL;
    }
    return (STAT_OK);
}
stat_t pwm_set_p1wpl(nvObj_t *nv) {
    if (active_toolhead) { active_toolhead->set_ccw_phase_lo(nv->value_flt); }
    return (STAT_OK);
}
stat_t pwm_get_p1wph(nvObj_t *nv) {
    if (active_toolhead) {
        nv->value_flt = active_toolhead->get_ccw_phase_hi();
        nv->valuetype = TYPE_FLOAT;
    }
    else {
        nv->valuetype = TYPE_NULL;
    }
    return (STAT_OK);
}
stat_t pwm_set_p1wph(nvObj_t *nv) {
    if (active_toolhead) { active_toolhead->set_ccw_phase_hi(nv->value_flt); }
    return (STAT_OK);
}
stat_t pwm_get_p1pof(nvObj_t *nv) {
    if (active_toolhead) {
        nv->value_flt = active_toolhead->get_phase_off();
        nv->valuetype = TYPE_FLOAT;
    }
    else {
        nv->valuetype = TYPE_NULL;
    }
    return (STAT_OK);
}
stat_t pwm_set_p1pof(nvObj_t *nv) {
    if (active_toolhead) { active_toolhead->set_phase_off(nv->value_flt); }
    return (STAT_OK);
}

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

static const char fmt_p1frq[] = "[p1frq] pwm frequency%18.0f Hz\n";
static const char fmt_p1csl[] = "[p1csl] pwm cw speed lo%16.0f RPM\n";
static const char fmt_p1csh[] = "[p1csh] pwm cw speed hi%16.0f RPM\n";
static const char fmt_p1cpl[] = "[p1cpl] pwm cw phase lo%16.3f [0..1]\n";
static const char fmt_p1cph[] = "[p1cph] pwm cw phase hi%16.3f [0..1]\n";
static const char fmt_p1wsl[] = "[p1wsl] pwm ccw speed lo%15.0f RPM\n";
static const char fmt_p1wsh[] = "[p1wsh] pwm ccw speed hi%15.0f RPM\n";
static const char fmt_p1wpl[] = "[p1wpl] pwm ccw phase lo%15.3f [0..1]\n";
static const char fmt_p1wph[] = "[p1wph] pwm ccw phase hi%15.3f [0..1]\n";
static const char fmt_p1pof[] = "[p1pof] pwm phase off%18.3f [0..1]\n";

void pwm_print_p1frq(nvObj_t *nv) { text_print(nv, fmt_p1frq);}     // all TYPE_FLOAT
void pwm_print_p1csl(nvObj_t *nv) { text_print(nv, fmt_p1csl);}
void pwm_print_p1csh(nvObj_t *nv) { text_print(nv, fmt_p1csh);}
void pwm_print_p1cpl(nvObj_t *nv) { text_print(nv, fmt_p1cpl);}
void pwm_print_p1cph(nvObj_t *nv) { text_print(nv, fmt_p1cph);}
void pwm_print_p1wsl(nvObj_t *nv) { text_print(nv, fmt_p1wsl);}
void pwm_print_p1wsh(nvObj_t *nv) { text_print(nv, fmt_p1wsh);}
void pwm_print_p1wpl(nvObj_t *nv) { text_print(nv, fmt_p1wpl);}
void pwm_print_p1wph(nvObj_t *nv) { text_print(nv, fmt_p1wph);}
void pwm_print_p1pof(nvObj_t *nv) { text_print(nv, fmt_p1pof);}

#endif // __TEXT_MODE

#include "settings.h"

constexpr cfgItem_t spindle_config_items_1[] = {
    // Spindle functions
    { "sp","spmo", _i0,  0, sp_print_spmo, get_nul,     set_nul,     nullptr, 0 }, // keeping this key around, but it returns null and does nothing
    { "sp","spph", _bip, 0, sp_print_spph, sp_get_spph, sp_set_spph, nullptr, SPINDLE_PAUSE_ON_HOLD },
    { "sp","spde", _fip, 2, sp_print_spde, sp_get_spde, sp_set_spde, nullptr, SPINDLE_SPINUP_DELAY },
    { "sp","spsn", _fip, 2, sp_print_spsn, sp_get_spsn, sp_set_spsn, nullptr, SPINDLE_SPEED_MIN},
    { "sp","spsm", _fip, 2, sp_print_spsm, sp_get_spsm, sp_set_spsm, nullptr, SPINDLE_SPEED_MAX},
    { "sp","spep", _iip, 0, sp_print_spep, sp_get_spep, sp_set_spep, nullptr, SPINDLE_ENABLE_POLARITY },
    { "sp","spdp", _iip, 0, sp_print_spdp, sp_get_spdp, sp_set_spdp, nullptr, SPINDLE_DIR_POLARITY },
    { "sp","spoe", _bip, 0, sp_print_spoe, sp_get_spoe, sp_set_spoe, nullptr, 0}, // SPINDLE_OVERRIDE_ENABLE
    { "sp","spo",  _fip, 3, sp_print_spo,  sp_get_spo,  sp_set_spo,  nullptr, 1.0}, // SPINDLE_OVERRIDE_FACTOR
    { "sp","spc",  _i0,  0, sp_print_spc,  sp_get_spc,  sp_set_spc,  nullptr, 0 },   // spindle state
    { "sp","sps",  _f0,  0, sp_print_sps,  sp_get_sps,  sp_set_sps,  nullptr, 0 },   // spindle speed
};
constexpr cfgSubtableFromStaticArray spindle_config_1 {spindle_config_items_1};
const configSubtable * const getSpindleConfig_1() { return &spindle_config_1; }

constexpr cfgItem_t p1_config_items_1[] = {
    // PWM settings
    { "p1","p1frq",_fip, 0, pwm_print_p1frq, pwm_get_p1frq, pwm_set_p1frq, nullptr, P1_PWM_FREQUENCY },
    { "p1","p1csl",_fip, 0, pwm_print_p1csl, pwm_get_p1csl, pwm_set_p1csl, nullptr, P1_CW_SPEED_LO },
    { "p1","p1csh",_fip, 0, pwm_print_p1csh, pwm_get_p1csh, pwm_set_p1csh, nullptr, P1_CW_SPEED_HI },
    { "p1","p1cpl",_fip, 3, pwm_print_p1cpl, pwm_get_p1cpl, pwm_set_p1cpl, nullptr, P1_CW_PHASE_LO },
    { "p1","p1cph",_fip, 3, pwm_print_p1cph, pwm_get_p1cph, pwm_set_p1cph, nullptr, P1_CW_PHASE_HI },
    { "p1","p1wsl",_fip, 0, pwm_print_p1wsl, pwm_get_p1wsl, pwm_set_p1wsl, nullptr, P1_CCW_SPEED_LO },
    { "p1","p1wsh",_fip, 0, pwm_print_p1wsh, pwm_get_p1wsh, pwm_set_p1wsh, nullptr, P1_CCW_SPEED_HI },
    { "p1","p1wpl",_fip, 3, pwm_print_p1wpl, pwm_get_p1wpl, pwm_set_p1wpl, nullptr, P1_CCW_PHASE_LO },
    { "p1","p1wph",_fip, 3, pwm_print_p1wph, pwm_get_p1wph, pwm_set_p1wph, nullptr, P1_CCW_PHASE_HI },
    { "p1","p1pof",_fip, 3, pwm_print_p1pof, pwm_get_p1pof, pwm_set_p1pof, nullptr, P1_PWM_PHASE_OFF },
};
constexpr cfgSubtableFromStaticArray p1_config_1 {p1_config_items_1};
const configSubtable * const getP1Config_1() { return &p1_config_1; }
