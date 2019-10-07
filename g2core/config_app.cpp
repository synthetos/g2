/*
 * config_app.cpp - application-specific part of configuration data
 * This file is part of the g2core project
 *
 * Copyright (c) 2013 - 2018 Alden S. Hart, Jr.
 * Copyright (c) 2016 - 2018 Robert Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* This file contains application specific data for the config system:
 *  - application-specific functions and function prototypes
 *  - application-specific message and print format strings
 *  - application-specific config array
 *  - any other application-specific data or functions
 *
 * See config_app.h for a detailed description of config objects and the config table
 */

#include "g2core.h"  // #1
#include "config.h"  // #2
#include "controller.h"
#include "canonical_machine.h"
#include "gcode.h"
#include "json_parser.h"
#include "text_parser.h"
#include "settings.h"
#include "planner.h"
#include "plan_arc.h"
#include "stepper.h"
#include "gpio.h"
#include "spindle.h"
#include "temperature.h"
#include "coolant.h"
#include "pwm.h"
#include "report.h"
#include "hardware.h"
#include "util.h"
#include "help.h"
#include "xio.h"

/*** structures ***/

cfgParameters_t cfg;         // application specific configuration parameters

/***********************************************************************************
 **** application-specific internal functions **************************************
 ***********************************************************************************/
// See config.cpp/.h for generic variables and functions that are not specific to
// g2core or the motion control application domain

// helpers (most helpers are defined immediately above their usage so they don't need prototypes here)

static stat_t _do_motors(nvObj_t *nv);      // print parameters for all motor groups
static stat_t _do_axes(nvObj_t *nv);        // print parameters for all axis groups
static stat_t _do_offsets(nvObj_t *nv);     // print offset parameters for G54-G59,G92, G28, G30
static stat_t _do_inputs(nvObj_t *nv);      // print parameters for all input groups
static stat_t _do_outputs(nvObj_t *nv);     // print parameters for all output groups
static stat_t _do_all(nvObj_t *nv);         // print all parameters

// communications settings and functions

static stat_t get_rx(nvObj_t *nv);          // get bytes in RX buffer
static stat_t get_tick(nvObj_t *nv);        // get system tick count

/***********************************************************************************
 **** CONFIG TABLE  ****************************************************************
 ***********************************************************************************
 *
 *  Read the notes in config.h first
 *
 *  NOTES AND CAVEATS
 *
 *  - Token matching occurs from the most specific to the least specific. This means
 *    that if shorter tokens overlap longer ones the longer one must precede the
 *    shorter one. E.g. "gco" needs to come before "gc"
 *
 *  - Mark group strings for entries that have no group as nul -->  "".
 *    This is important for group expansion.
 *
 *  - Groups do not have groups. Neither do uber-groups, e.g.
 *    'x' is --> { "", "x",    and 'm' is --> { "", "m",
 *
 *  - Be careful not to define groups longer than GROUP_LEN [4] and tokens longer
 *    than TOKEN_LEN [6]. (See config.h for lengths). The combined group + token
 *    cannot exceed TOKEN_LEN. String functions working on the table assume these
 *    rules are followed and do not check lengths or perform other validation.
 *
 *  - The precision value 'p' only affects JSON responses. You need to also set
 *    the %f in the corresponding format string to set text mode display precision
 *
 *  - Unit conversions are now conditional, and handled by convert_incoming_float() 
 *    and convert_outgoing_float(). Apply conversion flags to all axes, not just linear,
 *    as rotary axes may be treated as linear if in radius mode, so the flag is needed.
 */
const cfgItem_t cfgArray[] = {

    // group token flags p, print_func,   get_func,   set_func, get/set target,    default value
    { "sys", "fb", _fn,  2, hw_print_fb,  hw_get_fb,  set_ro, nullptr, 0 },   // MUST BE FIRST for persistence checking!
    { "sys", "fv", _fn,  2, hw_print_fv,  hw_get_fv,  set_ro, nullptr, 0 },
    { "sys", "fbs",_sn,  0, hw_print_fbs, hw_get_fbs, set_ro, nullptr, 0 },
    { "sys", "fbc",_sn,  0, hw_print_fbc, hw_get_fbc, set_ro, nullptr, 0 },
    { "sys", "hp", _sn,  0, hw_print_hp,  hw_get_hp,  set_ro, nullptr, 0 },
    { "sys", "hv", _sn,  0, hw_print_hv,  hw_get_hv,  set_ro, nullptr, 0 },
    { "sys", "id", _sn,  0, hw_print_id,  hw_get_id,  set_ro, nullptr, 0 },   // device ID (ASCII signature)

    // dynamic model attributes for reporting purposes (up front for speed)
    { "", "stat",_i0, 0, cm_print_stat, cm_get_stat, set_ro, nullptr, 0 },    // combined machine state
    { "","stat2",_i0, 0, cm_print_stat, cm_get_stat2,set_ro, nullptr, 0 },    // combined machine state
    { "", "n",   _ii, 0, cm_print_line, cm_get_mline,set_noop,nullptr,0 },    // Model line number
    { "", "line",_ii, 0, cm_print_line, cm_get_line, set_ro, nullptr, 0 },    // Active line number - model or runtime line number
    { "", "vel", _f0, 2, cm_print_vel,  cm_get_vel,  set_ro, nullptr, 0 },    // current velocity
    { "", "feed",_f0, 2, cm_print_feed, cm_get_feed, set_ro, nullptr, 0 },    // feed rate
    { "", "macs",_i0, 0, cm_print_macs, cm_get_macs, set_ro, nullptr, 0 },    // raw machine state
    { "", "cycs",_i0, 0, cm_print_cycs, cm_get_cycs, set_ro, nullptr, 0 },    // cycle state
    { "", "mots",_i0, 0, cm_print_mots, cm_get_mots, set_ro, nullptr, 0 },    // motion state
    { "", "hold",_i0, 0, cm_print_hold, cm_get_hold, set_ro, nullptr, 0 },    // feedhold state
    { "", "unit",_i0, 0, cm_print_unit, cm_get_unit, set_ro, nullptr, 0 },    // units mode
    { "", "coor",_i0, 0, cm_print_coor, cm_get_coor, set_ro, nullptr, 0 },    // coordinate system
    { "", "momo",_i0, 0, cm_print_momo, cm_get_momo, set_ro, nullptr, 0 },    // motion mode
    { "", "plan",_i0, 0, cm_print_plan, cm_get_plan, set_ro, nullptr, 0 },    // plane select
    { "", "path",_i0, 0, cm_print_path, cm_get_path, set_ro, nullptr, 0 },    // path control mode
    { "", "dist",_i0, 0, cm_print_dist, cm_get_dist, set_ro, nullptr, 0 },    // distance mode
    { "", "admo",_i0, 0, cm_print_admo, cm_get_admo, set_ro, nullptr, 0 },    // arc distance mode
    { "", "frmo",_i0, 0, cm_print_frmo, cm_get_frmo, set_ro, nullptr, 0 },    // feed rate mode
    { "", "tool",_i0, 0, cm_print_tool, cm_get_toolv,set_ro, nullptr, 0 },    // active tool
    { "", "g92e",_i0, 0, cm_print_g92e, cm_get_g92e, set_ro, nullptr, 0 },    // G92 enable state
#ifdef TEMPORARY_HAS_LEDS
    { "", "_leds",_i0, 0, tx_print_nul, _get_leds,_set_leds, nullptr, 0 },    // TEMPORARY - change LEDs
#endif

    { "mpo","mpox",_f0, 5, cm_print_mpo, cm_get_mpo, set_ro, nullptr, 0 },    // X machine position
    { "mpo","mpoy",_f0, 5, cm_print_mpo, cm_get_mpo, set_ro, nullptr, 0 },    // Y machine position
    { "mpo","mpoz",_f0, 5, cm_print_mpo, cm_get_mpo, set_ro, nullptr, 0 },    // Z machine position
    { "mpo","mpou",_f0, 5, cm_print_mpo, cm_get_mpo, set_ro, nullptr, 0 },    // U machine position
    { "mpo","mpov",_f0, 5, cm_print_mpo, cm_get_mpo, set_ro, nullptr, 0 },    // V machine position
    { "mpo","mpow",_f0, 5, cm_print_mpo, cm_get_mpo, set_ro, nullptr, 0 },    // W machine position
    { "mpo","mpoa",_f0, 5, cm_print_mpo, cm_get_mpo, set_ro, nullptr, 0 },    // A machine position
    { "mpo","mpob",_f0, 5, cm_print_mpo, cm_get_mpo, set_ro, nullptr, 0 },    // B machine position
    { "mpo","mpoc",_f0, 5, cm_print_mpo, cm_get_mpo, set_ro, nullptr, 0 },    // C machine position

    { "pos","posx",_f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0 },    // X work position
    { "pos","posy",_f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0 },    // Y work position
    { "pos","posz",_f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0 },    // Z work position
    { "pos","posu",_f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0 },    // U work position
    { "pos","posv",_f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0 },    // V work position
    { "pos","posw",_f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0 },    // W work position
    { "pos","posa",_f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0 },    // A work position
    { "pos","posb",_f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0 },    // B work position
    { "pos","posc",_f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0 },    // C work position

    { "ofs","ofsx",_f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0 },    // X work offset
    { "ofs","ofsy",_f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0 },    // Y work offset
    { "ofs","ofsz",_f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0 },    // Z work offset
    { "ofs","ofsu",_f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0 },    // U work offset
    { "ofs","ofsv",_f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0 },    // V work offset
    { "ofs","ofsw",_f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0 },    // W work offset
    { "ofs","ofsa",_f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0 },    // A work offset
    { "ofs","ofsb",_f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0 },    // B work offset
    { "ofs","ofsc",_f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0 },    // C work offset

    { "hom","home",_i0, 0, cm_print_home,cm_get_home,cm_set_home,nullptr,0 }, // homing state, invoke homing cycle
    { "hom","homx",_i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0 },    // X homed - Homing status group
    { "hom","homy",_i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0 },    // Y homed
    { "hom","homz",_i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0 },    // Z homed
    { "hom","homu",_i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0 },    // U homed
    { "hom","homv",_i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0 },    // V homed
    { "hom","homw",_i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0 },    // W homed
    { "hom","homa",_i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0 },    // A homed
    { "hom","homb",_i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0 },    // B homed
    { "hom","homc",_i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0 },    // C homed

    { "prb","prbe",_i0, 0, tx_print_nul, cm_get_prob,set_ro, nullptr, 0 },    // probing state
    { "prb","prbx",_f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0 },    // X probe results
    { "prb","prby",_f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0 },    // Y probe results
    { "prb","prbz",_f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0 },    // Z probe results
    { "prb","prbu",_f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0 },    // U probe results
    { "prb","prbv",_f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0 },    // V probe results
    { "prb","prbw",_f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0 },    // W probe results
    { "prb","prba",_f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0 },    // A probe results
    { "prb","prbb",_f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0 },    // B probe results
    { "prb","prbc",_f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0 },    // C probe results

    { "jog","jogx",_f0, 0, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},    // jog in X axis
    { "jog","jogy",_f0, 0, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},    // jog in Y axis
    { "jog","jogz",_f0, 0, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},    // jog in Z axis
    { "jog","jogu",_f0, 0, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},    // jog in U axis
    { "jog","jogv",_f0, 0, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},    // jog in V axis
    { "jog","jogw",_f0, 0, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},    // jog in W axis
    { "jog","joga",_f0, 0, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},    // jog in A axis
    { "jog","jogb",_f0, 0, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},    // jog in B axis
    { "jog","jogc",_f0, 0, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},    // jog in C axis

	{ "pwr","pwr1",_f0, 3, st_print_pwr, st_get_pwr, set_ro, nullptr, 0},	  // motor power readouts
	{ "pwr","pwr2",_f0, 3, st_print_pwr, st_get_pwr, set_ro, nullptr, 0},
#if (MOTORS > 2)
	{ "pwr","pwr3",_f0, 3, st_print_pwr, st_get_pwr, set_ro, nullptr, 0},
#endif
#if (MOTORS > 3)
	{ "pwr","pwr4",_f0, 3, st_print_pwr, st_get_pwr, set_ro, nullptr, 0},
#endif
#if (MOTORS > 4)
	{ "pwr","pwr5",_f0, 3, st_print_pwr, st_get_pwr, set_ro, nullptr, 0},
#endif
#if (MOTORS > 5)
	{ "pwr","pwr6",_f0, 3, st_print_pwr, st_get_pwr, set_ro, nullptr, 0},
#endif

    // Motor parameters
    { "1","1ma",_iip, 0, st_print_ma, st_get_ma, st_set_ma, nullptr, M1_MOTOR_MAP },
    { "1","1sa",_fip, 3, st_print_sa, st_get_sa, st_set_sa, nullptr, M1_STEP_ANGLE },
    { "1","1tr",_fipc,5, st_print_tr, st_get_tr, st_set_tr, nullptr, M1_TRAVEL_PER_REV },
    { "1","1su",_fipi,5, st_print_su, st_get_su, st_set_su, nullptr, M1_STEPS_PER_UNIT },
    { "1","1mi",_iip, 0, st_print_mi, st_get_mi, st_set_mi, nullptr, M1_MICROSTEPS },
    { "1","1po",_iip, 0, st_print_po, st_get_po, st_set_po, nullptr, M1_POLARITY },
    { "1","1pm",_iip, 0, st_print_pm, st_get_pm, st_set_pm, nullptr, M1_POWER_MODE },
    { "1","1pl",_fip, 3, st_print_pl, st_get_pl, st_set_pl, nullptr, M1_POWER_LEVEL },
    { "1","1ep",_iip, 0, st_print_ep, st_get_ep, st_set_ep, nullptr, M1_ENABLE_POLARITY },
    { "1","1sp",_iip, 0, st_print_sp, st_get_sp, st_set_sp, nullptr, M1_STEP_POLARITY },
//  { "1","1pi",_fip, 3, st_print_pi, st_get_pi, st_set_pi, (float *)&st_cfg.mot[MOTOR_1].power_idle,     M1_POWER_IDLE },
//  { "1","1mt",_fip, 2, st_print_mt, st_get_mt, st_set_mt, (float *)&st_cfg.mot[MOTOR_1].motor_timeout,  M1_MOTOR_TIMEOUT },
#if (MOTORS >= 2)
    { "2","2ma",_iip, 0, st_print_ma, st_get_ma, st_set_ma, nullptr, M2_MOTOR_MAP },
    { "2","2sa",_fip, 3, st_print_sa, st_get_sa, st_set_sa, nullptr, M2_STEP_ANGLE },
    { "2","2tr",_fipc,5, st_print_tr, st_get_tr, st_set_tr, nullptr, M2_TRAVEL_PER_REV },
    { "2","2su",_fipi,5, st_print_su, st_get_su, st_set_su, nullptr, M2_STEPS_PER_UNIT },
    { "2","2mi",_iip, 0, st_print_mi, st_get_mi, st_set_mi, nullptr, M2_MICROSTEPS },
    { "2","2po",_iip, 0, st_print_po, st_get_po, st_set_po, nullptr, M2_POLARITY },
    { "2","2pm",_iip, 0, st_print_pm, st_get_pm, st_set_pm, nullptr, M2_POWER_MODE },
    { "2","2pl",_fip, 3, st_print_pl, st_get_pl, st_set_pl, nullptr, M2_POWER_LEVEL},
    { "2","2ep",_iip, 0, st_print_ep, st_get_ep, st_set_ep, nullptr, M2_ENABLE_POLARITY },
    { "2","2sp",_iip, 0, st_print_sp, st_get_sp, st_set_sp, nullptr, M2_STEP_POLARITY },
//  { "2","2pi",_fip, 3, st_print_pi, st_get_pi, st_set_pi, (float *)&st_cfg.mot[MOTOR_2].power_idle,     M2_POWER_IDLE },
//  { "2","2mt",_fip, 2, st_print_mt, st_get_mt, st_set_mt,  float *)&st_cfg.mot[MOTOR_2].motor_timeout,  M2_MOTOR_TIMEOUT },
#endif
#if (MOTORS >= 3)
    { "3","3ma",_iip, 0, st_print_ma, st_get_ma, st_set_ma, nullptr, M3_MOTOR_MAP },
    { "3","3sa",_fip, 3, st_print_sa, st_get_sa, st_set_sa, nullptr, M3_STEP_ANGLE },
    { "3","3tr",_fipc,5, st_print_tr, st_get_tr, st_set_tr, nullptr, M3_TRAVEL_PER_REV },
    { "3","3su",_fipi,5, st_print_su, st_get_su, st_set_su, nullptr, M3_STEPS_PER_UNIT },
    { "3","3mi",_iip, 0, st_print_mi, st_get_mi, st_set_mi, nullptr, M3_MICROSTEPS },
    { "3","3po",_iip, 0, st_print_po, st_get_po, st_set_po, nullptr, M3_POLARITY },
    { "3","3pm",_iip, 0, st_print_pm, st_get_pm, st_set_pm, nullptr, M3_POWER_MODE },
    { "3","3pl",_fip, 3, st_print_pl, st_get_pl, st_set_pl, nullptr, M3_POWER_LEVEL },
    { "3","3ep",_iip, 0, st_print_ep, st_get_ep, st_set_ep, nullptr, M3_ENABLE_POLARITY },
    { "3","3sp",_iip, 0, st_print_sp, st_get_sp, st_set_sp, nullptr, M3_STEP_POLARITY },
//  { "3","3pi",_fip, 3, st_print_pi, st_get_pi, st_set_pi, (float *)&st_cfg.mot[MOTOR_3].power_idle,     M3_POWER_IDLE },
//  { "3","3mt",_fip, 2, st_print_mt, st_get_mt, st_set_mt, (float *)&st_cfg.mot[MOTOR_3].motor_timeout,  M3_MOTOR_TIMEOUT },
#endif
#if (MOTORS >= 4)
    { "4","4ma",_iip, 0, st_print_ma, st_get_ma, st_set_ma, nullptr, M4_MOTOR_MAP },
    { "4","4sa",_fip, 3, st_print_sa, st_get_sa, st_set_sa, nullptr, M4_STEP_ANGLE },
    { "4","4tr",_fipc,5, st_print_tr, st_get_tr, st_set_tr, nullptr, M4_TRAVEL_PER_REV },
    { "4","4su",_fipi,5, st_print_su, st_get_su, st_set_su, nullptr, M4_STEPS_PER_UNIT },
    { "4","4mi",_iip, 0, st_print_mi, st_get_mi, st_set_mi, nullptr, M4_MICROSTEPS },
    { "4","4po",_iip, 0, st_print_po, st_get_po, st_set_po, nullptr, M4_POLARITY },
    { "4","4pm",_iip, 0, st_print_pm, st_get_pm, st_set_pm, nullptr, M4_POWER_MODE },
    { "4","4pl",_fip, 3, st_print_pl, st_get_pl, st_set_pl, nullptr, M4_POWER_LEVEL },
    { "4","4ep",_iip, 0, st_print_ep, st_get_ep, st_set_ep, nullptr, M4_ENABLE_POLARITY },
    { "4","4sp",_iip, 0, st_print_sp, st_get_sp, st_set_sp, nullptr, M4_STEP_POLARITY },
//  { "4","4pi",_fip, 3, st_print_pi, st_get_pi, st_set_pi, (float *)&st_cfg.mot[MOTOR_4].power_idle,     M4_POWER_IDLE },
//  { "4","4mt",_fip, 2, st_print_mt, st_get_mt, st_set_mt, (float *)&st_cfg.mot[MOTOR_4].motor_timeout,  M4_MOTOR_TIMEOUT },
#endif
#if (MOTORS >= 5)
    { "5","5ma",_iip, 0, st_print_ma, st_get_ma, st_set_ma, nullptr, M5_MOTOR_MAP },
    { "5","5sa",_fip, 3, st_print_sa, st_get_sa, st_set_sa, nullptr, M5_STEP_ANGLE },
    { "5","5tr",_fipc,5, st_print_tr, st_get_tr, st_set_tr, nullptr, M5_TRAVEL_PER_REV },
    { "5","5su",_fipi,5, st_print_su, st_get_su, st_set_su, nullptr, M5_STEPS_PER_UNIT },
    { "5","5mi",_iip, 0, st_print_mi, st_get_mi, st_set_mi, nullptr, M5_MICROSTEPS },
    { "5","5po",_iip, 0, st_print_po, st_get_po, st_set_po, nullptr, M5_POLARITY },
    { "5","5pm",_iip, 0, st_print_pm, st_get_pm, st_set_pm, nullptr, M5_POWER_MODE },
    { "5","5pl",_fip, 3, st_print_pl, st_get_pl, st_set_pl, nullptr, M5_POWER_LEVEL },
    { "5","5ep",_iip, 0, st_print_ep, st_get_ep, st_set_ep, nullptr, M5_ENABLE_POLARITY },
    { "5","5sp",_iip, 0, st_print_sp, st_get_sp, st_set_sp, nullptr, M5_STEP_POLARITY },
//  { "5","5pi",_fip, 3, st_print_pi, st_get_pi, st_set_pi, (float *)&st_cfg.mot[MOTOR_5].power_idle,     M5_POWER_IDLE },
//  { "5","5mt",_fip, 2, st_print_mt, get_flt, st_set_mt,   (float *)&st_cfg.mot[MOTOR_5].motor_timeout,  M5_MOTOR_TIMEOUT },
#endif
#if (MOTORS >= 6)
    { "6","6ma",_iip, 0, st_print_ma, st_get_ma, st_set_ma, nullptr, M6_MOTOR_MAP },
    { "6","6sa",_fip, 3, st_print_sa, st_get_sa, st_set_sa, nullptr, M6_STEP_ANGLE },
    { "6","6tr",_fipc,5, st_print_tr, st_get_tr, st_set_tr, nullptr, M6_TRAVEL_PER_REV },
    { "6","6su",_fipi,5, st_print_su, st_get_su, st_set_su, nullptr, M6_STEPS_PER_UNIT },
    { "6","6mi",_iip, 0, st_print_mi, st_get_mi, st_set_mi, nullptr, M6_MICROSTEPS },
    { "6","6po",_iip, 0, st_print_po, st_get_po, st_set_po, nullptr, M6_POLARITY },
    { "6","6pm",_iip, 0, st_print_pm, st_get_pm, st_set_pm, nullptr, M6_POWER_MODE },
    { "6","6pl",_fip, 3, st_print_pl, st_get_pl, st_set_pl, nullptr, M6_POWER_LEVEL },
    { "6","6ep",_iip, 0, st_print_ep, st_get_ep, st_set_ep, nullptr, M6_ENABLE_POLARITY },
    { "6","6sp",_iip, 0, st_print_sp, st_get_sp, st_set_sp, nullptr, M6_STEP_POLARITY },
//  { "6","6pi",_fip, 3, st_print_pi, st_get_pi, st_set_pi, (float *)&st_cfg.mot[MOTOR_6].power_idle,     M6_POWER_IDLE },
//  { "6","6mt",_fip, 2, st_print_mt, st_get_mt, st_set_mt, (float *)&st_cfg.mot[MOTOR_6].motor_timeout,  M6_MOTOR_TIMEOUT },
// >>>>>>> refs/heads/edge
#endif

    // Axis parameters
    { "x","xam",_iip,  0, cm_print_am, cm_get_am, cm_set_am, nullptr, X_AXIS_MODE },
    { "x","xvm",_fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, X_VELOCITY_MAX },
    { "x","xfr",_fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, X_FEEDRATE_MAX },
    { "x","xtn",_fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, X_TRAVEL_MIN },
    { "x","xtm",_fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, X_TRAVEL_MAX },
    { "x","xjm",_fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, X_JERK_MAX },
    { "x","xjh",_fipc, 0, cm_print_jh, cm_get_jh, cm_set_jh, nullptr, X_JERK_HIGH_SPEED },
    { "x","xhi",_iip,  0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, X_HOMING_INPUT },
    { "x","xhd",_iip,  0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, X_HOMING_DIRECTION },
    { "x","xsv",_fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, X_SEARCH_VELOCITY },
    { "x","xlv",_fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, X_LATCH_VELOCITY },
    { "x","xlb",_fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, X_LATCH_BACKOFF },
    { "x","xzb",_fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, X_ZERO_BACKOFF },

    { "y","yam",_iip,  0, cm_print_am, cm_get_am, cm_set_am, nullptr, Y_AXIS_MODE },
    { "y","yvm",_fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, Y_VELOCITY_MAX },
    { "y","yfr",_fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, Y_FEEDRATE_MAX },
    { "y","ytn",_fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, Y_TRAVEL_MIN },
    { "y","ytm",_fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, Y_TRAVEL_MAX },
    { "y","yjm",_fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, Y_JERK_MAX },
    { "y","yjh",_fipc, 0, cm_print_jh, cm_get_jh, cm_set_jh, nullptr, Y_JERK_HIGH_SPEED },
    { "y","yhi",_iip,  0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, Y_HOMING_INPUT },
    { "y","yhd",_iip,  0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, Y_HOMING_DIRECTION },
    { "y","ysv",_fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, Y_SEARCH_VELOCITY },
    { "y","ylv",_fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, Y_LATCH_VELOCITY },
    { "y","ylb",_fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, Y_LATCH_BACKOFF },
    { "y","yzb",_fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, Y_ZERO_BACKOFF },

    { "z","zam",_iip,  0, cm_print_am, cm_get_am, cm_set_am, nullptr, Z_AXIS_MODE },
    { "z","zvm",_fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, Z_VELOCITY_MAX },
    { "z","zfr",_fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, Z_FEEDRATE_MAX },
    { "z","ztn",_fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, Z_TRAVEL_MIN },
    { "z","ztm",_fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, Z_TRAVEL_MAX },
    { "z","zjm",_fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, Z_JERK_MAX },
    { "z","zjh",_fipc, 0, cm_print_jh, cm_get_jm, cm_set_jh, nullptr, Z_JERK_HIGH_SPEED },
    { "z","zhi",_iip,  0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, Z_HOMING_INPUT },
    { "z","zhd",_iip,  0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, Z_HOMING_DIRECTION },
    { "z","zsv",_fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, Z_SEARCH_VELOCITY },
    { "z","zlv",_fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, Z_LATCH_VELOCITY },
    { "z","zlb",_fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, Z_LATCH_BACKOFF },
    { "z","zzb",_fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, Z_ZERO_BACKOFF },

    { "u","uam",_iip,  0, cm_print_am, cm_get_am, cm_set_am, nullptr, U_AXIS_MODE },
    { "u","uvm",_fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, U_VELOCITY_MAX },
    { "u","ufr",_fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, U_FEEDRATE_MAX },
    { "u","utn",_fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, U_TRAVEL_MIN },
    { "u","utm",_fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, U_TRAVEL_MAX },
    { "u","ujm",_fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, U_JERK_MAX },
    { "u","ujh",_fipc, 0, cm_print_jh, cm_get_jh, cm_set_jh, nullptr, U_JERK_HIGH_SPEED },
    { "u","uhi",_iip,  0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, U_HOMING_INPUT },
    { "u","uhd",_iip,  0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, U_HOMING_DIRECTION },
    { "u","usv",_fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, U_SEARCH_VELOCITY },
    { "u","ulv",_fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, U_LATCH_VELOCITY },
    { "u","ulb",_fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, U_LATCH_BACKOFF },
    { "u","uzb",_fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, U_ZERO_BACKOFF },

    { "v","vam",_iip,  0, cm_print_am, cm_get_am, cm_set_am, nullptr, V_AXIS_MODE },
    { "v","vvm",_fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, V_VELOCITY_MAX },
    { "v","vfr",_fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, V_FEEDRATE_MAX },
    { "v","vtn",_fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, V_TRAVEL_MIN },
    { "v","vtm",_fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, V_TRAVEL_MAX },
    { "v","vjm",_fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, V_JERK_MAX },
    { "v","vjh",_fipc, 0, cm_print_jh, cm_get_jh, cm_set_jh, nullptr, V_JERK_HIGH_SPEED },
    { "v","vhi",_iip,  0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, V_HOMING_INPUT },
    { "v","vhd",_iip,  0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, V_HOMING_DIRECTION },
    { "v","vsv",_fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, V_SEARCH_VELOCITY },
    { "v","vlv",_fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, V_LATCH_VELOCITY },
    { "v","vlb",_fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, V_LATCH_BACKOFF },
    { "v","vzb",_fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, V_ZERO_BACKOFF },

    { "w","wam",_iip,  0, cm_print_am, cm_get_am, cm_set_am, nullptr, W_AXIS_MODE },
    { "w","wvm",_fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, W_VELOCITY_MAX },
    { "w","wfr",_fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, W_FEEDRATE_MAX },
    { "w","wtn",_fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, W_TRAVEL_MIN },
    { "w","wtm",_fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, W_TRAVEL_MAX },
    { "w","wjm",_fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, W_JERK_MAX },
    { "w","wjh",_fipc, 0, cm_print_jh, cm_get_jh, cm_set_jh, nullptr, W_JERK_HIGH_SPEED },
    { "w","whi",_iip,  0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, W_HOMING_INPUT },
    { "w","whd",_iip,  0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, W_HOMING_DIRECTION },
    { "w","wsv",_fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, W_SEARCH_VELOCITY },
    { "w","wlv",_fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, W_LATCH_VELOCITY },
    { "w","wlb",_fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, W_LATCH_BACKOFF },
    { "w","wzb",_fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, W_ZERO_BACKOFF },

    { "a","aam",_iip,  0, cm_print_am, cm_get_am, cm_set_am, nullptr, A_AXIS_MODE },
    { "a","avm",_fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, A_VELOCITY_MAX },
    { "a","afr",_fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, A_FEEDRATE_MAX },
    { "a","atn",_fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, A_TRAVEL_MIN },
    { "a","atm",_fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, A_TRAVEL_MAX },
    { "a","ajm",_fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, A_JERK_MAX },
    { "a","ajh",_fipc, 0, cm_print_jh, cm_get_jh, cm_set_jh, nullptr, A_JERK_HIGH_SPEED },
    { "a","ara",_fipc, 5, cm_print_ra, cm_get_ra, cm_set_ra, nullptr, A_RADIUS},
    { "a","ahi",_iip,  0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, A_HOMING_INPUT },
    { "a","ahd",_iip,  0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, A_HOMING_DIRECTION },
    { "a","asv",_fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, A_SEARCH_VELOCITY },
    { "a","alv",_fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, A_LATCH_VELOCITY },
    { "a","alb",_fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, A_LATCH_BACKOFF },
    { "a","azb",_fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, A_ZERO_BACKOFF },

    { "b","bam",_iip,  0, cm_print_am, cm_get_am, cm_set_am, nullptr, B_AXIS_MODE },
    { "b","bvm",_fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, B_VELOCITY_MAX },
    { "b","bfr",_fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, B_FEEDRATE_MAX },
    { "b","btn",_fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, B_TRAVEL_MIN },
    { "b","btm",_fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, B_TRAVEL_MAX },
    { "b","bjm",_fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, B_JERK_MAX },
    { "b","bjh",_fipc, 0, cm_print_jh, cm_get_jh, cm_set_jh, nullptr, B_JERK_HIGH_SPEED },
    { "b","bra",_fipc, 5, cm_print_ra, cm_get_ra, cm_set_ra, nullptr, B_RADIUS },
    { "b","bhi",_iip,  0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, B_HOMING_INPUT },
    { "b","bhd",_iip,  0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, B_HOMING_DIRECTION },
    { "b","bsv",_fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, B_SEARCH_VELOCITY },
    { "b","blv",_fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, B_LATCH_VELOCITY },
    { "b","blb",_fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, B_LATCH_BACKOFF },
    { "b","bzb",_fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, B_ZERO_BACKOFF },

    { "c","cam",_iip,  0, cm_print_am, cm_get_am, cm_set_am, nullptr, C_AXIS_MODE },
    { "c","cvm",_fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, C_VELOCITY_MAX },
    { "c","cfr",_fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, C_FEEDRATE_MAX },
    { "c","ctn",_fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, C_TRAVEL_MIN },
    { "c","ctm",_fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, C_TRAVEL_MAX },
    { "c","cjm",_fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, C_JERK_MAX },
    { "c","cjh",_fipc, 0, cm_print_jh, cm_get_jh, cm_set_jh, nullptr, C_JERK_HIGH_SPEED },
    { "c","cra",_fipc, 5, cm_print_ra, cm_get_ra, cm_set_ra, nullptr, C_RADIUS },
    { "c","chi",_iip,  0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, C_HOMING_INPUT },
    { "c","chd",_iip,  0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, C_HOMING_DIRECTION },
    { "c","csv",_fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, C_SEARCH_VELOCITY },
    { "c","clv",_fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, C_LATCH_VELOCITY },
    { "c","clb",_fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, C_LATCH_BACKOFF },
    { "c","czb",_fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, C_ZERO_BACKOFF },

    // Digital input configs
    { "di1","di1mo",_iip, 0, io_print_mo, io_get_mo, io_set_mo, nullptr, DI1_MODE },
    { "di1","di1ac",_iip, 0, io_print_ac, io_get_ac, io_set_ac, nullptr, DI1_ACTION },
    { "di1","di1fn",_iip, 0, io_print_fn, io_get_fn, io_set_fn, nullptr, DI1_FUNCTION },

    { "di2","di2mo",_iip, 0, io_print_mo, io_get_mo, io_set_mo, nullptr, DI2_MODE },
    { "di2","di2ac",_iip, 0, io_print_ac, io_get_ac, io_set_ac, nullptr, DI2_ACTION },
    { "di2","di2fn",_iip, 0, io_print_fn, io_get_fn, io_set_fn, nullptr, DI2_FUNCTION },

    { "di3","di3mo",_iip, 0, io_print_mo, io_get_mo, io_set_mo, nullptr, DI3_MODE },
    { "di3","di3ac",_iip, 0, io_print_ac, io_get_ac, io_set_ac, nullptr, DI3_ACTION },
    { "di3","di3fn",_iip, 0, io_print_fn, io_get_fn, io_set_fn, nullptr, DI3_FUNCTION },

    { "di4","di4mo",_iip, 0, io_print_mo, io_get_mo, io_set_mo, nullptr, DI4_MODE },
    { "di4","di4ac",_iip, 0, io_print_ac, io_get_ac, io_set_ac, nullptr, DI4_ACTION },
    { "di4","di4fn",_iip, 0, io_print_fn, io_get_fn, io_set_fn, nullptr, DI4_FUNCTION },

    { "di5","di5mo",_iip, 0, io_print_mo, io_get_mo, io_set_mo, nullptr, DI5_MODE },
    { "di5","di5ac",_iip, 0, io_print_ac, io_get_ac, io_set_ac, nullptr, DI5_ACTION },
    { "di5","di5fn",_iip, 0, io_print_fn, io_get_fn, io_set_fn, nullptr, DI5_FUNCTION },

    { "di6","di6mo",_iip, 0, io_print_mo, io_get_mo, io_set_mo, nullptr, DI6_MODE },
    { "di6","di6ac",_iip, 0, io_print_ac, io_get_ac, io_set_ac, nullptr, DI6_ACTION },
    { "di6","di6fn",_iip, 0, io_print_fn, io_get_fn, io_set_fn, nullptr, DI6_FUNCTION },

    { "di7","di7mo",_iip, 0, io_print_mo, io_get_mo, io_set_mo, nullptr, DI7_MODE },
    { "di7","di7ac",_iip, 0, io_print_ac, io_get_ac, io_set_ac, nullptr, DI7_ACTION },
    { "di7","di7fn",_iip, 0, io_print_fn, io_get_fn, io_set_fn, nullptr, DI7_FUNCTION },

    { "di8","di8mo",_iip, 0, io_print_mo, io_get_mo, io_set_mo, nullptr, DI8_MODE },
    { "di8","di8ac",_iip, 0, io_print_ac, io_get_ac, io_set_ac, nullptr, DI8_ACTION },
    { "di8","di8fn",_iip, 0, io_print_fn, io_get_fn, io_set_fn, nullptr, DI8_FUNCTION },
#if (D_IN_CHANNELS >= 9)
    { "di9","di9mo",_iip, 0, io_print_mo, io_get_mo, io_set_mo, nullptr, DI9_MODE },
    { "di9","di9ac",_iip, 0, io_print_ac, io_get_ac, io_set_ac, nullptr, DI9_ACTION },
    { "di9","di9fn",_iip, 0, io_print_fn, io_get_fn, io_set_fn, nullptr, DI9_FUNCTION },
#endif
#if (D_IN_CHANNELS >= 10)
    { "di10","di10mo",_iip, 0, io_print_mo, io_get_mo, io_set_mo, nullptr, DI10_MODE },
    { "di10","di10ac",_iip, 0, io_print_ac, io_get_ac, io_set_ac, nullptr, DI10_ACTION },
    { "di10","di10fn",_iip, 0, io_print_fn, io_get_fn, io_set_fn, nullptr, DI10_FUNCTION },
#endif
#if (D_IN_CHANNELS >= 11)
    { "di11","di11mo",_iip, 0, io_print_mo, io_get_mo, io_set_mo, nullptr, DI1_MODE },
    { "di11","di11ac",_iip, 0, io_print_ac, io_get_ac, io_set_ac, nullptr, DI11_ACTION },
    { "di11","di11fn",_iip, 0, io_print_fn, io_get_fn, io_set_fn, nullptr, DI11_FUNCTION },
#endif
#if (D_IN_CHANNELS >= 12)
    { "di12","di12mo",_iip, 0, io_print_mo, io_get_mo, io_set_mo, nullptr, DI12_MODE },
    { "di12","di12ac",_iip, 0, io_print_ac, io_get_ac, io_set_ac, nullptr, DI12_ACTION },
    { "di12","di12fn",_iip, 0, io_print_fn, io_get_fn, io_set_fn, nullptr, DI12_FUNCTION },
#endif

    // Digital input state readers
    { "in","in1", _i0, 0, io_print_in, io_get_input, set_ro, nullptr, 0 },
    { "in","in2", _i0, 0, io_print_in, io_get_input, set_ro, nullptr, 0 },
    { "in","in3", _i0, 0, io_print_in, io_get_input, set_ro, nullptr, 0 },
    { "in","in4", _i0, 0, io_print_in, io_get_input, set_ro, nullptr, 0 },
    { "in","in5", _i0, 0, io_print_in, io_get_input, set_ro, nullptr, 0 },
    { "in","in6", _i0, 0, io_print_in, io_get_input, set_ro, nullptr, 0 },
    { "in","in7", _i0, 0, io_print_in, io_get_input, set_ro, nullptr, 0 },
    { "in","in8", _i0, 0, io_print_in, io_get_input, set_ro, nullptr, 0 },
#if (D_IN_CHANNELS >= 9)
    { "in","in9", _i0, 0, io_print_in, io_get_input, set_ro, nullptr, 0 },
#endif
#if (D_IN_CHANNELS >= 10)
    { "in","in10", _i0, 0, io_print_in, io_get_input, set_ro, nullptr, 0 },
#endif
#if (D_IN_CHANNELS >= 11)
    { "in","in11", _i0, 0, io_print_in, io_get_input, set_ro, nullptr, 0 },
#endif
#if (D_IN_CHANNELS >= 12)
    { "in","in12", _i0, 0, io_print_in, io_get_input, set_ro, nullptr, 0 },
#endif

    // digital output configs
    { "do1", "do1mo", _iip, 0, io_print_domode, io_get_domode, io_set_domode, nullptr, DO1_MODE },
    { "do2", "do2mo", _iip, 0, io_print_domode, io_get_domode, io_set_domode, nullptr, DO2_MODE },
    { "do3", "do3mo", _iip, 0, io_print_domode, io_get_domode, io_set_domode, nullptr, DO3_MODE },
    { "do4", "do4mo", _iip, 0, io_print_domode, io_get_domode, io_set_domode, nullptr, DO4_MODE },
    { "do5", "do5mo", _iip, 0, io_print_domode, io_get_domode, io_set_domode, nullptr, DO5_MODE },
    { "do6", "do6mo", _iip, 0, io_print_domode, io_get_domode, io_set_domode, nullptr, DO6_MODE },
    { "do7", "do7mo", _iip, 0, io_print_domode, io_get_domode, io_set_domode, nullptr, DO7_MODE },
    { "do8", "do8mo", _iip, 0, io_print_domode, io_get_domode, io_set_domode, nullptr, DO8_MODE },
    { "do9", "do9mo", _iip, 0, io_print_domode, io_get_domode, io_set_domode, nullptr, DO9_MODE },
    { "do10","do10mo",_iip, 0, io_print_domode, io_get_domode, io_set_domode, nullptr, DO10_MODE },
    { "do11","do11mo",_iip, 0, io_print_domode, io_get_domode, io_set_domode, nullptr, DO11_MODE },
    { "do12","do12mo",_iip, 0, io_print_domode, io_get_domode, io_set_domode, nullptr, DO12_MODE },
    { "do13","do13mo",_iip, 0, io_print_domode, io_get_domode, io_set_domode, nullptr, DO13_MODE },

    // Digital output state readers (default to non-active)
    { "out","out1",  _i0, 2, io_print_out, io_get_output, io_set_output, nullptr, 0 },
    { "out","out2",  _i0, 2, io_print_out, io_get_output, io_set_output, nullptr, 0 },
    { "out","out3",  _i0, 2, io_print_out, io_get_output, io_set_output, nullptr, 0 },
    { "out","out4",  _i0, 2, io_print_out, io_get_output, io_set_output, nullptr, 0 },
    { "out","out5",  _i0, 2, io_print_out, io_get_output, io_set_output, nullptr, 0 },
    { "out","out6",  _i0, 2, io_print_out, io_get_output, io_set_output, nullptr, 0 },
    { "out","out7",  _i0, 2, io_print_out, io_get_output, io_set_output, nullptr, 0 },
    { "out","out8",  _i0, 2, io_print_out, io_get_output, io_set_output, nullptr, 0 },
    { "out","out9",  _i0, 2, io_print_out, io_get_output, io_set_output, nullptr, 0 },
    { "out","out10", _i0, 2, io_print_out, io_get_output, io_set_output, nullptr, 0 },
    { "out","out11", _i0, 2, io_print_out, io_get_output, io_set_output, nullptr, 0 },
    { "out","out12", _i0, 2, io_print_out, io_get_output, io_set_output, nullptr, 0 },

    // PWM settings
    { "p1","p1frq",_fip, 0, pwm_print_p1frq, get_flt, pwm_set_pwm,(float *)&pwm.c[PWM_1].frequency,    P1_PWM_FREQUENCY },
    { "p1","p1csl",_fip, 0, pwm_print_p1csl, get_flt, pwm_set_pwm,(float *)&pwm.c[PWM_1].cw_speed_lo,  P1_CW_SPEED_LO },
    { "p1","p1csh",_fip, 0, pwm_print_p1csh, get_flt, pwm_set_pwm,(float *)&pwm.c[PWM_1].cw_speed_hi,  P1_CW_SPEED_HI },
    { "p1","p1cpl",_fip, 3, pwm_print_p1cpl, get_flt, pwm_set_pwm,(float *)&pwm.c[PWM_1].cw_phase_lo,  P1_CW_PHASE_LO },
    { "p1","p1cph",_fip, 3, pwm_print_p1cph, get_flt, pwm_set_pwm,(float *)&pwm.c[PWM_1].cw_phase_hi,  P1_CW_PHASE_HI },
    { "p1","p1wsl",_fip, 0, pwm_print_p1wsl, get_flt, pwm_set_pwm,(float *)&pwm.c[PWM_1].ccw_speed_lo, P1_CCW_SPEED_LO },
    { "p1","p1wsh",_fip, 0, pwm_print_p1wsh, get_flt, pwm_set_pwm,(float *)&pwm.c[PWM_1].ccw_speed_hi, P1_CCW_SPEED_HI },
    { "p1","p1wpl",_fip, 3, pwm_print_p1wpl, get_flt, pwm_set_pwm,(float *)&pwm.c[PWM_1].ccw_phase_lo, P1_CCW_PHASE_LO },
    { "p1","p1wph",_fip, 3, pwm_print_p1wph, get_flt, pwm_set_pwm,(float *)&pwm.c[PWM_1].ccw_phase_hi, P1_CCW_PHASE_HI },
    { "p1","p1pof",_fip, 3, pwm_print_p1pof, get_flt, pwm_set_pwm,(float *)&pwm.c[PWM_1].phase_off,    P1_PWM_PHASE_OFF },

    // temperature configs - pid active values (read-only)
    // NOTICE: If you change these PID group keys, you MUST change the get/set functions too!
    { "pid1","pid1p",_fip, 3, tx_print_nul, cm_get_pid_p, set_ro, nullptr, 0 },
    { "pid1","pid1i",_fip, 5, tx_print_nul, cm_get_pid_i, set_ro, nullptr, 0 },
    { "pid1","pid1d",_fip, 5, tx_print_nul, cm_get_pid_d, set_ro, nullptr, 0 },

    { "pid2","pid2p",_fip, 3, tx_print_nul, cm_get_pid_p, set_ro, nullptr, 0 },
    { "pid2","pid2i",_fip, 5, tx_print_nul, cm_get_pid_i, set_ro, nullptr, 0 },
    { "pid2","pid2d",_fip, 5, tx_print_nul, cm_get_pid_d, set_ro, nullptr, 0 },

    { "pid3","pid3p",_fip, 3, tx_print_nul, cm_get_pid_p, set_ro, nullptr, 0 },
    { "pid3","pid3i",_fip, 5, tx_print_nul, cm_get_pid_i, set_ro, nullptr, 0 },
    { "pid3","pid3d",_fip, 5, tx_print_nul, cm_get_pid_d, set_ro, nullptr, 0 },

    // temperature configs - heater set values (read-write)
    // NOTICE: If you change these heater group keys, you MUST change the get/set functions too!
    { "he1","he1e", _bip, 0, tx_print_nul, cm_get_heater_enable,   cm_set_heater_enable,   nullptr, H1_DEFAULT_ENABLE },
    { "he1","he1at",_b0,  0, tx_print_nul, cm_get_at_temperature,  set_ro,                 nullptr, 0 },
    { "he1","he1p", _fip, 3, tx_print_nul, cm_get_heater_p,        cm_set_heater_p,        nullptr, H1_DEFAULT_P },
    { "he1","he1i", _fip, 5, tx_print_nul, cm_get_heater_i,        cm_set_heater_i,        nullptr, H1_DEFAULT_I },
    { "he1","he1d", _fip, 5, tx_print_nul, cm_get_heater_d,        cm_set_heater_d,        nullptr, H1_DEFAULT_D },
    { "he1","he1st",_fi,  1, tx_print_nul, cm_get_set_temperature, cm_set_set_temperature, nullptr, 0 },
    { "he1","he1t", _fi,  1, tx_print_nul, cm_get_temperature,     set_ro,                 nullptr, 0 },
    { "he1","he1op",_fi,  3, tx_print_nul, cm_get_heater_output,   set_ro,                 nullptr, 0 },
    { "he1","he1tr",_fi,  3, tx_print_nul, cm_get_thermistor_resistance, set_ro,           nullptr, 0 },
    { "he1","he1an",_fi,  0, tx_print_nul, cm_get_heater_adc,      set_ro,                 nullptr, 0 },
    { "he1","he1fp",_fi,  1, tx_print_nul, cm_get_fan_power,       cm_set_fan_power,       nullptr, 0 },
    { "he1","he1fm",_fi,  1, tx_print_nul, cm_get_fan_min_power,   cm_set_fan_min_power,   nullptr, 0 },
    { "he1","he1fl",_fi,  1, tx_print_nul, cm_get_fan_low_temp,    cm_set_fan_low_temp,    nullptr, 0 },
    { "he1","he1fh",_fi,  1, tx_print_nul, cm_get_fan_high_temp,   cm_set_fan_high_temp,   nullptr, 0 },

    { "he2","he2e", _iip, 0, tx_print_nul, cm_get_heater_enable,   cm_set_heater_enable,   nullptr, H2_DEFAULT_ENABLE },
    { "he2","he2at",_b0,  0, tx_print_nul, cm_get_at_temperature,  set_ro,                 nullptr, 0 },
    { "he2","he2p", _fip, 3, tx_print_nul, cm_get_heater_p,        cm_set_heater_p,        nullptr, H2_DEFAULT_P },
    { "he2","he2i", _fip, 5, tx_print_nul, cm_get_heater_i,        cm_set_heater_i,        nullptr, H2_DEFAULT_I },
    { "he2","he2d", _fip, 5, tx_print_nul, cm_get_heater_d,        cm_set_heater_d,        nullptr, H2_DEFAULT_D },
    { "he2","he2st",_fi,  0, tx_print_nul, cm_get_set_temperature, cm_set_set_temperature, nullptr, 0 },
    { "he2","he2t", _fi,  1, tx_print_nul, cm_get_temperature,     set_ro,                 nullptr, 0 },
    { "he2","he2op",_fi,  3, tx_print_nul, cm_get_heater_output,   set_ro,                 nullptr, 0 },
    { "he2","he2tr",_fi,  3, tx_print_nul, cm_get_thermistor_resistance, set_ro,           nullptr, 0 },
    { "he2","he2an",_fi,  0, tx_print_nul, cm_get_heater_adc,      set_ro,                 nullptr, 0 },
    { "he2","he2fp",_fi,  1, tx_print_nul, cm_get_fan_power,       cm_set_fan_power,       nullptr, 0 },
    { "he2","he2fm",_fi,  1, tx_print_nul, cm_get_fan_min_power,   cm_set_fan_min_power,   nullptr, 0 },
    { "he2","he2fl",_fi,  1, tx_print_nul, cm_get_fan_low_temp,    cm_set_fan_low_temp,    nullptr, 0 },
    { "he2","he2fh",_fi,  1, tx_print_nul, cm_get_fan_high_temp,   cm_set_fan_high_temp,   nullptr, 0 },

    { "he3","he3e", _iip, 0, tx_print_nul, cm_get_heater_enable,   cm_set_heater_enable,   nullptr, H3_DEFAULT_ENABLE },
    { "he3","he3at",_b0,  0, tx_print_nul, cm_get_at_temperature,  set_ro,                 nullptr, 0 },
    { "he3","he3p", _fip, 3, tx_print_nul, cm_get_heater_p,        cm_set_heater_p,        nullptr, H3_DEFAULT_P },
    { "he3","he3i", _fip, 5, tx_print_nul, cm_get_heater_i,        cm_set_heater_i,        nullptr, H3_DEFAULT_I },
    { "he3","he3d", _fip, 5, tx_print_nul, cm_get_heater_d,        cm_set_heater_d,        nullptr, H3_DEFAULT_D },
    { "he3","he3st",_fi,  0, tx_print_nul, cm_get_set_temperature, cm_set_set_temperature, nullptr, 0 },
    { "he3","he3t", _fi,  1, tx_print_nul, cm_get_temperature,     set_ro,                 nullptr, 0 },
    { "he3","he3op",_fi,  3, tx_print_nul, cm_get_heater_output,   set_ro,                 nullptr, 0 },
    { "he3","he3tr",_fi,  3, tx_print_nul, cm_get_thermistor_resistance, set_ro,           nullptr, 0 },
    { "he3","he3an",_fi,  0, tx_print_nul, cm_get_heater_adc,      set_ro,                 nullptr, 0 },
    { "he3","he3fp",_fi,  1, tx_print_nul, cm_get_fan_power,       cm_set_fan_power,       nullptr, 0 },
    { "he3","he3fm",_fi,  1, tx_print_nul, cm_get_fan_min_power,   cm_set_fan_min_power,   nullptr, 0 },
    { "he3","he3fl",_fi,  1, tx_print_nul, cm_get_fan_low_temp,    cm_set_fan_low_temp,    nullptr, 0 },
    { "he3","he3fh",_fi,  1, tx_print_nul, cm_get_fan_high_temp,   cm_set_fan_high_temp,   nullptr, 0 },

    // Coordinate system offsets (G54-G59 and G92)
    { "g54","g54x",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_X_OFFSET },
    { "g54","g54y",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_Y_OFFSET },
    { "g54","g54z",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_Z_OFFSET },
    { "g54","g54u",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_U_OFFSET },
    { "g54","g54v",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_V_OFFSET },
    { "g54","g54w",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_W_OFFSET },
    { "g54","g54a",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_A_OFFSET },
    { "g54","g54b",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_B_OFFSET },
    { "g54","g54c",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_C_OFFSET },

    { "g55","g55x",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_X_OFFSET },
    { "g55","g55y",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_Y_OFFSET },
    { "g55","g55z",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_Z_OFFSET },
    { "g55","g55u",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_U_OFFSET },
    { "g55","g55v",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_V_OFFSET },
    { "g55","g55w",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_W_OFFSET },
    { "g55","g55a",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_A_OFFSET },
    { "g55","g55b",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_B_OFFSET },
    { "g55","g55c",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_C_OFFSET },

    { "g56","g56x",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_X_OFFSET },
    { "g56","g56y",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_Y_OFFSET },
    { "g56","g56z",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_Z_OFFSET },
    { "g56","g56u",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_U_OFFSET },
    { "g56","g56v",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_V_OFFSET },
    { "g56","g56w",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_W_OFFSET },
    { "g56","g56a",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_A_OFFSET },
    { "g56","g56b",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_B_OFFSET },
    { "g56","g56c",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_C_OFFSET },

    { "g57","g57x",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_X_OFFSET },
    { "g57","g57y",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_Y_OFFSET },
    { "g57","g57z",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_Z_OFFSET },
    { "g57","g57u",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_U_OFFSET },
    { "g57","g57v",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_V_OFFSET },
    { "g57","g57w",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_W_OFFSET },
    { "g57","g57a",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_A_OFFSET },
    { "g57","g57b",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_B_OFFSET },
    { "g57","g57c",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_C_OFFSET },

    { "g58","g58x",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_X_OFFSET },
    { "g58","g58y",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_Y_OFFSET },
    { "g58","g58z",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_Z_OFFSET },
    { "g58","g58u",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_U_OFFSET },
    { "g58","g58v",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_V_OFFSET },
    { "g58","g58w",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_W_OFFSET },
    { "g58","g58a",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_A_OFFSET },
    { "g58","g58b",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_B_OFFSET },
    { "g58","g58c",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_C_OFFSET },

    { "g59","g59x",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_X_OFFSET },
    { "g59","g59y",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_Y_OFFSET },
    { "g59","g59z",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_Z_OFFSET },
    { "g59","g59u",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_U_OFFSET },
    { "g59","g59v",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_V_OFFSET },
    { "g59","g59w",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_W_OFFSET },
    { "g59","g59a",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_A_OFFSET },
    { "g59","g59b",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_B_OFFSET },
    { "g59","g59c",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_C_OFFSET },

    { "g92","g92x",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },// G92 handled differently
    { "g92","g92y",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },
    { "g92","g92z",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },
    { "g92","g92u",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },
    { "g92","g92v",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },
    { "g92","g92w",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },
    { "g92","g92a",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },
    { "g92","g92b",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },
    { "g92","g92c",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },

    // Coordinate positions (G28, G30)
    { "g28","g28x",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },// g28 handled differently
    { "g28","g28y",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },
    { "g28","g28z",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },
    { "g28","g28u",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },
    { "g28","g28v",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },
    { "g28","g28w",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },
    { "g28","g28a",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },
    { "g28","g28b",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },
    { "g28","g28c",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },

    { "g30","g30x",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },// g30 handled differently
    { "g30","g30y",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },
    { "g30","g30z",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },
    { "g30","g30u",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },
    { "g30","g30v",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },
    { "g30","g30w",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },
    { "g30","g30a",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },
    { "g30","g30b",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },
    { "g30","g30c",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },

    // this is a 128bit UUID for identifying a previously committed job state
    { "jid","jida",_d0, 0, tx_print_nul, get_data, set_data, (float *)&cfg.job_id[0], 0 },
    { "jid","jidb",_d0, 0, tx_print_nul, get_data, set_data, (float *)&cfg.job_id[1], 0 },
    { "jid","jidc",_d0, 0, tx_print_nul, get_data, set_data, (float *)&cfg.job_id[2], 0 },
    { "jid","jidd",_d0, 0, tx_print_nul, get_data, set_data, (float *)&cfg.job_id[3], 0 },

    // Spindle functions
    { "sp","spmo", _iip, 0, sp_print_spmo, sp_get_spmo, sp_set_spmo, nullptr, SPINDLE_MODE },
    { "sp","spph", _bip, 0, sp_print_spph, sp_get_spph, sp_set_spph, nullptr, SPINDLE_PAUSE_ON_HOLD },
    { "sp","spde", _fip, 2, sp_print_spde, sp_get_spde, sp_set_spde, nullptr, SPINDLE_SPINUP_DELAY },
    { "sp","spsn", _fip, 2, sp_print_spsn, sp_get_spsn, sp_set_spsn, nullptr, SPINDLE_SPEED_MIN},
    { "sp","spsm", _fip, 2, sp_print_spsm, sp_get_spsm, sp_set_spsm, nullptr, SPINDLE_SPEED_MAX},
    { "sp","spep", _iip, 0, sp_print_spep, sp_get_spep, sp_set_spep, nullptr, SPINDLE_ENABLE_POLARITY },
    { "sp","spdp", _iip, 0, sp_print_spdp, sp_get_spdp, sp_set_spdp, nullptr, SPINDLE_DIR_POLARITY },
    { "sp","spoe", _bip, 0, sp_print_spoe, sp_get_spoe, sp_set_spoe, nullptr, SPINDLE_OVERRIDE_ENABLE},
    { "sp","spo",  _fip, 3, sp_print_spo,  sp_get_spo,  sp_set_spo,  nullptr, SPINDLE_OVERRIDE_FACTOR},
    { "sp","spc",  _i0,  0, sp_print_spc,  sp_get_spc,  sp_set_spc,  nullptr, 0 },   // spindle state
    { "sp","sps",  _f0,  0, sp_print_sps,  sp_get_sps,  sp_set_sps,  nullptr, 0 },   // spindle speed

    // Coolant functions
    { "co","coph", _bip, 0, co_print_coph, co_get_coph, co_set_coph, nullptr, COOLANT_PAUSE_ON_HOLD },
    { "co","comp", _iip, 0, co_print_comp, co_get_comp, co_set_comp, nullptr, COOLANT_MIST_POLARITY },
    { "co","cofp", _iip, 0, co_print_cofp, co_get_cofp, co_set_cofp, nullptr, COOLANT_FLOOD_POLARITY },
    { "co","com",  _i0,  0, co_print_com,  co_get_com,  co_set_com,  nullptr, 0 },   // mist coolant enable
    { "co","cof",  _i0,  0, co_print_cof,  co_get_cof,  co_set_cof,  nullptr, 0 },   // flood coolant enable

    // General system parameters
    { "sys","jt",  _fipn, 2, cm_print_jt,  cm_get_jt,  cm_set_jt,  nullptr, JUNCTION_INTEGRATION_TIME },
    { "sys","ct",  _fipnc,4, cm_print_ct,  cm_get_ct,  cm_set_ct,  nullptr, CHORDAL_TOLERANCE },
    { "sys","zl",  _fipnc,3, cm_print_zl,  cm_get_zl,  cm_set_zl,  nullptr, FEEDHOLD_Z_LIFT },
    { "sys","sl",  _bipn, 0, cm_print_sl,  cm_get_sl,  cm_set_sl,  nullptr, SOFT_LIMIT_ENABLE },
    { "sys","lim", _bipn, 0, cm_print_lim, cm_get_lim, cm_set_lim, nullptr, HARD_LIMIT_ENABLE },
    { "sys","saf", _bipn, 0, cm_print_saf, cm_get_saf, cm_set_saf, nullptr, SAFETY_INTERLOCK_ENABLE },
    { "sys","m48", _bin, 0, cm_print_m48,  cm_get_m48, cm_get_m48, nullptr, 1 },   // M48/M49 feedrate & spindle override enable
    { "sys","froe",_bin, 0, cm_print_froe, cm_get_froe,cm_get_froe,nullptr, FEED_OVERRIDE_ENABLE},
    { "sys","fro", _fin, 3, cm_print_fro,  cm_get_fro, cm_set_fro, nullptr, FEED_OVERRIDE_FACTOR},
    { "sys","troe",_bin, 0, cm_print_troe, cm_get_troe,cm_get_troe,nullptr, TRAVERSE_OVERRIDE_ENABLE},
    { "sys","tro", _fin, 3, cm_print_tro,  cm_get_tro, cm_set_tro, nullptr, TRAVERSE_OVERRIDE_FACTOR},
    { "sys","mt",  _fipn, 2, st_print_mt,  st_get_mt,  st_set_mt,  nullptr, MOTOR_POWER_TIMEOUT}, // N is seconds of timeout
    { "",   "me",  _f0,   0, st_print_me,  get_nul,    st_set_me,  nullptr, 0 },    // SET to enable motors
    { "",   "md",  _f0,   0, st_print_md,  get_nul,    st_set_md,  nullptr, 0 },    // SET to disable motors

    // Communications and reporting parameters
#ifdef __TEXT_MODE
    { "sys","tv", _iipn, 0, tx_print_tv, txt_get_tv, txt_set_tv, nullptr, TEXT_VERBOSITY },
#endif
    { "sys","ej", _iipn, 0, js_print_ej,  js_get_ej, js_set_ej, nullptr, COMM_MODE },
    { "sys","jv", _iipn, 0, js_print_jv,  js_get_jv, js_set_jv, nullptr, JSON_VERBOSITY },
    { "sys","qv", _iipn, 0, qr_print_qv,  qr_get_qv, qr_set_qv, nullptr, QUEUE_REPORT_VERBOSITY },
    { "sys","sv", _iipn, 0, sr_print_sv,  sr_get_sv, sr_set_sv, nullptr, STATUS_REPORT_VERBOSITY },
    { "sys","si", _iipn, 0, sr_print_si,  sr_get_si, sr_set_si, nullptr, STATUS_REPORT_INTERVAL_MS },

    // Gcode defaults
    // NOTE: The ordering within the gcode defaults is important for token resolution. gc must follow gco
    { "sys","gpl", _iipn, 0, cm_print_gpl, cm_get_gpl, cm_set_gpl, nullptr, GCODE_DEFAULT_PLANE },
    { "sys","gun", _iipn, 0, cm_print_gun, cm_get_gun, cm_set_gun, nullptr, GCODE_DEFAULT_UNITS },
    { "sys","gco", _iipn, 0, cm_print_gco, cm_get_gco, cm_set_gco, nullptr, GCODE_DEFAULT_COORD_SYSTEM },
    { "sys","gpa", _iipn, 0, cm_print_gpa, cm_get_gpa, cm_set_gpa, nullptr, GCODE_DEFAULT_PATH_CONTROL },
    { "sys","gdi", _iipn, 0, cm_print_gdi, cm_get_gdi, cm_set_gdi, nullptr, GCODE_DEFAULT_DISTANCE_MODE },
    { "",   "gc2", _s0,   0, tx_print_nul, gc_get_gc,  gc_run_gc,  nullptr, 0 },  // send gcode to secondary planner
    { "",   "gc",  _s0,   0, tx_print_nul, gc_get_gc,  gc_run_gc,  nullptr, 0 },  // gcode block - must be last in this group

    // Actions and Reports
    { "", "sr",   _n0, 0, sr_print_sr,   sr_get,    sr_set,    nullptr, 0 },    // request and set status reports
    { "", "qr",   _n0, 0, qr_print_qr,   qr_get,    set_nul,   nullptr, 0 },    // get queue value - planner buffers available
    { "", "qi",   _n0, 0, qr_print_qi,   qi_get,    set_nul,   nullptr, 0 },    // get queue value - buffers added to queue
    { "", "qo",   _n0, 0, qr_print_qo,   qo_get,    set_nul,   nullptr, 0 },    // get queue value - buffers removed from queue
    { "", "er",   _n0, 0, tx_print_nul,  rpt_er,    set_nul,   nullptr, 0 },    // get bogus exception report for testing
    { "", "rx",   _n0, 0, tx_print_int,  get_rx,    set_nul,   nullptr, 0 },    // get RX buffer bytes or packets
    { "", "dw",   _i0, 0, tx_print_int,  st_get_dw, set_noop,  nullptr, 0 },    // get dwell time remaining
    { "", "msg",  _s0, 0, tx_print_str,  get_nul,   set_noop,  nullptr, 0 },    // no operation on messages
    { "", "alarm",_n0, 0, tx_print_nul,  cm_alrm,   cm_alrm,   nullptr, 0 },    // trigger alarm
    { "", "panic",_n0, 0, tx_print_nul,  cm_pnic,   cm_pnic,   nullptr, 0 },    // trigger panic
    { "", "shutd",_n0, 0, tx_print_nul,  cm_shutd,  cm_shutd,  nullptr, 0 },    // trigger shutdown
    { "", "clear",_n0, 0, tx_print_nul,  cm_clr,    cm_clr,    nullptr, 0 },    // GET "clear" to clear alarm state
    { "", "clr",  _n0, 0, tx_print_nul,  cm_clr,    cm_clr,    nullptr, 0 },    // synonym for "clear"
    { "", "tick", _n0, 0, tx_print_int,  get_tick,  set_nul,   nullptr, 0 },    // get system time tic
    { "", "tram", _b0, 0, cm_print_tram,cm_get_tram,cm_set_tram,nullptr,0 },    // SET to attempt setting rotation matrix from probes
    { "", "defa", _b0, 0, tx_print_nul,  help_defa,set_defaults,nullptr,0 },    // set/print defaults / help screen
    { "", "flash",_b0, 0, tx_print_nul,  help_flash,hw_flash,  nullptr, 0 },

#ifdef __HELP_SCREENS
    { "", "help",_b0, 0, tx_print_nul, help_config, set_nul, nullptr, 0 },  // prints config help screen
    { "", "h",   _b0, 0, tx_print_nul, help_config, set_nul, nullptr, 0 },  // alias for "help"
#endif

#ifdef __USER_DATA
    // User defined data groups
    { "uda","uda0", _fip, 0, tx_print_int, get_data, set_data, &cfg.user_data_a[0], USER_DATA_A0 },
    { "uda","uda1", _fip, 0, tx_print_int, get_data, set_data, &cfg.user_data_a[1], USER_DATA_A1 },
    { "uda","uda2", _fip, 0, tx_print_int, get_data, set_data, &cfg.user_data_a[2], USER_DATA_A2 },
    { "uda","uda3", _fip, 0, tx_print_int, get_data, set_data, &cfg.user_data_a[3], USER_DATA_A3 },

    { "udb","udb0", _fip, 0, tx_print_int, get_data, set_data, &cfg.user_data_b[0], USER_DATA_B0 },
    { "udb","udb1", _fip, 0, tx_print_int, get_data, set_data, &cfg.user_data_b[1], USER_DATA_B1 },
    { "udb","udb2", _fip, 0, tx_print_int, get_data, set_data, &cfg.user_data_b[2], USER_DATA_B2 },
    { "udb","udb3", _fip, 0, tx_print_int, get_data, set_data, &cfg.user_data_b[3], USER_DATA_B3 },

    { "udc","udc0", _fip, 0, tx_print_int, get_data, set_data, &cfg.user_data_c[0], USER_DATA_C0 },
    { "udc","udc1", _fip, 0, tx_print_int, get_data, set_data, &cfg.user_data_c[1], USER_DATA_C1 },
    { "udc","udc2", _fip, 0, tx_print_int, get_data, set_data, &cfg.user_data_c[2], USER_DATA_C2 },
    { "udc","udc3", _fip, 0, tx_print_int, get_data, set_data, &cfg.user_data_c[3], USER_DATA_C3 },

    { "udd","udd0", _fip, 0, tx_print_int, get_data, set_data, &cfg.user_data_d[0], USER_DATA_D0 },
    { "udd","udd1", _fip, 0, tx_print_int, get_data, set_data, &cfg.user_data_d[1], USER_DATA_D1 },
    { "udd","udd2", _fip, 0, tx_print_int, get_data, set_data, &cfg.user_data_d[2], USER_DATA_D2 },
    { "udd","udd3", _fip, 0, tx_print_int, get_data, set_data, &cfg.user_data_d[3], USER_DATA_D3 },
#endif

    // Tool table offsets
    { "tof","tofx",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },
    { "tof","tofy",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },
    { "tof","tofz",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },
    { "tof","tofu",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },
    { "tof","tofv",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },
    { "tof","tofw",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },
    { "tof","tofa",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },
    { "tof","tofb",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },
    { "tof","tofc",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },

    // Tool table
    { "tt1","tt1x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT1_X_OFFSET },
    { "tt1","tt1y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT1_Y_OFFSET },
    { "tt1","tt1z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT1_Z_OFFSET },
    { "tt1","tt1u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT1_U_OFFSET },
    { "tt1","tt1v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT1_V_OFFSET },
    { "tt1","tt1w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT1_W_OFFSET },
    { "tt1","tt1a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT1_A_OFFSET },
    { "tt1","tt1b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT1_B_OFFSET },
    { "tt1","tt1c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT1_C_OFFSET },

    { "tt2","tt2x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT2_X_OFFSET },
    { "tt2","tt2y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT2_Y_OFFSET },
    { "tt2","tt2z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT2_Z_OFFSET },
    { "tt2","tt2u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT2_U_OFFSET },
    { "tt2","tt2v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT2_V_OFFSET },
    { "tt2","tt2w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT2_W_OFFSET },
    { "tt2","tt2a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT2_A_OFFSET },
    { "tt2","tt2b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT2_B_OFFSET },
    { "tt2","tt2c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT2_C_OFFSET },

    { "tt3","tt3x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT3_X_OFFSET },
    { "tt3","tt3y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT3_Y_OFFSET },
    { "tt3","tt3z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT3_Z_OFFSET },
    { "tt3","tt3u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT3_U_OFFSET },
    { "tt3","tt3v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT3_V_OFFSET },
    { "tt3","tt3w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT3_W_OFFSET },
    { "tt3","tt3a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT3_A_OFFSET },
    { "tt3","tt3b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT3_B_OFFSET },
    { "tt3","tt3c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT1_C_OFFSET },

    { "tt4","tt4x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT4_X_OFFSET },
    { "tt4","tt4y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT4_Y_OFFSET },
    { "tt4","tt4z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT4_Z_OFFSET },
    { "tt4","tt4u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT4_U_OFFSET },
    { "tt4","tt4v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT4_V_OFFSET },
    { "tt4","tt4w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT4_W_OFFSET },
    { "tt4","tt4a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT4_A_OFFSET },
    { "tt4","tt4b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT4_B_OFFSET },
    { "tt4","tt4c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT4_C_OFFSET },

    { "tt5","tt5x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT5_X_OFFSET },
    { "tt5","tt5y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT5_Y_OFFSET },
    { "tt5","tt5z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT5_Z_OFFSET },
    { "tt5","tt5u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT5_U_OFFSET },
    { "tt5","tt5v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT5_V_OFFSET },
    { "tt5","tt5w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT5_W_OFFSET },
    { "tt5","tt5a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT5_A_OFFSET },
    { "tt5","tt5b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT5_B_OFFSET },
    { "tt5","tt5c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT5_C_OFFSET },

    { "tt6","tt6x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT6_X_OFFSET },
    { "tt6","tt6y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT6_Y_OFFSET },
    { "tt6","tt6z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT6_Z_OFFSET },
    { "tt6","tt6u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT6_U_OFFSET },
    { "tt6","tt6v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT6_V_OFFSET },
    { "tt6","tt6w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT6_W_OFFSET },
    { "tt6","tt6a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT6_A_OFFSET },
    { "tt6","tt6b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT6_B_OFFSET },
    { "tt6","tt6c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT6_C_OFFSET },

    { "tt7","tt7x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT7_X_OFFSET },
    { "tt7","tt7y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT7_Y_OFFSET },
    { "tt7","tt7z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT7_Z_OFFSET },
    { "tt7","tt7u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT7_U_OFFSET },
    { "tt7","tt7v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT7_V_OFFSET },
    { "tt7","tt7w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT7_W_OFFSET },
    { "tt7","tt7a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT7_A_OFFSET },
    { "tt7","tt7b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT7_B_OFFSET },
    { "tt7","tt7c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT7_C_OFFSET },

    { "tt8","tt8x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT8_X_OFFSET },
    { "tt8","tt8y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT8_Y_OFFSET },
    { "tt8","tt8z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT8_Z_OFFSET },
    { "tt8","tt8u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT8_U_OFFSET },
    { "tt8","tt8v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT8_V_OFFSET },
    { "tt8","tt8w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT8_W_OFFSET },
    { "tt8","tt8a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT8_A_OFFSET },
    { "tt8","tt8b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT8_B_OFFSET },
    { "tt8","tt8c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT8_C_OFFSET },

    { "tt9","tt9x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT9_X_OFFSET },
    { "tt9","tt9y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT9_Y_OFFSET },
    { "tt9","tt9z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT9_Z_OFFSET },
    { "tt9","tt9u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT9_U_OFFSET },
    { "tt9","tt9v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT9_V_OFFSET },
    { "tt9","tt9w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT9_W_OFFSET },
    { "tt9","tt9a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT9_A_OFFSET },
    { "tt9","tt9b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT9_B_OFFSET },
    { "tt9","tt9c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT9_C_OFFSET },

    { "tt10","tt10x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT10_X_OFFSET },
    { "tt10","tt10y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT10_Y_OFFSET },
    { "tt10","tt10z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT10_Z_OFFSET },
    { "tt10","tt10u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT10_U_OFFSET },
    { "tt10","tt10v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT10_V_OFFSET },
    { "tt10","tt10w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT10_W_OFFSET },
    { "tt10","tt10a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT10_A_OFFSET },
    { "tt10","tt10b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT10_B_OFFSET },
    { "tt10","tt10c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT10_C_OFFSET },

    { "tt11","tt11x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT11_X_OFFSET },
    { "tt11","tt11y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT11_Y_OFFSET },
    { "tt11","tt11z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT11_Z_OFFSET },
    { "tt11","tt11u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT11_U_OFFSET },
    { "tt11","tt11v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT11_V_OFFSET },
    { "tt11","tt11w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT11_W_OFFSET },
    { "tt11","tt11a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT11_A_OFFSET },
    { "tt11","tt11b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT11_B_OFFSET },
    { "tt11","tt11c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT11_C_OFFSET },

    { "tt12","tt12x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT12_X_OFFSET },
    { "tt12","tt12y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT12_Y_OFFSET },
    { "tt12","tt12z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT12_Z_OFFSET },
    { "tt12","tt12u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT12_U_OFFSET },
    { "tt12","tt12v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT12_V_OFFSET },
    { "tt12","tt12w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT12_W_OFFSET },
    { "tt12","tt12a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT12_A_OFFSET },
    { "tt12","tt12b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT12_B_OFFSET },
    { "tt12","tt12c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT12_C_OFFSET },

    { "tt13","tt13x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT13_X_OFFSET },
    { "tt13","tt13y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT13_Y_OFFSET },
    { "tt13","tt13z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT13_Z_OFFSET },
    { "tt13","tt13u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT13_U_OFFSET },
    { "tt13","tt13v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT13_V_OFFSET },
    { "tt13","tt13w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT13_W_OFFSET },
    { "tt13","tt13a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT13_A_OFFSET },
    { "tt13","tt13b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT13_B_OFFSET },
    { "tt13","tt13c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT13_C_OFFSET },

    { "tt14","tt14x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT14_X_OFFSET },
    { "tt14","tt14y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT14_Y_OFFSET },
    { "tt14","tt14z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT14_Z_OFFSET },
    { "tt14","tt14u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT14_U_OFFSET },
    { "tt14","tt14v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT14_V_OFFSET },
    { "tt14","tt14w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT14_W_OFFSET },
    { "tt14","tt14a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT14_A_OFFSET },
    { "tt14","tt14b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT14_B_OFFSET },
    { "tt14","tt14c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT14_C_OFFSET },

    { "tt15","tt15x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT15_X_OFFSET },
    { "tt15","tt15y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT15_Y_OFFSET },
    { "tt15","tt15z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT15_Z_OFFSET },
    { "tt15","tt15u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT15_U_OFFSET },
    { "tt15","tt15v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT15_V_OFFSET },
    { "tt15","tt15w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT15_W_OFFSET },
    { "tt15","tt15a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT15_A_OFFSET },
    { "tt15","tt15b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT15_B_OFFSET },
    { "tt15","tt15c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT15_C_OFFSET },

    { "tt16","tt16x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT16_X_OFFSET },
    { "tt16","tt16y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT16_Y_OFFSET },
    { "tt16","tt16z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT16_Z_OFFSET },
    { "tt16","tt16u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT16_U_OFFSET },
    { "tt16","tt16v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT16_V_OFFSET },
    { "tt16","tt16w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT16_W_OFFSET },
    { "tt16","tt16a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT16_A_OFFSET },
    { "tt16","tt16b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT16_B_OFFSET },
    { "tt16","tt16c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT16_C_OFFSET },

    { "tt17","tt17x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT17_X_OFFSET },
    { "tt17","tt17y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT17_Y_OFFSET },
    { "tt17","tt17z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT17_Z_OFFSET },
    { "tt17","tt17u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT17_U_OFFSET },
    { "tt17","tt17v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT17_V_OFFSET },
    { "tt17","tt17w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT17_W_OFFSET },
    { "tt17","tt17a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT17_A_OFFSET },
    { "tt17","tt17b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT17_B_OFFSET },
    { "tt17","tt17c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT17_C_OFFSET },

    { "tt18","tt18x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT18_X_OFFSET },
    { "tt18","tt18y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT18_Y_OFFSET },
    { "tt18","tt18z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT18_Z_OFFSET },
    { "tt18","tt18u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT18_U_OFFSET },
    { "tt18","tt18v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT18_V_OFFSET },
    { "tt18","tt18w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT18_W_OFFSET },
    { "tt18","tt18a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT18_A_OFFSET },
    { "tt18","tt18b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT18_B_OFFSET },
    { "tt18","tt18c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT18_C_OFFSET },

    { "tt19","tt19x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT19_X_OFFSET },
    { "tt19","tt19y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT19_Y_OFFSET },
    { "tt19","tt19z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT19_Z_OFFSET },
    { "tt19","tt19u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT19_U_OFFSET },
    { "tt19","tt19v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT19_V_OFFSET },
    { "tt19","tt19w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT19_W_OFFSET },
    { "tt19","tt19a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT19_A_OFFSET },
    { "tt19","tt19b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT19_B_OFFSET },
    { "tt19","tt19c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT19_C_OFFSET },

    { "tt20","tt20x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT20_X_OFFSET },
    { "tt20","tt20y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT20_Y_OFFSET },
    { "tt20","tt20z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT20_Z_OFFSET },
    { "tt20","tt20u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT20_U_OFFSET },
    { "tt20","tt20v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT20_V_OFFSET },
    { "tt20","tt20w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT20_W_OFFSET },
    { "tt20","tt20a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT20_A_OFFSET },
    { "tt20","tt20b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT20_B_OFFSET },
    { "tt20","tt20c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT20_C_OFFSET },

    { "tt21","tt21x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT21_X_OFFSET },
    { "tt21","tt21y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT21_Y_OFFSET },
    { "tt21","tt21z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT21_Z_OFFSET },
    { "tt21","tt21u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT21_U_OFFSET },
    { "tt21","tt21v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT21_V_OFFSET },
    { "tt21","tt21w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT21_W_OFFSET },
    { "tt21","tt21a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT21_A_OFFSET },
    { "tt21","tt21b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT21_B_OFFSET },
    { "tt21","tt21c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT21_C_OFFSET },

    { "tt22","tt22x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT22_X_OFFSET },
    { "tt22","tt22y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT22_Y_OFFSET },
    { "tt22","tt22z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT22_Z_OFFSET },
    { "tt22","tt22u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT22_U_OFFSET },
    { "tt22","tt22v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT22_V_OFFSET },
    { "tt22","tt22w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT22_W_OFFSET },
    { "tt22","tt22a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT22_A_OFFSET },
    { "tt22","tt22b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT22_B_OFFSET },
    { "tt22","tt22c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT22_C_OFFSET },

    { "tt23","tt23x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT23_X_OFFSET },
    { "tt23","tt23y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT23_Y_OFFSET },
    { "tt23","tt23z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT23_Z_OFFSET },
    { "tt23","tt23u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT23_U_OFFSET },
    { "tt23","tt23v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT23_V_OFFSET },
    { "tt23","tt23w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT23_W_OFFSET },
    { "tt23","tt23a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT23_A_OFFSET },
    { "tt23","tt23b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT23_B_OFFSET },
    { "tt23","tt23c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT23_C_OFFSET },

    { "tt24","tt24x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT24_X_OFFSET },
    { "tt24","tt24y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT24_Y_OFFSET },
    { "tt24","tt24z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT24_Z_OFFSET },
    { "tt24","tt24u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT24_U_OFFSET },
    { "tt24","tt24v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT24_V_OFFSET },
    { "tt24","tt24w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT24_W_OFFSET },
    { "tt24","tt24a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT24_A_OFFSET },
    { "tt24","tt24b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT24_B_OFFSET },
    { "tt24","tt24c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT24_C_OFFSET },

    { "tt25","tt25x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT25_X_OFFSET },
    { "tt25","tt25y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT25_Y_OFFSET },
    { "tt25","tt25z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT25_Z_OFFSET },
    { "tt25","tt25u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT25_U_OFFSET },
    { "tt25","tt25v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT25_V_OFFSET },
    { "tt25","tt25w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT25_W_OFFSET },
    { "tt25","tt25a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT25_A_OFFSET },
    { "tt25","tt25b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT25_B_OFFSET },
    { "tt25","tt25c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT25_C_OFFSET },

    { "tt26","tt26x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT26_X_OFFSET },
    { "tt26","tt26y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT26_Y_OFFSET },
    { "tt26","tt26z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT26_Z_OFFSET },
    { "tt26","tt26u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT26_U_OFFSET },
    { "tt26","tt26v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT26_V_OFFSET },
    { "tt26","tt26w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT26_W_OFFSET },
    { "tt26","tt26a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT26_A_OFFSET },
    { "tt26","tt26b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT26_B_OFFSET },
    { "tt26","tt26c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT26_C_OFFSET },

    { "tt27","tt27x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT27_X_OFFSET },
    { "tt27","tt27y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT27_Y_OFFSET },
    { "tt27","tt27z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT27_Z_OFFSET },
    { "tt27","tt27u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT27_U_OFFSET },
    { "tt27","tt27v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT27_V_OFFSET },
    { "tt27","tt27w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT27_W_OFFSET },
    { "tt27","tt27a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT27_A_OFFSET },
    { "tt27","tt27b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT27_B_OFFSET },
    { "tt27","tt27c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT27_C_OFFSET },

    { "tt28","tt28x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT28_X_OFFSET },
    { "tt28","tt28y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT28_Y_OFFSET },
    { "tt28","tt28z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT28_Z_OFFSET },
    { "tt28","tt28u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT28_U_OFFSET },
    { "tt28","tt28v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT28_V_OFFSET },
    { "tt28","tt28w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT28_W_OFFSET },
    { "tt28","tt28a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT28_A_OFFSET },
    { "tt28","tt28b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT28_B_OFFSET },
    { "tt28","tt28c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT28_C_OFFSET },

    { "tt29","tt29x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT29_X_OFFSET },
    { "tt29","tt29y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT29_Y_OFFSET },
    { "tt29","tt29z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT29_Z_OFFSET },
    { "tt29","tt29u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT29_U_OFFSET },
    { "tt29","tt29v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT29_V_OFFSET },
    { "tt29","tt29w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT29_W_OFFSET },
    { "tt29","tt29a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT29_A_OFFSET },
    { "tt29","tt29b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT29_B_OFFSET },
    { "tt29","tt29c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT29_C_OFFSET },

    { "tt30","tt30x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT30_X_OFFSET },
    { "tt30","tt30y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT30_Y_OFFSET },
    { "tt30","tt30z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT30_Z_OFFSET },
    { "tt30","tt30u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT30_U_OFFSET },
    { "tt30","tt30v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT30_V_OFFSET },
    { "tt30","tt30w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT30_W_OFFSET },
    { "tt30","tt30a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT30_A_OFFSET },
    { "tt30","tt30b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT30_B_OFFSET },
    { "tt30","tt30c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT30_C_OFFSET },

    { "tt31","tt31x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT31_X_OFFSET },
    { "tt31","tt31y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT31_Y_OFFSET },
    { "tt31","tt31z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT31_Z_OFFSET },
    { "tt31","tt31u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT31_U_OFFSET },
    { "tt31","tt31v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT31_V_OFFSET },
    { "tt31","tt31w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT31_W_OFFSET },
    { "tt31","tt31a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT31_A_OFFSET },
    { "tt31","tt31b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT31_B_OFFSET },
    { "tt31","tt31c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT31_C_OFFSET },

    { "tt32","tt32x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT32_X_OFFSET },
    { "tt32","tt32y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT32_Y_OFFSET },
    { "tt32","tt32z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT32_Z_OFFSET },
    { "tt32","tt32u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT32_U_OFFSET },
    { "tt32","tt32v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT32_V_OFFSET },
    { "tt32","tt32w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT32_W_OFFSET },
    { "tt32","tt32a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT32_A_OFFSET },
    { "tt32","tt32b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT32_B_OFFSET },
    { "tt32","tt32c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT32_C_OFFSET },

    // Diagnostic parameters
#ifdef __DIAGNOSTIC_PARAMETERS
    { "",    "clc",_f0, 0, tx_print_nul, st_clc,  st_clc, &cs.null, 0 },  // clear diagnostic step counters

    { "_te","_tex",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->target[AXIS_X], 0 }, // X target endpoint
    { "_te","_tey",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->target[AXIS_Y], 0 },
    { "_te","_tez",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->target[AXIS_Z], 0 },
    { "_te","_tea",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->target[AXIS_A], 0 },
    { "_te","_teb",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->target[AXIS_B], 0 },
    { "_te","_tec",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->target[AXIS_C], 0 },

    { "_tr","_trx",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->gm.target[AXIS_X], 0 },  // X target runtime
    { "_tr","_try",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->gm.target[AXIS_Y], 0 },
    { "_tr","_trz",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->gm.target[AXIS_Z], 0 },
    { "_tr","_tra",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->gm.target[AXIS_A], 0 },
    { "_tr","_trb",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->gm.target[AXIS_B], 0 },
    { "_tr","_trc",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->gm.target[AXIS_C], 0 },

#if (MOTORS >= 1)
    { "_ts","_ts1",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->target_steps[MOTOR_1], 0 },      // Motor 1 target steps
    { "_ps","_ps1",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->position_steps[MOTOR_1], 0 },    // Motor 1 position steps
    { "_cs","_cs1",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->commanded_steps[MOTOR_1], 0 },   // Motor 1 commanded steps (delayed steps)
    { "_es","_es1",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->encoder_steps[MOTOR_1], 0 },     // Motor 1 encoder steps
    { "_xs","_xs1",_f0, 2, tx_print_flt, get_flt, set_nul, &st_pre.mot[MOTOR_1].corrected_steps, 0 }, // Motor 1 correction steps applied
    { "_fe","_fe1",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->following_error[MOTOR_1], 0 },   // Motor 1 following error in steps
#endif
#if (MOTORS >= 2)
    { "_ts","_ts2",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->target_steps[MOTOR_2], 0 },
    { "_ps","_ps2",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->position_steps[MOTOR_2], 0 },
    { "_cs","_cs2",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->commanded_steps[MOTOR_2], 0 },
    { "_es","_es2",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->encoder_steps[MOTOR_2], 0 },
    { "_xs","_xs2",_f0, 2, tx_print_flt, get_flt, set_nul, &st_pre.mot[MOTOR_2].corrected_steps, 0 },
    { "_fe","_fe2",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->following_error[MOTOR_2], 0 },
#endif
#if (MOTORS >= 3)
    { "_ts","_ts3",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->target_steps[MOTOR_3], 0 },
    { "_ps","_ps3",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->position_steps[MOTOR_3], 0 },
    { "_cs","_cs3",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->commanded_steps[MOTOR_3], 0 },
    { "_es","_es3",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->encoder_steps[MOTOR_3], 0 },
    { "_xs","_xs3",_f0, 2, tx_print_flt, get_flt, set_nul, &st_pre.mot[MOTOR_3].corrected_steps, 0 },
    { "_fe","_fe3",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->following_error[MOTOR_3], 0 },
#endif
#if (MOTORS >= 4)
    { "_ts","_ts4",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->target_steps[MOTOR_4], 0 },
    { "_ps","_ps4",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->position_steps[MOTOR_4], 0 },
    { "_cs","_cs4",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->commanded_steps[MOTOR_4], 0 },
    { "_es","_es4",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->encoder_steps[MOTOR_4], 0 },
    { "_xs","_xs4",_f0, 2, tx_print_flt, get_flt, set_nul, &st_pre.mot[MOTOR_4].corrected_steps, 0 },
    { "_fe","_fe4",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->following_error[MOTOR_4], 0 },
#endif
#if (MOTORS >= 5)
    { "_ts","_ts5",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->target_steps[MOTOR_5], 0 },
    { "_ps","_ps5",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->position_steps[MOTOR_5], 0 },
    { "_cs","_cs5",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->commanded_steps[MOTOR_5], 0 },
    { "_es","_es5",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->encoder_steps[MOTOR_5], 0 },
    { "_xs","_xs6",_f0, 2, tx_print_flt, get_flt, set_nul, &st_pre.mot[MOTOR_5].corrected_steps, 0 },
    { "_fe","_fe5",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->following_error[MOTOR_5], 0 },
#endif
#if (MOTORS >= 6)
    { "_ts","_ts6",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->target_steps[MOTOR_6], 0 },
    { "_ps","_ps6",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->position_steps[MOTOR_6], 0 },
    { "_cs","_cs6",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->commanded_steps[MOTOR_6], 0 },
    { "_es","_es6",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->encoder_steps[MOTOR_6], 0 },
    { "_xs","_xs5",_f0, 2, tx_print_flt, get_flt, set_nul, &st_pre.mot[MOTOR_6].corrected_steps, 0 },
    { "_fe","_fe6",_f0, 2, tx_print_flt, get_flt, set_nul, &mr->following_error[MOTOR_6], 0 },
#endif

#endif  //  __DIAGNOSTIC_PARAMETERS

    // Persistence for status report - must be in sequence
    // *** Count must agree with NV_STATUS_REPORT_LEN in report.h ***
    { "","se00",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[0],0 },
    { "","se01",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[1],0 },
    { "","se02",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[2],0 },
    { "","se03",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[3],0 },
    { "","se04",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[4],0 },
    { "","se05",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[5],0 },
    { "","se06",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[6],0 },
    { "","se07",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[7],0 },
    { "","se08",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[8],0 },
    { "","se09",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[9],0 },
    { "","se10",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[10],0 },
    { "","se11",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[11],0 },
    { "","se12",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[12],0 },
    { "","se13",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[13],0 },
    { "","se14",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[14],0 },
    { "","se15",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[15],0 },
    { "","se16",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[16],0 },
    { "","se17",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[17],0 },
    { "","se18",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[18],0 },
    { "","se19",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[19],0 },
    { "","se20",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[20],0 },
    { "","se21",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[21],0 },
    { "","se22",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[22],0 },
    { "","se23",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[23],0 },
    { "","se24",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[24],0 },
    { "","se25",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[25],0 },
    { "","se26",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[26],0 },
    { "","se27",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[27],0 },
    { "","se28",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[28],0 },
    { "","se29",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[29],0 },
    { "","se30",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[30],0 },
    { "","se31",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[31],0 },
    { "","se32",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[32],0 },
    { "","se33",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[33],0 },
    { "","se34",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[34],0 },
    { "","se35",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[35],0 },
    { "","se36",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[36],0 },
    { "","se37",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[37],0 },
    { "","se38",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[38],0 },
    { "","se39",_fp, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[39],0 },
    // Count is 40, since se00 counts as one.


    // Group lookups - must follow the single-valued entries for proper sub-string matching
    // *** Must agree with NV_COUNT_GROUPS below ***
    // *** If you adjust the number of entries in a group you must also adjust the count for that group ***
    // *** COUNT STARTS FROM HERE ***

#define FIXED_GROUPS 4
    { "","sys",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // system group
    { "","p1", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // PWM 1 group
    { "","sp", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // Spindle group
    { "","co", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // Coolant group

#define AXIS_GROUPS AXES
    { "","x",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // axis groups
    { "","y",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","z",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","u",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","v",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","w",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","a",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","b",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","c",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },

#define MOTOR_GROUPS MOTORS
    { "","1",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // motor groups
#if (MOTORS >= 2)
    { "","2",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
#endif
#if (MOTORS >= 3)
    { "","3",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
#endif
#if (MOTORS >= 4)
    { "","4",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
#endif
#if (MOTORS >= 5)
    { "","5",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
#endif
#if (MOTORS >= 6)
    { "","6",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
#endif

#define DIGITAL_IN_GROUPS 10
    { "","in",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // input state
    { "","di1", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // input configs
    { "","di2", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","di3", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","di4", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","di5", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","di6", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","di7", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","di8", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","di9", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
        
#define DIGITAL_OUT_GROUPS 14
    { "","out", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // output state
    { "","do1", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // output configs
    { "","do2", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","do3", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","do4", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","do5", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","do6", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","do7", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","do8", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","do9", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","do10", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","do11", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","do12", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","do13", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },

#define COORDINATE_OFFSET_GROUPS 9
    { "","g54",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // coord offset groups
    { "","g55",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","g56",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","g57",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","g58",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","g59",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","g92",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // origin offsets
    { "","g28",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // g28 home position
    { "","g30",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // g30 home position
        
#define TOOL_OFFSET_GROUPS 33
    { "","tof",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // current tool offsets
    { "","tt1",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
    { "","tt2",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
    { "","tt3",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
    { "","tt4",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
    { "","tt5",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
    { "","tt6",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
    { "","tt7",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
    { "","tt8",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
    { "","tt9",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
    { "","tt10",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt11",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt12",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt13",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt14",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt15",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt16",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt17",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt18",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt19",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt20",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt21",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt22",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt23",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt24",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt25",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt26",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt27",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt28",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt29",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt30",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt31",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
    { "","tt32",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // tt offsets
        
#define MACHINE_STATE_GROUPS 8
    { "","mpo",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // machine position group
    { "","pos",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // work position group
    { "","ofs",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // work offset group
    { "","hom",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // axis homing state group
    { "","prb",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // probing state group
    { "","pwr",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // motor power enagled group
    { "","jog",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // axis jogging state group
    { "","jid",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // job ID group

#define TEMPERATURE_GROUPS 6
    { "","he1", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // heater 1 group
    { "","he2", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // heater 2 group
    { "","he3", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // heater 3 group
    { "","pid1",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // PID 1 group
    { "","pid2",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // PID 2 group
    { "","pid3",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // PID 3 group

#ifdef __USER_DATA
#define USER_DATA_GROUPS 4
    { "","uda", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // user data group
    { "","udb", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // user data group
    { "","udc", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // user data group
    { "","udd", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // user data group
#endif

#ifdef __DIAGNOSTIC_PARAMETERS
#define DIAGNOSTIC_GROUPS 8
    { "","_te",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // target axis endpoint group
    { "","_tr",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // target axis runtime group
    { "","_ts",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // target motor steps group
    { "","_ps",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // position motor steps group
    { "","_cs",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // commanded motor steps group
    { "","_es",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // encoder steps group
    { "","_xs",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // correction steps group
    { "","_fe",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // following error group
#endif

#define NV_COUNT_UBER_GROUPS 6
    // Uber-group (groups of groups, for text-mode displays only)
    // *** Must agree with NV_COUNT_UBER_GROUPS below ****
    { "", "m", _f0, 0, tx_print_nul, _do_motors, set_nul, nullptr, 0 },
    { "", "q", _f0, 0, tx_print_nul, _do_axes,   set_nul, nullptr, 0 },
    { "", "o", _f0, 0, tx_print_nul, _do_offsets,set_nul, nullptr, 0 },
    { "", "di", _f0, 0, tx_print_nul,_do_inputs, set_nul, nullptr, 0 },
    { "", "do", _f0, 0, tx_print_nul,_do_outputs,set_nul, nullptr, 0 },
    { "", "$", _f0, 0, tx_print_nul, _do_all,    set_nul, nullptr, 0 }
};

/***** Make sure these defines line up with any changes in the above table *****/

#define NV_COUNT_GROUPS (FIXED_GROUPS \
                        + AXIS_GROUPS \
                        + MOTOR_GROUPS \
                        + DIGITAL_IN_GROUPS \
                        + DIGITAL_OUT_GROUPS \
                        + COORDINATE_OFFSET_GROUPS \
                        + TOOL_OFFSET_GROUPS \
                        + MACHINE_STATE_GROUPS \
                        + TEMPERATURE_GROUPS \
                        + USER_DATA_GROUPS \
                        + DIAGNOSTIC_GROUPS)

/* <DO NOT MESS WITH THESE DEFINES> */
#define NV_INDEX_MAX (sizeof(cfgArray) / sizeof(cfgItem_t))
#define NV_INDEX_END_SINGLES    (NV_INDEX_MAX - NV_COUNT_UBER_GROUPS - NV_COUNT_GROUPS - NV_STATUS_REPORT_LEN)
#define NV_INDEX_START_GROUPS    (NV_INDEX_MAX - NV_COUNT_UBER_GROUPS - NV_COUNT_GROUPS)
#define NV_INDEX_START_UBER_GROUPS (NV_INDEX_MAX - NV_COUNT_UBER_GROUPS)
/* </DO NOT MESS WITH THESE DEFINES> */

index_t nv_index_max() { return ( NV_INDEX_MAX );}
bool nv_index_is_single(index_t index) { return ((index <= NV_INDEX_END_SINGLES) ? true : false);}
bool nv_index_is_group(index_t index) { return (((index >= NV_INDEX_START_GROUPS) && (index < NV_INDEX_START_UBER_GROUPS)) ? true : false);}
bool nv_index_lt_groups(index_t index) { return ((index <= NV_INDEX_START_GROUPS) ? true : false);}

/***** APPLICATION SPECIFIC CONFIGS AND EXTENSIONS TO GENERIC FUNCTIONS *****/
/*
 * convert_incoming_float() - pre-process an incoming floating point number for canonical units
 * convert_outgoing_float() - pre-process an outgoing floating point number for units display
 *
 *  Incoming floats are destined for SET operations.
 *  Outgoing floats are the raw values from GET operations, destined for text or JSON display. 
 *
 *  Apologies in advance for these twisty little functions. These functions are used to
 *  convert incoming floats into the native, canonical form of a parameter (mm, or whatever)
 *  and outgoing floats into a display format appropriate to the units mode in effect. 
 *  They use the flags in the config table and other cues to determine what type of conversion 
 *  to perform. 
 *
 *  The conversions are complicated by the fact that only linear axes actually convert - 
 *  rotaries do not - unless they are in radius mode. Plus, determining the axis for a motor 
 *  requires unraveling the motor mapping (handled in cm_get_axis_type()). Also, there are 
 *  global SYS group values that are not associated with any axis. Lastly, the 
 *  steps-per-unit value (1su) is actually kept in inverse conversion form, as its native 
 *  form would be units-per-step.
 */

static void _convert(nvObj_t *nv, float conversion_factor) 
{
    if (nv->valuetype != TYPE_FLOAT) { return; } // can be called non-destructively for any value type
    if (isnan((double)nv->value_flt) || isinf((double)nv->value_flt)) { return; } // trap illegal float values
    ///+++ transform these checks into NaN or INF strings with an error return?

    if (cm_get_units_mode(MODEL) == INCHES) {
        cmAxisType axis_type = cm_get_axis_type(nv);        // linear, rotary, global or error
        if ((axis_type == AXIS_TYPE_LINEAR) || (axis_type == AXIS_TYPE_SYSTEM)) {
            if (cfgArray[nv->index].flags & F_CONVERT) {    // standard units conversion
                    nv->value_flt *= conversion_factor;
            } else                     
            if (cfgArray[nv->index].flags & F_ICONVERT) {   // inverse units conversion
                nv->value_flt /= conversion_factor;
            }
        }
    }  
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;
}

void convert_incoming_float(nvObj_t *nv) { return(_convert (nv, MM_PER_INCH)); }
void convert_outgoing_float(nvObj_t *nv) { return(_convert (nv, INCHES_PER_MM)); }

/*
 * get_float()       - boilerplate for retrieving raw floating point value
 * set_float()       - boilerplate for setting a floating point value with unit conversion
 * set_float_range() - set a floating point value with inclusive range check
 *
 *  get_float() loads nv->value with 'value' in internal canonical units (e.g. mm, degrees)
 *  without units conversion. If conversion is required call convert_outgoing_float()
 *  afterwards. The text mode and JSON display routines do this, so you generally don't
 *  have to worry about this.
 *
 *  set_float() is designed to capture incoming float values, so it performs unit conversion.
 *  set_float_range() perfoems an inclusive range test on the CONVERTED value
 */

stat_t get_float(nvObj_t *nv, const float value) {
    nv->value_flt = value;
    nv->valuetype = TYPE_FLOAT;
    nv->precision = GET_TABLE_WORD(precision);
    return STAT_OK;
}

stat_t set_float(nvObj_t *nv, float &value) {
    convert_incoming_float(nv);
    value = nv->value_flt;
    return (STAT_OK);
}

stat_t set_float_range(nvObj_t *nv, float &value, float low, float high) {

    char msg[64];

    convert_incoming_float(nv);      // conditional unit conversion
    if (nv->value_flt < low) {
        sprintf(msg, "Input is less than minimum value %0.4f", low);
        nv_add_conditional_message(msg);
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value_flt > high) {
        sprintf(msg, "Input is more than maximum value %0.4f", high);
        nv_add_conditional_message(msg);
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    value = nv->value_flt;
    return (STAT_OK);
}

/*
 * get_integer() - boilerplate for retrieving 8 and 32 bit integer values
 * set_integer() - boilerplate for setting 8 bit integer value with range checking
 * set_int32()   - boilerplate for setting 32 bit integer value with range checking
 */

static stat_t _set_int_tests(nvObj_t *nv, int32_t low, int32_t high)
{    
    char msg[64];

    if (nv->value_int < low) {
        sprintf(msg, "Input less than minimum value %d", (int)low);
        nv_add_conditional_message(msg);
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value_int > high) {
        sprintf(msg, "Input more than maximum value %d", (int)high);
        nv_add_conditional_message(msg);
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    return (STAT_OK);
}

stat_t get_integer(nvObj_t *nv, const int32_t value) 
{
    nv->value_int = value;
    nv->valuetype = TYPE_INTEGER;
    return STAT_OK;
}

stat_t set_integer(nvObj_t *nv, uint8_t &value, uint8_t low, uint8_t high) 
{
    ritorno(_set_int_tests(nv, low, high))
    value = nv->value_int;
    nv->valuetype = TYPE_INTEGER;
    return (STAT_OK);
}

stat_t set_int32(nvObj_t *nv, int32_t &value, int32_t low, int32_t high) 
{
    ritorno(_set_int_tests(nv, low, high))
    value = nv->value_int;  // note: valuetype = TYPE_INT already set
    nv->valuetype = TYPE_INTEGER;
    return (STAT_OK);
}

/*
 * get_string() - boilerplate for retrieving a string value
 */

stat_t get_string(nvObj_t *nv, const char *str)
{
    nv->valuetype = TYPE_STRING;
    return (nv_copy_string(nv, str));
}

/*
 * nv_group_is_prefixed() - hack
 *
 *  This little function deals with the exception cases that some groups don't use
 *  the parent token as a prefix to the child elements; SYS being a good example.
 */
bool nv_group_is_prefixed(char *group)
{
    if (strcmp("sys", group) == 0) {    // =0 means its a match
        return (false);
    }
    if (strcmp("sr", group) == 0) {
        return (false);
    }
    return (true);
}

/**** UberGroup Operations ****************************************************
 * Uber groups are groups of groups organized for convenience:
 *  - motors    - group of all motor groups
 *  - axes      - group of all axis groups
 *  - offsets   - group of all offsets and stored positions
 *  - all       - group of all groups
 *
 * _do_group_list() - get and print all groups in the list (iteration)
 * _do_motors()     - get and print motor uber group 1-N
 * _do_axes()       - get and print axis uber group XYZABC
 * _do_offsets()    - get and print offset uber group G54-G59, G28, G30, G92
 * _do_inputs()     - get and print inputs uber group di1 - diN
 * _do_outputs()    - get and print outputs uber group do1 - doN
 * _do_all()        - get and print all groups uber group
 */

static void _do_group(nvObj_t *nv, char *group)   // helper to a group
{
    nv_reset_nv_list();
    nv = nv_body;
    strncpy(nv->token, group, TOKEN_LEN);
    nv->index = nv_get_index((const char *)"", nv->token);
    nv_get_nvObj(nv);
    nv_print_list(STAT_OK, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT);
}

static stat_t _do_group_list(nvObj_t *nv, char list[][TOKEN_LEN+1]) // helper to print multiple groups in a list
{
    for (uint8_t i=0; i < NV_MAX_OBJECTS; i++) {
        if (list[i][0] == NUL) {
            return (STAT_COMPLETE);
        }
        _do_group(nv, list[i]);
    }
    return (STAT_COMPLETE);         // STAT_COMPLETE suppresses the normal response line
}

static stat_t _do_motors(nvObj_t *nv)  // print parameters for all motor groups
{
    char group[GROUP_LEN];
    for (uint8_t i=1; i < MOTORS+1; i++) {
        sprintf(group, "%d", i);
        _do_group(nv, group);
    }
    return (STAT_COMPLETE);         // STAT_COMPLETE suppresses the normal response line
}

static stat_t _do_axes(nvObj_t *nv)  // print parameters for all axis groups
{
    char list[][TOKEN_LEN+1] = {"x","y","z","u","v","w","a","b","c",""}; // must have a terminating element
    return (_do_group_list(nv, list));
}

static stat_t _do_offsets(nvObj_t *nv)  // print offset parameters for G54-G59,G92, G28, G30
{
    char list[][TOKEN_LEN+1] = {"g54","g55","g56","g57","g58","g59","g92","g28","g30",""}; // must have a terminating element
    return (_do_group_list(nv, list));
}

static stat_t _do_inputs(nvObj_t *nv)  // print parameters for all input groups
{
    char group[GROUP_LEN];
    for (uint8_t i=1; i < D_IN_CHANNELS+1; i++) {
        sprintf(group, "di%d", i);
        _do_group(nv, group);
    }
    return (STAT_COMPLETE);         // STAT_COMPLETE suppresses the normal response line
}

static stat_t _do_outputs(nvObj_t *nv)  // print parameters for all output groups
{
    char group[GROUP_LEN];
    for (uint8_t i=1; i < D_OUT_CHANNELS+1; i++) {
        sprintf(group, "do%d", i);
        _do_group(nv, group);
    }
    return (STAT_COMPLETE);         // STAT_COMPLETE suppresses the normal response line
}

static stat_t _do_heaters(nvObj_t *nv)  // print parameters for all heater groups
{
    char group[GROUP_LEN];
    for (uint8_t i=1; i < 4; i++) {
        sprintf(group, "he%d", i);
        _do_group(nv, group);
    }
    return (STAT_COMPLETE);         // STAT_COMPLETE suppresses the normal response line
}

static stat_t _do_all(nvObj_t *nv)  // print all parameters
{
    _do_group(nv, (char *)"sys");   // System group
    _do_motors(nv);
    _do_axes(nv);
    _do_inputs(nv);
    _do_outputs(nv);
    _do_heaters(nv);                // there are no text mode prints for heaters
    _do_group(nv, (char *)"p1");    // PWM group
    _do_offsets(nv);                // coordinate system offsets
    return (STAT_COMPLETE);         // STAT_COMPLETE suppresses a second JSON write that would cause a fault
}

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 * Most of these can be found in their respective modules.
 ***********************************************************************************/

/**** COMMUNICATIONS FUNCTIONS ******************************************************
 * get_rx()   - get bytes available in RX buffer
 * get_tick() - get system tick count
 */

static stat_t get_rx(nvObj_t *nv)
{
    nv->value_int = (float)254;        // ARM always says the serial buffer is available (max)
    nv->valuetype = TYPE_INTEGER;
    return (STAT_OK);
}

static stat_t get_tick(nvObj_t *nv)
{
    nv->value_int = (float)SysTickTimer.getValue();
    nv->valuetype = TYPE_INTEGER;
    return (STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

static const char fmt_rx[] = "rx:%d\n";
static const char fmt_ex[] = "[ex]  enable flow control%10d [0=off,1=XON/XOFF, 2=RTS/CTS]\n";

void cfg_print_rx(nvObj_t *nv) { text_print(nv, fmt_rx);}       // TYPE_INT
void cfg_print_ex(nvObj_t *nv) { text_print(nv, fmt_ex);}       // TYPE_INT

#endif // __TEXT_MODE
