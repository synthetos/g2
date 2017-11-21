/*
 * config_app.cpp - application-specific part of configuration data
 * This file is part of the g2core project
 *
 * Copyright (c) 2013 - 2017 Alden S. Hart, Jr.
 * Copyright (c) 2016 - 2017 Robert Giseburt
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
#include "gcode_parser.h"
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
 *  - If the count of lines in cfgArray exceeds 255 (which it does) you need to 
 *    ensure index_t is uint16_t in the config.h file (and not uint8_t).
 *
 *  - The precision value 'p' only affects JSON responses. You need to also set
 *    the %f in the corresponding format string to set text mode display precision
 */
const cfgItem_t cfgArray[] = {
    // group token flags p, print_func,  get_func,  set_func, target for get/set,       default value
    { "sys", "fb", _fipn,2, hw_print_fb, get_flt,   set_ro,   (float *)&cs.fw_build,    G2CORE_FIRMWARE_BUILD }, // MUST BE FIRST!
    { "sys", "fbs",_fn,  2, hw_print_fbs,hw_get_fbs,set_ro,   (float *)&cs.null, 0 },
    { "sys", "fbc",_fn,  2, hw_print_fbc,hw_get_fbc,set_ro,   (float *)&cs.null, 0 },
    { "sys", "fv", _fipn,2, hw_print_fv, get_flt,   set_ro,   (float *)&cs.fw_version,  G2CORE_FIRMWARE_VERSION },
    { "sys", "hp", _fipn,0, hw_print_hp, get_flt,   set_ro,   (float *)&cs.hw_platform, G2CORE_HARDWARE_PLATFORM },
    { "sys", "hv", _fipn,0, hw_print_hv, get_flt,   set_ro,   (float *)&cs.hw_version,  G2CORE_HARDWARE_VERSION },
    { "sys", "id", _fn,  0, hw_print_id, hw_get_id, set_ro,   (float *)&cs.null, 0 },   // device ID (ASCII signature)

    // dynamic model attributes for reporting purposes (up front for speed)
    { "",   "stat",_f0, 0, cm_print_stat, cm_get_stat, set_ro, (float *)&cs.null, 0 },      // combined machine state
    { "",   "n",   _fi, 0, cm_print_line, cm_get_mline,set_noop,(float *)&cs.null, 0 },     // Model line number
    { "",   "line",_fi, 0, cm_print_line, cm_get_line, set_ro, (float *)&cs.null, 0 },      // Active line number - model or runtime line number
    { "",   "vel", _f0, 2, cm_print_vel,  cm_get_vel,  set_ro, (float *)&cs.null, 0 },      // current velocity
    { "",   "feed",_f0, 2, cm_print_feed, cm_get_feed, set_ro, (float *)&cs.null, 0 },      // feed rate
    { "",   "macs",_f0, 0, cm_print_macs, cm_get_macs, set_ro, (float *)&cs.null, 0 },      // raw machine state
    { "",   "cycs",_f0, 0, cm_print_cycs, cm_get_cycs, set_ro, (float *)&cs.null, 0 },      // cycle state
    { "",   "mots",_f0, 0, cm_print_mots, cm_get_mots, set_ro, (float *)&cs.null, 0 },      // motion state
    { "",   "hold",_f0, 0, cm_print_hold, cm_get_hold, set_ro, (float *)&cs.null, 0 },      // feedhold state
    { "",   "unit",_f0, 0, cm_print_unit, cm_get_unit, set_ro, (float *)&cs.null, 0 },      // units mode
    { "",   "coor",_f0, 0, cm_print_coor, cm_get_coor, set_ro, (float *)&cs.null, 0 },      // coordinate system
    { "",   "momo",_f0, 0, cm_print_momo, cm_get_momo, set_ro, (float *)&cs.null, 0 },      // motion mode
    { "",   "plan",_f0, 0, cm_print_plan, cm_get_plan, set_ro, (float *)&cs.null, 0 },      // plane select
    { "",   "path",_f0, 0, cm_print_path, cm_get_path, set_ro, (float *)&cs.null, 0 },      // path control mode
    { "",   "dist",_f0, 0, cm_print_dist, cm_get_dist, set_ro, (float *)&cs.null, 0 },      // distance mode
    { "",   "admo",_f0, 0, cm_print_admo, cm_get_admo, set_ro, (float *)&cs.null, 0 },      // arc distance mode
    { "",   "frmo",_f0, 0, cm_print_frmo, cm_get_frmo, set_ro, (float *)&cs.null, 0 },      // feed rate mode
    { "",   "tool",_f0, 0, cm_print_tool, cm_get_toolv,set_ro, (float *)&cs.null, 0 },      // active tool
    { "",   "g92e",_f0, 0, cm_print_g92e, get_ui8,     set_ro, (float *)&cm.gmx.origin_offset_enable, 0 }, // G92 enabled

#ifdef TEMPORARY_HAS_LEDS
    { "",   "_leds",_f0, 0, tx_print_nul, _get_leds,_set_leds,(float *)&cs.null, 0 },      // TEMPORARY - change LEDs
#endif

    { "mpo","mpox",_f0, 3, cm_print_mpo, cm_get_mpo, set_ro, (float *)&cs.null, 0 },        // X machine position
    { "mpo","mpoy",_f0, 3, cm_print_mpo, cm_get_mpo, set_ro, (float *)&cs.null, 0 },        // Y machine position
    { "mpo","mpoz",_f0, 3, cm_print_mpo, cm_get_mpo, set_ro, (float *)&cs.null, 0 },        // Z machine position
    { "mpo","mpoa",_f0, 3, cm_print_mpo, cm_get_mpo, set_ro, (float *)&cs.null, 0 },        // A machine position
    { "mpo","mpob",_f0, 3, cm_print_mpo, cm_get_mpo, set_ro, (float *)&cs.null, 0 },        // B machine position
    { "mpo","mpoc",_f0, 3, cm_print_mpo, cm_get_mpo, set_ro, (float *)&cs.null, 0 },        // C machine position

    { "pos","posx",_f0, 3, cm_print_pos, cm_get_pos, set_ro, (float *)&cs.null, 0 },        // X work position
    { "pos","posy",_f0, 3, cm_print_pos, cm_get_pos, set_ro, (float *)&cs.null, 0 },        // Y work position
    { "pos","posz",_f0, 3, cm_print_pos, cm_get_pos, set_ro, (float *)&cs.null, 0 },        // Z work position
    { "pos","posa",_f0, 3, cm_print_pos, cm_get_pos, set_ro, (float *)&cs.null, 0 },        // A work position
    { "pos","posb",_f0, 3, cm_print_pos, cm_get_pos, set_ro, (float *)&cs.null, 0 },        // B work position
    { "pos","posc",_f0, 3, cm_print_pos, cm_get_pos, set_ro, (float *)&cs.null, 0 },        // C work position

    { "ofs","ofsx",_f0, 3, cm_print_ofs, cm_get_ofs, set_ro, (float *)&cs.null, 0 },        // X work offset
    { "ofs","ofsy",_f0, 3, cm_print_ofs, cm_get_ofs, set_ro, (float *)&cs.null, 0 },        // Y work offset
    { "ofs","ofsz",_f0, 3, cm_print_ofs, cm_get_ofs, set_ro, (float *)&cs.null, 0 },        // Z work offset
    { "ofs","ofsa",_f0, 3, cm_print_ofs, cm_get_ofs, set_ro, (float *)&cs.null, 0 },        // A work offset
    { "ofs","ofsb",_f0, 3, cm_print_ofs, cm_get_ofs, set_ro, (float *)&cs.null, 0 },        // B work offset
    { "ofs","ofsc",_f0, 3, cm_print_ofs, cm_get_ofs, set_ro, (float *)&cs.null, 0 },        // C work offset

    { "hom","home",_f0, 0, cm_print_home,cm_get_home,set_01,(float *)&cm.homing_state, 0 },     // homing state, invoke homing cycle
    { "hom","homx",_f0, 0, cm_print_hom, get_ui8, set_01, (float *)&cm.homed[AXIS_X], false },  // X homed - Homing status group
    { "hom","homy",_f0, 0, cm_print_hom, get_ui8, set_01, (float *)&cm.homed[AXIS_Y], false },  // Y homed
    { "hom","homz",_f0, 0, cm_print_hom, get_ui8, set_01, (float *)&cm.homed[AXIS_Z], false },  // Z homed
    { "hom","homa",_f0, 0, cm_print_hom, get_ui8, set_01, (float *)&cm.homed[AXIS_A], false },  // A homed
    { "hom","homb",_f0, 0, cm_print_hom, get_ui8, set_01, (float *)&cm.homed[AXIS_B], false },  // B homed
    { "hom","homc",_f0, 0, cm_print_hom, get_ui8, set_01, (float *)&cm.homed[AXIS_C], false },  // C homed

    { "prb","prbe",_f0, 0, tx_print_nul, get_ui8, set_ro, (float *)&cm.probe_state[0], 0 },     // probing state
    { "prb","prbx",_f0, 3, tx_print_nul, get_flt, set_ro, (float *)&cm.probe_results[0][AXIS_X], 0 },
    { "prb","prby",_f0, 3, tx_print_nul, get_flt, set_ro, (float *)&cm.probe_results[0][AXIS_Y], 0 },
    { "prb","prbz",_f0, 3, tx_print_nul, get_flt, set_ro, (float *)&cm.probe_results[0][AXIS_Z], 0 },
    { "prb","prba",_f0, 3, tx_print_nul, get_flt, set_ro, (float *)&cm.probe_results[0][AXIS_A], 0 },
    { "prb","prbb",_f0, 3, tx_print_nul, get_flt, set_ro, (float *)&cm.probe_results[0][AXIS_B], 0 },
    { "prb","prbc",_f0, 3, tx_print_nul, get_flt, set_ro, (float *)&cm.probe_results[0][AXIS_C], 0 },
    { "prb","prbr",_f0, 0, tx_print_nul, cm_get_prbr, cm_get_prbr, nullptr, 0 },    // enable probe report. Init in cm_init

    { "jog","jogx",_f0, 0, tx_print_nul, get_nul, cm_run_jogx, (float *)&cm.jogging_dest, 0},
    { "jog","jogy",_f0, 0, tx_print_nul, get_nul, cm_run_jogy, (float *)&cm.jogging_dest, 0},
    { "jog","jogz",_f0, 0, tx_print_nul, get_nul, cm_run_jogz, (float *)&cm.jogging_dest, 0},
    { "jog","joga",_f0, 0, tx_print_nul, get_nul, cm_run_joga, (float *)&cm.jogging_dest, 0},
//  { "jog","jogb",_f0, 0, tx_print_nul, get_nul, cm_run_jogb, (float *)&cm.jogging_dest, 0},
//  { "jog","jogc",_f0, 0, tx_print_nul, get_nul, cm_run_jogc, (float *)&cm.jogging_dest, 0},

	{ "pwr","pwr1",_f0, 3, st_print_pwr, st_get_pwr, set_ro,  (float *)&cs.null, 0},	// motor power readouts
	{ "pwr","pwr2",_f0, 3, st_print_pwr, st_get_pwr, set_ro,  (float *)&cs.null, 0},
#if (MOTORS > 2)
	{ "pwr","pwr3",_f0, 3, st_print_pwr, st_get_pwr, set_ro,  (float *)&cs.null, 0},
#endif
#if (MOTORS > 3)
	{ "pwr","pwr4",_f0, 3, st_print_pwr, st_get_pwr, set_ro,  (float *)&cs.null, 0},
#endif
#if (MOTORS > 4)
	{ "pwr","pwr5",_f0, 3, st_print_pwr, st_get_pwr, set_ro,  (float *)&cs.null, 0},
#endif
#if (MOTORS > 5)
	{ "pwr","pwr6",_f0, 3, st_print_pwr, st_get_pwr, set_ro,  (float *)&cs.null, 0},
#endif

    // Motor parameters
    { "1","1ma",_fip, 0, st_print_ma, get_ui8, st_set_ma,  (float *)&st_cfg.mot[MOTOR_1].motor_map,      M1_MOTOR_MAP },
    { "1","1sa",_fip, 3, st_print_sa, get_flt, st_set_sa,  (float *)&st_cfg.mot[MOTOR_1].step_angle,     M1_STEP_ANGLE },
    { "1","1tr",_fipc,4, st_print_tr, get_flt, st_set_tr,  (float *)&st_cfg.mot[MOTOR_1].travel_rev,     M1_TRAVEL_PER_REV },
    { "1","1mi",_fip, 0, st_print_mi, get_ui8, st_set_mi,  (float *)&st_cfg.mot[MOTOR_1].microsteps,     M1_MICROSTEPS },
    { "1","1su",_fipi,5, st_print_su, st_get_su,st_set_su, (float *)&st_cfg.mot[MOTOR_1].steps_per_unit, M1_STEPS_PER_UNIT },
    { "1","1po",_fip, 0, st_print_po, get_ui8, set_01,     (float *)&st_cfg.mot[MOTOR_1].polarity,       M1_POLARITY },
    { "1","1pm",_fip, 0, st_print_pm, st_get_pm,st_set_pm, (float *)&cs.null,                            M1_POWER_MODE },
    { "1","1pl",_fip, 3, st_print_pl, get_flt, st_set_pl,  (float *)&st_cfg.mot[MOTOR_1].power_level,    M1_POWER_LEVEL },
//  { "1","1pi",_fip, 3, st_print_pi, get_flt, st_set_pi,  (float *)&st_cfg.mot[MOTOR_1].power_idle,     M1_POWER_IDLE },
//  { "1","1mt",_fip, 2, st_print_mt, get_flt, st_set_mt,  (float *)&st_cfg.mot[MOTOR_1].motor_timeout,  M1_MOTOR_TIMEOUT },
#if (MOTORS >= 2)
    { "2","2ma",_fip, 0, st_print_ma, get_ui8, st_set_ma,  (float *)&st_cfg.mot[MOTOR_2].motor_map,      M2_MOTOR_MAP },
    { "2","2sa",_fip, 3, st_print_sa, get_flt, st_set_sa,  (float *)&st_cfg.mot[MOTOR_2].step_angle,     M2_STEP_ANGLE },
    { "2","2tr",_fipc,4, st_print_tr, get_flt, st_set_tr,  (float *)&st_cfg.mot[MOTOR_2].travel_rev,     M2_TRAVEL_PER_REV },
    { "2","2mi",_fip, 0, st_print_mi, get_ui8, st_set_mi,  (float *)&st_cfg.mot[MOTOR_2].microsteps,     M2_MICROSTEPS },
    { "2","2su",_fipi,5, st_print_su, st_get_su,st_set_su, (float *)&st_cfg.mot[MOTOR_2].steps_per_unit, M2_STEPS_PER_UNIT },
    { "2","2po",_fip, 0, st_print_po, get_ui8, set_01,     (float *)&st_cfg.mot[MOTOR_2].polarity,       M2_POLARITY },
    { "2","2pm",_fip, 0, st_print_pm, st_get_pm,st_set_pm, (float *)&cs.null,                            M2_POWER_MODE },
    { "2","2pl",_fip, 3, st_print_pl, get_flt, st_set_pl,  (float *)&st_cfg.mot[MOTOR_2].power_level,    M2_POWER_LEVEL},
//  { "2","2pi",_fip, 3, st_print_pi, get_flt, st_set_pi,  (float *)&st_cfg.mot[MOTOR_2].power_idle,     M2_POWER_IDLE },
//  { "2","2mt",_fip, 2, st_print_mt, get_flt, st_set_mt,  (float *)&st_cfg.mot[MOTOR_2].motor_timeout,  M2_MOTOR_TIMEOUT },
#endif
#if (MOTORS >= 3)
    { "3","3ma",_fip, 0, st_print_ma, get_ui8, st_set_ma,  (float *)&st_cfg.mot[MOTOR_3].motor_map,      M3_MOTOR_MAP },
    { "3","3sa",_fip, 3, st_print_sa, get_flt, st_set_sa,  (float *)&st_cfg.mot[MOTOR_3].step_angle,     M3_STEP_ANGLE },
    { "3","3tr",_fipc,4, st_print_tr, get_flt, st_set_tr,  (float *)&st_cfg.mot[MOTOR_3].travel_rev,     M3_TRAVEL_PER_REV },
    { "3","3mi",_fip, 0, st_print_mi, get_ui8, st_set_mi,  (float *)&st_cfg.mot[MOTOR_3].microsteps,     M3_MICROSTEPS },
    { "3","3su",_fipi,5, st_print_su, st_get_su,st_set_su, (float *)&st_cfg.mot[MOTOR_3].steps_per_unit, M3_STEPS_PER_UNIT },
    { "3","3po",_fip, 0, st_print_po, get_ui8, set_01,     (float *)&st_cfg.mot[MOTOR_3].polarity,       M3_POLARITY },
    { "3","3pm",_fip, 0, st_print_pm, st_get_pm,st_set_pm, (float *)&cs.null,                            M3_POWER_MODE },
    { "3","3pl",_fip, 3, st_print_pl, get_flt, st_set_pl,  (float *)&st_cfg.mot[MOTOR_3].power_level,    M3_POWER_LEVEL },
//  { "3","3pi",_fip, 3, st_print_pi, get_flt, st_set_pi,  (float *)&st_cfg.mot[MOTOR_3].power_idle,     M3_POWER_IDLE },
//  { "3","3mt",_fip, 2, st_print_mt, get_flt, st_set_mt,  (float *)&st_cfg.mot[MOTOR_3].motor_timeout,  M3_MOTOR_TIMEOUT },
#endif
#if (MOTORS >= 4)
    { "4","4ma",_fip, 0, st_print_ma, get_ui8, st_set_ma,  (float *)&st_cfg.mot[MOTOR_4].motor_map,      M4_MOTOR_MAP },
    { "4","4sa",_fip, 3, st_print_sa, get_flt, st_set_sa,  (float *)&st_cfg.mot[MOTOR_4].step_angle,     M4_STEP_ANGLE },
    { "4","4tr",_fipc,4, st_print_tr, get_flt, st_set_tr,  (float *)&st_cfg.mot[MOTOR_4].travel_rev,     M4_TRAVEL_PER_REV },
    { "4","4mi",_fip, 0, st_print_mi, get_ui8, st_set_mi,  (float *)&st_cfg.mot[MOTOR_4].microsteps,     M4_MICROSTEPS },
    { "4","4su",_fipi,5, st_print_su, st_get_su,st_set_su, (float *)&st_cfg.mot[MOTOR_4].steps_per_unit, M4_STEPS_PER_UNIT },
    { "4","4po",_fip, 0, st_print_po, get_ui8, set_01,     (float *)&st_cfg.mot[MOTOR_4].polarity,       M4_POLARITY },
    { "4","4pm",_fip, 0, st_print_pm, st_get_pm,st_set_pm, (float *)&cs.null,                            M4_POWER_MODE },
    { "4","4pl",_fip, 3, st_print_pl, get_flt, st_set_pl,  (float *)&st_cfg.mot[MOTOR_4].power_level,    M4_POWER_LEVEL },
//  { "4","4pi",_fip, 3, st_print_pi, get_flt, st_set_pi,  (float *)&st_cfg.mot[MOTOR_4].power_idle,     M4_POWER_IDLE },
//  { "4","4mt",_fip, 2, st_print_mt, get_flt, st_set_mt,  (float *)&st_cfg.mot[MOTOR_4].motor_timeout,  M4_MOTOR_TIMEOUT },
#endif
#if (MOTORS >= 5)
    { "5","5ma",_fip, 0, st_print_ma, get_ui8, st_set_ma,  (float *)&st_cfg.mot[MOTOR_5].motor_map,      M5_MOTOR_MAP },
    { "5","5sa",_fip, 3, st_print_sa, get_flt, st_set_sa,  (float *)&st_cfg.mot[MOTOR_5].step_angle,     M5_STEP_ANGLE },
    { "5","5tr",_fipc,4, st_print_tr, get_flt, st_set_tr,  (float *)&st_cfg.mot[MOTOR_5].travel_rev,     M5_TRAVEL_PER_REV },
    { "5","5mi",_fip, 0, st_print_mi, get_ui8, st_set_mi,  (float *)&st_cfg.mot[MOTOR_5].microsteps,     M5_MICROSTEPS },
    { "5","5su",_fipi,5, st_print_su, st_get_su,st_set_su, (float *)&st_cfg.mot[MOTOR_5].steps_per_unit, M5_STEPS_PER_UNIT },
    { "5","5po",_fip, 0, st_print_po, get_ui8, set_01,     (float *)&st_cfg.mot[MOTOR_5].polarity,       M5_POLARITY },
    { "5","5pm",_fip, 0, st_print_pm, st_get_pm,st_set_pm, (float *)&cs.null,                            M5_POWER_MODE },
    { "5","5pl",_fip, 3, st_print_pl, get_flt, st_set_pl,  (float *)&st_cfg.mot[MOTOR_5].power_level,    M5_POWER_LEVEL },
//  { "5","5pi",_fip, 3, st_print_pi, get_flt, st_set_pi,  (float *)&st_cfg.mot[MOTOR_5].power_idle,     M5_POWER_IDLE },
//  { "5","5mt",_fip, 2, st_print_mt, get_flt, st_set_mt,  (float *)&st_cfg.mot[MOTOR_5].motor_timeout,  M5_MOTOR_TIMEOUT },
#endif
#if (MOTORS >= 6)
    { "6","6ma",_fip, 0, st_print_ma, get_ui8, st_set_ma,  (float *)&st_cfg.mot[MOTOR_6].motor_map,      M6_MOTOR_MAP },
    { "6","6sa",_fip, 3, st_print_sa, get_flt, st_set_sa,  (float *)&st_cfg.mot[MOTOR_6].step_angle,     M6_STEP_ANGLE },
    { "6","6tr",_fipc,4, st_print_tr, get_flt, st_set_tr,  (float *)&st_cfg.mot[MOTOR_6].travel_rev,     M6_TRAVEL_PER_REV },
    { "6","6mi",_fip, 0, st_print_mi, get_ui8, st_set_mi,  (float *)&st_cfg.mot[MOTOR_6].microsteps,     M6_MICROSTEPS },
    { "6","6su",_fipi,5, st_print_su, st_get_su,st_set_su, (float *)&st_cfg.mot[MOTOR_6].steps_per_unit, M6_STEPS_PER_UNIT },
    { "6","6po",_fip, 0, st_print_po, get_ui8, set_01,     (float *)&st_cfg.mot[MOTOR_6].polarity,       M6_POLARITY },
    { "6","6pm",_fip, 0, st_print_pm, st_get_pm,st_set_pm, (float *)&cs.null,                            M6_POWER_MODE },
    { "6","6pl",_fip, 3, st_print_pl, get_flt, st_set_pl,  (float *)&st_cfg.mot[MOTOR_6].power_level,    M6_POWER_LEVEL },
//  { "6","6pi",_fip, 3, st_print_pi, get_flt, st_set_pi,  (float *)&st_cfg.mot[MOTOR_6].power_idle,     M6_POWER_IDLE },
//  { "6","6mt",_fip, 2, st_print_mt, get_flt, st_set_mt,  (float *)&st_cfg.mot[MOTOR_6].motor_timeout,  M6_MOTOR_TIMEOUT },
#endif
    // Axis parameters
    { "x","xam",_fip,  0, cm_print_am, cm_get_am, cm_set_am, (float *)&cm.a[AXIS_X].axis_mode,      X_AXIS_MODE },
    { "x","xvm",_fipc, 0, cm_print_vm, get_flt,   cm_set_vm, (float *)&cm.a[AXIS_X].velocity_max,   X_VELOCITY_MAX },
    { "x","xfr",_fipc, 0, cm_print_fr, get_flt,   cm_set_fr, (float *)&cm.a[AXIS_X].feedrate_max,   X_FEEDRATE_MAX },
    { "x","xtn",_fipc, 3, cm_print_tn, get_flt,   set_flu,   (float *)&cm.a[AXIS_X].travel_min,     X_TRAVEL_MIN },
    { "x","xtm",_fipc, 3, cm_print_tm, get_flt,   set_flu,   (float *)&cm.a[AXIS_X].travel_max,     X_TRAVEL_MAX },
    { "x","xjm",_fipc, 0, cm_print_jm, get_flt,   cm_set_jm, (float *)&cm.a[AXIS_X].jerk_max,       X_JERK_MAX },
    { "x","xjh",_fipc, 0, cm_print_jh, get_flt,   cm_set_jh, (float *)&cm.a[AXIS_X].jerk_high,      X_JERK_HIGH_SPEED },
    { "x","xhi",_fip,  0, cm_print_hi, get_ui8,   cm_set_hi, (float *)&cm.a[AXIS_X].homing_input,   X_HOMING_INPUT },
    { "x","xhd",_fip,  0, cm_print_hd, get_ui8,   set_01,    (float *)&cm.a[AXIS_X].homing_dir,     X_HOMING_DIRECTION },
    { "x","xsv",_fipc, 0, cm_print_sv, get_flt,   set_flup,  (float *)&cm.a[AXIS_X].search_velocity,X_SEARCH_VELOCITY },
    { "x","xlv",_fipc, 2, cm_print_lv, get_flt,   set_flup,  (float *)&cm.a[AXIS_X].latch_velocity, X_LATCH_VELOCITY },
    { "x","xlb",_fipc, 3, cm_print_lb, get_flt,   set_flu,   (float *)&cm.a[AXIS_X].latch_backoff,  X_LATCH_BACKOFF },
    { "x","xzb",_fipc, 3, cm_print_zb, get_flt,   set_flu,   (float *)&cm.a[AXIS_X].zero_backoff,   X_ZERO_BACKOFF },

    { "y","yam",_fip,  0, cm_print_am, cm_get_am, cm_set_am, (float *)&cm.a[AXIS_Y].axis_mode,      Y_AXIS_MODE },
    { "y","yvm",_fipc, 0, cm_print_vm, get_flt,   cm_set_vm, (float *)&cm.a[AXIS_Y].velocity_max,   Y_VELOCITY_MAX },
    { "y","yfr",_fipc, 0, cm_print_fr, get_flt,   cm_set_fr, (float *)&cm.a[AXIS_Y].feedrate_max,   Y_FEEDRATE_MAX },
    { "y","ytn",_fipc, 3, cm_print_tn, get_flt,   set_flu,   (float *)&cm.a[AXIS_Y].travel_min,     Y_TRAVEL_MIN },
    { "y","ytm",_fipc, 3, cm_print_tm, get_flt,   set_flu,   (float *)&cm.a[AXIS_Y].travel_max,     Y_TRAVEL_MAX },
    { "y","yjm",_fipc, 0, cm_print_jm, get_flt,   cm_set_jm, (float *)&cm.a[AXIS_Y].jerk_max,       Y_JERK_MAX },
    { "y","yjh",_fipc, 0, cm_print_jh, get_flt,   cm_set_jh, (float *)&cm.a[AXIS_Y].jerk_high,      Y_JERK_HIGH_SPEED },
    { "y","yhi",_fip,  0, cm_print_hi, get_ui8,   cm_set_hi, (float *)&cm.a[AXIS_Y].homing_input,   Y_HOMING_INPUT },
    { "y","yhd",_fip,  0, cm_print_hd, get_ui8,   set_01,    (float *)&cm.a[AXIS_Y].homing_dir,     Y_HOMING_DIRECTION },
    { "y","ysv",_fipc, 0, cm_print_sv, get_flt,   set_flup,  (float *)&cm.a[AXIS_Y].search_velocity,Y_SEARCH_VELOCITY },
    { "y","ylv",_fipc, 2, cm_print_lv, get_flt,   set_flup,  (float *)&cm.a[AXIS_Y].latch_velocity, Y_LATCH_VELOCITY },
    { "y","ylb",_fipc, 3, cm_print_lb, get_flt,   set_flu,   (float *)&cm.a[AXIS_Y].latch_backoff,  Y_LATCH_BACKOFF },
    { "y","yzb",_fipc, 3, cm_print_zb, get_flt,   set_flu,   (float *)&cm.a[AXIS_Y].zero_backoff,   Y_ZERO_BACKOFF },

    { "z","zam",_fip,  0, cm_print_am, cm_get_am, cm_set_am, (float *)&cm.a[AXIS_Z].axis_mode,      Z_AXIS_MODE },
    { "z","zvm",_fipc, 0, cm_print_vm, get_flt,   cm_set_vm, (float *)&cm.a[AXIS_Z].velocity_max,   Z_VELOCITY_MAX },
    { "z","zfr",_fipc, 0, cm_print_fr, get_flt,   cm_set_fr, (float *)&cm.a[AXIS_Z].feedrate_max,   Z_FEEDRATE_MAX },
    { "z","ztn",_fipc, 3, cm_print_tn, get_flt,   set_flu,   (float *)&cm.a[AXIS_Z].travel_min,     Z_TRAVEL_MIN },
    { "z","ztm",_fipc, 3, cm_print_tm, get_flt,   set_flu,   (float *)&cm.a[AXIS_Z].travel_max,     Z_TRAVEL_MAX },
    { "z","zjm",_fipc, 0, cm_print_jm, get_flt,   cm_set_jm, (float *)&cm.a[AXIS_Z].jerk_max,       Z_JERK_MAX },
    { "z","zjh",_fipc, 0, cm_print_jh, get_flt,   cm_set_jh, (float *)&cm.a[AXIS_Z].jerk_high,      Z_JERK_HIGH_SPEED },
    { "z","zhi",_fip,  0, cm_print_hi, get_ui8,   cm_set_hi, (float *)&cm.a[AXIS_Z].homing_input,   Z_HOMING_INPUT },
    { "z","zhd",_fip,  0, cm_print_hd, get_ui8,   set_01,    (float *)&cm.a[AXIS_Z].homing_dir,     Z_HOMING_DIRECTION },
    { "z","zsv",_fipc, 0, cm_print_sv, get_flt,   set_flup,  (float *)&cm.a[AXIS_Z].search_velocity,Z_SEARCH_VELOCITY },
    { "z","zlv",_fipc, 2, cm_print_lv, get_flt,   set_flup,  (float *)&cm.a[AXIS_Z].latch_velocity, Z_LATCH_VELOCITY },
    { "z","zlb",_fipc, 3, cm_print_lb, get_flt,   set_flu,   (float *)&cm.a[AXIS_Z].latch_backoff,  Z_LATCH_BACKOFF },
    { "z","zzb",_fipc, 3, cm_print_zb, get_flt,   set_flu,   (float *)&cm.a[AXIS_Z].zero_backoff,   Z_ZERO_BACKOFF },

    { "a","aam",_fip,  0, cm_print_am, cm_get_am, cm_set_am, (float *)&cm.a[AXIS_A].axis_mode,      A_AXIS_MODE },
    { "a","avm",_fip,  0, cm_print_vm, get_flt,   cm_set_vm, (float *)&cm.a[AXIS_A].velocity_max,   A_VELOCITY_MAX },
    { "a","afr",_fip,  0, cm_print_fr, get_flt,   cm_set_fr, (float *)&cm.a[AXIS_A].feedrate_max,   A_FEEDRATE_MAX },
    { "a","atn",_fip,  3, cm_print_tn, get_flt,   set_flt,   (float *)&cm.a[AXIS_A].travel_min,     A_TRAVEL_MIN },
    { "a","atm",_fip,  3, cm_print_tm, get_flt,   set_flt,   (float *)&cm.a[AXIS_A].travel_max,     A_TRAVEL_MAX },
    { "a","ajm",_fip,  0, cm_print_jm, get_flt,   cm_set_jm, (float *)&cm.a[AXIS_A].jerk_max,       A_JERK_MAX },
    { "a","ajh",_fip,  0, cm_print_jh, get_flt,   cm_set_jh, (float *)&cm.a[AXIS_A].jerk_high,      A_JERK_HIGH_SPEED },
    { "a","ara",_fipc, 3, cm_print_ra, get_flt,   set_flt,   (float *)&cm.a[AXIS_A].radius,         A_RADIUS},
    { "a","ahi",_fip,  0, cm_print_hi, get_ui8,   cm_set_hi, (float *)&cm.a[AXIS_A].homing_input,   A_HOMING_INPUT },
    { "a","ahd",_fip,  0, cm_print_hd, get_ui8,   set_01,    (float *)&cm.a[AXIS_A].homing_dir,     A_HOMING_DIRECTION },
    { "a","asv",_fip,  0, cm_print_sv, get_flt,   set_fltp,  (float *)&cm.a[AXIS_A].search_velocity,A_SEARCH_VELOCITY },
    { "a","alv",_fip,  2, cm_print_lv, get_flt,   set_fltp,  (float *)&cm.a[AXIS_A].latch_velocity, A_LATCH_VELOCITY },
    { "a","alb",_fip,  3, cm_print_lb, get_flt,   set_flt,   (float *)&cm.a[AXIS_A].latch_backoff,  A_LATCH_BACKOFF },
    { "a","azb",_fip,  3, cm_print_zb, get_flt,   set_flt,   (float *)&cm.a[AXIS_A].zero_backoff,   A_ZERO_BACKOFF },

    { "b","bam",_fip,  0, cm_print_am, cm_get_am, cm_set_am, (float *)&cm.a[AXIS_B].axis_mode,      B_AXIS_MODE },
    { "b","bvm",_fip,  0, cm_print_vm, get_flt,   cm_set_vm, (float *)&cm.a[AXIS_B].velocity_max,   B_VELOCITY_MAX },
    { "b","bfr",_fip,  0, cm_print_fr, get_flt,   cm_set_fr, (float *)&cm.a[AXIS_B].feedrate_max,   B_FEEDRATE_MAX },
    { "b","btn",_fip,  3, cm_print_tn, get_flt,   set_flt,   (float *)&cm.a[AXIS_B].travel_min,     B_TRAVEL_MIN },
    { "b","btm",_fip,  3, cm_print_tm, get_flt,   set_flt,   (float *)&cm.a[AXIS_B].travel_max,     B_TRAVEL_MAX },
    { "b","bjm",_fip,  0, cm_print_jm, get_flt,   cm_set_jm, (float *)&cm.a[AXIS_B].jerk_max,       B_JERK_MAX },
    { "b","bjh",_fip,  0, cm_print_jh, get_flt,   cm_set_jh, (float *)&cm.a[AXIS_B].jerk_high,      B_JERK_HIGH_SPEED },
    { "b","bra",_fipc, 3, cm_print_ra, get_flt,   set_flt,   (float *)&cm.a[AXIS_B].radius,         B_RADIUS },
    { "b","bhi",_fip,  0, cm_print_hi, get_ui8,   cm_set_hi, (float *)&cm.a[AXIS_B].homing_input,   B_HOMING_INPUT },
    { "b","bhd",_fip,  0, cm_print_hd, get_ui8,   set_01,    (float *)&cm.a[AXIS_B].homing_dir,     B_HOMING_DIRECTION },
    { "b","bsv",_fip,  0, cm_print_sv, get_flt,   set_fltp,  (float *)&cm.a[AXIS_B].search_velocity,B_SEARCH_VELOCITY },
    { "b","blv",_fip,  2, cm_print_lv, get_flt,   set_fltp,  (float *)&cm.a[AXIS_B].latch_velocity, B_LATCH_VELOCITY },
    { "b","blb",_fip,  3, cm_print_lb, get_flt,   set_flt,   (float *)&cm.a[AXIS_B].latch_backoff,  B_LATCH_BACKOFF },
    { "b","bzb",_fip,  3, cm_print_zb, get_flt,   set_flt,   (float *)&cm.a[AXIS_B].zero_backoff,   B_ZERO_BACKOFF },

    { "c","cam",_fip,  0, cm_print_am, cm_get_am, cm_set_am, (float *)&cm.a[AXIS_C].axis_mode,      C_AXIS_MODE },
    { "c","cvm",_fip,  0, cm_print_vm, get_flt,   cm_set_vm, (float *)&cm.a[AXIS_C].velocity_max,   C_VELOCITY_MAX },
    { "c","cfr",_fip,  0, cm_print_fr, get_flt,   cm_set_fr, (float *)&cm.a[AXIS_C].feedrate_max,   C_FEEDRATE_MAX },
    { "c","ctn",_fip,  3, cm_print_tn, get_flt,   set_flt,   (float *)&cm.a[AXIS_C].travel_min,     C_TRAVEL_MIN },
    { "c","ctm",_fip,  3, cm_print_tm, get_flt,   set_flt,   (float *)&cm.a[AXIS_C].travel_max,     C_TRAVEL_MAX },
    { "c","cjm",_fip,  0, cm_print_jm, get_flt,   cm_set_jm, (float *)&cm.a[AXIS_C].jerk_max,       C_JERK_MAX },
    { "c","cjh",_fip,  0, cm_print_jh, get_flt,   cm_set_jh, (float *)&cm.a[AXIS_C].jerk_high,       C_JERK_HIGH_SPEED },
    { "c","cra",_fipc, 3, cm_print_ra, get_flt,   set_flt,   (float *)&cm.a[AXIS_C].radius,         C_RADIUS },
    { "c","chi",_fip,  0, cm_print_hi, get_ui8,   cm_set_hi, (float *)&cm.a[AXIS_C].homing_input,   C_HOMING_INPUT },
    { "c","chd",_fip,  0, cm_print_hd, get_ui8,   set_01,    (float *)&cm.a[AXIS_C].homing_dir,     C_HOMING_DIRECTION },
    { "c","csv",_fip,  0, cm_print_sv, get_flt,   set_fltp,  (float *)&cm.a[AXIS_C].search_velocity,C_SEARCH_VELOCITY },
    { "c","clv",_fip,  2, cm_print_lv, get_flt,   set_fltp,  (float *)&cm.a[AXIS_C].latch_velocity, C_LATCH_VELOCITY },
    { "c","clb",_fip,  3, cm_print_lb, get_flt,   set_flt,   (float *)&cm.a[AXIS_C].latch_backoff,  C_LATCH_BACKOFF },
    { "c","czb",_fip,  3, cm_print_zb, get_flt,   set_flt,   (float *)&cm.a[AXIS_C].zero_backoff,   C_ZERO_BACKOFF },

    // Digital input configs
    { "di1","di1mo",_fip, 0, io_print_mo, get_int8,io_set_mo, (float *)&d_in[0].mode,     DI1_MODE },
    { "di1","di1ac",_fip, 0, io_print_ac, get_ui8, io_set_ac, (float *)&d_in[0].action,   DI1_ACTION },
    { "di1","di1fn",_fip, 0, io_print_fn, get_ui8, io_set_fn, (float *)&d_in[0].function, DI1_FUNCTION },

    { "di2","di2mo",_fip, 0, io_print_mo, get_int8,io_set_mo, (float *)&d_in[1].mode,     DI2_MODE },
    { "di2","di2ac",_fip, 0, io_print_ac, get_ui8, io_set_ac, (float *)&d_in[1].action,   DI2_ACTION },
    { "di2","di2fn",_fip, 0, io_print_fn, get_ui8, io_set_fn, (float *)&d_in[1].function, DI2_FUNCTION },

    { "di3","di3mo",_fip, 0, io_print_mo, get_int8,io_set_mo, (float *)&d_in[2].mode,     DI3_MODE },
    { "di3","di3ac",_fip, 0, io_print_ac, get_ui8, io_set_ac, (float *)&d_in[2].action,   DI3_ACTION },
    { "di3","di3fn",_fip, 0, io_print_fn, get_ui8, io_set_fn, (float *)&d_in[2].function, DI3_FUNCTION },

    { "di4","di4mo",_fip, 0, io_print_mo, get_int8,io_set_mo, (float *)&d_in[3].mode,     DI4_MODE },
    { "di4","di4ac",_fip, 0, io_print_ac, get_ui8, io_set_ac, (float *)&d_in[3].action,   DI4_ACTION },
    { "di4","di4fn",_fip, 0, io_print_fn, get_ui8, io_set_fn, (float *)&d_in[3].function, DI4_FUNCTION },

    { "di5","di5mo",_fip, 0, io_print_mo, get_int8,io_set_mo, (float *)&d_in[4].mode,     DI5_MODE },
    { "di5","di5ac",_fip, 0, io_print_ac, get_ui8, io_set_ac, (float *)&d_in[4].action,   DI5_ACTION },
    { "di5","di5fn",_fip, 0, io_print_fn, get_ui8, io_set_fn, (float *)&d_in[4].function, DI5_FUNCTION },

    { "di6","di6mo",_fip, 0, io_print_mo, get_int8,io_set_mo, (float *)&d_in[5].mode,     DI6_MODE },
    { "di6","di6ac",_fip, 0, io_print_ac, get_ui8, io_set_ac, (float *)&d_in[5].action,   DI6_ACTION },
    { "di6","di6fn",_fip, 0, io_print_fn, get_ui8, io_set_fn, (float *)&d_in[5].function, DI6_FUNCTION },

    { "di7","di7mo",_fip, 0, io_print_mo, get_int8,io_set_mo, (float *)&d_in[6].mode,     DI7_MODE },
    { "di7","di7ac",_fip, 0, io_print_ac, get_ui8, io_set_ac, (float *)&d_in[6].action,   DI7_ACTION },
    { "di7","di7fn",_fip, 0, io_print_fn, get_ui8, io_set_fn, (float *)&d_in[6].function, DI7_FUNCTION },

    { "di8","di8mo",_fip, 0, io_print_mo, get_int8,io_set_mo, (float *)&d_in[7].mode,     DI8_MODE },
    { "di8","di8ac",_fip, 0, io_print_ac, get_ui8, io_set_ac, (float *)&d_in[7].action,   DI8_ACTION },
    { "di8","di8fn",_fip, 0, io_print_fn, get_ui8, io_set_fn, (float *)&d_in[7].function, DI8_FUNCTION },
#if (D_IN_CHANNELS >= 9)
    { "di9","di9mo",_fip, 0, io_print_mo, get_int8,io_set_mo, (float *)&d_in[8].mode,     DI9_MODE },
    { "di9","di9ac",_fip, 0, io_print_ac, get_ui8, io_set_ac, (float *)&d_in[8].action,   DI9_ACTION },
    { "di9","di9fn",_fip, 0, io_print_fn, get_ui8, io_set_fn, (float *)&d_in[8].function, DI9_FUNCTION },
#endif
#if (D_IN_CHANNELS >= 10)
    { "di10","di10mo",_fip, 0, io_print_mo, get_int8,io_set_mo, (float *)&d_in[9].mode,     DI10_MODE },
    { "di10","di10ac",_fip, 0, io_print_ac, get_ui8, io_set_ac, (float *)&d_in[9].action,   DI10_ACTION },
    { "di10","di10fn",_fip, 0, io_print_fn, get_ui8, io_set_fn, (float *)&d_in[9].function, DI10_FUNCTION },
#endif
#if (D_IN_CHANNELS >= 11)
    { "di11","di11mo",_fip, 0, io_print_mo, get_int8,io_set_mo, (float *)&d_in[10].mode,     DI11_MODE },
    { "di11","di11ac",_fip, 0, io_print_ac, get_ui8, io_set_ac, (float *)&d_in[10].action,   DI11_ACTION },
    { "di11","di11fn",_fip, 0, io_print_fn, get_ui8, io_set_fn, (float *)&d_in[10].function, DI11_FUNCTION },
#endif
#if (D_IN_CHANNELS >= 12)
    { "di12","di12mo",_fip, 0, io_print_mo, get_int8,io_set_mo, (float *)&d_in[11].mode,     DI12_MODE },
    { "di12","di12ac",_fip, 0, io_print_ac, get_ui8, io_set_ac, (float *)&d_in[11].action,   DI12_ACTION },
    { "di12","di12fn",_fip, 0, io_print_fn, get_ui8, io_set_fn, (float *)&d_in[11].function, DI12_FUNCTION },
#endif

    // Digital input state readers
    { "in","in1", _f0, 0, io_print_in, io_get_input, set_ro,  (float *)&cs.null, 0 },
    { "in","in2", _f0, 0, io_print_in, io_get_input, set_ro,  (float *)&cs.null, 0 },
    { "in","in3", _f0, 0, io_print_in, io_get_input, set_ro,  (float *)&cs.null, 0 },
    { "in","in4", _f0, 0, io_print_in, io_get_input, set_ro,  (float *)&cs.null, 0 },
    { "in","in5", _f0, 0, io_print_in, io_get_input, set_ro,  (float *)&cs.null, 0 },
    { "in","in6", _f0, 0, io_print_in, io_get_input, set_ro,  (float *)&cs.null, 0 },
    { "in","in7", _f0, 0, io_print_in, io_get_input, set_ro,  (float *)&cs.null, 0 },
    { "in","in8", _f0, 0, io_print_in, io_get_input, set_ro,  (float *)&cs.null, 0 },
#if (D_IN_CHANNELS >= 9)
    { "in","in9", _f0, 0, io_print_in, io_get_input, set_ro,  (float *)&cs.null, 0 },
#endif
#if (D_IN_CHANNELS >= 10)
    { "in","in10", _f0, 0, io_print_in, io_get_input, set_ro,  (float *)&cs.null, 0 },
#endif
#if (D_IN_CHANNELS >= 11)
    { "in","in11", _f0, 0, io_print_in, io_get_input, set_ro,  (float *)&cs.null, 0 },
#endif
#if (D_IN_CHANNELS >= 12)
    { "in","in12", _f0, 0, io_print_in, io_get_input, set_ro,  (float *)&cs.null, 0 },
#endif

    // digital output configs
    { "do1", "do1mo", _fip, 0, io_print_domode, get_int8, io_set_domode, (float *)&d_out[0].mode,     DO1_MODE },
    { "do2", "do2mo", _fip, 0, io_print_domode, get_int8, io_set_domode, (float *)&d_out[1].mode,     DO2_MODE },
    { "do3", "do3mo", _fip, 0, io_print_domode, get_int8, io_set_domode, (float *)&d_out[2].mode,     DO3_MODE },
    { "do4", "do4mo", _fip, 0, io_print_domode, get_int8, io_set_domode, (float *)&d_out[3].mode,     DO4_MODE },
    { "do5", "do5mo", _fip, 0, io_print_domode, get_int8, io_set_domode, (float *)&d_out[4].mode,     DO5_MODE },
    { "do6", "do6mo", _fip, 0, io_print_domode, get_int8, io_set_domode, (float *)&d_out[5].mode,     DO6_MODE },
    { "do7", "do7mo", _fip, 0, io_print_domode, get_int8, io_set_domode, (float *)&d_out[6].mode,     DO7_MODE },
    { "do8", "do8mo", _fip, 0, io_print_domode, get_int8, io_set_domode, (float *)&d_out[7].mode,     DO8_MODE },
    { "do9", "do9mo", _fip, 0, io_print_domode, get_int8, io_set_domode, (float *)&d_out[8].mode,     DO9_MODE },
    { "do10","do10mo",_fip, 0, io_print_domode, get_int8, io_set_domode, (float *)&d_out[9].mode,     DO10_MODE },
    { "do11","do11mo",_fip, 0, io_print_domode, get_int8, io_set_domode, (float *)&d_out[10].mode,    DO11_MODE },
    { "do12","do12mo",_fip, 0, io_print_domode, get_int8, io_set_domode, (float *)&d_out[11].mode,    DO12_MODE },
    { "do13","do13mo",_fip, 0, io_print_domode, get_int8, io_set_domode, (float *)&d_out[12].mode,    DO13_MODE },

    // Digital output state readers (default to non-active)
    { "out","out1",  _f0, 2, io_print_out, io_get_output, io_set_output, (float *)&cs.null, 0 },
    { "out","out2",  _f0, 2, io_print_out, io_get_output, io_set_output, (float *)&cs.null, 0 },
    { "out","out3",  _f0, 2, io_print_out, io_get_output, io_set_output, (float *)&cs.null, 0 },
    { "out","out4",  _f0, 2, io_print_out, io_get_output, io_set_output, (float *)&cs.null, 0 },
    { "out","out5",  _f0, 2, io_print_out, io_get_output, io_set_output, (float *)&cs.null, 0 },
    { "out","out6",  _f0, 2, io_print_out, io_get_output, io_set_output, (float *)&cs.null, 0 },
    { "out","out7",  _f0, 2, io_print_out, io_get_output, io_set_output, (float *)&cs.null, 0 },
    { "out","out8",  _f0, 2, io_print_out, io_get_output, io_set_output, (float *)&cs.null, 0 },
    { "out","out9",  _f0, 2, io_print_out, io_get_output, io_set_output, (float *)&cs.null, 0 },
    { "out","out10", _f0, 2, io_print_out, io_get_output, io_set_output, (float *)&cs.null, 0 },
    { "out","out11", _f0, 2, io_print_out, io_get_output, io_set_output, (float *)&cs.null, 0 },
    { "out","out12", _f0, 2, io_print_out, io_get_output, io_set_output, (float *)&cs.null, 0 },

    // PWM settings
    { "p1","p1frq",_fip, 0, pwm_print_p1frq, get_flt, pwm_set_pwm,(float *)&pwm.c[PWM_1].frequency,     P1_PWM_FREQUENCY },
    { "p1","p1csl",_fip, 0, pwm_print_p1csl, get_flt, pwm_set_pwm,(float *)&pwm.c[PWM_1].cw_speed_lo,   P1_CW_SPEED_LO },
    { "p1","p1csh",_fip, 0, pwm_print_p1csh, get_flt, pwm_set_pwm,(float *)&pwm.c[PWM_1].cw_speed_hi,   P1_CW_SPEED_HI },
    { "p1","p1cpl",_fip, 3, pwm_print_p1cpl, get_flt, pwm_set_pwm,(float *)&pwm.c[PWM_1].cw_phase_lo,   P1_CW_PHASE_LO },
    { "p1","p1cph",_fip, 3, pwm_print_p1cph, get_flt, pwm_set_pwm,(float *)&pwm.c[PWM_1].cw_phase_hi,   P1_CW_PHASE_HI },
    { "p1","p1wsl",_fip, 0, pwm_print_p1wsl, get_flt, pwm_set_pwm,(float *)&pwm.c[PWM_1].ccw_speed_lo,  P1_CCW_SPEED_LO },
    { "p1","p1wsh",_fip, 0, pwm_print_p1wsh, get_flt, pwm_set_pwm,(float *)&pwm.c[PWM_1].ccw_speed_hi,  P1_CCW_SPEED_HI },
    { "p1","p1wpl",_fip, 3, pwm_print_p1wpl, get_flt, pwm_set_pwm,(float *)&pwm.c[PWM_1].ccw_phase_lo,  P1_CCW_PHASE_LO },
    { "p1","p1wph",_fip, 3, pwm_print_p1wph, get_flt, pwm_set_pwm,(float *)&pwm.c[PWM_1].ccw_phase_hi,  P1_CCW_PHASE_HI },
    { "p1","p1pof",_fip, 3, pwm_print_p1pof, get_flt, pwm_set_pwm,(float *)&pwm.c[PWM_1].phase_off,     P1_PWM_PHASE_OFF },

    // temperature configs - pid active values (read-only)
    // NOTICE: If you change these PID group keys, you MUST change the get/set functions too!!
    { "pid1","pid1p",_f0, 3, tx_print_nul, cm_get_pid_p, set_ro, (float *)&cs.null, 0 },
    { "pid1","pid1i",_f0, 5, tx_print_nul, cm_get_pid_i, set_ro, (float *)&cs.null, 0 },
    { "pid1","pid1d",_f0, 5, tx_print_nul, cm_get_pid_d, set_ro, (float *)&cs.null, 0 },

    { "pid2","pid2p",_f0, 3, tx_print_nul, cm_get_pid_p, set_ro, (float *)&cs.null, 0 },
    { "pid2","pid2i",_f0, 5, tx_print_nul, cm_get_pid_i, set_ro, (float *)&cs.null, 0 },
    { "pid2","pid2d",_f0, 5, tx_print_nul, cm_get_pid_d, set_ro, (float *)&cs.null, 0 },

    { "pid3","pid3p",_f0, 3, tx_print_nul, cm_get_pid_p, set_ro, (float *)&cs.null, 0 },
    { "pid3","pid3i",_f0, 5, tx_print_nul, cm_get_pid_i, set_ro, (float *)&cs.null, 0 },
    { "pid3","pid3d",_f0, 5, tx_print_nul, cm_get_pid_d, set_ro, (float *)&cs.null, 0 },

    // temperature configs - heater set values (read-write)
    // NOTICE: If you change these heater group keys, you MUST change the get/set functions too!!
    { "he1","he1e", _fip, 0, tx_print_nul, cm_get_heater_enable,   cm_set_heater_enable,   (float *)&cs.null, H1_DEFAULT_ENABLE },
    { "he1","he1p", _fi,  3, tx_print_nul, cm_get_heater_p,        cm_set_heater_p,        (float *)&cs.null, H1_DEFAULT_P },
    { "he1","he1i", _fi,  5, tx_print_nul, cm_get_heater_i,        cm_set_heater_i,        (float *)&cs.null, H1_DEFAULT_I },
    { "he1","he1d", _fi,  5, tx_print_nul, cm_get_heater_d,        cm_set_heater_d,        (float *)&cs.null, H1_DEFAULT_D },
    { "he1","he1st",_f0,  1, tx_print_nul, cm_get_set_temperature, cm_set_set_temperature, (float *)&cs.null, 0 },
    { "he1","he1t", _f0,  1, tx_print_nul, cm_get_temperature,     set_ro,                 (float *)&cs.null, 0 },
    { "he1","he1op",_f0,  3, tx_print_nul, cm_get_heater_output,   set_ro,                 (float *)&cs.null, 0 },
    { "he1","he1tr",_f0,  3, tx_print_nul, cm_get_thermistor_resistance, set_ro,           (float *)&cs.null, 0 },
    { "he1","he1at",_f0,  0, tx_print_nul, cm_get_at_temperature,  set_ro,                 (float *)&cs.null, 0 },
    { "he1","he1an",_f0,  0, tx_print_nul, cm_get_heater_adc,      set_ro,                 (float *)&cs.null, 0 },
    { "he1","he1fp",_f0,  1, tx_print_nul, cm_get_fan_power,       cm_set_fan_power,       (float *)&cs.null, 0 },
    { "he1","he1fm",_f0,  1, tx_print_nul, cm_get_fan_min_power,   cm_set_fan_min_power,   (float *)&cs.null, 0 },
    { "he1","he1fl",_f0,  1, tx_print_nul, cm_get_fan_low_temp,    cm_set_fan_low_temp,    (float *)&cs.null, 0 },
    { "he1","he1fh",_f0,  1, tx_print_nul, cm_get_fan_high_temp,   cm_set_fan_high_temp,   (float *)&cs.null, 0 },

    { "he2","he2e", _fip, 0, tx_print_nul, cm_get_heater_enable,   cm_set_heater_enable,   (float *)&cs.null, H2_DEFAULT_ENABLE },
    { "he2","he2p", _fi,  3, tx_print_nul, cm_get_heater_p,        cm_set_heater_p,        (float *)&cs.null, H2_DEFAULT_P },
    { "he2","he2i", _fi,  5, tx_print_nul, cm_get_heater_i,        cm_set_heater_i,        (float *)&cs.null, H2_DEFAULT_I },
    { "he2","he2d", _fi,  5, tx_print_nul, cm_get_heater_d,        cm_set_heater_d,        (float *)&cs.null, H2_DEFAULT_D },
    { "he2","he2st",_f0,  0, tx_print_nul, cm_get_set_temperature, cm_set_set_temperature, (float *)&cs.null, 0 },
    { "he2","he2t", _f0,  1, tx_print_nul, cm_get_temperature,     set_ro,                 (float *)&cs.null, 0 },
    { "he2","he2op",_f0,  3, tx_print_nul, cm_get_heater_output,   set_ro,                 (float *)&cs.null, 0 },
    { "he2","he2tr",_f0,  3, tx_print_nul, cm_get_thermistor_resistance, set_ro,           (float *)&cs.null, 0 },
    { "he2","he2at",_f0,  0, tx_print_nul, cm_get_at_temperature,  set_ro,                 (float *)&cs.null, 0 },
    { "he2","he2an",_f0,  0, tx_print_nul, cm_get_heater_adc,      set_ro,                 (float *)&cs.null, 0 },
    { "he2","he2fp",_f0,  1, tx_print_nul, cm_get_fan_power,       cm_set_fan_power,       (float *)&cs.null, 0 },
    { "he2","he2fm",_f0,  1, tx_print_nul, cm_get_fan_min_power,   cm_set_fan_min_power,   (float *)&cs.null, 0 },
    { "he2","he2fl",_f0,  1, tx_print_nul, cm_get_fan_low_temp,    cm_set_fan_low_temp,    (float *)&cs.null, 0 },
    { "he2","he2fh",_f0,  1, tx_print_nul, cm_get_fan_high_temp,   cm_set_fan_high_temp,   (float *)&cs.null, 0 },

    { "he3","he3e", _fip, 0, tx_print_nul, cm_get_heater_enable,   cm_set_heater_enable,   (float *)&cs.null, H3_DEFAULT_ENABLE },
    { "he3","he3p", _fi,  3, tx_print_nul, cm_get_heater_p,        cm_set_heater_p,        (float *)&cs.null, H3_DEFAULT_P },
    { "he3","he3i", _fi,  5, tx_print_nul, cm_get_heater_i,        cm_set_heater_i,        (float *)&cs.null, H3_DEFAULT_I },
    { "he3","he3d", _fi,  5, tx_print_nul, cm_get_heater_d,        cm_set_heater_d,        (float *)&cs.null, H3_DEFAULT_D },
    { "he3","he3st",_f0,  0, tx_print_nul, cm_get_set_temperature, cm_set_set_temperature, (float *)&cs.null, 0 },
    { "he3","he3t", _f0,  1, tx_print_nul, cm_get_temperature,     set_ro,                 (float *)&cs.null, 0 },
    { "he3","he3op",_f0,  3, tx_print_nul, cm_get_heater_output,   set_ro,                 (float *)&cs.null, 0 },
    { "he3","he3tr",_f0,  3, tx_print_nul, cm_get_thermistor_resistance, set_ro,           (float *)&cs.null, 0 },
    { "he3","he3at",_f0,  0, tx_print_nul, cm_get_at_temperature,  set_ro,                 (float *)&cs.null, 0 },
    { "he3","he3an",_f0,  0, tx_print_nul, cm_get_heater_adc,      set_ro,                 (float *)&cs.null, 0 },
    { "he3","he3fp",_f0,  1, tx_print_nul, cm_get_fan_power,       cm_set_fan_power,       (float *)&cs.null, 0 },
    { "he3","he3fm",_f0,  1, tx_print_nul, cm_get_fan_min_power,   cm_set_fan_min_power,   (float *)&cs.null, 0 },
    { "he3","he3fl",_f0,  1, tx_print_nul, cm_get_fan_low_temp,    cm_set_fan_low_temp,    (float *)&cs.null, 0 },
    { "he3","he3fh",_f0,  1, tx_print_nul, cm_get_fan_high_temp,   cm_set_fan_high_temp,   (float *)&cs.null, 0 },

    // Coordinate system offsets (G54-G59 and G92)
    { "g54","g54x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G54][AXIS_X], G54_X_OFFSET },
    { "g54","g54y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G54][AXIS_Y], G54_Y_OFFSET },
    { "g54","g54z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G54][AXIS_Z], G54_Z_OFFSET },
    { "g54","g54a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.offset[G54][AXIS_A], G54_A_OFFSET },
    { "g54","g54b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.offset[G54][AXIS_B], G54_B_OFFSET },
    { "g54","g54c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.offset[G54][AXIS_C], G54_C_OFFSET },

    { "g55","g55x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G55][AXIS_X], G55_X_OFFSET },
    { "g55","g55y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G55][AXIS_Y], G55_Y_OFFSET },
    { "g55","g55z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G55][AXIS_Z], G55_Z_OFFSET },
    { "g55","g55a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.offset[G55][AXIS_A], G55_A_OFFSET },
    { "g55","g55b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.offset[G55][AXIS_B], G55_B_OFFSET },
    { "g55","g55c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.offset[G55][AXIS_C], G55_C_OFFSET },

    { "g56","g56x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G56][AXIS_X], G56_X_OFFSET },
    { "g56","g56y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G56][AXIS_Y], G56_Y_OFFSET },
    { "g56","g56z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G56][AXIS_Z], G56_Z_OFFSET },
    { "g56","g56a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.offset[G56][AXIS_A], G56_A_OFFSET },
    { "g56","g56b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.offset[G56][AXIS_B], G56_B_OFFSET },
    { "g56","g56c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.offset[G56][AXIS_C], G56_C_OFFSET },

    { "g57","g57x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G57][AXIS_X], G57_X_OFFSET },
    { "g57","g57y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G57][AXIS_Y], G57_Y_OFFSET },
    { "g57","g57z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G57][AXIS_Z], G57_Z_OFFSET },
    { "g57","g57a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.offset[G57][AXIS_A], G57_A_OFFSET },
    { "g57","g57b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.offset[G57][AXIS_B], G57_B_OFFSET },
    { "g57","g57c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.offset[G57][AXIS_C], G57_C_OFFSET },

    { "g58","g58x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G58][AXIS_X], G58_X_OFFSET },
    { "g58","g58y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G58][AXIS_Y], G58_Y_OFFSET },
    { "g58","g58z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G58][AXIS_Z], G58_Z_OFFSET },
    { "g58","g58a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.offset[G58][AXIS_A], G58_A_OFFSET },
    { "g58","g58b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.offset[G58][AXIS_B], G58_B_OFFSET },
    { "g58","g58c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.offset[G58][AXIS_C], G58_C_OFFSET },

    { "g59","g59x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G59][AXIS_X], G59_X_OFFSET },
    { "g59","g59y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G59][AXIS_Y], G59_Y_OFFSET },
    { "g59","g59z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.offset[G59][AXIS_Z], G59_Z_OFFSET },
    { "g59","g59a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.offset[G59][AXIS_A], G59_A_OFFSET },
    { "g59","g59b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.offset[G59][AXIS_B], G59_B_OFFSET },
    { "g59","g59c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.offset[G59][AXIS_C], G59_C_OFFSET },

    { "g92","g92x",_fic, 3, cm_print_cofs, get_flt, set_ro, (float *)&cm.gmx.origin_offset[AXIS_X], 0 },// G92 handled differently
    { "g92","g92y",_fic, 3, cm_print_cofs, get_flt, set_ro, (float *)&cm.gmx.origin_offset[AXIS_Y], 0 },
    { "g92","g92z",_fic, 3, cm_print_cofs, get_flt, set_ro, (float *)&cm.gmx.origin_offset[AXIS_Z], 0 },
    { "g92","g92a",_fi,  3, cm_print_cofs, get_flt, set_ro, (float *)&cm.gmx.origin_offset[AXIS_A], 0 },
    { "g92","g92b",_fi,  3, cm_print_cofs, get_flt, set_ro, (float *)&cm.gmx.origin_offset[AXIS_B], 0 },
    { "g92","g92c",_fi,  3, cm_print_cofs, get_flt, set_ro, (float *)&cm.gmx.origin_offset[AXIS_C], 0 },

    // Coordinate positions (G28, G30)
    { "g28","g28x",_fic, 3, cm_print_cpos, get_flt, set_ro, (float *)&cm.gmx.g28_position[AXIS_X], 0 },// g28 handled differently
    { "g28","g28y",_fic, 3, cm_print_cpos, get_flt, set_ro, (float *)&cm.gmx.g28_position[AXIS_Y], 0 },
    { "g28","g28z",_fic, 3, cm_print_cpos, get_flt, set_ro, (float *)&cm.gmx.g28_position[AXIS_Z], 0 },
    { "g28","g28a",_fi,  3, cm_print_cpos, get_flt, set_ro, (float *)&cm.gmx.g28_position[AXIS_A], 0 },
    { "g28","g28b",_fi,  3, cm_print_cpos, get_flt, set_ro, (float *)&cm.gmx.g28_position[AXIS_B], 0 },
    { "g28","g28c",_fi,  3, cm_print_cpos, get_flt, set_ro, (float *)&cm.gmx.g28_position[AXIS_C], 0 },

    { "g30","g30x",_fic, 3, cm_print_cpos, get_flt, set_ro, (float *)&cm.gmx.g30_position[AXIS_X], 0 },// g30 handled differently
    { "g30","g30y",_fic, 3, cm_print_cpos, get_flt, set_ro, (float *)&cm.gmx.g30_position[AXIS_Y], 0 },
    { "g30","g30z",_fic, 3, cm_print_cpos, get_flt, set_ro, (float *)&cm.gmx.g30_position[AXIS_Z], 0 },
    { "g30","g30a",_fi,  3, cm_print_cpos, get_flt, set_ro, (float *)&cm.gmx.g30_position[AXIS_A], 0 },
    { "g30","g30b",_fi,  3, cm_print_cpos, get_flt, set_ro, (float *)&cm.gmx.g30_position[AXIS_B], 0 },
    { "g30","g30c",_fi,  3, cm_print_cpos, get_flt, set_ro, (float *)&cm.gmx.g30_position[AXIS_C], 0 },

    // Default values for current tool length offsets (not configurable, set to zero)
    { "tof","tofx",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tl_offset[AXIS_X], 0 },
    { "tof","tofy",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tl_offset[AXIS_Y], 0 },
    { "tof","tofz",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tl_offset[AXIS_Z], 0 },
    { "tof","tofa",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tl_offset[AXIS_A], 0 },
    { "tof","tofb",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tl_offset[AXIS_B], 0 },
    { "tof","tofc",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tl_offset[AXIS_C], 0 },

    // Tool table offsets
    { "tt1","tt1x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[1][AXIS_X], TT1_X_OFFSET },
    { "tt1","tt1y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[1][AXIS_Y], TT1_Y_OFFSET },
    { "tt1","tt1z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[1][AXIS_Z], TT1_Z_OFFSET },
    { "tt1","tt1a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[1][AXIS_A], TT1_A_OFFSET },
    { "tt1","tt1b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[1][AXIS_B], TT1_B_OFFSET },
    { "tt1","tt1c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[1][AXIS_C], TT1_C_OFFSET },

    { "tt2","tt2x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[2][AXIS_X], TT2_X_OFFSET },
    { "tt2","tt2y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[2][AXIS_Y], TT2_Y_OFFSET },
    { "tt2","tt2z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[2][AXIS_Z], TT2_Z_OFFSET },
    { "tt2","tt2a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[2][AXIS_A], TT2_A_OFFSET },
    { "tt2","tt2b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[2][AXIS_B], TT2_B_OFFSET },
    { "tt2","tt2c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[2][AXIS_C], TT2_C_OFFSET },

    { "tt3","tt3x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[3][AXIS_X], TT3_X_OFFSET },
    { "tt3","tt3y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[3][AXIS_Y], TT3_Y_OFFSET },
    { "tt3","tt3z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[3][AXIS_Z], TT3_Z_OFFSET },
    { "tt3","tt3a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[3][AXIS_A], TT3_A_OFFSET },
    { "tt3","tt3b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[3][AXIS_B], TT3_B_OFFSET },
    { "tt3","tt3c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[3][AXIS_C], TT1_C_OFFSET },

    { "tt4","tt4x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[4][AXIS_X], TT4_X_OFFSET },
    { "tt4","tt4y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[4][AXIS_Y], TT4_Y_OFFSET },
    { "tt4","tt4z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[4][AXIS_Z], TT4_Z_OFFSET },
    { "tt4","tt4a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[4][AXIS_A], TT4_A_OFFSET },
    { "tt4","tt4b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[4][AXIS_B], TT4_B_OFFSET },
    { "tt4","tt4c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[4][AXIS_C], TT4_C_OFFSET },

    { "tt5","tt5x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[5][AXIS_X], TT5_X_OFFSET },
    { "tt5","tt5y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[5][AXIS_Y], TT5_Y_OFFSET },
    { "tt5","tt5z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[5][AXIS_Z], TT5_Z_OFFSET },
    { "tt5","tt5a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[5][AXIS_A], TT5_A_OFFSET },
    { "tt5","tt5b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[5][AXIS_B], TT5_B_OFFSET },
    { "tt5","tt5c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[5][AXIS_C], TT5_C_OFFSET },

    { "tt6","tt6x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[6][AXIS_X], TT6_X_OFFSET },
    { "tt6","tt6y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[6][AXIS_Y], TT6_Y_OFFSET },
    { "tt6","tt6z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[6][AXIS_Z], TT6_Z_OFFSET },
    { "tt6","tt6a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[6][AXIS_A], TT6_A_OFFSET },
    { "tt6","tt6b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[6][AXIS_B], TT6_B_OFFSET },
    { "tt6","tt6c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[6][AXIS_C], TT6_C_OFFSET },

    { "tt7","tt7x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[7][AXIS_X], TT7_X_OFFSET },
    { "tt7","tt7y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[7][AXIS_Y], TT7_Y_OFFSET },
    { "tt7","tt7z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[7][AXIS_Z], TT7_Z_OFFSET },
    { "tt7","tt7a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[7][AXIS_A], TT7_A_OFFSET },
    { "tt7","tt7b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[7][AXIS_B], TT7_B_OFFSET },
    { "tt7","tt7c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[7][AXIS_C], TT7_C_OFFSET },

    { "tt8","tt8x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[8][AXIS_X], TT8_X_OFFSET },
    { "tt8","tt8y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[8][AXIS_Y], TT8_Y_OFFSET },
    { "tt8","tt8z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[8][AXIS_Z], TT8_Z_OFFSET },
    { "tt8","tt8a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[8][AXIS_A], TT8_A_OFFSET },
    { "tt8","tt8b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[8][AXIS_B], TT8_B_OFFSET },
    { "tt8","tt8c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[8][AXIS_C], TT8_C_OFFSET },

    { "tt9","tt9x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[9][AXIS_X], TT9_X_OFFSET },
    { "tt9","tt9y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[9][AXIS_Y], TT9_Y_OFFSET },
    { "tt9","tt9z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[9][AXIS_Z], TT9_Z_OFFSET },
    { "tt9","tt9a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[9][AXIS_A], TT9_A_OFFSET },
    { "tt9","tt9b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[9][AXIS_B], TT9_B_OFFSET },
    { "tt9","tt9c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[9][AXIS_C], TT9_C_OFFSET },

    { "tt10","tt10x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[10][AXIS_X], TT10_X_OFFSET },
    { "tt10","tt10y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[10][AXIS_Y], TT10_Y_OFFSET },
    { "tt10","tt10z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[10][AXIS_Z], TT10_Z_OFFSET },
    { "tt10","tt10a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[10][AXIS_A], TT10_A_OFFSET },
    { "tt10","tt10b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[10][AXIS_B], TT10_B_OFFSET },
    { "tt10","tt10c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[10][AXIS_C], TT10_C_OFFSET },

    { "tt11","tt11x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[11][AXIS_X], TT11_X_OFFSET },
    { "tt11","tt11y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[11][AXIS_Y], TT11_Y_OFFSET },
    { "tt11","tt11z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[11][AXIS_Z], TT11_Z_OFFSET },
    { "tt11","tt11a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[11][AXIS_A], TT11_A_OFFSET },
    { "tt11","tt11b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[11][AXIS_B], TT11_B_OFFSET },
    { "tt11","tt11c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[11][AXIS_C], TT11_C_OFFSET },

    { "tt12","tt12x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[12][AXIS_X], TT12_X_OFFSET },
    { "tt12","tt12y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[12][AXIS_Y], TT12_Y_OFFSET },
    { "tt12","tt12z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[12][AXIS_Z], TT12_Z_OFFSET },
    { "tt12","tt12a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[12][AXIS_A], TT12_A_OFFSET },
    { "tt12","tt12b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[12][AXIS_B], TT12_B_OFFSET },
    { "tt12","tt12c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[12][AXIS_C], TT12_C_OFFSET },

    { "tt13","tt13x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[13][AXIS_X], TT13_X_OFFSET },
    { "tt13","tt13y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[13][AXIS_Y], TT13_Y_OFFSET },
    { "tt13","tt13z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[13][AXIS_Z], TT13_Z_OFFSET },
    { "tt13","tt13a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[13][AXIS_A], TT13_A_OFFSET },
    { "tt13","tt13b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[13][AXIS_B], TT13_B_OFFSET },
    { "tt13","tt13c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[13][AXIS_C], TT13_C_OFFSET },

    { "tt14","tt14x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[14][AXIS_X], TT14_X_OFFSET },
    { "tt14","tt14y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[14][AXIS_Y], TT14_Y_OFFSET },
    { "tt14","tt14z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[14][AXIS_Z], TT14_Z_OFFSET },
    { "tt14","tt14a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[14][AXIS_A], TT14_A_OFFSET },
    { "tt14","tt14b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[14][AXIS_B], TT14_B_OFFSET },
    { "tt14","tt14c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[14][AXIS_C], TT14_C_OFFSET },

    { "tt15","tt15x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[15][AXIS_X], TT15_X_OFFSET },
    { "tt15","tt15y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[15][AXIS_Y], TT15_Y_OFFSET },
    { "tt15","tt15z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[15][AXIS_Z], TT15_Z_OFFSET },
    { "tt15","tt15a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[15][AXIS_A], TT15_A_OFFSET },
    { "tt15","tt15b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[15][AXIS_B], TT15_B_OFFSET },
    { "tt15","tt15c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[15][AXIS_C], TT15_C_OFFSET },

    { "tt16","tt16x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[16][AXIS_X], TT16_X_OFFSET },
    { "tt16","tt16y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[16][AXIS_Y], TT16_Y_OFFSET },
    { "tt16","tt16z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[16][AXIS_Z], TT16_Z_OFFSET },
    { "tt16","tt16a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[16][AXIS_A], TT16_A_OFFSET },
    { "tt16","tt16b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[16][AXIS_B], TT16_B_OFFSET },
    { "tt16","tt16c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[16][AXIS_C], TT16_C_OFFSET },

    { "tt17","tt17x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[17][AXIS_X], TT17_X_OFFSET },
    { "tt17","tt17y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[17][AXIS_Y], TT17_Y_OFFSET },
    { "tt17","tt17z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[17][AXIS_Z], TT17_Z_OFFSET },
    { "tt17","tt17a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[17][AXIS_A], TT17_A_OFFSET },
    { "tt17","tt17b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[17][AXIS_B], TT17_B_OFFSET },
    { "tt17","tt17c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[17][AXIS_C], TT17_C_OFFSET },

    { "tt18","tt18x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[18][AXIS_X], TT18_X_OFFSET },
    { "tt18","tt18y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[18][AXIS_Y], TT18_Y_OFFSET },
    { "tt18","tt18z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[18][AXIS_Z], TT18_Z_OFFSET },
    { "tt18","tt18a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[18][AXIS_A], TT18_A_OFFSET },
    { "tt18","tt18b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[18][AXIS_B], TT18_B_OFFSET },
    { "tt18","tt18c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[18][AXIS_C], TT18_C_OFFSET },

    { "tt19","tt19x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[19][AXIS_X], TT19_X_OFFSET },
    { "tt19","tt19y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[19][AXIS_Y], TT19_Y_OFFSET },
    { "tt19","tt19z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[19][AXIS_Z], TT19_Z_OFFSET },
    { "tt19","tt19a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[19][AXIS_A], TT19_A_OFFSET },
    { "tt19","tt19b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[19][AXIS_B], TT19_B_OFFSET },
    { "tt19","tt19c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[19][AXIS_C], TT19_C_OFFSET },

    { "tt20","tt20x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[20][AXIS_X], TT20_X_OFFSET },
    { "tt20","tt20y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[20][AXIS_Y], TT20_Y_OFFSET },
    { "tt20","tt20z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[20][AXIS_Z], TT20_Z_OFFSET },
    { "tt20","tt20a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[20][AXIS_A], TT20_A_OFFSET },
    { "tt20","tt20b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[20][AXIS_B], TT20_B_OFFSET },
    { "tt20","tt20c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[20][AXIS_C], TT20_C_OFFSET },

    { "tt21","tt21x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[21][AXIS_X], TT21_X_OFFSET },
    { "tt21","tt21y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[21][AXIS_Y], TT21_Y_OFFSET },
    { "tt21","tt21z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[21][AXIS_Z], TT21_Z_OFFSET },
    { "tt21","tt21a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[21][AXIS_A], TT21_A_OFFSET },
    { "tt21","tt21b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[21][AXIS_B], TT21_B_OFFSET },
    { "tt21","tt21c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[21][AXIS_C], TT21_C_OFFSET },

    { "tt22","tt22x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[22][AXIS_X], TT22_X_OFFSET },
    { "tt22","tt22y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[22][AXIS_Y], TT22_Y_OFFSET },
    { "tt22","tt22z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[22][AXIS_Z], TT22_Z_OFFSET },
    { "tt22","tt22a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[22][AXIS_A], TT22_A_OFFSET },
    { "tt22","tt22b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[22][AXIS_B], TT22_B_OFFSET },
    { "tt22","tt22c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[22][AXIS_C], TT22_C_OFFSET },

    { "tt23","tt23x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[23][AXIS_X], TT23_X_OFFSET },
    { "tt23","tt23y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[23][AXIS_Y], TT23_Y_OFFSET },
    { "tt23","tt23z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[23][AXIS_Z], TT23_Z_OFFSET },
    { "tt23","tt23a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[23][AXIS_A], TT23_A_OFFSET },
    { "tt23","tt23b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[23][AXIS_B], TT23_B_OFFSET },
    { "tt23","tt23c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[23][AXIS_C], TT23_C_OFFSET },

    { "tt24","tt24x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[24][AXIS_X], TT24_X_OFFSET },
    { "tt24","tt24y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[24][AXIS_Y], TT24_Y_OFFSET },
    { "tt24","tt24z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[24][AXIS_Z], TT24_Z_OFFSET },
    { "tt24","tt24a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[24][AXIS_A], TT24_A_OFFSET },
    { "tt24","tt24b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[24][AXIS_B], TT24_B_OFFSET },
    { "tt24","tt24c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[24][AXIS_C], TT24_C_OFFSET },

    { "tt25","tt25x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[25][AXIS_X], TT25_X_OFFSET },
    { "tt25","tt25y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[25][AXIS_Y], TT25_Y_OFFSET },
    { "tt25","tt25z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[25][AXIS_Z], TT25_Z_OFFSET },
    { "tt25","tt25a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[25][AXIS_A], TT25_A_OFFSET },
    { "tt25","tt25b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[25][AXIS_B], TT25_B_OFFSET },
    { "tt25","tt25c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[25][AXIS_C], TT25_C_OFFSET },

    { "tt26","tt26x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[26][AXIS_X], TT26_X_OFFSET },
    { "tt26","tt26y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[26][AXIS_Y], TT26_Y_OFFSET },
    { "tt26","tt26z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[26][AXIS_Z], TT26_Z_OFFSET },
    { "tt26","tt26a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[26][AXIS_A], TT26_A_OFFSET },
    { "tt26","tt26b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[26][AXIS_B], TT26_B_OFFSET },
    { "tt26","tt26c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[26][AXIS_C], TT26_C_OFFSET },

    { "tt27","tt27x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[27][AXIS_X], TT27_X_OFFSET },
    { "tt27","tt27y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[27][AXIS_Y], TT27_Y_OFFSET },
    { "tt27","tt27z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[27][AXIS_Z], TT27_Z_OFFSET },
    { "tt27","tt27a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[27][AXIS_A], TT27_A_OFFSET },
    { "tt27","tt27b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[27][AXIS_B], TT27_B_OFFSET },
    { "tt27","tt27c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[27][AXIS_C], TT27_C_OFFSET },

    { "tt28","tt28x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[28][AXIS_X], TT28_X_OFFSET },
    { "tt28","tt28y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[28][AXIS_Y], TT28_Y_OFFSET },
    { "tt28","tt28z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[28][AXIS_Z], TT28_Z_OFFSET },
    { "tt28","tt28a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[28][AXIS_A], TT28_A_OFFSET },
    { "tt28","tt28b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[28][AXIS_B], TT28_B_OFFSET },
    { "tt28","tt28c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[28][AXIS_C], TT28_C_OFFSET },

    { "tt29","tt29x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[29][AXIS_X], TT29_X_OFFSET },
    { "tt29","tt29y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[29][AXIS_Y], TT29_Y_OFFSET },
    { "tt29","tt29z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[29][AXIS_Z], TT29_Z_OFFSET },
    { "tt29","tt29a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[29][AXIS_A], TT29_A_OFFSET },
    { "tt29","tt29b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[29][AXIS_B], TT29_B_OFFSET },
    { "tt29","tt29c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[29][AXIS_C], TT29_C_OFFSET },

    { "tt30","tt30x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[30][AXIS_X], TT30_X_OFFSET },
    { "tt30","tt30y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[30][AXIS_Y], TT30_Y_OFFSET },
    { "tt30","tt30z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[30][AXIS_Z], TT30_Z_OFFSET },
    { "tt30","tt30a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[30][AXIS_A], TT30_A_OFFSET },
    { "tt30","tt30b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[30][AXIS_B], TT30_B_OFFSET },
    { "tt30","tt30c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[30][AXIS_C], TT30_C_OFFSET },

    { "tt31","tt31x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[31][AXIS_X], TT31_X_OFFSET },
    { "tt31","tt31y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[31][AXIS_Y], TT31_Y_OFFSET },
    { "tt31","tt31z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[31][AXIS_Z], TT31_Z_OFFSET },
    { "tt31","tt31a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[31][AXIS_A], TT31_A_OFFSET },
    { "tt31","tt31b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[31][AXIS_B], TT31_B_OFFSET },
    { "tt31","tt31c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[31][AXIS_C], TT31_C_OFFSET },

    { "tt32","tt32x",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[32][AXIS_X], TT32_X_OFFSET },
    { "tt32","tt32y",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[32][AXIS_Y], TT32_Y_OFFSET },
    { "tt32","tt32z",_fipc, 3, cm_print_cofs, get_flt, set_flu,(float *)&cm.tt_offset[32][AXIS_Z], TT32_Z_OFFSET },
    { "tt32","tt32a",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[32][AXIS_A], TT32_A_OFFSET },
    { "tt32","tt32b",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[32][AXIS_B], TT32_B_OFFSET },
    { "tt32","tt32c",_fip,  3, cm_print_cofs, get_flt, set_flt,(float *)&cm.tt_offset[32][AXIS_C], TT32_C_OFFSET },

    // this is a 128bit UUID for identifying a previously committed job state
    { "jid","jida",_f0, 0, tx_print_nul, get_data, set_data, (float *)&cfg.job_id[0], 0},
    { "jid","jidb",_f0, 0, tx_print_nul, get_data, set_data, (float *)&cfg.job_id[1], 0},
    { "jid","jidc",_f0, 0, tx_print_nul, get_data, set_data, (float *)&cfg.job_id[2], 0},
    { "jid","jidd",_f0, 0, tx_print_nul, get_data, set_data, (float *)&cfg.job_id[3], 0},

    // General system parameters
    { "sys","jt", _fipn, 2, cm_print_jt,  get_flt, cm_set_jt,(float *)&cm.junction_integration_time,JUNCTION_INTEGRATION_TIME },
    { "sys","ct", _fipnc,4, cm_print_ct,  get_flt, set_flup, (float *)&cm.chordal_tolerance,        CHORDAL_TOLERANCE },
    { "sys","sl", _fipn, 0, cm_print_sl,  get_ui8, set_01,   (float *)&cm.soft_limit_enable,        SOFT_LIMIT_ENABLE },
    { "sys","lim", _fipn,0, cm_print_lim, get_ui8, set_01,   (float *)&cm.limit_enable,             HARD_LIMIT_ENABLE },
    { "sys","saf", _fipn,0, cm_print_saf, get_ui8, set_01,   (float *)&cm.safety_interlock_enable,  SAFETY_INTERLOCK_ENABLE },
    { "sys","m48e",_fipn,0, cm_print_m48e,get_ui8, set_01,   (float *)&cm.gmx.m48_enable, 0 },      // M48/M49 feedrate & spindle override enable
    { "sys","mfoe",_fipn,0, cm_print_mfoe,get_ui8, set_01,   (float *)&cm.gmx.mfo_enable,           FEED_OVERRIDE_ENABLE},
    { "sys","mfo", _fipn,3, cm_print_mfo, get_flt,cm_set_mfo,(float *)&cm.gmx.mfo_factor,           FEED_OVERRIDE_FACTOR},
    { "sys","mtoe",_fipn,0, cm_print_mtoe,get_ui8, set_01,   (float *)&cm.gmx.mto_enable,           TRAVERSE_OVERRIDE_ENABLE},
    { "sys","mto", _fipn,3, cm_print_mto, get_flt,cm_set_mto,(float *)&cm.gmx.mto_factor,           TRAVERSE_OVERRIDE_FACTOR},
	
	// Power management 
    { "sys","mt",  _fipn,2, st_print_mt,  get_flt, st_set_mt,  (float *)&st_cfg.motor_power_timeout,  MOTOR_POWER_TIMEOUT},
    { "",   "me",  _f0,  0, st_print_me,  st_set_me, st_set_me,(float *)&cs.null, 0 },    // SET to enable  motors (null value sets to maintain compatability)
    { "",   "md",  _f0,  0, st_print_md,  st_set_md, st_set_md,(float *)&cs.null, 0 },    // SET to disable motors (null value sets to maintain compatability)

    // Spindle functions
    { "sys","spep",_fipn,0, cm_print_spep,get_ui8, set_01,   (float *)&spindle.enable_polarity,     SPINDLE_ENABLE_POLARITY },
    { "sys","spdp",_fipn,0, cm_print_spdp,get_ui8, set_01,   (float *)&spindle.dir_polarity,        SPINDLE_DIR_POLARITY },
    { "sys","spph",_fipn,0, cm_print_spph,get_ui8, set_01,   (float *)&spindle.pause_on_hold,       SPINDLE_PAUSE_ON_HOLD },
    { "sys","spdw",_fipn,2, cm_print_spdw,get_flt, set_flt,  (float *)&spindle.dwell_seconds,       SPINDLE_DWELL_TIME },
    { "sys","ssoe",_fipn,0, cm_print_ssoe,get_ui8, set_01,   (float *)&spindle.sso_enable,          SPINDLE_OVERRIDE_ENABLE},
    { "sys","sso", _fipn,3, cm_print_sso, get_flt,cm_set_sso,(float *)&spindle.sso_factor,          SPINDLE_OVERRIDE_FACTOR},
    { "",   "spe", _f0,  0, cm_print_spe, get_ui8, set_nul,  (float *)&spindle.enable, 0 },         // get spindle enable
    { "",   "spd", _f0,  0, cm_print_spd, get_ui8,cm_set_dir,(float *)&spindle.direction, 0 },      // get spindle direction
    { "",   "sps", _f0,  0, cm_print_sps, get_flt, set_nul,  (float *)&spindle.speed, 0 },          // get spindle speed

    // Coolant functions
    { "sys","cofp",_fipn,0, cm_print_cofp,get_ui8, set_01,   (float *)&coolant.flood_polarity,      COOLANT_FLOOD_POLARITY },
    { "sys","comp",_fipn,0, cm_print_comp,get_ui8, set_01,   (float *)&coolant.mist_polarity,       COOLANT_MIST_POLARITY },
    { "sys","coph",_fipn,0, cm_print_coph,get_ui8, set_01,   (float *)&coolant.pause_on_hold,       COOLANT_PAUSE_ON_HOLD },
    { "",   "com", _f0,  0, cm_print_com, get_ui8, set_nul,  (float *)&coolant.mist_enable, 0 },    // get mist coolant enable
    { "",   "cof", _f0,  0, cm_print_cof, get_ui8, set_nul,  (float *)&coolant.flood_enable, 0 },   // get flood coolant enable

    // Communications and reporting parameters
#ifdef __TEXT_MODE
    { "sys","tv", _fipn, 0, tx_print_tv,  get_ui8, set_01,     (float *)&txt.text_verbosity,        TEXT_VERBOSITY },
#endif
    { "sys","ej", _fipn, 0, js_print_ej,  get_ui8, json_set_ej,(float *)&cs.comm_mode,              COMM_MODE },
    { "sys","jv", _fipn, 0, js_print_jv,  get_ui8, json_set_jv,(float *)&js.json_verbosity,         JSON_VERBOSITY },
    { "sys","qv", _fipn, 0, qr_print_qv,  get_ui8, set_012,    (float *)&qr.queue_report_verbosity,  QR_OFF}, // default to OFF, set to QUEUE_REPORT_VERBOSITY after connected
    { "sys","sv", _fipn, 0, sr_print_sv,  get_ui8, set_012,    (float *)&sr.status_report_verbosity, SR_OFF}, // default to OFF, set to STATUS_REPORT_VERBOSITY after connectied
    { "sys","si", _fipn, 0, sr_print_si,  get_int, sr_set_si,  (float *)&sr.status_report_interval, STATUS_REPORT_INTERVAL_MS },
    { "", "nxln", _f0,   0, cm_print_nxln,cm_get_nxln,cm_set_nxln,(float *)&cs.null,                0 },

    // Gcode defaults
    // NOTE: The ordering within the gcode defaults is important for token resolution. gc must follow gco
    { "sys","gpl", _fipn, 0, cm_print_gpl, get_ui8, set_012, (float *)&cm.default_select_plane,     GCODE_DEFAULT_PLANE },
    { "sys","gun", _fipn, 0, cm_print_gun, get_ui8, set_01,  (float *)&cm.default_units_mode,       GCODE_DEFAULT_UNITS },
    { "sys","gco", _fipn, 0, cm_print_gco, get_ui8, set_ui8, (float *)&cm.default_coord_system,     GCODE_DEFAULT_COORD_SYSTEM },
    { "sys","gpa", _fipn, 0, cm_print_gpa, get_ui8, set_012, (float *)&cm.default_path_control,     GCODE_DEFAULT_PATH_CONTROL },
    { "sys","gdi", _fipn, 0, cm_print_gdi, get_ui8, set_01,  (float *)&cm.default_distance_mode,    GCODE_DEFAULT_DISTANCE_MODE },
    { "",   "gc",  _f0,   0, tx_print_nul, gc_get_gc,gc_run_gc,(float *)&cs.null, 0 },              // gcode block - must be last in this group

    // Actions and Reports
    { "", "sr",  _f0, 0, sr_print_sr,  sr_get,    sr_set,    (float *)&cs.null, 0 },    // request and set status reports
    { "", "qr",  _f0, 0, qr_print_qr,  qr_get,    set_ro,    (float *)&cs.null, 0 },    // get queue value - planner buffers available
    { "", "qi",  _f0, 0, qr_print_qi,  qi_get,    set_ro,    (float *)&cs.null, 0 },    // get queue value - buffers added to queue
    { "", "qo",  _f0, 0, qr_print_qo,  qo_get,    set_ro,    (float *)&cs.null, 0 },    // get queue value - buffers removed from queue
    { "", "er",  _f0, 0, tx_print_nul, rpt_er,    set_nul,   (float *)&cs.null, 0 },    // get bogus exception report for testing
    { "", "qf",  _f0, 0, tx_print_nul, get_nul,   cm_run_qf, (float *)&cs.null, 0 },    // SET to invoke queue flush
    { "", "rx",  _f0, 0, tx_print_int, get_rx,    set_ro,    (float *)&cs.null, 0 },    // get RX buffer bytes or packets
    { "", "msg", _f0, 0, tx_print_str, get_nul,   set_nul,   (float *)&cs.null, 0 },    // string for generic messages
    { "", "alarm",_f0,0, tx_print_nul, cm_alrm,   cm_alrm,   (float *)&cs.null, 0 },    // trigger alarm
    { "", "panic",_f0,0, tx_print_nul, cm_pnic,   cm_pnic,   (float *)&cs.null, 0 },    // trigger panic
    { "", "shutd",_f0,0, tx_print_nul, cm_shutd,  cm_shutd,  (float *)&cs.null, 0 },    // trigger shutdown
    { "", "clear",_f0,0, tx_print_nul, cm_clr,    cm_clr,    (float *)&cs.null, 0 },    // GET "clear" to clear alarm state
    { "", "clr",  _f0,0, tx_print_nul, cm_clr,    cm_clr,    (float *)&cs.null, 0 },    // synonym for "clear"
    { "", "tick", _f0,0, tx_print_int, get_tick,  set_nul,   (float *)&cs.null, 0 },    // get system time tic
    { "", "tram", _f0,0, cm_print_tram, cm_get_tram, cm_set_tram, (float *)&cs.null, 0 },// SET to attempt setting rotation matrix from probes
    { "", "defa",_f0, 0, tx_print_nul, help_defa, set_defaults,(float *)&cs.null,0 },   // set/print defaults / help screen
    { "", "flash",_f0,0, tx_print_nul, help_flash,hw_flash,  (float *)&cs.null,0 },

#ifdef __HELP_SCREENS
    { "", "help",_f0, 0, tx_print_nul, help_config, set_nul, (float *)&cs.null,0 }, // prints config help screen
    { "", "h",   _f0, 0, tx_print_nul, help_config, set_nul, (float *)&cs.null,0 }, // alias for "help"
#endif

#ifdef __USER_DATA
    // User defined data groups
    { "uda","uda0", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_a[0], USER_DATA_A0 },
    { "uda","uda1", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_a[1], USER_DATA_A1 },
    { "uda","uda2", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_a[2], USER_DATA_A2 },
    { "uda","uda3", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_a[3], USER_DATA_A3 },

    { "udb","udb0", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_b[0], USER_DATA_B0 },
    { "udb","udb1", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_b[1], USER_DATA_B1 },
    { "udb","udb2", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_b[2], USER_DATA_B2 },
    { "udb","udb3", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_b[3], USER_DATA_B3 },

    { "udc","udc0", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_c[0], USER_DATA_C0 },
    { "udc","udc1", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_c[1], USER_DATA_C1 },
    { "udc","udc2", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_c[2], USER_DATA_C2 },
    { "udc","udc3", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_c[3], USER_DATA_C3 },

    { "udd","udd0", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_d[0], USER_DATA_D0 },
    { "udd","udd1", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_d[1], USER_DATA_D1 },
    { "udd","udd2", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_d[2], USER_DATA_D2 },
    { "udd","udd3", _fip, 0, tx_print_int, get_data, set_data,(float *)&cfg.user_data_d[3], USER_DATA_D3 },
#endif

    // Diagnostic parameters
#ifdef __DIAGNOSTIC_PARAMETERS
    { "",    "clc",_f0, 0, tx_print_nul, st_clc,  st_clc, (float *)&cs.null, 0 },  // clear diagnostic step counters
    { "",   "_dam",_f0, 0, tx_print_nul, cm_dam,  cm_dam, (float *)&cs.null, 0 },  // dump active model

    { "_te","_tex",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target[AXIS_X], 0 }, // X target endpoint
    { "_te","_tey",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target[AXIS_Y], 0 },
    { "_te","_tez",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target[AXIS_Z], 0 },
    { "_te","_tea",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target[AXIS_A], 0 },
    { "_te","_teb",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target[AXIS_B], 0 },
    { "_te","_tec",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target[AXIS_C], 0 },

    { "_tr","_trx",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.gm.target[AXIS_X], 0 },  // X target runtime
    { "_tr","_try",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.gm.target[AXIS_Y], 0 },
    { "_tr","_trz",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.gm.target[AXIS_Z], 0 },
    { "_tr","_tra",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.gm.target[AXIS_A], 0 },
    { "_tr","_trb",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.gm.target[AXIS_B], 0 },
    { "_tr","_trc",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.gm.target[AXIS_C], 0 },

#if (MOTORS >= 1)
    { "_ts","_ts1",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target_steps[MOTOR_1], 0 },      // Motor 1 target steps
    { "_ps","_ps1",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.position_steps[MOTOR_1], 0 },    // Motor 1 position steps
    { "_cs","_cs1",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.commanded_steps[MOTOR_1], 0 },   // Motor 1 commanded steps (delayed steps)
    { "_es","_es1",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.encoder_steps[MOTOR_1], 0 },     // Motor 1 encoder steps
    { "_xs","_xs1",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&st_pre.mot[MOTOR_1].corrected_steps, 0 }, // Motor 1 correction steps applied
    { "_fe","_fe1",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.following_error[MOTOR_1], 0 },   // Motor 1 following error in steps
#endif
#if (MOTORS >= 2)
    { "_ts","_ts2",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target_steps[MOTOR_2], 0 },
    { "_ps","_ps2",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.position_steps[MOTOR_2], 0 },
    { "_cs","_cs2",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.commanded_steps[MOTOR_2], 0 },
    { "_es","_es2",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.encoder_steps[MOTOR_2], 0 },
    { "_xs","_xs2",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&st_pre.mot[MOTOR_2].corrected_steps, 0 },
    { "_fe","_fe2",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.following_error[MOTOR_2], 0 },
#endif
#if (MOTORS >= 3)
    { "_ts","_ts3",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target_steps[MOTOR_3], 0 },
    { "_ps","_ps3",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.position_steps[MOTOR_3], 0 },
    { "_cs","_cs3",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.commanded_steps[MOTOR_3], 0 },
    { "_es","_es3",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.encoder_steps[MOTOR_3], 0 },
    { "_xs","_xs3",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&st_pre.mot[MOTOR_3].corrected_steps, 0 },
    { "_fe","_fe3",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.following_error[MOTOR_3], 0 },
#endif
#if (MOTORS >= 4)
    { "_ts","_ts4",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target_steps[MOTOR_4], 0 },
    { "_ps","_ps4",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.position_steps[MOTOR_4], 0 },
    { "_cs","_cs4",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.commanded_steps[MOTOR_4], 0 },
    { "_es","_es4",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.encoder_steps[MOTOR_4], 0 },
    { "_xs","_xs4",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&st_pre.mot[MOTOR_4].corrected_steps, 0 },
    { "_fe","_fe4",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.following_error[MOTOR_4], 0 },
#endif
#if (MOTORS >= 5)
    { "_ts","_ts5",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target_steps[MOTOR_5], 0 },
    { "_ps","_ps5",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.position_steps[MOTOR_5], 0 },
    { "_cs","_cs5",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.commanded_steps[MOTOR_5], 0 },
    { "_es","_es5",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.encoder_steps[MOTOR_5], 0 },
    { "_xs","_xs6",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&st_pre.mot[MOTOR_5].corrected_steps, 0 },
    { "_fe","_fe5",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.following_error[MOTOR_5], 0 },
#endif
#if (MOTORS >= 6)
    { "_ts","_ts6",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.target_steps[MOTOR_6], 0 },
    { "_ps","_ps6",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.position_steps[MOTOR_6], 0 },
    { "_cs","_cs6",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.commanded_steps[MOTOR_6], 0 },
    { "_es","_es6",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.encoder_steps[MOTOR_6], 0 },
    { "_xs","_xs5",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&st_pre.mot[MOTOR_6].corrected_steps, 0 },
    { "_fe","_fe6",_f0, 2, tx_print_flt, get_flt, set_nul,(float *)&mr.following_error[MOTOR_6], 0 },
#endif
#endif  //  __DIAGNOSTIC_PARAMETERS

    // Persistence for status report - must be in sequence
    // *** Count must agree with NV_STATUS_REPORT_LEN in report.h ***
    { "","se00",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[0],0 },
    { "","se01",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[1],0 },
    { "","se02",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[2],0 },
    { "","se03",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[3],0 },
    { "","se04",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[4],0 },
    { "","se05",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[5],0 },
    { "","se06",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[6],0 },
    { "","se07",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[7],0 },
    { "","se08",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[8],0 },
    { "","se09",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[9],0 },
    { "","se10",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[10],0 },
    { "","se11",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[11],0 },
    { "","se12",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[12],0 },
    { "","se13",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[13],0 },
    { "","se14",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[14],0 },
    { "","se15",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[15],0 },
    { "","se16",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[16],0 },
    { "","se17",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[17],0 },
    { "","se18",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[18],0 },
    { "","se19",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[19],0 },
    { "","se20",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[20],0 },
    { "","se21",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[21],0 },
    { "","se22",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[22],0 },
    { "","se23",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[23],0 },
    { "","se24",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[24],0 },
    { "","se25",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[25],0 },
    { "","se26",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[26],0 },
    { "","se27",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[27],0 },
    { "","se28",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[28],0 },
    { "","se29",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[29],0 },
    { "","se30",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[30],0 },
    { "","se31",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[31],0 },
    { "","se32",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[32],0 },
    { "","se33",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[33],0 },
    { "","se34",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[34],0 },
    { "","se35",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[35],0 },
    { "","se36",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[36],0 },
    { "","se37",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[37],0 },
    { "","se38",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[38],0 },
    { "","se39",_fp, 0, tx_print_nul, get_int, set_int,(float *)&sr.status_report_list[39],0 },
    // Count is 40, since se00 counts as one.

    // Group lookups - must follow the single-valued entries for proper sub-string matching
    // *** Must agree with NV_COUNT_GROUPS below ***
    // *** START COUNTING FROM HERE ***
    // *** Do not count:
    //      - Optional motors (5 and 6)
    //      - Optional USER_DATA
    //      - Optional DIAGNOSTIC_PARAMETERS
    //      - Uber groups (count these separately)

    { "","sys",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // system group
    { "","p1", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // PWM 1 group
    // 2
    { "","1",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // motor groups
    { "","2",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","3",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","4",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
#if (MOTORS >= 5)
    { "","5",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
#endif
#if (MOTORS >= 6)
    { "","6",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
#endif
    // +4 = 6
    { "","x",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // axis groups
    { "","y",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","z",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","a",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","b",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","c",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    // +6 = 12
    { "","in",  _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },   // input state
    { "","di1", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },   // input configs
    { "","di2", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","di3", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","di4", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","di5", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","di6", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","di7", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","di8", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","di9", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    // +10 = 22
    { "","out", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },   // output state
    { "","do1", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },   // output configs
    { "","do2", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","do3", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","do4", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","do5", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","do6", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","do7", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","do8", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","do9", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","do10", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","do11", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","do12", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","do13", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    // +14 = 36
    { "","g54",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // coord offset groups
    { "","g55",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","g56",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","g57",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","g58",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","g59",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },
    { "","g92",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // origin offsets
    { "","g28",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // g28 home position
    { "","g30",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // g30 home position
    // +9 = 45
    { "","tof",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // tool offsets
    { "","tt1",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // tt offsets
    { "","tt2",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // tt offsets
    { "","tt3",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // tt offsets
    { "","tt4",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // tt offsets
    { "","tt5",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // tt offsets
    { "","tt6",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // tt offsets
    { "","tt7",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // tt offsets
    { "","tt8",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // tt offsets
    { "","tt9",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // tt offsets
    { "","tt10",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },   // tt offsets
    { "","tt11",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },   // tt offsets
    { "","tt12",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },   // tt offsets
    { "","tt13",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },   // tt offsets
    { "","tt14",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },   // tt offsets
    { "","tt15",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },   // tt offsets
    { "","tt16",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },   // tt offsets
    // +17 = 62
    { "","mpo",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // machine position group
    { "","pos",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // work position group
    { "","ofs",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // work offset group
    { "","hom",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // axis homing state group
    { "","prb",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // probing state group
    { "","pwr",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // motor power enagled group
    { "","jog",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // axis jogging state group
    { "","jid",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },    // job ID group
    // +8 = 70
    { "","he1", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },   // heater 1 group
    { "","he2", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },   // heater 2 group
    { "","he3", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },   // heater 3 group
    { "","pid1",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },   // PID 1 group
    { "","pid2",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },   // PID 2 group
    { "","pid3",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },   // PID 3 group
    // +6 = 76

#ifdef __USER_DATA
    { "","uda", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },  // user data group
    { "","udb", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },  // user data group
    { "","udc", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },  // user data group
    { "","udd", _f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },  // user data group
#endif
#ifdef __DIAGNOSTIC_PARAMETERS
    { "","_te",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },  // target axis endpoint group
    { "","_tr",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },  // target axis runtime group
    { "","_ts",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },  // target motor steps group
    { "","_ps",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },  // position motor steps group
    { "","_cs",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },  // commanded motor steps group
    { "","_es",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },  // encoder steps group
    { "","_xs",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },  // correction steps group
    { "","_fe",_f0, 0, tx_print_nul, get_grp, set_grp,(float *)&cs.null,0 },  // following error group
#endif

    // Uber-group (groups of groups, for text-mode displays only)
    // *** Must agree with NV_COUNT_UBER_GROUPS below ****
    { "", "m", _f0, 0, tx_print_nul, _do_motors, set_nul,(float *)&cs.null,0 },
    { "", "q", _f0, 0, tx_print_nul, _do_axes,   set_nul,(float *)&cs.null,0 },
    { "", "o", _f0, 0, tx_print_nul, _do_offsets,set_nul,(float *)&cs.null,0 },
    { "", "di", _f0, 0, tx_print_nul,_do_inputs, set_nul,(float *)&cs.null,0 },
    { "", "do", _f0, 0, tx_print_nul,_do_outputs,set_nul,(float *)&cs.null,0 },
    { "", "$", _f0, 0, tx_print_nul, _do_all,    set_nul,(float *)&cs.null,0 }
};

/***** Make sure these defines line up with any changes in the above table *****/

#define NV_COUNT_UBER_GROUPS    6     // count of uber-groups, above
#define FIXED_GROUPS            92    // count of fixed groups, excluding optional groups

#if (MOTORS >= 5)
#define MOTOR_GROUP_5           1
#else
#define MOTOR_GROUP_5           0
#endif

#if (MOTORS >= 6)
#define MOTOR_GROUP_6           1
#else
#define MOTOR_GROUP_6           0
#endif

#ifdef __USER_DATA
#define USER_DATA_GROUPS        4    // count of user data groups only
#else
#define USER_DATA_GROUPS        0
#endif

#ifdef __DIAGNOSTIC_PARAMETERS
#define DIAGNOSTIC_GROUPS       8    // count of diagnostic groups only
#else
#define DIAGNOSTIC_GROUPS       0
#endif

#define TEMPERATURE_GROUPS      6
#define NV_COUNT_GROUPS (FIXED_GROUPS + MOTOR_GROUP_5 + MOTOR_GROUP_6 + USER_DATA_GROUPS + DIAGNOSTIC_GROUPS + TEMPERATURE_GROUPS)

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
 * set_flu()  - set floating point number with G20/G21 units conversion
 * set_flup() - set positive floating point number with G20/G21 units conversion
 * set_fltp() - set positive floating point number with no units conversion
 *
 * The number 'setted' will have been delivered in external units (inches or mm).
 * It is written to the target memory location in internal canonical units (mm).
 * The original nv->value is also changed so persistence works correctly.
 * Displays should convert back from internal canonical form to external form.
 *
 *  !!!! WARNING !!!! set_flu() doesn't care about axes, so make sure you aren't passing it ABC axes
 */

stat_t set_flu(nvObj_t *nv)
{
    if (cm_get_units_mode(MODEL) == INCHES) {       // if in inches...
        nv->value *= MM_PER_INCH;                   // convert to canonical millimeter units
    }
    *((float *)GET_TABLE_WORD(target)) = nv->value; // write value as millimeters or degrees
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;
    return(STAT_OK);
}

stat_t set_flup(nvObj_t *nv)
{
    if (nv->value < 0) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    return (set_flu(nv));
}

stat_t set_fltp(nvObj_t *nv)
{
    if (nv->value < 0) {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    return (set_flt(nv));
}

/*
 * preprocess_float() - pre-process floating point number for units display
 *
 *  Apologies in advance for this twisty little function. This function is used to 
 *  convert the native, canonical form of a parameter (mm, or whatever), into a display
 *  format appropriate to the units mode in effect. It uses the flags in the config table
 *  to determine what type of conversion to perform. It's complicated by the fact that  
 *  only linear axes actually convert - rotaries do not. Plus, determining the axis for 
 *  a motor requires unraveling the motor mapping (handled in cm_get_axis_type()). 
 *  Also, there are global SYS group values that are not associated with any axis.
 *  Lastly, the steps-per-unit value (1su) is actually kept in inverse conversion form, 
 *  as its native form would be units-per-step.
 */

void preprocess_float(nvObj_t *nv)
{
    if (nv->valuetype != TYPE_FLOAT) { return; } // can be called non-destructively for any value type
    if (isnan((double)nv->value) || isinf((double)nv->value)) { return; } // trap illegal float values
    ///+++ transform these checks into NaN or INF strings with an error return?

    // We may need one of two types of units conversion, but only if in inches mode
    if (cm_get_units_mode(MODEL) == INCHES) {
        cmAxisType type = cm_get_axis_type(nv->index);      // linear, rotary or global
        if (cfgArray[nv->index].flags & F_CONVERT) {        // standard units conversion
            if ((type == AXIS_TYPE_LINEAR) || (type == AXIS_TYPE_SYSTEM)) {
                nv->value *= INCHES_PER_MM;
            }
        } else if (cfgArray[nv->index].flags & F_ICONVERT) {// inverse units conversion
            if ((type == AXIS_TYPE_LINEAR) || (type == AXIS_TYPE_SYSTEM)) {
                nv->value *= MM_PER_INCH;
            }
        }
    }
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;
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
    char list[][TOKEN_LEN+1] = {"x","y","z","a","b","c",""}; // must have a terminating element
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
    nv->value = (float)254;        // ARM always says the serial buffer is available (max)
    nv->valuetype = TYPE_INT;
    return (STAT_OK);
}

static stat_t get_tick(nvObj_t *nv)
{
    nv->value = (float)SysTickTimer.getValue();
    nv->valuetype = TYPE_INT;
    return (STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

static const char fmt_rx[] = "rx:%d\n";
static const char fmt_ex[] = "[ex]  enable flow control%10d [0=off,1=XON/XOFF, 2=RTS/CTS]\n";
//static const char fmt_ec[] = "[ec]  expand LF to CRLF on TX%6d [0=off,1=on]\n";
//static const char fmt_ee[] = "[ee]  enable echo%18d [0=off,1=on]\n";
//static const char fmt_baud[] = "[baud] USB baud rate%15d [1=9600,2=19200,3=38400,4=57600,5=115200,6=230400]\n";

void cfg_print_rx(nvObj_t *nv) { text_print(nv, fmt_rx);}       // TYPE_INT
void cfg_print_ex(nvObj_t *nv) { text_print(nv, fmt_ex);}       // TYPE_INT
//void cfg_print_ec(nvObj_t *nv) { text_print(nv, fmt_ec);}       // TYPE_INT
//void cfg_print_ee(nvObj_t *nv) { text_print(nv, fmt_ee);}       // TYPE_INT
//void cfg_print_baud(nvObj_t *nv) { text_print(nv, fmt_baud);}   // TYPE_INT

#endif // __TEXT_MODE
