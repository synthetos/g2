/*
 * config_app.cpp - application-specific part of configuration data
 * This file is part of the g2core project
 *
 * Copyright (c) 2013 - 2019 Alden S. Hart, Jr.
 * Copyright (c) 2016 - 2019 Robert Giseburt
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
#include "kinematics.h"
#include "safety_manager.h"

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

class configSubtableNode {
    const configSubtable * const subtable;
   public:
    const configSubtableNode * const next;

    configSubtableNode(size_t new_start_idx, const configSubtable *const s)
        : subtable{s}, next{nullptr}, start_idx(new_start_idx), end_idx(new_start_idx+subtable->length), length{subtable->length} {};

    configSubtableNode(size_t new_start_idx, const configSubtable *const s, const configSubtableNode *const n)
        : subtable{s}, next{n}, start_idx(new_start_idx), end_idx(new_start_idx+subtable->length), length(subtable->length + next->length) {};

    // The following is to optimize for linear table searches
    static const configSubtableNode *search_node_cache; // keep track of the last place we were searching
    static std::size_t idx_cache;                       // and the last index we were searching for
    static std::size_t idx_cache_offset;                // and how much we had to remove from it for this search node

    const size_t start_idx;
    const size_t end_idx;
    const size_t length;

    const cfgItem_t * const get(std::size_t idx) const {
        if (!search_node_cache || idx_cache > idx) {
            search_node_cache = this;
        }

        const configSubtableNode *search_node = search_node_cache;
        idx_cache = idx;

        while (idx >= (search_node->end_idx)) {
            if (!search_node->next) {
                idx_cache = 0;
                search_node_cache = nullptr;
                return nullptr;
            }
            search_node = search_node->next;
        }

        search_node_cache = search_node;
        return search_node->subtable->get(idx - search_node->start_idx);
    }

    // const cfgItem_t * const get(std::size_t idx) const {
    //     const configSubtableNode *search_node = this;
    //     while (idx >= search_node->subtable->length()) {
    //         if (!search_node->next) {
    //             return nullptr;
    //         }
    //         idx -= search_node->subtable->length();
    //         search_node = search_node->next;
    //     }
    //     return search_node->subtable->get(idx);
    // }

    index_t find(const char *token) {
        const configSubtableNode *search_node = this;
        index_t idx = 0;
        idx_cache_offset = 0;

        while ((idx = search_node->subtable->find(token)) == NO_MATCH) {
            if (!search_node->next) {
                idx_cache = 0;
                search_node_cache = nullptr;
                return NO_MATCH;
            }
            auto l = search_node->subtable->length;
            idx_cache_offset += l;
            search_node = search_node->next;
        }

        search_node_cache = search_node;
        idx_cache = idx + idx_cache_offset;

        return idx_cache;
    }
};

const configSubtableNode *configSubtableNode::search_node_cache = nullptr; // keep track of the last place we were searching
std::size_t configSubtableNode::idx_cache = 0;                             // and the last index we were searching for
std::size_t configSubtableNode::idx_cache_offset = 0;                      // and how much we had to remove from it for this search node

// make a compile-time linked list of subtables
template <typename... following_nodes_t>
class configSubtableNodes;

template <typename... following_nodes_t>
class configSubtableNodes<const configSubtable *, following_nodes_t...> {
    configSubtableNodes<following_nodes_t...> next_nodes;
public:
    configSubtableNode this_node;
    constexpr configSubtableNodes(const size_t start_index, const configSubtable * this_subtable, following_nodes_t... following_nodes) :
    next_nodes{start_index + this_subtable->length, following_nodes...}, this_node{start_index, this_subtable, &next_nodes.this_node}
    {}
};

template <>
class configSubtableNodes<const configSubtable * > {
public:
    configSubtableNode this_node;

    configSubtableNodes(const size_t start_index, const configSubtable * this_subtable) :
    this_node{start_index, this_subtable}
    {}
};

template <typename... following_nodes_t>
constexpr configSubtableNodes<following_nodes_t...> makeSubtableNodes(const size_t start_index, following_nodes_t... following_nodes) {
    return {start_index, following_nodes...};
}

constexpr cfgItem_t sys_config_items_1[] = {
    // group token flags p, print_func,   get_func,   set_func, get/set target,    default value
    { "sys", "fb", _fn,  2, hw_print_fb,  hw_get_fb,  set_ro, nullptr, 0 },   // MUST BE FIRST for persistence checking!
    { "sys", "fv", _fn,  2, hw_print_fv,  hw_get_fv,  set_ro, nullptr, 0 },
    { "sys", "fbs",_sn,  0, hw_print_fbs, hw_get_fbs, set_ro, nullptr, 0 },
    { "sys", "fbc",_sn,  0, hw_print_fbc, hw_get_fbc, set_ro, nullptr, 0 },
    { "sys", "hp", _sn,  0, hw_print_hp,  hw_get_hp,  set_ro, nullptr, 0 },
    { "sys", "hv", _sn,  0, hw_print_hv,  hw_get_hv,  set_ro, nullptr, 0 },
    { "sys", "id", _sn,  0, hw_print_id,  hw_get_id,  set_ro, nullptr, 0 },   // device ID (ASCII signature)
};
constexpr cfgSubtableFromStaticArray sys_config_1 {sys_config_items_1};
constexpr const configSubtable * const getSysConfig_1() { return &sys_config_1; }

// cm config 1
// mpo config 1
// pos config 1
// ofs config 1
// homine config 1
// probing config 1
// joggin cofig 1

constexpr cfgItem_t pwr_config_items_1[] = {
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
};
constexpr cfgSubtableFromStaticArray pwr_config_1 {pwr_config_items_1};
constexpr const configSubtable * const getPwrConfig_1() { return &pwr_config_1; }

constexpr cfgItem_t motor_config_items_1[] = {
  // Motor parameters
  // generated with ${PROJECT_ROOT}/Resources/generate_motors_cfgArray.js


#if (MOTORS >= 1)
    { "1","1ma", _iip,  0, st_print_ma, st_get_ma, st_set_ma, nullptr, M1_MOTOR_MAP },
    { "1","1sa", _fip,  3, st_print_sa, st_get_sa, st_set_sa, nullptr, M1_STEP_ANGLE },
    { "1","1tr", _fipc, 4, st_print_tr, st_get_tr, st_set_tr, nullptr, M1_TRAVEL_PER_REV },
    { "1","1su", _f0,   5, st_print_su, st_get_su, st_set_su, nullptr, M1_STEPS_PER_UNIT },
    { "1","1mi", _iip,  0, st_print_mi, st_get_mi, st_set_mi, nullptr, M1_MICROSTEPS },
    { "1","1po", _iip,  0, st_print_po, st_get_po, st_set_po, nullptr, M1_POLARITY },
    { "1","1pm", _iip,  0, st_print_pm, st_get_pm, st_set_pm, nullptr, M1_POWER_MODE },
    { "1","1pl", _fip,  3, st_print_pl, st_get_pl, st_set_pl, nullptr, M1_POWER_LEVEL },
    { "1","1ep", _iip,  0, st_print_ep, st_get_ep, st_set_ep, nullptr, M1_ENABLE_POLARITY },
    { "1","1sp", _iip,  0, st_print_sp, st_get_sp, st_set_sp, nullptr, M1_STEP_POLARITY },
    { "1","1pi", _fip,  3, st_print_pi, st_get_pi, st_set_pi, nullptr, M1_POWER_LEVEL_IDLE },
//  { "1","1mt", _fip,  2, st_print_mt, st_get_mt, st_set_mt, nullptr, M1_MOTOR_TIMEOUT },
#ifdef MOTOR_1_IS_TRINAMIC
    { "1","1ts",  _i0,  0, tx_print_nul, motor_1.get_ts_fn,  set_ro,              &motor_1, 0 },
    { "1","1pth", _iip, 0, tx_print_nul, motor_1.get_pth_fn, motor_1.set_pth_fn,  &motor_1, M1_TMC2130_TPWMTHRS },
    { "1","1cth", _iip, 0, tx_print_nul, motor_1.get_cth_fn, motor_1.set_cth_fn,  &motor_1, M1_TMC2130_TCOOLTHRS },
    { "1","1hth", _iip, 0, tx_print_nul, motor_1.get_hth_fn, motor_1.set_hth_fn,  &motor_1, M1_TMC2130_THIGH },
    { "1","1sgt", _iip, 0, tx_print_nul, motor_1.get_sgt_fn, motor_1.set_sgt_fn,  &motor_1, M1_TMC2130_SGT },
    { "1","1sgr", _i0,  0, tx_print_nul, motor_1.get_sgr_fn, set_ro,              &motor_1, 0 },
    { "1","1csa", _i0,  0, tx_print_nul, motor_1.get_csa_fn, set_ro,              &motor_1, 0 },
    { "1","1sgs", _i0,  0, tx_print_nul, motor_1.get_sgs_fn, set_ro,              &motor_1, 0 },
    { "1","1tbl", _iip, 0, tx_print_nul, motor_1.get_tbl_fn, motor_1.set_tbl_fn,  &motor_1, M1_TMC2130_TBL },
    { "1","1pgrd",_iip, 0, tx_print_nul, motor_1.get_pgrd_fn,motor_1.set_pgrd_fn, &motor_1, M1_TMC2130_PWM_GRAD },
    { "1","1pamp",_iip, 0, tx_print_nul, motor_1.get_pamp_fn,motor_1.set_pamp_fn, &motor_1, M1_TMC2130_PWM_AMPL },
    { "1","1hend",_iip, 0, tx_print_nul, motor_1.get_hend_fn,motor_1.set_hend_fn, &motor_1, M1_TMC2130_HEND },
    { "1","1hsrt",_iip, 0, tx_print_nul, motor_1.get_hsrt_fn,motor_1.set_hsrt_fn, &motor_1, M1_TMC2130_HSTRT },
    { "1","1smin",_iip, 0, tx_print_nul, motor_1.get_smin_fn,motor_1.set_smin_fn, &motor_1, M1_TMC2130_SMIN },
    { "1","1smax",_iip, 0, tx_print_nul, motor_1.get_smax_fn,motor_1.set_smax_fn, &motor_1, M1_TMC2130_SMAX },
    { "1","1sup", _iip, 0, tx_print_nul, motor_1.get_sup_fn, motor_1.set_sup_fn,  &motor_1, M1_TMC2130_SUP },
    { "1","1sdn", _iip, 0, tx_print_nul, motor_1.get_sdn_fn, motor_1.set_sdn_fn,  &motor_1, M1_TMC2130_SDN },
#endif
#endif

#if (MOTORS >= 2)
    { "2","2ma", _iip,  0, st_print_ma, st_get_ma, st_set_ma, nullptr, M2_MOTOR_MAP },
    { "2","2sa", _fip,  3, st_print_sa, st_get_sa, st_set_sa, nullptr, M2_STEP_ANGLE },
    { "2","2tr", _fipc, 4, st_print_tr, st_get_tr, st_set_tr, nullptr, M2_TRAVEL_PER_REV },
    { "2","2su", _f0,   5, st_print_su, st_get_su, st_set_su, nullptr, M2_STEPS_PER_UNIT },
    { "2","2mi", _iip,  0, st_print_mi, st_get_mi, st_set_mi, nullptr, M2_MICROSTEPS },
    { "2","2po", _iip,  0, st_print_po, st_get_po, st_set_po, nullptr, M2_POLARITY },
    { "2","2pm", _iip,  0, st_print_pm, st_get_pm, st_set_pm, nullptr, M2_POWER_MODE },
    { "2","2pl", _fip,  3, st_print_pl, st_get_pl, st_set_pl, nullptr, M2_POWER_LEVEL },
    { "2","2ep", _iip,  0, st_print_ep, st_get_ep, st_set_ep, nullptr, M2_ENABLE_POLARITY },
    { "2","2sp", _iip,  0, st_print_sp, st_get_sp, st_set_sp, nullptr, M2_STEP_POLARITY },
    { "2","2pi", _fip,  3, st_print_pi, st_get_pi, st_set_pi, nullptr, M2_POWER_LEVEL_IDLE },
//  { "2","2mt", _fip,  2, st_print_mt, st_get_mt, st_set_mt, nullptr, M2_MOTOR_TIMEOUT },
#ifdef MOTOR_2_IS_TRINAMIC
    { "2","2ts",  _i0,  0, tx_print_nul, motor_2.get_ts_fn,  set_ro,              &motor_2, 0 },
    { "2","2pth", _iip, 0, tx_print_nul, motor_2.get_pth_fn, motor_2.set_pth_fn,  &motor_2, M2_TMC2130_TPWMTHRS },
    { "2","2cth", _iip, 0, tx_print_nul, motor_2.get_cth_fn, motor_2.set_cth_fn,  &motor_2, M2_TMC2130_TCOOLTHRS },
    { "2","2hth", _iip, 0, tx_print_nul, motor_2.get_hth_fn, motor_2.set_hth_fn,  &motor_2, M2_TMC2130_THIGH },
    { "2","2sgt", _iip, 0, tx_print_nul, motor_2.get_sgt_fn, motor_2.set_sgt_fn,  &motor_2, M2_TMC2130_SGT },
    { "2","2sgr", _i0,  0, tx_print_nul, motor_2.get_sgr_fn, set_ro,              &motor_2, 0 },
    { "2","2csa", _i0,  0, tx_print_nul, motor_2.get_csa_fn, set_ro,              &motor_2, 0 },
    { "2","2sgs", _i0,  0, tx_print_nul, motor_2.get_sgs_fn, set_ro,              &motor_2, 0 },
    { "2","2tbl", _iip, 0, tx_print_nul, motor_2.get_tbl_fn, motor_2.set_tbl_fn,  &motor_2, M2_TMC2130_TBL },
    { "2","2pgrd",_iip, 0, tx_print_nul, motor_2.get_pgrd_fn,motor_2.set_pgrd_fn, &motor_2, M2_TMC2130_PWM_GRAD },
    { "2","2pamp",_iip, 0, tx_print_nul, motor_2.get_pamp_fn,motor_2.set_pamp_fn, &motor_2, M2_TMC2130_PWM_AMPL },
    { "2","2hend",_iip, 0, tx_print_nul, motor_2.get_hend_fn,motor_2.set_hend_fn, &motor_2, M2_TMC2130_HEND },
    { "2","2hsrt",_iip, 0, tx_print_nul, motor_2.get_hsrt_fn,motor_2.set_hsrt_fn, &motor_2, M2_TMC2130_HSTRT },
    { "2","2smin",_iip, 0, tx_print_nul, motor_2.get_smin_fn,motor_2.set_smin_fn, &motor_2, M2_TMC2130_SMIN },
    { "2","2smax",_iip, 0, tx_print_nul, motor_2.get_smax_fn,motor_2.set_smax_fn, &motor_2, M2_TMC2130_SMAX },
    { "2","2sup", _iip, 0, tx_print_nul, motor_2.get_sup_fn, motor_2.set_sup_fn,  &motor_2, M2_TMC2130_SUP },
    { "2","2sdn", _iip, 0, tx_print_nul, motor_2.get_sdn_fn, motor_2.set_sdn_fn,  &motor_2, M2_TMC2130_SDN },
#endif
#endif

#if (MOTORS >= 3)
    { "3","3ma", _iip,  0, st_print_ma, st_get_ma, st_set_ma, nullptr, M3_MOTOR_MAP },
    { "3","3sa", _fip,  3, st_print_sa, st_get_sa, st_set_sa, nullptr, M3_STEP_ANGLE },
    { "3","3tr", _fipc, 4, st_print_tr, st_get_tr, st_set_tr, nullptr, M3_TRAVEL_PER_REV },
    { "3","3su", _f0,   5, st_print_su, st_get_su, st_set_su, nullptr, M3_STEPS_PER_UNIT },
    { "3","3mi", _iip,  0, st_print_mi, st_get_mi, st_set_mi, nullptr, M3_MICROSTEPS },
    { "3","3po", _iip,  0, st_print_po, st_get_po, st_set_po, nullptr, M3_POLARITY },
    { "3","3pm", _iip,  0, st_print_pm, st_get_pm, st_set_pm, nullptr, M3_POWER_MODE },
    { "3","3pl", _fip,  3, st_print_pl, st_get_pl, st_set_pl, nullptr, M3_POWER_LEVEL },
    { "3","3ep", _iip,  0, st_print_ep, st_get_ep, st_set_ep, nullptr, M3_ENABLE_POLARITY },
    { "3","3sp", _iip,  0, st_print_sp, st_get_sp, st_set_sp, nullptr, M3_STEP_POLARITY },
    { "3","3pi", _fip,  3, st_print_pi, st_get_pi, st_set_pi, nullptr, M3_POWER_LEVEL_IDLE },
//  { "3","3mt", _fip,  2, st_print_mt, st_get_mt, st_set_mt, nullptr, M3_MOTOR_TIMEOUT },
#ifdef MOTOR_3_IS_TRINAMIC
    { "3","3ts",  _i0,  0, tx_print_nul, motor_3.get_ts_fn,  set_ro,              &motor_3, 0 },
    { "3","3pth", _iip, 0, tx_print_nul, motor_3.get_pth_fn, motor_3.set_pth_fn,  &motor_3, M3_TMC2130_TPWMTHRS },
    { "3","3cth", _iip, 0, tx_print_nul, motor_3.get_cth_fn, motor_3.set_cth_fn,  &motor_3, M3_TMC2130_TCOOLTHRS },
    { "3","3hth", _iip, 0, tx_print_nul, motor_3.get_hth_fn, motor_3.set_hth_fn,  &motor_3, M3_TMC2130_THIGH },
    { "3","3sgt", _iip, 0, tx_print_nul, motor_3.get_sgt_fn, motor_3.set_sgt_fn,  &motor_3, M3_TMC2130_SGT },
    { "3","3sgr", _i0,  0, tx_print_nul, motor_3.get_sgr_fn, set_ro,              &motor_3, 0 },
    { "3","3csa", _i0,  0, tx_print_nul, motor_3.get_csa_fn, set_ro,              &motor_3, 0 },
    { "3","3sgs", _i0,  0, tx_print_nul, motor_3.get_sgs_fn, set_ro,              &motor_3, 0 },
    { "3","3tbl", _iip, 0, tx_print_nul, motor_3.get_tbl_fn, motor_3.set_tbl_fn,  &motor_3, M3_TMC2130_TBL },
    { "3","3pgrd",_iip, 0, tx_print_nul, motor_3.get_pgrd_fn,motor_3.set_pgrd_fn, &motor_3, M3_TMC2130_PWM_GRAD },
    { "3","3pamp",_iip, 0, tx_print_nul, motor_3.get_pamp_fn,motor_3.set_pamp_fn, &motor_3, M3_TMC2130_PWM_AMPL },
    { "3","3hend",_iip, 0, tx_print_nul, motor_3.get_hend_fn,motor_3.set_hend_fn, &motor_3, M3_TMC2130_HEND },
    { "3","3hsrt",_iip, 0, tx_print_nul, motor_3.get_hsrt_fn,motor_3.set_hsrt_fn, &motor_3, M3_TMC2130_HSTRT },
    { "3","3smin",_iip, 0, tx_print_nul, motor_3.get_smin_fn,motor_3.set_smin_fn, &motor_3, M3_TMC2130_SMIN },
    { "3","3smax",_iip, 0, tx_print_nul, motor_3.get_smax_fn,motor_3.set_smax_fn, &motor_3, M3_TMC2130_SMAX },
    { "3","3sup", _iip, 0, tx_print_nul, motor_3.get_sup_fn, motor_3.set_sup_fn,  &motor_3, M3_TMC2130_SUP },
    { "3","3sdn", _iip, 0, tx_print_nul, motor_3.get_sdn_fn, motor_3.set_sdn_fn,  &motor_3, M3_TMC2130_SDN },
#endif
#endif

#if (MOTORS >= 4)
    { "4","4ma", _iip,  0, st_print_ma, st_get_ma, st_set_ma, nullptr, M4_MOTOR_MAP },
    { "4","4sa", _fip,  3, st_print_sa, st_get_sa, st_set_sa, nullptr, M4_STEP_ANGLE },
    { "4","4tr", _fipc, 4, st_print_tr, st_get_tr, st_set_tr, nullptr, M4_TRAVEL_PER_REV },
    { "4","4su", _f0,   5, st_print_su, st_get_su, st_set_su, nullptr, M4_STEPS_PER_UNIT },
    { "4","4mi", _iip,  0, st_print_mi, st_get_mi, st_set_mi, nullptr, M4_MICROSTEPS },
    { "4","4po", _iip,  0, st_print_po, st_get_po, st_set_po, nullptr, M4_POLARITY },
    { "4","4pm", _iip,  0, st_print_pm, st_get_pm, st_set_pm, nullptr, M4_POWER_MODE },
    { "4","4pl", _fip,  3, st_print_pl, st_get_pl, st_set_pl, nullptr, M4_POWER_LEVEL },
    { "4","4ep", _iip,  0, st_print_ep, st_get_ep, st_set_ep, nullptr, M4_ENABLE_POLARITY },
    { "4","4sp", _iip,  0, st_print_sp, st_get_sp, st_set_sp, nullptr, M4_STEP_POLARITY },
    { "4","4pi", _fip,  3, st_print_pi, st_get_pi, st_set_pi, nullptr, M4_POWER_LEVEL_IDLE },
//  { "4","4mt", _fip,  2, st_print_mt, st_get_mt, st_set_mt, nullptr, M4_MOTOR_TIMEOUT },
#ifdef MOTOR_4_IS_TRINAMIC
    { "4","4ts",  _i0,  0, tx_print_nul, motor_4.get_ts_fn,  set_ro,              &motor_4, 0 },
    { "4","4pth", _iip, 0, tx_print_nul, motor_4.get_pth_fn, motor_4.set_pth_fn,  &motor_4, M4_TMC2130_TPWMTHRS },
    { "4","4cth", _iip, 0, tx_print_nul, motor_4.get_cth_fn, motor_4.set_cth_fn,  &motor_4, M4_TMC2130_TCOOLTHRS },
    { "4","4hth", _iip, 0, tx_print_nul, motor_4.get_hth_fn, motor_4.set_hth_fn,  &motor_4, M4_TMC2130_THIGH },
    { "4","4sgt", _iip, 0, tx_print_nul, motor_4.get_sgt_fn, motor_4.set_sgt_fn,  &motor_4, M4_TMC2130_SGT },
    { "4","4sgr", _i0,  0, tx_print_nul, motor_4.get_sgr_fn, set_ro,              &motor_4, 0 },
    { "4","4csa", _i0,  0, tx_print_nul, motor_4.get_csa_fn, set_ro,              &motor_4, 0 },
    { "4","4sgs", _i0,  0, tx_print_nul, motor_4.get_sgs_fn, set_ro,              &motor_4, 0 },
    { "4","4tbl", _iip, 0, tx_print_nul, motor_4.get_tbl_fn, motor_4.set_tbl_fn,  &motor_4, M4_TMC2130_TBL },
    { "4","4pgrd",_iip, 0, tx_print_nul, motor_4.get_pgrd_fn,motor_4.set_pgrd_fn, &motor_4, M4_TMC2130_PWM_GRAD },
    { "4","4pamp",_iip, 0, tx_print_nul, motor_4.get_pamp_fn,motor_4.set_pamp_fn, &motor_4, M4_TMC2130_PWM_AMPL },
    { "4","4hend",_iip, 0, tx_print_nul, motor_4.get_hend_fn,motor_4.set_hend_fn, &motor_4, M4_TMC2130_HEND },
    { "4","4hsrt",_iip, 0, tx_print_nul, motor_4.get_hsrt_fn,motor_4.set_hsrt_fn, &motor_4, M4_TMC2130_HSTRT },
    { "4","4smin",_iip, 0, tx_print_nul, motor_4.get_smin_fn,motor_4.set_smin_fn, &motor_4, M4_TMC2130_SMIN },
    { "4","4smax",_iip, 0, tx_print_nul, motor_4.get_smax_fn,motor_4.set_smax_fn, &motor_4, M4_TMC2130_SMAX },
    { "4","4sup", _iip, 0, tx_print_nul, motor_4.get_sup_fn, motor_4.set_sup_fn,  &motor_4, M4_TMC2130_SUP },
    { "4","4sdn", _iip, 0, tx_print_nul, motor_4.get_sdn_fn, motor_4.set_sdn_fn,  &motor_4, M4_TMC2130_SDN },
#endif
#endif

#if (MOTORS >= 5)
    { "5","5ma", _iip,  0, st_print_ma, st_get_ma, st_set_ma, nullptr, M5_MOTOR_MAP },
    { "5","5sa", _fip,  3, st_print_sa, st_get_sa, st_set_sa, nullptr, M5_STEP_ANGLE },
    { "5","5tr", _fipc, 4, st_print_tr, st_get_tr, st_set_tr, nullptr, M5_TRAVEL_PER_REV },
    { "5","5su", _f0,   5, st_print_su, st_get_su, st_set_su, nullptr, M5_STEPS_PER_UNIT },
    { "5","5mi", _iip,  0, st_print_mi, st_get_mi, st_set_mi, nullptr, M5_MICROSTEPS },
    { "5","5po", _iip,  0, st_print_po, st_get_po, st_set_po, nullptr, M5_POLARITY },
    { "5","5pm", _iip,  0, st_print_pm, st_get_pm, st_set_pm, nullptr, M5_POWER_MODE },
    { "5","5pl", _fip,  3, st_print_pl, st_get_pl, st_set_pl, nullptr, M5_POWER_LEVEL },
    { "5","5ep", _iip,  0, st_print_ep, st_get_ep, st_set_ep, nullptr, M5_ENABLE_POLARITY },
    { "5","5sp", _iip,  0, st_print_sp, st_get_sp, st_set_sp, nullptr, M5_STEP_POLARITY },
    { "5","5pi", _fip,  3, st_print_pi, st_get_pi, st_set_pi, nullptr, M5_POWER_LEVEL_IDLE },
//  { "5","5mt", _fip,  2, st_print_mt, st_get_mt, st_set_mt, nullptr, M5_MOTOR_TIMEOUT },
#ifdef MOTOR_5_IS_TRINAMIC
    { "5","5ts",  _i0,  0, tx_print_nul, motor_5.get_ts_fn,  set_ro,              &motor_5, 0 },
    { "5","5pth", _iip, 0, tx_print_nul, motor_5.get_pth_fn, motor_5.set_pth_fn,  &motor_5, M5_TMC2130_TPWMTHRS },
    { "5","5cth", _iip, 0, tx_print_nul, motor_5.get_cth_fn, motor_5.set_cth_fn,  &motor_5, M5_TMC2130_TCOOLTHRS },
    { "5","5hth", _iip, 0, tx_print_nul, motor_5.get_hth_fn, motor_5.set_hth_fn,  &motor_5, M5_TMC2130_THIGH },
    { "5","5sgt", _iip, 0, tx_print_nul, motor_5.get_sgt_fn, motor_5.set_sgt_fn,  &motor_5, M5_TMC2130_SGT },
    { "5","5sgr", _i0,  0, tx_print_nul, motor_5.get_sgr_fn, set_ro,              &motor_5, 0 },
    { "5","5csa", _i0,  0, tx_print_nul, motor_5.get_csa_fn, set_ro,              &motor_5, 0 },
    { "5","5sgs", _i0,  0, tx_print_nul, motor_5.get_sgs_fn, set_ro,              &motor_5, 0 },
    { "5","5tbl", _iip, 0, tx_print_nul, motor_5.get_tbl_fn, motor_5.set_tbl_fn,  &motor_5, M5_TMC2130_TBL },
    { "5","5pgrd",_iip, 0, tx_print_nul, motor_5.get_pgrd_fn,motor_5.set_pgrd_fn, &motor_5, M5_TMC2130_PWM_GRAD },
    { "5","5pamp",_iip, 0, tx_print_nul, motor_5.get_pamp_fn,motor_5.set_pamp_fn, &motor_5, M5_TMC2130_PWM_AMPL },
    { "5","5hend",_iip, 0, tx_print_nul, motor_5.get_hend_fn,motor_5.set_hend_fn, &motor_5, M5_TMC2130_HEND },
    { "5","5hsrt",_iip, 0, tx_print_nul, motor_5.get_hsrt_fn,motor_5.set_hsrt_fn, &motor_5, M5_TMC2130_HSTRT },
    { "5","5smin",_iip, 0, tx_print_nul, motor_5.get_smin_fn,motor_5.set_smin_fn, &motor_5, M5_TMC2130_SMIN },
    { "5","5smax",_iip, 0, tx_print_nul, motor_5.get_smax_fn,motor_5.set_smax_fn, &motor_5, M5_TMC2130_SMAX },
    { "5","5sup", _iip, 0, tx_print_nul, motor_5.get_sup_fn, motor_5.set_sup_fn,  &motor_5, M5_TMC2130_SUP },
    { "5","5sdn", _iip, 0, tx_print_nul, motor_5.get_sdn_fn, motor_5.set_sdn_fn,  &motor_5, M5_TMC2130_SDN },
#endif
#endif

#if (MOTORS >= 6)
    { "6","6ma", _iip,  0, st_print_ma, st_get_ma, st_set_ma, nullptr, M6_MOTOR_MAP },
    { "6","6sa", _fip,  3, st_print_sa, st_get_sa, st_set_sa, nullptr, M6_STEP_ANGLE },
    { "6","6tr", _fipc, 4, st_print_tr, st_get_tr, st_set_tr, nullptr, M6_TRAVEL_PER_REV },
    { "6","6su", _f0,   5, st_print_su, st_get_su, st_set_su, nullptr, M6_STEPS_PER_UNIT },
    { "6","6mi", _iip,  0, st_print_mi, st_get_mi, st_set_mi, nullptr, M6_MICROSTEPS },
    { "6","6po", _iip,  0, st_print_po, st_get_po, st_set_po, nullptr, M6_POLARITY },
    { "6","6pm", _iip,  0, st_print_pm, st_get_pm, st_set_pm, nullptr, M6_POWER_MODE },
    { "6","6pl", _fip,  3, st_print_pl, st_get_pl, st_set_pl, nullptr, M6_POWER_LEVEL },
    { "6","6ep", _iip,  0, st_print_ep, st_get_ep, st_set_ep, nullptr, M6_ENABLE_POLARITY },
    { "6","6sp", _iip,  0, st_print_sp, st_get_sp, st_set_sp, nullptr, M6_STEP_POLARITY },
    { "6","6pi", _fip,  3, st_print_pi, st_get_pi, st_set_pi, nullptr, M6_POWER_LEVEL_IDLE },
//  { "6","6mt", _fip,  2, st_print_mt, st_get_mt, st_set_mt, nullptr, M6_MOTOR_TIMEOUT },
#ifdef MOTOR_6_IS_TRINAMIC
    { "6","6ts",  _i0,  0, tx_print_nul, motor_6.get_ts_fn,  set_ro,              &motor_6, 0 },
    { "6","6pth", _iip, 0, tx_print_nul, motor_6.get_pth_fn, motor_6.set_pth_fn,  &motor_6, M6_TMC2130_TPWMTHRS },
    { "6","6cth", _iip, 0, tx_print_nul, motor_6.get_cth_fn, motor_6.set_cth_fn,  &motor_6, M6_TMC2130_TCOOLTHRS },
    { "6","6hth", _iip, 0, tx_print_nul, motor_6.get_hth_fn, motor_6.set_hth_fn,  &motor_6, M6_TMC2130_THIGH },
    { "6","6sgt", _iip, 0, tx_print_nul, motor_6.get_sgt_fn, motor_6.set_sgt_fn,  &motor_6, M6_TMC2130_SGT },
    { "6","6sgr", _i0,  0, tx_print_nul, motor_6.get_sgr_fn, set_ro,              &motor_6, 0 },
    { "6","6csa", _i0,  0, tx_print_nul, motor_6.get_csa_fn, set_ro,              &motor_6, 0 },
    { "6","6sgs", _i0,  0, tx_print_nul, motor_6.get_sgs_fn, set_ro,              &motor_6, 0 },
    { "6","6tbl", _iip, 0, tx_print_nul, motor_6.get_tbl_fn, motor_6.set_tbl_fn,  &motor_6, M6_TMC2130_TBL },
    { "6","6pgrd",_iip, 0, tx_print_nul, motor_6.get_pgrd_fn,motor_6.set_pgrd_fn, &motor_6, M6_TMC2130_PWM_GRAD },
    { "6","6pamp",_iip, 0, tx_print_nul, motor_6.get_pamp_fn,motor_6.set_pamp_fn, &motor_6, M6_TMC2130_PWM_AMPL },
    { "6","6hend",_iip, 0, tx_print_nul, motor_6.get_hend_fn,motor_6.set_hend_fn, &motor_6, M6_TMC2130_HEND },
    { "6","6hsrt",_iip, 0, tx_print_nul, motor_6.get_hsrt_fn,motor_6.set_hsrt_fn, &motor_6, M6_TMC2130_HSTRT },
    { "6","6smin",_iip, 0, tx_print_nul, motor_6.get_smin_fn,motor_6.set_smin_fn, &motor_6, M6_TMC2130_SMIN },
    { "6","6smax",_iip, 0, tx_print_nul, motor_6.get_smax_fn,motor_6.set_smax_fn, &motor_6, M6_TMC2130_SMAX },
    { "6","6sup", _iip, 0, tx_print_nul, motor_6.get_sup_fn, motor_6.set_sup_fn,  &motor_6, M6_TMC2130_SUP },
    { "6","6sdn", _iip, 0, tx_print_nul, motor_6.get_sdn_fn, motor_6.set_sdn_fn,  &motor_6, M6_TMC2130_SDN },
#endif
#endif

  // END generated with ${PROJECT_ROOT}/Resources/generate_motors_cfgArray.js
};
constexpr cfgSubtableFromStaticArray motor_config_1 {motor_config_items_1};
constexpr const configSubtable * const getMotorConfig_1() { return &motor_config_1; }

// axis config

constexpr cfgItem_t di_config_items_1[] = {
    // Digital input configs
    // generated with ${PROJECT_ROOT}/Resources/generate_dins_cfgArray.js


#if (D_IN_CHANNELS >= 1)
    { "di1","di1en",_bip,   0, din_print_en, din_get_en, din_set_en, &din1,  DI1_ENABLED },
    { "di1","di1po",_iip,   0, din_print_po, din_get_po, din_set_po, &din1,  DI1_POLARITY },
    { "di1","di1ac",_iip,   0, din_print_ac, din_get_ac, din_set_ac, &din1,  DI1_ACTION },
    { "di1","di1in",_iip,   0, din_print_in, din_get_in, din_set_in, &din1,  DI1_EXTERNAL_NUMBER },
#endif

#if (D_IN_CHANNELS >= 2)
    { "di2","di2en",_bip,   0, din_print_en, din_get_en, din_set_en, &din2,  DI2_ENABLED },
    { "di2","di2po",_iip,   0, din_print_po, din_get_po, din_set_po, &din2,  DI2_POLARITY },
    { "di2","di2ac",_iip,   0, din_print_ac, din_get_ac, din_set_ac, &din2,  DI2_ACTION },
    { "di2","di2in",_iip,   0, din_print_in, din_get_in, din_set_in, &din2,  DI2_EXTERNAL_NUMBER },
#endif

#if (D_IN_CHANNELS >= 3)
    { "di3","di3en",_bip,   0, din_print_en, din_get_en, din_set_en, &din3,  DI3_ENABLED },
    { "di3","di3po",_iip,   0, din_print_po, din_get_po, din_set_po, &din3,  DI3_POLARITY },
    { "di3","di3ac",_iip,   0, din_print_ac, din_get_ac, din_set_ac, &din3,  DI3_ACTION },
    { "di3","di3in",_iip,   0, din_print_in, din_get_in, din_set_in, &din3,  DI3_EXTERNAL_NUMBER },
#endif

#if (D_IN_CHANNELS >= 4)
    { "di4","di4en",_bip,   0, din_print_en, din_get_en, din_set_en, &din4,  DI4_ENABLED },
    { "di4","di4po",_iip,   0, din_print_po, din_get_po, din_set_po, &din4,  DI4_POLARITY },
    { "di4","di4ac",_iip,   0, din_print_ac, din_get_ac, din_set_ac, &din4,  DI4_ACTION },
    { "di4","di4in",_iip,   0, din_print_in, din_get_in, din_set_in, &din4,  DI4_EXTERNAL_NUMBER },
#endif

#if (D_IN_CHANNELS >= 5)
    { "di5","di5en",_bip,   0, din_print_en, din_get_en, din_set_en, &din5,  DI5_ENABLED },
    { "di5","di5po",_iip,   0, din_print_po, din_get_po, din_set_po, &din5,  DI5_POLARITY },
    { "di5","di5ac",_iip,   0, din_print_ac, din_get_ac, din_set_ac, &din5,  DI5_ACTION },
    { "di5","di5in",_iip,   0, din_print_in, din_get_in, din_set_in, &din5,  DI5_EXTERNAL_NUMBER },
#endif

#if (D_IN_CHANNELS >= 6)
    { "di6","di6en",_bip,   0, din_print_en, din_get_en, din_set_en, &din6,  DI6_ENABLED },
    { "di6","di6po",_iip,   0, din_print_po, din_get_po, din_set_po, &din6,  DI6_POLARITY },
    { "di6","di6ac",_iip,   0, din_print_ac, din_get_ac, din_set_ac, &din6,  DI6_ACTION },
    { "di6","di6in",_iip,   0, din_print_in, din_get_in, din_set_in, &din6,  DI6_EXTERNAL_NUMBER },
#endif

#if (D_IN_CHANNELS >= 7)
    { "di7","di7en",_bip,   0, din_print_en, din_get_en, din_set_en, &din7,  DI7_ENABLED },
    { "di7","di7po",_iip,   0, din_print_po, din_get_po, din_set_po, &din7,  DI7_POLARITY },
    { "di7","di7ac",_iip,   0, din_print_ac, din_get_ac, din_set_ac, &din7,  DI7_ACTION },
    { "di7","di7in",_iip,   0, din_print_in, din_get_in, din_set_in, &din7,  DI7_EXTERNAL_NUMBER },
#endif

#if (D_IN_CHANNELS >= 8)
    { "di8","di8en",_bip,   0, din_print_en, din_get_en, din_set_en, &din8,  DI8_ENABLED },
    { "di8","di8po",_iip,   0, din_print_po, din_get_po, din_set_po, &din8,  DI8_POLARITY },
    { "di8","di8ac",_iip,   0, din_print_ac, din_get_ac, din_set_ac, &din8,  DI8_ACTION },
    { "di8","di8in",_iip,   0, din_print_in, din_get_in, din_set_in, &din8,  DI8_EXTERNAL_NUMBER },
#endif

#if (D_IN_CHANNELS >= 9)
    { "di9","di9en",_bip,   0, din_print_en, din_get_en, din_set_en, &din9,  DI9_ENABLED },
    { "di9","di9po",_iip,   0, din_print_po, din_get_po, din_set_po, &din9,  DI9_POLARITY },
    { "di9","di9ac",_iip,   0, din_print_ac, din_get_ac, din_set_ac, &din9,  DI9_ACTION },
    { "di9","di9in",_iip,   0, din_print_in, din_get_in, din_set_in, &din9,  DI9_EXTERNAL_NUMBER },
#endif

#if (D_IN_CHANNELS >= 10)
    { "di10","di10en",_bip,   0, din_print_en, din_get_en, din_set_en, &din10,  DI10_ENABLED },
    { "di10","di10po",_iip,   0, din_print_po, din_get_po, din_set_po, &din10,  DI10_POLARITY },
    { "di10","di10ac",_iip,   0, din_print_ac, din_get_ac, din_set_ac, &din10,  DI10_ACTION },
    { "di10","di10in",_iip,   0, din_print_in, din_get_in, din_set_in, &din10,  DI10_EXTERNAL_NUMBER },
#endif

#if (D_IN_CHANNELS >= 11)
    { "di11","di11en",_bip,   0, din_print_en, din_get_en, din_set_en, &din11,  DI11_ENABLED },
    { "di11","di11po",_iip,   0, din_print_po, din_get_po, din_set_po, &din11,  DI11_POLARITY },
    { "di11","di11ac",_iip,   0, din_print_ac, din_get_ac, din_set_ac, &din11,  DI11_ACTION },
    { "di11","di11in",_iip,   0, din_print_in, din_get_in, din_set_in, &din11,  DI11_EXTERNAL_NUMBER },
#endif

#if (D_IN_CHANNELS >= 12)
    { "di12","di12en",_bip,   0, din_print_en, din_get_en, din_set_en, &din12,  DI12_ENABLED },
    { "di12","di12po",_iip,   0, din_print_po, din_get_po, din_set_po, &din12,  DI12_POLARITY },
    { "di12","di12ac",_iip,   0, din_print_ac, din_get_ac, din_set_ac, &din12,  DI12_ACTION },
    { "di12","di12in",_iip,   0, din_print_in, din_get_in, din_set_in, &din12,  DI12_EXTERNAL_NUMBER },
#endif

    // END generated with ${PROJECT_ROOT}/Resources/generate_dins_cfgArray.js
};
constexpr cfgSubtableFromStaticArray di_config_1 {di_config_items_1};
constexpr const configSubtable * const getDIConfig_1() { return &di_config_1; }

constexpr cfgItem_t in_config_items_1[] = {
    // Digital input state readers
    { "in","in1", _i0, 0,  din_print_state, din_get_input, set_ro,  &in1,  0 },
    { "in","in2", _i0, 0,  din_print_state, din_get_input, set_ro,  &in2,  0 },
    { "in","in3", _i0, 0,  din_print_state, din_get_input, set_ro,  &in3,  0 },
    { "in","in4", _i0, 0,  din_print_state, din_get_input, set_ro,  &in4,  0 },
    { "in","in5", _i0, 0,  din_print_state, din_get_input, set_ro,  &in5,  0 },
    { "in","in6", _i0, 0,  din_print_state, din_get_input, set_ro,  &in6,  0 },
    { "in","in7", _i0, 0,  din_print_state, din_get_input, set_ro,  &in7,  0 },
    { "in","in8", _i0, 0,  din_print_state, din_get_input, set_ro,  &in8,  0 },
    { "in","in9", _i0, 0,  din_print_state, din_get_input, set_ro,  &in9,  0 },
    { "in","in10", _i0, 0, din_print_state, din_get_input, set_ro,  &in10, 0 },
    { "in","in11", _i0, 0, din_print_state, din_get_input, set_ro,  &in11, 0 },
    { "in","in12", _i0, 0, din_print_state, din_get_input, set_ro,  &in12, 0 },
    { "in","in13", _i0, 0, din_print_state, din_get_input, set_ro,  &in13, 0 },
    { "in","in14", _i0, 0, din_print_state, din_get_input, set_ro,  &in14, 0 },
    { "in","in15", _i0, 0, din_print_state, din_get_input, set_ro,  &in15, 0 },
    { "in","in16", _i0, 0, din_print_state, din_get_input, set_ro,  &in16, 0 },
};
constexpr cfgSubtableFromStaticArray in_config_1 {in_config_items_1};
constexpr const configSubtable * const getINConfig_1() { return &in_config_1; }

constexpr cfgItem_t do_config_items_1[] = {
    // digital output configs
    { "do1", "do1en", _iip, 0,   dout_print_en,  dout_get_en,  dout_set_en,  &dout1,  DO1_ENABLED },
    { "do1", "do1po", _iip, 0,   dout_print_po,  dout_get_po,  dout_set_po,  &dout1,  DO1_POLARITY },
    { "do1", "do1out",_iip, 0,   dout_print_out, dout_get_out, dout_set_out, &dout1,  DO1_EXTERNAL_NUMBER },
#if (D_OUT_CHANNELS >= 2)
    { "do2", "do2en", _iip, 0,   dout_print_en,  dout_get_en,  dout_set_en,  &dout2,  DO2_ENABLED },
    { "do2", "do2po", _iip, 0,   dout_print_po,  dout_get_po,  dout_set_po,  &dout2,  DO2_POLARITY },
    { "do2", "do2out",_iip, 0,   dout_print_out, dout_get_out, dout_set_out, &dout2,  DO2_EXTERNAL_NUMBER },
#endif
#if (D_OUT_CHANNELS >= 3)
    { "do3", "do3en", _iip, 0,   dout_print_en,  dout_get_en,  dout_set_en,  &dout3,  DO3_ENABLED },
    { "do3", "do3po", _iip, 0,   dout_print_po,  dout_get_po,  dout_set_po,  &dout3,  DO3_POLARITY },
    { "do3", "do3out",_iip, 0,   dout_print_out, dout_get_out, dout_set_out, &dout3,  DO3_EXTERNAL_NUMBER },
#endif
#if (D_OUT_CHANNELS >= 4)
    { "do4", "do4en", _iip, 0,   dout_print_en,  dout_get_en,  dout_set_en,  &dout4,  DO4_ENABLED },
    { "do4", "do4po", _iip, 0,   dout_print_po,  dout_get_po,  dout_set_po,  &dout4,  DO4_POLARITY },
    { "do4", "do4out",_iip, 0,   dout_print_out, dout_get_out, dout_set_out, &dout4,  DO4_EXTERNAL_NUMBER },
#endif
#if (D_OUT_CHANNELS >= 5)
    { "do5", "do5en", _iip, 0,   dout_print_en,  dout_get_en,  dout_set_en,  &dout5,  DO5_ENABLED },
    { "do5", "do5po", _iip, 0,   dout_print_po,  dout_get_po,  dout_set_po,  &dout5,  DO5_POLARITY },
    { "do5", "do5out",_iip, 0,   dout_print_out, dout_get_out, dout_set_out, &dout5,  DO5_EXTERNAL_NUMBER },
#endif
#if (D_OUT_CHANNELS >= 6)
    { "do6", "do6en", _iip, 0,   dout_print_en,  dout_get_en,  dout_set_en,  &dout6,  DO6_ENABLED },
    { "do6", "do6po", _iip, 0,   dout_print_po,  dout_get_po,  dout_set_po,  &dout6,  DO6_POLARITY },
    { "do6", "do6out",_iip, 0,   dout_print_out, dout_get_out, dout_set_out, &dout6,  DO6_EXTERNAL_NUMBER },
#endif
#if (D_OUT_CHANNELS >= 7)
    { "do7", "do7en", _iip, 0,   dout_print_en,  dout_get_en,  dout_set_en,  &dout7,  DO7_ENABLED },
    { "do7", "do7po", _iip, 0,   dout_print_po,  dout_get_po,  dout_set_po,  &dout7,  DO7_POLARITY },
    { "do7", "do7out",_iip, 0,   dout_print_out, dout_get_out, dout_set_out, &dout7,  DO7_EXTERNAL_NUMBER },
#endif
#if (D_OUT_CHANNELS >= 8)
    { "do8", "do8en", _iip, 0,   dout_print_en,  dout_get_en,  dout_set_en,  &dout8,  DO8_ENABLED },
    { "do8", "do8po", _iip, 0,   dout_print_po,  dout_get_po,  dout_set_po,  &dout8,  DO8_POLARITY },
    { "do8", "do8out",_iip, 0,   dout_print_out, dout_get_out, dout_set_out, &dout8,  DO8_EXTERNAL_NUMBER },
#endif
#if (D_OUT_CHANNELS >= 9)
    { "do9", "do9en", _iip, 0,   dout_print_en,  dout_get_en,  dout_set_en,  &dout9,  DO9_ENABLED },
    { "do9", "do9po", _iip, 0,   dout_print_po,  dout_get_po,  dout_set_po,  &dout9,  DO9_POLARITY },
    { "do9", "do9out",_iip, 0,   dout_print_out, dout_get_out, dout_set_out, &dout9,  DO9_EXTERNAL_NUMBER },
#endif
#if (D_OUT_CHANNELS >= 10)
    { "do10", "do10en", _iip, 0, dout_print_en,  dout_get_en,  dout_set_en,  &dout10, DO10_ENABLED },
    { "do10", "do10po", _iip, 0, dout_print_po,  dout_get_po,  dout_set_po,  &dout10, DO10_POLARITY },
    { "do10", "do10out",_iip, 0, dout_print_out, dout_get_out, dout_set_out, &dout10, DO10_EXTERNAL_NUMBER },
#endif
#if (D_OUT_CHANNELS >= 11)
    { "do11", "do11en", _iip, 0, dout_print_en,  dout_get_en,  dout_set_en,  &dout11, DO11_ENABLED },
    { "do11", "do11po", _iip, 0, dout_print_po,  dout_get_po,  dout_set_po,  &dout11, DO11_POLARITY },
    { "do11", "do11out",_iip, 0, dout_print_out, dout_get_out, dout_set_out, &dout11, DO11_EXTERNAL_NUMBER },
#endif
#if (D_OUT_CHANNELS >= 12)
    { "do12", "do12en", _iip, 0, dout_print_en,  dout_get_en,  dout_set_en,  &dout12, DO12_ENABLED },
    { "do12", "do12po", _iip, 0, dout_print_po,  dout_get_po,  dout_set_po,  &dout12, DO12_POLARITY },
    { "do12", "do12out",_iip, 0, dout_print_out, dout_get_out, dout_set_out, &dout12, DO12_EXTERNAL_NUMBER },
#endif
#if (D_OUT_CHANNELS >= 13)
    { "do13", "do13en", _iip, 0, dout_print_en,  dout_get_en,  dout_set_en,  &dout13, DO13_ENABLED },
    { "do13", "do13po", _iip, 0, dout_print_po,  dout_get_po,  dout_set_po,  &dout13, DO13_POLARITY },
    { "do13", "do13out",_iip, 0, dout_print_out, dout_get_out, dout_set_out, &dout13, DO13_EXTERNAL_NUMBER },
#endif
#if (D_OUT_CHANNELS >= 14)
    { "do14", "do14en", _iip, 0, dout_print_en,  dout_get_en,  dout_set_en,  &dout14, DO14_ENABLED },
    { "do14", "do14po", _iip, 0, dout_print_po,  dout_get_po,  dout_set_po,  &dout14, DO14_POLARITY },
    { "do14", "do14out",_iip, 0, dout_print_out, dout_get_out, dout_set_out, &dout14, DO14_EXTERNAL_NUMBER },
#endif
};
constexpr cfgSubtableFromStaticArray do_config_1 {do_config_items_1};
constexpr const configSubtable * const getDOConfig_1() { return &do_config_1; }

constexpr cfgItem_t out_config_items_1[] = {
    // Digital output state readers (default to non-active)
    { "out","out1",  _f0, 2, dout_print_out, dout_get_output, dout_set_output, &out1,  0 },
    { "out","out2",  _f0, 2, dout_print_out, dout_get_output, dout_set_output, &out2,  0 },
    { "out","out3",  _f0, 2, dout_print_out, dout_get_output, dout_set_output, &out3,  0 },
    { "out","out4",  _f0, 2, dout_print_out, dout_get_output, dout_set_output, &out4,  0 },
    { "out","out5",  _f0, 2, dout_print_out, dout_get_output, dout_set_output, &out5,  0 },
    { "out","out6",  _f0, 2, dout_print_out, dout_get_output, dout_set_output, &out6,  0 },
    { "out","out7",  _f0, 2, dout_print_out, dout_get_output, dout_set_output, &out7,  0 },
    { "out","out8",  _f0, 2, dout_print_out, dout_get_output, dout_set_output, &out8,  0 },
    { "out","out9",  _f0, 2, dout_print_out, dout_get_output, dout_set_output, &out9,  0 },
    { "out","out10", _f0, 2, dout_print_out, dout_get_output, dout_set_output, &out10, 0 },
    { "out","out11", _f0, 2, dout_print_out, dout_get_output, dout_set_output, &out11, 0 },
    { "out","out12", _f0, 2, dout_print_out, dout_get_output, dout_set_output, &out12, 0 },
    { "out","out13", _f0, 2, dout_print_out, dout_get_output, dout_set_output, &out13, 0 },
    { "out","out14", _f0, 2, dout_print_out, dout_get_output, dout_set_output, &out14, 0 },
    { "out","out15", _f0, 2, dout_print_out, dout_get_output, dout_set_output, &out15, 0 },
    { "out","out16", _f0, 2, dout_print_out, dout_get_output, dout_set_output, &out16, 0 },
};
constexpr cfgSubtableFromStaticArray out_config_1 {out_config_items_1};
constexpr const configSubtable * const getOUTConfig_1() { return &out_config_1; }

constexpr cfgItem_t ai_config_items_1[] = {
    // Analog input configs
#if (A_IN_CHANNELS >= 1)
    { "ai1","ai1en",_iip, 0, ai_print_en,         ai_get_en,         ai_set_en,      &ai1,  AI1_ENABLED },
    { "ai1","ai1ain",_iip,0, ai_print_ain,        ai_get_ain,        ai_set_ain,     &ai1,  AI1_EXTERNAL_NUMBER },
    { "ai1","ai1ty",_iip, 0, ai_print_type,       ai_get_type,       ai_set_type,    &ai1,  AI1_TYPE },
    { "ai1","ai1ct",_iip, 0, ai_print_circuit,    ai_get_circuit,    ai_set_circuit, &ai1,  AI1_CIRCUIT },
    { "ai1","ai1p1",_fip, 4, ai_print_p,          ai_get_p1,         ai_set_p1,      &ai1,  AI1_P1 },
    { "ai1","ai1p2",_fip, 4, ai_print_p,          ai_get_p2,         ai_set_p2,      &ai1,  AI1_P2 },
    { "ai1","ai1p3",_fip, 4, ai_print_p,          ai_get_p3,         ai_set_p3,      &ai1,  AI1_P3 },
    { "ai1","ai1p4",_fip, 4, ai_print_p,          ai_get_p4,         ai_set_p4,      &ai1,  AI1_P4 },
    { "ai1","ai1p5",_fip, 4, ai_print_p,          ai_get_p5,         ai_set_p5,      &ai1,  AI1_P5 },
#endif
#if (A_IN_CHANNELS >= 2)
    { "ai2","ai2en",_iip, 0, ai_print_en,         ai_get_en,         ai_set_en,      &ai2,  AI2_ENABLED },
    { "ai2","ai2ain",_iip,0, ai_print_ain,        ai_get_ain,        ai_set_ain,     &ai2,  AI2_EXTERNAL_NUMBER },
    { "ai2","ai2ty",_iip, 0, ai_print_type,       ai_get_type,       ai_set_type,    &ai2,  AI2_TYPE },
    { "ai2","ai2ct",_iip, 0, ai_print_circuit,    ai_get_circuit,    ai_set_circuit, &ai2,  AI2_CIRCUIT },
    { "ai2","ai2p1",_fip, 4, ai_print_p,          ai_get_p1,         ai_set_p1,      &ai2,  AI2_P1 },
    { "ai2","ai2p2",_fip, 4, ai_print_p,          ai_get_p2,         ai_set_p2,      &ai2,  AI2_P2 },
    { "ai2","ai2p3",_fip, 4, ai_print_p,          ai_get_p3,         ai_set_p3,      &ai2,  AI2_P3 },
    { "ai2","ai2p4",_fip, 4, ai_print_p,          ai_get_p4,         ai_set_p4,      &ai2,  AI2_P4 },
    { "ai2","ai2p5",_fip, 4, ai_print_p,          ai_get_p5,         ai_set_p5,      &ai2,  AI2_P5 },
#endif
#if (A_IN_CHANNELS >= 3)
    { "ai3","ai3en",_iip, 0, ai_print_en,         ai_get_en,         ai_set_en,      &ai3,  AI3_ENABLED },
    { "ai3","ai3ain",_iip,0, ai_print_ain,        ai_get_ain,        ai_set_ain,     &ai3,  AI3_EXTERNAL_NUMBER },
    { "ai3","ai3ty",_iip, 0, ai_print_type,       ai_get_type,       ai_set_type,    &ai3,  AI3_TYPE },
    { "ai3","ai3ct",_iip, 0, ai_print_circuit,    ai_get_circuit,    ai_set_circuit, &ai3,  AI3_CIRCUIT },
    { "ai3","ai3p1",_fip, 4, ai_print_p,          ai_get_p1,         ai_set_p1,      &ai3,  AI3_P1 },
    { "ai3","ai3p2",_fip, 4, ai_print_p,          ai_get_p2,         ai_set_p2,      &ai3,  AI3_P2 },
    { "ai3","ai3p3",_fip, 4, ai_print_p,          ai_get_p3,         ai_set_p3,      &ai3,  AI3_P3 },
    { "ai3","ai3p4",_fip, 4, ai_print_p,          ai_get_p4,         ai_set_p4,      &ai3,  AI3_P4 },
    { "ai3","ai3p5",_fip, 4, ai_print_p,          ai_get_p5,         ai_set_p5,      &ai3,  AI3_P5 },
#endif
#if (A_IN_CHANNELS >= 4)
    { "ai4","ai4en",_iip, 0, ai_print_en,         ai_get_en,         ai_set_en,      &ai4,  AI4_ENABLED },
    { "ai4","ai4ain",_iip,0, ai_print_ain,        ai_get_ain,        ai_set_ain,     &ai4,  AI4_EXTERNAL_NUMBER },
    { "ai4","ai4ty",_iip, 0, ai_print_type,       ai_get_type,       ai_set_type,    &ai4,  AI4_TYPE },
    { "ai4","ai4ct",_iip, 0, ai_print_circuit,    ai_get_circuit,    ai_set_circuit, &ai4,  AI4_CIRCUIT },
    { "ai4","ai4p1",_fip, 4, ai_print_p,          ai_get_p1,         ai_set_p1,      &ai4,  AI4_P1 },
    { "ai4","ai4p2",_fip, 4, ai_print_p,          ai_get_p2,         ai_set_p2,      &ai4,  AI4_P2 },
    { "ai4","ai4p3",_fip, 4, ai_print_p,          ai_get_p3,         ai_set_p3,      &ai4,  AI4_P3 },
    { "ai4","ai4p4",_fip, 4, ai_print_p,          ai_get_p4,         ai_set_p4,      &ai4,  AI4_P4 },
    { "ai4","ai4p5",_fip, 4, ai_print_p,          ai_get_p5,         ai_set_p5,      &ai4,  AI4_P5 },
#endif
};

constexpr cfgItem_t ain_config_items_1[] = {
    { "ain1","ain1vv",_f0,  4, ain_print_value,      ain_get_value,      set_ro,         &ain1,  0 },
    { "ain1","ain1rv",_f0,  2, ain_print_resistance, ain_get_resistance, set_ro,         &ain1,  0 },
    { "ain2","ain2vv",_f0,  4, ain_print_value,      ain_get_value,      set_ro,         &ain2,  0 },
    { "ain2","ain2rv",_f0,  2, ain_print_resistance, ain_get_resistance, set_ro,         &ain2,  0 },
    { "ain3","ain3vv",_f0,  4, ain_print_value,      ain_get_value,      set_ro,         &ain3,  0 },
    { "ain3","ain3rv",_f0,  2, ain_print_resistance, ain_get_resistance, set_ro,         &ain3,  0 },
    { "ain4","ain4vv",_f0,  4, ain_print_value,      ain_get_value,      set_ro,         &ain4,  0 },
    { "ain4","ain4rv",_f0,  2, ain_print_resistance, ain_get_resistance, set_ro,         &ain4,  0 },
    { "ain5","ain5vv",_f0,  4, ain_print_value,      ain_get_value,      set_ro,         &ain5,  0 },
    { "ain5","ain5rv",_f0,  2, ain_print_resistance, ain_get_resistance, set_ro,         &ain5,  0 },
    { "ain6","ain6vv",_f0,  4, ain_print_value,      ain_get_value,      set_ro,         &ain6,  0 },
    { "ain6","ain6rv",_f0,  2, ain_print_resistance, ain_get_resistance, set_ro,         &ain6,  0 },
    { "ain7","ain7vv",_f0,  4, ain_print_value,      ain_get_value,      set_ro,         &ain7,  0 },
    { "ain7","ain7rv",_f0,  2, ain_print_resistance, ain_get_resistance, set_ro,         &ain7,  0 },
    { "ain8","ain8vv",_f0,  4, ain_print_value,      ain_get_value,      set_ro,         &ain8,  0 },
    { "ain8","ain8rv",_f0,  2, ain_print_resistance, ain_get_resistance, set_ro,         &ain8,  0 },
};
constexpr cfgSubtableFromStaticArray ain_config_1 {ain_config_items_1};
constexpr const configSubtable * const getAINConfig_1() { return &ain_config_1; }

// p1_config_1

constexpr cfgItem_t pid_config_items_1[] = {
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
};
constexpr cfgSubtableFromStaticArray pid_config_1 {pid_config_items_1};
constexpr const configSubtable * const getPIDConfig_1() { return &pid_config_1; }

constexpr cfgItem_t he_config_items_1[] = {
    // temperature configs - heater set values (read-write)
    // NOTICE: If you change these heater group keys, you MUST change the get/set functions too!
    { "he1","he1e", _bip, 0, tx_print_nul, cm_get_heater_enable,   cm_set_heater_enable,   nullptr, H1_DEFAULT_ENABLE },
    { "he1","he1at",_b0,  0, tx_print_nul, cm_get_at_temperature,  set_ro,                 nullptr, 0 },
    { "he1","he1p", _fip, 3, tx_print_nul, cm_get_heater_p,        cm_set_heater_p,        nullptr, H1_DEFAULT_P },
    { "he1","he1i", _fip, 5, tx_print_nul, cm_get_heater_i,        cm_set_heater_i,        nullptr, H1_DEFAULT_I },
    { "he1","he1d", _fip, 5, tx_print_nul, cm_get_heater_d,        cm_set_heater_d,        nullptr, H1_DEFAULT_D },
    { "he1","he1f", _fi,  5, tx_print_nul, cm_get_heater_f,        cm_set_heater_f,        nullptr, H1_DEFAULT_F },
    { "he1","he1st",_fi,  1, tx_print_nul, cm_get_set_temperature, cm_set_set_temperature, nullptr, 0 },
    { "he1","he1t", _fi,  1, tx_print_nul, cm_get_temperature,     set_ro,                 nullptr, 0 },
    { "he1","he1op",_fi,  3, tx_print_nul, cm_get_heater_output,   set_ro,                 nullptr, 0 },
    { "he1","he1tr",_fi,  3, tx_print_nul, cm_get_thermistor_resistance, set_ro,           nullptr, 0 },
    { "he1","he1tv",_f0,  6, tx_print_nul, cm_get_thermistor_voltage, set_ro,              nullptr, 0 },
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
    { "he2","he2f", _fi,  5, tx_print_nul, cm_get_heater_f,        cm_set_heater_f,        nullptr, H2_DEFAULT_F },
    { "he2","he2st",_fi,  0, tx_print_nul, cm_get_set_temperature, cm_set_set_temperature, nullptr, 0 },
    { "he2","he2t", _fi,  1, tx_print_nul, cm_get_temperature,     set_ro,                 nullptr, 0 },
    { "he2","he2op",_fi,  3, tx_print_nul, cm_get_heater_output,   set_ro,                 nullptr, 0 },
    { "he2","he2tr",_fi,  3, tx_print_nul, cm_get_thermistor_resistance, set_ro,           nullptr, 0 },
    { "he1","he1tv",_f0,  6, tx_print_nul, cm_get_thermistor_voltage, set_ro,              nullptr, 0 },
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
    { "he3","he3f", _fi,  5, tx_print_nul, cm_get_heater_f,        cm_set_heater_f,        nullptr, H3_DEFAULT_F },
    { "he3","he3st",_fi,  0, tx_print_nul, cm_get_set_temperature, cm_set_set_temperature, nullptr, 0 },
    { "he3","he3t", _fi,  1, tx_print_nul, cm_get_temperature,     set_ro,                 nullptr, 0 },
    { "he3","he3op",_fi,  3, tx_print_nul, cm_get_heater_output,   set_ro,                 nullptr, 0 },
    { "he3","he3tr",_fi,  3, tx_print_nul, cm_get_thermistor_resistance, set_ro,           nullptr, 0 },
    { "he1","he1tv",_f0,  6, tx_print_nul, cm_get_thermistor_voltage, set_ro,              nullptr, 0 },
    { "he3","he3an",_fi,  0, tx_print_nul, cm_get_heater_adc,      set_ro,                 nullptr, 0 },
    { "he3","he3fp",_fi,  1, tx_print_nul, cm_get_fan_power,       cm_set_fan_power,       nullptr, 0 },
    { "he3","he3fm",_fi,  1, tx_print_nul, cm_get_fan_min_power,   cm_set_fan_min_power,   nullptr, 0 },
    { "he3","he3fl",_fi,  1, tx_print_nul, cm_get_fan_low_temp,    cm_set_fan_low_temp,    nullptr, 0 },
    { "he3","he3fh",_fi,  1, tx_print_nul, cm_get_fan_high_temp,   cm_set_fan_high_temp,   nullptr, 0 },
};
constexpr cfgSubtableFromStaticArray he_config_1 {he_config_items_1};
constexpr const configSubtable * const getHEConfig_1() { return &he_config_1; }

constexpr cfgItem_t cm_coor_config_items_1[] = {
    // Coordinate system offsets (G54-G59 and G92)
    { "g54","g54x",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_X_OFFSET },
    { "g54","g54y",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_Y_OFFSET },
    { "g54","g54z",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_Z_OFFSET },
#if (AXES == 9)
    { "g54","g54u",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_U_OFFSET },
    { "g54","g54v",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_V_OFFSET },
    { "g54","g54w",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_W_OFFSET },
#endif
    { "g54","g54a",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_A_OFFSET },
    { "g54","g54b",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_B_OFFSET },
    { "g54","g54c",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G54_C_OFFSET },

    { "g55","g55x",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_X_OFFSET },
    { "g55","g55y",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_Y_OFFSET },
    { "g55","g55z",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_Z_OFFSET },
#if (AXES == 9)
    { "g55","g55u",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_U_OFFSET },
    { "g55","g55v",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_V_OFFSET },
    { "g55","g55w",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_W_OFFSET },
#endif
    { "g55","g55a",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_A_OFFSET },
    { "g55","g55b",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_B_OFFSET },
    { "g55","g55c",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G55_C_OFFSET },

    { "g56","g56x",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_X_OFFSET },
    { "g56","g56y",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_Y_OFFSET },
    { "g56","g56z",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_Z_OFFSET },
#if (AXES == 9)
    { "g56","g56u",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_U_OFFSET },
    { "g56","g56v",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_V_OFFSET },
    { "g56","g56w",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_W_OFFSET },
#endif
    { "g56","g56a",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_A_OFFSET },
    { "g56","g56b",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_B_OFFSET },
    { "g56","g56c",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G56_C_OFFSET },

    { "g57","g57x",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_X_OFFSET },
    { "g57","g57y",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_Y_OFFSET },
    { "g57","g57z",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_Z_OFFSET },
#if (AXES == 9)
    { "g57","g57u",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_U_OFFSET },
    { "g57","g57v",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_V_OFFSET },
    { "g57","g57w",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_W_OFFSET },
#endif
    { "g57","g57a",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_A_OFFSET },
    { "g57","g57b",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_B_OFFSET },
    { "g57","g57c",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G57_C_OFFSET },

    { "g58","g58x",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_X_OFFSET },
    { "g58","g58y",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_Y_OFFSET },
    { "g58","g58z",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_Z_OFFSET },
#if (AXES == 9)
    { "g58","g58u",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_U_OFFSET },
    { "g58","g58v",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_V_OFFSET },
    { "g58","g58w",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_W_OFFSET },
#endif
    { "g58","g58a",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_A_OFFSET },
    { "g58","g58b",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_B_OFFSET },
    { "g58","g58c",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G58_C_OFFSET },

    { "g59","g59x",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_X_OFFSET },
    { "g59","g59y",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_Y_OFFSET },
    { "g59","g59z",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_Z_OFFSET },
#if (AXES == 9)
    { "g59","g59u",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_U_OFFSET },
    { "g59","g59v",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_V_OFFSET },
    { "g59","g59w",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_W_OFFSET },
#endif
    { "g59","g59a",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_A_OFFSET },
    { "g59","g59b",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_B_OFFSET },
    { "g59","g59c",_fipc, 5, cm_print_cofs, cm_get_coord, cm_set_coord, nullptr, G59_C_OFFSET },

    { "g92","g92x",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },// G92 handled differently
    { "g92","g92y",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },
    { "g92","g92z",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },
#if (AXES == 9)
    { "g92","g92u",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },
    { "g92","g92v",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },
    { "g92","g92w",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },
#endif
    { "g92","g92a",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },
    { "g92","g92b",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },
    { "g92","g92c",_fic, 5, cm_print_cofs, cm_get_g92, set_ro, nullptr, 0 },

    // Coordinate positions (G28, G30)
    { "g28","g28x",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },// g28 handled differently
    { "g28","g28y",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },
    { "g28","g28z",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },
#if (AXES == 9)
    { "g28","g28u",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },
    { "g28","g28v",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },
    { "g28","g28w",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },
#endif
    { "g28","g28a",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },
    { "g28","g28b",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },
    { "g28","g28c",_fic, 5, cm_print_cpos, cm_get_g28, set_ro, nullptr, 0 },

    { "g30","g30x",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },// g30 handled differently
    { "g30","g30y",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },
    { "g30","g30z",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },
#if (AXES == 9)
    { "g30","g30u",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },
    { "g30","g30v",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },
    { "g30","g30w",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },
#endif
    { "g30","g30a",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },
    { "g30","g30b",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },
    { "g30","g30c",_fic, 5, cm_print_cpos, cm_get_g30, set_ro, nullptr, 0 },
};
constexpr cfgSubtableFromStaticArray cm_coor_config_1 {cm_coor_config_items_1};
constexpr const configSubtable * const getCoorConfig_1() { return &cm_coor_config_1; }

constexpr cfgItem_t jobid_config_items_1[] = {
    // this is a 128bit UUID for identifying a previously committed job state
    { "jid","jida",_d0, 0, tx_print_nul, get_data, set_data, &cfg.job_id[0], 0 },
    { "jid","jidb",_d0, 0, tx_print_nul, get_data, set_data, &cfg.job_id[1], 0 },
    { "jid","jidc",_d0, 0, tx_print_nul, get_data, set_data, &cfg.job_id[2], 0 },
    { "jid","jidd",_d0, 0, tx_print_nul, get_data, set_data, &cfg.job_id[3], 0 },
};
constexpr cfgSubtableFromStaticArray jobid_config_1 {jobid_config_items_1};
constexpr const configSubtable * const getJobIDConfig_1() { return &jobid_config_1; }

constexpr cfgItem_t fixturing_config_items_1[] = {
    // fixturing information
    { "fxa","fxast",_fipc, 0, tx_print_nul, get_flt, set_flt, &cfg.fx_state_a, 0 },
    { "fxa","fxa1x",_fipc, 3, tx_print_nul, get_flt, set_flt, &cfg.fx_coords_a[0][0], 0 },
    { "fxa","fxa1y",_fipc, 3, tx_print_nul, get_flt, set_flt, &cfg.fx_coords_a[0][1], 0 },
    { "fxa","fxa2x",_fipc, 3, tx_print_nul, get_flt, set_flt, &cfg.fx_coords_a[1][0], 0 },
    { "fxa","fxa2y",_fipc, 3, tx_print_nul, get_flt, set_flt, &cfg.fx_coords_a[1][1], 0 },
    { "fxa","fxa3x",_fipc, 3, tx_print_nul, get_flt, set_flt, &cfg.fx_coords_a[2][0], 0 },
    { "fxa","fxa3y",_fipc, 3, tx_print_nul, get_flt, set_flt, &cfg.fx_coords_a[2][1], 0 },
    { "fxa","fxa4x",_fipc, 3, tx_print_nul, get_flt, set_flt, &cfg.fx_coords_a[3][0], 0 },
    { "fxa","fxa4y",_fipc, 3, tx_print_nul, get_flt, set_flt, &cfg.fx_coords_a[3][1], 0 },
};
constexpr cfgSubtableFromStaticArray fixturing_config_1 {fixturing_config_items_1};
constexpr const configSubtable * const getFixturingConfig_1() { return &fixturing_config_1; }

// spindle

constexpr cfgItem_t coolant_config_items_1[] = {
    // Coolant functions
    { "co","coph", _bip, 0, co_print_coph, co_get_coph, co_set_coph, nullptr, COOLANT_PAUSE_ON_HOLD },
    { "co","comp", _iip, 0, co_print_comp, co_get_comp, co_set_comp, nullptr, COOLANT_MIST_POLARITY },
    { "co","cofp", _iip, 0, co_print_cofp, co_get_cofp, co_set_cofp, nullptr, COOLANT_FLOOD_POLARITY },
    { "co","com",  _i0,  0, co_print_com,  co_get_com,  co_set_com,  nullptr, 0 },   // mist coolant enable
    { "co","cof",  _i0,  0, co_print_cof,  co_get_cof,  co_set_cof,  nullptr, 0 },   // flood coolant enable
};
constexpr cfgSubtableFromStaticArray coolant_config_1 {coolant_config_items_1};
constexpr const configSubtable * const getCoolantConfig_1() { return &coolant_config_1; }

constexpr cfgItem_t sys_config_items_2[] = {
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

    // kinematics controls
#if KINEMATICS==KINE_FOUR_CABLE
    { "sys","knfc", _f0, 4, tx_print_nul, kn_get_force,    kn_set_force,    nullptr,       0 },
    { "sys","knan", _f0, 0, tx_print_nul, kn_get_anchored, kn_set_anchored, nullptr,       0 },
    { "sys","knpa", _f0, 4, tx_print_nul, kn_get_pos_a,    set_nul,         nullptr,       0 },
    { "sys","knpb", _f0, 4, tx_print_nul, kn_get_pos_b,    set_nul,         nullptr,       0 },
    { "sys","knpc", _f0, 4, tx_print_nul, kn_get_pos_c,    set_nul,         nullptr,       0 },
    { "sys","knpd", _f0, 4, tx_print_nul, kn_get_pos_d,    set_nul,         nullptr,       0 },
#endif

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
    { "", "mark", _i0, 0, tx_print_nul,  get_int32, set_int32, &cfg.mark, 0 },
    { "", "flash",_b0, 0, tx_print_nul,  help_flash,hw_flash,  nullptr, 0 },

#ifdef __HELP_SCREENS
    { "", "help",_b0, 0, tx_print_nul, help_config, set_nul, nullptr, 0 },  // prints config help screen
    { "", "h",   _b0, 0, tx_print_nul, help_config, set_nul, nullptr, 0 },  // alias for "help"
#endif
};
constexpr cfgSubtableFromStaticArray sys_config_2 {sys_config_items_2};
constexpr const configSubtable * const getSysConfig_2() { return &sys_config_2; }

constexpr cfgItem_t user_data_config_items_1[] = {
#ifdef __USER_DATA
    // User defined data groups
    { "uda","uda0", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_a[0], USER_DATA_A0 },
    { "uda","uda1", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_a[1], USER_DATA_A1 },
    { "uda","uda2", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_a[2], USER_DATA_A2 },
    { "uda","uda3", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_a[3], USER_DATA_A3 },

    { "udb","udb0", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_b[0], USER_DATA_B0 },
    { "udb","udb1", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_b[1], USER_DATA_B1 },
    { "udb","udb2", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_b[2], USER_DATA_B2 },
    { "udb","udb3", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_b[3], USER_DATA_B3 },

    { "udc","udc0", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_c[0], USER_DATA_C0 },
    { "udc","udc1", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_c[1], USER_DATA_C1 },
    { "udc","udc2", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_c[2], USER_DATA_C2 },
    { "udc","udc3", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_c[3], USER_DATA_C3 },

    { "udd","udd0", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_d[0], USER_DATA_D0 },
    { "udd","udd1", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_d[1], USER_DATA_D1 },
    { "udd","udd2", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_d[2], USER_DATA_D2 },
    { "udd","udd3", _dip, 0, tx_print_int, get_data, set_data, &cfg.user_data_d[3], USER_DATA_D3 },
#endif
};
constexpr cfgSubtableFromStaticArray user_data_config_1 {user_data_config_items_1};
constexpr const configSubtable * const getUserDataConfig_1() { return &user_data_config_1; }

constexpr cfgItem_t tool_config_items_1[] = {
    // Tool table offsets
    { "tof","tofx",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },
    { "tof","tofy",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },
    { "tof","tofz",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },
#if (AXES == 9)
    { "tof","tofu",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },
    { "tof","tofv",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },
    { "tof","tofw",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },
#endif
    { "tof","tofa",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },
    { "tof","tofb",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },
    { "tof","tofc",_fipc, 5, cm_print_cofs, cm_get_tof, cm_set_tof, nullptr, 0 },

    // Tool table
    { "tt1","tt1x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT1_X_OFFSET },
    { "tt1","tt1y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT1_Y_OFFSET },
    { "tt1","tt1z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT1_Z_OFFSET },
#if (AXES == 9)
    { "tt1","tt1u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT1_U_OFFSET },
    { "tt1","tt1v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT1_V_OFFSET },
    { "tt1","tt1w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT1_W_OFFSET },
#endif
    { "tt1","tt1a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT1_A_OFFSET },
    { "tt1","tt1b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT1_B_OFFSET },
    { "tt1","tt1c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT1_C_OFFSET },

    { "tt2","tt2x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT2_X_OFFSET },
    { "tt2","tt2y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT2_Y_OFFSET },
    { "tt2","tt2z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT2_Z_OFFSET },
#if (AXES == 9)
    { "tt2","tt2u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT2_U_OFFSET },
    { "tt2","tt2v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT2_V_OFFSET },
    { "tt2","tt2w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT2_W_OFFSET },
#endif
    { "tt2","tt2a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT2_A_OFFSET },
    { "tt2","tt2b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT2_B_OFFSET },
    { "tt2","tt2c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT2_C_OFFSET },

    { "tt3","tt3x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT3_X_OFFSET },
    { "tt3","tt3y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT3_Y_OFFSET },
    { "tt3","tt3z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT3_Z_OFFSET },
#if (AXES == 9)
    { "tt3","tt3u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT3_U_OFFSET },
    { "tt3","tt3v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT3_V_OFFSET },
    { "tt3","tt3w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT3_W_OFFSET },
#endif
    { "tt3","tt3a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT3_A_OFFSET },
    { "tt3","tt3b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT3_B_OFFSET },
    { "tt3","tt3c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT1_C_OFFSET },

    { "tt4","tt4x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT4_X_OFFSET },
    { "tt4","tt4y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT4_Y_OFFSET },
    { "tt4","tt4z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT4_Z_OFFSET },
#if (AXES == 9)
    { "tt4","tt4u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT4_U_OFFSET },
    { "tt4","tt4v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT4_V_OFFSET },
    { "tt4","tt4w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT4_W_OFFSET },
#endif
    { "tt4","tt4a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT4_A_OFFSET },
    { "tt4","tt4b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT4_B_OFFSET },
    { "tt4","tt4c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT4_C_OFFSET },

    { "tt5","tt5x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT5_X_OFFSET },
    { "tt5","tt5y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT5_Y_OFFSET },
    { "tt5","tt5z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT5_Z_OFFSET },
#if (AXES == 9)
    { "tt5","tt5u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT5_U_OFFSET },
    { "tt5","tt5v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT5_V_OFFSET },
    { "tt5","tt5w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT5_W_OFFSET },
#endif
    { "tt5","tt5a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT5_A_OFFSET },
    { "tt5","tt5b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT5_B_OFFSET },
    { "tt5","tt5c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT5_C_OFFSET },

#if (TOOLS > 5)
    { "tt6","tt6x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT6_X_OFFSET },
    { "tt6","tt6y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT6_Y_OFFSET },
    { "tt6","tt6z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT6_Z_OFFSET },
#if (AXES == 9)
    { "tt6","tt6u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT6_U_OFFSET },
    { "tt6","tt6v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT6_V_OFFSET },
    { "tt6","tt6w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT6_W_OFFSET },
#endif
    { "tt6","tt6a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT6_A_OFFSET },
    { "tt6","tt6b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT6_B_OFFSET },
    { "tt6","tt6c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT6_C_OFFSET },

    { "tt7","tt7x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT7_X_OFFSET },
    { "tt7","tt7y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT7_Y_OFFSET },
    { "tt7","tt7z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT7_Z_OFFSET },
#if (AXES == 9)
    { "tt7","tt7u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT7_U_OFFSET },
    { "tt7","tt7v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT7_V_OFFSET },
    { "tt7","tt7w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT7_W_OFFSET },
#endif
    { "tt7","tt7a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT7_A_OFFSET },
    { "tt7","tt7b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT7_B_OFFSET },
    { "tt7","tt7c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT7_C_OFFSET },

    { "tt8","tt8x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT8_X_OFFSET },
    { "tt8","tt8y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT8_Y_OFFSET },
    { "tt8","tt8z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT8_Z_OFFSET },
#if (AXES == 9)
    { "tt8","tt8u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT8_U_OFFSET },
    { "tt8","tt8v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT8_V_OFFSET },
    { "tt8","tt8w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT8_W_OFFSET },
#endif
    { "tt8","tt8a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT8_A_OFFSET },
    { "tt8","tt8b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT8_B_OFFSET },
    { "tt8","tt8c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT8_C_OFFSET },

    { "tt9","tt9x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT9_X_OFFSET },
    { "tt9","tt9y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT9_Y_OFFSET },
    { "tt9","tt9z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT9_Z_OFFSET },
#if (AXES == 9)
    { "tt9","tt9u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT9_U_OFFSET },
    { "tt9","tt9v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT9_V_OFFSET },
    { "tt9","tt9w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT9_W_OFFSET },
#endif
    { "tt9","tt9a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT9_A_OFFSET },
    { "tt9","tt9b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT9_B_OFFSET },
    { "tt9","tt9c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT9_C_OFFSET },

    { "tt10","tt10x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT10_X_OFFSET },
    { "tt10","tt10y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT10_Y_OFFSET },
    { "tt10","tt10z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT10_Z_OFFSET },
#if (AXES == 9)
    { "tt10","tt10u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT10_U_OFFSET },
    { "tt10","tt10v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT10_V_OFFSET },
    { "tt10","tt10w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT10_W_OFFSET },
#endif
    { "tt10","tt10a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT10_A_OFFSET },
    { "tt10","tt10b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT10_B_OFFSET },
    { "tt10","tt10c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT10_C_OFFSET },

    { "tt11","tt11x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT11_X_OFFSET },
    { "tt11","tt11y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT11_Y_OFFSET },
    { "tt11","tt11z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT11_Z_OFFSET },
#if (AXES == 9)
    { "tt11","tt11u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT11_U_OFFSET },
    { "tt11","tt11v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT11_V_OFFSET },
    { "tt11","tt11w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT11_W_OFFSET },
#endif
    { "tt11","tt11a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT11_A_OFFSET },
    { "tt11","tt11b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT11_B_OFFSET },
    { "tt11","tt11c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT11_C_OFFSET },

    { "tt12","tt12x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT12_X_OFFSET },
    { "tt12","tt12y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT12_Y_OFFSET },
    { "tt12","tt12z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT12_Z_OFFSET },
#if (AXES == 9)
    { "tt12","tt12u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT12_U_OFFSET },
    { "tt12","tt12v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT12_V_OFFSET },
    { "tt12","tt12w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT12_W_OFFSET },
#endif
    { "tt12","tt12a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT12_A_OFFSET },
    { "tt12","tt12b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT12_B_OFFSET },
    { "tt12","tt12c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT12_C_OFFSET },

    { "tt13","tt13x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT13_X_OFFSET },
    { "tt13","tt13y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT13_Y_OFFSET },
    { "tt13","tt13z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT13_Z_OFFSET },
#if (AXES == 9)
    { "tt13","tt13u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT13_U_OFFSET },
    { "tt13","tt13v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT13_V_OFFSET },
    { "tt13","tt13w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT13_W_OFFSET },
#endif
    { "tt13","tt13a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT13_A_OFFSET },
    { "tt13","tt13b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT13_B_OFFSET },
    { "tt13","tt13c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT13_C_OFFSET },

    { "tt14","tt14x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT14_X_OFFSET },
    { "tt14","tt14y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT14_Y_OFFSET },
    { "tt14","tt14z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT14_Z_OFFSET },
#if (AXES == 9)
    { "tt14","tt14u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT14_U_OFFSET },
    { "tt14","tt14v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT14_V_OFFSET },
    { "tt14","tt14w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT14_W_OFFSET },
#endif
    { "tt14","tt14a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT14_A_OFFSET },
    { "tt14","tt14b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT14_B_OFFSET },
    { "tt14","tt14c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT14_C_OFFSET },

    { "tt15","tt15x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT15_X_OFFSET },
    { "tt15","tt15y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT15_Y_OFFSET },
    { "tt15","tt15z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT15_Z_OFFSET },
#if (AXES == 9)
    { "tt15","tt15u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT15_U_OFFSET },
    { "tt15","tt15v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT15_V_OFFSET },
    { "tt15","tt15w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT15_W_OFFSET },
#endif
    { "tt15","tt15a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT15_A_OFFSET },
    { "tt15","tt15b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT15_B_OFFSET },
    { "tt15","tt15c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT15_C_OFFSET },

    { "tt16","tt16x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT16_X_OFFSET },
    { "tt16","tt16y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT16_Y_OFFSET },
    { "tt16","tt16z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT16_Z_OFFSET },
#if (AXES == 9)
    { "tt16","tt16u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT16_U_OFFSET },
    { "tt16","tt16v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT16_V_OFFSET },
    { "tt16","tt16w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT16_W_OFFSET },
#endif
    { "tt16","tt16a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT16_A_OFFSET },
    { "tt16","tt16b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT16_B_OFFSET },
    { "tt16","tt16c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT16_C_OFFSET },

    { "tt17","tt17x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT17_X_OFFSET },
    { "tt17","tt17y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT17_Y_OFFSET },
    { "tt17","tt17z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT17_Z_OFFSET },
#if (AXES == 9)
    { "tt17","tt17u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT17_U_OFFSET },
    { "tt17","tt17v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT17_V_OFFSET },
    { "tt17","tt17w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT17_W_OFFSET },
#endif
    { "tt17","tt17a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT17_A_OFFSET },
    { "tt17","tt17b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT17_B_OFFSET },
    { "tt17","tt17c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT17_C_OFFSET },

    { "tt18","tt18x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT18_X_OFFSET },
    { "tt18","tt18y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT18_Y_OFFSET },
    { "tt18","tt18z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT18_Z_OFFSET },
#if (AXES == 9)
    { "tt18","tt18u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT18_U_OFFSET },
    { "tt18","tt18v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT18_V_OFFSET },
    { "tt18","tt18w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT18_W_OFFSET },
#endif
    { "tt18","tt18a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT18_A_OFFSET },
    { "tt18","tt18b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT18_B_OFFSET },
    { "tt18","tt18c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT18_C_OFFSET },

    { "tt19","tt19x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT19_X_OFFSET },
    { "tt19","tt19y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT19_Y_OFFSET },
    { "tt19","tt19z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT19_Z_OFFSET },
#if (AXES == 9)
    { "tt19","tt19u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT19_U_OFFSET },
    { "tt19","tt19v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT19_V_OFFSET },
    { "tt19","tt19w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT19_W_OFFSET },
#endif
    { "tt19","tt19a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT19_A_OFFSET },
    { "tt19","tt19b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT19_B_OFFSET },
    { "tt19","tt19c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT19_C_OFFSET },

    { "tt20","tt20x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT20_X_OFFSET },
    { "tt20","tt20y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT20_Y_OFFSET },
    { "tt20","tt20z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT20_Z_OFFSET },
#if (AXES == 9)
    { "tt20","tt20u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT20_U_OFFSET },
    { "tt20","tt20v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT20_V_OFFSET },
    { "tt20","tt20w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT20_W_OFFSET },
#endif
    { "tt20","tt20a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT20_A_OFFSET },
    { "tt20","tt20b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT20_B_OFFSET },
    { "tt20","tt20c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT20_C_OFFSET },

    { "tt21","tt21x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT21_X_OFFSET },
    { "tt21","tt21y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT21_Y_OFFSET },
    { "tt21","tt21z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT21_Z_OFFSET },
#if (AXES == 9)
    { "tt21","tt21u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT21_U_OFFSET },
    { "tt21","tt21v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT21_V_OFFSET },
    { "tt21","tt21w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT21_W_OFFSET },
#endif
    { "tt21","tt21a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT21_A_OFFSET },
    { "tt21","tt21b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT21_B_OFFSET },
    { "tt21","tt21c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT21_C_OFFSET },

    { "tt22","tt22x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT22_X_OFFSET },
    { "tt22","tt22y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT22_Y_OFFSET },
    { "tt22","tt22z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT22_Z_OFFSET },
#if (AXES == 9)
    { "tt22","tt22u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT22_U_OFFSET },
    { "tt22","tt22v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT22_V_OFFSET },
    { "tt22","tt22w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT22_W_OFFSET },
#endif
    { "tt22","tt22a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT22_A_OFFSET },
    { "tt22","tt22b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT22_B_OFFSET },
    { "tt22","tt22c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT22_C_OFFSET },

    { "tt23","tt23x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT23_X_OFFSET },
    { "tt23","tt23y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT23_Y_OFFSET },
    { "tt23","tt23z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT23_Z_OFFSET },
#if (AXES == 9)
    { "tt23","tt23u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT23_U_OFFSET },
    { "tt23","tt23v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT23_V_OFFSET },
    { "tt23","tt23w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT23_W_OFFSET },
#endif
    { "tt23","tt23a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT23_A_OFFSET },
    { "tt23","tt23b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT23_B_OFFSET },
    { "tt23","tt23c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT23_C_OFFSET },

    { "tt24","tt24x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT24_X_OFFSET },
    { "tt24","tt24y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT24_Y_OFFSET },
    { "tt24","tt24z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT24_Z_OFFSET },
#if (AXES == 9)
    { "tt24","tt24u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT24_U_OFFSET },
    { "tt24","tt24v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT24_V_OFFSET },
    { "tt24","tt24w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT24_W_OFFSET },
#endif
    { "tt24","tt24a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT24_A_OFFSET },
    { "tt24","tt24b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT24_B_OFFSET },
    { "tt24","tt24c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT24_C_OFFSET },

    { "tt25","tt25x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT25_X_OFFSET },
    { "tt25","tt25y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT25_Y_OFFSET },
    { "tt25","tt25z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT25_Z_OFFSET },
#if (AXES == 9)
    { "tt25","tt25u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT25_U_OFFSET },
    { "tt25","tt25v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT25_V_OFFSET },
    { "tt25","tt25w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT25_W_OFFSET },
#endif
    { "tt25","tt25a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT25_A_OFFSET },
    { "tt25","tt25b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT25_B_OFFSET },
    { "tt25","tt25c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT25_C_OFFSET },

    { "tt26","tt26x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT26_X_OFFSET },
    { "tt26","tt26y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT26_Y_OFFSET },
    { "tt26","tt26z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT26_Z_OFFSET },
#if (AXES == 9)
    { "tt26","tt26u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT26_U_OFFSET },
    { "tt26","tt26v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT26_V_OFFSET },
    { "tt26","tt26w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT26_W_OFFSET },
#endif
    { "tt26","tt26a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT26_A_OFFSET },
    { "tt26","tt26b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT26_B_OFFSET },
    { "tt26","tt26c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT26_C_OFFSET },

    { "tt27","tt27x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT27_X_OFFSET },
    { "tt27","tt27y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT27_Y_OFFSET },
    { "tt27","tt27z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT27_Z_OFFSET },
#if (AXES == 9)
    { "tt27","tt27u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT27_U_OFFSET },
    { "tt27","tt27v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT27_V_OFFSET },
    { "tt27","tt27w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT27_W_OFFSET },
#endif
    { "tt27","tt27a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT27_A_OFFSET },
    { "tt27","tt27b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT27_B_OFFSET },
    { "tt27","tt27c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT27_C_OFFSET },

    { "tt28","tt28x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT28_X_OFFSET },
    { "tt28","tt28y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT28_Y_OFFSET },
    { "tt28","tt28z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT28_Z_OFFSET },
#if (AXES == 9)
    { "tt28","tt28u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT28_U_OFFSET },
    { "tt28","tt28v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT28_V_OFFSET },
    { "tt28","tt28w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT28_W_OFFSET },
#endif
    { "tt28","tt28a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT28_A_OFFSET },
    { "tt28","tt28b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT28_B_OFFSET },
    { "tt28","tt28c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT28_C_OFFSET },

    { "tt29","tt29x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT29_X_OFFSET },
    { "tt29","tt29y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT29_Y_OFFSET },
    { "tt29","tt29z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT29_Z_OFFSET },
#if (AXES == 9)
    { "tt29","tt29u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT29_U_OFFSET },
    { "tt29","tt29v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT29_V_OFFSET },
    { "tt29","tt29w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT29_W_OFFSET },
#endif
    { "tt29","tt29a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT29_A_OFFSET },
    { "tt29","tt29b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT29_B_OFFSET },
    { "tt29","tt29c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT29_C_OFFSET },

    { "tt30","tt30x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT30_X_OFFSET },
    { "tt30","tt30y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT30_Y_OFFSET },
    { "tt30","tt30z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT30_Z_OFFSET },
#if (AXES == 9)
    { "tt30","tt30u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT30_U_OFFSET },
    { "tt30","tt30v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT30_V_OFFSET },
    { "tt30","tt30w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT30_W_OFFSET },
#endif
    { "tt30","tt30a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT30_A_OFFSET },
    { "tt30","tt30b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT30_B_OFFSET },
    { "tt30","tt30c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT30_C_OFFSET },

    { "tt31","tt31x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT31_X_OFFSET },
    { "tt31","tt31y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT31_Y_OFFSET },
    { "tt31","tt31z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT31_Z_OFFSET },
#if (AXES == 9)
    { "tt31","tt31u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT31_U_OFFSET },
    { "tt31","tt31v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT31_V_OFFSET },
    { "tt31","tt31w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT31_W_OFFSET },
#endif
    { "tt31","tt31a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT31_A_OFFSET },
    { "tt31","tt31b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT31_B_OFFSET },
    { "tt31","tt31c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT31_C_OFFSET },

    { "tt32","tt32x",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT32_X_OFFSET },
    { "tt32","tt32y",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT32_Y_OFFSET },
    { "tt32","tt32z",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT32_Z_OFFSET },
#if (AXES == 9)
    { "tt32","tt32u",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT32_U_OFFSET },
    { "tt32","tt32v",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT32_V_OFFSET },
    { "tt32","tt32w",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT32_W_OFFSET },
#endif
    { "tt32","tt32a",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT32_A_OFFSET },
    { "tt32","tt32b",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT32_B_OFFSET },
    { "tt32","tt32c",_fipc, 5, cm_print_cofs, cm_get_tt, cm_set_tt, nullptr, TT32_C_OFFSET },
#endif // TOOLS > 5
};
constexpr cfgSubtableFromStaticArray tool_config_1 {tool_config_items_1};
constexpr const configSubtable * const getToolConfig_1() { return &tool_config_1; }

#ifdef __DIAGNOSTIC_PARAMETERS
// Diagnostic parameters
constexpr cfgItem_t diagnostic_config_items_1[] = {
    { "",    "clc",_f0, 0, tx_print_nul, st_clc,  st_clc, nullptr, 0 },  // clear diagnostic step counters

    { "_te","_tex",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.target[AXIS_X], 0 }, // X target endpoint
    { "_te","_tey",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.target[AXIS_Y], 0 },
    { "_te","_tez",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.target[AXIS_Z], 0 },
    { "_te","_tea",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.target[AXIS_A], 0 },
    { "_te","_teb",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.target[AXIS_B], 0 },
    { "_te","_tec",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.target[AXIS_C], 0 },

    { "_tr","_trx",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.gm.target[AXIS_X], 0 },  // X target runtime
    { "_tr","_try",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.gm.target[AXIS_Y], 0 },
    { "_tr","_trz",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.gm.target[AXIS_Z], 0 },
    { "_tr","_tra",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.gm.target[AXIS_A], 0 },
    { "_tr","_trb",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.gm.target[AXIS_B], 0 },
    { "_tr","_trc",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.gm.target[AXIS_C], 0 },
};
constexpr cfgSubtableFromStaticArray diagnostic_config_1 {diagnostic_config_items_1};
constexpr const configSubtable * const getDiagnosticConfig_1() { return &diagnostic_config_1; }

constexpr cfgItem_t motor_diagnostic_config_items_1[] = {
#if (MOTORS >= 1)
    { "_ts","_ts1",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.target_steps[MOTOR_1], 0 },      // Motor 1 target steps
    { "_ps","_ps1",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.position_steps[MOTOR_1], 0 },    // Motor 1 position steps
    { "_cs","_cs1",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.commanded_steps[MOTOR_1], 0 },   // Motor 1 commanded steps (delayed steps)
    { "_es","_es1",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.encoder_steps[MOTOR_1], 0 },     // Motor 1 encoder steps
    { "_xs","_xs1",_f0, 2, tx_print_flt, get_flt, set_nul, &st_pre.mot[MOTOR_1].corrected_steps, 0 }, // Motor 1 correction steps applied
    { "_fe","_fe1",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.following_error[MOTOR_1], 0 },   // Motor 1 following error in steps
#endif
#if (MOTORS >= 2)
    { "_ts","_ts2",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.target_steps[MOTOR_2], 0 },
    { "_ps","_ps2",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.position_steps[MOTOR_2], 0 },
    { "_cs","_cs2",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.commanded_steps[MOTOR_2], 0 },
    { "_es","_es2",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.encoder_steps[MOTOR_2], 0 },
    { "_xs","_xs2",_f0, 2, tx_print_flt, get_flt, set_nul, &st_pre.mot[MOTOR_2].corrected_steps, 0 },
    { "_fe","_fe2",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.following_error[MOTOR_2], 0 },
#endif
#if (MOTORS >= 3)
    { "_ts","_ts3",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.target_steps[MOTOR_3], 0 },
    { "_ps","_ps3",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.position_steps[MOTOR_3], 0 },
    { "_cs","_cs3",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.commanded_steps[MOTOR_3], 0 },
    { "_es","_es3",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.encoder_steps[MOTOR_3], 0 },
    { "_xs","_xs3",_f0, 2, tx_print_flt, get_flt, set_nul, &st_pre.mot[MOTOR_3].corrected_steps, 0 },
    { "_fe","_fe3",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.following_error[MOTOR_3], 0 },
#endif
#if (MOTORS >= 4)
    { "_ts","_ts4",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.target_steps[MOTOR_4], 0 },
    { "_ps","_ps4",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.position_steps[MOTOR_4], 0 },
    { "_cs","_cs4",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.commanded_steps[MOTOR_4], 0 },
    { "_es","_es4",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.encoder_steps[MOTOR_4], 0 },
    { "_xs","_xs4",_f0, 2, tx_print_flt, get_flt, set_nul, &st_pre.mot[MOTOR_4].corrected_steps, 0 },
    { "_fe","_fe4",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.following_error[MOTOR_4], 0 },
#endif
#if (MOTORS >= 5)
    { "_ts","_ts5",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.target_steps[MOTOR_5], 0 },
    { "_ps","_ps5",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.position_steps[MOTOR_5], 0 },
    { "_cs","_cs5",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.commanded_steps[MOTOR_5], 0 },
    { "_es","_es5",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.encoder_steps[MOTOR_5], 0 },
    { "_xs","_xs5",_f0, 2, tx_print_flt, get_flt, set_nul, &st_pre.mot[MOTOR_5].corrected_steps, 0 },
    { "_fe","_fe5",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.following_error[MOTOR_5], 0 },
#endif
#if (MOTORS >= 6)
    { "_ts","_ts6",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.target_steps[MOTOR_6], 0 },
    { "_ps","_ps6",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.position_steps[MOTOR_6], 0 },
    { "_cs","_cs6",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.commanded_steps[MOTOR_6], 0 },
    { "_es","_es6",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.encoder_steps[MOTOR_6], 0 },
    { "_xs","_xs6",_f0, 2, tx_print_flt, get_flt, set_nul, &st_pre.mot[MOTOR_6].corrected_steps, 0 },
    { "_fe","_fe6",_f0, 2, tx_print_flt, get_flt, set_nul, &mr1.following_error[MOTOR_6], 0 },
#endif
};
#endif  //  __DIAGNOSTIC_PARAMETERS
constexpr cfgSubtableFromStaticArray motor_diagnostic_config_1 {motor_diagnostic_config_items_1};
constexpr const configSubtable * const getMotorDiagnosticConfig_1() { return &motor_diagnostic_config_1; }

constexpr cfgItem_t sr_presistence_config_items_1[] = {
  // Persistence for status report - must be in sequence
    // *** Count must agree with NV_STATUS_REPORT_LEN in report.h ***
    { "","se00",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[0].index,0 }, // 950
    { "","se01",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[1].index,0 },
    { "","se02",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[2].index,0 },
    { "","se03",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[3].index,0 },
    { "","se04",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[4].index,0 },
    { "","se05",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[5].index,0 },
    { "","se06",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[6].index,0 },
    { "","se07",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[7].index,0 },
    { "","se08",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[8].index,0 },
    { "","se09",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[9].index,0 },
    { "","se10",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[10].index,0 },
    { "","se11",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[11].index,0 },
    { "","se12",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[12].index,0 },
    { "","se13",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[13].index,0 },
    { "","se14",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[14].index,0 },
    { "","se15",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[15].index,0 },
    { "","se16",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[16].index,0 },
    { "","se17",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[17].index,0 },
    { "","se18",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[18].index,0 },
    { "","se19",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[19].index,0 },
    { "","se20",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[20].index,0 },
    { "","se21",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[21].index,0 },
    { "","se22",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[22].index,0 },
    { "","se23",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[23].index,0 },
    { "","se24",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[24].index,0 },
    { "","se25",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[25].index,0 },
    { "","se26",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[26].index,0 },
    { "","se27",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[27].index,0 },
    { "","se28",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[28].index,0 },
    { "","se29",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[29].index,0 },
    { "","se30",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[30].index,0 },
    { "","se31",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[31].index,0 },
    { "","se32",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[32].index,0 },
    { "","se33",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[33].index,0 },
    { "","se34",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[34].index,0 },
    { "","se35",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[35].index,0 },
    { "","se36",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[36].index,0 },
    { "","se37",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[37].index,0 },
    { "","se38",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[38].index,0 },
    // { "","se39",_ip, 0, tx_print_nul, get_int32, set_int32, &sr.status_report_list[39].index,0 },
    // Count is 40, since se00 counts as one.
};
constexpr cfgSubtableFromStaticArray sr_presistence_config_1 {sr_presistence_config_items_1};
constexpr const configSubtable * const getSrPersistenceConfig_1() { return &sr_presistence_config_1; }

    // Group lookups - must follow the single-valued entries for proper sub-string matching
    // *** Must agree with NV_COUNT_GROUPS below ***
    // *** If you adjust the number of entries in a group you must also adjust the count for that group ***
    // *** COUNT STARTS FROM HERE ***

constexpr cfgItem_t groups_config_items_1[] = {
#define FIXED_GROUPS 4
    { "","sys",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // system group
    { "","p1", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // PWM 1 group
    { "","sp", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // Spindle group
    { "","co", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // Coolant group

#define AXIS_GROUPS AXES
    { "","x",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // axis groups
    { "","y",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","z",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
#if (AXES == 9)
    { "","u",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","v",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
    { "","w",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
#endif
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

#define DIGITAL_IN_GROUPS D_IN_CHANNELS
    { "","in",  _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // input state
#if (D_IN_CHANNELS >= 1)
    { "","di1", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },   // input configs
#endif
#if (D_IN_CHANNELS >= 2)
    { "","di2", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
#endif
#if (D_IN_CHANNELS >= 3)
    { "","di3", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
#endif
#if (D_IN_CHANNELS >= 4)
    { "","di4", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
#endif
#if (D_IN_CHANNELS >= 5)
    { "","di5", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
#endif
#if (D_IN_CHANNELS >= 6)
    { "","di6", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
#endif
#if (D_IN_CHANNELS >= 7)
    { "","di7", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
#endif
#if (D_IN_CHANNELS >= 8)
    { "","di8", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
#endif
#if (D_IN_CHANNELS >= 9)
    { "","di9", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
#endif
#if (D_IN_CHANNELS >= 10)
    { "","di10", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },
#endif

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

#define ANALOG_IN_GROUPS 12
    { "","ai1", _f0, 0, tx_print_nul, get_grp, set_grp, nullptr,0 },   // analog input configs
    { "","ai2", _f0, 0, tx_print_nul, get_grp, set_grp,nullptr,0 },
    { "","ai3", _f0, 0, tx_print_nul, get_grp, set_grp,nullptr,0 },
    { "","ai4", _f0, 0, tx_print_nul, get_grp, set_grp,nullptr,0 },
    { "","ain1", _f0, 0, tx_print_nul, get_grp, set_grp,nullptr,0 },   // analog input configs
    { "","ain2", _f0, 0, tx_print_nul, get_grp, set_grp,nullptr,0 },
    { "","ain3", _f0, 0, tx_print_nul, get_grp, set_grp,nullptr,0 },
    { "","ain4", _f0, 0, tx_print_nul, get_grp, set_grp,nullptr,0 },
    { "","ain5", _f0, 0, tx_print_nul, get_grp, set_grp,nullptr,0 },   // analog input configs
    { "","ain6", _f0, 0, tx_print_nul, get_grp, set_grp,nullptr,0 },
    { "","ain7", _f0, 0, tx_print_nul, get_grp, set_grp,nullptr,0 },
    { "","ain8", _f0, 0, tx_print_nul, get_grp, set_grp,nullptr,0 },

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

#define TOOL_OFFSET_GROUPS (TOOLS+1)
    { "","tof",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // current tool offsets
    { "","tt1",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
    { "","tt2",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
    { "","tt3",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
    { "","tt4",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
    { "","tt5",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // tt offsets
#if (TOOLS > 5)
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
#endif

#define MACHINE_STATE_GROUPS 9
    { "","mpo",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // machine position group
    { "","pos",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // work position group
    { "","ofs",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // work offset group
    { "","hom",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // axis homing state group
    { "","prb",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // probing state group
    { "","pwr",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // motor power enagled group
    { "","jog",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // axis jogging state group
    { "","jid",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // job ID group
    { "","fxa",_f0, 0, tx_print_nul, get_grp, set_grp, nullptr, 0 },    // fixturing group a

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
#else
#define USER_DATA_GROUPS 0
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
};
constexpr cfgSubtableFromStaticArray groups_config_1 {groups_config_items_1};
constexpr const configSubtable * const getGroupsConfig_1() { return &groups_config_1; }

constexpr cfgItem_t uber_groups_config_items_1[] = {
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
constexpr cfgSubtableFromStaticArray uber_groups_config_1 {uber_groups_config_items_1};
constexpr const configSubtable * const getUberGroupsConfig_1() { return &uber_groups_config_1; }

auto nodes = makeSubtableNodes(
    0, getSysConfig_1(), getCmConfig_1(), getMpoConfig_1(), getPosConfig_1(), getOfsConfig_1(), getHomConfig_1(),
    getPrbConfig_1(), getJogConfig_1(), getPwrConfig_1(), getMotorConfig_1(), getAxisConfig_1(), getDIConfig_1(),
    getINConfig_1(), getDOConfig_1(), getOUTConfig_1(), getAINConfig_1(), getP1Config_1(), getPIDConfig_1(),
    getHEConfig_1(), getCoorConfig_1(), getJobIDConfig_1(), getFixturingConfig_1(), getSpindleConfig_1(),
    getCoolantConfig_1(), getSysConfig_2(), getSysConfig_3(), getUserDataConfig_1(), getToolConfig_1(), getDiagnosticConfig_1(),
    getMotorDiagnosticConfig_1(), getSrPersistenceConfig_1(), getGroupsConfig_1(), getUberGroupsConfig_1());


// template <typename T, size_t length>
// constexpr size_t size_of_array(T(&)[length]) {
//     return length;
// }


configSubtableNode *configSubtableHead = &nodes.this_node;

// Dummy config item for when there's an error
constexpr cfgItem_t nullCfg = {"", "", _f0, 0, tx_print_nul, get_nul, set_nul, nullptr, 0};

// Sythesize the old cfgArray[...] operator, must return SOMETHING
const cfgItem_t &cfgArraySynthesizer::operator[](std::size_t idx) const {
    if (!configSubtableHead) {
        return nullCfg;
    }
    const cfgItem_t * const c = configSubtableHead->get(idx);
    if (!c) {
        return nullCfg;
    }
    return *c;
}

index_t cfgArraySynthesizer::getIndex(const char *group, const char *token)
{
    if (!configSubtableHead) {
        return (NO_MATCH);
    }
    char str[TOKEN_LEN + GROUP_LEN+1];    // should actually never be more than TOKEN_LEN+1
    strncpy(str, group, GROUP_LEN+1);
    strncat(str, token, TOKEN_LEN+1);
    return configSubtableHead->find(str);

    // index_t i;
    // index_t index_max = nv_index_max();

    // for (i=0; i < index_max; i++) {
    //     auto config = cfgArray[i];
    //     if ((c = config.token[0]) != str[0]) { continue; }                  // 1st character mismatch
    //     if ((c = config.token[1]) == NUL) { if (str[1] == NUL) return(i); } // one character match
    //     if (c != str[1]) continue;                                          // 2nd character mismatch
    //     if ((c = config.token[2]) == NUL) { if (str[2] == NUL) return(i); } // two character match
    //     if (c != str[2]) continue;                                          // 3rd character mismatch
    //     if ((c = config.token[3]) == NUL) { if (str[3] == NUL) return(i); } // three character match
    //     if (c != str[3]) continue;                                          // 4th character mismatch
    //     if ((c = config.token[4]) == NUL) { if (str[4] == NUL) return(i); } // four character match
    //     if (c != str[4]) continue;                                          // 5th character mismatch
    //     if ((c = config.token[5]) == NUL) { if (str[5] == NUL) return(i); } // four character match
    //     if (c != str[5]) continue;                                          // 6th character mismatch
    //     if ((c = config.token[6]) == NUL) { if (str[6] == NUL) return(i); } // four character match
    //     if (c != str[6]) continue;                                          // 7th character mismatch
    //     if ((c = config.token[7]) == NUL) { if (str[7] == NUL) return(i); } // four character match
    //     if (c != str[7]) continue;                                          // 8th character mismatch
    //     if ((c = config.token[8]) == NUL) { if (str[8] == NUL) return(i); } // four character match
    //     if (c != str[8]) continue;                                          // 9th character mismatch
    //     return (i);                                                         // five character match
    // }
    // return (NO_MATCH);
}

cfgArraySynthesizer cfgArray {};

/***** Make sure these defines line up with any changes in the above table *****/

#define NV_COUNT_GROUPS (FIXED_GROUPS \
                        + AXIS_GROUPS \
                        + MOTOR_GROUPS \
                        + DIGITAL_IN_GROUPS \
                        + DIGITAL_OUT_GROUPS \
                        + ANALOG_IN_GROUPS \
                        + COORDINATE_OFFSET_GROUPS \
                        + TOOL_OFFSET_GROUPS \
                        + MACHINE_STATE_GROUPS \
                        + TEMPERATURE_GROUPS \
                        + USER_DATA_GROUPS \
                        + DIAGNOSTIC_GROUPS)

/* <DO NOT MESS WITH THESE DEFINES> */
#define NV_INDEX_MAX (nodes.this_node.length)
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

stat_t set_uint32(nvObj_t *nv, uint32_t &value, int32_t low, int32_t high)
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
#if (AXES == 9)
    char list[][TOKEN_LEN+1] = {"x","y","z","u","v","w","a","b","c",""}; // must have a terminating element
#else
    char list[][TOKEN_LEN+1] = {"x","y","z","a","b","c",""}; // must have a terminating element
#endif
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
    for (uint8_t i=1; i < A_IN_CHANNELS+1; i++) {
        sprintf(group, "ain%d", i);
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
