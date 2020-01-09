/*
 * bantam_safety_manager.cpp - The safety manager handles interlock and spindle safety controls
 * This file is part of the g2core project
 *
 * Copyright (c) 2019 Rob Giseburt
 *
 * This code is a loose implementation of Kramer, Proctor and Messina's
 * canonical machining functions as described in the NIST RS274/NGC v3
 */
/* This file ("the software") is free software: you can redistribute it and/or modify
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

#include "bantam_safety_manager.h"

BantamSafetyManager *bsm;

stat_t _bt_get_msg_helper(nvObj_t *nv, const char *const msg_array[], int32_t value)
{
    nv->value_int = value;
    nv->valuetype = TYPE_INTEGER;
    return(nv_copy_string(nv, (const char *)GET_TEXT_ITEM(msg_array, value)));
}

#ifdef __TEXT_MODE

static const char msg_safe0[] = "Interlock Circuit Closed/ESC nominal";
static const char msg_safe1[] = "Interlock Circuit Broken/ESC nominal";
static const char msg_safe2[] = "Interlock Circuit Closed/ESC rebooting";
static const char msg_safe3[] = "Interlock Circuit Broken/ESC rebooting";
static const char *const msg_safe[] = { msg_safe0, msg_safe1, msg_safe2, msg_safe3 };

static const char msg_estp0[] = "E-Stop Circuit Closed";
static const char msg_estp1[] = "E-Stop Circuit Closed but unacked";
static const char msg_estp2[] = "E-Stop Circuit Broken and acked";
static const char msg_estp3[] = "E-Stop Circuit Broken and unacked";
// Don't worry about indicating the "Active" state
static const char *const msg_estp[] = { msg_estp0, msg_estp1, msg_estp2, msg_estp3 };

static const char fmt_safe[] = "Safety System Flags: %s\n";
static const char fmt_estp[] = "Emergency Stop:      %s\n";

void cm_print_safe(nvObj_t *nv) { text_print_str(nv, fmt_safe);}
void cm_print_estp(nvObj_t *nv) { text_print_str(nv, fmt_estp);}

#else

#define msg_safe NULL
#define msg_estp NULL

#define cm_print_safe tx_print_stub
#define cm_print_estp tx_print_stub

#endif // __TEXT_MODE

stat_t cm_ack_estop(nvObj_t *nv)
{
    if (!bsm) {
        return (STAT_OK);
    }
    bsm->ack_estop();
    nv->value_flt = (float)bsm->get_estop_state();
    nv->valuetype = TYPE_FLOAT;
    return (STAT_OK);
}

stat_t cm_get_safe(nvObj_t *nv) {

    uint8_t safe = 0;
    if ((bsm) && (bsm->get_interlock_safety_state())) {
        safe |= 0x1;
    }
    if ((bsm) && (bsm->get_esc_safety_state())) {
        safe |= 0x2;
    }
    return (_bt_get_msg_helper(nv, msg_safe, safe));
}
stat_t cm_get_estp(nvObj_t *nv) { return (_bt_get_msg_helper(nv, msg_estp, bsm ? (bsm->get_estop_state() & 0x3) : 0)); }

constexpr cfgItem_t sys_config_items_3[] = {
    {"", "safe", _i0, 0, cm_print_safe, cm_get_safe, set_ro, nullptr, 0},          // interlock status
    {"", "estp", _i0, 0, cm_print_estp, cm_get_estp, cm_ack_estop, nullptr, 0},    // E-stop status (SET to ack)
    {"", "estpc", _i0, 0, cm_print_estp, cm_ack_estop, cm_ack_estop, nullptr, 0},  // E-stop status clear (GET to ack)
};
constexpr cfgSubtableFromStaticArray sys_config_3{sys_config_items_3};
const configSubtable * const getSysConfig_3() { return &sys_config_3; }
