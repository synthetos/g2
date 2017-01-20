/*
 * marlin_compatibility.cpp - support for marlin protocol and gcode
 * This file is part of the g2core project
 *
 * Copyright (c) 2017 Alden S. Hart, Jr.
 * Copyright (c) 2017 Rob Giseburt
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
#include "g2core.h"  // #1
#include "config.h"  // #2
#include "settings.h"

#if MARLIN_COMPAT_ENABLED == true

#include "controller.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "util.h"
#include "xio.h"            // for char definitions
#include "json_parser.h"
#include "planner.h"
#include "MotateTimers.h"            // for char definitions

// Structures used
enum STK500 {
    // Success
    STATUS_CMD_OK                       = 0x00,

    // Warnings
    STATUS_CMD_TOUT                     = 0x80,
    STATUS_RDY_BSY_TOUT                 = 0x81,
    STATUS_SET_PARAM_MISSING            = 0x82,

    // Errors
    STATUS_CMD_FAILED                   = 0xC0,
    STATUS_CKSUM_ERROR                  = 0xC1,
    STATUS_CMD_UNKNOWN                  = 0xC9,
};

// Local variables

bool temperature_requested = false;
bool position_requested = false;

// State machine to handle marlin temperature controls
enum class MarlinSetTempState {
    Idle = 0,
    SettingTemperature,
    StartingUpdates,
    StartingWait,
    StoppingUpdates,
    SettingTemperatureNoWait
};
MarlinSetTempState set_temp_state; // record the state for the temperature-control pseudo-cycle
// These next parameters for the next temperature-control pseudo-cycle are only needed until the calls are queued.
float next_temperature;            // as it says
uint8_t next_temperature_tool;     // 0-based, with 2 being the heat-bed

// Information about if we are to be dumping periodic temperature updates
bool temperature_updates_requested = false;
Motate::Timeout temperature_update_timeout;

// local helper functions and macros

/*
 * _report_temperatures() - convenience function to get a value via the JSON NV system
 */
nvObj_t *_get_spcific_nv(const char *key) {
    nvObj_t *nv = nv_reset_nv_list();           // returns first object in the body

    strncpy(nv->token, key, TOKEN_LEN);

    // validate and post-process the token
    if ((nv->index = nv_get_index((const char *)"", nv->token)) == NO_MATCH) { // get index or fail it
        // since we JUST provided the keys, this should never happen
        return nullptr;
    }
    strcpy(nv->group, cfgArray[nv->index].group); // capture the group string if there is one

    nv_get(nv);

    return nv;
}

/*
 * _report_temperatures() - convenience function called from marlin_response() and marlin_callback()
 */
void _report_temperatures(char *(&str)) {
    // Tool 0 is extruder 1
    uint8_t tool = cm.gm.tool_select;

    nvObj_t *nv = nullptr;

    if (tool == 0) {
        nv = _get_spcific_nv("he1t");
    } else if (tool == 1) {
        nv = _get_spcific_nv("he2t");
    } else {
        return; // we have no way of reporting errors here, ATM
    }
    if (!nv) { return; }
    str += sprintf(str, " T:%.2f", (float)nv->value);

    if (tool == 0) {
        nv = _get_spcific_nv("he1st");
    } else if (tool == 1) {
        nv = _get_spcific_nv("he2st");
    } else {
        return; // we have no way of reporting errors here, ATM
    }
    if (!nv) { return; }
    str += sprintf(str, " /%.2f", (float)nv->value);

    nv = _get_spcific_nv("he3t");
    if (!nv) { return; }
    str += sprintf(str, " B:%.2f", (float)nv->value);

    nv = _get_spcific_nv("he3st");
    if (!nv) { return; }
    str += sprintf(str, " /%.2f", (float)nv->value);

    if (tool == 0) {
        nv = _get_spcific_nv("he1op");
    } else if (tool == 1) {
        nv = _get_spcific_nv("he2op");
    }    if (!nv) { return; }
    str += sprintf(str, " @:%.0f", (float)nv->value * 255.0);

    nv = _get_spcific_nv("he3op");
    if (!nv) { return; }
    str += sprintf(str, " B@:%.0f", (float)nv->value * 255.0);
}

/*
 * _report_position() - convenience function called from marlin_response()
 */
void _report_position(char *(&str)) {
    str += sprintf(str, " X:%.2f", cm_get_work_position(ACTIVE_MODEL, 0));
    str += sprintf(str, " Y:%.2f", cm_get_work_position(ACTIVE_MODEL, 1));
    str += sprintf(str, " Z:%.2f", cm_get_work_position(ACTIVE_MODEL, 2));

    uint8_t tool = cm.gm.tool_select;
    if (tool == 0) {
        str += sprintf(str, " E:%.2f", cm_get_work_position(ACTIVE_MODEL, 3));
    } else if (tool == 1) {
        str += sprintf(str, " E:%.2f", cm_get_work_position(ACTIVE_MODEL, 4));
    }
}

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/



/*
 * marlin_verify_checksum() - check to see if we have a line number (cheaply) and a valid checksum
 *   called from gcode_parser
 */
stat_t marlin_verify_checksum(char *str)
{
    if (*str != 'N') { return STAT_OK; } // we only check if we have a line number

    char checksum = 0;
    char c = *str++;
    while (c && (c != '*')) {
        checksum ^= c;
        c = *str++;
    }

    // c might be 0 here, in which case we didn't get a checksum and we return STAT_OK

    if (c == '*') {
        *(str-1) = 0; // null terminate, the parser won't like this * here!
        if (strtol(str, NULL, 10) != checksum) {
            return STAT_CHECKSUM_MATCH_FAILED;
        }
    }
    return STAT_OK;
}

/*
 * _marlin_fake_stk500_response() - convenience function for formang responses from marlin_handle_fake_stk500()
 */
void _marlin_fake_stk500_response(char *resp, uint16_t length)
{
    char *str = resp;

    str[2] = (length >> 8) & 0xFF;
    str[3] = (length) & 0xFF;

    uint8_t crc = 0;

    for (uint16_t i = length + 5; i>0; i--) {
        crc ^= *resp++;
    }

    *resp = crc;

    xio_write(str, length + 6);
}

/*
 * marlin_handle_fake_stk500() - returns true if it handled something (IOW, don't futher process the line)
 */
bool marlin_handle_fake_stk500(char *str)
{
    char *resp = str;
    if (*str != 0x1B) { return false; }

    // we handle only a handful of messages ... poorly
    // for example: this is where we should validate the checksum, but we are going to not for now.

    str += 1 + 1 + 2 + 1; // 1 for 0x1B, 1 for sequence, 2 for length, 1 for 0x0E

    char c = *str++;

    if ((c == 0x01) || // CMD_SIGN_ON
        (c == 0x10) || // CMD_ENTER_PROGMODE_ISP
        (c == 0x11)    // CMD_LEAVE_PROGMODE_ISP
        )
    {
        *str = STATUS_CMD_OK;
        _marlin_fake_stk500_response(resp, 2);

        if (c == 0x11) { // CMD_LEAVE_PROGMODE_ISP
            xio_exit_fake_bootloader();
        }

        return true;
    }

    *str = STATUS_CMD_UNKNOWN;
    _marlin_fake_stk500_response(resp, 2);
    return true;
}


/*
 * marlin_request_temperature_report() - called from the gcode parser for M105
 */
stat_t marlin_request_temperature_report() // M105
{
    uint8_t tool = cm.gm.tool_select;
    if ((tool < 0) || (tool > 1)) {
        return STAT_INPUT_VALUE_RANGE_ERROR;
    }

    temperature_requested = true;
    return STAT_OK;
}


/*
 * marlin_request_temperature_report() - called from the gcode parser for M114
 */
stat_t marlin_request_position_report() // M114
{
    position_requested = true;
    return STAT_OK;
}


/*
 * marlin_response() - marlin mirror of text_response(), called from _dispatch_kernel() in controller.cpp
 */
void marlin_response(const stat_t status, char *buf)
{
    char buffer[128];
    char *str = buffer;

    if ((status == STAT_OK) || (status == STAT_EAGAIN) || (status == STAT_NOOP)) {
        strncpy(str, "ok", 2);
        str += 2;

        if (temperature_requested) {
            _report_temperatures(str);
        }

        if (position_requested) {
            _report_position(str);
        }

//        nvObj_t *nv = nv_body+1;
//
//        if (nv_get_type(nv) == NV_TYPE_MESSAGE) {
//            p += sprintf(p, (char *)*nv->stringp);
//        }
//        sprintf(p, "\n");

    } else {
//        str += sprintf(p, "echo:M%d %s", (int)status, get_status_message(status), buf);
        str += sprintf(str, "Error:[%d] %s", (int)status, get_status_message(status));
    }

    *str++ = '\n';
    *str++ = 0;

    // reset requests
    temperature_requested = false;
    position_requested = false;

    xio_writeline(buffer);
}


/*
 * _marlin_start_temperature_updates()/_marlin_end_temperature_updates() -
 *    calls from commands in the buffer to manage temperature_updates_requested
 */
void _marlin_start_temperature_updates(float* vect, bool* flag) {
    temperature_updates_requested = true;
    temperature_update_timeout.set(1); // immediately
}
void _marlin_end_temperature_updates(float* vect, bool* flag) {
    temperature_updates_requested = false;
}


/*
 * _queue_next_temperature_comands() - returns true if it finished
 */
bool _queue_next_temperature_commands()
{
    if (MarlinSetTempState::Idle != set_temp_state) {
        if (mp_planner_is_full()) {
            return false;
        }

        char buffer[128];
        char *str = buffer;

        if ((MarlinSetTempState::SettingTemperature == set_temp_state) ||
            (MarlinSetTempState::SettingTemperatureNoWait == set_temp_state))
        {
            str += sprintf(str, "{he%dst:%.2f}", next_temperature_tool+1, next_temperature);
            cm_json_command(buffer);

            if (MarlinSetTempState::SettingTemperatureNoWait == set_temp_state) {
                set_temp_state = MarlinSetTempState::Idle;
                return true;
            }

            set_temp_state = MarlinSetTempState::StartingUpdates;
            if (mp_planner_is_full()) {
                return false;
            }
        }

        if (MarlinSetTempState::StartingUpdates == set_temp_state) {
            mp_queue_command(_marlin_start_temperature_updates, nullptr, nullptr);

            set_temp_state = MarlinSetTempState::StartingWait;
            if (mp_planner_is_full()) {
                return false;
            }
        }

        if (MarlinSetTempState::StartingWait == set_temp_state) {
            str = buffer;
            str += sprintf(str, "{he%dat:t}", next_temperature_tool+1);
            cm_json_wait(buffer);

            set_temp_state = MarlinSetTempState::StoppingUpdates;
            if (mp_planner_is_full()) {
                return false;
            }
        }

        if (MarlinSetTempState::StoppingUpdates == set_temp_state) {
            mp_queue_command(_marlin_end_temperature_updates, nullptr, nullptr);

            set_temp_state = MarlinSetTempState::Idle;
        }
    }

    return true;
}


/*
 * marlin_callback() - called by controller dispatcher - return STAT_EAGAIN if it failed
 */
stat_t marlin_callback()
{
    if ((js.json_mode == MARLIN_COMM_MODE) && temperature_updates_requested && (temperature_update_timeout.isPast())) {
        char buffer[128];
        char *str = buffer;

        _report_temperatures(str);

        *str++ = '\n';
        *str++ = 0;

        temperature_update_timeout.set(1000); // every second
        
        xio_writeline(buffer);
    } // temperature updates

    if (!_queue_next_temperature_commands()) {
        return STAT_EAGAIN;
    }

    return STAT_OK;
}


/*
 * marlin_set_temperature() - called from the gcode parser for M104,M140,M109,M190
 */
stat_t marlin_set_temperature(uint8_t tool, float temperature, bool wait) {
    if (MarlinSetTempState::Idle != set_temp_state) {
        return (STAT_BUFFER_FULL_FATAL); // we shouldn't be here
    }
    if ((tool < 0) || (tool > 2)) {
        return STAT_INPUT_VALUE_RANGE_ERROR;
    }

    set_temp_state = wait ? MarlinSetTempState::SettingTemperature : MarlinSetTempState::SettingTemperatureNoWait;
    next_temperature = temperature;

    next_temperature_tool = tool;

    _queue_next_temperature_commands(); // we can ignore the return
    return (STAT_OK);
}


#ifdef MARLIN_G29_SCRIPT
auto marlin_g29_file = make_xio_flash_file(MARLIN_G29_SCRIPT);
#endif

/*
 * marlin_start_tramming_bed() - called from the gcode parser for G29
 */
stat_t marlin_start_tramming_bed() {
#ifndef MARLIN_G29_SCRIPT
    return (STAT_G29_NOT_CONFIGURED);
#else
    xio_send_file(marlin_g29_file);
    return (STAT_OK);
#endif
}



/*
 * cm_marlin_set_extruder_mode() - M82, M83 (affects MODEL only)
 *
 *  EXTRUDER_MOVES_ABSOLUTE = 0,    // M82
 *  EXTRUDER_MOVES_RELATIVE,        // M83
 *  EXTRUDER_MOVES_VOLUMETRIC       // Ultimaker2Marlin
 */

stat_t cm_marlin_set_extruder_mode(const uint8_t mode)
{
    cm.gmx.extruder_mode = (cmExtruderMode)mode;
    return (STAT_OK);
}


/*
 * marlin_set_fan_speed() - M106, M107
 *
 */

stat_t marlin_set_fan_speed(const uint8_t fan, float speed)
{
    char buffer[128];
    char *str = buffer;
    if ((fan != 0) || (speed < 0.0) || (speed > 255.0)) {
        return STAT_INPUT_VALUE_RANGE_ERROR;
    }

    // TODO: support other fans, or remapping output
    str += sprintf(str, "{out4:%.2f}", (speed < 1.0) ? speed : (speed / 255.0));
    cm_json_command(buffer);

    return (STAT_OK);
}


/*
 * marlin_disable_motors() - M84
 *
 */

stat_t marlin_disable_motors()
{
    char buffer[128];
    char *str = buffer;

    // TODO: support other parameters
    strncpy(str, "{md:0}", 6);
    cm_json_command(buffer);

    return (STAT_OK);
}
#endif // MARLIN_COMPAT_ENABLED == true
