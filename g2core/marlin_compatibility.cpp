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
#include "xio.h"                // for char definitions
#include "temperature.h"        // for temperature controls
#include "json_parser.h"
#include "planner.h"
#include "stepper.h"            // for MOTOR_TIMEOUT_SECONDS_MIN/MOTOR_TIMEOUT_SECONDS_MAX
#include "MotateTimers.h"       // for char definitions
#include "MotateUniqueID.h"     // for Motate::UUID

// Structures used
enum STK500 {
    // Success
    STATUS_CMD_OK            = 0x00,

    // Warnings
    STATUS_CMD_TOUT          = 0x80,
    STATUS_RDY_BSY_TOUT      = 0x81,
    STATUS_SET_PARAM_MISSING = 0x82,

    // Errors
    STATUS_CMD_FAILED        = 0xC0,
    STATUS_CKSUM_ERROR       = 0xC1,
    STATUS_CMD_UNKNOWN       = 0xC9,
};

// Global state variables

MarlinStateExtended_t mst;       // Marlin state object

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

/***********************************************************************************
 * _get_specific_nv() - convenience function to get an NV object
 */
nvObj_t *_get_specific_nv(const char *key) {
    nvObj_t *nv = nv_reset_nv_list();           // returns first object in the body

    strncpy(nv->token, key, TOKEN_LEN);

    // validate and post-process the token
    if ((nv->index = nv_get_index((const char *)"", nv->token)) == NO_MATCH) { // get index or fail it
        return nullptr;        // since we JUST provided the keys, this should never happen
    }
    strcpy(nv->group, cfgArray[nv->index].group); // capture the group string if there is one
    nv_get(nv);
    return nv;
}

/***********************************************************************************
 * _report_temperatures() - convenience function called from marlin_response() and marlin_callback()
 */
void _report_temperatures(char *(&str)) {
    // Tool 0 is extruder 1
    uint8_t tool = cm.gm.tool;

    str_concat(str, " T:");
    str += floattoa(str, cm_get_temperature(tool), 2);
    str_concat(str, " /");
    str += floattoa(str, cm_get_set_temperature(tool), 2);

    str_concat(str, " B:");
    str += floattoa(str, cm_get_temperature(3), 2);
    str_concat(str, " /");
    str += floattoa(str, cm_get_set_temperature(3), 2);

    str_concat(str, " @:");
    str += floattoa(str, cm_get_heater_output(tool), 0);

    str_concat(str, " B@:");
    str += floattoa(str, cm_get_heater_output(3), 0);
}

/***********************************************************************************
 * _report_position() - convenience function called from marlin_response()
 */
void _report_position(char *(&str)) {
    str_concat(str, " X:");
    str += floattoa(str, cm_get_work_position(ACTIVE_MODEL, 0), 2);
    str_concat(str, " Y:");
    str += floattoa(str, cm_get_work_position(ACTIVE_MODEL, 1), 2);
    str_concat(str, " Z:");
    str += floattoa(str, cm_get_work_position(ACTIVE_MODEL, 2), 2);

    uint8_t tool = cm.gm.tool;
    if ((tool > 0) && (tool < 3)) {
        str_concat(str, " E:");
        str += floattoa(str, cm_get_work_position(ACTIVE_MODEL, 2), tool + 2); // A or B, depending on tool
    }
}

/***********************************************************************************
 *** MARLIN GCODES AND MCODES
 *** Called from gcore_parser.cpp
 ***********************************************************************************/

/***********************************************************************************
 * marlin_start_tramming_bed() - G29 called from gcode parser
 * marlin G29 support - run a script to emulate a G29 homing command
 */

#ifdef MARLIN_G29_SCRIPT
auto marlin_g29_file = make_xio_flash_file(MARLIN_G29_SCRIPT);
#endif

stat_t marlin_start_tramming_bed() {
#ifndef MARLIN_G29_SCRIPT
    return (STAT_G29_NOT_CONFIGURED);
#else
    xio_send_file(marlin_g29_file);
    return (STAT_OK);
#endif
}

/***********************************************************************************
 * marlin_list_sd_response()    - M20 called from gcode parser
 * marlin_select_sd_response()  - M23 called from gcode parser
 */

stat_t marlin_list_sd_response()
{
    char buffer[128];
    char *str = buffer;

    str_concat(str, "Begin file list\nEnd file list\n");
    *str = 0;
    xio_writeline(buffer);

    return (STAT_OK);
}

stat_t marlin_select_sd_response(const char *file)
{
    char buffer[128];
    char *str = buffer;

    str_concat(str, "open failed, File: ");
    strncpy(str, file, Motate::strlen(file));
    str += Motate::strlen(file);
    str_concat(str, "\n");
    *str = 0;
    xio_writeline(buffer);

    return (STAT_OK);
}

/***********************************************************************************
 * cm_marlin_set_extruder_mode() - M82, M83 called from gcode parser (affects MODEL only)
 *
 *  EXTRUDER_MOVES_NORMAL   = 0,    // M82
 *  EXTRUDER_MOVES_RELATIVE,        // M83
 *  EXTRUDER_MOVES_VOLUMETRIC       // Ultimaker2Marlin
 */

stat_t marlin_set_extruder_mode(const uint8_t mode)
{
    mst.extruder_mode = (cmExtruderMode)mode;
    return (STAT_OK);
}

/***********************************************************************************
 * marlin_disable_motors() - M84 (without S) called from gcode parser
 */

stat_t marlin_disable_motors()
{
    char buffer[128];
    char *str = buffer;

    // TODO: support other parameters
    str_concat(str, "{md:0}");
    cm_json_command(buffer);

    return (STAT_OK);
}

/***********************************************************************************
 * marlin_set_motor_timeout() - M84 (with S), M85 Sxxx called from gcode parser
 */

stat_t marlin_set_motor_timeout(float s) // M18 Sxxx, M84 Sxxx, M85 Sxxx
{
    if (s < MOTOR_TIMEOUT_SECONDS_MIN) {
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (s > MOTOR_TIMEOUT_SECONDS_MAX) {
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    char buffer[128];
    char *str = buffer;

    // TODO: support other fans, or remapping output
    str_concat(str, "{mt:");
    str += floattoa(str, s, 1);
    str_concat(str, "}");

    cm_json_command(buffer);
    return (STAT_OK);
}

/***********************************************************************************
 * marlin_set_temperature() - M104, M140, M109, M190 called from gcode parser
 * _queue_next_temperature_comands() - returns true if it finished
 * _marlin_start_temperature_updates()
 * _marlin_end_temperature_updates()
 */

void _marlin_start_temperature_updates(float* vect, bool* flag) {
    temperature_updates_requested = true;
    temperature_update_timeout.set(1); // immediately
}

void _marlin_end_temperature_updates(float* vect, bool* flag) {
    temperature_updates_requested = false;
}

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
            str_concat(str, "{he");
            str += inttoa(str, next_temperature_tool);
            str_concat(str, "st:");
            str += floattoa(str, next_temperature, 2);
            str_concat(str, "}");
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
            str_concat(str, "{he");
            str += inttoa(str, next_temperature_tool);
            str_concat(str, "at:t}");
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

stat_t marlin_set_temperature(uint8_t tool, float temperature, bool wait) {
    if (MarlinSetTempState::Idle != set_temp_state) {
        return (STAT_BUFFER_FULL_FATAL); // we shouldn't be here
    }
    if ((tool < 1) || (tool > 3)) {
        return STAT_INPUT_VALUE_RANGE_ERROR;
    }

    set_temp_state = wait ? MarlinSetTempState::SettingTemperature : MarlinSetTempState::SettingTemperatureNoWait;
    next_temperature = temperature;

    next_temperature_tool = tool;

    _queue_next_temperature_commands(); // we can ignore the return
    return (STAT_OK);
}

/***********************************************************************************
 * marlin_request_temperature_report() - M105 called from gcode parser
 */
stat_t marlin_request_temperature_report() // M105
{
    uint8_t tool = cm.gm.tool;
    if ((tool < 1) || (tool > 2)) {
        return STAT_INPUT_VALUE_RANGE_ERROR;
    }

    temperature_requested = true;
    return STAT_OK;
}

/***********************************************************************************
 * marlin_set_fan_speed() - M106, M107 called from gcode parser
 */

stat_t marlin_set_fan_speed(const uint8_t fan, float speed)
{
    char buffer[128];
    char *str = buffer;
    if ((fan != 0) || (speed < 0.0) || (speed > 255.0)) {
        return STAT_INPUT_VALUE_RANGE_ERROR;
    }

    // TODO: support other fans, or remapping output
    str_concat(str, "{out4:");
    str += floattoa(str, (speed < 1.0) ? speed : (speed / 255.0), 4);
    str_concat(str, "}");

    cm_json_command(buffer);

    return (STAT_OK);
}

/***********************************************************************************
 * marlin_request_position_report() - M114 called from gcode parser
 */
stat_t marlin_request_position_report() // M114
{
    position_requested = true;
    return STAT_OK;
}

/***********************************************************************************
 * marlin_report_version() - M115
 */

stat_t marlin_report_version()
{
    char buffer[128];
    char *str = buffer;

    str_concat(str, "ok FIRMWARE_NAME:Marlin g2core-");
    str_concat(str, G2CORE_FIRMWARE_BUILD_STRING);
    *str = 0;
    xio_writeline(buffer);
    str = buffer;
    *str = 0;

    str_concat(str, " SOURCE_CODE_URL:https://github.com/synthetos/g2");
    *str = 0;
    xio_writeline(buffer);
    str = buffer; *str = 0;

    str_concat(str, " PROTOCOL_VERSION:1.0");
    *str = 0;
    xio_writeline(buffer);
    str = buffer; *str = 0;

    str_concat(str, " MACHINE_TYPE:");
#ifdef SETTINGS_FILE
#define settings_file_string1(s) #s
#define settings_file_string2(s) settings_file_string1(s)
    str_concat(str, settings_file_string2(SETTINGS_FILE));
#undef settings_file_string1
#undef settings_file_string2
#else
    str_concat(str, "<default-settings>");
#endif
    *str = 0;
    xio_writeline(buffer);
    str = buffer; *str = 0;

    // TODO: make this configurable, based on the tool table
    str_concat(str, " EXTRUDER_COUNT:1");
    *str = 0;
    xio_writeline(buffer);
    str = buffer; *str = 0;

    str_concat(str, " UUID:");
    const char *uuid = Motate::UUID;
    strncpy(str, uuid, Motate::strlen(uuid));
    str += Motate::strlen(uuid);
    str_concat(str, "\n");
    *str = 0;
    xio_writeline(buffer);
    str = buffer; *str = 0;

    return (STAT_OK);
}


/***********************************************************************************
 *** MARLIN INTERNAL FUNCTIONS
 ***********************************************************************************/

/***********************************************************************************
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

/***********************************************************************************
 * marlin_response() - marlin mirror of text_response(), called from _dispatch_kernel() in controller.cpp
 */
void marlin_response(const stat_t status, char *buf)
{
    char buffer[128];
    char *str = buffer;

    bool request_resend = false;

    if (cs.responses_suppressed) {
        return;
    }

    if ((status == STAT_OK) || (status == STAT_EAGAIN) || (status == STAT_NOOP)) {
        str_concat(str, "ok");

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

    }
    else if (status == STAT_CHECKSUM_MATCH_FAILED) {
        str_concat(str, "Error:checksum mismatch, Last Line: ");
        str += inttoa(str, cm.gmx.last_line_number);
        request_resend = true;
    }
    else if (status == STAT_LINE_NUMBER_OUT_OF_SEQUENCE) {
        str_concat(str, "Error:Line Number is not Last Line Number+1, Last Line: ");
        str += inttoa(str, cm.gmx.last_line_number);
        request_resend = true;
    }
    else {
        str += sprintf(str, "Error:%s", get_status_message(status));
    }

    *str++ = '\n';
    *str++ = 0;

    // reset requests
    temperature_requested = false;
    position_requested = false;

    xio_writeline(buffer);


    if (request_resend) {
        str = buffer;
        str_concat(str, "Resend: ");
        str += inttoa(str, cm.gmx.last_line_number+1);
        *str++ = '\n';
        *str++ = 0;
        xio_writeline(buffer);
    }
}

/***********************************************************************************
 * marlin_handle_fake_stk500() - returns true if it handled something (IOW, don't futher process the line)
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


#endif // MARLIN_COMPAT_ENABLED == true
