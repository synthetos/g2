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

// local helper functions and macros

/*
 * marlin_verify_checksum() - check to see if we have a line number (cheaply) and a valid checksum
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

    if ((c == '*') && strtol(str, NULL, 10) != checksum) {
        return STAT_CHECKSUM_MATCH_FAILED;
    }
    return STAT_OK;
}

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


/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

stat_t marlin_request_temperature_report() // M105
{
    uint8_t tool = cm.gm.tool_select;
    if (tool > 1) {
        return STAT_INPUT_VALUE_RANGE_ERROR;
    }

    temperature_requested = true;
    return STAT_OK;
}

stat_t marlin_request_position_report() // M114
{
    position_requested = true;
    return STAT_OK;
}

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
}

void _report_position(char *(&str)) {
    //
    // Tool 0 is extruder 1
    uint8_t tool = cm.gm.tool_select;

    str += sprintf(str, " X:%.2f", cm_get_work_position(ACTIVE_MODEL, 0));
    str += sprintf(str, " Y:%.2f", cm_get_work_position(ACTIVE_MODEL, 1));
    str += sprintf(str, " Z:%.2f", cm_get_work_position(ACTIVE_MODEL, 2));

    if (tool == 0) {
        str += sprintf(str, " E:%.2f", cm_get_work_position(ACTIVE_MODEL, 3));
    } else if (tool == 1) {
        str += sprintf(str, " E:%.2f", cm_get_work_position(ACTIVE_MODEL, 4));
    }
}

/*
 * marlin_response() - marlin mirror if text_response()
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

//stat_t gc_get_gc(nvObj_t *nv)
//{
//    ritorno(nv_copy_string(nv, cs.saved_buf));
//    nv->valuetype = TYPE_STRING;
//    return (STAT_OK);
//}
//
//stat_t gc_run_gc(nvObj_t *nv)
//{
//    return(gcode_parser(*nv->stringp));
//}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

// no text mode functions here. Move along

#endif // __TEXT_MODE

#endif // MARLIN_COMPAT_ENABLED == true
