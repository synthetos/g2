/*
 * json_parser.h - JSON parser and JSON support
 * This file is part of the g2core project
 *
 * Copyright (c) 2011 - 2016 Alden S. Hart, Jr.
 * Copyright (c) 2016 Rob Giseburt
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

#ifndef _JSON_PARSER_H_ONCE
#define _JSON_PARSER_H_ONCE

/**** Configs, Definitions and Structures ****/

/* JSON array definitions / revisions */
// for now there is only one JSON array in use - the footer
// if you add these make sure there are no collisions w/present or past numbers

#define FOOTER_REVISION 1

#define JSON_INPUT_STRING_MAX 512   // set an arbitrary max
#define JSON_OUTPUT_STRING_MAX (OUTPUT_BUFFER_LEN)
#define MAX_PAD_CHARS 8             // JSON whitespace padding allowable

typedef enum {
    JV_SILENT = 0,                  // [0] no response is provided for any command
    JV_FOOTER,                      // [1] returns footer only (no command echo, gcode blocks or messages)
    JV_MESSAGES,                    // [2] returns footer, messages (exception and gcode messages)
    JV_CONFIGS,                     // [3] returns footer, messages, config commands
    JV_LINENUM,                     // [4] returns footer, messages, config commands, gcode line numbers if present
    JV_VERBOSE,                     // [5] returns footer, messages, config commands, gcode blocks
    JV_EXCEPTIONS,                  // [6] returns only on messages, configs, and non-zero status
    JV_STATUS,                      // [7] returns status and any messages in abbreviated format
    JV_STATUS_COUNT,                // [8] returns status, count and messages in abbreviated format
    JV_MAX_VALUE
} jsonVerbosity;

typedef enum {                      // json output print modes
    JSON_NO_PRINT = 0,              // don't print anything if you find yourself in JSON mode
    JSON_OBJECT_FORMAT,             // print just the body as a json object
    JSON_RESPONSE_FORMAT,           // print the header/body/footer as a response object
    JSON_RESPONSE_TO_MUTED_FORMAT   // print the header/body/footer as a response object, only to muted channels
} jsonFormats;

typedef struct jsSingleton {

    /*** config values (PUBLIC) ***/
    commMode json_mode;             // 0=text mode, 1=JSON mode (loaded from cs.comm_mode)
    jsonVerbosity json_verbosity;   // see enum in this file for settings
    bool echo_json_footer;          // flags for JSON responses serialization
    bool echo_json_messages;
    bool echo_json_configs;
    bool echo_json_linenum;
    bool echo_json_gcode_block;

    /*** runtime values (PRIVATE) ***/

} jsSingleton_t;

/**** Externs - See report.c for allocation ****/

extern jsSingleton_t js;

/**** Function Prototypes ****/

void json_parser(char *str);
void json_parse_for_exec(char *str, bool execute);
uint16_t json_serialize(nvObj_t *nv, char *out_buf, uint16_t size);
void json_print_object(nvObj_t *nv);
void json_print_response(uint8_t status, const bool only_to_muted = false);
void json_print_list(stat_t status, uint8_t flags);

stat_t json_set_jv(nvObj_t *nv);
stat_t json_set_ej(nvObj_t *nv);

#ifdef __TEXT_MODE

    void js_print_ej(nvObj_t *nv);
    void js_print_jv(nvObj_t *nv);
    void js_print_js(nvObj_t *nv);
    void js_print_jf(nvObj_t *nv);

#else

    #define js_print_ej tx_print_stub
    #define js_print_jv tx_print_stub
    #define js_print_js tx_print_stub
    #define js_print_jf tx_print_stub

#endif // __TEXT_MODE

#endif // End of include guard: JSON_PARSER_H_ONCE
