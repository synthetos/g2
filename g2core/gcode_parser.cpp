/*
 * gcode_parser.cpp - rs274/ngc Gcode parser
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2017 Alden S. Hart, Jr.
 * Copyright (c) 2016 - 2017 Rob Giseburt
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
#include "controller.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "settings.h"
#include "spindle.h"
#include "coolant.h"
#include "util.h"
#include "xio.h"                    // for char definitions

#if MARLIN_COMPAT_ENABLED == true
#include "marlin_compatibility.h"
#include "json_parser.h"            // so we can switch js.comm_mode on a marlin M-code
#endif

// Helpers 

static stat_t _execute_gcode_block_marlin(void);

// Locally used enums

typedef enum {                          // Used for detecting gcode errors. See NIST section 3.4
    MODAL_GROUP_G0 = 0,                 // {G10,G28,G28.1,G92}  non-modal axis commands (note 1)
    MODAL_GROUP_G1,                     // {G0,G1,G2,G3,G80}    motion
    MODAL_GROUP_G2,                     // {G17,G18,G19}        plane selection
    MODAL_GROUP_G3,                     // {G90,G91}            distance mode
    MODAL_GROUP_G5,                     // {G93,G94}            feed rate mode
    MODAL_GROUP_G6,                     // {G20,G21}            units
    MODAL_GROUP_G7,                     // {G40,G41,G42}        cutter radius compensation
    MODAL_GROUP_G8,                     // {G43,G49}            tool length offset
    MODAL_GROUP_G9,                     // {G98,G99}            return mode in canned cycles
    MODAL_GROUP_G12,                    // {G54,G55,G56,G57,G58,G59} coordinate system selection
    MODAL_GROUP_G13,                    // {G61,G61.1,G64}      path control mode
    MODAL_GROUP_M4,                     // {M0,M1,M2,M30,M60}   stopping
    MODAL_GROUP_M6,                     // {M6}                 tool change
    MODAL_GROUP_M7,                     // {M3,M4,M5}           spindle turning
    MODAL_GROUP_M8,                     // {M7,M8,M9}           coolant (M7 & M8 may be active together)
    MODAL_GROUP_M9                      // {M48,M49}            speed/feed override switches
} cmModalGroup;
#define MODAL_GROUP_COUNT (MODAL_GROUP_M9+1)
// Note 1: Our G0 omits G4,G30,G53,G92.1,G92.2,G92.3 as these have no axis components to error check

/* The difference between NextAction and MotionMode is that NextAction is
 * used by the current block, and may carry non-modal commands, whereas
 * MotionMode persists across blocks (as G modal group 1)
 */

typedef enum {                                  // these are in order to optimized CASE statement
    NEXT_ACTION_DEFAULT = 0,                    // Must be zero (invokes motion modes)
    NEXT_ACTION_DWELL,                          // G4
    NEXT_ACTION_SET_G10_DATA,                   // G10
    NEXT_ACTION_GOTO_G28_POSITION,              // G28 go to machine position
    NEXT_ACTION_SET_G28_POSITION,               // G28.1 set position in abs coordinates
    NEXT_ACTION_SEARCH_HOME,                    // G28.2 homing cycle
    NEXT_ACTION_SET_ABSOLUTE_ORIGIN,            // G28.3 origin set
    NEXT_ACTION_HOMING_NO_SET,                  // G28.4 homing cycle with no coordinate setting
    NEXT_ACTION_GOTO_G30_POSITION,              // G30 go to machine position
    NEXT_ACTION_SET_G30_POSITION,               // G30.1 set position in abs coordinates
    NEXT_ACTION_STRAIGHT_PROBE_ERR,             // G38.2
    NEXT_ACTION_STRAIGHT_PROBE,                 // G38.3
    NEXT_ACTION_STRAIGHT_PROBE_AWAY_ERR,        // G38.4
    NEXT_ACTION_STRAIGHT_PROBE_AWAY,            // G38.5
    NEXT_ACTION_SET_TL_OFFSET,                  // G43
    NEXT_ACTION_SET_ADDITIONAL_TL_OFFSET,       // G43.2
    NEXT_ACTION_CANCEL_TL_OFFSET,               // G49
    NEXT_ACTION_SET_ORIGIN_OFFSETS,             // G92
    NEXT_ACTION_RESET_ORIGIN_OFFSETS,           // G92.1
    NEXT_ACTION_SUSPEND_ORIGIN_OFFSETS,         // G92.2
    NEXT_ACTION_RESUME_ORIGIN_OFFSETS,          // G92.3
    NEXT_ACTION_JSON_COMMAND_SYNC,              // M100
    NEXT_ACTION_JSON_COMMAND_ASYNC,             // M100.1
    NEXT_ACTION_JSON_WAIT,                      // M101

#if MARLIN_COMPAT_ENABLED == true
    NEXT_ACTION_MARLIN_TRAM_BED,                // G29
    NEXT_ACTION_MARLIN_DISABLE_MOTORS,          // M84
    NEXT_ACTION_MARLIN_SET_MT,                  // M84 (with S), M85
    NEXT_ACTION_MARLIN_SET_EXTRUDER_TEMP,       // M104, M109
    NEXT_ACTION_MARLIN_PRINT_TEMPERATURES,      // M105
    NEXT_ACTION_MARLIN_SET_FAN_SPEED,           // M106
    NEXT_ACTION_MARLIN_STOP_FAN,                // M107
    NEXT_ACTION_MARLIN_CANCEL_WAIT_TEMP,        // M108
    NEXT_ACTION_MARLIN_RESET_LINE_NUMBERS,      // M110
    NEXT_ACTION_MARLIN_DEBUG_STATEMENTS,        // M111 (not used)
    NEXT_ACTION_MARLIN_PRINT_POSITION,          // M114
    NEXT_ACTION_MARLIN_REPORT_VERSION,          // M115
    NEXT_ACTION_MARLIN_DISPLAY_ON_SCREEN,       // M117
    NEXT_ACTION_MARLIN_SET_BED_TEMP,            // M140, M190
#endif

} gpNextAction;

// Structures used by Gcode parser

typedef struct GCodeInputValue {    // Gcode inputs - meaning depends on context

    gpNextAction next_action;       // handles G modal group 1 moves & non-modals
    cmMotionMode motion_mode;       // Group1: G0, G1, G2, G3, G38.2, G80, G81, G82, G83, G84, G85, G86, G87, G88, G89
    uint8_t program_flow;           // used only by the gcode_parser
    uint32_t linenum;               // N word

    float target[AXES];             // XYZABC where the move should go
    float arc_offset[3];            // IJK - used by arc commands
    float arc_radius;               // R - radius value in arc radius mode

    float F_word;                   // F - normalized to millimeters/minute
    uint8_t H_word;                 // H word - used by G43s
    uint8_t L_word;                 // L word - used by G10s
    float P_word;                   // P - parameter used for dwell time in seconds, G10 coord select...
    float S_word;                   // S word - in RPM

    uint8_t feed_rate_mode;         // See cmFeedRateMode for settings
    uint8_t select_plane;           // G17,G18,G19 - values to set plane to
    uint8_t units_mode;             // G20,G21 - 0=inches (G20), 1 = mm (G21)
    uint8_t coord_system;           // G54-G59 - select coordinate system 1-9
    uint8_t path_control;           // G61... EXACT_PATH, EXACT_STOP, CONTINUOUS
    uint8_t distance_mode;          // G91   0=use absolute coords(G90), 1=incremental movement
    uint8_t arc_distance_mode;      // G90.1=use absolute IJK offsets, G91.1=incremental IJK offsets
    uint8_t origin_offset_mode;     // G92...TRUE=in origin offset mode
    uint8_t absolute_override;      // G53 TRUE = move using machine coordinates - this block only (G53)
    uint8_t tool;                   // Tool after T and M6 (tool_select and tool_change)
    uint8_t tool_select;            // T value - T sets this value
    uint8_t tool_change;            // M6 tool change flag - moves "tool_select" to "tool"
    uint8_t mist_coolant;           // TRUE = mist on (M7), FALSE = off (M9)
    uint8_t flood_coolant;          // TRUE = flood on (M8), FALSE = off (M9)
    uint8_t spindle_control;        // 0=OFF (M5), 1=CW (M3), 2=CCW (M4)

    bool m48_enable;                // M48/M49 input (enables for feed and spindle)
    bool mfo_control;               // M50 feedrate override control
    bool mto_control;               // M50.1 traverse override control
    bool sso_control;               // M51 spindle speed override control

#if MARLIN_COMPAT_ENABLED == true
    float E_word;                       // E - "extruder" - may be interpreted any number of ways
    bool marlin_relative_extruder_mode; // M82, M83 (Marlin-only)
#endif

} GCodeValue_t;

typedef struct GCodeFlags {         // Gcode input flags

    bool next_action;
    bool motion_mode;
    bool program_flow;
    bool linenum;

    bool target[AXES];
    bool arc_offset[3];
    bool arc_radius;

    bool F_word;
    bool H_word;
    bool L_word;
    bool P_word;
    bool S_word;

    bool feed_rate_mode;
    bool select_plane;
    bool units_mode;
    bool coord_system;
    bool path_control;
    bool distance_mode;
    bool arc_distance_mode;
    bool origin_offset_mode;
    bool absolute_override;
    bool tool;
    bool tool_select;
    bool tool_change;
    bool mist_coolant;
    bool flood_coolant;
    bool spindle_control;

    bool m48_enable;
    bool mfo_control;
    bool mto_control;
    bool sso_control;

    bool checksum;

#if MARLIN_COMPAT_ENABLED == true
    bool E_word;
    bool marlin_wait_for_temp;      // M140 or M190 - wait for temperature (Marlin-only)
    bool marlin_relative_extruder_mode;
#endif

} GCodeFlag_t;

typedef struct GCodeParser {
    bool modals[MODAL_GROUP_COUNT];
} GCodeParser_t;

GCodeParser_t gp;   // main parser struct
GCodeValue_t gv;    // gcode input values
GCodeFlag_t gf;     // gcode input flags

// local helper functions and macros
void _normalize_gcode_block(char *str, char **active_comment, uint8_t *block_delete_flag);
stat_t _get_next_gcode_word(char **pstr, char *letter, float *value);
stat_t _point(float value);
stat_t _verify_checksum(char *str);
stat_t _validate_gcode_block(char *active_comment);
stat_t _parse_gcode_block(char *line, char *active_comment); // Parse the block into the GN/GF structs
stat_t _execute_gcode_block(char *active_comment);           // Execute the gcode block

#define SET_MODAL(m,parm,val) ({gv.parm=val; gf.parm=true; gp.modals[m]=true; break;})
#define SET_NON_MODAL(parm,val) ({gv.parm=val; gf.parm=true; break;})
#define EXEC_FUNC(f,v) if(gf.v) { status=f(gv.v);}

/*
 * gcode_parser_init()
 */

void gcode_parser_init()
{
    memset(&gv, 0, sizeof(GCodeValue_t));
    memset(&gf, 0, sizeof(GCodeFlag_t));
}

/*
 * gcode_parser() - parse a block (line) of gcode
 *
 *  Top level of gcode parser. Normalizes block and looks for special cases
 */

stat_t gcode_parser(char *block)
{
    char *str = block;                      // gcode command or NUL string
    char none = NUL;
    char *active_comment = &none;           // gcode comment or NUL string
    uint8_t block_delete_flag;

    stat_t check_ret = _verify_checksum(str);
    if (check_ret != STAT_OK) {
        return check_ret;
    }

    _normalize_gcode_block(str, &active_comment, &block_delete_flag);

    // TODO, now MSG is put in the active comment, handle that.

    if (str[0] == NUL) {                    // normalization returned null string
        return (STAT_OK);                   // most likely a comment line
    }

    // Trap M30 and M2 as $clear conditions. This has no effect if not in ALARM or SHUTDOWN
    cm_parse_clear(str);                    // parse Gcode and clear alarms if M30 or M2 is found
    ritorno(cm_is_alarmed());               // return error status if in alarm, shutdown or panic

    // Block delete omits the line if a / char is present in the first space
    // For now this is unconditional and will always delete
//  if ((block_delete_flag == true) && (cm_get_block_delete_switch() == true)) {
    if (block_delete_flag == true) {
        return (STAT_NOOP);
    }
    return(_parse_gcode_block(block, active_comment));
}

/*
 * _verify_checksum() - ensure that, if there is a checksum, that it's valid
 *
 * Returns STAT_OK is it's valid.
 * Returns STAT_CHECKSUM_MATCH_FAILED if the checksum doesn't match.
 */
stat_t _verify_checksum(char *str)
{
    bool has_line_number = false; // -1 means we don't have one
    if (*str == 'N') {
        has_line_number = true;
    }

    char checksum = 0;
    char c = *str++;
    while (c && (c != '*') && (c != '\n') && (c != '\r')) {
        checksum ^= c;
        c = *str++;
    }

    // c might be 0 here, in which case we didn't get a checksum and we return STAT_OK

    if (c == '*') {
        *(str-1) = 0; // null terminate, the parser won't like this * here!
        gf.checksum = true;
        if (strtol(str, NULL, 10) != checksum) {
            _debug_trap("checksum failure");
            return STAT_CHECKSUM_MATCH_FAILED;
        }
        if (!has_line_number) {
            _debug_trap("line number missing with checksum");
            return STAT_MISSING_LINE_NUMBER_WITH_CHECKSUM;
        }
    }
    return STAT_OK;
}

/*
 * _normalize_gcode_block() - normalize a block (line) of gcode in place
 *
 *  Normalization functions:
 *   - Isolate "active comments"
 *   - Many of the following are performed in active comments as well
 *   - Strings are handled special (TODO)
 *   - Active comments are moved to the end of the string, and multiple active comments are merged into one
 *   - convert all letters to upper case
 *   - remove white space, control and other invalid characters
 *   - remove (erroneous) leading zeros that might be taken to mean Octal
 *   - identify and return start of comments and messages
 *   - signal if a block-delete character (/) was encountered in the first space
 *   - NOTE: Assumes no leading whitespace as this was removed at the controller dispatch level
 *
 *  So this: "g1 x100 Y100 f400" becomes this: "G1X100Y100F400"
 *
 *  Comment and message handling:
 *   - Active comments start with exactly "({" and end with "})" (no relaxing, invalid is invalid)
 *   - Comments field start with a '(' char or alternately a semicolon ';'
 *   - Active comments are moved to the end of the string and merged.
 *   - Messages are converted to ({msg:"blah"}) active comments.
 *     - The 'MSG' specifier in comment can have mixed case but cannot cannot have embedded white spaces
 *   - Other "plain" comments will be discarded.
 *   - Multiple embedded comments are acceptable.
 *   - Multiple active comments will be merged.
 *   - Only ONE MSG comment will be accepted.
 *
 *  Returns:
 *   - com points to comment string or to NUL if no comment
 *   - msg points to message string or to NUL if no comment
 *   - block_delete_flag is set true if block delete encountered, false otherwise
 */

char _normalize_scratch[RX_BUFFER_SIZE];

void _normalize_gcode_block(char *str, char **active_comment, uint8_t *block_delete_flag)
{
    _normalize_scratch[0] = 0;

    char *gc_rd = str;                  // read pointer
    char *gc_wr = _normalize_scratch;   // write pointer

    char *ac_rd = str;                  // read pointer
    char *ac_wr = _normalize_scratch;   // Active Comment write pointer

    bool last_char_was_digit = false;   // used for octal stripping

    /* Active comment notes:

     We will convert as follows:
        FROM: G0 ({blah: t}) x10 (comment)
        TO  : g0x10\0{blah:t}
        NOTES: Active comments moved to the end, stripped of (), everything lowercased, and plain comment removed.

        FROM: M100 ({a:t}) (comment) ({b:f}) (comment)
        TO  : m100\0{a:t,b:f}
        NOTES: multiple active comments merged, stripped of (), and actual comments ignored.
      */

    // Move the ac_wr point forward one for every non-AC character we KEEP (plus one for a NULL in between)
    ac_wr++;                            // account for the in-between NULL


    // mark block deletes
    if (*gc_rd == '/') {
        *block_delete_flag = true;
        gc_rd++;
    } else {
        *block_delete_flag = false;
    }

    while (*gc_rd != 0) {
        // check for ';' or '%' comments that end the line.
        if ((*gc_rd == ';') || (*gc_rd == '%')) {
            // go ahead and snap the string off cleanly here
            *gc_rd = 0;
            break;
        }

        // check for comment '('
        else if (*gc_rd == '(') {
            // We only care if it's a "({" in order to handle string-skipping properly
            gc_rd++;
            if ((*gc_rd == '{') || (((* gc_rd    == 'm') || (* gc_rd    == 'M')) &&
                                    ((*(gc_rd+1) == 's') || (*(gc_rd+1) == 'S')) &&
                                    ((*(gc_rd+2) == 'g') || (*(gc_rd+2) == 'G'))
                )) {
                if (ac_rd == nullptr) {
                    ac_rd = gc_rd; // note the start of the first AC
                }

                // skip the comment, handling strings carefully
                bool in_string = false;
                while (*(++gc_rd) != 0) {
                    if (*gc_rd=='"') {
                        in_string = true;
                    } else if (in_string) {
                        if ((*gc_rd == '\\') && (*(gc_rd+1) != 0)) {
                            gc_rd++; // Skip it, it's escaped.
                        }

                    } else if ((*gc_rd == ')')) {
                        break;
                    }
                }
                if (*gc_rd == 0) {      // We don't want the rd++ later to skip the NULL if we're at one
                    break;
                }
            } else {
                *(gc_rd-1) = ' ';       // Change the '(' to a space to simplify the comment copy later

                // skip ahead until we find a ')' (or NULL)
                while ((*gc_rd != 0) && (*gc_rd != ')')) {
                    gc_rd++;
                }
            }
        } else if (!isspace(*gc_rd)) {
            bool do_copy = false;

            // Perform Octal stripping - remove invalid leading zeros in number strings
            // Change 0123.004 to 123.004, or -0234.003 to -234.003
            if (isdigit(*gc_rd) || (*gc_rd == '.')) { // treat '.' as a digit so we don't strip after one
                if (last_char_was_digit || (*gc_rd != '0') || !isdigit(*(gc_rd+1))) {
                    do_copy = true;
                }
                last_char_was_digit = true;
            }
            else if ((isalnum((char)*gc_rd)) || (strchr("-.", *gc_rd))) { // all valid characters
                last_char_was_digit = false;
                do_copy = true;
            }

            if (do_copy) {
                *(gc_wr++) = toupper(*gc_rd);
                ac_wr++; // move the ac start position
            }
        }

        gc_rd++;
    }

    // Enforce null termination
    *gc_wr = 0;

    // note the beginning of the comments
    char *comment_start = ac_wr;

    if (ac_rd != nullptr) {

        // Now we'll copy the comments to the scratch
        while (*ac_rd != 0) {
            // check for comment '('
            // Remember: we're only "counting characters" at this point, no more.
            if (*ac_rd == '(') {
                // We only care if it's a "({" in order to handle string-skipping properly
                ac_rd++;

                bool do_copy = false;
                bool in_msg = false;
                if (((* ac_rd    == 'm') || (* ac_rd    == 'M')) &&
                    ((*(ac_rd+1) == 's') || (*(ac_rd+1) == 'S')) &&
                    ((*(ac_rd+2) == 'g') || (*(ac_rd+2) == 'G'))
                    ) {

                    ac_rd += 3;
                    if (*ac_rd == ' ') {
                        ac_rd++; // skip the first space.
                    }

                    if (*(ac_wr-1) == '}') {
                        *(ac_wr-1) = ',';
                    } else {
                        *(ac_wr++) = '{';
                    }
                    *(ac_wr++) = 'm';
                    *(ac_wr++) = 's';
                    *(ac_wr++) = 'g';
                    *(ac_wr++) = ':';
                    *(ac_wr++) = '"';

                    // TODO - FIX BUFFER OVERFLOW POTENTIAL
                    // "(msg)" is four characters. "{msg:" is five. If the write buffer is full, we'll overflow.
                    // Also " is MSG will be quoted, making one character into two.

                    in_msg = true;
                    do_copy = true;
                }

                else if (*ac_rd == '{') {
                    // merge json comments
                    if (*(ac_wr-1) == '}') {
                        *(ac_wr-1) = ',';

                        // don't copy the '{'
                        ac_rd++;
                    }

                    do_copy = true;
                }

                if (do_copy) {
                    // skip the comment, handling strings carefully
                    bool in_string = false;
                    bool escaped = false;
                    while (*ac_rd != 0) {
                        if (in_string && (*ac_rd == '\\')) {
                            escaped = true;
                        } else if (!escaped && (*ac_rd == '"')) {
                            // In msg comments, we have to escape "
                            if (in_msg) {
                                *(ac_wr++) = '\\';
                            } else {
                                in_string = !in_string;
                            }
                        } else if (!in_string && (*ac_rd == ')')) {
                            ac_rd++;
                            if (in_msg) {
                                *(ac_wr++) = '"';
                                *(ac_wr++) = '}';
                            }
                            break;
                        } else {
                            escaped = false;
                        }

                        // Skip spaces if we're not in a string or msg (implicit string)
                        if (in_string || in_msg || (*ac_rd != ' ')) {
                            *ac_wr = *ac_rd;
                            ac_wr++;
                        }

                        ac_rd++;
                    }
                }

                // We don't want the rd++ later to skip the NULL if we're at one
                if (*ac_rd == 0) {
                    break;
                }
            }

            ac_rd++;
        }
    }

    // Enforce null termination
    *ac_wr = 0;

    // Now copy it all back
    memcpy(str, _normalize_scratch, (ac_wr-_normalize_scratch)+1);

    *active_comment = str + (comment_start - _normalize_scratch);
}


/*
 * _get_next_gcode_word() - get gcode word consisting of a letter and a value
 *
 *  This function requires the Gcode string to be normalized.
 *  Normalization must remove any leading zeros or they will be converted to Octal
 *  G0X... is not interpreted as hexadecimal. This is trapped.
 */

stat_t _get_next_gcode_word(char **pstr, char *letter, float *value)
{
    if (**pstr == NUL) { return (STAT_COMPLETE); }    // no more words

    // get letter part
    if(isupper(**pstr) == false) {
        return (STAT_INVALID_OR_MALFORMED_COMMAND);
    }
    *letter = **pstr;
    (*pstr)++;

//    // X-axis-becomes-a-hexadecimal-number get-value case, e.g. G0X100 --> G255
//    if ((**pstr == '0') && (*(*pstr+1) == 'X')) {
//        *value = 0;
//        (*pstr)++;
//        return (STAT_OK);        // pointer points to X
//    }
//
//    // get-value general case
//    char *end = *pstr;
//    *value = strtof(*pstr, &end);

    // get-value general case
    char *end = *pstr;
    *value = c_atof(end);

    if(end == *pstr) {
#if MARLIN_COMPAT_ENABLED == true
        if (mst.marlin_flavor) {
            *value = 0;
        } else {
            return(STAT_BAD_NUMBER_FORMAT);
        }
#else
        return(STAT_BAD_NUMBER_FORMAT);
#endif
    }    // more robust test then checking for value=0;
    *pstr = end;
    return (STAT_OK);            // pointer points to next character after the word
}

/*
 * _point() - isolate the decimal point value as an integer
 */

uint8_t _point(const float value)
{
    return((uint8_t)(value*10 - trunc(value)*10));    // isolate the decimal point as an int
}

/*
 * _validate_gcode_block() - check for some gross Gcode block semantic violations
 */

stat_t _validate_gcode_block(char *active_comment)
{
//  Check for modal group violations. From NIST, section 3.4 "It is an error to put
//  a G-code from group 1 and a G-code from group 0 on the same line if both of them
//  use axis words. If an axis word-using G-code from group 1 is implicitly in effect
//  on a line (by having been activated on an earlier line), and a group 0 G-code that
//  uses axis words appears on the line, the activity of the group 1 G-code is suspended
//  for that line. The axis word-using G-codes from group 0 are G10, G28, G30, and G92"

//  if ((cm.gn.modals[MODAL_GROUP_G0] == true) && (cm.gn.modals[MODAL_GROUP_G1] == true)) {
//     return (STAT_MODAL_GROUP_VIOLATION);
//  }

// look for commands that require an axis word to be present
//  if ((cm.gn.modals[MODAL_GROUP_G0] == true) || (cm.gn.modals[MODAL_GROUP_G1] == true)) {
//     if (_axis_changed() == false)
//     return (STAT_GCODE_AXIS_IS_MISSING);
//  }
    return (STAT_OK);
}

/*
 * _parse_gcode_block() - parses one line of NULL terminated G-Code.
 *
 *  All the parser does is load the state values in gn (next model state) and set flags
 *  in gf (model state flags). The execute routine applies them. The buffer is assumed to
 *  contain only uppercase characters and signed floats (no whitespace).
 */

stat_t _parse_gcode_block(char *buf, char *active_comment)
{
    char *pstr = (char *)buf;                   // persistent pointer into gcode block for parsing words
    char letter;                                // parsed letter, eg.g. G or X or Y
    float value = 0;                            // value parsed from letter (e.g. 2 for G2)
    stat_t status = STAT_OK;

    // set initial state for new move
    memset(&gv, 0, sizeof(GCodeValue_t));       // clear all next-state values
    memset(&gf, 0, sizeof(GCodeFlag_t));        // clear all next-state flags
    gv.motion_mode = cm_get_motion_mode(MODEL); // get motion mode from previous block

    // Causes a later exception if
    //  (1) INVERSE_TIME_MODE is active and a feed rate is not provided or
    //  (2) INVERSE_TIME_MODE is changed to UNITS_PER_MINUTE and a new feed rate is missing
    if (cm.gm.feed_rate_mode == INVERSE_TIME_MODE) {// new feed rate req'd when in INV_TIME_MODE
        gv.F_word = 0;
        gf.F_word = true;
    }

    // extract commands and parameters
    while((status = _get_next_gcode_word(&pstr, &letter, &value)) == STAT_OK) {
        switch(letter) {
            case 'G':
            switch((uint8_t)value) {
                case 0:  SET_MODAL (MODAL_GROUP_G1, motion_mode, MOTION_MODE_STRAIGHT_TRAVERSE);
                case 1:  SET_MODAL (MODAL_GROUP_G1, motion_mode, MOTION_MODE_STRAIGHT_FEED);
                case 2:  SET_MODAL (MODAL_GROUP_G1, motion_mode, MOTION_MODE_CW_ARC);
                case 3:  SET_MODAL (MODAL_GROUP_G1, motion_mode, MOTION_MODE_CCW_ARC);
                case 4:  SET_NON_MODAL (next_action, NEXT_ACTION_DWELL);
                case 10: SET_MODAL (MODAL_GROUP_G0, next_action, NEXT_ACTION_SET_G10_DATA);
                case 17: SET_MODAL (MODAL_GROUP_G2, select_plane, CANON_PLANE_XY);
                case 18: SET_MODAL (MODAL_GROUP_G2, select_plane, CANON_PLANE_XZ);
                case 19: SET_MODAL (MODAL_GROUP_G2, select_plane, CANON_PLANE_YZ);
                case 20: SET_MODAL (MODAL_GROUP_G6, units_mode, INCHES);
                case 21: SET_MODAL (MODAL_GROUP_G6, units_mode, MILLIMETERS);
                case 28: {
                    switch (_point(value)) {
                        case 0: SET_MODAL (MODAL_GROUP_G0, next_action, NEXT_ACTION_GOTO_G28_POSITION);
                        case 1: SET_MODAL (MODAL_GROUP_G0, next_action, NEXT_ACTION_SET_G28_POSITION);
                        case 2: SET_NON_MODAL (next_action, NEXT_ACTION_SEARCH_HOME);
                        case 3: SET_NON_MODAL (next_action, NEXT_ACTION_SET_ABSOLUTE_ORIGIN);
                        case 4: SET_NON_MODAL (next_action, NEXT_ACTION_HOMING_NO_SET);
                        default: status = STAT_GCODE_COMMAND_UNSUPPORTED;
                    }
                    break;
                }
#if MARLIN_COMPAT_ENABLED == true
                case 29: SET_NON_MODAL (next_action, NEXT_ACTION_MARLIN_TRAM_BED);
#endif

                case 30: {
                    switch (_point(value)) {
                        case 0: SET_MODAL (MODAL_GROUP_G0, next_action, NEXT_ACTION_GOTO_G30_POSITION);
                        case 1: SET_MODAL (MODAL_GROUP_G0, next_action, NEXT_ACTION_SET_G30_POSITION);
                        default: status = STAT_GCODE_COMMAND_UNSUPPORTED;
                    }
                    break;
                }
                case 38: {
                    switch (_point(value)) {
                        case 2: SET_NON_MODAL (next_action, NEXT_ACTION_STRAIGHT_PROBE_ERR);
                        case 3: SET_NON_MODAL (next_action, NEXT_ACTION_STRAIGHT_PROBE);
                        case 4: SET_NON_MODAL (next_action, NEXT_ACTION_STRAIGHT_PROBE_AWAY_ERR);
                        case 5: SET_NON_MODAL (next_action, NEXT_ACTION_STRAIGHT_PROBE_AWAY);
                        default: status = STAT_GCODE_COMMAND_UNSUPPORTED;
                    }
                    break;
                }
                case 40: break;    // ignore cancel cutter radius compensation. But don't fail G40s.
                case 43: {
                    switch (_point(value)) {
                        case 0: SET_NON_MODAL (next_action, NEXT_ACTION_SET_TL_OFFSET);
                        case 2: SET_NON_MODAL (next_action, NEXT_ACTION_SET_ADDITIONAL_TL_OFFSET);
                        default: status = STAT_GCODE_COMMAND_UNSUPPORTED;
                    }
                    break;
                }
				case 49: SET_NON_MODAL (next_action, NEXT_ACTION_CANCEL_TL_OFFSET);
                case 53: SET_NON_MODAL (absolute_override, true);
                case 54: SET_MODAL (MODAL_GROUP_G12, coord_system, G54);
                case 55: SET_MODAL (MODAL_GROUP_G12, coord_system, G55);
                case 56: SET_MODAL (MODAL_GROUP_G12, coord_system, G56);
                case 57: SET_MODAL (MODAL_GROUP_G12, coord_system, G57);
                case 58: SET_MODAL (MODAL_GROUP_G12, coord_system, G58);
                case 59: SET_MODAL (MODAL_GROUP_G12, coord_system, G59);
                case 61: {
                    switch (_point(value)) {
                        case 0: SET_MODAL (MODAL_GROUP_G13, path_control, PATH_EXACT_PATH);
                        case 1: SET_MODAL (MODAL_GROUP_G13, path_control, PATH_EXACT_STOP);
                        default: status = STAT_GCODE_COMMAND_UNSUPPORTED;
                    }
                    break;
                }
                case 64: SET_MODAL (MODAL_GROUP_G13,path_control, PATH_CONTINUOUS);
                case 80: SET_MODAL (MODAL_GROUP_G1, motion_mode,  MOTION_MODE_CANCEL_MOTION_MODE);
                case 90: {
                    switch (_point(value)) {
                        case 0: SET_MODAL (MODAL_GROUP_G3, distance_mode, ABSOLUTE_DISTANCE_MODE);
                        case 1: SET_MODAL (MODAL_GROUP_G3, arc_distance_mode, ABSOLUTE_DISTANCE_MODE);
                        default: status = STAT_GCODE_COMMAND_UNSUPPORTED;
                    }
                    break;
                }
                case 91: {
                    switch (_point(value)) {
                        case 0: SET_MODAL (MODAL_GROUP_G3, distance_mode, INCREMENTAL_DISTANCE_MODE);
                        case 1: SET_MODAL (MODAL_GROUP_G3, arc_distance_mode, INCREMENTAL_DISTANCE_MODE);
                        default: status = STAT_GCODE_COMMAND_UNSUPPORTED;
                    }
                    break;
                }
                case 92: {
                    switch (_point(value)) {
                        case 0: SET_MODAL (MODAL_GROUP_G0, next_action, NEXT_ACTION_SET_ORIGIN_OFFSETS);
                        case 1: SET_NON_MODAL (next_action, NEXT_ACTION_RESET_ORIGIN_OFFSETS);
                        case 2: SET_NON_MODAL (next_action, NEXT_ACTION_SUSPEND_ORIGIN_OFFSETS);
                        case 3: SET_NON_MODAL (next_action, NEXT_ACTION_RESUME_ORIGIN_OFFSETS);
                        default: status = STAT_GCODE_COMMAND_UNSUPPORTED;
                    }
                    break;
                }
                case 93: SET_MODAL (MODAL_GROUP_G5, feed_rate_mode, INVERSE_TIME_MODE);
                case 94: SET_MODAL (MODAL_GROUP_G5, feed_rate_mode, UNITS_PER_MINUTE_MODE);
//              case 95: SET_MODAL (MODAL_GROUP_G5, feed_rate_mode, UNITS_PER_REVOLUTION_MODE);

                default: status = STAT_GCODE_COMMAND_UNSUPPORTED;
            }
            break;

            case 'M':
            switch((uint8_t)value) {
                case 0: case 1: case 60:
                        SET_MODAL (MODAL_GROUP_M4, program_flow, PROGRAM_STOP);
                case 2: case 30:
                        SET_MODAL (MODAL_GROUP_M4, program_flow, PROGRAM_END);
                case 3: SET_MODAL (MODAL_GROUP_M7, spindle_control, SPINDLE_CONTROL_CW);
                case 4: SET_MODAL (MODAL_GROUP_M7, spindle_control, SPINDLE_CONTROL_CCW);
                case 5: SET_MODAL (MODAL_GROUP_M7, spindle_control, SPINDLE_CONTROL_OFF);
                case 6: SET_NON_MODAL (tool_change, true);
                case 7: SET_MODAL (MODAL_GROUP_M8, mist_coolant, true);
                case 8: SET_MODAL (MODAL_GROUP_M8, flood_coolant, true);
                case 9: SET_MODAL (MODAL_GROUP_M8, flood_coolant, false);
                case 48: SET_MODAL (MODAL_GROUP_M9, m48_enable, true);
                case 49: SET_MODAL (MODAL_GROUP_M9, m48_enable, false);
                case 50: SET_MODAL (MODAL_GROUP_M9, mfo_control, true);
                    switch (_point(value)) {
                        case 0: SET_MODAL (MODAL_GROUP_M9, mfo_control, true);
                        case 1: SET_MODAL (MODAL_GROUP_M9, mto_control, true);
                        default: status = STAT_GCODE_COMMAND_UNSUPPORTED;
                    }
                    break;
                case 51: SET_MODAL (MODAL_GROUP_M9, sso_control, true);
                case 100:
                    switch (_point(value)) {
                        case 0: SET_NON_MODAL (next_action, NEXT_ACTION_JSON_COMMAND_SYNC);
                        case 1: SET_NON_MODAL (next_action, NEXT_ACTION_JSON_COMMAND_ASYNC);
                        default: status = STAT_GCODE_COMMAND_UNSUPPORTED;
                    }
                    break;
                case 101: SET_NON_MODAL (next_action, NEXT_ACTION_JSON_WAIT);

#if MARLIN_COMPAT_ENABLED == true
                case 20:marlin_list_sd_response();        status = STAT_COMPLETE; break;    // List SD card
                case 21:                                                                    // Initialize SD card
                case 22:                                  status = STAT_COMPLETE; break;    // Release SD card
                case 23: marlin_select_sd_response(pstr); status = STAT_COMPLETE; break;    // Select SD file

                case 82: SET_NON_MODAL (marlin_relative_extruder_mode, false);              // set relative extruder mode off
                case 83: SET_NON_MODAL (marlin_relative_extruder_mode, true);               // set relative extruder mode on

                case 18:                                                                    // compatibility alias for M84
                case 84: SET_NON_MODAL (next_action, NEXT_ACTION_MARLIN_DISABLE_MOTORS);    // disable all motors
                case 85: SET_NON_MODAL (next_action, NEXT_ACTION_MARLIN_SET_MT);            // set motor timeout

                case 105: SET_NON_MODAL (next_action, NEXT_ACTION_MARLIN_PRINT_TEMPERATURES);// request temperature report
                case 106: SET_NON_MODAL (next_action, NEXT_ACTION_MARLIN_SET_FAN_SPEED);    // set fan speed range 0 - 255
                case 107: SET_NON_MODAL (next_action, NEXT_ACTION_MARLIN_STOP_FAN);         // stop fan (speed = 0)
                case 108: SET_NON_MODAL (next_action, NEXT_ACTION_MARLIN_CANCEL_WAIT_TEMP); // cancel wait for temparature
                case 114: SET_NON_MODAL (next_action, NEXT_ACTION_MARLIN_PRINT_POSITION);   // request position report

                case 109:                gf.marlin_wait_for_temp = true; // NO break!       // set wait for temp and execute M104
                case 104: SET_NON_MODAL (next_action, NEXT_ACTION_MARLIN_SET_EXTRUDER_TEMP);// set extruder temperature

                case 190:                gf.marlin_wait_for_temp = true; // NO break!       // set wait for temp and execute M140
                case 140: SET_NON_MODAL (next_action, NEXT_ACTION_MARLIN_SET_BED_TEMP);     // set heated bed temperature

                case 110: SET_NON_MODAL (next_action, NEXT_ACTION_MARLIN_RESET_LINE_NUMBERS);// reset line numbers
                case 111: status = STAT_COMPLETE; break; // ignore M111 Marlin debug statements. Don't process contents of the line further

                case 115: SET_NON_MODAL (next_action, NEXT_ACTION_MARLIN_REPORT_VERSION);   // report version information
                case 117: status = STAT_COMPLETE; break;  //SET_NON_MODAL (next_action, NEXT_ACTION_MARLIN_DISPLAY_ON_SCREEN);
#endif // MARLIN_COMPAT_ENABLED

                default: status = STAT_MCODE_COMMAND_UNSUPPORTED;
            }
            break;

            case 'T': SET_NON_MODAL (tool_select, (uint8_t)trunc(value));
            case 'F': SET_NON_MODAL (F_word, value);
            case 'P': SET_NON_MODAL (P_word, value);                // used for dwell time, G10 coord select
            case 'S': SET_NON_MODAL (S_word, value);
            case 'X': SET_NON_MODAL (target[AXIS_X], value);
            case 'Y': SET_NON_MODAL (target[AXIS_Y], value);
            case 'Z': SET_NON_MODAL (target[AXIS_Z], value);
            case 'A': SET_NON_MODAL (target[AXIS_A], value);
            case 'B': SET_NON_MODAL (target[AXIS_B], value);
            case 'C': SET_NON_MODAL (target[AXIS_C], value);
        //  case 'U': SET_NON_MODAL (target[AXIS_U], value);        // reserved
        //  case 'V': SET_NON_MODAL (target[AXIS_V], value);        // reserved
        //  case 'W': SET_NON_MODAL (target[AXIS_W], value);        // reserved
            case 'H': SET_NON_MODAL (H_word, value);
            case 'I': SET_NON_MODAL (arc_offset[0], value);
            case 'J': SET_NON_MODAL (arc_offset[1], value);
            case 'K': SET_NON_MODAL (arc_offset[2], value);
            case 'L': SET_NON_MODAL (L_word, value);
            case 'R': SET_NON_MODAL (arc_radius, value);
            case 'N': SET_NON_MODAL (linenum,(uint32_t)value);      // line number
            
#if MARLIN_COMPAT_ENABLED == true
            case 'E': SET_NON_MODAL (E_word, value);                // extruder value
#endif

            default: status = STAT_GCODE_COMMAND_UNSUPPORTED;
        }
        if(status != STAT_OK) break;
    }
    if ((status != STAT_OK) && (status != STAT_COMPLETE)) return (status);
    ritorno(_validate_gcode_block(active_comment));
    return (_execute_gcode_block(active_comment));        // if successful execute the block
}

/*
 * _execute_gcode_block() - execute parsed block
 *
 *  Conditionally (based on whether a flag is set in gf) call the canonical
 *  machining functions in order of execution as per RS274NGC_3 table 8
 *  (below, with modifications):
 *
 *    0. record the line number
 *    1. comment (includes message) [handled during block normalization]
 *    2. set feed rate mode (G93, G94 - inverse time or per minute)
 *    3. set feed rate (F)
 *    3a. Marlin functions (optional)
 *    3b. set feed override rate (M50.1)
 *    3c. set traverse override rate (M50.2)
 *    4. set spindle speed (S)
 *    4a. set spindle override rate (M51.1)
 *    5. select tool (T)
 *    6. change tool (M6)
 *    7. spindle on or off (M3, M4, M5)
 *    8. coolant on or off (M7, M8, M9)
 *    9. enable or disable overrides (M48, M49, M50, M51)
 *    10. dwell (G4)
 *    11. set active plane (G17, G18, G19)
 *    12. set length units (G20, G21)
 *    13. cutter radius compensation on or off (G40, G41, G42)
 *    14. cutter length compensation on or off (G43, G49)
 *    15. coordinate system selection (G54, G55, G56, G57, G58, G59)
 *    16. set path control mode (G61, G61.1, G64)
 *    17. set distance mode (G90, G91)
 *    17a. set arc distance mode (G90.1, G91.1)
 *    18.  set retract mode (G98, G99)
 *    19a. homing functions (G28.2, G28.3, G28.1, G28, G30)
 *    19b. update system data (G10)
 *    19c. set axis offsets (G92, G92.1, G92.2, G92.3)
 *    20. perform motion (G0 to G3, G80-G89) as modified (possibly) by G53
 *    21. stop and end (M0, M1, M2, M30, M60)
 *
 *  Values in gn are in original units and should not be unit converted prior
 *  to calling the canonical functions (which do the unit conversions)
 */

stat_t _execute_gcode_block(char *active_comment)
{
    stat_t status = STAT_OK;

    if (gf.linenum) {
        cm_set_model_linenum(gv.linenum);
    }
    EXEC_FUNC(cm_set_feed_rate_mode, feed_rate_mode);       // G93, G94
    EXEC_FUNC(cm_set_feed_rate, F_word);                    // F

    ritorno(_execute_gcode_block_marlin());                 // execute Marlin commands if Marlin compatibility enabled
    if (gf.linenum && gf.checksum) {
        ritorno(cm_check_linenum());
    }

    EXEC_FUNC(cm_set_spindle_speed, S_word);                // S
    if (gf.sso_control) {                                   // spindle speed override
        ritorno(cm_sso_control(gv.P_word, gf.P_word));
    }

    EXEC_FUNC(cm_select_tool, tool_select);                 // tool_select is where it's written
    EXEC_FUNC(cm_change_tool, tool_change);                 // M6
    EXEC_FUNC(cm_spindle_control, spindle_control);         // spindle CW, CCW, OFF

    EXEC_FUNC(cm_mist_coolant_control, mist_coolant);       // M7, M9
    EXEC_FUNC(cm_flood_coolant_control, flood_coolant);     // M8, M9 also disables mist coolant if OFF
    EXEC_FUNC(cm_m48_enable, m48_enable);

    if (gf.mfo_control) {                                   // manual feedrate override
        ritorno(cm_mfo_control(gv.P_word, gf.P_word));
    }
    if (gf.mto_control) {                                   // manual traverse override
        ritorno(cm_mto_control(gv.P_word, gf.P_word));
    }

    if (gv.next_action == NEXT_ACTION_DWELL) {              // G4 - dwell
        ritorno(cm_dwell(gv.P_word));                       // return if error, otherwise complete the block
    }
    EXEC_FUNC(cm_select_plane, select_plane);               // G17, G18, G19
    EXEC_FUNC(cm_set_units_mode, units_mode);               // G20, G21
    //--> cutter radius compensation goes here

    switch (gv.next_action) {                               // Tool length offsets
        case NEXT_ACTION_SET_TL_OFFSET: {                   // G43
            ritorno(cm_set_tl_offset(gv.H_word, gf.H_word, false));
            break;
        }
        case NEXT_ACTION_SET_ADDITIONAL_TL_OFFSET: {        // G43.2
            ritorno(cm_set_tl_offset(gv.H_word, gf.H_word, true));
            break;
        }
        case NEXT_ACTION_CANCEL_TL_OFFSET: {                // G49
            ritorno(cm_cancel_tl_offset());
            break;
        }
        default:
            // quiet the compiler warning about all the things we don't handle
            break;
    }

    EXEC_FUNC(cm_set_coord_system, coord_system);           // G54, G55, G56, G57, G58, G59

    if (gf.path_control) {                                  // G61, G61.1, G64
        status = cm_set_path_control(MODEL, gv.path_control);
    }

    EXEC_FUNC(cm_set_distance_mode, distance_mode);         // G90, G91
    EXEC_FUNC(cm_set_arc_distance_mode, arc_distance_mode); // G90.1, G91.1
    //--> set retract mode goes here

    switch (gv.next_action) {
        case NEXT_ACTION_SET_G28_POSITION:  { status = cm_set_g28_position(); break;}                               // G28.1
        case NEXT_ACTION_GOTO_G28_POSITION: { status = cm_goto_g28_position(gv.target, gf.target); break;}          // G28
        case NEXT_ACTION_SET_G30_POSITION:  { status = cm_set_g30_position(); break;}                               // G30.1
        case NEXT_ACTION_GOTO_G30_POSITION: { status = cm_goto_g30_position(gv.target, gf.target); break;}          // G30

        case NEXT_ACTION_SEARCH_HOME:         { status = cm_homing_cycle_start(gv.target, gf.target); break;}       // G28.2
        case NEXT_ACTION_SET_ABSOLUTE_ORIGIN: { status = cm_set_absolute_origin(gv.target, gf.target); break;}      // G28.3
        case NEXT_ACTION_HOMING_NO_SET:       { status = cm_homing_cycle_start_no_set(gv.target, gf.target); break;}// G28.4

        case NEXT_ACTION_STRAIGHT_PROBE_ERR:     { status = cm_straight_probe(gv.target, gf.target, true, true); break;}  // G38.2
        case NEXT_ACTION_STRAIGHT_PROBE:         { status = cm_straight_probe(gv.target, gf.target, true, false); break;} // G38.3
        case NEXT_ACTION_STRAIGHT_PROBE_AWAY_ERR:{ status = cm_straight_probe(gv.target, gf.target, false, true); break;} // G38.4
        case NEXT_ACTION_STRAIGHT_PROBE_AWAY:    { status = cm_straight_probe(gv.target, gf.target, false, false); break;}// G38.5

        case NEXT_ACTION_SET_G10_DATA:           { status = cm_set_g10_data(gv.P_word, gf.P_word, gv.L_word, gf.L_word, gv.target, gf.target); break;} // G10
        case NEXT_ACTION_SET_ORIGIN_OFFSETS:     { status = cm_set_origin_offsets(gv.target, gf.target); break;}    // G92
        case NEXT_ACTION_RESET_ORIGIN_OFFSETS:   { status = cm_reset_origin_offsets(); break;}                      // G92.1
        case NEXT_ACTION_SUSPEND_ORIGIN_OFFSETS: { status = cm_suspend_origin_offsets(); break;}                    // G92.2
        case NEXT_ACTION_RESUME_ORIGIN_OFFSETS:  { status = cm_resume_origin_offsets(); break;}                     // G92.3

        case NEXT_ACTION_JSON_COMMAND_SYNC:       { status = cm_json_command(active_comment); break;}               // M100.0
        case NEXT_ACTION_JSON_COMMAND_ASYNC:      { status = cm_json_command_immediate(active_comment); break;}     // M100.1
        case NEXT_ACTION_JSON_WAIT:               { status = cm_json_wait(active_comment); break;}                  // M101

        case NEXT_ACTION_DEFAULT: {
            cm_set_absolute_override(MODEL, gv.absolute_override);    // apply absolute override
            switch (gv.motion_mode) {
                case MOTION_MODE_CANCEL_MOTION_MODE: { cm.gm.motion_mode = gv.motion_mode; break;}                  // G80
                case MOTION_MODE_STRAIGHT_TRAVERSE:  { status = cm_straight_traverse(gv.target, gf.target); break;} // G0
                case MOTION_MODE_STRAIGHT_FEED:      { status = cm_straight_feed(gv.target, gf.target); break;}     // G1
                case MOTION_MODE_CW_ARC:                                                                            // G2
                case MOTION_MODE_CCW_ARC: { status = cm_arc_feed(gv.target,     gf.target,                          // G3
                                                                 gv.arc_offset, gf.arc_offset,
                                                                 gv.arc_radius, gf.arc_radius,
                                                                 gv.P_word,     gf.P_word,
                                                                 gp.modals[MODAL_GROUP_G1],
                                                                 gv.motion_mode);
                                                                 break;
                                          }
                default: break;
            }
            cm_set_absolute_override(MODEL, ABSOLUTE_OVERRIDE_OFF);     // un-set absolute override once the move is planned
        }
        default:
            // quiet the compiler warning about all the things we don't handle
            break;
    }

    // do the program stops and ends : M0, M1, M2, M30, M60
    if (gf.program_flow == true) {
        if (gv.program_flow == PROGRAM_STOP) {
            cm_program_stop();
        } else {
            cm_program_end();
        }
    }
    return (status);
}

/***********************************************************************************
 * _execute_gcode_block_marlin() - collect Marlin exection functions here
 */

static stat_t _execute_gcode_block_marlin()
{
#if MARLIN_COMPAT_ENABLED == true
    // Check for sequential line numbers
    if (gf.linenum && gf.checksum) {
        if (gv.next_action != NEXT_ACTION_MARLIN_RESET_LINE_NUMBERS) {
            ritorno(cm_check_linenum());
        }
        cm.gmx.last_line_number = cm.gm.linenum;

        // since this is handled, clear gf.checksum so it doesn't again
        gf.checksum = false;
    }

    // E should ONLY be seen in marlin flavor
    if (gf.E_word) {
        mst.marlin_flavor = true;
    }

    // adjust T real quick
    if (mst.marlin_flavor && gf.tool_select) {
        gv.tool_select += 1;
        cm.gm.tool_select = gv.tool_select; // We need to go ahead and apply to tool select, and in Marlin 0 is valid, so add 1
        cm.gm.tool = cm.gm.tool_select;     // Also, in Marlin, tool changes are effective immediately :facepalm:
        gf.tool_select = false;             // prevent a tool_select command from being buffered (planning to zero)
    }
    else if (cm.gm.tool_select == 0) {
        cm.gm.tool_select = 1;              // We need to ensure we have a valid tool selected, often Marlin gcode won't have a T word at all
        cm.gm.tool = cm.gm.tool_select;     // Also, in Marlin, tool changes are effective immediately :facepalm:
    }

    // Deal with E
    if (gf.marlin_relative_extruder_mode) {                 // M82, M83
        marlin_set_extruder_mode(gv.marlin_relative_extruder_mode);
    }    
    if (gf.E_word) {
        // Ennn T0 -> Annn
        if (cm.gm.tool_select == 1) {
            gf.target[AXIS_A] = true;
            gv.target[AXIS_A] = gv.E_word;
        }
        // Ennn T1 -> Bnnn
        else if (cm.gm.tool_select == 2) {
            gf.target[AXIS_B] = true;
            gv.target[AXIS_B] = gv.E_word;
        }
        else {
            _debug_trap("invalid tool selection");
            return (STAT_INPUT_VALUE_RANGE_ERROR);
        }
    }

    if ((mst.marlin_flavor || (MARLIN_COMM_MODE == js.json_mode)) &&
        (NEXT_ACTION_GOTO_G28_POSITION == gv.next_action)) {
        gv.next_action = NEXT_ACTION_SEARCH_HOME;
    }

    switch (gv.next_action) {
        case NEXT_ACTION_MARLIN_PRINT_TEMPERATURES: {       // M105
            js.json_mode = MARLIN_COMM_MODE;                // we use M105 to know when to switch
            ritorno(marlin_request_temperature_report());
            break;
        }
        case NEXT_ACTION_MARLIN_PRINT_POSITION:  {          // M114
            js.json_mode = MARLIN_COMM_MODE;                // we use M105 to know when to switch
            ritorno(marlin_request_position_report());
            break;
        }
        case NEXT_ACTION_MARLIN_SET_EXTRUDER_TEMP:          // M104 or M109
        case NEXT_ACTION_MARLIN_SET_BED_TEMP:       {       // M140 or M190
            mst.marlin_flavor = true;                       // these gcodes are ONLY in marlin flavor
            float temp = 0;
            if (gf.S_word) { temp = gv.S_word; }
            if (gf.P_word) { temp = gv.P_word; }            // we treat them the same, for now

            uint8_t tool = (gv.next_action == NEXT_ACTION_MARLIN_SET_EXTRUDER_TEMP) ? cm.gm.tool_select : 3;
            ritorno(marlin_set_temperature(tool, temp, gf.marlin_wait_for_temp));

            gf.P_word = false;
            gf.S_word = false;
            break;
        }
        case NEXT_ACTION_MARLIN_CANCEL_WAIT_TEMP:   {       // M108
            js.json_mode = MARLIN_COMM_MODE;                // we use M105 to know when to switch
            cm_request_feedhold();
            cm_request_queue_flush();
            break;
        }
        case NEXT_ACTION_MARLIN_TRAM_BED:       {           // G29
            mst.marlin_flavor = true;                       // these gcodes are ONLY in marlin flavor
            ritorno(marlin_start_tramming_bed());
            break;
        }
        case NEXT_ACTION_MARLIN_SET_FAN_SPEED:  {           // M106
            mst.marlin_flavor = true;                       // these gcodes are ONLY in marlin flavor
            ritorno(marlin_set_fan_speed(
                gf.P_word? gv.P_word : 0,
                gf.S_word? gv.S_word : 0));
            gf.P_word = false;
            gf.S_word = false;
            break;
        }
        case NEXT_ACTION_MARLIN_STOP_FAN:      {            // M107
            mst.marlin_flavor = true;                       // these gcodes are ONLY in marlin flavor
            ritorno(marlin_set_fan_speed(gf.P_word ? gv.P_word : 0, 0));
            gf.P_word = false;
            gf.S_word = false;
            break;
        }
        // adjust G28 (already adjusted to G28.2)
        // with no X, Y, or Z, we assume all three
        case NEXT_ACTION_SEARCH_HOME:       {               // G28 (g2core G28.2)
            if (!gf.target[AXIS_X] && !gf.target[AXIS_Y] && !gf.target[AXIS_Z]) {
                gv.target[AXIS_X]=0.0; gf.target[AXIS_X]=true;
                gv.target[AXIS_Y]=0.0; gf.target[AXIS_Y]=true;
                gv.target[AXIS_Z]=0.0; gf.target[AXIS_Z]=true;
            }
            break;
        }
        case NEXT_ACTION_MARLIN_DISABLE_MOTORS:    {        // M84 and M18
            if (gf.S_word) {
                ritorno(marlin_set_motor_timeout(gv.S_word));
                gf.S_word = false;
            } else {
                ritorno(marlin_disable_motors());
            }
            break;
        }
        case NEXT_ACTION_MARLIN_SET_MT:            {        // M85
            if (gf.S_word) {
                ritorno(marlin_set_motor_timeout(gv.S_word));
                gf.S_word = false;
            } else {
                return (STAT_OK);                           // this means nothing, but it's not an error
            }
        }
        case NEXT_ACTION_MARLIN_DISPLAY_ON_SCREEN: {        // M117
            return (STAT_OK);                               // ignore for now
        }
        case NEXT_ACTION_MARLIN_REPORT_VERSION:    {        // M115
            js.json_mode = MARLIN_COMM_MODE;
            ritorno(marlin_report_version());
            break;
        }
        case NEXT_ACTION_MARLIN_RESET_LINE_NUMBERS:{        // M110
            js.json_mode = MARLIN_COMM_MODE;                // we already did this above in if (gf.linenum) {}
            return (STAT_OK);
        }
        case NEXT_ACTION_DEFAULT: {
            if (mst.marlin_flavor) {
                if (gf.motion_mode) {                       // adjust G0 to almost always be the same as G1
                    if (gf.E_word && (!gf.target[AXIS_X] && !gf.target[AXIS_Y] && !gf.target[AXIS_Z])) {
                        gv.motion_mode = MOTION_MODE_STRAIGHT_TRAVERSE; // G0
                    } else {
                        gv.motion_mode = MOTION_MODE_STRAIGHT_FEED;     // G1
                    }
                }
            }
            break;
        }
        default:
        // quiet the compiler warning about all the things we don't handle
        break;
    } // switch (gv.next_action)

#endif // MARLIN_COMPAT_ENABLED

    return (STAT_OK);
}


/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

stat_t gc_get_gc(nvObj_t *nv)
{
    ritorno(nv_copy_string(nv, cs.saved_buf));
    nv->valuetype = TYPE_STRING;
    return (STAT_OK);
}

stat_t gc_run_gc(nvObj_t *nv)
{
    return(gcode_parser(*nv->stringp));
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

// no text mode functions here. Move along

#endif // __TEXT_MODE
