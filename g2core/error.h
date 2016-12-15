/*
 * error.h - g2core status codes
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart, Jr.
 * Copyright (c) 2010 - 2016 Robert Giseburt
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
#ifndef ERROR_H_ONCE
#define ERROR_H_ONCE

/************************************************************************************
 * STATUS CODES
 *
 * Most of the status codes (except STAT_OK) are exceptions. These are typically
 * returned by the failed command and reported back via JSON or text.
 *
 * Status codes are divided into ranges for clarity and extensibility. At some point
 * this may break down and the whole thing will get messy(er), but it's advised not
 * to change the values of existing status codes once they are in distribution.
 *
 * Ranges are:
 *
 *   0 - 19     OS, communications and low-level status
 *              This range is aligned with the XIO codes and must be so (v8 only)
 *              Please don't change them without checking the corresponding values in xio.h
 *
 *  20 - 99     Generic internal and application errors.
 *              Internal errors start at 20 and work up,
 *              Assertion failures start at 99 and work down.
 *
 * 100 - 129    Generic data and input errors - not specific to Gcode or g2core
 *
 * 130 - 255    Gcode and g2core application errors and warnings
 *
 * See status_codes.cpp for associated message strings. Any changes to the codes may
 * also require changing the message strings and string array in status_codes.cpp
 */

// **** Declarations, functions and macros. See main.cpp for implementation ****

typedef uint8_t stat_t;
extern stat_t status_code;

#define GLOBAL_STRING_LEN 256  // allow sufficient space for JSON responses and message strings
extern char global_string_buf[];

char *get_status_message(stat_t status);

// ritorno is a handy way to provide exception returns
// It returns only if an error occurred. (ritorno is Italian for return)
#define ritorno(a) if((status_code=a) != STAT_OK) { return(status_code); }

/**********************
 **** STATUS CODES ****
 **********************/

// OS, communications and low-level status (must align with XIO_xxxx codes in xio.h)
#define STAT_OK 0                       // function completed OK
#define STAT_ERROR 1                    // generic error return (EPERM)
#define STAT_EAGAIN 2                   // function would block here (call again)
#define STAT_NOOP 3                     // function had no-operation
#define STAT_COMPLETE 4                 // operation is complete
#define STAT_SHUTDOWN 5                 // operation was shutdown (terminated gracefully)
#define STAT_PANIC 6                    // system panic (not graceful)
#define STAT_EOL 7                      // function returned end-of-line
#define STAT_EOF 8                      // function returned end-of-file
#define STAT_FILE_NOT_OPEN 9
#define STAT_FILE_SIZE_EXCEEDED 10
#define STAT_NO_SUCH_DEVICE 11
#define STAT_BUFFER_EMPTY 12
#define STAT_BUFFER_FULL 13
#define STAT_BUFFER_FULL_FATAL 14
#define STAT_INITIALIZING 15            // initializing - not ready for use
#define STAT_ENTERING_BOOT_LOADER 16    // this code actually emitted from boot loader, not g2
#define STAT_FUNCTION_IS_STUBBED 17
#define STAT_ALARM 18                   // system alarm triggered
#define STAT_NO_DISPLAY 19              // suppress results display - presumably handled upstream
// NOTE: XIO codes align to here

// Internal errors and startup messages
#define STAT_INTERNAL_ERROR 20          // unrecoverable internal error
#define STAT_INTERNAL_RANGE_ERROR 21    // number range other than by user input
#define STAT_FLOATING_POINT_ERROR 22    // number conversion error
#define STAT_DIVIDE_BY_ZERO 23
#define STAT_INVALID_ADDRESS 24
#define STAT_READ_ONLY_ADDRESS 25
#define STAT_INIT_FAILURE 26
#define STAT_ERROR_27 27                // was ALARMED in 0.97
#define STAT_FAILED_TO_GET_PLANNER_BUFFER 28
#define STAT_GENERIC_EXCEPTION_REPORT 29 // used for test

#define STAT_PREP_LINE_MOVE_TIME_IS_INFINITE 30
#define STAT_PREP_LINE_MOVE_TIME_IS_NAN 31
#define STAT_FLOAT_IS_INFINITE 32
#define STAT_FLOAT_IS_NAN 33
#define STAT_PERSISTENCE_ERROR 34
#define STAT_BAD_STATUS_REPORT_SETTING 35
#define STAT_FAILED_GET_PLANNER_BUFFER 36

#define STAT_ERROR_37 37
#define STAT_ERROR_38 38
#define STAT_ERROR_39 39

#define STAT_ERROR_40 40
#define STAT_ERROR_41 41
#define STAT_ERROR_42 42
#define STAT_ERROR_43 43
#define STAT_ERROR_44 44
#define STAT_ERROR_45 45
#define STAT_ERROR_46 46
#define STAT_ERROR_47 47
#define STAT_ERROR_48 48
#define STAT_ERROR_49 49

#define STAT_ERROR_50 50
#define STAT_ERROR_51 51
#define STAT_ERROR_52 52
#define STAT_ERROR_53 53
#define STAT_ERROR_54 54
#define STAT_ERROR_55 55
#define STAT_ERROR_56 56
#define STAT_ERROR_57 57
#define STAT_ERROR_58 58
#define STAT_ERROR_59 59

#define STAT_ERROR_60 60
#define STAT_ERROR_61 61
#define STAT_ERROR_62 62
#define STAT_ERROR_63 63
#define STAT_ERROR_64 64
#define STAT_ERROR_65 65
#define STAT_ERROR_66 66
#define STAT_ERROR_67 67
#define STAT_ERROR_68 68
#define STAT_ERROR_69 69

#define STAT_ERROR_70 70
#define STAT_ERROR_71 71
#define STAT_ERROR_72 72
#define STAT_ERROR_73 73
#define STAT_ERROR_74 74
#define STAT_ERROR_75 75
#define STAT_ERROR_76 76
#define STAT_ERROR_77 77
#define STAT_ERROR_78 78
#define STAT_ERROR_79 79

#define STAT_ERROR_80 80
#define STAT_ERROR_81 81
#define STAT_ERROR_82 82
#define STAT_ERROR_83 83
#define STAT_ERROR_84 84
#define STAT_ERROR_85 85
#define STAT_ERROR_86 86
#define STAT_ERROR_87 87

// Assertion failures - build down from 99 until they meet the system internal errors

#define STAT_BUFFER_FREE_ASSERTION_FAILURE 88
#define STAT_STATE_MANAGEMENT_ASSERTION_FAILURE 89
#define STAT_CONFIG_ASSERTION_FAILURE 90
#define STAT_XIO_ASSERTION_FAILURE 91
#define STAT_ENCODER_ASSERTION_FAILURE 92
#define STAT_STEPPER_ASSERTION_FAILURE 93
#define STAT_PLANNER_ASSERTION_FAILURE 94
#define STAT_CANONICAL_MACHINE_ASSERTION_FAILURE 95
#define STAT_CONTROLLER_ASSERTION_FAILURE 96
#define STAT_STACK_OVERFLOW 97
#define STAT_MEMORY_FAULT 98                    // generic memory corruption detected by magic numbers
#define STAT_GENERIC_ASSERTION_FAILURE 99       // generic assertion failure - unclassified

// Application and data input errors

// Generic data input errors

#define STAT_UNRECOGNIZED_NAME 100              // parser didn't recognize the name
#define STAT_INVALID_OR_MALFORMED_COMMAND 101   // malformed line to parser
#define STAT_BAD_NUMBER_FORMAT 102              // number format error
#define STAT_UNSUPPORTED_TYPE 103               // an otherwise valid JSON type is not supported
#define STAT_PARAMETER_IS_READ_ONLY 104         // input error: parameter cannot be set
#define STAT_PARAMETER_CANNOT_BE_READ 105       // input error: parameter cannot be returned
#define STAT_COMMAND_NOT_ACCEPTED 106           // input error: command cannot be accepted at this time
#define STAT_INPUT_EXCEEDS_MAX_LENGTH 107       // input error: input string is too long
#define STAT_INPUT_LESS_THAN_MIN_VALUE 108      // input error: value is under minimum
#define STAT_INPUT_EXCEEDS_MAX_VALUE 109        // input error: value is over maximum
#define STAT_INPUT_VALUE_RANGE_ERROR 110        // input error: value is out-of-range

#define STAT_JSON_SYNTAX_ERROR 111              // JSON input string is not well formed
#define STAT_JSON_TOO_MANY_PAIRS 112            // JSON input string has too many JSON pairs
#define STAT_JSON_OUTPUT_TOO_LONG 113           // JSON output exceeds buffer size
#define STAT_NESTED_TXT_CONTAINER 114           // JSON 'txt' fields cannot be nested
#define STAT_MAX_DEPTH_EXCEEDED 115             // JSON exceeded maximum nesting depth
#define STAT_VALUE_TYPE_ERROR 116               // JSON value does not agree with variable type

#define STAT_INPUT_FROM_MUTED_CHANNEL_ERROR 117               // input from a muted channel was ignored
#define STAT_ERROR_118 118
#define STAT_ERROR_119 119

#define STAT_ERROR_120 120
#define STAT_ERROR_121 121
#define STAT_ERROR_122 122
#define STAT_ERROR_123 123
#define STAT_ERROR_124 124
#define STAT_ERROR_125 125
#define STAT_ERROR_126 126
#define STAT_ERROR_127 127
#define STAT_ERROR_128 128
#define STAT_ERROR_129 129

// Gcode errors and warnings (Most originate from NIST - by concept, not number)
// Fascinating: http://www.cncalarms.com/

#define STAT_GCODE_GENERIC_INPUT_ERROR 130      // generic error for gcode input
#define STAT_GCODE_COMMAND_UNSUPPORTED 131      // G command is not supported
#define STAT_MCODE_COMMAND_UNSUPPORTED 132      // M command is not supported
#define STAT_GCODE_MODAL_GROUP_VIOLATION 133    // gcode modal group error
#define STAT_GCODE_AXIS_IS_MISSING 134          // command requires at least one axis present
#define STAT_GCODE_AXIS_CANNOT_BE_PRESENT 135   // error if G80 has axis words
#define STAT_GCODE_AXIS_IS_INVALID 136          // an axis is specified that is illegal for the command
#define STAT_GCODE_AXIS_IS_NOT_CONFIGURED 137   // WARNING: attempt to program an axis that is disabled
#define STAT_GCODE_AXIS_NUMBER_IS_MISSING 138   // axis word is missing its value
#define STAT_GCODE_AXIS_NUMBER_IS_INVALID 139   // axis word value is illegal

#define STAT_GCODE_ACTIVE_PLANE_IS_MISSING 140  // active plane is not programmed
#define STAT_GCODE_ACTIVE_PLANE_IS_INVALID 141  // active plane selected is not valid for this command
#define STAT_GCODE_FEEDRATE_NOT_SPECIFIED 142   // move has no feedrate
#define STAT_GCODE_INVERSE_TIME_MODE_CANNOT_BE_USED 143  // G38.2 and some canned cycles cannot accept inverse time mode
#define STAT_GCODE_ROTARY_AXIS_CANNOT_BE_USED 144   // G38.2 and some other commands cannot have rotary axes
#define STAT_GCODE_G53_WITHOUT_G0_OR_G1 145         // G0 or G1 must be active for G53
#define STAT_REQUESTED_VELOCITY_EXCEEDS_LIMITS 146
#define STAT_CUTTER_COMPENSATION_CANNOT_BE_ENABLED 147
#define STAT_PROGRAMMED_POINT_SAME_AS_CURRENT_POINT 148
#define STAT_SPINDLE_SPEED_BELOW_MINIMUM 149

#define STAT_SPINDLE_SPEED_MAX_EXCEEDED 150
#define STAT_SPINDLE_MUST_BE_OFF 151
#define STAT_SPINDLE_MUST_BE_TURNING 152            // some canned cycles require spindle to be turning when called
#define STAT_ARC_ERROR_RESERVED 153                 // RESERVED
#define STAT_ARC_HAS_IMPOSSIBLE_CENTER_POINT 154    // trap (.05 inch/.5 mm) OR ((.0005 inch/.005mm) AND .1% of radius condition
#define STAT_ARC_SPECIFICATION_ERROR 155            // generic arc specification error
#define STAT_ARC_AXIS_MISSING_FOR_SELECTED_PLANE 156  // arc is missing axis (axes) required by selected plane
#define STAT_ARC_OFFSETS_MISSING_FOR_SELECTED_PLANE 157 // one or both offsets are not specified
#define STAT_ARC_RADIUS_OUT_OF_TOLERANCE 158        // WARNING - radius arc is too large - accuracy in question
#define STAT_ARC_ENDPOINT_IS_STARTING_POINT 159

#define STAT_P_WORD_IS_MISSING 160                  // P must be present for dwells and other functions
#define STAT_P_WORD_IS_INVALID 161                  // generic P value error
#define STAT_P_WORD_IS_ZERO 162
#define STAT_P_WORD_IS_NEGATIVE 163                 // dwells require positive P values
#define STAT_P_WORD_IS_NOT_AN_INTEGER 164           // G10s and other commands require integer P numbers
#define STAT_P_WORD_IS_NOT_VALID_TOOL_NUMBER 165
#define STAT_D_WORD_IS_MISSING 166
#define STAT_D_WORD_IS_INVALID 167
#define STAT_E_WORD_IS_MISSING 168
#define STAT_E_WORD_IS_INVALID 169

#define STAT_H_WORD_IS_MISSING 170
#define STAT_H_WORD_IS_INVALID 171
#define STAT_L_WORD_IS_MISSING 172
#define STAT_L_WORD_IS_INVALID 173
#define STAT_Q_WORD_IS_MISSING 174
#define STAT_Q_WORD_IS_INVALID 175
#define STAT_R_WORD_IS_MISSING 176
#define STAT_R_WORD_IS_INVALID 177
#define STAT_S_WORD_IS_MISSING 178
#define STAT_S_WORD_IS_INVALID 179

#define STAT_T_WORD_IS_MISSING 180
#define STAT_T_WORD_IS_INVALID 181

/* reserved for Gcode or other program errors */

#define STAT_ERROR_183 183
#define STAT_ERROR_184 184
#define STAT_ERROR_185 185
#define STAT_ERROR_186 186
#define STAT_ERROR_187 187
#define STAT_ERROR_188 188
#define STAT_ERROR_189 189

#define STAT_ERROR_190 190
#define STAT_ERROR_191 191
#define STAT_ERROR_192 192
#define STAT_ERROR_193 193
#define STAT_ERROR_194 194
#define STAT_ERROR_195 195
#define STAT_ERROR_196 196
#define STAT_ERROR_197 197
#define STAT_ERROR_198 198
#define STAT_ERROR_199 199

// g2core errors and warnings

#define STAT_GENERIC_ERROR 200
#define STAT_MINIMUM_LENGTH_MOVE 201            // move is less than minimum length
#define STAT_MINIMUM_TIME_MOVE 202              // move is less than minimum time
#define STAT_LIMIT_SWITCH_HIT 203               // a limit switch was hit causing shutdown
#define STAT_COMMAND_REJECTED_BY_ALARM 204      // command was not processed because machine is alarmed
#define STAT_COMMAND_REJECTED_BY_SHUTDOWN 205   // command was not processed because machine is shutdown
#define STAT_COMMAND_REJECTED_BY_PANIC 206      // command was not processed because machine is paniced
#define STAT_KILL_JOB 207                       // ^d received (job kill)
#define STAT_NO_GPIO 208                        // no GPIO exists for this value

#define STAT_TEMPERATURE_CONTROL_ERROR 209      // temperature controls err'd out

#define STAT_ERROR_210 210
#define STAT_ERROR_211 211
#define STAT_ERROR_212 212
#define STAT_ERROR_213 213
#define STAT_ERROR_214 214
#define STAT_ERROR_215 215
#define STAT_ERROR_216 216
#define STAT_ERROR_217 217
#define STAT_ERROR_218 218
#define STAT_ERROR_219 219

#define STAT_SOFT_LIMIT_EXCEEDED 220            // soft limit error - axis unspecified
#define STAT_SOFT_LIMIT_EXCEEDED_XMIN 221       // soft limit error - X minimum
#define STAT_SOFT_LIMIT_EXCEEDED_XMAX 222       // soft limit error - X maximum
#define STAT_SOFT_LIMIT_EXCEEDED_YMIN 223       // soft limit error - Y minimum
#define STAT_SOFT_LIMIT_EXCEEDED_YMAX 224       // soft limit error - Y maximum
#define STAT_SOFT_LIMIT_EXCEEDED_ZMIN 225       // soft limit error - Z minimum
#define STAT_SOFT_LIMIT_EXCEEDED_ZMAX 226       // soft limit error - Z maximum
#define STAT_SOFT_LIMIT_EXCEEDED_AMIN 227       // soft limit error - A minimum
#define STAT_SOFT_LIMIT_EXCEEDED_AMAX 228       // soft limit error - A maximum
#define STAT_SOFT_LIMIT_EXCEEDED_BMIN 229       // soft limit error - B minimum

#define STAT_SOFT_LIMIT_EXCEEDED_BMAX 220       // soft limit error - B maximum
#define STAT_SOFT_LIMIT_EXCEEDED_CMIN 231       // soft limit error - C minimum
#define STAT_SOFT_LIMIT_EXCEEDED_CMAX 232       // soft limit error - C maximum
#define STAT_SOFT_LIMIT_EXCEEDED_ARC 233        // soft limit err on arc

#define STAT_ERROR_234 234
#define STAT_ERROR_235 235
#define STAT_ERROR_236 236
#define STAT_ERROR_237 237
#define STAT_ERROR_238 238
#define STAT_ERROR_239 239

#define STAT_HOMING_CYCLE_FAILED 240            // homing cycle did not complete
#define STAT_HOMING_ERROR_BAD_OR_NO_AXIS 241
#define STAT_HOMING_ERROR_ZERO_SEARCH_VELOCITY 242
#define STAT_HOMING_ERROR_ZERO_LATCH_VELOCITY 243
#define STAT_HOMING_ERROR_TRAVEL_MIN_MAX_IDENTICAL 244
#define STAT_HOMING_ERROR_NEGATIVE_LATCH_BACKOFF 245
#define STAT_HOMING_ERROR_HOMING_INPUT_MISCONFIGURED 246
#define STAT_HOMING_ERROR_MUST_CLEAR_SWITCHES_BEFORE_HOMING 247
#define STAT_ERROR_248 248
#define STAT_ERROR_249 249

#define STAT_PROBE_CYCLE_FAILED 250             // probing cycle did not complete
#define STAT_PROBE_TRAVEL_TOO_SMALL 251
#define STAT_NO_PROBE_SWITCH_CONFIGURED 252
#define STAT_MULTIPLE_PROBE_SWITCHES_CONFIGURED 253
#define STAT_PROBE_SWITCH_ON_ABC_AXIS 254

#define STAT_ERROR_255 255

// ****** !!! Do not exceed 255 without also changing stat_t typedef ******


/********************************
 **** Status message strings ****
 ********************************/

// NB: The ones that are never called are optimized out by the compiler (e.g. the placeholders)
static const char stat_00[] = "OK";
static const char stat_01[] = "Error";
static const char stat_02[] = "Eagain";
static const char stat_03[] = "No operation performed";
static const char stat_04[] = "Completed operation";
static const char stat_05[] = "System shutdown";
static const char stat_06[] = "System panic";
static const char stat_07[] = "End of line";
static const char stat_08[] = "End of file";
static const char stat_09[] = "File not open";

static const char stat_10[] = "Max file size exceeded";
static const char stat_11[] = "No such device";
static const char stat_12[] = "Buffer empty";
static const char stat_13[] = "Buffer full non-fatal";
static const char stat_14[] = "Buffer full FATAL";
static const char stat_15[] = "Initializing";
static const char stat_16[] = "Entering boot loader";
static const char stat_17[] = "Function is stubbed";
static const char stat_18[] = "System alarm";
static const char stat_19[] = "19";

static const char stat_20[] = "Internal error";
static const char stat_21[] = "Internal range error";
static const char stat_22[] = "Floating point error";
static const char stat_23[] = "Divide by zero";
static const char stat_24[] = "Invalid Address";
static const char stat_25[] = "Read-only address";
static const char stat_26[] = "Initialization failure";
static const char stat_27[] = "27";
static const char stat_28[] = "Failed to get planner buffer";
static const char stat_29[] = "Generic exception report";

static const char stat_30[] = "Move time is infinite";
static const char stat_31[] = "Move time is NAN";
static const char stat_32[] = "Float is infinite";
static const char stat_33[] = "Float is NAN";
static const char stat_34[] = "Persistence error";
static const char stat_35[] = "Bad status report setting";
static const char stat_36[] = "Failed to get planner buffer";

static const char stat_37[] = "Backplan hit running buffer";
static const char stat_38[] = "38";
static const char stat_39[] = "39";

static const char stat_40[] = "40";
static const char stat_41[] = "41";
static const char stat_42[] = "42";
static const char stat_43[] = "43";
static const char stat_44[] = "44";
static const char stat_45[] = "45";
static const char stat_46[] = "46";
static const char stat_47[] = "47";
static const char stat_48[] = "48";
static const char stat_49[] = "49";
static const char stat_50[] = "50";
static const char stat_51[] = "51";
static const char stat_52[] = "52";
static const char stat_53[] = "53";
static const char stat_54[] = "54";
static const char stat_55[] = "55";
static const char stat_56[] = "56";
static const char stat_57[] = "57";
static const char stat_58[] = "58";
static const char stat_59[] = "59";
static const char stat_60[] = "60";
static const char stat_61[] = "61";
static const char stat_62[] = "62";
static const char stat_63[] = "63";
static const char stat_64[] = "64";
static const char stat_65[] = "65";
static const char stat_66[] = "66";
static const char stat_67[] = "67";
static const char stat_68[] = "68";
static const char stat_69[] = "69";
static const char stat_70[] = "70";
static const char stat_71[] = "71";
static const char stat_72[] = "72";
static const char stat_73[] = "73";
static const char stat_74[] = "74";
static const char stat_75[] = "75";
static const char stat_76[] = "76";
static const char stat_77[] = "77";
static const char stat_78[] = "78";
static const char stat_79[] = "79";
static const char stat_80[] = "80";
static const char stat_81[] = "81";
static const char stat_82[] = "82";
static const char stat_83[] = "83";
static const char stat_84[] = "84";
static const char stat_85[] = "85";
static const char stat_86[] = "86";
static const char stat_87[] = "87";

static const char stat_88[] = "Buffer free assertion failure";
static const char stat_89[] = "State management assertion failure";
static const char stat_90[] = "Config assertion failure";
static const char stat_91[] = "XIO assertion failure";
static const char stat_92[] = "Encoder assertion failure";
static const char stat_93[] = "Stepper assertion failure";
static const char stat_94[] = "Planner assertion failure";
static const char stat_95[] = "Canonical machine assertion failure";
static const char stat_96[] = "Controller assertion failure";
static const char stat_97[] = "Stack overflow detected";
static const char stat_98[] = "Memory fault detected";
static const char stat_99[] = "Generic assertion failure";

static const char stat_100[] = "Unrecognized command or config name";
static const char stat_101[] = "Invalid or malformed command";
static const char stat_102[] = "Bad number format";
static const char stat_103[] = "Unsupported number or JSON type";
static const char stat_104[] = "Parameter is read-only";
static const char stat_105[] = "Parameter cannot be read";
static const char stat_106[] = "Command not accepted";
static const char stat_107[] = "Input exceeds max length";
static const char stat_108[] = "Input less than minimum value";
static const char stat_109[] = "Input exceeds maximum value";

static const char stat_110[] = "Input value range error";
static const char stat_111[] = "JSON syntax error";
static const char stat_112[] = "JSON has too many pairs";
static const char stat_113[] = "JSON string too long";
static const char stat_114[] = "JSON txt fields cannot be nested";
static const char stat_115[] = "JSON maximum nesting depth exceeded";
static const char stat_116[] = "JSON value does not agree with variable type";
static const char stat_117[] = "117";
static const char stat_118[] = "118";
static const char stat_119[] = "119";

static const char stat_120[] = "120";
static const char stat_121[] = "121";
static const char stat_122[] = "122";
static const char stat_123[] = "123";
static const char stat_124[] = "124";
static const char stat_125[] = "125";
static const char stat_126[] = "126";
static const char stat_127[] = "127";
static const char stat_128[] = "128";
static const char stat_129[] = "129";

static const char stat_130[] = "Generic Gcode input error";
static const char stat_131[] = "Gcode command unsupported";
static const char stat_132[] = "M code unsupported";
static const char stat_133[] = "Gcode modal group violation";
static const char stat_134[] = "Axis word missing";
static const char stat_135[] = "Axis cannot be present";
static const char stat_136[] = "Axis invalid for this command";
static const char stat_137[] = "Axis disabled";
static const char stat_138[] = "Axis target position missing";
static const char stat_139[] = "Axis target position invalid";

static const char stat_140[] = "Selected plane missing";
static const char stat_141[] = "Selected plane invalid";
static const char stat_142[] = "Feedrate not specified";
static const char stat_143[] = "Inverse time mode cannot be used with this command";
static const char stat_144[] = "Rotary axes cannot be used with this command";
static const char stat_145[] = "G0 or G1 must be active for G53";
static const char stat_146[] = "Requested velocity exceeds limits";
static const char stat_147[] = "Cutter compensation cannot be enabled";
static const char stat_148[] = "Programmed point same as current point";
static const char stat_149[] = "Spindle speed below minimum";

static const char stat_150[] = "Spindle speed exceeded maximum";
static const char stat_151[] = "Spindle must be off for this command";
static const char stat_152[] = "Spindle must be turning for this command";
static const char stat_153[] = "153";
static const char stat_154[] = "Arc specification error - impossible center point";
static const char stat_155[] = "Arc specification error";
static const char stat_156[] = "Arc specification error - missing axis(es)";
static const char stat_157[] = "Arc specification error - missing offset(s)";
//--------------------------------------1--------10--------20--------30--------40--------50--------60-64
static const char stat_158[] = "Arc specification error - radius arc out of tolerance";
static const char stat_159[] = "Arc specification error - endpoint is starting point";

static const char stat_160[] = "P word missing";
static const char stat_161[] = "P word invalid";
static const char stat_162[] = "P word zero";
static const char stat_163[] = "P word negative";
static const char stat_164[] = "P word not an integer";
static const char stat_165[] = "P word not a valid tool number";
static const char stat_166[] = "D word missing";
static const char stat_167[] = "D word invalid";
static const char stat_168[] = "E word missing";
static const char stat_169[] = "E word invalid";

static const char stat_170[] = "H word missing";
static const char stat_171[] = "H word invalid";
static const char stat_172[] = "L word missing";
static const char stat_173[] = "L word invalid";
static const char stat_174[] = "Q word missing";
static const char stat_175[] = "Q word invalid";
static const char stat_176[] = "R word missing";
static const char stat_177[] = "R word invalid";
static const char stat_178[] = "S word missing";
static const char stat_179[] = "S word invalid";

static const char stat_180[] = "T word missing";
static const char stat_181[] = "T word invalid";
static const char stat_182[] = "182";
static const char stat_183[] = "183";
static const char stat_184[] = "184";
static const char stat_185[] = "185";
static const char stat_186[] = "186";
static const char stat_187[] = "187";
static const char stat_188[] = "188";
static const char stat_189[] = "189";

static const char stat_190[] = "190";
static const char stat_191[] = "191";
static const char stat_192[] = "192";
static const char stat_193[] = "193";
static const char stat_194[] = "194";
static const char stat_195[] = "195";
static const char stat_196[] = "196";
static const char stat_197[] = "197";
static const char stat_198[] = "198";
static const char stat_199[] = "199";

static const char stat_200[] = "Generic error";
static const char stat_201[] = "Move < min length";
static const char stat_202[] = "Move < min time";
//--------------------------------------1--------10--------20--------30--------40--------50--------60-64
static const char stat_203[] = "Limit hit [$clear to reset, $lim=0 to override]";
static const char stat_204[] = "Command rejected by ALARM [$clear to reset]";
static const char stat_205[] = "Command rejected by SHUTDOWN [$clear to reset]";
static const char stat_206[] = "Command rejected by PANIC [^x to reset]";
static const char stat_207[] = "Kill job";
static const char stat_208[] = "No GPIO for this value";
static const char stat_209[] = "209";

static const char stat_210[] = "210";
static const char stat_211[] = "211";
static const char stat_212[] = "212";
static const char stat_213[] = "213";
static const char stat_214[] = "214";
static const char stat_215[] = "215";
static const char stat_216[] = "216";
static const char stat_217[] = "217";
static const char stat_218[] = "218";
static const char stat_219[] = "219";

static const char stat_220[] = "Soft limit";
static const char stat_221[] = "Soft limit - X min";
static const char stat_222[] = "Soft limit - X max";
static const char stat_223[] = "Soft limit - Y min";
static const char stat_224[] = "Soft limit - Y max";
static const char stat_225[] = "Soft limit - Z min";
static const char stat_226[] = "Soft limit - Z max";
static const char stat_227[] = "Soft limit - A min";
static const char stat_228[] = "Soft limit - A max";
static const char stat_229[] = "Soft limit - B min";
static const char stat_230[] = "Soft limit - B max";
static const char stat_231[] = "Soft limit - C min";
static const char stat_232[] = "Soft limit - C max";
static const char stat_233[] = "Soft limit during arc";
static const char stat_234[] = "234";
static const char stat_235[] = "235";
static const char stat_236[] = "236";
static const char stat_237[] = "237";
static const char stat_238[] = "238";
static const char stat_239[] = "239";

static const char stat_240[] = "Homing cycle failed";
static const char stat_241[] = "Homing Err - Bad or no axis specified";
static const char stat_242[] = "Homing Err - Search velocity is zero";
static const char stat_243[] = "Homing Err - Latch velocity is zero";
static const char stat_244[] = "Homing Err - Travel min & max are the same";
static const char stat_245[] = "245";
static const char stat_246[] = "Homing Err - Homing input is misconfigured";
static const char stat_247[] = "Homing Err - Must clear switches before homing";
static const char stat_248[] = "248";
static const char stat_249[] = "249";

static const char stat_250[] = "Probe cycle failed";
static const char stat_251[] = "Probe travel is too small";
static const char stat_252[] = "No probe switch configured";
static const char stat_253[] = "Multiple probe switches configured";
static const char stat_254[] = "Probe switch configured on ABC axis";
static const char stat_255[] = "255";

static const char *const stat_msg[] = {
    stat_00, stat_01, stat_02, stat_03, stat_04, stat_05, stat_06, stat_07, stat_08, stat_09,
    stat_10, stat_11, stat_12, stat_13, stat_14, stat_15, stat_16, stat_17, stat_18, stat_19,
    stat_20, stat_21, stat_22, stat_23, stat_24, stat_25, stat_26, stat_27, stat_28, stat_29,
    stat_30, stat_31, stat_32, stat_33, stat_34, stat_35, stat_36, stat_37, stat_38, stat_39,
    stat_40, stat_41, stat_42, stat_43, stat_44, stat_45, stat_46, stat_47, stat_48, stat_49,
    stat_50, stat_51, stat_52, stat_53, stat_54, stat_55, stat_56, stat_57, stat_58, stat_59,
    stat_60, stat_61, stat_62, stat_63, stat_64, stat_65, stat_66, stat_67, stat_68, stat_69,
    stat_70, stat_71, stat_72, stat_73, stat_74, stat_75, stat_76, stat_77, stat_78, stat_79,
    stat_80, stat_81, stat_82, stat_83, stat_84, stat_85, stat_86, stat_87, stat_88, stat_89,
    stat_90, stat_91, stat_92, stat_93, stat_94, stat_95, stat_96, stat_97, stat_98, stat_99,
    stat_100, stat_101, stat_102, stat_103, stat_104, stat_105, stat_106, stat_107, stat_108, stat_109,
    stat_110, stat_111, stat_112, stat_113, stat_114, stat_115, stat_116, stat_117, stat_118, stat_119,
    stat_120, stat_121, stat_122, stat_123, stat_124, stat_125, stat_126, stat_127, stat_128, stat_129,
    stat_130, stat_131, stat_132, stat_133, stat_134, stat_135, stat_136, stat_137, stat_138, stat_139,
    stat_140, stat_141, stat_142, stat_143, stat_144, stat_145, stat_146, stat_147, stat_148, stat_149,
    stat_150, stat_151, stat_152, stat_153, stat_154, stat_155, stat_156, stat_157, stat_158, stat_159,
    stat_160, stat_161, stat_162, stat_163, stat_164, stat_165, stat_166, stat_167, stat_168, stat_169,
    stat_170, stat_171, stat_172, stat_173, stat_174, stat_175, stat_176, stat_177, stat_178, stat_179,
    stat_180, stat_181, stat_182, stat_183, stat_184, stat_185, stat_186, stat_187, stat_188, stat_189,
    stat_190, stat_191, stat_192, stat_193, stat_194, stat_195, stat_196, stat_197, stat_198, stat_199,
    stat_200, stat_201, stat_202, stat_203, stat_204, stat_205, stat_206, stat_207, stat_208, stat_209,
    stat_210, stat_211, stat_212, stat_213, stat_214, stat_215, stat_216, stat_217, stat_218, stat_219,
    stat_220, stat_221, stat_222, stat_223, stat_224, stat_225, stat_226, stat_227, stat_228, stat_229,
    stat_230, stat_231, stat_232, stat_233, stat_234, stat_235, stat_236, stat_237, stat_238, stat_239,
    stat_240, stat_241, stat_242, stat_243, stat_244, stat_245, stat_246, stat_247, stat_248, stat_249,
    stat_250, stat_251, stat_252, stat_253, stat_254, stat_255
};

#endif // End of include guard: ERROR_H_ONCE
