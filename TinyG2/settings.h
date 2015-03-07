/*
 * settings.h - default runtime settings
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart Jr.
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
/*	The values in this file are the default settings that are loaded into a virgin EEPROM,
 *	and can be changed using the config commands. After initial load the EEPROM values
 *	(or changed values) are used.
 *
 *	System and hardware settings that you shouldn't need to change are in hardware.h
 *	Application settings that also shouldn't need to be changed are in tinyg.h
 */

#ifndef SETTINGS_H_ONCE
#define SETTINGS_H_ONCE

/**** MACHINE PROFILES ******************************************************
 *
 * Provide a SETTINGS_FILE in the makefile or compiler command line, e.g:
 *	SETTINGS_FILE="settings_shopbot_test.h"
 *
 * If no file is specified the default settings file will be used
 */
#ifdef SETTINGS_FILE
#define SETTINGS_FILE_PATH <settings/SETTINGS_FILE>
#include SETTINGS_FILE_PATH
#else
#include "settings/settings_default.h"				// Default settings for release
#endif

// Alternate settings files that may be available in the project:
//#include "settings/settings_othermill.h"				// OMC OtherMill
//#include "settings/settings_probotixV90.h"			// Probotix FireballV90
//#include "settings/settings_shapeoko2.h"				// Shapeoko2 standard kit
//#include "settings/settings_shopbot_sbv300.h"			// Shopbot sbv300 board profile
//#include "settings/settings_Ultimaker.h"				// Ultimaker 3D printer
//#include "settings/settings_Ultimaker_Rob_v9h.h"
//#include "settings/settings_zen7x12.h"				// Zen Toolworks 7x12


/****** TEMPORARY COMPATIBILITY CODE -- TO BE REMOVED ASAP *********/

#define X_SWITCH_MODE_MIN           SW_MODE_HOMING		// xsn  SW_MODE_DISABLED, SW_MODE_HOMING, SW_MODE_LIMIT, SW_MODE_HOMING_LIMIT
#define X_SWITCH_MODE_MAX           SW_MODE_DISABLED	// xsx  SW_MODE_DISABLED, SW_MODE_HOMING, SW_MODE_LIMIT, SW_MODE_HOMING_LIMIT
#define X_SWITCH_TYPE_MIN           SW_TYPE_NORMALLY_OPEN // rsn SW_TYPE_NORMALLY_OPEN, SW_TYPE_NORMALLY_CLOSED
#define X_SWITCH_TYPE_MAX           SW_TYPE_NORMALLY_OPEN // rsx SW_TYPE_NORMALLY_OPEN, SW_TYPE_NORMALLY_CLOSED

#define Y_SWITCH_MODE_MIN           SW_MODE_HOMING
#define Y_SWITCH_MODE_MAX           SW_MODE_DISABLED
#define Y_SWITCH_TYPE_MIN           SW_TYPE_NORMALLY_OPEN
#define Y_SWITCH_TYPE_MAX           SW_TYPE_NORMALLY_OPEN

#define Z_SWITCH_MODE_MIN           SW_MODE_DISABLED
#define Z_SWITCH_MODE_MAX           SW_MODE_HOMING
#define Z_SWITCH_TYPE_MIN           SW_TYPE_NORMALLY_OPEN
#define Z_SWITCH_TYPE_MAX           SW_TYPE_NORMALLY_OPEN

#define A_SWITCH_MODE_MIN           SW_MODE_HOMING
#define A_SWITCH_MODE_MAX           SW_MODE_DISABLED
#define A_SWITCH_TYPE_MIN           SW_TYPE_NORMALLY_OPEN
#define A_SWITCH_TYPE_MAX           SW_TYPE_NORMALLY_OPEN

#define B_SWITCH_MODE_MIN           SW_MODE_HOMING
#define B_SWITCH_MODE_MAX           SW_MODE_DISABLED
#define B_SWITCH_TYPE_MIN           SW_TYPE_NORMALLY_OPEN
#define B_SWITCH_TYPE_MAX           SW_TYPE_NORMALLY_OPEN

#define C_SWITCH_MODE_MIN           SW_MODE_HOMING
#define C_SWITCH_MODE_MAX           SW_MODE_DISABLED
#define C_SWITCH_TYPE_MIN           SW_TYPE_NORMALLY_OPEN
#define C_SWITCH_TYPE_MAX           SW_TYPE_NORMALLY_OPEN



/**** MACHINE PROFILES ******************************************************
 *
 * Setup resonable default for some settings, so that all of the settings files
 * don't heve to specify every value.
 *
 * Note that if a key value (e.g. DI1_MODE) is defined, then all of the ones in
 * that group (e.g. DI1_ACTION, DI1_FUNCTION) must be defined as well.
 *
 */

// Communications and reporting settings

#ifndef COMM_MODE
#define COMM_MODE                   JSON_MODE               // one of: TEXT_MODE, JSON_MODE
#endif

#ifndef COM_EXPAND_CR
#define COM_EXPAND_CR               false
#endif

#ifndef COM_ENABLE_ECHO
#define COM_ENABLE_ECHO             false
#endif

#ifndef COM_ENABLE_FLOW_CONTROL
#define COM_ENABLE_FLOW_CONTROL     FLOW_CONTROL_XON        // FLOW_CONTROL_OFF, FLOW_CONTROL_XON, FLOW_CONTROL_RTS
#endif

#ifndef NETWORK_MODE
#define NETWORK_MODE                NETWORK_STANDALONE
#endif

#ifndef TEXT_VERBOSITY
#define TEXT_VERBOSITY              TV_VERBOSE              // one of: TV_SILENT, TV_VERBOSE
#endif

#ifndef JSON_VERBOSITY
#define JSON_VERBOSITY              JV_MESSAGES             // one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE
#endif
#ifndef JSON_SYNTAX_MODE
#define JSON_SYNTAX_MODE            JSON_SYNTAX_STRICT      // one of JSON_SYNTAX_RELAXED, JSON_SYNTAX_STRICT
#endif
#ifndef JSON_FOOTER_STYLE
#define JSON_FOOTER_STYLE           1                       // 1 = footer w/checksum, 2 = footer w/window slots
#endif
#ifndef JSON_FOOTER_DEPTH
#define JSON_FOOTER_DEPTH           0                       // 0 = footer is child of R, 1 = footer is child of response object (deprecated)
#endif

#ifndef QUEUE_REPORT_VERBOSITY
#define QUEUE_REPORT_VERBOSITY      QR_OFF                  // one of: QR_OFF, QR_SINGLE, QR_TRIPLE
#endif

#ifndef STATUS_REPORT_VERBOSITY
#define STATUS_REPORT_VERBOSITY     SR_FILTERED             // one of: SR_OFF, SR_FILTERED, SR_VERBOSE
#endif
#ifndef STATUS_REPORT_MIN_MS
#define STATUS_REPORT_MIN_MS        100                     // milliseconds - enforces a viable minimum
#endif
#ifndef STATUS_REPORT_INTERVAL_MS
#define STATUS_REPORT_INTERVAL_MS   250                     // milliseconds - set $SV=0 to disable
#endif
#ifndef STATUS_REPORT_DEFAULTS
#define STATUS_REPORT_DEFAULTS "line","posx","posy","posz","posa","feed","vel","unit","coor","dist","frmo","momo","stat"
#endif

#ifndef GCODE_DEFAULT_UNITS
// Gcode startup defaults
#define GCODE_DEFAULT_UNITS         MILLIMETERS             // MILLIMETERS or INCHES
#endif
#ifndef GCODE_DEFAULT_PLANE
#define GCODE_DEFAULT_PLANE         CANON_PLANE_XY          // CANON_PLANE_XY, CANON_PLANE_XZ, or CANON_PLANE_YZ
#endif
#ifndef GCODE_DEFAULT_COORD_SYSTEM
#define GCODE_DEFAULT_COORD_SYSTEM  G54                     // G54, G55, G56, G57, G58 or G59
#endif
#ifndef GCODE_DEFAULT_PATH_CONTROL
#define GCODE_DEFAULT_PATH_CONTROL  PATH_CONTINUOUS
#endif
#ifndef GCODE_DEFAULT_DISTANCE_MODE
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_MODE
#endif


//*** Input / output settings ***

#ifndef DEFAULT_MODE
#define DEFAULT_MODE                NORMALLY_CLOSED
#define DEFAULT_ACTION              IO_ACTION_NONE
#define DEFAULT_FUNCTION            IO_FUNCTION_NONE
#endif

#ifdef X_SWITCH_MODE_MIN
#warning Please update your settings files! X_SWITCH_MODE_MIN is not longer valid.
#warning Please look in settings.h for DI1_MODE, DI1_ACTION, and DI1_FUNCTION.
#endif

#ifndef DI1_MODE
#define DI1_MODE                    DEFAULT_MODE
#define DI1_ACTION                  DEFAULT_ACTION
#define DI1_FUNCTION                DEFAULT_FUNCTION
#endif

#ifndef DI2_MODE
#define DI2_MODE                    DEFAULT_MODE
#define DI2_ACTION                  DEFAULT_ACTION
#define DI2_FUNCTION                DEFAULT_FUNCTION
#endif

#ifndef DI3_MODE
#define DI3_MODE                    DEFAULT_MODE
#define DI3_ACTION                  DEFAULT_ACTION
#define DI3_FUNCTION                DEFAULT_FUNCTION
#endif

#ifndef DI4_MODE
#define DI4_MODE                    DEFAULT_MODE
#define DI4_ACTION                  DEFAULT_ACTION
#define DI4_FUNCTION                DEFAULT_FUNCTION
#endif

#ifndef DI5_MODE
#define DI5_MODE                    DEFAULT_MODE
#define DI5_ACTION                  DEFAULT_ACTION
#define DI5_FUNCTION                DEFAULT_FUNCTION
#endif

#ifndef DI6_MODE
#define DI6_MODE                    DEFAULT_MODE
#define DI6_ACTION                  DEFAULT_ACTION
#define DI6_FUNCTION                DEFAULT_FUNCTION
#endif

#ifndef DI7_MODE
#define DI7_MODE                    DEFAULT_MODE
#define DI7_ACTION                  DEFAULT_ACTION
#define DI7_FUNCTION                DEFAULT_FUNCTION
#endif

#ifndef DI8_MODE
#define DI8_MODE                    DEFAULT_MODE
#define DI8_ACTION                  DEFAULT_ACTION
#define DI8_FUNCTION                DEFAULT_FUNCTION
#endif

#ifndef DI9_MODE
#define DI9_MODE                    DEFAULT_MODE
#define DI9_ACTION                  DEFAULT_ACTION
#define DI9_FUNCTION                DEFAULT_FUNCTION
#endif


/*** Handle optional modules that may not be in every machine ***/

// If PWM_1 is not defined fill it with default values
#ifndef	P1_PWM_FREQUENCY

#define P1_PWM_FREQUENCY            100                     // in Hz
#define P1_CW_SPEED_LO              1000                    // in RPM (arbitrary units)
#define P1_CW_SPEED_HI              2000
#define P1_CW_PHASE_LO              0.125                   // phase [0..1]
#define P1_CW_PHASE_HI              0.2
#define P1_CCW_SPEED_LO             1000
#define P1_CCW_SPEED_HI             2000
#define P1_CCW_PHASE_LO             0.125
#define P1_CCW_PHASE_HI             0.2
#define P1_PWM_PHASE_OFF            0.1

#endif //P1_PWM_FREQUENCY


// *** DEFAULT COORDINATE SYSTEM OFFSETS ***

#ifndef G54_X_OFFSET
#define G54_X_OFFSET 0	// G54 is often set to all zeros
#define G54_Y_OFFSET 0
#define G54_Z_OFFSET 0
#define G54_A_OFFSET 0
#define G54_B_OFFSET 0
#define G54_C_OFFSET 0
#endif

#ifndef G55_X_OFFSET
#define G55_X_OFFSET (X_TRAVEL_MAX/2)	// set to g55 middle of table
#define G55_Y_OFFSET (Y_TRAVEL_MAX/2)
#define G55_Z_OFFSET 0
#define G55_A_OFFSET 0
#define G55_B_OFFSET 0
#define G55_C_OFFSET 0
#endif

#ifndef G56_X_OFFSET
#define G56_X_OFFSET 0
#define G56_Y_OFFSET 0
#define G56_Z_OFFSET 0
#define G56_A_OFFSET 0
#define G56_B_OFFSET 0
#define G56_C_OFFSET 0
#endif

#ifndef G57_X_OFFSET
#define G57_X_OFFSET 0
#define G57_Y_OFFSET 0
#define G57_Z_OFFSET 0
#define G57_A_OFFSET 0
#define G57_B_OFFSET 0
#define G57_C_OFFSET 0
#endif

#ifndef G58_X_OFFSET
#define G58_X_OFFSET 0
#define G58_Y_OFFSET 0
#define G58_Z_OFFSET 0
#define G58_A_OFFSET 0
#define G58_B_OFFSET 0
#define G58_C_OFFSET 0
#endif

#ifndef G59_X_OFFSET
#define G59_X_OFFSET 0
#define G59_Y_OFFSET 0
#define G59_Z_OFFSET 0
#define G59_A_OFFSET 0
#define G59_B_OFFSET 0
#define G59_C_OFFSET 0
#endif

/*** User-Defined Data Defaults ***/

#ifndef USER_DATA_A0
#define USER_DATA_A0 0
#define USER_DATA_A1 0
#define USER_DATA_A2 0
#define USER_DATA_A3 0
#define USER_DATA_B0 0
#define USER_DATA_B1 0
#define USER_DATA_B2 0
#define USER_DATA_B3 0
#define USER_DATA_C0 0
#define USER_DATA_C1 0
#define USER_DATA_C2 0
#define USER_DATA_C3 0
#define USER_DATA_D0 0
#define USER_DATA_D1 0
#define USER_DATA_D2 0
#define USER_DATA_D3 0
#endif


#endif // End of include guard: SETTINGS_H_ONCE
