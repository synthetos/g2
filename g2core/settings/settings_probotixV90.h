/*
 * settings_probotix.h - Probotix Fireball V90 machine profile
 * This file is part the g2core project
 *
 * Copyright (c) 2011 - 2016 Alden S. Hart, Jr.
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

/***********************************************************************/
/**** Probotix Fireball V90 profile ************************************/
/***********************************************************************/

// ***> NOTE: The init message must be a single line with no CRs or LFs
#define INIT_MESSAGE "Initializing configs to Probotix Fireball V90 profile"


//**** GLOBAL / GENERAL SETTINGS ******************************************************

#define JUNCTION_INTEGRATION_TIME   1.50    // cornering - between 0.10 and 2.00 (higher is faster)
#define CHORDAL_TOLERANCE           0.01    // chordal accuracy for arc drawing (in mm)

#define SOFT_LIMIT_ENABLE           0       // 0=off, 1=on
#define HARD_LIMIT_ENABLE           1       // 0=off, 1=on
#define SAFETY_INTERLOCK_ENABLE     1       // 0=off, 1=on

#define SPINDLE_ENABLE_POLARITY     1       // 0=active low, 1=active high
#define SPINDLE_DIR_POLARITY        0       // 0=clockwise is low, 1=clockwise is high
#define SPINDLE_PAUSE_ON_HOLD       true
#define SPINDLE_DWELL_TIME          1.5     // after unpausing and turning the spindle on, dwell for 1.5s

//#define ESC_BOOT_TIME               5000    // how long the ESC takes to boot, in milliseconds
//#define ESC_LOCKOUT_TIME            900     // how long the interlock needs to be engaged before killing power... actually 1s, but be conservative

#define COOLANT_MIST_POLARITY       1       // 0=active low, 1=active high
#define COOLANT_FLOOD_POLARITY      1       // 0=active low, 1=active high
#define COOLANT_PAUSE_ON_HOLD       true

// Communications and reporting settings

#define TEXT_VERBOSITY              TV_VERBOSE              // one of: TV_SILENT, TV_VERBOSE
#define COMM_MODE                   JSON_MODE               // one of: TEXT_MODE, JSON_MODE

#define COM_EXPAND_CR               false
#define COM_ENABLE_ECHO             false
#define COM_ENABLE_FLOW_CONTROL     FLOW_CONTROL_XON        // FLOW_CONTROL_OFF, FLOW_CONTROL_XON, FLOW_CONTROL_RTS

#define JSON_VERBOSITY              JV_MESSAGES             // one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE
#define JSON_SYNTAX_MODE            JSON_SYNTAX_STRICT      // one of JSON_SYNTAX_RELAXED, JSON_SYNTAX_STRICT

#define QUEUE_REPORT_VERBOSITY      QR_OFF                  // one of: QR_OFF, QR_SINGLE, QR_TRIPLE

#define STATUS_REPORT_VERBOSITY     SR_FILTERED             // one of: SR_OFF, SR_FILTERED, SR_VERBOSE
#define STATUS_REPORT_MIN_MS        100                     // milliseconds - enforces a viable minimum
#define STATUS_REPORT_INTERVAL_MS   250                     // milliseconds - set $SV=0 to disable
#define STATUS_REPORT_DEFAULTS      "posx","posy","posz","posa","line","vel","feed","stat",\
                                    "macs","cycs","mots","hold","dist","admo"
                                    //,"in1","in2","in3","in4","in5","in6","in7","in8","in9"
                                    //,"home","homx","homy","homz"

// Gcode startup defaults
#define GCODE_DEFAULT_UNITS         MILLIMETERS             // MILLIMETERS or INCHES
#define GCODE_DEFAULT_PLANE         CANON_PLANE_XY          // CANON_PLANE_XY, CANON_PLANE_XZ, or CANON_PLANE_YZ
#define GCODE_DEFAULT_COORD_SYSTEM  G54                     // G54, G55, G56, G57, G58 or G59
#define GCODE_DEFAULT_PATH_CONTROL  PATH_CONTINUOUS
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_MODE

// *** motor settings ************************************************************************************

#define MOTOR_POWER_MODE            MOTOR_POWERED_IN_CYCLE  // default motor power mode (see cmMotorPowerMode in stepper.h)
#define MOTOR_POWER_TIMEOUT         2.00                    // motor power timeout in seconds

#define M1_MOTOR_MAP                AXIS_X                  // 1ma
#define M1_STEP_ANGLE               1.8                     // 1sa
#define M1_TRAVEL_PER_REV           5.08                    // 1tr
#define M1_MICROSTEPS               8                       // 1mi        1,2,4,8
#define M1_POLARITY                 1                       // 1po        0=normal, 1=reversed
#define M1_POWER_MODE               MOTOR_POWER_MODE        // 1pm        standard
#define M1_POWER_LEVEL              0.75

#define M2_MOTOR_MAP                AXIS_Y
#define M2_STEP_ANGLE               1.8
#define M2_TRAVEL_PER_REV           5.08
#define M2_MICROSTEPS               8
#define M2_POLARITY                 0
#define M2_POWER_MODE               MOTOR_POWER_MODE
#define M2_POWER_LEVEL              0.75

#define M3_MOTOR_MAP                AXIS_Z
#define M3_STEP_ANGLE               1.8
#define M3_TRAVEL_PER_REV           2.1166666
#define M3_MICROSTEPS               8
#define M3_POLARITY                 1
#define M3_POWER_MODE               MOTOR_POWER_MODE
#define M3_POWER_LEVEL              0.50

// *** axis settings **********************************************************************************

#define JERK_MAX    500

#define X_AXIS_MODE                 AXIS_STANDARD           // xam  see canonical_machine.h cmAxisMode for valid values
#define X_VELOCITY_MAX              1600                    // xvm  G0 max velocity in mm/min
#define X_FEEDRATE_MAX              X_VELOCITY_MAX          // xfr  G1 max feed rate in mm/min
#define X_TRAVEL_MIN                0                       // xtn  minimum travel - used by soft limits and homing
#define X_TRAVEL_MAX                400                     // xtm  maximum travel - used by soft limits and homing
#define X_JERK_MAX                  JERK_MAX                // xjm
#define X_JERK_HIGH_SPEED           X_JERK_MAX              // xjh
#define X_HOMING_INPUT              1                       // xhi  input used for homing or 0 to disable
#define X_HOMING_DIRECTION          0                       // xhd  0=search moves negative, 1= search moves positive
#define X_SEARCH_VELOCITY           1000                    // xsv  move in negative direction
#define X_LATCH_VELOCITY            100                     // xlv  mm/min
#define X_LATCH_BACKOFF             10                      // xlb  mm
#define X_ZERO_BACKOFF               2                      // xzb  mm

#define Y_AXIS_MODE                 AXIS_STANDARD
#define Y_VELOCITY_MAX              1600
#define Y_FEEDRATE_MAX              Y_VELOCITY_MAX
#define Y_TRAVEL_MIN                0
#define Y_TRAVEL_MAX                175
#define Y_JERK_MAX                  JERK_MAX
#define Y_JERK_HIGH_SPEED           Y_JERK_MAX
#define Y_HOMING_INPUT              3
#define Y_HOMING_DIRECTION          0
#define Y_SEARCH_VELOCITY           1000
#define Y_LATCH_VELOCITY            100
#define Y_LATCH_BACKOFF             10
#define Y_ZERO_BACKOFF              2

#define Z_AXIS_MODE                 AXIS_STANDARD
#define Z_VELOCITY_MAX              1000
#define Z_FEEDRATE_MAX              Z_VELOCITY_MAX
#define Z_TRAVEL_MIN                0
#define Z_TRAVEL_MAX                75
#define Z_JERK_MAX                  JERK_MAX
#define Z_JERK_HIGH_SPEED           Z_JERK_MAX
#define Z_HOMING_INPUT              6
#define Z_HOMING_DIRECTION          1
#define Z_SEARCH_VELOCITY           600
#define Z_LATCH_VELOCITY            100
#define Z_LATCH_BACKOFF             10
#define Z_ZERO_BACKOFF              2

//*** Input / output settings ***

/*
    IO_MODE_DISABLED
    IO_ACTIVE_LOW    aka NORMALLY_OPEN
    IO_ACTIVE_HIGH   aka NORMALLY_CLOSED

    INPUT_ACTION_NONE
    INPUT_ACTION_STOP
    INPUT_ACTION_FAST_STOP
    INPUT_ACTION_HALT
    INPUT_ACTION_RESET

    INPUT_FUNCTION_NONE
    INPUT_FUNCTION_LIMIT
    INPUT_FUNCTION_INTERLOCK
    INPUT_FUNCTION_SHUTDOWN
    INPUT_FUNCTION_PANIC
*/

// Xmin on v9 board
#define DI1_MODE                    IO_MODE_DISABLED
#define DI1_ACTION                  INPUT_ACTION_NONE
#define DI1_FUNCTION                INPUT_FUNCTION_NONE

// Xmax
#define DI2_MODE                    IO_MODE_DISABLED
#define DI2_ACTION                  INPUT_ACTION_NONE
#define DI2_FUNCTION                INPUT_FUNCTION_NONE

// Ymin
#define DI3_MODE                    IO_MODE_DISABLED
#define DI3_ACTION                  INPUT_ACTION_NONE
#define DI3_FUNCTION                INPUT_FUNCTION_NONE

// Ymax
#define DI4_MODE                    IO_MODE_DISABLED
#define DI4_ACTION                  INPUT_ACTION_NONE
#define DI4_FUNCTION                INPUT_FUNCTION_NONE

// Zmin
#define DI5_MODE                    IO_ACTIVE_LOW
#define DI5_ACTION                  INPUT_ACTION_NONE
#define DI5_FUNCTION                INPUT_FUNCTION_NONE

// Zmax
#define DI6_MODE                    IO_MODE_DISABLED
#define DI6_ACTION                  INPUT_ACTION_NONE
#define DI6_FUNCTION                INPUT_FUNCTION_NONE

// Amin
#define DI7_MODE                    IO_MODE_DISABLED
#define DI7_ACTION                  INPUT_ACTION_NONE
#define DI7_FUNCTION                INPUT_FUNCTION_NONE

// Amax
#define DI8_MODE                    IO_MODE_DISABLED
#define DI8_ACTION                  INPUT_ACTION_NONE
#define DI8_FUNCTION                INPUT_FUNCTION_NONE

// Safety line
#define DI9_MODE                    IO_MODE_DISABLED
#define DI9_ACTION                  INPUT_ACTION_NONE
#define DI9_FUNCTION                INPUT_FUNCTION_NONE
