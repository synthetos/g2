/*
 * settings_shapeoko375.h - Shopbot Test
 * This file is part of the g2core project
 *
 * Copyright (c) 2015 - 2016 Alden S. Hart, Jr.
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
/**** Shopbot Test profile ******************************************/
/***********************************************************************/
/*
 * NOTES:
 *  - This profile supports the Shopbot sbv300 board
 */

// ***> NOTE: The init message must be a single line with no CRs or LFs
#define INIT_MESSAGE "Initializing configs to Shopbot sbv300 profile"

//**** GLOBAL / GENERAL SETTINGS ******************************************************

#define JUNCTION_INTEGRATION_TIME (5000 * 25.4)  // centripetal acceleration around corners
#define CHORDAL_TOLERANCE 0.001              // chordal accuracy for arc drawing (in mm)

#define SOFT_LIMIT_ENABLE 0         // 0=off, 1=on
#define HARD_LIMIT_ENABLE 1         // 0=off, 1=on
#define SAFETY_INTERLOCK_ENABLE 1   // 0=off, 1=on

#define SPINDLE_ENABLE_POLARITY 1   // 0=active low, 1=active high
#define SPINDLE_DIR_POLARITY 0      // 0=clockwise is low, 1=clockwise is high
#define SPINDLE_PAUSE_ON_HOLD true
#define SPINDLE_DWELL_TIME 1.0

#define COOLANT_MIST_POLARITY 1     // 0=active low, 1=active high
#define COOLANT_FLOOD_POLARITY 1    // 0=active low, 1=active high
#define COOLANT_PAUSE_ON_HOLD true

// Communications and reporting settings

#define USB_SERIAL_PORTS_EXPOSED   1        // Valid options are 1 or 2, only!

#define COMM_MODE JSON_MODE                      // one of: TEXT_MODE, JSON_MODE
#define XIO_ENABLE_FLOW_CONTROL FLOW_CONTROL_RTS // FLOW_CONTROL_OFF, FLOW_CONTROL_RTS

#define TEXT_VERBOSITY TV_VERBOSE           // one of: TV_SILENT, TV_VERBOSE
#define JSON_VERBOSITY JV_MESSAGES          // one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE
#define QUEUE_REPORT_VERBOSITY QR_TRIPLE    // one of: QR_OFF, QR_SINGLE, QR_TRIPLE

#define STATUS_REPORT_VERBOSITY SR_FILTERED  // one of: SR_OFF, SR_FILTERED, SR_VERBOSE
#define STATUS_REPORT_MIN_MS 200             // milliseconds - enforces a viable minimum
#define STATUS_REPORT_INTERVAL_MS 250        // milliseconds - set $SV=0 to disable
#define STATUS_REPORT_DEFAULTS "posx", "posy", "posz", "posa", "posb", "vel", "stat", "hold", "line", "coor"

// Gcode startup defaults
#define GCODE_DEFAULT_UNITS INCHES          // MILLIMETERS or INCHES
#define GCODE_DEFAULT_PLANE CANON_PLANE_XY  // CANON_PLANE_XY, CANON_PLANE_XZ, or CANON_PLANE_YZ
#define GCODE_DEFAULT_COORD_SYSTEM G55      // G54, G55, G56, G57, G58 or G59
#define GCODE_DEFAULT_PATH_CONTROL PATH_CONTINUOUS
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_DISTANCE_MODE

constexpr float H1_DEFAULT_P = 7.0;
constexpr float H1_DEFAULT_I = 0.2;
constexpr float H1_DEFAULT_D = 100.0;

constexpr float H2_DEFAULT_P = 7.0;
constexpr float H2_DEFAULT_I = 0.2;
constexpr float H2_DEFAULT_D = 100.0;

constexpr float H3_DEFAULT_P = 7.0;
constexpr float H3_DEFAULT_I = 0.2;
constexpr float H3_DEFAULT_D = 100.0;

// *** motor settings ************************************************************************************

#define MOTOR_POWER_MODE MOTOR_POWERED_IN_CYCLE  // default motor power mode (see cmMotorPowerMode in stepper.h)
#define MOTOR_POWER_TIMEOUT 2.00                 // motor power timeout in seconds
#define MOTOR_POWER_LEVEL 0.375                  // default motor power level 0.00 - 1.00

#define M1_MOTOR_MAP AXIS_X               // 1ma
#define M1_STEP_ANGLE 1.8                 // 1sa
#define M1_TRAVEL_PER_REV (0.5 * 25.4)    // 1tr
#define M1_MICROSTEPS 10                  // 1mi        1,2,4,8
#define M1_POLARITY 0                     // 1po        0=normal, 1=reversed
#define M1_POWER_MODE MOTOR_POWER_MODE    // 1pm        TRUE=low power idle enabled
#define M1_POWER_LEVEL MOTOR_POWER_LEVEL  // 1pl        Irrelevant to Shopbot sbv300

#define M2_MOTOR_MAP AXIS_Y
#define M2_STEP_ANGLE 1.8
#define M2_TRAVEL_PER_REV (0.5 * 25.4)
#define M2_MICROSTEPS 10
#define M2_POLARITY 0
#define M2_POWER_MODE MOTOR_POWER_MODE
#define M2_POWER_LEVEL MOTOR_POWER_LEVEL

#define M3_MOTOR_MAP AXIS_Z
#define M3_STEP_ANGLE 1.8
#define M3_TRAVEL_PER_REV (0.5 * 25.4)
#define M3_MICROSTEPS 10
#define M3_POLARITY 0
#define M3_POWER_MODE MOTOR_POWER_MODE
#define M3_POWER_LEVEL MOTOR_POWER_LEVEL

#define M4_MOTOR_MAP AXIS_A
#define M4_STEP_ANGLE 1.8
#define M4_TRAVEL_PER_REV (0.5 * 25.4)
#define M4_MICROSTEPS 10
#define M4_POLARITY 0
#define M4_POWER_MODE MOTOR_POWER_MODE
#define M4_POWER_LEVEL MOTOR_POWER_LEVEL

#if (MOTORS >= 5)
#define M5_MOTOR_MAP AXIS_B
#define M5_STEP_ANGLE 1.8
#define M5_TRAVEL_PER_REV (0.5 * 25.4)
#define M5_MICROSTEPS 10
#define M5_POLARITY 0
#define M5_POWER_MODE MOTOR_POWER_MODE
#define M5_POWER_LEVEL MOTOR_POWER_LEVEL

#define M6_MOTOR_MAP AXIS_C
#define M6_STEP_ANGLE 1.8
#define M6_TRAVEL_PER_REV (0.5 * 25.4)
#define M6_MICROSTEPS 10
#define M6_POLARITY 0
#define M6_POWER_MODE MOTOR_POWER_MODE
#define M6_POWER_LEVEL MOTOR_POWER_LEVEL
#endif

#define MANUAL_FEEDRATE_OVERRIDE_ENABLE false
#define MANUAL_FEEDRATE_OVERRIDE_PARAMETER 1.00
// *** axis settings *********************************************************************************

#define X_AXIS_MODE AXIS_STANDARD                // xam    see canonical_machine.h cmAxisMode for valid values
#define X_VELOCITY_MAX (360 * 25.4)              // xvm    G0 max velocity in mm/min
#define X_FEEDRATE_MAX X_VELOCITY_MAX            // xfr     G1 max feed rate in mm/min
#define X_TRAVEL_MIN 0                           // xtn    minimum travel for soft limits
#define X_TRAVEL_MAX (25 * 25.4)                 // xtm    travel between switches or crashes
#define X_JERK_MAX (2 * 25.4)                    // xjm    jerk is multipled by 1,000,000 internally
#define X_JERK_HIGH_SPEED 10000                  // xjh
//#define X_HOMING_INPUT 1                         // xhi  input used for homing or 0 to disable
#define X_HOMING_DIRECTION 0                     // xhd  0=search moves negative, 1= search moves positive
#define X_SEARCH_VELOCITY (60 * 25.4)            // xsv    minus means move to minimum switch
#define X_LATCH_VELOCITY (30 * 25.4)             // xlv    mm/min
#define X_LATCH_BACKOFF (0.125 * 25.4)           // xlb    mm
#define X_ZERO_BACKOFF (0.375 * 25.4)            // xzb    mm
#define X_HOMING_INPUT 0
#define X_HOMING_DIR 0

#define Y_AXIS_MODE AXIS_STANDARD
#define Y_VELOCITY_MAX (360 * 25.4)
#define Y_FEEDRATE_MAX Y_VELOCITY_MAX
#define Y_TRAVEL_MIN 0
#define Y_TRAVEL_MAX (19 * 25.4)
#define Y_JERK_MAX (2 * 25.4)
#define Y_JERK_HIGH_SPEED 10000
#define Y_HOMING_INPUT 3
#define Y_HOMING_DIRECTION 0
#define Y_SEARCH_VELOCITY (60 * 25.4)
#define Y_LATCH_VELOCITY (30 * 25.4)
#define Y_LATCH_BACKOFF (0.125 * 25.4)
#define Y_ZERO_BACKOFF (0.375 * 25.4)
//#define Y_HOMING_INPUT 0
#define Y_HOMING_DIR 0


#define Z_AXIS_MODE AXIS_STANDARD
#define Z_VELOCITY_MAX (360 * 25.4)
#define Z_FEEDRATE_MAX Z_VELOCITY_MAX
#define Z_TRAVEL_MAX (6.5 * 25.4)
#define Z_TRAVEL_MIN 0
#define Z_JERK_MAX (2 * 25.4)
#define Z_JERK_HIGH_SPEED 1000
#define Z_HOMING_INPUT 6
#define Z_HOMING_DIRECTION 1
#define Z_SEARCH_VELOCITY (60 * 25.4)
#define Z_LATCH_VELOCITY (30 * 25.4)
#define Z_LATCH_BACKOFF (0.125 * 25.4)
#define Z_ZERO_BACKOFF (0.375 * 25.4)
//#define Z_HOMING_INPUT 0
#define Z_HOMING_DIR 0


#define A_AXIS_MODE AXIS_STANDARD
#define A_VELOCITY_MAX (360 * 25.4)
#define A_FEEDRATE_MAX 48000
#define A_TRAVEL_MIN -1  // degrees
#define A_TRAVEL_MAX -1  // same value means infinite, no limit
#define A_JERK_MAX (2 * 25.4)
#define A_JERK_HIGH_SPEED A_JERK_MAX
#define A_RADIUS 1.0
#define A_HOMING_INPUT 0
#define A_HOMING_DIRECTION 0
#define A_SEARCH_VELOCITY (60 * 25.4)
#define A_LATCH_VELOCITY (30 * 25.4)
#define A_LATCH_BACKOFF (0.125 * 25.4)
#define A_ZERO_BACKOFF (0.375 * 25.4)
#define A_HOMING_INPUT 0
#define A_HOMING_DIR 0

#define B_AXIS_MODE AXIS_DISABLED
#define B_VELOCITY_MAX (360 * 25.4)
#define B_FEEDRATE_MAX B_VELOCITY_MAX
#define B_TRAVEL_MAX -1
#define B_TRAVEL_MIN -1
#define B_JERK_MAX (2 * 25.4)
#define B_JERK_HIGH_SPEED B_JERK_MAX
#define B_RADIUS 1
#define B_HOMING_INPUT 0
#define B_HOMING_DIRECTION 0
#define B_SEARCH_VELOCITY (60 * 25.4)
#define B_LATCH_VELOCITY (30 * 25.4)
#define B_LATCH_BACKOFF (0.125 * 25.4)
#define B_ZERO_BACKOFF (0.375 * 25.4)
#define B_HOMING_INPUT 0
#define B_HOMING_DIR 0


#define C_AXIS_MODE AXIS_DISABLED
#define C_VELOCITY_MAX (360 * 25.4)
#define C_FEEDRATE_MAX C_VELOCITY_MAX
#define C_TRAVEL_MAX -1
#define C_TRAVEL_MIN -1
#define C_JERK_MAX (2 * 25.4)
#define C_JERK_HIGH_SPEED C_JERK_MAX
#define C_RADIUS 1
#define C_HOMING_INPUT 0
#define C_HOMING_DIRECTION 0
#define C_SEARCH_VELOCITY (60 * 25.4)
#define C_LATCH_VELOCITY (30 * 25.4)
#define C_LATCH_BACKOFF (0.125 * 25.4)
#define C_ZERO_BACKOFF (0.375 * 25.4)
#define C_HOMING_INPUT 0
#define C_HOMING_DIR 0

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
#define DI1_MODE IO_MODE_DISABLED
#define DI1_ACTION INPUT_ACTION_NONE
#define DI1_FUNCTION INPUT_FUNCTION_NONE

// Xmax
#define DI2_MODE IO_MODE_DISABLED
#define DI2_ACTION INPUT_ACTION_NONE
#define DI2_FUNCTION INPUT_FUNCTION_NONE

// Ymin
#define DI3_MODE IO_MODE_DISABLED
#define DI3_ACTION INPUT_ACTION_NONE
#define DI3_FUNCTION INPUT_FUNCTION_NONE

// Ymax
#define DI4_MODE IO_MODE_DISABLED
#define DI4_ACTION INPUT_ACTION_NONE
#define DI4_FUNCTION INPUT_FUNCTION_NONE

// Zmin
#define DI5_MODE IO_MODE_DISABLED
#define DI5_ACTION INPUT_ACTION_NONE
#define DI5_FUNCTION INPUT_FUNCTION_NONE

// Zmax
#define DI6_MODE IO_MODE_DISABLED
#define DI6_ACTION INPUT_ACTION_NONE
#define DI6_FUNCTION INPUT_FUNCTION_NONE

// Amin
#define DI7_MODE IO_MODE_DISABLED
#define DI7_ACTION INPUT_ACTION_NONE
#define DI7_FUNCTION INPUT_FUNCTION_NONE

// Amax
#define DI8_MODE IO_MODE_DISABLED
#define DI8_ACTION INPUT_ACTION_NONE
#define DI8_FUNCTION INPUT_FUNCTION_NONE

// Safety line
#define DI9_MODE IO_MODE_DISABLED
#define DI9_ACTION INPUT_ACTION_NONE
#define DI9_FUNCTION INPUT_FUNCTION_NONE


// Extruder1_PWM
#define DO1_MODE IO_ACTIVE_HIGH
// Extruder2_PWM
#define DO2_MODE IO_ACTIVE_HIGH
// Fan1A_PWM
#define DO3_MODE IO_ACTIVE_HIGH
// Fan1B_PWM
#define DO4_MODE IO_ACTIVE_HIGH
#define DO5_MODE IO_ACTIVE_HIGH
#define DO6_MODE IO_ACTIVE_HIGH
#define DO7_MODE IO_ACTIVE_HIGH
#define DO8_MODE IO_ACTIVE_HIGH
// SAFEin (Output) signal
#define DO9_MODE IO_ACTIVE_HIGH
#define DO10_MODE IO_ACTIVE_HIGH
// Header Bed FET
#define DO11_MODE IO_ACTIVE_HIGH
// Indicator_LED
#define DO12_MODE IO_ACTIVE_HIGH
#define DO13_MODE IO_ACTIVE_HIGH

/*** Handle optional modules that may not be in every machine ***/

#define P1_PWM_FREQUENCY 100  // in Hz
#define P1_CW_SPEED_LO 7900   // in RPM (arbitrary units)
#define P1_CW_SPEED_HI 12800
#define P1_CW_PHASE_LO 0.13  // phase [0..1]
#define P1_CW_PHASE_HI 0.17
#define P1_CCW_SPEED_LO 0
#define P1_CCW_SPEED_HI 0
#define P1_CCW_PHASE_LO 0.1
#define P1_CCW_PHASE_HI 0.1
#define P1_PWM_PHASE_OFF 0.1

// *** DEFAULT COORDINATE SYSTEM OFFSETS ***
// Our convention is:
//    - leave G54 in machine coordinates to act as a persistent absolute coordinate system
//    - set G55 to be a zero in the middle of the table
//    - no action for the others

#define G54_X_OFFSET 0  // G54 is traditionally set to all zeros
#define G54_Y_OFFSET 0
#define G54_Z_OFFSET 0
#define G54_A_OFFSET 0
#define G54_B_OFFSET 0
#define G54_C_OFFSET 0

#define G55_X_OFFSET 0  // use (X_TRAVEL_MAX/2) to set g55 to middle of table
#define G55_Y_OFFSET 0  // use (Y_TRAVEL_MAX/2) to set g55 to middle of table
#define G55_Z_OFFSET 0
#define G55_A_OFFSET 0
#define G55_B_OFFSET 0
#define G55_C_OFFSET 0

#define G56_X_OFFSET 0
#define G56_Y_OFFSET 0
#define G56_Z_OFFSET 0
#define G56_A_OFFSET 0
#define G56_B_OFFSET 0
#define G56_C_OFFSET 0

#define G57_X_OFFSET 0
#define G57_Y_OFFSET 0
#define G57_Z_OFFSET 0
#define G57_A_OFFSET 0
#define G57_B_OFFSET 0
#define G57_C_OFFSET 0

#define G58_X_OFFSET 0
#define G58_Y_OFFSET 0
#define G58_Z_OFFSET 0
#define G58_A_OFFSET 0
#define G58_B_OFFSET 0
#define G58_C_OFFSET 0

#define G59_X_OFFSET 0
#define G59_Y_OFFSET 0
#define G59_Z_OFFSET 0
#define G59_A_OFFSET 0
#define G59_B_OFFSET 0
#define G59_C_OFFSET 0

/*** User-Defined Data Defaults ***/

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
