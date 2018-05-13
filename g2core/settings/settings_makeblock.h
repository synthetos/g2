/*
 * settings_makeblock.h - makeblock engraving table
 * This file is part of the g2core project
 *
 * Copyright (c) 2016 - 2018 Alden S. Hart, Jr.
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
/**** Makeblock XY Plotter profile *************************************/
/***********************************************************************/

// ***> NOTE: The init message must be a single line with no CRs or LFs
#define INIT_MESSAGE "Initializing configs to Makeblock profile"

// Machine configuration settings

#define JUNCTION_INTEGRATION_TIME   1.25    // cornering - between 0.10 and 2.00 (higher is faster)
#define CHORDAL_TOLERANCE           0.1     // chordal tolerance for arcs (in mm)

#define SOFT_LIMIT_ENABLE           0       // 0=off, 1=on
#define HARD_LIMIT_ENABLE           0       // 0=off, 1=on
#define SAFETY_INTERLOCK_ENABLE     1       // 0=off, 1=on

#define SPINDLE_ENABLE_POLARITY     1       // 0=active low, 1=active high
#define SPINDLE_DIR_POLARITY        0       // 0=clockwise is low, 1=clockwise is high
#define SPINDLE_PAUSE_ON_HOLD       true
#define SPINDLE_SPINUP_DELAY        2.0
#define SPINDLE_SPEED_MIN           0.0 
#define SPINDLE_SPEED_MAX       20000.0 

#define COOLANT_MIST_POLARITY       1       // 0=active low, 1=active high
#define COOLANT_FLOOD_POLARITY      1       // 0=active low, 1=active high
#define COOLANT_PAUSE_ON_HOLD       true

#define FEEDHOLD_Z_LIFT             10      // mm to lift Z on feedhold 
#define PROBE_REPORT_ENABLE         true

// Communications and reporting settings

#define USB_SERIAL_PORTS_EXPOSED    1           // Valid options are 1 or 2, only!
#define MARLIN_COMPAT_ENABLED       false       // enable marlin compatibility mode
#define COMM_MODE                   JSON_MODE   // one of: TEXT_MODE, JSON_MODE

#define TEXT_VERBOSITY              TV_VERBOSE  // one of: TV_SILENT, TV_VERBOSE
#define JSON_VERBOSITY              JV_MESSAGES // one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE
#define QUEUE_REPORT_VERBOSITY      QR_OFF      // one of: QR_OFF, QR_SINGLE, QR_TRIPLE

#define STATUS_REPORT_VERBOSITY     SR_FILTERED // one of: SR_OFF, SR_FILTERED, SR_VERBOSE
#define STATUS_REPORT_MIN_MS        200         // milliseconds - enforces a viable minimum
#define STATUS_REPORT_INTERVAL_MS   250         // milliseconds - set $SV=0 to disable

#define STATUS_REPORT_DEFAULTS      "line","stat",\
                                    "posx","posy","posz",\
                                    "posu","posv","posw",\
                                    "vel", "unit","feed","coor","momo",\
                                    "plan","path","dist","prbe","prbz",\
                                    "mpox","mpoy","mpoz",\
                                    "mpou","mpov","mpow",\
                                    "admo","frmo","cycs","hold"
//                                     "_ts1","_cs1","_es1","_xs1","_fe1"

// Gcode startup defaults
#define GCODE_DEFAULT_UNITS             MILLIMETERS         // MILLIMETERS or INCHES
#define GCODE_DEFAULT_PLANE CANON_PLANE_XY                  // CANON_PLANE_XY, CANON_PLANE_XZ, or CANON_PLANE_YZ
#define GCODE_DEFAULT_COORD_SYSTEM      G54                 // G54, G55, G56, G57, G58 or G59
#define GCODE_DEFAULT_PATH_CONTROL      PATH_CONTINUOUS
#define GCODE_DEFAULT_DISTANCE_MODE     ABSOLUTE_DISTANCE_MODE

// *** motor settings ************************************************************************************

#define MOTOR_POWER_LEVEL       0.2                     // power level: 0.0=no power, 1.0=max power
#define MOTOR_POWER_MODE        MOTOR_POWERED_IN_CYCLE  // default motor power mode (see stPowerMode in stepper.h)
                                                        // 0=MOTOR_DISABLED, 
                                                        // 1=MOTOR_ALWAYS_POWERED, 
                                                        // 2=MOTOR_POWERED_IN_CYCLE, 
                                                        // 3=MOTOR_POWERED_ONLY_WHEN_MOVING
#define MOTOR_POWER_TIMEOUT     5.00                    // motor power timeout in seconds

#define M1_MOTOR_MAP            AXIS_X_EXTERNAL         // 1ma
#define M1_STEP_ANGLE           1.8                     // 1sa
#define M1_TRAVEL_PER_REV       36.576                  // 1tr  2.032mm pitch * 18 teeth per revolution
#define M1_MICROSTEPS           8                       // 1mi  1,2,4,8,16,32
#define M1_POLARITY             0                       // 1po  0=normal, 1=reversed
#define M1_POWER_MODE           MOTOR_POWER_MODE        // 1pm  0=MOTOR_DISABLED, 1=MOTOR_ALWAYS_POWERED, 2=MOTOR_POWERED_IN_CYCLE, 3=MOTOR_POWERED_ONLY_WHEN_MOVING
#define M1_POWER_LEVEL          MOTOR_POWER_LEVEL       // 1pl:   0.0=no power, 1.0=max power

#define M2_MOTOR_MAP            AXIS_Y_EXTERNAL
#define M2_STEP_ANGLE           1.8
#define M2_TRAVEL_PER_REV       36.576
#define M2_MICROSTEPS           8
#define M2_POLARITY             0
#define M2_POWER_MODE           MOTOR_POWER_MODE
#define M2_POWER_LEVEL          MOTOR_POWER_LEVEL

#define M3_MOTOR_MAP            AXIS_Z_EXTERNAL  // Imaginary Z axis. For testing
#define M3_STEP_ANGLE           1.8
#define M3_TRAVEL_PER_REV       1.25
#define M3_MICROSTEPS           8
#define M3_POLARITY             0
#define M3_POWER_MODE           MOTOR_POWER_MODE
#define M3_POWER_LEVEL          MOTOR_POWER_LEVEL

#define M4_MOTOR_MAP            AXIS_W_EXTERNAL  // Imaginary W axis. For testing
#define M4_STEP_ANGLE           1.8
#define M4_TRAVEL_PER_REV       1.25
#define M4_MICROSTEPS           8
#define M4_POLARITY             1
#define M4_POWER_MODE           MOTOR_POWER_MODE
#define M4_POWER_LEVEL          MOTOR_POWER_LEVEL

// *** axis settings **********************************************************************************

#define JERK_MAX 20000

#define X_AXIS_MODE             AXIS_STANDARD           // xam  see canonical_machine.h cmAxisMode for valid values
#define X_VELOCITY_MAX          40000                   // xvm  G0 max velocity in mm/min
#define X_FEEDRATE_MAX          X_VELOCITY_MAX          // xfr  G1 max feed rate in mm/min
#define X_TRAVEL_MIN            0                       // xtn  minimum travel for soft limits
#define X_TRAVEL_MAX            420                     // xtm  travel between switches or crashes
#define X_JERK_MAX              JERK_MAX                // xjm  jerk * 1,000,000
#define X_JERK_HIGH_SPEED       20000                   // xjh
#define X_HOMING_INPUT          1                       // xhi  input used for homing or 0 to disable
#define X_HOMING_DIRECTION      0                       // xhd  0=search moves negative, 1= search moves positive
#define X_SEARCH_VELOCITY       3000                    // xsv  minus means move to minimum switch
#define X_LATCH_VELOCITY        100                     // xlv  mm/min
#define X_LATCH_BACKOFF         4                       // xlb  mm
#define X_ZERO_BACKOFF          2                       // xzb  mm

#define Y_AXIS_MODE             AXIS_STANDARD
#define Y_VELOCITY_MAX          40000
#define Y_FEEDRATE_MAX          Y_VELOCITY_MAX
#define Y_TRAVEL_MIN            0
#define Y_TRAVEL_MAX            420
#define Y_JERK_MAX              JERK_MAX
#define Y_JERK_HIGH_SPEED       20000
#define Y_HOMING_INPUT          3
#define Y_HOMING_DIRECTION      0
#define Y_SEARCH_VELOCITY       3000
#define Y_LATCH_VELOCITY        100
#define Y_LATCH_BACKOFF         4
#define Y_ZERO_BACKOFF          2

#define Z_AXIS_MODE             AXIS_STANDARD
#define Z_VELOCITY_MAX          1200
#define Z_FEEDRATE_MAX          Z_VELOCITY_MAX
#define Z_TRAVEL_MAX            0
#define Z_TRAVEL_MIN            -95
#define Z_JERK_MAX              500
#define Z_JERK_HIGH_SPEED       1000
#define Z_HOMING_INPUT          6
#define Z_HOMING_DIRECTION      1
#define Z_SEARCH_VELOCITY       (Z_VELOCITY_MAX * 0.66666)
#define Z_LATCH_VELOCITY        25
#define Z_LATCH_BACKOFF         4
#define Z_ZERO_BACKOFF          2

#define U_AXIS_MODE             AXIS_STANDARD           // xam  see canonical_machine.h cmAxisMode for valid values
#define U_VELOCITY_MAX          40000                   // xvm  G0 max velocity in mm/min
#define U_FEEDRATE_MAX          U_VELOCITY_MAX          // xfr  G1 max feed rate in mm/min
#define U_TRAVEL_MIN            0                       // xtn  minimum travel for soft limits
#define U_TRAVEL_MAX            420                     // xtm  travel between switches or crashes
#define U_JERK_MAX              JERK_MAX                // xjm  jerk * 1,000,000
#define U_JERK_HIGH_SPEED       20000                   // xjh
#define U_HOMING_INPUT          1                       // xhi  input used for homing or 0 to disable
#define U_HOMING_DIRECTION      0                       // xhd  0=search moves negative, 1= search moves positive
#define U_SEARCH_VELOCITY       3000                    // xsv  minus means move to minimum switch
#define U_LATCH_VELOCITY        100                     // xlv  mm/min
#define U_LATCH_BACKOFF         4                       // xlb  mm
#define U_ZERO_BACKOFF          2                       // xzb  mm

#define V_AXIS_MODE             AXIS_STANDARD
#define V_VELOCITY_MAX          40000
#define V_FEEDRATE_MAX          V_VELOCITY_MAX
#define V_TRAVEL_MIN            0
#define V_TRAVEL_MAX            420
#define V_JERK_MAX              JERK_MAX
#define V_JERK_HIGH_SPEED       20000
#define V_HOMING_INPUT          3
#define V_HOMING_DIRECTION      0
#define V_SEARCH_VELOCITY       3000
#define V_LATCH_VELOCITY        100
#define V_LATCH_BACKOFF         4
#define V_ZERO_BACKOFF          2

#define W_AXIS_MODE             AXIS_STANDARD
#define W_VELOCITY_MAX          1200
#define W_FEEDRATE_MAX          W_VELOCITY_MAX
#define W_TRAVEL_MAX            0
#define W_TRAVEL_MIN            -95
#define W_JERK_MAX              500
#define W_JERK_HIGH_SPEED       1000
#define W_HOMING_INPUT          6
#define W_HOMING_DIRECTION      1
#define W_SEARCH_VELOCITY       (W_VELOCITY_MAX * 0.66666)
#define W_LATCH_VELOCITY        25
#define W_LATCH_BACKOFF         4
#define W_ZERO_BACKOFF          2

#define A_AXIS_MODE             AXIS_STANDARD
#define A_RADIUS                1
#define A_VELOCITY_MAX          360000
#define A_FEEDRATE_MAX          A_VELOCITY_MAX
#define A_TRAVEL_MIN            -1
#define A_TRAVEL_MAX            -1
#define A_JERK_MAX              100000
#define A_JERK_HIGH_SPEED       A_JERK_MAX
#define A_HOMING_INPUT          0
#define A_HOMING_DIRECTION      0
#define A_SEARCH_VELOCITY       600
#define A_LATCH_VELOCITY        100
#define A_LATCH_BACKOFF         10
#define A_ZERO_BACKOFF          2

#define B_AXIS_MODE             AXIS_STANDARD
#define B_RADIUS                1
#define B_VELOCITY_MAX          360000
#define B_FEEDRATE_MAX          B_VELOCITY_MAX
#define B_TRAVEL_MIN            -1
#define B_TRAVEL_MAX            -1
#define B_JERK_MAX              100000
#define B_JERK_HIGH_SPEED       B_JERK_MAX
#define B_HOMING_INPUT          0
#define B_HOMING_DIRECTION      0
#define B_SEARCH_VELOCITY       600
#define B_LATCH_VELOCITY        100
#define B_LATCH_BACKOFF         10
#define B_ZERO_BACKOFF          2

#define C_AXIS_MODE             AXIS_STANDARD
#define C_RADIUS                1
#define C_VELOCITY_MAX          360000
#define C_FEEDRATE_MAX          C_VELOCITY_MAX
#define C_TRAVEL_MIN            -1
#define C_TRAVEL_MAX            -1
#define C_JERK_MAX              100000
#define C_JERK_HIGH_SPEED       C_JERK_MAX
#define C_HOMING_INPUT          0
#define C_HOMING_DIRECTION      0
#define C_SEARCH_VELOCITY       600
#define C_LATCH_VELOCITY        100
#define C_LATCH_BACKOFF         10
#define C_ZERO_BACKOFF          2

//*** Input / output settings ***
/*
    See gpio.h GPIO defines for options

    Homing and probing settings are independent of ACTION and FUNCTION settings
    but rely on proper switch MODE setting (i.e. NC or NO)

    INPUT_MODE_DISABLED
    INPUT_ACTIVE_LOW    aka NORMALLY_OPEN
    INPUT_ACTIVE_HIGH   aka NORMALLY_CLOSED

    INPUT_ACTION_NONE
    INPUT_ACTION_STOP
    INPUT_ACTION_FAST_STOP
    INPUT_ACTION_HALT
    INPUT_ACTION_RESET

    INPUT_FUNCTION_NONE
    INPUT_FUNCTION_LIMIT
    INPUT_FUNCTION_INTERLOCK
    INPUT_FUNCTION_SHUTDOWN
    INPUT_FUNCTION_PROBE
*/

// Xmin on v9 board                         // X homing - see X axis setup
#define DI1_MODE        NORMALLY_CLOSED
#define DI1_ACTION      INPUT_ACTION_NONE
#define DI1_FUNCTION    INPUT_FUNCTION_NONE

// Xmax                                     // External ESTOP
#define DI2_MODE        IO_ACTIVE_HIGH
#define DI2_ACTION      INPUT_ACTION_HALT
#define DI2_FUNCTION    INPUT_FUNCTION_SHUTDOWN

// Ymin                                     // Y homing - see Y axis setup
#define DI3_MODE        NORMALLY_CLOSED
#define DI3_ACTION      INPUT_ACTION_NONE
#define DI3_FUNCTION    INPUT_FUNCTION_NONE

// Ymax                                     // Safety interlock
#define DI4_MODE        IO_ACTIVE_HIGH
#define DI4_ACTION      INPUT_ACTION_NONE  // (hold is performed by Interlock function)
#define DI4_FUNCTION    INPUT_FUNCTION_INTERLOCK

// Zmin                                     // Z probe
#define DI5_MODE        IO_ACTIVE_LOW
#define DI5_ACTION      INPUT_ACTION_NONE
#define DI5_FUNCTION    INPUT_FUNCTION_PROBE

// Zmax                                     // Z homing - see Z axis for setup
#define DI6_MODE        NORMALLY_CLOSED
#define DI6_ACTION      INPUT_ACTION_NONE
#define DI6_FUNCTION    INPUT_FUNCTION_NONE

// Amin                                     // Unused
#define DI7_MODE        IO_MODE_DISABLED
#define DI7_ACTION      INPUT_ACTION_NONE
#define DI7_FUNCTION    INPUT_FUNCTION_NONE

// Amax                                     // Unused
#define DI8_MODE        IO_MODE_DISABLED
#define DI8_ACTION      INPUT_ACTION_NONE
#define DI8_FUNCTION    INPUT_FUNCTION_NONE

// Safety line w/HW timer                   // Unused
#define DI9_MODE        IO_MODE_DISABLED
#define DI9_ACTION      INPUT_ACTION_NONE
#define DI9_FUNCTION    INPUT_FUNCTION_NONE
