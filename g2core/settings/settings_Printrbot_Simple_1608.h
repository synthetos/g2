/*
 * settings_printrbot_simple_1608.h - New Simple, 2016 version
 * This file is part of the the g2core project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart, Jr.
 * Copyright (c) 2010 - 2016 Robert Giseburt
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
/**** Printrbot Simple profile *******************************************/
/***********************************************************************/

// ***> NOTE: The init message must be a single line with no CRs or LFs
#define INIT_MESSAGE "Initializing configs to Printrbot Simple 1608 profile"

#ifndef PI
#define PI 3.14159628
#endif

//**** GLOBAL / GENERAL SETTINGS ******************************************************

#define JUNCTION_INTEGRATION_TIME   1.15                    // cornering - between 0.10 and 2.00 (higher is faster)
#define CHORDAL_TOLERANCE           0.01                    // chordal accuracy for arc drawing (in mm)

#define SOFT_LIMIT_ENABLE           0                       // 0=off, 1=on
#define HARD_LIMIT_ENABLE           1                       // 0=off, 1=on
#define SAFETY_INTERLOCK_ENABLE     1                       // 0=off, 1=on

#define SPINDLE_ENABLE_POLARITY     1                       // 0=active low, 1=active high
#define SPINDLE_DIR_POLARITY        0                       // 0=clockwise is low, 1=clockwise is high
#define SPINDLE_PAUSE_ON_HOLD       true
#define SPINDLE_DWELL_TIME          1.0

#define COOLANT_MIST_POLARITY       1                       // 0=active low, 1=active high
#define COOLANT_FLOOD_POLARITY      1                       // 0=active low, 1=active high
#define COOLANT_PAUSE_ON_HOLD       false

// Communications and reporting settings

#define COMM_MODE                   JSON_MODE               // one of: TEXT_MODE, JSON_MODE
#define XIO_ENABLE_FLOW_CONTROL FLOW_CONTROL_RTS            // FLOW_CONTROL_OFF, FLOW_CONTROL_RTS
#define XIO_UART_MUTES_WHEN_USB_CONNECTED 1                 // Mute the UART when USB connects

#define TEXT_VERBOSITY              TV_VERBOSE              // one of: TV_SILENT, TV_VERBOSE
#define JSON_VERBOSITY              JV_LINENUM              // one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE
#define QUEUE_REPORT_VERBOSITY      QR_OFF                  // one of: QR_OFF, QR_SINGLE, QR_TRIPLE

#define STATUS_REPORT_VERBOSITY     SR_FILTERED             // one of: SR_OFF, SR_FILTERED, SR_VERBOSE
#define STATUS_REPORT_MIN_MS        100                     // milliseconds - enforces a viable minimum
#define STATUS_REPORT_INTERVAL_MS   250                     // milliseconds - set $SV=0 to disable

// Defaults for 3DP
#define STATUS_REPORT_DEFAULTS    "line","posx","posy","posz","posa","vel","he1t","he1st","he1at","feed","unit","path","stat"
// There are no heater two or three, but these would show those: ,"he2t","he2st","he2at","he3t","he3st","he3at"

// Defaults for motion debugging
//#define STATUS_REPORT_DEFAULTS "line","posx","posy","posz","posa","he1t","he1st","he1at","he2t","he2st","he2at","he3t","he3st","he3at","_fe5","_fe4","feed","vel","unit","path","stat"

// Defaults for PID tuning
//#define STATUS_REPORT_DEFAULTS "line","posx","posy","posz","posa","he1t","he1st","he1at","he1op","pid1p","pid1i","pid1d","feed","vel","unit","path","stat"

// Gcode startup defaults
#define GCODE_DEFAULT_UNITS         MILLIMETERS             // MILLIMETERS or INCHES
#define GCODE_DEFAULT_PLANE         CANON_PLANE_XY          // CANON_PLANE_XY, CANON_PLANE_XZ, or CANON_PLANE_YZ
#define GCODE_DEFAULT_COORD_SYSTEM  G54                     // G54, G55, G56, G57, G58 or G59
#define GCODE_DEFAULT_PATH_CONTROL  PATH_CONTINUOUS
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_MODE


// *** motor settings ************************************************************************************

#define MOTOR_POWER_TIMEOUT         30 // don't disable motors (without an explicit {md:0}) for 30 seconds

#define MOTOR_POWER_MODE            MOTOR_POWERED_IN_CYCLE  // default motor power mode (see cmMotorPowerMode in stepper.h)
// 80 steps/mm at 1/16 microstepping = 40 mm/rev
#define M1_MOTOR_MAP                AXIS_X                  // 1ma
#define M1_STEP_ANGLE               1.8                     // 1sa
#define M1_TRAVEL_PER_REV           40.011604                   // 1tr
#define M1_MICROSTEPS               32                      // 1mi        1,2,4,8,16,32
#define M1_POLARITY                 0                       // 1po        0=normal, 1=reversed
#define M1_POWER_MODE               MOTOR_POWER_MODE        // 1pm        standard
#define M1_POWER_LEVEL              0.35                     // 1mp

// 80 steps/mm at 1/16 microstepping = 40 mm/rev
#define M3_MOTOR_MAP                AXIS_Y
#define M3_STEP_ANGLE               1.8
#define M3_TRAVEL_PER_REV           40.011604
#define M3_MICROSTEPS               32
#define M3_POLARITY                 0
#define M3_POWER_MODE               MOTOR_POWER_MODE
#define M3_POWER_LEVEL              0.35

// 2020 steps/mm at 1/16 microstepping = 1.58416 mm/rev
#define M2_MOTOR_MAP                AXIS_Z
#define M2_STEP_ANGLE               1.8
#define M2_TRAVEL_PER_REV           8
#define M2_MICROSTEPS               32
#define M2_POLARITY                 1
#define M2_POWER_MODE               MOTOR_POWER_MODE
#define M2_POWER_LEVEL              0.3

// 96 steps/mm at 1/16 microstepping = 33.3333 mm/rev
#define M4_MOTOR_MAP                AXIS_A
#define M4_STEP_ANGLE               1.8
#define M4_TRAVEL_PER_REV           360            // degrees moved per motor rev
#define M4_MICROSTEPS               32
#define M4_POLARITY                 1
#define M4_POWER_MODE               MOTOR_POWER_MODE
#define M4_POWER_LEVEL              0.3

// 96 steps/mm at 1/16 microstepping = 33.3333 mm/rev
#define M5_MOTOR_MAP                AXIS_B
#define M5_STEP_ANGLE               1.8
#define M5_TRAVEL_PER_REV           360            // degrees moved per motor rev
#define M5_MICROSTEPS               32
#define M5_POLARITY                 0
#define M5_POWER_MODE               MOTOR_POWER_MODE
#define M5_POWER_LEVEL              0.3

// *** axis settings **********************************************************************************

#define X_AXIS_MODE                 AXIS_STANDARD           // xam  see canonical_machine.h cmAxisMode for valid values
#define X_VELOCITY_MAX              15000                   // xvm  G0 max velocity in mm/min
#define X_FEEDRATE_MAX              X_VELOCITY_MAX          // xfr  G1 max feed rate in mm/min
#define X_TRAVEL_MIN                0                       // xtn  minimum travel - used by soft limits and homing
#define X_TRAVEL_MAX                215                     // xtm  travel between switches or crashes
#define X_JERK_MAX                  6000                    // xjm  yes, that's "100 billion" mm/(min^3)
#define X_JERK_HIGH_SPEED           6000                    // xjh
#define X_HOMING_INPUT              4                       // xhi  input used for homing or 0 to disable
#define X_HOMING_DIRECTION          0                       // xhd  0=search moves negative, 1= search moves positive
#define X_SEARCH_VELOCITY           2500                    // xsv  move in negative direction
#define X_LATCH_VELOCITY            200                     // xlv  mm/min
#define X_LATCH_BACKOFF             5                       // xlb  mm
#define X_ZERO_BACKOFF              0.5                     // xzb  mm

#define Y_AXIS_MODE                 AXIS_STANDARD
#define Y_VELOCITY_MAX              15000
#define Y_FEEDRATE_MAX              Y_VELOCITY_MAX
#define Y_TRAVEL_MIN                0
#define Y_TRAVEL_MAX                152.4
#define Y_JERK_MAX                  6000
#define Y_JERK_HIGH_SPEED           6000
#define Y_HOMING_INPUT              1
#define Y_HOMING_DIRECTION          1
#define Y_SEARCH_VELOCITY           3000
#define Y_LATCH_VELOCITY            200
#define Y_LATCH_BACKOFF             5
#define Y_ZERO_BACKOFF              0.5

#define Z_AXIS_MODE                 AXIS_STANDARD
#define Z_VELOCITY_MAX              2000
#define Z_FEEDRATE_MAX              Z_VELOCITY_MAX
#define Z_TRAVEL_MIN                0
#define Z_TRAVEL_MAX                215
#define Z_JERK_MAX                  1500
#define Z_JERK_HIGH_SPEED           3000
#define Z_HOMING_INPUT              5
#define Z_HOMING_DIRECTION          0
#define Z_SEARCH_VELOCITY           300
#define Z_LATCH_VELOCITY            100
#define Z_LATCH_BACKOFF             2
#define Z_ZERO_BACKOFF              0

// Rotary values are chosen to make the motor react the same as X for testing
/***************************************************************************************
 * To calculate the speeds here, in Wolfram Alpha-speak:
 *
 *   c=2*pi*r, r=0.609, d=c/360, s=((S*60)/d), S=40 for s
 *   c=2*pi*r, r=4.28394, d=c/360, s=((S*60)/d), S=40 for s
 *
 * Change r to A_RADIUS, and S to the desired speed, in mm/s or mm/s/s/s.
 *
 * It will return s= as the value you want to enter.
 *
 * If the value is over 1 million, the code will divide it by 1 million,
 * so you have to pre-multiply it by 1000000.0. (The value is in millions, btw.)
 *
 * Note that you need these to be floating point values, so always have a .0 at the end!
 *
 ***************************************************************************************/

#define A_AXIS_MODE             AXIS_RADIUS
//#define A_RADIUS                4.30964
#define A_RADIUS                4.28394
//#define A_VELOCITY_MAX        25920.0 // ~40 mm/s, 2,400 mm/min
//#define A_FEEDRATE_MAX        25920.0/2.0 // ~20 mm/s, 1,200 mm/min
#define A_VELOCITY_MAX          77760.0 // G0 rate ~120 mm/s, 2,400 mm/min
#define A_FEEDRATE_MAX          16050 // ~10 mm/s
//#define A_FEEDRATE_MAX        9720.0 // 9720.0 = G1 rate ~15 mm/s, 900 mm/min
#define A_TRAVEL_MIN            0
#define A_TRAVEL_MAX            10
//#define A_JERK_MAX              81000 // 250 million mm/min^3 = 324000
#define A_JERK_MAX            162000 // 250 million mm/min^3 = 324000
//#define A_JERK_MAX            324000 // 500 million mm/min^3 = 324000
//#define A_JERK_MAX            648000 // 1,000 million mm/min^3 = 648000
// * a million IF it's over a million
// c=2*pi*r, r=5.30516476972984, d=c/360, s=((1000*60)/d)
#define A_HOMING_INPUT          0
#define A_HOMING_DIRECTION      0
#define A_SEARCH_VELOCITY       2000
#define A_LATCH_VELOCITY        2000
#define A_LATCH_BACKOFF         5
#define A_ZERO_BACKOFF          2
#define A_JERK_HIGH_SPEED       A_JERK_MAX

#define B_AXIS_MODE             AXIS_DISABLED
#define B_RADIUS                1
#define B_VELOCITY_MAX          3600
#define B_FEEDRATE_MAX          B_VELOCITY_MAX
#define B_TRAVEL_MIN            0
#define B_TRAVEL_MAX            -1
//#define B_JERK_MAX            20000000
#define B_JERK_MAX              20
#define B_HOMING_INPUT          0
#define B_HOMING_DIRECTION      0
#define B_SEARCH_VELOCITY       600
#define B_LATCH_VELOCITY        100
#define B_LATCH_BACKOFF         10
#define B_ZERO_BACKOFF          2
#define B_JERK_HIGH_SPEED       A_JERK_MAX


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
// Inputs are defined for the g2ref(a) board
// Xmn (board label)
#define DI1_MODE                    IO_ACTIVE_HIGH
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
#define DI4_MODE                    IO_ACTIVE_HIGH
#define DI4_ACTION                  INPUT_ACTION_NONE
#define DI4_FUNCTION                INPUT_FUNCTION_NONE

// Zmin
#define DI5_MODE                    IO_ACTIVE_LOW   // Z probe
#define DI5_ACTION                  INPUT_ACTION_NONE
#define DI5_FUNCTION                INPUT_FUNCTION_NONE

// Zmax
#define DI6_MODE                    IO_MODE_DISABLED
#define DI6_ACTION                  INPUT_ACTION_NONE
#define DI6_FUNCTION                INPUT_FUNCTION_NONE

// Shutdown (Amin on v9 board)
#define DI7_MODE                    IO_MODE_DISABLED
#define DI7_ACTION                  INPUT_ACTION_NONE
#define DI7_FUNCTION                INPUT_FUNCTION_NONE

// High Voltage Z Probe In (Amax on v9 board)
#define DI8_MODE                    IO_ACTIVE_LOW
#define DI8_ACTION                  INPUT_ACTION_NONE
#define DI8_FUNCTION                INPUT_FUNCTION_NONE

// Hardware interlock input
#define DI9_MODE                    IO_MODE_DISABLED
#define DI9_ACTION                  INPUT_ACTION_NONE
#define DI9_FUNCTION                INPUT_FUNCTION_NONE

//Extruder1_PWM
#define DO1_MODE                    IO_ACTIVE_HIGH

//Extruder2_PWM
#define DO2_MODE                    IO_ACTIVE_HIGH

//Fan1A_PWM
#define DO3_MODE                    IO_ACTIVE_HIGH

//Fan1B_PWM
#define DO4_MODE                    IO_ACTIVE_HIGH

#define DO5_MODE                    IO_ACTIVE_HIGH
#define DO6_MODE                    IO_ACTIVE_HIGH
#define DO7_MODE                    IO_ACTIVE_HIGH
#define DO8_MODE                    IO_ACTIVE_HIGH

//SAFEin (Output) signal
#define DO9_MODE                    IO_ACTIVE_HIGH

#define DO10_MODE                   IO_ACTIVE_HIGH

//Header Bed FET
#define DO11_MODE                   IO_ACTIVE_HIGH

//Indicator_LED
#define DO12_MODE                   IO_ACTIVE_HIGH

#define DO13_MODE                   IO_ACTIVE_HIGH


/*** Extruders / Heaters ***/

#define MIX_FAN_VALUE               0.4   // (he1fm) at MIN_FAN_TEMP the fan comes on at this spped (0.0-1.0)
#define MAX_FAN_VALUE               0.75  // (he1fp) at MAX_FAN_TEMP the fan is at this spped (0.0-1.0)
#define MIN_FAN_TEMP                50.0  // (he1fl) at this temp the fan starts to ramp up linearly
#define MAX_FAN_TEMP                100.0 // (he1fh) at this temperature the fan is at "full speed" (MAX_FAN_VALUE)

#define H1_DEFAULT_ENABLE           true
#define H1_DEFAULT_P                7.0
#define H1_DEFAULT_I                0.05
#define H1_DEFAULT_D                150.0

#define H2_DEFAULT_ENABLE           false
#define H2_DEFAULT_P                7.0
#define H2_DEFAULT_I                0.05
#define H2_DEFAULT_D                150.0

#define H3_DEFAULT_ENABLE           false
#define H3_DEFAULT_P                9.0
#define H3_DEFAULT_I                0.12
#define H3_DEFAULT_D                400.0
