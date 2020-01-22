/*
 * settings_cheapo_laser.h - This is a settings file for one of the many possible chepo simple lasers available
 * This file is part of the g2core project
 *
 * Copyright (c) 2020 Robert Giseburt
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

// ***> NOTE: The init message must be a single line with no CRs or LFs
#define INIT_MESSAGE "Initializing configs to cheapo laser"

//**** GLOBAL / GENERAL SETTINGS ******************************************************

// Machine configuration settings

#define JUNCTION_INTEGRATION_TIME   0.05                    // cornering - between 0.10 and 2.00 (higher is faster)
#define CHORDAL_TOLERANCE           0.01                    // chordal tolerance for arcs (in mm)

#define HAS_LASER                   1                       // We have a laser, but no shark (yet)

#define SOFT_LIMIT_ENABLE           0                       // 0=off, 1=on
#define HARD_LIMIT_ENABLE           0                       // 0=off, 1=on
#define SAFETY_INTERLOCK_ENABLE     1                       // 0=off, 1=on

#define SPINDLE_ENABLE_OUTPUT_NUMBER 4
#define SPINDLE_ENABLE_POLARITY      1                       // 0=active low, 1=active high
#define SPINDLE_DIRECTION_OUTPUT_NUMBER 5
#define SPINDLE_DIR_POLARITY        0                       // 0=clockwise is low, 1=clockwise is high
#define SPINDLE_PAUSE_ON_HOLD       true
#define SPINDLE_SPINUP_DELAY        1.0
#define SPINDLE_PWM_NUMBER          6
#define SECONDARY_PWM_OUTPUT_NUMBER 0                       // disabled

// #define LASER_FIRE_PIN_NUMBER       Motate::kOutput3_PinNumber      // note this is a MOTATE pin number, NOT a GPIO pin number
#define LASER_FIRE_PIN_NUMBER       Motate::kOutput1_PinNumber      // note this is a MOTATE pin number, NOT a GPIO pin number
#define LASER_ENABLE_OUTPUT_NUMBER  4
#define LASER_TOOL                  1   // default tool is 1 - note that TOOLS may be limited to 5!
#define LASER_MIN_S                 0.0001// {th2mns:0.0001}
#define LASER_MAX_S                 255.0 // {th2mxs:255}
#define LASER_MIN_PPM               200   // {th2mnp:200}
#define LASER_MAX_PPM               8000  // {th2mxp:8000}

// Kinda hacky way to set the kinematics - since the Laser ToolHead overrides the kinematics, we have to set BASE_KINEMATICS
#define KINEMATICS                  KINE_OTHER
#define BASE_KINEMATICS             CartesianKinematics<AXES, MOTORS>
// Another option:
// #define BASE_KINEMATICS          CoreXYKinematics<AXES, MOTORS>

#define COOLANT_MIST_POLARITY       1                       // 0=active low, 1=active high
#define COOLANT_FLOOD_POLARITY      1                       // 0=active low, 1=active high
#define COOLANT_PAUSE_ON_HOLD       false
#define FLOOD_ENABLE_OUTPUT_NUMBER  0                       // disabled
#define MIST_ENABLE_OUTPUT_NUMBER 0                         // disabled

// Communications and reporting settings

#define USB_SERIAL_PORTS_EXPOSED	1						// 1=single endpoint usb, 2=dual endpoint usb
#define COMM_MODE                   JSON_MODE               // one of: TEXT_MODE, JSON_MODE
#define XIO_ENABLE_FLOW_CONTROL FLOW_CONTROL_RTS            // FLOW_CONTROL_OFF, FLOW_CONTROL_RTS

#define TEXT_VERBOSITY              TV_VERBOSE              // one of: TV_SILENT, TV_VERBOSE
#define JSON_VERBOSITY              JV_MESSAGES             // one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE
#define QUEUE_REPORT_VERBOSITY      QR_OFF                  // one of: QR_OFF, QR_SINGLE, QR_TRIPLE

#define STATUS_REPORT_VERBOSITY     SR_FILTERED             // one of: SR_OFF, SR_FILTERED, SR_VERBOSE

#define STATUS_REPORT_MIN_MS        100                     // milliseconds - enforces a viable minimum
#define STATUS_REPORT_INTERVAL_MS   250                     // milliseconds - set $SV=0 to disable

//#define STATUS_REPORT_DEFAULTS "line","posx","posy","posz","posa","feed","vel","unit","coor","dist","admo","frmo","momo","stat"
#define STATUS_REPORT_DEFAULTS "line","posx","posy","posz","feed","vel","momo","stat","1sgs","2sgs","3sgs"

// Alternate SRs that report in drawable units
//#define STATUS_REPORT_DEFAULTS "line","vel","mpox","mpoy","mpoz","mpoa","coor","ofsa","ofsx","ofsy","ofsz","dist","unit","stat","homz","homy","homx","momo"
//#define STATUS_REPORT_DEFAULTS "_ts1","_cs1","_es1","_xs1","_fe1","line","posx","posy","posz","vel","stat"

// Gcode startup defaults
#define GCODE_DEFAULT_UNITS         MILLIMETERS             // MILLIMETERS or INCHES
#define GCODE_DEFAULT_PLANE         CANON_PLANE_XY          // CANON_PLANE_XY, CANON_PLANE_XZ, or CANON_PLANE_YZ
#define GCODE_DEFAULT_COORD_SYSTEM  G54                     // G54, G55, G56, G57, G58 or G59
#define GCODE_DEFAULT_PATH_CONTROL  PATH_CONTINUOUS
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_DISTANCE_MODE

#define LASER_PULSE_DURATION        100                      // in microseconds {th2pd:5}

// *** motor settings ************************************************************************************

#define MOTOR_POWER_MODE            MOTOR_POWERED_IN_CYCLE  // default motor power mode (see cmMotorPowerMode in stepper.h)
#define MOTOR_POWER_TIMEOUT         2.00                    // motor power timeout in seconds

#define M1_MOTOR_MAP                AXIS_X_EXTERNAL         // 1ma
#define M1_STEP_ANGLE               1.8                     // 1sa
#define M1_TRAVEL_PER_REV           39                      // 1tr
#define M1_MICROSTEPS               64                      // 1mi  1,2,4,8,16,32
#define M1_POLARITY                 0                       // 1po  0=normal, 1=reversed
#define M1_POWER_MODE               MOTOR_ALWAYS_POWERED    // 1pm  TRUE=low power idle enabled
#define M1_POWER_LEVEL              0.600
#define M1_POWER_LEVEL_IDLE         0.100

#define M2_MOTOR_MAP                AXIS_Y_EXTERNAL
#define M2_STEP_ANGLE               1.8
#define M2_TRAVEL_PER_REV           39
#define M2_MICROSTEPS               64
#define M2_POLARITY                 1
#define M2_POWER_MODE               MOTOR_POWER_REDUCED_WHEN_IDLE
#define M2_POWER_LEVEL              0.200
#define M2_POWER_LEVEL_IDLE         0.100

#define M3_MOTOR_MAP                AXIS_Y_EXTERNAL
#define M3_STEP_ANGLE               1.8
#define M3_TRAVEL_PER_REV           39
#define M3_MICROSTEPS               64
#define M3_POLARITY                 0
#define M3_POWER_MODE               MOTOR_POWER_REDUCED_WHEN_IDLE
#define M3_POWER_LEVEL              0.200
#define M3_POWER_LEVEL_IDLE         0.100

#define M4_MOTOR_MAP                AXIS_Z_EXTERNAL
#define M4_STEP_ANGLE               1.8
#define M4_TRAVEL_PER_REV           1
#define M4_MICROSTEPS               64
#define M4_POLARITY                 1
#define M4_POWER_MODE               MOTOR_POWER_REDUCED_WHEN_IDLE
#define M4_POWER_LEVEL              0.750

// Whatever axis is on motor 6 is used for the laser and cannot be used for motion
#define M5_MOTOR_MAP                AXIS_C_EXTERNAL

// *** axis settings **********************************************************************************

#define JERK_MAX                    5000

#define X_AXIS_MODE                 AXIS_STANDARD           // xam  see canonical_machine.h cmAxisMode for valid values
#define X_VELOCITY_MAX              5000                   // xvm  G0 max velocity in mm/min
#define X_FEEDRATE_MAX              X_VELOCITY_MAX          // xfr  G1 max feed rate in mm/min
#define X_TRAVEL_MIN                0                       // xtn  minimum travel for soft limits
#define X_TRAVEL_MAX                170                     // xtm  travel between switches or crashes
#define X_JERK_MAX                  2000                    // xjm  jerk * 1,000,000 {xjm:3000,yjm:3000}{xfr:2000,yfr:2000}
#define X_JERK_HIGH_SPEED           20000                   // xjh
#define X_HOMING_INPUT              1                       // xhi  input used for homing or 0 to disable
#define X_HOMING_DIRECTION          0                       // xhd  0=search moves negative, 1= search moves positive
#define X_SEARCH_VELOCITY           2000                    // xsv  minus means move to minimum switch
#define X_LATCH_VELOCITY            100                     // xlv  mm/min
#define X_LATCH_BACKOFF             0                       // xlb  mm
#define X_ZERO_BACKOFF              0                       // xzb  mm

#define Y_AXIS_MODE                 AXIS_STANDARD
#define Y_VELOCITY_MAX              2000
#define Y_FEEDRATE_MAX              Y_VELOCITY_MAX
#define Y_TRAVEL_MIN                0
#define Y_TRAVEL_MAX                185
#define Y_JERK_MAX                  2000
#define Y_JERK_HIGH_SPEED           20000
#define Y_HOMING_INPUT              2
#define Y_HOMING_DIRECTION          0
#define Y_SEARCH_VELOCITY           2000
#define Y_LATCH_VELOCITY            100
#define Y_LATCH_BACKOFF             0
#define Y_ZERO_BACKOFF              0

#define Z_AXIS_MODE                 AXIS_DISABLED

//*** Input / output settings ***
/*
    IO_MODE_DISABLED
    IO_ACTIVE_LOW    aka NORMALLY_OPEN
    IO_ACTIVE_HIGH   aka NORMALLY_CLOSED

    INPUT_ACTION_NONE
    INPUT_ACTION_STOP         =  1 - stop at normal jerk - preserves positional accuracy
    INPUT_ACTION_FAST_STOP    =  2 - stop at high jerk - preserves positional accuracy
    INPUT_ACTION_HALT         =  3 - stop immediately - not guaranteed to preserve position
    INPUT_ACTION_CYCLE_START  =  4 - start / restart cycle after feedhold (RESERVED)
    INPUT_ACTION_ALARM        =  5 - initiate an alarm. stops everything immediately - preserves position
    INPUT_ACTION_SHUTDOWN     =  6 - initiate a shutdown. stops everything immediately - does not preserve position
    INPUT_ACTION_PANIC        =  7 - initiate a panic. stops everything immediately - does not preserve position
    INPUT_ACTION_RESET        =  8 - reset system

    INPUT_ACTION_LIMIT        =  9 - limit switch processing
    INPUT_ACTION_INTERLOCK    = 10 - interlock processing
*/

#define PROBING_INPUT 5

#define DI1_POLARITY                IO_ACTIVE_HIGH
// #define DI1_ACTION                  INPUT_ACTION_LIMIT
#define DI1_ACTION                  INPUT_ACTION_NONE

#define DI2_POLARITY                IO_ACTIVE_HIGH
// #define DI2_ACTION                  INPUT_ACTION_LIMIT
#define DI2_ACTION                  INPUT_ACTION_NONE

#define DI3_POLARITY                IO_ACTIVE_HIGH
//#define DI3_ACTION                  INPUT_ACTION_LIMIT
#define DI3_ACTION                  INPUT_ACTION_NONE

#define DI4_POLARITY                IO_ACTIVE_HIGH
//#define DI4_ACTION                  INPUT_ACTION_LIMIT
#define DI4_ACTION                  INPUT_ACTION_NONE

#define DI5_POLARITY                IO_ACTIVE_HIGH   // Z probe
#define DI5_ACTION                  INPUT_ACTION_NONE

#define DI6_POLARITY                IO_ACTIVE_HIGH
//#define DI6_ACTION                  INPUT_ACTION_LIMIT
#define DI6_ACTION                  INPUT_ACTION_NONE

#define DI7_ENABLED                 IO_DISABLED
#define DI7_ACTION                  INPUT_ACTION_NONE

#define DI8_ENABLED                 IO_DISABLED
#define DI8_ACTION                  INPUT_ACTION_NONE

#define DI9_ENABLED                 IO_DISABLED
#define DI9_ACTION                  INPUT_ACTION_NONE
