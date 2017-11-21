/*
 * settings_othermill.h - Other Machine Company Mini Milling Machine
 * This file is part of the g2core project
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
/**** Otherlab OtherMill profile ***************************************/
/***********************************************************************/

// ***> NOTE: The init message must be a single line with no CRs or LFs
#define INIT_MESSAGE "Initializing configs to OMC OtherMill settings"

//**** GLOBAL / GENERAL SETTINGS ******************************************************

#define JUNCTION_INTEGRATION_TIME 0.75 // cornering - between 0.10 and 2.00 (higher is faster)
#define CHORDAL_TOLERANCE 0.01      // chordal accuracy for arc drawing (in mm)

#define SOFT_LIMIT_ENABLE 0         // 0=off, 1=on
#define HARD_LIMIT_ENABLE 1         // 0=off, 1=on
#define SAFETY_INTERLOCK_ENABLE 1   // 0=off, 1=on

#define SPINDLE_ENABLE_POLARITY 1   // 0=active low, 1=active high
#define SPINDLE_DIR_POLARITY 0      // 0=clockwise is low, 1=clockwise is high
#define SPINDLE_PAUSE_ON_HOLD true
#define SPINDLE_DWELL_TIME 1.5      // after unpausing and turning the spindle on, dwell for 1.5s

#define ESC_BOOT_TIME 5000          // how long the ESC takes to boot, in milliseconds
#define ESC_LOCKOUT_TIME 900        // how long the interlock needs to be engaged before killing power... actually 1s, but be conservative

#define COOLANT_MIST_POLARITY 1     // 0=active low, 1=active high
#define COOLANT_FLOOD_POLARITY 1    // 0=active low, 1=active high
#define COOLANT_PAUSE_ON_HOLD true

// WARNING: Older Othermill machines use a 15deg can stack for their Z axis.
// new machines use a stepper which has the same config as the other axis.
#define HAS_CANSTACK_Z_AXIS 0
/*
// Switch definitions for interlock & E-stop
#define ENABLE_INTERLOCK_AND_ESTOP
#define INTERLOCK_SWITCH_AXIS       AXIS_Y
#define INTERLOCK_SWITCH_POSITION   SW_MAX
#define ESTOP_SWITCH_AXIS           AXIS_X
#define ESTOP_SWITCH_POSITION       SW_MAX
#define PAUSE_DWELL_TIME            1.5 //after unpausing and turning the spindle on, dwell for 1.5s
*/

// Communications and reporting settings

#define COMM_MODE JSON_MODE                     // one of: TEXT_MODE, JSON_MODE
#define XIO_ENABLE_FLOW_CONTROL FLOW_CONTROL_RTS // FLOW_CONTROL_OFF, FLOW_CONTROL_RTS

#define TEXT_VERBOSITY TV_VERBOSE               // one of: TV_SILENT, TV_VERBOSE
#define JSON_VERBOSITY JV_MESSAGES              // one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE
#define QUEUE_REPORT_VERBOSITY QR_SINGLE        // one of: QR_OFF, QR_SINGLE, QR_TRIPLE

#define STATUS_REPORT_VERBOSITY SR_FILTERED     // one of: SR_OFF, SR_FILTERED, SR_VERBOSE
#define STATUS_REPORT_MIN_MS 100                // milliseconds - enforces a viable minimum
#define STATUS_REPORT_INTERVAL_MS 250           // milliseconds - set $SV=0 to disable
#define STATUS_REPORT_DEFAULTS                                                                                      \
    "mpox", "mpoy", "mpoz", "ofsx", "ofsy", "ofsz", "g55x", "g55y", "g55z", "unit", "stat", "coor", "momo", "dist", \
        "home", "mots", "plan", "line", "path", "frmo", "hold", "macs", "cycs"
//      "path","frmo","prbe","safe","spe","spd","hold","macs","cycs","sps"

// Gcode startup defaults
#define GCODE_DEFAULT_UNITS MILLIMETERS     // MILLIMETERS or INCHES
#define GCODE_DEFAULT_PLANE CANON_PLANE_XY  // CANON_PLANE_XY, CANON_PLANE_XZ, or CANON_PLANE_YZ
#define GCODE_DEFAULT_COORD_SYSTEM G55      // G54, G55, G56, G57, G58 or G59
#define GCODE_DEFAULT_PATH_CONTROL PATH_CONTINUOUS
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_DISTANCE_MODE

// *** motor settings ************************************************************************************

// NOTE: Motor numbers are reversed from TinyGv8 in order to maintain compatibility with wiring harnesses

#define MOTOR_POWER_LEVEL_XY 0.375          // default motor power level 0.00 - 1.00
#define MOTOR_POWER_LEVEL_XY_IDLE 0.15
#define MOTOR_POWER_LEVEL_Z 0.375
#define MOTOR_POWER_LEVEL_Z_IDLE 0.15
#define MOTOR_POWER_LEVEL_DISABLED 0.05

#define MOTOR_POWER_MODE MOTOR_POWERED_IN_CYCLE
#define MOTOR_POWER_TIMEOUT 2.00            // motor power timeout in seconds

#define M1_MOTOR_MAP AXIS_X                  // 1ma
#define M1_STEP_ANGLE 1.8                    // 1sa
#define M1_TRAVEL_PER_REV 4.8768             // 1tr
#define M1_MICROSTEPS 8                      // 1mi  1,2,4,8,16,32
#define M1_POLARITY 1                        // 1po  0=normal, 1=reversed
#define M1_POWER_MODE MOTOR_POWER_MODE       // 1pm  See enum cmMotorPowerMode in stepper.h
#define M1_POWER_LEVEL MOTOR_POWER_LEVEL_XY  // 0.00=off, 1.00=max
#define M1_POWER_LEVEL_IDLE MOTOR_POWER_LEVEL_XY_IDLE

#define M2_MOTOR_MAP AXIS_Y
#define M2_STEP_ANGLE 1.8
#define M2_TRAVEL_PER_REV 4.8768
#define M2_MICROSTEPS 8
#define M2_POLARITY 1
#define M2_POWER_MODE MOTOR_POWER_MODE
#define M2_POWER_LEVEL MOTOR_POWER_LEVEL_XY
#define M2_POWER_LEVEL_IDLE MOTOR_POWER_LEVEL_XY_IDLE

#define M3_MOTOR_MAP AXIS_Z
#if HAS_CANSTACK_Z_AXIS
#define M3_STEP_ANGLE 15
#define M3_TRAVEL_PER_REV 1.27254
#else
#define M3_STEP_ANGLE 1.8
#define M3_TRAVEL_PER_REV 4.8768
#endif
#define M3_MICROSTEPS 8
#define M3_POLARITY 0
#define M3_POWER_MODE MOTOR_POWER_MODE
#define M3_POWER_LEVEL MOTOR_POWER_LEVEL_Z
#define M3_POWER_LEVEL_IDLE MOTOR_POWER_LEVEL_Z_IDLE

// *** axis settings **********************************************************************************

#define JERK_MAX 500          // 500 million mm/(min^3)
#define JERK_HIGH_SPEED 1000  // 1000 million mm/(min^3)        // Jerk during homing needs to stop *fast*
#define VELOCITY_MAX 1500
#define SEARCH_VELOCITY (VELOCITY_MAX / 3)
#define LATCH_VELOCITY 25  // reeeeally slow for accuracy

#define X_AXIS_MODE AXIS_STANDARD                   // xam  see canonical_machine.h cmAxisMode for valid values
#define X_VELOCITY_MAX VELOCITY_MAX                 // xvm  G0 max velocity in mm/min
#define X_FEEDRATE_MAX X_VELOCITY_MAX               // xfr  G1 max feed rate in mm/min
#define X_TRAVEL_MIN 0                              // xtn  minimum travel for soft limits
#define X_TRAVEL_MAX 145.6                          // xtr  travel between switches or crashes
#define X_JERK_MAX JERK_MAX                         // xjm
#define X_JERK_HIGH_SPEED JERK_HIGH_SPEED           // xjh
#define X_HOMING_INPUT 1                            // xhi  input used for homing or 0 to disable
#define X_HOMING_DIRECTION 0                        // xhd  0=search moves negative, 1= search moves positive
#define X_SEARCH_VELOCITY SEARCH_VELOCITY           // xsv
#define X_LATCH_VELOCITY LATCH_VELOCITY             // xlv  mm/min
#define X_LATCH_BACKOFF 1                           // xlb  mm
#define X_ZERO_BACKOFF 0.4                          // xzb  mm

#define Y_AXIS_MODE AXIS_STANDARD
#define Y_VELOCITY_MAX VELOCITY_MAX
#define Y_FEEDRATE_MAX Y_VELOCITY_MAX
#define Y_TRAVEL_MIN 0
#define Y_TRAVEL_MAX 119.1
#define Y_JERK_MAX JERK_MAX
#define Y_JERK_HIGH_SPEED JERK_HIGH_SPEED
#define Y_HOMING_INPUT 3
#define Y_HOMING_DIRECTION 0
#define Y_SEARCH_VELOCITY SEARCH_VELOCITY
#define Y_LATCH_VELOCITY LATCH_VELOCITY
#define Y_LATCH_BACKOFF 1
#define Y_ZERO_BACKOFF 0.4

#define Z_AXIS_MODE AXIS_STANDARD
#if HAS_CANSTACK_Z_AXIS
#define Z_VELOCITY_MAX 1000
#else
#define Z_VELOCITY_MAX VELOCITY_MAX
#endif
#define Z_FEEDRATE_MAX Z_VELOCITY_MAX
#define Z_TRAVEL_MIN -60.1
#define Z_TRAVEL_MAX 0
#define Z_JERK_MAX JERK_MAX
#define Z_JERK_HIGH_SPEED JERK_HIGH_SPEED
#define Z_HOMING_INPUT 6
#define Z_HOMING_DIRECTION 1
#define Z_SEARCH_VELOCITY SEARCH_VELOCITY
#define Z_LATCH_VELOCITY LATCH_VELOCITY
#define Z_LATCH_BACKOFF 1
#define Z_ZERO_BACKOFF 0.4

//*** Input / output settings ***
/*
    See gpio.h GPIO defines for options

    Homing and probing settings are independent of ACTION and FUNCTION settings
    but rely on proper switch MODE setting (i.e. NC or NO)

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

// Xmin on v9 board                 // X homing - see X axis setup
#define DI1_MODE NORMALLY_CLOSED
#define DI1_ACTION INPUT_ACTION_NONE
#define DI1_FUNCTION INPUT_FUNCTION_NONE

// Xmax                             // External ESTOP
#define DI2_MODE IO_ACTIVE_HIGH
#define DI2_ACTION INPUT_ACTION_HALT
#define DI2_FUNCTION INPUT_FUNCTION_SHUTDOWN

// Ymin                             // Y homing - see Y axis setup
#define DI3_MODE NORMALLY_CLOSED
#define DI3_ACTION INPUT_ACTION_NONE
#define DI3_FUNCTION INPUT_FUNCTION_NONE

// Ymax                             // Safety interlock
#define DI4_MODE IO_ACTIVE_HIGH
#define DI4_ACTION INPUT_ACTION_NONE  // (hold is performed by Interlock function)
#define DI4_FUNCTION INPUT_FUNCTION_INTERLOCK

// Zmin                             // Z probe
#define DI5_MODE IO_ACTIVE_LOW
#define DI5_ACTION INPUT_ACTION_NONE
#define DI5_FUNCTION INPUT_FUNCTION_NONE

// Zmax                             // Z homing - see Z axis for setup
#define DI6_MODE NORMALLY_CLOSED
#define DI6_ACTION INPUT_ACTION_NONE
#define DI6_FUNCTION INPUT_FUNCTION_NONE

// Amin                             // Unused
#define DI7_MODE IO_MODE_DISABLED
#define DI7_ACTION INPUT_ACTION_NONE
#define DI7_FUNCTION INPUT_FUNCTION_NONE

// Amax                             // Unused
#define DI8_MODE IO_MODE_DISABLED
#define DI8_ACTION INPUT_ACTION_NONE
#define DI8_FUNCTION INPUT_FUNCTION_NONE

// Safety line w/HW timer           // Unused
#define DI9_MODE IO_MODE_DISABLED
#define DI9_ACTION INPUT_ACTION_NONE
#define DI9_FUNCTION INPUT_FUNCTION_NONE

// *** PWM SPINDLE CONTROL ***

#define P1_PWM_FREQUENCY 100  // in Hz
#define P1_CW_SPEED_LO 10500  // in RPM (arbitrary units)
#define P1_CW_SPEED_HI 16400
#define P1_CW_PHASE_LO 0.13  // phase [0..1]
#define P1_CW_PHASE_HI 0.17
#define P1_CCW_SPEED_LO 0
#define P1_CCW_SPEED_HI 0
#define P1_CCW_PHASE_LO 0.1
#define P1_CCW_PHASE_HI 0.1
#define P1_PWM_PHASE_OFF 0.1

#define P1_USE_MAPPING_CUBIC
#define P1_MAPPING_CUBIC_X3 2.1225328766717546e-013
#define P1_MAPPING_CUBIC_X2 -7.2900167282605129e-009
#define P1_MAPPING_CUBIC_X1 8.5854646785876479e-005
#define P1_MAPPING_CUBIC_X0 -2.1301489219406905e-001
