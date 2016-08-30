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
#define CHORDAL_TOLERANCE 0.01    // chordal accuracy for arc drawing (in mm)

#define SOFT_LIMIT_ENABLE 0        // 0=off, 1=on
#define HARD_LIMIT_ENABLE 1        // 0=off, 1=on
#define SAFETY_INTERLOCK_ENABLE 1  // 0=off, 1=on

#define SPINDLE_ENABLE_POLARITY 1  // 0=active low, 1=active high
#define SPINDLE_DIR_POLARITY 0     // 0=clockwise is low, 1=clockwise is high
#define SPINDLE_PAUSE_ON_HOLD true
#define SPINDLE_DWELL_TIME 1.5  // after unpausing and turning the spindle on, dwell for 1.5s

#define ESC_BOOT_TIME 5000  // how long the ESC takes to boot, in milliseconds
#define ESC_LOCKOUT_TIME \
    900  // how long the interlock needs to be engaged before killing power... actually 1s, but be conservative

#define COOLANT_MIST_POLARITY 1   // 0=active low, 1=active high
#define COOLANT_FLOOD_POLARITY 1  // 0=active low, 1=active high
#define COOLANT_PAUSE_ON_HOLD true

constexpr float H1_DEFAULT_P = 7.0;
constexpr float H1_DEFAULT_I = 0.2;
constexpr float H1_DEFAULT_D = 100.0;

constexpr float H2_DEFAULT_P = 7.0;
constexpr float H2_DEFAULT_I = 0.2;
constexpr float H2_DEFAULT_D = 100.0;

constexpr float H3_DEFAULT_P = 7.0;
constexpr float H3_DEFAULT_I = 0.2;
constexpr float H3_DEFAULT_D = 100.0;

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
#define PAUSE_DWELL_TIME			1.5 //after unpausing and turning the spindle on, dwell for 1.5s
*/

// Communications and reporting settings

#define TEXT_VERBOSITY TV_VERBOSE  // one of: TV_SILENT, TV_VERBOSE
#define COMM_MODE JSON_MODE        // one of: TEXT_MODE, JSON_MODE
#define XIO_ENABLE_FLOW_CONTROL FLOW_CONTROL_RTS  // FLOW_CONTROL_OFF, FLOW_CONTROL_RTS

#define JSON_VERBOSITY JV_CONFIGS  // one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE
#define JSON_SYNTAX_MODE JSON_SYNTAX_STRICT  // one of JSON_SYNTAX_RELAXED, JSON_SYNTAX_STRICT

#define QUEUE_REPORT_VERBOSITY QR_SINGLE

#define STATUS_REPORT_VERBOSITY SR_FILTERED  // one of: SR_OFF, SR_FILTERED, SR_VERBOSE
#define STATUS_REPORT_MIN_MS 100             // milliseconds - enforces a viable minimum
#define STATUS_REPORT_INTERVAL_MS 250        // milliseconds - set $SV=0 to disable
#define STATUS_REPORT_DEFAULTS                                                                                      \
    "mpox", "mpoy", "mpoz", "ofsx", "ofsy", "ofsz", "g55x", "g55y", "g55z", "unit", "stat", "coor", "momo", "dist", \
        "home", "mots", "plan", "line", "path", "frmo", "prbe", "safe", "spe", "spd", "hold", "macs", "cycs", "sps"

// Gcode startup defaults
#define GCODE_DEFAULT_UNITS MILLIMETERS     // MILLIMETERS or INCHES
#define GCODE_DEFAULT_PLANE CANON_PLANE_XY  // CANON_PLANE_XY, CANON_PLANE_XZ, or CANON_PLANE_YZ
#define GCODE_DEFAULT_COORD_SYSTEM G55      // G54, G55, G56, G57, G58 or G59
#define GCODE_DEFAULT_PATH_CONTROL PATH_CONTINUOUS
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_MODE

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

#define M4_MOTOR_MAP AXIS_A
#define M4_STEP_ANGLE 1.8
#define M4_TRAVEL_PER_REV 360  // degrees moved per motor rev
#define M4_MICROSTEPS 8
#define M4_POLARITY 1
#define M4_POWER_MODE MOTOR_DISABLED
#define M4_POWER_LEVEL MOTOR_POWER_LEVEL_DISABLED
#define M4_POWER_LEVEL_IDLE MOTOR_POWER_LEVEL_DISABLED

#define M5_MOTOR_MAP AXIS_B
#define M5_STEP_ANGLE 1.8
#define M5_TRAVEL_PER_REV 360  // degrees moved per motor rev
#define M5_MICROSTEPS 8
#define M5_POLARITY 0
#define M5_POWER_MODE MOTOR_DISABLED
#define M5_POWER_LEVEL MOTOR_POWER_LEVEL_DISABLED
#define M5_POWER_LEVEL_IDLE MOTOR_POWER_LEVEL_DISABLED

#define M6_MOTOR_MAP AXIS_C
#define M6_STEP_ANGLE 1.8
#define M6_TRAVEL_PER_REV 360  // degrees moved per motor rev
#define M6_MICROSTEPS 8
#define M6_POLARITY 0
#define M6_POWER_MODE MOTOR_DISABLED
#define M6_POWER_LEVEL MOTOR_POWER_LEVEL_DISABLED
#define M6_POWER_LEVEL_IDLE MOTOR_POWER_LEVEL_DISABLED

// *** axis settings **********************************************************************************

#define JERK_MAX 500          // 500 million mm/(min^3)
#define JERK_HIGH_SPEED 1000  // 1000 million mm/(min^3)		// Jerk during homing needs to stop *fast*
#define LATCH_VELOCITY 25     // reeeeally slow for accuracy

#define JUNCTION_DEVIATION_XY 0.01   // larger is faster
#define JUNCTION_DEVIATION_Z 0.01    // larger is faster
#define JUNCTION_DEVIATION_ABC 0.01  // larger is faster

#define X_AXIS_MODE AXIS_STANDARD                   // xam  see canonical_machine.h cmAxisMode for valid values
#define X_VELOCITY_MAX 1500                         // xvm  G0 max velocity in mm/min
#define X_FEEDRATE_MAX X_VELOCITY_MAX               // xfr  G1 max feed rate in mm/min
#define X_TRAVEL_MIN 0                              // xtn  minimum travel for soft limits
#define X_TRAVEL_MAX 145.6                          // xtr  travel between switches or crashes
#define X_JERK_MAX JERK_MAX                         // xjm
#define X_JERK_HIGH_SPEED JERK_HIGH_SPEED           // xjh
#define X_JUNCTION_DEVIATION JUNCTION_DEVIATION_XY  // xjd
#define X_HOMING_INPUT 1                            // xhi  input used for homing or 0 to disable
#define X_HOMING_DIRECTION 0                        // xhd  0=search moves negative, 1= search moves positive
#define X_SEARCH_VELOCITY (X_FEEDRATE_MAX / 3)      // xsv
#define X_LATCH_VELOCITY LATCH_VELOCITY             // xlv  mm/min
#define X_LATCH_BACKOFF 5                           // xlb  mm
#define X_ZERO_BACKOFF 0.4                          // xzb  mm

#define Y_AXIS_MODE AXIS_STANDARD
#define Y_VELOCITY_MAX X_VELOCITY_MAX
#define Y_FEEDRATE_MAX Y_VELOCITY_MAX
#define Y_TRAVEL_MIN 0
#define Y_TRAVEL_MAX 119.1
#define Y_JERK_MAX JERK_MAX
#define Y_JERK_HIGH_SPEED JERK_HIGH_SPEED
#define Y_JUNCTION_DEVIATION JUNCTION_DEVIATION_XY
#define Y_HOMING_INPUT 3
#define Y_HOMING_DIRECTION 0
#define Y_SEARCH_VELOCITY (Y_FEEDRATE_MAX / 3)
#define Y_LATCH_VELOCITY LATCH_VELOCITY
#define Y_LATCH_BACKOFF 5
#define Y_ZERO_BACKOFF 0.4

#define Z_AXIS_MODE AXIS_STANDARD
#if HAS_CANSTACK_Z_AXIS
#define Z_VELOCITY_MAX 1000
#else
#define Z_VELOCITY_MAX X_VELOCITY_MAX
#endif
#define Z_FEEDRATE_MAX Z_VELOCITY_MAX
#define Z_TRAVEL_MIN -60.1
#define Z_TRAVEL_MAX 0
#define Z_JERK_MAX JERK_MAX
#define Z_JERK_HIGH_SPEED JERK_HIGH_SPEED
#define Z_JUNCTION_DEVIATION JUNCTION_DEVIATION_Z
#define Z_HOMING_INPUT 6
#define Z_HOMING_DIRECTION 1
#define Z_SEARCH_VELOCITY (Z_FEEDRATE_MAX / 3)
#define Z_LATCH_VELOCITY LATCH_VELOCITY
#define Z_LATCH_BACKOFF 5
#define Z_ZERO_BACKOFF 0.4

// Rotary values are chosen to make the motor react the same as X for testing
#define A_AXIS_MODE AXIS_DISABLED                                    // DISABLED
#define A_VELOCITY_MAX ((X_VELOCITY_MAX / M1_TRAVEL_PER_REV) * 360)  // set to the same speed as X axis
#define A_FEEDRATE_MAX A_VELOCITY_MAX
#define A_TRAVEL_MIN -1  // min/max the same means infinite, no limit
#define A_TRAVEL_MAX -1
#define A_JERK_MAX (X_JERK_MAX * (360 / M1_TRAVEL_PER_REV))
#define A_JERK_HIGH_SPEED A_JERK_MAX
#define A_JUNCTION_DEVIATION JUNCTION_DEVIATION_ABC
#define A_RADIUS (M1_TRAVEL_PER_REV / (2 * 3.14159628))
#define A_HOMING_INPUT 0
#define A_HOMING_DIRECTION 0
#define A_SEARCH_VELOCITY 600
#define A_LATCH_VELOCITY 100
#define A_LATCH_BACKOFF 5
#define A_ZERO_BACKOFF 2

#define B_AXIS_MODE AXIS_DISABLED  // DISABLED
#define B_VELOCITY_MAX ((X_VELOCITY_MAX / M1_TRAVEL_PER_REV) * 360)
#define B_FEEDRATE_MAX B_VELOCITY_MAX
#define B_TRAVEL_MIN -1
#define B_TRAVEL_MAX -1
#define B_JERK_MAX (X_JERK_MAX * (360 / M1_TRAVEL_PER_REV))
#define B_JERK_HIGH_SPEED B_JERK_MAX
#define B_JUNCTION_DEVIATION JUNCTION_DEVIATION_ABC
#define B_RADIUS (M1_TRAVEL_PER_REV / (2 * 3.14159628))
#define B_HOMING_INPUT 0
#define B_HOMING_DIRECTION 0
#define B_SEARCH_VELOCITY 600
#define B_LATCH_VELOCITY 100
#define B_LATCH_BACKOFF 5
#define B_ZERO_BACKOFF 2

#define C_AXIS_MODE AXIS_DISABLED  // DISABLED
#define C_VELOCITY_MAX ((X_VELOCITY_MAX / M1_TRAVEL_PER_REV) * 360)
#define C_FEEDRATE_MAX C_VELOCITY_MAX
#define C_TRAVEL_MIN -1
#define C_TRAVEL_MAX -1
#define C_JERK_MAX (X_JERK_MAX * (360 / M1_TRAVEL_PER_REV))
#define C_JERK_HIGH_SPEED C_JERK_MAX
#define C_JUNCTION_DEVIATION JUNCTION_DEVIATION_ABC
#define C_RADIUS (M1_TRAVEL_PER_REV / (2 * 3.14159628))
#define C_HOMING_INPUT 0
#define C_HOMING_DIRECTION 0
#define C_SEARCH_VELOCITY 600
#define C_LATCH_VELOCITY 100
#define C_LATCH_BACKOFF 5
#define C_ZERO_BACKOFF 2

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
    INPUT_FUNCTION_PANIC
*/

// Xmin on v9 board                 // X homing - see X axis setup
#define DI1_MODE NORMALLY_CLOSED
#define DI1_ACTION INPUT_ACTION_NONE
#define DI1_FUNCTION INPUT_FUNCTION_NONE

// Xmax                             // External ESTOP
#define DI2_MODE INPUT_ACTIVE_HIGH
#define DI2_ACTION INPUT_ACTION_HALT
#define DI2_FUNCTION INPUT_FUNCTION_SHUTDOWN

// Ymin                             // Y homing - see Y axis setup
#define DI3_MODE NORMALLY_CLOSED
#define DI3_ACTION INPUT_ACTION_NONE
#define DI3_FUNCTION INPUT_FUNCTION_NONE

// Ymax                             // Safety interlock
#define DI4_MODE INPUT_ACTIVE_HIGH
#define DI4_ACTION INPUT_ACTION_NONE  // (hold is performed by Interlock function)
#define DI4_FUNCTION INPUT_FUNCTION_INTERLOCK

// Zmin                             // Z probe
#define DI5_MODE INPUT_ACTIVE_LOW
#define DI5_ACTION INPUT_ACTION_NONE
#define DI5_FUNCTION INPUT_FUNCTION_NONE

// Zmax                             // Z homing - see Z axis for setup
#define DI6_MODE NORMALLY_CLOSED
#define DI6_ACTION INPUT_ACTION_NONE
#define DI6_FUNCTION INPUT_FUNCTION_NONE

// Amin                             // Unused
#define DI7_MODE INPUT_MODE_DISABLED
#define DI7_ACTION INPUT_ACTION_NONE
#define DI7_FUNCTION INPUT_FUNCTION_NONE

// Amax                             // Unused
#define DI8_MODE INPUT_MODE_DISABLED
#define DI8_ACTION INPUT_ACTION_NONE
#define DI8_FUNCTION INPUT_FUNCTION_NONE

// Safety line w/HW timer           // Unused
#define DI9_MODE INPUT_MODE_DISABLED
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

// *** DEFAULT COORDINATE SYSTEM OFFSETS ***

#define G54_X_OFFSET 0  // G54 is traditionally set to all zeros
#define G54_Y_OFFSET 0
#define G54_Z_OFFSET 0
#define G54_A_OFFSET 0
#define G54_B_OFFSET 0
#define G54_C_OFFSET 0

#define G55_X_OFFSET 0  // but the again, so is everyting else (at least for start)
#define G55_Y_OFFSET 0
#define G55_Z_OFFSET 0
#define G55_A_OFFSET 0
#define G55_B_OFFSET 0
#define G55_C_OFFSET 0  // this is where we currently store the tool offset

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
