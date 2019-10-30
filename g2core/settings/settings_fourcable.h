/*
 * settings_fourcalbe.h - Synthetos 4-Cable Robot
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart, Jr.
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
/**** Shapeoko2 500mm profile ******************************************/
/***********************************************************************/

// ***> NOTE: The init message must be a single line with no CRs or LFs
#define INIT_MESSAGE "Initializing configs to Pendulum v2"

//**** GLOBAL / GENERAL SETTINGS ******************************************************

// Machine configuration settings

#define KINEMATICS KINE_FOUR_CABLE

#define JUNCTION_INTEGRATION_TIME   1.2                    // cornering - between 0.10 and 2.00 (higher is faster)
#define CHORDAL_TOLERANCE           0.01                    // chordal tolerance for arcs (in mm)

#define SOFT_LIMIT_ENABLE           0                       // 0=off, 1=on
#define HARD_LIMIT_ENABLE           0                       // 0=off, 1=on
#define SAFETY_INTERLOCK_ENABLE     1                       // 0=off, 1=on

#define SPINDLE_ENABLE_POLARITY     1                       // 0=active low, 1=active high
#define SPINDLE_DIR_POLARITY        0                       // 0=clockwise is low, 1=clockwise is high
#define SPINDLE_PAUSE_ON_HOLD       true
#define SPINDLE_SPINUP_DELAY        1.0

#define COOLANT_MIST_POLARITY       1                       // 0=active low, 1=active high
#define COOLANT_FLOOD_POLARITY      1                       // 0=active low, 1=active high
#define COOLANT_PAUSE_ON_HOLD       false

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
#define STATUS_REPORT_DEFAULTS "line","posx","posy","posz","feed","vel","momo","stat","ain1vv","ain2vv","ain3vv","ain4vv","knpa","knpb","knpc","knpd"

//{sr:{"line":t,"posx":t,"posy":t,"posz":t,"feed":t,"vel":t,"momo":t,"stat":t,"ain1vv":t,"_ps1":t,"_ps2":t,"_ps3":t,"_ps4":t}}

// Alternate SRs that report in drawable units
//#define STATUS_REPORT_DEFAULTS "line","vel","mpox","mpoy","mpoz","mpoa","coor","ofsa","ofsx","ofsy","ofsz","dist","unit","stat","homz","homy","homx","momo"
//#define STATUS_REPORT_DEFAULTS "_ts1","_cs1","_es1","_xs1","_fe1","line","posx","posy","posz","vel","stat"

// Gcode startup defaults
#define GCODE_DEFAULT_UNITS         MILLIMETERS             // MILLIMETERS or INCHES
#define GCODE_DEFAULT_PLANE         CANON_PLANE_XY          // CANON_PLANE_XY, CANON_PLANE_XZ, or CANON_PLANE_YZ
#define GCODE_DEFAULT_COORD_SYSTEM  G54                     // G54, G55, G56, G57, G58 or G59
#define GCODE_DEFAULT_PATH_CONTROL  PATH_CONTINUOUS
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_DISTANCE_MODE

// These refer to the AINx, not the AIx
#define LOAD_CELL_A_INPUT 1
#define LOAD_CELL_B_INPUT 2
#define LOAD_CELL_C_INPUT 3
#define LOAD_CELL_D_INPUT 4

#define ANCHOR_A_INPUT 1
#define ANCHOR_B_INPUT 2
#define ANCHOR_C_INPUT 3
#define ANCHOR_D_INPUT 4

// *** motor settings ************************************************************************************

#define MOTOR_POWER_MODE            MOTOR_ALWAYS_POWERED  // default motor power mode (see cmMotorPowerMode in stepper.h)
#define MOTOR_POWER_TIMEOUT         2.00                    // motor power timeout in seconds

#define M1_MOTOR_MAP                AXIS_4WIRE_A
#define M1_STEP_ANGLE               0.54 // 1.8*(18/60)
#define M1_TRAVEL_PER_REV           81.6814089933
#define M1_MICROSTEPS               16
#define M1_POLARITY                 0
#define M1_POWER_MODE               MOTOR_POWER_MODE
#define M1_POWER_LEVEL              0.500
#define M1_ENCODER_INPUT_A          1
#define M1_ENCODER_INPUT_B          2

#define M2_MOTOR_MAP                AXIS_4WIRE_B
#define M2_STEP_ANGLE               0.54
#define M2_TRAVEL_PER_REV           M1_TRAVEL_PER_REV
#define M2_MICROSTEPS               16
#define M2_POLARITY                 0
#define M2_POWER_MODE               MOTOR_POWER_MODE
#define M2_POWER_LEVEL              0.500
#define M2_ENCODER_INPUT_A          3
#define M2_ENCODER_INPUT_B          4

#define M3_MOTOR_MAP                AXIS_4WIRE_C
#define M3_STEP_ANGLE               0.54
#define M3_TRAVEL_PER_REV           M1_TRAVEL_PER_REV
#define M3_MICROSTEPS               16
#define M3_POLARITY                 0
#define M3_POWER_MODE               MOTOR_POWER_MODE
#define M3_POWER_LEVEL              0.500
#define M3_ENCODER_INPUT_A          5
#define M3_ENCODER_INPUT_B          6

#define M4_MOTOR_MAP                AXIS_4WIRE_D
#define M4_STEP_ANGLE               0.54
#define M4_TRAVEL_PER_REV           M1_TRAVEL_PER_REV
#define M4_MICROSTEPS               16
#define M4_POLARITY                 0
#define M4_POWER_MODE               MOTOR_POWER_MODE
#define M4_POWER_LEVEL              0.500
#define M4_ENCODER_INPUT_A          7
#define M4_ENCODER_INPUT_B          8

#define M5_MOTOR_MAP                AXIS_4WIRE_Z
#define M5_STEP_ANGLE               1.6
#define M5_TRAVEL_PER_REV           8
#define M5_MICROSTEPS               32
#define M5_POLARITY                 1
#define M5_POWER_MODE               MOTOR_POWERED_IN_CYCLE
#define M5_POWER_LEVEL              0.500

// *** axis settings **********************************************************************************

// #define EXTERNAL_ENCODER_MM_PER_REV 83.6832318322 // 13.3186 mm radius
#define EXTERNAL_ENCODER_MM_PER_REV 86.8248244858 // 13.8186 mm radius
// 26.88 short of 2149.2536, making it 24.4443177694 rotations
// meaning it should be: 87.9244665478
// #define EXTERNAL_ENCODER_MM_PER_REV 87.9244665478 // 13.9936134698 mm radius

#define JERK_MAX                    5000

#define X_AXIS_MODE                 AXIS_STANDARD           // xam  see canonical_machine.h cmAxisMode for valid values
#define X_VELOCITY_MAX              3000                    // xvm  G0 max velocity in mm/min
#define X_FEEDRATE_MAX              20000                   // xfr  G1 max feed rate in mm/min
#define X_TRAVEL_MIN                0                       // xtn  minimum travel for soft limits
#define X_TRAVEL_MAX                1370                    // xtm  travel between switches or crashes
#define X_JERK_MAX                  150                    // xjm  jerk * 1,000,000
#define X_JERK_HIGH_SPEED           250000                  // xjh
#define X_HOMING_INPUT              1                       // xhi  input used for homing or 0 to disable
#define X_HOMING_DIRECTION          1                       // xhd  0=search moves negative, 1= search moves positive
#define X_SEARCH_VELOCITY           2000                    // xsv  minus means move to minimum switch
#define X_LATCH_VELOCITY            100                     // xlv  mm/min
#define X_LATCH_BACKOFF             4                       // xlb  mm
#define X_ZERO_BACKOFF              2                       // xzb  mm

#define Y_AXIS_MODE                 AXIS_STANDARD
#define Y_VELOCITY_MAX              X_VELOCITY_MAX
#define Y_FEEDRATE_MAX              X_FEEDRATE_MAX
#define Y_TRAVEL_MIN                X_TRAVEL_MIN
#define Y_TRAVEL_MAX                X_TRAVEL_MAX
#define Y_JERK_HIGH_SPEED           X_JERK_HIGH_SPEED
#define Y_JERK_MAX                  X_JERK_MAX
#define Y_HOMING_INPUT              X_HOMING_INPUT
#define Y_HOMING_DIRECTION          X_HOMING_DIRECTION
#define Y_SEARCH_VELOCITY           X_SEARCH_VELOCITY
#define Y_LATCH_VELOCITY            X_LATCH_VELOCITY
#define Y_LATCH_BACKOFF             X_LATCH_BACKOFF
#define Y_ZERO_BACKOFF              X_ZERO_BACKOFF

#define Z_AXIS_MODE                 AXIS_STANDARD
#define Z_VELOCITY_MAX              1200
#define Z_FEEDRATE_MAX              Z_VELOCITY_MAX
#define Z_TRAVEL_MAX                75
#define Z_TRAVEL_MIN                -15
#define Z_JERK_MAX                  500
#define Z_JERK_HIGH_SPEED           1000
#define Z_HOMING_INPUT              3
#define Z_HOMING_DIRECTION          1
#define Z_SEARCH_VELOCITY           (Z_VELOCITY_MAX * 0.66666)
#define Z_LATCH_VELOCITY            25
#define Z_LATCH_BACKOFF             4
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
#define DI1_POLARITY                IO_ACTIVE_LOW
//#define DI1_ACTION                  INPUT_ACTION_STOP
#define DI1_ACTION                  INPUT_ACTION_NONE

// Xmax
#define DI2_POLARITY                IO_ACTIVE_LOW
//#define DI2_ACTION                  INPUT_ACTION_STOP
#define DI2_ACTION                  INPUT_ACTION_NONE

// Ymin
#define DI3_POLARITY                IO_ACTIVE_LOW
//#define DI3_ACTION                  INPUT_ACTION_STOP
#define DI3_ACTION                  INPUT_ACTION_NONE

// Ymax
#define DI4_POLARITY                IO_ACTIVE_LOW
//#define DI4_ACTION                  INPUT_ACTION_STOP
#define DI4_ACTION                  INPUT_ACTION_NONE

// Zmin
#define DI5_POLARITY                IO_ACTIVE_LOW
#define DI5_ACTION                  INPUT_ACTION_NONE

// Zmax
#define DI6_POLARITY                IO_ACTIVE_LOW
#define DI6_ACTION                  INPUT_ACTION_NONE

// Amin
#define DI7_POLARITY                IO_ACTIVE_LOW
#define DI7_ACTION                  INPUT_ACTION_NONE

// Amax
#define DI8_POLARITY                IO_ACTIVE_LOW
#define DI8_ACTION                  INPUT_ACTION_NONE

// Hardware interlock input
#define DI9_ENABLED                 IO_DISABLED
#define DI9_POLARITY                IO_ACTIVE_LOW
#define DI9_ACTION                  INPUT_ACTION_NONE


// *** PWM SPINDLE CONTROL ***

/* VFD settings:
 P0 settings need to be changed:
 P0-000 = 1 (Select command source = Analog terminal control)
 P0-001 = 3 (Select frequency source = max（Main frequency source x，Assistant frequency source y))
 P0-002 = 2 (Main  frequency  source  x selection = AIN1)

 P0-007 = 400
 P0-024 = 400   (set maximum frequency at 100%)

 For run/stop and direction control:
 P0-016 = 1 (factory) (X1 terminal function = forward run)
 P0-017 = 2 (factory was 24) (X2 terminal function = reverse run)
 P0-020 = 1 (2-wire mode = 2)
 P0-021 = 0 (set minimum input)
 P0-022 = 0 (set minimum speed)

 Disconnect X1, X2, set P0-24 to 10, and P0-023 to 10, and read the monitor,
 then set the g2 to 100% output (via M3 or direct json), then use the shown value for P0-23

 P0-023 = 9.22 for me (set maximum input voltage)


 X1 = running
 X2 = reverse direction (low = forward)

*/

#define P1_PWM_FREQUENCY 100000  // in Hz
#define P1_CW_SPEED_LO 1  // in RPM (arbitrary units)
#define P1_CW_SPEED_HI 24000
#define P1_CW_PHASE_LO 0.05  // phase [0..1]
#define P1_CW_PHASE_HI 1.0
#define P1_CCW_SPEED_LO 1
#define P1_CCW_SPEED_HI 24000.0
#define P1_CCW_PHASE_LO 0.05
#define P1_CCW_PHASE_HI 1.0
#define P1_PWM_PHASE_OFF 0.0
