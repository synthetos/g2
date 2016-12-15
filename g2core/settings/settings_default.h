/*
 * settings_default.h - default machine profile - Set for Shapeoko2
 * This file is part of the g2core project
 *
 * Copyright (c) 2012 - 2016 Alden S. Hart, Jr.
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
/* Note: The values in this file are the default settings that are loaded
 *       into a virgin EEPROM, and can be changed using the config commands.
 *       After initial load the EEPROM values (or changed values) are used.
 *
 *       System and hardware settings that you shouldn't need to change
 *       are in hardware.h  Application settings that also shouldn't need
 *       to be changed are in g2core.h
 */
/*
 * This file is included to define settings that were not present in the
 * SETTINGS_FILE, or if SETTINGS_FILE was omitted.
 *
 * It does the following for each section
 *
 *  - Define rational machine defaults and Gcode power-on defaults
 *  - Define rational communications and reporting settings
 *  - Disable all motors. Motors used must be enabled in other settings file
 *  - Disable all axes. Axes used must be enabled in other settings file
 */
/***********************************************************************/
/**** Default profile for screw driven machines ************************/
/***********************************************************************/

// ***> NOTE: The init message must be a single line with no CRs or LFs
#ifndef INIT_MESSAGE
#define INIT_MESSAGE "Initializing configs to default settings"
#endif

//*****************************************************************************
//**** GLOBAL / GENERAL SETTINGS **********************************************
//*****************************************************************************

// *** Machine configuration settings *** //

#ifndef USB_SERIAL_PORTS_EXPOSED
#define USB_SERIAL_PORTS_EXPOSED   1        // Valid options are 1 or 2, only!
#endif


#ifndef JUNCTION_INTEGRATION_TIME
#define JUNCTION_INTEGRATION_TIME   0.75    // {jt: cornering - between 0.05 and 2.00 (max)
#endif

#ifndef CHORDAL_TOLERANCE
#define CHORDAL_TOLERANCE           0.01    // {ct: chordal tolerance for arcs (in mm)
#endif

#ifndef MOTOR_POWER_TIMEOUT
#define MOTOR_POWER_TIMEOUT         2.00    // {mt:  motor power timeout in seconds
#endif

#ifndef SOFT_LIMIT_ENABLE
#define SOFT_LIMIT_ENABLE           0       // {sl: 0=off, 1=on
#endif

#ifndef HARD_LIMIT_ENABLE
#define HARD_LIMIT_ENABLE           1       // {lim: 0=off, 1=on
#endif
#ifndef SAFETY_INTERLOCK_ENABLE
#define SAFETY_INTERLOCK_ENABLE     1       // {saf: 0=off, 1=on
#endif

#ifndef SPINDLE_ENABLE_POLARITY
#define SPINDLE_ENABLE_POLARITY     SPINDLE_ACTIVE_HIGH  // {spep: 0=active low, 1=active high
#endif

#ifndef SPINDLE_DIR_POLARITY
#define SPINDLE_DIR_POLARITY        0       // {spdp: 0=clockwise is low, 1=clockwise is high
#endif

#ifndef SPINDLE_PAUSE_ON_HOLD
#define SPINDLE_PAUSE_ON_HOLD       true    // {spph:
#endif

#ifndef SPINDLE_DWELL_TIME
#define SPINDLE_DWELL_TIME          1.0     // {spdw:
#endif

#ifndef COOLANT_MIST_POLARITY
#define COOLANT_MIST_POLARITY       1       // {comp: 0=active low, 1=active high
#endif

#ifndef COOLANT_FLOOD_POLARITY
#define COOLANT_FLOOD_POLARITY      1       // {cofp: 0=active low, 1=active high
#endif

#ifndef COOLANT_PAUSE_ON_HOLD
#define COOLANT_PAUSE_ON_HOLD       true    // {coph:
#endif

/* 
 * The following is to fix an issue where feedrate override was being defined in some users
 * settings files but not others. This would otherwise cause an undefined compile error.
 *
*/
#ifndef MANUAL_FEEDRATE_OVERRIDE_ENABLE
#define MANUAL_FEEDRATE_OVERRIDE_ENABLE false
#endif

#ifndef MANUAL_FEEDRATE_OVERRIDE_PARAMETER
#define MANUAL_FEEDRATE_OVERRIDE_PARAMETER 1.00
#endif

// *** Communications and Reporting Settings *** //

#ifndef TEXT_VERBOSITY
#define TEXT_VERBOSITY              TV_VERBOSE              // {tv: TV_SILENT, TV_VERBOSE
#endif

#ifndef COMM_MODE
#define COMM_MODE                   JSON_MODE               // {ej: TEXT_MODE, JSON_MODE
#endif

#ifndef XIO_ENABLE_FLOW_CONTROL
#define XIO_ENABLE_FLOW_CONTROL     FLOW_CONTROL_RTS        // FLOW_CONTROL_OFF, FLOW_CONTROL_XON, FLOW_CONTROL_RTS
#endif

#ifndef XIO_UART_MUTES_WHEN_USB_CONNECTED
#define XIO_UART_MUTES_WHEN_USB_CONNECTED  0                // UART will be muted when USB connected (off by default)
#endif

#ifndef JSON_VERBOSITY
#define JSON_VERBOSITY              JV_MESSAGES             // {jv: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE
#endif

#ifndef QUEUE_REPORT_VERBOSITY
#define QUEUE_REPORT_VERBOSITY      QR_OFF                  // {qv: QR_OFF, QR_SINGLE, QR_TRIPLE
#endif

#ifndef STATUS_REPORT_VERBOSITY
#define STATUS_REPORT_VERBOSITY     SR_FILTERED             // {sv: SR_OFF, SR_FILTERED, SR_VERBOSE
#endif

#ifndef STATUS_REPORT_MIN_MS
#define STATUS_REPORT_MIN_MS        200                     // (no JSON) milliseconds - enforces a viable minimum
#endif

#ifndef STATUS_REPORT_INTERVAL_MS
#define STATUS_REPORT_INTERVAL_MS   250                     // {si: milliseconds - set $SV=0 to disable
#endif

#ifndef STATUS_REPORT_DEFAULTS                              // {sr: See Status Reports wiki page
#define STATUS_REPORT_DEFAULTS "line","posx","posy","posz","posa","feed","vel","unit","coor","dist","admo","frmo","momo","stat"
// Alternate SRs that report in drawable units
//#define STATUS_REPORT_DEFAULTS "line","vel","mpox","mpoy","mpoz","mpoa","coor","ofsa","ofsx","ofsy","ofsz","dist","unit","stat","homz","homy","homx","momo"
#endif

// *** Gcode Startup Defaults *** //

#ifndef GCODE_DEFAULT_UNITS
#define GCODE_DEFAULT_UNITS         MILLIMETERS             // {gun: MILLIMETERS or INCHES
#endif

#ifndef GCODE_DEFAULT_PLANE
#define GCODE_DEFAULT_PLANE         CANON_PLANE_XY          // {gpl: CANON_PLANE_XY, CANON_PLANE_XZ, or CANON_PLANE_YZ
#endif

#ifndef GCODE_DEFAULT_COORD_SYSTEM
#define GCODE_DEFAULT_COORD_SYSTEM  G54                     // {gco: G54, G55, G56, G57, G58 or G59
#endif

#ifndef GCODE_DEFAULT_PATH_CONTROL
#define GCODE_DEFAULT_PATH_CONTROL  PATH_CONTINUOUS         // {gpa: PATH_EXACT_PATH, PATH_EXACT_STOP, PATH_CONTINUOUS
#endif

#ifndef GCODE_DEFAULT_DISTANCE_MODE
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_MODE           // {gdi: ABSOLUTE_MODE, INCREMENTAL_MODE
#endif


//*****************************************************************************
//*** Motor Settings **********************************************************
//*****************************************************************************

// MOTOR 1
#ifndef M1_MOTOR_MAP
#define M1_MOTOR_MAP                AXIS_X                  // {1ma: AXIS_X, AXIS_Y...
#endif
#ifndef M1_STEP_ANGLE
#define M1_STEP_ANGLE               1.8                     // {1sa: degrees per step
#endif
#ifndef M1_TRAVEL_PER_REV
#define M1_TRAVEL_PER_REV           1.25                    // {1tr:  1.25 is a typical value for a screw axis
#endif
#ifndef M1_MICROSTEPS
#define M1_MICROSTEPS               8                       // {1mi:  1,2,4,8,    16,32 (G2 ONLY)
#endif
#ifndef M1_STEPS_PER_UNIT
#define M1_STEPS_PER_UNIT           0                       // {1su:  steps to issue per unit of length or degrees of rotation
#endif
#ifndef M1_POLARITY
#define M1_POLARITY                 0                       // {1po:  0=normal direction, 1=inverted direction
#endif
#ifndef M1_POWER_MODE
#define M1_POWER_MODE               MOTOR_DISABLED          // {1pm:  MOTOR_DISABLED, MOTOR_ALWAYS_POWERED, MOTOR_POWERED_IN_CYCLE, MOTOR_POWERED_ONLY_WHEN_MOVING
#endif
#ifndef M1_POWER_LEVEL
#define M1_POWER_LEVEL              0.0                     // {1pl:   0.0=no power, 1.0=max power
#endif

// MOTOR 2
#ifndef M2_MOTOR_MAP
#define M2_MOTOR_MAP                AXIS_Y
#endif
#ifndef M2_STEP_ANGLE
#define M2_STEP_ANGLE               1.8
#endif
#ifndef M2_TRAVEL_PER_REV
#define M2_TRAVEL_PER_REV           40.00
#endif
#ifndef M2_MICROSTEPS
#define M2_MICROSTEPS               8
#endif
#ifndef M2_STEPS_PER_UNIT
#define M2_STEPS_PER_UNIT           0 
#endif
#ifndef M2_POLARITY
#define M2_POLARITY                 0
#endif
#ifndef M2_POWER_MODE
#define M2_POWER_MODE               MOTOR_DISABLED
#endif
#ifndef M2_POWER_LEVEL
#define M2_POWER_LEVEL              0.0
#endif

// MOTOR 3
#ifndef M3_MOTOR_MAP
#define M3_MOTOR_MAP                AXIS_Z
#endif
#ifndef M3_STEP_ANGLE
#define M3_STEP_ANGLE               1.8
#endif
#ifndef M3_TRAVEL_PER_REV
#define M3_TRAVEL_PER_REV           1.25                    // 1.25 is a typical value for a screw axis
#endif
#ifndef M3_MICROSTEPS
#define M3_MICROSTEPS               8
#endif
#ifndef M3_STEPS_PER_UNIT
#define M3_STEPS_PER_UNIT           0
#endif
#ifndef M3_POLARITY
#define M3_POLARITY                 1
#endif
#ifndef M3_POWER_MODE
#define M3_POWER_MODE               MOTOR_DISABLED
#endif
#ifndef M3_POWER_LEVEL
#define M3_POWER_LEVEL              0.0
#endif

// MOTOR 4
#ifndef M4_MOTOR_MAP
#define M4_MOTOR_MAP                AXIS_A
#endif
#ifndef M4_STEP_ANGLE
#define M4_STEP_ANGLE               1.8
#endif
#ifndef M4_TRAVEL_PER_REV
#define M4_TRAVEL_PER_REV           360.0                   // in degrees if rotary axis (ABC)
#endif
#ifndef M4_MICROSTEPS
#define M4_MICROSTEPS               8
#endif
#ifndef M4_STEPS_PER_UNIT
#define M4_STEPS_PER_UNIT           0
#endif
#ifndef M4_POLARITY
#define M4_POLARITY                 0
#endif
#ifndef M4_POWER_MODE
#define M4_POWER_MODE               MOTOR_DISABLED
#endif
#ifndef M4_POWER_LEVEL
#define M4_POWER_LEVEL              0.0
#endif

// MOTOR 5
#ifndef M5_MOTOR_MAP
#define M5_MOTOR_MAP                AXIS_B
#endif
#ifndef M5_STEP_ANGLE
#define M5_STEP_ANGLE               1.8
#endif
#ifndef M5_TRAVEL_PER_REV
#define M5_TRAVEL_PER_REV           360.0
#endif
#ifndef M5_MICROSTEPS
#define M5_MICROSTEPS               8
#endif
#ifndef M5_STEPS_PER_UNIT
#define M5_STEPS_PER_UNIT           0
#endif
#ifndef M5_POLARITY
#define M5_POLARITY                 0
#endif
#ifndef M5_POWER_MODE
#define M5_POWER_MODE               MOTOR_DISABLED
#endif
#ifndef M5_POWER_LEVEL
#define M5_POWER_LEVEL              0.0
#endif

// MOTOR 6
#ifndef M6_MOTOR_MAP
#define M6_MOTOR_MAP                AXIS_C
#endif
#ifndef M6_STEP_ANGLE
#define M6_STEP_ANGLE               1.8
#endif
#ifndef M6_TRAVEL_PER_REV
#define M6_TRAVEL_PER_REV           360.0
#endif
#ifndef M6_MICROSTEPS
#define M6_MICROSTEPS               8
#endif
#ifndef M6_STEPS_PER_UNIT
#define M6_STEPS_PER_UNIT           0
#endif
#ifndef M6_POLARITY
#define M6_POLARITY                 0
#endif
#ifndef M6_POWER_MODE
#define M6_POWER_MODE               MOTOR_DISABLED
#endif
#ifndef M6_POWER_LEVEL
#define M6_POWER_LEVEL              0.0
#endif

//*****************************************************************************
//*** Axis Settings ***********************************************************
//*****************************************************************************

// X AXIS
#ifndef X_AXIS_MODE
#define X_AXIS_MODE                 AXIS_DISABLED           // {xam:  see canonical_machine.h cmAxisMode for valid values
#endif
#ifndef X_VELOCITY_MAX
#define X_VELOCITY_MAX              1000.0                  // {xvm:  G0 max velocity in mm/min
#endif
#ifndef X_FEEDRATE_MAX
#define X_FEEDRATE_MAX              1000.0                  // {xfr:  G1 max feed rate in mm/min
#endif
#ifndef X_TRAVEL_MIN
#define X_TRAVEL_MIN                0.0                     // {xtn:  minimum travel for soft limits
#endif
#ifndef X_TRAVEL_MAX
#define X_TRAVEL_MAX                0.0                     // {xtm:  travel between switches or crashes
#endif
#ifndef X_JERK_MAX
#define X_JERK_MAX                  1000.0                  // {xjm:
#endif
#ifndef X_JERK_HIGH_SPEED
#define X_JERK_HIGH_SPEED           1000.0                  // {xjh:
#endif
#ifndef X_HOMING_INPUT
#define X_HOMING_INPUT              0                       // {xhi:  input used for homing or 0 to disable
#endif
#ifndef X_HOMING_DIRECTION
#define X_HOMING_DIRECTION          0                       // {xhd:  0=search moves negative, 1= search moves positive
#endif
#ifndef X_SEARCH_VELOCITY
#define X_SEARCH_VELOCITY           500.0                   // {xsv:  minus means move to minimum switch
#endif
#ifndef X_LATCH_VELOCITY
#define X_LATCH_VELOCITY            100.0                   // {xlv:  mm/min
#endif
#ifndef X_LATCH_BACKOFF
#define X_LATCH_BACKOFF             4.0                     // {xlb:  mm
#endif
#ifndef X_ZERO_BACKOFF
#define X_ZERO_BACKOFF              2.0                     // {xzb:  mm
#endif

// Y AXIS
#ifndef Y_AXIS_MODE
#define Y_AXIS_MODE                 AXIS_DISABLED
#endif
#ifndef Y_VELOCITY_MAX
#define Y_VELOCITY_MAX              1000.0
#endif
#ifndef Y_FEEDRATE_MAX
#define Y_FEEDRATE_MAX              1000.0
#endif
#ifndef Y_TRAVEL_MIN
#define Y_TRAVEL_MIN                0.0
#endif
#ifndef Y_TRAVEL_MAX
#define Y_TRAVEL_MAX                0.0
#endif
#ifndef Y_JERK_MAX
#define Y_JERK_MAX                  1000.0
#endif
#ifndef Y_JERK_HIGH_SPEED
#define Y_JERK_HIGH_SPEED           1000.0
#endif
#ifndef Y_HOMING_INPUT
#define Y_HOMING_INPUT              0
#endif
#ifndef Y_HOMING_DIRECTION
#define Y_HOMING_DIRECTION          0
#endif
#ifndef Y_SEARCH_VELOCITY
#define Y_SEARCH_VELOCITY           500.0
#endif
#ifndef Y_LATCH_VELOCITY
#define Y_LATCH_VELOCITY            100.0
#endif
#ifndef Y_LATCH_BACKOFF
#define Y_LATCH_BACKOFF             4.0
#endif
#ifndef Y_ZERO_BACKOFF
#define Y_ZERO_BACKOFF              2.0
#endif

// Z AXIS
#ifndef Z_AXIS_MODE
#define Z_AXIS_MODE                 AXIS_DISABLED
#endif
#ifndef Z_VELOCITY_MAX
#define Z_VELOCITY_MAX              1000.0
#endif
#ifndef Z_FEEDRATE_MAX
#define Z_FEEDRATE_MAX              1000.0
#endif
#ifndef Z_TRAVEL_MAX
#define Z_TRAVEL_MAX                0.0
#endif
#ifndef Z_TRAVEL_MIN
#define Z_TRAVEL_MIN                0.0
#endif
#ifndef Z_JERK_MAX
#define Z_JERK_MAX                  500.0
#endif
#ifndef Z_JERK_HIGH_SPEED
#define Z_JERK_HIGH_SPEED           500.0
#endif
#ifndef Z_HOMING_INPUT
#define Z_HOMING_INPUT              0
#endif
#ifndef Z_HOMING_DIRECTION
#define Z_HOMING_DIRECTION          0
#endif
#ifndef Z_SEARCH_VELOCITY
#define Z_SEARCH_VELOCITY           250.0
#endif
#ifndef Z_LATCH_VELOCITY
#define Z_LATCH_VELOCITY            25.0
#endif
#ifndef Z_LATCH_BACKOFF
#define Z_LATCH_BACKOFF             4.0
#endif
#ifndef Z_ZERO_BACKOFF
#define Z_ZERO_BACKOFF              2.0
#endif

/***************************************************************************************
 * Rotary values can be chosen to make the motor react the same as X for testing
 * To calculate the speeds here, in Wolfram Alpha-speak:
 *
 *   c=2*pi*r, r=0.609, d=c/360, s=((S*60)/d), S=40 for s
 *
 * Change r to A_RADIUS, and S to the desired speed, in mm/s or mm/s/s/s.
 *
 * It will return s= as the value you want to enter.
 *
 * If the value is over 1 million, the code will divide it by 1 million,
 * so you have to pre-multiply it by 1000000.0. (The value is in millions, btw.)
 *
 * Note that you need floating point values to always have a .0 at the end!

#define A_AXIS_MODE                 AXIS_RADIUS
#define A_RADIUS                    (M1_TRAVEL_PER_REV/(2*3.14159628))
#define A_VELOCITY_MAX              ((X_VELOCITY_MAX/M1_TRAVEL_PER_REV)*360) // set to the same speed as X axis
#define A_FEEDRATE_MAX              A_VELOCITY_MAX
#define A_TRAVEL_MIN                -1.0                                     // min/max the same means infinite, no limit
#define A_TRAVEL_MAX                -1.0
#define A_JERK_MAX                  (X_JERK_MAX*(360/M1_TRAVEL_PER_REV))
#define A_JERK_HIGH_SPEED           A_JERK_MAX
#define A_HOMING_INPUT              0
#define A_HOMING_DIRECTION                0
#define A_SEARCH_VELOCITY           (A_VELOCITY_MAX * 0.500)
#define A_LATCH_VELOCITY            (A_VELOCITY_MAX * 0.100)
#define A_LATCH_BACKOFF             5.0
#define A_ZERO_BACKOFF              2.0

 ***************************************************************************************/

// A AXIS
#ifndef A_AXIS_MODE
#define A_AXIS_MODE                 AXIS_DISABLED
#endif
#ifndef A_RADIUS
#define A_RADIUS                    (M1_TRAVEL_PER_REV/(2*3.14159628))
#endif
#ifndef A_VELOCITY_MAX
#define A_VELOCITY_MAX              ((X_VELOCITY_MAX/M1_TRAVEL_PER_REV)*360) // set to the same speed as X axis
#endif
#ifndef A_FEEDRATE_MAX
#define A_FEEDRATE_MAX              A_VELOCITY_MAX
#endif
#ifndef A_TRAVEL_MIN
#define A_TRAVEL_MIN                -1.0                    // min/max the same means infinite, no limit
#endif
#ifndef A_TRAVEL_MAX
#define A_TRAVEL_MAX                -1.0
#endif
#ifndef A_JERK_MAX
#define A_JERK_MAX                  (X_JERK_MAX*(360/M1_TRAVEL_PER_REV))
#endif
#ifndef A_JERK_HIGH_SPEED
#define A_JERK_HIGH_SPEED           A_JERK_MAX
#endif
#ifndef A_HOMING_INPUT
#define A_HOMING_INPUT              0
#endif
#ifndef A_HOMING_DIRECTION
#define A_HOMING_DIRECTION          0
#endif
#ifndef A_SEARCH_VELOCITY
#define A_SEARCH_VELOCITY           (A_VELOCITY_MAX * 0.500)
#endif
#ifndef A_LATCH_VELOCITY
#define A_LATCH_VELOCITY            (A_VELOCITY_MAX * 0.100)
#endif
#ifndef A_LATCH_BACKOFF
#define A_LATCH_BACKOFF             5.0
#endif
#ifndef A_ZERO_BACKOFF
#define A_ZERO_BACKOFF              2.0
#endif

// B AXIS
#ifndef B_AXIS_MODE
#define B_AXIS_MODE                 AXIS_DISABLED
#endif
#ifndef B_RADIUS
#define B_RADIUS                    (M1_TRAVEL_PER_REV/(2*3.14159628))
#endif
#ifndef B_VELOCITY_MAX
#define B_VELOCITY_MAX              ((X_VELOCITY_MAX/M1_TRAVEL_PER_REV)*360)
#endif
#ifndef B_FEEDRATE_MAX
#define B_FEEDRATE_MAX              B_VELOCITY_MAX
#endif
#ifndef B_TRAVEL_MIN
#define B_TRAVEL_MIN                -1.0
#endif
#ifndef B_TRAVEL_MAX
#define B_TRAVEL_MAX                -1.0
#endif
#ifndef B_JERK_MAX
#define B_JERK_MAX                  (X_JERK_MAX*(360/M1_TRAVEL_PER_REV))
#endif
#ifndef B_JERK_HIGH_SPEED
#define B_JERK_HIGH_SPEED           B_JERK_MAX
#endif
#ifndef B_HOMING_INPUT
#define B_HOMING_INPUT              0
#endif
#ifndef B_HOMING_DIRECTION
#define B_HOMING_DIRECTION          0
#endif
#ifndef B_SEARCH_VELOCITY
#define B_SEARCH_VELOCITY           (A_VELOCITY_MAX * 0.500)
#endif
#ifndef B_LATCH_VELOCITY
#define B_LATCH_VELOCITY            (A_VELOCITY_MAX * 0.100)
#endif
#ifndef B_LATCH_BACKOFF
#define B_LATCH_BACKOFF             5.0
#endif
#ifndef B_ZERO_BACKOFF
#define B_ZERO_BACKOFF              2.0
#endif

// C AXIS
#ifndef C_AXIS_MODE
#define C_AXIS_MODE                 AXIS_DISABLED
#endif
#ifndef C_RADIUS
#define C_RADIUS                    (M1_TRAVEL_PER_REV/(2*3.14159628))
#endif
#ifndef C_VELOCITY_MAX
#define C_VELOCITY_MAX              ((X_VELOCITY_MAX/M1_TRAVEL_PER_REV)*360)
#endif
#ifndef C_FEEDRATE_MAX
#define C_FEEDRATE_MAX              C_VELOCITY_MAX
#endif
#ifndef C_TRAVEL_MIN
#define C_TRAVEL_MIN                -1.0
#endif
#ifndef C_TRAVEL_MAX
#define C_TRAVEL_MAX                -1.0
#endif
#ifndef C_JERK_MAX
#define C_JERK_MAX                  (X_JERK_MAX*(360/M1_TRAVEL_PER_REV))
#endif
#ifndef C_JERK_HIGH_SPEED
#define C_JERK_HIGH_SPEED           C_JERK_MAX
#endif
#ifndef C_HOMING_INPUT
#define C_HOMING_INPUT              0
#endif
#ifndef C_HOMING_DIRECTION
#define C_HOMING_DIRECTION          0
#endif
#ifndef C_SEARCH_VELOCITY
#define C_SEARCH_VELOCITY           (A_VELOCITY_MAX * 0.500)
#endif
#ifndef C_LATCH_VELOCITY
#define C_LATCH_VELOCITY            (A_VELOCITY_MAX * 0.100)
#endif
#ifndef C_LATCH_BACKOFF
#define C_LATCH_BACKOFF             5.0
#endif
#ifndef C_ZERO_BACKOFF
#define C_ZERO_BACKOFF              2.0
#endif


//*****************************************************************************
//*** GPIO Input / Output Settings ********************************************
//*****************************************************************************

// DIGITAL INPUTS
// Set to allow the board to function if not otherwise set up
// (least disruptive settings)

/* Legend:
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
#ifndef DI1_MODE
#define DI1_MODE                    NORMALLY_OPEN
#endif
#ifndef DI1_ACTION
#define DI1_ACTION                  INPUT_ACTION_NONE
#endif
#ifndef DI1_FUNCTION
#define DI1_FUNCTION                INPUT_FUNCTION_NONE
#endif

// Xmax
#ifndef DI2_MODE
#define DI2_MODE                    NORMALLY_OPEN
#endif
#ifndef DI2_ACTION
#define DI2_ACTION                  INPUT_ACTION_NONE
#endif
#ifndef DI2_FUNCTION
#define DI2_FUNCTION                INPUT_FUNCTION_NONE
#endif

// Ymin
#ifndef DI3_MODE
#define DI3_MODE                    NORMALLY_OPEN
#endif
#ifndef DI3_ACTION
#define DI3_ACTION                  INPUT_ACTION_NONE
#endif
#ifndef DI3_FUNCTION
#define DI3_FUNCTION                INPUT_FUNCTION_NONE
#endif

// Ymax
#ifndef DI4_MODE
#define DI4_MODE                    NORMALLY_OPEN
#endif
#ifndef DI4_ACTION
#define DI4_ACTION                  INPUT_ACTION_NONE
#endif
#ifndef DI4_FUNCTION
#define DI4_FUNCTION                INPUT_FUNCTION_NONE
#endif

// Zmin
#ifndef DI5_MODE
#define DI5_MODE                    NORMALLY_OPEN
#endif
#ifndef DI5_ACTION
#define DI5_ACTION                  INPUT_ACTION_NONE
#endif
#ifndef DI5_FUNCTION
#define DI5_FUNCTION                INPUT_FUNCTION_NONE
#endif

// Zmax
#ifndef DI6_MODE
#define DI6_MODE                    NORMALLY_OPEN
#endif
#ifndef DI6_ACTION
#define DI6_ACTION                  INPUT_ACTION_NONE
#endif
#ifndef DI6_FUNCTION
#define DI6_FUNCTION                INPUT_FUNCTION_NONE
#endif

// Amin
#ifndef DI7_MODE
#define DI7_MODE                    NORMALLY_OPEN
#endif
#ifndef DI7_ACTION
#define DI7_ACTION                  INPUT_ACTION_NONE
#endif
#ifndef DI7_FUNCTION
#define DI7_FUNCTION                INPUT_FUNCTION_NONE
#endif

// Amax
#ifndef DI8_MODE
#define DI8_MODE                    NORMALLY_OPEN
#endif
#ifndef DI8_ACTION
#define DI8_ACTION                  INPUT_ACTION_NONE
#endif
#ifndef DI8_FUNCTION
#define DI8_FUNCTION                INPUT_FUNCTION_NONE
#endif

// Safety line
#ifndef DI9_MODE
#define DI9_MODE                    NORMALLY_CLOSED     // Normally closed
#endif
#ifndef DI9_ACTION
#define DI9_ACTION                  INPUT_ACTION_NONE
#endif
#ifndef DI9_FUNCTION
#define DI9_FUNCTION                INPUT_FUNCTION_NONE
#endif

#ifndef DI10_MODE
#define DI10_MODE                   IO_ACTIVE_LOW     // Normally open
#endif
#ifndef DI10_ACTION
#define DI10_ACTION                 INPUT_ACTION_NONE
#endif
#ifndef DI10_FUNCTION
#define DI10_FUNCTION               INPUT_FUNCTION_NONE
#endif

#ifndef DI11_MODE
#define DI11_MODE                   IO_ACTIVE_LOW     // Normally open
#endif
#ifndef DI11_ACTION
#define DI11_ACTION                 INPUT_ACTION_NONE
#endif
#ifndef DI11_FUNCTION
#define DI11_FUNCTION               INPUT_FUNCTION_NONE
#endif

#ifndef DI12_MODE
#define DI12_MODE                   IO_ACTIVE_LOW     // Normally open
#endif
#ifndef DI12_ACTION
#define DI12_ACTION                 INPUT_ACTION_NONE
#endif
#ifndef DI12_FUNCTION
#define DI12_FUNCTION               INPUT_FUNCTION_NONE
#endif

// DIGITAL OUTPUTS - CUrrently these are hard-wired to extruders

//Extruder1_PWM
#ifndef DO1_MODE
#define DO1_MODE                    IO_ACTIVE_HIGH
#endif

//Extruder2_PWM
#ifndef DO2_MODE
#define DO2_MODE                    IO_ACTIVE_HIGH
#endif

//Fan1A_PWM
#ifndef DO3_MODE
#define DO3_MODE                    IO_ACTIVE_HIGH
#endif

//Fan1B_PWM
#ifndef DO4_MODE
#define DO4_MODE                    IO_ACTIVE_HIGH
#endif

#ifndef DO5_MODE
#define DO5_MODE                    IO_ACTIVE_HIGH
#endif
#ifndef DO6_MODE
#define DO6_MODE                    IO_ACTIVE_HIGH
#endif
#ifndef DO7_MODE
#define DO7_MODE                    IO_ACTIVE_HIGH
#endif
#ifndef DO8_MODE
#define DO8_MODE                    IO_ACTIVE_HIGH
#endif

//SAFEin (Output) signal
#ifndef DO9_MODE
#define DO9_MODE                    IO_ACTIVE_HIGH
#endif

#ifndef DO10_MODE
#define DO10_MODE                   IO_ACTIVE_HIGH
#endif

//Header Bed FET
#ifndef DO11_MODE
#define DO11_MODE                   IO_ACTIVE_HIGH
#endif

//Indicator_LED
#ifndef DO12_MODE
#define DO12_MODE                   IO_ACTIVE_HIGH
#endif

#ifndef DO13_MODE
#define DO13_MODE                   IO_ACTIVE_HIGH
#endif

// *** PWM Settings *** //

#ifndef P1_PWM_FREQUENCY
#define P1_PWM_FREQUENCY            100                   // in Hz
#endif
#ifndef P1_CW_SPEED_LO
#define P1_CW_SPEED_LO              7900                  // in RPM (arbitrary units)
#endif
#ifndef P1_CW_SPEED_HI
#define P1_CW_SPEED_HI              12800
#endif
#ifndef P1_CW_PHASE_LO
#define P1_CW_PHASE_LO              0.13                    // phase [0..1]
#endif
#ifndef P1_CW_PHASE_HI
#define P1_CW_PHASE_HI              0.17
#endif
#ifndef P1_CCW_SPEED_LO
#define P1_CCW_SPEED_LO             0.0
#endif
#ifndef P1_CCW_SPEED_HI
#define P1_CCW_SPEED_HI             0.0
#endif
#ifndef P1_CCW_PHASE_LO
#define P1_CCW_PHASE_LO             0.1
#endif
#ifndef P1_CCW_PHASE_HI
#define P1_CCW_PHASE_HI             0.1
#endif
#ifndef P1_PWM_PHASE_OFF
#define P1_PWM_PHASE_OFF            0.1
#endif

// *** Heater Settings - relevant to 3dp machines *** //


#ifndef MIN_FAN_TEMP
#define MIN_FAN_TEMP                40.0     // Temperature that the upper-extruder fan starts
#endif
#ifndef MIN_FAN_VALUE
#define MIN_FAN_VALUE               0.4      // Minimum output value (0.0-1.0) of the upper-extruder fan
#endif
#ifndef MAX_FAN_VALUE
#define MAX_FAN_VALUE               1.0      // Maximum output value (0.0-1.0) of the upper-extruder fan
#endif
#ifndef MAX_FAN_TEMP
#define MAX_FAN_TEMP                150.0    // Temperature at and above which the upper-extruder fan is at 1.0
#endif
#ifndef H1_DEFAULT_ENABLE
#define H1_DEFAULT_ENABLE           false
#endif
#ifndef H1_DEFAULT_P
#define H1_DEFAULT_P                9.0
#endif
#ifndef H1_DEFAULT_I
#define H1_DEFAULT_I                0.12
#endif
#ifndef H1_DEFAULT_D
#define H1_DEFAULT_D                400.0
#endif

#ifndef H2_DEFAULT_ENABLE
#define H2_DEFAULT_ENABLE           false
#endif
#ifndef H2_DEFAULT_P
#define H2_DEFAULT_P                9.0
#endif
#ifndef H2_DEFAULT_I
#define H2_DEFAULT_I                0.12
#endif
#ifndef H2_DEFAULT_D
#define H2_DEFAULT_D                400.0
#endif

#ifndef H3_DEFAULT_ENABLE
#define H3_DEFAULT_ENABLE           false
#endif
#ifndef H3_DEFAULT_P
#define H3_DEFAULT_P                9.0
#endif
#ifndef H3_DEFAULT_I
#define H3_DEFAULT_I                0.12
#endif
#ifndef H3_DEFAULT_D
#define H3_DEFAULT_D                400.0
#endif

// *** DEFAULT COORDINATE SYSTEM OFFSETS ***

#ifndef G54_X_OFFSET
#define G54_X_OFFSET 0    // G54 is often set to all zeros
#endif
#ifndef G54_Y_OFFSET
#define G54_Y_OFFSET 0
#endif
#ifndef G54_Z_OFFSET
#define G54_Z_OFFSET 0
#endif
#ifndef G54_A_OFFSET
#define G54_A_OFFSET 0
#endif
#ifndef G54_B_OFFSET
#define G54_B_OFFSET 0
#endif
#ifndef G54_C_OFFSET
#define G54_C_OFFSET 0
#endif

#ifndef G55_X_OFFSET
#define G55_X_OFFSET 0
#endif
#ifndef G55_Y_OFFSET
#define G55_Y_OFFSET 0
#endif
#ifndef G55_Z_OFFSET
#define G55_Z_OFFSET 0
#endif
#ifndef G55_A_OFFSET
#define G55_A_OFFSET 0
#endif
#ifndef G55_B_OFFSET
#define G55_B_OFFSET 0
#endif
#ifndef G55_C_OFFSET
#define G55_C_OFFSET 0
#endif

#ifndef G56_X_OFFSET
#define G56_X_OFFSET 0
#endif
#ifndef G56_Y_OFFSET
#define G56_Y_OFFSET 0
#endif
#ifndef G56_Z_OFFSET
#define G56_Z_OFFSET 0
#endif
#ifndef G56_A_OFFSET
#define G56_A_OFFSET 0
#endif
#ifndef G56_B_OFFSET
#define G56_B_OFFSET 0
#endif
#ifndef G56_C_OFFSET
#define G56_C_OFFSET 0
#endif

#ifndef G57_X_OFFSET
#define G57_X_OFFSET 0
#endif
#ifndef G57_Y_OFFSET
#define G57_Y_OFFSET 0
#endif
#ifndef G57_Z_OFFSET
#define G57_Z_OFFSET 0
#endif
#ifndef G57_A_OFFSET
#define G57_A_OFFSET 0
#endif
#ifndef G57_B_OFFSET
#define G57_B_OFFSET 0
#endif
#ifndef G57_C_OFFSET
#define G57_C_OFFSET 0
#endif

#ifndef G58_X_OFFSET
#define G58_X_OFFSET 0
#endif
#ifndef G58_Y_OFFSET
#define G58_Y_OFFSET 0
#endif
#ifndef G58_Z_OFFSET
#define G58_Z_OFFSET 0
#endif
#ifndef G58_A_OFFSET
#define G58_A_OFFSET 0
#endif
#ifndef G58_B_OFFSET
#define G58_B_OFFSET 0
#endif
#ifndef G58_C_OFFSET
#define G58_C_OFFSET 0
#endif

#ifndef G59_X_OFFSET
#define G59_X_OFFSET 0
#endif
#ifndef G59_Y_OFFSET
#define G59_Y_OFFSET 0
#endif
#ifndef G59_Z_OFFSET
#define G59_Z_OFFSET 0
#endif
#ifndef G59_A_OFFSET
#define G59_A_OFFSET 0
#endif
#ifndef G59_B_OFFSET
#define G59_B_OFFSET 0
#endif
#ifndef G59_C_OFFSET
#define G59_C_OFFSET 0
#endif

// *** User-Defined Data Defaults *** //

#ifndef USER_DATA_A0
#define USER_DATA_A0 0
#endif
#ifndef USER_DATA_A1
#define USER_DATA_A1 0
#endif
#ifndef USER_DATA_A2
#define USER_DATA_A2 0
#endif
#ifndef USER_DATA_A3
#define USER_DATA_A3 0
#endif

#ifndef USER_DATA_B0
#define USER_DATA_B0 0
#endif
#ifndef USER_DATA_B1
#define USER_DATA_B1 0
#endif
#ifndef USER_DATA_B2
#define USER_DATA_B2 0
#endif
#ifndef USER_DATA_B3
#define USER_DATA_B3 0
#endif

#ifndef USER_DATA_C0
#define USER_DATA_C0 0
#endif
#ifndef USER_DATA_C1
#define USER_DATA_C1 0
#endif
#ifndef USER_DATA_C2
#define USER_DATA_C2 0
#endif
#ifndef USER_DATA_C3
#define USER_DATA_C3 0
#endif

#ifndef USER_DATA_D0
#define USER_DATA_D0 0
#endif
#ifndef USER_DATA_D1
#define USER_DATA_D1 0
#endif
#ifndef USER_DATA_D2
#define USER_DATA_D2 0
#endif
#ifndef USER_DATA_D3
#define USER_DATA_D3 0
#endif
