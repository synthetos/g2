/*
 * settings_default.h - default machine profile - Set for Shapeoko2
 * This file is part of the g2core project
 *
 * Copyright (c) 2012 - 2018 Alden S. Hart, Jr.
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

#ifndef SPINDLE_MODE
#define SPINDLE_MODE                1       // {spmo; 0=diabled, 1=plan to stop, 2=continuous
#endif

#ifndef SPINDLE_ENABLE_POLARITY
#define SPINDLE_ENABLE_POLARITY     1       // 0=active low, 1=active high
#endif

#ifndef SPINDLE_DIR_POLARITY
#define SPINDLE_DIR_POLARITY        0       // {spdp: 0=clockwise is low, 1=clockwise is high
#endif

#ifndef SPINDLE_PAUSE_ON_HOLD
#define SPINDLE_PAUSE_ON_HOLD       false   // {spph:
#endif

#ifndef SPINDLE_SPINUP_DELAY
#ifdef SPINDLE_DWELL_TIME
#define SPINDLE_SPINUP_DELAY SPINDLE_DWELL_TIME
#warning Please update your settings! SPINDLE_DWELL_TIME is now SPINDLE_SPINUP_DELAY
#else
#define SPINDLE_SPINUP_DELAY        0     // {spde:
#endif
#endif

#ifndef SPINDLE_OVERRIDE_ENABLE
#define SPINDLE_OVERRIDE_ENABLE 1
#endif

#ifndef SPINDLE_OVERRIDE_FACTOR
#define SPINDLE_OVERRIDE_FACTOR 1.0
#endif

#ifndef SPINDLE_OVERRIDE_MIN
#define SPINDLE_OVERRIDE_MIN FEED_OVERRIDE_MIN
#endif

#ifndef SPINDLE_OVERRIDE_MAX
#define SPINDLE_OVERRIDE_MAX FEED_OVERRIDE_MAX
#endif

#ifndef SPINDLE_DWELL_MAX
#define SPINDLE_DWELL_MAX   10000000.0      // maximum allowable dwell time. May be overridden in settings files
#endif

#ifndef SPINDLE_SPEED_MIN
#define SPINDLE_SPEED_MIN           0.0     // {spsn:
#endif

#ifndef SPINDLE_SPEED_MAX
#define SPINDLE_SPEED_MAX     1000000.0     // {spsm:
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

#ifndef FEEDHOLD_Z_LIFT
#define FEEDHOLD_Z_LIFT             0       // {zl: mm to lift Z on feedhold
#endif

#ifndef PROBE_REPORT_ENABLE
#define PROBE_REPORT_ENABLE         true    // {prbr:
#endif

#ifndef MANUAL_FEEDRATE_OVERRIDE_ENABLE
#define MANUAL_FEEDRATE_OVERRIDE_ENABLE false
#endif

#ifndef MANUAL_FEEDRATE_OVERRIDE_PARAMETER
#define MANUAL_FEEDRATE_OVERRIDE_PARAMETER 1.00
#endif

// *** Communications and Reporting Settings *** //

#ifndef USB_SERIAL_PORTS_EXPOSED
#define USB_SERIAL_PORTS_EXPOSED   1                        // Valid options are 1 or 2, only!
#endif

#ifndef XIO_ENABLE_FLOW_CONTROL
#define XIO_ENABLE_FLOW_CONTROL     FLOW_CONTROL_RTS        // {ex: FLOW_CONTROL_OFF, FLOW_CONTROL_XON, FLOW_CONTROL_RTS
#endif

#ifndef COMM_MODE
#define COMM_MODE                   JSON_MODE               // {ej: TEXT_MODE, JSON_MODE
#endif

#ifndef TEXT_VERBOSITY
#define TEXT_VERBOSITY              TV_VERBOSE              // {tv: TV_SILENT, TV_VERBOSE
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
#define STATUS_REPORT_MIN_MS        100                     // (no JSON) milliseconds - enforces a viable minimum
#endif

#ifndef STATUS_REPORT_INTERVAL_MS
#define STATUS_REPORT_INTERVAL_MS   250                     // {si: milliseconds - set $SV=0 to disable
#endif

#ifndef STATUS_REPORT_DEFAULTS                              // {sr: See Status Reports wiki page
#define STATUS_REPORT_DEFAULTS "line","posx","posy","posz","posa","feed","vel","unit","coor","dist","admo","frmo","momo","stat"
// Alternate SRs that report in drawable units
//#define STATUS_REPORT_DEFAULTS "line","vel","mpox","mpoy","mpoz","mpoa","coor","ofsa","ofsx","ofsy","ofsz","dist","unit","stat","homz","homy","homx","momo"
#endif

#ifndef MARLIN_COMPAT_ENABLED
#define MARLIN_COMPAT_ENABLED       false                   // boolean, either true or false
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
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_DISTANCE_MODE  // {gdi: ABSOLUTE_DISTANCE_MODE, INCREMENTAL_DISTANCE_MODE
#endif


//*****************************************************************************
//*** Motor Settings **********************************************************
//*****************************************************************************

// KINEMATICS (which may later change the following values)
#define KINE_OTHER -1
#define KINE_CARTESIAN 0
#define KINE_CORE_XY 1
#define KINE_FOUR_CABLE 2
#define KINE_PRESSURE 3

#ifndef KINEMATICS
#define KINEMATICS KINE_CARTESIAN
#endif


// MOTOR 1
#ifndef M1_MOTOR_MAP
#define M1_MOTOR_MAP                AXIS_X_EXTERNAL         // {1ma: AXIS_X, AXIS_Y...
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
#ifndef M1_ENABLE_POLARITY
#define M1_ENABLE_POLARITY          IO_ACTIVE_LOW           // {1ep:  IO_ACTIVE_LOW or IO_ACTIVE_HIGH
#endif
#ifndef M1_STEP_POLARITY
#define M1_STEP_POLARITY            IO_ACTIVE_HIGH          // {1ps:  IO_ACTIVE_LOW or IO_ACTIVE_HIGH
#endif
#ifndef M1_POWER_MODE
#define M1_POWER_MODE               MOTOR_DISABLED          // {1pm:  MOTOR_DISABLED, MOTOR_ALWAYS_POWERED, MOTOR_POWERED_IN_CYCLE, MOTOR_POWERED_ONLY_WHEN_MOVING
#endif
#ifndef M1_POWER_LEVEL
#define M1_POWER_LEVEL              0.0                     // {1pl:   0.0=no power, 1.0=max power
#endif
#ifndef M1_POWER_LEVEL_IDLE
#define M1_POWER_LEVEL_IDLE         (M1_POWER_LEVEL/2.0)
#endif

// MOTOR 2
#ifndef M2_MOTOR_MAP
#define M2_MOTOR_MAP                AXIS_Y_EXTERNAL
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
#ifndef M2_ENABLE_POLARITY
#define M2_ENABLE_POLARITY          IO_ACTIVE_LOW
#endif
#ifndef M2_STEP_POLARITY
#define M2_STEP_POLARITY            IO_ACTIVE_HIGH
#endif
#ifndef M2_POWER_MODE
#define M2_POWER_MODE               MOTOR_DISABLED
#endif
#ifndef M2_POWER_LEVEL
#define M2_POWER_LEVEL              0.0
#endif
#ifndef M2_POWER_LEVEL_IDLE
#define M2_POWER_LEVEL_IDLE         (M2_POWER_LEVEL/2.0)
#endif

// MOTOR 3
#ifndef M3_MOTOR_MAP
#define M3_MOTOR_MAP                AXIS_Z_EXTERNAL
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
#define M3_POLARITY                 0
#endif
#ifndef M3_ENABLE_POLARITY
#define M3_ENABLE_POLARITY          IO_ACTIVE_LOW
#endif
#ifndef M3_STEP_POLARITY
#define M3_STEP_POLARITY            IO_ACTIVE_HIGH
#endif
#ifndef M3_POWER_MODE
#define M3_POWER_MODE               MOTOR_DISABLED
#endif
#ifndef M3_POWER_LEVEL
#define M3_POWER_LEVEL              0.0
#endif
#ifndef M3_POWER_LEVEL_IDLE
#define M3_POWER_LEVEL_IDLE         (M3_POWER_LEVEL/2.0)
#endif

// MOTOR 4
#ifndef M4_MOTOR_MAP
#define M4_MOTOR_MAP                AXIS_A_EXTERNAL
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
#ifndef M4_ENABLE_POLARITY
#define M4_ENABLE_POLARITY          IO_ACTIVE_LOW
#endif
#ifndef M4_STEP_POLARITY
#define M4_STEP_POLARITY            IO_ACTIVE_HIGH
#endif
#ifndef M4_POWER_MODE
#define M4_POWER_MODE               MOTOR_DISABLED
#endif
#ifndef M4_POWER_LEVEL
#define M4_POWER_LEVEL              0.0
#endif
#ifndef M4_POWER_LEVEL_IDLE
#define M4_POWER_LEVEL_IDLE         (M4_POWER_LEVEL/2.0)
#endif

// MOTOR 5
#ifndef M5_MOTOR_MAP
#define M5_MOTOR_MAP                AXIS_B_EXTERNAL
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
#ifndef M5_ENABLE_POLARITY
#define M5_ENABLE_POLARITY          IO_ACTIVE_LOW
#endif
#ifndef M5_STEP_POLARITY
#define M5_STEP_POLARITY            IO_ACTIVE_HIGH
#endif
#ifndef M5_POWER_MODE
#define M5_POWER_MODE               MOTOR_DISABLED
#endif
#ifndef M5_POWER_LEVEL
#define M5_POWER_LEVEL              0.0
#endif
#ifndef M5_POWER_LEVEL_IDLE
#define M5_POWER_LEVEL_IDLE         (M5_POWER_LEVEL/2.0)
#endif

// MOTOR 6
#ifndef M6_MOTOR_MAP
#define M6_MOTOR_MAP                AXIS_C_EXTERNAL
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
#ifndef M6_ENABLE_POLARITY
#define M6_ENABLE_POLARITY          IO_ACTIVE_LOW
#endif
#ifndef M6_STEP_POLARITY
#define M6_STEP_POLARITY            IO_ACTIVE_HIGH
#endif
#ifndef M6_POWER_MODE
#define M6_POWER_MODE               MOTOR_DISABLED
#endif
#ifndef M6_POWER_LEVEL
#define M6_POWER_LEVEL              0.0
#endif
#ifndef M6_POWER_LEVEL_IDLE
#define M6_POWER_LEVEL_IDLE         (M6_POWER_LEVEL/2.0)
#endif

// TMC2130 config defaults
// START Generated with ${PROJECT_ROOT}/Resources/generate_motors_default_config.js
#ifndef M1_TMC2130_TPWMTHRS
#define M1_TMC2130_TPWMTHRS         1200                    // 1pth
#endif
#ifndef M1_TMC2130_TCOOLTHRS
#define M1_TMC2130_TCOOLTHRS        1000                    // 1cth
#endif
#ifndef M1_TMC2130_THIGH
#define M1_TMC2130_THIGH            10                      // 1hth
#endif
#ifndef M1_TMC2130_SGT
#define M1_TMC2130_SGT              4                       // 1sgt
#endif
#ifndef M1_TMC2130_TBL
#define M1_TMC2130_TBL              2                       // 1tbl
#endif
#ifndef M1_TMC2130_PWM_GRAD
#define M1_TMC2130_PWM_GRAD         1                       // 1pgrd
#endif
#ifndef M1_TMC2130_PWM_AMPL
#define M1_TMC2130_PWM_AMPL         200                     // 1pamp
#endif
#ifndef M1_TMC2130_HEND
#define M1_TMC2130_HEND             0                       // 1hend
#endif
#ifndef M1_TMC2130_HSTRT
#define M1_TMC2130_HSTRT            0                       // 1hsrt
#endif
#ifndef M1_TMC2130_SMIN
#define M1_TMC2130_SMIN             5                       // 1smin
#endif
#ifndef M1_TMC2130_SMAX
#define M1_TMC2130_SMAX             5                       // 1smax
#endif
#ifndef M1_TMC2130_SUP
#define M1_TMC2130_SUP              2                       // 1sup
#endif
#ifndef M1_TMC2130_SDN
#define M1_TMC2130_SDN              1                       // 1sdn
#endif

#ifndef M2_TMC2130_TPWMTHRS
#define M2_TMC2130_TPWMTHRS         1200                    // 2pth
#endif
#ifndef M2_TMC2130_TCOOLTHRS
#define M2_TMC2130_TCOOLTHRS        1000                    // 2cth
#endif
#ifndef M2_TMC2130_THIGH
#define M2_TMC2130_THIGH            10                      // 2hth
#endif
#ifndef M2_TMC2130_SGT
#define M2_TMC2130_SGT              4                       // 2sgt
#endif
#ifndef M2_TMC2130_TBL
#define M2_TMC2130_TBL              2                       // 2tbl
#endif
#ifndef M2_TMC2130_PWM_GRAD
#define M2_TMC2130_PWM_GRAD         1                       // 2pgrd
#endif
#ifndef M2_TMC2130_PWM_AMPL
#define M2_TMC2130_PWM_AMPL         200                     // 2pamp
#endif
#ifndef M2_TMC2130_HEND
#define M2_TMC2130_HEND             0                       // 2hend
#endif
#ifndef M2_TMC2130_HSTRT
#define M2_TMC2130_HSTRT            0                       // 2hsrt
#endif
#ifndef M2_TMC2130_SMIN
#define M2_TMC2130_SMIN             5                       // 2smin
#endif
#ifndef M2_TMC2130_SMAX
#define M2_TMC2130_SMAX             5                       // 2smax
#endif
#ifndef M2_TMC2130_SUP
#define M2_TMC2130_SUP              2                       // 2sup
#endif
#ifndef M2_TMC2130_SDN
#define M2_TMC2130_SDN              1                       // 2sdn
#endif

#ifndef M3_TMC2130_TPWMTHRS
#define M3_TMC2130_TPWMTHRS         1200                    // 3pth
#endif
#ifndef M3_TMC2130_TCOOLTHRS
#define M3_TMC2130_TCOOLTHRS        1000                    // 3cth
#endif
#ifndef M3_TMC2130_THIGH
#define M3_TMC2130_THIGH            10                      // 3hth
#endif
#ifndef M3_TMC2130_SGT
#define M3_TMC2130_SGT              4                       // 3sgt
#endif
#ifndef M3_TMC2130_TBL
#define M3_TMC2130_TBL              2                       // 3tbl
#endif
#ifndef M3_TMC2130_PWM_GRAD
#define M3_TMC2130_PWM_GRAD         1                       // 3pgrd
#endif
#ifndef M3_TMC2130_PWM_AMPL
#define M3_TMC2130_PWM_AMPL         200                     // 3pamp
#endif
#ifndef M3_TMC2130_HEND
#define M3_TMC2130_HEND             0                       // 3hend
#endif
#ifndef M3_TMC2130_HSTRT
#define M3_TMC2130_HSTRT            0                       // 3hsrt
#endif
#ifndef M3_TMC2130_SMIN
#define M3_TMC2130_SMIN             5                       // 3smin
#endif
#ifndef M3_TMC2130_SMAX
#define M3_TMC2130_SMAX             5                       // 3smax
#endif
#ifndef M3_TMC2130_SUP
#define M3_TMC2130_SUP              2                       // 3sup
#endif
#ifndef M3_TMC2130_SDN
#define M3_TMC2130_SDN              1                       // 3sdn
#endif

#ifndef M4_TMC2130_TPWMTHRS
#define M4_TMC2130_TPWMTHRS         1200                    // 4pth
#endif
#ifndef M4_TMC2130_TCOOLTHRS
#define M4_TMC2130_TCOOLTHRS        1000                    // 4cth
#endif
#ifndef M4_TMC2130_THIGH
#define M4_TMC2130_THIGH            10                      // 4hth
#endif
#ifndef M4_TMC2130_SGT
#define M4_TMC2130_SGT              4                       // 4sgt
#endif
#ifndef M4_TMC2130_TBL
#define M4_TMC2130_TBL              2                       // 4tbl
#endif
#ifndef M4_TMC2130_PWM_GRAD
#define M4_TMC2130_PWM_GRAD         1                       // 4pgrd
#endif
#ifndef M4_TMC2130_PWM_AMPL
#define M4_TMC2130_PWM_AMPL         200                     // 4pamp
#endif
#ifndef M4_TMC2130_HEND
#define M4_TMC2130_HEND             0                       // 4hend
#endif
#ifndef M4_TMC2130_HSTRT
#define M4_TMC2130_HSTRT            0                       // 4hsrt
#endif
#ifndef M4_TMC2130_SMIN
#define M4_TMC2130_SMIN             5                       // 4smin
#endif
#ifndef M4_TMC2130_SMAX
#define M4_TMC2130_SMAX             5                       // 4smax
#endif
#ifndef M4_TMC2130_SUP
#define M4_TMC2130_SUP              2                       // 4sup
#endif
#ifndef M4_TMC2130_SDN
#define M4_TMC2130_SDN              1                       // 4sdn
#endif

#ifndef M5_TMC2130_TPWMTHRS
#define M5_TMC2130_TPWMTHRS         1200                    // 5pth
#endif
#ifndef M5_TMC2130_TCOOLTHRS
#define M5_TMC2130_TCOOLTHRS        1000                    // 5cth
#endif
#ifndef M5_TMC2130_THIGH
#define M5_TMC2130_THIGH            10                      // 5hth
#endif
#ifndef M5_TMC2130_SGT
#define M5_TMC2130_SGT              4                       // 5sgt
#endif
#ifndef M5_TMC2130_TBL
#define M5_TMC2130_TBL              2                       // 5tbl
#endif
#ifndef M5_TMC2130_PWM_GRAD
#define M5_TMC2130_PWM_GRAD         1                       // 5pgrd
#endif
#ifndef M5_TMC2130_PWM_AMPL
#define M5_TMC2130_PWM_AMPL         200                     // 5pamp
#endif
#ifndef M5_TMC2130_HEND
#define M5_TMC2130_HEND             0                       // 5hend
#endif
#ifndef M5_TMC2130_HSTRT
#define M5_TMC2130_HSTRT            0                       // 5hsrt
#endif
#ifndef M5_TMC2130_SMIN
#define M5_TMC2130_SMIN             5                       // 5smin
#endif
#ifndef M5_TMC2130_SMAX
#define M5_TMC2130_SMAX             5                       // 5smax
#endif
#ifndef M5_TMC2130_SUP
#define M5_TMC2130_SUP              2                       // 5sup
#endif
#ifndef M5_TMC2130_SDN
#define M5_TMC2130_SDN              1                       // 5sdn
#endif

#ifndef M6_TMC2130_TPWMTHRS
#define M6_TMC2130_TPWMTHRS         1200                    // 6pth
#endif
#ifndef M6_TMC2130_TCOOLTHRS
#define M6_TMC2130_TCOOLTHRS        1000                    // 6cth
#endif
#ifndef M6_TMC2130_THIGH
#define M6_TMC2130_THIGH            10                      // 6hth
#endif
#ifndef M6_TMC2130_SGT
#define M6_TMC2130_SGT              4                       // 6sgt
#endif
#ifndef M6_TMC2130_TBL
#define M6_TMC2130_TBL              2                       // 6tbl
#endif
#ifndef M6_TMC2130_PWM_GRAD
#define M6_TMC2130_PWM_GRAD         1                       // 6pgrd
#endif
#ifndef M6_TMC2130_PWM_AMPL
#define M6_TMC2130_PWM_AMPL         200                     // 6pamp
#endif
#ifndef M6_TMC2130_HEND
#define M6_TMC2130_HEND             0                       // 6hend
#endif
#ifndef M6_TMC2130_HSTRT
#define M6_TMC2130_HSTRT            0                       // 6hsrt
#endif
#ifndef M6_TMC2130_SMIN
#define M6_TMC2130_SMIN             5                       // 6smin
#endif
#ifndef M6_TMC2130_SMAX
#define M6_TMC2130_SMAX             5                       // 6smax
#endif
#ifndef M6_TMC2130_SUP
#define M6_TMC2130_SUP              2                       // 6sup
#endif
#ifndef M6_TMC2130_SDN
#define M6_TMC2130_SDN              1                       // 6sdn
#endif
// END Generated

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

// U AXIS
#ifndef U_AXIS_MODE
#define U_AXIS_MODE                 AXIS_DISABLED           // {xam:  see canonical_machine.h cmAxisMode for valid values
#endif
#ifndef U_VELOCITY_MAX
#define U_VELOCITY_MAX              1000.0                  // {xvm:  G0 max velocity in mm/min
#endif
#ifndef U_FEEDRATE_MAX
#define U_FEEDRATE_MAX              1000.0                  // {xfr:  G1 max feed rate in mm/min
#endif
#ifndef U_TRAVEL_MIN
#define U_TRAVEL_MIN                0.0                     // {xtn:  minimum travel for soft limits
#endif
#ifndef U_TRAVEL_MAX
#define U_TRAVEL_MAX                0.0                     // {xtm:  travel between switches or crashes
#endif
#ifndef U_JERK_MAX
#define U_JERK_MAX                  1000.0                  // {xjm:
#endif
#ifndef U_JERK_HIGH_SPEED
#define U_JERK_HIGH_SPEED           1000.0                  // {xjh:
#endif
#ifndef U_HOMING_INPUT
#define U_HOMING_INPUT              0                       // {xhi:  input used for homing or 0 to disable
#endif
#ifndef U_HOMING_DIRECTION
#define U_HOMING_DIRECTION          0                       // {xhd:  0=search moves negative, 1= search moves positive
#endif
#ifndef U_SEARCH_VELOCITY
#define U_SEARCH_VELOCITY           500.0                   // {xsv:  minus means move to minimum switch
#endif
#ifndef U_LATCH_VELOCITY
#define U_LATCH_VELOCITY            100.0                   // {xlv:  mm/min
#endif
#ifndef U_LATCH_BACKOFF
#define U_LATCH_BACKOFF             4.0                     // {xlb:  mm
#endif
#ifndef U_ZERO_BACKOFF
#define U_ZERO_BACKOFF              2.0                     // {xzb:  mm
#endif

// V AXIS
#ifndef V_AXIS_MODE
#define V_AXIS_MODE                 AXIS_DISABLED
#endif
#ifndef V_VELOCITY_MAX
#define V_VELOCITY_MAX              1000.0
#endif
#ifndef V_FEEDRATE_MAX
#define V_FEEDRATE_MAX              1000.0
#endif
#ifndef V_TRAVEL_MIN
#define V_TRAVEL_MIN                0.0
#endif
#ifndef V_TRAVEL_MAX
#define V_TRAVEL_MAX                0.0
#endif
#ifndef V_JERK_MAX
#define V_JERK_MAX                  1000.0
#endif
#ifndef V_JERK_HIGH_SPEED
#define V_JERK_HIGH_SPEED           1000.0
#endif
#ifndef V_HOMING_INPUT
#define V_HOMING_INPUT              0
#endif
#ifndef V_HOMING_DIRECTION
#define V_HOMING_DIRECTION          0
#endif
#ifndef V_SEARCH_VELOCITY
#define V_SEARCH_VELOCITY           500.0
#endif
#ifndef V_LATCH_VELOCITY
#define V_LATCH_VELOCITY            100.0
#endif
#ifndef V_LATCH_BACKOFF
#define V_LATCH_BACKOFF             4.0
#endif
#ifndef V_ZERO_BACKOFF
#define V_ZERO_BACKOFF              2.0
#endif

// W AXIS
#ifndef W_AXIS_MODE
#define W_AXIS_MODE                 AXIS_DISABLED
#endif
#ifndef W_VELOCITY_MAX
#define W_VELOCITY_MAX              1000.0
#endif
#ifndef W_FEEDRATE_MAX
#define W_FEEDRATE_MAX              1000.0
#endif
#ifndef W_TRAVEL_MAX
#define W_TRAVEL_MAX                0.0
#endif
#ifndef W_TRAVEL_MIN
#define W_TRAVEL_MIN                0.0
#endif
#ifndef W_JERK_MAX
#define W_JERK_MAX                  500.0
#endif
#ifndef W_JERK_HIGH_SPEED
#define W_JERK_HIGH_SPEED           500.0
#endif
#ifndef W_HOMING_INPUT
#define W_HOMING_INPUT              0
#endif
#ifndef W_HOMING_DIRECTION
#define W_HOMING_DIRECTION          0
#endif
#ifndef W_SEARCH_VELOCITY
#define W_SEARCH_VELOCITY           250.0
#endif
#ifndef W_LATCH_VELOCITY
#define W_LATCH_VELOCITY            25.0
#endif
#ifndef W_LATCH_BACKOFF
#define W_LATCH_BACKOFF             4.0
#endif
#ifndef W_ZERO_BACKOFF
#define W_ZERO_BACKOFF              2.0
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
#define A_RADIUS                    (M4_TRAVEL_PER_REV/(2*3.14159628))
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
#define A_RADIUS                    (M4_TRAVEL_PER_REV/(2*3.14159628))
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
#define B_RADIUS                    (M5_TRAVEL_PER_REV/(2*3.14159628))
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
#define B_SEARCH_VELOCITY           (B_VELOCITY_MAX * 0.500)
#endif
#ifndef B_LATCH_VELOCITY
#define B_LATCH_VELOCITY            (B_VELOCITY_MAX * 0.100)
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
#define C_RADIUS                    (M6_TRAVEL_PER_REV/(2*3.14159628))
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
#define C_SEARCH_VELOCITY           (C_VELOCITY_MAX * 0.500)
#endif
#ifndef C_LATCH_VELOCITY
#define C_LATCH_VELOCITY            (C_VELOCITY_MAX * 0.100)
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

// Set the probing input (universal, all axis probe)
#ifndef PROBING_INPUT
#define PROBING_INPUT 0
#endif

/* Legend of valid options:
  DIn_ENABLED
    IO_UNAVAILABLE   // input/output is missing/used/unavailable
    IO_DISABLED      // input/output is disabled
    IO_ENABLED       // input/output enabled

  DIn_POLARITY:
    IO_ACTIVE_LOW    aka NORMALLY_OPEN
    IO_ACTIVE_HIGH   aka NORMALLY_CLOSED

  DIn_ACTION:
    INPUT_ACTION_NONE
    INPUT_ACTION_STOP        - stop at normal jerk - preserves positional accuracy
    INPUT_ACTION_FAST_STOP   - stop at high jerk - preserves positional accuracy
    INPUT_ACTION_HALT        - stop immediately - not guaranteed to preserve position
    INPUT_ACTION_CYCLE_START - start / restart cycle after feedhold (RESERVED)
    INPUT_ACTION_ALARM       - initiate an alarm. stops everything immediately - preserves position
    INPUT_ACTION_SHUTDOWN    - initiate a shutdown. stops everything immediately - does not preserve position
    INPUT_ACTION_PANIC       - initiate a panic. stops everything immediately - does not preserve position
    INPUT_ACTION_RESET       - reset system

    INPUT_ACTION_LIMIT       - limit switch processing
    INPUT_ACTION_INTERLOCK   - interlock processing
*/

// Xmin on v9 board
#ifdef DI1_MODE
#error DI1_MODE is no longer used, please update your settings file
#error Use DI1_ENABLED and DI1_POLARITY instead
#endif
#ifndef DI1_ENABLED
#define DI1_ENABLED                 IO_ENABLED
#endif
#ifndef DI1_POLARITY
#define DI1_POLARITY                IO_ACTIVE_LOW     // Normally open
#endif
#ifndef DI1_ACTION
#define DI1_ACTION                  INPUT_ACTION_NONE
#endif
#ifdef DI1_FUNCTION
#error DI1_FUNCTION is no longer used, please update your settings file
#error Use DI1_ACTION instead
#endif
#ifndef DI1_EXTERNAL_NUMBER
#define DI1_EXTERNAL_NUMBER         1
#endif

// Xmax
#ifdef DI2_MODE
#error DI2_MODE is no longer used, please update your settings file
#error Use DI2_ENABLED and DI2_POLARITY instead
#endif
#ifndef DI2_ENABLED
#define DI2_ENABLED                 IO_ENABLED
#endif
#ifndef DI2_POLARITY
#define DI2_POLARITY                IO_ACTIVE_LOW     // Normally open
#endif
#ifndef DI2_ACTION
#define DI2_ACTION                  INPUT_ACTION_NONE
#endif
#ifdef DI2_FUNCTION
#error DI2_FUNCTION is no longer used, please update your settings file
#error Use DI2_ACTION instead
#endif
#ifndef DI2_EXTERNAL_NUMBER
#define DI2_EXTERNAL_NUMBER         2
#endif

// Ymin
#ifdef DI3_MODE
#error DI3_MODE is no longer used, please update your settings file
#error Use DI3_ENABLED and DI3_POLARITY instead
#endif
#ifndef DI3_ENABLED
#define DI3_ENABLED                 IO_ENABLED
#endif
#ifndef DI3_POLARITY
#define DI3_POLARITY                IO_ACTIVE_LOW     // Normally open
#endif
#ifndef DI3_ACTION
#define DI3_ACTION                  INPUT_ACTION_NONE
#endif
#ifdef DI3_FUNCTION
#error DI3_FUNCTION is no longer used, please update your settings file
#error Use DI3_ACTION instead
#endif
#ifndef DI3_EXTERNAL_NUMBER
#define DI3_EXTERNAL_NUMBER         3
#endif

// Ymax
#ifdef DI4_MODE
#error DI4_MODE is no longer used, please update your settings file
#error Use DI4_ENABLED and DI4_POLARITY instead
#endif
#ifndef DI4_ENABLED
#define DI4_ENABLED                 IO_ENABLED
#endif
#ifndef DI4_POLARITY
#define DI4_POLARITY                IO_ACTIVE_LOW     // Normally open
#endif
#ifndef DI4_ACTION
#define DI4_ACTION                  INPUT_ACTION_NONE
#endif
#ifdef DI4_FUNCTION
#error DI4_FUNCTION is no longer used, please update your settings file
#error Use DI4_ACTION instead
#endif
#ifndef DI4_EXTERNAL_NUMBER
#define DI4_EXTERNAL_NUMBER         4
#endif

// Zmin
#ifdef DI5_MODE
#error DI5_MODE is no longer used, please update your settings file
#error Use DI5_ENABLED and DI5_POLARITY instead
#endif
#ifndef DI5_ENABLED
#define DI5_ENABLED                 IO_ENABLED
#endif
#ifndef DI5_POLARITY
#define DI5_POLARITY                IO_ACTIVE_LOW     // Normally open
#endif
#ifndef DI5_ACTION
#define DI5_ACTION                  INPUT_ACTION_NONE
#endif
#ifdef DI5_FUNCTION
#error DI5_FUNCTION is no longer used, please update your settings file
#error Use DI5_ACTION instead
#endif
#ifndef DI5_EXTERNAL_NUMBER
#define DI5_EXTERNAL_NUMBER         5
#endif

// Zmax
#ifdef DI6_MODE
#error DI6_MODE is no longer used, please update your settings file
#error Use DI6_ENABLED and DI6_POLARITY instead
#endif
#ifndef DI6_ENABLED
#define DI6_ENABLED                 IO_ENABLED
#endif
#ifndef DI6_POLARITY
#define DI6_POLARITY                IO_ACTIVE_LOW     // Normally open
#endif
#ifndef DI6_ACTION
#define DI6_ACTION                  INPUT_ACTION_NONE
#endif
#ifdef DI6_FUNCTION
#error DI6_FUNCTION is no longer used, please update your settings file
#error Use DI6_ACTION instead
#endif
#ifndef DI6_EXTERNAL_NUMBER
#define DI6_EXTERNAL_NUMBER         6
#endif

// Amin
#ifdef DI7_MODE
#error DI7_MODE is no longer used, please update your settings file
#error Use DI7_ENABLED and DI7_POLARITY instead
#endif
#ifndef DI7_ENABLED
#define DI7_ENABLED                 IO_ENABLED
#endif
#ifndef DI7_POLARITY
#define DI7_POLARITY                IO_ACTIVE_LOW     // Normally open
#endif
#ifndef DI7_ACTION
#define DI7_ACTION                  INPUT_ACTION_NONE
#endif
#ifdef DI7_FUNCTION
#error DI7_FUNCTION is no longer used, please update your settings file
#error Use DI7_ACTION instead
#endif
#ifndef DI7_EXTERNAL_NUMBER
#define DI7_EXTERNAL_NUMBER         7
#endif

// Amax
#ifdef DI8_MODE
#error DI8_MODE is no longer used, please update your settings file
#error Use DI8_ENABLED and DI8_POLARITY instead
#endif
#ifndef DI8_ENABLED
#define DI8_ENABLED                 IO_ENABLED
#endif
#ifndef DI8_POLARITY
#define DI8_POLARITY                IO_ACTIVE_LOW     // Normally open
#endif
#ifndef DI8_ACTION
#define DI8_ACTION                  INPUT_ACTION_NONE
#endif
#ifdef DI8_FUNCTION
#error DI8_FUNCTION is no longer used, please update your settings file
#error Use DI8_ACTION instead
#endif
#ifndef DI8_EXTERNAL_NUMBER
#define DI8_EXTERNAL_NUMBER         8
#endif

// Safety line
#ifdef DI9_MODE
#error DI9_MODE is no longer used, please update your settings file
#error Use DI9_ENABLED and DI9_POLARITY instead
#endif
#ifndef DI9_ENABLED
#define DI9_ENABLED                 IO_ENABLED
#endif
#ifndef DI9_POLARITY
#define DI9_POLARITY                IO_ACTIVE_HIGH     // Normally closed
#endif
#ifndef DI9_ACTION
#define DI9_ACTION                  INPUT_ACTION_NONE
#endif
#ifdef DI9_FUNCTION
#error DI9_FUNCTION is no longer used, please update your settings file
#error Use DI9_ACTION instead
#endif
#ifndef DI9_EXTERNAL_NUMBER
#define DI9_EXTERNAL_NUMBER         9
#endif

#ifdef DI10_MODE
#error DI10_MODE is no longer used, please update your settings file
#error Use DI10_ENABLED and DI10_POLARITY instead
#endif
#ifndef DI10_ENABLED
#define DI10_ENABLED                IO_ENABLED
#endif
#ifndef DI10_POLARITY
#define DI10_POLARITY               IO_ACTIVE_LOW     // Normally open
#endif
#ifndef DI10_ACTION
#define DI10_ACTION                 INPUT_ACTION_NONE
#endif
#ifdef DI10_FUNCTION
#error DI10_FUNCTION is no longer used, please update your settings file
#error Use DI10_ACTION instead
#endif
#ifndef DI10_EXTERNAL_NUMBER
#define DI10_EXTERNAL_NUMBER         10
#endif

#ifdef DI11_MODE
#error DI11_MODE is no longer used, please update your settings file
#error Use DI11_ENABLED and DI11_POLARITY instead
#endif
#ifndef DI11_ENABLED
#define DI11_ENABLED                IO_ENABLED
#endif
#ifndef DI11_POLARITY
#define DI11_POLARITY               IO_ACTIVE_LOW     // Normally open
#endif
#ifndef DI11_ACTION
#define DI11_ACTION                 INPUT_ACTION_NONE
#endif
#ifdef DI11_FUNCTION
#error DI11_FUNCTION is no longer used, please update your settings file
#error Use DI11_ACTION instead
#endif
#ifndef DI11_EXTERNAL_NUMBER
#define DI11_EXTERNAL_NUMBER         11
#endif

#ifdef DI12_MODE
#error DI12_MODE is no longer used, please update your settings file
#error Use DI12_ENABLED and DI12_POLARITY instead
#endif
#ifndef DI12_ENABLED
#define DI12_ENABLED                IO_ENABLED
#endif
#ifndef DI12_POLARITY
#define DI12_POLARITY               IO_ACTIVE_LOW     // Normally open
#endif
#ifndef DI12_ACTION
#define DI12_ACTION                 INPUT_ACTION_NONE
#endif
#ifdef DI12_FUNCTION
#error DI12_FUNCTION is no longer used, please update your settings file
#error Use DI12_ACTION instead
#endif
#ifndef DI12_EXTERNAL_NUMBER
#define DI12_EXTERNAL_NUMBER         12
#endif


#ifndef PROBING_INPUT
#define PROBING_INPUT                Z_HOMING_INPUT // default to the z homing input
#endif

// DIGITAL OUTPUTS - Currently these are hard-wired to extruders

//Extruder1_PWM
#ifndef DO1_ENABLED
#define DO1_ENABLED                 IO_ENABLED
#endif
#ifndef DO1_POLARITY
#define DO1_POLARITY                IO_ACTIVE_HIGH
#endif
#ifndef DO1_EXTERNAL_NUMBER
#define DO1_EXTERNAL_NUMBER         1
#endif

//Extruder2_PWM
#ifndef DO2_ENABLED
#define DO2_ENABLED                 IO_ENABLED
#endif
#ifndef DO2_POLARITY
#define DO2_POLARITY                IO_ACTIVE_HIGH
#endif
#ifndef DO2_EXTERNAL_NUMBER
#define DO2_EXTERNAL_NUMBER         2
#endif

//Fan1A_PWM
#ifndef DO3_ENABLED
#define DO3_ENABLED                 IO_ENABLED
#endif
#ifndef DO3_POLARITY
#define DO3_POLARITY                IO_ACTIVE_HIGH
#endif
#ifndef DO3_EXTERNAL_NUMBER
#define DO3_EXTERNAL_NUMBER         3
#endif

//Fan1B_PWM
#ifndef DO4_ENABLED
#define DO4_ENABLED                 IO_ENABLED
#endif
#ifndef DO4_POLARITY
#define DO4_POLARITY                IO_ACTIVE_HIGH
#endif
#ifndef DO4_EXTERNAL_NUMBER
#define DO4_EXTERNAL_NUMBER         4
#endif

#ifndef DO5_ENABLED
#define DO5_ENABLED                 IO_ENABLED
#endif
#ifndef DO5_POLARITY
#define DO5_POLARITY                IO_ACTIVE_HIGH
#endif
#ifndef DO5_EXTERNAL_NUMBER
#define DO5_EXTERNAL_NUMBER         5
#endif

#ifndef DO6_ENABLED
#define DO6_ENABLED                 IO_ENABLED
#endif
#ifndef DO6_POLARITY
#define DO6_POLARITY                IO_ACTIVE_HIGH
#endif
#ifndef DO6_EXTERNAL_NUMBER
#define DO6_EXTERNAL_NUMBER         6
#endif

#ifndef DO7_ENABLED
#define DO7_ENABLED                 IO_ENABLED
#endif
#ifndef DO7_POLARITY
#define DO7_POLARITY                IO_ACTIVE_HIGH
#endif
#ifndef DO7_EXTERNAL_NUMBER
#define DO7_EXTERNAL_NUMBER         7
#endif

#ifndef DO8_ENABLED
#define DO8_ENABLED                 IO_ENABLED
#endif
#ifndef DO8_POLARITY
#define DO8_POLARITY                IO_ACTIVE_HIGH
#endif
#ifndef DO8_EXTERNAL_NUMBER
#define DO8_EXTERNAL_NUMBER         8
#endif

//SAFEin (Output) signal
#ifndef DO9_ENABLED
#define DO9_ENABLED                 IO_ENABLED
#endif
#ifndef DO9_POLARITY
#define DO9_POLARITY                IO_ACTIVE_HIGH
#endif
#ifndef DO9_EXTERNAL_NUMBER
#define DO9_EXTERNAL_NUMBER         9
#endif

#ifndef DO10_ENABLED
#define DO10_ENABLED                IO_ENABLED
#endif
#ifndef DO10_POLARITY
#define DO10_POLARITY               IO_ACTIVE_HIGH
#endif
#ifndef DO10_EXTERNAL_NUMBER
#define DO10_EXTERNAL_NUMBER         10
#endif

//Header Bed FET
#ifndef DO11_ENABLED
#define DO11_ENABLED                IO_ENABLED
#endif
#ifndef DO11_POLARITY
#define DO11_POLARITY               IO_ACTIVE_HIGH
#endif
#ifndef DO11_EXTERNAL_NUMBER
#define DO11_EXTERNAL_NUMBER         11
#endif

//Indicator_LED
#ifndef DO12_ENABLED
#define DO12_ENABLED                IO_ENABLED
#endif
#ifndef DO12_POLARITY
#define DO12_POLARITY               IO_ACTIVE_HIGH
#endif
#ifndef DO12_EXTERNAL_NUMBER
#define DO12_EXTERNAL_NUMBER         12
#endif

#ifndef DO13_ENABLED
#define DO13_ENABLED                IO_ENABLED
#endif
#ifndef DO13_POLARITY
#define DO13_POLARITY               IO_ACTIVE_HIGH
#endif
#ifndef DO13_EXTERNAL_NUMBER
#define DO13_EXTERNAL_NUMBER         13
#endif

// foind in gpio.h:
// #ifndef AI1_TYPE
// #define AI1_TYPE                    AIN_TYPE_DISABLED
// #endif
// #ifndef AI1_CIRCUIT
// #define AI1_CIRCUIT                 AIN_CIRCUIT_DISABLED
// #endif
// #ifndef AI1_P1
// #define AI1_P1                      0.0
// #endif
// #ifndef AI1_P2
// #define AI1_P2                      0.0
// #endif
// #ifndef AI1_P3
// #define AI1_P3                      0.0
// #endif
// #ifndef AI1_P4
// #define AI1_P4                      0.0
// #endif
// #ifndef AI1_P5
// #define AI1_P5                      0.0
// #endif
//
//
// #ifndef AI2_TYPE
// #define AI2_TYPE                    AIN_TYPE_DISABLED
// #endif
// #ifndef AI2_CIRCUIT
// #define AI2_CIRCUIT                 AIN_CIRCUIT_DISABLED
// #endif
// #ifndef AI2_P1
// #define AI2_P1                      0.0
// #endif
// #ifndef AI2_P2
// #define AI2_P2                      0.0
// #endif
// #ifndef AI2_P3
// #define AI2_P3                      0.0
// #endif
// #ifndef AI2_P4
// #define AI2_P4                      0.0
// #endif
// #ifndef AI2_P5
// #define AI2_P5                      0.0
// #endif
//
// #ifndef AI3_TYPE
// #define AI3_TYPE                    AIN_TYPE_DISABLED
// #endif
// #ifndef AI3_CIRCUIT
// #define AI3_CIRCUIT                 AIN_CIRCUIT_DISABLED
// #endif
// #ifndef AI3_P1
// #define AI3_P1                      0.0
// #endif
// #ifndef AI3_P2
// #define AI3_P2                      0.0
// #endif
// #ifndef AI3_P3
// #define AI3_P3                      0.0
// #endif
// #ifndef AI3_P4
// #define AI3_P4                      0.0
// #endif
// #ifndef AI3_P5
// #define AI3_P5                      0.0
// #endif
//
// #ifndef AI4_TYPE
// #define AI4_TYPE                    AIN_TYPE_DISABLED
// #endif
// #ifndef AI4_CIRCUIT
// #define AI4_CIRCUIT                 AIN_CIRCUIT_DISABLED
// #endif
// #ifndef AI4_P1
// #define AI4_P1                      0.0
// #endif
// #ifndef AI4_P2
// #define AI4_P2                      0.0
// #endif
// #ifndef AI4_P3
// #define AI4_P3                      0.0
// #endif
// #ifndef AI4_P4
// #define AI4_P4                      0.0
// #endif
// #ifndef AI4_P5
// #define AI4_P5                      0.0
// #endif

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
#define P1_CCW_SPEED_LO             7900    // 0.0
#endif
#ifndef P1_CCW_SPEED_HI
#define P1_CCW_SPEED_HI            12800    // 0.0
#endif
#ifndef P1_CCW_PHASE_LO
#define P1_CCW_PHASE_LO             0.13    // 0.1
#endif
#ifndef P1_CCW_PHASE_HI
#define P1_CCW_PHASE_HI             0.17    // 0.1
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
#ifndef H1_DEFAULT_F
#define H1_DEFAULT_F                0.0
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
#ifndef H2_DEFAULT_F
#define H2_DEFAULT_F                0.0
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
#ifndef H3_DEFAULT_F
#define H3_DEFAULT_F                0.0
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
#ifndef G54_U_OFFSET
#define G54_U_OFFSET 0
#endif
#ifndef G54_V_OFFSET
#define G54_V_OFFSET 0
#endif
#ifndef G54_W_OFFSET
#define G54_W_OFFSET 0
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
#ifndef G55_U_OFFSET
#define G55_U_OFFSET 0
#endif
#ifndef G55_V_OFFSET
#define G55_V_OFFSET 0
#endif
#ifndef G55_W_OFFSET
#define G55_W_OFFSET 0
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
#ifndef G56_U_OFFSET
#define G56_U_OFFSET 0
#endif
#ifndef G56_V_OFFSET
#define G56_V_OFFSET 0
#endif
#ifndef G56_W_OFFSET
#define G56_W_OFFSET 0
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
#ifndef G57_U_OFFSET
#define G57_U_OFFSET 0
#endif
#ifndef G57_V_OFFSET
#define G57_V_OFFSET 0
#endif
#ifndef G57_W_OFFSET
#define G57_W_OFFSET 0
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
#ifndef G58_U_OFFSET
#define G58_U_OFFSET 0
#endif
#ifndef G58_V_OFFSET
#define G58_V_OFFSET 0
#endif
#ifndef G58_W_OFFSET
#define G58_W_OFFSET 0
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
#ifndef G59_U_OFFSET
#define G59_U_OFFSET 0
#endif
#ifndef G59_V_OFFSET
#define G59_V_OFFSET 0
#endif
#ifndef G59_W_OFFSET
#define G59_W_OFFSET 0
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

// *** Tool Table Defaults *** //

#ifndef TT1_X_OFFSET
#define TT1_X_OFFSET 0
#endif
#ifndef TT1_Y_OFFSET
#define TT1_Y_OFFSET 0
#endif
#ifndef TT1_Z_OFFSET
#define TT1_Z_OFFSET 0
#endif
#ifndef TT1_U_OFFSET
#define TT1_U_OFFSET 0
#endif
#ifndef TT1_V_OFFSET
#define TT1_V_OFFSET 0
#endif
#ifndef TT1_W_OFFSET
#define TT1_W_OFFSET 0
#endif
#ifndef TT1_A_OFFSET
#define TT1_A_OFFSET 0
#endif
#ifndef TT1_B_OFFSET
#define TT1_B_OFFSET 0
#endif
#ifndef TT1_C_OFFSET
#define TT1_C_OFFSET 0
#endif

#ifndef TT2_X_OFFSET
#define TT2_X_OFFSET 0
#endif
#ifndef TT2_Y_OFFSET
#define TT2_Y_OFFSET 0
#endif
#ifndef TT2_Z_OFFSET
#define TT2_Z_OFFSET 0
#endif
#ifndef TT2_U_OFFSET
#define TT2_U_OFFSET 0
#endif
#ifndef TT2_V_OFFSET
#define TT2_V_OFFSET 0
#endif
#ifndef TT2_W_OFFSET
#define TT2_W_OFFSET 0
#endif
#ifndef TT2_A_OFFSET
#define TT2_A_OFFSET 0
#endif
#ifndef TT2_B_OFFSET
#define TT2_B_OFFSET 0
#endif
#ifndef TT2_C_OFFSET
#define TT2_C_OFFSET 0
#endif

#ifndef TT3_X_OFFSET
#define TT3_X_OFFSET 0
#endif
#ifndef TT3_Y_OFFSET
#define TT3_Y_OFFSET 0
#endif
#ifndef TT3_Z_OFFSET
#define TT3_Z_OFFSET 0
#endif
#ifndef TT3_U_OFFSET
#define TT3_U_OFFSET 0
#endif
#ifndef TT3_V_OFFSET
#define TT3_V_OFFSET 0
#endif
#ifndef TT3_W_OFFSET
#define TT3_W_OFFSET 0
#endif
#ifndef TT3_A_OFFSET
#define TT3_A_OFFSET 0
#endif
#ifndef TT3_B_OFFSET
#define TT3_B_OFFSET 0
#endif
#ifndef TT3_C_OFFSET
#define TT3_C_OFFSET 0
#endif

#ifndef TT4_X_OFFSET
#define TT4_X_OFFSET 0
#endif
#ifndef TT4_Y_OFFSET
#define TT4_Y_OFFSET 0
#endif
#ifndef TT4_Z_OFFSET
#define TT4_Z_OFFSET 0
#endif
#ifndef TT4_U_OFFSET
#define TT4_U_OFFSET 0
#endif
#ifndef TT4_V_OFFSET
#define TT4_V_OFFSET 0
#endif
#ifndef TT4_W_OFFSET
#define TT4_W_OFFSET 0
#endif
#ifndef TT4_A_OFFSET
#define TT4_A_OFFSET 0
#endif
#ifndef TT4_B_OFFSET
#define TT4_B_OFFSET 0
#endif
#ifndef TT4_C_OFFSET
#define TT4_C_OFFSET 0
#endif

#ifndef TT5_X_OFFSET
#define TT5_X_OFFSET 0
#endif
#ifndef TT5_Y_OFFSET
#define TT5_Y_OFFSET 0
#endif
#ifndef TT5_Z_OFFSET
#define TT5_Z_OFFSET 0
#endif
#ifndef TT5_U_OFFSET
#define TT5_U_OFFSET 0
#endif
#ifndef TT5_V_OFFSET
#define TT5_V_OFFSET 0
#endif
#ifndef TT5_W_OFFSET
#define TT5_W_OFFSET 0
#endif
#ifndef TT5_A_OFFSET
#define TT5_A_OFFSET 0
#endif
#ifndef TT5_B_OFFSET
#define TT5_B_OFFSET 0
#endif
#ifndef TT5_C_OFFSET
#define TT5_C_OFFSET 0
#endif

#ifndef TT6_X_OFFSET
#define TT6_X_OFFSET 0
#endif
#ifndef TT6_Y_OFFSET
#define TT6_Y_OFFSET 0
#endif
#ifndef TT6_Z_OFFSET
#define TT6_Z_OFFSET 0
#endif
#ifndef TT6_U_OFFSET
#define TT6_U_OFFSET 0
#endif
#ifndef TT6_V_OFFSET
#define TT6_V_OFFSET 0
#endif
#ifndef TT6_W_OFFSET
#define TT6_W_OFFSET 0
#endif
#ifndef TT6_A_OFFSET
#define TT6_A_OFFSET 0
#endif
#ifndef TT6_B_OFFSET
#define TT6_B_OFFSET 0
#endif
#ifndef TT6_C_OFFSET
#define TT6_C_OFFSET 0
#endif

#ifndef TT7_X_OFFSET
#define TT7_X_OFFSET 0
#endif
#ifndef TT7_Y_OFFSET
#define TT7_Y_OFFSET 0
#endif
#ifndef TT7_Z_OFFSET
#define TT7_Z_OFFSET 0
#endif
#ifndef TT7_U_OFFSET
#define TT7_U_OFFSET 0
#endif
#ifndef TT7_V_OFFSET
#define TT7_V_OFFSET 0
#endif
#ifndef TT7_W_OFFSET
#define TT7_W_OFFSET 0
#endif
#ifndef TT7_A_OFFSET
#define TT7_A_OFFSET 0
#endif
#ifndef TT7_B_OFFSET
#define TT7_B_OFFSET 0
#endif
#ifndef TT7_C_OFFSET
#define TT7_C_OFFSET 0
#endif

#ifndef TT8_X_OFFSET
#define TT8_X_OFFSET 0
#endif
#ifndef TT8_Y_OFFSET
#define TT8_Y_OFFSET 0
#endif
#ifndef TT8_U_OFFSET
#define TT8_U_OFFSET 0
#endif
#ifndef TT8_V_OFFSET
#define TT8_V_OFFSET 0
#endif
#ifndef TT8_W_OFFSET
#define TT8_W_OFFSET 0
#endif
#ifndef TT8_Z_OFFSET
#define TT8_Z_OFFSET 0
#endif
#ifndef TT8_A_OFFSET
#define TT8_A_OFFSET 0
#endif
#ifndef TT8_B_OFFSET
#define TT8_B_OFFSET 0
#endif
#ifndef TT8_C_OFFSET
#define TT8_C_OFFSET 0
#endif

#ifndef TT9_X_OFFSET
#define TT9_X_OFFSET 0
#endif
#ifndef TT9_Y_OFFSET
#define TT9_Y_OFFSET 0
#endif
#ifndef TT9_Z_OFFSET
#define TT9_Z_OFFSET 0
#endif
#ifndef TT9_U_OFFSET
#define TT9_U_OFFSET 0
#endif
#ifndef TT9_V_OFFSET
#define TT9_V_OFFSET 0
#endif
#ifndef TT9_W_OFFSET
#define TT9_W_OFFSET 0
#endif
#ifndef TT9_A_OFFSET
#define TT9_A_OFFSET 0
#endif
#ifndef TT9_B_OFFSET
#define TT9_B_OFFSET 0
#endif
#ifndef TT9_C_OFFSET
#define TT9_C_OFFSET 0
#endif

#ifndef TT10_X_OFFSET
#define TT10_X_OFFSET 0
#endif
#ifndef TT10_Y_OFFSET
#define TT10_Y_OFFSET 0
#endif
#ifndef TT10_Z_OFFSET
#define TT10_Z_OFFSET 0
#endif
#ifndef TT10_U_OFFSET
#define TT10_U_OFFSET 0
#endif
#ifndef TT10_V_OFFSET
#define TT10_V_OFFSET 0
#endif
#ifndef TT10_W_OFFSET
#define TT10_W_OFFSET 0
#endif
#ifndef TT10_A_OFFSET
#define TT10_A_OFFSET 0
#endif
#ifndef TT10_B_OFFSET
#define TT10_B_OFFSET 0
#endif
#ifndef TT10_C_OFFSET
#define TT10_C_OFFSET 0
#endif

#ifndef TT11_X_OFFSET
#define TT11_X_OFFSET 0
#endif
#ifndef TT11_Y_OFFSET
#define TT11_Y_OFFSET 0
#endif
#ifndef TT11_Z_OFFSET
#define TT11_Z_OFFSET 0
#endif
#ifndef TT11_U_OFFSET
#define TT11_U_OFFSET 0
#endif
#ifndef TT11_V_OFFSET
#define TT11_V_OFFSET 0
#endif
#ifndef TT11_W_OFFSET
#define TT11_W_OFFSET 0
#endif
#ifndef TT11_A_OFFSET
#define TT11_A_OFFSET 0
#endif
#ifndef TT11_B_OFFSET
#define TT11_B_OFFSET 0
#endif
#ifndef TT11_C_OFFSET
#define TT11_C_OFFSET 0
#endif

#ifndef TT12_X_OFFSET
#define TT12_X_OFFSET 0
#endif
#ifndef TT12_Y_OFFSET
#define TT12_Y_OFFSET 0
#endif
#ifndef TT12_Z_OFFSET
#define TT12_Z_OFFSET 0
#endif
#ifndef TT12_U_OFFSET
#define TT12_U_OFFSET 0
#endif
#ifndef TT12_V_OFFSET
#define TT12_V_OFFSET 0
#endif
#ifndef TT12_W_OFFSET
#define TT12_W_OFFSET 0
#endif
#ifndef TT12_A_OFFSET
#define TT12_A_OFFSET 0
#endif
#ifndef TT12_B_OFFSET
#define TT12_B_OFFSET 0
#endif
#ifndef TT12_C_OFFSET
#define TT12_C_OFFSET 0
#endif

#ifndef TT13_X_OFFSET
#define TT13_X_OFFSET 0
#endif
#ifndef TT13_Y_OFFSET
#define TT13_Y_OFFSET 0
#endif
#ifndef TT13_Z_OFFSET
#define TT13_Z_OFFSET 0
#endif
#ifndef TT13_U_OFFSET
#define TT13_U_OFFSET 0
#endif
#ifndef TT13_V_OFFSET
#define TT13_V_OFFSET 0
#endif
#ifndef TT13_W_OFFSET
#define TT13_W_OFFSET 0
#endif
#ifndef TT13_A_OFFSET
#define TT13_A_OFFSET 0
#endif
#ifndef TT13_B_OFFSET
#define TT13_B_OFFSET 0
#endif
#ifndef TT13_C_OFFSET
#define TT13_C_OFFSET 0
#endif

#ifndef TT14_X_OFFSET
#define TT14_X_OFFSET 0
#endif
#ifndef TT14_Y_OFFSET
#define TT14_Y_OFFSET 0
#endif
#ifndef TT14_Z_OFFSET
#define TT14_Z_OFFSET 0
#endif
#ifndef TT14_U_OFFSET
#define TT14_U_OFFSET 0
#endif
#ifndef TT14_V_OFFSET
#define TT14_V_OFFSET 0
#endif
#ifndef TT14_W_OFFSET
#define TT14_W_OFFSET 0
#endif
#ifndef TT14_A_OFFSET
#define TT14_A_OFFSET 0
#endif
#ifndef TT14_B_OFFSET
#define TT14_B_OFFSET 0
#endif
#ifndef TT14_C_OFFSET
#define TT14_C_OFFSET 0
#endif

#ifndef TT15_X_OFFSET
#define TT15_X_OFFSET 0
#endif
#ifndef TT15_Y_OFFSET
#define TT15_Y_OFFSET 0
#endif
#ifndef TT15_Z_OFFSET
#define TT15_Z_OFFSET 0
#endif
#ifndef TT15_U_OFFSET
#define TT15_U_OFFSET 0
#endif
#ifndef TT15_V_OFFSET
#define TT15_V_OFFSET 0
#endif
#ifndef TT15_W_OFFSET
#define TT15_W_OFFSET 0
#endif
#ifndef TT15_A_OFFSET
#define TT15_A_OFFSET 0
#endif
#ifndef TT15_B_OFFSET
#define TT15_B_OFFSET 0
#endif
#ifndef TT15_C_OFFSET
#define TT15_C_OFFSET 0
#endif

#ifndef TT16_X_OFFSET
#define TT16_X_OFFSET 0
#endif
#ifndef TT16_Y_OFFSET
#define TT16_Y_OFFSET 0
#endif
#ifndef TT16_Z_OFFSET
#define TT16_Z_OFFSET 0
#endif
#ifndef TT16_U_OFFSET
#define TT16_U_OFFSET 0
#endif
#ifndef TT16_V_OFFSET
#define TT16_V_OFFSET 0
#endif
#ifndef TT16_W_OFFSET
#define TT16_W_OFFSET 0
#endif
#ifndef TT16_A_OFFSET
#define TT16_A_OFFSET 0
#endif
#ifndef TT16_B_OFFSET
#define TT16_B_OFFSET 0
#endif
#ifndef TT16_C_OFFSET
#define TT16_C_OFFSET 0
#endif

#ifndef TT17_X_OFFSET
#define TT17_X_OFFSET 0
#endif
#ifndef TT17_Y_OFFSET
#define TT17_Y_OFFSET 0
#endif
#ifndef TT17_Z_OFFSET
#define TT17_Z_OFFSET 0
#endif
#ifndef TT17_U_OFFSET
#define TT17_U_OFFSET 0
#endif
#ifndef TT17_V_OFFSET
#define TT17_V_OFFSET 0
#endif
#ifndef TT17_W_OFFSET
#define TT17_W_OFFSET 0
#endif
#ifndef TT17_A_OFFSET
#define TT17_A_OFFSET 0
#endif
#ifndef TT17_B_OFFSET
#define TT17_B_OFFSET 0
#endif
#ifndef TT17_C_OFFSET
#define TT17_C_OFFSET 0
#endif

#ifndef TT18_X_OFFSET
#define TT18_X_OFFSET 0
#endif
#ifndef TT18_Y_OFFSET
#define TT18_Y_OFFSET 0
#endif
#ifndef TT18_Z_OFFSET
#define TT18_Z_OFFSET 0
#endif
#ifndef TT18_U_OFFSET
#define TT18_U_OFFSET 0
#endif
#ifndef TT18_V_OFFSET
#define TT18_V_OFFSET 0
#endif
#ifndef TT18_W_OFFSET
#define TT18_W_OFFSET 0
#endif
#ifndef TT18_A_OFFSET
#define TT18_A_OFFSET 0
#endif
#ifndef TT18_B_OFFSET
#define TT18_B_OFFSET 0
#endif
#ifndef TT18_C_OFFSET
#define TT18_C_OFFSET 0
#endif

#ifndef TT19_X_OFFSET
#define TT19_X_OFFSET 0
#endif
#ifndef TT19_Y_OFFSET
#define TT19_Y_OFFSET 0
#endif
#ifndef TT19_Z_OFFSET
#define TT19_Z_OFFSET 0
#endif
#ifndef TT19_U_OFFSET
#define TT19_U_OFFSET 0
#endif
#ifndef TT19_V_OFFSET
#define TT19_V_OFFSET 0
#endif
#ifndef TT19_W_OFFSET
#define TT19_W_OFFSET 0
#endif
#ifndef TT19_A_OFFSET
#define TT19_A_OFFSET 0
#endif
#ifndef TT19_B_OFFSET
#define TT19_B_OFFSET 0
#endif
#ifndef TT19_C_OFFSET
#define TT19_C_OFFSET 0
#endif

#ifndef TT20_X_OFFSET
#define TT20_X_OFFSET 0
#endif
#ifndef TT20_Y_OFFSET
#define TT20_Y_OFFSET 0
#endif
#ifndef TT20_Z_OFFSET
#define TT20_Z_OFFSET 0
#endif
#ifndef TT20_U_OFFSET
#define TT20_U_OFFSET 0
#endif
#ifndef TT20_V_OFFSET
#define TT20_V_OFFSET 0
#endif
#ifndef TT20_W_OFFSET
#define TT20_W_OFFSET 0
#endif
#ifndef TT20_A_OFFSET
#define TT20_A_OFFSET 0
#endif
#ifndef TT20_B_OFFSET
#define TT20_B_OFFSET 0
#endif
#ifndef TT20_C_OFFSET
#define TT20_C_OFFSET 0
#endif

#ifndef TT21_X_OFFSET
#define TT21_X_OFFSET 0
#endif
#ifndef TT21_Y_OFFSET
#define TT21_Y_OFFSET 0
#endif
#ifndef TT21_Z_OFFSET
#define TT21_Z_OFFSET 0
#endif
#ifndef TT21_U_OFFSET
#define TT21_U_OFFSET 0
#endif
#ifndef TT21_V_OFFSET
#define TT21_V_OFFSET 0
#endif
#ifndef TT21_W_OFFSET
#define TT21_W_OFFSET 0
#endif
#ifndef TT21_A_OFFSET
#define TT21_A_OFFSET 0
#endif
#ifndef TT21_B_OFFSET
#define TT21_B_OFFSET 0
#endif
#ifndef TT21_C_OFFSET
#define TT21_C_OFFSET 0
#endif

#ifndef TT22_X_OFFSET
#define TT22_X_OFFSET 0
#endif
#ifndef TT22_Y_OFFSET
#define TT22_Y_OFFSET 0
#endif
#ifndef TT22_Z_OFFSET
#define TT22_Z_OFFSET 0
#endif
#ifndef TT22_U_OFFSET
#define TT22_U_OFFSET 0
#endif
#ifndef TT22_V_OFFSET
#define TT22_V_OFFSET 0
#endif
#ifndef TT22_W_OFFSET
#define TT22_W_OFFSET 0
#endif
#ifndef TT22_A_OFFSET
#define TT22_A_OFFSET 0
#endif
#ifndef TT22_B_OFFSET
#define TT22_B_OFFSET 0
#endif
#ifndef TT22_C_OFFSET
#define TT22_C_OFFSET 0
#endif

#ifndef TT23_X_OFFSET
#define TT23_X_OFFSET 0
#endif
#ifndef TT23_Y_OFFSET
#define TT23_Y_OFFSET 0
#endif
#ifndef TT23_Z_OFFSET
#define TT23_Z_OFFSET 0
#endif
#ifndef TT23_U_OFFSET
#define TT23_U_OFFSET 0
#endif
#ifndef TT23_V_OFFSET
#define TT23_V_OFFSET 0
#endif
#ifndef TT23_W_OFFSET
#define TT23_W_OFFSET 0
#endif
#ifndef TT23_A_OFFSET
#define TT23_A_OFFSET 0
#endif
#ifndef TT23_B_OFFSET
#define TT23_B_OFFSET 0
#endif
#ifndef TT23_C_OFFSET
#define TT23_C_OFFSET 0
#endif

#ifndef TT24_X_OFFSET
#define TT24_X_OFFSET 0
#endif
#ifndef TT24_Y_OFFSET
#define TT24_Y_OFFSET 0
#endif
#ifndef TT24_Z_OFFSET
#define TT24_Z_OFFSET 0
#endif
#ifndef TT24_U_OFFSET
#define TT24_U_OFFSET 0
#endif
#ifndef TT24_V_OFFSET
#define TT24_V_OFFSET 0
#endif
#ifndef TT24_W_OFFSET
#define TT24_W_OFFSET 0
#endif
#ifndef TT24_A_OFFSET
#define TT24_A_OFFSET 0
#endif
#ifndef TT24_B_OFFSET
#define TT24_B_OFFSET 0
#endif
#ifndef TT24_C_OFFSET
#define TT24_C_OFFSET 0
#endif

#ifndef TT25_X_OFFSET
#define TT25_X_OFFSET 0
#endif
#ifndef TT25_Y_OFFSET
#define TT25_Y_OFFSET 0
#endif
#ifndef TT25_Z_OFFSET
#define TT25_Z_OFFSET 0
#endif
#ifndef TT25_U_OFFSET
#define TT25_U_OFFSET 0
#endif
#ifndef TT25_V_OFFSET
#define TT25_V_OFFSET 0
#endif
#ifndef TT25_W_OFFSET
#define TT25_W_OFFSET 0
#endif
#ifndef TT25_A_OFFSET
#define TT25_A_OFFSET 0
#endif
#ifndef TT25_B_OFFSET
#define TT25_B_OFFSET 0
#endif
#ifndef TT25_C_OFFSET
#define TT25_C_OFFSET 0
#endif

#ifndef TT26_X_OFFSET
#define TT26_X_OFFSET 0
#endif
#ifndef TT26_Y_OFFSET
#define TT26_Y_OFFSET 0
#endif
#ifndef TT26_Z_OFFSET
#define TT26_Z_OFFSET 0
#endif
#ifndef TT26_U_OFFSET
#define TT26_U_OFFSET 0
#endif
#ifndef TT26_V_OFFSET
#define TT26_V_OFFSET 0
#endif
#ifndef TT26_W_OFFSET
#define TT26_W_OFFSET 0
#endif
#ifndef TT26_A_OFFSET
#define TT26_A_OFFSET 0
#endif
#ifndef TT26_B_OFFSET
#define TT26_B_OFFSET 0
#endif
#ifndef TT26_C_OFFSET
#define TT26_C_OFFSET 0
#endif

#ifndef TT27_X_OFFSET
#define TT27_X_OFFSET 0
#endif
#ifndef TT27_Y_OFFSET
#define TT27_Y_OFFSET 0
#endif
#ifndef TT27_Z_OFFSET
#define TT27_Z_OFFSET 0
#endif
#ifndef TT27_U_OFFSET
#define TT27_U_OFFSET 0
#endif
#ifndef TT27_V_OFFSET
#define TT27_V_OFFSET 0
#endif
#ifndef TT27_W_OFFSET
#define TT27_W_OFFSET 0
#endif
#ifndef TT27_A_OFFSET
#define TT27_A_OFFSET 0
#endif
#ifndef TT27_B_OFFSET
#define TT27_B_OFFSET 0
#endif
#ifndef TT27_C_OFFSET
#define TT27_C_OFFSET 0
#endif

#ifndef TT28_X_OFFSET
#define TT28_X_OFFSET 0
#endif
#ifndef TT28_Y_OFFSET
#define TT28_Y_OFFSET 0
#endif
#ifndef TT28_Z_OFFSET
#define TT28_Z_OFFSET 0
#endif
#ifndef TT28_U_OFFSET
#define TT28_U_OFFSET 0
#endif
#ifndef TT28_V_OFFSET
#define TT28_V_OFFSET 0
#endif
#ifndef TT28_W_OFFSET
#define TT28_W_OFFSET 0
#endif
#ifndef TT28_A_OFFSET
#define TT28_A_OFFSET 0
#endif
#ifndef TT28_B_OFFSET
#define TT28_B_OFFSET 0
#endif
#ifndef TT28_C_OFFSET
#define TT28_C_OFFSET 0
#endif

#ifndef TT29_X_OFFSET
#define TT29_X_OFFSET 0
#endif
#ifndef TT29_Y_OFFSET
#define TT29_Y_OFFSET 0
#endif
#ifndef TT29_Z_OFFSET
#define TT29_Z_OFFSET 0
#endif
#ifndef TT29_U_OFFSET
#define TT29_U_OFFSET 0
#endif
#ifndef TT29_V_OFFSET
#define TT29_V_OFFSET 0
#endif
#ifndef TT29_W_OFFSET
#define TT29_W_OFFSET 0
#endif
#ifndef TT29_A_OFFSET
#define TT29_A_OFFSET 0
#endif
#ifndef TT29_B_OFFSET
#define TT29_B_OFFSET 0
#endif
#ifndef TT29_C_OFFSET
#define TT29_C_OFFSET 0
#endif

#ifndef TT30_X_OFFSET
#define TT30_X_OFFSET 0
#endif
#ifndef TT30_Y_OFFSET
#define TT30_Y_OFFSET 0
#endif
#ifndef TT30_Z_OFFSET
#define TT30_Z_OFFSET 0
#endif
#ifndef TT30_U_OFFSET
#define TT30_U_OFFSET 0
#endif
#ifndef TT30_V_OFFSET
#define TT30_V_OFFSET 0
#endif
#ifndef TT30_W_OFFSET
#define TT30_W_OFFSET 0
#endif
#ifndef TT30_A_OFFSET
#define TT30_A_OFFSET 0
#endif
#ifndef TT30_B_OFFSET
#define TT30_B_OFFSET 0
#endif
#ifndef TT30_C_OFFSET
#define TT30_C_OFFSET 0
#endif

#ifndef TT31_X_OFFSET
#define TT31_X_OFFSET 0
#endif
#ifndef TT31_Y_OFFSET
#define TT31_Y_OFFSET 0
#endif
#ifndef TT31_Z_OFFSET
#define TT31_Z_OFFSET 0
#endif
#ifndef TT31_U_OFFSET
#define TT31_U_OFFSET 0
#endif
#ifndef TT31_V_OFFSET
#define TT31_V_OFFSET 0
#endif
#ifndef TT31_W_OFFSET
#define TT31_W_OFFSET 0
#endif
#ifndef TT31_A_OFFSET
#define TT31_A_OFFSET 0
#endif
#ifndef TT31_B_OFFSET
#define TT31_B_OFFSET 0
#endif
#ifndef TT31_C_OFFSET
#define TT31_C_OFFSET 0
#endif

#ifndef TT32_X_OFFSET
#define TT32_X_OFFSET 0
#endif
#ifndef TT32_Y_OFFSET
#define TT32_Y_OFFSET 0
#endif
#ifndef TT32_Z_OFFSET
#define TT32_Z_OFFSET 0
#endif
#ifndef TT32_U_OFFSET
#define TT32_U_OFFSET 0
#endif
#ifndef TT32_V_OFFSET
#define TT32_V_OFFSET 0
#endif
#ifndef TT32_W_OFFSET
#define TT32_W_OFFSET 0
#endif
#ifndef TT32_A_OFFSET
#define TT32_A_OFFSET 0
#endif
#ifndef TT32_B_OFFSET
#define TT32_B_OFFSET 0
#endif
#ifndef TT32_C_OFFSET
#define TT32_C_OFFSET 0
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
