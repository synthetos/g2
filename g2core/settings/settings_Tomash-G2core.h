
// ********************************************************************************************** //
// ********************************** TOMASH-G2core profile ************************************* //
// ********************************************************************************************** //


// ***> NOTE: The init message must be a single line with no CRs or LFs
#define INIT_MESSAGE "Ucitavam konfiguraciju Tomash G2core"


// ********************************************************************************************** //
// ********************************** GLOBAL / GENERAL SETTINGS ********************************* //
// ********************************************************************************************** //


// *** Machine configuration settings *** //

#define JUNCTION_INTEGRATION_TIME   0.8    // {jt: cornering - between 0.05 and 2.00 (max)
#define CHORDAL_TOLERANCE           0.001    // {ct: chordal tolerance for arcs (in mm)

#define SOFT_LIMIT_ENABLE           1       // {sl: 0=off, 1=on
#define HARD_LIMIT_ENABLE           1       // {lim: 0=off, 1=on
#define SAFETY_INTERLOCK_ENABLE     1       // {saf: 0=off, 1=on

#define FEEDHOLD_Z_LIFT             10       // {zl: mm to lift Z on feedhold

#define PROBE_REPORT_ENABLE         true    // {prbr:

#define MANUAL_FEEDRATE_OVERRIDE_ENABLE true
#define MANUAL_FEEDRATE_OVERRIDE_PARAMETER 1.00

    
// *** Communications and Reporting Settings *** //

#define USB_SERIAL_PORTS_EXPOSED    1                        // Valid options are 1 or 2, only!
#define XIO_ENABLE_FLOW_CONTROL     FLOW_CONTROL_RTS        // {ex: FLOW_CONTROL_OFF, FLOW_CONTROL_XON, FLOW_CONTROL_RTS
#define COMM_MODE                   JSON_MODE               // {ej: TEXT_MODE, JSON_MODE
#define TEXT_VERBOSITY              TV_VERBOSE              // {tv: TV_SILENT, TV_VERBOSE
#define XIO_UART_MUTES_WHEN_USB_CONNECTED  0                // UART will be muted when USB connected (off by default)
#define JSON_VERBOSITY              JV_MESSAGES             // {jv: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE
#define QUEUE_REPORT_VERBOSITY      QR_OFF                  // {qv: QR_OFF, QR_SINGLE, QR_TRIPLE
#define STATUS_REPORT_VERBOSITY     SR_FILTERED             // {sv: SR_OFF, SR_FILTERED, SR_VERBOSE
#define STATUS_REPORT_MIN_MS        100                     // (no JSON) milliseconds - enforces a viable minimum
#define STATUS_REPORT_INTERVAL_MS   100                     // {si: milliseconds - set $SV=0 to disable
#define STATUS_REPORT_DEFAULTS "line","posx","posy","posz","posa","feed","vel","unit","coor","dist","admo","frmo","momo","stat"
// Alternate SRs that report in drawable units
//#define STATUS_REPORT_DEFAULTS "line","vel","mpox","mpoy","mpoz","mpoa","coor","ofsa","ofsx","ofsy","ofsz","dist","unit","stat","homz","homy","homx","momo"
#define MARLIN_COMPAT_ENABLED       false                   // boolean, either true or false


// *** Gcode Startup Defaults *** //

#define GCODE_DEFAULT_UNITS         MILLIMETERS             // {gun: MILLIMETERS or INCHES
#define GCODE_DEFAULT_PLANE         CANON_PLANE_XY          // {gpl: CANON_PLANE_XY, CANON_PLANE_XZ, or CANON_PLANE_YZ
#define GCODE_DEFAULT_COORD_SYSTEM  G54                     // {gco: G54, G55, G56, G57, G58 or G59
#define GCODE_DEFAULT_PATH_CONTROL  PATH_CONTINUOUS         // {gpa: PATH_EXACT_PATH, PATH_EXACT_STOP, PATH_CONTINUOUS
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_DISTANCE_MODE  // {gdi: ABSOLUTE_DISTANCE_MODE, INCREMENTAL_DISTANCE_MODE


// ********************************************************************************************** //
// ****************************** PWM & SPINDLE & COOLANT Settings ****************************** //
// ********************************************************************************************** //


// *** SPINDLE settings *** //

#define SPINDLE_MODE                1       // {spmo; 0=diabled, 1=plan to stop, 2=continuous
#define SPINDLE_ENABLE_POLARITY     1       // 0=active low, 1=active high
#define SPINDLE_DIR_POLARITY        0       // {spdp: 0=clockwise is low, 1=clockwise is high
#define SPINDLE_PAUSE_ON_HOLD       true   // {spph:
#define SPINDLE_SPINUP_DELAY        3     // {spde:
//#define SPINDLE_OVERRIDE_ENABLE 1
//#define SPINDLE_OVERRIDE_FACTOR 1.0
//#define SPINDLE_OVERRIDE_MIN FEED_OVERRIDE_MIN
//#define SPINDLE_OVERRIDE_MAX FEED_OVERRIDE_MAX
#define SPINDLE_DWELL_MAX   10000000.0      // maximum allowable dwell time. May be overridden in settings files
#define SPINDLE_SPEED_MIN           0.0     // {spsn:
#define SPINDLE_SPEED_MAX     1000000.0     // {spsm:


// *** COOLANT settings *** //

#define COOLANT_MIST_POLARITY       1       // {comp: 0=active low, 1=active high
#define COOLANT_FLOOD_POLARITY      1       // {cofp: 0=active low, 1=active high
#define COOLANT_PAUSE_ON_HOLD       true    // {coph:


// *** PWM Settings *** //

#define P1_PWM_FREQUENCY            3000                   // in Hz
#define P1_CW_SPEED_LO              0                  // in RPM (arbitrary units)
#define P1_CW_SPEED_HI              24000
#define P1_CW_PHASE_LO              0.0                    // phase [0..1]
#define P1_CW_PHASE_HI              1.00
#define P1_PWM_PHASE_OFF            0.0
#define P1_PWM2_FREQUENCY2            1000                   // in Hz
#define P1_CCW_SPEED_LO             0    // 0.0
#define P1_CCW_SPEED_HI             100    // 0.0
#define P1_CCW_PHASE_LO             0.0    // 0.1
#define P1_CCW_PHASE_HI             1.00    // 0.1


// ********************************************************************************************** //
// *************************************** MOTOR SETTINGS *************************************** //
// ********************************************************************************************** //


#define MOTOR_POWER_MODE            MOTOR_ALWAYS_POWERED
#define MOTOR_POWER_TIMEOUT         2.00    // {mt:  motor power timeout in seconds

// MOTOR 1
#define M1_MOTOR_MAP                AXIS_Z_EXTERNAL         // {1ma: AXIS_X, AXIS_Y...
#define M1_STEP_ANGLE               1.8                     // {1sa: degrees per step
#define M1_TRAVEL_PER_REV           5.00086                 // {1tr:  1.25 is a typical value for a screw axis
#define M1_MICROSTEPS               16                      // {1mi:  1,2,4,8,    16,32 (G2 ONLY)
//#define M1_STEPS_PER_UNIT           639.88983               // {1su:  steps to issue per unit of length or degrees of rotation
#define M1_POLARITY                 1                       // {1po:  0=normal direction, 1=inverted direction
#define M1_ENABLE_POLARITY          IO_ACTIVE_LOW           // {1ep:  IO_ACTIVE_LOW or IO_ACTIVE_HIGH
#define M1_STEP_POLARITY            IO_ACTIVE_HIGH          // {1ps:  IO_ACTIVE_LOW or IO_ACTIVE_HIGH
#define M1_POWER_MODE               MOTOR_POWER_MODE        // {1pm:  MOTOR_DISABLED, MOTOR_ALWAYS_POWERED, MOTOR_POWERED_IN_CYCLE, MOTOR_POWERED_ONLY_WHEN_MOVING
#define M1_POWER_LEVEL              1.0                     // {1pl:   0.0=no power, 1.0=max power
#define M1_POWER_LEVEL_IDLE         (M1_POWER_LEVEL/2.0)


// MOTOR 2
#define M2_MOTOR_MAP                AXIS_X_EXTERNAL
#define M2_STEP_ANGLE               1.8
#define M2_TRAVEL_PER_REV           23.00555
#define M2_MICROSTEPS               16
//#define M2_STEPS_PER_UNIT           139.09687
#define M2_POLARITY                 1
#define M2_ENABLE_POLARITY          IO_ACTIVE_LOW
#define M2_STEP_POLARITY            IO_ACTIVE_HIGH
#define M2_POWER_MODE               MOTOR_POWER_MODE
#define M2_POWER_LEVEL              1.0
#define M2_POWER_LEVEL_IDLE         (M2_POWER_LEVEL/2.0)


// MOTOR 3
#define M3_MOTOR_MAP                AXIS_Y_EXTERNAL
#define M3_STEP_ANGLE               1.8
#define M3_TRAVEL_PER_REV           23.00094                    // 1.25 is a typical value for a screw axis
#define M3_MICROSTEPS               16
//#define M3_STEPS_PER_UNIT           139.12475
#define M3_POLARITY                 0
#define M3_ENABLE_POLARITY          IO_ACTIVE_LOW
#define M3_STEP_POLARITY            IO_ACTIVE_HIGH
#define M3_POWER_MODE               MOTOR_POWER_MODE
#define M3_POWER_LEVEL              1.0
#define M3_POWER_LEVEL_IDLE         (M3_POWER_LEVEL/2.0)


// ************************* DISABLED MOTORS ***************************** //


// MOTOR 4
#define M4_MOTOR_MAP                AXIS_Y_EXTERNAL
#define M4_STEP_ANGLE               1.8
#define M4_TRAVEL_PER_REV           23.00094                    // 1.25 is a typical value for a screw axis
#define M4_MICROSTEPS               16
//#define M4_STEPS_PER_UNIT           139.12475
#define M4_POLARITY                 1
#define M4_ENABLE_POLARITY          IO_ACTIVE_LOW
#define M4_STEP_POLARITY            IO_ACTIVE_HIGH
#define M4_POWER_MODE               MOTOR_POWER_MODE
#define M4_POWER_LEVEL              1.0
#define M4_POWER_LEVEL_IDLE         (M3_POWER_LEVEL/2.0)


// MOTOR 5
#define M5_MOTOR_MAP                AXIS_A_EXTERNAL
#define M5_STEP_ANGLE               1.8
#define M5_TRAVEL_PER_REV           360.0
#define M5_MICROSTEPS               8
#define M5_STEPS_PER_UNIT           0
#define M5_POLARITY                 0
#define M5_ENABLE_POLARITY          IO_ACTIVE_LOW
#define M5_STEP_POLARITY            IO_ACTIVE_HIGH
#define M5_POWER_MODE               MOTOR_DISABLED
#define M5_POWER_LEVEL              0.0
#define M5_POWER_LEVEL_IDLE         (M5_POWER_LEVEL/2.0)


// MOTOR 6
#define M6_MOTOR_MAP                AXIS_B_EXTERNAL
#define M6_STEP_ANGLE               1.8
#define M6_TRAVEL_PER_REV           360.0
#define M6_MICROSTEPS               8
#define M6_STEPS_PER_UNIT           0
#define M6_POLARITY                 0
#define M6_ENABLE_POLARITY          IO_ACTIVE_LOW
#define M6_STEP_POLARITY            IO_ACTIVE_HIGH
#define M6_POWER_MODE               MOTOR_DISABLED
#define M6_POWER_LEVEL              0.0
#define M6_POWER_LEVEL_IDLE         (M6_POWER_LEVEL/2.0)


// ********************************************************************************************** //
// ***************************************** AXIS SETTINGS ************************************** //
// ********************************************************************************************** //

// X AXIS
#define X_AXIS_MODE                 AXIS_STANDARD           // {xam:  see canonical_machine.h cmAxisMode for valid values
#define X_VELOCITY_MAX              9000.0                  // {xvm:  G0 max velocity in mm/min
#define X_FEEDRATE_MAX              4000.0                  // {xfr:  G1 max feed rate in mm/min
#define X_TRAVEL_MIN                -70.0                   // {xtn:  minimum travel for soft limits
#define X_TRAVEL_MAX                1700.0                  // {xtm:  travel between switches or crashes
#define X_JERK_MAX                  1000.0                  // {xjm:
#define X_JERK_HIGH_SPEED           10000.0                 // {xjh:
#define X_HOMING_INPUT              1                       // {xhi:  input used for homing or 0 to disable
#define X_HOMING_DIRECTION          0                       // {xhd:  0=search moves negative, 1= search moves positive
#define X_SEARCH_VELOCITY           2500.0                   // {xsv:  minus means move to minimum switch
#define X_LATCH_VELOCITY            100.0                   // {xlv:  mm/min
#define X_LATCH_BACKOFF             4.0                     // {xlb:  mm
#define X_ZERO_BACKOFF              2.0                     // {xzb:  mm


// Y AXIS
#define Y_AXIS_MODE                 AXIS_STANDARD
#define Y_VELOCITY_MAX              9000.0
#define Y_FEEDRATE_MAX              4000.0
#define Y_TRAVEL_MIN                -2400.0
#define Y_TRAVEL_MAX                0.0
#define Y_JERK_MAX                  1000.0
#define Y_JERK_HIGH_SPEED           10000.0
#define Y_HOMING_INPUT              3
#define Y_HOMING_DIRECTION          1
#define Y_SEARCH_VELOCITY           2500.0
#define Y_LATCH_VELOCITY            100.0
#define Y_LATCH_BACKOFF             4.0
#define Y_ZERO_BACKOFF              2.0


// Z AXIS
#define Z_AXIS_MODE                 AXIS_STANDARD
#define Z_VELOCITY_MAX              4000.0
#define Z_FEEDRATE_MAX              2500.0
#define Z_TRAVEL_MAX                0.0
#define Z_TRAVEL_MIN                -285.0
#define Z_JERK_MAX                  500.0
#define Z_JERK_HIGH_SPEED           10000.0
#define Z_HOMING_INPUT              6
#define Z_HOMING_DIRECTION          1
#define Z_SEARCH_VELOCITY           1500.0
#define Z_LATCH_VELOCITY            50.0
#define Z_LATCH_BACKOFF             4.0
#define Z_ZERO_BACKOFF              2.0


// ************************* DISABLED AXIS ***************************** //


// U AXIS
#define U_AXIS_MODE                 AXIS_DISABLED           // {xam:  see canonical_machine.h cmAxisMode for valid values
#define U_VELOCITY_MAX              1000.0                  // {xvm:  G0 max velocity in mm/min
#define U_FEEDRATE_MAX              1000.0                  // {xfr:  G1 max feed rate in mm/min
#define U_TRAVEL_MIN                0.0                     // {xtn:  minimum travel for soft limits
#define U_TRAVEL_MAX                0.0                     // {xtm:  travel between switches or crashes
#define U_JERK_MAX                  1000.0                  // {xjm:
#define U_JERK_HIGH_SPEED           1000.0                  // {xjh:
#define U_HOMING_INPUT              0                       // {xhi:  input used for homing or 0 to disable
#define U_HOMING_DIRECTION          0                       // {xhd:  0=search moves negative, 1= search moves positive
#define U_SEARCH_VELOCITY           500.0                   // {xsv:  minus means move to minimum switch
#define U_LATCH_VELOCITY            100.0                   // {xlv:  mm/min
#define U_LATCH_BACKOFF             4.0                     // {xlb:  mm
#define U_ZERO_BACKOFF              2.0                     // {xzb:  mm


// V AXIS
#define V_AXIS_MODE                 AXIS_DISABLED
#define V_VELOCITY_MAX              1000.0
#define V_FEEDRATE_MAX              1000.0
#define V_TRAVEL_MIN                0.0
#define V_TRAVEL_MAX                0.0
#define V_JERK_MAX                  1000.0
#define V_JERK_HIGH_SPEED           1000.0
#define V_HOMING_INPUT              0
#define V_HOMING_DIRECTION          0
#define V_SEARCH_VELOCITY           500.0
#define V_LATCH_VELOCITY            100.0
#define V_LATCH_BACKOFF             4.0
#define V_ZERO_BACKOFF              2.0


// W AXIS
#define W_AXIS_MODE                 AXIS_DISABLED
#define W_VELOCITY_MAX              1000.0
#define W_FEEDRATE_MAX              1000.0
#define W_TRAVEL_MAX                0.0
#define W_TRAVEL_MIN                0.0
#define W_JERK_MAX                  500.0
#define W_JERK_HIGH_SPEED           500.0
#define W_HOMING_INPUT              0
#define W_HOMING_DIRECTION          0
#define W_SEARCH_VELOCITY           250.0
#define W_LATCH_VELOCITY            25.0
#define W_LATCH_BACKOFF             4.0
#define W_ZERO_BACKOFF              2.0


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
#define A_AXIS_MODE                 AXIS_DISABLED
#define A_RADIUS                    (M4_TRAVEL_PER_REV/(2*3.14159628))
#define A_VELOCITY_MAX              ((X_VELOCITY_MAX/M1_TRAVEL_PER_REV)*360) // set to the same speed as X axis
#define A_FEEDRATE_MAX              A_VELOCITY_MAX
#define A_TRAVEL_MIN                -1.0                    // min/max the same means infinite, no limit
#define A_TRAVEL_MAX                -1.0
#define A_JERK_MAX                  (X_JERK_MAX*(360/M1_TRAVEL_PER_REV))
#define A_JERK_HIGH_SPEED           A_JERK_MAX
#define A_HOMING_INPUT              0
#define A_HOMING_DIRECTION          0
#define A_SEARCH_VELOCITY           (A_VELOCITY_MAX * 0.500)
#define A_LATCH_VELOCITY            (A_VELOCITY_MAX * 0.100)
#define A_LATCH_BACKOFF             5.0
#define A_ZERO_BACKOFF              2.0


// B AXIS
#define B_AXIS_MODE                 AXIS_DISABLED
#define B_RADIUS                    (M5_TRAVEL_PER_REV/(2*3.14159628))
#define B_VELOCITY_MAX              ((X_VELOCITY_MAX/M1_TRAVEL_PER_REV)*360)
#define B_FEEDRATE_MAX              B_VELOCITY_MAX
#define B_TRAVEL_MIN                -1.0
#define B_TRAVEL_MAX                -1.0
#define B_JERK_MAX                  (X_JERK_MAX*(360/M1_TRAVEL_PER_REV))
#define B_JERK_HIGH_SPEED           B_JERK_MAX
#define B_HOMING_INPUT              0
#define B_HOMING_DIRECTION          0
#define B_SEARCH_VELOCITY           (B_VELOCITY_MAX * 0.500)
#define B_LATCH_VELOCITY            (B_VELOCITY_MAX * 0.100)
#define B_LATCH_BACKOFF             5.0
#define B_ZERO_BACKOFF              2.0


// C AXIS
#define C_AXIS_MODE                 AXIS_DISABLED
#define C_RADIUS                    (M6_TRAVEL_PER_REV/(2*3.14159628))
#define C_VELOCITY_MAX              ((X_VELOCITY_MAX/M1_TRAVEL_PER_REV)*360)
#define C_FEEDRATE_MAX              C_VELOCITY_MAX
#define C_TRAVEL_MIN                -1.0
#define C_TRAVEL_MAX                -1.0
#define C_JERK_MAX                  (X_JERK_MAX*(360/M1_TRAVEL_PER_REV))
#define C_JERK_HIGH_SPEED           C_JERK_MAX
#define C_HOMING_INPUT              0
#define C_HOMING_DIRECTION          0
#define C_SEARCH_VELOCITY           (C_VELOCITY_MAX * 0.500)
#define C_LATCH_VELOCITY            (C_VELOCITY_MAX * 0.100)
#define C_LATCH_BACKOFF             5.0
#define C_ZERO_BACKOFF              2.0


//*****************************************************************************
//*** GPIO Input / Output Settings ********************************************
//*****************************************************************************

// DIGITAL INPUTS
// Set to allow the board to function if not otherwise set up
// (least disruptive settings)

// Set the probing input (universal, all axis probe)
//#define PROBING_INPUT 5

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
#define DI1_MODE                    IO_ACTIVE_HIGH     // Normally closed
#define DI1_ACTION                  INPUT_ACTION_FAST_STOP
#define DI1_FUNCTION                INPUT_FUNCTION_LIMIT

// Xmax
#define DI2_MODE                    IO_ACTIVE_HIGH     // Normally closed
#define DI2_ACTION                  INPUT_ACTION_FAST_STOP
#define DI2_FUNCTION                INPUT_FUNCTION_LIMIT

// Ymin
#define DI3_MODE                    IO_ACTIVE_HIGH     // Normally closed
#define DI3_ACTION                  INPUT_ACTION_FAST_STOP
#define DI3_FUNCTION                INPUT_FUNCTION_LIMIT

// Ymax
#define DI4_MODE                    IO_ACTIVE_HIGH     // Normally closed
#define DI4_ACTION                  INPUT_ACTION_FAST_STOP
#define DI4_FUNCTION                INPUT_FUNCTION_LIMIT

// Zmin
#define DI5_MODE                    IO_ACTIVE_HIGH     // Normally closed
#define DI5_ACTION                  INPUT_ACTION_FAST_STOP
#define DI5_FUNCTION                INPUT_FUNCTION_PROBE

// Zmax
#define DI6_MODE                    IO_ACTIVE_HIGH     // Normally closed
#define DI6_ACTION                  INPUT_ACTION_FAST_STOP
#define DI6_FUNCTION                INPUT_FUNCTION_LIMIT

// Amin (X min probe)
#define DI7_MODE                    IO_ACTIVE_HIGH     // Normally closed
#define DI7_ACTION                  INPUT_ACTION_FAST_STOP
#define DI7_FUNCTION                INPUT_FUNCTION_LIMIT

// Amax
#define DI8_MODE                    IO_MODE_DISABLED     // Normally closed
#define DI8_ACTION                  INPUT_ACTION_NONE
#define DI8_FUNCTION                INPUT_FUNCTION_NONE

// Safety line (E-STOP)
#define DI9_MODE                    IO_ACTIVE_HIGH     // Normally closed
#define DI9_ACTION                  INPUT_ACTION_ALARM
#define DI9_FUNCTION                INPUT_FUNCTION_SHUTDOWN

#define DI10_MODE                   IO_MODE_DISABLED     // Normally closed
#define DI10_ACTION                 INPUT_ACTION_NONE
#define DI10_FUNCTION               INPUT_FUNCTION_NONE

#define DI11_MODE                   IO_MODE_DISABLED     // Normally closed
#define DI11_ACTION                 INPUT_ACTION_NONE
#define DI11_FUNCTION               INPUT_FUNCTION_NONE

#define DI12_MODE                   IO_MODE_DISABLED     // Normally closed
#define DI12_ACTION                 INPUT_ACTION_NONE
#define DI12_FUNCTION               INPUT_FUNCTION_NONE



//// Xmin on v9 board
//#define DI1_ENABLED                 IO_ENABLED
//#define DI1_POLARITY                IO_ACTIVE_HIGH     // Normally closed
//#define DI1_ACTION                  INPUT_ACTION_LIMIT
//#define DI1_EXTERNAL_NUMBER         1
//
//
//// Xmax
//#define DI2_ENABLED                 IO_DISABLED
//#define DI2_POLARITY                IO_ACTIVE_HIGH     // Normally closed
//#define DI2_ACTION                  INPUT_ACTION_LIMIT
//#define DI2_EXTERNAL_NUMBER         2
//
//
//// Ymin
//#define DI3_ENABLED                 IO_ENABLED
//#define DI3_POLARITY                IO_ACTIVE_HIGH     // Normally closed
//#define DI3_ACTION                  INPUT_ACTION_LIMIT
//#define DI3_EXTERNAL_NUMBER         3
//
//
//// Ymax
//#define DI4_ENABLED                 IO_ENABLED
//#define DI4_POLARITY                IO_ACTIVE_HIGH     // Normally closed
//#define DI4_ACTION                  INPUT_ACTION_NONE
//#define DI4_EXTERNAL_NUMBER         4
//
//
//// Zmin
//#define DI5_ENABLED                 IO_ENABLED
//#define DI5_POLARITY                IO_ACTIVE_HIGH     // Normally closed
//#define DI5_ACTION                  INPUT_ACTION_NONE
//#define DI5_EXTERNAL_NUMBER         5
//
//
//// Zmax
//#define DI6_ENABLED                 IO_ENABLED
//#define DI6_POLARITY                IO_ACTIVE_HIGH     // Normally closed
//#define DI6_ACTION                  INPUT_ACTION_LIMIT
//#define DI6_EXTERNAL_NUMBER         6
//
//
//// Amin
//#define DI7_ENABLED                 IO_ENABLED
//#define DI7_POLARITY                IO_ACTIVE_HIGH     // Normally closed
//#define DI7_ACTION                  INPUT_ACTION_LIMIT
//#define DI7_EXTERNAL_NUMBER         7
//
//
//// Amax
//#define DI8_ENABLED                 IO_DISABLED
//#define DI8_POLARITY                IO_ACTIVE_HIGH     // Normally closed
//#define DI8_ACTION                  INPUT_ACTION_NONE
//#define DI8_EXTERNAL_NUMBER         8
//
//
//// Safety line
//#define DI9_ENABLED                 IO_DISABLED
//#define DI9_POLARITY                IO_ACTIVE_HIGH     // Normally closed
//#define DI9_ACTION                  INPUT_ACTION_NONE
//#define DI9_EXTERNAL_NUMBER         9
//
//
//#define DI10_ENABLED                IO_DISABLED
//#define DI10_POLARITY               IO_ACTIVE_HIGH     // Normally open
//#define DI10_ACTION                 INPUT_ACTION_NONE
//#define DI10_EXTERNAL_NUMBER         10
//
//
//#define DI11_ENABLED                IO_DISABLED
//#define DI11_POLARITY               IO_ACTIVE_LOW     // Normally open
//#define DI11_ACTION                 INPUT_ACTION_NONE
//#define DI11_EXTERNAL_NUMBER         11
//
//
//#define DI12_ENABLED                IO_DISABLED
//#define DI12_POLARITY               IO_ACTIVE_LOW     // Normally open
//#define DI12_ACTION                 INPUT_ACTION_NONE
//#define DI12_EXTERNAL_NUMBER         12


//#define PROBING_INPUT                Z_HOMING_INPUT // default to the z homing input
//#define PROBING_INPUT                5 // default to the z homing input


// DIGITAL OUTPUTS - Currently these are hard-wired to extruders

//Extruder1_PWM
//#define DO1_ENABLED                 IO_UNAVAILABLE
//#define DO1_POLARITY                IO_ACTIVE_HIGH
//#define DO1_EXTERNAL_NUMBER         1
//
//
////Extruder2_PWM
//#define DO2_ENABLED                 IO_UNAVAILABLE
//#define DO2_POLARITY                IO_ACTIVE_HIGH
//#define DO2_EXTERNAL_NUMBER         2
//
//
////Fan1A_PWM
//#define DO3_ENABLED                 IO_UNAVAILABLE
//#define DO3_POLARITY                IO_ACTIVE_HIGH
//#define DO3_EXTERNAL_NUMBER         3
//
//
////Fan1B_PWM
//#define DO4_ENABLED                 IO_UNAVAILABLE
//#define DO4_POLARITY                IO_ACTIVE_HIGH
//#define DO4_EXTERNAL_NUMBER         4
//
//
//#define DO5_ENABLED                 IO_UNAVAILABLE
//#define DO5_POLARITY                IO_ACTIVE_HIGH
//#define DO5_EXTERNAL_NUMBER         5
//
//
//#define DO6_ENABLED                 IO_UNAVAILABLE
//#define DO6_POLARITY                IO_ACTIVE_HIGH
//#define DO6_EXTERNAL_NUMBER         6
//
//
//#define DO7_ENABLED                 IO_UNAVAILABLE
//#define DO7_POLARITY                IO_ACTIVE_HIGH
//#define DO7_EXTERNAL_NUMBER         7
//
//
//#define DO8_ENABLED                 IO_UNAVAILABLE
//#define DO8_POLARITY                IO_ACTIVE_HIGH
//#define DO8_EXTERNAL_NUMBER         8
//
////SAFEin (Output) signal
//#define DO9_ENABLED                 IO_UNAVAILABLE
//#define DO9_POLARITY                IO_ACTIVE_HIGH
//#define DO9_EXTERNAL_NUMBER         9
//
//
//#define DO10_ENABLED                IO_UNAVAILABLE
//#define DO10_POLARITY               IO_ACTIVE_HIGH
//#define DO10_EXTERNAL_NUMBER         10
//
//
////Header Bed FET
//#define DO11_ENABLED                IO_UNAVAILABLE
//#define DO11_POLARITY               IO_ACTIVE_HIGH
//#define DO11_EXTERNAL_NUMBER         11
//
//
////Indicator_LED
//#define DO12_ENABLED                IO_UNAVAILABLE
//#define DO12_POLARITY               IO_ACTIVE_HIGH
//#define DO12_EXTERNAL_NUMBER         12
//
//
//#define DO13_ENABLED                IO_UNAVAILABLE
//#define DO13_POLARITY               IO_ACTIVE_HIGH
//#define DO13_EXTERNAL_NUMBER         13


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



