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
#define INIT_MESSAGE "Initializing configs to Ultimaker 2+ profile"

#ifndef PI
#define PI 3.14159628
#endif

//**** GLOBAL / GENERAL SETTINGS ******************************************************

#define JUNCTION_INTEGRATION_TIME   1.2                    // cornering - between 0.10 and 2.00 (higher is faster)
//{jt:1.2}
//{jt:0.75}
#define CHORDAL_TOLERANCE           0.01                    // chordal accuracy for arc drawing (in mm)

#define SOFT_LIMIT_ENABLE           0                       // 0=off, 1=on
#define HARD_LIMIT_ENABLE           1                       // 0=off, 1=on
#define SAFETY_INTERLOCK_ENABLE     1                       // 0=off, 1=on

#define SPINDLE_ENABLE_POLARITY     1                       // 0=active low, 1=active high
#define SPINDLE_DIR_POLARITY        0                       // 0=clockwise is low, 1=clockwise is high
#define SPINDLE_PAUSE_ON_HOLD       true
#define SPINDLE_SPINUP_DELAY        1.0

#define COOLANT_MIST_POLARITY       1                       // 0=active low, 1=active high
#define COOLANT_FLOOD_POLARITY      1                       // 0=active low, 1=active high
#define COOLANT_PAUSE_ON_HOLD       false

#define TRAVERSE_AT_HIGH_JERK       true                    // EXPERIMENTAL!!

// Communications and reporting settings

#define MARLIN_COMPAT_ENABLED       true                    // enable marlin compatibility mode
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
//#define STATUS_REPORT_DEFAULTS    "line","posx","posy","posz","posa","vel","he1t","he1st","he1at","feed","unit","path","stat"
// There are no heater two or three, but these would show those: ,"he2t","he2st","he2at","he3t","he3st","he3at"

// Defaults for motion debugging
//#define STATUS_REPORT_DEFAULTS "line","posx","posy","posz","posa","he1t","he1st","he1at","he2t","he2st","he2at","he3t","he3st","he3at","_fe5","_fe4","feed","vel","unit","path","stat"

// Defaults for PID tuning
//#define STATUS_REPORT_DEFAULTS "line","posx","posy","posz","posa","he1t","he1st","he1at","he1op","pid1p","pid1i","pid1d","feed","vel","unit","path","stat"

// Defaults for Trinamic tuning
// #define STATUS_REPORT_DEFAULTS "line","posx","posy","posz","posa","he1t","he1st","he1at","he1op","he3t","he3st","he3at","he3op","feed","vel","unit","path","stat","1ts","1sgr","1csa","2ts","2sgr","2csa","3ts","3sgr","3csa","4ts","4sgr","4csa"

// Defaults for thermistor tuning: cut out: ,"he2t","he2st","he2at","he2tr","he2tv","he2op",
#define STATUS_REPORT_DEFAULTS "line","posx","posy","posz","posa","aso","he1t","he1st","he1at","he1tr","he1tv","he1op","he3t","he3st","he3at","he3tr","he3tv","he3op","feed","vel","unit","path","stat","_xs1","_xs2","_xs3","_xs4","_fe1","_fe2","_fe3","_fe4"
// Gcode startup defaults
#define GCODE_DEFAULT_UNITS         MILLIMETERS             // MILLIMETERS or INCHES
#define GCODE_DEFAULT_PLANE         CANON_PLANE_XY          // CANON_PLANE_XY, CANON_PLANE_XZ, or CANON_PLANE_YZ
#define GCODE_DEFAULT_COORD_SYSTEM  G54                     // G54, G55, G56, G57, G58 or G59
#define GCODE_DEFAULT_PATH_CONTROL  PATH_CONTINUOUS
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_DISTANCE_MODE


// *** motor settings ************************************************************************************

#define MOTOR_POWER_TIMEOUT         30 // don't disable motors (without an explicit {md:0}) for 30 seconds

#if 0

{jt:1.2}

{afr:1000}

{jt:1.0}

{afr:900}
{sr:{"line":t,"posx":t,"posy":t,"posz":t,"posa":t,"he1t":t,"he1st":t,"he1at":t,"he1op":t,"he3t":t,"he3st":t,"he3at":t,"he3op":t,"feed":t,"vel":t,"stat":t,"1ts":t,"1sgr":t,"1csa":t,"2ts":t,"2sgr":t,"2csa":t,"3ts":t,"3sgr":t,"3csa":t,"4ts":t,"4sgr":t,"4csa":t}}

{afr:2000}
{afr:6000}

#endif

#define MOTOR_POWER_MODE            MOTOR_POWERED_IN_CYCLE  // default motor power mode (see cmMotorPowerMode in stepper.h)
// 80 steps/mm at 1/16 microstepping = 40 mm/rev
#define M1_MOTOR_MAP                AXIS_X                  // 1ma
#define M1_STEP_ANGLE               1.8                     // 1sa
// Marlin says 80 steps/unit, and 16 microsteps, with a 200-step/rev motor
#define M1_TRAVEL_PER_REV           40                      // 1tr
#define M1_MICROSTEPS               128                     // 1mi        1,2,4,8,16,32
#define M1_POLARITY                 0                       // 1po        0=normal, 1=reversed
#define M1_POWER_MODE               MOTOR_POWERED_IN_CYCLE  // 1pm        standard
#define M1_POWER_LEVEL              0.65                    // 1pl
#define M1_TMC2130_TPWMTHRS         1200                    // 1pth
#define M1_TMC2130_TCOOLTHRS        1000                    // 1cth
#define M1_TMC2130_THIGH            10                      // 1hth
#define M1_TMC2130_SGT              4                       // 1sgt
#define M1_TMC2130_TBL              2                       // 1tbl
#define M1_TMC2130_PWM_GRAD         1                       // 1pgrd
#define M1_TMC2130_PWM_AMPL         200                     // 1pamp
#define M1_TMC2130_HEND             0                       // 1hend
#define M1_TMC2130_HSTRT            0                       // 1hsrt
#define M1_TMC2130_SMIN             5                       // 1smin
#define M1_TMC2130_SMAX             5                       // 1smax
#define M1_TMC2130_SUP              2                       // 1sup
#define M1_TMC2130_SDN              1                       // 1sdn

// 80 steps/mm at 1/16 microstepping = 40 mm/rev
#define M2_MOTOR_MAP                AXIS_Y
#define M2_STEP_ANGLE               1.8
// Marlin says 80 steps/unit, and 16 microsteps, with a 200-step/rev motor
#define M2_TRAVEL_PER_REV           40
#define M2_MICROSTEPS               128
#define M2_POLARITY                 1
#define M2_POWER_MODE               MOTOR_POWERED_IN_CYCLE
#define M2_POWER_LEVEL              0.65
#define M2_TMC2130_TPWMTHRS         1200
#define M2_TMC2130_TCOOLTHRS        1000
#define M2_TMC2130_THIGH            10
#define M2_TMC2130_SGT              4
#define M2_TMC2130_TBL              2
#define M2_TMC2130_PWM_GRAD         1
#define M2_TMC2130_PWM_AMPL         200
#define M2_TMC2130_HEND             0
#define M2_TMC2130_HSTRT            0
#define M2_TMC2130_SMIN             5
#define M2_TMC2130_SMAX             5
#define M2_TMC2130_SUP              2
#define M2_TMC2130_SDN              1

#define M3_MOTOR_MAP                AXIS_Z
#define M3_STEP_ANGLE               1.8
// Marlin says 200 steps/unit, and 8 microsteps, with a 200-step/rev motor
#define M3_TRAVEL_PER_REV           8
#define M3_MICROSTEPS               128
#define M3_POLARITY                 0
#define M3_POWER_MODE               MOTOR_POWERED_IN_CYCLE
#define M3_POWER_LEVEL              0.6
#define M3_TMC2130_TPWMTHRS         300
#define M3_TMC2130_TCOOLTHRS        200
#define M3_TMC2130_THIGH            10
#define M3_TMC2130_SGT              4
#define M3_TMC2130_TBL              2
#define M3_TMC2130_PWM_GRAD         1
#define M3_TMC2130_PWM_AMPL         200
#define M3_TMC2130_HEND             0
#define M3_TMC2130_HSTRT            0
#define M3_TMC2130_SMIN             5
#define M3_TMC2130_SMAX             12
#define M3_TMC2130_SUP              2
#define M3_TMC2130_SDN              2

#define M4_MOTOR_MAP                AXIS_A
#define M4_STEP_ANGLE               1.8
#define M4_TRAVEL_PER_REV           360            // degrees moved per motor rev
#define M4_MICROSTEPS               16
#define M4_POLARITY                 0
#define M4_POWER_MODE               MOTOR_POWER_MODE
#define M4_POWER_LEVEL              0.7
#define M4_TMC2130_TPWMTHRS         180000
#define M4_TMC2130_TCOOLTHRS        100000
#define M4_TMC2130_THIGH            10
#define M4_TMC2130_SGT              3
#define M4_TMC2130_TBL              2
#define M4_TMC2130_PWM_GRAD         15
#define M4_TMC2130_PWM_AMPL         255
#define M4_TMC2130_HEND             0
#define M4_TMC2130_HSTRT            0
#define M4_TMC2130_SMIN             5
#define M4_TMC2130_SMAX             10
#define M4_TMC2130_SUP              3
#define M4_TMC2130_SDN              0

#define M5_MOTOR_MAP                AXIS_B
#define M5_STEP_ANGLE               1.8
#define M5_TRAVEL_PER_REV           40
#define M5_MICROSTEPS               128
#define M5_POLARITY                 1
#define M5_POWER_MODE               MOTOR_DISABLED
#define M5_POWER_LEVEL              0.8
#define M5_TMC2130_TPWMTHRS         1200
#define M5_TMC2130_TCOOLTHRS        1000
#define M5_TMC2130_THIGH            10
#define M5_TMC2130_SGT              4
#define M5_TMC2130_TBL              2
#define M5_TMC2130_PWM_GRAD         1
#define M5_TMC2130_PWM_AMPL         200
#define M5_TMC2130_HEND             0
#define M5_TMC2130_HSTRT            0
#define M5_TMC2130_SMIN             5
#define M5_TMC2130_SMAX             12
#define M5_TMC2130_SUP              2
#define M5_TMC2130_SDN              1

// *** axis settings **********************************************************************************

#define X_AXIS_MODE                 AXIS_STANDARD           // xam  see canonical_machine.h cmAxisMode for valid values
#define X_VELOCITY_MAX              15000                    // xvm  G0 max velocity in mm/min
#define X_FEEDRATE_MAX              X_VELOCITY_MAX          // xfr  G1 max feed rate in mm/min
#define X_TRAVEL_MIN                0                       // xtn  minimum travel - used by soft limits and homing
#define X_TRAVEL_MAX                230                     // xtm  travel between switches or crashes
#define X_JERK_MAX                  8000                    // xjm  yes, that's "100 billion" mm/(min^3)
#define X_JERK_HIGH_SPEED           8000                    // xjh
#define X_HOMING_INPUT              1                       // xhi  input used for homing or 0 to disable
#define X_HOMING_DIRECTION          0                       // xhd  0=search moves negative, 1= search moves positive
#define X_SEARCH_VELOCITY           2500                    // xsv  move in negative direction
#define X_LATCH_VELOCITY            200                     // xlv  mm/min
#define X_LATCH_BACKOFF             10                       // xlb  mm
#define X_ZERO_BACKOFF              0.5                     // xzb  mm
//{xjm:5000}
//{yjm:5000}

#define Y_AXIS_MODE                 AXIS_STANDARD
#define Y_VELOCITY_MAX              15000
#define Y_FEEDRATE_MAX              Y_VELOCITY_MAX
#define Y_TRAVEL_MIN                0
#define Y_TRAVEL_MAX                224.5
#define Y_JERK_MAX                  8000
#define Y_JERK_HIGH_SPEED           8000
#define Y_HOMING_INPUT              3
#define Y_HOMING_DIRECTION          1
#define Y_SEARCH_VELOCITY           3000
#define Y_LATCH_VELOCITY            200
#define Y_LATCH_BACKOFF             10
#define Y_ZERO_BACKOFF              0.5

#define Z_AXIS_MODE                 AXIS_STANDARD
#define Z_VELOCITY_MAX              1500
#define Z_FEEDRATE_MAX              1000
#define Z_TRAVEL_MIN                0
#define Z_TRAVEL_MAX                215
#define Z_JERK_MAX                  500
//#define Z_JERK_MAX                  800
#define Z_JERK_HIGH_SPEED           1000
#define Z_HOMING_INPUT              6
#define Z_HOMING_DIRECTION          1
#define Z_SEARCH_VELOCITY           1000
#define Z_LATCH_VELOCITY            100
#define Z_LATCH_BACKOFF             10
#define Z_ZERO_BACKOFF              0

#define G55_Z_OFFSET                0.35 // higher number is farther away from the bed
// {g55z:0.35}
//#define G55_Z_OFFSET                0.25 // higher number is farther away from the bed
// {g55z:0.25}

// Rotary values are chosen to make the motor react the same as X for testing
/***************************************************************************************
 * To calculate the speeds here, in Wolfram Alpha-speak:
 *
 *   c=2*pi*r, r=1.428, d=c/360, s=((S*60)/d), S=40 for s
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
#define A_RADIUS                1.428
#define A_VELOCITY_MAX          288886.4  // {avm:288886.4}
//#define A_VELOCITY_MAX          144443.0  // G0 rate ~60 mm/s, 3,600 mm/min
//#define A_VELOCITY_MAX          72221.5  // G0 rate ~30 mm/s, 3,600 mm/min
//NYLON
//#define A_VELOCITY_MAX           30000.0 // G0 rate ~20 mm/s {avm:60000.0}

//#define A_FEEDRATE_MAX          48147.7 // ~20 mm/s
//#define A_FEEDRATE_MAX          36110.8 // ~15 mm/s
//#define A_FEEDRATE_MAX          24073.9 // ~10 mm/s
#define A_FEEDRATE_MAX          12036.95 // ~5 mm/s
//#define A_FEEDRATE_MAX          6018.475 // ~2.5 mm/s
//#define A_FEEDRATE_MAX          3000.0 // ~0.415 mm/s {afr:2000}
// NYLON
//#define A_FEEDRATE_MAX          800.0  // {afr:800}
//#define A_FEEDRATE_MAX          500.0 // ~0.2075 mm/s
#define A_TRAVEL_MIN            0
#define A_TRAVEL_MAX            10
//#define A_JERK_MAX              288886.4 // ~120 million mm/min^3
//#define A_JERK_MAX              144443.2 // ~60 million mm/min^3
//#define A_JERK_MAX              40000.0 // ~20 million mm/min^3 {ajm:48147.7}
#define A_JERK_MAX              2000.0 // ~20 million mm/min^3 {ajm:48147.7}
//NYLON
//#define A_JERK_MAX              25000.0 // ~20 million mm/min^3 {ajm:15000.0}
#define A_HOMING_INPUT          0
#define A_HOMING_DIRECTION      0
#define A_SEARCH_VELOCITY       2000
#define A_LATCH_VELOCITY        2000
#define A_LATCH_BACKOFF         5
#define A_ZERO_BACKOFF          2
//#define A_JERK_HIGH_SPEED       288886.4 // ~120 million mm/min^3
//#define A_JERK_HIGH_SPEED       240739.0 // ~100 million mm/min^3
//#define A_JERK_HIGH_SPEED       144443.2 // ~60 million mm/min^3
//#define A_JERK_HIGH_SPEED         120000.0 //
//#define A_JERK_HIGH_SPEED        95000.0 // ~40 million mm/min^3
#define A_JERK_HIGH_SPEED         2000.0
// NYLON
//#define A_JERK_HIGH_SPEED       35000.0 // ~30 million mm/min^3 {ajh:35000.0}


#define B_AXIS_MODE             AXIS_RADIUS
#define B_RADIUS                1.428
#define B_VELOCITY_MAX          144443.0  // G0 rate ~60 mm/s, 3,600 mm/min
#define B_FEEDRATE_MAX          96295.4 // ~40 mm/s
#define B_TRAVEL_MIN            0
#define B_TRAVEL_MAX            10
#define B_JERK_MAX              180554.0 // ~75 million mm/min^3
#define B_HOMING_INPUT          0
#define B_HOMING_DIRECTION      0
#define B_SEARCH_VELOCITY       2000
#define B_LATCH_VELOCITY        2000
#define B_LATCH_BACKOFF         5
#define B_ZERO_BACKOFF          2
#define B_JERK_HIGH_SPEED       361108.0 // ~150 million mm/min^3

/* NYLON
M100.1 ({{avm:90000.0}})
M100.1 ({{afr:1000}})
M100.1 ({{ajm:25000.0}})
M100.1 ({{ajh:144000.0}})

 {ajh:50000.0}
 {ajh:60000.0}

 {avm:90000.0}
 {afr:1000}
 {ajh:70000.0}
 {ajm:25000.0}
 {ajh:144000.0}
 */

//*** Input / output settings ***

//** Temperature Sensors **

#define HAS_TEMPERATURE_SENSOR_1  false
#if HAS_TEMPERATURE_SENSOR_1
//    #define TEMPERATURE_SENSOR_1_CIRCUIT_TYPE ADCCircuitDifferentialPullup
//    #define TEMPERATURE_SENSOR_1_CIRCUIT_INIT { /*pullup_resistance:*/ 200 }
//    #define TEMPERATURE_SENSOR_1_TYPE  PT100<ADCDifferentialPair<Motate::kADC1_Neg_PinNumber, kADC1_Pos_PinNumber>>
//    #define TEMPERATURE_SENSOR_1_INIT {&temperature_sensor_1_circuit}

    #define TEMPERATURE_SENSOR_1_CIRCUIT_TYPE ADCCircuitRawResistance
    #define TEMPERATURE_SENSOR_1_CIRCUIT_INIT { }
    #define TEMPERATURE_SENSOR_1_TYPE  PT100<MAX31865<SPIBus_used_t::SPIBusDevice>>
    #define TEMPERATURE_SENSOR_1_INIT {&temperature_sensor_1_circuit, spiBus, spiCSPinMux.getCS(5), /*pullup_resistance:*/ 430.0f}
#endif // HAS_TEMPERATURE_SENSOR_1

#define EXTRUDER_1_OUTPUT_PIN kHeaterOutput1_PinNumber
#define EXTRUDER_1_FAN_PIN    kOutput5_PinNumber

#define HAS_TEMPERATURE_SENSOR_2  false
#if HAS_TEMPERATURE_SENSOR_2
    #define TEMPERATURE_SENSOR_2_CIRCUIT_TYPE ADCCircuitDifferentialPullup
    #define TEMPERATURE_SENSOR_2_CIRCUIT_INIT { /*pullup_resistance:*/ 200 }
    #define TEMPERATURE_SENSOR_2_TYPE  PT100<ADCDifferentialPair<Motate::kADC2_Neg_PinNumber, kADC2_Pos_PinNumber>>
    #define TEMPERATURE_SENSOR_2_INIT {&temperature_sensor_2_circuit}

    // #define TEMPERATURE_SENSOR_2_CIRCUIT_TYPE ADCCircuitRawResistance
    // #define TEMPERATURE_SENSOR_2_CIRCUIT_INIT { /*pullup_resistance:*/ 430 }
    // #define TEMPERATURE_SENSOR_2_TYPE  PT100<MAX31865<SPIBus_used_t::SPIBusDevice>>
    // #define TEMPERATURE_SENSOR_2_INIT {/*pullup_resistance:*/ 430, /*inline_resistance*/0, spiBus, spiCSPinMux.getCS(5)}
#endif // HAS_TEMPERATURE_SENSOR_2

#define EXTRUDER_2_OUTPUT_PIN kHeaterOutput2_PinNumber

#define HAS_TEMPERATURE_SENSOR_3  false
#if HAS_TEMPERATURE_SENSOR_3
//    #define TEMPERATURE_SENSOR_3_CIRCUIT_TYPE ADCCircuitDifferentialPullup
//    #define TEMPERATURE_SENSOR_3_CIRCUIT_INIT { /*pullup_resistance:*/ 200 }
//    #define TEMPERATURE_SENSOR_3_TYPE  PT100<ADCDifferentialPair<Motate::kADC3_Neg_PinNumber, kADC3_Pos_PinNumber>>
//    #define TEMPERATURE_SENSOR_3_INIT {&temperature_sensor_3_circuit}

    #define TEMPERATURE_SENSOR_3_CIRCUIT_TYPE ADCCircuitRawResistance
    #define TEMPERATURE_SENSOR_3_CIRCUIT_INIT { }
    #define TEMPERATURE_SENSOR_3_TYPE  PT100<MAX31865<SPIBus_used_t::SPIBusDevice>>
    #define TEMPERATURE_SENSOR_3_INIT {&temperature_sensor_3_circuit, spiBus, spiCSPinMux.getCS(6), /*pullup_resistance:*/ 430.0f}
#endif // HAS_TEMPERATURE_SENSOR_3

#define BED_OUTPUT_PIN kHeaterOutput11_PinNumber

//** Digital Inputs **
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
#define DI1_MODE                    IO_ACTIVE_LOW
#define DI1_ACTION                  INPUT_ACTION_NONE
#define DI1_FUNCTION                INPUT_FUNCTION_NONE

// Xmax
#define DI2_MODE                    IO_MODE_DISABLED
#define DI2_ACTION                  INPUT_ACTION_NONE
#define DI2_FUNCTION                INPUT_FUNCTION_NONE

// Ymin
#define DI3_MODE                    IO_ACTIVE_LOW
#define DI3_ACTION                  INPUT_ACTION_NONE
#define DI3_FUNCTION                INPUT_FUNCTION_NONE

// Ymax
#define DI4_MODE                    IO_MODE_DISABLED
#define DI4_ACTION                  INPUT_ACTION_NONE
#define DI4_FUNCTION                INPUT_FUNCTION_NONE

// Zmin
#define DI5_MODE                    IO_MODE_DISABLED
#define DI5_ACTION                  INPUT_ACTION_NONE
#define DI5_FUNCTION                INPUT_FUNCTION_NONE

// Zmax
#define DI6_MODE                    IO_ACTIVE_LOW
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
#define DO1_MODE                    IO_ACTIVE_HIGH // unavailable, is the extruder output

//Extruder2_PWM
#define DO2_MODE                    IO_ACTIVE_HIGH // unavailable, is the extruder output

//Fan1A_PWM
#define DO3_MODE                    IO_ACTIVE_LOW

//Fan1B_PWM
#define DO4_MODE                    IO_ACTIVE_HIGH

#define DO5_MODE                    IO_ACTIVE_HIGH
#define DO6_MODE                    IO_ACTIVE_HIGH
#define DO7_MODE                    IO_ACTIVE_HIGH
#define DO8_MODE                    IO_ACTIVE_HIGH  // 5V Fan

//SAFEin (Output) signal
#define DO9_MODE                    IO_ACTIVE_HIGH

#define DO10_MODE                   IO_ACTIVE_HIGH

//Header Bed FET
#define DO11_MODE                   IO_ACTIVE_LOW // unavailable, is the extruder output

//Indicator_LED
#define DO12_MODE                   IO_ACTIVE_HIGH

#define DO13_MODE                   IO_ACTIVE_HIGH


/*** Extruders / Heaters ***/

#define MIN_FAN_VALUE               0.4   // (he1fm) at MIN_FAN_TEMP the fan comes on at this spped (0.0-1.0)
#define MAX_FAN_VALUE               1.0  // (he1fp) at MAX_FAN_TEMP the fan is at this spped (0.0-1.0)
#define MIN_FAN_TEMP                50.0  // (he1fl) at this temp the fan starts to ramp up linearly
#define MAX_FAN_TEMP                100.0 // (he1fh) at this temperature the fan is at "full speed" (MAX_FAN_VALUE)

// PID debug string: {sr:{"he1t":t,"he1st":t,"pid1p":t, "pid1i":t, "pid1d":t, "pid1f":t, "he1op":t, "line":t, "stat":t}}

#define H1_DEFAULT_ENABLE           true
#define H1_DEFAULT_P                5
#define H1_DEFAULT_I                0.01
#define H1_DEFAULT_D                500
#define H1_DEFAULT_F                0.0015
#if 0

{he1p:1}
{he1i:0.005}
{he1d:500}
{he1f:0.0015}

{he1fp:0.25}
{out4:0.7}

{he1fp:0.0}

{he1fp:1.0}
{out4:1.0}

#endif

#define H2_DEFAULT_ENABLE           false
#define H2_DEFAULT_P                7.0
#define H2_DEFAULT_I                0.05
#define H2_DEFAULT_D                150.0
#define H2_DEFAULT_F                0.0

#define H3_DEFAULT_ENABLE           true
#define H3_DEFAULT_P                20.0
#define H3_DEFAULT_I                0.05
#define H3_DEFAULT_D                50
#define H3_DEFAULT_F                0.0015

#define TEMP_MIN_BED_RISE_DEGREES_OVER_TIME 0.1
