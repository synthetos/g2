/*
 * settings_probotix.h - Probotix Fireball V90 machine profile
 * This file is part the TinyG project
 *
 * Copyright (c) 2011 - 2015 Alden S. Hart, Jr.
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
 * 		 into a virgin EEPROM, and can be changed using the config commands.
 *		 After initial load the EEPROM values (or changed values) are used.
 *
 *		 System and hardware settings that you shouldn't need to change
 *		 are in hardware.h  Application settings that also shouldn't need
 *		 to be changed are in tinyg.h
 */

/***********************************************************************/
/**** Probotix Fireball V90 profile ************************************/
/***********************************************************************/

// ***> NOTE: The init message must be a single line with no CRs or LFs
#define INIT_MESSAGE "Initializing configs to Probotix Fireball V90 profile"


//**** GLOBAL / GENERAL SETTINGS ******************************************************

#define JUNCTION_ACCELERATION       1000000					// centripetal acceleration around corners
#define CHORDAL_TOLERANCE           0.01					// chordal accuracy for arc drawing (in mm)

#define SOFT_LIMIT_ENABLE           0						// 0=off, 1=on
#define HARD_LIMIT_ENABLE           1						// 0=off, 1=on
#define SAFETY_INTERLOCK_ENABLE     1						// 0=off, 1=on

#define SPINDLE_ENABLE_POLARITY     1                       // 0=active low, 1=active high
#define SPINDLE_DIR_POLARITY        0                       // 0=clockwise is low, 1=clockwise is high
#define SPINDLE_PAUSE_ON_HOLD       true
#define SPINDLE_DWELL_TIME          1.0

#define COOLANT_MIST_POLARITY       1                       // 0=active low, 1=active high
#define COOLANT_FLOOD_POLARITY      1                       // 0=active low, 1=active high
#define COOLANT_PAUSE_ON_HOLD       true

// Communications and reporting settings

#define TEXT_VERBOSITY              TV_VERBOSE              // one of: TV_SILENT, TV_VERBOSE
#define COMM_MODE                   JSON_MODE               // one of: TEXT_MODE, JSON_MODE

#define COM_EXPAND_CR               false
#define COM_ENABLE_ECHO             false
#define COM_ENABLE_FLOW_CONTROL     FLOW_CONTROL_XON        // FLOW_CONTROL_OFF, FLOW_CONTROL_XON, FLOW_CONTROL_RTS

#define JSON_VERBOSITY              JV_MESSAGES             // one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE
#define JSON_SYNTAX_MODE            JSON_SYNTAX_STRICT      // one of JSON_SYNTAX_RELAXED, JSON_SYNTAX_STRICT

#define QUEUE_REPORT_VERBOSITY      QR_OFF                  // one of: QR_OFF, QR_SINGLE, QR_TRIPLE

#define STATUS_REPORT_VERBOSITY     SR_FILTERED             // one of: SR_OFF, SR_FILTERED, SR_VERBOSE
#define STATUS_REPORT_MIN_MS        100                     // milliseconds - enforces a viable minimum
#define STATUS_REPORT_INTERVAL_MS   250                     // milliseconds - set $SV=0 to disable
#define STATUS_REPORT_DEFAULTS "posx","posy","posz","posa"\
,"line","vel","feed","stat",\
"macs","cycs","mots","hold","dist","admo"
//,"in1","in2","in3","in4","in5","in6","in7","in8","in9"
//,"home","homx","homy","homz"

// Gcode startup defaults
#define GCODE_DEFAULT_UNITS         MILLIMETERS             // MILLIMETERS or INCHES
#define GCODE_DEFAULT_PLANE         CANON_PLANE_XY          // CANON_PLANE_XY, CANON_PLANE_XZ, or CANON_PLANE_YZ
#define GCODE_DEFAULT_COORD_SYSTEM  G54                     // G54, G55, G56, G57, G58 or G59
#define GCODE_DEFAULT_PATH_CONTROL  PATH_CONTINUOUS
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_MODE

// *** motor settings ************************************************************************************

#define MOTOR_POWER_MODE            MOTOR_POWERED_IN_CYCLE  // default motor power mode (see cmMotorPowerMode in stepper.h)
#define MOTOR_POWER_TIMEOUT         2.00                    // motor power timeout in seconds
#define MOTOR_POWER_LEVEL           0.375                   // default motor power level 0.00 - 1.00 (ARM only)

#define M1_MOTOR_MAP 			    AXIS_X					// 1ma
#define M1_STEP_ANGLE 		    	1.8						// 1sa
#define M1_TRAVEL_PER_REV	    	5.08					// 1tr
#define M1_MICROSTEPS		    	8						// 1mi		1,2,4,8
#define M1_POLARITY			    	1						// 1po		0=normal, 1=reversed
#define M1_POWER_MODE		    	MOTOR_POWER_MODE		// 1pm		standard
#define M1_POWER_LEVEL		    	0.75		            // 1pl

#define M2_MOTOR_MAP	 	    	AXIS_Y
#define M2_STEP_ANGLE		    	1.8
#define M2_TRAVEL_PER_REV	    	5.08
#define M2_MICROSTEPS		    	8
#define M2_POLARITY			    	0
#define M2_POWER_MODE		    	MOTOR_POWER_MODE
#define M2_POWER_LEVEL		    	0.75

#define M3_MOTOR_MAP		    	AXIS_Z
#define M3_STEP_ANGLE		    	1.8
#define M3_TRAVEL_PER_REV	    	2.1166666
#define M3_MICROSTEPS		    	8
#define M3_POLARITY			    	1
#define M3_POWER_MODE		    	MOTOR_POWER_MODE
#define M3_POWER_LEVEL		    	0.50

#define M4_MOTOR_MAP		    	AXIS_A
#define M4_STEP_ANGLE		    	1.8
#define M4_TRAVEL_PER_REV	    	360			// degrees moved per motor rev
#define M4_MICROSTEPS		    	8
#define M4_POLARITY			    	0
#define M4_POWER_MODE		    	MOTOR_POWER_MODE
#define M4_POWER_LEVEL		    	MOTOR_POWER_LEVEL

#define M5_MOTOR_MAP		    	AXIS_B
#define M5_STEP_ANGLE		    	1.8
#define M5_TRAVEL_PER_REV	    	360			// degrees moved per motor rev
#define M5_MICROSTEPS		    	8
#define M5_POLARITY			    	0
#define M5_POWER_MODE		    	MOTOR_POWER_MODE
#define M5_POWER_LEVEL		    	MOTOR_POWER_LEVEL

#define M6_MOTOR_MAP		    	AXIS_C
#define M6_STEP_ANGLE		    	1.8
#define M6_TRAVEL_PER_REV	    	360			// degrees moved per motor rev
#define M6_MICROSTEPS		    	8
#define M6_POLARITY			    	0
#define M6_POWER_MODE		    	MOTOR_POWER_MODE
#define M6_POWER_LEVEL		    	MOTOR_POWER_LEVEL

// *** axis settings **********************************************************************************

#define JUNCTION_DEVIATION_XY       0.05                    // larger is faster
#define JUNCTION_DEVIATION_Z        0.01                    // larger is faster
#define JUNCTION_DEVIATION_ABC      0.1                     // larger is faster

#define X_AXIS_MODE			    	AXIS_STANDARD           // xam  see canonical_machine.h cmAxisMode for valid values
#define X_VELOCITY_MAX		    	1600                    // xvm  G0 max velocity in mm/min
#define X_FEEDRATE_MAX		    	X_VELOCITY_MAX          // xfr  G1 max feed rate in mm/min
#define X_TRAVEL_MIN		    	0                       // xtn  minimum travel - used by soft limits and homing
#define X_TRAVEL_MAX 		    	400                     // xtm  maximum travel - used by soft limits and homing
#define X_JERK_MAX			    	100			            // xjm
#define X_JERK_HIGH_SPEED	    	X_JERK_MAX              // xjh
#define X_JUNCTION_DEVIATION    	JUNCTION_DEVIATION_XY   // xjd
#define X_HOMING_INPUT              1                       // xhi  input used for homing or 0 to disable
#define X_HOMING_DIR                0                       // xhd  0=search moves negative, 1= search moves positive
#define X_SEARCH_VELOCITY	    	1000                    // xsv  move in negative direction
#define X_LATCH_VELOCITY	    	100                     // xlv  mm/min
#define X_LATCH_BACKOFF		    	10                      // xlb  mm
#define X_ZERO_BACKOFF		    	2                       // xzb  mm

#define Y_AXIS_MODE			    	AXIS_STANDARD
#define Y_VELOCITY_MAX		    	1600
#define Y_FEEDRATE_MAX		        Y_VELOCITY_MAX
#define Y_TRAVEL_MIN			    0
#define Y_TRAVEL_MAX		    	175
#define Y_JERK_MAX			    	100
#define Y_JERK_HIGH_SPEED       	Y_JERK_MAX
#define Y_JUNCTION_DEVIATION    	JUNCTION_DEVIATION_XY
#define Y_HOMING_INPUT              3
#define Y_HOMING_DIR                0
#define Y_SEARCH_VELOCITY	    	1000
#define Y_LATCH_VELOCITY	    	100
#define Y_LATCH_BACKOFF		    	10
#define Y_ZERO_BACKOFF		    	2

#define Z_AXIS_MODE			    	AXIS_STANDARD
#define Z_VELOCITY_MAX		    	1200
#define Z_FEEDRATE_MAX		    	Z_VELOCITY_MAX
#define Z_TRAVEL_MIN		    	0
#define Z_TRAVEL_MAX		    	75
#define Z_JERK_MAX			    	100
#define Z_JERK_HIGH_SPEED       	Z_JERK_MAX
#define Z_JUNCTION_DEVIATION    	JUNCTION_DEVIATION_Z  // smaller deviation as Z has more threads
#define Z_HOMING_INPUT              6
#define Z_HOMING_DIR                1
#define Z_SEARCH_VELOCITY	    	600
#define Z_LATCH_VELOCITY	    	100
#define Z_LATCH_BACKOFF		    	10
#define Z_ZERO_BACKOFF		    	2

// Rotary values are chosen to make the motor react the same as X for testing
#define A_AXIS_MODE 		    	AXIS_RADIUS
#define A_VELOCITY_MAX 		    	((X_VELOCITY_MAX/M1_TRAVEL_PER_REV)*360) // set to the same speed as X axis
#define A_FEEDRATE_MAX 		    	A_VELOCITY_MAX
#define A_TRAVEL_MIN		    	-1										// min/max the same means infinite, no limit
#define A_TRAVEL_MAX 		    	-1
#define A_JERK_MAX 			    	(X_JERK_MAX*(360/M1_TRAVEL_PER_REV))
#define A_JERK_HIGH_SPEED       	A_JERK_MAX
#define A_JUNCTION_DEVIATION    	JUNCTION_DEVIATION_ABC
#define A_RADIUS 			    	(M1_TRAVEL_PER_REV/(2*3.14159628))
#define A_HOMING_INPUT              0
#define A_HOMING_DIR                0
#define A_SEARCH_VELOCITY 	    	600
#define A_LATCH_VELOCITY 	    	100
#define A_LATCH_BACKOFF 	    	5
#define A_ZERO_BACKOFF 		    	2

#define B_AXIS_MODE 		    	AXIS_RADIUS
#define B_VELOCITY_MAX 		    	((X_VELOCITY_MAX/M1_TRAVEL_PER_REV)*360)
#define B_FEEDRATE_MAX 		    	B_VELOCITY_MAX
#define B_TRAVEL_MIN		    	-1
#define B_TRAVEL_MAX 		    	-1
#define B_JERK_MAX 			    	(X_JERK_MAX*(360/M1_TRAVEL_PER_REV))
#define B_JERK_HIGH_SPEED       	B_JERK_MAX
#define B_JUNCTION_DEVIATION    	JUNCTION_DEVIATION_ABC
#define B_RADIUS 			    	(M1_TRAVEL_PER_REV/(2*3.14159628))
#define B_HOMING_INPUT              0
#define B_HOMING_DIR                0
#define B_SEARCH_VELOCITY 	    	600
#define B_LATCH_VELOCITY 	    	100
#define B_LATCH_BACKOFF 	    	5
#define B_ZERO_BACKOFF 		    	2

#define C_AXIS_MODE 		    	AXIS_RADIUS
#define C_VELOCITY_MAX 		    	((X_VELOCITY_MAX/M1_TRAVEL_PER_REV)*360)
#define C_FEEDRATE_MAX 		    	C_VELOCITY_MAX
#define C_TRAVEL_MIN		    	-1
#define C_TRAVEL_MAX 		    	-1
#define C_JERK_MAX 			    	(X_JERK_MAX*(360/M1_TRAVEL_PER_REV))
#define C_JERK_HIGH_SPEED       	C_JERK_MAX
#define C_JUNCTION_DEVIATION    	JUNCTION_DEVIATION_ABC
#define C_RADIUS			    	(M1_TRAVEL_PER_REV/(2*3.14159628))
#define C_HOMING_INPUT              0
#define C_HOMING_DIR                0
#define C_SEARCH_VELOCITY 	    	600
#define C_LATCH_VELOCITY 	    	100
#define C_LATCH_BACKOFF 	    	5
#define C_ZERO_BACKOFF 		    	2

//*** Input / output settings ***

/* 
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

// Xmin on v9 board
#define DI1_MODE                    INPUT_MODE_DISABLED
#define DI1_ACTION                  INPUT_ACTION_NONE
#define DI1_FUNCTION                INPUT_FUNCTION_NONE

// Xmax
#define DI2_MODE                    INPUT_MODE_DISABLED
#define DI2_ACTION                  INPUT_ACTION_NONE
#define DI2_FUNCTION                INPUT_FUNCTION_NONE

// Ymin
#define DI3_MODE                    INPUT_MODE_DISABLED
#define DI3_ACTION                  INPUT_ACTION_NONE
#define DI3_FUNCTION                INPUT_FUNCTION_NONE

// Ymax
#define DI4_MODE                    INPUT_MODE_DISABLED
#define DI4_ACTION                  INPUT_ACTION_NONE
#define DI4_FUNCTION                INPUT_FUNCTION_NONE

// Zmin
#define DI5_MODE                    INPUT_ACTIVE_LOW
#define DI5_ACTION                  INPUT_ACTION_NONE
#define DI5_FUNCTION                INPUT_FUNCTION_NONE

// Zmax
#define DI6_MODE                    INPUT_MODE_DISABLED
#define DI6_ACTION                  INPUT_ACTION_NONE
#define DI6_FUNCTION                INPUT_FUNCTION_NONE

// Amin
#define DI7_MODE                    INPUT_MODE_DISABLED
#define DI7_ACTION                  INPUT_ACTION_NONE
#define DI7_FUNCTION                INPUT_FUNCTION_NONE

// Amax
#define DI8_MODE                    INPUT_MODE_DISABLED
#define DI8_ACTION                  INPUT_ACTION_NONE
#define DI8_FUNCTION                INPUT_FUNCTION_NONE

// Safety line
#define DI9_MODE                    INPUT_MODE_DISABLED
#define DI9_ACTION                  INPUT_ACTION_NONE
#define DI9_FUNCTION                INPUT_FUNCTION_NONE


/*** Handle optional modules that may not be in every machine ***/

#define P1_PWM_FREQUENCY            100                     // in Hz
#define P1_CW_SPEED_LO              7900                    // in RPM (arbitrary units)
#define P1_CW_SPEED_HI              12800
#define P1_CW_PHASE_LO              0.13                   // phase [0..1]
#define P1_CW_PHASE_HI              0.17
#define P1_CCW_SPEED_LO             0
#define P1_CCW_SPEED_HI             0
#define P1_CCW_PHASE_LO             0.1
#define P1_CCW_PHASE_HI             0.1
#define P1_PWM_PHASE_OFF            0.1

// *** DEFAULT COORDINATE SYSTEM OFFSETS ***

#define G54_X_OFFSET 0			// G54 is traditionally set to all zeros
#define G54_Y_OFFSET 0
#define G54_Z_OFFSET 0
#define G54_A_OFFSET 0
#define G54_B_OFFSET 0
#define G54_C_OFFSET 0

#define G55_X_OFFSET 0	// use (X_TRAVEL_MAX/2) to set g55 to middle of table
#define G55_Y_OFFSET 0  // use (Y_TRAVEL_MAX/2) to set g55 to middle of table
#define G55_Z_OFFSET 0
#define G55_A_OFFSET 0
#define G55_B_OFFSET 0
#define G55_C_OFFSET 0

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

