/*
 * settings_pocketnc.h - machine profile for Pocket NC 5 axis tabletop machining center
 * This file is part of the TinyG2 project
 *
 * Copyright (c) 2012 - 2014 Alden S. Hart, Jr.
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
/* Note: The values in this file are the default settings that are loaded on power-up 
 *		 and can be changed using the config commands.
 */
/***********************************************************************/
/**** Profile for PocketNC 5 axis machining center *********************/
/***********************************************************************/
// NOTE: Non-machine specific systems settings can be found in settings.h
// These may be overridden using undefs

// REVISED 3/19/14 - ash

/**** PocketNC Axis COnfiguration ****

	X axis is top-side table carrying the Z axis and spindle. Positive direction moves slide to rear of machine
	Y axis is arbor lifting rotary axes. Positive direction moves rotary assembly downwards
	Z axis moves spindle. Positive is away from rotary table (away from the work)
	A axis is rotary table positioning B axis. Positive is counter-clockwise movement. 0 degrees is defined as B table in vertical position
	B axis is rotary table normal to Z axis. Positive is counter-clockwise movement. 0 degrees is defined relative to work.
*/

/***** Manual Homing and Default Coordinate System ****

	G54 the default coordinate system. G54 is set so that a G0 X0 Y0 Z0 A0 B0 will center the machine from a manually homed position

	To manually home the machine perform the following in sequence:
	 - Move Z axis to maximum positive (furthest away from B table)
	 - Move X axis to maximum positive (rear of machine)
	 - Move Y axis to maximum positive (bottom of arbor travel)
	 - Position A axis to facing upwards and level, parallel to plane of table top (this can be moved manually)
	 - Position B axis to correct starting position for the work piece mounted
	 
	This Gcode sequence will find the positive limits and then home and center the machine.
		G0 Z250							(retract Z first to avoid tool or machine damage)
		G0 X180 Y200					(move A out of the way first)
		G0 A200							(it's OK if the axes run to the limits of their travel and stall)
		G28.3 X58.9 Y63.65 Z76.2 A90	(align with G54 coordinate offsets)
		G0 X0 Y0 A0						(center machine, except for Z which depends on tool length and work center)

G0 Z250
G0 X180 Y200
G0 A200
G28.3 X58.9 Y63.65 Z76.2 A90
G0 X0 Y0 A0
*/
/* Note to self: Motor plugging is

	Motor1 == 5
	Motor2 == 2
	Motor3 == 3
	Motor4 == 4
	Motor5 == 1	
*/


// ***> NOTE: The init message must be a single line with no CRs or LFs 
#define INIT_MESSAGE "Initializing configs to PocketNC settings"

#define JERK_MAX 				500					// that's 500 million mm/(min^3)
#define JUNCTION_DEVIATION		0.05				// default value, in mm
#define JUNCTION_ACCELERATION	100000				// centripetal acceleration around corners

// **** settings.h overrides ****

#undef	SWITCH_TYPE
//#define SWITCH_TYPE 			SW_TYPE_NORMALLY_CLOSED	// one of: SW_TYPE_NORMALLY_OPEN, SW_TYPE_NORMALLY_CLOSED
#define SWITCH_TYPE 			SW_TYPE_NORMALLY_OPEN	// one of: SW_TYPE_NORMALLY_OPEN, SW_TYPE_NORMALLY_CLOSED

#undef	MOTOR_POWER_LEVEL
#define MOTOR_POWER_LEVEL		0.25				// default motor power level 0.00 - 1.00 (ARM only)

#undef STATUS_REPORT_DEFAULTS
#define STATUS_REPORT_DEFAULTS "line","posx","posy","posz","posa","posb","feed","vel","unit","coor","dist","frmo","momo","stat"

#ifndef PI
#define PI 3.14159628
#endif

// *** motor settings ***

#define M1_MOTOR_MAP 			AXIS_X					// 1ma
#define M1_STEP_ANGLE 			1.8						// 1sa
#define M1_TRAVEL_PER_REV		2.438					// 1tr
#define M1_MICROSTEPS			4						// 1mi		1,2,4,8
#define M1_POLARITY				1						// 1po		0=normal, 1=reversed
#define M1_POWER_MODE			MOTOR_ALWAYS_POWERED	// 1pm		standard
#define M1_POWER_LEVEL			MOTOR_POWER_LEVEL		// 1mp

#define M2_MOTOR_MAP	 		AXIS_Y
#define M2_STEP_ANGLE			1.8
#define M2_TRAVEL_PER_REV		2.438
#define M2_MICROSTEPS			4
#define M2_POLARITY				0
#define M2_POWER_MODE			MOTOR_ALWAYS_POWERED
#define M2_POWER_LEVEL			MOTOR_POWER_LEVEL

#define M3_MOTOR_MAP			AXIS_Z
#define M3_STEP_ANGLE			1.8
#define M3_TRAVEL_PER_REV		2.54
#define M3_MICROSTEPS			4
#define M3_POLARITY				1
#define M3_POWER_MODE			MOTOR_ALWAYS_POWERED
#define M3_POWER_LEVEL			MOTOR_POWER_LEVEL

#define M4_MOTOR_MAP			AXIS_A
#define M4_STEP_ANGLE			1.8
#define M4_TRAVEL_PER_REV		45			// degrees moved per motor rev
#define M4_MICROSTEPS			8
#define M4_POLARITY				0
#define M4_POWER_MODE			MOTOR_ALWAYS_POWERED
#define M4_POWER_LEVEL			MOTOR_POWER_LEVEL

#define M5_MOTOR_MAP			AXIS_B
#define M5_STEP_ANGLE			1.8
#define M5_TRAVEL_PER_REV		45			// degrees moved per motor rev
#define M5_MICROSTEPS			8
#define M5_POLARITY				1
#define M5_POWER_MODE			MOTOR_ALWAYS_POWERED
#define M5_POWER_LEVEL			MOTOR_POWER_LEVEL

#define M6_MOTOR_MAP			AXIS_C
#define M6_STEP_ANGLE			1.8
#define M6_TRAVEL_PER_REV		360			// degrees moved per motor rev
#define M6_MICROSTEPS			8
#define M6_POLARITY				0
#define M6_POWER_MODE			MOTOR_DISABLED
#define M6_POWER_LEVEL			MOTOR_POWER_LEVEL

// *** axis settings ***

#define X_AXIS_MODE 			AXIS_STANDARD		// xam		see canonical_machine.h cmAxisMode for valid values
#define X_VELOCITY_MAX 			1800 				// xvm		G0 max velocity in mm/min
#define X_FEEDRATE_MAX 			1800				// xfr 		G1 max feed rate in mm/min
#define X_TRAVEL_MIN			0					// xtn
#define X_TRAVEL_MAX 			150					// xtm		travel between switches or crashes
#define X_JERK_MAX 				JERK_MAX			// xjm
#define X_JUNCTION_DEVIATION 	JUNCTION_DEVIATION	// xjd
#define X_SWITCH_MODE_MIN 		SW_MODE_DISABLED	// xsn		SW_MODE_DISABLED, SW_MODE_HOMING, SW_MODE_LIMIT, SW_MODE_HOMING_LIMIT
#define X_SWITCH_MODE_MAX 		SW_MODE_HOMING		// xsx		SW_MODE_DISABLED, SW_MODE_HOMING, SW_MODE_LIMIT, SW_MODE_HOMING_LIMIT
#define X_SEARCH_VELOCITY 		500					// xsv		move in negative direction
#define X_LATCH_VELOCITY 		100					// xlv		mm/min
#define X_LATCH_BACKOFF 		2					// xlb		mm
#define X_ZERO_BACKOFF 			1					// xzb		mm
#define X_JERK_HOMING			X_JERK_MAX			// xjh

#define Y_AXIS_MODE 			AXIS_STANDARD
#define Y_VELOCITY_MAX 			1000				// was 600
#define Y_FEEDRATE_MAX 			1000				// was 600
#define Y_TRAVEL_MIN			0
#define Y_TRAVEL_MAX 			40
#define Y_JERK_MAX 				JERK_MAX			// was 10
#define Y_JUNCTION_DEVIATION 	JUNCTION_DEVIATION
#define Y_SWITCH_MODE_MIN 		SW_MODE_DISABLED
#define Y_SWITCH_MODE_MAX 		SW_MODE_HOMING
#define Y_SEARCH_VELOCITY 		500
#define Y_LATCH_VELOCITY 		100
#define Y_LATCH_BACKOFF 		2
#define Y_ZERO_BACKOFF 			1
#define Y_JERK_HOMING			Y_JERK_MAX

#define Z_AXIS_MODE 			AXIS_STANDARD
#define Z_VELOCITY_MAX 			1800
#define Z_FEEDRATE_MAX 			1800
#define Z_TRAVEL_MIN			0
#define Z_TRAVEL_MAX 			96
#define Z_JERK_MAX 				JERK_MAX
#define Z_JUNCTION_DEVIATION 	JUNCTION_DEVIATION
#define Z_SWITCH_MODE_MIN 		SW_MODE_DISABLED
#define Z_SWITCH_MODE_MAX 		SW_MODE_HOMING
#define Z_SEARCH_VELOCITY 		400
#define Z_LATCH_VELOCITY 		100
#define Z_LATCH_BACKOFF 		2
#define Z_ZERO_BACKOFF 			1
#define Z_JERK_HOMING			Z_JERK_MAX

#define A_AXIS_MODE 			AXIS_STANDARD
#define A_VELOCITY_MAX 			7200 // set to the same speed as X axis
#define A_FEEDRATE_MAX 			3600
#define A_TRAVEL_MIN 			0
#define A_TRAVEL_MAX 			180
#define A_JERK_MAX 				JERK_MAX
#define A_JUNCTION_DEVIATION	JUNCTION_DEVIATION
#define A_RADIUS 				(M5_TRAVEL_PER_REV/(2*PI))	// Radius to make the A/B/C motors react the same as X for testing
#define A_SWITCH_MODE_MIN 		SW_MODE_DISABLED				// ...need to select Radius Mode for the axis for this to happen
#define A_SWITCH_MODE_MAX 		SW_MODE_HOMING
#define A_SEARCH_VELOCITY 		600
#define A_LATCH_VELOCITY 		100
#define A_LATCH_BACKOFF 		5
#define A_ZERO_BACKOFF 			2
#define A_JERK_HOMING			A_JERK_MAX

#define B_AXIS_MODE 			AXIS_STANDARD
#define B_VELOCITY_MAX 			7200 // set to the same speed as X axis
#define B_FEEDRATE_MAX 			3600
#define B_TRAVEL_MIN 			0
#define B_TRAVEL_MAX 			1000
#define B_JERK_MAX 				JERK_MAX
#define B_JUNCTION_DEVIATION	JUNCTION_DEVIATION
#define B_RADIUS 				(M5_TRAVEL_PER_REV/(2*PI))
#define B_SWITCH_MODE_MIN 		SW_MODE_DISABLED
#define B_SWITCH_MODE_MAX 		SW_MODE_HOMING
#define B_SEARCH_VELOCITY 		600
#define B_LATCH_VELOCITY 		100
#define B_LATCH_BACKOFF 		5
#define B_ZERO_BACKOFF 			2
#define B_JERK_HOMING			B_JERK_MAX

#define C_AXIS_MODE 			AXIS_DISABLED			// AXIS_RADIUS
#define C_VELOCITY_MAX 			((X_VELOCITY_MAX/M1_TRAVEL_PER_REV)*360) // set to the same speed as X axis
#define C_FEEDRATE_MAX 			C_VELOCITY_MAX
#define C_TRAVEL_MIN 			-1
#define C_TRAVEL_MAX 			-1
#define C_JERK_MAX 				(X_JERK_MAX*(360/M1_TRAVEL_PER_REV))
#define C_JUNCTION_DEVIATION	JUNCTION_DEVIATION
#define C_RADIUS 				(M5_TRAVEL_PER_REV/(2*PI))
#define C_SWITCH_MODE_MIN 		SW_MODE_HOMING
#define C_SWITCH_MODE_MAX 		SW_MODE_DISABLED
#define C_SEARCH_VELOCITY 		600
#define C_LATCH_VELOCITY 		100
#define C_LATCH_BACKOFF 		5
#define C_ZERO_BACKOFF 			2
#define C_JERK_HOMING			C_JERK_MAX

// *** DEFAULT COORDINATE SYSTEM OFFSETS ***

/* Default position (see homing notes)
*/
#define G54_X_OFFSET -58.9		// default position
#define G54_Y_OFFSET -63.65
#define G54_Z_OFFSET -76.2
#define G54_A_OFFSET -90.0		// was 96.4
#define G54_B_OFFSET 0
#define G54_C_OFFSET 0

#define G55_X_OFFSET (X_TRAVEL_MAX/2)	// set to middle of table
#define G55_Y_OFFSET (Y_TRAVEL_MAX/2)
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
