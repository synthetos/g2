/*
 * config_app.h - application-specific part of configuration data
 * Part of Kinen project
 *
 * Copyright (c) 2013 Alden S. Hart Jr.
 * Copyright (c) 2013 Robert Giseburt
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
/* config_app.cpp/.h contain application specific data for the config system:
 *	-,application-specific functions and function prototypes 
 *	- application-specific message and print format strings
 *	- application-specific config array
 *	- any other application-specific data
 */
/*
 * --- Config objects and the config list ---
 *
 *	The config system provides a structured way to access and set configuration variables
 *	and to invoke commands and functions from the command line and from JSON input.
 *	It also provides a way to get to an arbitrary variable for reading or display.
 *
 *	Config operates as a collection of "objects" (OK, so they are not really objects) 
 *	that encapsulate each variable. The objects are collected into a list (a body) 
 *	which may also have header and footer objects. 
 *
 *	This way the internals don't care about how the variable is represented or 
 *	communicated externally as all operations occur on the cmdObj list. The list is 
 *	populated by the text_parser or the JSON_parser depending on the mode. Lists 
 *	are also used for responses and are read out (printed) by a text-mode or JSON 
 *	print functions.
 */
/* --- Config variables, tables and strings ---
 *
 *	Each configuration value is identified by a short mnemonic string (token). The token 
 *	is resolved to an index into the cfgArray which is an program memory (PROGMEM) array 
 *	of structures with the static assignments for each variable. The array is organized as:
 * 
 *	  - group string identifying what group the variable is part of (if any)
 *	  - token string - the token for that variable - pre-pended with the group (if any)
 *	  - operations flags - flag if the value should be initialized, persisted, etc.
 *	  - pointer to a formatted print string also in program memory (Used only for text mode)
 *	  - function pointer for formatted print() method or text-mode readouts
 *	  - function pointer for get() method - gets values from memory
 *	  - function pointer for set() method - sets values and runs functions
 *	  - target - memory location that the value is written to / read from
 *	  - default value - for cold initialization
 *
 *	Persistence is provided by an NVM array containing values in EEPROM as floats; 
 *	indexed by cfgArray index
 *
 *	The following rules apply to mnemonic tokens
 *	 - are up to 5 alphnuneric characters and cannot contain whitespace or separators
 *	 - must be unique (non colliding).
 *
 *  "Groups" are collections of values that mimic REST composite resources. Groups include:
 *	 - a system group is identified by "sys" and contains a collection of otherwise unrelated values
 *
 *	"Uber-groups" are groups of groups that are only used for text-mode printing - e.g.
 *	 - group of all groups
 */
/* --- Making changes and adding new values
 *
 *	Adding a new value to config (or changing an existing one) involves touching the following places:
 *
 *	 - Add a formatting string to fmt_XXX strings. Not needed if there is no text-mode print function
 *	   of you are using one of the generic print format strings.
 * 
 *	 - Create a new record in cfgArray[]. Use existing ones for examples. You can usually use 
 *	   generic functions for get and set; or create a new one if you need a specialized function.
 *
 *	   The ordering of group displays is set by the order of items in cfgArray. None of the other 
 *	   orders matter but are generally kept sequenced for easier reading and code maintenance. Also,
 *	   Items earlier in the array will resolve token searches faster than ones later in the array.
 *
 *	   Note that matching will occur from the most specific to the least specific, meaning that
 *	   if tokens overlap the longer one should be earlier in the array: "gco" should precede "gc".
 */
/* --- Rules, guidelines and random stuff
 *
 *	It's the responsibility of the object creator to set the index in the cmdObj when a variable
 *	is "hydrated". Many downstream function expect a valid index int he cmdObj struct. Set the 
 *	index by calling cmd_get_index(). This also validates the token and group if no lookup exists.
 */

#ifndef _CONFIG_APP_H_
#define _CONFIG_APP_H_

#ifdef __cplusplus
extern "C"{
#endif

/***********************************************************************************
 **** APPLICATION_SPECIFIC CONFIG STRUCTURE(S) *************************************
 ***********************************************************************************/

typedef struct cfgAxisParameters {
	uint8_t axis_mode;				// see tgAxisMode in gcode.h
	float feedrate_max;				// max velocity in mm/min or deg/min
	float velocity_max;				// max velocity in mm/min or deg/min
	float travel_max;				// work envelope w/warned or rejected blocks
	float jerk_max;					// max jerk (Jm) in mm/min^3
	float junction_dev;				// aka cornering delta
	float radius;					// radius in mm for rotary axis modes
	float search_velocity;			// homing search velocity
	float latch_velocity;			// homing latch velocity
	float latch_backoff;			// backoff from switches prior to homing latch movement
	float zero_backoff;				// backoff from switches for machine zero
	float jerk_homing;				// homing jerk (Jh) in mm/min^3
} cfgAxis_t;

typedef struct cfgMotorParameters {
	uint8_t	motor_map;				// map motor to axis
	uint8_t microsteps;				// microsteps to apply for each axis (ex: 8)
	uint8_t polarity;				// 0=normal polarity, 1=reverse motor direction
	uint8_t power_mode;				// 1=lo power idle mode, 0=full power idle mode
	float step_angle;				// degrees per whole step (ex: 1.8)
	float travel_rev;				// mm or deg of travel per motor revolution
	float steps_per_unit;			// steps (usteps)/mm or deg of travel
} cfgMotor_t;

typedef struct cfgPWMParameters {
	float frequency;				// base frequency for PWM driver, in Hz
	float cw_speed_lo;				// minimum clockwise spindle speed [0..N]
	float cw_speed_hi;				// maximum clockwise spindle speed
	float cw_phase_lo;				// pwm phase at minimum CW spindle speed, clamped [0..1]
	float cw_phase_hi;				// pwm phase at maximum CW spindle speed, clamped [0..1]
	float ccw_speed_lo;				// minimum counter-clockwise spindle speed [0..N]
	float ccw_speed_hi;				// maximum counter-clockwise spindle speed
	float ccw_phase_lo;				// pwm phase at minimum CCW spindle speed, clamped [0..1]
	float ccw_phase_hi;				// pwm phase at maximum CCW spindle speed, clamped
	float phase_off;				// pwm phase when spindle is disabled
} cfgPWM_t;

typedef struct cfgParameters {
	uint16_t magic_start;			// magic number to test memory integity
	uint16_t nvm_base_addr;			// NVM base address
	uint16_t nvm_profile_base;		// NVM base address of current profile

	// hidden settings				// not part of system group, but still accessible
	float min_segment_len;			// line drawing resolution in mm
	float arc_segment_len;			// arc drawing resolution in mm
	float chordal_tolerance;		// arc chordal accuracy setting in mm
	float estd_segment_usec;		// approximate segment time in microseconds

	// system group settings
	float junction_acceleration;	// centripetal acceleration max for cornering
	// float max_spindle_speed;		// in RPM

	// gcode power-on default settings - defaults are not the same as the gm state
	uint8_t coord_system;			// G10 active coordinate system default
	uint8_t select_plane;			// G17,G18,G19 reset default
	uint8_t units_mode;				// G20,G21 reset default
	uint8_t path_control;			// G61,G61.1,G64 reset default
	uint8_t distance_mode;			// G90,G91 reset default

	// communications settings
	uint8_t comm_mode;				// TG_TEXT_MODE or TG_JSON_MODE
	uint8_t ignore_crlf;			// ignore CR or LF on RX --- these 4 are shadow settings for XIO cntrl bits
	uint8_t enable_cr;				// enable CR in CRFL expansion on TX
	uint8_t enable_echo;			// enable text-mode echo
	uint8_t enable_xon;				// enable XON/XOFF mode

	uint8_t queue_report_verbosity;	// queue reports enabled and verbosity level
	uint8_t queue_report_hi_water;
	uint8_t queue_report_lo_water;
	uint8_t json_verbosity;			// see enum in this file for settings
	uint8_t text_verbosity;			// see enum in this file for settings
	uint8_t usb_baud_rate;			// see xio_usart.h for XIO_BAUD values
	uint8_t usb_baud_flag;			// technically this belongs in the controller singleton

	uint8_t echo_json_footer;		// flags for JSON responses serialization
	uint8_t echo_json_messages;
	uint8_t echo_json_configs;
	uint8_t echo_json_linenum;
	uint8_t echo_json_gcode_block;

	// status report configs
	uint8_t status_report_verbosity;					// see enum in this file for settings
	uint32_t status_report_interval;					// in MS. set non-zero to enable
	index_t status_report_list[CMD_STATUS_REPORT_LEN];	// status report elements to report
	float status_report_value[CMD_STATUS_REPORT_LEN];	// previous values for filtered reporting

	// coordinate systems and offsets
	float offset[COORDS+1][AXES];	// persistent coordinate offsets: absolute + G54,G55,G56,G57,G58,G59

	// motor and axis structs
	cfgMotor_t m[MOTORS];			// settings for motors 1-4
	cfgAxis_t a[AXES];				// settings for axes X,Y,Z,A B,C
	cfgPWM_t p;						// settings for PWM p

	uint16_t magic_end;
} cfgParameters_t;
extern cfgParameters_t cfg; 		// declared in config_app.cpp


#ifdef __cplusplus
}
#endif

#endif //_CONFIG_APP_H_

