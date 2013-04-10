/*
 * config_app.h - application-specific part of configuration data
 * Part of Kinen project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* This file contains application specific data for the config system:
 *	- application-specific functions and function prototypes 
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
 *	Persistence is provided by an NVM array containing values in EEPROM as doubles; 
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

/***********************************************************************************
 **** MORE INCLUDE FILES ***********************************************************
 ***********************************************************************************
 * Depending on what's in your functions you may require more include files here
 */
//#include <ctype.h>
//#include <stdlib.h>
//#include <string.h>
//#include <stdio.h>
#include <stdbool.h>
//#include <avr/pgmspace.h>

#include "kinen.h"
#include "tinyg2.h"
#include "config.h"
#include "config_app.h"

/***********************************************************************************
 **** PROGRAM MEMORY STRINGS AND STRING ARRAYS *************************************
 ***********************************************************************************/
/* PROGMEM strings for print formatting
 * NOTE: DO NOT USE TABS IN FORMAT STRINGS
 */
#ifdef __ENABLE_TEXTMODE

const char fmt_nul[] PROGMEM = "";
const char fmt_ui8[] PROGMEM = "%d\n";	// generic format for ui8s
const char fmt_dbl[] PROGMEM = "%f\n";	// generic format for doubles
const char fmt_str[] PROGMEM = "%s\n";	// generic format for string message (with no formatting)

// System group and ungrouped formatting strings
const char fmt_fv[] PROGMEM = "[fv]  firmware version%16.2f\n";
const char fmt_fb[] PROGMEM = "[fb]  firmware build%18.2f\n";
const char fmt_hv[] PROGMEM = "[hv]  hardware version%16.2f\n";
//const char fmt_id[] PROGMEM = "[id]  TinyG ID%30s\n";

#endif // __ENABLE_TEXTMODE

/***********************************************************************************
 **** CONFIG ARRAY  ****************************************************************
 ***********************************************************************************
 *
 *	NOTES:
 *	- Token matching occurs from the most specific to the least specific.
 *	  This means that if shorter tokens overlap longer ones the longer one
 *	  must precede the shorter one. E.g. "gco" needs to come before "gc"
 *
 *	- Mark group strings for entries that have no group as nul -->" ". 
 *	  This is important for group expansion.
 *
 *	- Groups do not have groups. Neither do uber-groups, e.g.
 *	  'x' is --> { "", "x",  	and 'm' is --> { "", "m", 
 *
 *	NOTE: If the count of lines in cfgArray exceeds 255 you need to change index_t 
 *	uint16_t in the config.h file.
 *
 *	NOTE: The table must be different depending on whether textmode is enabled or not.
 *
 * Table format with textmode enabled is this:
 * const cfgItem_t cfgArray[] PROGMEM = {
	// grp  token flags get_func, set_func  target for get/set,   	   default value
	{ "sys","fb", _f07, _get_dbl, _set_dbl, (double *)&cfg.fw_build,   BUILD_NUMBER }, // MUST BE FIRST!
	{ "sys","fv", _f07, _get_dbl, _set_dbl, (double *)&cfg.fw_version, VERSION_NUMBER },
	{ "sys","hv", _f07, _get_dbl, _set_dbl, (double *)&cfg.hw_version, HARDWARE_VERSION },

 * Table format with text mode disabled is this:
 * const cfgItem_t cfgArray[] PROGMEM = {
	// grp  token flags format*, print_func, get_func, set_func  target for get/set,   default value
	{ "sys","fb", _f07, fmt_fb, _print_nul, _get_dbl, _set_dbl, (double *)&cfg.fw_build,   BUILD_NUMBER }, // MUST BE FIRST!
	{ "sys","fv", _f07, fmt_fv, _print_nul, _get_dbl, _set_dbl, (double *)&cfg.fw_version, VERSION_NUMBER },
	{ "sys","hv", _f07, fmt_hv, _print_nul, _get_dbl, _set_dbl, (double *)&cfg.hw_version, HARDWARE_VERSION },

 */


//const cfgItem_t cfgArray[] PROGMEM = {
const cfgItem_t cfgArray[] = {
	// grp  token flags get_func, set_func  target for get/set,		   default value
	{ "sys","fb", _f07, _get_dbl, _set_dbl, (double *)&cfg.fw_build,   TINYG2_BUILD_NUMBER }, // MUST BE FIRST!
	{ "sys","fv", _f07, _get_dbl, _set_dbl, (double *)&cfg.fw_version, TINYG2_VERSION_NUMBER },
	{ "sys","hv", _f07, _get_dbl, _set_dbl, (double *)&cfg.hw_version, TINYG2_HARDWARE_VERSION },
/*
	// Heater object
	{ "h1", "h1st",  _f00, _get_ui8, _set_ui8,(double *)&heater.state, HEATER_OFF },
	{ "h1", "h1tmp", _f00, _get_dbl, _set_dbl,(double *)&heater.temperature, LESS_THAN_ZERO },
	{ "h1", "h1set", _f00, _get_dbl, _set_dbl,(double *)&heater.setpoint, HEATER_HYSTERESIS },
	{ "h1", "h1hys", _f00, _get_ui8, _set_ui8,(double *)&heater.hysteresis, HEATER_HYSTERESIS },
	{ "h1", "h1amb", _f00, _get_dbl, _set_dbl,(double *)&heater.ambient_temperature, HEATER_AMBIENT_TEMPERATURE },
	{ "h1", "h1ovr", _f00, _get_dbl, _set_dbl,(double *)&heater.overheat_temperature, HEATER_OVERHEAT_TEMPERATURE },
	{ "h1", "h1ato", _f00, _get_dbl, _set_dbl,(double *)&heater.ambient_timeout, HEATER_AMBIENT_TIMEOUT },
	{ "h1", "h1reg", _f00, _get_dbl, _set_dbl,(double *)&heater.regulation_range, HEATER_REGULATION_RANGE },
	{ "h1", "h1rto", _f00, _get_dbl, _set_dbl,(double *)&heater.regulation_timeout, HEATER_REGULATION_TIMEOUT },
	{ "h1", "h1bad", _f00, _get_ui8, _set_ui8,(double *)&heater.bad_reading_max, HEATER_BAD_READING_MAX },

	// Sensor object
	{ "s1", "s1st",  _f00, _get_ui8, _set_ui8,(double *)&sensor.state, SENSOR_OFF },
	{ "s1", "s1tmp", _f00, _get_dbl, _set_dbl,(double *)&sensor.temperature, LESS_THAN_ZERO },
	{ "s1", "s1svm", _f00, _get_dbl, _set_dbl,(double *)&sensor.sample_variance_max, SENSOR_SAMPLE_VARIANCE_MAX },
	{ "s1", "s1rvm", _f00, _get_dbl, _set_dbl,(double *)&sensor.reading_variance_max, SENSOR_READING_VARIANCE_MAX },

	// PID object
//	{ "p1", "p1st",  _f00, _get_ui8, _set_ui8,(double *)&pid.state, 0 },
	{ "p1", "p1kp",	 _f00, _get_dbl, _set_dbl,(double *)&pid.Kp, PID_Kp },
	{ "p1", "p1ki",	 _f00, _get_dbl, _set_dbl,(double *)&pid.Ki, PID_Ki },
	{ "p1", "p1kd",	 _f00, _get_dbl, _set_dbl,(double *)&pid.Kd, PID_Kd },
	{ "p1", "p1smx", _f00, _get_dbl, _set_dbl,(double *)&pid.output_max, PID_MAX_OUTPUT },
	{ "p1", "p1smn", _f00, _get_dbl, _set_dbl,(double *)&pid.output_min, PID_MIN_OUTPUT },

	// Group lookups - must follow the single-valued entries for proper sub-string matching
	// *** Must agree with CMD_COUNT_GROUPS below ****
	{ "","sys",_f00, _get_grp, _set_grp,(double *)&kc.null,0 },	// system group
	{ "","h1", _f00, _get_grp, _set_grp,(double *)&kc.null,0 },	// heater group
	{ "","s1", _f00, _get_grp, _set_grp,(double *)&kc.null,0 },	// sensor group
	{ "","p1", _f00, _get_grp, _set_grp,(double *)&kc.null,0 }		// PID group
//																				   ^  watch the final (missing) comma!
	// Uber-group (groups of groups, for text-mode displays only)
	// *** Must agree with CMD_COUNT_UBER_GROUPS below ****
//	{ "", "$", _f00, _get_nul, _set_nul,(double *)&kc.null,0 }
*/
};
/***** Make sure these defines line up with any changes in the above table *****/

#define CMD_COUNT_GROUPS 		4		// count of simple groups
#define CMD_COUNT_UBER_GROUPS 	0 		// count of uber-groups

#define CMD_INDEX_MAX (sizeof cfgArray / sizeof(cfgItem_t))
//#define CMD_INDEX_END_SINGLES		(CMD_INDEX_MAX - CMD_COUNT_UBER_GROUPS - CMD_COUNT_GROUPS - CMD_STATUS_REPORT_LEN)
#define CMD_INDEX_END_SINGLES		(CMD_INDEX_MAX - CMD_COUNT_UBER_GROUPS - CMD_COUNT_GROUPS)
#define CMD_INDEX_START_GROUPS		(CMD_INDEX_MAX - CMD_COUNT_UBER_GROUPS - CMD_COUNT_GROUPS)
#define CMD_INDEX_START_UBER_GROUPS (CMD_INDEX_MAX - CMD_COUNT_UBER_GROUPS)

//index_t cmd_get_max_index() { return (CMD_INDEX_MAX);}
uint8_t cmd_index_lt_max(index_t index) { return ((index < CMD_INDEX_MAX) ? true : false);}
uint8_t cmd_index_is_single(index_t index) { return ((index <= CMD_INDEX_END_SINGLES) ? true : false);}
uint8_t cmd_index_is_group(index_t index) { return (((index >= CMD_INDEX_START_GROUPS) && (index < CMD_INDEX_START_UBER_GROUPS)) ? true : false);}
uint8_t cmd_index_lt_groups(index_t index) { return ((index <= CMD_INDEX_START_GROUPS) ? true : false);}


/***********************************************************************************
 **** FUNCTIONS AND PROTOTYPES *****************************************************
 ***********************************************************************************/
/*
static uint8_t _get_htmp(cmdObj_t *cmd)
{
	cmd->value = heater.temperature;
	cmd->type = TYPE_FLOAT;
	return (SC_OK);
}
*/
