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

#ifndef _CONFIG_APP_H_
#define _CONFIG_APP_H_

#ifdef __cplusplus
extern "C"{
#endif

/***********************************************************************************
 **** APPLICATION_SPECIFIC CONFIG STRUCTURE(S) *************************************
 ***********************************************************************************

 * Define the cfg structures(s) used by the application
 */
 typedef struct cfgParameters {
//	double fw_build;			// tinyg firmware build number
//	double fw_version;			// tinyg firmware version number
//	double hw_version;			// tinyg hardware compatibility
 } cfgParameters_t;
extern cfgParameters_t cfg; 	// declared in config_app.cpp

/***********************************************************************************
 **** PROGRAM MEMORY STRINGS AND STRING ARRAYS *************************************
 ***********************************************************************************/
/* PROGMEM strings for print formatting
 * NOTE: DO NOT USE TABS IN FORMAT STRINGS
 */

//static const char_t msg_units0[] PROGMEM = " in";	// used by generic print functions
//static const char_t msg_units1[] PROGMEM = " mm";
//static const char_t msg_units2[] PROGMEM = " deg";
//static const char_t *msg_units[] PROGMEM = { msg_units0, msg_units1, msg_units2 };
//#define F_DEG 2

/***********************************************************************************
 **** FUNCTIONS AND PROTOTYPES *****************************************************
 ***********************************************************************************/
/* Imported from tinyg / config.c
//static void _pr_ma_str(cmdObj_t *cmd); // generic print functions for motors and axes
static void _pr_ma_ui8(cmdObj_t *cmd);
//static void _pr_ma_int(cmdObj_t *cmd); // placeholder
//static void _pr_ma_dbl(cmdObj_t *cmd); // placeholder
static void _pr_ma_lin(cmdObj_t *cmd);
static void _pr_ma_rot(cmdObj_t *cmd);
static void _print_coor(cmdObj_t *cmd);	// print coordinate offsets with linear units
static void _print_corr(cmdObj_t *cmd);	// print coordinate offsets with rotary units

// helpers for generic functions
static char *_get_format(const index_t i, char *format);
static int8_t _get_motor(const index_t i);
//static int8_t _get_axis(const index_t i);
static int8_t _get_pos_axis(const index_t i);
static uint8_t _text_parser(char *str, cmdObj_t *c);
static uint8_t _get_msg_helper(cmdObj_t *cmd, prog_char_ptr msg, uint8_t value);
*/
/*
// parameter-specific internal functions
static uint8_t _set_hv(cmdObj_t *cmd);		// set hardware version
static uint8_t _get_sr(cmdObj_t *cmd);		// run status report (as data)
static void _print_sr(cmdObj_t *cmd);		// run status report (as printout)
static uint8_t _set_sr(cmdObj_t *cmd);		// set status report specification
static uint8_t _set_si(cmdObj_t *cmd);		// set status report interval
static uint8_t _run_boot(cmdObj_t *cmd);	// jump to the bootloader
static uint8_t _get_id(cmdObj_t *cmd);		// get device ID
static uint8_t _get_qr(cmdObj_t *cmd);		// get a queue report (as data)
static uint8_t _run_qf(cmdObj_t *cmd);		// execute a queue flush block
static uint8_t _get_er(cmdObj_t *cmd);		// invoke a bogus exception report for testing purposes
static uint8_t _get_rx(cmdObj_t *cmd);		// get bytes in RX buffer

static uint8_t _get_gc(cmdObj_t *cmd);		// get current gcode block
static uint8_t _run_gc(cmdObj_t *cmd);		// run a gcode block
static uint8_t _run_home(cmdObj_t *cmd);	// invoke a homing cycle

static uint8_t _get_line(cmdObj_t *cmd);	// get runtime line number
static uint8_t _get_stat(cmdObj_t *cmd);	// get combined machine state as value and string
static uint8_t _get_macs(cmdObj_t *cmd);	// get raw machine state as value and string
static uint8_t _get_cycs(cmdObj_t *cmd);	// get raw cycle state (etc etc)...
static uint8_t _get_mots(cmdObj_t *cmd);	// get raw motion state...
static uint8_t _get_hold(cmdObj_t *cmd);	// get raw hold state...
static uint8_t _get_home(cmdObj_t *cmd);	// get raw homing state...
static uint8_t _get_unit(cmdObj_t *cmd);	// get unit mode...
static uint8_t _get_coor(cmdObj_t *cmd);	// get coordinate system in effect...
static uint8_t _get_momo(cmdObj_t *cmd);	// get motion mode...
static uint8_t _get_plan(cmdObj_t *cmd);	// get active plane...
static uint8_t _get_path(cmdObj_t *cmd);	// get patch control mode...
static uint8_t _get_dist(cmdObj_t *cmd);	// get distance mode...
static uint8_t _get_frmo(cmdObj_t *cmd);	// get feedrate mode...
static uint8_t _get_vel(cmdObj_t *cmd);		// get runtime velocity...
static uint8_t _get_pos(cmdObj_t *cmd);		// get runtime work position...
static uint8_t _get_mpos(cmdObj_t *cmd);	// get runtime machine position...
static uint8_t _get_ofs(cmdObj_t *cmd);		// get runtime work offset...
static void _print_pos(cmdObj_t *cmd);		// print runtime work position in prevailing units
static void _print_mpos(cmdObj_t *cmd);		// print runtime work position always in MM uints

static uint8_t _set_defa(cmdObj_t *cmd);	// reset config to default values

static uint8_t _set_sa(cmdObj_t *cmd);		// set motor step angle
static uint8_t _set_tr(cmdObj_t *cmd);		// set motor travel per revolution
static uint8_t _set_mi(cmdObj_t *cmd);		// set microsteps
static uint8_t _set_po(cmdObj_t *cmd);		// set motor polarity

static uint8_t _set_sw(cmdObj_t *cmd);		// must run any time you change a switch setting
static uint8_t _get_am(cmdObj_t *cmd);		// get axis mode
static uint8_t _set_am(cmdObj_t *cmd);		// set axis mode
static void _print_am(cmdObj_t *cmd);		// print axis mode

static uint8_t _set_ic(cmdObj_t *cmd);		// ignore CR or LF on RX input
static uint8_t _set_ec(cmdObj_t *cmd);		// expand CRLF on TX outout
static uint8_t _set_ee(cmdObj_t *cmd);		// enable character echo
static uint8_t _set_ex(cmdObj_t *cmd);		// enable XON/XOFF
static uint8_t _set_baud(cmdObj_t *cmd);	// set USB baud rate
*/


/*
static uint8_t _get_htmp(cmdObj_t *cmd)
{
	cmd->value = heater.temperature;
	cmd->type = TYPE_FLOAT;
	return (SC_OK);
}
*/
#ifdef __cplusplus
}
#endif

#endif //_CONFIG_APP_H_

