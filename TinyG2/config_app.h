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
/* This file contains application speoific data for the config system:
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

//static const char msg_units0[] PROGMEM = " in";	// used by generic print functions
//static const char msg_units1[] PROGMEM = " mm";
//static const char msg_units2[] PROGMEM = " deg";
//static PGM_P const msg_units[] PROGMEM = { msg_units0, msg_units1, msg_units2 };
//#define F_DEG 2

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
#ifdef __cplusplus
}
#endif

#endif //_CONFIG_APP_H_

