/*
 * config.cpp - configuration handling and persistence; master function table
 * This file is part of the TinyG2 project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
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
#include "tinyg2.h"
#include "config.h"
#include "json_parser.h"
#include "text_parser.h"
#include "controller.h"
#include "canonical_machine.h"
#include "util.h"
#include "xio.h"

#ifdef __cplusplus
extern "C"{
#endif

/***********************************************************************************
 **** STRUCTURE ALLOCATIONS ********************************************************
 ***********************************************************************************/

cmdStr_t cmdStr;
cmdObj_t cmd_list[CMD_LIST_LEN];	// JSON header element

/***********************************************************************************
 **** GENERIC STATIC FUNCTIONS AND VARIABLES ***************************************
 ***********************************************************************************/

static stat_t set_defa(cmdObj_t *cmd);	// reset config to default values
//static void _do_group_list(cmdObj_t *cmd, char list[][CMD_TOKEN_LEN+1]); // helper to print multiple groups in a list

/***********************************************************************************
 **** CODE *************************************************************************
 ***********************************************************************************/
/* Primary access points to cmd functions
 * These gatekeeper functions check index ranges so others don't have to
 *
 * cmd_set() 	- Write a value or invoke a function - operates on single valued elements or groups
 * cmd_get() 	- Build a cmdObj with the values from the target & return the value
 *			   	  Populate cmd body with single valued elements or groups (iterates)
 * cmd_print()	- Output a formatted string for the value.
 * cmd_persist()- persist value to NVM. Takes special cases into account
 */
stat_t cmd_set(cmdObj_t *cmd)
{
//	ritorno(cmd_index_lt_max(cmd->index));	// validate index or return
	return (cfgArray[cmd->index].set(cmd));
}

stat_t cmd_get(cmdObj_t *cmd)
{
//	ritorno(cmd_index_lt_max(cmd->index));	// validate index or return
	return (cfgArray[cmd->index].get(cmd));
}

void cmd_print(cmdObj_t *cmd)
{
//	if (cmd->index >= CMD_INDEX_MAX) return;
	cfgArray[cmd->index].print(cmd);
}

void cmd_persist(cmdObj_t *cmd)
{
#ifdef __ENABLE_PERSISTENCE	
//	if (cmd_index_lt_groups(cmd->index) == false) return;
	if ((cfgArray[cmd->index].flags) & F_PERSIST) {
		cmd_write_NVM_value(cmd);
	}
#endif
	return;
}

/****************************************************************************
 * cfg_init() - called once on hard reset
 * _set_defa() - reset NVM with default values for active profile
 *
 * Will perform one of 2 actions:
 *	(1) if NVM is set up or out-of-rev: load RAM and NVM with hardwired default settings
 *	(2) if NVM is set up and at current config version: use NVM data for config
 *
 *	You can assume the cfg struct has been zeroed by a hard reset. 
 *	Do not clear it as the version and build numbers have already been set by tg_init()
 */
void cfg_init()
{
	cmdObj_t *cmd = cmd_reset_list();
	cs.comm_mode = JSON_MODE;						// initial value until EEPROM is read
//	cs.nvm_base_addr = NVM_BASE_ADDR;
//	cs.nvm_profile_base = cfg.nvm_base_addr;
	cmd->value = true;
	set_defa(cmd);		// this subroutine called from here and from the $defa=1 command
}

static stat_t set_defa(cmdObj_t *cmd) 
{
	if (fp_FALSE(cmd->value)) { return (STAT_OK);}	// failsafe. Must set true or no action occurs
//	rpt_print_initializing_message();
	for (cmd->index=0; cmd_index_is_single(cmd->index); cmd->index++) {
		if ((cfgArray[cmd->index].flags & F_INITIALIZE) != 0) {
			cmd->value = cfgArray[cmd->index].def_value;
			strcpy(cmd->token, cfgArray[cmd->index].token);
			cmd_set(cmd);
			cmd_persist(cmd);
		}
	}
	return (STAT_OK);
}

/***** Generic Internal Functions *********************************************/

/* Generic gets()
 *  get_nul() - get nothing (returns STAT_NOOP)
 *  get_ui8() - get value as 8 bit uint8_t w/o unit conversion
 *  get_int() - get value as 32 bit integer w/o unit conversion
 *  get_flt() - get value as float w/o unit conversion
 *  get_flu() - get value as double w/unit conversion
 *	get_format() - internal accessor for printf() format string
 */
stat_t get_nul(cmdObj_t *cmd) 
{ 
	cmd->type = TYPE_NULL;
	return (STAT_NOOP);
}

stat_t get_ui8(cmdObj_t *cmd)
{
	cmd->value = (float)*((uint8_t *)cfgArray[cmd->index].target);
	cmd->type = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t get_int(cmdObj_t *cmd)
{
	cmd->value = (float)*((uint32_t *)cfgArray[cmd->index].target);
	cmd->type = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t get_flt(cmdObj_t *cmd)
{
	cmd->value = *((float *)cfgArray[cmd->index].target);
	cmd->precision = cfgArray[cmd->index].precision;
	cmd->type = TYPE_FLOAT;
	return (STAT_OK);
}

stat_t get_flu(cmdObj_t *cmd)
{
	get_flt(cmd);
	if (cm_get_units_mode() == INCHES) {
		cmd->value *= INCH_PER_MM;
	}
	cmd->precision = cfgArray[cmd->index].precision;
	cmd->type = TYPE_FLOAT;
	return (STAT_OK);
}
/* REPLACED BY A MACRO - See config.h
char *get_format(const index_t index)
{
	return ((char *)cfgArray[index].format);
}
*/

/* Generic sets()
 *  set_nul() - set nothing (returns STAT_NOOP)
 *  set_ui8() - set value as 8 bit uint8_t value w/o unit conversion
 *  set_01()  - set a 0 or 1 uint8_t value with validation
 *  set_012() - set a 0, 1 or 2 uint8_t value with validation
 *  set_int() - set value as 32 bit integer w/o unit conversion
 *  set_flt() - set value as float w/o unit conversion
 *  set_flu() - set value as float w/unit conversion
 */
stat_t set_nul(cmdObj_t *cmd) { return (STAT_NOOP);}

stat_t set_ui8(cmdObj_t *cmd)
{
	*((uint8_t *)cfgArray[cmd->index].target) = cmd->value;
	cmd->type = TYPE_INTEGER;
	return(STAT_OK);
}

stat_t set_01(cmdObj_t *cmd)
{
	if (cmd->value > 1) {
		return (STAT_INPUT_VALUE_UNSUPPORTED);
	} else {
		return (set_ui8(cmd));
	}
}

stat_t set_012(cmdObj_t *cmd)
{
	if (cmd->value > 2) {
		return (STAT_INPUT_VALUE_UNSUPPORTED);
	} else {
		return (set_ui8(cmd));
	}
}

stat_t set_int(cmdObj_t *cmd)
{
	*((uint32_t *)cfgArray[cmd->index].target) = cmd->value;
	cmd->type = TYPE_INTEGER;
	return(STAT_OK);
}

stat_t set_flt(cmdObj_t *cmd)
{
	*((float *)cfgArray[cmd->index].target) = cmd->value;
	cmd->precision = cfgArray[cmd->index].precision;
	cmd->type = TYPE_FLOAT;
	return(STAT_OK);
}

stat_t set_flu(cmdObj_t *cmd)
{
	if (cm_get_units_mode() == INCHES) { cmd->value *= MM_PER_INCH;}
	*((float *)cfgArray[cmd->index].target) = cmd->value;
	cmd->precision = cfgArray[cmd->index].precision;
	cmd->type = TYPE_FLOAT_UNITS;
	return(STAT_OK);
}

/* Generic prints()
 * print_nul() - print nothing
 * print_str() - print string value
 * print_ui8() - print uint8_t value w/no units or unit conversion
 * print_int() - print integer value w/no units or unit conversion
 * print_flt() - print float value w/no units or unit conversion
 * print_lin() - print linear value with units and in/mm unit conversion
 * print_rot() - print rotary value with units
 */
void print_nul(cmdObj_t *cmd) {}

void print_str(cmdObj_t *cmd)
{
	cmd_get(cmd);
	fprintf(stderr, get_format(cmd->index), *(cmd->stringp));
}

void print_ui8(cmdObj_t *cmd)
{
	cmd_get(cmd);
	fprintf(stderr, get_format(cmd->index), (uint8_t)cmd->value);
}

void print_int(cmdObj_t *cmd)
{
	cmd_get(cmd);
	fprintf(stderr, get_format(cmd->index), (uint32_t)cmd->value);
}

void print_flt(cmdObj_t *cmd)
{
	cmd_get(cmd);
	fprintf(stderr, get_format(cmd->index), cmd->value);
}

void print_lin(cmdObj_t *cmd)
{
	cmd_get(cmd);
//	fprintf(stderr, get_format(cmd->index), cmd->value, (PGM_P)pgm_read_word(&msg_units[cm_get_units_mode()]));
	fprintf(stderr, get_format(cmd->index), cmd->value);
}

void print_rot(cmdObj_t *cmd)
{
	cmd_get(cmd);
//	fprintf(stderr, get_format(cmd->index), cmd->value, (PGM_P)pgm_read_word(&msg_units[F_DEG]));
	fprintf(stderr, get_format(cmd->index), cmd->value);
}


/********************************************************************************
 * Group operations
 */
/* 
 * get_grp() - read data from a group
 *
 *	get_grp() is a group expansion function that expands the parent group and  returns 
 *	the values of all the children in that group. It expects the first cmdObj in the 
 *	cmdBody to have a valid group name in the token field. This first object will be set 
 *	to a TYPE_PARENT. The group field is left nul - as the group field refers to a parent 
 *	group, which this group has none.
 *
 *	All subsequent cmdObjs in the body will be populated with their values.
 *	The token field will be populated as will the parent name in the group field. 
 *
 *	The sys group is an exception where the childern carry a blank group field, even though 
 *	the sys parent is labeled as a TYPE_PARENT.
 */

stat_t get_grp(cmdObj_t *cmd)
{
	uint8_t *parent_group = cmd->token;		// token in the parent cmd object is the group
	uint8_t group[CMD_GROUP_LEN+1];			// group string retrieved from cfgArray child
	cmd->type = TYPE_PARENT;				// make first object the parent 
//	for (index_t i=0; i<=CMD_INDEX_END_SINGLES; i++) {
	for (index_t i=0; cmd_index_is_single(i); i++) {
		strcpy(group, cfgArray[i].group);  // don't need strncpy as it's always terminated
		if (strcmp(parent_group, group) != 0) continue;
		(++cmd)->index = i;
		cmd_get_cmdObj(cmd);
	}
	return (STAT_OK);
}

/*
 * set_grp() - get or set one or more values in a group
 *
 *	This functions is called "set_group()" but technically it's a getter and a setter. 
 *	It iterates the group children and either gets the value or sets the value for each 
 *	depending on the cmd->type.
 *
 *	This function serves JSON mode only as text mode shouldn't call it.
 */
stat_t set_grp(cmdObj_t *cmd)
{
	if (cs.comm_mode == TEXT_MODE) return (STAT_UNRECOGNIZED_COMMAND);
	for (uint8_t i=0; i<CMD_MAX_OBJECTS; i++) {
		if ((cmd = cmd->nx) == NULL) break;
		if (cmd->type == TYPE_EMPTY) break;
		else if (cmd->type == TYPE_NULL)	// NULL means GET the value
			cmd_get(cmd);
		else {
			cmd_set(cmd);
			cmd_persist(cmd);
		}
	}
	return (STAT_OK);
}

/*
 * cmd_group_is_prefixed() - hack
 *
 *	This little function deals with the fact that some groups don't use the parent 
 *	token as a prefix to the child elements; SR being a good example.
 */
uint8_t cmd_group_is_prefixed(uint8_t *group)
{
	if (strstr("sr", group) != NULL) {	// you can extend like this: "sr,sys,xyzzy"
		return (false);
	}
	return (true);
}


/***********************************************************************************
 ***** cmdObj functions ************************************************************
 ***********************************************************************************/

/*****************************************************************************
 * cmdObj helper functions and other low-level cmd helpers
 * cmd_get_index() 		 - get index from mnenonic token + group
 * cmd_get_type()		 - returns command type as a CMD_TYPE enum
 * cmd_persist_offsets() - write any changed G54 (et al) offsets back to NVM
 * 
 * cmd_get_index() is the most expensive routine in the whole config. It does a linear table scan 
 * of the PROGMEM strings, which of course could be further optimized with indexes or hashing.
 */
index_t cmd_get_index(const char_t *group, const char_t *token)
{
	char_t c;
	char_t str[CMD_TOKEN_LEN+1];
	strcpy_U(str, group);
	strcat(str, token);

	index_t index_max = cmd_index_max();

	for (index_t i=0; index_max; i++) {
		if ((c = cfgArray[i].token[0]) != str[0]) {	continue; }					// 1st character mismatch
		if ((c = cfgArray[i].token[1]) == NUL) { if (str[1] == NUL) return(i);}	// one character match
		if (c != str[1]) continue;												// 2nd character mismatch
		if ((c = cfgArray[i].token[2]) == NUL) { if (str[2] == NUL) return(i);}	// two character match
		if (c != str[2]) continue;												// 3rd character mismatch
		if ((c = cfgArray[i].token[3]) == NUL) { if (str[3] == NUL) return(i);}	// three character match
		if (c != str[3]) continue;												// 4th character mismatch
		if ((c = cfgArray[i].token[4]) == NUL) { if (str[4] == NUL) return(i);}	// four character match
		if (c != str[4]) continue;												// 5th character mismatch
		return (i);																// five character match
	}
	return (NO_MATCH);
}
/*
 * cmdObj low-level object and list operations
 * cmd_get_cmdObj()		- setup a cmd object by providing the index
 * cmd_reset_obj()		- quick clear for a new cmd object
 * cmd_reset_list()		- clear entire header, body and footer for a new use
 * cmd_copy_string()	- used to write a string to shared string storage and link it
 * cmd_copy_string_P()	- same, but for progmem string sources
 * cmd_add_object()		- write contents of parameter to  first free object in the body
 * cmd_add_integer()	- add an integer value to end of cmd body (Note 1)
 * cmd_add_float()		- add a floating point value to end of cmd body
 * cmd_add_string()		- add a string object to end of cmd body
 * cmd_add_string_P()	- add a program memory string as a string object to end of cmd body
 * cmd_add_message()	- add a mesasge to cmd body
 * cmd_add_message_P()	- add a program memory message the the cmd body
 *
 *	Note: Functions that return a cmd pointer point to the object that was modified
 *	or a NULL pointer if there was an error
 *
 *	Note Adding a really large integer (like a checksum value) may lose precision 
 *	due to the cast to a float. Sometimes it's better to load an integer as a 
 *	string if all you want to do is display it.
 */

void cmd_get_cmdObj(cmdObj_t *cmd)
{
	if (cmd_index_lt_max(cmd->index)) { return;}
	index_t tmp = cmd->index;
	cmd_reset_obj(cmd);
	cmd->index = tmp;

	strcpy(cmd->group, cfgArray[cmd->index].group); // group field is always terminated
	strcpy(cmd->token, cfgArray[cmd->index].token); // token field is always terminated

	// special processing for system groups and stripping tokens for groups
	if (cmd->group[0] != NUL) {
		if (cfgArray[cmd->index].flags & F_NOSTRIP) {
			cmd->group[0] = NUL;
		} else {
			strcpy(cmd->token, &cmd->token[strlen(cmd->group)]); // strip group from the token
		}
	}
	((fptrCmd)(cfgArray[cmd->index].get))(cmd);	// populate the value
}
 
cmdObj_t *cmd_reset_obj(cmdObj_t *cmd)	// clear a single cmdObj structure
{
	cmd->type = TYPE_EMPTY;				// selective clear is much faster than calling memset
	cmd->index = 0;
	cmd->value = 0;
	cmd->precision = 0;
	cmd->token[0] = NUL;
	cmd->group[0] = NUL;
	cmd->stringp = NULL;

	if (cmd->pv == NULL) { 				// set depth correctly
		cmd->depth = 0;
	} else {
		if (cmd->pv->type == TYPE_PARENT) { 
			cmd->depth = cmd->pv->depth + 1;
		} else {
			cmd->depth = cmd->pv->depth;
		}
	}
	return (cmd);
}

cmdObj_t *cmd_reset_list()					// clear the header and response body
{
	cmdStr.wp = 0;							// reset the shared string
	cmdObj_t *cmd = cmd_list;				// set up linked list and initialize elements	
	for (uint8_t i=0; i<CMD_LIST_LEN; i++, cmd++) {
		cmd->pv = (cmd-1);					// the ends are bogus & corrected later
		cmd->nx = (cmd+1);
		cmd->index = 0;
		cmd->depth = 1;						// header and footer are corrected later
		cmd->type = TYPE_EMPTY;
		cmd->token[0] = NUL;
	}
	(--cmd)->nx = NULL;
	cmd = cmd_list;							// setup response header element ('r')
	cmd->pv = NULL;
	cmd->depth = 0;
	cmd->type = TYPE_PARENT;
	strcpy(cmd->token, "r");
	return (cmd_body);						// this is a convenience for calling routines
}

stat_t cmd_copy_string(cmdObj_t *cmd, const char_t *src)
{
	if ((cmdStr.wp + strlen(src)) > CMD_SHARED_STRING_LEN) { return (STAT_BUFFER_FULL);}
	char_t *dst = &cmdStr.string[cmdStr.wp];
	strcpy(dst, src);						// copy string to current head position
	cmdStr.wp += strlen(src)+1;				// advance head for next string
	cmd->stringp = (char_t (*)[])dst;
	return (STAT_OK);
}

cmdObj_t *cmd_add_object(const char_t *token)		// add an object to the body using a token
{
	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN; i++) {
		if (cmd->type != TYPE_EMPTY) {
			cmd = cmd->nx;
			continue;
		}
		// load the index from the token or die trying
		if ((cmd->index = cmd_get_index((const uint8_t*)"", token)) == NO_MATCH) { return (NULL);}
		cmd_get_cmdObj(cmd);				// populate the object from the index
		return (cmd);
	}
	return (NULL);
}

cmdObj_t *cmd_add_integer(const char_t *token, const uint32_t value)// add an integer object to the body
{
	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN; i++) {
		if (cmd->type != TYPE_EMPTY) {
			cmd = cmd->nx;
			continue;
		}
		strncpy(cmd->token, token, CMD_TOKEN_LEN);
		cmd->value = (float) value;
		cmd->type = TYPE_INTEGER;
		return (cmd);
	}
	return (NULL);
}

cmdObj_t *cmd_add_float(const char_t *token, const float value)	// add a float object to the body
{
	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN; i++) {
		if (cmd->type != TYPE_EMPTY) {
			cmd = cmd->nx;
			continue;
		}
		strncpy(cmd->token, token, CMD_TOKEN_LEN);
		cmd->value = value;
		cmd->type = TYPE_FLOAT;
		return (cmd);
	}
	return (NULL);
}

cmdObj_t *cmd_add_string(const char_t *token, const char_t *string)	// add a string object to the body
{
	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN; i++) {
		if (cmd->type != TYPE_EMPTY) {
			cmd = cmd->nx;
			continue;
		}
		strncpy(cmd->token, token, CMD_TOKEN_LEN);
		if (cmd_copy_string(cmd, string) != STAT_OK) { return (NULL);}
		cmd->index = cmd_get_index((const uint8_t *)"", cmd->token);
		cmd->type = TYPE_STRING;
		return (cmd);
	}
	return (NULL);
}

cmdObj_t *cmd_add_message(const char_t *string)	// conditionally add a message object to the body
{
	return(cmd_add_string((const char_t *)"msg", string));
}


/* deprecated PROGMEM functions 

stat_t cmd_copy_string_P(cmdObj_t *cmd, const char_t *src_P)
{
	char_t buf[CMD_SHARED_STRING_LEN];
	//	strncpy_P(buf, src_P, CMD_SHARED_STRING_LEN);
	strncpy(buf, src_P, CMD_SHARED_STRING_LEN);
	return (cmd_copy_string(cmd, buf));
}


cmdObj_t *cmd_add_string_P(const char_t *token, const char_t *string)
{
	char_t message[CMD_MESSAGE_LEN];
	//	sprintf_P(message, string);
	sprintf((char *)message, (const char *)string);
	return(cmd_add_string(token, (char_t *)message));
}

cmdObj_t *cmd_add_message_P(const char_t *string)	// conditionally add a message object to the body
{
	char_t message[CMD_MESSAGE_LEN]; 
//	sprintf_P(message, string);
	sprintf((char *)message, (const char *)string);
	return(cmd_add_string((const char_t *)"msg", message));
}
*/
/**** cmd_print_list() - print cmd_array as JSON or text ****
 *
 * 	Use this function for all text and JSON output that wants to be in a response header
 *	(don't just printf stuff)
 * 	It generates and prints the JSON and text mode output strings 
 *	In JSON mode it generates the footer with the status code, buffer count and checksum
 *	In text mode it uses the the textmode variable to set the output format
 *
 *	Inputs:
 *	  json_flags = JSON_OBJECT_FORMAT - print just the body w/o header or footer
 *	  json_flags = JSON_RESPONSE_FORMAT - print a full "r" object with footer
 *
 *	  text_flags = TEXT_INLINE_PAIRS - print text as name/value pairs on a single line
 *	  text_flags = TEXT_INLINE_VALUES - print text as comma separated values on a single line
 *	  text_flags = TEXT_MULTILINE_FORMATTED - print text one value per line with formatting string
 */

void cmd_print_list(stat_t status, uint8_t text_flags, uint8_t json_flags)
{
	if (cs.comm_mode == JSON_MODE) {
		switch (json_flags) {
			case JSON_NO_PRINT: { break; } 
			case JSON_OBJECT_FORMAT: { json_print_object(cmd_body); break; }
			case JSON_RESPONSE_FORMAT: { json_print_response(status); break; }
		}
	} else {
		switch (text_flags) {
			case TEXT_NO_PRINT: { break; } 
			case TEXT_INLINE_PAIRS: { text_print_inline_pairs(cmd_body); break; }
			case TEXT_INLINE_VALUES: { text_print_inline_values(cmd_body); break; }
			case TEXT_MULTILINE_FORMATTED: { text_print_multiline_formatted(cmd_body);}
		}
	}
}

/************************************************************************************
 ***** EEPROM PERSISTENCE FUNCTIONS *************************************************
 ************************************************************************************
 * cmd_read_NVM_value()	 - return value (as float) by index
 * cmd_write_NVM_value() - write to NVM by index, but only if the value has changed
 *
 *	It's the responsibility of the caller to make sure the index does not exceed range
 */

stat_t cmd_read_NVM_value(cmdObj_t *cmd)
{
//	int8_t nvm_byte_array[NVM_VALUE_LEN];
//	uint16_t nvm_address = cfg.nvm_profile_base + (cmd->index * NVM_VALUE_LEN);
//	(void)EEPROM_ReadBytes(nvm_address, nvm_byte_array, NVM_VALUE_LEN);
//	memcpy(&cmd->value, &nvm_byte_array, NVM_VALUE_LEN);
	return (STAT_OK);
}

stat_t cmd_write_NVM_value(cmdObj_t *cmd)
{
//	float tmp = cmd->value;
//	ritorno(cmd_read_NVM_value(cmd));
//	if (cmd->value != tmp) {		// catches the isnan() case as well
//		cmd->value = tmp;
//		int8_t nvm_byte_array[NVM_VALUE_LEN];
//		memcpy(&nvm_byte_array, &tmp, NVM_VALUE_LEN);
//		uint16_t nvm_address = cfg.nvm_profile_base + (cmd->index * NVM_VALUE_LEN);
//		(void)EEPROM_WriteBytes(nvm_address, nvm_byte_array, NVM_VALUE_LEN);
//	}
	return (STAT_OK);
}

#ifdef __cplusplus
}
#endif // __cplusplus

