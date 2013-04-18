/*
 * text_parser.cpp - text parser for TinyG
 * Part of TinyG project
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
/* See the wiki for module details and additional information:
 *	 http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Developer-Info
 *	 http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-JSON
 */

#include "tinyg2.h"
#include "controller.h"
#include "config.h"					// JSON sits on top of the config system
#include "text_parser.h"
#include "util.h"
#include "xio.h"					// for char definitions

#ifdef __cplusplus
extern "C"{
#endif

static uint8_t _text_parser_kernal(char *str, cmdObj_t *cmd);

/******************************************************************************
 * text_parser() - update a config setting from a text block (text mode)
 * _text_parser_helper() - helper for above
 * 
 * Use cases handled:
 *	- $xfr=1200	set a parameter
 *	- $xfr		display a parameter
 *	- $x		display a group
 *	- ?			generate a status report (multiline format)
 */
uint8_t text_parser(char *str)
{
	cmdObj_t *cmd = cmd_reset_list();		// returns first object in the body
	uint8_t status = TG_OK;

	if (str[0] == '?') {					// handle status report case
		rpt_run_text_status_report();
		return (TG_OK);
	}
	if ((str[0] == '$') && (str[1] == NUL)) {  // treat a lone $ as a sys request
		strcat(str,"sys");
	}
	// single-unit parser processing
	ritorno(_text_parser(str, cmd));		// decode the request or return if error
	if ((cmd->type == TYPE_PARENT) || (cmd->type == TYPE_NULL)) {
		if (cmd_get(cmd) == TG_COMPLETE) {	// populate value, group values, or run uber-group displays
			return (TG_OK);					// return for uber-group displays so they don't print twice
		}
	} else { 								// process SET and RUN commands
		status = cmd_set(cmd);				// set single value
		cmd_persist(cmd);
	}
	cmd_print_list(status, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT); // print the results
	return (status);
}

static uint8_t _text_parser_helper(char *str, cmdObj_t *cmd)
{
	char *ptr_rd, *ptr_wr;					// read and write pointers
	char separators[] = {" =:|\t"};			// any separator someone might use

	// string pre-processing
	cmd_reset_obj(cmd);						// initialize config object
	cmd_copy_string(cmd, str);				// make a copy for eventual reporting
	if (*str == '$') str++;					// ignore leading $
	for (ptr_rd = ptr_wr = str; *ptr_rd!=NUL; ptr_rd++, ptr_wr++) {
		*ptr_wr = tolower(*ptr_rd);			// convert string to lower case
		if (*ptr_rd==',') {
			*ptr_wr = *(++ptr_rd);			// skip over comma
		}
	}
	*ptr_wr = NUL;

	// field processing
	cmd->type = TYPE_NULL;
	if ((ptr_rd = strpbrk(str, separators)) == NULL) { // no value part
		strncpy(cmd->token, str, CMD_TOKEN_LEN);
	} else {
		*ptr_rd = NUL;						// terminate at end of name
		strncpy(cmd->token, str, CMD_TOKEN_LEN);
		str = ++ptr_rd;
		cmd->value = strtod(str, &ptr_rd);	// ptr_rd used as end pointer
		if (ptr_rd != str) {
			cmd->type = TYPE_FLOAT;
		}
	}
	if ((cmd->index = cmd_get_index("",cmd->token)) == NO_MATCH) { 
		return (TG_UNRECOGNIZED_COMMAND);
	}
	return (TG_OK);
}


/************************************************************************************
 * text_response() - text mode responses
 */
static const uint8_t prompt_mm[] = "mm";
static const uint8_t prompt_in[] = "inch";
static const uint8_t prompt_ok[] = "tinyg [%S] ok> ";
static const uint8_t prompt_err[] = "tinyg [%S] err: %s: %s ";

void text_response(const uint8_t status, const uint8_t *buf)
{
	/*
	if (cs.text_verbosity == TV_SILENT) return;	// skip all this

	const char *units;								// becomes pointer to progmem string
	if (cm_get_units_mode() != INCHES) { 
		units = (PGM_P)&prompt_mm;
	} else {
		units = (PGM_P)&prompt_in;
	}
	if ((status == TG_OK) || (status == TG_EAGAIN) || (status == TG_NOOP) || (status == TG_ZERO_LENGTH_MOVE)) {
		fprintf_P(stderr, (PGM_P)&prompt_ok, units);
	} else {
		char status_message[STATUS_MESSAGE_LEN];
		fprintf_P(stderr, (PGM_P)prompt_err, units, rpt_get_status_message(status, status_message), buf);
	}
	cmdObj_t *cmd = cmd_body+1;
	if (cmd->token[0] == 'm') {
		fprintf(stderr, *cmd->stringp);
	}
	*/
	fprintf(stderr, *cmd->stringp);
	fprintf(stderr, "\n");
}


#ifdef __cplusplus
}
#endif // __cplusplus
