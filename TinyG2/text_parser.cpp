/*
 * text_parser.cpp - text parser for TinyG
 * This file is part of the TinyG2 project
 *
 * Copyright (c) 2013 Alden S. Hart Jr.
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
#include "controller.h"
#include "canonical_machine.h"
#include "text_parser.h"
#include "report.h"
#include "util.h"
#include "xio.h"					// for char definitions

#ifdef __cplusplus
extern "C"{
#endif

static stat_t _text_parser_kernal(uint8_t *str, cmdObj_t *cmd);

/******************************************************************************
 * text_parser() - update a config setting from a text block (text mode)
 * _text_parser_kernal() - helper for above
 * 
 * Use cases handled:
 *	- $xfr=1200	set a parameter
 *	- $xfr		display a parameter
 *	- $x		display a group
 *	- ?			generate a status report (multiline format)
 */
stat_t text_parser(uint8_t *str)
{
	cmdObj_t *cmd = cmd_reset_list();		// returns first object in the body
	uint8_t status = STAT_OK;

	if (str[0] == '?') {					// handle status report case
		rpt_run_text_status_report();
		return (STAT_OK);
	}
	if ((str[0] == '$') && (str[1] == NUL)) {  // treat a lone $ as a sys request
		strcat(str,"sys");
	}
	// single-unit parser processing
	ritorno(_text_parser_kernal(str, cmd));	// decode the request or return if error
	if ((cmd->type == TYPE_PARENT) || (cmd->type == TYPE_NULL)) {
		if (cmd_get(cmd) == STAT_COMPLETE) {// populate value, group values, or run uber-group displays
			return (STAT_OK);				// return for uber-group displays so they don't print twice
		}
	} else { 								// process SET and RUN commands
		status = cmd_set(cmd);				// set single value
		cmd_persist(cmd);
	}
	cmd_print_list(status, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT); // print the results
	return (status);
}

static stat_t _text_parser_kernal(uint8_t *str, cmdObj_t *cmd)
{
	uint8_t *ptr_rd, *ptr_wr;				// read and write pointers
//	uint8_t separators[] = {" =:|\t"};		// any separator someone might use
	uint8_t separators[] = {"="};			// only separator allowed is = sign

	// string pre-processing
//	cmd_reset_obj(cmd);						// initialize config object
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
	if ((cmd->index = cmd_get_index((uint8_t *)"", cmd->token)) == NO_MATCH) { 
		return (STAT_UNRECOGNIZED_COMMAND);
	}
	return (STAT_OK);
}


/************************************************************************************
 * text_response() - text mode responses
 */
static const char_t prompt_mm[] = "mm";
static const char_t prompt_in[] = "inch";
static const char_t prompt_ok[] = "tinyg [%S] ok> ";
static const char_t prompt_err[] = "tinyg [%S] err: %s: %s ";

void text_response(const uint8_t status, char_t *buf)
{
	cmdObj_t *cmd = cmd_body;

	if (cfg.text_verbosity == TV_SILENT) return;		// skip all this

	char *units;
	if (cm_get_units_mode() != INCHES) { 
		units = (char *)prompt_mm;
	} else {
		units = (char *)prompt_in;
	}
	if ((status == STAT_OK) || (status == STAT_EAGAIN) || (status == STAT_NOOP) || (status == STAT_ZERO_LENGTH_MOVE)) {
		fprintf(stderr, (char *)prompt_ok, units);
	} else {
		fprintf(stderr, (char *)prompt_err, units, (char *)get_status_message(status), buf);
	}
	cmd = cmd_body+1;
	if (cmd->token[0] == 'm') {
		fprintf(stderr, (char *)*cmd->stringp);
	}
}
//#else
//	cmdObj_t *cmd = cmd_body;
//	fprintf(stderr, (char *)cmd->stringp);
//	fprintf(stderr, "\n");
//#endif
//}

/************************************************************************************
 * text_print_inline_pairs()
 * text_print_inline_values()
 * text_print_multiline_formatted()
 */

void text_print_inline_pairs(cmdObj_t *cmd)
{
//	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN-1; i++) {
		switch (cmd->type) {
			case TYPE_PARENT:	{ cmd = cmd->nx; continue; }
			case TYPE_FLOAT:	{ fprintf(stderr,("%s:%1.3f"), cmd->token, cmd->value); break;}
			case TYPE_INTEGER:	{ fprintf(stderr,("%s:%1.0f"), cmd->token, cmd->value); break;}
			case TYPE_STRING:	{ fprintf(stderr,("%s:%s"), cmd->token, *cmd->stringp); break;}
			case TYPE_EMPTY:	{ fprintf(stderr,("\n")); return; }
		}
		cmd = cmd->nx;
		if (cmd->type != TYPE_EMPTY) { fprintf(stderr,(","));}
	}
}

void text_print_inline_values(cmdObj_t *cmd)
{
//	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN-1; i++) {
		switch (cmd->type) {
			case TYPE_PARENT:	{ cmd = cmd->nx; continue; }
			case TYPE_FLOAT:	{ fprintf(stderr,("%1.3f"), cmd->value); break;}
			case TYPE_INTEGER:	{ fprintf(stderr,("%1.0f"), cmd->value); break;}
			case TYPE_STRING:	{ fprintf(stderr,("%s"), *cmd->stringp); break;}
			case TYPE_EMPTY:	{ fprintf(stderr,("\n")); return; }
		}
		cmd = cmd->nx;
		if (cmd->type != TYPE_EMPTY) { fprintf(stderr,(","));}
	}
}

void text_print_multiline_formatted(cmdObj_t *cmd)
{
//	cmdObj_t *cmd = cmd_body;
	for (uint8_t i=0; i<CMD_BODY_LEN-1; i++) {
		if (cmd->type != TYPE_PARENT) { 
			cmd_print(cmd);
		}
		cmd = cmd->nx;
		if (cmd->type == TYPE_EMPTY) { 
			break;
		}
	}
}

#ifdef __cplusplus
}
#endif // __cplusplus
