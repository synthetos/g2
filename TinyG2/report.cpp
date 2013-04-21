/*
 * report.cpp - TinyG status report and other reporting functions.
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
#include "controller.h"
#include "report.h"
#include "json_parser.h"
#include "text_parser.h"
#include "settings.h"

/*
#include "util.h"
#include "planner.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
*/

/**** Status and Exception Messages **************************************************
 * rpt_get_status_message() - return the status message
 * rpt_exception() - send an exception report (JSON formatted)
 *
 * These strings must align with the status codes in tinyg.h
 * The number of elements in the indexing array must match the # of strings
 */
static const char_t stat_00[] = "OK";
static const char_t stat_01[] = "Error";
static const char_t stat_02[] = "Eagain";
static const char_t stat_03[] = "Noop";
static const char_t stat_04[] = "Complete";
static const char_t stat_05[] = "Terminated";
static const char_t stat_06[] = "Hard reset";
static const char_t stat_07[] = "End of line";
static const char_t stat_08[] = "End of file";
static const char_t stat_09[] = "File not open";
static const char_t stat_10[] = "Max file size exceeded";
static const char_t stat_11[] = "No such device";
static const char_t stat_12[] = "Buffer empty";
static const char_t stat_13[] = "Buffer full";
static const char_t stat_14[] = "Buffer full - fatal";
static const char_t stat_15[] = "Initializing";
static const char_t stat_16[] = "Entering boot loader";
static const char_t stat_17[] = "Function is stubbed";
static const char_t stat_18[] = "18";
static const char_t stat_19[] = "19";

static const char_t stat_20[] = "Internal error";
static const char_t stat_21[] = "Internal range error";
static const char_t stat_22[] = "Floating point error";
static const char_t stat_23[] = "Divide by zero";
static const char_t stat_24[] = "Invalid Address";
static const char_t stat_25[] = "Read-only address";
static const char_t stat_26[] = "Initialization failure";
static const char_t stat_27[] = "System shutdown";
static const char_t stat_28[] = "Memory corruption";
static const char_t stat_29[] = "29";
static const char_t stat_30[] = "30";
static const char_t stat_31[] = "31";
static const char_t stat_32[] = "32";
static const char_t stat_33[] = "33";
static const char_t stat_34[] = "34";
static const char_t stat_35[] = "35";
static const char_t stat_36[] = "36";
static const char_t stat_37[] = "37";
static const char_t stat_38[] = "38";
static const char_t stat_39[] = "39";

static const char_t stat_40[] = "Unrecognized command";
static const char_t stat_41[] = "Expected command letter";
static const char_t stat_42[] = "Bad number format";
static const char_t stat_43[] = "Input exceeds max length";
static const char_t stat_44[] = "Input value too small";
static const char_t stat_45[] = "Input value too large";
static const char_t stat_46[] = "Input value range error";
static const char_t stat_47[] = "Input value unsupported";
static const char_t stat_48[] = "JSON syntax error";
static const char_t stat_49[] = "JSON input has too many pairs";
static const char_t stat_50[] = "JSON output too long";
static const char_t stat_51[] = "Out of buffer space";
static const char_t stat_52[] = "52";
static const char_t stat_53[] = "53";
static const char_t stat_54[] = "54";
static const char_t stat_55[] = "55";
static const char_t stat_56[] = "56";
static const char_t stat_57[] = "57";
static const char_t stat_58[] = "58";
static const char_t stat_59[] = "59";

static const char_t stat_60[] = "Zero length move";
static const char_t stat_61[] = "Gcode block skipped";
static const char_t stat_62[] = "Gcode input error";
static const char_t stat_63[] = "Gcode feedrate error";
static const char_t stat_64[] = "Gcode axis word missing";
static const char_t stat_65[] = "Gcode modal group violation";
static const char_t stat_66[] = "Homing cycle failed";
static const char_t stat_67[] = "Max travel exceeded";
static const char_t stat_68[] = "Max spindle speed exceeded";
static const char_t stat_69[] = "Arc specification error";

static const char_t *stat_msg[] = {
	stat_00, stat_01, stat_02, stat_03, stat_04, stat_05, stat_06, stat_07, stat_08, stat_09,
	stat_10, stat_11, stat_12, stat_13, stat_14, stat_15, stat_16, stat_17, stat_18, stat_19,
	stat_20, stat_21, stat_22, stat_23, stat_24, stat_25, stat_26, stat_27, stat_28, stat_29,
	stat_30, stat_31, stat_32, stat_33, stat_34, stat_35, stat_36, stat_37, stat_38, stat_39,
	stat_40, stat_41, stat_42, stat_43, stat_44, stat_45, stat_46, stat_47, stat_48, stat_49,
	stat_50, stat_51, stat_52, stat_53, stat_54, stat_55, stat_56, stat_57, stat_58, stat_59,
	stat_60, stat_61, stat_62, stat_63, stat_64, stat_65, stat_66, stat_67, stat_68, stat_69
};

/*
 * rpt_exception() - generate an exception message
 */
void rpt_exception(uint8_t status, int16_t value)
{
	printf("{\"er\":{\"fb\":%0.2f,\"st\":%d,\"msg\":\"%s\",\"val\":%d}\n", 
		TINYG_FIRMWARE_BUILD, status, stat_msg[status], value);
}

/**** Application Messages *********************************************************
 * rpt_print_message() 				   - print a character string passed as argument
 * rpt_print_initializing_message()	   - initializing configs from hard-coded profile
 * rpt_print_loading_configs_message() - loading configs from EEPROM
 * rpt_print_system_ready_message()    - system ready message
 *
 *	These messages are always in JSON format to allow UIs to sync
 */

void _startup_helper(uint8_t status, const char *msg)
{
#ifndef __SUPPRESS_STARTUP_MESSAGES
	cmd_reset_list();
	cmd_add_object((const char_t *)"fb");
	cmd_add_object((const char_t *)"fv");
	cmd_add_object((const char_t *)"hv");
	cmd_add_object((const char_t *)"id");
	cmd_add_string((const char_t *)"msg", (char_t *)msg);
	json_print_response(status);
#endif
}

void rpt_print_initializing_message(void)
{
	_startup_helper(STAT_INITIALIZING, INIT_MESSAGE);
}

void rpt_print_loading_configs_message(void)
{
	_startup_helper(STAT_INITIALIZING, "Loading configs from EEPROM");
}

void rpt_print_system_ready_message(void)
{
	_startup_helper(STAT_OK, "SYSTEM READY");
	if (cfg.comm_mode == TEXT_MODE) { text_response(STAT_OK, (char_t *)"");}// prompt
}

/*****************************************************************************
 * Status Reports
 *
 *	Status report behaviors
 *
 *	Configuration:
 *
 *		Status reports are configurable only from JSON. SRs are configured
 *		by sending a status report SET object, e.g:
 *
 *		  {"sr":{"line":true,"posx":true,"posy":true....."motm":true,"stat":true}}
 *
 *	Status report formats: The following formats exist for status reports:
 *
 *	  -	JSON format: Returns a JSON object as above, but with the values filled in. 
 *		In JSON form all values are returned as numeric values or enumerations. 
 *		E.g. "posx" is returned as 124.523 and "unit" is returned as 0 for 
 *		inches (G20) and 1 for mm (G21).
 *
 *	  - CSV format: Returns a single line of comma separated token:value pairs.
 *		Values are returned as numeric values or English text.
 *		E.g. "posx" is still returned as 124.523 but "unit" is returned as 
 *		"inch" for inches (G20) and "mm" for mm (G21).
 *
 *	  - Multi-line format: Returns a multi-line report where each value occupies 
 *		one line. Each line contains explanatory English text. Enumerated values are
 *		returned as English text as per CSV form.
 *	
 *	Status report invocation: Status reports can be invoked in the following ways:
 *
 *	  - Ad-hoc request in JSON mode. Issue {"sr":""} (or equivalent). Returns a 
 *		JSON format report (wrapped in a response header, of course).
 *
 *	  - Automatic status reports in JSON mode. Returns JSON format reports 
 *		according to "si" setting.
 *
 *	  - Ad-hoc request in text mode. Triggered by sending ?<cr>. Returns status 
 *		report in multi-line format. Additionally, a line starting with ? will put 
 *		the system into text mode.
 *
 *	  - Automatic status reports in text mode return CSV format
 */

/* 
 * rpt_init_status_report()
 *
 *	Call this function to completely re-initialze the status report
 *	Sets SR list to hard-coded defaults and re-initializes sr values in NVM
 */
/*
void rpt_init_status_report()
{
	cmdObj_t *cmd = cmd_reset_list();	// used for status report persistence locations
	char sr_defaults[CMD_STATUS_REPORT_LEN][CMD_TOKEN_LEN+1] = { SR_DEFAULTS };	// see settings.h
	cm.status_report_counter = (cfg.status_report_interval / RTC_PERIOD);	// RTC fires every 10 ms

	cmd->index = cmd_get_index("","se00");				// set first SR persistence index
	for (uint8_t i=0; i < CMD_STATUS_REPORT_LEN ; i++) {
		if (sr_defaults[i][0] == NUL) break;			// quit on first blank array entry
		cfg.status_report_value[i] = -1234567;			// pre-load values with an unlikely number
		cmd->value = cmd_get_index("", sr_defaults[i]);	// load the index for the SR element
		cmd_set(cmd);
		cmd_persist(cmd);								// conditionally persist - automatic by cmd_persis()
		cmd->index++;									// increment SR NVM index
	}
}
*/
/* 
 * rpt_set_status_report() - interpret an sr setup string and return current report
 */
/*
stat_t rpt_set_status_report(cmdObj_t *cmd)
{
	uint8_t elements = 0;
	index_t status_report_list[CMD_STATUS_REPORT_LEN];
	memset(status_report_list, 0, sizeof(status_report_list));
	index_t sr_start = cmd_get_index("","se00");		// set first SR persistence index

	for (uint8_t i=0; i<CMD_STATUS_REPORT_LEN; i++) {
		if (((cmd = cmd->nx) == NULL) || (cmd->type == TYPE_EMPTY)) { break;}
		if ((cmd->type == TYPE_BOOL) && (cmd->value == true)) {
			status_report_list[i] = cmd->index;
			cmd->value = cmd->index;					// persist the index as the value
			cmd->index = sr_start + i;					// index of the SR persistence location
			cmd_persist(cmd);
			elements++;
		} else {
			return (STAT_INPUT_VALUE_UNSUPPORTED);
		}
	}
	if (elements == 0) { return (STAT_INPUT_VALUE_UNSUPPORTED);}
	memcpy(cfg.status_report_list, status_report_list, sizeof(status_report_list));
	rpt_populate_unfiltered_status_report();			// return current values
	return (STAT_OK);
}
*/
/* 
 * rpt_run_text_status_report()	- generate a text mode status report in multiline format
 * rpt_request_status_report()	- request a status report to run after minimum interval
 * rpt_status_report_rtc_callback()	- real-time clock downcount for minimum reporting interval
 * rpt_status_report_callback()	- main loop callback to send a report if one is ready
 *
 *	Status reports can be request from a number of sources including:
 *	  - direct request from command line in the form of ? or {"sr:""}
 *	  - timed requests during machining cycle
 *	  - filtered request after each Gcode block
 *
 *	Status reports are generally returned with minimal delay (from the controller callback), 
 *	but will not be provided more frequently than the status report interval
 */
/*
void rpt_run_text_status_report()
{
	rpt_populate_unfiltered_status_report();
	cmd_print_list(STAT_OK, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT);
}

void rpt_request_status_report()
{
	cm.status_report_request = true;
}

void rpt_status_report_rtc_callback() 		// called by 10ms real-time clock
{
	if (cm.status_report_counter != 0) { cm.status_report_counter--;} // stick at zero
}

stat_t rpt_status_report_callback() 		// called by controller dispatcher
{
	if ((cfg.status_report_verbosity == SR_OFF) || 
		(cm.status_report_counter != 0) ||
		(cm.status_report_request == false)) {
		return (STAT_NOOP);
	}
	if (cfg.status_report_verbosity == SR_FILTERED) {
		if (rpt_populate_filtered_status_report() == true) {
			cmd_print_list(STAT_OK, TEXT_INLINE_PAIRS, JSON_OBJECT_FORMAT);
		}
	} else {
		rpt_populate_unfiltered_status_report();
		cmd_print_list(STAT_OK, TEXT_INLINE_PAIRS, JSON_OBJECT_FORMAT);
	}
	cm.status_report_counter = (cfg.status_report_interval / RTC_PERIOD);	// reset minimum interval
	cm.status_report_request = false;
	return (STAT_OK);
}
*/
/*
 * rpt_populate_unfiltered_status_report() - populate cmdObj body with status values
 *
 *	Designed to be run as a response; i.e. have a "r" header and a footer.
 */
/*
void rpt_populate_unfiltered_status_report()
{
	char tmp[CMD_TOKEN_LEN+1];
	cmdObj_t *cmd = cmd_reset_list();		// sets *cmd to the start of the body
	cmd->type = TYPE_PARENT; 				// setup the parent object
	strcpy(cmd->token, "sr");
//	sprintf_P(cmd->token, PSTR("sr"));		// alternate form of above: less RAM, more FLASH & cycles
	cmd->index = cmd_get_index("","sr");	// set the index - may be needed by calling function
	cmd = cmd->nx;

	for (uint8_t i=0; i<CMD_STATUS_REPORT_LEN; i++) {
		if ((cmd->index = cfg.status_report_list[i]) == 0) { break;}
		cmd_get_cmdObj(cmd);
		strcpy(tmp, cmd->group);			// concatenate groups and tokens
		strcat(tmp, cmd->token);
		strcpy(cmd->token, tmp);
		cmd = cmd->nx;
	}
}
*/
/*
 * rpt_populate_filtered_status_report() - populate cmdObj body with status values
 *
 *	Designed to be displayed as a JSON object; i;e; no footer or header
 *	Returns 'true' if the report has new data, 'false' if there is nothing to report.
 *
 *	NOTE: Unlike rpt_populate_unfiltered_status_report(), this function does NOT set 
 *	the SR index, which is a relatively expensive operation. In current use this 
 *	doesn't matter, but if the caller assumes its set it may lead to a side-effect (bug)
 */
/*
uint8_t rpt_populate_filtered_status_report()
{
	uint8_t has_data = false;
	char tmp[CMD_TOKEN_LEN+1];
	cmdObj_t *cmd = cmd_reset_list();		// sets cmd to the start of the body

	cmd->type = TYPE_PARENT; 				// setup the parent object
	strcpy(cmd->token, "sr");
//	sprintf_P(cmd->token, PSTR("sr"));		// alternate form of above: less RAM, more FLASH & cycles
//	cmd->index = cmd_get_index("","sr");	// OMITTED - set the index - may be needed by calling function
	cmd = cmd->nx;

	for (uint8_t i=0; i<CMD_STATUS_REPORT_LEN; i++) {
		if ((cmd->index = cfg.status_report_list[i]) == 0) { break;}

		cmd_get_cmdObj(cmd);
		if (cfg.status_report_value[i] == cmd->value) {	// float == comparison runs the risk of overreporting. So be it
			continue;
		} else {
			strcpy(tmp, cmd->group);		// flatten out groups
			strcat(tmp, cmd->token);
			strcpy(cmd->token, tmp);
			cfg.status_report_value[i] = cmd->value;
			cmd = cmd->nx;
//			if (cmd == NULL) { return (false);}	// This is never supposed to happen
			has_data = true;
		}
	}
	cmd->pv->nx = NULL;						// back up one and terminate the body
	return (has_data);
}
*/
/*****************************************************************************
 * Queue Reports
 * rpt_request_queue_report()	- request a queue report with current values
 * rpt_queue_report_callback()	- run the queue report w/stored values
 */
/*
struct qrIndexes {				// static data for queue reports
	uint8_t request;			// set to true to request a report
	uint8_t buffers_available;	// stored value used by callback
	uint8_t prev_available;		// used to filter reports
};
static struct qrIndexes qr;

void rpt_request_queue_report() 
{ 
	if (cfg.queue_report_verbosity == QR_OFF) return;

	qr.buffers_available = mp_get_planner_buffers_available();

	// perform filtration for QR_FILTERED reports
	if (cfg.queue_report_verbosity == QR_FILTERED) {
		if (qr.buffers_available == qr.prev_available) {
			return;
		}
		if ((qr.buffers_available > cfg.queue_report_lo_water) && 	// e.g. > 2 buffers available
			(qr.buffers_available < cfg.queue_report_hi_water)) {	// e.g. < 20 buffers available
			return;
		}
	}
	qr.prev_available = qr.buffers_available;
	qr.request = true;
}

stat_t rpt_queue_report_callback()
{
	if (qr.request == false) { return (STAT_NOOP);}
	qr.request = false;

	// cget a clean cmd object
//	cmdObj_t *cmd = cmd_reset_list();		// normally you do a list reset but the following is more time efficient
	cmdObj_t *cmd = cmd_body;
	cmd_reset_obj(cmd);
	cmd->nx = NULL;							// terminate the list

	// make a qr object and print it
	sprintf_P(cmd->token, PSTR("qr"));
	cmd->value = qr.buffers_available;
	cmd->type = TYPE_INTEGER;
	cmd_print_list(STAT_OK, TEXT_INLINE_PAIRS, JSON_OBJECT_FORMAT);
	return (STAT_OK);
}
*/
/****************************************************************************
 ***** Report Unit Tests ****************************************************
 ****************************************************************************/

#ifdef __UNIT_TESTS
#ifdef __UNIT_TEST_REPORT

void sr_unit_tests(void)
{
	sr_init();
	tg.communications_mode = STAT_JSON_MODE;
	sr_run_status_report();
}
#endif
#endif
