/*
 * report.cpp - Status reports and other reporting functions
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart, Jr.
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

#include "g2core.h"
#include "config.h"
#include "report.h"
#include "controller.h"
#include "json_parser.h"
#include "text_parser.h"
#include "planner.h"
#include "settings.h"
#include "util.h"
#include "xio.h"


/**** Allocation ****/

srSingleton_t sr;
qrSingleton_t qr;

/**** Exception Reports ************************************************************
 *
 * rpt_exception() - generate an exception message - always in JSON format
 *
 * Returns incoming status value and a message to the exception.
 * Do not use global_string_buf[] for *msg or it will get trampled.
 *
 * WARNING: Do not call this function from MED or HI interrupts (LO is OK)
 *          or there is a potential for deadlock in the TX buffer.
 */

stat_t rpt_exception(stat_t status, const char *msg)
{
    if (status != STAT_OK) { // makes it possible to call exception reports w/o checking status value

        // you cannot send an exception report if the USB has not been set up. Causes a processor exception.
        if (cs.controller_state >= CONTROLLER_READY) {

            // always sends a strict string regardless of JS setting
           sprintf(global_string_buf, "{\"er\":{\"fb\":%0.2f,\"st\":%d,\"msg\":\"%s - %s\"}}\n",
                                        G2CORE_FIRMWARE_BUILD, status, get_status_message(status), msg);
           xio_writeline(global_string_buf);
        }
    }
    return (status);            // makes it possible to inline, e.g: return(rpt_exception(status));
}

/*
 * rpt_er()    - send a bogus exception report for testing purposes (it's not real)
 */
stat_t rpt_er(nvObj_t *nv)
{
    return(rpt_exception(STAT_GENERIC_EXCEPTION_REPORT, "bogus exception report")); // bogus exception report for testing
}

/**** Application Messages *********************************************************
 * rpt_print_initializing_message()       - initializing configs from hard-coded profile
 * rpt_print_loading_configs_message() - loading configs from EEPROM
 * rpt_print_system_ready_message()    - system ready message
 *
 *  These messages are always in JSON format to allow UIs to sync
 */

void _startup_helper(stat_t status, const char *msg)
{
#ifndef __SUPPRESS_STARTUP_MESSAGES
    nv_reset_nv_list();
    nv_add_object("fv");        // firmware version
    nv_add_object("fb");        // firmware build
    nv_add_object("fbs");       // firmware build string
    nv_add_object("fbc");       // settings configuration file used
    nv_add_object("hp");        // hardware platform
    nv_add_object("hv");        // hardware version
    nv_add_object("id");        // hardware ID
    nv_add_string("msg",msg);   // startup message
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
    if (cs.comm_mode == TEXT_MODE) { text_response(STAT_OK, (char *)"");}// prompt
}

/*****************************************************************************
 * Status Reports
 *
 *  Status report behaviors
 *
 *  Configuration:
 *
 *      Status reports are configurable only from JSON. SRs are configured
 *      by sending a status report SET object, e.g:
 *
 *        {"sr":{"line":true,"posx":true,"posy":true....."motm":true,"stat":true}}
 *
 *  Status report formats: The following formats exist for status reports:
 *
 *    - JSON format: Returns a JSON object as above, but with the values filled in.
 *      In JSON form all values are returned as numeric values or enumerations.
 *      E.g. "posx" is returned as 124.523 and "unit" is returned as 0 for
 *      inches (G20) and 1 for mm (G21).
 *
 *    - CSV format: Returns a single line of comma separated token:value pairs.
 *      Values are returned as numeric values or English text.
 *      E.g. "posx" is still returned as 124.523 but "unit" is returned as
 *      "inch" for inches (G20) and "mm" for mm (G21).
 *
 *    - Multi-line format: Returns a multi-line report where each value occupies
 *      one line. Each line contains explanatory English text. Enumerated values are
 *      returned as English text as per CSV form.
 *
 *  Status report invocation: Status reports can be invoked in the following ways:
 *
 *    - Ad-hoc request in JSON mode. Issue {"sr":""} (or equivalent). Returns a
 *      JSON format report (wrapped in a response header, of course).
 *
 *    - Automatic status reports in JSON mode. Returns JSON format reports
 *      according to "si" setting.
 *
 *    - Ad-hoc request in text mode. Triggered by sending ?<cr>. Returns status
 *      report in multi-line format. Additionally, a line starting with ? will put
 *      the system into text mode.
 *
 *    - Automatic status reports in text mode return CSV format according to si setting
 */
static stat_t _populate_unfiltered_status_report(void);
static uint8_t _populate_filtered_status_report(void);

uint8_t _is_stat(nvObj_t *nv)
{
    char tok[TOKEN_LEN+1];

    GET_TOKEN_STRING(nv->value, tok);
    if (strcmp(tok, "stat") == 0) { return (true);}
    return (false);
}

/*
 * sr_init_status_report()
 *
 *  Call this function to completely re-initialize the status report
 *  Sets SR list to hard-coded defaults and re-initializes SR values in NVM
 */

void sr_init_status_report()
{
    nvObj_t *nv = nv_reset_nv_list();    // used for status report persistence locations
    sr.status_report_request = SR_OFF;
    char sr_defaults[NV_STATUS_REPORT_LEN][TOKEN_LEN+1] = { STATUS_REPORT_DEFAULTS };
    nv->index = nv_get_index((const char *)"", (char *)"se00");    // set first SR persistence index
    sr.stat_index = nv_get_index((const char *)"", (const char *)"stat");

    for (uint8_t i=0; i < NV_STATUS_REPORT_LEN ; i++) {
        if (sr_defaults[i][0] == NUL) break;                    // quit on first blank array entry
        sr.status_report_value[i] = -1234567;                   // pre-load values with an unlikely number
        nv->value = nv_get_index((const char *)"", sr_defaults[i]);// load the index for the SR element
        if (fp_EQ(nv->value, NO_MATCH)) {
            rpt_exception(STAT_BAD_STATUS_REPORT_SETTING, "sr_init_status_report() encountered bad SR setting"); // trap mis-configured profile settings
            return;
        }
        nv_set(nv);
        nv_persist(nv);                                         // conditionally persist - automatic by nv_persist()
        nv->index++;                                            // increment SR NVM index
    }
    // record the index of the "stat" variable so we can use it during reporting
    sr.index_of_stat_variable = nv_get_index((const char *)"", (const char *)"stat");
}

/*
 * sr_set_status_report() - interpret an SR setup string and return current report
 *
 *  Note: By the time this function is called any unrecognized tokens have been detected and
 *  rejected by the JSON or text parser. In other words, it should never get to here if
 *  there is an unrecognized token in the SR string.
 */

stat_t sr_set_status_report(nvObj_t *nv)
{
    uint8_t elements = 0;
    index_t status_report_list[NV_STATUS_REPORT_LEN];
    memset(status_report_list, 0, sizeof(status_report_list));
    index_t sr_start = nv_get_index((const char *)"",(const char *)"se00");// set first SR persistence index

    for (uint8_t i=0; i<NV_STATUS_REPORT_LEN; i++) {
        if (((nv = nv->nx) == NULL) || (nv->valuetype == TYPE_EMPTY)) {
            break;
        }
        if ((nv->valuetype == TYPE_BOOL) && (fp_TRUE(nv->value))) {
            status_report_list[i] = nv->index;
            nv->value = nv->index;                          // persist the index as the value
            nv->index = sr_start + i;                       // index of the SR persistence location
            nv_persist(nv);
            elements++;
        } else {
            return (STAT_UNSUPPORTED_TYPE);
        }
    }
    if (elements == 0) {
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    memcpy(sr.status_report_list, status_report_list, sizeof(status_report_list));
    return(_populate_unfiltered_status_report());            // return current values
}

/*
 * sr_request_status_report() - request a status report
 *
 *  Status reports can be request from a number of sources including:
 *    - direct request from command line in the form of ? or {"sr:""}
 *    - timed requests during machining cycle
 *    - filtered request after each Gcode block
 *
 *  Status reports are generally returned with minimal delay (from the controller callback),
 *  but will not be provided more frequently than the status report interval
 *
 *  Requests can specify immediate or timed reports, and can also force a filtered or full report.
 *  See cmStatusReportRequest enum in report.h for details.
 */

stat_t sr_request_status_report(cmStatusReportRequest request_type)
{
    if (sr.status_report_request != SR_OFF) {       // ignore multiple requests. First one wins.
        return (STAT_OK);
   }

    sr.status_report_systick = SysTickTimer_getValue();
    if (request_type == SR_REQUEST_IMMEDIATE) {
        sr.status_report_request = SR_FILTERED;         // will trigger a filtered or verbose report depending on verbosity setting

    } else if (request_type == SR_REQUEST_IMMEDIATE_FULL) {
        sr.status_report_request = SR_VERBOSE;        // will always trigger verbose report, regardless of verbosity setting

    } else if (request_type == SR_REQUEST_TIMED) {
        sr.status_report_request = sr.status_report_verbosity;
        sr.status_report_systick += sr.status_report_interval;

    } else {
        sr.status_report_request = SR_VERBOSE;
        sr.status_report_systick += sr.status_report_interval;
    }
    return (STAT_OK);
}

/*
 * sr_status_report_callback() - main loop callback to send a report if one is ready
 */
stat_t sr_status_report_callback()         // called by controller dispatcher
{
    // conditions where autogenerated SRs will not be returned
    if ((sr.status_report_request == SR_OFF) ||
        (sr.status_report_verbosity == SR_OFF) ||
        (SysTickTimer_getValue() < sr.status_report_systick) ) {
        return (STAT_NOOP);
    }

   // don't send an SR if you the planner is experiencing a time constraint
   if (!mp_is_phat_city_time()) {
        if (++sr.throttle_counter != SR_THROTTLE_COUNT) {
            return (STAT_NOOP);
        }
        sr.throttle_counter = 0;
    }

    sr.status_report_request = SR_OFF;
    if (sr.status_report_request == SR_VERBOSE) {
        _populate_unfiltered_status_report();
    } else {
        if (_populate_filtered_status_report() == false) {    // no new data
            return (STAT_OK);
        }
    }
    nv_print_list(STAT_OK, TEXT_MULTILINE_FORMATTED, JSON_OBJECT_FORMAT);
    return (STAT_OK);
}

/*
 * sr_run_text_status_report() - generate a text mode status report in multiline format
 */
stat_t sr_run_text_status_report()
{
    _populate_unfiltered_status_report();
    nv_print_list(STAT_OK, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT);
    return (STAT_OK);
}

/*
 * _populate_unfiltered_status_report() - populate nvObj body with status values
 *
 *  Designed to be run as a response; i.e. have a "r" header and a footer.
 */
static stat_t _populate_unfiltered_status_report()
{
    const char sr_str[] = "sr";
    char tmp[TOKEN_LEN+1];
    nvObj_t *nv = nv_reset_nv_list();        // sets *nv to the start of the body

    nv->valuetype = TYPE_PARENT;             // setup the parent object (no length checking required)
    strcpy(nv->token, sr_str);
    nv->index = nv_get_index((const char *)"", sr_str);// set the index - may be needed by calling function
    nv = nv->nx;                            // no need to check for NULL as list has just been reset

    for (uint8_t i=0; i<NV_STATUS_REPORT_LEN; i++) {
        if ((nv->index = sr.status_report_list[i]) == 0) { break;}
        nv_get_nvObj(nv);

        strcpy(tmp, nv->group);            // flatten out groups - WARNING - you cannot use strncpy here...
        strcat(tmp, nv->token);
        strcpy(nv->token, tmp);            //...or here.

        if ((nv = nv->nx) == NULL) {
            return (cm_panic(STAT_BUFFER_FULL_FATAL, "_populate_unfiltered_status_report() sr link NULL"));    // should never be NULL unless SR length exceeds available buffer array
        }
    }
    return (STAT_OK);
}

/*
 * _populate_filtered_status_report() - populate nvObj body with status values
 *
 *  Designed to be displayed as a JSON object; i.e. no footer or header
 *  Returns 'true' if the report has new data, 'false' if there is nothing to report.
 *
 *  NOTE: Unlike sr_populate_unfiltered_status_report(), this function does NOT set
 *  the SR index, which is a relatively expensive operation. In current use this
 *  doesn't matter, but if the caller assumes its set it may lead to a side-effect (bug)
 *
 *  NOTE: Room for improvement - look up the SR index initially and cache it, use the
 *        cached value for all remaining reports.
 */
static uint8_t _populate_filtered_status_report()
{
    const char sr_str[] = "sr";
    bool has_data = false;
    char tmp[TOKEN_LEN+1];
    nvObj_t *nv = nv_reset_nv_list();           // sets nv to the start of the body

    nv->valuetype = TYPE_PARENT;                // setup the parent object (no need to length check the copy)
    strcpy(nv->token, sr_str);
//    nv->index = nv_get_index((const char *)"", sr_str);// OMITTED - set the index - may be needed by calling function
    nv = nv->nx;                                // no need to check for NULL as list has just been reset

    for (uint8_t i=0; i<NV_STATUS_REPORT_LEN; i++) {
        if ((nv->index = sr.status_report_list[i]) == 0) {  // end of list
            break;
        }
        nv_get_nvObj(nv);

        // report values that have changed by more than 0.0001, but always stops and ends
        if ((fabs(nv->value - sr.status_report_value[i]) > EPSILON3) ||
            ((nv->index == sr.stat_index) && fp_EQ(nv->value, COMBINED_PROGRAM_STOP)) ||
            ((nv->index == sr.stat_index) && fp_EQ(nv->value, COMBINED_PROGRAM_END))) {

            strcpy(tmp, nv->group);            // flatten out groups - WARNING - you cannot use strncpy here...
            strcat(tmp, nv->token);
            strcpy(nv->token, tmp);            //...or here.
            sr.status_report_value[i] = nv->value;
            if ((nv = nv->nx) == NULL) return (false);    // should never be NULL unless SR length exceeds available buffer array
            has_data = true;

        } else {
            nv->valuetype = TYPE_EMPTY;     // filter this value out of the report
        }
    }
    return (has_data);
}


/****************************
 * END OF REPORT FUNCTIONS *
 ****************************/

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/*
 * Wrappers and Setters - for calling from nvArray table
 *
 * sr_get()        - run status report
 * sr_set()        - set status report elements
 * sr_set_si()    - set status report interval
 */

stat_t sr_get(nvObj_t *nv)
{
    return (_populate_unfiltered_status_report());
}

stat_t sr_set(nvObj_t *nv)
{
    return (sr_set_status_report(nv));
}

stat_t sr_set_si(nvObj_t *nv)
{
    if (nv->value < STATUS_REPORT_MIN_MS) {
        nv->value = STATUS_REPORT_MIN_MS;
    }
    set_int(nv);
    return(STAT_OK);
}

/*********************
 * TEXT MODE SUPPORT *
 *********************/
#ifdef __TEXT_MODE

static const char fmt_si[] = "[si]  status interval%14d ms\n";
static const char fmt_sv[] = "[sv]  status report verbosity%6d [0=off,1=filtered,2=verbose]\n";

void sr_print_sr(nvObj_t *nv) { _populate_unfiltered_status_report();}
void sr_print_si(nvObj_t *nv) { text_print(nv, fmt_si);}
void sr_print_sv(nvObj_t *nv) { text_print(nv, fmt_sv);}

#endif // __TEXT_MODE


/*****************************************************************************
 * Queue Reports
 *
 *  Queue reports can report three values:
 *    - qr    queue depth - # of buffers availabel in planner queue
 *    - qi    buffers added to planner queue since las report
 *    - qo    buffers removed from planner queue since last report
 *
 *  A QR_SINGLE report returns qr only. A QR_TRIPLE returns all 3 values
 *
 *  There are 2 ways to get queue reports:
 *
 *   1. Enable single or triple queue reports using the QV variable. This will
 *      return a queue report every time the buffer depth changes
 *
 *   2. Add qr, qi and qo (or some combination) to the status report. This will
 *      return queue report data when status reports are generated.
 */
/*
 * qr_init_queue_report() - initialize or clear queue report values
 */

void qr_init_queue_report()
{
    qr.queue_report_requested = false;
    qr.buffers_added = 0;
    qr.buffers_removed = 0;
    qr.init_tick = SysTickTimer_getValue();        // Uses C mapping of SysTickTimer.getValue();
}

/*
 * qr_request_queue_report() - request a queue report
 *
 *  Requests a queue report and also records the buffers added and removed
 *  since the last init (usually re-initted when a report is generated).
 */

void qr_request_queue_report(int8_t buffers)
{
    // get buffer depth and added/removed count
    qr.buffers_available = mp_get_planner_buffers();
    if (buffers > 0) {
        qr.buffers_added += buffers;
    } else {
        qr.buffers_removed -= buffers;
    }

    // time-throttle requests while generating arcs
    qr.motion_mode = cm_get_motion_mode(ACTIVE_MODEL);
    if ((qr.motion_mode == MOTION_MODE_CW_ARC) || (qr.motion_mode == MOTION_MODE_CCW_ARC)) {
        uint32_t tick = SysTickTimer_getValue();
        if (tick - qr.init_tick < MIN_ARC_QR_INTERVAL) {
            qr.queue_report_requested = false;
            return;
        }
    }

    // either return or request a report
    if (qr.queue_report_verbosity != QR_OFF) {
        qr.queue_report_requested = true;
    }
}

/*
 * qr_queue_report_callback() - generate a queue report if one has been requested
 */

stat_t qr_queue_report_callback()         // called by controller dispatcher
{
    if ((qr.queue_report_verbosity == QR_OFF) ||
        (js.json_verbosity == JV_SILENT) ||
        (qr.queue_report_requested == false) ||
        (!mp_is_phat_city_time())) {
        return (STAT_NOOP);
    }

    qr.queue_report_requested = false;

    char report[32];    // we know these reports can't be longer than 30 bytes

    if (cs.comm_mode == TEXT_MODE) {
        if (qr.queue_report_verbosity == QR_SINGLE) {
            sprintf(report, "qr:%d\n", qr.buffers_available);
        } else  {
            sprintf(report, "qr:%d, qi:%d, qo:%d\n", qr.buffers_available,qr.buffers_added,qr.buffers_removed);
        }

    } else if (js.json_syntax == JSON_SYNTAX_RELAXED) {
        if (qr.queue_report_verbosity == QR_SINGLE) {
            sprintf(report, "{qr:%d}\n", qr.buffers_available);
        } else {
            sprintf(report, "{qr:%d,qi:%d,qo:%d}\n", qr.buffers_available, qr.buffers_added,qr.buffers_removed);
        }

    } else {
        if (qr.queue_report_verbosity == QR_SINGLE) {
            sprintf(report, "{\"qr\":%d}\n", qr.buffers_available);
        } else {
            sprintf(report, "{\"qr\":%d,\"qi\":%d,\"qo\":%d}\n", qr.buffers_available, qr.buffers_added,qr.buffers_removed);
        }
    }
    xio_writeline(report);
    qr_init_queue_report();
    return (STAT_OK);
}

/* Alternate Formulation for a Single report - using nvObj list

    // get a clean nv object
//    nvObj_t *nv = nv_reset_nv_list();        // normally you do a list reset but the following is more time efficient
    nvObj_t *nv = nv_body;
    nv_reset_nv(nv);
    nv->nx = NULL;                            // terminate the list

    // make a qr object and print it
    sprintf(nv->token, "qr");
    nv->value = qr.buffers_available;
    nv->valuetype = TYPE_INT;
    nv_print_list(STAT_OK, TEXT_INLINE_PAIRS, JSON_OBJECT_FORMAT);
    return (STAT_OK);
*/

/*
 * Wrappers and Setters - for calling from cfgArray table
 *
 * qr_get() - run a queue report (as data)
 * qi_get() - run a queue report - buffers in
 * qo_get() - run a queue report - buffers out
 */
stat_t qr_get(nvObj_t *nv)
{
    nv->value = (float)mp_get_planner_buffers(); // ensure that manually requested QR count is always up to date
    nv->valuetype = TYPE_INT;
    return (STAT_OK);
}

stat_t qi_get(nvObj_t *nv)
{
    nv->value = (float)qr.buffers_added;
    nv->valuetype = TYPE_INT;
    qr.buffers_added = 0;                // reset it
    return (STAT_OK);
}

stat_t qo_get(nvObj_t *nv)
{
    nv->value = (float)qr.buffers_removed;
    nv->valuetype = TYPE_INT;
    qr.buffers_removed = 0;                // reset it
    return (STAT_OK);
}

/*****************************************************************************
 * JOB ID REPORTS
 *
 *  job_populate_job_report()
 *  job_set_job_report()
 *  job_report_callback()
 *  job_get()
 *  job_set()
 *  job_print_job()
 */

stat_t job_populate_job_report()
{
    const char job_str[] = "job";
    char tmp[TOKEN_LEN+1];
    nvObj_t *nv = nv_reset_nv_list();        // sets *nv to the start of the body

    nv->valuetype = TYPE_PARENT;             // setup the parent object
    strcpy(nv->token, job_str);

    //nv->index = nv_get_index((const char *)"", job_str);// set the index - may be needed by calling function
    nv = nv->nx;                            // no need to check for NULL as list has just been reset

    index_t job_start = nv_get_index((const char *)"",(const char *)"job1");// set first job persistence index
    for (uint8_t i=0; i<4; i++) {

        nv->index = job_start + i;
        nv_get_nvObj(nv);

        strcpy(tmp, nv->group);                // concatenate groups and tokens - do NOT use strncpy()
        strcat(tmp, nv->token);
        strcpy(nv->token, tmp);

        if ((nv = nv->nx) == NULL) return (STAT_OK); // should never be NULL unless SR length exceeds available buffer array
    }
    return (STAT_OK);
}

stat_t job_set_job_report(nvObj_t *nv)
{
    index_t job_start = nv_get_index((const char *)"",(const char *)"job1");// set first job persistence index

    for (uint8_t i=0; i<4; i++) {
        if (((nv = nv->nx) == NULL) || (nv->valuetype == TYPE_EMPTY)) { break;}
        if (nv->valuetype == TYPE_INT) {
            cfg.job_id[i] = nv->value;
            nv->index = job_start + i;        // index of the SR persistence location
            nv_persist(nv);
        } else {
            return (STAT_UNSUPPORTED_TYPE);
        }
    }
    job_populate_job_report();                // return current values
    return (STAT_OK);
}

uint8_t job_report_callback()
{
    if (cs.comm_mode == TEXT_MODE) {
        // no-op, job_ids are client app state
        return (STAT_OK);
    }

    if (js.json_syntax == JSON_SYNTAX_RELAXED) {
        sprintf(cs.out_buf, "{job:[%lu,%lu,%lu,%lu]}\n", cfg.job_id[0], cfg.job_id[1], cfg.job_id[2], cfg.job_id[3] );
        xio_writeline(cs.out_buf);
    } else {
        sprintf(cs.out_buf, "{\"job\":[%lu,%lu,%lu,%lu]}\n", cfg.job_id[0], cfg.job_id[1], cfg.job_id[2], cfg.job_id[3] );
        xio_writeline(cs.out_buf);
        //job_clear_report();
    }
    return (STAT_OK);
}

stat_t job_get(nvObj_t *nv) { return (job_populate_job_report());}
stat_t job_set(nvObj_t *nv) { return (job_set_job_report(nv));}
void job_print_job(nvObj_t *nv) { job_populate_job_report();}

/*********************
 * TEXT MODE SUPPORT *
 *********************/
#ifdef __TEXT_MODE

static const char fmt_qr[] = "qr:%d\n";
static const char fmt_qi[] = "qi:%d\n";
static const char fmt_qo[] = "qo:%d\n";
static const char fmt_qv[] = "[qv]  queue report verbosity%7d [0=off,1=single,2=triple]\n";

void qr_print_qr(nvObj_t *nv) { text_print(nv, fmt_qr);}    // TYPE_INT
void qr_print_qi(nvObj_t *nv) { text_print(nv, fmt_qi);}    // TYPE_INT
void qr_print_qo(nvObj_t *nv) { text_print(nv, fmt_qo);}    // TYPE_INT
void qr_print_qv(nvObj_t *nv) { text_print(nv, fmt_qv);}    // TYPE_INT

#endif // __TEXT_MODE
