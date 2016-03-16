/*
 * controller.cpp - tinyg controller and top level parser
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2015 Robert Giseburt
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

#include "tinyg2.h"				// #1
#include "config.h"				// #2
#include "controller.h"
#include "json_parser.h"
#include "text_parser.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "plan_arc.h"
#include "planner.h"
#include "stepper.h"
#include "encoder.h"
#include "hardware.h"
#include "gpio.h"
#include "report.h"
#include "help.h"
#include "util.h"
#include "xio.h"
#include "settings.h"

#ifdef __ARM
#include "Reset.h"
#endif

/***********************************************************************************
 **** STRUCTURE ALLOCATIONS *********************************************************
 ***********************************************************************************/

controller_t cs;		// controller state structure

/***********************************************************************************
 **** STATICS AND LOCALS ***********************************************************
 ***********************************************************************************/

static void _controller_HSM(void);
static stat_t _led_indicator(void);             // twiddle the LED indicator
static stat_t _shutdown_handler(void);          // new (replaces _interlock_estop_handler)
static stat_t _interlock_handler(void);         // new (replaces _interlock_estop_handler)
static stat_t _limit_switch_handler(void);      // revised for new GPIO code

static void _init_assertions(void);
static stat_t _test_assertions(void);
static stat_t _test_system_assertions(void);

static stat_t _sync_to_planner(void);
static stat_t _sync_to_tx_buffer(void);
static stat_t _dispatch_command(void);
static stat_t _dispatch_control(void);
static void _dispatch_kernel(void);
static stat_t _controller_state(void);          // manage controller state transitions

/***********************************************************************************
 **** CODE *************************************************************************
 ***********************************************************************************/
/*
 * controller_init() - controller init
 */

void controller_init(uint8_t std_in, uint8_t std_out, uint8_t std_err)
{
    // preserve settable parameters that may have already been set up
    uint8_t comm_mode = cs.comm_mode;
    uint8_t network_mode = cs.network_mode;

	memset(&cs, 0, sizeof(controller_t));           // clear all values, job_id's, pointers and status
	_init_assertions();

    cs.comm_mode = comm_mode;                       // restore parameters
    cs.network_mode = network_mode;

	cs.fw_build = TINYG_FIRMWARE_BUILD;             // set up identification
	cs.fw_version = TINYG_FIRMWARE_VERSION;
	cs.config_version = TINYG_CONFIG_VERSION;
	cs.hw_platform = TINYG_HARDWARE_PLATFORM;       // NB: HW version is set from EEPROM
	cs.controller_state = CONTROLLER_STARTUP;       // ready to run startup lines

#ifdef __AVR
	xio_set_stdin(std_in);
	xio_set_stdout(std_out);
	xio_set_stderr(std_err);
	xio.default_src = std_in;
	controller_set_primary_source(xio.default_src);
#endif

#ifdef __ARM
	IndicatorLed.setFrequency(100000);
#endif
}

/*
 * controller_run() - MAIN LOOP - top-level controller
 *
 * The order of the dispatched tasks is very important.
 * Tasks are ordered by increasing dependency (blocking hierarchy).
 * Tasks that are dependent on completion of lower-level tasks must be
 * later in the list than the task(s) they are dependent upon.
 *
 * Tasks must be written as continuations as they will be called repeatedly,
 * and are called even if they are not currently active.
 *
 * The DISPATCH macro calls the function and returns to the controller parent
 * if not finished (STAT_EAGAIN), preventing later routines from running
 * (they remain blocked). Any other condition - OK or ERR - drops through
 * and runs the next routine in the list.
 *
 * A routine that had no action (i.e. is OFF or idle) should return STAT_NOOP
 */

void controller_run()
{
	while (true) {
		_controller_HSM();
	}
}

#define	DISPATCH(func) if (func == STAT_EAGAIN) return;
static void _controller_HSM()
{
//----- Interrupt Service Routines are the highest priority controller functions ----//
//      See hardware.h for a list of ISRs and their priorities.
//
//----- kernel level ISR handlers ----(flags are set in ISRs)------------------------//
                                                // Order is important:
	DISPATCH(_led_indicator());				    // blink LEDs at the current rate
    DISPATCH(_shutdown_handler());              // invoke shutdown
 	DISPATCH(_interlock_handler());             // invoke / remove safety interlock
	DISPATCH(_limit_switch_handler());          // invoke limit switch
    DISPATCH(_controller_state());              // controller state management
	DISPATCH(_test_system_assertions());        // system integrity assertions
	DISPATCH(_dispatch_control());              // read any control messages prior to executing cycles

//----- planner hierarchy for gcode and cycles ---------------------------------------//

    DISPATCH(st_motor_power_callback());        // stepper motor power sequencing
#ifdef __AVR
    DISPATCH(switch_debounce_callback());       // debounce switches
#endif
    DISPATCH(sr_status_report_callback());      // conditionally send status report
    DISPATCH(qr_queue_report_callback());       // conditionally send queue report
    DISPATCH(rx_report_callback());             // conditionally send rx report

    DISPATCH(cm_feedhold_sequencing_callback());// feedhold state machine runner
    DISPATCH(mp_plan_buffer());		            // attempt to plan unplanned moves (conditionally)
    DISPATCH(cm_arc_callback());                // arc generation runs as a cycle above lines
    DISPATCH(cm_homing_cycle_callback());       // homing cycle operation (G28.2)
    DISPATCH(cm_probing_cycle_callback());      // probing cycle operation (G38.2)
    DISPATCH(cm_jogging_cycle_callback());      // jog cycle operation
//    DISPATCH(st_motor_power_callback());        // stepper motor power sequencing
    DISPATCH(cm_deferred_write_callback());     // persist G10 changes when not in machining cycle

//----- command readers and parsers --------------------------------------------------//

    DISPATCH(_sync_to_planner());               // ensure there is at least one free buffer in planning queue
    DISPATCH(_sync_to_tx_buffer());             // sync with TX buffer (pseudo-blocking)
#ifdef __AVR
    DISPATCH(set_baud_callback());              // perform baud rate update (must be after TX sync)
#endif
    DISPATCH(_dispatch_command());              // MUST BE LAST - read and execute next command
}

/*
 * controller_set_connected(bool) - hook for xio to tell the controller that we
 * have/don't have a connection.
 */

void controller_set_connected(bool is_connected) {
    if (is_connected) {
        cs.controller_state = CONTROLLER_CONNECTED; // we JUST connected
    } else {  // we just disconnected from the last device, we'll expect a banner again
        cs.controller_state = CONTROLLER_NOT_CONNECTED;
    }
}

/*
 * controller_parse_control() - return true if command is a control (versus data)
 * Note: parsing for control is somewhat naiive. This will need to get better
 */

bool controller_parse_control(char *p) {
    if (strchr("{$?!~%Hh", *p) != NULL) {		    // a match indicates control line
        return (true);
    }
    return (false);
}

/*
 * controller_reset_source() 		 - reset source to default input device (see note)
 * controller_set_primary_source() 	 - set current primary input source
 * controller_set_secondary_source() - set current primary input source
 *
 * Note: Once multiple serial devices are supported reset_source() should be expanded to
 * also set the stdout/stderr console device so the prompt and other messages are sent
 * to the active device.
 */
#ifdef __AVR
void controller_reset_source() { controller_set_primary_source(xio.default_src);}
void controller_set_primary_source(uint8_t dev) { xio.primary_src = dev;}
void controller_set_secondary_source(uint8_t dev) { xio.secondary_src = dev;}
#endif

/*****************************************************************************
 * command dispatchers
 * _dispatch_control - entry point for control-only dispatches
 * _dispatch_command - entry point for control and data dispatches
 * _dispatch_kernel - core dispatch routines
 *
 *	Reads next command line and dispatches to relevant parser or action
 *
 *  Note: The dispatchers must only read and process a single line from the
 *        RX queue before returning control to the main loop.
 */

static stat_t _dispatch_control()
{
    if (cs.controller_state != CONTROLLER_PAUSED) {
        devflags_t flags = DEV_IS_CTRL;
        if ((cs.bufp = xio_readline(flags, cs.linelen)) != NULL) {
            _dispatch_kernel();
        }
    }
    return (STAT_OK);
}

static stat_t _dispatch_command()
{
    if (cs.controller_state != CONTROLLER_PAUSED) {
        devflags_t flags = DEV_IS_BOTH;
        if ((mp_get_planner_buffers_available() > PLANNER_BUFFER_HEADROOM) &&
            (cs.bufp = xio_readline(flags, cs.linelen)) != NULL) {
            _dispatch_kernel();
            mp_plan_buffer();   // +++ removed for test. This is called from the main loop
        }
    }
	return (STAT_OK);
}

static void _dispatch_kernel()
{
    while ((*cs.bufp == SPC) || (*cs.bufp == TAB)) {        // position past any leading whitespace
        cs.bufp++;
    }
	strncpy(cs.saved_buf, cs.bufp, SAVED_BUFFER_LEN-1);		// save input buffer for reporting

	if (*cs.bufp == NUL) {									// blank line - just a CR or the 2nd termination in a CRLF
		if (cs.comm_mode == TEXT_MODE) {
			text_response(STAT_OK, cs.saved_buf);
            return;
		}
    }

	// trap single character commands
    if      (*cs.bufp == '!') { cm_request_feedhold(); }
    else if (*cs.bufp == '%') { cm_request_queue_flush(); }
	else if (*cs.bufp == '~') { cm_request_end_hold(); }
    else if (*cs.bufp == EOT) { cm_alarm(STAT_KILL_JOB, NULL); }
    else if (*cs.bufp == CAN) { hw_hard_reset(); }          // reset immediately

	else if (*cs.bufp == '{') {                             // process as JSON mode
		cs.comm_mode = JSON_MODE;                           // switch to JSON mode
		json_parser(cs.bufp);
    }
#ifdef __TEXT_MODE
    else if (strchr("$?Hh", *cs.bufp) != NULL) {            // process as text mode
		cs.comm_mode = TEXT_MODE;                           // switch to text mode
		text_response(text_parser(cs.bufp), cs.saved_buf);
    }
	else if (cs.comm_mode == TEXT_MODE) {                   // anything else is interpreted as Gcode
        text_response(gcode_parser(cs.bufp), cs.saved_buf);
    }
#endif
	else {  // anything else is interpreted as Gcode
        strncpy(cs.out_buf, cs.bufp, (USB_LINE_BUFFER_SIZE-11)); // use out_buf as temp; '-11' is buffer for JSON chars
        sprintf((char *)cs.bufp,"{\"gc\":\"%s\"}\n", (char *)cs.out_buf);  // Read and toss if machine is alarmed
        json_parser(cs.bufp);
	}
}

/**** Local Functions ********************************************************/
/*
 * _controller_state() - manage controller connection, startup, and other state changes
 */

Motate::Timeout _connection_timeout;
static stat_t _controller_state()
{
	if (cs.controller_state == CONTROLLER_CONNECTED) {		// first time through after reset
        cs.controller_state = CONTROLLER_STARTUP;
        // This is here just to put a small delay in before the startup message.
        _connection_timeout.set(10);
    } else if ((cs.controller_state == CONTROLLER_STARTUP) && (_connection_timeout.isPast())) {		// first time through after reset
		cs.controller_state = CONTROLLER_READY;
		rpt_print_system_ready_message();
	}
	return (STAT_OK);
}

/*
 * _led_indicator() - blink an LED to show it we are normal, alarmed, or shut down
 */
static stat_t _led_indicator()
{
    uint32_t blink_rate;
    if (cm_get_machine_state() == MACHINE_ALARM) {
        blink_rate = LED_ALARM_BLINK_RATE;
    } else if (cm_get_machine_state() == MACHINE_SHUTDOWN) {
        blink_rate = LED_SHUTDOWN_BLINK_RATE;
    } else if (cm_get_machine_state() == MACHINE_PANIC) {
        blink_rate = LED_PANIC_BLINK_RATE;
    } else {
        blink_rate = LED_NORMAL_BLINK_RATE;
    }

    if (blink_rate != cs.led_blink_rate) {
        cs.led_blink_rate =  blink_rate;
        cs.led_timer = 0;
    }
	if (SysTickTimer_getValue() > cs.led_timer) {
		cs.led_timer = SysTickTimer_getValue() + cs.led_blink_rate;
		IndicatorLed.toggle();
	}
	return (STAT_OK);
}

/*
 * _sync_to_tx_buffer() - return eagain if TX queue is backed up
 * _sync_to_planner() - return eagain if planner is not ready for a new command
 */
static stat_t _sync_to_tx_buffer()
{
#ifdef __AVR
	if ((xio_get_tx_bufcount_usart(ds[XIO_DEV_USB].x) >= XOFF_TX_LO_WATER_MARK)) {
		return (STAT_EAGAIN);
	}
#endif
	return (STAT_OK);
}

static stat_t _sync_to_planner()
{
	if (mp_get_planner_buffers_available() < PLANNER_BUFFER_HEADROOM) { // allow up to N planner buffers for this line
		return (STAT_EAGAIN);
	}
	return (STAT_OK);
}

/* ALARM STATE HANDLERS
 *
 * _shutdown_handler() - put system into shutdown state
 * _limit_switch_handler() - shut down system if limit switch fired
 * _interlock_handler() - feedhold and resume depending on edge
 *
 *	Some handlers return EAGAIN causing the control loop to never advance beyond that point.
 *
 * _interlock_handler() reacts the follwing ways:
 *   - safety_interlock_requested == INPUT_EDGE_NONE is normal operation (no interlock)
 *   - safety_interlock_requested == INPUT_EDGE_LEADING is interlock onset
 *   - safety_interlock_requested == INPUT_EDGE_TRAILING is interlock offset
 */
static stat_t _shutdown_handler(void)
{
    if (cm.shutdown_requested != 0) {  // request may contain the (non-zero) input number
	    char msg[10];
	    sprintf_P(msg, PSTR("input %d"), (int)cm.shutdown_requested);
	    cm.shutdown_requested = false; // clear limit request used here ^
        cm_shutdown(STAT_SHUTDOWN, msg);
    }
    return(STAT_OK);
}

static stat_t _limit_switch_handler(void)
{
    if ((cm.limit_enable == true) && (cm.limit_requested != 0)) {
	    char msg[10];
	    sprintf_P(msg, PSTR("input %d"), (int)cm.limit_requested);
        cm.limit_requested = false; // clear limit request used here ^
        cm_alarm(STAT_LIMIT_SWITCH_HIT, msg);
    }
    return (STAT_OK);
}

static stat_t _interlock_handler(void)
{
    if (cm.safety_interlock_enable) {
    // interlock broken
        if (cm.safety_interlock_disengaged != 0) {
            cm.safety_interlock_disengaged = 0;
            cm.safety_interlock_state = SAFETY_INTERLOCK_DISENGAGED;
            cm_request_feedhold();                                  // may have already requested STOP as INPUT_ACTION
            // feedhold was initiated by input action in gpio
            // pause spindle
            // pause coolant
        }

        // interlock restored
        if ((cm.safety_interlock_reengaged != 0) && (mp_runtime_is_idle())) {
            cm.safety_interlock_reengaged = 0;
            cm.safety_interlock_state = SAFETY_INTERLOCK_ENGAGED;   // interlock restored
            // restart spindle with dwell
            cm_request_end_hold();                                // use cm_request_end_hold() instead of just ending
            // restart coolant
        }
    }
    return(STAT_OK);
}

/*
 * _init_assertions() - initialize controller memory integrity assertions
 * _test_assertions() - check controller memory integrity assertions
 * _test_system_assertions() - check assertions for entire system
 */

static void _init_assertions()
{
	cs.magic_start = MAGICNUM;
	cs.magic_end = MAGICNUM;
}

static stat_t _test_assertions()
{
	if ((cs.magic_start != MAGICNUM) || (cs.magic_end != MAGICNUM)) {
        return(cm_panic(STAT_CONTROLLER_ASSERTION_FAILURE, "cs magic numbers"));
    }
	return (STAT_OK);
}

stat_t _test_system_assertions()
{
	_test_assertions();         // these functions will panic if an assertion fails
    config_test_assertions();
    canonical_machine_test_assertions();
    planner_test_assertions();
    stepper_test_assertions();
    encoder_test_assertions();
    xio_test_assertions();
	return (STAT_OK);
}
