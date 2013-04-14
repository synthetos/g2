/*
 * controller.c - tinyg controller and top level parser
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* See the wiki for module details and additional information:
 *	 http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Developer-Info
 */

#include "tinyg2.h"
#include "controller.h"
#include "xio.h"

/*
#include <ctype.h>				// for parsing

#include "tinyg.h"				// #1 unfortunately, there are some dependencies
#include "config.h"				// #2
#include "settings.h"
#include "json_parser.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "plan_arc.h"
#include "planner.h"
#include "stepper.h"
#include "system.h"
#include "gpio.h"
#include "report.h"
#include "util.h"
#include "help.h"
*/

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

controller_t controller_state;	// controller state structure

// local helpers
static void _controller_HSM(controller_t *cs);
static uint8_t _command_dispatch(controller_t *cs);
static uint8_t _normal_idler(controller_t *cs);
/*
static uint8_t _reset_handler(void);
static uint8_t _bootloader_handler(void);
static uint8_t _limit_switch_handler(void);
static uint8_t _shutdown_idler(void);
static uint8_t _system_assertions(void);
static uint8_t _feedhold_handler(void);
static uint8_t _cycle_start_handler(void);
static uint8_t _sync_to_tx_buffer(void);
static uint8_t _sync_to_planner(void);
*/
/*
 * tg_init() - controller init
 */

void controller_init(controller_t *cs, uint8_t std_in, uint8_t std_out, uint8_t std_err) 
{
	cs->magic_start = MAGICNUM;
	cs->magic_end = MAGICNUM;
	cs->fw_build = TINYG_FIRMWARE_BUILD;
	cs->fw_version = TINYG_FIRMWARE_VERSION;// NB: HW version is set from EEPROM
	cs->linelen = 0;						// initialize index for read_line()

//	xio_set_stdin(std_in);
//	xio_set_stdout(std_out);
//	xio_set_stderr(std_err);
//	cs.default_src = std_in;
//	tg_set_active_source(cs.default_src);	// set initial active source
}

/* 
 * tg_controller() - MAIN LOOP - top-level controller
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
 * if not finished (TG_EAGAIN), preventing later routines from running 
 * (they remain blocked). Any other condition - OK or ERR - drops through 
 * and runs the next routine in the list.
 *
 * A routine that had no action (i.e. is OFF or idle) should return TG_NOOP
 *
 * Useful reference on state machines:
 * http://johnsantic.com/comp/state.html, "Writing Efficient State Machines in C"
 */

void controller_run(controller_t *cs) 
{ 
	while (true) { 
		_controller_HSM(cs);
	}
}

#define	DISPATCH(func) if (func == STAT_EAGAIN) return; 
static void _controller_HSM(controller_t *cs)
{
//----- ISRs. These should be considered the highest priority scheduler functions ----//
/*
 *	HI	Stepper DDA pulse generation			// see stepper.h
 *	HI	Stepper load routine SW interrupt		// see stepper.h
 *	HI	Dwell timer counter 					// see stepper.h
 *	MED	GPIO1 switch port - limits / homing		// see gpio.h
 *  MED	Serial RX for USB						// see xio_usart.h
 *  LO	Segment execution SW interrupt			// see stepper.h
 *  LO	Serial TX for USB & RS-485				// see xio_usart.h
 *	LO	Real time clock interrupt				// see xmega_rtc.h
 */
//----- kernel level ISR handlers ----(flags are set in ISRs)-----------//
												// Order is important:
//	DISPATCH(_reset_handler(cs));				// 1. software reset received
//	DISPATCH(_bootloader_handler(cs));			// 2. received ESC char to start bootloader
//	DISPATCH(_limit_switch_handler(cs));		// 3. limit switch has been thrown
//	DISPATCH(_alarm_idler(cs));					// 4. idle in shutdown state (alarmed)
//	DISPATCH(_system_assertions(cs));			// 5. system integrity assertions
//	DISPATCH(_feedhold_handler(cs));			// 6. feedhold requested
//	DISPATCH(_cycle_start_handler(cs));			// 7. cycle start requested

//----- planner hierarchy for gcode and cycles -------------------------//
//	DISPATCH(rpt_status_report_callback(cs));	// conditionally send status report
//	DISPATCH(rpt_queue_report_callback(cs));	// conditionally send queue report
//	DISPATCH(mp_plan_hold_callback(cs));		// plan a feedhold
//	DISPATCH(mp_end_hold_callback(cs));			// end a feedhold
//	DISPATCH(ar_arc_callback(cs));				// arc generation runs behind lines
//	DISPATCH(cm_homing_callback(cs));			// G28.2 continuation

//----- command readers and parsers ------------------------------------//
//	DISPATCH(_sync_to_planner(cs));				// ensure there is at least one free buffer in planning queue
//	DISPATCH(_sync_to_tx_buffer(cs));			// sync with TX buffer (pseudo-blocking)
//	DISPATCH(cfg_baud_rate_callback(cs));		// perform baud rate update (must be after TX sync)
	DISPATCH(_command_dispatch(cs));			// read and execute next command
	DISPATCH(_normal_idler(cs));				// blink LEDs slowly to show everything is OK
}

/***************************************************************************** 
 * _command_dispatch() - dispatch line received from active input device
 *
 *	Reads next command line and dispatches to relevant parser or action
 *	Accepts commands if the move queue has room - EAGAINS if it doesn't
 *	Manages cutback to serial input from file devices (EOF)
 *	Also responsible for prompts and for flow control 
 */

static status_t _command_dispatch(controller_t *cs)
{
//	printf("printf test2 %d %f...\n", 10, 10.003);

	status_t status;

	// read input line or return if not a completed line
	if ((status = read_line(cs->in_buf, &cs->linelen, sizeof(cs->in_buf))) != STAT_OK) {
		return (status);	// Note that STAT_EAGAIN, STAT_BUFFER_FULL etc. will just flow through
	}
	write (cs->in_buf, cs->linelen);
	return (STAT_OK);
}

//	const uint8_t ptr[] = ("Hello Kitty...");
//	SerialUSB.write(ptr, sizeof(ptr));
//	SerialUSB.print("Hello Kitty...");
//	Serial.print("Hello Kitty2...");
//	c = SerialUSB.read();		// read is non-blocking
//	SerialUSB.write(c);

/*
	cmd_reset_list();
	cs.linelen = strlen(cs.in_buf)+1;
	strncpy(cs.saved_buf, cs.in_buf, SAVED_BUFFER_LEN-1);	// save input buffer for reporting

	// dispatch the new text line
	switch (toupper(cs.in_buf[0])) {

		case NUL: { 							// blank line (just a CR)
			if (cfg.comm_mode != JSON_MODE) {
				tg_text_response(TG_OK, cs.saved_buf);
			}
			break;
		}
		case 'H': { 							// intercept help screens
			cfg.comm_mode = TEXT_MODE;
			print_general_help();
			tg_text_response(TG_OK, cs.in_buf);
			break;
		}
		case '$': case '?':{ 					// text-mode configs
			cfg.comm_mode = TEXT_MODE;
			tg_text_response(cfg_text_parser(cs.in_buf), cs.saved_buf);
			break;
		}
		case '{': { 							// JSON input
			cfg.comm_mode = JSON_MODE;
			js_json_parser(cs.in_buf);
			break;
		}
		default: {								// anything else must be Gcode
			if (cfg.comm_mode == JSON_MODE) {
				strncpy(cs.out_buf, cs.in_buf, INPUT_BUFFER_LEN -8);	// use out_buf as temp
				sprintf(cs.in_buf,"{\"gc\":\"%s\"}\n", cs.out_buf);		// '-8' is used for JSON chars
				js_json_parser(cs.in_buf);
			} else {
				tg_text_response(gc_gcode_parser(cs.in_buf), cs.saved_buf);
			}
		}
	}
	return (STAT_OK);
}
*/

/* 
 * _normal_idler() - blink LED13 slowly to show everything is OK
 */

static status_t _normal_idler( controller_t *cs )
{
	if (--(cs->led_counter) < 0) {
		cs->led_counter = LED_NORMAL_COUNTER;
		cs->led_state ^= 1;
		digitalWrite(INDICATOR_LED, cs->led_state);
	}
	return (STAT_OK);
}

/* 
 * _alarm_idler() - revent any further activity form occurring if shut down
 *
 *	This function returns EAGAIN causing the control loop to never advance beyond
 *	this point. It's important that the reset handler is still called so a SW reset
 *	(ctrl-x) can be processed.
 */
/*
static uint8_t _alarm_idler( controller *cs )
{
	if (cm_get_machine_state() != MACHINE_SHUTDOWN) { return (TG_OK);}

	if (--(cs->led_counter) < 0) {
		cs->led_counter = LED_ALARM_COUNTER;
		cs->led_state ^= 1;
		digitalWrite(INDICATOR_LED, cs->led_state);
	}
	return (TG_EAGAIN);	 // EAGAIN prevents any other actions from running
}
*/
/* 
 * _system_assertions() - check memory integrity and other assertions
 */
/*
uint8_t _system_assertions()
{
	uint8_t value = 0;
	
	if (cs.magic_start		!= MAGICNUM) { value = 1; }		// Note: reported VALue is offset by ALARM_MEMORY_OFFSET
	if (cs.magic_end		!= MAGICNUM) { value = 2; }
	if (cm.magic_start 		!= MAGICNUM) { value = 3; }
	if (cm.magic_end		!= MAGICNUM) { value = 4; }
	if (gm.magic_start		!= MAGICNUM) { value = 5; }
	if (gm.magic_end 		!= MAGICNUM) { value = 6; }
	if (cfg.magic_start		!= MAGICNUM) { value = 7; }
	if (cfg.magic_end		!= MAGICNUM) { value = 8; }
	if (cmdStr.magic_start	!= MAGICNUM) { value = 9; }
	if (cmdStr.magic_end	!= MAGICNUM) { value = 10; }
	if (mb.magic_start		!= MAGICNUM) { value = 11; }
	if (mb.magic_end		!= MAGICNUM) { value = 12; }
	if (mr.magic_start		!= MAGICNUM) { value = 13; }
	if (mr.magic_end		!= MAGICNUM) { value = 14; }
	if (ar.magic_start		!= MAGICNUM) { value = 15; }
	if (ar.magic_end		!= MAGICNUM) { value = 16; }
	if (st_get_st_magic()	!= MAGICNUM) { value = 17; }
	if (st_get_sps_magic()	!= MAGICNUM) { value = 18; }
	if (rtc.magic_end 		!= MAGICNUM) { value = 19; }
	xio_assertions(&value);									// run xio assertions

	if (value == 0) { return (TG_OK);}
	rpt_exception(TG_MEMORY_CORRUPTION, value);
	cm_shutdown(ALARM_MEMORY_OFFSET + value);	
	return (STAT_EAGAIN);
}
*/

#ifdef __cplusplus
}
#endif // __cplusplus
