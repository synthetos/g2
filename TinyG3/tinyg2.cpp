/*
 * main.c - TinyG2 - An embedded rs274/ngc CNC controller
 * Part of TinyG2 project
 *
 * Copyright (c) 2013 Alden S. Hart Jr.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/*
#include <stdio.h>				// precursor for xio.h
#include <avr/pgmspace.h>		// precursor for xio.h
#include <avr/interrupt.h>
#include <avr/wdt.h>			// used for software reset

#include "xmega/xmega_interrupts.h"
//#include "xmega/xmega_eeprom.h"	// uncomment for unit tests
#include "xmega/xmega_rtc.h"
#include "xio/xio.h"

#include "tinyg.h"				// #1 There are some dependencies
#include "system.h"
#include "util.h"				// #2
#include "config.h"				// #3
#include "controller.h"
#include "canonical_machine.h"
#include "json_parser.h"
#include "gcode_parser.h"
#include "report.h"
#include "planner.h"
#include "spindle.h"
#include "network.h"
#include "gpio.h"
#include "test.h"
#include "pwm.h"

static void _unit_tests(void);
*/

#include "stepper.h"

/*
 * TinyG2 application setup (init) - called from Arduino main
 */
void tg_setup(void)
{
	// There are a lot of dependencies in the order of these inits.
	// Don't change the ordering unless you understand this.

	st_init(); 			// stepper subsystem 				- must precede gpio_init()

/*
	cli();

	// system and drivers
	sys_init();			// system hardware setup 			- must be first
	rtc_init();			// real time counter
	xio_init();			// xmega io subsystem
	sig_init();			// signal flags
	st_init(); 			// stepper subsystem 				- must precede gpio_init()
	gpio_init();		// switches and parallel IO
	pwm_init();			// pulse width modulation drivers	- must follow gpio_init()

	// application structures
	tg_init(STD_INPUT);	// tinyg controller (controller.c)	- must be first app init; reqs xio_init()
	cfg_init();			// config records from eeprom 		- must be next app init
	mp_init();			// motion planning subsystem
	cm_init();			// canonical machine				- must follow cfg_init()
	sp_init();			// spindle PWM and variables

	// now bring up the interupts and get started
	PMIC_SetVectorLocationToApplication(); // as opposed to boot ROM
	PMIC_EnableHighLevel();			// all levels are used, so don't bother to abstract them
	PMIC_EnableMediumLevel();
	PMIC_EnableLowLevel();
	sei();							// enable global interrupts
	rpt_print_system_ready_message();// (LAST) announce system is ready

	_unit_tests();					// run any unit tests that are enabled
	tg_canned_startup();			// run any pre-loaded commands

	while (true) {
//		if (tg.network == NET_MASTER) { 
//			tg_repeater();
//		} else if (tg.network == NET_SLAVE) { 
//			tg_receiver();
//		} else {
			tg_controller();		// NET_STANDALONE
//		}
	}
*/
	return;
}

/*
 * tg_reset() - software hard reset using watchdog timer
 */

void tg_reset(void)			// software hard reset using the watchdog timer
{
//	wdt_enable(WDTO_15MS);
//	while (true);			// loops for about 15ms then resets
}

/*
 * _unit_tests() - uncomment __UNITS... line in .h files to enable unit tests
 */

static void _unit_tests(void) 
{
#ifdef __UNIT_TESTS
	XIO_UNITS;				// conditional unit tests for xio sub-system
//	EEPROM_UNITS;			// if you want this you must include the .h file in this file
	CONFIG_UNITS;
	JSON_UNITS;
	GPIO_UNITS;
	REPORT_UNITS;
	PLANNER_UNITS;
	PWM_UNITS;
#endif
}