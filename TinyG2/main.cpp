/*
 * main.cpp
 * This file is part of the TinyG2 project.
 *
 * Copyright (c) 2013 Alden S. Hart Jr.
 * Copyright (c) 2013 Robert Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
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
#include "planner.h"
#include "stepper.h"
#include "report.h"
#include "hardware.h"
#include "xio.h"

/*
#include "util.h"				// #2
#include "json_parser.h"
#include "gcode_parser.h"
#include "spindle.h"
#include "network.h"
#include "gpio.h"
#include "test.h"
#include "pwm.h"
*/

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

void _init() __attribute__ ((weak));
void _init() {;}
static void _application_init(void);

// collect all weird little globals here
stat_t status_code;		// declared for ritorno, see tinyg2.h

/******************** Application Code ************************/

void setup( void )
{
	USBDevice.attach();
	SerialUSB.begin(115200);
	_application_init();
}

void loop( void )
{
	controller_run( );			// single pass through the controller
}

/*
 * \brief Main entry point of Arduino application
 */
int main( void )
{
	init();
	setup();

	for (;;) {
		loop();
	}
	return 0;
}

static void _application_init(void)
{
	// There are a lot of dependencies in the order of these inits.
	// Don't change the ordering unless you understand this.

//	cli();
	hardware_init();				// system hardware setup 			- must be first
	stepper_init(); 				// must precede gpio_init()
//	gpio_init();					// switches and parallel IO
//	pwm_init();						// pulse width modulation drivers	- must follow gpio_init()

	controller_init( DEV_STDIN, DEV_STDOUT, DEV_STDERR );
	config_init();					// config records from eeprom 		- must be next app init
	planner_init();					// motion planning subsystem
	canonical_machine_init();		// canonical machine				- must follow cfg_init()
//	spindle_init();					// spindle PWM and variables

	// now bring up the interrupts and get started
//	sei();							// enable global interrupts
	rpt_print_system_ready_message();// (LAST) announce system is ready

//	_unit_tests();					// run any unit tests that are enabled
//	tg_canned_startup();			// run any pre-loaded commands
	return;
}

void tg_reset(void)			// software hard reset using the watchdog timer
{
	//	wdt_enable(WDTO_15MS);
	//	while (true);			// loops for about 15ms then resets
}

#ifdef __cplusplus
}
#endif // __cplusplus
