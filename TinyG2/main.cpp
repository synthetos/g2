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

//#define ARDUINO_MAIN

#include "sam.h"
#include "Arduino.h"

#include "tinyg2.h"
#include "controller.h"
#include "stepper.h"
#include "xio.h"

/*
#include "system.h"
#include "util.h"				// #2
#include "config.h"				// #3
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
*/

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

void _init() __attribute__ ((weak));
void _init() {;}

/*
extern void SysTick_Handler( void )
{
  // Increment tick count each ms
  TimeTick_Increment() ;
}
*/


#define DEV_STDIN 0
#define DEV_STDOUT 0
#define DEV_STDERR 0

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
//	delay(1);
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

	controller_init( DEV_STDIN, DEV_STDOUT, DEV_STDERR );
	stepper_init(); 			// must precede gpio_init()

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
//		if (cs.network == NET_MASTER) { 
//			tg_repeater();
//		} else if (cs.network == NET_SLAVE) { 
//			tg_receiver();
//		} else {
			tg_controller();		// NET_STANDALONE
//		}
	}
*/
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
