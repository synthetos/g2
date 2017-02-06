/*
 * main.cpp - g2core - An embedded rs274/ngc CNC controller
 * This file is part of the g2core project.
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2016 Robert Giseburt
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
/* See github.com/Synthetos/G2 for code and docs on the wiki
 */

#include "g2core.h"  // #1 There are some dependencies
#include "config.h"  // #2
#include "hardware.h"
#include "persistence.h"
#include "controller.h"
#include "canonical_machine.h"
#include "json_parser.h"			// required for unit tests only
#include "report.h"
#include "planner.h"
#include "stepper.h"
#include "encoder.h"
#include "spindle.h"
#include "temperature.h"
#include "gpio.h"
#include "pwm.h"
#include "xio.h"

#include "util.h"
#include "MotateUniqueID.h"

/***** NOTE: *****

   The actual main.cpp for g2core is in the Motate project.
   It calls the setup() and loop() functions in this file

*****/
/******************** System Globals *************************/

stat_t status_code;						    // allocate a variable for the ritorno macro

/************* System Globals For Diagnostics ****************/

// Using motate pins for profiling
// see https://github.com/synthetos/g2/wiki/Using-Pin-Changes-for-Timing-(and-light-debugging)

using namespace Motate;
OutputPin<kDebug1_PinNumber> debug_pin1;
OutputPin<kDebug2_PinNumber> debug_pin2;
OutputPin<kDebug3_PinNumber> debug_pin3;
//OutputPin<kDebug4_PinNumber> debug_pin4;

//OutputPin<-1> debug_pin1;
//OutputPin<-1> debug_pin2;
//OutputPin<-1> debug_pin3;
//OutputPin<-1> debug_pin4;

// Put these lines in the .cpp file where you are using the debug pins (appropriately commented / uncommented)
/*
using namespace Motate;
extern OutputPin<kDebug1_PinNumber> debug_pin1;
extern OutputPin<kDebug2_PinNumber> debug_pin2;
extern OutputPin<kDebug3_PinNumber> debug_pin3;
//extern OutputPin<kDebug4_PinNumber> debug_pin4;

//extern OutputPin<-1> debug_pin1;
//extern OutputPin<-1> debug_pin2;
//extern OutputPin<-1> debug_pin3;
//extern OutputPin<-1> debug_pin4;
*/

/******************** Application Code ************************/

/*
 * _application_init_services()
 * _application_init_machine()
 * _application_init_startup()
 *
 * There are a lot of dependencies in the order of these inits.
 * Don't change the ordering unless you understand this.
 */

void application_init_services(void)
{
    hardware_init();				// system hardware setup 			- must be first
    persistence_init();				// set up EEPROM or other NVM		- must be second
    xio_init();						// xtended io subsystem				- must be third
}

void application_init_machine(void)
{
    cm.machine_state = MACHINE_INITIALIZING;

    stepper_init();                 // stepper subsystem
    encoder_init();                 // virtual encoders
    gpio_init();                    // inputs and outputs
    pwm_init();                     // pulse width modulation drivers
    planner_init();                 // motion planning subsystem
    canonical_machine_init();       // canonical machine
}

void application_init_startup(void)
{
    // start the application
    controller_init();              // should be first startup init (requires xio_init())
    config_init();					// apply the config settings from persistence
    canonical_machine_reset();
    spindle_init();                 // should be after PWM and canonical machine inits and config_init()
    spindle_reset();
    temperature_init();
    gpio_reset();
}

/*
 * get_status_message() - global support for status messages.
 */

char *get_status_message(stat_t status)
{
    return ((char *)GET_TEXT_ITEM(stat_msg, status));
}

/*
 * main() - See Motate main.cpp for the actual main()
 */

void setup(void)
{
    // application setup
    application_init_services();
    while (SysTickTimer_getValue() < 400);  // delay 400 ms for USB to come up

    application_init_machine();
    application_init_startup();
}

void loop() {
    // main loop
    for (;;) {
        controller_run( );			// single pass through the controller
    }
}

/*
 * Traps for debugging. These must be in main.cpp for proper linker ordering
 */

void MemManage_Handler  ( void ) { __asm__("BKPT"); }
void BusFault_Handler   ( void ) { __asm__("BKPT"); }
void UsageFault_Handler ( void ) { __asm__("BKPT"); }
void HardFault_Handler  ( void ) { __asm__("BKPT"); }
