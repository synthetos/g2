/*
 * main.cpp - TinyG - An embedded rs274/ngc CNC controller
 * This file is part of the TinyG project.
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2015 Robert Giseburt
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

#include "tinyg2.h"					// #1 There are some dependencies
#include "config.h"					// #2
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
#include "gpio.h"
#include "test.h"
#include "pwm.h"
#include "xio.h"

#ifdef __AVR
#include <avr/interrupt.h>
#include "xmega/xmega_interrupts.h"
#endif // __AVR

#ifdef __ARM
#include "UniqueId.h"
#include "MotateTimers.h"
using Motate::delay;

/**************************
 *** C++ specific stuff ***
 **************************/
#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

void _init() __attribute__ ((weak));
void _init() {;}
void __libc_init_array(void);

#ifdef __cplusplus
}
#endif // __cplusplus
/************************** to here ***************************/

void* __dso_handle = nullptr;

#endif // __ARM

/******************** System Globals *************************/

stat_t status_code;						// allocate a variable for the ritorno macro
char global_string_buf[MESSAGE_LEN];	// allocate a string for global message use

/******************** Application Code ************************/

#ifdef __ARM
const Motate::USBSettings_t Motate::USBSettings = {
	/*gVendorID         = */ 0x1d50,
	/*gProductID        = */ 0x606d,
	/*gProductVersion   = */ TINYG_FIRMWARE_VERSION,
	/*gAttributes       = */ kUSBConfigAttributeSelfPowered,
	/*gPowerConsumption = */ 500
};
	/*gProductVersion   = */ //0.1,

//Motate::USBDevice< Motate::USBCDC > usb;
Motate::USBDevice< Motate::USBCDC, Motate::USBCDC > usb;

decltype(usb._mixin_0_type::Serial) &SerialUSB = usb._mixin_0_type::Serial;
decltype(usb._mixin_1_type::Serial) &SerialUSB1 = usb._mixin_1_type::Serial;

MOTATE_SET_USB_VENDOR_STRING( {'S' ,'y', 'n', 't', 'h', 'e', 't', 'o', 's'} )
MOTATE_SET_USB_PRODUCT_STRING( {'T', 'i', 'n', 'y', 'G', ' ', 'v', '2'} )
MOTATE_SET_USB_SERIAL_NUMBER_STRING_FROM_CHIPID()

//Motate::SPI<kSocket4_SPISlaveSelectPinNumber> spi;
#endif

/*
 * _system_init()
 * _application_init_services()
 * _application_init_machine()
 * _application_init_startup()
 *
 * There are a lot of dependencies in the order of these inits.
 * Don't change the ordering unless you understand this.
 */

void _system_init(void)
{
#ifdef __ARM
	SystemInit();
	WDT->WDT_MR = WDT_MR_WDDIS;     // Disable watchdog
	__libc_init_array();            // Initialize C library
    cacheUniqueId();                // Store the flash UUID
	usb.attach();                   // USB setup
	delay(1000);
#endif
#ifdef __AVR
    cli();
#endif
}

void application_init_services(void)
{
	hardware_init();				// system hardware setup 			- must be first
	persistence_init();				// set up EEPROM or other NVM		- must be second
	xio_init();						// xtended io subsystem				- must be third
//	rtc_init();						// real time counter
}

void application_init_machine(void)
{
	cm.machine_state = MACHINE_INITIALIZING;

    stepper_init();                 // stepper subsystem (must precede gpio_init() on AVR)
    encoder_init();                 // virtual encoders
    gpio_init();                    // inputs and outputs
    pwm_init();                     // pulse width modulation drivers
    planner_init();                 // motion planning subsystem
    canonical_machine_init();       // canonical machine
}

void application_init_startup(void)
{
#ifdef __AVR
    // now bring up the interrupts and get started
    PMIC_SetVectorLocationToApplication();// as opposed to boot ROM
    PMIC_EnableHighLevel();			// all levels are used, so don't bother to abstract them
    PMIC_EnableMediumLevel();
    PMIC_EnableLowLevel();
    sei();							// enable global interrupts
#endif

    // start the application
    controller_init(STD_IN, STD_OUT, STD_ERR);  // should be first startup init (requires xio_init())
    config_init();					            // apply the config settings from persistence
    canonical_machine_reset();
    spindle_init();                 // should be after PWM and canonical machine inits and config_init()
    spindle_reset();
    // We delayed driving spindle enable until we know the enable polarity
    spindle_enable_pin.init();
    // MOVED: report the system is ready is now in xio
}

/*
 * main()
 */

int main(void)
{
	// system initialization
	_system_init();

	// TinyG application setup
	application_init_services();
	application_init_machine();
	application_init_startup();
	run_canned_startup();			// run any pre-loaded commands

	// main loop
	for (;;) {
		controller_run( );			// single pass through the controller
	}
	return 0;
}

/*
 * get_status_message() - support for status messages.
 */

char *get_status_message(stat_t status)
{
    return ((char *)GET_TEXT_ITEM(stat_msg, status));
}
