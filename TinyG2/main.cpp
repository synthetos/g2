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
#include "hardware.h"
#include "canonical_machine.h"
//#include "json_parser.h"
//#include "gcode_parser.h"
#include "report.h"
#include "planner.h"
#include "stepper.h"
#include "spindle.h"
//#include "network.h"
#include "switch.h"
//#include "gpio.h"
//#include "test.h"
//#include "pwm.h"
#include "util.h"

#include "xio.h"

#include "MotateTimers.h"
using Motate::delay;

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

void _init() __attribute__ ((weak));
void _init() {;}

void __libc_init_array(void);

#ifdef __cplusplus
}
#endif // __cplusplus

static void _application_init(void);

/******************** Application Code ************************/

const Motate::USBSettings_t Motate::USBSettings = {
	/*gVendorID         = */ 0x1d50,
	/*gProductID        = */ 0x606d,
	/*gProductVersion   = */ TINYG_FIRMWARE_VERSION,
	/*gAttributes       = */ kUSBConfigAttributeSelfPowered,
	/*gPowerConsumption = */ 500
};
	/*gProductVersion   = */ //0.1,

Motate::USBDevice< Motate::USBCDC > usb;
//Motate::USBDevice< Motate::USBCDC, Motate::USBCDC > usb;

typeof usb._mixin_0_type::Serial &SerialUSB = usb._mixin_0_type::Serial;
//typeof usb._mixin_1_type::Serial &SerialUSB1 = usb._mixin_1_type::Serial;

MOTATE_SET_USB_VENDOR_STRING( {'S' ,'y', 'n', 't', 'h', 'e', 't', 'o', 's'} )
MOTATE_SET_USB_PRODUCT_STRING( {'T', 'i', 'n', 'y', 'G', ' ', 'v', '2'} )

void init( void )
{
	SystemInit();

	// Disable watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;

	// Initialize C library
	__libc_init_array();
}

int main( void )
{
	// system initialization
	init();
	delay(1);
	usb.attach();					// USB setup
	delay(1000);

	// TinyG application setup
	_application_init();

	// main loop
	for (;;) {
		controller_run( );			// single pass through the controller
	}
	return 0;
}

static void _application_init(void)
{
	// There are a lot of dependencies in the order of these inits.
	// Don't change the ordering unless you understand this.

	// do these first
	hardware_init();				// system hardware setup 			- must be first
	config_init();					// config records from eeprom 		- must be second
	switch_init();					// switches and other inputs
//	pwm_init();						// pulse width modulation drivers

	// do these next
	controller_init( DEV_STDIN, DEV_STDOUT, DEV_STDERR );
	planner_init();					// motion planning subsystem
	canonical_machine_init();		// canonical machine				- must follow config_init()
	spindle_init();					// spindle PWM and variables

	// do these last
	stepper_init();

	// now get started
//	rpt_print_system_ready_message();// (LAST) announce system is ready
//	_unit_tests();					// run any unit tests that are enabled
//	tg_canned_startup();			// run any pre-loaded commands
	return;
}

void tg_reset(void)			// software hard reset using the watchdog timer
{
	//	wdt_enable(WDTO_15MS);
	//	while (true);			// loops for about 15ms then resets
}

/**** Status Messages ***************************************************************
 * get_status_message() - return the status message
 *
 * See tinyg.h for status codes. These strings must align with the status codes in tinyg.h
 * The number of elements in the indexing array must match the # of strings
 *
 * Reference for putting display strings and string arrays in AVR program memory:
 * http://www.cs.mun.ca/~paul/cs4723/material/atmel/avr-libc-user-manual-1.6.5/pgmspace.html
 */

stat_t status_code;						// allocate a variable for this macro
char shared_buf[STATUS_MESSAGE_LEN];	// allocate string for global use

static const char PROGMEM stat_00[] = "OK";
static const char PROGMEM stat_01[] = "Error";
static const char PROGMEM stat_02[] = "Eagain";
static const char PROGMEM stat_03[] = "Noop";
static const char PROGMEM stat_04[] = "Complete";
static const char PROGMEM stat_05[] = "Terminated";
static const char PROGMEM stat_06[] = "Hard reset";
static const char PROGMEM stat_07[] = "End of line";
static const char PROGMEM stat_08[] = "End of file";
static const char PROGMEM stat_09[] = "File not open";
static const char PROGMEM stat_10[] = "Max file size exceeded";
static const char PROGMEM stat_11[] = "No such device";
static const char PROGMEM stat_12[] = "Buffer empty";
static const char PROGMEM stat_13[] = "Buffer full";
static const char PROGMEM stat_14[] = "Buffer full - fatal";
static const char PROGMEM stat_15[] = "Initializing";
static const char PROGMEM stat_16[] = "Entering boot loader";
static const char PROGMEM stat_17[] = "Function is stubbed";
static const char PROGMEM stat_18[] = "stat_18";
static const char PROGMEM stat_19[] = "stat_19";

static const char PROGMEM stat_20[] = "Internal error";
static const char PROGMEM stat_21[] = "Internal range error";
static const char PROGMEM stat_22[] = "Floating point error";
static const char PROGMEM stat_23[] = "Divide by zero";
static const char PROGMEM stat_24[] = "Invalid Address";
static const char PROGMEM stat_25[] = "Read-only address";
static const char PROGMEM stat_26[] = "Initialization failure";
static const char PROGMEM stat_27[] = "System alarm - shutting down";
static const char PROGMEM stat_28[] = "Memory fault or corruption";
static const char PROGMEM stat_29[] = "stat_29";
static const char PROGMEM stat_30[] = "stat_30";
static const char PROGMEM stat_31[] = "stat_31";
static const char PROGMEM stat_32[] = "stat_32";
static const char PROGMEM stat_33[] = "stat_33";
static const char PROGMEM stat_34[] = "stat_34";
static const char PROGMEM stat_35[] = "stat_35";
static const char PROGMEM stat_36[] = "stat_36";
static const char PROGMEM stat_37[] = "stat_37";
static const char PROGMEM stat_38[] = "stat_38";
static const char PROGMEM stat_39[] = "stat_39";

static const char PROGMEM stat_40[] = "Unrecognized command";
static const char PROGMEM stat_41[] = "Expected command letter";
static const char PROGMEM stat_42[] = "Bad number format";
static const char PROGMEM stat_43[] = "Input exceeds max length";
static const char PROGMEM stat_44[] = "Input value too small";
static const char PROGMEM stat_45[] = "Input value too large";
static const char PROGMEM stat_46[] = "Input value range error";
static const char PROGMEM stat_47[] = "Input value unsupported";
static const char PROGMEM stat_48[] = "JSON syntax error";
static const char PROGMEM stat_49[] = "JSON input has too many pairs";	// current longest message: 30 chars
static const char PROGMEM stat_50[] = "JSON output too long";
static const char PROGMEM stat_51[] = "Out of buffer space";
static const char PROGMEM stat_52[] = "Config rejected during cycle";
static const char PROGMEM stat_53[] = "stat_53";
static const char PROGMEM stat_54[] = "stat_54";
static const char PROGMEM stat_55[] = "stat_55";
static const char PROGMEM stat_56[] = "stat_56";
static const char PROGMEM stat_57[] = "stat_57";
static const char PROGMEM stat_58[] = "stat_58";
static const char PROGMEM stat_59[] = "stat_59";

static const char PROGMEM stat_60[] = "Move less than minimum length";
static const char PROGMEM stat_61[] = "Move less than minimum time";
static const char PROGMEM stat_62[] = "Gcode block skipped";
static const char PROGMEM stat_63[] = "Gcode input error";
static const char PROGMEM stat_64[] = "Gcode feedrate error";
static const char PROGMEM stat_65[] = "Gcode axis word missing";
static const char PROGMEM stat_66[] = "Gcode modal group violation";
static const char PROGMEM stat_67[] = "Homing cycle failed";
static const char PROGMEM stat_68[] = "Max travel exceeded";
static const char PROGMEM stat_69[] = "Max spindle speed exceeded";
static const char PROGMEM stat_70[] = "Arc specification error";
static const char PROGMEM stat_71[] = "Soft limit exceeded";
static const char PROGMEM stat_72[] = "Command not accepted";
static const char PROGMEM stat_73[] = "Probing cycle failed";

static const char PROGMEM *stat_msg[] = {
	stat_00, stat_01, stat_02, stat_03, stat_04, stat_05, stat_06, stat_07, stat_08, stat_09,
	stat_10, stat_11, stat_12, stat_13, stat_14, stat_15, stat_16, stat_17, stat_18, stat_19,
	stat_20, stat_21, stat_22, stat_23, stat_24, stat_25, stat_26, stat_27, stat_28, stat_29,
	stat_30, stat_31, stat_32, stat_33, stat_34, stat_35, stat_36, stat_37, stat_38, stat_39,
	stat_40, stat_41, stat_42, stat_43, stat_44, stat_45, stat_46, stat_47, stat_48, stat_49,
	stat_50, stat_51, stat_52, stat_53, stat_54, stat_55, stat_56, stat_57, stat_58, stat_59,
	stat_60, stat_61, stat_62, stat_63, stat_64, stat_65, stat_66, stat_67, stat_68, stat_69,
	stat_70, stat_71, stat_72, stat_73
};

char *get_status_message(stat_t status)
{
	return ((char *)GET_TEXT_ITEM(stat_msg, status));
}
