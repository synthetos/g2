/*
 * hardware.cpp - general hardware support functions
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
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

#ifdef __AVR
#include <avr/wdt.h>			// used for software reset
#endif

#include "tinyg2.h"		// #1
#include "config.h"		// #2
#include "hardware.h"
#include "controller.h"
#include "text_parser.h"
#ifdef __ARM
#include "UniqueId.h"
#include "Reset.h"
#endif
#ifdef __AVR
#include "xmega/xmega_init.h"
#include "xmega/xmega_rtc.h"
#endif

/*
 * hardware_init() - lowest level hardware init
 */

void hardware_init()
{
	return;
}

/*
 * hw_hard_reset() - reset system now
 * hw_flash_loader() - enter flash loader to reflash board
 */

void hw_hard_reset(void)
{
    banzai(0);   // arg=0 resets the system
}

void hw_flash_loader(void)
{
    banzai(1);  // arg=1 erases FLASH and enters FLASH loader
}

/*
 * _get_id() - get a human readable signature
 *
 *	Produce a unique deviceID based on the factory calibration data.
 *	Truncate to SYS_ID_DIGITS length
 */

void _get_id(char *id)
{
    char *p = id;
    const uint16_t *uuid = readUniqueIdString();

    for(uint8_t i=0; i<SYS_ID_DIGITS; i++) {
        *p++ = uuid[i];
        if ( (i & 0x03) == 3) {     // put a dash every 4 digits
            *p++ = '-';
        }
    }
    *(--p) = 0; // nul termination
}

/***** END OF SYSTEM FUNCTIONS *****/

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/*
 * hw_get_fbs() - get firmware build string
 */

stat_t hw_get_fbs(nvObj_t *nv)
{
    nv->valuetype = TYPE_STRING;
//    ritorno(nv_copy_string(nv, TINYG_FIRMWARE_BUILD_STRING));
    ritorno(nv_copy_string(nv, "build"));
    return (STAT_OK);
}

/*
 * hw_get_id() - get device ID (signature)
 */

stat_t hw_get_id(nvObj_t *nv)
{
	char tmp[SYS_ID_LEN];
	_get_id(tmp);
	nv->valuetype = TYPE_STRING;
	ritorno(nv_copy_string(nv, tmp));
	return (STAT_OK);
}

/*
 * hw_flash() - invoke FLASH loader from command input
 */
stat_t hw_flash(nvObj_t *nv)
{
    hw_flash_loader();
	return(STAT_OK);
}

/*
 * hw_set_hv() - set hardware version number
 */
stat_t hw_set_hv(nvObj_t *nv)
{
	return (STAT_OK);
}


/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

static const char fmt_fb[] PROGMEM =  "[fb]  firmware build%18.2f\n";
static const char fmt_fbs[] PROGMEM = "[fbs] firmware build \"%32s\"\n";
static const char fmt_fv[] PROGMEM =  "[fv]  firmware version%16.2f\n";
static const char fmt_cv[] PROGMEM =  "[cv]  configuration version%11.2f\n";
static const char fmt_hp[] PROGMEM =  "[hp]  hardware platform%15.2f\n";
static const char fmt_hv[] PROGMEM =  "[hv]  hardware version%16.2f\n";
static const char fmt_id[] PROGMEM =  "[id]  TinyG ID%21s\n";

void hw_print_fb(nvObj_t *nv)  { text_print(nv, fmt_fb);}    // TYPE_FLOAT
void hw_print_fbs(nvObj_t *nv) { text_print(nv, fmt_fbs);}  // TYPE_STRING
void hw_print_fv(nvObj_t *nv)  { text_print(nv, fmt_fv);}    // TYPE_FLOAT
void hw_print_cv(nvObj_t *nv)  { text_print(nv, fmt_cv);}    // TYPE_FLOAT
void hw_print_hp(nvObj_t *nv)  { text_print(nv, fmt_hp);}    // TYPE_FLOAT
void hw_print_hv(nvObj_t *nv)  { text_print(nv, fmt_hv);}    // TYPE_FLOAT
void hw_print_id(nvObj_t *nv)  { text_print(nv, fmt_id);}    // TYPE_STRING

#endif //__TEXT_MODE
