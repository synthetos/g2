/*
 * hardware.cpp - general hardware support functions
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2014 Alden S. Hart, Jr.
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
#include "switch.h"
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

#ifdef __cplusplus
extern "C"{
#endif

/*
 * hardware_init() - lowest level hardware init
 */

void hardware_init()
{
	return;
}

/*
 * _get_id() - get a human readable signature
 *
 *	Produce a unique deviceID based on the factory calibration data.
 *	Truncate to SYS_ID_LEN bytes
 */

void _get_id(char_t *id)
{
    const uint16_t *uuid = readUniqueIdString();
    for(int i = 0; i < SYS_ID_LEN-1; ++i)
        id[i] = uuid[i];
    id[SYS_ID_LEN-1] = 0;
}
 
 /*
 * Hardware Reset Handlers
 *
 * hw_request_hard_reset()
 * hw_hard_reset()			- hard reset using watchdog timer
 * hw_hard_reset_handler()	- controller's rest handler
 */
void hw_request_hard_reset() { cs.hard_reset_requested = true; }

void hw_hard_reset(void)			// software hard reset using the watchdog timer
{
    banzai(0);
}

stat_t hw_hard_reset_handler(void)
{
	if (cs.hard_reset_requested == false) { return (STAT_NOOP);}
	hw_hard_reset();				// hard reset - identical to hitting RESET button
	return (STAT_EAGAIN);
}

/*
 * Bootloader Handlers
 *
 * hw_request_bootloader()
 * hw_bootloader()
 * hw_request_bootloader_handler() - executes a software reset using CCPWrite
 */

void hw_request_bootloader() { cs.bootloader_requested = true;}
    
void hw_bootloader(void)
{
    banzai(1);
}

stat_t hw_bootloader_handler(void)
{
    if (cs.bootloader_requested == false) { return (STAT_NOOP);}
    hw_bootloader();
    return (STAT_EAGAIN);
}

/***** END OF SYSTEM FUNCTIONS *****/


/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/*
 * hw_get_id() - get device ID (signature)
 */

stat_t hw_get_id(nvObj_t *nv) 
{
	char_t tmp[SYS_ID_LEN];
	_get_id(tmp);
	nv->valuetype = TYPE_STRING;
	ritorno(nv_copy_string(nv, tmp));
	return (STAT_OK);
}

/*
 * hw_run_boot() - invoke boot form the cfgArray
 */
stat_t hw_run_boot(nvObj_t *nv)
{
	hw_request_bootloader();
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

static const char fmt_fb[] PROGMEM = "[fb]  firmware build%18.2f\n";
static const char fmt_fv[] PROGMEM = "[fv]  firmware version%16.2f\n";
static const char fmt_cv[] PROGMEM = "[cv]  configuration version%11.2f\n";
static const char fmt_hp[] PROGMEM = "[hp]  hardware platform%15.2f\n";
static const char fmt_hv[] PROGMEM = "[hv]  hardware version%16.2f\n";
static const char fmt_id[] PROGMEM = "[id]  TinyG ID%30s\n";

void hw_print_fb(nvObj_t *nv) { text_print_flt(nv, fmt_fb);}
void hw_print_fv(nvObj_t *nv) { text_print_flt(nv, fmt_fv);}
void hw_print_cv(nvObj_t *nv) { text_print_flt(nv, fmt_cv);}
void hw_print_hp(nvObj_t *nv) { text_print_flt(nv, fmt_hp);}
void hw_print_hv(nvObj_t *nv) { text_print_flt(nv, fmt_hv);}
void hw_print_id(nvObj_t *nv) { text_print_str(nv, fmt_id);}

#endif //__TEXT_MODE 

#ifdef __cplusplus
}
#endif
