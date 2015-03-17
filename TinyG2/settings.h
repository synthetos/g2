/*
 * settings.h - default runtime settings
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart Jr.
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
/*	The values in this file are the default settings that are loaded into a virgin EEPROM,
 *	and can be changed using the config commands. After initial load the EEPROM values
 *	(or changed values) are used.
 *
 *	System and hardware settings that you shouldn't need to change are in hardware.h
 *	Application settings that also shouldn't need to be changed are in tinyg.h
 */

#ifndef SETTINGS_H_ONCE
#define SETTINGS_H_ONCE

/**** MACHINE PROFILES ******************************************************
 *
 * Provide a SETTINGS_FILE in the makefile or compiler command line, e.g:
 *	SETTINGS_FILE="settings_shopbot_test.h"
 *
 * If no file is specified the default settings file will be used
 */
#ifdef SETTINGS_FILE
#define SETTINGS_FILE_PATH <settings/SETTINGS_FILE>
#include SETTINGS_FILE_PATH
#else
#include "settings/settings_default.h"				// Default settings for release
#endif

// Alternate settings files that may be available in the project:
//#include "settings/settings_othermill.h"				// OMC OtherMill
//#include "settings/settings_probotixV90.h"			// Probotix FireballV90
//#include "settings/settings_shapeoko2.h"				// Shapeoko2 standard kit
//#include "settings/settings_shopbot_sbv300.h"			// Shopbot sbv300 board profile
//#include "settings/settings_Ultimaker.h"				// Ultimaker 3D printer
//#include "settings/settings_Ultimaker_Rob_v9h.h"
//#include "settings/settings_zen7x12.h"				// Zen Toolworks 7x12


/****** TEMPORARY COMPATIBILITY CODE -- TO BE REMOVED ASAP *********/

#define X_SWITCH_MODE_MIN           SW_MODE_HOMING		// xsn  SW_MODE_DISABLED, SW_MODE_HOMING, SW_MODE_LIMIT, SW_MODE_HOMING_LIMIT
#define X_SWITCH_MODE_MAX           SW_MODE_DISABLED	// xsx  SW_MODE_DISABLED, SW_MODE_HOMING, SW_MODE_LIMIT, SW_MODE_HOMING_LIMIT
#define X_SWITCH_TYPE_MIN           SW_TYPE_NORMALLY_OPEN // rsn SW_TYPE_NORMALLY_OPEN, SW_TYPE_NORMALLY_CLOSED
#define X_SWITCH_TYPE_MAX           SW_TYPE_NORMALLY_OPEN // rsx SW_TYPE_NORMALLY_OPEN, SW_TYPE_NORMALLY_CLOSED

#define Y_SWITCH_MODE_MIN           SW_MODE_HOMING
#define Y_SWITCH_MODE_MAX           SW_MODE_DISABLED
#define Y_SWITCH_TYPE_MIN           SW_TYPE_NORMALLY_OPEN
#define Y_SWITCH_TYPE_MAX           SW_TYPE_NORMALLY_OPEN

#define Z_SWITCH_MODE_MIN           SW_MODE_DISABLED
#define Z_SWITCH_MODE_MAX           SW_MODE_HOMING
#define Z_SWITCH_TYPE_MIN           SW_TYPE_NORMALLY_OPEN
#define Z_SWITCH_TYPE_MAX           SW_TYPE_NORMALLY_OPEN

#define A_SWITCH_MODE_MIN           SW_MODE_HOMING
#define A_SWITCH_MODE_MAX           SW_MODE_DISABLED
#define A_SWITCH_TYPE_MIN           SW_TYPE_NORMALLY_OPEN
#define A_SWITCH_TYPE_MAX           SW_TYPE_NORMALLY_OPEN

#define B_SWITCH_MODE_MIN           SW_MODE_HOMING
#define B_SWITCH_MODE_MAX           SW_MODE_DISABLED
#define B_SWITCH_TYPE_MIN           SW_TYPE_NORMALLY_OPEN
#define B_SWITCH_TYPE_MAX           SW_TYPE_NORMALLY_OPEN

#define C_SWITCH_MODE_MIN           SW_MODE_HOMING
#define C_SWITCH_MODE_MAX           SW_MODE_DISABLED
#define C_SWITCH_TYPE_MIN           SW_TYPE_NORMALLY_OPEN
#define C_SWITCH_TYPE_MAX           SW_TYPE_NORMALLY_OPEN

/**** MACHINE PROFILES ******************************************************
 *
 * Setup resonable default for some settings, so that all of the settings files
 * don't heve to specify every value.
 *
 * Note that if a key value (e.g. DI1_MODE) is defined, then all of the ones in
 * that group (e.g. DI1_ACTION, DI1_FUNCTION) must be defined as well.
 *
 */

//*** Input / output settings ***
/*
#ifndef DEFAULT_MODE
#define DEFAULT_MODE                NORMALLY_CLOSED
#define DEFAULT_ACTION              IO_ACTION_NONE
#define DEFAULT_FUNCTION            IO_FUNCTION_NONE
#endif

#ifdef X_SWITCH_MODE_MIN
#warning Please update your settings files! X_SWITCH_MODE_MIN is not longer valid.
#warning Please look in settings.h for DI1_MODE, DI1_ACTION, and DI1_FUNCTION.
#endif

#ifndef DI1_MODE
#define DI1_MODE                    DEFAULT_MODE
#define DI1_ACTION                  DEFAULT_ACTION
#define DI1_FUNCTION                DEFAULT_FUNCTION
#endif

#ifndef DI2_MODE
#define DI2_MODE                    DEFAULT_MODE
#define DI2_ACTION                  DEFAULT_ACTION
#define DI2_FUNCTION                DEFAULT_FUNCTION
#endif

#ifndef DI3_MODE
#define DI3_MODE                    DEFAULT_MODE
#define DI3_ACTION                  DEFAULT_ACTION
#define DI3_FUNCTION                DEFAULT_FUNCTION
#endif

#ifndef DI4_MODE
#define DI4_MODE                    DEFAULT_MODE
#define DI4_ACTION                  DEFAULT_ACTION
#define DI4_FUNCTION                DEFAULT_FUNCTION
#endif

#ifndef DI5_MODE
#define DI5_MODE                    DEFAULT_MODE
#define DI5_ACTION                  DEFAULT_ACTION
#define DI5_FUNCTION                DEFAULT_FUNCTION
#endif

#ifndef DI6_MODE
#define DI6_MODE                    DEFAULT_MODE
#define DI6_ACTION                  DEFAULT_ACTION
#define DI6_FUNCTION                DEFAULT_FUNCTION
#endif

#ifndef DI7_MODE
#define DI7_MODE                    DEFAULT_MODE
#define DI7_ACTION                  DEFAULT_ACTION
#define DI7_FUNCTION                DEFAULT_FUNCTION
#endif

#ifndef DI8_MODE
#define DI8_MODE                    DEFAULT_MODE
#define DI8_ACTION                  DEFAULT_ACTION
#define DI8_FUNCTION                DEFAULT_FUNCTION
#endif

#ifndef DI9_MODE
#define DI9_MODE                    DEFAULT_MODE
#define DI9_ACTION                  DEFAULT_ACTION
#define DI9_FUNCTION                DEFAULT_FUNCTION
#endif
*/

#endif // End of include guard: SETTINGS_H_ONCE
