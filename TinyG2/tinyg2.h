/*
 * tinyg2.h - tinyg2 main header
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
 * Copyright (c) 2010 - 2015 Robert Giseburt
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
/* Is this code over documented? Possibly.
 * We try to follow this (at least we are evolving to it). It's worth a read.
 * ftp://ftp.idsoftware.com/idstuff/doom3/source/CodeStyleConventions.doc
 */
#ifndef TINYG_H_ONCE
#define TINYG_H_ONCE

// common system includes
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "MotatePins.h"

/****** REVISIONS ******/

#ifndef TINYG_FIRMWARE_BUILD
#define TINYG_FIRMWARE_BUILD   		083.14 // split off status codes and strings into separate error.h file
#endif

#define TINYG_FIRMWARE_VERSION		0.98						// firmware major version
#define TINYG_CONFIG_VERSION		7							// CV values started at 5 to provide backwards compatibility
#define TINYG_HARDWARE_PLATFORM		HW_PLATFORM_TINYG_V9		// hardware platform indicator (2 = Native Arduino Due)
#define TINYG_HARDWARE_VERSION		HW_VERSION_TINYGV9I			// hardware platform revision number
#define TINYG_HARDWARE_VERSION_MAX (TINYG_HARDWARE_VERSION)

/****** COMPILE-TIME SETTINGS ******/

#define __TEXT_MODE                 // enable text mode support (~14Kb) (also disables help screens)
#define __HELP_SCREENS              // enable help screens      (~3.5Kb)
#define __CANNED_TESTS              // enable $tests            (~12Kb)

/****** DEVELOPMENT SETTINGS ******/

#define __STEP_CORRECTION
#define __DIAGNOSTICS               // enables various debug functions
#define __DIAGNOSTIC_PARAMETERS     // enables system diagnostic parameters (_xx) in config_app
#define __CANNED_STARTUP            // run any canned startup moves

/******************************************************************************
 ***** TINYG APPLICATION DEFINITIONS ******************************************
 ******************************************************************************/

typedef uint16_t magic_t;		    // magic number size
#define MAGICNUM 0x12EF			    // used for memory integrity assertions
#define BAD_MAGIC(a) (a != MAGICNUM)// simple assertion test

/***** Axes, motors & PWM channels used by the application *****/
// Axes, motors & PWM channels must be defines (not enums) so #ifdef <value> can be used

#define AXES		6               // number of axes supported in this version
#define HOMING_AXES	4               // number of axes that can be homed (assumes Zxyabc sequence)
#define MOTORS		6               // number of motors on the board
#define PWMS		2               // number of supported PWM channels
#define COORDS		6               // number of supported coordinate systems (1-6)
                                    // if this changes check the config_apps table

// The following must lined up with the AXES, MOTORS and PWMS settings above
#define AXIS_X		0
#define AXIS_Y		1
#define AXIS_Z		2
#define AXIS_A		3
#define AXIS_B		4
#define AXIS_C		5
#define AXIS_U		6               // reserved
#define AXIS_V		7               // reserved
#define AXIS_W		8               // reserved

#define MOTOR_1		0               // define motor numbers and array indexes
#define MOTOR_2		1               // must be defines. enums don't work
#define MOTOR_3		2
#define MOTOR_4		3
#define MOTOR_5		4
#define MOTOR_6		5

#define PWM_1		0
#define PWM_2		1

/************************************************************************************
 ***** PLATFORM COMPATIBILITY *******************************************************
 ************************************************************************************/
//#undef __AVR
//#define __AVR
#undef __ARM
#define __ARM

/*********************
 * AVR Compatibility *
 *********************/
#ifdef __AVR

#include <avr/pgmspace.h>		// defines PROGMEM and PSTR

//typedef char char_t;			// ARM/C++ version uses uint8_t as char_t
																	// gets rely on nv->index having been set
#define GET_TABLE_WORD(a)  pgm_read_word(&cfgArray[nv->index].a)	// get word value from cfgArray
#define GET_TABLE_BYTE(a)  pgm_read_byte(&cfgArray[nv->index].a)	// get byte value from cfgArray
#define GET_TABLE_FLOAT(a) pgm_read_float(&cfgArray[nv->index].a)	// get float value from cfgArray
#define GET_TOKEN_BYTE(a)  (char_t)pgm_read_byte(&cfgArray[i].a)	// get token byte value from cfgArray

// populate the shared buffer with the token string given the index
#define GET_TOKEN_STRING(i,a) strcpy_P(a, (char *)&cfgArray[(index_t)i].token);

// get text from an array of strings in PGM and convert to RAM string
#define GET_TEXT_ITEM(b,a) strncpy_P(shared_buf,(const char *)pgm_read_word(&b[a]), MESSAGE_LEN-1)

// get units from array of strings in PGM and convert to RAM string
#define GET_UNITS(a) 	   strncpy_P(shared_buf,(const char *)pgm_read_word(&msg_units[cm_get_units_mode(a)]), MESSAGE_LEN-1)

// IO settings
#define STD_IN 	XIO_DEV_USB		// default IO settings
#define STD_OUT	XIO_DEV_USB
#define STD_ERR	XIO_DEV_USB

// String compatibility
#define strtof strtod			// strtof is not in the AVR lib

#endif // __AVR

/*********************
 * ARM Compatibility *
 *********************/
#ifdef __ARM
								// Use macros to fake out AVR's PROGMEM and other AVRisms.
#define PROGMEM					// ignore PROGMEM declarations in ARM/GCC++
#define PSTR (const char *)		// AVR macro is: PSTR(s) ((const PROGMEM char *)(s))

													// gets rely on nv->index having been set
#define GET_TABLE_WORD(a)  cfgArray[nv->index].a	// get word value from cfgArray
#define GET_TABLE_BYTE(a)  cfgArray[nv->index].a	// get byte value from cfgArray
#define GET_TABLE_FLOAT(a) cfgArray[nv->index].a	// get byte value from cfgArray
#define GET_TOKEN_BYTE(a)  cfgArray[i].a            // get token byte value from cfgArray

#define GET_TOKEN_STRING(i,a) strcpy_P(a, (char *)&cfgArray[(index_t)i].token); // populate the token string given the index

#define GET_TEXT_ITEM(b,a) b[a]						// get text from an array of strings in flash
#define GET_UNITS(a) msg_units[cm_get_units_mode(a)]

// IO settings
#define STD_IN 0				// STDIO defaults (stdio is not yet used in the ARM version)
#define STD_OUT 0
#define STD_ERR 0

/* String compatibility
 *
 * AVR GCC has "_P" variants that take PROGMEM strings as args.
 * On the ARM/GCC++ the _P functions are just aliases of the non-P variants.
 *
 * Note that we have to be sure to cast non char variables to char types when used
 * with standard functions. We must maintain const when it's required as well.
 *
 *      Example: char *ret = strcpy((char *)d, (const char *)s);
 *      The compiler will be your guide when you get it wrong. :)
 *
 * Avoid redefining global defines if possible The following inline jump functions are better.
 */
inline char* strcpy_P(char* d, const char* s) { return (char *)strcpy((char *)d, (const char *)s); }
inline char* strncpy_P(char* d, const char* s, size_t l) { return (char *)strncpy((char *)d, (const char *)s, l); }

// These we'll allow for the sake of not having to pass the variadic variables...
#define printf_P printf		// these functions want char * as inputs, not char_t *
#define fprintf_P fprintf
#define sprintf_P sprintf

#endif // __ARM

/******************************************************************************
 ***** STATUS CODE DEFINITIONS ************************************************
 ******************************************************************************/
// NB: must follow platform compatibility to manage PROGMEM and PSTR

#include "error.h"

#endif // End of include guard: TINYG2_H_ONCE
