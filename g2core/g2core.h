/*
 * g2core.h - g2core main header file
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2019 Alden S. Hart, Jr.
 * Copyright (c) 2010 - 2019 Robert Giseburt
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

#ifndef G2CORE_H_ONCE
#define G2CORE_H_ONCE

// common system includes
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "error.h"                  // Status code definitions and strings
#include "g2core_info.h"            // see this file for build number and other identifying information

/****** COMPILE-TIME SETTINGS ******/

#define __TEXT_MODE                 // enable text mode support (~14Kb) (also disables help screens)
#define __HELP_SCREENS              // enable help screens      (~3.5Kb)
#define __USER_DATA                 // enable user defined data groups
#define __STEP_CORRECTION           // enable virtual encoder step correction

/****** DEVELOPMENT SETTINGS ******/

#define __DIAGNOSTICS               // enables various debug functions
#define __DIAGNOSTIC_PARAMETERS     // enables system diagnostic parameters (_xx) in config_app

/******************************************************************************
 ***** APPLICATION DEFINITIONS ************************************************
 ******************************************************************************/

typedef uint16_t magic_t;		        // magic number size
#define MAGICNUM 0x12EF			        // used for memory integrity assertions
#define BAD_MAGIC(a) (a != MAGICNUM)    // simple assertion test

// Note: If you change COORDS you must adjust the entries in cfgArray table in config.c

#include "hardware.h"                  // Status code definitions and strings
#ifndef AXES
    #define AXES 9      // number of axes supported in this version
#elif (AXES != 6 && AXES != 9)
#error AXES must be undefined (defaulting to 9) or defined as 6 or 9
#endif
#define HOMING_AXES 4   // number of axes that can be homed (assumes Zxyabc sequence)
#define COORDS 6        // number of supported coordinate systems (index starts at 1)
#ifndef TOOLS
    #define TOOLS 32        // number of entries in tool table (index starts at 1)
#endif

typedef enum {
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
#if (AXES == 9)
    AXIS_U,
    AXIS_V,
    AXIS_W,
#endif
    AXIS_A,
    AXIS_B,
    AXIS_C,
    AXIS_COREXY_A = AXIS_X, // CoreXY uses A and B
    AXIS_COREXY_B = AXIS_Y,
    AXIS_4WIRE_A = AXIS_X, // 4Wire uses A, B, C, D
    AXIS_4WIRE_B = AXIS_Y, // 4Wire uses A, B, C, D
    AXIS_4WIRE_C = AXIS_Z, // 4Wire uses A, B, C, D
    AXIS_4WIRE_D = AXIS_A, // 4Wire uses A, B, C, D
    AXIS_4WIRE_Z = AXIS_B, // 4Wire uses A, B, C, D
} cmAxes;

#if (AXES == 9)
constexpr cmAxes LAST_LINEAR_AXIS = AXIS_W;
#else
constexpr cmAxes LAST_LINEAR_AXIS = AXIS_Z;
#endif

typedef enum {  // external representation of axes (used in initialization)
    AXIS_X_EXTERNAL = 0,
    AXIS_Y_EXTERNAL,
    AXIS_Z_EXTERNAL,
    AXIS_A_EXTERNAL,
    AXIS_B_EXTERNAL,
    AXIS_C_EXTERNAL,
#if (AXES == 9)
    AXIS_U_EXTERNAL,
    AXIS_V_EXTERNAL,
    AXIS_W_EXTERNAL
#endif
} cmAxesExternal;

typedef enum {
    OFS_I = 0,
    OFS_J,
    OFS_K
} cmIJKOffsets;

typedef enum {
    MOTOR_1 = 0,
    MOTOR_2,
    MOTOR_3,
    MOTOR_4,
    MOTOR_5,
    MOTOR_6,
    MOTOR_7,    // reserved
    MOTOR_8,    // reserved
    MOTOR_9     // reserved
} cmMotors;

typedef enum {
    PWM_1 = 0,
    PWM_2
} cmPWMs;

/**********************************************************************************
 * AVR/ARM Compatibility
 *
 * This is a holdover from AVR/ARM compatibility work in earlier builds
 * and may be removed at some later time.
 */                                                 // gets rely on nv->index having been set
#define GET_TABLE_WORD(a)  cfgArray[nv->index].a	// get word value from cfgArray
#define GET_TABLE_BYTE(a)  cfgArray[nv->index].a	// get byte value from cfgArray
#define GET_TABLE_FLOAT(a) cfgArray[nv->index].a	// get byte value from cfgArray
#define GET_TOKEN_BYTE(a)  cfgArray[i].a            // get token byte value from cfgArray
#define GET_TEXT_ITEM(b,a) b[a]						// get text from an array of strings in flash
#define GET_UNITS(a) msg_units[cm_get_units_mode(a)]
#define GET_TOKEN_STRING(i,a) strcpy(a, (char *)&cfgArray[(index_t)i].token); // populate the token string given the index

#endif // End of include guard: G2CORE_H_ONCE
