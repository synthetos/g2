/*
 * settings.h - default runtime settings
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2018 Alden S. Hart Jr.
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
 *	Application settings that also shouldn't need to be changed are in g2core.h
 */

#ifndef SETTINGS_H_ONCE
#define SETTINGS_H_ONCE

// Defines that need to be here instead of a more logical place like canonical_machine.h

#define RADIUS_MIN     (0.0001)   // minimum value for ABC radius settings

/**** MACHINE PROFILES ******************************************************
 *
 * Provide an optional SETTINGS_FILE in the makefile or compiler command line:
 *  SETTINGS_FILE="settings_shopbot_test.h"
 *
 * settings_default.h provides values if no SETTINGS_FILE is selected,
 * or if any values are missing from the SETTINGS_FILE file. 
 */

// This file sets up the compile-time defaults
#ifdef SETTINGS_FILE
#define SETTINGS_FILE_PATH <settings/SETTINGS_FILE>
#include SETTINGS_FILE_PATH
#endif

// Anything not set by the above file is set by this one...
// The defaults below generally disable that function
#include "settings/settings_default.h"

// compile-time assertions - mostly checking the settings are not impossible

#define stringify2(a) #a
#define stringify(a) stringify2(a)

//static_assert ( bool_constexpr , message )    // bool_constexpr must be true or assertion will fail
static_assert ( (A_RADIUS > RADIUS_MIN), "A axis radius must be more than " stringify(RADIUS_MIN) ", but is " stringify(A_RADIUS) );
static_assert ( (B_RADIUS > RADIUS_MIN), "B axis radius must be more than " stringify(RADIUS_MIN) ", but is " stringify(B_RADIUS) );
static_assert ( (C_RADIUS > RADIUS_MIN), "C axis radius must be more than " stringify(RADIUS_MIN) ", but is " stringify(C_RADIUS) );

#undef stringify 
#undef stringify2

#endif  // End of include guard: SETTINGS_H_ONCE
