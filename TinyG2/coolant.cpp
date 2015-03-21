/*
 * coolant.cpp - canonical machine coolant driver
 * This file is part of the TinyG project
 *
 * Copyright (c) 2015 Alden S. Hart, Jr.
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

#include "tinyg2.h"             // #1 dependency order
#include "config.h"             // #2
#include "canonical_machine.h"  // #3
#include "text_parser.h"        // #4

#include "coolant.h"
#include "planner.h"
#include "hardware.h"

/**** Allocate structures ****/

cmCoolantSingleton_t coolant;

/**** Static functions ****/

static void _exec_mist_coolant_control(float *value, float *flag);
static void _exec_flood_coolant_control(float *value, float *flag);

/*
 * coolant_init()
 * coolant_reset()
 */
void coolant_init()
{
    coolant.mist_state = COOLANT_OFF;
    coolant.flood_state = COOLANT_OFF;
}

void coolant_reset()
{
    coolant_init();
    cm_coolant_off_immediate();
}

void cm_coolant_off_immediate()     // turn off all coolant
{
    float vect[] = { 0,0,0,0,0,0 };
    _exec_flood_coolant_control(vect, vect);
}

/*
 * cm_mist_coolant_control()
 * _exec_mist_coolant_control()
 * cm_flood_coolant_control
 * _exec_flood_coolant_control
 */

#ifdef __ARM
#define _set_coolant_enable_bit_hi() coolant_enable_pin.set()
#define _set_coolant_enable_bit_lo() coolant_enable_pin.clear()
#endif
#ifdef __AVR
#define _set_coolant_enable_bit_hi() gpio_set_bit_on(COOLANT_BIT)
#define _set_coolant_enable_bit_lo() gpio_set_bit_off(COOLANT_BIT)
#endif

stat_t cm_mist_coolant_control(uint8_t mist_state)
{
    float value[AXES] = { (float)mist_state, 0,0,0,0,0 };
    mp_queue_command(_exec_mist_coolant_control, value, value);
    return (STAT_OK);
}

static void _exec_mist_coolant_control(float *value, float *flag)
{
    coolant.mist_state = (cmCoolantState)value[0];

    if (!(coolant.mist_state ^ coolant.mist_polarity)) {    // inverted XOR
        _set_coolant_enable_bit_hi();
    } else {
        _set_coolant_enable_bit_lo();
    }
}

stat_t cm_flood_coolant_control(uint8_t flood_state)
{
    float value[AXES] = { (float)flood_state, 0,0,0,0,0 };
    mp_queue_command(_exec_flood_coolant_control, value, value);
    return (STAT_OK);
}

static void _exec_flood_coolant_control(float *value, float *flag)
{
    coolant.flood_state = (cmCoolantState)value[0];

    if (!(coolant.flood_state ^ coolant.flood_polarity)) {
        _set_coolant_enable_bit_hi();
    } else {
        _set_coolant_enable_bit_lo();
        float vect[] = { 0,0,0,0,0,0 };				// turn off mist coolant
        _exec_mist_coolant_control(vect, vect);		// M9 special function
    }
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

const char fmt_coph[] PROGMEM = "[coph] coolant pause on hold%7d [0=no,1=pause_on_hold]\n";
const char fmt_comp[] PROGMEM = "[comp] coolant mist polarity%7d [0=low is ON,1=high is ON]\n";
const char fmt_cofp[] PROGMEM = "[cofp] coolant flood polarity%6d [0=low is ON,1=high is ON]\n";
const char fmt_com[] PROGMEM = "Mist coolant:%6d [0=OFF,1=ON]\n";
const char fmt_cof[] PROGMEM = "Flood coolant:%5d [0=OFF,1=ON]\n";

void cm_print_coph(nvObj_t *nv) { text_print(nv, fmt_coph);}  // TYPE_INT
void cm_print_comp(nvObj_t *nv) { text_print(nv, fmt_comp);}  // TYPE_INT
void cm_print_cofp(nvObj_t *nv) { text_print(nv, fmt_cofp);}  // TYPE_INT
void cm_print_com(nvObj_t *nv) { text_print(nv, fmt_com);}    // TYPE_INT
void cm_print_cof(nvObj_t *nv) { text_print(nv, fmt_cof);}    // TYPE_INT

#endif // __TEXT_MODE
