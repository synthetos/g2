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
#include "util.h"

/**** Allocate structures ****/

cmCoolantSingleton_t coolant;

/**** Static functions ****/

static void _exec_coolant_control(float *value, float *flags);

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

/*
 * cm_coolant_off_immediate() - turn off all coolant
 * cm_coolant_optional_pause() - pause coolants if options are true
 * cm_coolant_resume() - restart paused coolants
 */

void cm_coolant_off_immediate()
{
    float value[] = { 0,0,0,0,0,0 };
    float flags[] = { 1,1,0,0,0,0 };
    _exec_coolant_control(value, flags);
}

void cm_coolant_optional_pause(bool option)
{
    if (!option) { return; }    // don't pause if they haven't selected the option

    float value[] = { 0,0,0,0,0,0 };
    float flags[] = { 0,0,0,0,0,0 };

    if (coolant.flood_state) {
        flags[COOLANT_FLOOD] = 1.0;        
        coolant.flood_pause = COOLANT_PAUSE;    // mark as paused
    }
    if (coolant.mist_state) {
        flags[COOLANT_MIST] = 1.0;
        coolant.mist_pause = COOLANT_PAUSE;     // mark as paused
    }
    _exec_coolant_control(value, flags);        // execute (w/o changing local state)
}

void cm_coolant_resume()
{
    float value[] = { 0,0,0,0,0,0 };
    float flags[] = { 0,0,0,0,0,0 };

    if (coolant.flood_pause == COOLANT_PAUSE) {
        flags[COOLANT_FLOOD] = 1.0;
        value[COOLANT_FLOOD] = (float)coolant.flood_state;
        coolant.flood_pause = COOLANT_NORMAL;       // mark as not paused
    }
    if (coolant.mist_pause == COOLANT_PAUSE) {
        flags[COOLANT_MIST] = 1.0;
        value[COOLANT_MIST] = (float)coolant.flood_state;
        coolant.mist_pause = COOLANT_NORMAL;        // mark as not paused
    }
    _exec_coolant_control(value, flags);            // execute (w/o changing local state)
}

#ifdef __ARM
#define _set_coolant_enable_bit_hi() coolant_enable_pin.set()
#define _set_coolant_enable_bit_lo() coolant_enable_pin.clear()
#endif
#ifdef __AVR
#define _set_coolant_enable_bit_hi() gpio_set_bit_on(COOLANT_BIT)
#define _set_coolant_enable_bit_lo() gpio_set_bit_off(COOLANT_BIT)
#endif

/*
 * cm_mist_coolant_control() - access points from Gcode parser
 * cm_flood_coolant_control() - access points from Gcode parser
 * _exec_coolant_control() - combined flood and mist coolant control
 *
 *  - value[0] is flood state
 *  - value[1] is mist state
 *  - uses flags to determine which to run
 */

stat_t cm_flood_coolant_control(uint8_t flood_state)
{
    float value[] = { (float)flood_state, 0,0,0,0,0 };
    float flags[] = { 1,0,0,0,0,0 };
    mp_queue_command(_exec_coolant_control, value, flags);
    return (STAT_OK);
}

stat_t cm_mist_coolant_control(uint8_t mist_state)
{
    float value[] = { 0, (float)mist_state, 0,0,0,0 };
    float flags[] = { 0,1,0,0,0,0 };
    mp_queue_command(_exec_coolant_control, value, flags);
    return (STAT_OK);
}

static void _exec_coolant_control(float *value, float *flags)
{
    if (fp_TRUE(flags[COOLANT_FLOOD])) {
        coolant.flood_state = (cmCoolantState)value[COOLANT_FLOOD];
        if (!(coolant.flood_state ^ coolant.flood_polarity)) {    // inverted XOR
            _set_coolant_enable_bit_hi();
        } else {
            _set_coolant_enable_bit_lo();
        }        
    }
    if (fp_TRUE(flags[COOLANT_MIST])) {
        coolant.mist_state = (cmCoolantState)value[COOLANT_MIST];
        if (!(coolant.mist_state ^ coolant.mist_polarity)) {
            _set_coolant_enable_bit_hi();
        } else {
            _set_coolant_enable_bit_lo();
        }
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
