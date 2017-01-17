/*
 * coolant.cpp - canonical machine coolant driver
 * This file is part of the g2core project
 *
 * Copyright (c) 2015 - 2017 Alden S. Hart, Jr.
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

#include "g2core.h"             // #1 dependency order
#include "config.h"             // #2
#include "canonical_machine.h"  // #3
#include "text_parser.h"        // #4

#include "coolant.h"
#include "planner.h"
#include "hardware.h"
#include "util.h"

/**** Allocate structures ****/

coCoolant_t coolant;

/**** Static functions ****/

static void _exec_coolant_control(float* value, bool* flag);

/*
 * coolant_init()
 * coolant_reset()
 */
void coolant_init() {
    coolant.mist_state  = COOLANT_OFF;
    coolant.flood_state = COOLANT_OFF;
}

void coolant_reset() {
    coolant_init();
    coolant_control_immediate(COOLANT_OFF, COOLANT_BOTH);
}

/*
 * coolant_control_immediate() - execute coolant control immediately
 * coolant_control_sync()      - queue a coolant control to the planner buffer
 * _exec_coolant_control()     - actually execute the coolant command
 */

stat_t coolant_control_immediate(coControl control, coSelect select)
{
    float value[] = { (float)control };
    bool flags[] = { (select & COOLANT_MIST), (select & COOLANT_FLOOD) };
    _exec_coolant_control(value, flags);
    return(STAT_OK);       
}    

stat_t coolant_control_sync(coControl control, coSelect select)
{
    // skip the PAUSE operation if pause-enable is not enabled (pause-on-hold)
    if ((control == COOLANT_PAUSE) && (!coolant.pause_enable)) {
        return (STAT_OK);
    }
    
    // queue the coolant control
    float value[] = { (float)control };
    bool flags[]  = { (select & COOLANT_MIST), (select & COOLANT_FLOOD) };
    mp_queue_command(_exec_coolant_control, value, flags);
    return(STAT_OK);
}

static void _exec_coolant_control(float* value, bool* flag) {
    
    coControl control = (coControl)value[0];
    if (control > COOLANT_ACTION_MAX) {
        return;
    }
    
    bool action;            // set true to perform the action
    int8_t enable_bit;
//    coControl previous_state = control.mist_state;
    
    if (flag[0]) {          // Mist, M7
        action = true;
        enable_bit = 0;
        switch (control) {
            case COOLANT_OFF: {
                coolant.mist_state = COOLANT_OFF;
                break;
            }
            case COOLANT_ON: {
                coolant.mist_state = COOLANT_ON;
                enable_bit = 1;
                break;
            }
            case COOLANT_PAUSE: {
                if (coolant.mist_state == COOLANT_ON) {
                    coolant.mist_state = COOLANT_PAUSE;
                } else {
                    action = false;
                }
                break;
            }
            case COOLANT_RESUME: {
                if (coolant.mist_state == COOLANT_PAUSE) {
                    coolant.mist_state = COOLANT_ON;
                    enable_bit = 1;
                } else {
                    action = false;
                }
                break;
            }
        }
        if (action) {
            if (!(enable_bit ^ coolant.mist_polarity)) {  // inverted XOR
                mist_enable_pin.set();
            } else {
                mist_enable_pin.clear();
            }
        }
    }
    if (flag[1]) {      // Flood, M8
        action = true;
        enable_bit = 0;
        switch (control) {
            case COOLANT_OFF: {
                coolant.flood_state = COOLANT_OFF;
                break;
            }
            case COOLANT_ON: {
                coolant.flood_state = COOLANT_ON;
                enable_bit = 1;
                break;
            }
            case COOLANT_PAUSE: {
                if (coolant.flood_state == COOLANT_ON) {
                    coolant.flood_state = COOLANT_PAUSE;
                } else {
                    action = false;
                }                
                break;
            }
            case COOLANT_RESUME: {
                if (coolant.flood_state == COOLANT_PAUSE) {
                    coolant.flood_state = COOLANT_ON;
                    enable_bit = 1;
                } else {
                    action = false;
                }
                break;
            }
        }
        if (action) {
            if (!(enable_bit ^ coolant.flood_polarity)) {  // inverted XOR
                flood_enable_pin.set();
            } else {
                flood_enable_pin.clear();
            }
        }        
    }    
} 

/***********************************************************************************
 **** Coolant Settings *************************************************************
 ***********************************************************************************/

stat_t co_get_coph(nvObj_t *nv) { return(get_int(nv, coolant.pause_enable)); }
stat_t co_set_coph(nvObj_t *nv) { return(set_int(nv, (uint8_t &)coolant.pause_enable, 0, 1)); }
stat_t co_get_comp(nvObj_t *nv) { return(get_int(nv, coolant.mist_polarity)); }
stat_t co_set_comp(nvObj_t *nv) { return(set_int(nv, (uint8_t &)coolant.mist_polarity, 0, 1)); }
stat_t co_get_cofp(nvObj_t *nv) { return(get_int(nv, coolant.flood_polarity)); }
stat_t co_set_cofp(nvObj_t *nv) { return(set_int(nv, (uint8_t &)coolant.flood_polarity, 0, 1)); }

stat_t co_get_com(nvObj_t *nv) { return(get_int(nv, coolant.flood_state)); }
stat_t co_set_com(nvObj_t *nv) { return(coolant_control_immediate((coControl)nv->value, COOLANT_MIST)); }
stat_t co_get_cof(nvObj_t *nv) { return(get_float(nv, coolant.mist_state)); }
stat_t co_set_cof(nvObj_t *nv) { return(coolant_control_immediate((coControl)nv->value, COOLANT_FLOOD)); }

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

const char fmt_coph[] = "[coph] coolant pause on hold%7d [0=no,1=pause_on_hold]\n";
const char fmt_comp[] = "[comp] coolant mist polarity%7d [0=low is ON,1=high is ON]\n";
const char fmt_cofp[] = "[cofp] coolant flood polarity%6d [0=low is ON,1=high is ON]\n";
const char fmt_com[]  = "[com]  coolant mist%16d [0=OFF,1=ON]\n";
const char fmt_cof[]  = "[cof]  coolant flood%15d [0=OFF,1=ON]\n";

void co_print_coph(nvObj_t* nv) { text_print(nv, fmt_coph); }  // TYPE_INT
void co_print_comp(nvObj_t* nv) { text_print(nv, fmt_comp); }  // TYPE_INT
void co_print_cofp(nvObj_t* nv) { text_print(nv, fmt_cofp); }  // TYPE_INT
void co_print_com(nvObj_t* nv) { text_print(nv, fmt_com); }    // TYPE_INT
void co_print_cof(nvObj_t* nv) { text_print(nv, fmt_cof); }    // TYPE_INT

#endif  // __TEXT_MODE
