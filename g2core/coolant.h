/*
 * coolant.h - coolant driver
 * This file is part of the g2core project
 *
 * Copyright (c) 2015 - 2018 Alden S. Hart, Jr.
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

#ifndef COOLANT_H_ONCE
#define COOLANT_H_ONCE

typedef enum {
    COOLANT_OFF = 0,        // don't change the order. 0/1 get masked.
    COOLANT_ON = 1,
//  COOLANT_RESERVED = 2,   // not used. ALigns pause and resume with spindle enum
    COOLANT_PAUSE = 3,
    COOLANT_RESUME = 4
} coControl;
#define COOLANT_ACTION_MAX COOLANT_RESUME

// *** NOTE: The coolant polarity active hi/low values currently agree with ioMode in gpio.h
// These will all need to be changed to ACTIVE_HIGH = 0, ACTIVE_LOW = 1
// See: https://github.com/synthetos/g2_private/wiki/GPIO-Design-Discussion#settings-common-to-all-io-types

typedef enum { 
    COOLANT_ACTIVE_LOW = 0, 
    COOLANT_ACTIVE_HIGH = 1
} coPolarity;

typedef enum {              // don't change the order. used as a bitmask
    COOLANT_NONE = 0,       // none selected
    COOLANT_MIST = 1,       // mist coolant
    COOLANT_FLOOD = 2,      // flood coolant
    COOLANT_BOTH = 3        // both types ('or' of both coolants)
} coSelect;

/*
 * Coolant control structures
 */

typedef struct coCoolantChannel {
    bool        pause_enable;       // {coph:} pause on feedhold
    coControl   state;              // coolant state: coControl
    coPolarity  polarity;           // 0=active low, 1=active high
} coCoolantChannel_t;

typedef struct coCoolant {
    coCoolantChannel_t mist;        // M7 - treated as the "master" for reading pause setting
    coCoolantChannel_t flood;       // M8
} coCoolant_t;
extern coCoolant_t coolant;

/*
 * Global Scope Functions
 */

void coolant_init();
void coolant_reset();

stat_t coolant_control_immediate(coControl control, coSelect select);
stat_t coolant_control_sync(coControl control, coSelect select);

stat_t co_get_coph(nvObj_t *nv);
stat_t co_set_coph(nvObj_t *nv);

stat_t co_get_comp(nvObj_t *nv);
stat_t co_set_comp(nvObj_t *nv);
stat_t co_get_cofp(nvObj_t *nv);
stat_t co_set_cofp(nvObj_t *nv);

stat_t co_get_com(nvObj_t *nv);
stat_t co_set_com(nvObj_t *nv);
stat_t co_get_cof(nvObj_t *nv);
stat_t co_set_cof(nvObj_t *nv);

/*--- text_mode support functions ---*/

#ifdef __TEXT_MODE

void co_print_coph(nvObj_t* nv);  // coolant pause on hold
void co_print_comp(nvObj_t* nv);  // coolant polarity mist
void co_print_cofp(nvObj_t* nv);  // coolant polarity flood
void co_print_com(nvObj_t* nv);   // report mist coolant state
void co_print_cof(nvObj_t* nv);   // report flood coolant state

#else

#define co_print_coph tx_print_stub
#define co_print_comp tx_print_stub
#define co_print_cofp tx_print_stub
#define co_print_com tx_print_stub
#define co_print_cof tx_print_stub

#endif  // __TEXT_MODE

#endif  // End of include guard: COOLANT_H_ONCE
