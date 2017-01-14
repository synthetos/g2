/*
 * spindle.h - spindle driver
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2017 Alden S. Hart, Jr.
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

#ifndef SPINDLE_H_ONCE
#define SPINDLE_H_ONCE

typedef enum {
    SPINDLE_DISABLED = 0,       // spindle will not operate
    SPINDLE_PLAN_TO_STOP,       // spindle operating, plans to stop
    SPINDLE_CONTINUOUS,         // spindle operating, does not plan to stop
} spMode;
#define SPINDLE_MODE_MAX SPINDLE_CONTINUOUS

// spState is used for multiple purposes:
//  - request the spindle operation (OFF, CW, CCW)
//  - request PAUSE and RESUME
//  - run the spindle state machine for spindle wait states
//  - store current direction (1=CW, 2=CCW) in spindle.direction
typedef enum {                  // how spindle controls are presented by the Gcode parser
    SPINDLE_OFF = 0,            // M5
    SPINDLE_CW = 1,             // M3 and store CW to spindle.direction
    SPINDLE_CCW = 2,            // M4 and store CCW to spsindle.direction
    SPINDLE_PAUSE,              // request PAUSE and store PAUSED state to spindle.state
    SPINDLE_RESUME,             // request RESUME and revert spindle.state to CW, CCW
    SPINDLE_WAIT                // handle transient WAIT states
} spState;

// SPINDLE_ON is either of SPINDLE_CW or SPINDLE_CCW
//#define SPINDLE_ON(s) ((s == SPINDLE_CW) || (s == SPINDLE__CCW))  

// *** NOTE: The spindle polarity active hi/low values currently agree with ioMode in gpio.h
// These will all need to be changed to ACTIVE_HIGH = 0, ACTIVE_LOW = 1
// See: https://github.com/synthetos/g2_private/wiki/GPIO-Design-Discussion#settings-common-to-all-io-types

typedef enum {                  // Note: These values agree with 
    SPINDLE_ACTIVE_LOW = 0,     // Will set output to 0 to enable the spindle or CW direction
    SPINDLE_ACTIVE_HIGH = 1,    // Will set output to 1 to enable the spindle or CW direction
} spPolarity;

/*
typedef enum {                  // basic spindle state machine. Do not change this enum
    SPINDLE_OFF = 0,
    SPINDLE_ON = 1,             // spindle on and at speed
    SPINDLE_PAUSE = 2,          // meaning it was on and now it's off
    SPINDLE_WAITING = 3         // spindle not at speed yet
} spState;
*/

typedef enum {                  // electronic speed controller for some spindles
    ESC_ONLINE = 0,
    ESC_OFFLINE,
    ESC_LOCKOUT,
    ESC_REBOOTING,
    ESC_LOCKOUT_AND_REBOOTING,
} ESCState;

#define SPINDLE_OVERRIDE_ENABLE false
#define SPINDLE_OVERRIDE_FACTOR 1.00
#define SPINDLE_OVERRIDE_MIN 0.05       // 5%
#define SPINDLE_OVERRIDE_MAX 2.00       // 200%
#define SPINDLE_OVERRIDE_RAMP_TIME 1    // change sped in seconds

/*
 * Spindle control structure
 */

typedef struct spSpindle {

    spMode      mode;               // spindle operating mode
    spState     state;              // OFF, ON, PAUSE, RESUME, WAIT
    spState     direction;          // 1=CW, 2=CCW (subset of above state)

    float       speed;              // S in RPM
    float       speed_min;          // minimum settable spindle speed
    float       speed_max;          // maximum settable spindle speed

    spPolarity  enable_polarity;    // 0=active low, 1=active high
    spPolarity  dir_polarity;       // 0=clockwise low, 1=clockwise high
    float       dwell_seconds;      // dwell on spindle resume
    bool        pause_on_hold;      // pause on feedhold

    bool        sso_enable;         // TRUE = spindle speed override enabled (see also m48_enable in canonical machine)
    float       sso_factor;         // 1.0000 x S spindle speed. Go up or down from there
    
    // Spindle speed controller variables
    ESCState    esc_state;          // state management for ESC controller
    uint32_t    esc_boot_timer;     // When the ESC last booted up
    uint32_t    esc_lockout_timer;  // When the ESC lockout last triggered

} spSpindle_t;
extern spSpindle_t spindle;

/*
 * Global Scope Functions
 */

void spindle_init();
void spindle_reset();

stat_t spindle_control_immediate(spState control);
stat_t spindle_control_sync(spState control);
stat_t spindle_speed_immediate(float speed);    // S parameter
stat_t spindle_speed_sync(float speed);         // S parameter

//void spindle_off_immediate(void);
//void spindle_optional_pause(bool option);       // stop spindle based on system options selected
//void spindle_resume(float dwell_seconds);       // restart spindle after pause based on previous state

stat_t spindle_override_control(const float P_word, const bool P_flag); // M51
void spindle_start_override(const float ramp_time, const float override_factor);
void spindle_end_override(const float ramp_time);

stat_t sp_get_spmo(nvObj_t *nv);
stat_t sp_set_spmo(nvObj_t *nv);
stat_t sp_get_spep(nvObj_t *nv);
stat_t sp_set_spep(nvObj_t *nv);
stat_t sp_get_spdp(nvObj_t *nv);
stat_t sp_set_spdp(nvObj_t *nv);
stat_t sp_get_spph(nvObj_t *nv);
stat_t sp_set_spph(nvObj_t *nv);

stat_t sp_get_spdw(nvObj_t *nv);
stat_t sp_set_spdw(nvObj_t *nv);
stat_t sp_get_spsn(nvObj_t *nv);
stat_t sp_set_spsn(nvObj_t *nv);
stat_t sp_get_spsm(nvObj_t *nv);
stat_t sp_set_spsm(nvObj_t *nv);

stat_t sp_get_ssoe(nvObj_t* nv);
stat_t sp_set_ssoe(nvObj_t* nv);
stat_t sp_get_sso(nvObj_t* nv);
stat_t sp_set_sso(nvObj_t* nv);

stat_t sp_get_sps(nvObj_t* nv);
stat_t sp_set_sps(nvObj_t* nv);
stat_t sp_get_spe(nvObj_t* nv);
stat_t sp_set_spe(nvObj_t* nv);
stat_t sp_get_spd(nvObj_t* nv);
stat_t sp_set_spd(nvObj_t* nv);

/*--- text_mode support functions ---*/

#ifdef __TEXT_MODE

    void sp_print_spmo(nvObj_t* nv);
    void sp_print_spep(nvObj_t* nv);
    void sp_print_spdp(nvObj_t* nv);
    void sp_print_spph(nvObj_t* nv);
    void sp_print_spdw(nvObj_t* nv);
    void sp_print_spsn(nvObj_t* nv);
    void sp_print_spsm(nvObj_t* nv);
    void sp_print_ssoe(nvObj_t* nv);
    void sp_print_sso(nvObj_t* nv);
    void sp_print_sps(nvObj_t* nv);
    void sp_print_spe(nvObj_t* nv);
    void sp_print_spd(nvObj_t* nv);

#else

    #define sp_print_spmo tx_print_stub
    #define sp_print_spep tx_print_stub
    #define sp_print_spdp tx_print_stub
    #define sp_print_spph tx_print_stub
    #define sp_print_spdw tx_print_stub
    #define sp_print_spsn tx_print_stub
    #define sp_print_spsm tx_print_stub
    #define sp_print_ssoe tx_print_stub
    #define sp_print_spe tx_print_stub
    #define sp_print_sps tx_print_stub
    #define sp_print_sso tx_print_stub
    #define sp_print_spd tx_print_stub

#endif  // __TEXT_MODE

#endif  // End of include guard: SPINDLE_H_ONCE
