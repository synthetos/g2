/*
 * spindle.h - spindle driver
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
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

typedef enum {				        // spindle state settings (See hardware.h for bit settings)
    SPINDLE_OFF = 0,
    SPINDLE_CW,
    SPINDLE_CCW,
} spSpindleState;

typedef enum {
    SPINDLE_ACTIVE_LOW = 0,
    SPINDLE_ACTIVE_HIGH
} spPolarity;

typedef enum {
    SPINDLE_NORMAL = 0,
    SPINDLE_PAUSED,
} spSpindlePause;

typedef enum {
    ESC_ONLINE = 0,
    ESC_OFFLINE,
    ESC_LOCKOUT,
    ESC_REBOOTING,
    ESC_LOCKOUT_AND_REBOOTING,
} cmESCState;

/*
 * Spindle control structure
 */

typedef struct spSpindleSingleton {
    // configuration
    spSpindlePause pause_on_hold;   // pause on feedhold
    spPolarity polarity_enable;     // 0=active low, 1=active high
    spPolarity polarity_dir;        // 0=clockwise low, 1=clockwise high
    float dwell_seconds;            // dwell on spindle resume
//    float override_factor;          // 1.0000 x S spindle speed. Go up or down from there
//    uint8_t override_enable;        // TRUE = override enabled

    // state variables
    float speed;
    spSpindleState state;           // current spindle state, OFF, CW, CCW. Might be paused, though
    spSpindlePause pause;           // pause state - applies to spindle state, above

    cmESCState esc_state;           // state management for ESC controller
    uint32_t esc_boot_timer;        // When the ESC last booted up
    uint32_t esc_lockout_timer;     // When the ESC lockout last triggered

} spSpindleton_t;
extern spSpindleton_t spindle;

/*
 * Global Scope Functions
 */

void cm_spindle_init();
stat_t cm_set_spindle_speed(float speed);			    // S parameter
stat_t cm_spindle_control(uint8_t spindle_state);// M3, M4, M5 integrated spindle control
void cm_spindle_control_immediate(spSpindleState spindle_state); //like cm_spindle_control but not synchronized to planner
void cm_spindle_optional_pause(bool option);            // stop spindle based on system options selected
void cm_spindle_resume(float dwell_seconds);            // restart spindle after pause based on previous state

//stat_t cm_spindle_override_enable(uint8_t flag);    // M51
//stat_t cm_spindle_override_factor(uint8_t flag);    // M51.1

/*--- text_mode support functions ---*/

#ifdef __TEXT_MODE

    void cm_print_spo(nvObj_t *nv);
    void cm_print_spd(nvObj_t *nv);
    void cm_print_sph(nvObj_t *nv);
    void cm_print_sdw(nvObj_t *nv);
    void cm_print_spc(nvObj_t *nv);
    void cm_print_sps(nvObj_t *nv);

#else

    #define cm_print_spo tx_print_stub
    #define cm_print_spd tx_print_stub
    #define cm_print_sph tx_print_stub
    #define cm_print_sdw tx_print_stub
    #define cm_print_spc tx_print_stub
    #define cm_print_sps tx_print_stub

#endif // __TEXT_MODE

#endif	// End of include guard: SPINDLE_H_ONCE
