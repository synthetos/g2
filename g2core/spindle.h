/*
 * spindle.h - spindle driver
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2018 Alden S. Hart, Jr.
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

// spControl enum is used for multiple purposes:
//  - request a spindle action (OFF, CW, CCW, PAUSE, RESUME)
//  - keep current spindle state in spindle.state
//  - store as current direction (CW/CCW) in spindle.direction
//  - enumerate internal actions such as NOP, REV that are neither states nor controls
enum spControl {                  // how spindle controls are presented by the Gcode parser
    SPINDLE_OFF = 0,            // M5
    SPINDLE_CW = 1,             // M3 and store CW to spindle.direction
    SPINDLE_CCW = 2,            // M4 and store CCW to spsindle.direction
    SPINDLE_PAUSE = 3,          // request PAUSE and store PAUSED state to spindle.state
    SPINDLE_RESUME = 4,         // request RESUME and revert spindle.state to CW, CCW
    SPINDLE_NOP,                // no operation
    SPINDLE_REV                 // operation to reverse spindle direction
};
#define SPINDLE_ACTION_MAX SPINDLE_RESUME

/*
 * Global Scope Functions
 */

void spindle_init();
void spindle_reset();

stat_t spindle_control_immediate(spControl control);
stat_t spindle_control_sync(spControl control);
stat_t spindle_speed_immediate(float speed);    // S parameter
stat_t spindle_speed_sync(float speed);         // S parameter

bool is_spindle_ready_to_resume(); // if the spindle can resume at this time, return true
bool is_spindle_on_or_paused(); // returns if the stepper is on or paused - IOW would it try to resume from feedhold
bool do_spindle_speed_ramp_from_systick(); // used only in systick call from stepper.cpp

stat_t spindle_override_control(const float P_word, const bool P_flag);  // M51
void spindle_start_override(const float ramp_time, const float override_factor);
void spindle_end_override(const float ramp_time);

const configSubtable * const getSpindleConfig_1();

#endif  // End of include guard: SPINDLE_H_ONCE
