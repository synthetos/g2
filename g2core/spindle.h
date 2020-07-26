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

#include "error.h"  // for stat_t
#include "config.h" // for configSubtable
#include "gpio.h"   // for ioPolarity

enum spDirection {                  // how spindle controls are presented by the Gcode parser
    SPINDLE_OFF = 0,            // M5
    SPINDLE_CW = 1,             // M3 and store CW to spindle.direction
    SPINDLE_CCW = 2,            // M4 and store CCW to spsindle.direction
};

class GCodeState_t;

class ToolHead  // TODO: Move to a toolhead file
{
   public:
    virtual void init();
    virtual void reset() {
         // default to stop
         stop();
    }

    virtual void stop()    // stop - loses state - called when a job ends or is killed, even if outside a "job"
    {
        // default to setting the spindle to OFF and speed to 0
        set_direction(SPINDLE_OFF);
        set_speed(0);
    }

    virtual void pause();   // soft-stop the toolhead (usually for a feedhold) - retain all state for resume
    virtual void resume();  // resume from the pause - return STAT_EAGAIN if it's not yet ready
    virtual bool ready_to_resume() { return true; } // return true if paused and resume would not result in an error
    virtual bool busy() { return false; } // return true if motion should continue waiting for this toolhead

    // the result of an S word
    // return true if a command (and plan-to-stop) is needed, and false if not
    virtual bool set_speed(float speed) { return (true); }
    virtual float get_speed();

    // set the override value for spindle speed
    virtual bool set_override(float override) { return (true); }
    virtual float get_override();

    // enable or disable the override
    virtual bool set_override_enable(bool override_enable) { return (true); }
    virtual bool get_override_enable();

    // the result of an M3/M4/M5
    // return true if a command (and plan-to-stop) is needed, and false if not
    virtual bool set_direction(spDirection direction) { return (true); }
    virtual spDirection get_direction();

    // called from the loader right before a move, with the gcode model to use
    virtual void engage(const GCodeState_t &gm);

    virtual bool is_on();     // return if the current direction is anything but OFF, **even if paused**

    // support for legacy interfaces, overriding is optional

    virtual bool set_pwm_output(const uint8_t pwm_pin_number) { return false; };
    virtual uint8_t get_pwm_output() { return 0; };
    virtual bool set_pwm_polarity(const ioPolarity new_polarity) { return false; };
    virtual ioPolarity get_pwm_polarity() { return IO_ACTIVE_HIGH; };

    virtual bool set_enable_output(const uint8_t enable_pin_number) { return false; };
    virtual uint8_t get_enable_output() { return 0; };
    virtual bool set_enable_polarity(const ioPolarity new_polarity) { return false; };
    virtual ioPolarity get_enable_polarity() { return IO_ACTIVE_HIGH; };

    virtual bool set_direction_output(const uint8_t direction_pin_number) { return false; };
    virtual uint8_t get_direction_output() { return 0; };
    virtual bool set_direction_polarity(const ioPolarity new_polarity) { return false; };
    virtual ioPolarity get_direction_polarity() { return IO_ACTIVE_HIGH; };

    // getters and setters for optional stuff - to support legacy JSON
    virtual void set_frequency(float new_frequency) { /* do nothing */ }
    virtual float get_frequency() { return 0.0; }

    virtual void set_speed_min(float new_speed_min) { /* do nothing */ }
    virtual float get_speed_min() { return 0.0; }
    virtual void set_speed_max(float new_speed_max) { /* do nothing */ }
    virtual float get_speed_max() { return 0.0; }
    virtual void set_speed_change_per_tick(float new_speed_change_per_tick) { /* do nothing */ }
    virtual float get_speed_change_per_tick() { return 0.0; }
    virtual void set_spinup_delay(float new_spinup_delay) { /* do nothing */ }
    virtual float get_spinup_delay() { return 0.0; }

    virtual void set_cw_speed_lo(float new_speed_lo) { /* do nothing */ }
    virtual float get_cw_speed_lo() { return 0.0; }
    virtual void set_cw_speed_hi(float new_speed_hi) { /* do nothing */ }
    virtual float get_cw_speed_hi() { return 0.0; }
    virtual void set_cw_phase_lo(float new_phase_lo) { /* do nothing */ }
    virtual float get_cw_phase_lo() { return 0.0; }
    virtual void set_cw_phase_hi(float new_phase_hi) { /* do nothing */ }
    virtual float get_cw_phase_hi() { return 0.0; }

    virtual void set_ccw_speed_lo(float new_speed_lo) { /* do nothing */ }
    virtual float get_ccw_speed_lo() { return 0.0; }
    virtual void set_ccw_speed_hi(float new_speed_hi) { /* do nothing */ }
    virtual float get_ccw_speed_hi() { return 0.0; }
    virtual void set_ccw_phase_lo(float new_phase_lo) { /* do nothing */ }
    virtual float get_ccw_phase_lo() { return 0.0; }
    virtual void set_ccw_phase_hi(float new_phase_hi) { /* do nothing */ }
    virtual float get_ccw_phase_hi() { return 0.0; }

    virtual void set_phase_off(float new_phase_off) { /* do nothing */ }
    virtual float get_phase_off() { return 0.0; }
};

/*
 * Global Scope Functions
 */

ToolHead *toolhead_for_tool(uint8_t tool); // return the correct toolhead for this given tool number - MAY retrun nullptr!

void spindle_init();   // init all known toolheads
void spindle_set_toolhead(ToolHead *toolhead); // set the active toolhead
void spindle_reset();  // reset the current toolhead

void spindle_stop();
void spindle_pause();
void spindle_resume();
stat_t spindle_set_speed(float speed);                // S parameter - returns STAT_EAGAIN if a command should be queued
float  spindle_get_speed();                           // return current speed - in the same units as the S parameter
stat_t spindle_set_direction(spDirection direction);  // M3/M4/M5 - returns STAT_EAGAIN if a command should be queued
spDirection spindle_get_direction();                  // return if any fo M3/M4/M5 are active (actual, not gcode model)

void spindle_engage(const GCodeState_t &gm);          // called from the loader right before a move, with the gcode model to use

bool is_spindle_ready_to_resume();  // if the spindle can resume at this time, return true
bool is_spindle_on_or_paused();     // returns if the spindle is on or paused - IOW would it try to resume from feedhold
bool is_a_toolhead_busy();          // returns true if motion should continue to hold for ANY toolhead

// stat_t spindle_override_control(const float P_word, const bool P_flag);  // M51
// void spindle_start_override(const float ramp_time, const float override_factor);
// void spindle_end_override(const float ramp_time);

const configSubtable *const getSpindleConfig_1();
const configSubtable *const getP1Config_1();

#endif  // End of include guard: SPINDLE_H_ONCE
