/*
 * esc_spindle.h - toolhead driver for a ESC-driven brushless spindle
 * This file is part of the g2core project
 *
 * Copyright (c) 2020 Robert Giseburt
 * Copyright (c) 2020 Alden S. Hart, Jr.
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

#ifndef ESC_SPINDLE_H_ONCE
#define ESC_SPINDLE_H_ONCE

#include "spindle.h"
#include "stepper.h" // for st_request_load_move
#include "util.h" // for fp_NE
#include "safety_manager.h" // for safety_manager

/* A few notes:
 *
 * This is primarily for an ESC(electronic speed controller)-based spindle, where a brushless motor
 * is used as the spindle.
 *
 * Generally, the ESC does not get direction and enable signals, and ONLY honors the PWM output.
 * Also note that many ESCs can NOT reverse, and will only go one direction.
 *
 * We still handle those pins here for the sake of compatibility and status display (LEDs on those pins) and debugging.
 *
 */


// class declaration
// note implementation is after
class ESCSpindle : public ToolHead {
    spDirection direction;        // direction
    float speed;                  // S in RPM
    float speed_actual;           // actual speed (during speed ramping)

    float speed_min;              // minimum settable spindle speed
    float speed_max;              // maximum settable spindle speed

    bool paused;                  // true if paused, false is not

    float speed_change_per_tick;  // speed ramping rate per tick (ms)
    float spinup_delay;           // optional delay on spindle start (set to 0 to disable)

    struct speedToPhase {
        float speed_lo;              // minimum spindle speed [0..N]
        float speed_hi;              // maximum spindle speed

        float phase_lo;              // pwm phase at minimum spindle speed, clamped [0..1]
        float phase_hi;              // pwm phase at maximum spindle speed, clamped [0..1]

        // convert a speed value in the range of (speed_lo .. speed_hi)
        // to a value in the range of (phase_lo .. phase_hi)
        float speed_to_phase(float speed) {
            speed = (std::max(speed_lo, std::min(speed_hi, speed)) - speed_lo) / (speed_hi - speed_lo);
            return (speed * (phase_hi - phase_lo)) + phase_lo;
        }
    };

    speedToPhase cw;                // clockwise speed and phase settings
    speedToPhase ccw;               // counter-clockwise speed and phase settings

    float phase_off;                // pwm phase when spindle is disabled

    uint8_t pwm_output_num;
    gpioDigitalOutput *pwm_output = nullptr;
    uint8_t enable_output_num;
    gpioDigitalOutput *enable_output = nullptr;
    uint8_t direction_output_num;
    gpioDigitalOutput *direction_output = nullptr;

    Motate::SysTickEvent spindle_systick_event{[&] {
                                                   bool done = false;
                                                   if (paused) {
                                                       // paused may have changed since this handler was registered
                                                       speed_actual = 0; // just in case there was a race condition
                                                       done = true;
                                                   } else if (fp_NE(speed, speed_actual)) {
                                                       if (speed_actual < speed) {
                                                           // spin up
                                                           speed_actual += speed_change_per_tick;
                                                           if (speed_actual > speed) {
                                                               speed_actual = speed;
                                                               done = true;
                                                           }
                                                       } else {
                                                           // spin down
                                                           speed_actual -= speed_change_per_tick;
                                                           if (speed_actual < speed) {
                                                               speed_actual = speed;
                                                               done = true;
                                                           }
                                                       }
                                                   } else {
                                                       done = true;
                                                   }
                                                   set_pwm_value();
                                                   if (done) {
                                                       SysTickTimer.unregisterEvent(&spindle_systick_event);
                                                       st_request_load_move();  // request to load the next move
                                                   }
                                               },
                                               nullptr};

    void set_pwm_value(); // using all of the settings, set the value fo the pwm pin
    void complete_change(); // after an engage or resume, handle the rest

   public:
    // constructor - provide it with the default output pins - 0 means no pin
    ESCSpindle(const uint8_t pwm_pin_number, const uint8_t enable_pin_number, const uint8_t direction_pin_number, const float change_per_tick);

    // ToolHead overrides
    void init() override;

    void pause() override;          // soft-stop the toolhead (usually for a feedhold) - retain all state for resume
    void resume() override;         // resume from the pause - return STAT_EAGAIN if it's not yet ready
    bool ready_to_resume() override;  // return true if paused and resume would not result in an error
    bool busy() override;             // return true if motion should continue waiting for this toolhead

    // the result of an S word
    // DON'T override set_speed - use engage instead
    float get_speed() override;

    // the result of an M3/M4/M5
    // DON'T override set_direction - use engage instead
    spDirection get_direction() override;

    void stop() override;

    // called from the loader right before a move, with the gcode model to use
    void engage(const GCodeState_t &gm) override;

    bool is_on() override;  // return if the current direction is anything but OFF, **even if paused**

    void set_pwm_output(const uint8_t pwm_pin_number) override;
    void set_enable_output(const uint8_t enable_pin_number) override;
    void set_direction_output(const uint8_t direction_pin_number) override;

    void set_frequency(float new_frequency)override;
    float get_frequency() override;

    // trivial getters and setters - inlined
    void set_speed_min(float new_speed_min) override { speed_min = new_speed_min; }
    float get_speed_min() override { return speed_min; }
    void set_speed_max(float new_speed_max) override { speed_max = new_speed_max; }
    float get_speed_max() override { return speed_max; }
    void set_speed_change_per_tick(float new_speed_change_per_tick) override { speed_change_per_tick = new_speed_change_per_tick; }
    float get_speed_change_per_tick() override { return speed_change_per_tick; }
    void set_spinup_delay(float new_spinup_delay) override { spinup_delay = new_spinup_delay; }
    float get_spinup_delay() override { return spinup_delay; }

    void set_cw_speed_lo(float new_speed_lo) override { cw.speed_lo = new_speed_lo; }
    float get_cw_speed_lo() override { return cw.speed_lo; }
    void set_cw_speed_hi(float new_speed_hi) override { cw.speed_hi = new_speed_hi; }
    float get_cw_speed_hi() override { return cw.speed_hi; }
    void set_cw_phase_lo(float new_phase_lo) override { cw.phase_lo = new_phase_lo; }
    float get_cw_phase_lo() override { return cw.phase_lo; }
    void set_cw_phase_hi(float new_phase_hi) override { cw.phase_hi = new_phase_hi; }
    float get_cw_phase_hi() override { return cw.phase_hi; }

    void set_ccw_speed_lo(float new_speed_lo) override { ccw.speed_lo = new_speed_lo; }
    float get_ccw_speed_lo() override { return ccw.speed_lo; }
    void set_ccw_speed_hi(float new_speed_hi) override { ccw.speed_hi = new_speed_hi; }
    float get_ccw_speed_hi() override { return ccw.speed_hi; }
    void set_ccw_phase_lo(float new_phase_lo) override { ccw.phase_lo = new_phase_lo; }
    float get_ccw_phase_lo() override { return ccw.phase_lo; }
    void set_ccw_phase_hi(float new_phase_hi) override { ccw.phase_hi = new_phase_hi; }
    float get_ccw_phase_hi() override { return ccw.phase_hi; }

    void set_phase_off(float new_phase_off) override { phase_off = new_phase_off; }
    float get_phase_off() override { return phase_off; }
};

// method implementations follow

ESCSpindle::ESCSpindle(const uint8_t pwm_pin_number, const uint8_t enable_pin_number,
                       const uint8_t direction_pin_number, const float change_per_tick)
    : speed_change_per_tick{change_per_tick},
      pwm_output_num{pwm_pin_number},
      enable_output_num{enable_pin_number},
      direction_output_num{direction_pin_number} {}

void ESCSpindle::init()
{
    // TODO - ensure outputs are withing range
    set_pwm_output(pwm_output_num);
    set_enable_output(enable_output_num);
    set_direction_output(direction_output_num);
}

void ESCSpindle::pause() {
    if (paused) {
        return;
    }

    paused = true;
    this->complete_change();
}

void ESCSpindle::resume() {
    if (!paused) {
        return;
    }

    paused = false;
    this->complete_change();
}

bool ESCSpindle::ready_to_resume() { return paused && safety_manager->ok_to_spindle(); }
bool ESCSpindle::busy() {
    // return true when not paused, on, and ramping up to speed
    if (paused || (direction == SPINDLE_OFF) || fp_EQ(speed, speed_actual)) {
        return false;
    }
    return true;
}

// DON'T override set_speed - use engage instead
float ESCSpindle::get_speed() { return speed_actual; }

// DON'T override set_direction - use engage instead
spDirection ESCSpindle::get_direction() { return direction; }

void ESCSpindle::stop() {
    paused = false;
    speed = 0;
    direction = SPINDLE_OFF;

    this->complete_change();
}

// called from a command that was queued when the default set_speed and set_direction returned STAT_EAGAIN
// ALSO called from the loader right before a move
// we are handed the gcode model to use
void ESCSpindle::engage(const GCodeState_t &gm) {
    if ((direction == gm.spindle_direction) && fp_EQ(speed, gm.spindle_speed)) {
        // nothing changed
        return;
    }

    // special handling for reversals - we set the speed to zero and ramp up
    if ((gm.spindle_direction != direction) && (direction != SPINDLE_OFF) && (gm.spindle_direction != SPINDLE_OFF)) {
        speed_actual = 0;
    }

    speed = gm.spindle_speed;
    direction = gm.spindle_direction;

    // handle the rest
    this->complete_change();
}

bool ESCSpindle::is_on() { return (direction != SPINDLE_OFF); }

// ESCSpindle-specific functions
void ESCSpindle::set_pwm_output(const uint8_t pwm_pin_number) {
    if (pwm_pin_number == 0) {
        pwm_output = nullptr;
    } else {
        pwm_output = d_out[pwm_pin_number - 1];
        pwm_output->setEnabled(IO_ENABLED);
        // set the frequency on the output -- not here
        // set the polarity on the output -- not here
    }
}
void ESCSpindle::set_enable_output(const uint8_t enable_pin_number) {
    if (enable_pin_number == 0) {
        enable_output = nullptr;
    } else {
        enable_output = d_out[enable_pin_number - 1];
        enable_output->setEnabled(IO_ENABLED);
        // set the polarity on the output -- not here
    }
}
void ESCSpindle::set_direction_output(const uint8_t direction_pin_number) {
    if (direction_pin_number == 0) {
        direction_output = nullptr;
    } else {
        direction_output = d_out[direction_pin_number - 1];
        direction_output->setEnabled(IO_ENABLED);
        // set the polarity on the output -- not here
    }
}

void ESCSpindle::set_frequency(float new_frequency)
{
    if (pwm_output) {
        pwm_output->setFrequency(new_frequency);
    }
}
float ESCSpindle::get_frequency()
{
    if (pwm_output) {
        return pwm_output->getFrequency();
    }
    return 0.0;
}

// Private functions

void ESCSpindle::set_pwm_value() {
    if (pwm_output == nullptr) {
        return;
    }
    float value = phase_off;
    if (paused || fp_ZERO(speed)) {
        // nothing - leave it at phase_off
    } else if (direction == SPINDLE_CW) {
        value = cw.speed_to_phase(speed_actual);
    } else if (direction == SPINDLE_CCW) {
        value = ccw.speed_to_phase(speed_actual);
    }
    pwm_output->setValue(value);
}

void ESCSpindle::complete_change() {
    // if the spindle is not on (or paused), make sure we stop it
    if (paused || direction == SPINDLE_OFF) {
        speed_actual = 0;
        set_pwm_value();

        if (enable_output != nullptr) {
            enable_output->setValue(false);
        }

        return;
    } else if (direction == SPINDLE_CW) {
        if (enable_output != nullptr) {
            enable_output->setValue(true);
        }
        if (direction_output != nullptr) {
            direction_output->setValue(true);
        }
    } else if (direction == SPINDLE_CCW) {
        if (enable_output != nullptr) {
            enable_output->setValue(true);
        }
        if (direction_output != nullptr) {
            direction_output->setValue(false);
        }
    }

    // setup for the rest to happen during systick
    SysTickTimer.registerEvent(&spindle_systick_event);
}

#endif  // End of include guard: ESC_SPINDLE_H_ONCE
