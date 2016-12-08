/*
 * step_dir_hobbyservo.cpp - control over a hobby servo (PWM-driven) using steper Step/Direction/Enable from software
 * This file is part of G2 project
 *
 * Copyright (c) 2016 Alden S. Hart, Jr.
 * Copyright (c) 2016 Robert Giseburt
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
#ifndef STEPP_DIR_HOBBYSERVO_H_ONCE
#define STEPP_DIR_HOBBYSERVO_H_ONCE

#include "MotatePins.h"
#include "MotateTimers.h"

#include "stepper.h"

using Motate::pin_number;
using Motate::OutputPin;
using Motate::PWMOutputPin;
using Motate::kStartHigh;
using Motate::kNormal;
using Motate::Timeout;


// Motor structures
template <pin_number pwm_pin_num>  // Setup a stepper template to hold our pins
struct StepDirHobbyServo final : Stepper {
    /* stepper pin assignments */

    int16_t                _microsteps_per_step = 1;
    bool                   _step_is_forward = false;
    int32_t                _position = 0; // in steps from 0 - 6400 for a full "rotation"
    int32_t                _position_computed = 0; // in steps from 0 - 6400 for a full "rotation"
    float                  _min_value;
    float                  _max_value;
    float                  _value_range;
    bool                   _enabled = false;
    PWMOutputPin<pwm_pin_num> _pwm_pin;
    Motate::Timeout check_timer;

    // sets default pwm freq for all motor vrefs (commented line below also sets HiZ)
    StepDirHobbyServo(const uint32_t frequency = 50) : Stepper{}, _pwm_pin{kNormal, frequency} {
        _pwm_pin.setFrequency(frequency); // redundant due to a bug
        uint16_t _top_value = _pwm_pin.getTopValue();
        float frequency_inv = 1.0/(float)frequency;
        _min_value = (float)_top_value / ((frequency_inv)/(750.0/1000000.0));
        _max_value = (float)_top_value / ((frequency_inv)/(2000.0/1000000.0));
        _value_range = _max_value - _min_value;
        _position_computed = _min_value;
        check_timer.set(1);
    };

    /* Optional override of init */

    /* Functions that must be implemented in subclasses */

    bool canStep() override { return true; };

    void setMicrosteps(const uint8_t microsteps) override {
        switch (microsteps) {
            case (1): {
                _microsteps_per_step = 32;
                break;
            }
            case (2): {
                _microsteps_per_step = 16;
                break;
            }
            case (4): {
                _microsteps_per_step = 8;
                break;
            }
            case (8): {
                _microsteps_per_step = 4;
                break;
            }
            case (16): {
                _microsteps_per_step = 2;
                break;
            }
            case (32): {
                _microsteps_per_step = 1;
                break;
            }
        }
    };

    void _enableImpl() override {
        _enabled = true;
        _pwm_pin.setExactDutyCycle(_position_computed, true);
    };

    void _disableImpl() override {
        _enabled = false;
        _pwm_pin.setExactDutyCycle(0, true);
    };

    void stepStart() override {
        if (!_enabled) return;

        if (_step_is_forward) {
            _position += _microsteps_per_step;
        } else {
            _position -= _microsteps_per_step;
        }

        if (!check_timer.isPast()) { return; }
        check_timer.set(10);

        float used_position = _position;
        if (used_position > 6400.0) {
            used_position = 6400.0;
        }
        if (used_position < 0.0) {
            used_position = 0.0;
        }

        _position_computed = _min_value + ((used_position/6400.0) * _value_range);
        _pwm_pin.setExactDutyCycle(_position_computed, true); // apply the change
    };

    void stepEnd() override {
    };

    void setDirection(uint8_t new_direction) override {
        _step_is_forward = new_direction;
    };

    void setPowerLevel(float new_pl) override {
        ; // ignore this
    };
};

#endif  // STEPP_DIR_HOBBYSERVO_H_ONCE
