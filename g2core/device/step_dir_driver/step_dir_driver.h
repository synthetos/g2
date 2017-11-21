/*
 * step_dir_driver.cpp - control over a Step/Direction/Enable stepper motor driver
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
#ifndef STEPP_DIR_DRIVER_H_ONCE
#define STEPP_DIR_DRIVER_H_ONCE

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
template <pin_number step_num,  // Setup a stepper template to hold our pins
          pin_number dir_num,
          pin_number enable_num,
          pin_number ms0_num,
          pin_number ms1_num,
          pin_number ms2_num,
          pin_number vref_num>
struct StepDirStepper final : Stepper  {
    /* stepper pin assignments */

    OutputPin<step_num>    _step;
    uint8_t                _step_downcount;
    OutputPin<dir_num>     _dir;
    OutputPin<enable_num>  _enable{kStartHigh};
    OutputPin<ms0_num>     _ms0;
    OutputPin<ms1_num>     _ms1;
    OutputPin<ms2_num>     _ms2;
    PWMOutputPin<vref_num> _vref;

    // sets default pwm freq for all motor vrefs (commented line below also sets HiZ)
    StepDirStepper(const uint32_t frequency = 250000) : Stepper{}, _vref{kNormal, frequency} {};

    /* Optional override of init */

    /* Functions that must be implemented in subclasses */

    bool canStep() override { return !_step.isNull(); };

    void setMicrosteps(const uint8_t microsteps) override {
        if (!_enable.isNull()) {
            switch (microsteps) {
                case (1): {
                    _ms2 = 0;
                    _ms1 = 0;
                    _ms0 = 0;
                    break;
                }
                case (2): {
                    _ms2 = 0;
                    _ms1 = 0;
                    _ms0 = 1;
                    break;
                }
                case (4): {
                    _ms2 = 0;
                    _ms1 = 1;
                    _ms0 = 0;
                    break;
                }
                case (8): {
                    _ms2 = 0;
                    _ms1 = 1;
                    _ms0 = 1;
                    break;
                }
                case (16): {
                    _ms2 = 1;
                    _ms1 = 0;
                    _ms0 = 0;
                    break;
                }
                case (32): {
                    _ms2 = 1;
                    _ms1 = 0;
                    _ms0 = 1;
                    break;
                }
            }
        }
    };

    void _enableImpl() override {
        if (!_enable.isNull()) {
            _enable.clear();
        }
    };

    void _disableImpl() override {
        if (!_enable.isNull()) {
            _enable.set();
        }
    };

    void stepStart() override { _step.set(); };

    void stepEnd() override { _step.clear(); };

    void setDirection(uint8_t new_direction) override {
        if (!_dir.isNull()) {
            if (new_direction == DIRECTION_CW) {
                _dir.clear();
            } else {
                _dir.set();  // set the bit for CCW motion
            }
        }
    };

    void setPowerLevel(float new_pl) override {
        if (!_vref.isNull()) {
            _vref = new_pl;
        }
    };
};

#endif  // STEPP_DIR_DRIVER_H_ONCE
