/*
 * step_dir_driver.cpp - control over a Step/Direction/Enable stepper motor driver
 * This file is part of G2 project
 *
 * Copyright (c) 2016 - 2018 Alden S. Hart, Jr.
 * Copyright (c) 2016 - 2018 Robert Giseburt
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
#ifndef STEP_DIR_DRIVER_H_ONCE
#define STEP_DIR_DRIVER_H_ONCE

#include "MotatePins.h"
#include "MotateTimers.h"

#include "stepper.h"
#include "gpio.h" // for ioPolarity

using Motate::pin_number;
using Motate::OutputPin;
using Motate::PWMOutputPin;
using Motate::kStartHigh;
using Motate::kStartLow;
using Motate::kNormal;
using Motate::Timeout;

// Stepper power management settings
// These should be more flexible, but for now this will do
#define Vcc         3.3                 // volts
#define MaxVref    2.25                 // max vref for driver circuit. Our ckt is 2.25 volts
#define POWER_LEVEL_SCALE_FACTOR ((MaxVref/Vcc)) // scale power level setting for voltage range

// Motor structures
template <pin_number step_num,  // Setup a stepper template to hold our pins
          pin_number dir_num,
          pin_number enable_num,
          pin_number ms0_num,
          pin_number ms1_num,
          pin_number ms2_num,
          pin_number vref_num>
struct StepDirStepper final : Stepper  {
   protected:
    OutputPin<step_num>    _step;
    uint8_t                _step_downcount;
    OutputPin<dir_num>     _dir;
    OutputPin<enable_num>  _enable;
    OutputPin<ms0_num>     _ms0;
    OutputPin<ms1_num>     _ms1;
    OutputPin<ms2_num>     _ms2;
    PWMOutputPin<vref_num> _vref;

    ioPolarity _step_polarity;                   // IO_ACTIVE_LOW or IO_ACTIVE_HIGH
    ioPolarity _enable_polarity;                 // IO_ACTIVE_LOW or IO_ACTIVE_HIGH

    Timeout _motor_activity_timeout;         // this is the timeout object that will let us know when time is up
    uint32_t _motor_activity_timeout_ms;     // the number of ms that the timeout is reset to
    enum stPowerState {                          // used w/start and stop flags to sequence motor power
        MOTOR_OFF = 0,                      // motor is stopped and deenergized
        MOTOR_IDLE,                         // motor is stopped and may be partially energized for torque maintenance
        MOTOR_RUNNING,                      // motor is running (and fully energized)
        MOTOR_POWER_TIMEOUT_START,          // transitional state to start power-down timeout
        MOTOR_POWER_TIMEOUT_COUNTDOWN       // count down the time to de-energizing motor
    } _power_state;              // state machine for managing motor power
    stPowerMode _power_mode;                // See stPowerMode for values

    float _active_power_level; // the power level during motion
    float _idle_power_level; // the power level when idle
    float _power_level; // the power level now

    void _updatePowerLevel() {
        if (MOTOR_IDLE == _power_state) {
            _power_level = _idle_power_level;
        } else {
            _power_level = _active_power_level;
        }

        if (!_vref.isNull()) {
            _vref = _power_level * POWER_LEVEL_SCALE_FACTOR;
        }
    }

   public:
    // sets default pwm freq for all motor vrefs (commented line below also sets HiZ)
    StepDirStepper(ioPolarity step_polarity, ioPolarity enable_polarity, const uint32_t frequency = 250000) :
        Stepper{},
        _step{step_polarity==IO_ACTIVE_LOW?kStartHigh:kStartLow},
        _enable{enable_polarity==IO_ACTIVE_LOW?kStartHigh:kStartLow},
        _vref{kNormal, frequency},
        _step_polarity{step_polarity},
        _enable_polarity{enable_polarity}
    {};

    /* Optional override of init */

    /* Functions that must be implemented in subclasses */

    bool canStep() override { return !_step.isNull(); };

    void setMicrosteps(const uint16_t microsteps) override {
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

    void enableWithTimeout(float timeout_ms) override {
        if (_power_mode == MOTOR_DISABLED || _power_state == MOTOR_RUNNING) {
            return;
        }

        if (timeout_ms < 0.1) {
            timeout_ms = _motor_activity_timeout_ms;
        }

        _power_state = MOTOR_POWER_TIMEOUT_COUNTDOWN;
        if (_power_mode == MOTOR_POWERED_IN_CYCLE || _power_mode == MOTOR_POWER_REDUCED_WHEN_IDLE) {
            _motor_activity_timeout.set(timeout_ms);
        }

        if (!_enable.isNull()) {
            if (_enable_polarity == IO_ACTIVE_HIGH) {
                _enable.set();
            } else {
                _enable.clear();
            }
        }
    };

    void _enableImpl() override {
        if (_power_mode == MOTOR_DISABLED || _power_state == MOTOR_RUNNING) {
            return;
        }

        _power_state = MOTOR_RUNNING;
        _updatePowerLevel();

        if (!_enable.isNull()) {
            if (_enable_polarity == IO_ACTIVE_HIGH) {
                _enable.set();
            } else {
                _enable.clear();
            }
        }
    };

    void _disableImpl() override {
        if (this->getPowerMode() == MOTOR_ALWAYS_POWERED) {
            return;
        }
        if (!_enable.isNull()) {
            if (_enable_polarity == IO_ACTIVE_HIGH) {
                _enable.clear();
            } else {
                _enable.set();
            }
        }
        _motor_activity_timeout.clear();
        _power_state = MOTOR_OFF;
    };

    void stepStart() override {
    	if (_step_polarity == IO_ACTIVE_LOW)
    	    _step.clear();
    	else
    	    _step.set();
    };

    void stepEnd() override {
    	if (_step_polarity == IO_ACTIVE_LOW)
    	    _step.set();
    	else
    	    _step.clear();
    };

    void setDirection(uint8_t new_direction) override {
        if (!_dir.isNull()) {
            if (new_direction == DIRECTION_CW) {
                _dir.clear();
            } else {
                _dir.set();  // set the bit for CCW motion
            }
        }
    };

    virtual void setPowerMode(stPowerMode new_pm)
    {
        _power_mode = new_pm;
        if (_power_mode == MOTOR_ALWAYS_POWERED) {
            enable();
        } else if (_power_mode == MOTOR_DISABLED) {
            disable();
        }
    };

    stPowerMode getPowerMode() override
    {
         return _power_mode;
    };

    float getCurrentPowerLevel() override
    {
        return _power_level;
    };

    void setPowerLevels(float new_active_pl, float new_idle_pl) override
    {
        _active_power_level = new_active_pl;
        _idle_power_level = new_idle_pl;

        _updatePowerLevel();
    };

    ioPolarity getStepPolarity() const override
    {
    	return _step_polarity;
    };

    void setStepPolarity(ioPolarity new_sp) override
    {
    	_step_polarity = new_sp;
    	stepEnd();
    };

    ioPolarity getEnablePolarity() const override
    {
        return _enable_polarity;
    };

    void setEnablePolarity(ioPolarity new_mp) override
    {
        _enable_polarity = new_mp;
        // this is a misnomer, but handles the logic we need for asserting the newly adjusted enable line correctly
        motionStopped();
    };

    // turn off motor is only powered when moving
    // HOT - called from the DDA interrupt
    void motionStopped() //HOT_FUNC
    {
        if (_power_mode == MOTOR_POWERED_IN_CYCLE) {
            this->enable();
        } else if (_power_mode == MOTOR_POWER_REDUCED_WHEN_IDLE) {
            _power_state = MOTOR_POWER_TIMEOUT_START;
        } else if (_power_mode == MOTOR_POWERED_ONLY_WHEN_MOVING) {
            if (_power_state == MOTOR_RUNNING) {
                // flag for periodicCheck - not actually using a timeout
                _power_state = MOTOR_POWER_TIMEOUT_START;
            }
        }
    };

    virtual void setActivityTimeout(float idle_milliseconds) override
    {
        _motor_activity_timeout_ms = idle_milliseconds;
    };


    void periodicCheck(bool have_actually_stopped) override
    {
        if (_power_state == MOTOR_POWER_TIMEOUT_START && _power_mode != MOTOR_ALWAYS_POWERED) {
            if (_power_mode == MOTOR_POWERED_ONLY_WHEN_MOVING) {
                this->disable();
                return;
            }

            // start timeouts initiated during a load so the loader does not need to burn these cycles
            _power_state = MOTOR_POWER_TIMEOUT_COUNTDOWN;
            if (_power_mode == MOTOR_POWERED_IN_CYCLE || _power_mode == MOTOR_POWER_REDUCED_WHEN_IDLE) {
                _motor_activity_timeout.set(_motor_activity_timeout_ms);
            }
        }

        // count down and time out the motor
        if (_power_state == MOTOR_POWER_TIMEOUT_COUNTDOWN) {
            if (_motor_activity_timeout.isPast()) {
                if (_power_mode == MOTOR_POWER_REDUCED_WHEN_IDLE) {
                    _power_state = MOTOR_IDLE;
                    this->_updatePowerLevel();
                } else {
                    this->disable();
                }

                // NOTE: Only global call allowed!
                sr_request_status_report(SR_REQUEST_TIMED);
            }
        }
    };

};

#endif  // STEP_DIR_DRIVER_H_ONCE
