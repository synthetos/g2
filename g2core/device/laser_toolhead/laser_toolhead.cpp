/*
 * laser_toolhead.cpp - toolhead driver for a laser, controlled by spindle commands
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

#include <stdint.h>
#include "laser_toolhead.h"

LaserTool::LaserTool(const uint8_t pwm_pin_number, const uint8_t enable_pin_number)
    : pwm_output_num{pwm_pin_number},
      enable_output_num{enable_pin_number} {}

void LaserTool::init()
{
    // TODO - ensure outputs are within range
    set_pwm_output(pwm_output_num);
    set_enable_output(enable_output_num);
}

void LaserTool::pause() {
    if (paused) {
        return;
    }

    paused = true;
    this->complete_change();
}

void LaserTool::resume() {
    if (!paused) {
        return;
    }

    paused = false;
    this->complete_change();
}

bool LaserTool::ready_to_resume() { return paused && safety_manager->ok_to_spindle(); }
// bool LaserTool::busy() {
//     // return true when not paused, on, and ramping up to speed
//     if (paused || (direction == SPINDLE_OFF)) {
//         return false;
//     }
//     return true;
// }

float LaserTool::get_speed() { return speed; }

spDirection LaserTool::get_direction() { return direction; }

void LaserTool::stop() {
    paused = false;
    speed = 0;
    direction = SPINDLE_OFF;

    this->complete_change();
}

// called from a command that was queued when the default set_speed and set_direction returned true
// ALSO called from the loader right before a move
// we are handed the gcode model to use
void LaserTool::engage(const GCodeState_t &gm) {
    if ((direction == gm.spindle_direction) && fp_EQ(speed, gm.spindle_speed)) {
        // nothing changed
        return;
    }

    // // special handling for reversals - we set the speed to zero and ramp up
    // if ((gm.spindle_direction != direction) && (direction != SPINDLE_OFF) && (gm.spindle_direction != SPINDLE_OFF)) {
    //     speed_actual = 0;
    // }

    speed = gm.spindle_speed;
    direction = gm.spindle_direction;

    // handle the rest
    this->complete_change();
}

bool LaserTool::is_on() { return (direction != SPINDLE_OFF); }

// LaserTool-specific functions
void LaserTool::set_pwm_output(const uint8_t pwm_pin_number) {
    if (pwm_pin_number == 0) {
        pwm_output = nullptr;
    } else {
        pwm_output = d_out[pwm_pin_number - 1];
        pwm_output->setEnabled(IO_ENABLED);
        // set the frequency on the output -- not here
        // set the polarity on the output -- not here
    }
}
void LaserTool::set_enable_output(const uint8_t enable_pin_number) {
    if (enable_pin_number == 0) {
        enable_output = nullptr;
    } else {
        enable_output = d_out[enable_pin_number - 1];
        enable_output->setEnabled(IO_ENABLED);
        // set the polarity on the output -- not here
    }
}

void LaserTool::set_frequency(float new_frequency)
{
    if (pwm_output) {
        pwm_output->setFrequency(new_frequency);
    }
}
float LaserTool::get_frequency()
{
    if (pwm_output) {
        return pwm_output->getFrequency();
    }
    return 0.0;
}

// Stepper functons

void LaserTool::_enableImpl() {
    enabled = true;
};

void LaserTool::_disableImpl() {
    enabled = false;
};

void LaserTool::stepStart() {
    if (!enabled) return;

};

void LaserTool::stepEnd() {
};

void LaserTool::setDirection(uint8_t new_direction) {
};

void LaserTool::setPowerLevels(float active_pl, float idle_pl) {
    ; // ignore this
};


// Private functions

void LaserTool::complete_change() {
    // if the spindle is not on (or paused), make sure we stop it
    if (paused || direction == SPINDLE_OFF) {
        if (enable_output != nullptr) {
            enable_output->setValue(false);
        }

        return;
    } else if (direction == SPINDLE_CW) {
        if (enable_output != nullptr) {
            enable_output->setValue(true);
        }
    } else if (direction == SPINDLE_CCW) {
        if (enable_output != nullptr) {
            enable_output->setValue(true);
        }
    }
}
