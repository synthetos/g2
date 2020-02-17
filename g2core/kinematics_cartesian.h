/*
 * kinematics_carteisan.h - inverse kinematics routines for cartesian and CoreXY
 * This file is part of the g2core project
 *
 * Copyright (c) 2020 Alden S. Hart, Jr.
 * Copyright (c) 2020 Robert Giseburt
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

#ifndef KINEMATICS_CARTESIAN_H_ONCE
#define KINEMATICS_CARTESIAN_H_ONCE

#include "g2core.h"
#include "config.h"
#include "canonical_machine.h"
#include "stepper.h"
#include "kinematics.h"
#include "util.h"
#include "settings.h"
#include "gpio.h"
#include "encoder.h" // for encoder grabbing

#include <atomic>

#include "kinematics.h"

template <uint8_t axes, uint8_t motors>
struct CartesianKinematics : KinematicsBase<axes, motors> {
    static const uint8_t joints = axes; // For cartesian we have one joint per axis

    // Joints are defined as these axes in order:
    //  0 = X
    //  1 = Y
    //  2 = Z
    //  3 = A
    //  4 = B
    //  5 = C
    //  6 = U (maybe)
    //  7 = V (maybe)
    //  8 = W (maybe)

    float steps_per_unit[motors];
    float motor_offset[motors];
    bool needs_sync_encoders = true; // if true, we need to update the steps_offset
    int8_t motor_map[motors];  // for each motor, which joint it maps from

    float joint_position[joints];

    void configure(const float new_steps_per_unit[motors], const int8_t new_motor_map[motors]) override
    {
        for (uint8_t motor = 0; motor < motors; motor++) {
            motor_map[motor] = new_motor_map[motor];
            int8_t joint = motor_map[motor];
            if (joint == -1) {
                motor_offset[motor] = 0;
                steps_per_unit[motor] = 1;
            } else {
                float steps = (joint_position[joint] * steps_per_unit[motor]) + motor_offset[motor];
                steps_per_unit[motor] = new_steps_per_unit[motor];
                motor_offset[motor] = steps - (joint_position[joint] * steps_per_unit[motor]);
            }
        }
    }

    void inverse_kinematics(const GCodeState_t &gm, const float target[axes], const float position[axes], const float start_velocity,
                            const float end_velocity, const float segment_time, float steps[motors]) override
    {
        // joint == axis in cartesian kinematics
        for (uint8_t motor = 0; motor < motors; motor++) {
            int8_t joint = motor_map[motor];
            if (joint == -1) {
                continue;
            }

            steps[motor] = (target[joint] * steps_per_unit[motor]) + motor_offset[motor];
        }

        for (uint8_t joint = 0; joint < joints; joint++) {
            joint_position[joint] = target[joint];
        }
    }

    void get_position(float position[axes]) override
    {
        for (uint8_t axis = 0; axis < axes; axis++) {
            position[axis] = joint_position[axis];
        }
    }

    float best_steps_per_unit[axes];

    void forward_kinematics(const float steps[joints], float position[axes]) override
    {
        // Setup
        for (uint8_t axis = 0; axis < axes; axis++) {
            position[axis] = 0.0;
        }
        for (uint8_t motor = 0; motor < motors; motor++) {
            int8_t joint = motor_map[motor];
            if (joint == -1) {
                continue;
            }

            auto axis = joint; // it's cartesian, baby!
            best_steps_per_unit[axis] = -1.0;

            // If this motor has a better (or the only) resolution, then we use this motor's value
            if (best_steps_per_unit[axis] < steps_per_unit[motor]) {
                best_steps_per_unit[axis] = steps_per_unit[motor];
                position[axis]            = (steps[motor]-motor_offset[motor]) / steps_per_unit[motor];
            }

            joint_position[joint] = position[joint];
        }
    }

    void sync_encoders(const float step_position[motors], const float position[axes]) override {
        // We need to make joint_offset[joint] adjust any given position so that if it's given as a target
        // to inverse_kinematics then step_position[motor] will be given as the return steps[motor]

        // Why? Externally position[] may be unrelated to step_position[], so we need to adjust.
        for (uint8_t motor = 0; motor < motors; motor++) {
            int8_t joint = motor_map[motor];
            if (joint == -1) {
                continue;
            }

            // This, solved for motor_offset: step_position[motor] = (position[joint]] * steps_per_unit[motor]) + motor_offset[motor];
            motor_offset[motor] = step_position[motor] - (position[joint] * steps_per_unit[motor]);
        }
    }
};


// Support for CoreXY Kinematics - http://corexy.com/
template <uint8_t axes, uint8_t motors>
struct CoreXYKinematics final : CartesianKinematics<axes, motors> {
    typedef CartesianKinematics<axes, motors> parent;
    using parent::joints;

    // Joints are mapped to:
    //  0 = CoreXY A
    //  1 = CoreXY B
    //  2 = Z
    //  3 = A
    //  4 = B
    //  5 = C
    //  6 = U (maybe)
    //  7 = V (maybe)
    //  8 = W (maybe)

    void inverse_kinematics(const GCodeState_t &gm, const float target[axes], const float position[axes], const float start_velocity,
                            const float end_velocity, const float segment_time, float steps[motors]) override
    {
        // need to have a place to store the adjusted COREXY A and B
        float axes_target[axes];

        // The COREXY A and B are the X and Y axes mixed as follows
        axes_target[0]=target[0]+target[1];
        axes_target[1]=target[0]-target[1];

        // the rest are just copied
        for (uint8_t axis = 2; axis < AXES; axis++) {
            axes_target[axis]=target[axis];
        }

        // just use the cartesian method from here on
        parent::inverse_kinematics(gm, axes_target, position, start_velocity, end_velocity, segment_time, steps);
    }

    void forward_kinematics(const float steps[motors], float position[axes]) override
    {
        // start by letting the cartesian kinematics work
        parent::forward_kinematics(steps, position);

        // Then adjust X and Y from CoreXY-A and CoreXY-B
        /*
         * deltaX=1/2(deltaA+deltaB)
         * deltaY=1/2(deltaA-deltaB)
         *
         * At this moment:
         *   position[0] = deltaA
         *   position[1] = deltaB
         *
         * We want:
         *   position[0] = deltaX
         *   position[1] = deltaY
         */

        float deltaA = position[0];
        float deltaB = position[1];

        position[0] = 0.5 * (deltaA + deltaB);
        position[1] = 0.5 * (deltaA - deltaB);
    }
};

#endif  // End of include Guard: KINEMATICS_CARTESIAN_H_ONCE
