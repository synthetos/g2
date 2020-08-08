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
#include <cmath>
#include <atomic>

#include <atomic>

#include "kinematics.h"

template <uint8_t axes, uint8_t motors>
struct CartesianKinematics : KinematicsBase<axes, motors> {
    static const uint8_t joints = axes; // For cartesian we have one joint per axis
    const float PI = 3.1415927; // yeah, we have to do this :-/

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
    float joint_position_encoder_max[joints];
    float joint_position_encoder_min[joints];
    float motor_joint_position_offset[motors];
    float joint_position_delayed[joints];  // joint_position, but delayed by 20 segments to accomodate delay in the servo
    float joint_target[joints];  // that last requested target, for idle to keep track of where to aim for

    double joint_vel[joints];
    double joint_accel[joints];
    double joint_jerk[joints];

    double encoder_position[motors];  // Position of the external encoder
    double encoder_offset[motors];  // amount of error tracked by the external encoders vs the internal encoders
    double encoder_error[motors];   // amount of error detected in this last pass (ephemeral)
    uint8_t encoder_reads[motors];  // keep track of how many times the encoder is read before uses
    bool encoder_needs_read[motors];               // as it says - used to know when to request another sensor read
    bool encoder_synced[motors];  // used to know when the encoer offset is valid (false means no, and they need synced)

    gpioDigitalInputPinVirtual *virtual_pin;

    bool inited = false;

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

            #if HAS_HOBBY_SERVO_MOTOR
            if (!inited && motor == 5) {
                auto encoder = external_linear_encoders[motor];
                encoder->setCallback([motor, this](bool worked, float new_position) {
                    // NOTE: `this` is captured by reference
                    // For clarity, below this-> is used explicitly when it could be left implcit

                    this->encoder_needs_read[motor] = true;
                    this->encoder_reads[motor]++;
                    // Motate::debug.send(encoder_reads[motor], 0);

                    if (!worked) {
                        return;  // bail early
                    }

                    this->encoder_position[motor] = -new_position; // invert, for now non-configurable FIXME

                    // external_linear_encoders[motor]->requestPositionMMs();
                });

                this->encoder_needs_read[motor] = false;
                external_linear_encoders[motor]->requestPositionMMs();
            }  // if (!initied)
            #endif // HAS_HOBBY_SERVO_MOTOR
        }

        inited = true;
    }

    bool homing_pin = false;

    void inverse_kinematics(const GCodeState_t &gm, const float target[axes], const float position[axes],
                            float start_velocities[motors], float end_velocities[motors], const float segment_time,
                            float steps[motors]) override
    {
        // joint == axis in cartesian kinematics
        for (uint8_t motor = 0; motor < motors; motor++) {
            int8_t joint = motor_map[motor];
            if (joint == -1) {
                continue;
            }

#if HAS_HOBBY_SERVO_MOTOR
            if (motor != 5) {
                steps[motor] = (target[joint] * steps_per_unit[motor]) + motor_offset[motor];
                joint_position[joint] = target[joint] + motor_joint_position_offset[motor];
                joint_target[joint] = target[joint];
            } else {
                // special hobby servo handling - and this is mostly specific to the axidraw at the moment
                // convert the requested position to a velue that represents the angle that will yield the correct position
                // Assumptions:
                // min position is zero, max position is motor.travel_rev
                // min angle is -pi/2, max angle is pi/2
                // angle 0 = motor.travel_rev/2

                float e_pos = encoder_position[motor] + encoder_offset[motor];
                if (!encoder_synced[motor]) { //  || e_pos < 0 || joint_position[joint] >= 22.07
                    encoder_offset[motor] = joint_position[joint] - encoder_position[motor];
                    encoder_synced[motor] = true;

                    e_pos = encoder_position[motor];
                }

                encoder_error[motor] = 0;
                joint_position_delayed[joint] =
                    (joint_position[joint] * 1.0 / 20.0) + (joint_position_delayed[joint] * 19.0 / 20.0);

                encoder_error[motor] = e_pos - joint_position_delayed[joint];
                if (encoder_error[motor] > 2.0) {
                    homing_pin = true;
                } else if (encoder_error[motor] < 0.01) {
                    homing_pin = false;
                }

                // encoder_error[motor] = e_pos - joint_position[joint];

                // if (e_pos < joint_position_encoder_min[joint]) {
                //     encoder_error[motor] = e_pos - joint_position_encoder_min[joint];
                // } else if (e_pos > joint_position_encoder_max[joint]) {
                //     encoder_error[motor] = e_pos - joint_position_encoder_max[joint];
                // }
                // Motate::debug.send(e_pos, 1);
                // Motate::debug.send(joint_position_delayed[joint], 2);

                double new_target = 0;
                double prev_joint_position = joint_position[joint];
                double old_joint_vel = joint_vel[joint];
                double old_joint_accel = joint_accel[joint];

                joint_target[joint] = target[joint];

                // float expected_position = prev_joint_position + encoder_error[motor];
                // motor_joint_position_offset[motor] -= encoder_error[motor] / 2;

                // new_target = target[joint] + motor_joint_position_offset[motor];
                new_target = target[joint] + motor_joint_position_offset[motor] + encoder_error[motor] * 1/20;

                // jerk-limit the new target move
#if 1
                double jmax = cm->a[joint].jerk_high * JERK_MULTIPLIER;
                const double vmax = cm->a[joint].velocity_max;

                // find out what velocity we'll need to end up with to get to the requested position
                double requested_velocity = 2 / segment_time * (new_target - prev_joint_position) - old_joint_vel;
                joint_vel[joint] = requested_velocity;

                // // limit velocity to stop at zero for polarity changes, and then limit to +- max
                // // we allow near-zero velocity changes to prevent deadlock - we'll clean up polarity change later
                // if ((old_joint_vel < -5) && (requested_velocity > 5)) {
                //     requested_velocity = 0;
                // }
                // else if ((old_joint_vel > 5) && (requested_velocity < -5)) {
                //     requested_velocity = 0;
                // }

                // limit the maximum accleration so that we'll stop at the target velocity and never violate jerk
                // note: thanks to the square root, we need to celan up the sign
                double max_accel = sqrt(std::abs(requested_velocity - old_joint_vel) * jmax * 2.0);
                double sign = 1.0;
                // choose a jerk value that will not violate the max_acceleration withing two time segments
                if ((requested_velocity - old_joint_vel) < 0.0) {
                    // want to accelerate in the negative direction
                    sign = -1.0;
                }

                // compute the new joint_vel to stay below max_accel with a 4x margin of error
                if ((std::abs(old_joint_accel) + jmax * segment_time * 4.0) < max_accel) {
                    joint_accel[joint] = (std::abs(old_joint_accel) + jmax * segment_time) * sign;
                    joint_jerk[joint] = jmax * sign;
                } else {
                    joint_accel[joint] = (std::abs(old_joint_accel) - jmax * segment_time) * sign;
                    joint_jerk[joint] = -jmax * sign;
                }
                joint_vel[joint] = joint_vel[joint] + joint_accel[joint]*segment_time + jmax*segment_time*segment_time*0.5;

                // limit velocity
                if (joint_vel[joint] < -vmax) {
                    joint_vel[joint] = -vmax;
                } else if (joint_vel[joint] > vmax) {
                    joint_vel[joint] = vmax;
                }

                // // now that everything is done adjusting joint_vel[joint], we can recompute joint_accel[joint] (for next time)
                joint_accel[joint] = (joint_vel[joint] - old_joint_vel) / segment_time - jmax * segment_time * 0.5;

                // setup start and end velocities
                start_velocities[motor] = std::abs(old_joint_vel);
                end_velocities[motor] = std::abs(joint_vel[joint]);

                // sanity check, we can't do a reversal in the middle of a segment,
                // so the start velocity and the end velocity have to have the same sign
                // note: start_velocities[joimotornt] and end_velocities[motor] are both ABS, so this sign change is lost there!
                if (((old_joint_vel > 0) && (joint_vel[joint] < 0)) || ((old_joint_vel < 0) && (joint_vel[joint] > 0))) {
                    // solution: since we are reversing, we are going to start from zero
                    start_velocities[motor] = std::abs(old_joint_vel + joint_vel[joint])*0.5;
                    end_velocities[motor] = start_velocities[motor];
                }

                // // and finally what our actual target position will be
                joint_position[joint] = joint_position[joint] + ((old_joint_vel + joint_vel[joint]) * 0.5 * segment_time);
#else
                joint_vel[joint] = 2 / segment_time * (new_target - prev_joint_position) - old_joint_vel;

                start_velocities[motor] = std::abs(old_joint_vel);
                end_velocities[motor] = std::abs(joint_vel[joint]);

                joint_position[joint] = new_target;
#endif
                // Motate::debug.send(joint_position[joint], 2);

                // Lookup table time!!

                const float lookup_table[][2] =
                {  //in      out
                    {     0.0   ,  0.0     }, //  0
                    {     0.0454,  0.679   }, //  1
                    {     0.0767,  0.91862 }, //  2
                    {     0.25  ,  1.41027 }, //  3
                    {     0.65  ,  2.12191 }, //  4
                    {     1.4   ,  3.37188 }, //  5
                    {     2.5596,  4.35519 }, //  6
                    {     4.3   ,  5.70515 }, //  7
                    {     6.5   ,  7.04844 }, //  8
                    {    13.3823, 11.3     }, //  9
                    {    16.1919, 13.1     }, // 10
                    {    18.5   , 14.8     }, // 11
                    {    20.0   , 16.0     }, // 12
                    {    21.0   , 17.6     }, // 13
                    {    21.5376, 18.46979 }, // 14
                    {    21.9687, 20.35    }, // 15
                    {    22.0513, 21.73804 }, // 16
                    {    22.07  , 22.07    }, // 17
                };

                float target_pos = 0;

                // apply the lookup table
                for (int i = 0; i < 17; i++) {
                    if (lookup_table[i][0] > joint_position[joint]) {
                        target_pos = 0;
                        break;
                    }
                    if (lookup_table[i][0] <= joint_position[joint] &&
                        lookup_table[i + 1][0] >= joint_position[joint]) {
                        // adjust target_pos_fractional
                        target_pos = ((joint_position[joint] - lookup_table[i][0]) /
                                        (lookup_table[i + 1][0] - lookup_table[i][0])) *
                                            (lookup_table[i + 1][1] - lookup_table[i][1]) +
                                        lookup_table[i][1];
                        break;
                    }
                    if (i == 16) {
                        target_pos = lookup_table[i + 1][1];
                        break;
                    }
                }

                // Motate::debug.send(target_pos, 2);

                steps[motor] = (target_pos * steps_per_unit[motor]); // ignore motor offset here
                // setup encoder reading and stash values for next segment
                if (this->encoder_needs_read[motor]) {
                    this->encoder_needs_read[motor] = true;
                    external_linear_encoders[motor]->requestPositionMMs();
                }

                float segment_min = std::min((float)prev_joint_position, joint_position[joint]);
                float segment_max = std::max((float)prev_joint_position, joint_position[joint]);

                joint_position_encoder_min[joint] = segment_min;
                joint_position_encoder_max[joint] = segment_max;

                // trigger if heading up and we go over the top
                if (joint_position[joint] > 22.07 && prev_joint_position < joint_position[joint]) {
                    homing_pin = true;

                // release trigger if we're already triggered, going down below 22, and started above 22
                } else if (homing_pin == true && joint_position[joint] <= 22.07  && prev_joint_position > 22.07) {
                    homing_pin = false;
                }

                encoder_reads[motor] = 0;
            }
#else // if !HAS_HOBBY_SERVO_MOTOR
            steps[motor] = (target[joint] * steps_per_unit[motor]) + motor_offset[motor];
#endif

            if (virtual_pin) {
                virtual_pin->pin_changed(homing_pin ? INPUT_ACTIVE : INPUT_INACTIVE);
            }
        }

#if !HAS_HOBBY_SERVO_MOTOR
        for (uint8_t joint = 0; joint < joints; joint++) {
            joint_position[joint] = target[joint];
        }
        #endif
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

            auto axis = joint;  // it's cartesian, baby!
            best_steps_per_unit[axis] = -1.0;

#if HAS_HOBBY_SERVO_MOTOR

            if (motor != 5) {
                // If this motor has a better (or the only) resolution, then we use this motor's value
                if (best_steps_per_unit[axis] < steps_per_unit[motor]) {
                    best_steps_per_unit[axis] = steps_per_unit[motor];
                    position[axis]            = (steps[motor]-motor_offset[motor]) / steps_per_unit[motor];
                }

                joint_position[joint] = position[joint];
            } else {
                position[axis] = encoder_position[motor] + encoder_offset[motor];
                // joint_position[joint] = position[joint];
            }

#else // !HAS_HOBBY_SERVO_MOTOR

            // If this motor has a better (or the only) resolution, then we use this motor's value
            if (best_steps_per_unit[axis] < steps_per_unit[motor]) {
                best_steps_per_unit[axis] = steps_per_unit[motor];
                position[axis]            = (steps[motor]-motor_offset[motor]) / steps_per_unit[motor];
            }

            joint_position[joint] = position[joint];

#endif
        }
    }

    void sync_encoders(const float step_position[motors], const float position[axes]) override {
        // We need to make motor_offset[joint] adjust any given position so that if it's given as a target
        // to inverse_kinematics then step_position[motor] will be given as the return steps[motor]

        // Why? Externally position[] may be unrelated to step_position[], so we need to adjust.
        for (uint8_t motor = 0; motor < motors; motor++) {
            int8_t joint = motor_map[motor];
            if (joint == -1) {
                continue;
            }

            // This, solved for motor_offset: step_position[motor] = (position[joint]] * steps_per_unit[motor]) + motor_offset[motor];
            motor_offset[motor] = step_position[motor] - (position[joint] * steps_per_unit[motor]);

            // Now adjust the joint position offset so that when given this position as a target, the current joint_position is returned
            motor_joint_position_offset[motor] = joint_position[joint] - position[joint];
            joint_target[joint] -= motor_joint_position_offset[motor];
            // encoder_synced[motor] = false;
        }
    }

    // bool idle_task() override {
    //     float start_velocities[motors];
    //     float end_velocities[motors];
    //     float steps[motors];

    //     for (int motor = 0; motor < motors; motor++) {
    //         start_velocities[motor] = 0;
    //         end_velocities[motor] = 0;
    //         steps[motor] = 0;
    //     }

    //     inverse_kinematics(cm->gm, joint_target, joint_target, start_velocities, end_velocities, MIN_SEGMENT_TIME,
    //                        steps);

    //     // tell the planner and runtime about them
    //     mp_set_target_steps(steps, start_velocities, end_velocities, MIN_SEGMENT_TIME);

    //     return true;
    // }

    void set_virtual_input(gpioDigitalInputPinVirtual *const pin) override { virtual_pin = pin; }
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

    void inverse_kinematics(const GCodeState_t &gm, const float target[axes], const float position[axes],
                            float start_velocities[motors], float end_velocities[motors], const float segment_time,
                            float steps[motors]) override
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
        parent::inverse_kinematics(gm, axes_target, position, start_velocities, end_velocities, segment_time, steps);
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

    void sync_encoders(const float step_position[motors], const float position[axes]) override {
        // need to have a place to store the adjusted COREXY A and B
        float axis_position[axes];

        // The COREXY A and B are the X and Y axes mixed as follows
        axis_position[0]=position[0]+position[1];
        axis_position[1]=position[0]-position[1];

        // the rest are just copied
        for (uint8_t axis = 2; axis < AXES; axis++) {
            axis_position[axis]=position[axis];
        }

        // just use the cartesian method from here on
        parent::sync_encoders(step_position, axis_position);
    }

    float idle_start_velocities[motors];
    float idle_end_velocities[motors];
    float idle_steps[motors];

    bool idle_task() override {
        for (int motor = 0; motor < motors; motor++) {
            idle_start_velocities[motor] = 0;
            idle_end_velocities[motor] = 0;
            idle_steps[motor] = 0;
        }

        parent::inverse_kinematics(cm->gm, parent::joint_target, parent::joint_position, idle_start_velocities, idle_end_velocities, MIN_SEGMENT_TIME,
                           idle_steps);

        // tell the planner and runtime about them
        mp_set_target_steps(idle_steps, idle_start_velocities, idle_end_velocities, MIN_SEGMENT_TIME);

        return true;
    }

};

#endif  // End of include Guard: KINEMATICS_CARTESIAN_H_ONCE
