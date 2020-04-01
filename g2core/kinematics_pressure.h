/*
 * kinematics.h - inverse kinematics routines
 * This file is part of the g2core project
 *
 * Copyright (c) 2013 - 2018 Alden S. Hart, Jr.
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

#ifndef KINEMATICS_FOUR_CABLE_H_ONCE
#define KINEMATICS_FOUR_CABLE_H_ONCE

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
struct PressureKinematics : KinematicsBase<axes, motors> {
    static const uint8_t joints = axes; // For cartesian we have one joint per axis

    double joint_vel[4];
    double joint_accel[4];
    double joint_jerk[4];

    static constexpr uint8_t pressure_sensor_count = 1;
    double raw_sensor_value[pressure_sensor_count];  // stored from last time they were read
    double sensor_value[pressure_sensor_count];  // stored from last time they were read


    double sensor_zero_target = 0.0;
    double sensor_proportional_factor = 20;
    double sensor_inetgral_store = 0;
    double sensor_inetgral_factor = 0.0001;
    double sensor_error_store = 0;
    double sensor_derivative_factor = 20;

    double reverse_target_pressure = -10;

    const float sensor_skip_detection_jump = 10000;

    double friction_loss_normal     = 45;       // percentage of loss due to friction per segment, parked
    double friction_midpoint_normal = 15.0; // velocity (mm/min) at the midpoint for friction per segment, parked

    double friction_loss_slowed     = 300;   // percentage of loss due to friction per segment, NOT parked
    double friction_midpoint_slowed = 20.0;   // velocity (mm/min) at the midpoint for friction per segment, NOT parked

    double pressure_target = 0;
    double seconds_between_events = 6.0;
    double seconds_to_hold_event = 2;

    bool is_anchored = false;

    double prev_joint_position[4];
    double prev_joint_vel[4];
    double prev_joint_accel[4];
    double joint_limit[motors];

    float start_velocities[motors];
    float end_velocities[motors];
    double target_accel[4] = {0.0, 0.0, 0.0, 0.0};
    double sensor_diff[4] = {0.0, 0.0, 0.0, 0.0};
    bool last_switch_state[4];

    // use a timer to let the sensors be initied and their readings settle
    Motate::Timeout sensor_settle_timer;
    Motate::Timeout at_pressure_timer;
    Motate::Timeout inter_event_timer;

    #define ANCHOR_A_INPUT 1
    #define ANCHOR_B_INPUT 2
    #define ANCHOR_C_INPUT 3
    #define ANCHOR_D_INPUT 4

    // setup the inputs array - compile-time for now
    gpioDigitalInputReader * const anchor_inputs[4] = {
        in_r[ANCHOR_A_INPUT-1],
        in_r[ANCHOR_B_INPUT-1],
        in_r[ANCHOR_C_INPUT-1],
        in_r[ANCHOR_D_INPUT-1],
    };

    PressureSensor *const pressure_sensors[pressure_sensor_count] = {&pressure_sensor};

    // 1 - let the sensors settle
    // 2 - back the motors off 10mm (SKIP for now)
    // 3 - read the sensors - record that as baseline
    // 4 - start normal idle activity

    PressureKinematics() {
        sensor_settle_timer.set(5000);
        at_pressure_timer.clear();
    }

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

    bool read_sensors() {
        // do nothing until the settle timer is past
        if (!sensor_settle_timer.isPast()) {
            // let the sensors settle
            // for (uint8_t joint = 0; joint < 4; joint++) {
            //     sensor_zero_value[joint] = (sensor_zero_value[joint] * 0.2) + ((sensor_inputs[joint]->getValue()/* - sensor_zero_target*/) * 0.8);
            //
            //     // it's VITAL that these be zero until we have valid readings
            //     sensor_value[joint] = 0.0;
            //     sensor_diff[joint] = 0.0;
            // }
            return false; // let the caller know
        }

        for (uint8_t joint = 0; joint < pressure_sensor_count; joint++)
        {
            // note we invert and zero the values
            raw_sensor_value[joint] = pressure_sensors[joint]->getPressure(PressureUnits::cmH2O);

            // new_sensor_value at -1 is max under presure, 1 is max over pressure, and 0 is goldilocks
            double e = (raw_sensor_value[joint] - sensor_zero_target);
            sensor_inetgral_store += e;
            if (sensor_inetgral_store > 50) {
                sensor_inetgral_store = 50;
            } else if (sensor_inetgral_store < 50) {
                sensor_inetgral_store = -50;
            }

            double p_v = e * sensor_proportional_factor;
            double i_v = sensor_inetgral_store * sensor_inetgral_factor;
            double d_v = (e - sensor_error_store) * sensor_derivative_factor;
            sensor_error_store = e;

            double new_sensor_value = p_v + i_v - d_v;

            // new_sensor_diff is literally the change of the sensor value since we last read it
            sensor_diff[joint]  = (new_sensor_value - sensor_value[joint]);

            sensor_value[joint] = new_sensor_value;
        }
        return true;
    }

    bool anchored() { return is_anchored; }
    void anchored(bool v) {
        is_anchored = v;

        // if we are setting it to false, do NOT reset the cables
        if (!is_anchored) {
            return;
        }

        // nothing to do yet
    }

    // if we requested a move, return true, otherwise false
    bool last_segment_was_idle = false;
    bool over_pressure = false;
    bool idle_task() override {
        /* Notes about this situation:
        * 1. This is called from Exec, which is called from Load, which is called (ignoring bootstrapping) from the stepper
        *    when a segment is over.
        * 2. The currently running segment in the stepper subsystem (which may be a movement-free segment) has a target of
        *    the current joint_position[] (as it's known in this part of the code) and the start position of
        *    prev_joint_position[].
        * 3. The encoder was read during the last segment encoder_readings_taken[] times (may be zero).
        * 4. If encoder_readings_taken[] is non-zero, then the last reading was taken at some point during the last segment,
        *    and should be somewhere between prev_joint_position[] and joint_position[].
        */

        if (!read_sensors() || is_anchored) {
            return false; // too soon - sensors are still settling
        }

        if (!last_segment_was_idle) {
            for (uint8_t joint = 0; joint < pressure_sensor_count; joint++) {
                joint_vel[joint] = 0.0;
                joint_accel[joint] = 0.0;
                joint_jerk[joint] = 0.0;
                sensor_value[joint] = 0.0;
                sensor_diff[joint] = 0.0;
            }
        }
        last_segment_was_idle = true;

        /*
        pressure_target
            float seconds_between_events = 6.0;
            float seconds_to_hold_event = 2;
            inter_event_timer
        */

       if (!inter_event_timer.isSet() || inter_event_timer.isPast()) {
           sensor_zero_target = pressure_target;
           inter_event_timer.set(seconds_between_events * 1000.0);
       }

        // check inputs to be sure we aren't anchored
        // if we are anchored, then set the zero-position offsets
        // the first segment that isn't anchored will use them

        const float segment_time = MIN_SEGMENT_TIME; // time in MINUTES

        for (uint8_t joint = 0; joint < pressure_sensor_count; joint++) {
            bool switch_state = anchor_inputs[joint]->getState();
            if (switch_state && (sensor_zero_target > 7)) {
                // ignore the switch state and head toward making pressure!
                switch_state = false;
            }
            bool overpressure_detected = (raw_sensor_value[joint] > (sensor_zero_target*0.75)) && (sensor_zero_target > 7);
            if (over_pressure && at_pressure_timer.isPast()) {
                sensor_zero_target = reverse_target_pressure;
                over_pressure = false;
                at_pressure_timer.clear();
            } else if (overpressure_detected && !at_pressure_timer.isSet()) {
                over_pressure = true;
                at_pressure_timer.set(seconds_to_hold_event*1000.0);
            }

            // bool switch_state = false; // ignore switches
            if (switch_state && !last_switch_state[joint]) {
                if (sensor_zero_target < 0) {
                    // stop the motion
                    sensor_zero_target = 0.1;
                }

                // cable_stepper_offset[joint] = joint_position[joint] - cable_zero_offsets[joint];
            } else if (!switch_state) {
                if (last_switch_state[joint]) {
                    joint_limit[joint] = joint_position[joint] - 130;
                } else {
                    if ((sensor_zero_target > 0) && (joint_position[joint] < joint_limit[joint])) {
                        sensor_zero_target = reverse_target_pressure;
                    }
                }
            }

            // prev_joint_accel[joint] = joint_accel[joint];
            start_velocities[joint] = std::abs(joint_vel[joint]);
            // prev_joint_vel[joint] = joint_vel[joint];

            // heiristic to detect skips
            // if the pressure rises the 80% of the full range in a segment

            // if (sensor_diff[joint] > sensor_skip_detection_jump) {
            //     joint_jerk[joint] = 0.0; // clear it out
            //     joint_accel[joint] = 0.0; // same
            //     joint_vel[joint] = 50.0; // it's already stopped, back it off some
            // }

            // float diff_or_guide = sensor_zero_target > 0 ? sensor_diff[joint] : 0.1;
            double diff_or_guide = sensor_diff[joint];

            // static friction
            // auto friction = (((switch_state || over_pressure) ? friction_loss_slowed : friction_loss_normal)/100.0);
            // auto friction_midpoint = ((switch_state || over_pressure) ? friction_midpoint_slowed : friction_midpoint_normal);
            auto friction = ((switch_state ? friction_loss_slowed : friction_loss_normal)/100.0);
            auto friction_midpoint = (switch_state ? friction_midpoint_slowed : friction_midpoint_normal);

            double jmax = cm->a[AXIS_X].jerk_max * JERK_MULTIPLIER;
            double jhigh = cm->a[AXIS_X].jerk_high * JERK_MULTIPLIER;

            joint_jerk[joint] = diff_or_guide * jmax;
            joint_accel[joint] = joint_accel[joint] + joint_jerk[joint]*segment_time;

            double old_joint_vel = joint_vel[joint];

            auto friction_loss_vel = (friction * friction_midpoint) / (std::abs(joint_vel[joint]) + friction_midpoint);
            joint_vel[joint] = joint_vel[joint] - joint_vel[joint] * friction_loss_vel;
            joint_vel[joint] = joint_vel[joint] + joint_accel[joint]*segment_time;

            double vmax_diff = jhigh * segment_time * segment_time;
            if (std::abs(old_joint_vel - joint_vel[joint]) > vmax_diff) {
                if (old_joint_vel < joint_vel[joint]) {
                    joint_vel[joint] = old_joint_vel + vmax_diff;
                } else {
                    joint_vel[joint] = old_joint_vel - vmax_diff;
                }
            }

            // limit velocity
            const double vmax = cm->a[AXIS_X].velocity_max;
            if (joint_vel[joint] < -vmax) {
                joint_vel[joint] = -vmax;
                // error_offset = 0; // none of it was applied
            } else if (joint_vel[joint] > vmax) {
                joint_vel[joint] = vmax;
                // error_offset = 0; // none of it was applied
            }

            end_velocities[joint] = std::abs(joint_vel[joint]);
            prev_joint_position[joint] = joint_position[joint];
            joint_position[joint] = joint_position[joint] + (joint_vel[joint] * segment_time);

            last_switch_state[joint] = switch_state;
        } // for joint

        // convert them to steps
        float target_steps[motors];
        for (uint8_t motor = 0; motor < motors; motor++) {
            int8_t joint = motor_map[motor];
            if (joint == -1) {
                continue;
            }

            target_steps[motor] = (joint_position[joint] * steps_per_unit[motor]) + motor_offset[motor];
        }

        // tell the planner and runtime about them
        mp_set_target_steps(target_steps, start_velocities, end_velocities, segment_time);

        return true;
    }
};


#endif  // End of include Guard: KINEMATICS_FOUR_CABLE_H_ONCE
