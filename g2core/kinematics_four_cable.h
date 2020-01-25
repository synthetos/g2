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

template<typename t>
struct Point3_ {
    t x;
    t y;
    t z;

    constexpr t& operator[] (const std::size_t index) {
        // assert(index >= 0 && index < 3);
        switch (index) {
            case 0: return x;
            case 1: return y;
            default: return z;
        }
    }
    constexpr const t& operator[] (const std::size_t index) const {
        // assert(index >= 0 && index < 3);
        switch (index) {
            case 0: return x;
            case 1: return y;
            default: return z;
        }
    }

    Point3_<t> operator+ (const Point3_<t> &p) const {
        return {
            x+p.x,
            y+p.y,
            z+p.z
        };
    }

    t distance_to(const Point3_<t> &p) const {
        t temp_x = (x-p.x);
        t temp_y = (y-p.y);
        t temp_z = (z-p.z);
        return std::sqrt(
            temp_x*temp_x +
            temp_y*temp_y +
            temp_z*temp_z
        );
    }
};
typedef Point3_<float> Point3F;
typedef Point3_<double> Point3D;

// Support for 4-cable robot kinematics with independent Z-axis
// axes is in cartesian, so 6 means X, Y, Z, A, B, C
// motors is how many motors are available
template <uint8_t axes, uint8_t motors>
struct FourCableKinematics : KinematicsBase<axes, motors> {
    // We have the four cables for X and Y, then one joint per axis from there
    static const uint8_t joints = (axes-2)+4;

    float steps_per_unit[motors];
    int8_t joint_map[joints]; // for each joint, which motor or -1

    // points relative to the machine control point (center) to the body-side of the cable
    // technically we should compute the point along the pulley tangent to the current exit angle
    // for now we'll use an average point tangent of 45º exit
    // 165.00 mm offset + 0.70710678118 * 13.3186 radius = 174.4176723758
    // 165.00 mm offset + 0.70710678118 * 13.8186 radius = 174.7712257664
    Point3F body_points[4] = {
        {-174.4176723758, -174.4176723758, 0.0}, // A
        {-174.4176723758,  174.4176723758, 0.0}, // B
        { 174.4176723758,  174.4176723758, 0.0}, // C
        { 174.4176723758, -174.4176723758, 0.0}, // D
    };

    // anchor points on the frame
    // const float frame_width = 3019.42; // diagonally 4266mm or 4258
    const float frame_width = 3011;  // diagonally 4266mm-8 or 4258
    // const float frame_width = 3035.0;
    Point3F frame_points[4] = {
        {((frame_width) / 2.0), -((frame_width) / 2.0), 0.0},   // A (closest to D)
        {((frame_width) / 2.0), ((frame_width) / 2.0), 0.0},    // B (closest to C)
        {-((frame_width) / 2.0), ((frame_width) / 2.0), 0.0},   // C (closest to B)
        {-((frame_width) / 2.0), -((frame_width) / 2.0), 0.0},  // D (closest to A)
    };

    // cable zero offset - the additional length of the cable past the anchor point (switch hit)
    float cable_zero_offsets[4] = {
        0.0, // A
        0.0, // B
        0.0, // C
        0.0, // D
    };

    // cable spring factor - how much the cable pulls the sled under pressure
    // float cable_spring_factor[4] = {
    //     1.9-0.5, // A
    //     3.8-0.5, // B
    //     4.5-0.5, // C
    //     4.3-0.6, // D
    // };

    // precompute z-offset (j)
    // TODO: Fix pluralization inconsistencies
    double j[4];
    double j_sq[4];
    double cable_position[4];
    double cable_stepper_offset[4];  // the difference between cable_position and stepper position (as mm)
    double other_axes[axes - 2];     // to keep track of the Z, A, B, C, etc.
    double cable_vel[4];
    double cable_accel[4];
    double cable_jerk[4];
    double cable_external_encoder_position[4];  // Number of rotations of the external encoders
    double cable_encoder_offset[4];  // amount of error tracked by the external encoders vs the internal encoders
    double cable_encoder_error[4];  // amount of error detected in this last pass (ephemeral)
    uint8_t cable_external_encoder_reads[4];  // keep track of how many times the encoder is read before uses
    bool encoder_needs_read[4];               // as it says - used to know when to request another sensor read
    bool encoder_synced[4];  // used to know when the encoer offset is valid (false means no, and they need synced)

    // float sensor_zero_value[4] = {2.5, 2.5, 2.5, 2.5};
    float sensor_zero_value[4] = {1.45, 1.00, 0.61, 0.97};
    float sensor_value[4];     // stored from last time they were read
    float raw_sensor_value[4];     // stored from last time they were read

    #if (KINEMATICS != KINE_FOUR_CABLE)
        // define a few things to shut the compiler up - kinda hacky until a more modular system is developed
        #define EXTERNAL_ENCODER_MM_PER_REV 1.0
        #define ANCHOR_A_INPUT 0
        #define ANCHOR_B_INPUT 0
        #define ANCHOR_C_INPUT 0
        #define ANCHOR_D_INPUT 0
    #endif

    float const external_encoder_mm_per_rev[4] = {EXTERNAL_ENCODER_MM_PER_REV, -EXTERNAL_ENCODER_MM_PER_REV,
                                                  EXTERNAL_ENCODER_MM_PER_REV, -EXTERNAL_ENCODER_MM_PER_REV};

    // amount the cable rises (or lowers, if negative) per rotation of the motor
    float z_off = 3.17/324.1730919421;
    float z_off_sq = z_off*z_off;

    const float sensor_to_pounds[4] = {0.0371, 0.0371, 0.0371, 0.0371};
    float sensor_zero_target = 3.0;
    // const float sensor_max = 9.0;
    const float sensor_variance = 6.0;

    const float sensor_skip_detection_jump = 10;

    // float sensor_variance = 0.6f; // +-0.6
    // float sensor_zero_target = 0.1f;

    // float friction_loss_parked       = 25.0;   // percentage of loss due to friction per segment, parked
    // float friction_midpoint_parked   = 1000.0; // velocity (mm/min) at the midpoint for friction per segment, parked
    float friction_loss_parked = 15.0;        // percentage of loss due to friction per segment, parked
    float friction_midpoint_parked   = 100.0; // velocity (mm/min) at the midpoint for friction per segment, parked

    float friction_loss_unparked     = 15.0;   // percentage of loss due to friction per segment, NOT parked
    float friction_midpoint_unparked = 15.0;   // velocity (mm/min) at the midpoint for friction per segment, NOT parked

    bool is_anchored = false;

    // use a timer to let the sensors be initied and their readings settle
    Motate::Timeout sensor_settle_timer;

    // setup the analog inputs array - compile-time for now
    gpioAnalogInputReader * const sensor_inputs[4] = {
        ain_r[ANCHOR_A_INPUT-1],
        ain_r[ANCHOR_B_INPUT-1],
        ain_r[ANCHOR_C_INPUT-1],
        ain_r[ANCHOR_D_INPUT-1],
    };

    // setup the inputs array - compile-time for now
    gpioDigitalInputReader * const anchor_inputs[4] = {
        in_r[ANCHOR_A_INPUT-1],
        in_r[ANCHOR_B_INPUT-1],
        in_r[ANCHOR_C_INPUT-1],
        in_r[ANCHOR_D_INPUT-1],
    };

    // 1 - let the sensors settle
    // 2 - back the motors off 10mm (SKIP for now)
    // 3 - read the sensors - record that as baseline
    // 4 - start normal idle activity

    // Joints are mapped to:
    //  0 = FourCable A
    //  1 = FourCable B
    //  2 = FourCable C
    //  3 = FourCable D
    //  4 = Z
    //  5 = A
    //  6 = B
    //  7 = C
    //  8 = U (maybe)
    //  9 = V (maybe)
    // 10 = W (maybe)

    FourCableKinematics() {
        sensor_settle_timer.set(5000);
    }

    bool inited_ = false;

    void sync_encoders(const float step_position[motors], const float position[axes]) override {
        for (uint8_t cable = 0; cable < 4; cable++) {
            encoder_synced[cable] = false; // need to re-sync encoders to the cable
        }
    }

    void configure(const float new_steps_per_unit[motors], const int8_t motor_map[motors]) override
    {
        for (uint8_t joint = 0; joint < joints; joint++) {
            joint_map[joint] = -1;
        }
        for (uint8_t motor = 0; motor < motors; motor++) {
            auto joint = motor_map[motor];
            if (joint >= 0) {
                joint_map[joint] = motor;
            }
            steps_per_unit[motor] = new_steps_per_unit[motor];
        }
        for (uint8_t cable = 0; cable < 4; cable++) {
            j[cable] = body_points[cable][3] - frame_points[cable][3];
            j_sq[cable] = j[cable] * j[cable];
            cable_vel[cable] = 0;
            cable_accel[cable] = 0;
            cable_jerk[cable] = 0;

            if (!inited_) {
                // NOTE: This dictates that the encoders ALWAYS map to the first four joints, in order
                auto encoder = ExternalEncoders[cable];
                encoder->setCallback([joint = cable, this](bool worked, float new_partial_position) {
                    this->encoder_needs_read[joint] = true;
                    this->cable_external_encoder_reads[joint]++;

                    // NOTE: `this` is captured by reference
                    // For clarity, below this-> is used explicitly when it could be left implcit

                    if (!worked) {
                        return;  // bail early
                    }

                    double old_position = this->cable_external_encoder_position[joint];
                    double position_diff;
                    double new_position;

                    auto old_partial_position = old_position - std::trunc(old_position);
                    // partion position is part of a rotation, where 0.0 <= partial_position < 1.0
                    // to convert that to full rotation, first find the difference, then determine
                    // if there was a positive or negative rollover, and convert that to a diff
                    // note we assume that between polls it's impossible to rotate more that +-0.5
                    position_diff = new_partial_position - old_partial_position;
                    if (position_diff < -0.5) {
                        position_diff += 1.0;
                    } else if (position_diff > 0.5) {
                        position_diff -= 1.0;
                    }

                    new_position = old_position+position_diff;

                    this->cable_external_encoder_position[joint] = new_position;

                    ExternalEncoders[joint]->requestAngleFraction();
                });

                this->encoder_needs_read[cable] = true;
                ExternalEncoders[cable]->requestAngleFraction();
            }  // if (!initied)
        } // for (cable)

        // din_handlers[INPUT_ACTION_NONE].registerHandler(&_pin_input_handler);

        inited_ = true; // only allow init to happen once
    }
    void compute_cable_position(const float target[axes])
    {
        Point3F target_point = {target[0], target[1], 0};

        // 0 Compute the four cable lengths
        // Note that Z in target is treated seperately
        Point3F body_points_adj[4] = {
            body_points[0]+target_point,
            body_points[1]+target_point,
            body_points[2]+target_point,
            body_points[3]+target_point
        };

        // 1 determine the ideal cable length (b)
        float b[4] = {
            body_points_adj[0].distance_to(frame_points[0]),
            body_points_adj[1].distance_to(frame_points[1]),
            body_points_adj[2].distance_to(frame_points[2]),
            body_points_adj[3].distance_to(frame_points[3]),
        };

#if 0
        double b_sq[4] = {
            b[0]*b[0],
            b[1]*b[1],
            b[2]*b[2],
            b[3]*b[3],
        };

        // 2 compute the additional z-offset (f) from the ideal lengths
        // f = z_off * (sqrt(b^2+j^2-b^2*z_off^2)+j*z_off)/(1-z_off^2)
        // f_j = f + j
        double denom_inv = 1/(1-z_off_sq);
        double f_j[4] = {
            z_off * (std::sqrt(b_sq[0]+j_sq[0]+b_sq[0]*(-z_off_sq))+j_sq[0]*z_off)*(denom_inv)+j[0],
            z_off * (std::sqrt(b_sq[1]+j_sq[1]+b_sq[1]*(-z_off_sq))+j_sq[1]*z_off)*(denom_inv)+j[1],
            z_off * (std::sqrt(b_sq[2]+j_sq[2]+b_sq[2]*(-z_off_sq))+j_sq[2]*z_off)*(denom_inv)+j[2],
            z_off * (std::sqrt(b_sq[3]+j_sq[3]+b_sq[3]*(-z_off_sq))+j_sq[3]*z_off)*(denom_inv)+j[3],
        };

        for (uint8_t joint = 0; joint < 4; joint++) {
            // 3 adjust for the new z-offset
            // a_1^2 = b^2 + (f + j)^2
            cable_position[joint] = std::sqrt(b_sq[joint] + f_j[joint] * f_j[joint]);
        }
#else
        cable_position[0] = b[0];
        cable_position[1] = b[1];
        cable_position[2] = b[2];
        cable_position[3] = b[3];
#endif

        // squirrel away the other axes
        for (uint8_t axis = 2; axis < axes; axis++) {
            other_axes[axis-2] = target[axis];
        }
    }

    void cables_to_steps(float steps[motors]) {
        // joint == motor in cartesian kinematics, but NOT in 4-cable
        // note that Z is the fifth axis (axis 4 if zero-based), not the third!
        for (uint8_t joint = 0; joint < joints; joint++) {
            auto motor = joint_map[joint];
            if (motor == -1) { continue; }

            // index:         0,         1,         2,         3,  4, 5, 6, 7, 8, 9, 10
            //  axis:         X,         Y,         Z,         A,  B, C, U, V, W
            // joint: FourWireA, FourWireB, FourWireC, FourWireD,  Z, A, B, C, U, V, W
            // NOTE: U V W may not yet be supported, in which case axes will be 6

            if (joint < 4) {
                steps[motor] = (cable_position[joint] + cable_stepper_offset[joint]) * steps_per_unit[motor];
            } else {
                // other_axes[0] is the value for Z, and joint == 4 means axis Z
                steps[motor] = other_axes[joint-4] * steps_per_unit[motor];
            }
        }
    }

    double prev_cable_position[4];
    double prev_cable_vel[4];
    double prev_cable_accel[4];

    void inverse_kinematics(const GCodeState_t &gm, const float target[axes], const float position[axes], const float start_velocity,
                            const float end_velocity, const float segment_time, float steps[motors]) override {

        // read_sensors() also calls compute_encoder_error() which adjusts cable_position() incorporating the error
        read_sensors();

        // capture old position, etc., before computing the new ones
        for (uint8_t joint = 0; joint < 4; joint++) {
            prev_cable_position[joint] = cable_position[joint];
            prev_cable_vel[joint] = cable_vel[joint];
            prev_cable_accel[joint] = cable_accel[joint];
        }

        // computes the ideal cable lengths without reguard to encoders etc.
        compute_cable_position(target);

        // Note that there are two points in time represented here:
        // The start of this segment, and the end of this segment

        // The segment will take segment_time to complete, and
        // start at start_velocity and end at end_velocity,
        // which are BOTH cartesian!

        // We can assume start_velocity is the end_velocity of the last segment,
        // or is assumed (by the planner) to be something we can achieve from
        // the previous end_velocity, so we'll ingore the given start_velocity
        // here.

        // So now we need to compute the velocity, acceleration, and jerk at the
        // end of this move. For now we simply record them and use them in
        // idle-kinematics.

        // L   := length
        // P_1 := start positon
        // P_2 := end position
        // V   := average velocity
        // V_1 := start velocity
        // V_2 := end velocity
        // T   := time
        // A_1 := this move acceleration
        // A_0 := previous acceleration
        // J   := jerk

#if 0
        for (uint8_t joint = 0; joint < 4; joint++) {

            // compute cable vel, accel, jerk

            // L = V*T          ->  V = L/T
            // V = (V_1+V_2)/T  ->  V_2 = (2*V) - V_1  ->  V_1 = (2*V) - V_2
            // L = P_2 - P_1
            // Combined: V_2 = (2*(P_2-P_1)/T)) - V_1
            // cable_vel[joint] = 2.0*(cable_position[joint] - prev_cable_position[joint])/segment_time - prev_cable_vel[joint];
            cable_vel[joint] = (cable_position[joint] - prev_cable_position[joint]) / segment_time;

            // A_1 = (V_2-V_1)/T
            // Can also be computed from V_1, P_1, and P_2,
            //  but V_1 computation requires previous move info and
            //  so we might as well use computed and stored velocity.
            cable_accel[joint] = (cable_vel[joint]-prev_cable_vel[joint])/segment_time;

            // J = (A_1-A_0)/T
            cable_jerk[joint] = (cable_accel[joint]-prev_cable_accel[joint])/segment_time;

            // float jmax = cm->a[AXIS_X].jerk_max * JERK_MULTIPLIER * 1.5; // 1.5 margin for cartesian plan to cable jerk
            bool jerk_or_velocity_adjusted = false;
            // if (cable_jerk[joint] > jmax) {
            //     cable_jerk[joint] = jmax;
            // } else if (cable_jerk[joint] < -jmax) {
            //     cable_jerk[joint] = -jmax;
            // }

            // if (jerk_or_velocity_adjusted) {
            //     cable_accel[joint] = cable_accel[joint] + cable_jerk[joint]*segment_time;
            //     cable_vel[joint] = cable_vel[joint] + cable_accel[joint]*segment_time;
            // }

            if ((cable_vel[joint] > 10.0 && this->cable_encoder_error[joint] < -0.0) || std::abs(this->cable_encoder_error[joint]) > 2.0) {
                // const float segment_time = MIN_SEGMENT_TIME; // time in MINUTES
                cable_vel[joint] = cable_vel[joint]*0.9 + ((cable_position[joint] - prev_cable_position[joint])/segment_time)*0.1;
                jerk_or_velocity_adjusted = true;
            }

            // limit velocity
            const double vmax = cm->a[AXIS_X].velocity_max * 1.7;
            if (cable_vel[joint] < -vmax) {
                cable_vel[joint] = -vmax;
                jerk_or_velocity_adjusted = true;
            } else if (cable_vel[joint] > vmax) {
                cable_vel[joint] = vmax;
                jerk_or_velocity_adjusted = true;
            }

            if (jerk_or_velocity_adjusted) {
                // cable_position[joint] = prev_cable_position[joint] + (cable_vel[joint] * segment_time);
            }
        }
#else
        cable_vel[0] = 0.0;
        cable_vel[1] = 0.0;
        cable_vel[2] = 0.0;
        cable_vel[3] = 0.0;

        cable_accel[0] = 0.0;
        cable_accel[1] = 0.0;
        cable_accel[2] = 0.0;
        cable_accel[3] = 0.0;

        cable_jerk[0] = 0.0;
        cable_jerk[1] = 0.0;
        cable_jerk[2] = 0.0;
        cable_jerk[3] = 0.0;
#endif

        cables_to_steps(steps);

        // prev_segment_time = segment_time;

        last_segment_was_idle = false;
    }

    // void inverse_kinematics(const float target[axes], float steps[motors]) override
    // {
    //     compute_cable_position(target);

    //     for (uint8_t joint = 0; joint < 4; joint++) {
    //         cable_vel[joint] = 0.0;
    //         cable_accel[joint] = 0.0;
    //         cable_jerk[joint] = 0.0;
    //     }

    //     cables_to_steps(steps);

    //     last_segment_was_idle = false;
    // }

    float best_steps_per_unit[joints];

    void forward_kinematics(const float steps[motors], float position[axes]) override
    {
        // pass 1: convert steps to cable lengths - reset the cable_position and other_axes

        // Setup
        for (uint8_t axis = 0; axis < axes; axis++) {
            position[axis] = 0.0;
        }
        for (uint8_t joint = 0; joint < joints; joint++) {
            best_steps_per_unit[joint] = -1.0;
        }

        // joint != motor here
        for (uint8_t joint = 0; joint < joints; joint++) {
            auto motor = joint_map[joint];
            if (motor == -1) { continue; }

            // If this motor has a better (or the only) resolution, then we use this motor's value
            if (best_steps_per_unit[joint] < steps_per_unit[motor]) {
                best_steps_per_unit[joint] = steps_per_unit[motor];
                float position_temp        = steps[motor] / steps_per_unit[motor];
                if (joint < 4) {
                    // computation for steps is:
                    //  steps[motor] = (cable_position[joint] + cable_stepper_offset[joint]) * steps_per_unit[motor];
                    //  AKA: s = (p + o) * u
                    //  Solved for p (cable_position): p = s/u - o

                    cable_position[joint] = position_temp - cable_stepper_offset[joint];
                } else {
                    // other_axes[0] is the value for Z, and joint == 4 means axis Z
                    other_axes[joint-4] = position_temp;
                }
            }

            encoder_synced[joint] = false; // need to re-sync encoders to the cable
        }

        // pass 2: convert cable lengths to cartesian position
        return get_position(position);
    }

    void get_position(float position[axes]) override
    {
        const float x_body_width = std::abs(body_points[3].x - body_points[0].x);
        const float x_frame_width = std::abs(frame_points[3].x - frame_points[0].x);
        const float w = x_frame_width - x_body_width;

        const float a = cable_position[0];// - cable_stepper_offset[0];
        // const float b = cable_position[1] - cable_stepper_offset[1];
        // const float c = cable_position[2] - cable_stepper_offset[2];
        const float d = cable_position[3];//    -cable_stepper_offset[3];

        const float e = sqrt(std::abs((a - d - w) * (a + d - w) * (a - d + w) * (a + d + w))) / (2.0f*w);
        const float g = sqrt(a*a - e*e);

        // X
        position[0] = (g + frame_points[0].x) - body_points[0].x;
        // Y
        position[1] = (e + frame_points[0].y) - body_points[0].y;

        // joint == motor in cartesian kinematics
        for (uint8_t axis = 2; axis < axes; axis++) {
            // use the stored value for the other axes, and in other_axes z is first
            position[axis] = other_axes[axis-2];
        }
    }

    float start_velocities[motors];
    float end_velocities[motors];
    float target_accel[4] = {0.0, 0.0, 0.0, 0.0};
    float sensor_diff[4] = {0.0, 0.0, 0.0, 0.0};
    bool last_switch_state[4];

    bool anchored() { return is_anchored; }
    void anchored(bool v) {
        is_anchored = v;

        // if we are setting it to false, do NOT reset the cables
        if (!is_anchored) {
            return;
        }

        // TODO: NOT assume 0,0

        float target[axes]; // will init to 0
        target[0] = 0.0;
        target[1] = 0.0;
        for (uint8_t axis = 2; axis < axes; axis++) {
            // use the stored value for the other axes, and in other_axes z is first
            target[axis] = other_axes[axis-2];
        }

        sync_encoders();
        compute_cable_position(target);

        cable_vel[0] = 0.0;
        cable_vel[1] = 0.0;
        cable_vel[2] = 0.0;
        cable_vel[3] = 0.0;

        cable_accel[0] = 0.0;
        cable_accel[1] = 0.0;
        cable_accel[2] = 0.0;
        cable_accel[3] = 0.0;

        cable_jerk[0] = 0.0;
        cable_jerk[1] = 0.0;
        cable_jerk[2] = 0.0;
        cable_jerk[3] = 0.0;
    }

    uint8_t encoder_failures[4] = {0,0,0,0};

    bool read_sensors() {
        compute_encoder_error();

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

        for (uint8_t joint = 0; joint < 4; joint++) {
            // note we invert and zero the values
            raw_sensor_value[joint] = (sensor_inputs[joint]->getValue() - sensor_zero_value[joint]) / sensor_to_pounds[joint];

            // new_sensor_value at -1 is zero tension, 1 is max tension, and 0 is goldilocks
            float new_sensor_value = (raw_sensor_value[joint] - sensor_zero_target) / sensor_variance;

            // new_sensor_diff is literally the change of the sensor value since we last read it
            sensor_diff[joint]  = (new_sensor_value - sensor_value[joint]);

            sensor_value[joint] = new_sensor_value;
        }
        return true;
    }

    void compute_encoder_error() {
        for (uint8_t joint = 0; joint < 4; joint++) {
            if (cable_external_encoder_reads[joint] == 0) {
                if (++encoder_failures[joint] > 15) {
                    encoder_synced[joint] = false;
                }
                if ((encoder_failures[joint] > 30)) {
                   cm_alarm(STAT_ENCODER_ASSERTION_FAILURE, "encoder stopped returning values");
                }
                // #ifdef IN_DEBUGGER
                // __asm__("BKPT");  // no encoder reads!?
                // #endif
                continue;
            }
            encoder_failures[joint] = 0;

            cable_external_encoder_reads[joint] = 0;
            double external_encoder_mm =
                this->cable_external_encoder_position[joint] * external_encoder_mm_per_rev[joint];

            // Adjust position based on load (should be in newtons)

            // measured 26.88 off of 2149.2536, at roughly 3lb of load vs no load
            // At 3lb of load, it stretches 26.88mm to 2149.2536, meaning that the unloaded length would be 2,122.3736
            // So, 2,122.3736 + (2,122.3736*stretch_factor*lb) = 2149.2536
            // b + (b*s*f) = l
            // b*s*f=l-b
            // s = (l-b)/(b*f)
            // s = 0.03799519557
            // double const sensor_stretch_factor = 0.03799519557;
            // double const sensor_stretch_factor = 0.0015;
            // external_encoder_mm = external_encoder_mm + (external_encoder_mm * sensor_stretch_factor * raw_sensor_value[joint]);

            if (encoder_synced[joint]) {
                external_encoder_mm = external_encoder_mm + this->cable_encoder_offset[joint];
                double start_cable_position = prev_cable_position[joint];

                double smaller = std::min(start_cable_position, cable_position[joint]);
                double bigger = std::max(start_cable_position, cable_position[joint]);

                double new_error_offset = 0.0; // + for EE too high, - for EE too low

                if (external_encoder_mm < smaller) {
                    new_error_offset = external_encoder_mm - smaller;
                } else if (external_encoder_mm > bigger) {
                    new_error_offset = external_encoder_mm - bigger;
                } else {
                    new_error_offset = 0.0;
                }

                this->cable_encoder_error[joint] = new_error_offset;

                if (std::abs(this->cable_encoder_error[joint]) > std::abs(external_encoder_mm_per_rev[joint])) {
                    // we're off by more than one full rotation (how?), resync and erase the error
                    new_error_offset = 0; // we'll catch the actual error next time around
                    encoder_synced[joint] = false;
                }
                // else if (std::abs(this->cable_encoder_error[joint]) > std::abs(external_encoder_mm_per_rev[joint] * 0.5)) {
                //     // the guess about which way it was revolving was wrong, adjust by one rotation
                //     if ((this->cable_encoder_error[joint] < 0) && (external_encoder_mm_per_rev[joint] < 0)) {
                //         this->cable_external_encoder_position[joint] -= 1;
                //     } else {
                //         this->cable_external_encoder_position[joint] += 1;
                //     }
                //     new_error_offset = 0; // we'll catch the actual error next time around
                // }

                double new_error_offset_adjustment = new_error_offset * 0.001;
                // double new_error_offset_adjustment = 0.0; // encoders disabled!!

                // Adjust BOTH the stepper adjustment and the cable length so that the steps computed
                // from before this adjustment will match those computed after this adjustment.
                // IOW, we're adjusting cable_position AND it's conversion to steps to more closely match the encoders,
                // without any additional motion. The motion correction will be natural from cable_position being
                // computed relatively as the new target.
                cable_position[joint] = cable_position[joint] + new_error_offset_adjustment;
                cable_stepper_offset[joint] = cable_stepper_offset[joint] - new_error_offset_adjustment;

                // adjust cable_vel to match reality, mostly for idle_time, if it would slack the line
                if ((cable_vel[joint] > 10.0 && new_error_offset < -0.0) || std::abs(new_error_offset) > 2.0) {
                    const float segment_time = MIN_SEGMENT_TIME; // time in MINUTES
                    cable_vel[joint] = cable_vel[joint]*0.9 + ((cable_position[joint] - start_cable_position)/segment_time)*0.1;
                }

            } else if (raw_sensor_value[joint] > 1) { // once the cable has some minimal load
                // Reset the encoder offset to this new value.
                // Make it so that, right now, (external_encoder_mm+cable_encoder_offset)-cable_position = 0 error
                encoder_synced[joint] = true;
                this->cable_encoder_offset[joint] = cable_position[joint]-external_encoder_mm;
            }
        }
    }

    // if we requested a move, return true, otherwise false
    bool last_segment_was_idle = false;
    bool idle_task() override {

        /* Notes about this situation:
        * 1. This is called from Exec, which is called from Load, which is called (ignoring bootstrapping) from the stepper
        *    when a segment is over.
        * 2. The currently running segment in the stepper subsystem (which may be a movement-free segment) has a target of
        *    the current cable_position[] (as it's known in this part of the code) and the start position of
        *    prev_cable_position[].
        * 3. The encoder was read during the last segment encoder_readings_taken[] times (may be zero).
        * 4. If encoder_readings_taken[] is non-zero, then the last reading was taken at some point during the last segment,
        *    and should be somewhere between prev_cable_position[] and cable_position[].
        */

        if (!read_sensors() || is_anchored) {
            return false; // too soon - sensors are still settling
        }

        if (!last_segment_was_idle) {
            for (uint8_t joint = 0; joint < 4; joint++) {
                cable_vel[joint] = 0.0;
                cable_accel[joint] = 0.0;
                cable_jerk[joint] = 0.0;
                sensor_value[joint] = 0.0;
                sensor_diff[joint] = 0.0;
            }
        }
        last_segment_was_idle = true;

        // check inputs to be sure we aren't anchored
        // if we are anchored, then set the zero-position offsets
        // the first segment that isn't anchored will use them

        const float segment_time = MIN_SEGMENT_TIME; // time in MINUTES

        for (uint8_t joint = 0; joint < 4; joint++) {
            // bool switch_state = anchor_inputs[joint]->getState();
            bool switch_state = false; // ignore switches
            /* ignoring switch for cable_stepper_offset for now
            if (switch_state && !last_switch_state[joint]) {
                // active means anchored - update the offset
                // when the switch goes inactive, then the last used offset is right

                // we account for cable_zero_offsets here, since there is some cable
                // beyond the switch
                cable_stepper_offset[joint] = cable_position[joint] - cable_zero_offsets[joint];
            }
            */

            // prev_cable_accel[joint] = cable_accel[joint];
            start_velocities[joint] = std::abs(cable_vel[joint]);
            // prev_cable_vel[joint] = cable_vel[joint];

            // heiristic to detect skips
            // if the pressure rises the 80% of the full range in a segment

            // if (sensor_diff[joint] > sensor_skip_detection_jump) {
            //     cable_jerk[joint] = 0.0; // clear it out
            //     cable_accel[joint] = 0.0; // same
            //     cable_vel[joint] = 50.0; // it's already stopped, back it off some
            // }

            float jmax = cm->a[AXIS_X].jerk_high * JERK_MULTIPLIER;
            cable_jerk[joint] = sensor_diff[joint] * jmax;
            cable_accel[joint] = cable_accel[joint] + cable_jerk[joint]*segment_time;

            // static friction
            auto friction = ((switch_state ? friction_loss_parked : friction_loss_unparked)/100.0);
            auto friction_midpoint = (switch_state ? friction_midpoint_parked : friction_midpoint_unparked);
            auto friction_loss = (friction*friction_midpoint)/(std::abs(cable_vel[joint]) + friction_midpoint);
            cable_vel[joint] = cable_vel[joint] - cable_vel[joint] * friction_loss;
            cable_vel[joint] = cable_vel[joint] + cable_accel[joint]*segment_time;

            // limit velocity
            const double vmax = cm->a[AXIS_X].velocity_max;
            if (cable_vel[joint] < -vmax) {
                cable_vel[joint] = -vmax;
                // error_offset = 0; // none of it was applied
            } else if (cable_vel[joint] > vmax) {
                cable_vel[joint] = vmax;
                // error_offset = 0; // none of it was applied
            }

            end_velocities[joint] = std::abs(cable_vel[joint]);
            prev_cable_position[joint] = cable_position[joint];
            cable_position[joint] = cable_position[joint] + (cable_vel[joint] * segment_time);

            last_switch_state[joint] = switch_state;
        } // for joint

        // convert them to steps
        float target_steps[motors];
        cables_to_steps(target_steps);

        // tell the planner and runtime about them
        mp_set_target_steps(target_steps, start_velocities, end_velocities, segment_time);

        return true;
    }
};


#endif  // End of include Guard: KINEMATICS_FOUR_CABLE_H_ONCE
