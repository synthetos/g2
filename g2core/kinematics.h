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

#ifndef KINEMATICS_H_ONCE
#define KINEMATICS_H_ONCE

#include "util.h"
#include "gcode.h" // for GCodeState_t

/* Generic Functions
 *
 * In this first section we want to write the kinematic functions that DO NOT reach out of this file.
 * IOW, these classes have *no knowledge* of globals such as cm, st_cfg, etc.
 *
 * This will facilitate decoupling and later move to full dependency-injection.
 *
 */

template <uint8_t axes, uint8_t motors>
struct KinematicsBase {
    // configure each joint (steps-per-unit, joint mapping)
    virtual void configure(const float steps_per_unit[motors], const int8_t motor_map[motors]);

    // take the target (in cartesian coordinates in mm), and convert them to steps for each joints
    // taking the joint_map into consideration, and returning the values in the provided array steps[]
    // must be as fast as possible while retaining precision
    // the other information is for the sake of tracking and intelligent error correction
    // the derivatives (acceleration, jerk) or other considerations
    // the gcode model is passed in for additional context, and may be ignored
    // the target is in the gcode model, but may be modified, so it's passed separately
    virtual void inverse_kinematics(const GCodeState_t &gm, const float target[axes], const float position[axes], const float start_velocity, const float end_velocity, const float segment_time, float steps[motors]) {
    }

    // if the planner buffer is empty, the idel_task will be given the opportunity to drive the runtime
    // if motion was requested, return true.
    // the default action is to do nothing, and return false
    virtual bool idle_task() {
        return false;
    }

    // take the position (in steps) of each joint and convert them to cartesian coordinates
    // taking the joint_map into consideration, and returning the values in the provided array position[]
    // can be relatively slow, must be precise
    virtual void forward_kinematics(const float steps[motors], float position[axes]);


    // take the position of each joint at idle time and convert them to cartesian coordinates
    // taking the joint_map into consideration, and returning the values in the provided array position[]
    // can be relatively slow, must be precise
    virtual void get_position(float position[axes]);

    // sync any external sensors with the current step position
    virtual void sync_encoders(const float step_position[motors], const float position[axes]);
};

extern KinematicsBase<AXES, MOTORS> *kn;

/*
 * Old-style Global Scope Functions (depricated)
 */
#if KINEMATICS==KINE_FOUR_CABLE
// force
stat_t kn_get_force(nvObj_t *nv);
stat_t kn_set_force(nvObj_t *nv);

// anchored
stat_t kn_get_anchored(nvObj_t *nv);
stat_t kn_set_anchored(nvObj_t *nv);

// joint positions
stat_t kn_get_pos_a(nvObj_t *nv);
stat_t kn_get_pos_b(nvObj_t *nv);
stat_t kn_get_pos_c(nvObj_t *nv);
stat_t kn_get_pos_d(nvObj_t *nv);
#endif

void kn_config_changed();
void kn_forward_kinematics(const float steps[], float travel[]);

#endif  // End of include Guard: KINEMATICS_H_ONCE
