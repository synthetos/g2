/*
 * kinematics.cpp - inverse kinematics routines
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2019 Alden S. Hart, Jr.
 * Copyright (c) 2016 - 2019 Rob Giseburt
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

#include "g2core.h"
#include "settings.h"
#include "kinematics.h"
#include "stepper.h"

// Note that *technically* kn can be switched at runtime - but that would likely break all kinds of stuff

#if KINEMATICS==KINE_OTHER
// kn must be assigned elsewhere!
// KinematicsBase<AXES, MOTORS> *kn = &other_kinematics;
#endif
#if KINEMATICS==KINE_CARTESIAN
#include "kinematics_cartesian.h"
CartesianKinematics<AXES, MOTORS> cartesian_kinematics;
KinematicsBase<AXES, MOTORS> *kn = &cartesian_kinematics;
#endif
#if KINEMATICS==KINE_CORE_XY
#include "kinematics_cartesian.h"
CoreXYKinematics<AXES, MOTORS> core_xy_kinematics;
KinematicsBase<AXES, MOTORS> *kn = &core_xy_kinematics;
#endif
#if KINEMATICS==KINE_FOUR_CABLE
#include "kinematics_four_cable.h"
FourCableKinematics<AXES, MOTORS> four_cable_kinematics;
KinematicsBase<AXES, MOTORS> *kn = &four_cable_kinematics;

// gpioDigitalInputHandler _pin_input_handler{
//     [&](const bool state, const inputEdgeFlag edge, const uint8_t triggering_pin_number) {
//         if (state == true) {
//             four_cable_kinematics.anchored((four_cable_kinematics.anchored());
//         }

//         return false; // allow others to see this notice
//     },
//     5,       // priority
//     nullptr  // next - nullptr to start with
// };

// force
stat_t kn_get_force(nvObj_t *nv)
{
    nv->valuetype = TYPE_FLOAT;
    nv->precision = 4;
    nv->value = four_cable_kinematics.sensor_zero_target; // read it as a float

    return (STAT_OK);
};
stat_t kn_set_force(nvObj_t *nv)
{
    float value = nv->value; // read it as a float
    four_cable_kinematics.sensor_zero_target = value;
    return (STAT_OK);
};

// anchored

stat_t kn_get_anchored(nvObj_t *nv)
{
    nv->valuetype = TYPE_BOOL;
    nv->value = four_cable_kinematics.anchored();

    return (STAT_OK);
};
stat_t kn_set_anchored(nvObj_t *nv)
{
    bool value = std::fabs(nv->value) > 0.1;
    four_cable_kinematics.anchored(value);
    return (STAT_OK);
};

// joint positions
stat_t kn_get_pos_a(nvObj_t *nv)
{
    nv->valuetype = TYPE_FLOAT;
    nv->precision = 4;
    nv->value = four_cable_kinematics.cable_position[0];

    return (STAT_OK);
};
stat_t kn_get_pos_b(nvObj_t *nv)
{
    nv->valuetype = TYPE_FLOAT;
    nv->precision = 4;
    nv->value = four_cable_kinematics.cable_position[1];

    return (STAT_OK);
};
stat_t kn_get_pos_c(nvObj_t *nv)
{
    nv->valuetype = TYPE_FLOAT;
    nv->precision = 4;
    nv->value = four_cable_kinematics.cable_position[2];

    return (STAT_OK);
};
stat_t kn_get_pos_d(nvObj_t *nv)
{
    nv->valuetype = TYPE_FLOAT;
    nv->precision = 4;
    nv->value = four_cable_kinematics.cable_position[3];

    return (STAT_OK);
};
#endif // KINEMATICS==KINE_FOUR_CABLE


// Concrete functions that involve kinematics

/*
 * kn_config_changed() - call to update the configuration from the globals
 */
void kn_config_changed() {
    // temporary load these up every time until we can hook them to the configuration
    int8_t motor_map[MOTORS];
    float steps_per_unit[MOTORS];

    for (uint8_t motor = 0; motor < MOTORS; motor++) {
        auto axis = st_cfg.mot[motor].motor_map;
#ifdef IN_DEBUGGER
        if (axis >= AXES) {
            __asm__("BKPT");  // about to send non-Setup message
        }
#endif
        if (cm->a[axis].axis_mode == AXIS_INHIBITED) {
            motor_map[motor] = -1;
            steps_per_unit[motor] = 1;  // this is the denominator above, avoid 0
        } else {
            motor_map[motor] = axis;
            steps_per_unit[motor] = st_cfg.mot[motor].steps_per_unit;
        }
    }

    kn->configure(steps_per_unit, motor_map);

    mp_set_steps_to_runtime_position();
}

/*
 * kn_forward_kinematics() - forward kinematics for a cartesian machine
 *
 * This is designed for PRECISION, not PERFORMANCE!
 *
 * This function is NOT to be used where high-speed is important. If that becomes the case,
 * there are many opportunities for caching and optimization for performance here.
 *
 */

void kn_forward_kinematics(const float steps[], float travel[]) {
    // PRESUMPTION: inverse kinematics has been called at least once since the mapping or steps_unit has changed
    kn->forward_kinematics(steps, travel);
}
