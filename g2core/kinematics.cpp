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
#if KINEMATICS==KINE_PRESSURE
#include "kinematics_pressure.h"
PressureKinematics<AXES, MOTORS> pressure_kinematics;
KinematicsBase<AXES, MOTORS> *kn = &pressure_kinematics;

// volume
stat_t kn_get_force(nvObj_t *nv)
{
    nv->valuetype = TYPE_FLOAT;
    nv->precision = 4;
    nv->value_flt = pressure_kinematics.event_pressure_target; // read it as a float

    return (STAT_OK);
};
stat_t kn_set_force(nvObj_t *nv)
{
    float value = nv->value_flt; // read it as a float
    nv->precision = 4;
    pressure_kinematics.event_pressure_target = value;
    return (STAT_OK);
};

stat_t kn_get_target(nvObj_t *nv)
{
    nv->valuetype = TYPE_FLOAT;
    nv->precision = 4;
    nv->value_flt = pressure_kinematics.event_pressure_target; // read it as a float

    return (STAT_OK);
};
stat_t kn_set_target(nvObj_t *nv)
{
    float value = nv->value_flt; // read it as a float
    nv->precision = 4;
    pressure_kinematics.event_pressure_target = value;
    return (STAT_OK);
};

stat_t kn_get_epm(nvObj_t *nv)
{
    nv->valuetype = TYPE_FLOAT;
    nv->precision = 4;
    nv->value_flt = 60.0 / pressure_kinematics.seconds_between_events;

    return (STAT_OK);
};
stat_t kn_set_epm(nvObj_t *nv)
{
    float value = nv->value_flt; // read it as a float
    nv->precision = 4;
    pressure_kinematics.seconds_between_events = 60.0 / value;
    pressure_kinematics.seconds_to_hold_event =
        pressure_kinematics.seconds_between_events / (pressure_kinematics.pressure_hold_release_ratio + 1);

    return (STAT_OK);
};


stat_t kn_get_hold_time(nvObj_t *nv)
{
    nv->valuetype = TYPE_FLOAT;
    nv->precision = 4;
    nv->value_flt = pressure_kinematics.seconds_to_hold_event;

    return (STAT_OK);
};
stat_t kn_set_hold_time(nvObj_t *nv)
{
    float value = nv->value_flt; // read it as a float
    nv->precision = 4;
    pressure_kinematics.seconds_to_hold_event = value;
    pressure_kinematics.pressure_hold_release_ratio =
        (pressure_kinematics.seconds_between_events / pressure_kinematics.seconds_to_hold_event) - 1;

    return (STAT_OK);
};


stat_t kn_get_hold_ratio(nvObj_t *nv)
{
    nv->valuetype = TYPE_FLOAT;
    nv->precision = 4;
    nv->value_flt = pressure_kinematics.pressure_hold_release_ratio;

    return (STAT_OK);
};
stat_t kn_set_hold_ratio(nvObj_t *nv)
{
    float value = nv->value_flt; // read it as a float
    nv->precision = 4;
    pressure_kinematics.pressure_hold_release_ratio = value;
    pressure_kinematics.seconds_to_hold_event =
        pressure_kinematics.seconds_between_events / (pressure_kinematics.pressure_hold_release_ratio + 1);
    return (STAT_OK);
};


stat_t kn_get_backoff_pressure(nvObj_t *nv)
{
    nv->valuetype = TYPE_FLOAT;
    nv->precision = 4;
    nv->value_flt = pressure_kinematics.reverse_target_pressure;

    return (STAT_OK);
};
stat_t kn_set_backoff_pressure(nvObj_t *nv)
{
    float value = nv->value_flt; // read it as a float
    nv->precision = 4;
    pressure_kinematics.reverse_target_pressure = value;
    return (STAT_OK);
};


stat_t kn_get_e_value(nvObj_t *nv)
{
    nv->valuetype = TYPE_FLOAT;
    nv->precision = 4;
    nv->value_flt = pressure_kinematics.sensor_error_store;

    return (STAT_OK);
};
stat_t kn_get_i_value(nvObj_t *nv)
{
    nv->valuetype = TYPE_FLOAT;
    nv->precision = 4;
    nv->value_flt = pressure_kinematics.sensor_integral_store;

    return (STAT_OK);
};
stat_t kn_get_d_value(nvObj_t *nv)
{
    nv->valuetype = TYPE_FLOAT;
    nv->precision = 4;
    nv->value_flt = pressure_kinematics.sensor_derivative_store;

    return (STAT_OK);
};

stat_t kn_get_uoc_value(nvObj_t *nv)
{
    nv->valuetype = TYPE_INTEGER;
    nv->value_int = pressure_kinematics.unable_to_obtian_error_counter;

    return (STAT_OK);
};
stat_t kn_get_umc_value(nvObj_t *nv)
{
    nv->valuetype = TYPE_INTEGER;
    nv->value_int = pressure_kinematics.unable_to_maintian_error_counter;

    return (STAT_OK);
};
stat_t kn_get_ec_value(nvObj_t *nv)
{
    nv->valuetype = TYPE_INTEGER;
    nv->value_int = pressure_kinematics.event_counter;

    return (STAT_OK);
};

stat_t kn_get_p_factor(nvObj_t *nv)
{
    nv->valuetype = TYPE_FLOAT;
    nv->precision = 4;
    nv->value_flt = pressure_kinematics.sensor_proportional_factor;

    return (STAT_OK);
};
stat_t kn_set_p_factor(nvObj_t *nv)
{
    float value = nv->value_flt; // read it as a float
    nv->precision = 4;
    pressure_kinematics.sensor_proportional_factor = value;
    return (STAT_OK);
};


stat_t kn_get_i_factor(nvObj_t *nv)
{
    nv->valuetype = TYPE_FLOAT;
    nv->precision = 4;
    nv->value_flt = pressure_kinematics.sensor_inetgral_factor;

    return (STAT_OK);
};
stat_t kn_set_i_factor(nvObj_t *nv)
{
    float value = nv->value_flt; // read it as a float
    nv->precision = 4;
    pressure_kinematics.sensor_inetgral_factor = value;
    return (STAT_OK);
};


stat_t kn_get_d_factor(nvObj_t *nv)
{
    nv->valuetype = TYPE_FLOAT;
    nv->precision = 4;
    nv->value_flt = pressure_kinematics.sensor_derivative_factor;

    return (STAT_OK);
};
stat_t kn_set_d_factor(nvObj_t *nv)
{
    float value = nv->value_flt; // read it as a float
    nv->precision = 4;
    pressure_kinematics.sensor_derivative_factor = value;
    return (STAT_OK);
};

// anchored

stat_t kn_get_anchored(nvObj_t *nv)
{
    nv->valuetype = TYPE_BOOLEAN;
    nv->value_flt = pressure_kinematics.anchored();

    return (STAT_OK);
};
stat_t kn_set_anchored(nvObj_t *nv)
{
    bool value = std::fabs(nv->value_flt) > 0.1;
    pressure_kinematics.anchored(value);
    return (STAT_OK);
};

stat_t _kn_get_pos(uint8_t joint, nvObj_t *nv) {
    float position[AXES];
    pressure_kinematics.get_position(position);

    nv->valuetype = TYPE_FLOAT;
    nv->precision = 4;
    nv->value_flt = position[joint];

    return (STAT_OK);
}

stat_t kn_get_pos_1(nvObj_t *nv) { return _kn_get_pos(1-1, nv); }
stat_t kn_get_pos_2(nvObj_t *nv) { return _kn_get_pos(2-1, nv); }
stat_t kn_get_pos_3(nvObj_t *nv) { return _kn_get_pos(3-1, nv); }
stat_t kn_get_pos_4(nvObj_t *nv) { return _kn_get_pos(4-1, nv); }
stat_t kn_get_pos_5(nvObj_t *nv) { return _kn_get_pos(5-1, nv); }

stat_t get_flow_volume(nvObj_t *nv)
{
    nv->valuetype = TYPE_FLOAT;
    nv->precision = 4;
    nv->value_flt = pressure_kinematics.volume_value[0];

    return (STAT_OK);
};

#endif // KINEMATICS==KINE_PRESSURE


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
