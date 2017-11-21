/*
 * settings_zen7x12.h - Zen Toolworks 7x12 machine profile
 * This file is part of the g2core project
 *
 * Copyright (c) 2011 - 2016 Alden S. Hart, Jr.
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

/***********************************************************************/
/**** Zen Toolworks 7x12 profile ***************************************/
/***********************************************************************/

// ***> NOTE: The init message must be a single line with no CRs or LFs
#define INIT_MESSAGE "Initializing configs to Zen Toolworks 7x12 profile"

#define JERK_MAX_LINEAR         500         // 500,000,000 mm/(min^3)
#define JERK_MAX_ROTARY         10000       // 10 billion mm/(min^3)
#define JUNCTION_ACCELERATION   100000      // centripetal acceleration around corners

// *** settings.h overrides ***

#undef COMM_MODE
#define COMM_MODE               JSON_MODE

#undef JSON_VERBOSITY
#define JSON_VERBOSITY          JV_VERBOSE

// *** motor settings ***

#define M1_MOTOR_MAP            AXIS_X                  // 1ma
#define M1_STEP_ANGLE           1.8                     // 1sa
#define M1_TRAVEL_PER_REV       1.25                    // 1tr
#define M1_MICROSTEPS           8                       // 1mi        1,2,4,8
#define M1_POLARITY             1                       // REVERSE// 1po        0=normal, 1=reverse
#define M1_POWER_MODE           MOTOR_POWERED_IN_CYCLE  // 1pm        standard
#define M1_POWER_LEVEL          0.5                     // 1pl:   0.0=no power, 1.0=max power

#define M2_MOTOR_MAP            AXIS_Y
#define M2_STEP_ANGLE           1.8
#define M2_TRAVEL_PER_REV       1.25
#define M2_MICROSTEPS           8
#define M2_POLARITY             0
#define M2_POWER_MODE           MOTOR_POWERED_IN_CYCLE
#define M2_POWER_LEVEL          0.5

#define M3_MOTOR_MAP            AXIS_Z
#define M3_STEP_ANGLE           1.8
#define M3_TRAVEL_PER_REV       1.25
#define M3_MICROSTEPS           8
#define M3_POLARITY             1            // REVERSE
#define M3_POWER_MODE           MOTOR_POWERED_IN_CYCLE
#define M3_POWER_LEVEL          0.5

// *** axis settings ***

#define X_AXIS_MODE             AXIS_STANDARD           // xam        see canonical_machine.h cmAxisMode for valid values
#define X_VELOCITY_MAX          600                     // xvm        G0 max velocity in mm/min
#define X_FEEDRATE_MAX          X_VELOCITY_MAX          // xfr         G1 max feed rate in mm/min
#define X_TRAVEL_MIN            0                       // xtn        minimum travel - used by soft limits and homing
#define X_TRAVEL_MAX            475                     // xtm        maximum travel - used by soft limits and homing
#define X_JERK_MAX              JERK_MAX_LINEAR         // xjm
#define X_JERK_HIGH_SPEED       X_JERK_MAX              // xjh
#define X_SWITCH_MODE_MIN       SW_MODE_HOMING          // xsn        SW_MODE_DISABLED, SW_MODE_HOMING, SW_MODE_LIMIT, SW_MODE_HOMING_LIMIT
#define X_SWITCH_MODE_MAX       SW_MODE_LIMIT           // xsx        SW_MODE_DISABLED, SW_MODE_HOMING, SW_MODE_LIMIT, SW_MODE_HOMING_LIMIT
//#define X_SWITCH_MODE_MAX         SW_MODE_DISABLED    // xsx        SW_MODE_DISABLED, SW_MODE_HOMING, SW_MODE_LIMIT, SW_MODE_HOMING_LIMIT
#define X_SWITCH_TYPE_MIN       SW_TYPE_NORMALLY_OPEN   // rsn    SW_TYPE_NORMALLY_OPEN, SW_TYPE_NORMALLY_CLOSED
#define X_SWITCH_TYPE_MAX       SW_TYPE_NORMALLY_OPEN   // rsx    SW_TYPE_NORMALLY_OPEN, SW_TYPE_NORMALLY_CLOSED
#define X_SEARCH_VELOCITY       500                     // xsv        move in negative direction
#define X_LATCH_VELOCITY        100                     // xlv        mm/min
#define X_LATCH_BACKOFF         2                       // xlb        mm
#define X_ZERO_BACKOFF          1                       // xzb        mm

#define Y_AXIS_MODE             AXIS_STANDARD
#define Y_VELOCITY_MAX          600
#define Y_FEEDRATE_MAX          Y_VELOCITY_MAX
#define Y_TRAVEL_MIN            0
#define Y_TRAVEL_MAX            200
#define Y_JERK_MAX              JERK_MAX_LINEAR
#define Y_JERK_HIGH_SPEED       Y_JERK_MAX
#define Y_SWITCH_MODE_MIN       SW_MODE_HOMING
#define Y_SWITCH_MODE_MAX       SW_MODE_LIMIT
#define Y_SWITCH_TYPE_MIN       SW_TYPE_NORMALLY_OPEN
#define Y_SWITCH_TYPE_MAX       SW_TYPE_NORMALLY_OPEN
#define Y_SEARCH_VELOCITY       500
#define Y_LATCH_VELOCITY        100
#define Y_LATCH_BACKOFF         2
#define Y_ZERO_BACKOFF          1

#define Z_AXIS_MODE             AXIS_STANDARD
#define Z_VELOCITY_MAX          500
#define Z_FEEDRATE_MAX          Z_VELOCITY_MAX
#define Z_TRAVEL_MIN            0
#define Z_TRAVEL_MAX            75
#define Z_JERK_MAX              JERK_MAX_LINEAR
#define Z_JERK_HIGH_SPEED       Z_JERK_MAX
#define Z_SWITCH_MODE_MIN       SW_MODE_DISABLED
#define Z_SWITCH_MODE_MAX       SW_MODE_HOMING
#define Z_SWITCH_TYPE_MIN       SW_TYPE_NORMALLY_OPEN
#define Z_SWITCH_TYPE_MAX       SW_TYPE_NORMALLY_OPEN
#define Z_SEARCH_VELOCITY       400
#define Z_LATCH_VELOCITY        100
#define Z_LATCH_BACKOFF         2
#define Z_ZERO_BACKOFF          1
