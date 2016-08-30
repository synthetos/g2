/*
 * cycle_jogging.c - jogging cycle extension to canonical_machine.c
 * This file is part of the g2core project
 *
 * by Mike Estee, Tom Cauchois - Other Machine Company
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
#include "config.h"
#include "json_parser.h"
#include "text_parser.h"
#include "canonical_machine.h"
#include "planner.h"
#include "util.h"
#include "xio.h"

/**** Jogging singleton structure ****/

struct jmJoggingSingleton {         // persistent jogging runtime variables
    // controls for jogging cycle
    int8_t  axis;                   // axis currently being jogged
    float   dest_pos;               // distance relative to start position to travel
    float   start_pos;
    float   velocity_start;         // initial jog feed
    float   velocity_max;
    uint8_t step;                   // what step of the ramp the jogging cycle is currently on

    uint8_t (*func)(int8_t axis);   // binding for callback function state machine

    // state saved from gcode model
    float   saved_feed_rate;        // F setting
    uint8_t saved_units_mode;       // G20,G21 global setting
    uint8_t saved_coord_system;     // G54 - G59 setting
    uint8_t saved_distance_mode;    // G90,G91 global setting
    uint8_t saved_feed_rate_mode;
    float   saved_jerk;             // saved and restored for each axis jogged
};
static struct jmJoggingSingleton jog;


/**** NOTE: global prototypes and other .h info is located in canonical_machine.h ****/

static stat_t _set_jogging_func(uint8_t (*func)(int8_t axis));
static stat_t _jogging_axis_start(int8_t axis);
static stat_t _jogging_axis_ramp_jog(int8_t axis);
static stat_t _jogging_axis_move(int8_t axis, float target, float velocity);
static stat_t _jogging_finalize_exit(int8_t axis);

/*****************************************************************************
 * cm_jogging_cycle_start() - jogging cycle using soft limits
 *
 *  --- Some further details ---
 *
 *  Note: When coding a cycle (like this one) you get to perform one queued
 *  move per entry into the continuation, then you must exit.
 *
 *  Another Note: When coding a cycle (like this one) you must wait until
 *  the last move has actually been queued (or has finished) before declaring
 *  the cycle to be done. Otherwise there is a nasty race condition in the
 *  tg_controller() that will accept the next command before the position of
 *  the final move has been recorded in the Gcode model. That's what the call
 *  to cm_isbusy() is about.
 */

stat_t cm_jogging_cycle_start(uint8_t axis) {
    // save relevant non-axis parameters from Gcode model
    jog.saved_units_mode     = cm_get_units_mode(ACTIVE_MODEL);     // cm.gm.units_mode;
    jog.saved_coord_system   = cm_get_coord_system(ACTIVE_MODEL);   // cm.gm.coord_system;
    jog.saved_distance_mode  = cm_get_distance_mode(ACTIVE_MODEL);  // cm.gm.distance_mode;
    jog.saved_feed_rate_mode = cm_get_feed_rate_mode(ACTIVE_MODEL);
    jog.saved_feed_rate      = (ACTIVE_MODEL)->feed_rate;  // cm.gm.feed_rate;
    jog.saved_jerk           = cm.a[axis].jerk_max;

    // set working values
    cm_set_units_mode(MILLIMETERS);
    cm_set_distance_mode(ABSOLUTE_MODE);
    cm_set_coord_system(ABSOLUTE_COORDS);  // jogging is done in machine coordinates
    cm_set_feed_rate_mode(UNITS_PER_MINUTE_MODE);

    jog.velocity_start = JOGGING_START_VELOCITY;  // see canonical_machine.h for #define
    jog.velocity_max   = cm.a[axis].velocity_max;

    jog.start_pos = cm_get_absolute_position(RUNTIME, axis);
    jog.dest_pos  = cm_get_jogging_dest();
    jog.step      = 0;

    jog.axis = axis;
    jog.func = _jogging_axis_start;  // bind initial processing function

    cm.machine_state = MACHINE_CYCLE;
    cm.cycle_state   = CYCLE_JOG;
    return (STAT_OK);
}


/* Jogging axis moves - these execute in sequence for each axis
 * cm_jogging_cycle_callback()  - main loop callback for running the jogging cycle
 *  _set_jogging_func()         - a convenience for setting the next dispatch vector and exiting
 *  _jogging_axis_start()       - setup the jog
 *  _jogging_axis_ramp_jog()    - ramp the jog
 *  _jogging_axis_move()        - move the axis
 *  _jogging_finalize_exit()    - clean up
 */

stat_t cm_jogging_cycle_callback(void) {
    if (cm.cycle_state != CYCLE_JOG) {
        return (STAT_NOOP);  // exit if not in a jogging cycle
    }
    if (jog.func == _jogging_finalize_exit && cm_get_runtime_busy() == true) {
        return (STAT_EAGAIN);  // sync to planner move ends
    }
    //    if (jog.func == _jogging_axis_ramp_jog && mp_get_buffers_available() < PLANNER_BUFFER_HEADROOM) {
    if (jog.func == _jogging_axis_ramp_jog && mp_planner_is_full()) {
        return (STAT_EAGAIN);  // prevent flooding the queue with jog moves
    }
    return (jog.func(jog.axis));  // execute the current jogging move
}

static stat_t _set_jogging_func(stat_t (*func)(int8_t axis)) {
    jog.func = func;
    return (STAT_EAGAIN);
}

static stat_t _jogging_axis_start(int8_t axis) {
    //    cm_end_hold();                                          // ends hold if one is in effect
    return (_set_jogging_func(_jogging_axis_ramp_jog));
}

#define INITIAL_RAMP 0.01
#define RAMP_DIST 2.0
#define MAX_STEPS 25

static stat_t _jogging_axis_ramp_jog(int8_t axis)  // run the jog ramp
{
    float   direction = jog.start_pos <= jog.dest_pos ? 1. : -1.;
    float   delta     = fabs(jog.dest_pos - jog.start_pos);
    uint8_t last      = 0;

    float velocity =
        jog.velocity_start + (jog.velocity_max - jog.velocity_start) * pow(10.0, (jog.step / ((float)MAX_STEPS)) - 1.0);
    float offset = INITIAL_RAMP + RAMP_DIST * ((jog.step * (jog.step + 1.0)) / (2.0 * MAX_STEPS));
    if (offset >= delta || jog.step >= MAX_STEPS) {
        offset = delta;
        last   = 1;
    }
    float target = jog.start_pos + offset * direction;

    _jogging_axis_move(axis, target, velocity);
    jog.step++;

    if (last) {
        return (_set_jogging_func(_jogging_finalize_exit));
    } else {
        return (_set_jogging_func(_jogging_axis_ramp_jog));
    }
}

static stat_t _jogging_axis_move(int8_t axis, float target, float velocity) {
    float vect[]  = {0, 0, 0, 0, 0, 0};
    bool  flags[] = {false, false, false, false, false, false};

    vect[axis]  = target;
    flags[axis] = true;
    cm_set_feed_rate(velocity);
    ritorno(cm_straight_feed(vect, flags));
    return (STAT_EAGAIN);
}

static stat_t _jogging_finalize_exit(int8_t axis)  // finish a jog
{
    //    cm_end_hold();                                // ends hold if one is in effect

    cm_set_coord_system(jog.saved_coord_system);  // restore to work coordinate system
    cm_set_units_mode(jog.saved_units_mode);
    cm_set_distance_mode(jog.saved_distance_mode);
    cm_set_feed_rate_mode(jog.saved_feed_rate_mode);
    (MODEL)->feed_rate = jog.saved_feed_rate;
    cm_set_motion_mode(MODEL, MOTION_MODE_CANCEL_MOTION_MODE);
    cm_canned_cycle_end();
    xio_writeline("{\"jog\":0}\n");  // needed by OMC jogging function
    return (STAT_OK);
}

/*
static stat_t _jogging_error_exit(int8_t axis)
{
        // Generate the warning message. Since the error exit returns via the jogging callback
        // - and not the main controller - it requires its own display processing
//    nv_reset_nv_list();
        _jogging_finalize_exit(axis);                    // clean up
        return (STAT_JOGGING_CYCLE_FAILED);                // jogging state
}
*/
