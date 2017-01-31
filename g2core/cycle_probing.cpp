/*
 * cycle_probing.c - probing cycle extension to canonical_machine.c
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2017 Alden S Hart, Jr., Sarah Tappon, Tom Cauchois, Robert Giseburt
 * With contributions from Other Machine Company.
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
#include "kinematics.h"
#include "encoder.h"
#include "spindle.h"
#include "report.h"
#include "gpio.h"
#include "planner.h"
#include "util.h"
#include "xio.h"

/**** Probe singleton structure ****/

#define MINIMUM_PROBE_TRAVEL 0.254

struct pbProbingSingleton {             // persistent probing runtime variables
    bool wait_for_motion_end;           // flag to use to now when the motion has ended
    bool alarm_if_fail;                 // flag for G38.2 and G38.4, where failure is NOT an option
    bool move_toward_contact;           // flag for G38.4 and G38.5, where we move off of the contact

    stat_t (*func)();                   // binding for callback function state machine

    // controls for probing cycle
    int8_t probe_input;                 // which input should we check?

    // state saved from gcode model
    cmDistanceMode saved_distance_mode; // G90,G91 global setting
    cmCoordSystem saved_coord_system;   // G54 - G59 setting
    bool saved_soft_limit_enable;       // turn off soft limits during probing
    float saved_jerk[AXES];             // saved and restored for each axis

    // probe destination
    float target[AXES];
    bool  flags[AXES];
};
static struct pbProbingSingleton pb;

/**** NOTE: global prototypes and other .h info is located in canonical_machine.h ****/

static stat_t _probing_start();
static stat_t _probing_backoff();
static stat_t _probing_exit();
static stat_t _probing_error_exit(stat_t status);
static stat_t _probe_axis_move(const float target[], bool exact_position);

// helper

static void _motion_end_callback(float* vect, bool* flag)
{
    pb.wait_for_motion_end = false;
}

/***********************************************************************************
 **** G38.x Probing Cycle ***********************************************************
 ***********************************************************************************/

/****************************************************************************************
 * cm_probing_cycle_start()    - G38.x probing cycle using contact (digital input)
 * cm_probing_cycle_callback() - main loop callback for running the probing cycle
 *
 *  All cm_probe_cycle_start does is prevent any new commands from queueing to the
 *  planner so that the planner can move to a stop and report MACHINE_PROGRAM_STOP.
 *  OK, it also queues the function that's called once motion has stopped.
 *
 *  Note: When coding a cycle (like this one) you get to perform one queued
 *  move per entry into the continuation, then you must exit. We put two buffer
 *  items into the queue: We queue a move, then we queue a "command" that simply
 *  sets a flag in the probing object (pb.waiting_for_motion_end) to tell us that
 *  the move has finished. The runtime has a special exception for probing and
 *  homing where if a move is interrupted it clears it out of the queue.
 *
 *  --- Some further details ---
 *  Starting from the definition of G38.x from the LinuxCNC docs:
 *  http://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G38-probe
 *
 *  Once we are past the starting conditions for the probe to succeed as listed
 *  in the LinuxCNC documentation, we will then execute the move. After the move
 *  we interpret "success" as the probe value changing in the correct direction,
 *  and "failure" as it not changing. IOW, the move can finish and the probe input 
 *  not change, which we consider to be a failure.
 *
 *  Taking polarity of the probe input into account to give a value of Active or
 *  Inactive, for G38.2 and G38.3, success requires going from Inactive to Active,
 *  and for G38.4 and G38.5 success requires an edge from Inactive to Active.
 *
 *  For G38.2 and G38.4 we also put the machine into an ALARM state if the probing
 *  "fails".
 *
 *  When the probe input fires, the input interrupt takes a snapshot of the internal
 *  encoders, then requests a "high speed" feedhold. We then run forward kinematics
 *  on the encoder snapshot to get the reported position. We also execute a move
 *  from the final position (after the feedhold) back to the point we report.
 *
 *  Additionally, we record the last PROBES_STORED (at least 3) probe points that
 *  succeeded. The current or most recent probe (be it success, failure, or
 *  in-progress) occupies one of those positions, which is the one reported by the
 *  "prb" JSON.
 *
 *  Internally we store the active/most recent probe in cm.probe_results[0] and
 *  cm.probe_state[0]. Before we start a new probe, if cm.probe_state[0] ==
 *  PROBE_SUCCEEDED, then we roll 0 to 1, and 1 to 2, up to PROBES_STORED-1.
 *  The oldest probe is "lost."
 *
 *  Note: Spindle and coolant are not affected during probing. Some probes require 
 *  the spindle to be turned on.
 */

uint8_t cm_straight_probe(float target[], bool flags[], bool alarm_if_fail, bool move_toward_contact) 
{
    // error if zero feed rate
    if (fp_ZERO(cm.gm.feed_rate)) {
        return (STAT_GCODE_FEEDRATE_NOT_SPECIFIED);
    }

    // error if no axes specified
    if (!(flags[AXIS_X] | flags[AXIS_Y] | flags[AXIS_Z] |
          flags[AXIS_A] | flags[AXIS_B] | flags[AXIS_C])) {
        return (STAT_GCODE_AXIS_IS_MISSING);
    }

    // initialize the probe input; error if no probe input specified
    if ((pb.probe_input = gpio_get_probing_input()) == -1) {
        return (STAT_NO_PROBE_INPUT_CONFIGURED);
    }

    // setup
    pb.alarm_if_fail = alarm_if_fail;
    pb.move_toward_contact = move_toward_contact;
    pb.func = _probing_start;               // bind probing start function

    cm_set_model_target(target, flags);     // set probe move endpoint taking all offsets into account
    copy_vector(pb.target, cm.gm.target);
    copy_vector(pb.flags, flags);           // set axes involved on the move

    // if the previous probe succeeded, roll probes to the next position
    if (cm.probe_state[0] == PROBE_SUCCEEDED) {
        for (uint8_t n = PROBES_STORED - 1; n > 0; n--) {
            cm.probe_state[n] = cm.probe_state[n - 1];
            for (uint8_t axis = 0; axis < AXES; axis++) { 
                cm.probe_results[n][axis] = cm.probe_results[n - 1][axis]; 
            }
        }
    }
    // clear the old probe position
    clear_vector(cm.probe_results[0]);      // NOTE: relying on cm.probe_results will not detect a probe to 0,0,0.

    // queue a function to let us know when we can start probing
    cm.probe_state[0] = PROBE_WAITING;      // wait until planner queue empties before completing initialization
    pb.wait_for_motion_end = true;
    mp_queue_command(_motion_end_callback, nullptr, nullptr);  // note: these args are ignored
    return (STAT_OK);
} 

/*
 *  cm_probing_cycle_callback() - handle probing progress
 *
 *  This is called regularly from the controller. If we report NOOP, the controller 
 *  will continue with other tasks. Otherwise the controller will not execute any 
 *  later tasks, including read any more "data".
 *
 *  Note: When coding a cycle (like this one) you must wait until the last move has
 *  actually been queued (or has finished) before declaring the cycle to be done. 
 *  Otherwise there is a nasty race condition in _controller_HSM() that may accept 
 *  the next command before the position of the final move has been recorded in the 
 *  Gcode model. That's what the wait_for_motion_end callback is about.
 */

uint8_t cm_probing_cycle_callback(void) 
{
    if ((cm.cycle_state != CYCLE_PROBE) && (cm.probe_state[0] != PROBE_WAITING)) {  // exit if not in a probing cycle
        return (STAT_NOOP);
    }
    if (pb.wait_for_motion_end) {  // sync to planner move ends (using callback)
        return (STAT_EAGAIN);
    }
    return (pb.func());  // execute the current probing move
}

/*
 * _probing_start() - start the probe or skip it if contact is already active
 */

static uint8_t _probing_start() 
{
    float start_position[AXES];

    // Initializations. These initializations are required before starting the probing cycle
    // but must be done after the planner has exhausted all current CYCLE moves as
    // they affect the runtime (specifically the digital input modes). Side effects would
    // include limit switches initiating probe actions instead of just killing movement
    
    // so optimistic... ;)
    // NOTE: it is *not* an error condition for the probe not to trigger.
    // it is an error for the limit or homing switches to fire, or for some other configuration error.
    cm.probe_state[0] = PROBE_FAILED;
    cm.machine_state = MACHINE_CYCLE;
    cm.cycle_state = CYCLE_PROBE;

    // save relevant non-axis parameters from Gcode model
    pb.saved_coord_system = (cmCoordSystem)cm_get_coord_system(ACTIVE_MODEL);
    pb.saved_distance_mode = (cmDistanceMode)cm_get_distance_mode(ACTIVE_MODEL);
    pb.saved_soft_limit_enable = cm.soft_limit_enable;
    
    // set working values
    cm_set_distance_mode(ABSOLUTE_DISTANCE_MODE);
    cm_set_coord_system(ABSOLUTE_COORDS);  // probing is done in machine coordinates

    // initialize the axes - save the jerk settings & change to the high-speed jerk settings
    for (uint8_t axis = 0; axis < AXES; axis++) {
        pb.saved_jerk[axis] = cm_get_axis_jerk(axis);  // save the max jerk value
        cm_set_axis_jerk(axis, cm.a[axis].jerk_high);  // use the high-speed jerk for probe
        start_position[axis] = cm_get_absolute_position(ACTIVE_MODEL, axis);
    }

    // error if the probe target is too close to the current position
    if (get_axis_vector_length(start_position, pb.target) < MINIMUM_PROBE_TRAVEL) {
        return(_probing_error_exit(STAT_PROBE_TRAVEL_TOO_SMALL));
    }

    gpio_set_probing_mode(pb.probe_input, true);

    // initial probe state, don't probe if we're already contacted!
    // INPUT_INACTIVE (false) is the correct start condition for G38.2 and G38.3
    // INPUT_ACTIVE (true) is the right start condition for G38.4 and G38.5
    // Note that we're testing for SUCCESS here
    int8_t probe_input = gpio_read_input(pb.probe_input);   // returns `true` if contacted
    
    if (pb.move_toward_contact == probe_input) {            // exclusive nor for booleans
        return(_probing_error_exit(STAT_PROBE_HAS_INVALID_START_STATE));
    }
    
    if (probe_input == (pb.move_toward_contact ? INPUT_INACTIVE : INPUT_ACTIVE)) {
        _probe_axis_move(pb.target, false);
        pb.func = _probing_backoff;
        return (STAT_EAGAIN);
    }

    cm.probe_state[0] = PROBE_FAILED;               // we failed
    pb.func = _probing_exit;
    return (STAT_EAGAIN);
}

/*
 * _probing_backoff() - runs after the probe move, whether it contacted or not
 *
 * Back off to the measured touch position captured by encoder snapshot
 */

static stat_t _probing_backoff() 
{    
    // Test if we've contacted
    // INPUT_INACTIVE (false) is the correct start condition for G38.2 and G38.3
    // INPUT_ACTIVE (true) is the right start condition for G38.4 and G38.5
    // Note that we're testing for SUCCESS here
    int8_t probe = gpio_read_input(pb.probe_input);
    if (probe == (pb.move_toward_contact ? INPUT_ACTIVE : INPUT_INACTIVE)) {
        cm.probe_state[0] = PROBE_SUCCEEDED;

        // capture contact position in step space and convert from steps to mm.
        // snapshot was taken by input interrupt at the time of closure
        float contact_position[AXES];
        kn_forward_kinematics(en_get_encoder_snapshot_vector(), contact_position);

        _probe_axis_move(contact_position, true);   // NB: feed rate is the same as the probe move
    } else {
        cm.probe_state[0] = PROBE_FAILED;
    }
    pb.func = _probing_exit;
    return (STAT_EAGAIN);
}

/*
 * _probe_axis_move() - function to execute probing moves
 */

static stat_t _probe_axis_move(const float target[], bool exact_position) 
{
    auto stored_units_mode    = cm.gm.units_mode;
    auto stored_distance_mode = cm.gm.distance_mode;
    if (exact_position) {
        cm_set_absolute_override(MODEL, ABSOLUTE_OVERRIDE_ON);  // Position was stored in absolute coords
        cm.gm.units_mode    = MILLIMETERS;
        cm.gm.distance_mode = ABSOLUTE_DISTANCE_MODE;
    }

    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {  // set all positions
        cm_set_position(axis, mp_get_runtime_absolute_position(axis));
    }

    // set this BEFORE the motion starts
    pb.wait_for_motion_end = true;

    cm_straight_feed(target, pb.flags);

    if (exact_position) {
        cm.gm.units_mode    = stored_units_mode;
        cm.gm.distance_mode = stored_distance_mode;
    }
    mp_queue_command(_motion_end_callback, nullptr, nullptr);      // the last two arguments are ignored anyway
    return (STAT_EAGAIN);
}

/*
 * _probe_restore_settings() - helper for both exits
 * _probing_exit() - clean up and exit
 * _probing_error_exit()
 */

static void _probe_restore_settings() 
{    
    gpio_set_probing_mode(pb.probe_input, false);       // set input back to normal operation

    for (uint8_t axis = 0; axis < AXES; axis++) {       // restore axis jerks
        cm.a[axis].jerk_max = pb.saved_jerk[axis]; 
    }
    cm_set_coord_system(pb.saved_coord_system);         // restore coordinate system...
    cm_set_distance_mode(pb.saved_distance_mode);       // ...distance mode...
    cm.soft_limit_enable = pb.saved_soft_limit_enable;  // ...and soft limit setting

    cm_set_motion_mode(MODEL, MOTION_MODE_CANCEL_MOTION_MODE); // cancel feed modes used during probing
    sr_request_status_report(SR_REQUEST_IMMEDIATE);     // request SR for success or failure
    cm_canned_cycle_end();
}

static stat_t _probing_exit()
{
    _probe_restore_settings();          // cleanup first

    // set absolute position in probe results vector
    for (uint8_t axis = 0; axis < AXES; axis++) {
        cm.probe_results[0][axis] = cm_get_absolute_position(ACTIVE_MODEL, axis);
    }
    if (cm.probe_state[0] == PROBE_SUCCEEDED) {
        return (STAT_OK);
    }

    // handle failure cases
    if (pb.alarm_if_fail) {
        cm_alarm(STAT_PROBE_CYCLE_FAILED, "probing error");
    }
    return (STAT_OK);
}

static stat_t _probing_error_exit(stat_t status) 
{
    _probe_restore_settings();          // cleanup first

    if (pb.alarm_if_fail) {             // generate an alarm
        char message[16] = { "probe error" };
        cm_alarm(status, message);
    }
    return (status);
}
