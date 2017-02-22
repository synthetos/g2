/*
 * cycle_feedhold.cpp - canonical machine feedhold processing
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2017 Alden S Hart, Jr.
 * Copyright (c) 2014 - 2017 Robert Giseburt
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

#include "g2core.h"     // #1
#include "config.h"     // #2
#include "gcode.h"      // #3
#include "canonical_machine.h"
#include "planner.h"
#include "plan_arc.h"
#include "stepper.h"
#include "spindle.h"
#include "coolant.h"
#include "util.h"
//#include "xio.h"        // DIAGNOSTIC

static stat_t _run_p1_hold_entry_actions(void);
static void   _sync_to_p1_hold_entry_actions_done(float* vect, bool* flag);

static stat_t _run_p1_hold_exit_actions(void);
static void   _sync_to_p1_hold_exit_actions_done(float* vect, bool* flag);
static stat_t _finalize_p1_hold_exit(void);
static stat_t _finalize_p2_hold_exit(void);

/***********************************************************************************
 **** Feedholds ********************************************************************
 ***********************************************************************************/
/*
 *  Feedholds, queue flushes and end_holds are all related and are in this file.
 *  Feedholds are implemented as a state machine (cmFeedholdState) that runs in the 
 *  planner (plan_exec.cpp, see Feedhold Processing around line 500) and in this file.
 *
 *  Feedholds also use the dual planner (secondary context, or hold context) where a
 *  new canonical machine and planner are spun up and entered when a feedhold is initiated. 
 *  This allows (almost) all of the machine operations to be accessible during a feedhold.
 *
 *  Feedholds are initiated and ended by a series of request flags (requests).
 *  The request functions set flags or change state to "REQUESTED". 
 *  The sequencing callback interprets the flags as so:
 *    - A feedhold request received during motion should be honored
 *    - A feedhold request received during a feedhold should be ignored
 *    - A feedhold request received during a motion stop should be ignored
 *
 *    - A queue flush request should only be honored while in a feedhold
 *    - Said queue flush request received during a feedhold should be deferred until
 *      the feedhold enters a HOLD state (i.e. until deceleration is complete and motors stop).
 *    - A queue flush request received during a motion stop should be honored
 *
 *    - An end_hold (cycle start) request should only be honored while in a feedhold
 *    - Said end_hold request received during a feedhold should be deferred until the
 *      feedhold enters a HOLD state (i.e. until deceleration is complete).
 *      If a queue flush request is also present the queue flush should be done first
 */
/*  Below the request level, feedholds work like this:
 *    - The hold is initiated by calling cm_start_hold(). cm->hold_state is set to
 *      FEEDHOLD_SYNC, motion_state is set to MOTION_HOLD, and the spindle is turned off
 *      (if it it on). The remainder of feedhold
 *      processing occurs in plan_exec.c in the mp_exec_aline() function.
 *
 *      - MOTION_HOLD and FEEDHOLD_SYNC tells mp_exec_aline() to begin feedhold processing
 *      after the current move segment is finished (< 5 ms later). (Cases handled by
 *      feedhold processing are listed in plan_exec.c).
 *
 *    - FEEDHOLD_SYNC causes the current move in mr to be replanned into a deceleration.
 *      If the distance remaining in the executing move is sufficient for a full deceleration
 *      then motion will stop in the current block. Otherwise the deceleration phase
 *      will extend across as many blocks necessary until one will stop.
 *
 *    - Once deceleration is complete hold state transitions to FEEDHOLD_FINALIZING and 
 *      the distance remaining in the bf last block is replanned up from zero velocity.
 *      The move in the bf block is NOT released (unlike normal operation), as it will
 *      be used again to restart from hold.
 *
 *    - When cm_end_hold() is called it releases the hold, restarts the move and restarts
 *      the spindle if the spindle is active.
 */

/* With the addition of the secondary CM, feedhold state management gets tricky.
   What you see below is a temporary solution until we decide the general solution.
   
   The general solution causes a feedhold from the primary context to switch 
   into the secondary context to perform feedhold actions. When in the secondary
   context an additional feedhold will perform the usual STOP operation, but 
   will remain in the secondary context, and therefore not perform any feedhold
   actions (lifts, spindle, etc.). This is needed to support homing and probing 
   operations from within the secondary context.
   
   Oddities of the general solution:
      - Should we allow a feedhold to be performed if the tool is not moving? 
        Right now we don't, but with the secondary context this might be useful
   
    What you see here is a Q&D to only allow feedholds from the primary context. 
    It has the following limitations:
      - Feedhold requests are only honored form the primary context
      - Queue flush requests are only honored form the primary context
      - Machine alarm state is not (yet) taken into account in feedhold sequencing and restart
 */

/***********************************************************************************
 * cm_has_hold()   - return true if a hold condition exists (or a pending hold request)
 * cm_start_hold() - start a feedhhold external to feedhold request & sequencing
 *
 *  It's OK to call start_hold directly in order to get a hold quickly (see gpio.cpp)
 */
bool cm_has_hold()
{
    return (cm1.hold_state != FEEDHOLD_OFF);
}

void cm_start_hold()
{
    // Can only request a feedhold if the machine is in motion and there not one is not already in progress 
    if ((cm1.hold_state == FEEDHOLD_OFF) && (mp_has_runnable_buffer(mp))) {
        cm_set_motion_state(MOTION_HOLD);
        cm1.hold_state = FEEDHOLD_SYNC;   // invokes hold from aline execution
    }
}

/***********************************************************************************
 * cm_request_feedhold()
 * cm_request_exit_hold()
 * cm_request_queue_flush()
 *
 *  p1 is the primary planner, p2 is the secondary planner, which is active if the 
 *  primary planner is in hold. IOW p2 can only be in a hold if p1 is already in one.
 *  Request_feedhold, request_end_hold, and request_queue_flush are contextual:
 * 
 *  request_feedhold:
 *    - If p1 is not in HOLD & is in motion, request_feedhold requests a p1 hold
 *    - If p1 is in HOLD & p2 is in motion, request_feedhold requests a p2 hold
 *    - If both p1 and p2 are in HOLD, request_feedhold is ignored
 *
 *  request_end_hold:
 *    - If p1 is not in HOLD, request_end_hold is ignored
 *    - If p1 is in HOLD request_end_hold will end p1 hold & resume motion.
 *      Pre-defined exit actions (coolant, spindle, Z move) are completed first
 *      Any executing or pending "in-hold" moves are stopped prior to the exit actions
 *
 *  request_queue_flush:
 *    - If p1 is not in HOLD, request_queue_flush is ignored
 *    - If p1 is in HOLD request_queue_flush will end p1 hold & queue flush (stop motion).
 *      Pre-defined exit actions (coolant, spindle, Z move) are completed first
 *      Any executing or pending "in-hold" moves are stopped prior to the exit actions
 */

void cm_request_feedhold(void)  // !
{
    // Only generate request if not already in a feedhold and the machine is in motion    
    if ((cm1.hold_state == FEEDHOLD_OFF) && (cm1.motion_state != MOTION_STOP)) {
        cm1.hold_state = FEEDHOLD_REQUESTED;
    } else 
    if ((cm2.hold_state == FEEDHOLD_OFF) && (cm2.motion_state != MOTION_STOP)) {
        cm2.hold_state = FEEDHOLD_REQUESTED;
    }
}

void cm_request_exit_hold(void)  // ~
{
    if (cm1.hold_state != FEEDHOLD_OFF) {
        cm1.hold_exit_requested = true;
    }
}

void cm_request_queue_flush()   // %
{
    // NOTE: this function used to flush input buffers, but this is handled in xio *prior* to queue flush now
    if ((cm1.hold_state != FEEDHOLD_OFF) &&         // don't honor request unless you are in a feedhold
        (cm1.flush_state == FLUSH_OFF)) {           // ...and only once
        cm1.flush_state = FLUSH_REQUESTED;          // request planner flush once motion has stopped
    }
}

/***********************************************************************************
 * cm_feedhold_sequencing_callback() - sequence feedhold, queue_flush, and end_hold requests
 *
 * Expected behaviors:           (no-hold means machine is not in hold, etc)
 *
 *  (no-cycle) !    No action. Feedhold is not run (nothing to hold!)
 *  (no-hold)  ~    No action. Cannot exit a feedhold that does not exist
 *  (no-hold)  %    No action. Queue flush is not honored except during a feedhold
 *  (in-cycle) !    Start a feedhold
 *  (in-hold)  ~    Wait for feedhold actions to complete, exit feedhold, resume motion 
 *  (in-hold)  %    Wait for feedhold actions to complete, exit feedhold, do not resume motion
 *  (in-cycle) !~   Start a feedhold, do enter and exit actions, exit feedhold, resume motion
 *  (in-cycle) !%   Start a feedhold, do enter and exit actions, exit feedhold, do not resume motion
 *  (in-cycle) !%~  Same as above
 *  (in-cycle) !~%  Same as above (this one's an anomaly, but the intent would be to Q flush)
 */

stat_t cm_feedhold_command_blocker()
{
    if (cm1.hold_state != FEEDHOLD_OFF) {
        return (STAT_EAGAIN);
    }
    return (STAT_OK);
}

stat_t cm_feedhold_sequencing_callback()
{
    // invoking a p1 feedhold is a 2 step process - get to the stop, then execute the hold actions
    if (cm1.hold_state == FEEDHOLD_REQUESTED) {
        if (mp_has_runnable_buffer(&mp1)) {         // bypass cm_start_hold() to start from here
            cm_set_motion_state(MOTION_HOLD);
            cm1.hold_state = FEEDHOLD_SYNC;         // invokes hold from aline execution
        }
    }
    if (cm1.hold_state == FEEDHOLD_ACTIONS_START) { // perform Z lift, spindle & coolant actions
        _run_p1_hold_entry_actions();
    }

    // p2 feedhold states - feedhold in feedhold
    if (cm2.hold_state == FEEDHOLD_REQUESTED) {
        if (mp_has_runnable_buffer(&mp2)) {
            cm_set_motion_state(MOTION_HOLD);
            cm2.hold_state = FEEDHOLD_SYNC;
        }
    }
    if (cm2.hold_state == FEEDHOLD_P2_EXIT) {
        return(_finalize_p2_hold_exit());
    }

    // queue flush won't run until the hold is complete and all (subsequent) motion has stopped
    if ((cm1.flush_state == FLUSH_REQUESTED) && (cm1.hold_state == FEEDHOLD_HOLD) &&
        (mp_runtime_is_idle())) {                   // don't flush planner during movement
            cm_queue_flush(&cm1);
            cm1.hold_exit_requested = true;         // p1 queue flush always ends the hold
            qr_request_queue_report(0);             // request a queue report, since we've changed the number of buffers available
    }

    // exit_hold runs for both ~ and % feedhold ends
    if (cm1.hold_exit_requested) {
        
        // Flush must complete before exit_hold runs. Trap possible race condition if flush request was
        if (cm1.flush_state == FLUSH_REQUESTED) {   // ...received when this callback was running
            return (STAT_OK);
        } 
        if (cm1.hold_state == FEEDHOLD_HOLD) {      // don't run end_hold until fully into a hold
            cm1.hold_exit_requested = false;
            _run_p1_hold_exit_actions();            // runs once only
        }       
    }
    if (cm1.hold_state == FEEDHOLD_P1_EXIT) {
        return(_finalize_p1_hold_exit());           // run multiple times until actions are complete
    }
    return (STAT_OK);
}
    
/***********************************************************************************
 * _run_p1_hold_entry_actions()          - run actions in p2 that complete the p1 hold
 * _sync_to_p1_hold_entry_actions_done() - final state change occurs here
 *
 *  This function assumes that the feedhold sequencing callback has resolved all 
 *  state and timing issues and it's OK to call this now. Do not call this function
 *  directly. Always use the feedhold sequencing callback.
 *
 *  Moving between planners is only safe when the machine is completely stopped.
 *
 *  _sync_to_p1_hold_entry_actions_done() is a callback to run when the ACTIONS from 
 *  feedhold in p1 are finished. This function hits cm1 directly as ACTIONS for a 
 *  feedhold in p1 actually run in the secondary planner (p2). Feedholds from p2 do 
 *  not run actions, so this function is never called for p2 feedholds. It's called 
 *  from an interrupt, so it only sets a flag.
 */
static void _sync_to_p1_hold_entry_actions_done(float* vect, bool* flag)
{
    cm1.hold_state = FEEDHOLD_HOLD;
    sr_request_status_report(SR_REQUEST_IMMEDIATE);
}

static stat_t _run_p1_hold_entry_actions()
{    
    cm->hold_state = FEEDHOLD_ACTIONS_WAIT;   // penultimate state before transitioning to HOLD
    
    debug_trap_if_true(st_runtime_isbusy(), "_run_p1_hold_entry_actions() - runtime is busy");
    
    // copy the primary canonical machine to the secondary, 
    // fix the planner pointer, and reset the secondary planner
    memcpy(&cm2, &cm1, sizeof(cmMachine_t));
    cm2.mp = &mp2;
    planner_reset((mpPlanner_t *)cm2.mp);   // mp is a void pointer

    // set parameters in cm, gm and gmx so you can actually use it
    cm2.hold_state = FEEDHOLD_OFF;
    cm2.gm.motion_mode = MOTION_MODE_CANCEL_MOTION_MODE;
    cm2.gm.absolute_override = ABSOLUTE_OVERRIDE_OFF;
    cm2.flush_state = FLUSH_OFF;
    cm2.gm.feed_rate = 0;

    // clear the target and set the positions to the current hold position
    memset(&(cm2.gm.target), 0, sizeof(cm2.gm.target));
    memset(&(cm2.return_flags), 0, sizeof(cm2.return_flags));
    copy_vector(cm2.gm.target_comp, cm1.gm.target_comp); // preserve original Kahan compensation
    copy_vector(cm2.gmx.position, mr1.position);
    copy_vector(mp2.position, mr1.position);
    copy_vector(mr2.position, mr1.position);

    // reassign the globals to the secondary CM
    cm = &cm2;
    mp = (mpPlanner_t *)cm->mp;     // mp is a void pointer
    mr = mp->mr;

    // set motion state and ACTIVE_MODEL. This must be performed after cm is set to cm2
    cm_set_g30_position();
    cm_set_motion_state(MOTION_STOP);   // sets cm2 active model to MODEL

    // execute feedhold actions
    if (fp_NOT_ZERO(cm->feedhold_z_lift)) {                 // optional Z lift
        cm_set_distance_mode(INCREMENTAL_DISTANCE_MODE);
        bool flags[] = { 0,0,1,0,0,0 };            
        float target[] = { 0,0, _to_inches(cm->feedhold_z_lift), 0,0,0 };   // convert to inches if in inches mode
        cm_straight_traverse(target, flags);
        cm_set_distance_mode(cm1.gm.distance_mode);         // restore distance mode to p1 setting
    }
    spindle_control_sync(SPINDLE_PAUSE);                    // optional spindle pause
    coolant_control_sync(COOLANT_PAUSE, COOLANT_BOTH);      // optional coolant pause    
    mp_queue_command(_sync_to_p1_hold_entry_actions_done, nullptr, nullptr);
    return (STAT_OK);
}

/***********************************************************************************
 *  _run_p1_hold_exit_actions()          - initiate return from feedhold planner
 *  _sync_to_p1_hold_exit_actions_done() - callback to sync to end of planner operations
 *  _finalize_p1_hold_exit()             - callback to finsh return once moves are done 
 *
 *  These functions assume that the feedhold sequencing callback has resolved all
 *  state and timing issues and it's OK to call this now. Do not call this function
 *  directly. Always use the feedhold sequencing callback.
 *
 *  The finalization moves are performed in _sync_to_p1_hold_exit_actions_done() because 
 *  the sync runs from an interrupt. Finalization needs to run from the main loop.
 */

static stat_t _run_p1_hold_exit_actions()     // LATER: if value == true return with offset corrections
{
    // perform end-hold actions --- while still in secondary machine
    coolant_control_sync(COOLANT_RESUME, COOLANT_BOTH); // resume coolant if paused
    spindle_control_sync(SPINDLE_RESUME);               // resume spindle if paused
    
    // do return move though an intermediate point; queue a wait
    cm2.return_flags[AXIS_Z] = false;
    cm_goto_g30_position(cm2.gmx.g30_position, cm2.return_flags);         
    mp_queue_command(_sync_to_p1_hold_exit_actions_done, nullptr, nullptr);
    return (STAT_OK);
}

// Callback to run when the G30 return move is finished. This function is only ever
// called by the secondary planner, and only when exiting a feedhold from planner 1. 
// It's called from an interrupt, so it only sets a flag.

static void _sync_to_p1_hold_exit_actions_done(float* vect, bool* flag)
{
    cm1.hold_state = FEEDHOLD_P1_EXIT;      // penultimate state before transitioning to FEEDHOLD_OFF
    sr_request_status_report(SR_REQUEST_IMMEDIATE);
}

static stat_t _finalize_p1_hold_exit()
{
    // skip out if not ready to finalize the exit
    if (cm1.hold_state != FEEDHOLD_P1_EXIT) {
        return (STAT_NOOP);                 // ??? return (STAT_EAGAIN);
    }
    
    // return to primary planner (p1)
    cm = &cm1;
    mp = (mpPlanner_t *)cm->mp;             // cm->mp is a void pointer
    mr = mp->mr;

    // execute this block if a queue flush was performed
    // adjust p1 planner positions to runtime positions
    if (cm1.flush_state == FLUSH_WAS_RUN) {
        cm_reset_position_to_absolute_position(cm);
        cm1.flush_state = FLUSH_OFF;
    }

    // resume motion from primary planner or end cycle if no moves in planner
    cm1.hold_state = FEEDHOLD_OFF;
    if (mp_has_runnable_buffer(&mp1)) {
        cm_set_motion_state(MOTION_RUN);
        cm_cycle_start();
        st_request_exec_move();
    } else {
        cm_set_motion_state(MOTION_STOP);
        cm_cycle_end();
    }
    return (STAT_OK);
}

/***********************************************************************************
 * _finalize_p2_hold_exit()
 */

static stat_t _finalize_p2_hold_exit()
{
    float position[AXES];
    copy_vector(position, mr2.position);            // save the final position
    cm_queue_flush(&cm2);
    copy_vector(mr2.position, position);            // restore the final position
    cm_reset_position_to_absolute_position(&cm2);   // propagate position 

    cm2.hold_state = FEEDHOLD_OFF;
    cm_set_motion_state(MOTION_STOP);
    cm_cycle_end();
    sr_request_status_report(SR_REQUEST_IMMEDIATE);
    return (STAT_OK);
}

/***********************************************************************************
 * Queue Flush operations
 *
 * This one's complicated. See here first:
 * https://github.com/synthetos/g2/wiki/Job-Exception-Handling
 * https://github.com/synthetos/g2/wiki/Alarm-Processing
 *
 * We want to use queue flush for a few different use cases, as per the above wiki pages.
 * The % behavior implements Exception Handling cases 1 and 2 - Stop a Single Move and
 * Stop Multiple Moves. This is complicated further by the processing in single USB and
 * dual USB being different. Also, the state handling is located in xio.cpp / readline(),
 * controller.cpp _dispatch_kernel() and cm_request_queue_flush(), below.
 * So it's documented here.
 *
 * Single or Dual USB Channels:
 *  - If a % is received outside of a feed hold or ALARM state, ignore it.
 *      Change the % to a ; comment symbol (xio)
 *
 * Single USB Channel Operation:
 *  - Enter a feedhold (!)
 *  - Receive a queue flush (%) Both dispatch it and store a marker (ACK) in the input
 *      buffer in place of the the % (xio)
 *  - Execute the feedhold to a hold condition (plan_exec)
 *  - Execute the dispatched % to flush queues (canonical_machine)
 *  - Silently reject any commands up to the % in the input queue (controller)
 *  - When ETX is encountered transition to STOP state (controller/canonical_machine)
 *
 * Dual USB Channel Operation:
 *  - Same as above except that we expect the % to arrive on the control channel
 *  - The system will read and dump all commands in the data channel until either a
 *    clear is encountered ({clear:n} or $clear), or an ETX is encountered on either
 *    channel, but it really should be on the data channel to ensure all queued commands
 *    are dumped. It is the host's responsibility to both write the clear (or ETX), and
 *    to ensure that it either arrives on the data channel or that the data channel is
 *    empty before writing it to the control channel.
 */

/***********************************************************************************
 * cm_queue_flush() - Flush planner queue
 *
 *  This function assumes that the feedhold sequencing callback has resolved all 
 *  state and timing issues and it's OK to call this now. Do not call this function
 *  directly. Always use the feedhold sequencing callback.
 */

void cm_queue_flush(cmMachine_t *_cm)
{
    cm_abort_arc(_cm);                      // kill arcs so they don't just create more alines
    planner_reset((mpPlanner_t *)_cm->mp);  // reset primary planner. also resets the mr under the planner
    _cm->flush_state = FLUSH_WAS_RUN;
}
