/*
 * cycle_feedhold.cpp - canonical machine feedhold processing
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2018 Alden S Hart, Jr.
 * Copyright (c) 2014 - 2018 Robert Giseburt
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

//static void _start_feedhold(void);
static void _start_cycle_restart(void);
static void _start_queue_flush(void);
static void _start_job_kill(void);

// Feedhold actions
static stat_t _feedhold_skip(void);
static stat_t _feedhold_no_actions(void);
static stat_t _feedhold_with_actions(void);
static stat_t _feedhold_restart_with_actions(void);
static stat_t _feedhold_restart_no_actions(void);

// Feedhold exits (finalization)
static stat_t _run_restart_cycle(void);
static stat_t _run_queue_flush(void);
static stat_t _run_program_stop(void);
static stat_t _run_program_end(void);
static stat_t _run_alarm(void);
static stat_t _run_shutdown(void);
static stat_t _run_interlock(void);
static stat_t _run_reset_position(void);
 
/****************************************************************************************
 * OPERATIONS AND ACTIONS
 *
 *  Operations provide a way to assemble a multi-step function from underlying actions,
 *  then execute the actions in sequence until the operation either completes or returns
 *  an error. It handles actions that complete immediately (synchronous) as well as long 
 *  running asynchronous operations such as a series of multiple moves.
 *
 *  It works by assembling an operation using a series of add_action() calls, then running 
 *  the operation by one or more run_operation() calls. The cm_operation_sequencing_callback()
 *  both runs long-running operations, as well as queues operation requests that must
 *  run sequentially of have other conditions.
 *
 *  Actions are coded to return:
 *    STAT_OK       - successful completion of the action
 *    STAT_EAGAIN   - ran to continuation - the action needs to be called again to complete
 *    STAT_XXXXX    - any other status is an error that should quit the operation
 *
 *  run_operation() returns:
 *    STAT_NOOP     - no operation is set up, but it's OK to call the operation runner
 *    STAT_OK       - operation has completed successfully
 *    STAT_EAGAIN   - operation needs to be re-entered to complete (via operation callback)
 *    STAT_XXXXX    - any other status is an error that quits the operation 
 *
 *  Current constraints to keep this simple (at least for now):
 *    - Operations run to completion. They cannot be canceled, or preempted by other operations
 *    - Actions cannot be added to an operation once it is being run
 *    - Actions do not have parameters. Use the CM context if needed (e.g. hold_type)
 */

/*** Object Definitions ***/

#define ACTION_MAX 6                        // maximum actions that can be queued for an operation
typedef stat_t (*action_exec_t)();          // callback to action execution function

typedef struct cmAction {                   // struct to manage execution of operations
    uint8_t number;                         // DIAGNOSTIC for easier debugging. Not used functionally.
    struct cmAction *nx;                    // static pointer to next buffer
    action_exec_t func;                     // callback to operation action function. NULL == disabled

    void reset() {                          // clears function pointer
        func = NULL;
    };
} cmAction_t;

typedef struct cmOperation {                // operation runner object

    cmAction action[ACTION_MAX];            // singly linked list of action structures
    cmAction *add;                          // pointer to next action to be added
    cmAction *run;                          // pointer to action being executed
    bool in_operation;                      // set true when an operation is running

    void reset(void) {
        for (uint8_t i=0; i < ACTION_MAX; i++) {
            action[i].reset();              // reset the action controller object
            action[i].number = i;           // DIAGNOSTIC only. Otherwise not used
            action[i].nx = &action[i+1];    // link to the next action
        }
        action[ACTION_MAX-1].nx = NULL;     // set last action (end of list)
        add = action;                       // initialize pointers to first action struct
        run = action;
        in_operation = false;
    };

    stat_t add_action(stat_t(*action_exec)()) {
        if (in_operation) { return (STAT_COMMAND_NOT_ACCEPTED); }       // can't add
        if (add == NULL)  { return (STAT_INPUT_EXCEEDS_MAX_LENGTH); }   // no more room
        add->func = action_exec;
        add = add->nx;
        return (STAT_OK);
    };

    stat_t run_operation(void) {
        if (run->func == NULL) { return (STAT_NOOP); }  // not an error. This is normal.
        in_operation = true;                // disable add_action during operations
 
        stat_t status;
        while ((status = run->func()) == STAT_OK) {
            run = run->nx;
            if (run->func == NULL) {        // operation has completed
                reset();                    // setup for next operation
                return (STAT_OK);
            }
        }
        if (status == STAT_EAGAIN) { return (STAT_EAGAIN); }
        reset();                            // reset operation if action threw an error
        return (status);                    // return error code
    };
      
} cmOperation_t;

cmOperation_t op;   // operations runner object

/****************************************************************************************
 * cm_operation_init()
 */

void cm_operation_init()
{
    op.reset();
}

/****************************************************************************************
 **** Feedhold and Related Functions ****************************************************
 ****************************************************************************************/
/*
 *  Feedholds, queue flushes and the various feedhold exits are all related. 
 *  These are performed in this file and in plan_exec.cpp. Feedholds are implemented 
 *  as a state machine (cmFeedholdState) that runs in these files using Operations. 
 *
 *  BACKGROUND: There are 2 planners: p1 (primary planner) and p2 (secondary planner). 
 *
 *  A feedhold (!) received while in p1 stops motion in p1 and optionally transitions to p2, 
 *  where feedhold entry actions such as Z lift, parking moves, spindle and coolant pause 
 *  are run. While in p2 (almost) all machine operations are available. There are different
 *  Types of feedholds; Feedhold with action and feedholds with no action transition to p2,
 *  but others do not (e.g. feedhold with sync).
 *
 *  A cycle_start (~) returns to p1 and exits the feedhold, performing exit actions if entry
 *  actions were performed. Motion resumes in p1 from the held point.
 *
 *  A queue_flush (~) returns to p1 and exits the feedhold, performing exit actions if entry
 *  actions were performed. The p1 planner is flushed, and motion does not resume. The machine
 *  executes a program_stop, and ends in the STOP state.
 *
 *  A feedhold (!) received while in p2 (a feedhold within a feedhold - very Inception)
 *  stops motion in p2 and flushes the p2 planner. Control remains in p2.
 *
 *  Other variants of feedhold and exit exist, but these are invoked internally only to put
 *  the machine in END, ALARM, SHUTDOWN, INTERLOCK and other states.
 */
/*
 * Feedhold State Machine Processing
 *
 *  Feedhold is run as a state machine using the following states:
 *    FEEDHOLD_OFF  - Not in a feedhold. May be in a cycle or not running
 *    FEEDHOLD_HOLD - Feedhold stable state. Achieved when machine has stopped in the hold.
 *    FEEDHOLD_XXXX - Any other feedhold state is transient; the machine is headed towards
 *                    FEEDHOLD_HOLD or FEEDHOLD_OFF.
 *
 *  For internal purposes, any state other than FEEDHOLD_OFF is considered to be in a hold.
 * 
 *  Feedhold processing performs the following (in rough sequence order):
 *
 *  (0) - Feedhold is request by calling cm_feedhold_request() 
 *
 * Control transfers to plan_exec.cpp feedhold functions:
 *
 *  (1) - Feedhold arrives while we are in the middle executing of a block
 *   (1a) - The block is currently accelerating - wait for the end of acceleration
 *   (1b) - The block is in a head, but has not started execution yet - start deceleration
 *    (1b1) - The deceleration fits into the current block
 *    (1b2) - The deceleration does not fit and needs to continue in the next block
 *   (1c) - The block is in a body - start deceleration
 *    (1c1) - The deceleration fits into the current block
 *    (1c2) - The deceleration does not fit and needs to continue in the next block
 *   (1d) - The block is currently in the tail - wait until the end of the block
 *   (1e) - We have a new block and a new feedhold request that arrived at EXACTLY the same time
 *          (unlikely, but handled as 1b).
 *
 *  (2) - The block has decelerated to some velocity > zero, so needs continuation into next block
 *  (3) - The end of deceleration is detected inline in mp_exec_aline()
 *  (4) - Finished all runtime work, now wait for motion to stop at HOLD point. When it does:
 *   (4a) - It's a homing or probing feedhold - ditch the remaining buffer & go directly to OFF
 *   (4b) - It's a p2 feedhold - ditch the remaining buffer & signal we want a p2 queue flush
 *   (4c) - It's a normal feedhold - signal we want the p2 entry actions to execute
 *
 * Control transfers back to cycle_feedhold.cpp feedhold functions:
 *
 *  (5) - Run the P2 entry actions and transition to HOLD state when complete
 *  (6) - Remove the hold state / there is queued motion - see cycle_feedhold.cpp
 *  (7) - Remove the hold state / there is no queued motion - see cycle_feedhold.cpp
 */

/****************************************************************************************
 * cm_operation_runner_callback() - run feedhold operations and sequence queued requests
 *
 *  Operations are requested by calling their repective request function, e.g. cm_request_feedhold().
 *  The operation callback runs the current operation, and sequences requests that must be queued.
 *  Expected behaviors: (no-hold means machine is not in hold, etc)
 *
 *  (no-cycle) !    No action. Feedhold is not run (nothing to hold!)
 *  (no-hold)  ~    No action. Cannot exit a feedhold that does not exist
 *  (no-hold)  %    No action. Queue flush is not honored except during a feedhold
 *  (in-cycle) !    Start a hold to motion in the p1 planner
 *  (in-hold)  ~    Wait for feedhold actions to complete, exit feedhold, resume motion 
 *  (in-hold)  %    Wait for feedhold actions to complete, exit feedhold, do not resume motion
 *  (in-p2)    !    If moving in p2 during a p1 hold ! will perform a SYNC type hold in p2
 *  (in-cycle) !~   Start a feedhold, do enter and exit actions, exit feedhold, resume motion
 *  (in-cycle) !%   Start a feedhold, do enter and exit actions, exit feedhold, do not resume motion
 *  (in-cycle) !%~  Same as above
 *  (in-cycle) !~%  Same as above (this one's an anomaly, but the intent would be to Q flush)
 *
 *  The requests are arranged in priority order, highest priority first.
 *  Note that feedholds from p1 are initiated immediately from cm_request_feedhld(), 
 *  and are not triggered here. Only queued p2 feedholds (feedhold in feedhold) are 
 *  handled in the sequencer.
 */

stat_t cm_operation_runner_callback()
{
    if (cm1.job_kill_state == JOB_KILL_REQUESTED) {         // job kill must wait for any active hold to complete
        _start_job_kill();
    }
//    if (cm1.hold_state == FEEDHOLD_REQUESTED) {             // look for a queued p2 feedhold
//        _start_feedhold();
//    }
    if (cm1.queue_flush_state == QUEUE_FLUSH_REQUESTED) {   // look for a queued flush request
        _start_queue_flush();
    }
    if (cm1.cycle_start_state == CYCLE_START_REQUESTED) {   // look for a queued cycle start ot restart
        _start_cycle_restart();
    }

    // run the operation or operation continuation (callback)
    return (op.run_operation());
}

/*
 * cm_has_hold() - return true if a hold condition exists (or a pending hold request)
 */

bool cm_has_hold()
{
    return (cm1.hold_state != FEEDHOLD_OFF);
}

/*
 * cm_feedhold_command_blocker() - prevents new Gcode commands from reaching the parser while feedhold is in effect 
 */

stat_t cm_feedhold_command_blocker()
{
    if (cm1.hold_state != FEEDHOLD_OFF) {
        return (STAT_EAGAIN);
    }
    return (STAT_OK);
}

/*
 * end state functions and helpers
 */

static stat_t _run_program_stop()
{
    cm_cycle_end();                         // end cycle and run program stop
    return (STAT_OK);
}

static stat_t _run_program_end()
{
    cm_program_end();
    return (STAT_OK);
}

static stat_t _run_reset_position()
{
    cm_reset_position_to_absolute_position(cm);
    return (STAT_OK);
}

static stat_t _run_alarm() { return (STAT_OK); }
static stat_t _run_shutdown() { return (STAT_OK); }
static stat_t _run_interlock() { return (STAT_OK); }

/****************************************************************************************
 * cm_request_cycle_start() - set request enum only
 * _start_cycle_start()  - run the cycle start
 */

void cm_request_cycle_start()
{
    if (cm1.hold_state != FEEDHOLD_OFF) {           // restart from a feedhold
        if (cm1.queue_flush_state == QUEUE_FLUSH_REQUESTED) {   // possible race condition. Flush wins
            cm1.cycle_start_state = CYCLE_START_OFF;
        } else {
            cm1.cycle_start_state = CYCLE_START_REQUESTED; 
        }
    } else {                                        // execute cycle start directly
        if (mp_has_runnable_buffer(&mp1)) {
            cm_cycle_start();
            st_request_exec_move();
        }
        cm1.cycle_start_state = CYCLE_START_OFF;        
    }    
}

static void _start_cycle_restart()
{
    // Feedhold cycle restart builds an operation to complete multiple actions
    if (cm1.hold_state == FEEDHOLD_HOLD) {
        cm1.cycle_start_state = CYCLE_START_OFF;
        switch (cm1.hold_type) {
            case FEEDHOLD_TYPE_HOLD:    { op.add_action(_feedhold_restart_no_actions); break; }
            case FEEDHOLD_TYPE_ACTIONS: { op.add_action(_feedhold_restart_with_actions); break; }
            default: {}
        }
        switch (cm1.hold_exit) {
            case FEEDHOLD_EXIT_CYCLE:     { op.add_action(_run_restart_cycle); break; }
            case FEEDHOLD_EXIT_FLUSH:     { op.add_action(_run_queue_flush); } // no break
            case FEEDHOLD_EXIT_STOP:      { op.add_action(_run_program_stop); break; }
            case FEEDHOLD_EXIT_END:       { op.add_action(_run_program_end); break; }
            case FEEDHOLD_EXIT_ALARM:     { op.add_action(_run_alarm); break; }
            case FEEDHOLD_EXIT_SHUTDOWN:  { op.add_action(_run_shutdown); break; }
            case FEEDHOLD_EXIT_INTERLOCK: { op.add_action(_run_interlock); break; }
            default: {}
        }
    } 
}

/****************************************************************************************
 * cm_request_queue_flush() - set request enum only
 * _start_queue_flush()     - run a queue flush from a %
 * _run_queue_flush()       - run a queue flush from an action
 *
 * cm_request_queue_flush() should be called concurrently with xio_flush_to_command(), like this:
 *      { cm_request_queue_flush(); xio_flush_to_command(); }
 */

void cm_request_queue_flush()
{
    // Can only initiate a queue flush if in a feedhold
    if (cm1.hold_state != FEEDHOLD_OFF) {
        cm1.queue_flush_state = QUEUE_FLUSH_REQUESTED;
    } else {
        cm1.queue_flush_state = QUEUE_FLUSH_OFF;        
    }
}

static void _start_queue_flush()
{
    // Don't initiate the queue until in HOLD state (this also means that runtime is idle)
    if ((cm1.queue_flush_state == QUEUE_FLUSH_REQUESTED) && (cm1.hold_state == FEEDHOLD_HOLD)) {
        if (cm1.hold_type == FEEDHOLD_TYPE_ACTIONS) {
            op.add_action(_feedhold_restart_with_actions);
        } else {
            op.add_action(_feedhold_restart_no_actions);
        }
        op.add_action(_run_queue_flush);
        op.add_action(_run_program_stop);
    }
}

// _run_queue_flush() should not be called until motion has stopped.
// It is completely synchronous so it can be called directly;
// it does not need to be part of an operation().

static stat_t _run_queue_flush()            // typically runs from cm1 planner
{
    cm_abort_arc(cm);                       // kill arcs so they don't just create more alines
    planner_reset((mpPlanner_t *)cm->mp);   // reset primary planner. also resets the mr under the planner
    cm_reset_position_to_absolute_position(cm);
    cm1.queue_flush_state = QUEUE_FLUSH_OFF;
    qr_request_queue_report(0);             // request a queue report, since we've changed the number of buffers available
    return (STAT_OK);
}

/****************************************************************************************
 * cm_request_job_kill() - Control-D handler - set request flag only by ^d
 * _run_job_kill()       - perform the job kill. queue flush, program_end
 * _start_job_kill()     - invoke the job kill function, which may start from various states
 *
 * cm_request_job_kill() should be called concurrently with xio_flush_to_command(), like this:
 *      { cm_request_job_kill(); xio_flush_to_command(); }
 *
 *  Job kill cases:                             Actions:
 *  (0)  job kill from ALARM, SHUTDOWN, PANIC   no action, end request
 *  (1)  job kill from READY, STOP, END         perform PROGRAM_END
 *  (2a) Job kill from machining cycle          hold, flush, perform PROGRAM_END
 *  (2b) Job kill from pending hold             wait for hold to complete 
 *  (2c) Job kill from finished hold            flush, perform PROGRAM_END
 *  (3)  Job kill from PROBE                    flush, perform PROGRAM_END
 *  (4)  Job kill from HOMING                   flush, perform PROGRAM_END
 *  (5)  Job kill from JOGGING                  flush, perform PROGRAM_END
 *  (6)  job kill from INTERLOCK                perform PROGRAM_END
 */

void cm_request_job_kill()
{
    cm1.job_kill_state = JOB_KILL_REQUESTED;
}

// _run_job_kill() should not be called until motion has stopped.
// It is completely synchronous so it can be called directly;
// it does not need to be part of an operation().

static stat_t _run_job_kill()
{
    // if in p2 switch to p1 and copy actual position back to p1
    if (cm == &cm2) {
        cm = &cm1;                                      // return to primary planner (p1)
        mp = (mpPlanner_t *)cm->mp;                     // cm->mp is a void pointer
        mr = mp->mr;
        
        copy_vector(cm1.gmx.position, mr2.position);    // transfer actual position back to p1
        copy_vector(cm1.gm.target, mr2.position);
        copy_vector(mp1.position, mr2.position);
        copy_vector(mr1.position, mr2.position);
    }

    _run_queue_flush();

    coolant_control_immediate(COOLANT_OFF, COOLANT_BOTH); // stop coolant
    spindle_control_immediate(SPINDLE_OFF);               // stop spindle

    cm_set_motion_state(MOTION_STOP);                     // set to stop and set the active model
    cm->hold_state = FEEDHOLD_OFF;
    cm_program_end();

    rpt_exception(STAT_KILL_JOB, "Job killed by ^d");
    sr_request_status_report(SR_REQUEST_IMMEDIATE);    
    cm->job_kill_state = JOB_KILL_OFF;
    return (STAT_OK);
}

// _start_job_kill() will be entered multiple times until the REQUEST is reset to OFF

static void _start_job_kill()
{
    switch (cm1.machine_state) {
        case MACHINE_ALARM:                             // Case 0's - nothing to do. turn off the request
        case MACHINE_SHUTDOWN:
        case MACHINE_PANIC: {
            cm1.job_kill_state = JOB_KILL_OFF;
            return;
        }
        case MACHINE_CYCLE: {                           // Case 2's 
            if (cm1.hold_state == FEEDHOLD_OFF) {       // Case 2a - in cycle and not in a hold
                op.add_action(_feedhold_no_actions);
//                op.add_action(_run_job_kill);
            }
            if (cm1.hold_state == FEEDHOLD_HOLD) {      // Case 2c - in a finished hold
                _run_job_kill();
            }
            return;                                     // Case 2b - hold is in progress. Wait for hold to reach HOLD
        }
        default: { _run_job_kill(); }                   // Cases 1,3,4,5,6 
    }
}

/****************************************************************************************
 *  cm_request_feedhold()    - request a feedhold - do not run it yet
 *  _feedhold_skip()         - run feedhold that will skip remaining unused buffer length
 *  _feedhold_no_actions()   - run feedhold with no entry actions
 *  _feedhold_with_actions() - run feedhold entry actions
 *  _feedhold_actions_done_callback() - planner callback to reach sync point
 *
 *  Input arguments
 *    - See cmFeedholdType  - how the feedhold will execute
 *    - See cmFeedholdFinal - the final state when the feedhold is exited
 */

void cm_request_feedhold(cmFeedholdType type, cmFeedholdExit exit)
{
    // Can only initiate a feedhold if you are in a machining cycle, running, and not already in a feedhold

    // +++++ This needs to be extended to allow HOLDs to be requested when motion has stopped
    if ((cm1.hold_state == FEEDHOLD_OFF) &&
        (cm1.machine_state == MACHINE_CYCLE) && (cm1.motion_state == MOTION_RUN)) {

        cm1.hold_type = type;
        cm1.hold_exit = exit;
        cm1.hold_profile = ((type == FEEDHOLD_TYPE_ACTIONS) || (type == FEEDHOLD_TYPE_HOLD)) ?
        PROFILE_NORMAL : PROFILE_FAST;

        switch (cm1.hold_type) {
            case FEEDHOLD_TYPE_HOLD:     { op.add_action(_feedhold_no_actions); break; }
            case FEEDHOLD_TYPE_ACTIONS:  { op.add_action(_feedhold_with_actions); break; }
            case FEEDHOLD_TYPE_SKIP:     { op.add_action(_feedhold_skip); break; }
            default: {}
        }
        switch (cm1.hold_exit) {
            case FEEDHOLD_EXIT_STOP:      { op.add_action(_run_program_stop); break; }
            case FEEDHOLD_EXIT_END:       { op.add_action(_run_program_end); break; }
            case FEEDHOLD_EXIT_ALARM:     { op.add_action(_run_alarm); break; }
            case FEEDHOLD_EXIT_SHUTDOWN:  { op.add_action(_run_shutdown); break; }
            case FEEDHOLD_EXIT_INTERLOCK: { op.add_action(_run_interlock); break; }
            case FEEDHOLD_EXIT_RESET_POSITION: { op.add_action(_run_reset_position); break; }
            default: {}
        }
        return;
    }

    // Look for p2 feedhold (feedhold in a feedhold)
    if ((cm1.hold_state == FEEDHOLD_HOLD) &&
        (cm2.hold_state == FEEDHOLD_OFF) && (cm2.machine_state == MACHINE_CYCLE)) {
        cm2.hold_state = FEEDHOLD_REQUESTED;
        return;
    }
    
    // Reset the request if it's invalid
    if ((cm1.machine_state != MACHINE_CYCLE) || (cm1.motion_state == MOTION_STOP)) {
        cm->hold_state = FEEDHOLD_OFF;          // cannot honor the feedhold request. reset it
    }

}
/*
static void _start_p2_feedhold()
{
    // P2 feedholds only allow skip types
    if ((cm2.hold_state == FEEDHOLD_REQUESTED) && (cm2.motion_state == MOTION_RUN)) {
        op.add_action(_feedhold_skip);
        cm2.hold_state = FEEDHOLD_SYNC;
    }
}
*/

/*  
 * _enter_p2()  - enter p2 planner with proper state transfer from p1
 * _exit_p2()   - reenter p1 planner with proper state transfer from p2
 *
 * Encapsulate entering and exiting p2, as this is tricky and must be done exactly right
 */

static void _enter_p2()
{
    // Copy the primary canonical machine to the secondary. Here it's OK to co a memcpy.
    // Set parameters in cm, gm and gmx so you can actually use it
    memcpy(&cm2, &cm1, sizeof(cmMachine_t));
    cm2.hold_state = FEEDHOLD_OFF;
    cm2.gm.motion_mode = MOTION_MODE_CANCEL_MOTION_MODE;
    cm2.gm.absolute_override = ABSOLUTE_OVERRIDE_OFF;
    cm2.queue_flush_state = QUEUE_FLUSH_OFF;
    cm2.gm.feed_rate = 0;
    cm2.arc.run_state = BLOCK_INACTIVE;     // Stop a running p1 arc from continuing to execute in p2

    // Set mp planner to p2 and reset it
    cm2.mp = &mp2;
    planner_reset((mpPlanner_t *)cm2.mp);   // mp is a void pointer

    // Clear the target and set the positions to the current hold position
    memset(&(cm2.return_flags), 0, sizeof(cm2.return_flags));
    memset(&(cm2.gm.target), 0, sizeof(cm2.gm.target));
    memset(&(cm2.gm.target_comp), 0, sizeof(cm2.gm.target_comp)); // zero Kahan compensation

    copy_vector(cm2.gmx.position, mr1.position);
    copy_vector(mp2.position, mr1.position);
    copy_vector(mr2.position, mr1.position);

    // Copy MR position and encoder terms - needed for following error correction state
    copy_vector(mr2.target_steps, mr1.target_steps);
    copy_vector(mr2.position_steps, mr1.position_steps);
    copy_vector(mr2.commanded_steps, mr1.commanded_steps);
    copy_vector(mr2.encoder_steps, mr1.encoder_steps);  // NB: following error is re-computed in p2

    // Reassign the globals to the secondary CM
    cm = &cm2;
    mp = (mpPlanner_t *)cm2.mp;     // mp is a void pointer
    mr = mp2.mr;
}

static void _exit_p2()
{
    cm = &cm1;                          // return to primary planner (p1)
    mp = (mpPlanner_t *)cm1.mp;         // cm->mp is a void pointer
    mr = mp1.mr;
}

static void _check_motion_stopped()
{
    if (mp_runtime_is_idle()) {                         // wait for steppers to actually finish
            
        mpBuf_t *bf = mp_get_r();
            
        // Motion has stopped, so we can rely on positions and other values to be stable
        // If SKIP type, discard the remainder of the block and position to the next block
        if (cm->hold_type == FEEDHOLD_TYPE_SKIP) {
            copy_vector(mp->position, mr->position);    // update planner position to the final runtime position
            mp_free_run_buffer();                       // advance to next block, discarding the rest of the move
        } else { // Otherwise setup the block to complete motion (regardless of how hold will ultimately be exited)
            bf->length = get_axis_vector_length(mr->position, mr->target); // update bf w/remaining length in move
            bf->block_state = BLOCK_INITIAL_ACTION;     // tell _exec to re-use the bf buffer
            bf->buffer_state = MP_BUFFER_BACK_PLANNED;  // so it can be forward planned again
            bf->plannable = true;                       // needed so block can be re-planned
        }
        mr->reset();                                    // reset MR for next use and for forward planning
        cm_set_motion_state(MOTION_STOP);
        cm->hold_state = FEEDHOLD_MOTION_STOPPED;
        sr_request_status_report(SR_REQUEST_IMMEDIATE);
    }
}

static stat_t _feedhold_skip()
{
    if (cm1.hold_state == FEEDHOLD_OFF) {       // if entered while OFF start a feedhold
        cm1.hold_type = FEEDHOLD_TYPE_SKIP;
        cm1.hold_state = FEEDHOLD_SYNC;         // ...FLUSH can be overridden by setting hold_exit after this function
    }
    if (cm1.hold_state < FEEDHOLD_MOTION_STOPPED) {
        return (STAT_EAGAIN);
    }
    cm1.hold_state = FEEDHOLD_OFF;              // cannot be in HOLD or command won't plan (see mp_plan_block_list())
    mp_replan_queue(mp_get_r());                // unplan current forward plan (bf head block), and reset all blocks
    st_request_forward_plan();                  // replan from the new bf buffer
    return (STAT_OK);
}

static stat_t _feedhold_no_actions()
{
    // initiate the feedhold
    if (cm1.hold_state == FEEDHOLD_OFF) {       // start a feedhold
        cm1.hold_type = FEEDHOLD_TYPE_HOLD;
//      cm1.hold_exit = FEEDHOLD_EXIT_STOP;     // default exit for NO_ACTIONS is STOP...

        if (cm1.motion_state == MOTION_STOP) {  // if motion has already stopped declare that you are in a feedhold
            _check_motion_stopped();
            cm1.hold_state = FEEDHOLD_HOLD;
        } else {
            cm1.hold_state = FEEDHOLD_SYNC;     // ... STOP can be overridden by setting hold_exit after this function
            return (STAT_EAGAIN);
        }
    }
    
    // wait until feedhold reaches the hold point
    if (cm1.hold_state < FEEDHOLD_MOTION_STOPPED) {
        return (STAT_EAGAIN);
    }
    // complete the feedhold
    mp_replan_queue(mp_get_r());                // unplan current forward plan (bf head block), and reset all blocks
    st_request_forward_plan();                  // replan from the new bf buffer
    cm1.hold_state = FEEDHOLD_HOLD;
    return (STAT_OK);
}

static void _feedhold_actions_done_callback(float* vect, bool* flag)
{
    cm1.hold_state = FEEDHOLD_HOLD_ACTIONS_COMPLETE; // penultimate state before transitioning to FEEDHOLD_HOLD
    sr_request_status_report(SR_REQUEST_IMMEDIATE);
}

static stat_t _feedhold_with_actions()          // Execute Case (5)
{
    // if entered while OFF start a feedhold
    if (cm1.hold_state == FEEDHOLD_OFF) {
        cm1.hold_type = FEEDHOLD_TYPE_ACTIONS;
//      cm1.hold_exit = FEEDHOLD_EXIT_STOP;     // default exit for ACTIONS is STOP...
        if (cm1.motion_state == MOTION_STOP) {  // if motion has already stopped declare that you are in a feedhold
            _check_motion_stopped();
            cm1.hold_state = FEEDHOLD_HOLD;
        } else {
            cm1.hold_state = FEEDHOLD_SYNC;     // ... STOP can be overridden by setting hold_exit after this function
            return (STAT_EAGAIN);
        }
    }

    // Code to run once motion has stopped 
    if (cm1.hold_state == FEEDHOLD_MOTION_STOPPED) {
        cm->hold_state = FEEDHOLD_HOLD_ACTIONS_PENDING;         // next state
        _enter_p2();                                            // enter p2 correctly
        cm_set_g30_position();                                  // set position to return to on exit

        // execute feedhold actions
        if (fp_NOT_ZERO(cm->feedhold_z_lift)) {                 // optional Z lift
            cm_set_distance_mode(INCREMENTAL_DISTANCE_MODE);
            bool flags[] = { 0,0,1,0,0,0 };
            float target[] = { 0,0, _to_inches(cm->feedhold_z_lift), 0,0,0 };   // convert to inches if in inches mode
            cm_straight_traverse(target, flags, PROFILE_NORMAL);
            cm_set_distance_mode(cm1.gm.distance_mode);         // restore distance mode to p1 setting
        }
        spindle_control_sync(SPINDLE_PAUSE);                    // optional spindle pause
        coolant_control_sync(COOLANT_PAUSE, COOLANT_BOTH);      // optional coolant pause
        mp_queue_command(_feedhold_actions_done_callback, nullptr, nullptr);
        return (STAT_EAGAIN);
    }

    // wait for hold actions to complete
    if (cm1.hold_state == FEEDHOLD_HOLD_ACTIONS_PENDING) {
        return (STAT_EAGAIN);
    }
    
    // finalize feedhold entry after callback (this is needed so we can return STAT_OK)
    if (cm1.hold_state == FEEDHOLD_HOLD_ACTIONS_COMPLETE) {
        cm1.hold_state = FEEDHOLD_HOLD;
        return (STAT_OK);
    }
    return (STAT_EAGAIN);                   // keep the compiler happy. Never executed.
}

/****************************************************************************************
 *  _feedhold_restart_no_actions()   - perform hold restart with no actions
 *  _feedhold_restart_with_actions() - perform hold restart with actions
 *  _feedhold_restart_actions_done_callback()
 */

static void _feedhold_restart_actions_done_callback(float* vect, bool* flag)
{
    cm1.hold_state = FEEDHOLD_EXIT_ACTIONS_COMPLETE;    // penultimate state before transitioning to FEEDHOLD_OFF
    sr_request_status_report(SR_REQUEST_IMMEDIATE);
}

//+++++ Make this more robust so it handles being called before reaching HOLD state
static stat_t _feedhold_restart_no_actions()
{
    if (cm1.hold_state == FEEDHOLD_OFF) {
        return (STAT_OK);                       // was called erroneously. Can happen for !%~
    }
    cm = &cm1;                                  // return to primary planner (p1)
    mp = (mpPlanner_t *)cm->mp;                 // cm->mp is a void pointer
    mr = mp->mr;
    return (STAT_OK);
}

static stat_t _feedhold_restart_with_actions()   // Execute Cases (6) and (7)
{
    if (cm1.hold_state == FEEDHOLD_OFF) {
        return (STAT_OK);                       // was called erroneously. Can happen for !%~
    }

    // Check to run first-time code
    if (cm1.hold_state == FEEDHOLD_HOLD) {
        // perform end-hold actions --- while still in secondary machine
        coolant_control_sync(COOLANT_RESUME, COOLANT_BOTH); // resume coolant if paused
        spindle_control_sync(SPINDLE_RESUME);               // resume spindle if paused

        // do return move though an intermediate point; queue a wait
        cm2.return_flags[AXIS_Z] = false;
        cm_goto_g30_position(cm2.gmx.g30_position, cm2.return_flags);
        mp_queue_command(_feedhold_restart_actions_done_callback, nullptr, nullptr);
        cm1.hold_state = FEEDHOLD_EXIT_ACTIONS_PENDING;
        return (STAT_EAGAIN);
    }
    
    // wait for exit actions to complete
    if (cm1.hold_state == FEEDHOLD_EXIT_ACTIONS_PENDING) {
        return (STAT_EAGAIN);
    }
    
    // finalize feedhold exit
    if (cm1.hold_state == FEEDHOLD_EXIT_ACTIONS_COMPLETE) {
        _exit_p2();                         // re-enter p1 correctly
        return (STAT_OK);
    }
    
    return (STAT_EAGAIN);                   // still waiting
}

static stat_t _run_restart_cycle(void)
{
    cm1.hold_state = FEEDHOLD_OFF;          // must precede st_request_exec_move()
    if (mp_has_runnable_buffer(&mp1)) {
        cm_cycle_start();
        st_request_exec_move();
    } else {
        cm_cycle_end();
    }
    return (STAT_OK);
}
