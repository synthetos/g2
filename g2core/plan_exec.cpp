/*
 * plan_exec.cpp - execution function for acceleration managed lines
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart, Jr.
 * Copyright (c) 2012 - 2016 Rob Giseburt
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
#include "controller.h"
#include "planner.h"
#include "kinematics.h"
#include "stepper.h"
#include "encoder.h"
#include "report.h"
#include "util.h"
#include "spindle.h"
#include "xio.h"    //+++++DIAGNOSTIC

// execute routines (NB: These are all called from the LO interrupt)
static stat_t _exec_aline_head(mpBuf_t *bf); // passing bf because body might need it, and it might call body
static stat_t _exec_aline_body(mpBuf_t *bf); // passing bf so that body can extend itself if the exit velocity rises.
static stat_t _exec_aline_tail(mpBuf_t *bf);
static stat_t _exec_aline_segment(void);

static void _init_forward_diffs(float v_0, float v_1);

/*******************************************************************************
 * mp_forward_plan() - plan commands and moves ahead of exec; call ramping for moves
 *
 **** WARNING ****
 * This function should NOT be called directly! 
 * Instead call st_request_forward_plan(), which mediates access. 
 *
 *  mp_forward_plan() performs just-in-time forward planning immediately before 
 *  lines and commands are queued to the move execution runtime (exec). 
 *  Unlike backward planning, buffers are only forward planned once.
 *
 *  mp_forward_plan() is called aggressively via st_request_forward_plan().
 *  It has a relatively low interrupt level to call its own.
 *  See also: Planner Overview notes in planner.h
 * 
 *  It examines the currently running buffer and its adjacent buffers to:
 *
 *  - Stop the system from re-planning or planning something that's not prepped
 *  - Plan the next available ALINE (movement) block past the COMMAND blocks
 *  - Skip past/ or pre-plan COMMAND blocks while labeling them as PLANNED
 *
 *  Returns:
 *   - STAT_OK if exec should be called to kickstart (or continue) movement 
 *   - STAT_NOOP to exit with no action taken (do not call exec)
 */
/*
 * --- Forward Planning Processing and Cases ---
 *    
 *  These cases describe all possible sequences of buffers in the planner queue starting 
 *  with the currently executing (or about to execute) Run buffer, looking forward
 *  to more recently arrived buffers. In most cases only one or two buffers need to 
 *  be examined, but contiguous groups of commands may need to be processed. 
 *
 * 'Running' cases are where the run buffer state is RUNNING. Bootstrap handles all other cases.
 * 'Bootstrap' occurs during the startup phase where moves are collected before starting movement.
 *  Conditions that are impossible based on this definition are not listed in the tables below.
 *
 *  See planner.h / bufferState enum for shorthand used in the descriptions. 
 *  All cases assume a mix of moves and commands, as noted in the shorthand.
 *  All cases assume 2 'blocks' - Run block & Plan block. Cases will need to be 
 *  revisited and generalized if more blocks are used in the future (deeper 
 *  forward planning).
 *
 *  'NOT_PREPPED' refers to any preliminary state below PREPPED, i.e. < PREPPED.
 * ' NOT_PREPPED' can be either a move or command, we don't care so it's not specified.
 *
 *  'COMMAND' or 'COMMAND(s)' refers to one command or a contiguous group of command buffers
 *  that may be in PREPPED or PLANNED states. Processing is always the same. Plan all 
 *  PREPPED commands and skip past all PLANNED commands.
 *
 *  Note 1: For MOVEs use the exit velocity of the Run block (mr.r->exit_velocity) as the 
 *  entry velocity of the next adjacent move.
 *
 *  Note 1a: In this special COMMAND case we trust mr.r->exit_velocity  because the 
 *  backplanner has already handled this case for us.
 *
 *  Note 2: For COMMANDs use the entry velocity of the current runtime (mr.entry_velocity)
 *  as the entry velocity for the next adjacent move. mr.entry_velocity is almost always 0, 
 *  but could be non-0 in a race condition. 
 *  FYI: mr.entry_velocity is set at the end of the last running block in mp_exec_aline().
 *
 *
 *  CASE:
 *    0.  Nothing to do
 *
 *             run_buffer
 *             ----------
 *          a. <no buffer>     Run buffer has not yet been initialized (prep null buffer and return NOOP)
 *          b. NOT_PREPPED     No moves or commands in run buffer. Exit with no action
 *
 *    1.  Bootstrap cases (buffer state < RUNNING)
 *
 *             run_buffer       next N bufs     terminal buf    Actions
 *             ----------       -----------     ------------    ----------------------------------
 *          a. PREPPED-MOVE     <don't care>    <don't care>    plan move, exit OK
 *          b. PLANNED-MOVE     NOT_PREPPED     <don't care>    exit NOOP
 *          c. PLANNED-MOVE     PREPPED-MOVE    <don't care>    exit NOOP (don't plan past a PLANNED buffer)
 *          d. PLANNED-MOVE     PLANNED-MOVE    <don't care>    trap illegal condition, exit NOOP
 *          e. PLANNED-MOVE     COMMAND(s)      <don't care>    exit NOOP
 *          f. PREPPED-COMMAND  NOT_PREPPED     <don't care>    plan command, exit OK
 *          g. PREPPED-COMMAND  PREPPED-MOVE    <don't care>    plan command, plan move (Note 2), exit OK
 *          h. PREPPED-COMMAND  PLANNED-MOVE    <don't care>    trap illegal condition, exit NOOP
 *          i. PLANNED-COMMAND  NOT_PREPPED     <don't care>    skip command, exit OK
 *          j. PLANNED-COMMAND  PREPPED-MOVE    <don't care>    skip command, plan move (Note 2), exit OK
 *          k. PLANNED-COMMAND  PLANNED-MOVE    <don't care>    exit NOOP
 *
 *    2.  Running cases (buffer state == RUNNING)
 *
 *             run_buffer       next N bufs     terminal buf    Actions
 *             ----------       -----------     ------------    ----------------------------------
 *          a. RUNNING-MOVE     PREPPED-MOVE    <don't care>    plan move, exit OK
 *          b. RUNNING-MOVE     PLANNED-MOVE    <don't care>    exit NOOP
 *          c. RUNNING-MOVE     COMMAND(s)      NOT_PREPPED     skip/plan command(s), exit OK
 *          d. RUNNING-MOVE     COMMAND(s)      PREPPED-MOVE    skip/plan command(s), plan move, exit OK
 *          e. RUNNING-MOVE     COMMAND(s)      PLANNED-MOVE    exit NOOP
 *          f. RUNNING-COMMAND  PREPPED-MOVE    <don't care>    plan move, exit OK
 *          g. RUNNING-COMMAND  PLANNED-MOVE    <don't care>    exit NOOP
 *          h. RUNNING-COMMAND  COMMAND(s)      NOT_PREPPED     skip/plan command(s), exit OK
 *          i. RUNNING-COMMAND  COMMAND(s)      PREPPED-MOVE    skip/plan command(s), plan move (Note 1a), exit OK
 *          j. RUNNING-COMMAND  COMMAND(s)      PLANNED-MOVE    skip command(s), exit NOOP
 *                                                              (Note: all COMMAND(s) in j. should be in PLANNED state)
 */

static mpBuf_t *_plan_commands(mpBuf_t *bf)         // plan or skip commands; return bf past last command
{
    // must test for buffer state first as the buffer is only "safe" once it's >= PREPPED
    while ((bf->buffer_state >= MP_BUFFER_PREPPED) && (bf->block_type >= BLOCK_TYPE_COMMAND)) {
        if (bf->buffer_state != MP_BUFFER_PLANNED) {        // skip already planned buffers
            bf->buffer_state = MP_BUFFER_PLANNED;           // "planning" is just setting the state (for now)
        }
        bf = bf->nx;
    }
    return (bf);
}

static stat_t _plan_move(mpBuf_t *bf, float entry_velocity)
{
    // Calculate ramps for the current planning block and the next PREPPED buffer
    // The PREPPED buffer will be set to PLANNED later...
    //
    // Pass in the bf buffer that will "link" with the planned block
    // The block and the buffer are implicitly linked for exec_aline()
    //
    // Note that that can only be one PLANNED move at a time.
    // This is to help sync mr.p to point to the next planned mr.bf
    // mr.p is only advanced in mp_exec_aline(), after mp.r = mr.p.
    // This code aligns the buffers and the blocks for exec_aline().

    mpBlockRuntimeBuf_t* block = mr.p;              // set a local planning block so it doesn't change on you
    mp_calculate_ramps(block, bf, entry_velocity);  // (which it will if you don't do this)

    // diagnostic traps
#ifdef IN_DEBUGGER
    if (block->exit_velocity > block->cruise_velocity)  {
        __asm__("BKPT");                            // exit > cruise after calculate_block
    }
    if (block->head_length < 0.00001 && block->body_length < 0.00001 && block->tail_length < 0.00001)  {
        __asm__("BKPT");                            // zero or negative length block
    }
#endif
    bf->buffer_state = MP_BUFFER_PLANNED;           //...here
    bf->plannable = false;
    return (STAT_OK);                               // report that we planned something...
}

stat_t mp_forward_plan()
{
    mpBuf_t *bf = mp_get_run_buffer();
    float entry_velocity;
    
    // Case 0: Examine current running buffer for early exit conditions
    if (bf == NULL) {                               // case 0a: NULL means nothing is running - this is OK
        st_prep_null();
        return (STAT_NOOP);
    }
    if (bf->buffer_state < MP_BUFFER_PREPPED) {     // case 0b: nothing to do. get outta here.
        return (STAT_NOOP);
    }

    // Case 2: Running cases - move bf past run buffer so it acts like case 1
    if (bf->buffer_state == MP_BUFFER_RUNNING) {
        bf = bf->nx;
        entry_velocity = mr.r->exit_velocity;       // set Note 1 entry_velocity (move cases)
    } else {
        entry_velocity = mr.entry_velocity;         // set Note 2 entry velocity (command cases)
    }

    // bf points to command; start cases 1f, 1g, 1h, 1i, 1j, 1k, 2c, 2d, 2e, 2h, 2i, 2j
    if (bf->block_type != BLOCK_TYPE_ALINE) {       // meaning it's a COMMAND
        bf = _plan_commands(bf);                    // plan commands or skip past already planned commands
        // bf now points to the first non-command buffer past the command(s)
        if ((bf->block_type == BLOCK_TYPE_ALINE) && (bf->buffer_state > MP_BUFFER_PREPPED )) { // case 1i
            entry_velocity = mr.r->exit_velocity;   // set entry_velocity for Note 1a
        }        
    } 
    // bf will always be on a non-command at this point - either a move or empty buffer

    // process move                           
    if (bf->block_type == BLOCK_TYPE_ALINE) {       // do cases 1a - 1e; finish cases 1f - 1k
        if (bf->buffer_state == MP_BUFFER_PREPPED) {// do 1a; finish 1f, 1j, 2d, 2i
            return (_plan_move(bf, entry_velocity));
        } else {
            return (STAT_NOOP);                     // do 1b, 1c, 1d, 1e; finish 1g, 1h, 1j, 1k, 2e, 2j
        }
    }
    return (STAT_OK);                               // report that we planned something...
}

/*************************************************************************
 * mp_exec_move() - execute runtime functions to prep move for steppers
 *
 *  Dequeues the buffer queue and executes the move continuations.
 *  Manages run buffers and other details
 */

stat_t mp_exec_move()
{
    mpBuf_t *bf;

    // NULL means nothing's running - this is OK
    if ((bf = mp_get_run_buffer()) == NULL) {
        st_prep_null();
        return (STAT_NOOP);
    }

    if (bf->block_type == BLOCK_TYPE_ALINE) {             // cycle auto-start for lines only

        // first-time operations
        if (bf->buffer_state != MP_BUFFER_RUNNING) {
            if ((bf->buffer_state < MP_BUFFER_PREPPED) && (cm.motion_state == MOTION_RUN)) {
#ifdef IN_DEBUGGER
                __asm__("BKPT");    // mp_exec_move() buffer is not prepped
#endif
                // IMPORTANT: We can't rpt_exception from here!
                st_prep_null();
                return (STAT_NOOP);
            }
            if (bf->nx->buffer_state < MP_BUFFER_PREPPED) {
                // This detects buffer starvation, but also can be a single-line "jog" or command
                // rpt_exception(42, "mp_exec_move() next buffer is empty");
                // ^^^ CAUSES A CRASH. We can't rpt_exception from here!
            }

            if (bf->buffer_state == MP_BUFFER_PREPPED) {
                if (cm.motion_state == MOTION_RUN) {
#ifdef IN_DEBUGGER
                    __asm__("BKPT"); // we are running but don't have a block planned
#endif
                }
                // We need to have it planned. We don't want to do this here, as it
                // might already be happening in a lower interrupt.
                st_request_forward_plan();
                return (STAT_NOOP);
            }

            if (bf->buffer_state == MP_BUFFER_PLANNED) {
                bf->buffer_state = MP_BUFFER_RUNNING;               // must precede mp_planner_time_acccounting()
            } else {
                return (STAT_NOOP);
            }
            mp_planner_time_accounting();
        }

        if (bf->nx->buffer_state >= MP_BUFFER_PREPPED) {
            // We go ahead and *ask* for a forward planning of the next move.
            // This won't call mp_plan_move until we leave this function
            // (and have called mp_exec_aline via bf->bf_func).
            // This also allows mp_exec_aline to advance mr.p first.
            st_request_forward_plan();
        }

        // Manage motion state transitions
        if ((cm.motion_state != MOTION_RUN) && (cm.motion_state != MOTION_HOLD)) {
            cm_set_motion_state(MOTION_RUN);
        }
    }
    if (bf->bf_func == NULL) {
        return(cm_panic(STAT_INTERNAL_ERROR, "mp_exec_move()")); // never supposed to get here
    }
    return (bf->bf_func(bf));                             // run the move callback in the planner buffer
}

/*************************************************************************/
/**** ALINE EXECUTION ROUTINES *******************************************/
/*************************************************************************
 * ---> Everything here fires from interrupts and must be interrupt safe
 *
 *  _exec_aline()          - acceleration line main routine
 *    _exec_aline_head()      - helper for acceleration section
 *    _exec_aline_body()      - helper for cruise section
 *    _exec_aline_tail()      - helper for deceleration section
 *    _exec_aline_segment() - helper for running a segment
 *
 *  Returns:
 *     STAT_OK        move is done
 *     STAT_EAGAIN    move is not finished - has more segments to run
 *     STAT_NOOP      cause no operation from the steppers - do not load the move
 *     STAT_xxxxx     fatal error. Ends the move and frees the bf buffer
 *
 *  This routine is called from the (LO) interrupt level. The interrupt sequencing 
 *  relies on the behaviors of the routines being exactly correct. Each call to 
 *  _exec_aline() must execute and prep *one and only one* segment. If the segment 
 *  is the not the last segment in the bf buffer the _aline() must return STAT_EAGAIN. 
 *  If it's the last segment it must return STAT_OK. If it encounters a fatal error 
 *  that would terminate the move it should return a valid error code. Failure to 
 *  obey this will introduce subtle and very difficult to diagnose bugs (trust us on this).
 *
 *   Note 1: Returning STAT_OK ends the move and frees the bf buffer.
 *           Returning STAT_OK at this point does NOT advance position meaning any
 *           position error will be compensated by the next move.
 *
 *   Note 2: Solves a potential race condition where the current move ends but the
 *           new move has not started because the previous move is still being run
 *           by the steppers. Planning can overwrite the new move.
 */
/* --- State transitions - hierarchical state machine ---
 *
 *  bf->block_state transitions:
 *     from _NEW to _RUN on first call (sub_state set to _OFF)
 *     from _RUN to _OFF on final call
 *      or just remains _OFF
 *
 *  mr.block_state transitions on first call from _OFF to one of _HEAD, _BODY, _TAIL
 *  Within each section state may be
 *     _NEW - trigger initialization
 *     _RUN1 - run the first part
 *     _RUN2 - run the second part
 *
 *  Important distinction to note:
 *    - mp_plan move() is called for every type of move
 *    - mp_exec_move() is called for every type of move
 *    - mp_exec_aline() is only called for alines
 */
/* Synchronization of run BUFFER and run BLOCK
 *
 * Note first: mp_exec_aline() makes a huge assumption: When it comes time to get a 
 *  new run block (mr.r) it assumes the planner block (mr.p) has been fully planned 
 *  via the JIT forward planning and is ready for use as the new run block.
 *
 * The runtime uses 2 structures for the current move or commend, the run BUFFER 
 *  from the planner queue (mb.r, aka bf), and the run BLOCK from the runtime 
 *  singleton (mr.r). These structures are synchronized implicitly, but not 
 *  explicitly referenced, as pointers can lead to race conditions. 
 *  See plan_zoid.cpp / mp_calculate_ramps() for more details
 *
 * When mp_exec_aline() needs to grab a new planner buffer for a new move or command 
 *  (i.e. block state is inactive) it swaps (rolls) the run and planner BLOCKS so that
 *  mr.p (planner block) is now the mr.r (run block), and the old mr.r block becomes 
 *  available for planning; it becomes mr.p block.
 *
 * At the same time, it's when finished with its current run buffer (mb.r), it has already 
 *  advanced to the next buffer. mp_exec_move() does this at the end of previous move.
 *  Or in the bootstrap case, there never was a previous mb.r, so the current one is OK.
 * 
 * As if by magic, the new mb.r aligns with the run block that was just moved in from 
 *  the planning block.
 */

/**** NOTICE ** NOTICE ** NOTICE ****
 **
 **    mp_exec_aline() is called in
 **     --INTERRUPT CONTEXT!!--
 **
 **    Things we MUST NOT do (even indirectly):
 **       mp_plan_buffer()
 **       mp_plan_block_list()
 **       printf()
 **
 **** NOTICE ** NOTICE ** NOTICE ****/

stat_t mp_exec_aline(mpBuf_t *bf)
{
    if (bf->block_state == BLOCK_INACTIVE) {
        return (STAT_NOOP);
    }

    // Initialize all new blocks, regardless of normal or feedhold operation
    if (mr.block_state == BLOCK_INACTIVE) {

        // too short lines have already been removed...
        // so is the following code is no longer needed ++++ ash
        // But let's still alert the condition should it ever occur
        if (fp_ZERO(bf->length)) {                        // ...looks for an actual zero here
            rpt_exception(STAT_PLANNER_ASSERTION_FAILURE, "mp_exec_aline() zero length move");
        }

        // Start a new move by setting up the runtime singleton (mr)
        memcpy(&mr.gm, &(bf->gm), sizeof(GCodeState_t)); // copy in the gcode model state
        bf->block_state = BLOCK_ACTIVE;                  // note that this buffer is running
                                                         // note the planner doesn't look at block_state
        mr.block_state = BLOCK_INITIAL_ACTION;
        mr.section = SECTION_HEAD;
        mr.section_state = SECTION_NEW;

        // This is the only place in the system where mr.r and mr.p are allowed to be changed
        mr.r = mr.p;        // we are now going to run the planning block
        mr.p = mr.p->nx;    // re-use the old running block as the new planning block

        // Assumptions that are required for this to work:
        // entry velocity <= cruise velocity && cruise velocity >= exit velocity
        // Even if the move is head or tail only, cruise velocity needs to be valid.
        // This is because a "head" is *always* entry->cruise, and a "tail" is *always* cruise->exit,
        // even if there are not other sections int he move. (This is a significant time savings.)

        // Here we will check to make sure that the sections are longer than MIN_SEGMENT_TIME

        if ((!fp_ZERO(mr.r->head_length)) && (mr.r->head_time < MIN_SEGMENT_TIME)) {
                // head_time !== body_time
                // We have to compute the new body time addition.
                mr.r->body_length += mr.r->head_length;
                mr.r->body_time = mr.r->body_length/mr.r->cruise_velocity;

                mr.r->head_length = 0;
                mr.r->head_time = 0;
        }
        if ((!fp_ZERO(mr.r->tail_length)) && (mr.r->tail_time < MIN_SEGMENT_TIME)) {
            // tail_time !== body_time
            // We have to compute the new body time addition.
            mr.r->body_length += mr.r->tail_length;
            mr.r->body_time = mr.r->body_length/mr.r->cruise_velocity;

            mr.r->tail_length = 0;
            mr.r->tail_time = 0;
        }

        // At this point, we've already possibly merged head and/or tail into the body.
        // If the body is too "short" (brief) still, we *might* be able to add it to a head or tail.
        // If there's still a head or a tail, we will add the body to whichever there is, maybe both.
        // We saved it for last since it's the most expensive.
        if ((!fp_ZERO(mr.r->body_length)) && (mr.r->body_time < MIN_SEGMENT_TIME)) {

            // We'll add the time to either the head or the tail or split it
            if (mr.r->tail_length > 0) {
                if (mr.r->head_length > 0) {
                    // We'll split the body to the head and tail
                    float body_split = mr.r->body_length/2.0;
                    mr.r->body_length = 0;
                    mr.r->body_time = 0;

                    mr.r->head_length += body_split;
                    mr.r->tail_length += body_split;

                    mr.r->head_time = (2.0 * mr.r->head_length)/(mr.entry_velocity + mr.r->cruise_velocity);
                    mr.r->tail_time = (2.0 * mr.r->tail_length)/(mr.r->cruise_velocity + mr.r->exit_velocity);
                } else {
                    // We'll put it all in the tail
                    mr.r->tail_length += mr.r->body_length;
                    mr.r->body_length = 0;
                    mr.r->body_time = 0;

                    mr.r->tail_time = (2.0 * mr.r->tail_length)/(mr.r->cruise_velocity + mr.r->exit_velocity);
                }
            }
            else if (mr.r->head_length > 0) {
                // We'll put it all in the head
                mr.r->head_length += mr.r->body_length;
                mr.r->body_length = 0;
                mr.r->body_time = 0;

                mr.r->head_time = (2.0 * mr.r->head_length)/(mr.entry_velocity + mr.r->cruise_velocity);
            }
            else {
                // Uh oh! We have a move that's all body, and is still too short!!
                // ++++ RG For now, we'll consider this impossible.
                while (1);
            }
        }

        copy_vector(mr.unit, bf->unit);
        copy_vector(mr.target, bf->gm.target);          // save the final target of the move
        copy_vector(mr.axis_flags, bf->axis_flags);

        // generate the way points for position correction at section ends
        for (uint8_t axis=0; axis<AXES; axis++) {
            mr.waypoint[SECTION_HEAD][axis] = mr.position[axis] + mr.unit[axis] * mr.r->head_length;
            mr.waypoint[SECTION_BODY][axis] = mr.position[axis] + mr.unit[axis] * (mr.r->head_length + mr.r->body_length);
            mr.waypoint[SECTION_TAIL][axis] = mr.position[axis] + mr.unit[axis] * (mr.r->head_length + mr.r->body_length + mr.r->tail_length);
        }
    }

    // Feed Override Processing - We need to handle the following cases (listed in rough sequence order):
    //  (1) - We've received a feed override request in the middle of a cycle

    // Feedhold Processing - We need to handle the following cases (listed in rough sequence order):
    //  (1) - We have a block midway through normal execution and a new feedhold request
    //   (1a) - The deceleration will fit in the length remaining in the running block (mr)
    //   (1b) - The deceleration will not fit in the running block
    //   (1c) - 1a, except the remaining length would be zero or EPSILON close to zero (unlikely)
    //  (2) - We have a new block and a new feedhold request that arrived at EXACTLY the same time (unlikely, but handled)
    //  (3) - We are in the middle of a block
    //   (3a) - The block is currently accelerating (we wait for the body to start)
    //   (3b) - The block is currently in the tail (we wait until the end of the block)
    //  (4) - We have decelerated a block to some velocity > zero (needs continuation in next block)
    //  (5) - We have decelerated a block to zero velocity
    //  (6) - We have finished all the runtime work now we have to wait for the steppers to stop
    //  (7) - The steppers have stopped. No motion should occur
    //  (8) - We are removing the hold state and there is queued motion (handled outside this routine)
    //  (9) - We are removing the hold state and there is no queued motion (also handled outside this routine)

    if (cm.motion_state == MOTION_HOLD) {
        // Case (7) - all motion has ceased
        if (cm.hold_state == FEEDHOLD_HOLD) {
            return (STAT_NOOP);                 // VERY IMPORTANT to exit as a NOOP. No more movement
        }

        // Case (6) - wait for the steppers to stop
        if (cm.hold_state == FEEDHOLD_PENDING) {
            if (mp_runtime_is_idle()) {                                 // wait for the steppers to actually clear out
                if ((cm.cycle_state == CYCLE_HOMING) || (cm.cycle_state == CYCLE_PROBE)) {
                    // when homing, we don't need to stay in HOLD
                    cm.hold_state = FEEDHOLD_OFF;
                } else {
                    cm.hold_state = FEEDHOLD_HOLD;
                }
                mp_zero_segment_velocity();                             // for reporting purposes
                sr_request_status_report(SR_REQUEST_IMMEDIATE);         // was SR_REQUEST_TIMED
                cs.controller_state = CONTROLLER_READY;                 // remove controller readline() PAUSE
            }
            return (STAT_OK);                                           // hold here. No more movement
        }

        // Case (5) - decelerated to zero
        // Update the run buffer then force a replan of the whole planner queue
        if (cm.hold_state == FEEDHOLD_DECEL_END) {
            mr.block_state = BLOCK_INACTIVE;                                    // invalidate mr buffer to reset the new move
            bf->block_state = BLOCK_INITIAL_ACTION;                             // tell _exec to re-use the bf buffer
            bf->length = get_axis_vector_length(mr.target, mr.position);// reset length
            //bf->entry_vmax = 0;                                         // set bp+0 as hold point

            cm.hold_state = FEEDHOLD_PENDING;

            // No point bothering with the rest of this move if homing or probing
            if ((cm.cycle_state == CYCLE_HOMING) || (cm.cycle_state == CYCLE_PROBE)) {
                mp_free_run_buffer();
            }

            mp_replan_queue(mb.r);                                      // make it replan all the blocks

            return (STAT_OK);
        }

        // Cases (1a, 1b), Case (2), Case (4)
        // Build a tail-only move from here. Decelerate as fast as possible in the space we have.
        if ((cm.hold_state == FEEDHOLD_SYNC) ||
            ((cm.hold_state == FEEDHOLD_DECEL_CONTINUE) && (mr.block_state == BLOCK_INITIAL_ACTION))) {

            // Case (3a) - already decelerating, continue the deceleration.
            if (mr.section == SECTION_TAIL) {   // if already in a tail don't decelerate. You already are
                if (fp_ZERO(mr.r->exit_velocity)) {
                    cm.hold_state = FEEDHOLD_DECEL_TO_ZERO;
                } else {
                    cm.hold_state = FEEDHOLD_DECEL_CONTINUE;
                }

            // Case (3b) - currently accelerating - is simply skipped and waited for
            // Small exception, if we *just started* the head, then we're not actually accelerating yet.
            } else if ((mr.section != SECTION_HEAD) || (mr.section_state == SECTION_NEW)) {
                mr.entry_velocity = mr.segment_velocity;

                mr.section = SECTION_TAIL;
                mr.section_state = SECTION_NEW;

                mr.r->head_length = 0;
                mr.r->body_length = 0;

                float available_length = get_axis_vector_length(mr.target, mr.position);
                mr.r->tail_length = mp_get_target_length(0, mr.r->cruise_velocity, bf);  // braking length

                if (fp_ZERO(available_length - mr.r->tail_length)) {    // (1c) the deceleration time is almost exactly the remaining of the current move
                    cm.hold_state = FEEDHOLD_DECEL_TO_ZERO;
                    mr.r->tail_length = available_length;
                    mr.r->exit_velocity = 0;

                } else if (available_length < mr.r->tail_length) {      // (1b) the deceleration has to span multiple moves
                    cm.hold_state = FEEDHOLD_DECEL_CONTINUE;
                    mr.r->tail_length = available_length;
                    mr.r->exit_velocity = mp_get_decel_velocity(mr.r->cruise_velocity, mr.r->tail_length, bf);

                } else {                                                // (1a)the deceleration will fit into the current move
                    cm.hold_state = FEEDHOLD_DECEL_TO_ZERO;
                    mr.r->exit_velocity = 0;
                }
                mr.r->tail_time = mr.r->tail_length*2 / (mr.r->exit_velocity + mr.r->cruise_velocity);
            }
        }
    }

    mr.block_state = BLOCK_ACTIVE;

    // NB: from this point on the contents of the bf buffer do not affect execution

    //**** main dispatcher to process segments ***
    stat_t status = STAT_OK;
         if (mr.section == SECTION_HEAD) { status = _exec_aline_head(bf);}
    else if (mr.section == SECTION_BODY) { status = _exec_aline_body(bf);}
    else if (mr.section == SECTION_TAIL) { status = _exec_aline_tail(bf);}
    else    { return(cm_panic(STAT_INTERNAL_ERROR, "exec_aline()"));}    // never supposed to get here

    // We can't use the if/else block above, since the head may call body, and body call tail, so we wait till after
    if ((mr.section == SECTION_TAIL) // Once we're in the tail, we can't plan the block anymore
        || ((mr.section == SECTION_BODY) && (mr.segment_count < 3))) { // or are too close to the end of the body

        bf->plannable = false;
    }

    // Feedhold Case (5): Look for the end of the deceleration to go into HOLD state
    if ((cm.hold_state == FEEDHOLD_DECEL_TO_ZERO) && (status == STAT_OK)) {
        cm.hold_state = FEEDHOLD_DECEL_END;
        bf->block_state = BLOCK_INITIAL_ACTION;                      // reset bf so it can restart the rest of the move
    }

    // There are 4 things that can happen here depending on return conditions:
    //  status          bf->block_state     Description
    //  -----------     --------------      ----------------------------------------
    //  STAT_EAGAIN   <don't care>          mr buffer has more segments to run
    //  STAT_OK       BLOCK_ACTIVE          mr and bf buffers are done
    //  STAT_OK       BLOCK_INITIAL_ACTION  mr done; bf must be run again (it's been reused)
    //  There is no fourth thing. Nobody expects the Spanish Inquisition

    if (status == STAT_EAGAIN) {
        sr_request_status_report(SR_REQUEST_TIMED);        // continue reporting mr buffer
        // Note that tha'll happen in a lower interrupt level.
    } else {
        mr.block_state = BLOCK_INACTIVE;                        // invalidate mr buffer (reset)
        mr.section_state = SECTION_OFF;
        mp.run_time_remaining = 0.0;                    // it's done, so time goes to zero

        mr.entry_velocity     = mr.r->exit_velocity;     // feed the old exit into the entry.

        if (bf->block_state == BLOCK_ACTIVE) {
            if (mp_free_run_buffer()) { // returns true of the buffer is empty
                if (cm.hold_state == FEEDHOLD_OFF) {
                    cm_cycle_end();    // free buffer & end cycle if planner is empty
                }
            } else {
                st_request_forward_plan();
            }
        }
    }
    return (status);
}

/*
 * mp_exit_hold_state() - end a feedhold
 *
 *  Feedhold is executed as cm.hold_state transitions executed inside _exec_aline()
 *  Invoke a feedhold by calling cm_request_hold() or cm_start_hold() directly
 *  Return from feedhold by calling cm_request_end_hold() or cm_end_hold directly.
 *  See canonical_macine.c for a more detailed explanation of feedhold operation.
 */

void mp_exit_hold_state()
{
    cm.hold_state = FEEDHOLD_OFF;
    if (mp_has_runnable_buffer()) {
        cm_set_motion_state(MOTION_RUN);
        sr_request_status_report(SR_REQUEST_IMMEDIATE);
    } else {
        cm_set_motion_state(MOTION_STOP);
    }
}

/*
 * Forward difference math explained:
 *
 *  We are using a quintic (fifth-degree) Bezier polynomial for the velocity curve.
 *  This gives us a "linear pop" velocity curve; with pop being the sixth derivative of position:
 *  velocity - 1st, acceleration - 2nd, jerk - 3rd, snap - 4th, crackle - 5th, pop - 6th
 *
 *  The Bezier curve takes the form:
 *
 *  V(t) = P_0 * B_0(t) + P_1 * B_1(t) + P_2 * B_2(t) + P_3 * B_3(t) + P_4 * B_4(t) + P_5 * B_5(t)
 *
 *  Where 0 <= t <= 1, and V(t) is the velocity. P_0 through P_5 are the control points, and B_0(t)
 *  through B_5(t) are the Bernstein basis as follows:
 *
 *        B_0(t) =   (1-t)^5        =   -t^5 +  5t^4 - 10t^3 + 10t^2 -  5t   +   1
 *        B_1(t) =  5(1-t)^4 * t    =   5t^5 - 20t^4 + 30t^3 - 20t^2 +  5t
 *        B_2(t) = 10(1-t)^3 * t^2  = -10t^5 + 30t^4 - 30t^3 + 10t^2
 *        B_3(t) = 10(1-t)^2 * t^3  =  10t^5 - 20t^4 + 10t^3
 *        B_4(t) =  5(1-t)   * t^4  =  -5t^5 +  5t^4
 *        B_5(t) =             t^5  =    t^5
 *                                      ^       ^       ^       ^       ^       ^
 *                                      |       |       |       |       |       |
 *                                      A       B       C       D       E       F
 *
 *
 *  We use forward-differencing to calculate each position through the curve.
 *    This requires a formula of the form:
 *
 *        V_f(t) = A*t^5 + B*t^4 + C*t^3 + D*t^2 + E*t + F
 *
 *  Looking at the above B_0(t) through B_5(t) expanded forms, if we take the coefficients of t^5
 *  through t of the Bezier form of V(t), we can determine that:
 *
 *        A =    -P_0 +  5*P_1 - 10*P_2 + 10*P_3 -  5*P_4 +  P_5
 *        B =   5*P_0 - 20*P_1 + 30*P_2 - 20*P_3 +  5*P_4
 *        C = -10*P_0 + 30*P_1 - 30*P_2 + 10*P_3
 *        D =  10*P_0 - 20*P_1 + 10*P_2
 *        E = - 5*P_0 +  5*P_1
 *        F =     P_0
 *
 *  Now, since we will (currently) *always* want the initial acceleration and jerk values to be 0,
 *  We set P_i = P_0 = P_1 = P_2 (initial velocity), and P_t = P_3 = P_4 = P_5 (target velocity),
 *  which, after simplification, resolves to:
 *
 *        A = - 6*P_i +  6*P_t
 *        B =  15*P_i - 15*P_t
 *        C = -10*P_i + 10*P_t
 *        D = 0
 *        E = 0
 *        F = P_i
 *
 *  Given an interval count of I to get from P_i to P_t, we get the parametric "step" size of h = 1/I.
 *  We need to calculate the initial value of forward differences (F_0 - F_5) such that the inital
 *  velocity V = P_i, then we iterate over the following I times:
 *
 *        V   += F_5
 *        F_5 += F_4
 *        F_4 += F_3
 *        F_3 += F_2
 *        F_2 += F_1
 *
 *  See http://www.drdobbs.com/forward-difference-calculation-of-bezier/184403417 for an example of
 *  how to calculate F_0 - F_5 for a cubic bezier curve. Since this is a quintic bezier curve, we
 *  need to extend the formulas somewhat. I'll not go into the long-winded step-by-step here,
 *  but it gives the resulting formulas:
 *
 *        a = A, b = B, c = C, d = D, e = E, f = F
 *        F_5(t+h)-F_5(t) = (5ah)t^4 + (10ah^2 + 4bh)t^3 + (10ah^3 + 6bh^2 + 3ch)t^2 +
 *            (5ah^4 + 4bh^3 + 3ch^2 + 2dh)t + ah^5 + bh^4 + ch^3 + dh^2 + eh
 *
 *        a = 5ah
 *        b = 10ah^2 + 4bh
 *        c = 10ah^3 + 6bh^2 + 3ch
 *        d = 5ah^4 + 4bh^3 + 3ch^2 + 2dh
 *
 *  (After substitution, simplification, and rearranging):
 *        F_4(t+h)-F_4(t) = (20ah^2)t^3 + (60ah^3 + 12bh^2)t^2 + (70ah^4 + 24bh^3 + 6ch^2)t +
 *            30ah^5 + 14bh^4 + 6ch^3 + 2dh^2
 *
 *        a = (20ah^2)
 *        b = (60ah^3 + 12bh^2)
 *        c = (70ah^4 + 24bh^3 + 6ch^2)
 *
 *  (After substitution, simplification, and rearranging):
 *        F_3(t+h)-F_3(t) = (60ah^3)t^2 + (180ah^4 + 24bh^3)t + 150ah^5 + 36bh^4 + 6ch^3
 *
 *  (You get the picture...)
 *        F_2(t+h)-F_2(t) = (120ah^4)t + 240ah^5 + 24bh^4
 *        F_1(t+h)-F_1(t) = 120ah^5
 *
 *  Normally, we could then assign t = 0, use the A-F values from above, and get out initial F_* values.
 *  However, for the sake of "averaging" the velocity of each segment, we actually want to have the initial
 *  V be be at t = h/2 and iterate I-1 times. So, the resulting F_* values are (steps not shown):
 *
 *        F_5 = (121Ah^5)/16 + 5Bh^4 + (13Ch^3)/4 + 2Dh^2 + Eh
 *        F_4 = (165Ah^5)/2 + 29Bh^4 + 9Ch^3 + 2Dh^2
 *        F_3 = 255Ah^5 + 48Bh^4 + 6Ch^3
 *        F_2 = 300Ah^5 + 24Bh^4
 *        F_1 = 120Ah^5
 *
 *  Note that with our current control points, D and E are actually 0.
 */

// Total time: 147us
static void _init_forward_diffs(const float v_0, const float v_1)
{
    // Times from *here*
/* Full formulation:
     const float fifth_T        = T * 0.2; //(1/5) T
     const float two_fifths_T   = T * 0.4; //(2/5) T
     const float twentienth_T_2 = T * T * 0.05; // (1/20) T^2

     const float P_0 = v_0;
     const float P_1 = v_0 +      fifth_T*a_0;
     const float P_2 = v_0 + two_fifths_T*a_0 + twentienth_T_2*j_0;
     const float P_3 = v_1 - two_fifths_T*a_1 + twentienth_T_2*j_1;
     const float P_4 = v_1 -      fifth_T*a_1;
     const float P_5 = v_1;

     const float A =  5*( P_1 - P_4 + 2*(P_3 - P_2) ) +   P_5 - P_0;
     const float B =  5*( P_0 + P_4 - 4*(P_3 + P_1)   + 6*P_2 );
     const float C = 10*( P_3 - P_0 + 3*(P_1 - P_2) );
     const float D = 10*( P_0 + P_2 - 2*P_1 );
     const float E =  5*( P_1 - P_0 );
     //const float F = P_0;
*/
    float A =  -6.0*v_0 +  6.0*v_1;
    float B =  15.0*v_0 - 15.0*v_1;
    float C = -10.0*v_0 + 10.0*v_1;
    // D = 0
    // E = 0
    // F = Vi


    const float h   = 1/(mr.segments);
    const float h_2 = h   * h;
    const float h_3 = h_2 * h;
    const float h_4 = h_3 * h;
    const float h_5 = h_4 * h;

    const float Ah_5 = A * h_5;
    const float Bh_4 = B * h_4;
    const float Ch_3 = C * h_3;

    const float const1 = 7.5625; // (121.0/16.0)
    const float const2 = 3.25;   // ( 13.0/ 4.0)
    const float const3 = 82.5;   // (165.0/ 2.0)

    /*
     *  F_5 = (121/16)A h^5 +  5 B h^4 + (13/4) C h^3 + 2 D h^2 + Eh
     *  F_4 =  (165/2)A h^5 + 29 B h^4 +     9  C h^3 + 2 D h^2
     *  F_3 =     255 A h^5 + 48 B h^4 +     6  C h^3
     *  F_2 =     300 A h^5 + 24 B h^4
     *  F_1 =     120 A h^5
     */

    mr.forward_diff_5 = const1*Ah_5 +  5.0*Bh_4 + const2*Ch_3;
    mr.forward_diff_4 = const3*Ah_5 + 29.0*Bh_4 +    9.0*Ch_3;
    mr.forward_diff_3 =  255.0*Ah_5 + 48.0*Bh_4 +    6.0*Ch_3;
    mr.forward_diff_2 =  300.0*Ah_5 + 24.0*Bh_4;
    mr.forward_diff_1 =  120.0*Ah_5;

    // Calculate the initial velocity by calculating V(h/2)
    const float half_h   = h * 0.5; // h/2
    const float half_h_3 = half_h   * half_h * half_h;
    const float half_h_4 = half_h_3 * half_h;
    const float half_h_5 = half_h_4 * half_h;

    const float half_Ch_3 = C * half_h_3;
    const float half_Bh_4 = B * half_h_4;
    const float half_Ah_5 = A * half_h_5;

    mr.segment_velocity = half_Ah_5 + half_Bh_4 + half_Ch_3 + v_0;
}

/*********************************************************************************************
 * _exec_aline_head()
 */

static stat_t _exec_aline_head(mpBuf_t *bf)
{
    bool first_pass = false;
    if (mr.section_state == SECTION_NEW) {                          // INITIALIZATION
        first_pass = true;
        if (fp_ZERO(mr.r->head_length)) {
            mr.section = SECTION_BODY;
            return(_exec_aline_body(bf));                            // skip ahead to the body generator
        }
        mr.segments = ceil(uSec(mr.r->head_time) / NOM_SEGMENT_USEC);// # of segments for the section
        mr.segment_count = (uint32_t)mr.segments;
        mr.segment_time = mr.r->head_time / mr.segments;             // time to advance for each segment

        if (mr.segment_count == 1) {
            // We will only have one segment, simply average the velocities
            mr.segment_velocity = mr.r->head_length / mr.segment_time;
        } else {
            _init_forward_diffs(mr.entry_velocity, mr.r->cruise_velocity); // <-- sets inital segment_velocity
        }
        if (mr.segment_time < MIN_SEGMENT_TIME) {
            _debug_trap("mr.segment_time < MIN_SEGMENT_TIME");
            return(STAT_OK);                                        // exit without advancing position, say we're done
        }
        mr.section = SECTION_HEAD;
        mr.section_state = SECTION_RUNNING;
    } else {
        mr.segment_velocity += mr.forward_diff_5;
    }

    if (_exec_aline_segment() == STAT_OK) {                     // set up for second half
        if ((fp_ZERO(mr.r->body_length)) && (fp_ZERO(mr.r->tail_length))) {
            return(STAT_OK);                                    // ends the move
        }

        mr.section = SECTION_BODY;
        mr.section_state = SECTION_NEW;
    } else if (!first_pass) {
        mr.forward_diff_5 += mr.forward_diff_4;
        mr.forward_diff_4 += mr.forward_diff_3;
        mr.forward_diff_3 += mr.forward_diff_2;
        mr.forward_diff_2 += mr.forward_diff_1;
    }
    return(STAT_EAGAIN);
}

/*********************************************************************************************
 * _exec_aline_body()
 *
 *    The body is broken into little segments even though it is a straight line so that
 *    feed holds can happen in the middle of a line with a minimum of latency
 */
static stat_t _exec_aline_body(mpBuf_t *bf)
{
    if (mr.section_state == SECTION_NEW) {
        if (fp_ZERO(mr.r->body_length)) {
            mr.section = SECTION_TAIL;
            return(_exec_aline_tail(bf));                   // skip ahead to tail periods
        }

        float body_time = mr.r->body_time;
        mr.segments = ceil(uSec(body_time) / NOM_SEGMENT_USEC);
        mr.segment_time = body_time / mr.segments;
        mr.segment_velocity = mr.r->cruise_velocity;
        mr.segment_count = (uint32_t)mr.segments;
        if (mr.segment_time < MIN_SEGMENT_TIME) {
            _debug_trap("mr.segment_time < MIN_SEGMENT_TIME");
            return(STAT_OK);                                // exit without advancing position, say we're done
        }

        mr.section = SECTION_BODY;
        mr.section_state = SECTION_RUNNING;                 // uses PERIOD_2 so last segment detection works
    }
    if (_exec_aline_segment() == STAT_OK) {                 // OK means this section is done
        if (fp_ZERO(mr.r->tail_length)) {
            return(STAT_OK);                                    // ends the move
        }
        mr.section = SECTION_TAIL;
        mr.section_state = SECTION_NEW;
    }
    return(STAT_EAGAIN);
}

/*********************************************************************************************
 * _exec_aline_tail()
 */

static stat_t _exec_aline_tail(mpBuf_t *bf)
{
    bool first_pass = false;
    if (mr.section_state == SECTION_NEW) {                          // INITIALIZATION
        first_pass = true;

        // Mark the block as unplannable
        bf->plannable = false;

        if (fp_ZERO(mr.r->tail_length)) { return(STAT_OK);}         // end the move
        mr.segments = ceil(uSec(mr.r->tail_time) / NOM_SEGMENT_USEC);// # of segments for the section
        mr.segment_count = (uint32_t)mr.segments;
        mr.segment_time = mr.r->tail_time / mr.segments;             // time to advance for each segment

        if (mr.segment_count == 1) {
            mr.segment_velocity = mr.r->tail_length / mr.segment_time;
        } else {
            _init_forward_diffs(mr.r->cruise_velocity, mr.r->exit_velocity); // <-- sets inital segment_velocity
        }
        if (mr.segment_time < MIN_SEGMENT_TIME) {
            _debug_trap("mr.segment_time < MIN_SEGMENT_TIME");
            return(STAT_OK);                                        // exit without advancing position, say we're done
         // return(STAT_MINIMUM_TIME_MOVE);                         // exit without advancing position
        }
        mr.section = SECTION_TAIL;
        mr.section_state = SECTION_RUNNING;
    } else {
        mr.segment_velocity += mr.forward_diff_5;
    }

    if (_exec_aline_segment() == STAT_OK) {
        return(STAT_OK);                                        // STAT_OK completes the move
    } else if (!first_pass) {
        mr.forward_diff_5 += mr.forward_diff_4;
        mr.forward_diff_4 += mr.forward_diff_3;
        mr.forward_diff_3 += mr.forward_diff_2;
        mr.forward_diff_2 += mr.forward_diff_1;
    }
    return(STAT_EAGAIN);
}

/*********************************************************************************************
 * _exec_aline_segment() - segment runner helper
 *
 * NOTES ON STEP ERROR CORRECTION:
 *
 *  The commanded_steps are the target_steps delayed by one more segment.
 *  This lines them up in time with the encoder readings so a following error can be generated
 *
 *  The following_error term is positive if the encoder reading is greater than (ahead of)
 *  the commanded steps, and negative (behind) if the encoder reading is less than the
 *  commanded steps. The following error is not affected by the direction of movement -
 *  it's purely a statement of relative position. Examples:
 *
 *      Encoder  Commanded   Following Err
 *          100         90           +10        encoder is 10 steps ahead of commanded steps
 *          -90       -100           +10        encoder is 10 steps ahead of commanded steps
 *           90        100           -10        encoder is 10 steps behind commanded steps
 *         -100        -90           -10        encoder is 10 steps behind commanded steps
 */

static stat_t _exec_aline_segment()
{
    float travel_steps[MOTORS];

    // Set target position for the segment
    // If the segment ends on a section waypoint synchronize to the head, body or tail end
    // Otherwise if not at a section waypoint compute target from segment time and velocity
    // Don't do waypoint correction if you are going into a hold.

    if ((--mr.segment_count == 0) && (cm.motion_state != MOTION_HOLD)) {
        copy_vector(mr.gm.target, mr.waypoint[mr.section]);
    } else {
        float segment_length = mr.segment_velocity * mr.segment_time;
        // see https://en.wikipedia.org/wiki/Kahan_summation_algorithm
        //   for the summation compensation description
        for (uint8_t a=0; a<AXES; a++) {
#if 1
            float to_add = (mr.unit[a] * segment_length) - mr.gm.target_comp[a];
            float target = mr.position[a] + to_add;
            mr.gm.target_comp[a] = (target - mr.position[a]) - to_add;
            mr.gm.target[a] = target;
#else
            mr.gm.target[a] = mr.position[a] + (mr.unit[a] * segment_length);
#endif
        }
    }

    // Convert target position to steps
    // Bucket-brigade the old target down the chain before getting the new target from kinematics
    //
    // NB: The direct manipulation of steps to compute travel_steps only works for Cartesian kinematics.
    //       Other kinematics may require transforming travel distance as opposed to simply subtracting steps.


    for (uint8_t m=0; m<MOTORS; m++) {
        mr.commanded_steps[m] = mr.position_steps[m];       // previous segment's position, delayed by 1 segment
        mr.position_steps[m] = mr.target_steps[m];          // previous segment's target becomes position
        mr.encoder_steps[m] = en_read_encoder(m);           // get current encoder position (time aligns to commanded_steps)
        mr.following_error[m] = mr.encoder_steps[m] - mr.commanded_steps[m];
    }
    kn_inverse_kinematics(mr.gm.target, mr.target_steps);   // now determine the target steps...
    for (uint8_t m=0; m<MOTORS; m++) {                      // and compute the distances to be traveled
        travel_steps[m] = mr.target_steps[m] - mr.position_steps[m];
    }

    // Update the mb->run_time_remaining -- we know it's missing the current segment's time before it's loaded, that's ok.
    mp.run_time_remaining -= mr.segment_time;
    if (mp.run_time_remaining < 0) {
        mp.run_time_remaining = 0.0;
    }

    // Call the stepper prep function
    ritorno(st_prep_line(travel_steps, mr.following_error, mr.segment_time));
    copy_vector(mr.position, mr.gm.target);                 // update position from target
    if (mr.segment_count == 0) {
        return (STAT_OK);                                   // this section has run all its segments
    }
    return (STAT_EAGAIN);                                   // this section still has more segments to run
}
