/*
 * planner.h - cartesian trajectory planning and motion execution
 * This file is part of the TinyG project
 *
 * Copyright (c) 2013 - 2015 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2015 Robert Giseburt
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
/*  PLANNER OPERATION:
 *
 *  The planner plans in "forward mode" - it starts close to the running block and plans
 *  towards the new block (bf->nx). Planning accelerations and cruises comes easy with
 *  forward planning, decelerations do not. When the planner detects that a deceleration
 *  is required - such as for the final deceleration to zero at the end of a sequence of
 *  moves (the tail) - it backtracks to plan the deceleration.
 *
 *  The planner operates in either optimistic or pessimistic mode. In optimistic mode it
 *  assumes that new blocks will continue to arrive so it does not plan the last block.
 *  Blocks are (which would...
 */

#ifndef PLANNER_H_ONCE
#define PLANNER_H_ONCE

#include "canonical_machine.h"	// used for GCodeState_t

/*
 * Enums and other type definitions
 *
 * All the enums that equal zero must be zero. Don't change them
 */

typedef void (*cm_exec_t)(float[], bool[]); // callback to canonical_machine execution function

typedef enum {                      // bf->buffer_state values
    MP_BUFFER_EMPTY = 0,            // struct is available for use (MUST BE 0)
    MP_BUFFER_NOT_PLANNED,          // just opened, or written to queue and ready for planning
    MP_BUFFER_PLANNED,              // planned at least once. May still be replanned
    MP_BUFFER_RUNNING,              // current running buffer
    MP_BUFFER_POLAND,
    MP_BUFFER_UKRAINE
} mpBufferState;

typedef enum {				        // bf->move_type values
    MOVE_TYPE_NULL = 0,		        // null move - does a no-op
    MOVE_TYPE_ALINE,		        // acceleration planned line
    MOVE_TYPE_DWELL,                // delay with no movement
    MOVE_TYPE_COMMAND,              // general command
    MOVE_TYPE_TOOL,                 // T command
    MOVE_TYPE_SPINDLE_SPEED,        // S command
    MOVE_TYPE_STOP,                 // program stop
    MOVE_TYPE_END                   // program end
} moveType;

typedef enum {
    MOVE_OFF = 0,                   // move inactive (MUST BE ZERO)
    MOVE_NEW,                       // general value if you need an initialization
    MOVE_RUN                        // general run state (for non-acceleration moves)
} moveState;

typedef enum {
    SECTION_HEAD = 0,               // acceleration
    SECTION_BODY,                   // cruise
    SECTION_TAIL                    // deceleration
} moveSection;
#define SECTIONS 3

typedef enum {
    SECTION_OFF = 0,                // section inactive
    SECTION_NEW,                    // uninitialized section
    SECTION_1st_HALF,               // first half of S curve
    SECTION_2nd_HALF                // second half of S curve or running a BODY (cruise)
} sectionState;

typedef enum {                      // code blocks for planning and trapezoid generation
    NO_HINT = 0,                    // block is not hinted
    PERFECT_ACCEL,                  // straight line acceleration at jerk (H)
    PERFECT_DECEL,                  // straight line deceleration at jerk (T)
    PERFECT_CRUISE,                 // move is all body (B)
    MIXED_ACCEL,                    // kinked acceleration reaches and holds cruise (HB)
    MIXED_DECEL,                    // kinked deceleration starts with a cruise region (BT)
    COMMAND_BLOCK,                  // this block is a command
    HINT_IS_STALE                   // the hint may no longer be valid
} blockHint;

typedef enum {                      // planner operating state
    PLANNER_IDLE = 0,               // planner and movement are idle
    PLANNER_STARTUP,                // ingesting blocks before movement is started
    PLANNER_OPTIMISTIC,             // plan by leaving last block unplanned
    PLANNER_PESSIMISTIC             // plan by planning all blocks, including the tail
} plannerState;

typedef enum {
    ZOID_EXIT_NULL = 0,
    ZOID_EXIT_1a,
    ZOID_EXIT_1c,
    ZOID_EXIT_1d,
    ZOID_EXIT_2a,
    ZOID_EXIT_2c,
    ZOID_EXIT_2d,
    ZOID_EXIT_3c,
    ZOID_EXIT_3s,
    ZOID_EXIT_3s2,
    ZOID_EXIT_3d2,
    ZOID_EXIT_3a2
} zoidExitPoint;

/*** Most of these factors are the result of a lot of tweaking. Change with caution.***/

#define PLANNER_BUFFER_POOL_SIZE 48                     // Suggest 12 min. Limit is 255
#define PLANNER_BUFFER_HEADROOM 4                       // Buffers to reserve in planner before processing new input line
//#define JUNCTION_AGGRESSION     0.75                  // Actually this factor will be divided by 1 million
#define JERK_MULTIPLIER         ((float)1000000)        // DO NOT CHANGE - must always be 1 million

#define MIN_SEGMENT_MS          ((float)0.750)          // minimum segment milliseconds (also minimum move time)
#define NOM_SEGMENT_MS          ((float)1.5)            // nominal segment ms
#define NOM_SEGMENT_TIME        ((float)(NOM_SEGMENT_MS / 60000))   // DO NOT CHANGE - time in minutes
#define NOM_SEGMENT_USEC        ((float)(NOM_SEGMENT_MS * 1000))    // DO NOT CHANGE - time in microseconds
#define MIN_SEGMENT_TIME        ((float)(MIN_SEGMENT_MS / 60000))   // DO NOT CHANGE - time in minutes

#define NEW_BLOCK_TIMEOUT_MS    ((float)50.0)           // MS before deciding there are no new blocks arriving
#define PLANNER_CRITICAL_MS     ((float)20.0)           // MS threshold for planner critical state
#define PLANNER_THROTTLE_MS     ((float)100.0)          // MS threshold to start planner throttling
//#define PLANNER_JIT_MS          ((float)150.0)        // MS threshold for just-in-time planning
#define PHAT_CITY_MS            PLANNER_THROTTLE_MS     // if you have at least this much time in the planner
#define NEW_BLOCK_TIMEOUT_TIME  ((float)(NEW_BLOCK_TIMEOUT_MS / 60000)) // DO NOT CHANGE - time in minutes
#define PLANNER_CRITICAL_TIME   ((float)(PLANNER_CRITICAL_MS / 60000))  // DO NOT CHANGE - time in minutes
#define PLANNER_THROTTLE_TIME   ((float)(PLANNER_THROTTLE_MS / 60000))  // DO NOT CHANGE - time in minutes
#define PLANNER_JIT_TIME        ((float)(PLANNER_JIT_MS / 60000))       // DO NOT CHANGE - time in minutes
#define PHAT_CITY_TIME          ((float)(PHAT_CITY_MS / 60000))         // DO NOT CHANGE - time in minutes

#define THROTTLE_MIN            ((float)0.10)           // minimum factor to slow down for planner throttling
#define THROTTLE_MAX            ((float)1.00)           // unity factor; must always = 1.00
#define THROTTLE_SLOPE          ((float)(1-THROTTLE_MIN) / (PLANNER_THROTTLE_TIME - PLANNER_CRITICAL_TIME))
#define THROTTLE_INTERCEPT      ((float)THROTTLE_MIN)

#define FEED_OVERRIDE_MIN           0.05                // 5% minimum
#define FEED_OVERRIDE_MAX           2.00                // 200% maximum
#define FEED_OVERRIDE_RAMP_TIME     (0.500/60)          // ramp time for feed overrides
#define FEED_OVERRIDE_ENABLE        false               // initial value
#define FEED_OVERRIDE_FACTOR        1.00                // initial value

#define TRAVERSE_OVERRIDE_MIN       0.05                // 5% minimum
#define TRAVERSE_OVERRIDE_MAX       1.00                // 100% maximum
#define TRAVERSE_OVERRIDE_ENABLE    false               // initial value
#define TRAVERSE_OVERRIDE_FACTOR    1.00                // initial value

#define JUNCTION_AGGRESSION_MIN      0.001               // minimum allowable setting
#define JUNCTION_AGGRESSION_MAX      10.00               // maximum allowable setting

//#define MIN_MANUAL_FEEDRATE_OVERRIDE 0.05   // 5%
//#define MAX_MANUAL_FEEDRATE_OVERRIDE 2.00   // 200%
//#define MIN_MANUAL_TRAVERSE_OVERRIDE 0.05   // 5%
//#define MAX_MANUAL_TRAVERSE_OVERRIDE 1.00   // 100%

// Specialized equalities for comparing velocities with tolerances. Read as:
// VELOCITY_EQ  "If deltaV is less than 1% of v1, or v1 and v2 are very close to zero then velocities are effectively equal"
// VELOCITY_EQ2 "If deltaV is less than 1% of v1 then the velocities are effectively equal"
// VELOCITY_NE  "If deltaV is greater than 1% of v1 then the velocities are effectively equal"
// VELOCITY_LT  "If v1 is less than v2 by at least 1% then v1 is effectively less than than v2"

#define VELOCITY_EQ(v1,v2)      (fabs(v1-v2)<v1*0.01 || (v1<0.1 && v2<0.1))
#define VELOCITY_EQ2(v1,v2)     (fabs(v1-v2)<v1*0.01)
#define VELOCITY_NE(v1,v2)      (fabs(v1-v2)>v1*0.01)
#define VELOCITY_LT(v1,v2)      (v1<(v2-v2*0.01))

//#define ASCII_ART(s)            xio_writeline(s)
#define ASCII_ART(s)

/*
 *	Planner structures
 */

//#define flag_vector axis_flags

typedef struct mpBuffer {           // See Planning Velocity Notes for variable usage

    // *** CAUTION *** These two pointers are not reset by _clear_buffer()
    struct mpBuffer *pv;            // static pointer to previous buffer
    struct mpBuffer *nx;            // static pointer to next buffer

    // Note: _clear_buffer() zeros all data from this point down
    stat_t (*bf_func)(struct mpBuffer *bf); // callback to buffer exec function
    cm_exec_t cm_func;              // callback to canonical machine execution function

    //+++++ DIAGNOSTICS for easier debugging
    uint32_t linenum;
    uint8_t buffer_number;
    float move_time_ms;
    float time_in_plan_ms;
    zoidExitPoint zoid_exit;
    //+++++ to here

    mpBufferState buffer_state;     // used to manage queuing/dequeuing
    moveType move_type;             // used to dispatch to run routine
    moveState move_state;           // move state machine sequence
    uint8_t move_code;              // byte that can be used by used exec functions
    blockHint hint;                 // block is coded for trapezoid planning

    float mfo_factor;               // override factor for this block
    float throttle;                 // preserved for backplanning

    // block parameters
    float unit[AXES];               // unit vector for axis scaling & planning
    bool axis_flags[AXES];          // set true for axes participating in the move & for command parameters

    float length;                   // total length of line or helix in mm
    float head_length;
    float body_length;
    float tail_length;

    float head_time;                // computed move time for head
    float body_time;                // ...body
    float tail_time;                // ...tail
    float move_time;                // ...entire move

									// *** SEE NOTES ON THESE VARIABLES, in aline() ***
    float entry_velocity;           // entry velocity requested for the move
    float cruise_velocity;          // cruise velocity requested & achieved
    float exit_velocity;            // exit velocity requested for the move

    float entry_vmax;               // max junction velocity at entry of this move
    float cruise_vmax;              // max cruise velocity requested for move
    float exit_vmax;                // max exit velocity possible (redundant)
    float delta_vmax;               // max velocity difference for this move //++++REMOVE LATER
    float absolute_vmax;            // fastest this block can move w/o exceeding constraints
    float junction_vmax;            // maximum the move can go through the junction

    float jerk;                     // maximum linear jerk term for this move
    float jerk_sq;                  // Jm^2 is used for planning (computed and cached)
    float recip_jerk;               // 1/Jm used for planning (computed and cached)

    GCodeState_t gm;				// Gcode model state - passed from model, used by planner and runtime

} mpBuf_t;

typedef struct mpBufferPool {		// ring buffer for sub-moves
	magic_t magic_start;			// magic number to test memory integrity
	uint8_t buffers_available;		// running count of available buffers

    //+++++DIAGNOSTICS
    float time_in_run_ms;
    float time_in_plan_ms;
    float time_total_ms;
    float move_time_ms;

    // planner state variables
    plannerState planner_state;     // current state of planner
    float time_in_run;		        // time left in the buffer executed by the runtime
    float time_in_plan;	            // time in the planner (exclusive of runtime)
    float time_total;	            // total time planned in the run and planner
    bool request_planning;          // a process has requested unconditional planning (used by feedhold)
    bool new_block;                 // marks the arrival of a new block for planning
    bool new_block_timeout;         // block arrival rate is timing out (no new blocks arriving)
    uint32_t new_block_timer;       // timeout if no new block received N ms after last block committed
    bool backplanning;              // true if planner is in a back-planning pass
    mpBuf_t *backplan_return;       // buffer to return to once back-planning is complete

    // feed overrides and ramp variables (these extend the variables in cm.gmx)
    bool mfo_active;                // true if mfo override is in effect
    float mfo_factor;               // runtime override factor
    bool ramp_active;               // true when a ramp is occurring
    float ramp_target;
    float ramp_dvdt;

    // pointers and buffers
	mpBuf_t *r;						// run buffer pointer
	mpBuf_t *w;						// write buffer pointer
	mpBuf_t *p;						// planner buffer pointer
	mpBuf_t *c;						// pointer to buffer immediately following critical region

	mpBuf_t bf[PLANNER_BUFFER_POOL_SIZE];// buffer storage
	magic_t magic_end;
} mpBufferPool_t;

typedef struct mpMoveMasterSingleton {  // common variables for planning (move master)
	magic_t magic_start;                // magic number to test memory integrity
	float position[AXES];               // final move position for planning purposes
	magic_t magic_end;
} mpMoveMasterSingleton_t;

typedef struct mpMoveRuntimeSingleton {	// persistent runtime variables
//	uint8_t (*run_move)(struct mpMoveRuntimeSingleton *m); // currently running move - left in for reference
	magic_t magic_start;                // magic number to test memory integrity
	moveState move_state;               // state of the overall move
	moveSection section;                // what section is the move in?
	sectionState section_state;         // state within a move section

	float unit[AXES];                   // unit vector for axis scaling & planning
    bool axis_flags[AXES];              // set true for axes participating in the move
	float target[AXES];                 // final target for bf (used to correct rounding errors)
	float position[AXES];               // current move position
	float waypoint[SECTIONS][AXES];     // head/body/tail endpoints for correction

	float target_steps[MOTORS];         // current MR target (absolute target as steps)
	float position_steps[MOTORS];       // current MR position (target from previous segment)
	float commanded_steps[MOTORS];      // will align with next encoder sample (target from 2nd previous segment)
	float encoder_steps[MOTORS];        // encoder position in steps - ideally the same as commanded_steps
	float following_error[MOTORS];      // difference between encoder_steps and commanded steps

	float head_length;                  // copies of bf variables of same name
	float body_length;
	float tail_length;

	float head_time;                    // copies of bf variables of same name
	float body_time;
	float tail_time;

	float entry_velocity;               // actual velocities for the move
	float cruise_velocity;
	float exit_velocity;

	float segments;                     // number of segments in line (also used by arc generation)
	uint32_t segment_count;             // count of running segments
	float segment_velocity;             // computed velocity for aline segment
	float segment_time;                 // actual time increment per aline segment
	float jerk;                         // max linear jerk

	float forward_diff_1;               // forward difference level 1
	float forward_diff_2;               // forward difference level 2
	float forward_diff_3;               // forward difference level 3
	float forward_diff_4;               // forward difference level 4
	float forward_diff_5;               // forward difference level 5

	GCodeState_t gm;                    // gcode model state currently executing

	magic_t magic_end;
} mpMoveRuntimeSingleton_t;

// Reference global scope structures
extern mpBufferPool_t mb;               // move buffer queue
extern mpMoveMasterSingleton_t mm;      // context for line planning
extern mpMoveRuntimeSingleton_t mr;     // context for line runtime

/*
 * Global Scope Functions
 */

//planner.cpp functions

void planner_init(void);
void planner_reset(void);
void planner_init_assertions(void);
stat_t planner_test_assertions(void);

void mp_halt_runtime(void);
void mp_flush_planner(void);
void mp_set_planner_position(uint8_t axis, const float position);
void mp_set_runtime_position(uint8_t axis, const float position);
void mp_set_steps_to_runtime_position(void);

void mp_queue_command(void(*cm_exec_t)(float[], bool[]), float *value, bool *flag);
stat_t mp_runtime_command(mpBuf_t *bf);

stat_t mp_dwell(const float seconds);
void mp_end_dwell(void);
void mp_request_out_of_band_dwell(float seconds);
stat_t mp_exec_out_of_band_dwell(void);

// planner functions and helpers
uint8_t mp_get_planner_buffers(void);
bool mp_planner_is_full(void);
bool mp_has_runnable_buffer(void);
bool mp_is_phat_city_time(void);

stat_t mp_planner_callback();
void mp_replan_queue(mpBuf_t *bf);
void mp_start_feed_override(const float ramp_time, const float override_factor);
void mp_end_feed_override(const float ramp_time);

// planner buffer primitives
void mp_init_buffers(void);

//mpBuf_t * mp_get_prev_buffer(const mpBuf_t *bf);      // Use the following macro instead
//mpBuf_t * mp_get_next_buffer(const mpBuf_t *bf);      // Use the following macro instead
#define mp_get_prev_buffer(b) ((mpBuf_t *)(b->pv))
#define mp_get_next_buffer(b) ((mpBuf_t *)(b->nx))

mpBuf_t * mp_get_write_buffer(void);
void mp_commit_write_buffer(const moveType move_type);
mpBuf_t * mp_get_run_buffer(void);
bool mp_free_run_buffer(void);

// plan_line.c functions
void mp_zero_segment_velocity(void);                    // getters and setters...
float mp_get_runtime_velocity(void);
float mp_get_runtime_absolute_position(uint8_t axis);
float mp_get_runtime_work_position(uint8_t axis);
void mp_set_runtime_work_offset(float offset[]);
bool mp_get_runtime_busy(void);
bool mp_runtime_is_idle(void);

stat_t mp_aline(GCodeState_t *gm_in);                   // line planning...
void mp_plan_block_list(void);

// plan_zoid.c functions
void mp_calculate_trapezoid(mpBuf_t *bf);
float mp_get_target_length(const float Vi, const float Vf, const mpBuf_t *bf);
float mp_get_target_velocity(const float Vi, const float L, const mpBuf_t *bf);

// plan_exec.c functions
stat_t mp_exec_move(void);
stat_t mp_exec_aline(mpBuf_t *bf);
void mp_exit_hold_state(void);

#endif	// End of include Guard: PLANNER_H_ONCE
