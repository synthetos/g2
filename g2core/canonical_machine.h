/*
 * canonical_machine.h - rs274/ngc canonical machining functions
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2018 Alden S. Hart Jr.
 * Copyright (c) 2016 - 2018 Rob Giseburt
 *
 * This code is a loose implementation of Kramer, Proctor and Messina's
 * canonical machining functions as described in the NIST RS274/NGC v3
 */
/* This file ("the software") is free software: you can redistribute it and/or modify
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

#ifndef CANONICAL_MACHINE_H_ONCE
#define CANONICAL_MACHINE_H_ONCE

#include "config.h"
#include "hardware.h"                       // Note: hardware.h is specific to the hardware target selected
#include "settings.h"
#include "gcode.h"

#if MARLIN_COMPAT_ENABLED == true
#include "marlin_compatibility.h"           // import Marlin definitions and enums
#endif

/* Defines, Macros, and Assorted Parameters */

#define MODEL   (GCodeState_t *)&cm->gm     // absolute pointer from canonical machine gm model
#define RUNTIME (GCodeState_t *)&mr->gm     // absolute pointer from runtime mm struct
#define ACTIVE_MODEL cm->am                 // active model pointer is maintained by cm_set_motion_state()

#define _to_millimeters(a)  ((cm->gm.units_mode == INCHES) ? ((float)a * (float)MM_PER_INCH) : (float)a)
#define _to_inches(a)       ((cm->gm.units_mode == INCHES) ? ((float)a * (float)(1/MM_PER_INCH)) : (float)a)

#define DISABLE_SOFT_LIMIT  (999999)
#define JERK_INPUT_MIN      (0.01)          // minimum allowable jerk setting in millions mm/min^3
#define JERK_INPUT_MAX      (1000000)       // maximum allowable jerk setting in millions mm/min^3
#define PROBES_STORED       3               // we store three probes for coordinate rotation computation
#define MAX_LINENUM         2000000000      // set 2 billion as max line number

/*****************************************************************************
 * MACHINE STATE MODEL
 *
 * The following main variables track canonical machine state and state transitions.
 *    - cm->machine_state - overall state of machine and program execution
 *    - cm->motion_state  - state of movement
 *    - cm->cycle_type    - what type of cycle the machine is executing (or none)
 */
// *** Note: check config printout strings align with all the state variables

// ### LAYER 8 CRITICAL REGION ###
// ### DO NOT CHANGE THESE ENUMERATIONS WITHOUT COMMUNITY INPUT ###
typedef enum {                      // check alignment with messages in config.c / msg_stat strings
    COMBINED_INITIALIZING = 0,      // [0] machine is initializing
    COMBINED_READY,                 // [1] machine is ready for use
    COMBINED_ALARM,                 // [2] machine in alarm state
    COMBINED_PROGRAM_STOP,          // [3] program stop/no more blocks
    COMBINED_PROGRAM_END,           // [4] program end
    COMBINED_RUN,                   // [5] machine is running
    COMBINED_HOLD,                  // [6] machine is holding
    COMBINED_PROBE,                 // [7] probe cycle activ
    COMBINED_CYCLE,                 // [8] reserved for canned cycles
    COMBINED_HOMING,                // [9] homing cycle active
    COMBINED_JOG,                   // [10] jogging cycle active
    COMBINED_INTERLOCK,             // [11] machine in safety interlock hold
    COMBINED_SHUTDOWN,              // [12] machine in shutdown state
    COMBINED_PANIC                  // [13] machine in panic state
} cmCombinedState;
//### END CRITICAL REGION ###

typedef enum {                      // Note: MachineState signals if the machine is in cycle (5) or some other non-cycle state
    MACHINE_INITIALIZING = 0,       // machine is initializing
    MACHINE_READY,                  // machine is ready for use but idle
    MACHINE_ALARM,                  // machine is in alarm state
    MACHINE_PROGRAM_STOP,           // no blocks to run; like PROGRAM_END but without the M2 to reset gcode state
    MACHINE_PROGRAM_END,            // program end (same as MACHINE_READY, really...)
    MACHINE_CYCLE,                  // machine is in cycle, running; blocks still to run, or steppers are busy
    MACHINE_INTERLOCK,              // machine is in interlock state
    MACHINE_SHUTDOWN,               // machine is in shutdown state
    MACHINE_PANIC                   // machine is in panic state
} cmMachineState;

typedef enum {
    MOTION_STOP = 0,                // motion has stopped: set when the steppers reach the end of the planner queue
    MOTION_RUN                      // machine is in motion: set when the steppers execute an ALINE segment
} cmMotionState;

typedef enum {
    CYCLE_NONE = 0,                 // not in a cycle
    CYCLE_MACHINING,                // in normal machining cycle
    CYCLE_HOMING,                   // in homing cycle
    CYCLE_PROBE,                    // in probe cycle
    CYCLE_JOG                       // in jogging cycle
//  CYCLE_G81                       // illustration of canned cycles
//  ...
} cmCycleType;

typedef enum {                      // feedhold type parameter
    FEEDHOLD_TYPE_HOLD,             // simple feedhold at max jerk with no actions
    FEEDHOLD_TYPE_ACTIONS,          // feedhold at max jerk with hold entry actions
    FEEDHOLD_TYPE_SKIP,             // feedhold at max jerk with queue flush and sync command
    FEEDHOLD_TYPE_SCRAM             // feedhold at high jerk and stop all active devices
} cmFeedholdType;

typedef enum {                      // feedhold final operation 
    FEEDHOLD_EXIT_CYCLE = 0,        // exit feedhold with cycle restart
    FEEDHOLD_EXIT_FLUSH,            // exit feedhold with flush
    FEEDHOLD_EXIT_STOP,             // perform program stop
    FEEDHOLD_EXIT_END,              // perform program end
    FEEDHOLD_EXIT_ALARM,            // perform alarm
    FEEDHOLD_EXIT_SHUTDOWN,         // perform shutdown
    FEEDHOLD_EXIT_INTERLOCK,        // report as interlock
    FEEDHOLD_EXIT_RESET_POSITION    // reset machine positions to hold point
} cmFeedholdExit;

typedef enum {                      // feedhold state machine
    FEEDHOLD_OFF = 0,               // no feedhold in effect
    FEEDHOLD_REQUESTED,             // feedhold has been requested but not started yet
    FEEDHOLD_SYNC,                  // start hold - sync to latest aline segment
    FEEDHOLD_DECEL_CONTINUE,        // in deceleration that will not end at zero
    FEEDHOLD_DECEL_TO_ZERO,         // in deceleration that will go to zero
    FEEDHOLD_DECEL_COMPLETE,        // feedhold deceleration has completed, but motors may not have stopped yet
    FEEDHOLD_MOTION_STOPPING,       // waiting for motors to have stopped at hold point (motion stop)
    FEEDHOLD_MOTION_STOPPED,        // motion has stopped at hold point
    FEEDHOLD_HOLD_ACTIONS_PENDING,  // wait for feedhold actions to complete
    FEEDHOLD_HOLD_ACTIONS_COMPLETE, // 
    FEEDHOLD_HOLD,                  // HOLDING (steady state)
    FEEDHOLD_EXIT_ACTIONS_PENDING,  // performing exit actions
    FEEDHOLD_EXIT_ACTIONS_COMPLETE  // completed exit actions
} cmFeedholdState;

typedef enum {                      // Motion profiles
    PROFILE_NORMAL = 0,             // Normal jerk in effect
    PROFILE_FAST                    // High speed jerk in effect
} cmMotionProfile;

typedef enum {                      // state machine for cycle start
    CYCLE_START_OFF = 0,            // not requested
    CYCLE_START_REQUESTED,
    CYCLE_START_COMPLETE
} cmCycleState;

typedef enum {                      // queue flush state machine
    QUEUE_FLUSH_OFF = 0,            // no queue flush in effect
    QUEUE_FLUSH_REQUESTED           // flush has been requested but not started yet
} cmFlushState;

typedef enum {                      // applies to cm->homing_state
    HOMING_NOT_HOMED = 0,           // machine is not homed (0=false)
    HOMING_HOMED = 1,               // machine is homed (1=true)
    HOMING_WAITING                  // machine waiting to be homed
} cmHomingState;

typedef enum {                      // applies to cm->probe_state
    PROBE_FAILED = 0,               // probe reached endpoint without triggering
    PROBE_SUCCEEDED = 1,            // probe was triggered, cm->probe_results has position
    PROBE_WAITING = 2               // probe is waiting to be started or is running
} cmProbeState;

typedef enum {
    SAFETY_INTERLOCK_ENGAGED = 0,   // meaning the interlock input is CLOSED (low)
    SAFETY_INTERLOCK_DISENGAGED
} cmSafetyState;

typedef enum {                      // feed override state machine
    MFO_OFF = 0,
    MFO_REQUESTED,
    MFO_SYNC
} cmOverrideState;

typedef enum {                      // job kill state machine
    JOB_KILL_OFF = 0,
    JOB_KILL_REQUESTED,
    JOB_KILL_RUNNING    
} cmJobKillState;

/*****************************************************************************
 * CANONICAL MACHINE STRUCTURES
 */

typedef struct cmAxis {

    // axis settings
    cmAxisMode axis_mode;                   // see cmAxisMode above
    float velocity_max;                     // max velocity in mm/min or deg/min
    float feedrate_max;                     // max velocity in mm/min or deg/min
    float jerk_max;                         // max jerk (Jm) in mm/min^3 divided by 1 million
    float jerk_high;                        // high speed deceleration jerk (Jh) in mm/min^3 divided by 1 million
    float travel_min;                       // min work envelope for soft limits
    float travel_max;                       // max work envelope for soft limits
    float radius;                           // radius in mm for rotary axis modes

    // internal derived variables - computed during data entry and cached for computational efficiency
    float recip_velocity_max;
    float recip_feedrate_max;
    float max_junction_accel;
    float high_junction_accel;

    // homing settings
    uint8_t homing_input;                   // set 1-N for homing input. 0 will disable homing
    uint8_t homing_dir;                     // 0=search to negative, 1=search to positive
    float search_velocity;                  // homing search velocity
    float latch_velocity;                   // homing latch velocity
    float latch_backoff;                    // backoff sufficient to clear a switch
    float zero_backoff;                     // backoff from switches for machine zero
} cfgAxis_t;

typedef struct cmArc {                      // planner and runtime variables for arc generation
    magic_t magic_start;
    uint8_t run_state;                      // runtime state machine sequence

    float position[AXES];                   // accumulating runtime position
    float ijk_offset[3];                    // arc IJK offsets

    float length;                           // length of line or helix in mm
    float radius;                           // Raw R value, or computed via offsets
    float theta;                            // starting angle of arc
    float angular_travel;                   // travel along the arc in radians
    float planar_travel;                    // travel in arc plane in mm
    float linear_travel;                    // travel along linear axis of arc in mm
    bool  full_circle;                      // True if full circle arcs specified
    float rotations;                        // number of full rotations to add (P value + sign)

    cmAxes plane_axis_0;                    // arc plane axis 0 - e.g. X for G17
    cmAxes plane_axis_1;                    // arc plane axis 1 - e.g. Y for G17
    cmAxes linear_axis;                     // linear axis (normal to plane)

    float   segments;                       // number of segments in arc or blend
    int32_t segment_count;                  // count of running segments
    float   segment_theta;                  // angular motion per segment
    float   segment_linear_travel;          // linear motion per segment
    float   center_0;                       // center of circle at plane axis 0 (e.g. X for G17)
    float   center_1;                       // center of circle at plane axis 1 (e.g. Y for G17)

    GCodeState_t gm;                        // Gcode state struct is passed for each arc segment.
    magic_t magic_end;
} cmArc_t;

typedef struct cmMachine {                  // struct to manage canonical machine globals and state
    magic_t magic_start;                    // magic number to test memory integrity

    /**** Config variables (PUBLIC) ****/

    // System group settings
    float junction_integration_time;        // how aggressively will the machine corner? 1.6 or so is about the upper limit
    float chordal_tolerance;                // arc chordal accuracy setting in mm
    float feedhold_z_lift;                  // mm to move Z axis on feedhold, or 0 to disable
    bool soft_limit_enable;                 // true to enable soft limit testing on Gcode inputs
    bool limit_enable;                      // true to enable limit switches (disabled is same as override)

    // Coordinate systems and offsets
    float coord_offset[COORDS+1][AXES];     // persistent coordinate offsets: absolute (G53) + G54,G55,G56,G57,G58,G59
    float tool_offset[AXES];                // current tool offset

    // Axis settings
    cfgAxis_t a[AXES];

    // gcode power-on default settings - defaults are not the same as the gm state
    cmCoordSystem    default_coord_system;  // G10 active coordinate system default
    cmCanonicalPlane default_select_plane;  // G17,G18,G19 reset default
    cmUnitsMode      default_units_mode;    // G20,G21 reset default
    cmPathControl    default_path_control;  // G61,G61.1,G64 reset default
    cmDistanceMode   default_distance_mode; // G90,G91 reset default

  /**** Runtime variables (PRIVATE) ****/

    // Global state variables and flags

    cmMachineState  machine_state;          // macs: machine/cycle/motion is the actual machine state
    cmCycleType     cycle_type;             // cycs
    cmMotionState   motion_state;           // mots

    cmFeedholdType  hold_type;              // hold: type of feedhold requested
    cmFeedholdExit  hold_exit;              // hold: final state of hold on exit
    cmMotionProfile hold_profile;           // hold: motion profile to use for deceleration
    cmFeedholdState hold_state;             // hold: feedhold state machine

    cmFlushState    queue_flush_state;      // queue flush state machine
    cmCycleState    cycle_start_state;      // used to manage cycle starts and restarts
    cmJobKillState  job_kill_state;         // used to manage job kill transitions
    cmOverrideState mfo_state;              // feed override state machine

    bool return_flags[AXES];                // flags for recording which axes moved - used in feedhold exit move

    uint8_t limit_requested;                // set non-zero to request limit switch processing (value is input number)
    uint8_t shutdown_requested;             // set non-zero to request shutdown in support of external estop (value is input number)
    bool deferred_write_flag;               // G10 data has changed (e.g. offsets) - flag to persist them
    
    bool safety_interlock_enable;           // true to enable safety interlock system
    bool request_interlock;                 // enter interlock
    bool request_interlock_exit;            // exit interlock
    uint8_t safety_interlock_disengaged;    // set non-zero to start interlock processing (value is input number)
    uint8_t safety_interlock_reengaged;     // set non-zero to end interlock processing (value is input number)
    cmSafetyState safety_interlock_state;   // safety interlock state
    uint32_t esc_boot_timer;                // timer for Electronic Speed Control (Spindle electronics) to boot

    cmHomingState homing_state;             // home: homing cycle sub-state machine
    uint8_t homed[AXES];                    // individual axis homing flags

    bool probe_report_enable;                 // 0=disabled, 1=enabled
    cmProbeState probe_state[PROBES_STORED];  // probing state machine (simple)
    float probe_results[PROBES_STORED][AXES]; // probing results

    float rotation_matrix[3][3];            // three-by-three rotation matrix. We ignore UVW and ABC axes
    float rotation_z_offset;                // separately handle a z-offset to maintain consistent distance to bed

    float jogging_dest;                     // jogging destination as a relative move from current position

  /**** Model state structures ****/
    void *mp;                               // linked mpPlanner_t - use a void pointer to avoid circular header files
    cmArc_t arc;                            // arc parameters
    GCodeState_t *am;                       // active Gcode model is maintained by state management
    GCodeState_t  gm;                       // core gcode model state
    GCodeStateX_t gmx;                      // extended gcode model state

    magic_t magic_end;
} cmMachine_t;

typedef struct cmToolTable {                // struct to keep a global tool table
    float tt_offset[TOOLS+1][AXES];         // persistent tool table offsets
} cmToolTable_t;

/**** Externs - See canonical_machine.cpp for allocation ****/

extern cmMachine_t *cm;                     // pointer to active canonical machine
extern cmMachine_t cm1;                     // canonical machine primary machine
extern cmMachine_t cm2;                     // canonical machine secondary machine
extern cmToolTable_t tt;

/*****************************************************************************
 * FUNCTION PROTOTYPES
 * Serves as a table of contents for the rather large canonical machine source file.
 */

/*--- Internal functions and helpers ---*/

// Model state getters and setters
cmCombinedState cm_get_combined_state(cmMachine_t *_cm);
cmMachineState cm_get_machine_state(void);
cmCycleType cm_get_cycle_type(void);
cmMotionState cm_get_motion_state(void);
cmFeedholdState cm_get_hold_state(void);
cmHomingState cm_get_homing_state(void);
cmProbeState cm_get_probe_state(void);
uint8_t cm_get_jogging_state(void);

void cm_set_motion_state(const cmMotionState motion_state);

uint32_t cm_get_linenum(const GCodeState_t *gcode_state);
cmMotionMode cm_get_motion_mode(const GCodeState_t *gcode_state);
uint8_t cm_get_coord_system(const GCodeState_t *gcode_state);
uint8_t cm_get_units_mode(const GCodeState_t *gcode_state);
uint8_t cm_get_select_plane(const GCodeState_t *gcode_state);
uint8_t cm_get_path_control(const GCodeState_t *gcode_state);
uint8_t cm_get_distance_mode(const GCodeState_t *gcode_state);
uint8_t cm_get_arc_distance_mode(const GCodeState_t *gcode_state);
uint8_t cm_get_feed_rate_mode(const GCodeState_t *gcode_state);
uint8_t cm_get_tool(const GCodeState_t *gcode_state);
uint8_t  cm_get_block_delete_switch(void);
uint8_t cm_get_runtime_busy(void);
float cm_get_feed_rate(const GCodeState_t *gcode_state);

void cm_set_motion_mode(GCodeState_t *gcode_state, const uint8_t motion_mode);
void cm_set_tool_number(GCodeState_t *gcode_state, const uint8_t tool);
void cm_set_absolute_override(GCodeState_t *gcode_state, const uint8_t absolute_override);
void cm_set_model_linenum(int32_t linenum);
stat_t cm_check_linenum();

// Coordinate systems and offsets
float cm_get_combined_offset(const uint8_t axis);
float cm_get_display_offset(const GCodeState_t *gcode_state, const uint8_t axis);
void cm_set_display_offsets(GCodeState_t *gcode_state);
float cm_get_display_position(const GCodeState_t *gcode_state, const uint8_t axis);
float cm_get_absolute_position(const GCodeState_t *gcode_state, const uint8_t axis);

// Critical helpers
void cm_update_model_position(void);
stat_t cm_deferred_write_callback(void);
void cm_set_model_target(const float target[], const bool flag[]);
bool cm_get_soft_limits(void);
void cm_set_soft_limits(bool enable);

stat_t cm_test_soft_limits(const float target[]);

/*--- Canonical machining functions (loosely) defined by NIST [organized by NIST Gcode doc] ---*/

// Initialization and termination (4.3.2)
void canonical_machine_inits(void);
void canonical_machine_init(cmMachine_t *_cm, void *_mp);
void canonical_machine_reset_rotation(cmMachine_t *_cm);        // NOT in NIST
void canonical_machine_reset(cmMachine_t *_cm);
void canonical_machine_init_assertions(cmMachine_t *_cm);
stat_t canonical_machine_test_assertions(cmMachine_t *_cm);

// Representation (4.3.3)
stat_t cm_select_plane(const uint8_t plane);                                // G17, G18, G19
stat_t cm_set_units_mode(const uint8_t mode);                               // G20, G21
stat_t cm_set_distance_mode(const uint8_t mode);                            // G90, G91
stat_t cm_set_arc_distance_mode(const uint8_t mode);                        // G90.1, G91.1
stat_t cm_set_tl_offset(const uint8_t H_word, const bool H_flag,            // G43, G43.2
                        const bool apply_additional);
stat_t cm_cancel_tl_offset(void);                                           // G49
stat_t cm_set_g10_data(const uint8_t P_word, const bool P_flag,             // G10
                       const uint8_t L_word, const bool L_flag,
                       const float offset[], const bool flag[]);

void cm_set_position_by_axis(const uint8_t axis, const float position);     // set position to abs pos - single axis
void cm_reset_position_to_absolute_position(cmMachine_t *_cm);              // set position to abs pos - all axes
stat_t cm_set_absolute_origin(const float origin[], bool flag[]);           // G28.3
void cm_set_axis_origin(uint8_t axis, const float position);                // G28.3 planner callback

stat_t cm_set_coord_system(const uint8_t coord_system);                     // G54 - G59
stat_t cm_set_g92_offsets(const float offset[], const bool flag[]);         // G92
stat_t cm_reset_g92_offsets(void);                                          // G92.1
stat_t cm_suspend_g92_offsets(void);                                        // G92.2
stat_t cm_resume_g92_offsets(void);                                         // G92.3

// Free Space Motion (4.3.4)
stat_t cm_straight_traverse(const float *target, const bool *flags, const uint8_t motion_profile); // G0
stat_t cm_set_g28_position(void);                                           // G28.1
stat_t cm_goto_g28_position(const float target[], const bool flags[]);      // G28
stat_t cm_set_g30_position(void);                                           // G30.1
stat_t cm_goto_g30_position(const float target[], const bool flags[]);      // G30

// Machining Attributes (4.3.5)
stat_t cm_set_feed_rate(const float feed_rate);                             // F parameter
stat_t cm_set_feed_rate_mode(const uint8_t mode);                           // G93, G94, (G95 unimplemented)
stat_t cm_set_path_control(GCodeState_t *gcode_state, const uint8_t mode);  // G61, G61.1, G64

// Machining Functions (4.3.6)
stat_t cm_straight_feed(const float *target, const bool *flags, const uint8_t motion_profile); //G1
stat_t cm_dwell(const float seconds);                                       // G4, P parameter

stat_t cm_arc_feed(const float target[], const bool target_f[],             // G2/G3 - target endpoint
                   const float offset[], const bool offset_f[],             // IJK offsets
                   const float radius, const bool radius_f,                 // radius if radius mode
                   const float P_word, const bool P_word_f,                 // parameter
                   const bool modal_g1_f,                                   // modal group flag for motion group
                   const cmMotionMode motion_mode);                         // defined motion mode

// Spindle Functions (4.3.7)
// see spindle.h for spindle functions - which would go right here

// Tool Functions (4.3.8)
stat_t cm_select_tool(const uint8_t tool);                      // T parameter
stat_t cm_change_tool(const uint8_t tool);                      // M6

// Miscellaneous Functions (4.3.9)
// see coolant.h for coolant functions - which would go right here

void cm_message(const char *message);                           // msg to console (e.g. Gcode comments)

void cm_reset_overrides(void);
stat_t cm_m48_enable(uint8_t enable);                           // M48, M49
stat_t cm_fro_control(const float P_word, const bool P_flag);   // M50
stat_t cm_tro_control(const float P_word, const bool P_flag);   // M50.1
// See spindle.cpp for cm_spo_control()                         // M51        

// Program Functions (4.3.10)
void cm_cycle_start(void);                                      // (no Gcode)
void cm_cycle_end(void);                                        // (no Gcode)
void cm_canned_cycle_end(void);                                 // end of canned cycle
void cm_program_stop(void);                                     // M0
void cm_optional_program_stop(void);                            // M1
void cm_program_end(void);                                      // M2

stat_t cm_json_command(char *json_string);                      // M100
stat_t cm_json_command_immediate(char *json_string);            // M100.1
stat_t cm_json_wait(char *json_string);                         // M102

/**** Cycles and External FIles ****/

// Feedhold and related functions (cycle_feedhold.cpp)
void cm_operation_init(void);
stat_t cm_operation_runner_callback(void);                      // operation action runner

void cm_request_alarm(void);
void cm_request_fasthold(void);
void cm_request_cycle_start(void);
void cm_request_feedhold(cmFeedholdType type, cmFeedholdExit exit);
void cm_request_queue_flush(void);
stat_t cm_feedhold_sequencing_callback(void);                   // process feedhold, cycle start and queue flush requests
stat_t cm_feedhold_command_blocker(void);

bool cm_has_hold(void);                                         // has hold in primary planner

// Homing cycles (cycle_homing.cpp)
stat_t cm_homing_cycle_start(const float axes[], const bool flags[]);        // G28.2
stat_t cm_homing_cycle_start_no_set(const float axes[], const bool flags[]); // G28.4
stat_t cm_homing_cycle_callback(void);                          // G28.2/.4 main loop callback

// Probe cycles
stat_t cm_straight_probe(float target[], bool flags[],          // G38.x
                         bool trip_sense, bool alarm_flag);
stat_t cm_probing_cycle_callback(void);                         // G38.x main loop callback
stat_t cm_get_prbr(nvObj_t *nv);                                // enable/disable probe report
stat_t cm_set_prbr(nvObj_t *nv);

// Jogging cycle (cycle_jogging.cpp)
stat_t cm_jogging_cycle_callback(void);                         // jogging cycle main loop
stat_t cm_jogging_cycle_start(uint8_t axis);                    // {"jogx":-100.3}
float cm_get_jogging_dest(void);                                // get jogging destination

// Alarm management (alarm.cpp)
stat_t cm_alrm(nvObj_t *nv);                                    // trigger alarm from command input
stat_t cm_shutd(nvObj_t *nv);                                   // trigger shutdown from command input
stat_t cm_pnic(nvObj_t *nv);                                    // trigger panic from command input
stat_t cm_clr(nvObj_t *nv);                                     // clear alarm and shutdown from command input
void cm_clear(void);                                            // raw clear command
void cm_parse_clear(const char *s);                             // parse gcode for M30 or M2 clear condition
stat_t cm_is_alarmed(void);                                     // return non-zero status if alarm, shutdown or panic
void cm_halt(void);                                             // halt motion, spindle, coolant, heaters
void cm_halt_motion(void);                                      // halt motion (immediate stop) but not spindle & other IO
stat_t cm_alarm(const stat_t status, const char *msg);          // enter alarm state - preserve Gcode state
stat_t cm_shutdown(const stat_t status, const char *msg);       // enter shutdown state - dump all state
stat_t cm_panic(const stat_t status, const char *msg);          // enter panic state - needs RESET
void cm_request_job_kill(void);                                 // control-D handler

/**** cfgArray interface functions ****/

char cm_get_axis_char(const int8_t axis);
cmAxisType cm_get_axis_type(const nvObj_t *nv);

stat_t cm_get_mline(nvObj_t *nv);       // get model line number
stat_t cm_get_line(nvObj_t *nv);        // get active (model or runtime) line number
stat_t cm_get_stat(nvObj_t *nv);        // get combined machine state as value and string
stat_t cm_get_stat2(nvObj_t *nv);       // get combined machine state as value and string
stat_t cm_get_macs(nvObj_t *nv);        // get raw machine state as value and string
stat_t cm_get_cycs(nvObj_t *nv);        // get raw cycle state
stat_t cm_get_mots(nvObj_t *nv);        // get raw motion state
stat_t cm_get_hold(nvObj_t *nv);        // get raw hold state

stat_t cm_get_home(nvObj_t *nv);        // get machine homing state
stat_t cm_set_home(nvObj_t *nv);        // set machine homing state
stat_t cm_get_hom(nvObj_t *nv);         // get homing state for axis
stat_t cm_get_prob(nvObj_t *nv);        // get probe state
stat_t cm_get_prb (nvObj_t *nv);        // get probe result for axis
stat_t cm_run_jog(nvObj_t *nv);         // start jogging cycle

stat_t cm_get_unit(nvObj_t *nv);        // get unit mode
stat_t cm_get_coor(nvObj_t *nv);        // get coordinate system in effect
stat_t cm_get_momo(nvObj_t *nv);        // get motion mode
stat_t cm_get_plan(nvObj_t *nv);        // get active plane
stat_t cm_get_path(nvObj_t *nv);        // get patch control mode
stat_t cm_get_dist(nvObj_t *nv);        // get distance mode
stat_t cm_get_admo(nvObj_t *nv);        // get arc offset mode
stat_t cm_get_frmo(nvObj_t *nv);        // get feedrate mode
stat_t cm_get_toolv(nvObj_t *nv);       // get tool (value)
stat_t cm_get_pwr(nvObj_t *nv);         // get motor power enable state

stat_t cm_get_vel(nvObj_t *nv);         // get runtime velocity
stat_t cm_get_feed(nvObj_t *nv);        // get feed rate, converted to units
stat_t cm_get_pos(nvObj_t *nv);         // get runtime work position
stat_t cm_get_mpo(nvObj_t *nv);         // get runtime machine position
stat_t cm_get_ofs(nvObj_t *nv);         // get runtime work offset
stat_t cm_get_coord(nvObj_t *nv);       // get coordinate offset
stat_t cm_set_coord(nvObj_t *nv);       // set coordinate offset

stat_t cm_get_g92e(nvObj_t *nv);        // get g92 enable state
stat_t cm_get_g92(nvObj_t *nv);         // get g92 offset
stat_t cm_get_g28(nvObj_t *nv);         // get g28 offset
stat_t cm_get_g30(nvObj_t *nv);         // get g30 offset

//stat_t cm_run_qf(nvObj_t *nv);          // run queue flush
stat_t cm_run_home(nvObj_t *nv);        // start homing cycle

//stat_t cm_dam(nvObj_t *nv);             // dump active model (debugging command)

stat_t cm_get_tof(nvObj_t *nv);         // get tool offset
stat_t cm_set_tof(nvObj_t *nv);         // set tool offset
stat_t cm_get_tt(nvObj_t *nv);          // get tool table value
stat_t cm_set_tt(nvObj_t *nv);          // set tool table value

stat_t cm_get_am(nvObj_t *nv);          // get axis mode
stat_t cm_set_am(nvObj_t *nv);          // set axis mode
stat_t cm_get_tn(nvObj_t *nv);          // get travel minimum
stat_t cm_set_tn(nvObj_t *nv);          // set travel minimum
stat_t cm_get_tm(nvObj_t *nv);          // get travel maximum
stat_t cm_set_tm(nvObj_t *nv);          // set travel maximum
stat_t cm_get_ra(nvObj_t *nv);          // get radius
stat_t cm_set_ra(nvObj_t *nv);          // set radius

float cm_get_axis_jerk(const uint8_t axis);
void cm_set_axis_max_jerk(const uint8_t axis, const float jerk);
void cm_set_axis_high_jerk(const uint8_t axis, const float jerk);

stat_t cm_get_vm(nvObj_t *nv);          // get velocity max
stat_t cm_set_vm(nvObj_t *nv);          // set velocity max and reciprocal
stat_t cm_get_fr(nvObj_t *nv);          // get feedrate max
stat_t cm_set_fr(nvObj_t *nv);          // set feedrate max and reciprocal
stat_t cm_get_jm(nvObj_t *nv);          // get jerk max with 1,000,000 correction
stat_t cm_set_jm(nvObj_t *nv);          // set jerk max with 1,000,000 correction
stat_t cm_get_jh(nvObj_t *nv);          // get jerk high with 1,000,000 correction
stat_t cm_set_jh(nvObj_t *nv);          // set jerk high with 1,000,000 correction

stat_t cm_get_hi(nvObj_t *nv);          // get homing input
stat_t cm_set_hi(nvObj_t *nv);          // set homing input
stat_t cm_get_hd(nvObj_t *nv);          // get homing direction
stat_t cm_set_hd(nvObj_t *nv);          // set homing direction
stat_t cm_get_sv(nvObj_t *nv);          // get homing search velocity
stat_t cm_set_sv(nvObj_t *nv);          // set homing search velocity
stat_t cm_get_lv(nvObj_t *nv);          // get homing latch velocity
stat_t cm_set_lv(nvObj_t *nv);          // set homing latch velocity
stat_t cm_get_lb(nvObj_t *nv);          // get homing latch backoff
stat_t cm_set_lb(nvObj_t *nv);          // set homing latch backoff
stat_t cm_get_zb(nvObj_t *nv);          // get homing zero backoff
stat_t cm_set_zb(nvObj_t *nv);          // set homing zero backoff

stat_t cm_get_jt(nvObj_t *nv);          // get junction integration time constant
stat_t cm_set_jt(nvObj_t *nv);          // set junction integration time constant
stat_t cm_get_ct(nvObj_t *nv);          // get chordal tolerance
stat_t cm_set_ct(nvObj_t *nv);          // set chordal tolerance
stat_t cm_get_zl(nvObj_t *nv);          // get feedhold Z lift
stat_t cm_set_zl(nvObj_t *nv);          // set feedhold Z lift
stat_t cm_get_sl(nvObj_t *nv);          // get soft limit enable
stat_t cm_set_sl(nvObj_t *nv);          // set soft limit enable
stat_t cm_get_lim(nvObj_t *nv);         // get hard limit enable
stat_t cm_set_lim(nvObj_t *nv);         // set hard limit enable
stat_t cm_get_saf(nvObj_t *nv);         // get safety interlock enable
stat_t cm_set_saf(nvObj_t *nv);         // set safety interlock enable

stat_t cm_get_m48(nvObj_t *nv);         // get M48 value (enable/disable overrides)
stat_t cm_set_m48(nvObj_t *nv);         // set M48 value (enable/disable overrides)
stat_t cm_get_froe(nvObj_t *nv);        // get feedrate override enable
stat_t cm_set_froe(nvObj_t *nv);        // set feedrate override enable
stat_t cm_get_fro(nvObj_t *nv);         // get feedrate override factor
stat_t cm_set_fro(nvObj_t *nv);         // set feedrate override factor

stat_t cm_get_troe(nvObj_t *nv);        // get traverse override enable
stat_t cm_set_troe(nvObj_t *nv);        // set traverse override enable
stat_t cm_get_tro(nvObj_t *nv);         // get traverse override factor
stat_t cm_set_tro(nvObj_t *nv);         // set traverse override factor

stat_t cm_set_tram(nvObj_t *nv);        // attempt setting the rotation matrix
stat_t cm_get_tram(nvObj_t *nv);        // return if the rotation matrix is non-identity

stat_t cm_set_nxln(nvObj_t *nv);    // set what value we expect the next line number to have
stat_t cm_get_nxln(nvObj_t *nv);    // return what value we expect the next line number to have

stat_t cm_get_gpl(nvObj_t *nv);         // get gcode default plane
stat_t cm_set_gpl(nvObj_t *nv);         // set gcode default plane
stat_t cm_get_gun(nvObj_t *nv);         // get gcode default units mode
stat_t cm_set_gun(nvObj_t *nv);         // set gcode default units mode
stat_t cm_get_gco(nvObj_t *nv);         // get gcode default coordinate system
stat_t cm_set_gco(nvObj_t *nv);         // set gcode default coordinate system
stat_t cm_get_gpa(nvObj_t *nv);         // get gcode default path control mode
stat_t cm_set_gpa(nvObj_t *nv);         // set gcode default path control mode
stat_t cm_get_gdi(nvObj_t *nv);         // get gcode default distance mode
stat_t cm_set_gdi(nvObj_t *nv);         // set gcode default distance mode

/*--- text_mode support functions ---*/

#ifdef __TEXT_MODE

    void cm_print_vel(nvObj_t *nv);       // model state reporting
    void cm_print_feed(nvObj_t *nv);
    void cm_print_line(nvObj_t *nv);
    void cm_print_stat(nvObj_t *nv);
    void cm_print_macs(nvObj_t *nv);
    void cm_print_cycs(nvObj_t *nv);
    void cm_print_mots(nvObj_t *nv);
    void cm_print_hold(nvObj_t *nv);
    void cm_print_home(nvObj_t *nv);
    void cm_print_hom(nvObj_t *nv);
    void cm_print_unit(nvObj_t *nv);
    void cm_print_coor(nvObj_t *nv);
    void cm_print_momo(nvObj_t *nv);
    void cm_print_plan(nvObj_t *nv);
    void cm_print_path(nvObj_t *nv);
    void cm_print_dist(nvObj_t *nv);
    void cm_print_admo(nvObj_t *nv);
    void cm_print_frmo(nvObj_t *nv);
    void cm_print_tool(nvObj_t *nv);
    void cm_print_g92e(nvObj_t *nv);

    void cm_print_gpl(nvObj_t *nv);         // Gcode defaults
    void cm_print_gun(nvObj_t *nv);
    void cm_print_gco(nvObj_t *nv);
    void cm_print_gpa(nvObj_t *nv);
    void cm_print_gdi(nvObj_t *nv);

    void cm_print_lin(nvObj_t *nv);         // generic print for linear values
    void cm_print_pos(nvObj_t *nv);         // print runtime work position in prevailing units
    void cm_print_mpo(nvObj_t *nv);         // print runtime work position always in MM uints
    void cm_print_ofs(nvObj_t *nv);         // print runtime work offset always in MM uints
    void cm_print_tof(nvObj_t *nv);         // print tool length offset

    void cm_print_jt(nvObj_t *nv);          // global CM settings
    void cm_print_ct(nvObj_t *nv);
    void cm_print_zl(nvObj_t *nv);
    void cm_print_sl(nvObj_t *nv);
    void cm_print_lim(nvObj_t *nv);
    void cm_print_saf(nvObj_t *nv);

    void cm_print_m48(nvObj_t *nv);
    void cm_print_froe(nvObj_t *nv);
    void cm_print_fro(nvObj_t *nv);
    void cm_print_troe(nvObj_t *nv);
    void cm_print_tro(nvObj_t *nv);

    void cm_print_tram(nvObj_t *nv);        // print if the axis has been rotated
    void cm_print_nxln(nvObj_t *nv);    // print the value of the next line number expected

    void cm_print_am(nvObj_t *nv);          // axis print functions
    void cm_print_fr(nvObj_t *nv);
    void cm_print_vm(nvObj_t *nv);
    void cm_print_tm(nvObj_t *nv);
    void cm_print_tn(nvObj_t *nv);
    void cm_print_jm(nvObj_t *nv);
    void cm_print_jh(nvObj_t *nv);
    void cm_print_ra(nvObj_t *nv);

    void cm_print_hi(nvObj_t *nv);
    void cm_print_hd(nvObj_t *nv);
    void cm_print_sv(nvObj_t *nv);
    void cm_print_lv(nvObj_t *nv);
    void cm_print_lb(nvObj_t *nv);
    void cm_print_zb(nvObj_t *nv);
    void cm_print_cofs(nvObj_t *nv);
    void cm_print_cpos(nvObj_t *nv);

#else // __TEXT_MODE

    #define cm_print_vel tx_print_stub      // model state reporting
    #define cm_print_feed tx_print_stub
    #define cm_print_line tx_print_stub
    #define cm_print_stat tx_print_stub
    #define cm_print_macs tx_print_stub
    #define cm_print_cycs tx_print_stub
    #define cm_print_mots tx_print_stub
    #define cm_print_hold tx_print_stub
    #define cm_print_home tx_print_stub
    #define cm_print_hom tx_print_stub
    #define cm_print_unit tx_print_stub
    #define cm_print_coor tx_print_stub
    #define cm_print_momo tx_print_stub
    #define cm_print_plan tx_print_stub
    #define cm_print_path tx_print_stub
    #define cm_print_dist tx_print_stub
    #define cm_print_admo tx_print_stub
    #define cm_print_frmo tx_print_stub
    #define cm_print_tool tx_print_stub
    #define cm_print_g92e tx_print_stub

    #define cm_print_gpl tx_print_stub      // Gcode defaults
    #define cm_print_gun tx_print_stub
    #define cm_print_gco tx_print_stub
    #define cm_print_gpa tx_print_stub
    #define cm_print_gdi tx_print_stub

    #define cm_print_lin tx_print_stub      // generic print for linear values
    #define cm_print_pos tx_print_stub      // print runtime work position in prevailing units
    #define cm_print_mpo tx_print_stub      // print runtime work position always in MM units
    #define cm_print_ofs tx_print_stub      // print runtime work offset always in MM units

    #define cm_print_jt tx_print_stub       // global CM settings
    #define cm_print_ct tx_print_stub
    #define cm_print_zl tx_print_stub
    #define cm_print_sl tx_print_stub
    #define cm_print_lim tx_print_stub
    #define cm_print_saf tx_print_stub

    #define cm_print_m48 tx_print_stub
    #define cm_print_froe tx_print_stub
    #define cm_print_fro tx_print_stub
    #define cm_print_troe tx_print_stub
    #define cm_print_tro tx_print_stub
    #define cm_print_tram tx_print_stub

    #define cm_print_tram tx_print_stub
    #define cm_print_nxln tx_print_stub

    #define cm_print_am tx_print_stub    // axis print functions
    #define cm_print_fr tx_print_stub
    #define cm_print_vm tx_print_stub
    #define cm_print_tm tx_print_stub
    #define cm_print_tn tx_print_stub
    #define cm_print_jm tx_print_stub
    #define cm_print_jh tx_print_stub
    #define cm_print_ra tx_print_stub

    #define cm_print_hi tx_print_stub
    #define cm_print_hd tx_print_stub
    #define cm_print_sv tx_print_stub
    #define cm_print_lv tx_print_stub
    #define cm_print_lb tx_print_stub
    #define cm_print_zb tx_print_stub
    #define cm_print_cofs tx_print_stub
    #define cm_print_cpos tx_print_stub

    #define cm_print_pdt txt_print_stub

#endif // __TEXT_MODE

#endif // End of include guard: CANONICAL_MACHINE_H_ONCE
