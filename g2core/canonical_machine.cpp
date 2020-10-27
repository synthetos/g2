/*
 * canonical_machine.cpp - rs274/ngc canonical machine.
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2019 Alden S Hart, Jr.
 * Copyright (c) 2014 - 2019 Robert Giseburt
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
/*
 *  This code is a loose implementation of Kramer, Proctor and Messina's canonical
 *  machining functions as described in the NIST RS274/NGC v3
 *
 *  The canonical machine is the layer between the Gcode parser and the motion control
 *  code for a specific robot. It keeps state and executes commands - passing the
 *  stateless commands to the motion planning layer.
 */
/* --- System state contexts - Gcode models ---
 *
 *  Useful reference for doing C callbacks http://www.newty.de/fpt/fpt.html
 *
 *  There are 3 layered contexts for dynamic system state ("gcode model"):
 *      - The gcode model in the canonical machine (the MODEL context, held in cm->gm)
 *      - The gcode model used by the planner (PLANNER context, held in mp & its buffers)
 *      - The gcode model used during motion for reporting (RUNTIME context, held in mr)
 *
 *  It's a bit more complicated than this. The 'gm' struct contains the core Gcode model
 *  context. This originates in the canonical machine and is copied to each planner buffer
 *  (bf buffer) during motion planning. Finally, the gm context is passed to the runtime
 *  (mr) for the RUNTIME context. So at last count the Gcode model exists in as many as
 *  30 copies in the system. (1+28+1)
 *
 *  Depending on the need, any one of these contexts may be called for reporting or by
 *  a function. Most typically, all new commends from the gcode parser work form the MODEL
 *  context, and status reports pull from the RUNTIME while in motion, and from MODEL when
 *  at rest. A convenience is provided in the ACTIVE_MODEL pointer to point to the right
 *  context.
 */
/* --- Synchronizing command execution ---
 *
 *  Some gcode commands only set the MODEL state for interpretation of the current Gcode
 *  block. For example, cm_set_feed_rate(). This sets the MODEL so the move time is
 *  properly calculated for the current (and subsequent) blocks, so it's effected
 *  immediately.
 *
 *  "Synchronous commands" are commands that affect the runtime need to be synchronized
 *  with movement. Examples include G4 dwells, program stops and ends, and most M commands.
 *  These are queued into the planner queue and execute from the queue. Synchronous commands
 *  work like this:
 *
 *    - Call the cm_xxx_xxx() function which will do any input validation and return an
 *      error if it detects one.
 *
 *    - The cm_ function calls mp_queue_command(). Arguments are a callback to the _exec_...()
 *      function, which is the runtime execution routine, and any arguments that are needed
 *      by the runtime. See typedef for *exec in planner.h for details
 *
 *    - mp_queue_command() stores the callback and the args in a planner buffer.
 *
 *    - When planner execution reaches the buffer it executes the callback w/ the args.
 *      Take careful note that the callback executes under an interrupt, so beware of
 *      variables that may need to be volatile.
 *
 *  Note:
 *    - The synchronous command execution mechanism uses 2 vectors in the bf buffer to store
 *      and return values for the callback. It's obvious, but impractical to pass the entire
 *      bf buffer to the callback as some of these commands are actually executed locally
 *      and have no buffer.
 */

#include "g2core.h"     // #1
#include "config.h"     // #2
#include "gcode.h"      // #3
#include "canonical_machine.h"
#include "hardware.h"
#include "controller.h"
#include "json_parser.h"
#include "text_parser.h"
#include "settings.h"

#include "plan_arc.h"
#include "planner.h"
#include "stepper.h"
#include "encoder.h"
#include "spindle.h"
#include "coolant.h"
#include "pwm.h"
#include "report.h"
#include "gpio.h"
#include "temperature.h"
#include "hardware.h"
#include "util.h"
#include "settings.h"
#include "xio.h"            // for serial queue flush
#include "kinematics.h"     // for forward kinematics in cm_cycle_start

/****************************************************************************************
 **** CM GLOBALS & STRUCTURE ALLOCATIONS ************************************************
 ****************************************************************************************/

cmMachine_t *cm;            // pointer to active canonical machine
cmMachine_t cm1;            // canonical machine primary machine
cmMachine_t cm2;            // canonical machine secondary machine
cmToolTable_t tt;           // global tool table

/****************************************************************************************
 **** GENERIC STATIC FUNCTIONS AND VARIABLES ********************************************
 ****************************************************************************************/

static int8_t _axis(const nvObj_t *nv);     // return axis number from token/group in nv

/*
 * _hold_input_handler - a gpioDigitalInputHandler to capture pin change events
 *   Will be registered at init
 */
gpioDigitalInputHandler _hold_input_handler {
    [](const bool state, const inputEdgeFlag edge, const uint8_t triggering_pin_number) {
        if (edge != INPUT_EDGE_LEADING) { return false; }

        // any action = d_in[triggering_pin_number]->getAction();
        // if (action == INPUT_ACTION_STOP) {
        //     cm_request_feedhold(FEEDHOLD_TYPE_HOLD, FEEDHOLD_EXIT_STOP);
        // }
        // if (action == INPUT_ACTION_FAST_STOP) {
        //     cm_request_feedhold(FEEDHOLD_TYPE_HOLD, FEEDHOLD_EXIT_STOP);
        // }

        cm_request_feedhold(FEEDHOLD_TYPE_HOLD, FEEDHOLD_EXIT_STOP);

        return false; // allow others to see this notice
    },
    5,    // priority
    nullptr // next - nullptr to start with
};

/*
 * _halt_input_handler - a gpioDigitalInputHandler to capture pin change events
 *   Will be registered at init
 */
gpioDigitalInputHandler _halt_input_handler {
    [](const bool state, const inputEdgeFlag edge, const uint8_t triggering_pin_number) {
        if (edge != INPUT_EDGE_LEADING) { return false; }

        cm_halt();                              // hard stop, including spindle, coolant and heaters

        return false; // allow others to see this notice
    },
    5,    // priority
    nullptr // next - nullptr to start with
};


/*
 * _alarm_input_handler - a gpioDigitalInputHandler to capture pin change events
 *   Will be registered at init
 */
gpioDigitalInputHandler _alarm_input_handler {
    [](const bool state, const inputEdgeFlag edge, const uint8_t triggering_pin_number) {
        if (edge != INPUT_EDGE_LEADING) { return false; }

        char msg[10];
        sprintf(msg, "input %d", triggering_pin_number);
        cm_alarm(STAT_ALARM, msg);

        return false; // allow others to see this notice
    },
    5,    // priority
    nullptr // next - nullptr to start with
};

/*
 * _panic_input_handler - a gpioDigitalInputHandler to capture pin change events
 *   Will be registered at init
 */
gpioDigitalInputHandler _panic_input_handler {
    [](const bool state, const inputEdgeFlag edge, const uint8_t triggering_pin_number) {
        if (edge != INPUT_EDGE_LEADING) { return false; }

        char msg[10];
        sprintf(msg, "input %d", triggering_pin_number);
        cm_panic(STAT_PANIC, msg);

        return false; // allow others to see this notice
    },
    5,    // priority
    nullptr // next - nullptr to start with
};

/*
 * _reset_input_handler - a gpioDigitalInputHandler to capture pin change events
 *   Will be registered at init
 */
gpioDigitalInputHandler _reset_input_handler {
    [](const bool state, const inputEdgeFlag edge, const uint8_t triggering_pin_number) {
        if (edge != INPUT_EDGE_LEADING) { return false; }

        hw_hard_reset();

        return false; // this likely won't be seen, but just in case...
    },
    5,    // priority
    nullptr // next - nullptr to start with
};

/****************************************************************************************
 **** CODE ******************************************************************************
 ****************************************************************************************/

/****************************************************************************************
 **** Initialization ********************************************************************
 ****************************************************************************************/
/*
 * canonical_machine_inits() - combined cm inits
 * canonical_machine_init()  - initialize cm struct
 * canonical_machine_reset() - apply startup settings or reset to startup
 * canonical_machine_reset_rotation()
 */

void canonical_machine_inits()
{
    planner_init(&mp1, &mr1, mp1_queue, PLANNER_QUEUE_SIZE);
    planner_init(&mp2, &mr2, mp2_queue, SECONDARY_QUEUE_SIZE);
    canonical_machine_init(&cm1, &mp1); // primary canonical machine
    canonical_machine_init(&cm2, &mp2); // secondary canonical machine
    cm = &cm1;                          // set global canonical machine pointer to primary machine
    mp = &mp1;                          // set global pointer to the primary planner
    mr = &mr1;                          // and primary runtime

    din_handlers[INPUT_ACTION_STOP].registerHandler(&_hold_input_handler);
    din_handlers[INPUT_ACTION_FAST_STOP].registerHandler(&_hold_input_handler);
    din_handlers[INPUT_ACTION_HALT].registerHandler(&_halt_input_handler);
    din_handlers[INPUT_ACTION_ALARM].registerHandler(&_alarm_input_handler);
    din_handlers[INPUT_ACTION_PANIC].registerHandler(&_panic_input_handler);
    din_handlers[INPUT_ACTION_RESET].registerHandler(&_reset_input_handler);
}

void canonical_machine_init(cmMachine_t *_cm, void *_mp)
{
    // Note cm* was assignd in main()
    // If you can assume all memory has been zeroed by a hard reset you don't need this code:
    memset(_cm, 0, sizeof(cmMachine_t));            // do not reset canonicalMachine once it's been initialized
    memset(&_cm->gm, 0, sizeof(GCodeState_t));      // clear all values, pointers and status

    canonical_machine_init_assertions(_cm);         // establish assertions
    cm_arc_init(_cm);                               // setup arcs. Note: spindle and coolant inits are independent
    _cm->mp = _mp;                                  // point to associated planner
    _cm->am = MODEL;                                // setup initial Gcode model pointer
}

// *** Note: Run canonical_machine_init and profile initializations beforehand ***
void canonical_machine_reset(cmMachine_t *_cm)
{
    // reset canonical machine assertions
    canonical_machine_init_assertions(_cm);

    // set canonical machine gcode defaults
    cm_set_units_mode(cm->default_units_mode);
    cm_set_coord_system(cm->default_coord_system);   // NB: queues a block to the planner with the coordinates
    cm_select_plane(cm->default_select_plane);
    cm_set_path_control(MODEL, cm->default_path_control);
    cm_set_distance_mode(cm->default_distance_mode);
    cm_set_arc_distance_mode(INCREMENTAL_DISTANCE_MODE); // always the default
    cm_set_feed_rate_mode(UNITS_PER_MINUTE_MODE);   // always the default
    cm_reset_overrides();                           // set overrides to initial conditions

    // NOTE: Should unhome axes here
    _cm->homing_state = HOMING_NOT_HOMED;

    // reset request state and flags
    _cm->queue_flush_state = QUEUE_FLUSH_OFF;
    _cm->cycle_start_state = CYCLE_START_OFF;
    _cm->job_kill_state = JOB_KILL_OFF;
    _cm->limit_requested = 0;                       // resets switch closures that occurred during initialization

    // set initial state and signal that the machine is ready for action
    _cm->cycle_type = CYCLE_NONE;
    _cm->motion_state = MOTION_STOP;
    _cm->hold_state = FEEDHOLD_OFF;
    _cm->gmx.block_delete_switch = true;
    _cm->gm.motion_mode = MOTION_MODE_CANCEL_MOTION_MODE; // never start in a motion mode
    _cm->machine_state = MACHINE_READY;

    cm_operation_init();                            // reset operations runner

    canonical_machine_reset_rotation(_cm);
    memset(&_cm->probe_state, 0, sizeof(cmProbeState)*PROBES_STORED);
    memset(&_cm->probe_results, 0, sizeof(float)*PROBES_STORED*AXES);
}

void canonical_machine_reset_rotation(cmMachine_t *_cm) {

    // Make it an identity matrix for no rotation
    memset(&_cm->rotation_matrix, 0, sizeof(float)*3*3);
    _cm->rotation_matrix[0][0] = 1.0;
    _cm->rotation_matrix[1][1] = 1.0;
    _cm->rotation_matrix[2][2] = 1.0;

    // Separately handle a z-offset so that the new plane maintains a consistent
    // distance from the old one. We only need z, since we are rotating to the z axis.
    _cm->rotation_z_offset = 0.0;
}

/****************************************************************************************
 * canonical_machine_init_assertions()
 * canonical_machine_test_assertions() - test assertions, return error code if violation exists
 */

void canonical_machine_init_assertions(cmMachine_t *_cm)
{
    _cm->magic_start = MAGICNUM;
    _cm->magic_end = MAGICNUM;
    _cm->gmx.magic_start = MAGICNUM;
    _cm->gmx.magic_end = MAGICNUM;
    _cm->arc.magic_start = MAGICNUM;
    _cm->arc.magic_end = MAGICNUM;
}

stat_t canonical_machine_test_assertions(cmMachine_t *_cm)
{
    if ((BAD_MAGIC(_cm->magic_start))     || (BAD_MAGIC(_cm->magic_end)) ||
        (BAD_MAGIC(_cm->gmx.magic_start)) || (BAD_MAGIC(_cm->gmx.magic_end)) ||
        (BAD_MAGIC(_cm->arc.magic_start)) || (BAD_MAGIC(_cm->arc.magic_end))) {
        return(cm_panic(STAT_CANONICAL_MACHINE_ASSERTION_FAILURE, "canonical_machine_test_assertions()"));
    }
    return (STAT_OK);
}

/****************************************************************************************
 **** Canonical Machine State Management ************************************************
 ****************************************************************************************/
/*
 * cm_set_motion_state() - adjusts active model pointer as well
 */
void cm_set_motion_state(const cmMotionState motion_state)
{
    cm->motion_state = motion_state;
    ACTIVE_MODEL = ((motion_state == MOTION_STOP) ? MODEL : RUNTIME);
}

/*
 * cm_get_machine_state()
 * cm_get_motion_state()
 * cm_get_cycle_type()
 * cm_get_hold_state()
 * cm_get_homing_state()
 * cm_get_probe_state()
 */
cmMachineState  cm_get_machine_state() { return cm->machine_state;}
cmCycleType     cm_get_cycle_type()    { return cm->cycle_type;}
cmMotionState   cm_get_motion_state()  { return cm->motion_state;}
cmFeedholdState cm_get_hold_state()    { return cm->hold_state;}
cmHomingState   cm_get_homing_state()  { return cm->homing_state;}
cmProbeState    cm_get_probe_state()   { return cm->probe_state[0];}

/*
 * cm_get_combined_state() - combines raw states into something a user might want to see
 */
cmCombinedState cm_get_combined_state(cmMachine_t *_cm)
{
    switch(_cm->machine_state) {
        case MACHINE_INITIALIZING:
        case MACHINE_READY:
        case MACHINE_ALARM:
        case MACHINE_PROGRAM_STOP:
        case MACHINE_PROGRAM_END:     { return ((cmCombinedState)_cm->machine_state); }
        case MACHINE_INTERLOCK:       { return (COMBINED_INTERLOCK); }
        case MACHINE_SHUTDOWN:        { return (COMBINED_SHUTDOWN); }
        case MACHINE_PANIC:           { return (COMBINED_PANIC); }
        case MACHINE_CYCLE: {
            switch(_cm->cycle_type)   {
                case CYCLE_NONE:      { break; } // CYCLE_NONE cannot ever get here
                case CYCLE_MACHINING: { return (_cm->hold_state == FEEDHOLD_OFF ? COMBINED_RUN : COMBINED_HOLD); }
                case CYCLE_HOMING:    { return (COMBINED_HOMING); }
                case CYCLE_PROBE:     { return (COMBINED_PROBE); }
                case CYCLE_JOG:       { return (COMBINED_JOG); }
            }
        }
    }
    cm_panic(STAT_STATE_MANAGEMENT_ASSERTION_FAILURE, "cm_get_combined_state() undefined state");
    return (COMBINED_PANIC);
}

/****************************************************************************************
 **** Model State Getters and Setters ***************************************************
 ****************************************************************************************/
/*  These getters and setters will work on any gm model with inputs:
 *    MODEL         (GCodeState_t *)&cm->gm     // absolute pointer from canonical machine gm model
 *    RUNTIME       (GCodeState_t *)&mr->gm     // absolute pointer from runtime mm struct
 *    ACTIVE_MODEL   cm->am                     // active model pointer is maintained by state management
 */

uint32_t cm_get_linenum(const GCodeState_t *gcode_state) { return gcode_state->linenum;}
cmMotionMode cm_get_motion_mode(const GCodeState_t *gcode_state) { return gcode_state->motion_mode;}
uint8_t cm_get_coord_system(const GCodeState_t *gcode_state) { return gcode_state->coord_system;}
uint8_t cm_get_units_mode(const GCodeState_t *gcode_state) { return gcode_state->units_mode;}
uint8_t cm_get_select_plane(const GCodeState_t *gcode_state) { return gcode_state->select_plane;}
uint8_t cm_get_path_control(const GCodeState_t *gcode_state) { return gcode_state->path_control;}
uint8_t cm_get_distance_mode(const GCodeState_t *gcode_state) { return gcode_state->distance_mode;}
uint8_t cm_get_arc_distance_mode(const GCodeState_t *gcode_state) { return gcode_state->arc_distance_mode;}
uint8_t cm_get_feed_rate_mode(const GCodeState_t *gcode_state) { return gcode_state->feed_rate_mode;}
uint8_t cm_get_tool(const GCodeState_t *gcode_state) { return gcode_state->tool;}
uint8_t cm_get_block_delete_switch() { return cm->gmx.block_delete_switch;}
uint8_t cm_get_runtime_busy() { return (mp_get_runtime_busy());}
float cm_get_feed_rate(const GCodeState_t *gcode_state) { return gcode_state->feed_rate;}

void cm_set_motion_mode(GCodeState_t *gcode_state, const uint8_t motion_mode)
{
    gcode_state->motion_mode = (cmMotionMode)motion_mode;
}

void cm_set_tool_number(GCodeState_t *gcode_state, const uint8_t tool)
{
    gcode_state->tool = tool;
}

void cm_set_absolute_override(GCodeState_t *gcode_state, const uint8_t absolute_override)
{
    gcode_state->absolute_override = (cmAbsoluteOverride)absolute_override;
    cm_set_display_offsets(MODEL);      // must reset offsets if you change absolute override
}

void cm_set_model_linenum(int32_t linenum)
{
    if ((linenum < 0) || (linenum > MAX_LINENUM)) {
        linenum = 0;
        rpt_exception(STAT_INPUT_VALUE_RANGE_ERROR, "line number > 2B or negative; set to zero");
    }
    cm->gm.linenum = linenum;           // you must first set the model line number,
    nv_add_object((const char *)"n");   // then add the line number to the nv list
}

/*
 * cm_check_linenum() - Check line number for Marlin protocol
 */

stat_t cm_check_linenum() {
    if (cm->gmx.last_line_number+1 != cm->gm.linenum) {
        debug_trap("line number out of sequence");
        return STAT_LINE_NUMBER_OUT_OF_SEQUENCE;
    }
    cm->gmx.last_line_number = cm->gm.linenum;
    return STAT_OK;
}

/****************************************************************************************
 * COORDINATE SYSTEMS AND OFFSETS
 * Functions to get, set and report coordinate systems and work offsets
 * These functions are not part of the NIST defined functions
 ****************************************************************************************/
/*
 * cm_get_combined_offset() - return the combined offsets for an axis (G53-G59, G92, Tools)
 * cm_get_display_offset()  - return the current display offset from pecified Gcode model
 * cm_set_display_offsets() - capture combined offsets from the model into absolute values
 *                            in the active Gcode dynamic model
 *
 * Notes on Coordinate System and Offset functions
 *
 * All positional information in the canonical machine is kept as absolute coords and in
 *    canonical units (mm, mm/min). The offsets are only used to translate in and out of
 *    canonical form during incoming processing, and for displays in responses.
 *
 * Managing coordinate systems & offsets is somewhat complicated. The following affect offsets:
 *    - coordinate system selected. 1-6 correspond to G54-G59
 *    - G92 offsets are added "on top of" coord system offsets -- if g92_offset_enable == true
 *    - tool offsets are also accounted for
 *    - absolute override (G53)
 *
 * Position displays {pos:n} are always in work coordinates aka using 'display' offsets
 *    - position displays are assembled by applying all active offsets to the current machine position
 *    - an absolute override forces current move to be interpreted in machine coordinates: G53 (system 0)
 *    - G53 is an explicit absolute override requested by the program, so displays in absolute coords
 *    - G28 and G30 moves are run in absolute coordinates but display using current offsets
 *    - Probing also has a very short move that behaves this way
 *
 * The offsets themselves are "the truth". These are:
 *    - cm.coord_offset[coord][axis]  coordinate offsets for G53-G59, by axis - persistent
 *    - cm.tool_offset[axis]          offsets for currently selected and active tool - persistent
 *    - cm.gmx.g92_offset[axis]       G92 origin offset. Not persistent
 *
 *    - cm_get_combined_offset() puts the above together to provide a combined, active offset.
 *      G92 offsets are only included if g92 is active (gmx.g92_offset_enable == true)
 *
 *  Display offsets
 *      *** Display offsets are for display only and CANNOT be used to set positions ***
 *    - cm_set_display_offsets() writes combined offsets to cm.gm.display_offset[]
 *    - cm_set_display_offsets() should be called every time underlying data would cause a change
 *    - cm_set_display_offsets() takes absolute override display rules into account
 *    - Use cm_get_display_offset() to return the display offset value
 */
/*  Absolute Override is the Gcode G53 convention to allow one and only one Gcode block
 *  to be run in absolute coordinates, regardless of coordinate offsets, G92 offsets, and
 *  tool offsets. See cmAbsoluteOverride for enumerations.
 *
 *    - If absolute_override is set to ABSOLUTE_OVERRIDE_ON_DISPLAY_WITH_OFFSETS
 *      move will run in absolute coordinates but POS will display using current offsets.
 *      This is to support G28 and G30 return moves and other moves that run in absolute
 *      override mode but may want position to be reported using all current offsets
 *
 *    - If absolute_override is set to ABSOLUTE_OVERRIDE_ON_DISPLAY_WITH_NO_OFFSETS
 *      move will run in absolute coordinates and POS will display using no offsets.
 */

float cm_get_combined_offset(const uint8_t axis)
{
    if (cm->gm.absolute_override >= ABSOLUTE_OVERRIDE_ON_DISPLAY_WITH_OFFSETS) {
        return (0);
    }
    float offset = cm->coord_offset[cm->gm.coord_system][axis] + cm->tool_offset[axis];
    if (cm->gmx.g92_offset_enable == true) {
        offset += cm->gmx.g92_offset[axis];
    }
    return (offset);
}

float cm_get_display_offset(const GCodeState_t *gcode_state, const uint8_t axis)
{
    return (gcode_state->display_offset[axis]);
}

void cm_set_display_offsets(GCodeState_t *gcode_state)
{
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {

        // if absolute override is on for G53 so position should be displayed with no offsets
        if (cm->gm.absolute_override == ABSOLUTE_OVERRIDE_ON_DISPLAY_WITH_NO_OFFSETS) {
            gcode_state->display_offset[axis] = 0;
        }

        // all other cases: position should be displayed with currently active offsets
        else {
            gcode_state->display_offset[axis] = cm->coord_offset[cm->gm.coord_system][axis] +
                                                cm->tool_offset[axis];
            if (cm->gmx.g92_offset_enable == true) {
                gcode_state->display_offset[axis] += cm->gmx.g92_offset[axis];
            }
        }
    }

    // If we're not in cycle, then no moves are queued to update the runtime offsets
    // So let's do that
    if ((gcode_state == MODEL) && (cm->machine_state != MACHINE_CYCLE)) {
        mp_set_runtime_display_offset(gcode_state->display_offset);
    }
}

/*
 * cm_get_display_position() - return position in external form from the active Gcode dynamic model
 *
 *    ... that means in prevailing units (mm/inch) and with all offsets applied
 */

float cm_get_display_position(const GCodeState_t *gcode_state, const uint8_t axis)
{
    float position;

    if (gcode_state == MODEL) {
        position = cm->gmx.position[axis] - cm_get_display_offset(MODEL, axis);
    } else {
        position = mp_get_runtime_display_position(axis);
    }
    if (axis <= LAST_LINEAR_AXIS) {   // linears
        if (gcode_state->units_mode == INCHES) {
            position /= MM_PER_INCH;
        }
    }
    return (position);
}

/*
 * cm_get_absolute_position() - get position of axis in absolute coordinates from the active Gcode dynamic model
 *
 *      ... machine position is always returned in mm mode. No units conversion is performed
 */

float cm_get_absolute_position(const GCodeState_t *gcode_state, const uint8_t axis)
{
    if (gcode_state == MODEL) {
        return (cm->gmx.position[axis]);
    }
    return (mp_get_runtime_absolute_position(mr, axis));
}

/****************************************************************************************
 **** CRITICAL HELPERS ******************************************************************
 ****************************************************************************************
 * Core functions supporting the canonical machining functions
 * These functions are not part of the NIST defined functions
 ****************************************************************************************/

/****************************************************************************************
 * cm_update_model_position() - set gmx endpoint position for a traverse, feed or arc
 *
 *  Note: As far as the canonical machine is concerned the final position of a Gcode block
 *  (move) is achieved as soon as the move is planned and the move target becomes the new
 *  model position. In reality the planner will (in all likelihood) have only just queued
 *  the move for later execution, and the real tool position is still close to the starting
 *  point.
 */

void cm_update_model_position()
{
    copy_vector(cm->gmx.position, cm->gm.target);   // would be mr->gm.target if from runtime
}

/****************************************************************************************
 * cm_deferred_write_callback() - write any changed G10 values back to persistence
 *
 *  Only runs if there is G10 data to write, there is no movement, and the serial queues
 *  are quiescent.
 */

stat_t cm_deferred_write_callback()
{
    if ((cm->cycle_type == CYCLE_NONE) && (cm->deferred_write_flag == true)) {
        cm->deferred_write_flag = false;
        nvObj_t nv;
        for (uint8_t i=1; i<=COORDS; i++) {
            for (uint8_t j=0; j<AXES; j++) {
#if (AXES == 9)
                sprintf((char *)nv.token, "g%2d%c", 53+i, ("xyzuvwabc")[j]);
#else
                sprintf((char *)nv.token, "g%2d%c", 53+i, ("xyzabc")[j]);
#endif
                nv.index = nv_get_index((const char *)"", nv.token);
                nv.value_flt = cm->coord_offset[i][j];
                nv_persist(&nv);    // Note: nv_persist() only writes values that have changed
            }
        }
    }
    return (STAT_OK);
}

/****************************************************************************************
 * cm_get_tram() - JSON query to determine if the rotation matrix is set (non-identity)
 * cm_set_tram() - JSON command to trigger computing the rotation matrix
 *
 * For set_tram there MUST be three valid probes stored.
 */

stat_t cm_get_tram(nvObj_t *nv)
{
    nv->value_int = true;   // believe it or not, the compiler likes this form best - most efficient code

    if (fp_NOT_ZERO(cm->rotation_z_offset) ||
    fp_NOT_ZERO(cm->rotation_matrix[0][1]) ||
    fp_NOT_ZERO(cm->rotation_matrix[0][2]) ||
    fp_NOT_ZERO(cm->rotation_matrix[1][0]) ||
    fp_NOT_ZERO(cm->rotation_matrix[1][2]) ||
    fp_NOT_ZERO(cm->rotation_matrix[2][0]) ||
    fp_NOT_ZERO(cm->rotation_matrix[2][1]) ||
    fp_NE(1.0,  cm->rotation_matrix[0][0]) ||
    fp_NE(1.0,  cm->rotation_matrix[1][1]) ||
    fp_NE(1.0,  cm->rotation_matrix[2][2]))
    {
        nv->value_int = false;
    }
    nv->valuetype = TYPE_BOOLEAN;
    return (STAT_OK);
}

stat_t cm_set_tram(nvObj_t *nv)
{
    if (!nv->value_int) {                       // if false, reset the matrix and return
        canonical_machine_reset_rotation(cm);
        return (STAT_OK);
    }

    // check to make sure we have three valid probes in a row
    if (!((cm->probe_state[0] == PROBE_SUCCEEDED) &&
          (cm->probe_state[1] == PROBE_SUCCEEDED) &&
          (cm->probe_state[2] == PROBE_SUCCEEDED))) {
            return (STAT_COMMAND_NOT_ACCEPTED);     // do not have 3 valid probes
    }

    // Step 1: Get the normal of the plane formed by the three probes. Naming:
    //    d0_{xyz} is the delta between point 0 and point 1
    //    d2_{xyz} is the delta between point 2 and point 1
    //    n_{xyz} is the unit normal

    // Step 1a: get the deltas
    float d0_x = cm->probe_results[0][0] - cm->probe_results[1][0];
    float d0_y = cm->probe_results[0][1] - cm->probe_results[1][1];
    float d0_z = cm->probe_results[0][2] - cm->probe_results[1][2];
    float d2_x = cm->probe_results[2][0] - cm->probe_results[1][0];
    float d2_y = cm->probe_results[2][1] - cm->probe_results[1][1];
    float d2_z = cm->probe_results[2][2] - cm->probe_results[1][2];

    // Step 1b: compute the combined magnitude
    // since sqrt(a)*sqrt(b) = sqrt(a*b), we can save a sqrt in making the unit normal
    float combined_magnitude_inv = 1.0/sqrt((d0_x*d0_x + d0_y*d0_y + d0_z*d0_z) *
                                            (d2_x*d2_x + d2_y*d2_y + d2_z*d2_z));

    // Step 1c: compute the cross product and normalize
    float n_x = (d0_z*d2_y - d0_y*d2_z) * combined_magnitude_inv;
    float n_y = (d0_x*d2_z - d0_z*d2_x) * combined_magnitude_inv;
    float n_z = (d0_y*d2_x - d0_x*d2_y) * combined_magnitude_inv;

    // Step 1d: flip the normal if it's negative
    if (n_z < 0.0) {
        n_x = -n_x;
        n_y = -n_y;
        n_z = -n_z;
    }

    // Step 2: make the quaternion for the rotation to {0,0,1}
    float p = sqrt(n_x*n_x + n_y*n_y + n_z*n_z);
    float m = sqrt(2.0)*sqrt(p*(p+n_z));
    float q_w = (n_z + p) / m;
    float q_x = -n_y / m;
    float q_y = n_x / m;
    //float q_z = 0; // already optimized out

    // Step 3: compute the rotation matrix
    float q_wx_2 = q_w * q_x * 2.0;
    float q_wy_2 = q_w * q_y * 2.0;
    float q_xx_2 = q_x * q_x * 2.0;
    float q_xy_2 = q_x * q_y * 2.0;
    float q_yy_2 = q_y * q_y * 2.0;

    /*
        matrix = {
                 {1 - q_yy_2,   q_xy_2,        q_wy_2,               0},
                 {q_xy_2,       1 - q_xx_2,   -q_wx_2,               0},
                 {-q_wy_2,      q_wx_2,       1 - q_xx_2 - q_yy_2,   i},
                 {0,            0,            0,                     1}
                 }
    */
    cm->rotation_matrix[0][0] = 1 - q_yy_2;
    cm->rotation_matrix[0][1] = q_xy_2;
    cm->rotation_matrix[0][2] = q_wy_2;

    cm->rotation_matrix[1][0] = q_xy_2;
    cm->rotation_matrix[1][1] = 1 - q_xx_2;
    cm->rotation_matrix[1][2] = -q_wx_2;

    cm->rotation_matrix[2][0] = -q_wy_2;
    cm->rotation_matrix[2][1] = q_wx_2;
    cm->rotation_matrix[2][2] = 1 - q_xx_2 - q_yy_2;

    // Step 4: compute the z-offset
    cm->rotation_z_offset = (n_x*cm->probe_results[1][0] +
                             n_y*cm->probe_results[1][1]) /
                             n_z + cm->probe_results[1][2];
    return (STAT_OK);
}

/****************************************************************************************
 * cm_set_nxt_line() - JSON command to set the next line number
 * cm_get_nxt_line() - JSON query to get the next expected line number
 */

stat_t cm_set_nxln(nvObj_t *nv)
{
    if (nv->valuetype == TYPE_INTEGER || nv->valuetype == TYPE_FLOAT)
    {
        cm->gmx.last_line_number = nv->value_int - 1;
        return (STAT_OK);
    }
    return (STAT_INPUT_VALUE_RANGE_ERROR);
}

stat_t cm_get_nxln(nvObj_t *nv)
{
    nv->value_int = cm->gmx.last_line_number+1;
    nv->valuetype = TYPE_INTEGER;
    return (STAT_OK);
}

/****************************************************************************************
 * cm_set_model_target() - set target vector in GM model
 *
 * This is a core routine. It handles:
 *    - conversion of linear units to internal canonical form (mm)
 *    - conversion of relative mode to absolute (internal canonical form)
 *    - translation of work coordinates to machine coordinates (internal canonical form)
 *    - computation and application of axis modes as so:
 *
 *    DISABLED  - Incoming value is ignored. Target value is not changed
 *    ENABLED   - Convert axis values to canonical format and store as target
 *    INHIBITED - Same processing as ENABLED, but axis will not actually be run
 *    RADIUS    - ABC axis value is provided in Gcode block in linear units
 *              - Target is set to degrees based on axis' Radius value
 *              - Radius mode is only processed for ABC axes. Application to XYZ is ignored.
 *
 *  Target coordinates are provided in target[]
 *  Axes that need processing are signaled in flag[]
 */

// ESTEE: _calc_ABC is a fix to workaround a gcc compiler bug wherein it runs out of spill
//        registers we moved this block into its own function so that we get a fresh stack push
// ALDEN: This shows up in avr-gcc 4.7.0 and avr-libc 1.8.0

static float _calc_ABC(const uint8_t axis, const float target[])
{
    if ((cm->a[axis].axis_mode == AXIS_STANDARD) || (cm->a[axis].axis_mode == AXIS_INHIBITED)) {
        return(target[axis]);    // no mm conversion - it's in degrees
    }
    // radius mode
    return (_to_millimeters(target[axis]) * 360.0 / (2 * M_PI * cm->a[axis].radius));
}

void cm_set_model_target(const float target[], const bool flags[])
{
    uint8_t axis;
    float tmp = 0;

    // copy position to target so it always starts correctly
    copy_vector(cm->gm.target, cm->gmx.position);

    // process linear axes (XYZUVW) first
    for (axis=AXIS_X; axis<=LAST_LINEAR_AXIS; axis++) {
        if (!flags[axis] || cm->a[axis].axis_mode == AXIS_DISABLED) {
            continue;        // skip axis if not flagged for update or its disabled
        } else if ((cm->a[axis].axis_mode == AXIS_STANDARD) || (cm->a[axis].axis_mode == AXIS_INHIBITED)) {
            if (cm->gm.distance_mode == ABSOLUTE_DISTANCE_MODE) {
                cm->gm.target[axis] = cm_get_combined_offset(axis) + _to_millimeters(target[axis]);
            } else {
                cm->gm.target[axis] += _to_millimeters(target[axis]);
            }
            cm->return_flags[axis] = true;  // used to make a synthetic G28/G30 intermediate move
        }
    }
    // FYI: The ABC loop below relies on the XYZUVW loop having been run first
    for (axis=AXIS_A; axis<=AXIS_C; axis++) {
        if (!flags[axis] || cm->a[axis].axis_mode == AXIS_DISABLED) {
            continue;        // skip axis if not flagged for update or its disabled
        } else {
            tmp = _calc_ABC(axis, target);
        }

#if MARLIN_COMPAT_ENABLED == true
        // If we are in absolute mode (generally), but the extruder is relative,
        // then we adjust the extruder to a relative position
        if (mst.marlin_flavor && (cm->a[axis].axis_mode == AXIS_RADIUS)) {
            if ((cm->gm.distance_mode == INCREMENTAL_DISTANCE_MODE) || (mst.extruder_mode == EXTRUDER_MOVES_RELATIVE)) {
                cm->gm.target[axis] += tmp;
            }
            else { // if (cm.gmx.extruder_mode == EXTRUDER_MOVES_NORMAL)
                cm->gm.target[axis] = tmp + cm_get_combined_offset(axis);
            }
            // TODO - volumetric filament conversion
//            else {
//                cm->gm.target[axis] += tmp * cm.gmx.volume_to_filament_length[axis-3];
//            }
        }
        else
#endif // MARLIN_COMPAT_ENABLED

        if (cm->gm.distance_mode == ABSOLUTE_DISTANCE_MODE) {
            cm->gm.target[axis] = tmp + cm_get_combined_offset(axis); // sacidu93's fix to Issue #22
        }
        else {
            cm->gm.target[axis] += tmp;
        }
        cm->return_flags[axis] = true;
    }
}

/****************************************************************************************
 * cm_get_soft_limits()
 * cm_set_soft_limits()
 * cm_test_soft_limits() - return error code if soft limit is exceeded
 *
 *  The target[] arg must be in absolute machine coordinates. Best done after cm_set_model_target().
 *
 *  Tests for soft limit for any homed axis if min and max are different values. You can set min
 *  and max to the same value (e.g. 0,0) to disable soft limits for an axis. Also will not test
 *  a min or a max if the value is more than +/- 1000000 (plus or minus 1 million ).
 *  This allows a single end to be tested w/the other disabled, should that requirement ever arise.
 */

bool cm_get_soft_limits() { return (cm->soft_limit_enable); }
void cm_set_soft_limits(bool enable) { cm->soft_limit_enable = enable; }

static stat_t _finalize_soft_limits(const stat_t status)
{
    cm->gm.motion_mode = MOTION_MODE_CANCEL_MOTION_MODE;    // cancel motion
    copy_vector(cm->gm.target, cm->gmx.position);           // reset model target
    return (cm_alarm(status, "soft_limits"));               // throw an alarm
}

stat_t cm_test_soft_limits(const float target[])
{
    if (cm->soft_limit_enable == true) {
        for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
            if (cm->homed[axis] != true) { continue; }                               // skip axis if not homed
            if (fp_EQ(cm->a[axis].travel_min, cm->a[axis].travel_max)) { continue; } // skip axis if identical
            if (fabs(cm->a[axis].travel_min) > DISABLE_SOFT_LIMIT) { continue; }     // skip min test if disabled
            if (fabs(cm->a[axis].travel_max) > DISABLE_SOFT_LIMIT) { continue; }     // skip max test if disabled

            if (target[axis] < cm->a[axis].travel_min) {
                return (_finalize_soft_limits(STAT_SOFT_LIMIT_EXCEEDED_XMIN + 2*axis));
            }
            if (target[axis] > cm->a[axis].travel_max) {
                return (_finalize_soft_limits(STAT_SOFT_LIMIT_EXCEEDED_XMAX + 2*axis));
            }
        }
    }
    return (STAT_OK);
}

/****************************************************************************************
 **** CANONICAL MACHINING FUNCTIONS *****************************************************
 ****************************************************************************************
 *  Organized by section number in the order they are found in NIST RS274 NGCv3
 ***************************************************************************************/

/****************************************************************************************
 **** Representation (4.3.3) ************************************************************
 ****************************************************************************************/

/*****************************************************************************************
 * Representation functions that affect the Gcode model only (asynchronous)
 *
 *  cm_select_plane()           - G17,G18,G19 select axis plane
 *  cm_set_units_mode()         - G20, G21
 *  cm_set_distance_mode()      - G90, G91
 *  cm_set_arc_distance_mode()  - G90.1, G91.1
 *  cm_set_g10_data()           - G10 (delayed persistence)
 *
 *  These functions assume input validation occurred upstream, most likely in gcode parser.
 */

stat_t cm_select_plane(const uint8_t plane)
{
    cm->gm.select_plane = (cmCanonicalPlane)plane;
    return (STAT_OK);
}

stat_t cm_set_units_mode(const uint8_t mode)
{
    cm->gm.units_mode = (cmUnitsMode)mode;               // 0 = inches, 1 = mm.
    return(STAT_OK);
}

stat_t cm_set_distance_mode(const uint8_t mode)
{
    cm->gm.distance_mode = (cmDistanceMode)mode;         // 0 = absolute mode, 1 = incremental
    return (STAT_OK);
}

stat_t cm_set_arc_distance_mode(const uint8_t mode)
{
    cm->gm.arc_distance_mode = (cmDistanceMode)mode;     // 0 = absolute mode, 1 = incremental
    return (STAT_OK);
}

/****************************************************************************************
 * cm_set_g10_data() - G10 L1/L2/L10/L20 Pn (affects MODEL only)
 *
 *  This function applies the offset to the GM model but does not persist the offsets
 *  during the Gcode cycle. The persist flag is used to persist offsets once the cycle
 *  has ended. You can also use $g54x - $g59c config functions to change offsets.
 *  It also does resets the display offsets to reflect the new values.
 */

stat_t cm_set_g10_data(const uint8_t P_word, const bool P_flag,
                       const uint8_t L_word, const bool L_flag,
                       const float offset[], const bool flag[])
{
    if (!L_flag) {
        return (STAT_L_WORD_IS_MISSING);
    }

    if ((L_word == 2) || (L_word == 20)) {
        // coordinate system offset command
        if ((P_word < G54) || (P_word > COORD_SYSTEM_MAX)) {
            return (STAT_P_WORD_IS_INVALID);                // you can't set G53
        }
        for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
            if (flag[axis]) {
                if (L_word == 2) {
                    cm->coord_offset[P_word][axis] = _to_millimeters(offset[axis]);
                } else {
                    // Should L20 take into account G92 offsets?
                    cm->coord_offset[P_word][axis] = cm->gmx.position[axis] -
                        _to_millimeters(offset[axis]) -
                        cm->tool_offset[axis];
                }
                cm->deferred_write_flag = true;         // persist offsets once machining cycle is over
            }
        }
    }
    else if ((L_word == 1) || (L_word == 10)) {
        if ((P_word < 1) || (P_word > TOOLS)) {         // tool table offset command. L11 not supported atm.
            return (STAT_P_WORD_IS_INVALID);
        }
        for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
            if (flag[axis]) {
                if (L_word == 1) {
                    tt.tt_offset[P_word][axis] = _to_millimeters(offset[axis]);
                } else {                                // L10 should also take into account G92 offset
                    tt.tt_offset[P_word][axis] =
                        cm->gmx.position[axis] - _to_millimeters(offset[axis]) -
                        cm->coord_offset[cm->gm.coord_system][axis] -
                        (cm->gmx.g92_offset[axis] * cm->gmx.g92_offset_enable);
                }
                cm->deferred_write_flag = true;         // persist offsets once machining cycle is over
            }
        }
    }
    else {
        return (STAT_L_WORD_IS_INVALID);
    }
    cm_set_display_offsets(MODEL);
    return (STAT_OK);
}

/******************************************************************************************
 * Representation functions that affect gcode model and are queued to planner (synchronous)
 *
 * cm_set_tl_offset()    - G43
 * cm_cancel_tl_offset() - G49
 * cm_set_coord_system() - G54-G59
 */

stat_t cm_set_tl_offset(const uint8_t H_word, const bool H_flag, const bool apply_additional)
{
    uint8_t tool;
    if (H_flag) {
        if (H_word > TOOLS) {
            return (STAT_H_WORD_IS_INVALID);
        }
        if (H_word == 0) {    // interpret H0 as "current tool", just like no H at all.
            tool = cm->gm.tool;
        } else {
            tool = H_word;
        }
    } else {
        tool = cm->gm.tool;
    }
    if (apply_additional) {
        for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
            cm->tool_offset[axis] += tt.tt_offset[tool][axis];
        }
        } else {
        for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
            cm->tool_offset[axis] = tt.tt_offset[tool][axis];
        }
    }
    cm_set_display_offsets(MODEL);                      // display new offsets in the model right now
    return (STAT_OK);
}

stat_t cm_cancel_tl_offset()
{
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        cm->tool_offset[axis] = 0;
    }
    cm_set_display_offsets(MODEL);                      // display new offsets in the model right now
   return (STAT_OK);
}

stat_t cm_set_coord_system(const uint8_t coord_system)  // set coordinate system sync'd with planner
{
    cm->gm.coord_system = (cmCoordSystem)coord_system;
    cm_set_display_offsets(MODEL);                      // must reset display offsets if you change coordinate system
    return (STAT_OK);
}

/******************************************************************************************
 * cm_set_position_by_axis() - set the position of a single axis in the model, planner and runtime
 * cm_reset_position_to_absolute_position() - set all positions to current absolute position in mr
 *
 *  This command sets an axis/axes to a position provided as an argument.
 *  This is useful for setting origins for homing, probing, and other operations.
 *
 *  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *  !!!!! DO NOT CALL THESE FUNCTIONS WHILE IN A MACHINING CYCLE !!!!!
 *  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 *  More specifically, do not call these functions if there are any moves in the planner or
 *  if the runtime is moving. The system must be quiescent or you will introduce positional
 *  errors. This is true because the planned / running moves have a different reference frame
 *  than the one you are now going to set. These functions should only be called during
 *  initialization sequences and during cycles (such as homing cycles) when you know there
 *  are no more moves in the planner and that all motion has stopped.
 *  You can use cm_get_runtime_busy() to be sure the system is quiescent.
 *
 *  TODO: Turn this into a queued command so it executes from the planner
 */

void cm_set_position_by_axis(const uint8_t axis, const float position)
{
    cm->gmx.position[axis] = position;
    cm->gm.target[axis] = position;
    mp_set_planner_position(axis, position);
    mp_set_runtime_position(axis, position);
    mp_set_steps_to_runtime_position();
}

void cm_reset_position_to_absolute_position(cmMachine_t *_cm)
{
    mpPlanner_t *_mp = (mpPlanner_t *)_cm->mp;
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        cm_set_position_by_axis(axis, mp_get_runtime_absolute_position(_mp->mr, axis));
    }
}

/*
 * cm_set_absolute_origin() - G28.3 - model, planner and queue to runtime
 * _exec_absolute_origin()  - callback from planner
 *
 *  cm_set_absolute_origin() takes a vector of origins (presumably 0's, but not necessarily)
 *  and applies them to all axes where the corresponding position in the flag vector is true (1).
 *
 *  This is a 2 step process. The model and planner contexts are set immediately, the runtime
 *  command is queued and synchronized with the planner queue. This includes the runtime position
 *  and the step recording done by the encoders. At that point any axis that is set is also marked
 *  as homed.
 */

static void _exec_absolute_origin(float *value, bool *flag)
{
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        if (flag[axis]) {
            mp_set_runtime_position(axis, value[axis]);
            cm->homed[axis] = true;    // G28.3 is not considered homed until you get here
        }
    }
    mp_set_steps_to_runtime_position();
}

stat_t cm_set_absolute_origin(const float origin[], bool flag[])
{
    float value[AXES];

    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        if (flag[axis]) {
// REMOVED  value[axis] = cm->offset[cm->gm.coord_system][axis] + _to_millimeters(origin[axis]);    // G2 Issue #26
            value[axis] = _to_millimeters(origin[axis]);    // replaced the above
            cm->gmx.position[axis] = value[axis];           // set model position
            cm->gm.target[axis] = value[axis];              // reset model target
            mp_set_planner_position(axis, value[axis]);     // set mm position
        }
    }
    mp_queue_command(_exec_absolute_origin, value, flag);
    return (STAT_OK);
}

/******************************************************************************************
 * cm_set_g92_offsets()     - G92
 * cm_reset_g92_offsets()   - G92.1
 * cm_suspend_g92_offsets() - G92.2
 * cm_resume_g92_offsets()  - G92.3
 *
 * G92's behave according to NIST 3.5.18 & LinuxCNC G92
 * http://linuxcnc.org/docs/html/gcode/gcode.html#sec:G92-G92.1-G92.2-G92.3
 */

stat_t cm_set_g92_offsets(const float offset[], const bool flag[])
{
    // set offsets in the Gcode model extended context
    cm->gmx.g92_offset_enable = true;
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        if (flag[axis]) {
            cm->gmx.g92_offset[axis] = cm->gmx.position[axis] -
                                       cm->coord_offset[cm->gm.coord_system][axis] -
                                       cm->tool_offset[axis] -
                                       _to_millimeters(offset[axis]);
        }
    }
    // now pass the offset to the callback - setting the coordinate system also applies the offsets
    cm_set_display_offsets(MODEL);
    return (STAT_OK);
}

stat_t cm_reset_g92_offsets()
{
    cm->gmx.g92_offset_enable = false;
    for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
        cm->gmx.g92_offset[axis] = 0;
    }
    cm_set_display_offsets(MODEL);
    return (STAT_OK);
}

stat_t cm_suspend_g92_offsets()
{
    cm->gmx.g92_offset_enable = false;
    cm_set_display_offsets(MODEL);
    return (STAT_OK);
}

stat_t cm_resume_g92_offsets()
{
    cm->gmx.g92_offset_enable = true;
    cm_set_display_offsets(MODEL);
    return (STAT_OK);
}

/****************************************************************************************
 **** Free Space Motion (4.3.4) *********************************************************
 ****************************************************************************************/
/*
 * cm_straight_traverse() - G0 linear rapid
 */

stat_t cm_straight_traverse(const float *target, const bool *flags, const cmMotionProfile motion_profile)
{
    cm->gm.motion_mode = MOTION_MODE_STRAIGHT_TRAVERSE;
#ifdef TRAVERSE_AT_HIGH_JERK
    cm->gm.motion_profile = PROFILE_FAST; // override to make all traverses use high jerk
#else
    cm->gm.motion_profile = motion_profile;
#endif

    // it's legal for a G0 to have no axis words but we don't want to process it
    if (!(flags[AXIS_X] | flags[AXIS_Y] | flags[AXIS_Z] |
#if (AXES == 9)
          flags[AXIS_U] | flags[AXIS_V] | flags[AXIS_W] |
#endif
          flags[AXIS_A] | flags[AXIS_B] | flags[AXIS_C])) {
        return(STAT_OK);
    }
    cm_set_model_target(target, flags);
    ritorno(cm_test_soft_limits(cm->gm.target));  // test soft limits; exit if thrown
    cm_set_display_offsets(MODEL);                // capture the fully resolved offsets to the state
    cm_cycle_start();                             // required here for homing & other cycles
    stat_t status = mp_aline(MODEL);              // send the move to the planner
    cm_update_model_position();                   // update gmx.position to ready for next incoming move

    if (status == STAT_MINIMUM_LENGTH_MOVE) {
        if (!mp_has_runnable_buffer(mp) &&
            !st_runtime_isbusy()) {  // handle condition where zero-length move is last or only move
            cm_cycle_end();          // ...otherwise cycle will not end properly
        }
        status = STAT_OK;
    }
    return (status);
}

/****************************************************************************************
 * cm_goto_g28_position()  - G28
 * cm_set_g28_position()   - G28.1
 * cm_goto_g30_position()  - G30
 * cm_set_g30_position()   - G30.1
 * _goto_stored_position() - helper
 */

stat_t _goto_stored_position(const float stored_position[],     // always in mm
                             const float intermediate_target[], // in current units (G20/G21)
                             const bool flags[])                // all false if no intermediate move
{
    // Go through intermediate point if one is provided
    while (mp_planner_is_full(mp));                             // Make sure you have available buffers
    ritorno(cm_straight_traverse(intermediate_target, flags, PROFILE_NORMAL));  // w/no action if no axis flags

    // If G20 adjust stored position (always in mm) to inches so traverse will be correct
    float target[AXES]; // make a local stored position as it may be modified
    copy_vector(target, stored_position);

    if (cm->gm.units_mode == INCHES) {
        for (uint8_t i=0; i<AXIS_A; i++) {                  // Only convert linears (not rotaries)
            target[i] *= INCHES_PER_MM;
        }
    }

    // Run the stored position move
    while (mp_planner_is_full(mp));                         // Make sure you have available buffers

    uint8_t saved_distance_mode = cm_get_distance_mode(MODEL);
    cm_set_absolute_override(MODEL, ABSOLUTE_OVERRIDE_ON_DISPLAY_WITH_OFFSETS);  // Position stored in abs coords
    cm_set_distance_mode(ABSOLUTE_DISTANCE_MODE);           // Must run in absolute distance mode

    bool flags2[] = INIT_AXES_TRUE;
    stat_t status = cm_straight_traverse(target, flags2, PROFILE_NORMAL);   // Go to stored position
    cm_set_absolute_override(MODEL, ABSOLUTE_OVERRIDE_OFF);
    cm_set_distance_mode(saved_distance_mode);              // Restore distance mode
    return (status);
}

stat_t cm_set_g28_position(void)
{
    copy_vector(cm->gmx.g28_position, cm->gmx.position);    // in MM and machine coordinates
    return (STAT_OK);
}

stat_t cm_goto_g28_position(const float target[], const bool flags[])
{
    return (_goto_stored_position(cm->gmx.g28_position, target, flags));
}

stat_t cm_set_g30_position(void)
{
    copy_vector(cm->gmx.g30_position, cm->gmx.position);    // in MM and machine coordinates
    return (STAT_OK);
}

stat_t cm_goto_g30_position(const float target[], const bool flags[])
{
    return (_goto_stored_position(cm->gmx.g30_position, target, flags));
}

/****************************************************************************************
 **** Machining Attributes (4.3.5) ******************************************************
 ****************************************************************************************/
/*
 * cm_set_feed_rate() - F parameter (affects MODEL only)
 *
 * Normalize feed rate to mm/min or to minutes if in inverse time mode
 */

stat_t cm_set_feed_rate(const float feed_rate)
{
    if (cm->gm.feed_rate_mode == INVERSE_TIME_MODE) {
        if (fp_ZERO(feed_rate)) {
            return (STAT_FEEDRATE_NOT_SPECIFIED);
        }
        cm->gm.feed_rate = 1/feed_rate;    // normalize to minutes (NB: active for this gcode block only)
    } else {
        cm->gm.feed_rate = _to_millimeters(feed_rate);
    }
    return (STAT_OK);
}

/****************************************************************************************
 * cm_set_feed_rate_mode() - G93, G94 (affects MODEL only)
 *
 *  INVERSE_TIME_MODE = 0,          // G93
 *  UNITS_PER_MINUTE_MODE,          // G94
 *  UNITS_PER_REVOLUTION_MODE       // G95 (unimplemented)
 */

stat_t cm_set_feed_rate_mode(const uint8_t mode)
{
    cm->gm.feed_rate_mode = (cmFeedRateMode)mode;
    return (STAT_OK);
}

/****************************************************************************************
 * cm_set_path_control() - G61, G61.1, G64
 */

stat_t cm_set_path_control(GCodeState_t *gcode_state, const uint8_t mode)
{
    gcode_state->path_control = (cmPathControl)mode;
    return (STAT_OK);
}

/****************************************************************************************
 **** Machining Functions (4.3.6) *******************************************************
 ****************************************************************************************/
/*
 * cm_arc_feed() - SEE plan_arc.cpp
 */

/****************************************************************************************
 * cm_dwell() - G4, P parameter (seconds)
 */
stat_t cm_dwell(const float seconds)
{
    cm->gm.P_word = seconds;
    mp_dwell(seconds);
    return (STAT_OK);
}

/****************************************************************************************
 * cm_straight_feed() - G1
 */

stat_t cm_straight_feed(const float *target, const bool *flags, const cmMotionProfile motion_profile)
{
    // trap zero feed rate condition
    if (fp_ZERO(cm->gm.feed_rate)) {
        return (STAT_FEEDRATE_NOT_SPECIFIED);
    }
    cm->gm.motion_mode = MOTION_MODE_STRAIGHT_FEED;

    // it's legal for a G0 to have no axis words but we don't want to process it
    if (!(flags[AXIS_X] | flags[AXIS_Y] | flags[AXIS_Z] |
#if (AXES == 9)
          flags[AXIS_U] | flags[AXIS_V] | flags[AXIS_W] |
#endif
          flags[AXIS_A] | flags[AXIS_B] | flags[AXIS_C])) {
        return(STAT_OK);
    }

    cm_set_model_target(target, flags);
    ritorno(cm_test_soft_limits(cm->gm.target));  // test soft limits; exit if thrown
    cm_set_display_offsets(MODEL);                // capture the fully resolved offsets to the state
    cm_cycle_start();                             // required for homing & other cycles
    stat_t status = mp_aline(MODEL);              // send the move to the planner
    cm_update_model_position();                   // <-- ONLY safe because we don't care about status...

    if (status == STAT_MINIMUM_LENGTH_MOVE) {
        if (!mp_has_runnable_buffer(mp) &&
            !st_runtime_isbusy()) {  // handle condition where zero-length move is last or only move
            cm_cycle_end();          // ...otherwise cycle will not end properly
        }
        status = STAT_OK;
    }
    return (status);
}

/****************************************************************************************
 **** Spindle Functions (4.3.7) *********************************************************
 ****************************************************************************************/
// see spindle.cpp/.h

/****************************************************************************************
 **** Tool Functions (4.3.8) ************************************************************
 ****************************************************************************************/
/*
 * cm_select_tool()     - T parameter
 * _exec_select_tool()  - execution callback
 *
 * cm_change_tool()     - M6 (This might become a complete tool change cycle)
 * _exec_change_tool()  - execution callback
 *
 * Note: These functions don't actually do anything for now, and there's a bug
 *       where T and M in different blocks don't work correctly
 */
static void _exec_select_tool(float *value, bool *flag)
{
    cm->gm.tool_select = (uint8_t)value[0];
}

stat_t cm_select_tool(const uint8_t tool_select)
{
    if (tool_select > TOOLS) {
        return (STAT_T_WORD_IS_INVALID);
    }
    float value[] = { (float)tool_select };
    mp_queue_command(_exec_select_tool, value, nullptr);
    return (STAT_OK);
}

static void _exec_change_tool(float *value, bool *flag)
{
    cm->gm.tool = cm->gm.tool_select;

    spindle_set_toolhead(toolhead_for_tool(cm->gm.tool));
    // TODO - change tool offsets and update display offsets
}

stat_t cm_change_tool(const uint8_t tool_change)
{
    mp_queue_command(_exec_change_tool, nullptr, nullptr);
    return (STAT_OK);
}

/****************************************************************************************
 **** Miscellaneous Functions (4.3.9) ***************************************************
 ****************************************************************************************/
// see coolant.cpp/.h

/****************************************************************************************
 * cm_message() - queue a RAM string as a message in the response (unconditionally)
 */

void cm_message(const char *message)
{
    nv_add_string((const char *)"msg", message);    // add message to the response object
}

/****************************************************************************************
 **** Overrides *************************************************************************
 ****************************************************************************************/

/****************************************************************************************
 * cm_reset_overrides() - reset manual feedrate and spindle overrides to initial conditions
 */

void cm_reset_overrides()
{
    cm->gmx.m48_enable = true;
    cm->gmx.mfo_enable = true;  // feed rate overrides
    cm->gmx.mfo_factor = 1.0;
    cm->gmx.mto_enable = true;  // traverse overrides
    cm->gmx.mto_factor = 1.0;
}

/****************************************************************************************
 * cm_m48_enable() - M48, M49
 *
 * M48 is the master enable for manual feedrate override and spindle override
 * If M48 is asserted M50 (mfo), M50.1 (mto) and M51 (spo) settings are in effect
 * If M49 is asserted M50 (mfo), M50.1 (mto) and M51 (spo) settings are in ignored
 *
 * See http://linuxcnc.org/docs/html/gcode/m-code.html#sec:M48,-M49-Speed-and-Feed-Override-Control
 */
/*  M48state    M48new      M50state    action (notes):
 *  disable     disable     disable     no action, no state change
 *  disable     disable     ENABLE      no action, no state change
 *
 *  disable     ENABLE      disable     no action, no state change
 *  disable     ENABLE      ENABLE      start ramp w/stored P value
 *
 *  ENABLE      disable     disable     no action, no state change
 *  ENABLE      disable     ENABLE      end ramp
 *
 *  ENABLE      ENABLE      disable     no action, no state change
 *  ENABLE      ENABLE      ENABLE      no action, no state change
 */
stat_t cm_m48_enable(uint8_t enable)        // M48, M49
{
    // handle changes to feed override given new state of m48/m49

    cm->gmx.m48_enable = enable;             // update state
    return (STAT_OK);
}

/****************************************************************************************
 * cm_fro_control() - M50 feed rate override comtrol
 * cm_tro_control() - M50.1 traverse override comtrol
 *
 *  M50 enables manual feedrate override and the optional P override parameter.
 *  P is expressed as M% to N% of programmed feedrate, typically a value from 0.05 to 2.000.
 *  P may also be zero or missing. Behaviors:
 *
 *    P < minimum or P > maximum parameter, and not zero. Return error, no state change or action
 *    P omitted. Turn on feedrate override to current stored P value
 *    P = 0. Turn off feedrate override. (Do not change stored P value
 *    P = N. Turn on feedrate override to value of N, preserve new P value
 * See http://www.linuxcnc.org/docs/2.4/html/gcode_main.html#sec:M50:-Feed-Override
 *
 *  M48 is set ON on initialization and program end
 *  M50 is set OFF on initialization and program end
 *  P is set to 1.000 on initialization and program end (there is always a valid value)
 */
/* Implementation Notes:
 *
 * To do this correctly need to look not just at new values, but at current state
 * and transitions. See m48 for M48 transitions
 *
 *  M48state    M50enable   M50new   M50 endstate:  action (notes):
 *  disable     disable     M50 P0    disable       no action or state change
 *  disable     disable     M50       ENABLE        no action (m48 is disabled)
 *  disable     disable     M50 Pn    ENABLE        store new P value (no other action)
 *
 *  disable     ENABLE      M50 P0    disable       no action
 *  disable     ENABLE      M50       ENABLE        no action (m48 is disabled)
 *  disable     ENABLE      M50 Pn    ENABLE        store new P value (no other action)
 *
 *  ENABLE      disable     M50 P0    disable       no action or state change
 *  ENABLE      disable     M50       ENABLE        start ramp w/stored P value
 *  ENABLE      disable     M50 Pn    ENABLE        start ramp w/new P value; store P value
 *
 *  ENABLE      ENABLE      M50 P0    disable       end ramp
 *  ENABLE      ENABLE      M50       ENABLE        no action
 *  ENABLE      ENABLE      M50 Pn    ENABLE        start ramp w/new P value; store P value
 *                                                  (Note: new ramp will supercede any existing ramp)
 */

// stat_t cm_fro_control(const float P_word, const bool P_flag) // M50
// {
//     bool new_enable = true;
//     bool new_override = false;
//     if (P_flag) {                           // if parameter is present in Gcode block
//         if (fp_ZERO(P_word)) {
//             new_enable = false;             // P0 disables override
//         } else {
//             if (P_word < FEED_OVERRIDE_MIN) {
//                 return (STAT_INPUT_LESS_THAN_MIN_VALUE);
//             }
//             if (P_word > FEED_OVERRIDE_MAX) {
//                 return (STAT_INPUT_EXCEEDS_MAX_VALUE);
//             }
//             cm->gmx.mfo_factor = P_word;    // P word is valid, store it.
//             new_override = true;
//         }
//     }
//     if (cm->gmx.m48_enable) {               // if master enable is ON
//         if (new_enable && (new_override || !cm->gmx.mfo_enable)) {   // 3 cases to start a ramp
//             mp_start_feed_override(FEED_OVERRIDE_RAMP_TIME, cm->gmx.mfo_factor);
//         } else if (cm->gmx.mfo_enable && !new_enable) {              // case to turn off the ramp
//             mp_end_feed_override(FEED_OVERRIDE_RAMP_TIME);
//         }
//     }
//     cm->gmx.mfo_enable = new_enable;        // always update the enable state
//     return (STAT_OK);
// }

// stat_t cm_tro_control(const float P_word, const bool P_flag) // M50.1
// {
//     bool new_enable = true;
//     bool new_override = false;
//     if (P_flag) {                           // if parameter is present in Gcode block
//         if (fp_ZERO(P_word)) {
//             new_enable = false;             // P0 disables override
//         } else {
//             if (P_word < TRAVERSE_OVERRIDE_MIN) {
//                 return (STAT_INPUT_LESS_THAN_MIN_VALUE);
//             }
//             if (P_word > TRAVERSE_OVERRIDE_MAX) {
//                 return (STAT_INPUT_EXCEEDS_MAX_VALUE);
//             }
//             cm->gmx.mto_factor = P_word;    // P word is valid, store it.
//             new_override = true;
//         }
//     }
//     if (cm->gmx.m48_enable) {               // if master enable is ON
//         if (new_enable && (new_override || !cm->gmx.mfo_enable)) {   // 3 cases to start a ramp
//             mp_start_traverse_override(FEED_OVERRIDE_RAMP_TIME, cm->gmx.mto_factor);
//         } else if (cm->gmx.mto_enable && !new_enable) {              // case to turn off the ramp
//             mp_end_traverse_override(FEED_OVERRIDE_RAMP_TIME);
//         }
//     }
//     cm->gmx.mto_enable = new_enable;        // always update the enable state
//     return (STAT_OK);
// }

/****************************************************************************************
 **** Program Functions (4.3.10) ********************************************************
 ****************************************************************************************/
/* This group implements stop, start, and end functions.
 * It is extended beyond the NIST spec to handle various situations.
 *
 * _exec_program_finalize()     - helper
 * cm_cycle_start()             - sets MACHINE_CYCLE condition wwhich enables planner execution
 * cm_cycle_end()               - resets MACHINE_CYCLE
 * cm_program_stop()            - M0 - performs NIST STOP functions
 * cm_optional_program_stop()   - M1 - conditionally performs NIST STOP functions
 * cm_program_end()             - M2, M30 - performs NIST END functions (with some differences)
 */
/*
 * Program and cycle state functions
 *
 *  cm_program_stop and cm_optional_program_stop are synchronous Gcode commands that are
 *  received through the interpreter. They cause all motion to stop at the end of the
 *  current command, including spindle motion.
 *
 *  Note that the stop occurs at the end of the immediately preceding command
 *  (i.e. the stop is queued behind the last command).
 *
 *  cm_program_end is a stop that also resets the machine to initial state
 *
 *  cm_program_end() implements M2 and M30
 *  The END behaviors are defined by NIST 3.6.1 are:
 *    1a. Coordinate offsets are set to the default (like G54)
 *    1b. G92 origin offsets are set to zero (like G92.2)
 *    2.  Selected plane is set to CANON_PLANE_XY (like G17)
 *    3.  Distance mode is set to MODE_ABSOLUTE (like G90)
 *    4.  Feed rate mode is set to UNITS_PER_MINUTE (like G94)
 *    5.  Feed and speed overrides are set to ON (like M48)
 *    6.  Cutter compensation is turned off (like G40)
 *    7.  The spindle is stopped (like M5)
 *    8.  The current motion mode is set to G_1 (like G1)
 *    9.  Coolant is turned off (like M9)
 *
 * cm_program_end() implments things slightly differently (1a, 8):
 *    1a. Set default coordinate system (uses $gco, not G54)
 *    1b. G92 origin offsets are SUSPENDED (G92.2)
 *    2.  Selected plane is set to default plane ($gpl)
 *    3.  Distance mode is set to MODE_ABSOLUTE (like G90)
 *    4.  Feed rate mode is set to UNITS_PER_MINUTE (like G94)
 *    5.  Feed and speed overrides are set to ON (like M48)
 *    6.  (Cutter compensation not implemented)
 *    7.  The spindle is stopped (like M5)
 *    8.  Motion mode is CANCELED like G80 (not set to G1 as per NIST)
 *    9.  Coolant is turned off (like M9)
 *   10.  Turn off all heaters and fans
 */

static void _exec_program_finalize(float* value, bool* flag) {
    // perform the following resets if it's a program END
    if (cm->machine_state == MACHINE_PROGRAM_END) {
        spindle_stop();             // immediate M5
        coolant_control_immediate(COOLANT_OFF,COOLANT_BOTH);// immediate M9
        temperature_reset();                                // turn off all heaters and fans
        cm_reset_overrides();                               // enable G48, reset feed rate, traverse and spindle overrides
    }

    sr_request_status_report(SR_REQUEST_IMMEDIATE);         // request a final and full status report (not filtered)
}

static void _exec_program_stop_end(cmMachineState machine_state)
{
    // WARNING: We must not queue more than four things here, or we'll use up all the spare queue slots and crash the system
    // The good news is we shouldn't need to queue much

    // If we are already out of cycle, then adjust the machine state
    if ((cm->cycle_type == CYCLE_NONE) && // cm->cycle_type == CYCLE_MACHINING ||
        (cm->machine_state != MACHINE_ALARM) &&
        (cm->machine_state != MACHINE_SHUTDOWN)) {
        cm->machine_state = machine_state;                  // don't update macs/cycs if we're in the middle of a canned cycle,
    }

    // reset the rest of the states
    cm->hold_state = FEEDHOLD_OFF;
    // mp_zero_segment_velocity();                             // for reporting purposes

    // perform the following resets if it's a program END
    if (machine_state == MACHINE_PROGRAM_END) {
        cm_suspend_g92_offsets();                           //  G92.2 - as per NIST
        cm_set_coord_system(cm->default_coord_system);      //  reset to default coordinate system
        cm_select_plane(cm->default_select_plane);          //  reset to default arc plane
        cm_set_distance_mode(cm->default_distance_mode);    //  reset to default distance mode
        cm_set_arc_distance_mode(INCREMENTAL_DISTANCE_MODE);//  always the default
        cm_set_feed_rate_mode(UNITS_PER_MINUTE_MODE);       //  G94
        cm_set_motion_mode(MODEL, MOTION_MODE_CANCEL_MOTION_MODE); // NIST specifies G1 (MOTION_MODE_STRAIGHT_FEED), but we cancel motion mode. Safer.

        // the rest will be queued and executed in _exec_program_finalize()
    }

    cm_set_motion_state(MOTION_STOP);                       // also changes active model back to MODEL

    mp_queue_command(_exec_program_finalize, nullptr, nullptr);
}

// Will start a cycle regardless of whether the planner has moves or not
void cm_cycle_start()
{
    if (cm->cycle_type == CYCLE_NONE) {                     // don't (re)start homing, probe or other canned cycles
        cm->cycle_type = CYCLE_MACHINING;
        cm->machine_state = MACHINE_CYCLE;
        qr_init_queue_report();                             // clear queue reporting buffer counts//
    }
}

void cm_cycle_end() {
    if (cm->cycle_type == CYCLE_MACHINING) {
        cm->machine_state = MACHINE_PROGRAM_STOP;
        cm->cycle_type = CYCLE_NONE;
        cm_set_motion_state(MOTION_STOP);

        sr_request_status_report(SR_REQUEST_IMMEDIATE);         // request a final and full status report (not filtered)
    }
}

void cm_canned_cycle_end()
{
    cm->cycle_type = CYCLE_NONE;
    _exec_program_stop_end(MACHINE_PROGRAM_STOP);
}

void cm_program_stop()
{
    _exec_program_stop_end(MACHINE_PROGRAM_STOP);
}

void cm_optional_program_stop()
{
    _exec_program_stop_end(MACHINE_PROGRAM_STOP);
}

void cm_program_end()
{
    _exec_program_stop_end(MACHINE_PROGRAM_END);
}

/****************************************************************************************
 **** Additional Functions **************************************************************
 ****************************************************************************************/
/*
 * cm_json_command() - M100
 * cm_json_wait() - M102
 */
stat_t cm_json_command(char *json_string)
{
    return mp_json_command(json_string);
}

/*
 * cm_json_command_immediate() - M100.1
 */
stat_t cm_json_command_immediate(char *json_string)
{
    return mp_json_command_immediate(json_string);
}

/*
 * cm_json_wait() - M102
 */
stat_t cm_json_wait(char *json_string)
{
    return mp_json_wait(json_string);
}

/****************************************************************************************
 * cm_run_home() - run homing sequence
 */

stat_t cm_run_home(nvObj_t *nv)
{
    if (nv->value_int) {    // if true
        float axes[] = INIT_AXES_ONES;
        bool flags[] = INIT_AXES_TRUE;
        cm_homing_cycle_start(axes, flags);
    }
    return (STAT_OK);
}

/****************************************************************************************
 * Jogging Commands
 *
 * cm_get_jogging_dest()
 * cm_run_jog()
 */

float cm_get_jogging_dest(void)
{
    return cm->jogging_dest;
}

stat_t cm_run_jog(nvObj_t *nv)
{
    set_float(nv, cm->jogging_dest);
    cm_jogging_cycle_start(_axis(nv));
    return (STAT_OK);
}

/**************************************
 * END OF CANONICAL MACHINE FUNCTIONS *
 **************************************/

/****************************************************************************************
 **** CONFIGURATION AND INTERFACE FUNCTIONS *********************************************
 ****************************************************************************************
 * Functions to get and set variables from the cfgArray table
 * These functions are not part of the NIST defined functions
 ****************************************************************************************/

/***** AXIS HELPERS **********************************************************************
 * _coord()           - return coordinate system number (53=0...59=6) or -1 if error
 * _axis()            - return axis # or -1 if not an axis (works for mapped motors as well)
 * cm_get_axis_type() - return linear axis (0), rotary axis (1) or error (-1)
 * cm_get_axis_char() - return ASCII char for internal axis number provided
 */

static int8_t _coord(nvObj_t *nv)   // extract coordinate system from 3rd character
{
    char *ptr = ((*nv->group == 0) ? &nv->token[1] : &nv->group[1]); // skip past the 'g' to the number
    return (std::max ((atoi(ptr)-53), -1));  // return G54-G59 as 0-5, error as -1
}

/* _axis()
 *
 *  Cases handled:
 *    - sys/...       value is a system parameter (global), there is no axis (AXIS_TYPE_SYSTEM)
 *    - xam, yvm      any prefixed axis parameter
 *    - 1ma, 2tr      any motor parameter will return mapped axis for that motor
 *    - posx, mpox    readouts
 *    - g54x, g92z    offsets
 *    - tofx          tool offsets
 *    - tt1x, tt32x   tool table
 *    - _tex, _tra    diagnostic parameters
 *    - u,v,w         handles U, V and W axes
 *
 *  Note that this function will return an erroneous value if called by a non-axis tag,
 *  such as 'coph' But it should not be called in these cases in any event.
 */

static int8_t _axis(const nvObj_t *nv)
{
    auto &cfgTmp = cfgArray[nv->index];

    // // test if this is a SYS parameter (global), in which case there will be no axis
    // if (strcmp("sys", cfgTmp.group) == 0) {
    //     return (AXIS_TYPE_SYSTEM);
    // }

    // if the leading character of the token is a number it's a motor
    char c = cfgTmp.token[0];

    // if (isdigit(c)) {
    //     return(st_cfg.mot[c-0x31].motor_map);   // return the axis associated with the motor
    // }

    // otherwise it's an axis. Or undefined, which is usually a global.
    char *ptr;
#if (AXES == 9)
    char axes[] = {"xyzuvwabc"};
#else
    char axes[] = {"xyzabc"};
#endif
    if ((ptr = strchr(axes, c)) == NULL) {      // not NULL indicates a prefixed axis
        c = *(cfgTmp.token + strlen(cfgTmp.token) -1); // get the last character
        if ((ptr = strchr(axes, c)) == NULL) {  // test for a postfixed axis
            return (AXIS_TYPE_UNDEFINED);
        }
    }
    return (ptr - axes);
}

cmAxisType cm_get_axis_type(const nvObj_t *nv)
{
    int8_t axis = _axis(nv);
    if (axis <= AXIS_TYPE_UNDEFINED) {
        return ((cmAxisType)axis);
    }
    if (axis >= AXIS_A) {
        return (AXIS_TYPE_ROTARY);
    }
    return (AXIS_TYPE_LINEAR);
}

char cm_get_axis_char(const int8_t axis)    // Uses internal axis numbering
{
#if (AXES == 9)
    char axis_char[] = "XYZUVWABC";
#else
    char axis_char[] = "XYZABC";
#endif
    if ((axis < 0) || (axis > AXES)) return (' ');
    return (axis_char[axis]);
}

/**** Functions called directly from cfgArray table - mostly wrappers ****
 * _get_msg_helper() - helper to get string values
 *
 * cm_get_stat()  - get combined machine state as value and string
 * cm_get_macs()  - get raw machine state as value and string
 * cm_get_cycs()  - get raw cycle state as value and string
 * cm_get_mots()  - get raw motion state as value and string
 * cm_get_hold()  - get raw hold state as value and string
 * cm_get_home()  - get raw homing state as value and string
 *
 * cm_get_unit()  - get units mode as integer and display string
 * cm_get_coor()  - get goodinate system
 * cm_get_momo()  - get runtime motion mode
 * cm_get_plan()  - get model plane select
 * cm_get_path()  - get model path control mode
 * cm_get_dist()  - get model distance mode
 * cm_get_admo()  - get model arc distance mode
 * cm_get_frmo()  - get model feed rate mode
 * cm_get_tool()  - get tool
 * cm_get_feed()  - get feed rate
 * cm_get_mline() - get model line number for status reports
 * cm_get_line()  - get active (model or runtime) line number for status reports
 * cm_get_vel()   - get runtime velocity
 * cm_get_ofs()   - get current work offset (runtime)
 * cm_get_pos()   - get current work position (runtime)
 * cm_get_mpos()  - get current machine position (runtime)
 *
 * cm_print_pos() - print work position (with proper units)
 * cm_print_mpos()- print machine position (always mm units)
 * cm_print_coor()- print coordinate offsets with linear units
 * cm_print_corr()- print coordinate offsets with rotary units
 */

#ifdef __TEXT_MODE

// Strings for text mode displays:

static const char msg_units0[] = " in";    // used by generic print functions
static const char msg_units1[] = " mm";
static const char msg_units2[] = " deg";
static const char *const msg_units[] = { msg_units0, msg_units1, msg_units2 };
#define DEGREE_INDEX 2

static const char msg_am00[] = "[disabled]";
static const char msg_am01[] = "[standard]";
static const char msg_am02[] = "[inhibited]";
static const char msg_am03[] = "[radius]";
static const char *const msg_am[] = { msg_am00, msg_am01, msg_am02, msg_am03};

static const char msg_g20[] = "G20 - inches mode";
static const char msg_g21[] = "G21 - millimeter mode";
static const char *const msg_unit[] = { msg_g20, msg_g21 };

static const char msg_stat0[] = "Initializing";    // combined state (stat) uses this array
static const char msg_stat1[] = "Ready";
static const char msg_stat2[] = "Alarm";
static const char msg_stat3[] = "Stop";
static const char msg_stat4[] = "End";
static const char msg_stat5[] = "Run";
static const char msg_stat6[] = "Hold";
static const char msg_stat7[] = "Probe";
static const char msg_stat8[] = "Cycle";
static const char msg_stat9[] = "Homing";
static const char msg_stat10[] = "Jog";
static const char msg_stat11[] = "Interlock";
static const char msg_stat12[] = "Shutdown";
static const char msg_stat13[] = "Panic";
static const char *const msg_stat[] = { msg_stat0, msg_stat1, msg_stat2, msg_stat3,
                                        msg_stat4, msg_stat5, msg_stat6, msg_stat7,
                                        msg_stat8, msg_stat9, msg_stat10, msg_stat11,
                                        msg_stat12, msg_stat13 };

static const char msg_macs0[] = "Initializing";
static const char msg_macs1[] = "Ready";
static const char msg_macs2[] = "Alarm";
static const char msg_macs3[] = "Stop";
static const char msg_macs4[] = "End";
static const char msg_macs5[] = "Cycle";
static const char msg_macs6[] = "Interlock";
static const char msg_macs7[] = "SHUTDOWN";
static const char msg_macs8[] = "PANIC";
static const char *const msg_macs[] = { msg_macs0, msg_macs1, msg_macs2, msg_macs3,
                                        msg_macs4, msg_macs5, msg_macs6, msg_macs7,
                                        msg_macs8 };

static const char msg_cycs0[] = "Off";
static const char msg_cycs1[] = "Machining";
static const char msg_cycs2[] = "Homing";
static const char msg_cycs3[] = "Probe";
static const char msg_cycs4[] = "Jog";
static const char *const msg_cycs[] = { msg_cycs0, msg_cycs1, msg_cycs2, msg_cycs3,  msg_cycs4 };

static const char msg_mots0[] = "Stop";
static const char msg_mots1[] = "Planning";
static const char msg_mots2[] = "Run";
static const char msg_mots3[] = "Hold";
static const char *const msg_mots[] = { msg_mots0, msg_mots1, msg_mots2, msg_mots3 };

static const char msg_hold0[] = "Off";
static const char msg_hold1[] = "Requested";
static const char msg_hold2[] = "Sync";
static const char msg_hold3[] = "Decel Continue";
static const char msg_hold4[] = "Decel To Zero";
static const char msg_hold5[] = "Decel Complete";
static const char msg_hold6[] = "Motion Stopping";
static const char msg_hold7[] = "Motion Stopped";
static const char msg_hold8[] = "Hold Actions Pending";
static const char msg_hold9[] = "Hold Actions Complete";
static const char msg_hold10[] = "Holding";
static const char msg_hold11[] = "Hold Exit Actions Pending";
static const char msg_hold12[] = "Hold Exit Actions Complete";
static const char *const msg_hold[] = { msg_hold0, msg_hold1, msg_hold2, msg_hold3, msg_hold4,
                                        msg_hold5, msg_hold6, msg_hold7, msg_hold8, msg_hold9,
                                        msg_hold10, msg_hold11, msg_hold12 };

static const char msg_home0[] = "Not Homed";
static const char msg_home1[] = "Homed";
static const char msg_home2[] = "Homing";
static const char *const msg_home[] = { msg_home0, msg_home1, msg_home2 };

static const char msg_probe0[] = "Probe Failed";
static const char msg_probe1[] = "Probe Succeeded";
static const char msg_probe2[] = "Probe Waiting";
static const char *const msg_probe[] = { msg_probe0, msg_probe1, msg_probe2 };

static const char msg_g53[] = "G53 - machine coordinate system";
static const char msg_g54[] = "G54 - coordinate system 1";
static const char msg_g55[] = "G55 - coordinate system 2";
static const char msg_g56[] = "G56 - coordinate system 3";
static const char msg_g57[] = "G57 - coordinate system 4";
static const char msg_g58[] = "G58 - coordinate system 5";
static const char msg_g59[] = "G59 - coordinate system 6";
static const char *const msg_coor[] = { msg_g53, msg_g54, msg_g55, msg_g56, msg_g57, msg_g58, msg_g59 };

static const char msg_g00[] = "G0  - linear traverse";
static const char msg_g01[] = "G1  - linear feed";
static const char msg_g02[] = "G2  - clockwise arc feed";
static const char msg_g03[] = "G3  - counter clockwise arc feed";
static const char msg_g80[] = "G80 - cancel motion mode (none active)";
static const char *const msg_momo[] = { msg_g00, msg_g01, msg_g02, msg_g03, msg_g80 };

static const char msg_g17[] = "G17 - XY plane";
static const char msg_g18[] = "G18 - XZ plane";
static const char msg_g19[] = "G19 - YZ plane";
static const char *const msg_plan[] = { msg_g17, msg_g18, msg_g19 };

static const char msg_g61[] = "G61 - exact path mode";
static const char msg_g6a[] = "G61.1 - exact stop mode";
static const char msg_g64[] = "G64 - continuous mode";
static const char *const msg_path[] = { msg_g61, msg_g6a, msg_g64 };

static const char msg_g90[] = "G90 - absolute distance mode";
static const char msg_g91[] = "G91 - incremental distance mode";
static const char *const msg_dist[] = { msg_g90, msg_g91 };

static const char msg_g901[] = "G90.1 - absolute distance mode";
static const char msg_g911[] = "G91.1 - incremental distance mode (default mode)";
static const char *const msg_admo[] = { msg_g901, msg_g911 };

static const char msg_g93[] = "G93 - inverse time mode";
static const char msg_g94[] = "G94 - units-per-minute mode (i.e. feedrate mode)";
static const char msg_g95[] = "G95 - units-per-revolution mode";
static const char *const msg_frmo[] = { msg_g93, msg_g94, msg_g95 };

#else

#define msg_units NULL
#define msg_unit NULL
#define msg_stat NULL
#define msg_macs NULL
#define msg_cycs NULL
#define msg_mots NULL
#define msg_hold NULL
#define msg_home NULL
#define msg_coor NULL
#define msg_momo NULL
#define msg_plan NULL
#define msg_path NULL
#define msg_dist NULL
#define msg_admo NULL
#define msg_frmo NULL
#define msg_am NULL

#endif // __TEXT_MODE

//_get_msg_helper() - add the string for the enum to the nv, but leave it as a TYPE_INTEGER
stat_t _get_msg_helper(nvObj_t *nv, const char *const msg_array[], int32_t value)
{
    nv->value_int = value;
    nv->valuetype = TYPE_INTEGER;
    return(nv_copy_string(nv, (const char *)GET_TEXT_ITEM(msg_array, value)));
}

stat_t cm_get_stat(nvObj_t *nv) { return(_get_msg_helper(nv, msg_stat, cm_get_combined_state(&cm1)));}
stat_t cm_get_stat2(nvObj_t *nv){ return(_get_msg_helper(nv, msg_stat, cm_get_combined_state(&cm2)));}
stat_t cm_get_macs(nvObj_t *nv) { return(_get_msg_helper(nv, msg_macs, cm_get_machine_state()));}
stat_t cm_get_cycs(nvObj_t *nv) { return(_get_msg_helper(nv, msg_cycs, cm_get_cycle_type()));}
stat_t cm_get_mots(nvObj_t *nv) { return(_get_msg_helper(nv, msg_mots, cm_get_motion_state()));}
stat_t cm_get_hold(nvObj_t *nv) { return(_get_msg_helper(nv, msg_hold, cm_get_hold_state()));}

stat_t cm_get_unit(nvObj_t *nv) { return(_get_msg_helper(nv, msg_unit, cm_get_units_mode(ACTIVE_MODEL)));}
stat_t cm_get_coor(nvObj_t *nv) { return(_get_msg_helper(nv, msg_coor, cm_get_coord_system(ACTIVE_MODEL)));}
stat_t cm_get_momo(nvObj_t *nv) { return(_get_msg_helper(nv, msg_momo, cm_get_motion_mode(ACTIVE_MODEL)));}
stat_t cm_get_plan(nvObj_t *nv) { return(_get_msg_helper(nv, msg_plan, cm_get_select_plane(ACTIVE_MODEL)));}
stat_t cm_get_path(nvObj_t *nv) { return(_get_msg_helper(nv, msg_path, cm_get_path_control(ACTIVE_MODEL)));}
stat_t cm_get_dist(nvObj_t *nv) { return(_get_msg_helper(nv, msg_dist, cm_get_distance_mode(ACTIVE_MODEL)));}
stat_t cm_get_admo(nvObj_t *nv) { return(_get_msg_helper(nv, msg_admo, cm_get_arc_distance_mode(ACTIVE_MODEL)));}
stat_t cm_get_frmo(nvObj_t *nv) { return(_get_msg_helper(nv, msg_frmo, cm_get_feed_rate_mode(ACTIVE_MODEL)));}

stat_t cm_get_toolv(nvObj_t *nv) { return(get_integer(nv, cm_get_tool(ACTIVE_MODEL))); }
stat_t cm_get_mline(nvObj_t *nv) { return(get_integer(nv, cm_get_linenum(MODEL))); }
stat_t cm_get_line(nvObj_t *nv)  { return(get_integer(nv, cm_get_linenum(ACTIVE_MODEL))); }

stat_t cm_get_vel(nvObj_t *nv)
{
    if (cm_get_motion_state() == MOTION_STOP) {
        nv->value_flt = 0;
    } else {
        nv->value_flt = mp_get_runtime_velocity();
        if (cm_get_units_mode(RUNTIME) == INCHES) {
            nv->value_flt *= INCHES_PER_MM;
        }
    }
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;
    return (STAT_OK);
}

stat_t cm_get_feed(nvObj_t *nv) { return (get_float(nv, cm_get_feed_rate(ACTIVE_MODEL))); }
stat_t cm_get_pos(nvObj_t *nv)  { return (get_float(nv, cm_get_display_position(ACTIVE_MODEL, _axis(nv)))); }
stat_t cm_get_mpo(nvObj_t *nv)  { return (get_float(nv, cm_get_absolute_position(ACTIVE_MODEL, _axis(nv)))); }
stat_t cm_get_ofs(nvObj_t *nv)  { return (get_float(nv, cm_get_display_offset(ACTIVE_MODEL, _axis(nv)))); }

stat_t cm_get_home(nvObj_t *nv) { return(_get_msg_helper(nv, msg_home, cm_get_homing_state())); }
stat_t cm_set_home(nvObj_t *nv) { return (set_integer(nv, ((uint8_t &)(cm->homing_state)), false, true)); }
stat_t cm_get_hom(nvObj_t *nv)  { return (get_integer(nv, cm->homed[_axis(nv)])); }

stat_t cm_get_prob(nvObj_t *nv) { return(_get_msg_helper(nv, msg_probe, cm_get_probe_state())); }
stat_t cm_get_prb(nvObj_t *nv)  { return (get_float(nv, cm->probe_results[0][_axis(nv)])); }
stat_t cm_get_probe_input(nvObj_t *nv) { return (get_integer(nv, cm->probe_input)); }
stat_t cm_set_probe_input(nvObj_t *nv) { return (set_integer(nv, cm->probe_input, 0, D_IN_CHANNELS)); }

stat_t cm_get_coord(nvObj_t *nv) { return (get_float(nv, cm->coord_offset[_coord(nv)][_axis(nv)])); }
stat_t cm_set_coord(nvObj_t *nv) { return (set_float(nv, cm->coord_offset[_coord(nv)][_axis(nv)])); }

stat_t cm_get_g92e(nvObj_t *nv)  { return (get_integer(nv, cm->gmx.g92_offset_enable)); }
stat_t cm_get_g92(nvObj_t *nv)   { return (get_float(nv, cm->gmx.g92_offset[_axis(nv)])); }
stat_t cm_get_g28(nvObj_t *nv)   { return (get_float(nv, cm->gmx.g28_position[_axis(nv)])); }
stat_t cm_get_g30(nvObj_t *nv)   { return (get_float(nv, cm->gmx.g30_position[_axis(nv)])); }

/*****************************************************
 **** TOOL TABLE AND OFFSET GET AND SET FUNCTIONS ****
 *****************************************************/

static uint8_t _tool(nvObj_t *nv)
{
    if (nv->group[0] != 0) {
        return (atoi(&nv->group[2]));   // ttNN is the group, axis is in the token
    }
    return (atoi(&nv->token[2]));       // ttNNx is all in the token
}

stat_t cm_get_tof(nvObj_t *nv) { return (get_float(nv, cm->tool_offset[_axis(nv)])); }
stat_t cm_set_tof(nvObj_t *nv) { return (set_float(nv, cm->tool_offset[_axis(nv)])); }

stat_t cm_get_tt(nvObj_t *nv)
{
    uint8_t toolnum = _tool(nv);
    if (toolnum > TOOLS) {
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    return (get_float(nv, tt.tt_offset[toolnum][_axis(nv)]));
}

stat_t cm_set_tt(nvObj_t *nv)
{
    uint8_t toolnum = _tool(nv);
    if (toolnum > TOOLS) {
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    return(set_float(nv, tt.tt_offset[toolnum][_axis(nv)]));
}

/************************************
 **** AXIS GET AND SET FUNCTIONS ****
 ************************************/
/*
 * cm_get_am() - get axis mode w/enumeration string
 * cm_set_am() - set axis mode w/exception handling for axis type
 * cm_get_tn() - get axis travel min
 * cm_set_tn() - set axis travel min
 * cm_get_tm() - get axis travel max
 * cm_set_tm() - set axis travel max
 */

stat_t cm_get_am(nvObj_t *nv)
{
    int8_t axis = _axis(nv);
    nv->value_int = cm->a[axis].axis_mode;
    return(_get_msg_helper(nv, msg_am, nv->value_int));
}

stat_t cm_set_am(nvObj_t *nv)        // axis mode
{
    if (cm_get_axis_type(nv) == AXIS_TYPE_LINEAR) {
        if (nv->value_int > AXIS_MODE_LINEAR_MAX) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_EXCEEDS_MAX_VALUE);
        }
    } else {
        if (nv->value_int > AXIS_MODE_ROTARY_MAX) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_EXCEEDS_MAX_VALUE);
        }
    }
    nv->valuetype = TYPE_INTEGER;
    cm->a[_axis(nv)].axis_mode = (cmAxisMode)nv->value_int;
    return(STAT_OK);
}

stat_t cm_get_tn(nvObj_t *nv) { return (get_float(nv, cm->a[_axis(nv)].travel_min)); }
stat_t cm_set_tn(nvObj_t *nv) { return (set_float(nv, cm->a[_axis(nv)].travel_min)); }
stat_t cm_get_tm(nvObj_t *nv) { return (get_float(nv, cm->a[_axis(nv)].travel_max)); }
stat_t cm_set_tm(nvObj_t *nv) { return (set_float(nv, cm->a[_axis(nv)].travel_max)); }
stat_t cm_get_ra(nvObj_t *nv) { return (get_float(nv, cm->a[_axis(nv)].radius)); }
stat_t cm_set_ra(nvObj_t *nv) { return (set_float_range(nv, cm->a[_axis(nv)].radius, RADIUS_MIN, 1000000)); }

/**** Axis Jerk Primitives
 * cm_get_axis_jerk() - returns max jerk for an axis
 * cm_set_axis_jerk() - sets the jerk for an axis, including reciprocal and cached values
 */
float cm_get_axis_jerk(const uint8_t axis) { return (cm->a[axis].jerk_max); }

// Precompute sqrt(3)/10 for the max_junction_accel.
// See plan_line.cpp -> _calculate_junction_vmax() notes for details.
static const float _junction_accel_multiplier = sqrt(3.0)/10.0;

// Important note: Actual jerk is stored jerk * JERK_MULTIPLIER, and
// Time Quanta is junction_integration_time / 1000.
void _cm_recalc_junction_accel(const uint8_t axis) {
    float T = cm->junction_integration_time / 1000.0;
    float T2 = T*T;
    cm->a[axis].max_junction_accel = _junction_accel_multiplier * T2 * (cm->a[axis].jerk_max * JERK_MULTIPLIER);
    cm->a[axis].high_junction_accel = _junction_accel_multiplier * T2 * (cm->a[axis].jerk_high * JERK_MULTIPLIER);
}

void cm_set_axis_max_jerk(const uint8_t axis, const float jerk)
{
    cm->a[axis].jerk_max = jerk;
    _cm_recalc_junction_accel(axis);    // Must recalculate the max_junction_accel now that the jerk has changed.
}

void cm_set_axis_high_jerk(const uint8_t axis, const float jerk)
{
    cm->a[axis].jerk_high = jerk;
    _cm_recalc_junction_accel(axis);    // Must recalculate the max_junction_accel now that the jerk has changed.
}

/**** Axis Velocity and Jerk Settings
 *
 * cm_get_vm() - get velocity max value - called from dispatch table
 * cm_set_vm() - set velocity max value - called from dispatch table
 * cm_get_fr() - get feedrate max value - called from dispatch table
 * cm_set_fr() - set feedrate max value - called from dispatch table
 * cm_get_jm() - get jerk max value     - called from dispatch table
 * cm_set_jm() - set jerk max value     - called from dispatch table
 * cm_get_jh() - get jerk homing value  - called from dispatch table
 * cm_set_jh() - set jerk homing value  - called from dispatch table
 *
 *  Jerk values can be rather large, often in the billions. This makes for some pretty big
 *  numbers for people to deal with. Jerk values are stored in the system in truncated format;
 *  values are divided by 1,000,000 then reconstituted before use.
 *
 *  The axis_jerk() functions expect the jerk in divided-by 1,000,000 form.
 *  The set_xjm() and set_xjh() functions accept values divided by 1,000,000.
 *  This is corrected to mm/min^3 by the internals of the code.
 */

stat_t cm_get_vm(nvObj_t *nv) { return (get_float(nv, cm->a[_axis(nv)].velocity_max)); }
stat_t cm_set_vm(nvObj_t *nv)
{
    uint8_t axis = _axis(nv);
    ritorno(set_float_range(nv, cm->a[axis].velocity_max, 0, MAX_LONG));
    cm->a[axis].recip_velocity_max = 1/nv->value_flt;
    return(STAT_OK);
}

stat_t cm_get_fr(nvObj_t *nv) { return (get_float(nv, cm->a[_axis(nv)].feedrate_max)); }
stat_t cm_set_fr(nvObj_t *nv)
{
    uint8_t axis = _axis(nv);
    ritorno(set_float_range(nv, cm->a[axis].feedrate_max, 0, MAX_LONG));
    cm->a[axis].recip_feedrate_max = 1/nv->value_flt;
    return(STAT_OK);
}

stat_t cm_get_jm(nvObj_t *nv) { return (get_float(nv, cm->a[_axis(nv)].jerk_max)); }
stat_t cm_set_jm(nvObj_t *nv)
{
    uint8_t axis = _axis(nv);
    ritorno(set_float_range(nv, cm->a[axis].jerk_max, JERK_INPUT_MIN, JERK_INPUT_MAX));
    cm_set_axis_max_jerk(axis, nv->value_flt);
    return(STAT_OK);
}

stat_t cm_get_jh(nvObj_t *nv) { return (get_float(nv, cm->a[_axis(nv)].jerk_high)); }
stat_t cm_set_jh(nvObj_t *nv)
{
    uint8_t axis = _axis(nv);
    ritorno(set_float_range(nv, cm->a[axis].jerk_high, JERK_INPUT_MIN, JERK_INPUT_MAX));
    cm_set_axis_high_jerk(axis, nv->value_flt);
    return(STAT_OK);
}

/**** Axis Homing Settings
 * cm_get_hi() - get homing input
 * cm_set_hi() - set homing input
 * cm_get_hd() - get homing direction
 * cm_set_hd() - set homing direction
 * cm_get_sv() - get homing search velocity
 * cm_set_sv() - set homing search velocity
 * cm_get_lv() - get homing latch velocity
 * cm_set_lv() - set homing latch velocity
 * cm_get_lb() - get homing latch backoff
 * cm_set_lb() - set homing latch backoff
 * cm_get_zb() - get homing zero backoff
 * cm_set_zb() - set homing zero backoff
 */

stat_t cm_get_hi(nvObj_t *nv) { return (get_integer(nv, cm->a[_axis(nv)].homing_input)); }
stat_t cm_set_hi(nvObj_t *nv) { return (set_integer(nv, cm->a[_axis(nv)].homing_input, 0, D_IN_CHANNELS)); }
stat_t cm_get_hd(nvObj_t *nv) { return (get_integer(nv, cm->a[_axis(nv)].homing_dir)); }
stat_t cm_set_hd(nvObj_t *nv) { return (set_integer(nv, cm->a[_axis(nv)].homing_dir, 0, 1)); }
stat_t cm_get_sv(nvObj_t *nv) { return (get_float(nv, cm->a[_axis(nv)].search_velocity)); }
stat_t cm_set_sv(nvObj_t *nv) { return (set_float_range(nv, cm->a[_axis(nv)].search_velocity, 0, MAX_LONG)); }
stat_t cm_get_lv(nvObj_t *nv) { return (get_float(nv, cm->a[_axis(nv)].latch_velocity)); }
stat_t cm_set_lv(nvObj_t *nv) { return (set_float_range(nv, cm->a[_axis(nv)].latch_velocity, 0, MAX_LONG)); }
stat_t cm_get_lb(nvObj_t *nv) { return (get_float(nv, cm->a[_axis(nv)].latch_backoff)); }
stat_t cm_set_lb(nvObj_t *nv) { return (set_float(nv, cm->a[_axis(nv)].latch_backoff)); }
stat_t cm_get_zb(nvObj_t *nv) { return (get_float(nv, cm->a[_axis(nv)].zero_backoff)); }
stat_t cm_set_zb(nvObj_t *nv) { return (set_float(nv, cm->a[_axis(nv)].zero_backoff)); }

/*** Canonical Machine Global Settings ***/
/*
 * cm_get_jt()  - get junction integration time
 * cm_set_jt()  - set junction integration time
 * cm_get_ct()  - get chordal tolerance
 * cm_set_ct()  - set chordal tolerance
 * cm_get_sl()  - get soft limit enable
 * cm_set_sl()  - set soft limit enable
 * cm_get_lim() - get hard limit enable
 * cm_set_lim() - set hard limit enable
 * cm_get_saf() - get safety interlock enable
 * cm_set_saf() - set safety interlock enable
 * cm_set_mfo() - set manual feedrate override factor
 * cm_set_mto() - set manual traverse override factor
 */

stat_t cm_get_jt(nvObj_t *nv) { return(get_float(nv, cm->junction_integration_time)); }
stat_t cm_set_jt(nvObj_t *nv)
{
    ritorno(set_float_range(nv, cm->junction_integration_time, JUNCTION_INTEGRATION_MIN, JUNCTION_INTEGRATION_MAX));
    for (uint8_t axis=0; axis<AXES; axis++) { // recalculate max_junction_accel now that time quanta has changed.
        _cm_recalc_junction_accel(axis);
    }
    return(STAT_OK);
}

stat_t cm_get_ct(nvObj_t *nv) { return(get_float(nv, cm->chordal_tolerance)); }
stat_t cm_set_ct(nvObj_t *nv) { return(set_float_range(nv, cm->chordal_tolerance, CHORDAL_TOLERANCE_MIN, 10000000)); }

stat_t cm_get_zl(nvObj_t *nv) { return(get_float(nv, cm->feedhold_z_lift)); }
stat_t cm_set_zl(nvObj_t *nv) { return(set_float(nv, cm->feedhold_z_lift)); }

stat_t cm_get_sl(nvObj_t *nv) { return(get_integer(nv, cm->soft_limit_enable)); }
stat_t cm_set_sl(nvObj_t *nv) { return(set_integer(nv, (uint8_t &)cm->soft_limit_enable, 0, 1)); }

stat_t cm_get_lim(nvObj_t *nv) { return(get_integer(nv, cm->limit_enable)); }
stat_t cm_set_lim(nvObj_t *nv) { return(set_integer(nv, (uint8_t &)cm->limit_enable, 0, 1)); }

stat_t cm_get_m48(nvObj_t *nv) { return(get_integer(nv, cm->gmx.m48_enable)); }
stat_t cm_set_m48(nvObj_t *nv) { return(set_integer(nv, (uint8_t &)cm->gmx.m48_enable, 0, 1)); }

stat_t cm_get_froe(nvObj_t *nv) { return(get_integer(nv, cm->gmx.mfo_enable)); }
stat_t cm_set_froe(nvObj_t *nv) { return(set_integer(nv, (uint8_t &)cm->gmx.mfo_enable, 0, 1)); }
stat_t cm_get_fro(nvObj_t *nv)  { return(get_float(nv, cm->gmx.mfo_factor)); }
stat_t cm_set_fro(nvObj_t *nv)  { return(set_float_range(nv, cm->gmx.mfo_factor, FEED_OVERRIDE_MIN, FEED_OVERRIDE_MAX)); }

stat_t cm_get_troe(nvObj_t *nv) { return(get_integer(nv, cm->gmx.mto_enable)); }
stat_t cm_set_troe(nvObj_t *nv) { return(set_integer(nv, (uint8_t &)cm->gmx.mto_enable, 0, 1)); }
stat_t cm_get_tro(nvObj_t *nv)  { return(get_float(nv, cm->gmx.mto_factor)); }
stat_t cm_set_tro(nvObj_t *nv)  { return(set_float_range(nv, cm->gmx.mto_factor, TRAVERSE_OVERRIDE_MIN, TRAVERSE_OVERRIDE_MAX)); }

stat_t cm_get_gpl(nvObj_t *nv) { return(get_integer(nv, cm->default_select_plane)); }
stat_t cm_set_gpl(nvObj_t *nv) { return(set_integer(nv, (uint8_t &)cm->default_select_plane, CANON_PLANE_XY, CANON_PLANE_YZ)); }

stat_t cm_get_gun(nvObj_t *nv) { return(get_integer(nv, cm->default_units_mode)); }
stat_t cm_set_gun(nvObj_t *nv) { return(set_integer(nv, (uint8_t &)cm->default_units_mode, INCHES, MILLIMETERS)); }

stat_t cm_get_gco(nvObj_t *nv) { return(get_integer(nv, cm->default_coord_system)); }
stat_t cm_set_gco(nvObj_t *nv) { return(set_integer(nv, (uint8_t &)cm->default_coord_system, G54, G59)); }

stat_t cm_get_gpa(nvObj_t *nv) { return(get_integer(nv, cm->default_path_control)); }
stat_t cm_set_gpa(nvObj_t *nv) { return(set_integer(nv, (uint8_t &)cm->default_path_control, PATH_EXACT_PATH, PATH_CONTINUOUS)); }

stat_t cm_get_gdi(nvObj_t *nv) { return(get_integer(nv, cm->default_distance_mode)); }
stat_t cm_set_gdi(nvObj_t *nv) { return(set_integer(nv, (uint8_t &)cm->default_distance_mode, ABSOLUTE_DISTANCE_MODE, INCREMENTAL_DISTANCE_MODE)); }

/*** Canonical Machine Global Settings Table Additions ***/
/*
 * The following functions are called by config_app.cpp to retrieve the relevant portions of the config table
 * getCmConfig_1() - general cononical machine info
 * getMpoConfig_1() - machine-coordinate position
 * getPosConfig_1() - external (gcode model) position
 * getOfsConfig_1() - offsets in play
 * getHomConfig_1() - homing info (should be in cycle_home)
 * getPrbConfig_1() - probing info (should in in cycle_probe)
 * getJogConfig_1() - control jogging (should be in cycle_jog)
 * getAxisConfig_1() - axis specific info
 */

struct grouplessCfgItem_t {
    const char * token;            // token - stripped of group prefix (w/NUL termination)
    uint8_t flags;                      // operations flags - see defines below
    int8_t precision;                   // decimal precision for display (JSON)
    fptrPrint print;                    // print binding: aka void (*print)(nvObj_t *nv);
    fptrCmd get;                        // GET binding aka uint8_t (*get)(nvObj_t *nv)
    fptrCmd set;                        // SET binding aka uint8_t (*set)(nvObj_t *nv)
};

// class cfgSubtableFromGrouplessStaticArray : public configSubtable {
//     // const size_t array_length;
//     const grouplessCfgItem_t *items;

//     // hold on to a reloadable full cfgItem_t to setup and return
//     static char tmpToken[TOKEN_LEN + 1];
//     static cfgItem_t cfgTmp;

//    public:
//    template<typename T, size_t length>
//     constexpr cfgSubtableFromGrouplessStaticArray(T (&i)[length]) : configSubtable{length}, items{i} {};

//     const cfgItem_t * const get(std::size_t idx) const override {
//         // group is ""
//         strcpy(tmpToken, items[idx].token);
//         cfgTmp.flags = items[idx].flags;
//         cfgTmp.precision = items[idx].precision;
//         cfgTmp.print = items[idx].print;
//         cfgTmp.get = items[idx].get;
//         cfgTmp.set = items[idx].set;
//         return &cfgTmp;
//     }

//     index_t find(const char *token) const override {
//         std::size_t idx = 0;
//         while (idx < length) {
//             if (strcmp(token, items[idx].token) == 0) {
//                 return idx;
//             }
//             idx++;
//         }
//         return NO_MATCH;
//     }

//     // const size_t length() const override {
//     //     return array_length;
//     // }
// };
// char cfgSubtableFromGrouplessStaticArray::tmpToken[TOKEN_LEN + 1];
// cfgItem_t cfgSubtableFromGrouplessStaticArray::cfgTmp = {"", &tmpToken[0], _i0, 0, nullptr, nullptr, nullptr, nullptr, 0};

constexpr cfgItem_t cm_config_items_1[] = {
    // dynamic model attributes for reporting purposes (up front for speed)
    {"",  "stat",  _i0, 0, cm_print_stat, cm_get_stat,  set_ro,        nullptr, 0},  // combined machine state
    {"",  "stat2", _i0, 0, cm_print_stat, cm_get_stat2, set_ro,        nullptr, 0},  // combined machine state
    {"",  "n",     _ii, 0, cm_print_line, cm_get_mline, set_noop,      nullptr, 0},  // Model line number
    {"",  "line",  _ii, 0, cm_print_line, cm_get_line,  set_ro,        nullptr, 0},  // Active line number - model or runtime line number
    {"",  "vel",   _f0, 2, cm_print_vel,  cm_get_vel,   set_ro,        nullptr, 0},  // current velocity
    {"",  "feed",  _f0, 2, cm_print_feed, cm_get_feed,  set_ro,        nullptr, 0},  // feed rate
    {"",  "macs",  _i0, 0, cm_print_macs, cm_get_macs,  set_ro,        nullptr, 0},  // raw machine state
    {"",  "cycs",  _i0, 0, cm_print_cycs, cm_get_cycs,  set_ro,        nullptr, 0},  // cycle state
    {"",  "mots",  _i0, 0, cm_print_mots, cm_get_mots,  set_ro,        nullptr, 0},  // motion state
    {"",  "hold",  _i0, 0, cm_print_hold, cm_get_hold,  set_ro,        nullptr, 0},  // feedhold state
    {"",  "unit",  _i0, 0, cm_print_unit, cm_get_unit,  set_ro,        nullptr, 0},  // units mode
    {"",  "coor",  _i0, 0, cm_print_coor, cm_get_coor,  set_ro,        nullptr, 0},  // coordinate system
    {"",  "momo",  _i0, 0, cm_print_momo, cm_get_momo,  set_ro,        nullptr, 0},  // motion mode
    {"",  "plan",  _i0, 0, cm_print_plan, cm_get_plan,  set_ro,        nullptr, 0},  // plane select
    {"",  "path",  _i0, 0, cm_print_path, cm_get_path,  set_ro,        nullptr, 0},  // path control mode
    {"",  "dist",  _i0, 0, cm_print_dist, cm_get_dist,  set_ro,        nullptr, 0},  // distance mode
    {"",  "admo",  _i0, 0, cm_print_admo, cm_get_admo,  set_ro,        nullptr, 0},  // arc distance mode
    {"",  "frmo",  _i0, 0, cm_print_frmo, cm_get_frmo,  set_ro,        nullptr, 0},  // feed rate mode
    {"",  "tool",  _i0, 0, cm_print_tool, cm_get_toolv, set_ro,        nullptr, 0},  // active tool
    {"",  "g92e",  _i0, 0, cm_print_g92e, cm_get_g92e,  set_ro,        nullptr, 0},  // G92 enable state
#ifdef TEMPORARY_HAS_LEDS
    {"",  "_leds", _i0, 0, tx_print_nul, _get_leds,_set_leds,          nullptr, 0}, // TEMPORARY - change LEDs
#endif
};
const cfgSubtableFromStaticArray cm_config_1 {cm_config_items_1};
const configSubtable * const getCmConfig_1() { return &cm_config_1; }

class cfgSubtableFromTokenList : public configSubtable {
    const char *group_name;
    const size_t group_name_length;
    const char * const * subkeys; // read it backward: "pointer to const pointers to const chars"
    // const size_t subkeys_length;

    // hold on to a reloadable full cfgItem_t to setup and return
    static char tmpToken[TOKEN_LEN + 1];
    cfgItem_t cfgTmp;

   public:
    template <size_t group_length, size_t subkey_count>
    constexpr cfgSubtableFromTokenList(const char (&new_group_name)[group_length], const char *const (&i)[subkey_count],
                                       const uint8_t new_flags, const int8_t new_precision, const fptrPrint new_print,
                                       const fptrCmd new_get_fn, const fptrCmd new_set_fn)
        : configSubtable{subkey_count},
          group_name{new_group_name},
          group_name_length{group_length - 1},  // string lengths include the null
          subkeys{i},
          //   subkeys_length{subkey_count},
          cfgTmp{&new_group_name[0], tmpToken, new_flags, new_precision, new_print, new_get_fn, new_set_fn, nullptr, 0}
        {};

    const cfgItem_t * const get(std::size_t idx) const override {
        char *dst = tmpToken;
        // strcpy(dst, group_name);
        // strcpy(dst+group_name_length, subkeys[idx]);
        // groupname is incorporated into tokens
        strcpy(dst, subkeys[idx]);
        return &cfgTmp;
    }

    index_t find(const char *token) const override {
        std::size_t idx = 0;
        if (*(token+group_name_length) == 0 || strncmp(token, group_name, group_name_length) != 0) {
            // wrong group or looking for just the group
            return NO_MATCH;
        }
        while (idx < length) {
            // groupname is incorporated into tokens
            if (strcmp(subkeys[idx]+group_name_length, token+group_name_length) == 0) {
                return idx;
            }
            idx++;
        }
        return NO_MATCH;
    }
};
char cfgSubtableFromTokenList::tmpToken[TOKEN_LEN + 1];
// cfgItem_t cfgSubtableFromTokenList::cfgTmp = {nullptr, &tmpToken[0], _i0, 0, nullptr, nullptr, nullptr, nullptr, 0};

#if (AXES == 9)
const char * const mpo_axis_keys[] = {"mpox", "mpoy", "mpoz", "mpou", "mpov", "mpow", "mpoa", "mpob", "mpoc"};
#else
const char * const mpo_axis_keys[] = {"mpox", "mpoy", "mpoz", "mpoa", "mpob", "mpoc"};
#endif
cfgSubtableFromTokenList mpo_config{"mpo", mpo_axis_keys, _f0, 5, cm_print_mpo, cm_get_mpo, set_ro};  // machine position
const configSubtable *const getMpoConfig_1() { return &mpo_config; }

// constexpr cfgItem_t pos_config_items_1[] = {
//     {"pos", "posx", _f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0},  // X work position
//     {"pos", "posy", _f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0},  // Y work position
//     {"pos", "posz", _f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0},  // Z work position
//     {"pos", "posu", _f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0},  // U work position
//     {"pos", "posv", _f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0},  // V work position
//     {"pos", "posw", _f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0},  // W work position
//     {"pos", "posa", _f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0},  // A work position
//     {"pos", "posb", _f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0},  // B work position
//     {"pos", "posc", _f0, 5, cm_print_pos, cm_get_pos, set_ro, nullptr, 0},  // C work position
// };
// constexpr cfgSubtableFromStaticArray pos_config_1{pos_config_items_1};
#if (AXES == 9)
const char * const pos_axis_keys[] = {"posx", "posy", "posz", "posu", "posv", "posw", "posa", "posb", "posc"};
#else
const char * const pos_axis_keys[] = {"posx", "posy", "posz", "posa", "posb", "posc"};
#endif
cfgSubtableFromTokenList pos_config_1{"pos", pos_axis_keys, _f0, 5, cm_print_pos, cm_get_pos, set_ro};  // work position
const configSubtable *const getPosConfig_1() { return &pos_config_1; }

// constexpr cfgItem_t ofs_config_items_1[] = {
//     {"ofs", "ofsx", _f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0},  // X work offset
//     {"ofs", "ofsy", _f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0},  // Y work offset
//     {"ofs", "ofsz", _f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0},  // Z work offset
//     {"ofs", "ofsu", _f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0},  // U work offset
//     {"ofs", "ofsv", _f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0},  // V work offset
//     {"ofs", "ofsw", _f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0},  // W work offset
//     {"ofs", "ofsa", _f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0},  // A work offset
//     {"ofs", "ofsb", _f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0},  // B work offset
//     {"ofs", "ofsc", _f0, 5, cm_print_ofs, cm_get_ofs, set_ro, nullptr, 0},  // C work offset
// };
// constexpr cfgSubtableFromStaticArray ofs_config_1{ofs_config_items_1};
#if (AXES == 9)
const char * const ofs_axis_keys[] = {"ofsx", "ofsy", "ofsz", "ofsu", "ofsv", "ofsw", "ofsa", "ofsb", "ofsc"};
#else
const char * const ofs_axis_keys[] = {"ofsx", "ofsy", "ofsz", "ofsa", "ofsb", "ofsc"};
#endif
cfgSubtableFromTokenList ofs_config_1{"ofs", ofs_axis_keys, _f0, 5, cm_print_ofs, cm_get_ofs, set_ro};  // work offsets
const configSubtable *const getOfsConfig_1() { return &ofs_config_1; }

constexpr cfgItem_t hom_config_items_1[] = {
    {"hom", "home", _i0, 0, cm_print_home, cm_get_home, cm_set_home, nullptr, 0},  // homing state, invoke homing cycle
    {"hom", "homx", _i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0},         // X homed - Homing status group
    {"hom", "homy", _i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0},         // Y homed
    {"hom", "homz", _i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0},         // Z homed
    {"hom", "homu", _i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0},         // U homed
    {"hom", "homv", _i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0},         // V homed
    {"hom", "homw", _i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0},         // W homed
    {"hom", "homa", _i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0},         // A homed
    {"hom", "homb", _i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0},         // B homed
    {"hom", "homc", _i0, 0, cm_print_hom, cm_get_hom, set_ro, nullptr, 0},         // C homed
};
constexpr cfgSubtableFromStaticArray hom_config_1{hom_config_items_1};
const configSubtable *const getHomConfig_1() { return &hom_config_1; }

constexpr cfgItem_t prb_config_items_1[] = {
    {"prb", "prbe", _i0, 0, tx_print_nul, cm_get_prob, set_ro, nullptr, 0},    // probing state
    {"prb", "prbx", _f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0},     // X probe results
    {"prb", "prby", _f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0},     // Y probe results
    {"prb", "prbz", _f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0},     // Z probe results
    {"prb", "prbu", _f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0},     // U probe results
    {"prb", "prbv", _f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0},     // V probe results
    {"prb", "prbw", _f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0},     // W probe results
    {"prb", "prba", _f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0},     // A probe results
    {"prb", "prbb", _f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0},     // B probe results
    {"prb", "prbc", _f0, 5, tx_print_nul, cm_get_prb, set_ro, nullptr, 0},     // C probe results
    {"prb", "prbs", _i0, 0, tx_print_nul, get_nul, cm_set_probe, nullptr, 0},  // store probe
    {"prb", "prbr", _bip, 0, tx_print_nul, cm_get_prbr, cm_set_prbr, nullptr,
     PROBE_REPORT_ENABLE},  // enable probe report. Init in cm_init
    {"prb", "prbin", _iip, 0, tx_print_nul, cm_get_probe_input, cm_set_probe_input, nullptr,
     PROBING_INPUT},  // probing input
};
constexpr cfgSubtableFromStaticArray prb_config_1{prb_config_items_1};
const configSubtable *const getPrbConfig_1() { return &prb_config_1; }

// constexpr cfgItem_t jog_config_items_1[] = {
//     {"jog", "jogx", _f0, 0, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},  // jog in X axis
//     {"jog", "jogy", _f0, 0, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},  // jog in Y axis
//     {"jog", "jogz", _f0, 0, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},  // jog in Z axis
//     {"jog", "jogu", _f0, 0, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},  // jog in U axis
//     {"jog", "jogv", _f0, 0, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},  // jog in V axis
//     {"jog", "jogw", _f0, 0, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},  // jog in W axis
//     {"jog", "joga", _f0, 0, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},  // jog in A axis
//     {"jog", "jogb", _f0, 0, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},  // jog in B axis
//     {"jog", "jogc", _f0, 0, tx_print_nul, get_nul, cm_run_jog, nullptr, 0},  // jog in C axis
// };
// constexpr cfgSubtableFromStaticArray jog_config_1{jog_config_items_1};
#if (AXES == 9)
const char * const jog_axis_keys[] = {"jogx", "jogy", "jogz", "jogu", "jogv", "jogw", "joga", "jogb", "jogc"};
#else
const char * const jog_axis_keys[] = {"jogx", "jogy", "jogz", "joga", "jogb", "jogc"};
#endif
cfgSubtableFromTokenList jog_config_1{"jog", jog_axis_keys, _f0, 0, tx_print_nul, get_nul, cm_run_jog};  // job
const configSubtable *const getJogConfig_1() { return &jog_config_1; }

constexpr cfgItem_t axis_config_items_1[] = {
    // Axis parameters
    {"x", "xam", _iip, 0, cm_print_am, cm_get_am, cm_set_am, nullptr, X_AXIS_MODE},
    {"x", "xvm", _fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, X_VELOCITY_MAX},
    {"x", "xfr", _fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, X_FEEDRATE_MAX},
    {"x", "xtn", _fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, X_TRAVEL_MIN},
    {"x", "xtm", _fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, X_TRAVEL_MAX},
    {"x", "xjm", _fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, X_JERK_MAX},
    {"x", "xjh", _fipc, 0, cm_print_jh, cm_get_jh, cm_set_jh, nullptr, X_JERK_HIGH_SPEED},
    {"x", "xhi", _iip, 0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, X_HOMING_INPUT},
    {"x", "xhd", _iip, 0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, X_HOMING_DIRECTION},
    {"x", "xsv", _fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, X_SEARCH_VELOCITY},
    {"x", "xlv", _fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, X_LATCH_VELOCITY},
    {"x", "xlb", _fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, X_LATCH_BACKOFF},
    {"x", "xzb", _fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, X_ZERO_BACKOFF},

    {"y", "yam", _iip, 0, cm_print_am, cm_get_am, cm_set_am, nullptr, Y_AXIS_MODE},
    {"y", "yvm", _fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, Y_VELOCITY_MAX},
    {"y", "yfr", _fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, Y_FEEDRATE_MAX},
    {"y", "ytn", _fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, Y_TRAVEL_MIN},
    {"y", "ytm", _fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, Y_TRAVEL_MAX},
    {"y", "yjm", _fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, Y_JERK_MAX},
    {"y", "yjh", _fipc, 0, cm_print_jh, cm_get_jh, cm_set_jh, nullptr, Y_JERK_HIGH_SPEED},
    {"y", "yhi", _iip, 0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, Y_HOMING_INPUT},
    {"y", "yhd", _iip, 0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, Y_HOMING_DIRECTION},
    {"y", "ysv", _fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, Y_SEARCH_VELOCITY},
    {"y", "ylv", _fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, Y_LATCH_VELOCITY},
    {"y", "ylb", _fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, Y_LATCH_BACKOFF},
    {"y", "yzb", _fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, Y_ZERO_BACKOFF},

    {"z", "zam", _iip, 0, cm_print_am, cm_get_am, cm_set_am, nullptr, Z_AXIS_MODE},
    {"z", "zvm", _fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, Z_VELOCITY_MAX},
    {"z", "zfr", _fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, Z_FEEDRATE_MAX},
    {"z", "ztn", _fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, Z_TRAVEL_MIN},
    {"z", "ztm", _fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, Z_TRAVEL_MAX},
    {"z", "zjm", _fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, Z_JERK_MAX},
    {"z", "zjh", _fipc, 0, cm_print_jh, cm_get_jh, cm_set_jh, nullptr, Z_JERK_HIGH_SPEED},
    {"z", "zhi", _iip, 0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, Z_HOMING_INPUT},
    {"z", "zhd", _iip, 0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, Z_HOMING_DIRECTION},
    {"z", "zsv", _fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, Z_SEARCH_VELOCITY},
    {"z", "zlv", _fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, Z_LATCH_VELOCITY},
    {"z", "zlb", _fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, Z_LATCH_BACKOFF},
    {"z", "zzb", _fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, Z_ZERO_BACKOFF},

#if (AXES == 9)
    {"u", "uam", _iip, 0, cm_print_am, cm_get_am, cm_set_am, nullptr, U_AXIS_MODE},
    {"u", "uvm", _fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, U_VELOCITY_MAX},
    {"u", "ufr", _fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, U_FEEDRATE_MAX},
    {"u", "utn", _fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, U_TRAVEL_MIN},
    {"u", "utm", _fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, U_TRAVEL_MAX},
    {"u", "ujm", _fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, U_JERK_MAX},
    {"u", "ujh", _fipc, 0, cm_print_jh, cm_get_jh, cm_set_jh, nullptr, U_JERK_HIGH_SPEED},
    {"u", "uhi", _iip, 0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, U_HOMING_INPUT},
    {"u", "uhd", _iip, 0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, U_HOMING_DIRECTION},
    {"u", "usv", _fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, U_SEARCH_VELOCITY},
    {"u", "ulv", _fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, U_LATCH_VELOCITY},
    {"u", "ulb", _fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, U_LATCH_BACKOFF},
    {"u", "uzb", _fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, U_ZERO_BACKOFF},

    {"v", "vam", _iip, 0, cm_print_am, cm_get_am, cm_set_am, nullptr, V_AXIS_MODE},
    {"v", "vvm", _fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, V_VELOCITY_MAX},
    {"v", "vfr", _fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, V_FEEDRATE_MAX},
    {"v", "vtn", _fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, V_TRAVEL_MIN},
    {"v", "vtm", _fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, V_TRAVEL_MAX},
    {"v", "vjm", _fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, V_JERK_MAX},
    {"v", "vjh", _fipc, 0, cm_print_jh, cm_get_jh, cm_set_jh, nullptr, V_JERK_HIGH_SPEED},
    {"v", "vhi", _iip, 0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, V_HOMING_INPUT},
    {"v", "vhd", _iip, 0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, V_HOMING_DIRECTION},
    {"v", "vsv", _fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, V_SEARCH_VELOCITY},
    {"v", "vlv", _fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, V_LATCH_VELOCITY},
    {"v", "vlb", _fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, V_LATCH_BACKOFF},
    {"v", "vzb", _fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, V_ZERO_BACKOFF},

    {"w", "wam", _iip, 0, cm_print_am, cm_get_am, cm_set_am, nullptr, W_AXIS_MODE},
    {"w", "wvm", _fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, W_VELOCITY_MAX},
    {"w", "wfr", _fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, W_FEEDRATE_MAX},
    {"w", "wtn", _fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, W_TRAVEL_MIN},
    {"w", "wtm", _fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, W_TRAVEL_MAX},
    {"w", "wjm", _fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, W_JERK_MAX},
    {"w", "wjh", _fipc, 0, cm_print_jh, cm_get_jh, cm_set_jh, nullptr, W_JERK_HIGH_SPEED},
    {"w", "whi", _iip, 0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, W_HOMING_INPUT},
    {"w", "whd", _iip, 0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, W_HOMING_DIRECTION},
    {"w", "wsv", _fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, W_SEARCH_VELOCITY},
    {"w", "wlv", _fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, W_LATCH_VELOCITY},
    {"w", "wlb", _fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, W_LATCH_BACKOFF},
    {"w", "wzb", _fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, W_ZERO_BACKOFF},
#endif

    {"a", "aam", _iip, 0, cm_print_am, cm_get_am, cm_set_am, nullptr, A_AXIS_MODE},
    {"a", "avm", _fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, A_VELOCITY_MAX},
    {"a", "afr", _fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, A_FEEDRATE_MAX},
    {"a", "atn", _fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, A_TRAVEL_MIN},
    {"a", "atm", _fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, A_TRAVEL_MAX},
    {"a", "ajm", _fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, A_JERK_MAX},
    {"a", "ajh", _fipc, 0, cm_print_jh, cm_get_jh, cm_set_jh, nullptr, A_JERK_HIGH_SPEED},
    {"a", "ara", _fipc, 5, cm_print_ra, cm_get_ra, cm_set_ra, nullptr, A_RADIUS},
    {"a", "ahi", _iip, 0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, A_HOMING_INPUT},
    {"a", "ahd", _iip, 0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, A_HOMING_DIRECTION},
    {"a", "asv", _fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, A_SEARCH_VELOCITY},
    {"a", "alv", _fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, A_LATCH_VELOCITY},
    {"a", "alb", _fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, A_LATCH_BACKOFF},
    {"a", "azb", _fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, A_ZERO_BACKOFF},

    {"b", "bam", _iip, 0, cm_print_am, cm_get_am, cm_set_am, nullptr, B_AXIS_MODE},
    {"b", "bvm", _fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, B_VELOCITY_MAX},
    {"b", "bfr", _fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, B_FEEDRATE_MAX},
    {"b", "btn", _fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, B_TRAVEL_MIN},
    {"b", "btm", _fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, B_TRAVEL_MAX},
    {"b", "bjm", _fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, B_JERK_MAX},
    {"b", "bjh", _fipc, 0, cm_print_jh, cm_get_jh, cm_set_jh, nullptr, B_JERK_HIGH_SPEED},
    {"b", "bra", _fipc, 5, cm_print_ra, cm_get_ra, cm_set_ra, nullptr, B_RADIUS},
    {"b", "bhi", _iip, 0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, B_HOMING_INPUT},
    {"b", "bhd", _iip, 0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, B_HOMING_DIRECTION},
    {"b", "bsv", _fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, B_SEARCH_VELOCITY},
    {"b", "blv", _fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, B_LATCH_VELOCITY},
    {"b", "blb", _fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, B_LATCH_BACKOFF},
    {"b", "bzb", _fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, B_ZERO_BACKOFF},

    {"c", "cam", _iip, 0, cm_print_am, cm_get_am, cm_set_am, nullptr, C_AXIS_MODE},
    {"c", "cvm", _fipc, 0, cm_print_vm, cm_get_vm, cm_set_vm, nullptr, C_VELOCITY_MAX},
    {"c", "cfr", _fipc, 0, cm_print_fr, cm_get_fr, cm_set_fr, nullptr, C_FEEDRATE_MAX},
    {"c", "ctn", _fipc, 5, cm_print_tn, cm_get_tn, cm_set_tn, nullptr, C_TRAVEL_MIN},
    {"c", "ctm", _fipc, 5, cm_print_tm, cm_get_tm, cm_set_tm, nullptr, C_TRAVEL_MAX},
    {"c", "cjm", _fipc, 0, cm_print_jm, cm_get_jm, cm_set_jm, nullptr, C_JERK_MAX},
    {"c", "cjh", _fipc, 0, cm_print_jh, cm_get_jh, cm_set_jh, nullptr, C_JERK_HIGH_SPEED},
    {"c", "cra", _fipc, 5, cm_print_ra, cm_get_ra, cm_set_ra, nullptr, C_RADIUS},
    {"c", "chi", _iip, 0, cm_print_hi, cm_get_hi, cm_set_hi, nullptr, C_HOMING_INPUT},
    {"c", "chd", _iip, 0, cm_print_hd, cm_get_hd, cm_set_hd, nullptr, C_HOMING_DIRECTION},
    {"c", "csv", _fipc, 0, cm_print_sv, cm_get_sv, cm_set_sv, nullptr, C_SEARCH_VELOCITY},
    {"c", "clv", _fipc, 2, cm_print_lv, cm_get_lv, cm_set_lv, nullptr, C_LATCH_VELOCITY},
    {"c", "clb", _fipc, 5, cm_print_lb, cm_get_lb, cm_set_lb, nullptr, C_LATCH_BACKOFF},
    {"c", "czb", _fipc, 5, cm_print_zb, cm_get_zb, cm_set_zb, nullptr, C_ZERO_BACKOFF},
};
constexpr cfgSubtableFromStaticArray axis_config_1 {axis_config_items_1};
const configSubtable * const getAxisConfig_1() { return &axis_config_1; }

/***********************************************************************************
 * Debugging Commands
 ***********************************************************************************/
/*
 * cm_dam() - dump active model
 */

stat_t cm_dam(nvObj_t *nv)
{
    xio_writeline("Active model:\n");
    cm_print_vel(nv);
    cm_print_feed(nv);
    cm_print_line(nv);
    cm_print_stat(nv);
    cm_print_macs(nv);
    cm_print_cycs(nv);
    cm_print_mots(nv);
    cm_print_hold(nv);
    cm_print_home(nv);
    cm_print_unit(nv);
    cm_print_coor(nv);
    cm_print_momo(nv);
    cm_print_plan(nv);
    cm_print_path(nv);
    cm_print_dist(nv);
    cm_print_frmo(nv);
    cm_print_tool(nv);

    return (STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

/* model state print functions */

static const char fmt_vel[]  = "Velocity:%17.3f%s/min\n";
static const char fmt_feed[] = "Feed rate:%16.3f%s/min\n";
static const char fmt_line[] = "Line number:%10lu\n";
static const char fmt_stat[] = "Machine state:       %s\n"; // combined machine state
static const char fmt_macs[] = "Raw machine state:   %s\n"; // raw machine state
static const char fmt_cycs[] = "Cycle state:         %s\n";
static const char fmt_mots[] = "Motion state:        %s\n";
static const char fmt_hold[] = "Feedhold state:      %s\n";
static const char fmt_home[] = "Homing state:        %s\n";
static const char fmt_unit[] = "Units:               %s\n"; // units mode as ASCII string
static const char fmt_coor[] = "Coordinate system:   %s\n";
static const char fmt_momo[] = "Motion mode:         %s\n";
static const char fmt_plan[] = "Plane:               %s\n";
static const char fmt_path[] = "Path Mode:           %s\n";
static const char fmt_dist[] = "Distance mode:       %s\n";
static const char fmt_admo[] = "Arc Distance mode:   %s\n";
static const char fmt_frmo[] = "Feed rate mode:      %s\n";
static const char fmt_tool[] = "Tool number          %d\n";
static const char fmt_g92e[] = "G92 enabled          %d\n";

void cm_print_vel(nvObj_t *nv) { text_print_flt_units(nv, fmt_vel, GET_UNITS(ACTIVE_MODEL));}
void cm_print_feed(nvObj_t *nv) { text_print_flt_units(nv, fmt_feed, GET_UNITS(ACTIVE_MODEL));}
void cm_print_line(nvObj_t *nv) { text_print(nv, fmt_line);}     // TYPE_INT
void cm_print_tool(nvObj_t *nv) { text_print(nv, fmt_tool);}     // TYPE_INT
void cm_print_g92e(nvObj_t *nv) { text_print(nv, fmt_g92e);}     // TYPE_INT
void cm_print_stat(nvObj_t *nv) { text_print_str(nv, fmt_stat);} // print all these as TYPE_STRING
void cm_print_macs(nvObj_t *nv) { text_print_str(nv, fmt_macs);} // See _get_msg_helper() for details
void cm_print_cycs(nvObj_t *nv) { text_print_str(nv, fmt_cycs);}
void cm_print_mots(nvObj_t *nv) { text_print_str(nv, fmt_mots);}
void cm_print_hold(nvObj_t *nv) { text_print_str(nv, fmt_hold);}
void cm_print_home(nvObj_t *nv) { text_print_str(nv, fmt_home);}
void cm_print_unit(nvObj_t *nv) { text_print_str(nv, fmt_unit);}
void cm_print_coor(nvObj_t *nv) { text_print_str(nv, fmt_coor);}
void cm_print_momo(nvObj_t *nv) { text_print_str(nv, fmt_momo);}
void cm_print_plan(nvObj_t *nv) { text_print_str(nv, fmt_plan);}
void cm_print_path(nvObj_t *nv) { text_print_str(nv, fmt_path);}
void cm_print_dist(nvObj_t *nv) { text_print_str(nv, fmt_dist);}
void cm_print_admo(nvObj_t *nv) { text_print_str(nv, fmt_admo);}
void cm_print_frmo(nvObj_t *nv) { text_print_str(nv, fmt_frmo);}

static const char fmt_gpl[] = "[gpl] default gcode plane%10d [0=G17,1=G18,2=G19]\n";
static const char fmt_gun[] = "[gun] default gcode units mode%5d [0=G20,1=G21]\n";
static const char fmt_gco[] = "[gco] default gcode coord system%3d [1-6 (G54-G59)]\n";
static const char fmt_gpa[] = "[gpa] default gcode path control%3d [0=G61,1=G61.1,2=G64]\n";
static const char fmt_gdi[] = "[gdi] default gcode distance mode%2d [0=G90,1=G91]\n";

void cm_print_gpl(nvObj_t *nv) { text_print(nv, fmt_gpl);}  // TYPE_INT
void cm_print_gun(nvObj_t *nv) { text_print(nv, fmt_gun);}  // TYPE_INT
void cm_print_gco(nvObj_t *nv) { text_print(nv, fmt_gco);}  // TYPE_INT
void cm_print_gpa(nvObj_t *nv) { text_print(nv, fmt_gpa);}  // TYPE_INT
void cm_print_gdi(nvObj_t *nv) { text_print(nv, fmt_gdi);}  // TYPE_INT

/* system parameter print functions */

static const char fmt_jt[] = "[jt]  junction integration time%7.2f\n";
static const char fmt_ct[] = "[ct]  chordal tolerance%17.4f%s\n";
static const char fmt_zl[] = "[zl]  Z lift on feedhold%16.3f%s\n";
static const char fmt_sl[] = "[sl]  soft limit enable%12d [0=disable,1=enable]\n";
static const char fmt_lim[] ="[lim] limit switch enable%10d [0=disable,1=enable]\n";
static const char fmt_saf[] ="[saf] safety interlock enable%6d [0=disable,1=enable]\n";

void cm_print_jt(nvObj_t *nv) { text_print(nv, fmt_jt);}        // TYPE FLOAT
void cm_print_ct(nvObj_t *nv) { text_print_flt_units(nv, fmt_ct, GET_UNITS(ACTIVE_MODEL));}
void cm_print_zl(nvObj_t *nv) { text_print_flt_units(nv, fmt_zl, GET_UNITS(ACTIVE_MODEL));}
void cm_print_sl(nvObj_t *nv) { text_print(nv, fmt_sl);}        // TYPE_INT
void cm_print_lim(nvObj_t *nv){ text_print(nv, fmt_lim);}       // TYPE_INT
void cm_print_saf(nvObj_t *nv){ text_print(nv, fmt_saf);}       // TYPE_INT

static const char fmt_m48[]  = "[m48] overrides enabled%12d [0=disable,1=enable]\n";
static const char fmt_froe[] = "[froe] feed override enable%8d [0=disable,1=enable]\n";
static const char fmt_fro[]  = "[fro]  feedrate override%15.3f [0.05 < mfo < 2.00]\n";
static const char fmt_troe[] = "[troe] traverse over enable%8d [0=disable,1=enable]\n";
static const char fmt_tro[]  = "[tro]  traverse override%15.3f [0.05 < mto < 1.00]\n";
static const char fmt_tram[] = "[tram] is coordinate space rotated to be tram %s\n";
static const char fmt_nxln[] = "[nxln] next line number %lu\n";

void cm_print_m48(nvObj_t *nv)  { text_print(nv, fmt_m48);}    // TYPE_INT
void cm_print_froe(nvObj_t *nv) { text_print(nv, fmt_froe);}    // TYPE INT
void cm_print_fro(nvObj_t *nv)  { text_print(nv, fmt_fro);}     // TYPE FLOAT
void cm_print_troe(nvObj_t *nv) { text_print(nv, fmt_troe);}    // TYPE INT
void cm_print_tro(nvObj_t *nv)  { text_print(nv, fmt_tro);}     // TYPE FLOAT
void cm_print_tram(nvObj_t *nv) { text_print(nv, fmt_tram);};   // TYPE BOOL
void cm_print_nxln(nvObj_t *nv) { text_print(nv, fmt_nxln);};   // TYPE INT

/*
 * axis print functions
 *
 *    _print_axis_ui8() - helper to print an integer value with no units
 *    _print_axis_flt() - helper to print a floating point linear value in prevailing units
 *    _print_pos_helper()
 *
 *    cm_print_am()
 *    cm_print_fr()
 *    cm_print_vm()
 *    cm_print_tm()
 *    cm_print_tn()
 *    cm_print_jm()
 *    cm_print_jh()
 *    cm_print_ra()
 *    cm_print_hi()
 *    cm_print_hd()
 *    cm_print_lv()
 *    cm_print_lb()
 *    cm_print_zb()
 *
 *    cm_print_pos() - print position with unit displays for MM or Inches
 *    cm_print_mpo() - print position with fixed unit display - always in Degrees or MM
 *    cm_print_tram() - print if the coordinate system is rotated
 */

static const char fmt_Xam[] = "[%s%s] %s axis mode%18d %s\n";
static const char fmt_Xfr[] = "[%s%s] %s feedrate maximum%11.0f%s/min\n";
static const char fmt_Xvm[] = "[%s%s] %s velocity maximum%11.0f%s/min\n";
static const char fmt_Xtm[] = "[%s%s] %s travel maximum%17.3f%s\n";
static const char fmt_Xtn[] = "[%s%s] %s travel minimum%17.3f%s\n";
static const char fmt_Xjm[] = "[%s%s] %s jerk maximum%15.0f%s/min^3 * 1 million\n";
static const char fmt_Xjh[] = "[%s%s] %s jerk homing%16.0f%s/min^3 * 1 million\n";
static const char fmt_Xra[] = "[%s%s] %s radius value%20.4f%s\n";
static const char fmt_Xhi[] = "[%s%s] %s homing input%15d [input 1-N or 0 to disable homing this axis]\n";
static const char fmt_Xhd[] = "[%s%s] %s homing direction%11d [0=search-to-negative, 1=search-to-positive]\n";
static const char fmt_Xsv[] = "[%s%s] %s search velocity%12.0f%s/min\n";
static const char fmt_Xlv[] = "[%s%s] %s latch velocity%13.2f%s/min\n";
static const char fmt_Xlb[] = "[%s%s] %s latch backoff%18.3f%s\n";
static const char fmt_Xzb[] = "[%s%s] %s zero backoff%19.3f%s\n";
static const char fmt_cofs[] = "[%s%s] %s %s offset%20.3f%s\n";
static const char fmt_cpos[] = "[%s%s] %s %s position%18.3f%s\n";

static const char fmt_pos[] = "%c position:%15.3f%s\n";
static const char fmt_mpo[] = "%c machine posn:%11.3f%s\n";
static const char fmt_ofs[] = "%c work offset:%12.3f%s\n";
static const char fmt_tof[] = "%c tool length offset:%12.3f%s\n";
static const char fmt_hom[] = "%c axis homing state:%2.0f\n";

static void _print_axis_ui8(nvObj_t *nv, const char *format)
{
    sprintf(cs.out_buf, format, nv->group, nv->token, nv->group, nv->value_int);
    xio_writeline(cs.out_buf);
}

static void _print_axis_flt(nvObj_t *nv, const char *format)
{
    char *units;
    if (cm_get_axis_type(nv) == AXIS_TYPE_LINEAR) {
        units = (char *)GET_UNITS(MODEL);
    } else {
        units = (char *)GET_TEXT_ITEM(msg_units, DEGREE_INDEX);
    }
    sprintf(cs.out_buf, format, nv->group, nv->token, nv->group, nv->value_flt, units);
    xio_writeline(cs.out_buf);
}

static void _print_axis_coord_flt(nvObj_t *nv, const char *format)
{
    char *units;
    if (cm_get_axis_type(nv) == AXIS_TYPE_LINEAR) {
        units = (char *)GET_UNITS(MODEL);
    } else {
        units = (char *)GET_TEXT_ITEM(msg_units, DEGREE_INDEX);
    }
    sprintf(cs.out_buf, format, nv->group, nv->token, nv->group, nv->token, nv->value_flt, units);
    xio_writeline(cs.out_buf);
}

static void _print_pos(nvObj_t *nv, const char *format, uint8_t units)
{
    char axes[] = {"XYZABC"};
    uint8_t axis = _axis(nv);
    if (axis >= AXIS_A) { units = DEGREES;}
    sprintf(cs.out_buf, format, axes[axis], nv->value_flt, GET_TEXT_ITEM(msg_units, units));
    xio_writeline(cs.out_buf);
}

static void _print_hom(nvObj_t *nv, const char *format)
{
    char axes[] = {"XYZABC"};
    uint8_t axis = _axis(nv);
    sprintf(cs.out_buf, format, axes[axis], nv->value_int);
    xio_writeline(cs.out_buf);
}

void cm_print_am(nvObj_t *nv)    // print axis mode with enumeration string
{
    sprintf(cs.out_buf, fmt_Xam, nv->group, nv->token, nv->group, (int)nv->value_int,
        GET_TEXT_ITEM(msg_am, nv->value_int));
    xio_writeline(cs.out_buf);
}

void cm_print_fr(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xfr);}
void cm_print_vm(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xvm);}
void cm_print_tm(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xtm);}
void cm_print_tn(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xtn);}
void cm_print_jm(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xjm);}
void cm_print_jh(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xjh);}
void cm_print_ra(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xra);}

void cm_print_hi(nvObj_t *nv) { _print_axis_ui8(nv, fmt_Xhi);}
void cm_print_hd(nvObj_t *nv) { _print_axis_ui8(nv, fmt_Xhd);}
void cm_print_sv(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xsv);}
void cm_print_lv(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xlv);}
void cm_print_lb(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xlb);}
void cm_print_zb(nvObj_t *nv) { _print_axis_flt(nv, fmt_Xzb);}

void cm_print_cofs(nvObj_t *nv) { _print_axis_coord_flt(nv, fmt_cofs);}
void cm_print_cpos(nvObj_t *nv) { _print_axis_coord_flt(nv, fmt_cpos);}

void cm_print_pos(nvObj_t *nv) { _print_pos(nv, fmt_pos, cm_get_units_mode(MODEL));}
void cm_print_mpo(nvObj_t *nv) { _print_pos(nv, fmt_mpo, MILLIMETERS);}
void cm_print_ofs(nvObj_t *nv) { _print_pos(nv, fmt_ofs, MILLIMETERS);}
void cm_print_tof(nvObj_t *nv) { _print_pos(nv, fmt_tof, MILLIMETERS);}
void cm_print_hom(nvObj_t *nv) { _print_hom(nv, fmt_hom);}

#endif // __TEXT_MODE
