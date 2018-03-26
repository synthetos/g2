/*
 * alarm.cpp - canonical machine alarm handlers
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
#include "report.h"
#include "spindle.h"
#include "coolant.h"
#include "temperature.h"
#include "util.h"

/****************************************************************************************
 * ALARM, SHUTDOWN, and PANIC are nested dolls.
 *
 * cm_alrm()  - invoke alarm from command
 * cm_shutd() - invoke shutdown from command
 * cm_pnic()  - invoke panic from command
 * cm_clr()   - clear alarm or shutdown from command
 *
 * The alarm states can be invoked from the above commands for testing and clearing
 */
stat_t cm_alrm(nvObj_t *nv)               // invoke alarm from command
{
    cm_alarm(STAT_ALARM, "sent by host");
    return (STAT_OK);
}

stat_t cm_shutd(nvObj_t *nv)              // invoke shutdown from command
{
    cm_shutdown(STAT_SHUTDOWN, "sent by host");
    return (STAT_OK);
}

stat_t cm_pnic(nvObj_t *nv)               // invoke panic from command
{
    cm_panic(STAT_PANIC, "sent by host");
    return (STAT_OK);
}

stat_t cm_clr(nvObj_t *nv)                // clear alarm or shutdown from command line
{
    cm_clear();
    return (STAT_OK);
}

/****************************************************************************************
 * cm_clear() - clear ALARM and SHUTDOWN states
 * cm_parse_clear() - parse incoming gcode for M30 or M2 clears if in ALARM state
 *
 * Parse clear interprets an M30 or M2 PROGRAM_END as a $clear condition and clear ALARM
 * but not SHUTDOWN or PANIC. Assumes Gcode string has no leading or embedded whitespace
 */

void cm_clear()
{
    if (cm->machine_state == MACHINE_ALARM) {
        cm->machine_state = MACHINE_PROGRAM_STOP;
    } else if (cm->machine_state == MACHINE_SHUTDOWN) {
        cm->machine_state = MACHINE_READY;
    }
}

void cm_parse_clear(const char *s)
{
    if (cm->machine_state == MACHINE_ALARM) {
        if (toupper(s[0]) == 'M') {
            if (( (s[1]=='3') && (s[2]=='0') && (s[3]==0)) || ((s[1]=='2') && (s[2]==0) )) {
                cm_clear();
            }
        }
    }
}

/****************************************************************************************
 * cm_is_alarmed() - return alarm status code or OK if no alarms
 */

stat_t cm_is_alarmed()
{
    if (cm->machine_state == MACHINE_ALARM)    { return (STAT_COMMAND_REJECTED_BY_ALARM); }
    if (cm->machine_state == MACHINE_SHUTDOWN) { return (STAT_COMMAND_REJECTED_BY_SHUTDOWN); }
    if (cm->machine_state == MACHINE_PANIC)    { return (STAT_COMMAND_REJECTED_BY_PANIC); }
    return (STAT_OK);
}

/****************************************************************************************
 * cm_halt() - stop motion, spindle, coolant and heaters immediately
 * cm_halt_motion() - stop motion immediately. Does not affect spindle, coolant, or other IO
 *
 * Stop motors and reset all system states accordingly.
 * Does not de-energize motors as in some cases the motors must remain energized
 * in order to prevent an axis from crashing.
 */

void cm_halt(void)
{
    cm_halt_motion();
    spindle_control_immediate(SPINDLE_OFF);
    coolant_control_immediate(COOLANT_OFF, COOLANT_BOTH);
    temperature_init();
}

void cm_halt_motion(void)
{
    mp_halt_runtime();                  // stop the runtime. Do this immediately. (Reset is in cm_clear)
    canonical_machine_reset(cm);        // halt the currently active machine
    cm->cycle_type = CYCLE_NONE;        // Note: leaves machine_state alone
    cm->motion_state = MOTION_STOP;
    cm->hold_state = FEEDHOLD_OFF;
}

/****************************************************************************************
 * cm_alarm() - enter ALARM state
 *
 * An ALARM sets the ALARM machine state, starts a feedhold to stop motion, stops the
 * spindle, turns off coolant, clears out queued planner moves and serial input,
 * and rejects new action commands (gcode blocks, SET commands, and other actions)
 * until the alarm is cleared.
 *
 * ALARM is typically entered by a soft limit or a limit switch being hit. In the
 * limit switch case the INPUT_ACTION will override the feedhold - i.e. if the
 * input action is "FAST_STOP" or "HALT" that setting will take precedence over
 * the feedhold native to the alarm function.
 *
 * Gcode and machine state is preserved. It may be possible to recover the job from
 * an alarm, but in many cases this is not possible. Since ALARM attempts to preserve
 * Gcode and machine state it does not END the job.
 *
 * ALARM may also be invoked from the command line using {alarm:n} or $alarm
 * ALARM can be manually cleared by entering: {clear:n}, {clr:n}, $clear, or $clr
 * ALARMs will also clear on receipt of an M30 or M2 command if one is received
 * while draining the host command queue.
 */

stat_t cm_alarm(const stat_t status, const char *msg)
{
    if ((cm->machine_state == MACHINE_ALARM) || (cm->machine_state == MACHINE_SHUTDOWN) ||
        (cm->machine_state == MACHINE_PANIC)) {
        return (STAT_OK);                       // don't alarm if already in an alarm state
    }
    cm_request_feedhold(FEEDHOLD_TYPE_SCRAM, FEEDHOLD_EXIT_ALARM);  // fast stop and alarm
    rpt_exception(status, msg);                 // send alarm message
    sr_request_status_report(SR_REQUEST_TIMED);
    return (status);
}

/****************************************************************************************
 * cm_shutdown() - enter shutdown state
 *
 * SHUTDOWN stops all motion, spindle and coolant immediately, sets a SHUTDOWN machine
 * state, clears out queued moves and serial input, and rejects new action commands
 * (gcode blocks, SET commands, and some others).
 *
 * Shutdown is typically invoked as an electrical input signal sent to the board as
 * part of an external emergency stop (Estop). Shutdown is meant to augment but not
 * replace the external Estop functions that shut down power to motors, spindles and
 * other moving parts.
 *
 * Shutdown may also be invoked from the command line using {shutd:n} or $shutd
 * Shutdown must be manually cleared by entering: {clear:n}, {clr:n}, $clear, or $clr
 * Shutdown does not clear on M30 or M2 Gcode commands
 */

stat_t cm_shutdown(const stat_t status, const char *msg)
{
    if ((cm->machine_state == MACHINE_SHUTDOWN) || (cm->machine_state == MACHINE_PANIC)) {
        return (STAT_OK);                       // don't shutdown if shutdown or panic'd
    }
    cm_request_feedhold(FEEDHOLD_TYPE_SCRAM, FEEDHOLD_EXIT_SHUTDOWN);  // fast stop and shutdown

//    spindle_reset();                            // stop spindle immediately and set speed to 0 RPM
//    coolant_reset();                            // stop coolant immediately
//    temperature_reset();                        // turn off heaters and fans
//    cm_queue_flush(&cm1);                       // flush all queues and reset positions

    for (uint8_t i = 0; i < HOMING_AXES; i++) { // unhome axes and the machine
        cm->homed[i] = false;
    }
    cm->homing_state = HOMING_NOT_HOMED;

//    cm1.machine_state = MACHINE_SHUTDOWN;       // shut down both machines...
//    cm2.machine_state = MACHINE_SHUTDOWN;       //...do this after all other activity
    rpt_exception(status, msg);                 // send exception report
    sr_request_status_report(SR_REQUEST_TIMED);
    return (status);
}

/****************************************************************************************
 * cm_panic() - enter panic state
 *
 * PANIC occurs if the firmware has detected an unrecoverable internal error
 * such as an assertion failure or a code condition that should never occur.
 * It sets PANIC machine state, and leaves the system inspect able (if possible).
 *
 * PANIC can only be exited by a hardware reset or soft reset (^x)
 */

stat_t cm_panic(const stat_t status, const char *msg)
{
    debug_trap(msg);

    if (cm->machine_state == MACHINE_PANIC) {    // only do this once
        return (STAT_OK);
    }
    cm_halt_motion();                           // halt motors (may have already been done from GPIO)
    spindle_reset();                            // stop spindle immediately and set speed to 0 RPM
    coolant_reset();                            // stop coolant immediately
    temperature_reset();                        // turn off heaters and fans

    cm1.machine_state = MACHINE_PANIC;          // don't reset anything. Panics are not recoverable
    cm2.machine_state = MACHINE_PANIC;          // don't reset anything. Panics are not recoverable
    rpt_exception(status, msg);                 // send panic report
    return (status);
}
