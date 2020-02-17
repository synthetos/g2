/*
 * safety_manager.h - The safety manager handles interlock and spindle safety controls
 * This file is part of the g2core project
 *
 * Copyright (c) 2019 Rob Giseburt
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

#ifndef SAFETY_MANAGER_H_ONCE
#define SAFETY_MANAGER_H_ONCE

#include "canonical_machine.h"
#include "planner.h"
#include "gpio.h"

class SafetyManager; // forward declaration
extern SafetyManager *safety_manager;  // setup in hardware.cpp

// class to optionally override functions for managing safety functions
// default implementation handles interlock and alarm/shutdown states
class SafetyManager {
    typedef enum {
        SAFETY_INTERLOCK_ENGAGED = 0,   // meaning the interlock input is CLOSED (low)
        SAFETY_INTERLOCK_DISENGAGING,   // meaning the interlock opened and we're dealing with it
        SAFETY_INTERLOCK_DISENGAGED,
        SAFETY_INTERLOCK_ENGAGING
    } cmSafetyState;

    uint8_t shutdown_requested;             // set non-zero to request shutdown in support of external estop (value is input number)
    bool safety_interlock_enable;           // true to enable safety interlock system
    bool request_interlock;                 // enter interlock
    bool request_interlock_exit;            // exit interlock
    uint8_t safety_interlock_disengaged;    // set non-zero to start interlock processing (value is input number)
    uint8_t safety_interlock_reengaged;     // set non-zero to end interlock processing (value is input number)
    cmSafetyState safety_interlock_state;   // safety interlock state

    gpioDigitalInputHandler _shutdown_input_handler {
        [](const bool state, const inputEdgeFlag edge, const uint8_t triggering_pin_number) {
            if (edge != INPUT_EDGE_LEADING) { return GPIO_NOT_HANDLED; }

            safety_manager->shutdown_requested = triggering_pin_number;

            return GPIO_HANDLED;
        },
        5,    // priority
        nullptr // next - nullptr to start with
    };
    gpioDigitalInputHandler _interlock_input_handler {
        [](const bool state, const inputEdgeFlag edge, const uint8_t triggering_pin_number) {
            if (edge != INPUT_EDGE_LEADING) {
                safety_manager->safety_interlock_disengaged = triggering_pin_number;
            } else { // edge == INPUT_EDGE_TRAILING
                safety_manager->safety_interlock_reengaged = triggering_pin_number;
            }

            return GPIO_HANDLED;
        },
        5,       // priority
        nullptr  // next - nullptr to start with
    };

   public:
    virtual void init() {
        safety_interlock_disengaged = 0;           // ditto
        safety_interlock_reengaged = 0;            // ditto
        shutdown_requested = 0;                    // ditto
        request_interlock = false;
        request_interlock_exit = false;

        din_handlers[INPUT_ACTION_SHUTDOWN].registerHandler(&_shutdown_input_handler);
        din_handlers[INPUT_ACTION_INTERLOCK].registerHandler(&_interlock_input_handler);
    }

    virtual bool ok_to_spindle() {
        // default, disable the spindle in interlock, alarm, shutdown, and panic states
        if ((cm1.machine_state == MACHINE_INTERLOCK) || (cm1.machine_state == MACHINE_ALARM) ||
            (cm1.machine_state == MACHINE_SHUTDOWN) || (cm1.machine_state == MACHINE_PANIC)) {
            return false;
        }

        // otherwise safe
        return true;
    }

    virtual bool ok_to_coolant() {
        // default, disable the coolant if the spindle isn't allowed
        return this->ok_to_spindle();
    }

    virtual bool can_clear() {
        if ((cm->machine_state == MACHINE_ALARM) || (cm->machine_state == MACHINE_SHUTDOWN)) {
            return true;
        }
        return false;
    }

    virtual bool can_queue_flush() {
        return true;
    }

    virtual stat_t is_system_alarmed() {
        if (cm->machine_state == MACHINE_ALARM)    { return (STAT_COMMAND_REJECTED_BY_ALARM); }
        if (cm->machine_state == MACHINE_SHUTDOWN) { return (STAT_COMMAND_REJECTED_BY_SHUTDOWN); }
        if (cm->machine_state == MACHINE_PANIC)    { return (STAT_COMMAND_REJECTED_BY_PANIC); }
        return (STAT_OK);
    }

    virtual stat_t handle_shutdown () {
        // called from periodic handler - useful to partially override it

        // SHUTDOWN handling
        if (shutdown_requested != 0) {  // request may contain the (non-zero) input number
            char msg[10];
            sprintf(msg, "input %d", (int)shutdown_requested);
            shutdown_requested = false; // clear limit request used here
            cm_shutdown(STAT_SHUTDOWN, msg);
        }
        return(STAT_OK);
    }

    virtual stat_t handle_interlock() {
        // called from periodic handler - useful to partially override it

        // INTERLOCK handling
        if (safety_interlock_enable) {
            // interlock broken
            if ((safety_interlock_disengaged != 0) && (safety_interlock_state == SAFETY_INTERLOCK_ENGAGED)) {
                safety_interlock_disengaged = 0;
                safety_interlock_state = SAFETY_INTERLOCK_DISENGAGING;
                cm_request_feedhold(FEEDHOLD_TYPE_ACTIONS, FEEDHOLD_EXIT_INTERLOCK);  // may have already requested STOP as INPUT_ACTION
            }

            // interlock restored
            if ((safety_interlock_reengaged != 0) && mp_runtime_is_idle() && (safety_interlock_state == SAFETY_INTERLOCK_DISENGAGED)) {
                safety_interlock_reengaged = 0;
                safety_interlock_state = SAFETY_INTERLOCK_ENGAGING;  // interlock restored
                cm_request_cycle_start();                               // proper way to restart the cycle
            }
        }
        return(STAT_OK);
    }

    virtual stat_t periodic_handler() {
        ritorno(this->handle_shutdown());
        ritorno(this->handle_interlock());
        return(STAT_OK);
    }

    virtual void start_interlock_after_feedhold() {
        safety_interlock_state = SAFETY_INTERLOCK_DISENGAGED;
    }

    virtual void end_interlock_after_feedhold() {
        safety_interlock_state = SAFETY_INTERLOCK_ENGAGED;
    }

    virtual bool get_interlock_enable() { return safety_interlock_enable; }
    virtual void set_interlock_enable(bool enable) { safety_interlock_enable = enable; }
};

stat_t cm_get_saf(nvObj_t *nv);         // get safety interlock enable
stat_t cm_set_saf(nvObj_t *nv);         // set safety interlock enable

#endif // SAFETY_MANAGER_H_ONCE
