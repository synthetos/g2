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

#ifndef BANTAM_SAFETY_MANAGER_H_ONCE
#define BANTAM_SAFETY_MANAGER_H_ONCE

#include "safety_manager.h"
#include "hardware.h"

#include "text_parser.h"

class BantamSafetyManager;
extern BantamSafetyManager *bsm;

class BantamSafetyManager : public SafetyManager {
    typedef enum {
        ESTOP_RELEASED = 0,  // pressed/released is physical state, acked/unacked is machine control state,
                             // active/inactive is whether we're currently in estop mode
        ESTOP_ACKED = 0,
        ESTOP_INACTIVE = 0,
        ESTOP_PRESSED = 0x1,
        ESTOP_UNACKED = 0x2,
        ESTOP_ACTIVE = 0x4,

        ESTOP_ACTIVE_MASK = 0x4,
        ESTOP_ACK_MASK = 0x2,
        ESTOP_PRESSED_MASK = 0x1,
    } cmEstopState;

    typedef enum {
        SAFETY_INTERLOCK_CLOSED = 0,
        SAFETY_INTERLOCK_OPEN = 0x1,

        SAFETY_ESC_ONLINE = 0,
        SAFETY_ESC_OFFLINE = 0x2,
        SAFETY_ESC_LOCKOUT = 0x4,
        SAFETY_ESC_REBOOTING = 0x8,
        SAFETY_ESC_LOCKOUT_AND_REBOOTING = 0xC,

        SAFETY_INTERLOCK_MASK = 0x1,
        SAFETY_ESC_MASK = 0xE,
    } cmBantamSafetyState;

    uint8_t safety_state;               // Tracks whether interlock has been triggered, whether esc is rebooting, etc
    uint8_t estop_state;                // Whether estop has been triggered
    Motate::Timeout esc_boot_timer;     // When the ESC last booted up
    Motate::Timeout esc_lockout_timer;  // When the ESC lockout last triggered

   public:
    void init() override {
        SafetyManager::init();

        safety_state = estop_state = 0;
        esc_boot_timer.set(ESC_BOOT_TIME);
        safety_state = SAFETY_ESC_REBOOTING;

        // if (gpio_read_input(INTERLOCK_SWITCH_INPUT) == INPUT_ACTIVE) {
        //     // safety_interlock_disengaged = INTERLOCK_SWITCH_INPUT;
        //     safety_state |= SAFETY_INTERLOCK_OPEN;
        // }

        bsm = this;
    }

    bool ok_to_spindle() override {
        if ((estop_state != 0) || (safety_state != 0)) {
            return false;
        }

        // otherwise safe
        return SafetyManager::ok_to_spindle();
    }

    bool can_clear() override {
        if ((estop_state != 0) || SafetyManager::can_clear()) {
            return true;
        }
        return false;
    }

    bool can_queue_flush() override {
        return (estop_state == 0);
    }

    stat_t is_system_alarmed() override {
        if (estop_state != 0)                  { return (STAT_COMMAND_REJECTED_BY_SHUTDOWN); }
        return SafetyManager::is_system_alarmed();
    }

    stat_t handle_interlock() override {
        bool report = false;

        // Process E-Stop and Interlock signals

        // Door opened and was closed
        if ((safety_state & SAFETY_INTERLOCK_MASK) == SAFETY_INTERLOCK_CLOSED && (gpio_read_input(INTERLOCK_SWITCH_INPUT) == INPUT_ACTIVE)) {
            safety_state |= SAFETY_INTERLOCK_OPEN;

            // Check if the spindle is on
            if (is_spindle_on_or_paused()) {
                if (cm1.machine_state == MACHINE_CYCLE) {
                    cm_request_feedhold(FEEDHOLD_TYPE_ACTIONS, FEEDHOLD_EXIT_CYCLE);
                } else {
                    spindle_stop();
                }
            }

            // If we just entered interlock and we're not off, start the lockout timer
            if ((safety_state & SAFETY_ESC_MASK) == SAFETY_ESC_ONLINE ||
                (safety_state & SAFETY_ESC_MASK) == SAFETY_ESC_REBOOTING) {
                esc_lockout_timer.set(ESC_LOCKOUT_TIME);
                safety_state |= SAFETY_ESC_LOCKOUT;
            }
            report = true;

        // Door closed and was open
        } else if ((safety_state & SAFETY_INTERLOCK_MASK) == SAFETY_INTERLOCK_OPEN && (gpio_read_input(INTERLOCK_SWITCH_INPUT) == INPUT_INACTIVE)) {
            safety_state &= ~SAFETY_INTERLOCK_OPEN;
            // If we just left interlock, stop the lockout timer
            if ((safety_state & SAFETY_ESC_LOCKOUT) == SAFETY_ESC_LOCKOUT) {
                safety_state &= ~SAFETY_ESC_LOCKOUT;
                esc_lockout_timer.clear();
            }
            report = true;
        }

        // EStop was pressed
        if ((estop_state & ESTOP_PRESSED_MASK) == ESTOP_RELEASED && gpio_read_input(ESTOP_SWITCH_INPUT) == INPUT_ACTIVE) {
            estop_state = ESTOP_PRESSED | ESTOP_UNACKED | ESTOP_ACTIVE;
            cm_shutdown(STAT_SHUTDOWN, "e-stop pressed");

            // E-stop always sets the ESC to off
            safety_state &= ~SAFETY_ESC_MASK;
            safety_state |= SAFETY_ESC_OFFLINE;
            report = true;

        // EStop was released
        } else if ((estop_state & ESTOP_PRESSED_MASK) == ESTOP_PRESSED && gpio_read_input(ESTOP_SWITCH_INPUT) == INPUT_INACTIVE) {
            estop_state &= ~ESTOP_PRESSED;
            report = true;
        }

        // if E-Stop and Interlock are both 0, and we're off, go into "ESC Reboot"
        if ((safety_state & SAFETY_ESC_MASK) == SAFETY_ESC_OFFLINE && (estop_state & ESTOP_PRESSED) == 0 && (safety_state & SAFETY_INTERLOCK_OPEN) == 0) {
            safety_state &= ~SAFETY_ESC_MASK;
            safety_state |= SAFETY_ESC_REBOOTING;
            esc_boot_timer.set(ESC_BOOT_TIME);
            report = true;
        }

        // Check if ESC lockout timer or reboot timer have expired
        if ((safety_state & SAFETY_ESC_LOCKOUT) != 0 && esc_lockout_timer.isPast()) {
            safety_state &= ~SAFETY_ESC_MASK;
            safety_state |= SAFETY_ESC_OFFLINE;
            report = true;
        }
        if ((safety_state & SAFETY_ESC_MASK) == SAFETY_ESC_REBOOTING && esc_boot_timer.isPast()) {
            safety_state &= ~SAFETY_ESC_MASK;
            report = true;
        }

        // If we've successfully ended all the ESTOP conditions, then end ESTOP
        if (estop_state == ESTOP_ACTIVE) {
            estop_state = 0;
            report = true;
        }

        if (report) {
            sr_request_status_report(SR_REQUEST_IMMEDIATE);
        }
        return (STAT_OK);
    }

    uint8_t get_estop_state() { return estop_state; }
    void ack_estop() {
            estop_state &= ~BantamSafetyManager::ESTOP_UNACKED;
    }

    uint8_t get_interlock_safety_state() { return (safety_state & BantamSafetyManager::SAFETY_INTERLOCK_MASK) != 0; }
    uint8_t get_esc_safety_state() { return (safety_state & BantamSafetyManager::SAFETY_ESC_MASK) != 0; }
};

#endif //BANTAM_SAFETY_MANAGER_H_ONCE
