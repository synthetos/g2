
#include "can_bus.h"

#ifdef CAN_ENABLED

//#include "can_routing.h"

void can_send_message (uint32_t id, uint8_t length, uint8_t* data) {
  hw_can_send_frame(id, length, data);
}

void can_message_received (uint32_t id, uint8_t length, uint8_t* data) {
  switch (id) {
    case 0: break;

  }
}

void can_digital_output (uint8_t pin_num, bool value) {
  uint8_t data = (uint8_t)value;
  switch (pin_num) {
    case 1: can_send_message(100, 1, &data); break;
  }
}

/*
void canDigitalInput::reset() {
    if (D_IN_CHANNELS < ext_pin_number) { return; }

    d_in_t *in = &d_in[ext_pin_number-1];

    if (in->mode == IO_MODE_DISABLED) {
        in->state = INPUT_DISABLED;
        return;
    }

    bool pin_value = (bool)input_pin;
    int8_t pin_value_corrected = (pin_value ^ ((int)in->mode ^ 1));    // correct for NO or NC mode
    in->state = (ioState)pin_value_corrected;
    inReset=true;
}

void canDigitalInput::message_received (uint8_t *data) {

    d_in_t *in = &d_in[ext_pin_number-1];

    // return if input is disabled (not supposed to happen)
    if (in->mode == IO_MODE_DISABLED) {
        in->state = INPUT_DISABLED;
        return;
    }

    // return if the input is in lockout period (take no action)
    if (in->lockout_timer.isSet() && !in->lockout_timer.isPast()) {
        return;
    }

    // return if no change in state
    bool pin_value = (bool)input_pin;
    int8_t pin_value_corrected = (pin_value ^ ((int)in->mode ^ 1));    // correct for NO or NC mode
    if (in->state == (ioState)pin_value_corrected) {
        return;
    }

    // lockout the pin for lockout_ms
    in->lockout_timer.set(in->lockout_ms);

    // record the changed state
    in->state = (ioState)pin_value_corrected;
    if (pin_value_corrected == INPUT_ACTIVE) {
        in->edge = INPUT_EDGE_LEADING;
    } else {
        in->edge = INPUT_EDGE_TRAILING;
    }

    // perform homing operations if in homing mode
    if (in->homing_mode) {
        if (in->edge == INPUT_EDGE_LEADING) {   // we only want the leading edge to fire
            en_take_encoder_snapshot();
            cm_start_hold();
        }
        return;
    }

    // perform probing operations if in probing mode
    if (in->probing_mode) {
        // We want to capture either way.
        // Probing tests the start condition for the correct direction ahead of time.
        // If we see any edge, it's the right one.
        en_take_encoder_snapshot();
        cm_start_hold();
        return;
    }

    // NOTE: From this point on all conditionals assume we are NOT in homing or probe mode

    // trigger the action on leading edges
    if (in->edge == INPUT_EDGE_LEADING) {
        if (in->action == INPUT_ACTION_STOP) {
            cm_start_hold();
        }
        if (in->action == INPUT_ACTION_FAST_STOP) {
            cm_start_hold();                        // for now is same as STOP
        }
        if (in->action == INPUT_ACTION_HALT) {
            cm_halt_all();                            // hard stop, including spindle and coolant
        }
        if (in->action == INPUT_ACTION_ALARM) {
            char msg[10];
            sprintf(msg, "input %d", ext_pin_number);
            cm_alarm(STAT_ALARM, msg);
        }
        if (in->action == INPUT_ACTION_SHUTDOWN) {
            char msg[10];
            sprintf(msg, "input %d", ext_pin_number);
            cm_shutdown(STAT_SHUTDOWN, msg);
        }
        if (in->action == INPUT_ACTION_PANIC) {
            char msg[10];
            sprintf(msg, "input %d", ext_pin_number);
            cm_panic(STAT_PANIC, msg);
        }
        if (in->action == INPUT_ACTION_RESET) {
            hw_hard_reset();
        }
    }

    // these functions trigger on the leading edge
    if (in->edge == INPUT_EDGE_LEADING) {
        if (in->function == INPUT_FUNCTION_LIMIT) {
            cm.limit_requested = ext_pin_number;

        } else if (in->function == INPUT_FUNCTION_SHUTDOWN) {
            cm.shutdown_requested = ext_pin_number;

        } else if (in->function == INPUT_FUNCTION_INTERLOCK) {
            cm.safety_interlock_disengaged = ext_pin_number;
        }
    }

    // trigger interlock release on trailing edge
    if (in->edge == INPUT_EDGE_TRAILING) {
        if (in->function == INPUT_FUNCTION_INTERLOCK) {
            cm.safety_interlock_reengaged = ext_pin_number;
        }
    }

    sr_request_status_report(SR_REQUEST_TIMED);   //+++++ Put this one back in.
}
*/

#endif
