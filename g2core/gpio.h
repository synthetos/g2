/*
 * gpio.h - Digital IO  handling functions
 * This file is part of the g2core project
 *
 * Copyright (c) 2015 - 2017 Alden S. Hart, Jr.
 * Copyright (c) 2015 - 2017 Robert Giseburt
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

#ifndef GPIO_H_ONCE
#define GPIO_H_ONCE

#include "g2core.h"  // #1
#include "config.h"  // #2
#include "encoder.h" // for post-probe/homing encoder grabbing
#include "report.h"  // for sr triggering
#include "canonical_machine.h" // for pin-change function/action handling

// Note: "board_gpio.h" is inlcluded at the end of this file

#include <utility> // for std::forward

#include "MotatePins.h"
using Motate::kPullUp;
using Motate::kDebounce;
using Motate::kStartLow;
using Motate::kPWMPinInverted;

/*
 * GPIO defines
 */

// moved to board_gpio.h, which is copied for each board in subfolders of board/

//--- do not change from here down ---//

typedef enum {
    IO_ACTIVE_LOW = 0,                  // input/output is active low (aka normally open)
    IO_ACTIVE_HIGH = 1,                 // input/output is active high (aka normally closed)
    IO_MODE_DISABLED = 2                // input/output is disabled
} ioMode;
#define NORMALLY_OPEN   IO_ACTIVE_LOW   // equivalent
#define NORMALLY_CLOSED IO_ACTIVE_HIGH  // equivalent
#define IO_MODE_MAX     IO_MODE_DISABLED // for range checking

typedef enum {                          // actions are initiated from within the input's ISR
    INPUT_ACTION_NONE = 0,
    INPUT_ACTION_STOP,                  // stop at normal jerk - preserves positional accuracy
    INPUT_ACTION_FAST_STOP,             // stop at high jerk - preserves positional accuracy
    INPUT_ACTION_HALT,                  // stop immediately - not guaranteed to preserve position
    INPUT_ACTION_CYCLE_START,           // start / restart cycle after feedhold (RESERVED)
    INPUT_ACTION_ALARM,                 // initiate an alarm. stops everything immediately - preserves position
    INPUT_ACTION_SHUTDOWN,              // initiate a shutdown. stops everything immediately - does not preserve position
    INPUT_ACTION_PANIC,                 // initiate a panic. stops everything immediately - does not preserve position
    INPUT_ACTION_RESET,                 // reset system
} inputAction;
#define INPUT_ACTION_MAX INPUT_ACTION_RESET // for range checking

typedef enum {                          // functions are requested from the ISR, run from the main loop
    INPUT_FUNCTION_NONE = 0,
    INPUT_FUNCTION_LIMIT = 1,           // limit switch processing
    INPUT_FUNCTION_INTERLOCK = 2,       // interlock processing
    INPUT_FUNCTION_SHUTDOWN = 3,        // shutdown in support of external emergency stop
    INPUT_FUNCTION_PROBE = 4,           // assign input as probe input
} inputFunc;
#define INPUT_FUNCTION_MAX INPUT_FUNCTION_PROBE

typedef enum {
    INPUT_INACTIVE = 0,                 // aka switch open, also read as 'false'
    INPUT_ACTIVE = 1,                   // aka switch closed, also read as 'true'
    INPUT_DISABLED = 2                  // value returned if input is disabled
} ioState;

typedef enum {
    INPUT_EDGE_NONE = 0,                // no edge detected or edge flag reset (must be zero)
    INPUT_EDGE_LEADING,                 // flag is set when leading edge is detected
    INPUT_EDGE_TRAILING                 // flag is set when trailing edge is detected
} inputEdgeFlag;

/*
 * GPIO structures
 */
struct gpioDigitalInput {
    // this is the implementation for a non-existant pin - see gpioDigitalInputPin for a real pin

    // functions for use by other parts of the code

    virtual bool getState();

    virtual inputFunc getFunction();
    virtual bool setFunction(const inputFunc);

    virtual inputAction getAction();
    virtual bool setAction(const inputAction);

    virtual ioMode getMode();
    virtual bool setMode(const ioMode);

    virtual void setExternalNumber(const uint8_t);
    virtual void setIsHoming(const bool);
    virtual void setIsProbing(const bool);


    // functions that take nvObj_t* and return stat_t, NOT overridden

    stat_t getState(nvObj_t *nv)
    {
        if (getMode() == IO_MODE_DISABLED) {
            nv->value = IO_MODE_DISABLED; // historical consistency
            nv->valuetype = TYPE_NULL;
            return (STAT_OK);
        }
        nv->value = getState();
        nv->valuetype = TYPE_BOOL;
        return (STAT_OK);
    };
    // no setState

    stat_t getMode(nvObj_t *nv)
    {
        nv->value = getMode();
        nv->valuetype = TYPE_INT;
        return (STAT_OK);
    };
    stat_t setMode(nvObj_t *nv)
    {
        if ((nv->value < IO_ACTIVE_LOW) || (nv->value >= IO_MODE_MAX)) {
            return (STAT_INPUT_VALUE_RANGE_ERROR);
        }
        if (!setMode((ioMode)nv->value)) {
            return STAT_PARAMETER_IS_READ_ONLY;
        }
        return (STAT_OK);
    };

    stat_t getAction(nvObj_t *nv)
    {
        nv->value = getAction();
        nv->valuetype = TYPE_INT;
        return (STAT_OK);
    };
    stat_t setAction(nvObj_t *nv)
    {
        if ((nv->value < INPUT_ACTION_NONE) || (nv->value >= INPUT_ACTION_MAX)) {
            return (STAT_INPUT_VALUE_RANGE_ERROR);
        }
        if (!setAction((inputAction)nv->value)) {
            return STAT_PARAMETER_IS_READ_ONLY;
        }
        return (STAT_OK);
    };

    stat_t getFunction(nvObj_t *nv)
    {
        nv->value = getFunction();
        nv->valuetype = TYPE_INT;
        return (STAT_OK);
    };
    stat_t setFunction(nvObj_t *nv)
    {
        if ((nv->value < INPUT_FUNCTION_NONE) || (nv->value >= INPUT_FUNCTION_MAX)) {
            return (STAT_INPUT_VALUE_RANGE_ERROR);
        }
        if (!setFunction((inputFunc)nv->value)) {
            return STAT_PARAMETER_IS_READ_ONLY;
        }
        return (STAT_OK);
    };
};

template <typename pinType>
struct gpioDigitalInputPin : gpioDigitalInput {
protected: // so we know if anyone tries to reach in
    ioMode mode;					    // 0=active low (NO), 1= active high (NC), 2=disabled
    inputAction action;                 // 0=none, 1=stop, 2=halt, 3=stop_steps, 4=reset
    inputFunc function;                 // function to perform when activated / deactivated

    inputEdgeFlag edge;                 // keeps a transient record of edges for immediate inquiry

    bool homing_mode;                   // set true when input is in homing mode.
    bool probing_mode;                  // set true when input is in probing mode.

    uint8_t ext_pin_number;             // the number used externally for this pin ("in" + ext_pin_number)

    uint16_t lockout_ms;                // number of milliseconds for debounce lockout
    Motate::Timeout lockout_timer;      // time to expire current debounce lockout, or 0 if no lockout

    pinType pin;                        // the actual pin object itself

public:
    // In constructor, simply forward all values to the pinType
    // To get a different behavior, override this object.
    template <typename... T>
    gpioDigitalInputPin(const ioMode _mode, const uint8_t _ext_pin_number, T&&... V) :
        gpioDigitalInput{},
        mode{_mode},
        ext_pin_number{_ext_pin_number},
        pin{((mode == IO_ACTIVE_LOW) ? kPullUp|kDebounce : kDebounce), [&]{this->pin_changed();}, std::forward<T>(V)...}
    {};

    // gpioDigitalInputPin(const ioMode _mode, const uint8_t _ext_pin_number) :
    //     gpioDigitalInput{},
    //     mode{_mode},
    //     ext_pin_number{_ext_pin_number},
    //     pin{((mode == IO_ACTIVE_LOW) ? kPullUp|kDebounce : kDebounce), [&]{this->pin_changed();}}
    // {};

    // functions for use by other parts of the code, and are overridden

    bool getState() override
    {
        if (mode == IO_MODE_DISABLED) {
            return false;
        }
        const bool v = (const bool)pin;
        return (mode == IO_ACTIVE_HIGH) ? v : !v;
    };

    inputFunc getFunction() override
    {
        return function;
    }
    bool setFunction(const inputFunc f) override
    {
        function = f;
        return true;
    };

    inputAction getAction() override
    {
        return action;
    };
    bool setAction(const inputAction a) override
    {
        action = a;
        return true;
    };

    ioMode getMode() override
    {
        return mode;
    };
    bool setMode(const ioMode m) override
    {
        mode = m;
        pin.setOptions((mode == IO_ACTIVE_LOW) ? kPullUp|kDebounce : kDebounce);
        return true;
    };

    void setExternalNumber(const uint8_t e) override
    {
        ext_pin_number = e;
    }

    virtual void setIsHoming(const bool m)
    {
        homing_mode = m;
    }

    virtual void setIsProbing(const bool m)
    {
        probing_mode = m;
    }


    // support function for pin change interrupt handling

    void pin_changed()
    {
        // return if input is disabled
        if (mode == IO_MODE_DISABLED) {
            return;
        }

        // return if the input is in lockout period (take no action)
        if (lockout_timer.isSet() && !lockout_timer.isPast()) {
            return;
        }

        ioState pin_value = (ioState)(bool)pin;
        ioState pin_value_corrected = (ioState)(pin_value ^ ((bool)mode ^ 1));    // correct for NO or NC mode

        // This shouldn't be necessary - the processor shouldn't call without an edge
        // if (state == (ioState)pin_value_corrected) {
        //     return;
        // }

        // lockout the pin for lockout_ms
        lockout_timer.set(lockout_ms);

        // record the changed state
        if (pin_value_corrected == INPUT_ACTIVE) {
            edge = INPUT_EDGE_LEADING;
        } else {
            edge = INPUT_EDGE_TRAILING;
        }

        // TODO - refactor homing_mode and probing_mode out to use a dynamically
        //        configured linked list of functions

        // perform homing operations if in homing mode
        if (homing_mode) {
            if (edge == INPUT_EDGE_LEADING) {   // we only want the leading edge to fire
                en_take_encoder_snapshot();
                cm_start_hold();
            }
            return;
        }

        // perform probing operations if in probing mode
        if (probing_mode) {
            // We want to capture either way.
            // Probing tests the start condition for the correct direction ahead of time.
            // If we see any edge, it's the right one.
            en_take_encoder_snapshot();
            cm_start_hold();
            return;
        }

        // *** NOTE: From this point on all conditionals assume we are NOT in homing or probe mode ***

        // trigger the action on leading edges
        if (edge == INPUT_EDGE_LEADING) {
            if (action == INPUT_ACTION_STOP) {
                cm_start_hold();
            }
            if (action == INPUT_ACTION_FAST_STOP) {
                cm_start_hold();                        // for now is same as STOP
            }
            if (action == INPUT_ACTION_HALT) {
                cm_halt_all();                            // hard stop, including spindle and coolant
            }
            if (action == INPUT_ACTION_ALARM) {
                char msg[10];
                sprintf(msg, "input %d", ext_pin_number);
                cm_alarm(STAT_ALARM, msg);
            }
            if (action == INPUT_ACTION_SHUTDOWN) {
                char msg[10];
                sprintf(msg, "input %d", ext_pin_number);
                cm_shutdown(STAT_SHUTDOWN, msg);
            }
            if (action == INPUT_ACTION_PANIC) {
                char msg[10];
                sprintf(msg, "input %d", ext_pin_number);
                cm_panic(STAT_PANIC, msg);
            }
            if (action == INPUT_ACTION_RESET) {
                hw_hard_reset();
            }

            // these functions also trigger on the leading edge

            if (function == INPUT_FUNCTION_LIMIT) {
                cm.limit_requested = ext_pin_number;

            } else if (function == INPUT_FUNCTION_SHUTDOWN) {
                cm.shutdown_requested = ext_pin_number;

            } else if (function == INPUT_FUNCTION_INTERLOCK) {
                cm.safety_interlock_disengaged = ext_pin_number;
            }
        } // if (edge == INPUT_EDGE_LEADING)

        // trigger interlock release on trailing edge
        if (edge == INPUT_EDGE_TRAILING) {
            if (function == INPUT_FUNCTION_INTERLOCK) {
                cm.safety_interlock_reengaged = ext_pin_number;
            }
        }

        sr_request_status_report(SR_REQUEST_TIMED);   //+++++ Put this one back in.
    };

};

struct gpioDigitalOutput {

    // functions for use by other parts of the code, and are what to override
    virtual ioMode getMode()
    {
        return IO_MODE_DISABLED;
    };
    virtual bool setMode(const ioMode)
    {
        return false;
    };

    virtual float getValue()
    {
        return -1;
    };
    virtual bool setValue(const float)
    {
        return false;
    };


    // functions that take nvObj_t* and return stat_t, NOT overridden

    virtual stat_t getMode(nvObj_t *nv)
    {
        nv->value = getMode();
        nv->valuetype = TYPE_INT;
        return (STAT_OK);
    };
    virtual stat_t setMode(nvObj_t *nv)
    {
        if ((nv->value < IO_ACTIVE_LOW) || (nv->value >= IO_MODE_MAX)) {
            return (STAT_INPUT_VALUE_RANGE_ERROR);
        }
        if (!setMode((ioMode)nv->value)) {
            return STAT_INPUT_VALUE_RANGE_ERROR;
        }
        return (STAT_OK);
    };

    virtual stat_t getValue(nvObj_t *nv)
    {
        auto mode = getMode();
        if (mode == IO_MODE_DISABLED) {
            nv->value = 0;
            nv->valuetype = TYPE_NULL;   // reports back as NULL
        } else {
            nv->valuetype = TYPE_FLOAT;
            nv->precision = 2;
            nv->value = getValue(); // read it as a float

            bool invert = (mode == 0);
            if (invert) {
                nv->value = 1.0 - nv->value;
            }
        }
        return (STAT_OK);
    };
    virtual stat_t setValue(nvObj_t *nv)
    {
        auto mode = getMode();
        if (mode == IO_MODE_DISABLED) {
            nv->valuetype = TYPE_NULL;   // reports back as NULL
        } else {
            float value = nv->value; // read it as a float

            bool invert = (mode == 0);
            if (invert) {
                value = 1.0 - value;
            }

            if (!setValue(value)) {
                return STAT_INPUT_VALUE_RANGE_ERROR;
            }
        }
        return (STAT_OK);
    };

};

template <typename pinType>
struct gpioDigitalOutputPin : gpioDigitalOutput {
    ioMode mode;					    // 0=active low (NO), 1= active high (NC), 2=disabled
    pinType pin;

    // In constructor, simply forward all values to the pinType
    template <typename... T>
    gpioDigitalOutputPin(const ioMode _mode, T&&... V) :
        gpioDigitalOutput{},
        mode{ _mode },
        pin{((mode == IO_ACTIVE_LOW) ? kStartHigh|kPWMPinInverted : kStartLow), std::forward<T>(V)...}
    {};

    // functions for use by other parts of the code, and are overridden

    ioMode getMode() override
    {
        return mode;
    };
    bool setMode(const ioMode m) override
    {
        mode = m;
        pin.setOptions((mode == IO_ACTIVE_LOW) ? kPullUp|kDebounce : kDebounce); // prepare the pin
        pin = ((mode == IO_ACTIVE_LOW) ? true : false);                    // set the output to inactive
        return true;
    };


    float getValue() override
    {
        return (const float)pin;
    };
    bool setValue(const float v) override
    {
        if (pin.isNull()) {
            return false;
        }
        pin = v;
        return true;
    };

};

struct gpioAnalogInput {        // one struct per analog input
    ioMode mode;
};

struct gpioAnalogOutput {       // one struct per analog output
    ioMode mode;
};

/*
 * GPIO function prototypes
 */

void gpio_init(void);
void gpio_reset(void);
void inputs_reset(void);
void outputs_reset(void);

bool gpio_read_input(const uint8_t input_num);
void gpio_set_homing_mode(const uint8_t input_num, const bool is_homing);
void gpio_set_probing_mode(const uint8_t input_num, const bool is_probing);
int8_t gpio_get_probing_input(void);

stat_t din_get_mo(nvObj_t *nv);
stat_t din_set_mo(nvObj_t *nv);
stat_t din_get_ac(nvObj_t *nv);
stat_t din_set_ac(nvObj_t *nv);
stat_t din_get_fn(nvObj_t *nv);
stat_t din_set_fn(nvObj_t *nv);

stat_t din_get_input(nvObj_t *nv);

stat_t dout_get_mo(nvObj_t *nv);			// output sense
stat_t dout_set_mo(nvObj_t *nv);			// output sense
stat_t dout_get_output(nvObj_t *nv);
stat_t dout_set_output(nvObj_t *nv);

#ifdef __TEXT_MODE
    void din_print_mo(nvObj_t *nv);
    void din_print_ac(nvObj_t *nv);
    void din_print_fn(nvObj_t *nv);
    void din_print_in(nvObj_t *nv);
    void dout_print_mo(nvObj_t *nv);
    void dout_print_out(nvObj_t *nv);
#else
    #define din_print_mo tx_print_stub
    #define din_print_ac tx_print_stub
    #define din_print_fn tx_print_stub
    #define din_print_in tx_print_stub
    #define dout_print_mo tx_print_stub
    #define dout_print_out tx_print_stub
#endif // __TEXT_MODE

#include "board_gpio.h"

#endif // End of include guard: GPIO_H_ONCE
