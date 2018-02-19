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
using Motate::kStartHigh;
using Motate::kStartLow;
using Motate::kPWMPinInverted;

/*
 * GPIO defines
 */

// moved to board_gpio.h, which is copied for each board in subfolders of board/

//--- do not change from here down ---//

typedef enum {
    IO_UNAVAILABLE = -1,   // input/output is missing/used/unavailable
    IO_DISABLED    =  0,   // input/output is disabled
    IO_ENABLED     =  1    // input/output enabled
} ioEnabled;

typedef enum {
    IO_ACTIVE_HIGH = 0,                 // input/output is active high (aka normally closed)
    IO_ACTIVE_LOW = 1,                  // input/output is active low (aka normally open)
} ioPolarity;
#define NORMALLY_OPEN   IO_ACTIVE_LOW   // equivalent
#define NORMALLY_CLOSED IO_ACTIVE_HIGH  // equivalent

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
 * gpioDigitalInput - digital input base class
 */
struct gpioDigitalInput {
    // this is the generic implementation for a "any"" digital input pin
    // see gpioDigitalInputPin for a real pin

    // functions for use by other parts of the code

    virtual bool getState();

    virtual inputFunc getFunction();
    virtual bool setFunction(const inputFunc);

    virtual inputAction getAction();
    virtual bool setAction(const inputAction);

    virtual ioEnabled getEnabled();
    virtual bool setEnabled(const ioEnabled);

    virtual ioPolarity getPolarity();
    virtual bool setPolarity(const ioPolarity);

    virtual void setExternalNumber(const uint8_t);
    virtual void setIsHoming(const bool);
    virtual void setIsProbing(const bool);


    // functions that take nvObj_t* and return stat_t, NOT overridden

    stat_t getState(nvObj_t *nv)
    {
        if (getEnabled() <= IO_DISABLED) {
            nv->valuetype = TYPE_NULL;
            return (STAT_OK);
        }
        nv->value = getState();
        nv->valuetype = TYPE_BOOL;
        return (STAT_OK);
    };
    // no setState

    stat_t getEnabled(nvObj_t *nv)
    {
        nv->value = getEnabled();
        nv->valuetype = TYPE_INT;
        return (STAT_OK);
    };
    stat_t setEnabled(nvObj_t *nv)
    {
        if ((nv->value < IO_DISABLED) || (nv->value > IO_ENABLED)) {
            return (STAT_INPUT_VALUE_RANGE_ERROR);
        }
        if (!setEnabled((ioEnabled)nv->value)) {
            return STAT_PARAMETER_IS_READ_ONLY;
        }
        return (STAT_OK);
    };

    stat_t getPolarity(nvObj_t *nv)
    {
        nv->value = getPolarity();
        nv->valuetype = TYPE_INT;
        return (STAT_OK);
    };
    stat_t setPolarity(nvObj_t *nv)
    {
        if ((nv->value < IO_ACTIVE_HIGH) || (nv->value > IO_ACTIVE_LOW)) {
            return (STAT_INPUT_VALUE_RANGE_ERROR);
        }
        if (!setPolarity((ioPolarity)nv->value)) {
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
        if ((nv->value < INPUT_ACTION_NONE) || (nv->value > INPUT_ACTION_MAX)) {
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
        if ((nv->value < INPUT_FUNCTION_NONE) || (nv->value > INPUT_FUNCTION_MAX)) {
            return (STAT_INPUT_VALUE_RANGE_ERROR);
        }
        if (!setFunction((inputFunc)nv->value)) {
            return STAT_PARAMETER_IS_READ_ONLY;
        }
        return (STAT_OK);
    };
};

/*
 * gpioDigitalInputPin - concrete child of gpioDigitalInput
 */
template <typename Pin_t>
struct gpioDigitalInputPin : gpioDigitalInput {
    ioEnabled enabled;					// -1=unavailable, 0=disabled, 1=enabled
    ioPolarity polarity;                // 0=normal/active high, 1=inverted/active low
    inputAction action;                 // 0=none, 1=stop, 2=halt, 3=stop_steps, 4=reset
    inputFunc input_function;                 // function to perform when activated / deactivated

    inputEdgeFlag edge;                 // keeps a transient record of edges for immediate inquiry

    bool homing_mode;                   // set true when input is in homing mode.
    bool probing_mode;                  // set true when input is in probing mode.

    uint8_t ext_pin_number;             // the number used externally for this pin ("in" + ext_pin_number)

    uint16_t lockout_ms;                // number of milliseconds for debounce lockout
    Motate::Timeout lockout_timer;      // time to expire current debounce lockout, or 0 if no lockout

    Pin_t pin;                        // the actual pin object itself

    // In constructor, simply forward all values to the Pin_t
    // To get a different behavior, override this object.
    template <typename... T>
    gpioDigitalInputPin(const ioEnabled _enabled, const ioPolarity _polarity, const uint8_t _ext_pin_number, T&&... V) :
        gpioDigitalInput{},
        enabled{_enabled},
        polarity{_polarity},
        ext_pin_number{_ext_pin_number},
        pin{((polarity == IO_ACTIVE_LOW) ? kPullUp|kDebounce : kDebounce), [&]{this->pin_changed();}, std::forward<T>(V)...}
    {
        if (pin.isNull()) {
            enabled = IO_UNAVAILABLE;
        }
    };

    // functions for use by other parts of the code, and are overridden

    bool getState() override
    {
        if (enabled <= IO_DISABLED) {
            return false;
        }
        const bool v = (const bool)pin;
        return (polarity == IO_ACTIVE_HIGH) ? v : !v;
    };

    inputFunc getFunction() override
    {
        return input_function;
    }
    bool setFunction(const inputFunc f) override
    {
        input_function = f;
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

    ioEnabled getEnabled() override
    {
        return enabled;
    };
    bool setEnabled(const ioEnabled m) override
    {
        if (enabled == IO_UNAVAILABLE) {
            return false;
        }
        enabled = m;
        return true;
    };

    ioPolarity getPolarity() override
    {
        return polarity;
    };
    bool setPolarity(const ioPolarity new_polarity) override
    {
        polarity = new_polarity;
        pin.setOptions((polarity == IO_ACTIVE_LOW) ? kPullUp|kDebounce : kDebounce);
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
        if (enabled == IO_DISABLED) {
            return;
        }

        // return if the input is in lockout period (take no action)
        if (lockout_timer.isSet() && !lockout_timer.isPast()) {
            return;
        }

        ioState pin_value = (ioState)(bool)pin;
        ioState pin_value_corrected = (ioState)(pin_value ^ ((bool)polarity));    // correct for NO or NC mode

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

            if (input_function == INPUT_FUNCTION_LIMIT) {
                cm.limit_requested = ext_pin_number;

            } else if (input_function == INPUT_FUNCTION_SHUTDOWN) {
                cm.shutdown_requested = ext_pin_number;

            } else if (input_function == INPUT_FUNCTION_INTERLOCK) {
                cm.safety_interlock_disengaged = ext_pin_number;
            }
        } // if (edge == INPUT_EDGE_LEADING)

        // trigger interlock release on trailing edge
        if (edge == INPUT_EDGE_TRAILING) {
            if (input_function == INPUT_FUNCTION_INTERLOCK) {
                cm.safety_interlock_reengaged = ext_pin_number;
            }
        }

        sr_request_status_report(SR_REQUEST_TIMED);   //+++++ Put this one back in.
    };
};

/*
 * gpioDigitalOutput - digital/PWM output base class
 */
struct gpioDigitalOutput {
    // this is the generic implementation for a "any"" output pin (PWM or Digital)
    // see gpioDigitalOutputPin for a real pin

    // functions for use by other parts of the code, and are what to override
    virtual ioEnabled getEnabled();
    virtual bool setEnabled(const ioEnabled);

    virtual ioPolarity getPolarity();
    virtual bool setPolarity(const ioPolarity);

    virtual float getValue();
    virtual bool setValue(const float);

    virtual float getFrequency();
    virtual bool setFrequency(const float);


    // functions that take nvObj_t* and return stat_t, NOT overridden

    stat_t getEnabled(nvObj_t *nv)
    {
        nv->value = getEnabled();
        nv->valuetype = TYPE_INT;
        return (STAT_OK);
    };
    stat_t setEnabled(nvObj_t *nv)
    {
        if ((nv->value < IO_DISABLED) || (nv->value > IO_ENABLED)) {
            return (STAT_INPUT_VALUE_RANGE_ERROR);
        }
        if (!setEnabled((ioEnabled)nv->value)) {
            return STAT_PARAMETER_IS_READ_ONLY;
        }
        return (STAT_OK);
    };
    
    stat_t getPolarity(nvObj_t *nv)
    {
        nv->value = getPolarity();
        nv->valuetype = TYPE_INT;
        return (STAT_OK);
    };
    stat_t setPolarity(nvObj_t *nv)
    {
        if ((nv->value < IO_ACTIVE_HIGH) || (nv->value > IO_ACTIVE_LOW)) {
            return (STAT_INPUT_VALUE_RANGE_ERROR);
        }
        if (!setPolarity((ioPolarity)nv->value)) {
            return STAT_PARAMETER_IS_READ_ONLY;
        }
        return (STAT_OK);
    };

    virtual stat_t getValue(nvObj_t *nv)
    {
        auto enabled = getEnabled();
        if (enabled <= IO_DISABLED) {
            nv->value = 0;
            nv->valuetype = TYPE_NULL;   // reports back as NULL
        } else {
            nv->valuetype = TYPE_FLOAT;
            nv->precision = 2;
            nv->value = getValue(); // read it as a float

            bool invert = (getPolarity() == IO_ACTIVE_LOW);
            if (invert) {
                nv->value = 1.0 - nv->value;
            }
        }
        return (STAT_OK);
    };
    virtual stat_t setValue(nvObj_t *nv)
    {
        auto enabled = getEnabled();
        if (enabled <= IO_DISABLED) {
            nv->valuetype = TYPE_NULL;   // reports back as NULL
        } else {
            float value = nv->value; // read it as a float

            bool invert = (getPolarity() == IO_ACTIVE_LOW);
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

/*
 * gpioDigitalOutputPin - concrete child of gpioDigitalOutput
 */
template <typename Pin_t>
struct gpioDigitalOutputPin : gpioDigitalOutput {
    ioEnabled enabled;                  // -1=unavailable, 0=disabled, 1=enabled
    ioPolarity polarity;                // 0=normal/active high, 1=inverted/active low
    Pin_t pin;

    // In constructor, simply forward all values to the Pin_t
    template <typename... T>
    gpioDigitalOutputPin(const ioEnabled _enabled, const ioPolarity _polarity, T&&... V) :
        gpioDigitalOutput{},
        enabled{ _enabled },
        polarity{ _polarity },
        pin{((polarity == IO_ACTIVE_LOW) ? kStartHigh|kPWMPinInverted : kStartLow), std::forward<T>(V)...}
    {
        if (pin.isNull()) {
            enabled = IO_UNAVAILABLE;
        }
    };

    // functions for use by other parts of the code, and are overridden

    ioEnabled getEnabled() override
    {
        return enabled;
    };
    bool setEnabled(const ioEnabled m) override
    {
        if (enabled == IO_UNAVAILABLE) {
            return false;
        }
        enabled = m;
        return true;
    };

    ioPolarity getPolarity() override
    {
        return polarity;
    };
    bool setPolarity(const ioPolarity new_polarity) override
    {
        polarity = new_polarity;
        pin.setOptions((polarity == IO_ACTIVE_LOW) ? kPullUp|kDebounce : kDebounce);
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

    float _last_set_frequency = 0;
    // it must be set through this interface at least once before it can be read back
    float getFrequency() override
    {
        return _last_set_frequency;
    };
    bool setFrequency(const float freq) override
    {
        pin.setFrequency(freq);
        _last_set_frequency = freq;
        return true;
    };

};

/*
 * gpioAnalogInput - analog (ADC) input base class
 */
struct gpioAnalogInput {
    // type of analog input source - read only - defined by the board
    enum AnalogInputType_t {
        AIN_TYPE_DISABLED = 0, // the whole input is disabled
        AIN_TYPE_INTERNAL = 1, // single-ended or differential
        AIN_TYPE_EXTERNAL = 2, // for externally (SPI) connected inputs
    };

    // type of circuit connected - for use in determining the resistance
    enum AnalogCircuit_t {
        AIN_CIRCUIT_DISABLED     = 0, // there is no circuit, resistance will read -1
                                      //  no additional configuration
        AIN_CIRCUIT_PULLUP       = 1, // resistance being measured is pulling up to VCC
                                      //  the pull-up resistance is measured (rt)
                                      //  p1 is the set pull-down resistance (r1)
        AIN_CIRCUIT_EXTERNAL     = 2, // for externally (SPI) connected inputs
                                      //  no additional configuration
        AIN_CIRCUIT_INV_OPAMP    = 3, // inverted op-amp connected
                                      //  the pull-up resistance is measured (rt)
                                      //  p1 is the set pull-down resistance of the bias(+) (r1)
                                      //  p2 is the set pull-up resistance of the gain(-) (r2)
                                      //  p3 is the set pull-down to output of the gain(-) (r3)
        AIN_CIRCUIT_CC_INV_OPAMP = 4, // for externally (SPI) connected inputs
                                      //  the pull-up resistance to the current source is measured (rt)
                                      //  p4 is the set pull-up resistance of the bias(+) (r4)
                                      //  p1 is the set pull-down resistance of the bias(+) (r1)
                                      //  p2 is the set pull-up resistance of the gain(-) (r2)
                                      //  p3 is the set pull-down to output of the gain(-) (r3)
                                      //  p5 is the set constant current in millivolts (c1)
    };
    static const auto AIN_CIRCUIT_MAX = AIN_CIRCUIT_CC_INV_OPAMP;

    // this is the generic implementation for a "any"" analog input pin
    // see gpioAnalogInputPin for a real pin

    // functions for use by other parts of the code

    virtual float getValue();
    virtual float getResistance();

    virtual AnalogInputType_t getType();
    virtual bool setType(const AnalogInputType_t);

    virtual AnalogCircuit_t getCircuit();
    virtual bool setCircuit(const AnalogCircuit_t);

    virtual float getParameter(const uint8_t p);
    virtual bool setParameter(const uint8_t p, const float v);

    virtual void startSampling();

    // functions that take nvObj_t* and return stat_t, NOT overridden

    stat_t getValue(nvObj_t *nv)
    {
        if (getType() == AIN_TYPE_DISABLED) {
            nv->valuetype = TYPE_NULL;
            return (STAT_OK);
        }
        nv->value = getValue();
        nv->valuetype = TYPE_FLOAT;
        return (STAT_OK);
    };
    // no setValue

    stat_t getResistance(nvObj_t *nv)
    {
        if (getType() == AIN_TYPE_DISABLED || getCircuit() == AIN_CIRCUIT_DISABLED) {
            nv->valuetype = TYPE_NULL;
            return (STAT_OK);
        }
        nv->value = getResistance();
        nv->valuetype = TYPE_FLOAT;
        return (STAT_OK);
    };
    // no setResistance

    stat_t getType(nvObj_t *nv)
    {
        nv->value = getType();
        nv->valuetype = TYPE_INT;
        return (STAT_OK);
    };
    stat_t setType(nvObj_t *nv)
    {
        if ((nv->value < AIN_TYPE_DISABLED) || (nv->value > AIN_TYPE_EXTERNAL)) {
            return (STAT_INPUT_VALUE_RANGE_ERROR);
        }
        if (!setType((AnalogInputType_t)nv->value)) {
            return STAT_PARAMETER_IS_READ_ONLY;
        }
        return (STAT_OK);
    };

    stat_t getCircuit(nvObj_t *nv)
    {
        nv->value = getCircuit();
        nv->valuetype = TYPE_INT;
        return (STAT_OK);
    };
    stat_t setCircuit(nvObj_t *nv)
    {
        if ((nv->value < AIN_CIRCUIT_DISABLED) || (nv->value > AIN_CIRCUIT_MAX)) {
            return (STAT_INPUT_VALUE_RANGE_ERROR);
        }
        if (!setCircuit((AnalogCircuit_t)nv->value)) {
            return STAT_PARAMETER_IS_READ_ONLY;
        }
        return (STAT_OK);
    };

    stat_t getParameter(nvObj_t *nv, const uint8_t p)
    {
        nv->value = getParameter(p);
        nv->valuetype = TYPE_FLOAT;
        return (STAT_OK);
    };
    stat_t setParameter(nvObj_t *nv, const uint8_t p)
    {
        if (!setParameter(p, nv->value)) {
            return STAT_PARAMETER_IS_READ_ONLY;
        }
        return (STAT_OK);
    };
};

// statistical sampling utility class
template<uint16_t sample_count>
struct ValueHistory {

    float variance_max = 2.0;
    ValueHistory() {};
    ValueHistory(float v_max) : variance_max{v_max} {};

    struct sample_t {
        float value;
        float value_sq;
        void set(float v) { value = v; value_sq = v*v; }
    };
    sample_t samples[sample_count];
    uint16_t next_sample = 0;
    void _bump_index(uint16_t &v) {
        ++v;
        if (v == sample_count) {
            v = 0;
        }
    };
    uint16_t sampled = 0;

    float rolling_sum = 0;
    float rolling_sum_sq = 0;
    float rolling_mean = 0;
    void add_sample(float t) {
        rolling_sum -= samples[next_sample].value;
        rolling_sum_sq -= samples[next_sample].value_sq;

        samples[next_sample].set(t);

        rolling_sum += samples[next_sample].value;
        rolling_sum_sq += samples[next_sample].value_sq;

        _bump_index(next_sample);
        if (sampled < sample_count) { ++sampled; }

        rolling_mean = rolling_sum/(float)sampled;
    };

    float get_std_dev() {
        // Important note: this is a POPULATION standard deviation, not a population standard deviation
        float variance = (rolling_sum_sq/(float)sampled) - (rolling_mean*rolling_mean);
        return sqrt(std::abs(variance));
    };

    float value() {
        // we'll shoot through the samples and ignore the outliers
        uint16_t samples_kept = 0;
        float temp = 0;
        float std_dev = get_std_dev();

        for (uint16_t i=0; i<sampled; i++) {
            if (std::abs(samples[i].value - rolling_mean) < (variance_max * std_dev)) {
                temp += samples[i].value;
                ++samples_kept;
            }
        }

        // fallback position
        if (samples_kept == 0) {
            return rolling_mean;
        }

        return (temp / (float)samples_kept);
    };
};

template <typename ADCPin_t>
struct gpioAnalogInputPin : gpioAnalogInput {
protected: // so we know if anyone tries to reach in
    AnalogInputType_t type;
    AnalogCircuit_t circuit;
    float parameters[6];

    uint8_t ext_pin_number;             // the number used externally for this pin ("in" + ext_pin_number)

    const float variance_max = 1.1;
    ValueHistory<20> history {variance_max};

    float last_raw_value;

    ADCPin_t pin;                        // the actual pin object itself

public:
    // In constructor, simply forward all values to the pin
    // To get a different behavior, override this object.
    template <typename... T>
    gpioAnalogInputPin(const AnalogInputType_t _type, const uint8_t _ext_pin_number, T&&... additional_values) :
    gpioAnalogInput{},
    type{_type},
    ext_pin_number{_ext_pin_number},
    pin{Motate::kNormal, [&]{this->adc_has_new_value();}, std::forward<T>(additional_values)...}
    {
        pin.setInterrupts(Motate::kPinInterruptOnChange|Motate::kInterruptPriorityLow);
        pin.setVoltageRange(3.29, 0.0, 3.29, 1000000.0);
    };

    // functions for use by other parts of the code, and are overridden

    float getValue() override
    {
        if (type == AIN_TYPE_DISABLED) {
            return 0;
        }
        return history.value();
    };
    float getResistance() override
    {
        // NOTE: AIN_CIRCUIT_EXTERNAL is NOT handled here!
        //       That needs to be handled in a separate override!
        if (type == AIN_TYPE_DISABLED || circuit == AIN_CIRCUIT_DISABLED) {
            return -1;
        }
        const float v = history.value();
        const float s = pin.getTopVoltage();
        switch (circuit) {

            case AIN_CIRCUIT_PULLUP:
            {
                float r1 = parameters[0]; // pull-up

                if (ADCPin_t::is_differential) {
                    return (v * 2.0 * r1)/(s - v);
                } else {
                    return (v * r1)/(s - v);
                }
                break;
            }

            case AIN_CIRCUIT_INV_OPAMP:
            {
                float r1 = parameters[0]; // pull-down from bias(+) side of op-amp
                float r2 = parameters[1]; // pull-up from gain(-) side of op-amp
                float r3 = parameters[2]; // pull-to-output from gain(-) side of op-amp

                return (r1 * r2 * (s - v))/(r2 * v + r3 * s);
                break;
            }

            case AIN_CIRCUIT_CC_INV_OPAMP:
            {
                //  the pull-up resistance to the current source is measured (rt)
                float r4 = parameters[3]; // pull-up resistance of the bias(+) side of op-amp
                float r1 = parameters[0]; // pull-down from bias(+) side of op-amp
                float r2 = parameters[1]; // pull-up from gain(-) side of op-amp
                float r3 = parameters[2]; // pull-to-output from gain(-) side of op-amp
                float c = parameters[4]; // constant current in volts (c1)

                // r_0 = (r_1 (r_2 (s - v) + r_3 s) - v r_2 r_4)/(c r_3 (r_1 + r_4))
                return (r1 * (r2 * (s - v) + r3 * s) - v * r2 * r4)/(c * r3 * (r1 + r4));
                break;
            }

            // AIN_CIRCUIT_EXTERNAL is specifically missing!

            default:
                return -1;
        }

    };

    AnalogInputType_t getType() override
    {
        return type;
    };
    bool setType(const AnalogInputType_t t) override
    {
        // NOTE: AIN_TYPE_EXTERNAL is NOT handled here!
        //       That needs to be handled in a separate override!
        if (t == AIN_TYPE_EXTERNAL) {
            return false;
        }
        type = t;
        return true;
    };

    AnalogCircuit_t getCircuit() override
    {
        return circuit;
    };
    bool setCircuit(const AnalogCircuit_t c) override
    {
        // prevent setting circuit to AIN_CIRCUIT_EXTERNAL
        if (c == AIN_CIRCUIT_EXTERNAL) {
            return false;
        }
        circuit = c;
        return true;
    };

    float getParameter(const uint8_t p) override
    {
        if (p < 0 || p >= 6) {
            return 0;
        }
        return parameters[p];
    };
    bool setParameter(const uint8_t p, const float v) override
    {
        if (p < 0 || p >= 6) {
            return false;
        }
        parameters[p] = v;
        return true;
    };

    void startSampling() override {
        pin.startSampling();
    };

    // support function for pin value update interrupt handling

    void adc_has_new_value() {
        last_raw_value = pin.getRaw();
        history.add_sample(pin.getVoltage());
    };
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

stat_t din_get_en(nvObj_t *nv);     // enabled
stat_t din_set_en(nvObj_t *nv);
stat_t din_get_po(nvObj_t *nv);     // input sense
stat_t din_set_po(nvObj_t *nv);
stat_t din_get_ac(nvObj_t *nv);     // input action
stat_t din_set_ac(nvObj_t *nv);
stat_t din_get_fn(nvObj_t *nv);     // input function
stat_t din_set_fn(nvObj_t *nv);
stat_t din_get_input(nvObj_t *nv);

stat_t dout_get_en(nvObj_t *nv);     // enabled
stat_t dout_set_en(nvObj_t *nv);
stat_t dout_get_po(nvObj_t *nv);     // output sense
stat_t dout_set_po(nvObj_t *nv);
stat_t dout_get_output(nvObj_t *nv); // actual output value (float)
stat_t dout_set_output(nvObj_t *nv);

stat_t ain_get_value(nvObj_t *nv);      // get the voltage level
// no ain_set_value
stat_t ain_get_resistance(nvObj_t *nv); // get the resistance in ohms
// no ain_set_resistance
stat_t ain_get_type(nvObj_t *nv);       // get the ADC type (or disabled)
stat_t ain_set_type(nvObj_t *nv);       // set the type (used to disable/enable)
stat_t ain_get_circuit(nvObj_t *nv);    // get the circuit type
stat_t ain_set_circuit(nvObj_t *nv);    // set the circuit type
stat_t ain_get_parameter(nvObj_t *nv, const uint8_t p); // get the value of parameter p
stat_t ain_set_parameter(nvObj_t *nv, const uint8_t p); // set the value of parameter p
stat_t ain_get_p1(nvObj_t *nv);
stat_t ain_set_p1(nvObj_t *nv);
stat_t ain_get_p2(nvObj_t *nv);
stat_t ain_set_p2(nvObj_t *nv);
stat_t ain_get_p3(nvObj_t *nv);
stat_t ain_set_p3(nvObj_t *nv);
stat_t ain_get_p4(nvObj_t *nv);
stat_t ain_set_p4(nvObj_t *nv);
stat_t ain_get_p5(nvObj_t *nv);
stat_t ain_set_p5(nvObj_t *nv);

#ifdef __TEXT_MODE
    void din_print_en(nvObj_t *nv);
    void din_print_po(nvObj_t *nv);
    void din_print_ac(nvObj_t *nv);
    void din_print_fn(nvObj_t *nv);
    void din_print_in(nvObj_t *nv);

    void dout_print_en(nvObj_t *nv);
    void dout_print_po(nvObj_t *nv);
    void dout_print_out(nvObj_t *nv);

    void ain_print_value(nvObj_t *nv);
    void ain_print_resistance(nvObj_t *nv);
    void ain_print_type(nvObj_t *nv);
    void ain_print_circuit(nvObj_t *nv);
    void ain_print_p(nvObj_t *nv);
#else
    #define din_print_en tx_print_stub
    #define din_print_po tx_print_stub
    #define din_print_ac tx_print_stub
    #define din_print_fn tx_print_stub
    #define din_print_in tx_print_stub

    #define dout_print_en tx_print_stub
    #define dout_print_po tx_print_stub
    #define dout_print_out tx_print_stub

    #define ain_print_value tx_print_stub
    #define ain_print_resistance tx_print_stub
    #define ain_print_type tx_print_stub
    #define ain_print_circuit tx_print_stub
    #define ain_print_p tx_print_stub
#endif // __TEXT_MODE

#include "board_gpio.h"

#ifndef AI1_TYPE
#define AI1_TYPE                    AIN_TYPE_DISABLED
#endif
#ifndef AI1_CIRCUIT
#define AI1_CIRCUIT                 AIN_CIRCUIT_DISABLED
#endif
#ifndef AI1_P1
#define AI1_P1                      0.0
#endif
#ifndef AI1_P2
#define AI1_P2                      0.0
#endif
#ifndef AI1_P3
#define AI1_P3                      0.0
#endif
#ifndef AI1_P4
#define AI1_P4                      0.0
#endif
#ifndef AI1_P5
#define AI1_P5                      0.0
#endif


#ifndef AI2_TYPE
#define AI2_TYPE                    AIN_TYPE_DISABLED
#endif
#ifndef AI2_CIRCUIT
#define AI2_CIRCUIT                 AIN_CIRCUIT_DISABLED
#endif
#ifndef AI2_P1
#define AI2_P1                      0.0
#endif
#ifndef AI2_P2
#define AI2_P2                      0.0
#endif
#ifndef AI2_P3
#define AI2_P3                      0.0
#endif
#ifndef AI2_P4
#define AI2_P4                      0.0
#endif
#ifndef AI2_P5
#define AI2_P5                      0.0
#endif

#ifndef AI3_TYPE
#define AI3_TYPE                    AIN_TYPE_DISABLED
#endif
#ifndef AI3_CIRCUIT
#define AI3_CIRCUIT                 AIN_CIRCUIT_DISABLED
#endif
#ifndef AI3_P1
#define AI3_P1                      0.0
#endif
#ifndef AI3_P2
#define AI3_P2                      0.0
#endif
#ifndef AI3_P3
#define AI3_P3                      0.0
#endif
#ifndef AI3_P4
#define AI3_P4                      0.0
#endif
#ifndef AI3_P5
#define AI3_P5                      0.0
#endif

#ifndef AI4_TYPE
#define AI4_TYPE                    AIN_TYPE_DISABLED
#endif
#ifndef AI4_CIRCUIT
#define AI4_CIRCUIT                 AIN_CIRCUIT_DISABLED
#endif
#ifndef AI4_P1
#define AI4_P1                      0.0
#endif
#ifndef AI4_P2
#define AI4_P2                      0.0
#endif
#ifndef AI4_P3
#define AI4_P3                      0.0
#endif
#ifndef AI4_P4
#define AI4_P4                      0.0
#endif
#ifndef AI4_P5
#define AI4_P5                      0.0
#endif

#endif // End of include guard: GPIO_H_ONCE
