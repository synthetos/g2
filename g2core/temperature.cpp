/*
 * temperature.cpp - temperature control module - drives heaters or coolers
 * This file is part of the g2core project
 *
 * Copyright (c) 2016 Robert Giseburt
 * Copyright (c) 2016 Alden S. Hart, Jr.
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

#include "g2core.h"             // #1 dependency order
#include "config.h"             // #2
#include "canonical_machine.h"  // #3
#include "text_parser.h"        // #4

#include "temperature.h"
#include "planner.h"
#include "hardware.h"
#include "pwm.h"
#include "report.h"
#include "util.h"
#include "settings.h"


/**** Local safety/limit settings ****/

#ifndef HAS_TEMPERATURE_SENSOR_1
#define HAS_TEMPERATURE_SENSOR_1  false
#endif
#ifndef HAS_TEMPERATURE_SENSOR_2
#define HAS_TEMPERATURE_SENSOR_2  false
#endif
#ifndef HAS_TEMPERATURE_SENSOR_3
#define HAS_TEMPERATURE_SENSOR_3  false
#endif
#ifndef EXTRUDER_1_OUTPUT_PIN
#define EXTRUDER_1_OUTPUT_PIN kOutput1_PinNumber
#endif
#ifndef EXTRUDER_1_FAN_PIN
#define EXTRUDER_1_FAN_PIN    kOutput3_PinNumber
#endif
#ifndef EXTRUDER_2_OUTPUT_PIN
#define EXTRUDER_2_OUTPUT_PIN kOutput2_PinNumber
#endif
#ifndef BED_OUTPUT_PIN
#define BED_OUTPUT_PIN kOutput11_PinNumber
#endif
#ifndef BED_OUTPUT_INIT
#define BED_OUTPUT_INIT {kNormal, fet_pin3_freq}
// OR
//#define BED_OUTPUT_INIT {kPWMPinInverted, fet_pin3_freq};
#endif

// These could be moved to settings
// If the temperature stays at set_point +- TEMP_SETPOINT_HYSTERESIS for more
// than TEMP_SETPOINT_HOLD_TIME ms, it's "at temp".
#ifndef TEMP_SETPOINT_HYSTERESIS
#define TEMP_SETPOINT_HYSTERESIS (float)1.0 // +- 1 degrees C
#endif
#ifndef TEMP_SETPOINT_HOLD_TIME
#define TEMP_SETPOINT_HOLD_TIME 1000 // a full second
#endif

// Below TEMP_OFF_BELOW is considered "off".
// With a set temp of < TEMP_OFF_BELOW, and a measured temp of < TEMP_OFF_BELOW,
// we are "at temp".
#ifndef TEMP_OFF_BELOW
#define TEMP_OFF_BELOW (float)45.0 // "safe to touch and hold for metal" with 5º margin
#endif

// If the read temp is more than TEMP_FULL_ON_DIFFERENCE less than set temp,
// just turn the heater full-on.
#ifndef TEMP_FULL_ON_DIFFERENCE
#define TEMP_FULL_ON_DIFFERENCE (float)50.0
#endif

// If the temp is more than TEMP_MAX_SETPOINT, just turn the heater off,
// regardless of set temp.
#ifndef TEMP_MAX_SETPOINT
#define TEMP_MAX_SETPOINT (float)300.0
#endif

// If the resistance reads higher than TEMP_MIN_DISCONNECTED_RESISTANCE, the
// thermistor is considered disconnected.
#ifndef TEMP_MIN_DISCONNECTED_RESISTANCE
#define TEMP_MIN_DISCONNECTED_RESISTANCE (float)1000000.0
#endif

// If the temperature doesn't rise more than TEMP_MIN_RISE_DEGREES_OVER_TIME in
// TEMP_MIN_RISE_TIME milliseconds, then it's a failure (the sensor is likely
// physically dislocated.)
#ifndef TEMP_MIN_RISE_DEGREES_OVER_TIME
#define TEMP_MIN_RISE_DEGREES_OVER_TIME (float)10.0
#endif
#ifndef TEMP_MIN_BED_RISE_DEGREES_OVER_TIME
#define TEMP_MIN_BED_RISE_DEGREES_OVER_TIME (float)3.0
#endif
#ifndef TEMP_MIN_RISE_TIME
#define TEMP_MIN_RISE_TIME (float)(60.0 * 1000.0) // one minute
#endif
#ifndef TEMP_MIN_RISE_DEGREES_FROM_TARGET
#define TEMP_MIN_RISE_DEGREES_FROM_TARGET (float)10.0
#endif


/**** Allocate structures ****/

// This makes the Motate:: prefix unnecessary.
using namespace Motate;

/****** Create file-global objects ******/

// The should be set in hardware.h for each board.
// Luckily, we only use boards that are 3.3V logic at the moment.
const float kSystemVoltage = 3.3;

// This may be used as a base class in the future, but for now it's just a dummy sensor
struct TemperatureSensor {
    TemperatureSensor() {}

    float temperature_exact() {
        return -1; // invalid temperature
    };

    float get_resistance() {
        return -1; // invalid temperature from a thermistor
    };

    uint16_t get_raw_value() {
        return 0;
    };

    float get_voltage() {
        return -1;
    };

    void start_sampling() {
    };
};

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


struct ADCCircuit
{
    virtual float get_resistance(const float voltage) const;
    virtual float get_voltage(const float resistance) const;
};

struct ADCCircuitSimplePullup : ADCCircuit
{
    const float _pullup_resistance;
    ADCCircuitSimplePullup(const float pullup_resistance) : _pullup_resistance{pullup_resistance} {};

    float get_resistance(const float v) const override
    {
        return ((_pullup_resistance * v) / (kSystemVoltage - v));
    };

    float get_voltage(const float r) const override
    {
        return r/(r+_pullup_resistance)*kSystemVoltage;
    };
};

struct ADCCircuitDifferentialPullup : ADCCircuit
{
    const float _pullup_resistance;
    ADCCircuitDifferentialPullup(const float pullup_resistance) : _pullup_resistance{pullup_resistance} {};

    float get_resistance(float v) const override
    {
        float v2 = v / kSystemVoltage;
        return (v2 * 2.0 * _pullup_resistance)/(1.0 - v2);
    };

    float get_voltage(const float r) const override
    {
        return (kSystemVoltage * r)/(2.0 * _pullup_resistance + r);
    };
};

struct ADCCircuitRawResistance : ADCCircuit
{
    ADCCircuitRawResistance() {};

    float get_resistance(float v) const override
    {
        return v;
    };

    float get_voltage(const float r) const override
    {
        return r;
    };
};


template<typename ADC_t, uint16_t min_temp = 0, uint16_t max_temp = 300>
struct Thermistor {
    float c1, c2, c3;
    const ADCCircuit *circuit;

    ADC_t adc_pin;
    uint16_t raw_adc_value = 0;
    float raw_adc_voltage = 0.0;

    const float variance_max = 1.1;
    ValueHistory<20> history {variance_max};

    typedef Thermistor<ADC_t, min_temp, max_temp> type;

    // References for thermistor formulas:
    //  http://assets.newport.com/webDocuments-EN/images/AN04_Thermistor_Calibration_IX.PDF
    //  http://hydraraptor.blogspot.com/2012/11/more-accurate-thermistor-tables.html

//    Thermistor(const float temp_low, const float temp_med, const float temp_high, const float res_low, const float res_med, const float res_high, const ADCCircuit *_circuit)
//    : circuit{_circuit}
//      adc_pin {kNormal, [&]{this->adc_has_new_value();} }
//    {
//        setup(temp_low, temp_med, temp_high, res_low, res_med, res_high);
//        adc_pin.setInterrupts(kPinInterruptOnChange|kInterruptPriorityLow);
//        adc_pin.setVoltageRange(kSystemVoltage,
//                                0, //get_voltage_of_temp(min_temp),
//                                kSystemVoltage, //get_voltage_of_temp(max_temp),
//                                1000000.0);
//    };

    template <typename... Ts>
    Thermistor(const float temp_low, const float temp_med, const float temp_high, const float res_low, const float res_med, const float res_high, const ADCCircuit *_circuit, Ts&&... additional_values)
    : circuit{_circuit},
      adc_pin{kNormal, [&]{this->adc_has_new_value();}, additional_values...}
    {
        setup(temp_low, temp_med, temp_high, res_low, res_med, res_high);
        adc_pin.setInterrupts(kPinInterruptOnChange|kInterruptPriorityLow);
        adc_pin.setVoltageRange(kSystemVoltage,
                                0, //get_voltage_of_temp(min_temp),
                                kSystemVoltage, //get_voltage_of_temp(max_temp),
                                1000000.0);
    };

    void setup(const float temp_low, const float temp_med, const float temp_high, const float res_low, const float res_med, const float res_high) {
        float temp_low_fixed = temp_low + 273.15;
        float temp_med_fixed = temp_med + 273.15;
        float temp_high_fixed = temp_high + 273.15;

        // Intermediates - using cryptic names from the calibration paper for consistency.

        float a1 = log(res_low);
        float a2 = log(res_med);
        float a3 = log(res_high);

        float z = a1 - a2;
        float y = a1 - a3;
        float x = 1/temp_low_fixed - 1/temp_med_fixed;
        float w = 1/temp_low_fixed - 1/temp_high_fixed;

        float v = pow(a1,3) - pow(a2,3);
        float u = pow(a1,3) - pow(a3,3);

        c3 = (x-z*w/y)/(v-z*u/y);
        c2 = (x-c3*v)/z;
        c1 = 1/temp_low_fixed-c3*pow(a1,3)-c2*a1;
    };

    float temperature_exact() {
        // Sanity check:
        if (raw_adc_value < 1) {
            return -1; // invalid temperature from a thermistor
        }

        float r = get_resistance(); // resistance of thermistor

        if ((r < 0) || (r > TEMP_MIN_DISCONNECTED_RESISTANCE)) {
            return -1;
        }

        float lnr = log(r);
        float Tinv = c1 + (c2*lnr) + (c3*pow(lnr,3));
        return (1/Tinv) - 273.15; // final temperature
    };

    float get_resistance() {
        raw_adc_voltage = history.value();

        if (isnan(raw_adc_voltage)) {
            return -1;
        }

        return circuit->get_resistance(raw_adc_voltage);
    };

//    float get_resistance() {
//        if (raw_adc_value < 1) {
//            return -1; // invalid temperature from a thermistor
//        }
//
//        float v = raw_adc_voltage; // convert the ADC value to a voltage
//        return ((pullup_resistance * v) / (kSystemVoltage - v));   // resistance of thermistor
//    };

    uint16_t get_raw_value() {
        return raw_adc_value;
    };

    float get_voltage() {
        return raw_adc_voltage;
    };

    void start_sampling() {
        adc_pin.startSampling();
    };

    // Call back function from the ADC to tell it that the ADC has a new sample...
    void adc_has_new_value() {
        raw_adc_value = adc_pin.getRaw();
        float v = std::abs(adc_pin.getVoltage());
        history.add_sample(v);
    };
};


template<typename ADC_t, uint16_t min_temp = 0, uint16_t max_temp = 400>
struct PT100 {
    const ADCCircuit *circuit;

    ADC_t adc_pin;
    float raw_adc_voltage = 0.0;
    int32_t raw_adc_value = 0;

    bool new_sample_since_read = false;
    uint8_t reads_without_sample = 0;

    const float variance_max = 1.1;
    ValueHistory<20> history {variance_max};

    typedef PT100<ADC_t, min_temp, max_temp> type;

//    PT100(const ADCCircuit *_circuit)
//    : circuit{_circuit},
//      adc_pin{ADC_t::is_differential ? kDifferentialPair : kNormal, [&]{this->adc_has_new_value();} }
//    {
//        adc_pin.setInterrupts(kPinInterruptOnChange|kInterruptPriorityLow);
//        adc_pin.setVoltageRange(kSystemVoltage,
//                                get_voltage_of_temp(min_temp),
//                                get_voltage_of_temp(max_temp),
//                                6400.0);
//    };

    template <typename... Ts>
    PT100(const ADCCircuit *_circuit, Ts&&... additional_values)
    : circuit{_circuit},
      adc_pin{kNormal, [&](bool e){this->adc_has_new_value(e);}, additional_values...}
    {
        adc_pin.setInterrupts(kPinInterruptOnChange|kInterruptPriorityLow);
        adc_pin.setVoltageRange(kSystemVoltage,
                                get_voltage_of_temp(min_temp),
                                get_voltage_of_temp(max_temp),
                                1);    // ignored
    };

    constexpr float get_resistance_of_temp(float t) {
        // R = 100(1 + A*T + B*T^2); A = 3.9083*10^-3; B = -5.775*10^-7
        return 100 * (1 + 0.0039083*t + -0.0000005775*t*t);
    };

    constexpr float get_voltage_of_temp(float t) {
        float r = get_resistance_of_temp(t);

        return circuit->get_voltage(r);
    };

    float temperature_exact() {
        if (!new_sample_since_read) {
            reads_without_sample++;
            if (reads_without_sample > 10) {
                cm_alarm(STAT_TEMPERATURE_CONTROL_ERROR, "Sensor read failed 10 times.");
            }
        } else {
            reads_without_sample = 0;
        }
        new_sample_since_read = false;

        float r = get_resistance();
        if (r < 0.0) { return -1; }

        // from https://www.maximintegrated.com/en/app-notes/index.mvp/id/3450
        // run through wolfram as:
        // solve R = 100(1 + A*T + B*T^2); A = 3.9083*10^-3; B = -5.775*10^-7 for T
        float t = 3383.81 - (0.287154*sqrt(159861899.0 - 210000.0*r));

        if (t > max_temp) {
            return -1;
        }

        return t;
    };

    float get_resistance() {
        raw_adc_voltage = history.value();

        if (isnan(raw_adc_voltage)) {
            return -1;
        }

        return circuit->get_resistance(raw_adc_voltage);
    };

//    float get_resistance() {
//        float r;
//        raw_adc_voltage = history.value();
//
//        if (isnan(raw_adc_voltage)) {
//            return -1;
//        }
//
//        if (gives_raw_resistance) {
//            r = raw_adc_voltage;
//        }
//        else if (differential) {
//            float v = raw_adc_voltage / kSystemVoltage;
//            r = (v * 2.0 * pullup_resistance)/(1.0 - v) - inline_resistance;
//        }
//        else {
//            float v = raw_adc_voltage;
//            r = ((pullup_resistance * v) / (kSystemVoltage - v)) - inline_resistance;
//        }
//        return r;
//    };

    uint16_t get_raw_value() {
        return raw_adc_value;
    };

    float get_voltage() {
//        return history.value();
        return raw_adc_voltage;
    };

    void start_sampling() {
        adc_pin.startSampling();
    };

    // Call back function from the ADC to tell it that the ADC has a new sample...
    void adc_has_new_value(bool error = false) {
        raw_adc_value = adc_pin.getRaw();
        float v = std::abs(adc_pin.getVoltage());
//        if (v < 0) {
//            char buffer[128];
//            char *str = buffer;
//            str += sprintf(str, "Heater sensor failure. Reading was: %f", v);
//            cm_alarm(STAT_TEMPERATURE_CONTROL_ERROR, buffer);
//            return;
//        }
        history.add_sample(v);
        new_sample_since_read = true;
    };
};

// Temperature debug string: {sr:{"he1t":t,"he1st":t,"he1at":t, "he1tr":t, "he1op":t}}
// PID debug string: {sr:{"he1t":t,"he1st":t,"pid1p":t, "pid1i":t, "pid1d":t, "pid1f":t, "he1op":t, "line":t, "stat":t}}

#if HAS_TEMPERATURE_SENSOR_1
// Extruder 1
TEMPERATURE_SENSOR_1_CIRCUIT_TYPE temperature_sensor_1_circuit TEMPERATURE_SENSOR_1_CIRCUIT_INIT;
TEMPERATURE_SENSOR_1_TYPE temperature_sensor_1 TEMPERATURE_SENSOR_1_INIT;
#else
TemperatureSensor temperature_sensor_1;
#endif

// Extruder 2
#if HAS_TEMPERATURE_SENSOR_2
// Extruder 2
TEMPERATURE_SENSOR_2_CIRCUIT_TYPE temperature_sensor_2_circuit TEMPERATURE_SENSOR_2_CIRCUIT_INIT;
TEMPERATURE_SENSOR_2_TYPE temperature_sensor_2 TEMPERATURE_SENSOR_2_INIT;
#else
TemperatureSensor temperature_sensor_2;
#endif

#if HAS_TEMPERATURE_SENSOR_3
// Heated bed
TEMPERATURE_SENSOR_3_CIRCUIT_TYPE temperature_sensor_3_circuit TEMPERATURE_SENSOR_3_CIRCUIT_INIT;
TEMPERATURE_SENSOR_3_TYPE temperature_sensor_3 TEMPERATURE_SENSOR_3_INIT;
#else
TemperatureSensor temperature_sensor_3;
#endif

float last_reported_temp1 = 0; // keep track of what we've reported for SR generation
float last_reported_temp2 = 0;
float last_reported_temp3 = 0;


// Output 1 FET info
// DO_1: Extruder1_PWM
const int32_t fet_pin1_freq = 2000;
#if TEMPERATURE_OUTPUT_ON == 1
PWMOutputPin<EXTRUDER_1_OUTPUT_PIN> fet_pin1 {kNormal, fet_pin1_freq};// {kPWMPinInverted, fet_pin1_freq};
#else
PWMOutputPin<-1> fet_pin1;// {kPWMPinInverted};
#endif

// DO_2: Extruder2_PWM
const int32_t fet_pin2_freq = 2000;
#if TEMPERATURE_OUTPUT_ON == 1
PWMOutputPin<EXTRUDER_2_OUTPUT_PIN> fet_pin2 {kNormal, fet_pin2_freq};// {kPWMPinInverted, fet_pin1_freq};
#else
PWMOutputPin<-1> fet_pin2;// {kPWMPinInverted};
#endif

// DO_11: Heated Bed FET
// Warning, HeatBED is likely NOT a PWM pin, so it'll be binary output (duty cucle >= 50%).
const int32_t fet_pin3_freq = 100;
#if TEMPERATURE_OUTPUT_ON == 1
PWMOutputPin<BED_OUTPUT_PIN> fet_pin3 BED_OUTPUT_INIT;
#else
PWMOutputPin<-1> fet_pin3;// {kPWMPinInverted};
#endif


// DO_3: Fan1A_PWM
//PWMOutputPin<kOutput3_PinNumber> fan_pin1;
// DO_4: Fan1B_PWM
//PWMOutputPin<kOutput4_PinNumber> fan_pin2;
// DO_5: Fan2A_PWM
//PWMOutputPin<kOutput5_PinNumber> fan_pin3;


//// We're going to utilize the fet_pin1 PWMOutputPin<>'s timer interrupt to drive the ADC sampling.
//const int16_t fet_pin1_sample_freq = 1; // every fet_pin1_sample_freq interrupts, sample
//int16_t fet_pin1_sample_counter = fet_pin1_sample_freq;
//#if TEMPERATURE_OUTPUT_ON == 1
//namespace Motate {
//    template<>
//    void PWMOutputPin<kOutput1_PinNumber>::parentTimerType::interrupt() {
//        if (!--fet_pin1_sample_counter) {
//            ADC_Module::startSampling();
//            fet_pin1_sample_counter = fet_pin1_sample_freq;
//        }
//    };
//}
//#endif

#if TEMPERATURE_OUTPUT_ON == 1

// We're going to register a SysTick event
const int16_t temperature_sample_freq = 10; // every fet_pin1_sample_freq interrupts, sample
int16_t temperature_sample_counter = temperature_sample_freq;
SysTickEvent adc_tick_event {[&] {
    if (!--temperature_sample_counter) {
        temperature_sensor_1.start_sampling();
        temperature_sensor_2.start_sampling();
        temperature_sensor_3.start_sampling();
        temperature_sample_counter = temperature_sample_freq;
    }
}, nullptr};

#endif


struct PID {
    static constexpr float output_max = 1.0;
    static constexpr float derivative_contribution = 1.0/10.0;

    float _p_factor;                // the scale for P values
    float _i_factor;                // the scale for I values
    float _d_factor;                // the scale for D values
    float _f_factor;                // the scale for O values

    float _proportional = 0.0;      // _proportional storage
    float _integral = 0.0;          // _integral storage
    float _derivative = 0.0;        // _derivative storage
    float _feed_forward = 0.0;            // _feed_forward storage
    float _previous_input = 0.0;    // _derivative storage

    float _set_point;

    Timeout _set_point_timeout;     // used to keep track of if we are at set temp and stay there
    bool _at_set_point;

    Timeout _rise_time_timeout;     // used to keep track of if we are increasing temperature fast enough
    float _min_rise_over_time;      // the amount of degrees that it must rise in the given time
    float _rise_time_checkpoint;    // when we start the timer, we set _rise_time_checkpoint to the minimum goal

    float _average_output = 0;

    bool _enable;                   // set true to enable this heater

    PID(float P, float I, float D, float F, float min_rise_over_time, float startSetPoint = 0.0) : _p_factor{P/100.0f}, _i_factor{I/100.0f}, _d_factor{D/100.0f}, _f_factor{F/100.0f}, _set_point{startSetPoint}, _at_set_point{false}, _min_rise_over_time(min_rise_over_time) {};

    float getNewOutput(float input) {
        // If the input is < 0, the sensor failed
        if (input < 0) {
            if (_set_point > TEMP_OFF_BELOW) {
                cm_alarm(STAT_TEMPERATURE_CONTROL_ERROR, "Heater set, but sensor read failed.");
            }

            return 0;
        }

        // Calculate the e (error)
        float e = _set_point - input;

        if (std::abs(e) < TEMP_SETPOINT_HYSTERESIS) {
            if (!_set_point_timeout.isSet()) {
                _set_point_timeout.set(TEMP_SETPOINT_HOLD_TIME);
            } else if (_set_point_timeout.isPast()) {
                _at_set_point = true;
                _set_point_timeout.clear();
            }
        } else {
            _at_set_point = false;

            // Check to see if we already have the rise_time timeout set
            if (_rise_time_timeout.isSet()) {
                if (_rise_time_timeout.isPast()) {
                    if (input < _rise_time_checkpoint) {
                        // FAILURE!!
                        char buffer[128];
                        char *str = buffer;
                        str += sprintf(str, "Heater temperature failed to rise fast enough. At: %f Set: %f", input, _set_point);
                        cm_alarm(STAT_TEMPERATURE_CONTROL_ERROR, buffer);
                        _set_point = 0;
                        _rise_time_timeout.clear();
                        return -1;
                    }

                    _rise_time_timeout.clear();
                }
            }

            if (!_rise_time_timeout.isSet() && (_set_point > (input + TEMP_MIN_RISE_DEGREES_FROM_TARGET))) {
                _rise_time_timeout.set(TEMP_MIN_RISE_TIME);
                _rise_time_checkpoint = min(input + _min_rise_over_time, _set_point + TEMP_SETPOINT_HYSTERESIS);
            }
        }

        // P = Proportional

        float p = _p_factor * e;
        // For output's sake, we'll store this, otherwise we don't need it:
        _proportional = p;


        // I = Integral

        // Now, to restrict windup, prevent the integral from contributing too much, AND to keep it sane:
        // 1) Limit the i contribution to the output
        // 2) Limit the _integral maximum value
        // 3) Reset _integral to e if output has to be clamped (after output is computed)
        _integral += e;
        float i = _integral * _i_factor;

        if (i > 0.75) {
            i = 0.75;
            _integral = 0.75 / _i_factor;
        } else if (i < -0.75) {
            i = -0.75;
            _integral = -0.75 / _i_factor;
        }

        // D = derivative

        // This needs to be smoothed somewhat, so we use a exponential moving average.
        // See https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average


        _derivative = (input - _previous_input)*(derivative_contribution) + (_derivative * (1.0-derivative_contribution));
        float d = _derivative * _d_factor;

        // F = feed-forward

        _feed_forward = (_set_point-21); // 21 is for a roughly ideal room temperature

        float f = _f_factor * _feed_forward;

        _previous_input = input;

        // Now that we've computed all that, we'll decide when to ignore it

        float output = p + i + f - d;
        if (output < 0.0f) {
            output = 0;

            // reset the integral to prevent windup
            _integral = e;
        } else if (output > output_max) {
            output = output_max;

            // reset the integral to prevent windup
            _integral = e;
        }

        // If the setpoint is "off" or the temperature is higher than MAX, always return OFF
        if ((_set_point < TEMP_OFF_BELOW) || (input > TEMP_MAX_SETPOINT)) {
            output = 0; // "off"
            _average_output = 0;

            return 0;
        // If we are too far from the set point, turn the heater full on
        }
//        else if (e > TEMP_FULL_ON_DIFFERENCE) {
//            output = 1; // "on"
//        }

        // Keep track of our output with some averaging for output purposes
        _average_output = (0.5*output) + (0.5*_average_output);

        return _average_output; // return the smoothed value
    };

    bool atSetPoint() {
        return _at_set_point;
    }
};

// NOTICE, the JSON alters incoming values for these!
// {he1p:9} == 9.0/100.0 here

PID pid1 { 9.0, 0.11, 400.0, 0, TEMP_MIN_RISE_DEGREES_OVER_TIME }; // default values
PID pid2 { 7.5, 0.12, 400.0, 0, TEMP_MIN_RISE_DEGREES_OVER_TIME }; // default values
PID pid3 { 7.5, 0.12, 400.0, 0, TEMP_MIN_BED_RISE_DEGREES_OVER_TIME }; // default values
Timeout pid_timeout;


template<pin_number heater_fan_pinnum>
struct HeaterFan {
#if TEMPERATURE_OUTPUT_ON == 1
    PWMOutputPin<heater_fan_pinnum> heater_fan_pin;
#endif
    float min_value = MIN_FAN_VALUE;
    float max_value = MAX_FAN_VALUE;
    float low_temp = MIN_FAN_TEMP;
    float high_temp = MIN_FAN_TEMP;

    HeaterFan() {
#if TEMPERATURE_OUTPUT_ON == 1
        heater_fan_pin.setFrequency(200000);
        heater_fan_pin = 0;
#endif
    }

    void newTemp(float temp) {
#if TEMPERATURE_OUTPUT_ON == 1
        if ((temp > low_temp) && (temp < high_temp)) {
            heater_fan_pin = max_value * (((temp - low_temp)/(high_temp - low_temp))*(1.0 - min_value) + min_value);
        } else if (temp > high_temp) {
            heater_fan_pin = max_value;
        } else {
            heater_fan_pin = 0.0;
        }
#endif
    }
};

HeaterFan<EXTRUDER_1_FAN_PIN> heater_fan1;

/**** Static functions ****/


/*
 * temperature_init()
 * temperature_init() - stop spindle, set speed to zero, and reset values
 */
void temperature_init()
{
    // setup heater PWM
//    fet_pin1.setFrequency(fet_pin1_freq);
//    fet_pin2.setFrequency(fet_pin2_freq);
//    fet_pin3.setFrequency(fet_pin3_freq);

//    fan_pin1 = 0;
//    fan_pin1.setFrequency(200000);
//    fan_pin2 = 0;
//    fan_pin2.setFrequency(50000);
//    fan_pin3 = 1;
//    fan_pin3.setFrequency(200000);

    // Register the SysTick event (described above)
#if TEMPERATURE_OUTPUT_ON == 1
    SysTickTimer.registerEvent(&adc_tick_event);
#endif

    temperature_reset();
}

void temperature_reset()
{
    // make setpoint 0
    fet_pin1 = 0.0f;
    pid1._set_point = 0.0;

    fet_pin2 = 0.0f;
    pid2._set_point = 0.0;

    fet_pin3 = 0.0f;
    pid3._set_point = 0.0;

    pid_timeout.set(100);
}

// Minimum difference in temp before it'll trigger an SR
const float kTempDiffSRTrigger = 0.25;

stat_t temperature_callback()
{
    if (cm.machine_state == MACHINE_ALARM) {
        // Force the heaters off (redundant with the safety circuit)
        fet_pin1 = 0.0;
        fet_pin2 = 0.0;
        fet_pin3 = 0.0;

        // Force all PIDs to off too
        pid1._set_point = 0.0;
        pid2._set_point = 0.0;
        pid3._set_point = 0.0;

        return (STAT_OK);
    }

    if (pid_timeout.isPast()) {
        pid_timeout.set(100);

        float temp = 0.0;
        float fan_temp = 0.0;
        bool sr_requested = false;

        if (pid1._enable) {
            temp = temperature_sensor_1.temperature_exact();
            float out1 = pid1.getNewOutput(temp);
            fet_pin1.write(out1);

            if (std::abs(temp - last_reported_temp1) > kTempDiffSRTrigger) {
                last_reported_temp1 = temp;
                sr_requested = true;
            }
        }
        fan_temp = temp;

        if (pid2._enable) {
            temp = temperature_sensor_2.temperature_exact();
            float out2 = pid2.getNewOutput(temp);
            fet_pin2.write(out2);

            if (std::abs(temp - last_reported_temp2) > kTempDiffSRTrigger) {
                last_reported_temp2 = temp;
                sr_requested = true;
            }
        }
        fan_temp = max(fan_temp, temp);

        heater_fan1.newTemp(fan_temp);

        if (pid3._enable) {
            temp = temperature_sensor_3.temperature_exact();
            float out3 = pid3.getNewOutput(temp);
            fet_pin3.write(out3);

            if (std::abs(temp - last_reported_temp3) > kTempDiffSRTrigger) {
                last_reported_temp3 = temp;
                sr_requested = true;
            }
        }

        if (sr_requested) {
            sr_request_status_report(SR_REQUEST_TIMED);
        }
    }
    return (STAT_OK);
}


/********************************
 * END OF TEMPERATURE FUNCTIONS *
 ********************************/

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

// In these functions, nv->group == "he1", "he2", or "he3"
char _get_heater_number(nvObj_t *nv) {
    if (!nv->group[0]) {
        return nv->token[2];
    }
    return nv->group[2];
}

stat_t cm_get_heater_enable(nvObj_t *nv)
{
    switch(_get_heater_number(nv)) {
        case '1': { nv->value = pid1._enable; break; }
        case '2': { nv->value = pid2._enable; break; }
        case '3': { nv->value = pid3._enable; break; }
        // Failsafe. We can only get here if we set it up in config_app, but not here.
        default: { return(STAT_INPUT_VALUE_RANGE_ERROR); break; }
    }
    nv->valuetype = TYPE_BOOL;
    return (STAT_OK);
}

stat_t cm_set_heater_enable(nvObj_t *nv)
{
    bool enable = false;
    if (nv->value > 1) {                        // testing a boolean value
        return (STAT_INPUT_VALUE_RANGE_ERROR);
    }
    if (nv->value > 0.1) {
        enable = true;
    }
    // The above manipulation of 'enable' was necessary because the compiler won't accept this cast:
    // pid1._enable = (bool)nv->value;   // says it's unsafe to compare ==, != an FP number

    switch(_get_heater_number(nv)) {
        //        case '1': { pid1._enable = (bool)nv->value; break; }
        //        case '2': { pid2._enable = (bool)nv->value; break; }
        //        case '3': { pid3._enable = (bool)nv->value; break; }
        case '1': { pid1._enable = enable; break; }
        case '2': { pid2._enable = enable; break; }
        case '3': { pid3._enable = enable; break; }
        default: { return(STAT_INPUT_VALUE_RANGE_ERROR); break; } // Failsafe. We can only get here if we set it up in config_app, but not here.
    }
    return (STAT_OK);
}

/*
 * cm_get_heater_p()/cm_set_heater_p() - get/set the P parameter of the PID
 */
stat_t cm_get_heater_p(nvObj_t *nv)
{
    // there are three of them, so we can use a simple switch
    switch(_get_heater_number(nv)) {
        case '1': { nv->value = pid1._p_factor * 100.0; break; }
        case '2': { nv->value = pid2._p_factor * 100.0; break; }
        case '3': { nv->value = pid3._p_factor * 100.0; break; }

        // Failsafe. We can only get here if we set it up in config_app, but not here.
        default: { nv->value = 0.0; break; }
    }
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}
stat_t cm_set_heater_p(nvObj_t *nv)
{
    switch(_get_heater_number(nv)) {
        case '1': { pid1._p_factor = nv->value / 100.0; break; }
        case '2': { pid2._p_factor = nv->value / 100.0; break; }
        case '3': { pid3._p_factor = nv->value / 100.0; break; }

        default: { break; }
    }

    return (STAT_OK);
}

/*
 * cm_get_heater_i()/cm_set_heater_i() - get/set the I parameter of the PID
 */
stat_t cm_get_heater_i(nvObj_t *nv)
{
    // there are three of them, so we can use a simple switch
    switch(_get_heater_number(nv)) {
        case '1': { nv->value = pid1._i_factor * 100.0; break; }
        case '2': { nv->value = pid2._i_factor * 100.0; break; }
        case '3': { nv->value = pid3._i_factor * 100.0; break; }

        default: { nv->value = 0.0; break; }
    }

    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}
stat_t cm_set_heater_i(nvObj_t *nv)
{
    switch(_get_heater_number(nv)) {
        case '1': { pid1._i_factor = nv->value / 100.0; break; }
        case '2': { pid2._i_factor = nv->value / 100.0; break; }
        case '3': { pid3._i_factor = nv->value / 100.0; break; }

        default: { break; }
    }

    return (STAT_OK);
}

/*
 * cm_get_heater_d()/cm_set_heater_d() - get/set the D parameter of the PID
 */
stat_t cm_get_heater_d(nvObj_t *nv)
{
    switch(_get_heater_number(nv)) {
        case '1': { nv->value = pid1._d_factor * 100.0; break; }
        case '2': { nv->value = pid2._d_factor * 100.0; break; }
        case '3': { nv->value = pid3._d_factor * 100.0; break; }

        default: { nv->value = 0.0; break; }
    }
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}
stat_t cm_set_heater_d(nvObj_t *nv)
{
    switch(_get_heater_number(nv)) {
        case '1': { pid1._d_factor = nv->value / 100.0; break; }
        case '2': { pid2._d_factor = nv->value / 100.0; break; }
        case '3': { pid3._d_factor = nv->value / 100.0; break; }

        default: { break; }
    }
    return (STAT_OK);
}

/*
 * cm_get_heater_f()/cm_set_heater_f() - get/set the F parameter of the PIDF
 */
stat_t cm_get_heater_f(nvObj_t *nv)
{
    switch(_get_heater_number(nv)) {
        case '1': { nv->value = pid1._f_factor * 100.0; break; }
        case '2': { nv->value = pid2._f_factor * 100.0; break; }
        case '3': { nv->value = pid3._f_factor * 100.0; break; }

        default: { nv->value = 0.0; break; }
    }
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}
stat_t cm_set_heater_f(nvObj_t *nv)
{
    switch(_get_heater_number(nv)) {
        case '1': { pid1._f_factor = nv->value / 100.0; break; }
        case '2': { pid2._f_factor = nv->value / 100.0; break; }
        case '3': { pid3._f_factor = nv->value / 100.0; break; }

        default: { break; }
    }
    return (STAT_OK);
}

/*
 * cm_get_set_temperature()/cm_set_set_temperature() - get/set the set value of the PID
 *
 *   There are both the file-to-file use version, and the NV-pair form (which uses the other).
 */
float cm_get_set_temperature(const uint8_t heater)
{
    switch(heater) {
        case 1: { return pid1._set_point; break; }
        case 2: { return pid2._set_point; break; }
        case 3: { return pid3._set_point; break; }
        default: { break; }
    }

    return 0.0;
}
stat_t cm_get_set_temperature(nvObj_t *nv)
{
    nv->value = cm_get_set_temperature(_get_heater_number(nv) - '0');
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}
void cm_set_set_temperature(const uint8_t heater, const float value)
{
    switch(heater) {
        case 1: { pid1._set_point = min(TEMP_MAX_SETPOINT, value); break; }
        case 2: { pid2._set_point = min(TEMP_MAX_SETPOINT, value); break; }
        case 3: { pid3._set_point = min(TEMP_MAX_SETPOINT, value); break; }

        // default to quiet the compiler
        default: { break; }
    }
}
stat_t cm_set_set_temperature(nvObj_t *nv)
{
    cm_set_set_temperature(_get_heater_number(nv) - '0', nv->value);
    return (STAT_OK);
}

/*
 * cm_get_fan_power()/cm_set_fan_power() - get/set the set high-value setting of the heater fan
 */
float cm_get_fan_power(const uint8_t heater)
{
    switch(heater) {
        case 1: { return min(1.0f, heater_fan1.max_value); }
//        case 2: { return min(1.0f, heater_fan2.max_value); }
//        case 3: { return min(1.0f, heater_fan3.max_value); }

        default: { break; }
    }

    return 0.0;
}
stat_t cm_get_fan_power(nvObj_t *nv)
{
    nv->value = cm_get_fan_power(_get_heater_number(nv) - '0');
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}

void cm_set_fan_power(const uint8_t heater, const float value)
{
    switch(heater) {
        case 1: { heater_fan1.max_value = max(0.0f, value); break; }
//        case 2: { heater_fan2.max_value = max(0.0, value); break; }
//        case 3: { heater_fan3.max_value = max(0.0, value); break; }
        default: { break; }
    }
}
stat_t cm_set_fan_power(nvObj_t *nv)
{
    cm_set_fan_power(_get_heater_number(nv) - '0', nv->value);
    return (STAT_OK);
}

/*
 * cm_get_fan_min_power()/cm_set_fan_min_power() - get/set the set high-value setting of the heater fan
 */
stat_t cm_get_fan_min_power(nvObj_t *nv)
{
    switch(_get_heater_number(nv)) {
        case '1': { nv->value = heater_fan1.min_value; break; }
//        case '2': { nv->value = heater_fan2.min_value; break; }
//        case '3': { nv->value = heater_fan3.min_value; break; }

        default: { nv->value = 0.0; break; }
    }

    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}
stat_t cm_set_fan_min_power(nvObj_t *nv)
{
    switch(_get_heater_number(nv)) {
        case '1': { heater_fan1.max_value = min(0.0f, nv->value); break; }
//        case '2': { heater_fan2.min_value = min(0.0, nv->value); break; }
//        case '3': { heater_fan3.min_value = min(0.0, nv->value); break; }

        default: { break; }
    }

    return (STAT_OK);
}

/*
 * cm_get_fan_low_temp()/cm_set_fan_low_temp() - get/set the set high-value setting of the heater fan
 */
stat_t cm_get_fan_low_temp(nvObj_t *nv)
{
    switch(_get_heater_number(nv)) {
        case '1': { nv->value = heater_fan1.low_temp; break; }
//        case '2': { nv->value = heater_fan2.low_temp; break; }
//        case '3': { nv->value = heater_fan3.low_temp; break; }

        default: { nv->value = 0.0; break; }
    }

    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}
stat_t cm_set_fan_low_temp(nvObj_t *nv)
{
    switch(_get_heater_number(nv)) {
        case '1': { heater_fan1.low_temp = min(0.0f, nv->value); break; }
//        case '2': { heater_fan2.low_temp = min(0.0f, nv->value); break; }
//        case '3': { heater_fan3.low_temp = min(0.0f, nv->value); break; }

        default: { break; }
    }

    return (STAT_OK);
}

/*
 * cm_get_fan_high_temp()/cm_set_fan_high_temp() - get/set the set high-value setting of the heater fan
 */
stat_t cm_get_fan_high_temp(nvObj_t *nv)
{
    switch(_get_heater_number(nv)) {
        case '1': { nv->value = heater_fan1.high_temp; break; }
//        case '2': { nv->value = heater_fan2.high_temp; break; }
//        case '3': { nv->value = heater_fan3.high_temp; break; }

        default: { nv->value = 0.0; break; }
    }

    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}
stat_t cm_set_fan_high_temp(nvObj_t *nv)
{
    switch(_get_heater_number(nv)) {
        case '1': { heater_fan1.high_temp = min(0.0f, nv->value); break; }
//        case '2': { heater_fan2.high_temp = min(0.0f, nv->value); break; }
//        case '3': { heater_fan3.high_temp = min(0.0f, nv->value); break; }

        default: { break; }
    }

    return (STAT_OK);
}

/*
 * cm_get_at_temperature() - get a boolean if the heater has reaced the set value of the PID
 */
bool cm_get_at_temperature(const uint8_t heater)
{
    switch(heater) {
        case 1: { return pid1._at_set_point; }
        case 2: { return pid2._at_set_point; }
        case 3: { return pid3._at_set_point; }

        default: { break; }
    }

    return false;
}
stat_t cm_get_at_temperature(nvObj_t *nv)
{
    nv->value = cm_get_at_temperature(_get_heater_number(nv) - '0');
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_BOOL;

    return (STAT_OK);
}


/*
 * cm_get_heater_output() - get the output value (PWM duty cycle) of the PID
 */
float cm_get_heater_output(const uint8_t heater)
{
    switch(heater) {
        case 1: { return pid1._average_output; }
        case 2: { return pid2._average_output; }
        case 3: { return pid3._average_output; }

        default: { break; }
    }
    return 0.0;
}
stat_t cm_get_heater_output(nvObj_t *nv)
{
    nv->value = cm_get_heater_output(_get_heater_number(nv) - '0');
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}

/*
 * cm_get_heater_adc() - get the raw adc value of the PID
 */
stat_t cm_get_heater_adc(nvObj_t *nv)
{
    switch(_get_heater_number(nv)) {
        case '1': { nv->value = (float)temperature_sensor_1.get_raw_value(); break; }
        case '2': { nv->value = (float)temperature_sensor_2.get_raw_value(); break; }
        case '3': { nv->value = (float)temperature_sensor_3.get_raw_value(); break; }

        default: { nv->value = 0.0; break; }
    }
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}

/*
 * cm_get_temperature() - get the current temperature
 */
 float cm_get_temperature(const uint8_t heater)
 {
     switch(heater) {
         case 1: { return (last_reported_temp1 = temperature_sensor_1.temperature_exact()); }
         case 2: { return (last_reported_temp2 = temperature_sensor_2.temperature_exact()); }
         case 3: { return (last_reported_temp3 = temperature_sensor_3.temperature_exact()); }

         default: { break; }
     }

     return 0.0;
 }
stat_t cm_get_temperature(nvObj_t *nv)
{
    nv->value = cm_get_temperature(_get_heater_number(nv) - '0');
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}

/*
 * cm_get_thermistor_resistance() - get the current temperature
 */
stat_t cm_get_thermistor_resistance(nvObj_t *nv)
{
    switch(_get_heater_number(nv)) {
        case '1': { nv->value = temperature_sensor_1.get_resistance(); break; }
        case '2': { nv->value = temperature_sensor_2.get_resistance(); break; }
        case '3': { nv->value = temperature_sensor_3.get_resistance(); break; }

        default: { nv->value = 0.0; break; }
    }
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}

/*
 * cm_get_thermistor_resistance() - get the current temperature
 */
stat_t cm_get_thermistor_voltage(nvObj_t* nv)
{
    switch(_get_heater_number(nv)) {
        case '1': { nv->value = temperature_sensor_1.get_voltage(); break; }
        case '2': { nv->value = temperature_sensor_2.get_voltage(); break; }
        case '3': { nv->value = temperature_sensor_3.get_voltage(); break; }

            // Failsafe. We can only get here if we set it up in config_app, but not here.
        default: { nv->value = 0.0; break; }
    }
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}



// In these functions, nv->group == "pid1", "pid2", or "pid3"
char _get_pid_number(nvObj_t *nv) {
    if (!nv->group[0]) {
        return nv->token[3];
    }
    return nv->group[3];
}


/*
 * cm_get_pid_p() - get the active P of the PID (read-only)
 */
stat_t cm_get_pid_p(nvObj_t *nv)
{
    switch(_get_pid_number(nv)) {
        case '1': { nv->value = pid1._proportional; break; }
        case '2': { nv->value = pid2._proportional; break; }
        case '3': { nv->value = pid3._proportional; break; }

        default: { nv->value = 0.0; break; }
    }
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}

/*
 * cm_get_pid_i() - get the active I of the PID (read-only)
 */
stat_t cm_get_pid_i(nvObj_t *nv)
{
    switch(_get_pid_number(nv)) {
        case '1': { nv->value = pid1._integral; break; }
        case '2': { nv->value = pid2._integral; break; }
        case '3': { nv->value = pid3._integral; break; }

        default: { nv->value = 0.0; break; }
    }
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}

/*
 * cm_get_pid_d() - get the active D of the PID (read-only)
 */
stat_t cm_get_pid_d(nvObj_t *nv)
{
    switch(_get_pid_number(nv)) {
        case '1': { nv->value = pid1._derivative; break; }
        case '2': { nv->value = pid2._derivative; break; }
        case '3': { nv->value = pid3._derivative; break; }

        default: { nv->value = 0.0; break; }
    }
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}

/*
 * cm_get_pid_f() - get the active F of the PID (read-only)
 */
stat_t cm_get_pid_f(nvObj_t *nv)
{
    switch(_get_pid_number(nv)) {
        case '1': { nv->value = pid1._feed_forward; break; }
        case '2': { nv->value = pid2._feed_forward; break; }
        case '3': { nv->value = pid3._feed_forward; break; }

        default: { nv->value = 0.0; break; }
    }
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}


/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

//const char fmt_spep[] = "[spep] spindle enable polarity%5d [0=active_low,1=active_high]\n";
//const char fmt_spdp[] = "[spdp] spindle direction polarity%2d [0=CW_low,1=CW_high]\n";
//const char fmt_spph[] = "[spph] spindle pause on hold%7d [0=no,1=pause_on_hold]\n";
//const char fmt_spdw[] = "[spdw] spindle dwell time%12.1f seconds\n";
//const char fmt_ssoe[] ="[ssoe] spindle speed override ena%2d [0=disable,1=enable]\n";
//const char fmt_sso[] ="[sso] spindle speed override%11.3f [0.050 < sso < 2.000]\n";
//const char fmt_spe[] = "Spindle Enable:%7d [0=OFF,1=ON,2=PAUSE]\n";
//const char fmt_spd[] = "Spindle Direction:%4d [0=CW,1=CCW]\n";
//const char fmt_sps[] = "Spindle Speed: %7.0f rpm\n";
//
//void cm_print_spep(nvObj_t *nv) { text_print(nv, fmt_spep);}    // TYPE_INT
//void cm_print_spdp(nvObj_t *nv) { text_print(nv, fmt_spdp);}    // TYPE_INT
//void cm_print_spph(nvObj_t *nv) { text_print(nv, fmt_spph);}    // TYPE_INT
//void cm_print_spdw(nvObj_t *nv) { text_print(nv, fmt_spdw);}    // TYPE_FLOAT
//void cm_print_ssoe(nvObj_t *nv) { text_print(nv, fmt_ssoe);}    // TYPE INT
//void cm_print_sso(nvObj_t *nv)  { text_print(nv, fmt_sso);}     // TYPE FLOAT
//void cm_print_spe(nvObj_t *nv)  { text_print(nv, fmt_spe);}     // TYPE_INT
//void cm_print_spd(nvObj_t *nv)  { text_print(nv, fmt_spd);}     // TYPE_INT
//void cm_print_sps(nvObj_t *nv)  { text_print(nv, fmt_sps);}     // TYPE_FLOAT

#endif // __TEXT_MODE
