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


template<pin_number adc_pin_num, uint16_t min_temp = 0, uint16_t max_temp = 300, uint32_t table_size=64>
struct Thermistor {
    float c1, c2, c3, pullup_resistance, inline_resistance;
    // We'll pull adc top value from the adc_pin.getTop()

    ADCPin<adc_pin_num> adc_pin;
    uint16_t raw_adc_value = 0;

    typedef Thermistor<adc_pin_num, min_temp, max_temp, table_size> type;

    // References for thermistor formulas:
    //  http://assets.newport.com/webDocuments-EN/images/AN04_Thermistor_Calibration_IX.PDF
    //  http://hydraraptor.blogspot.com/2012/11/more-accurate-thermistor-tables.html

    Thermistor(const float temp_low, const float temp_med, const float temp_high, const float res_low, const float res_med, const float res_high, const float pullup_resistance_, const float inline_resistance_ = 0)
    : pullup_resistance{ pullup_resistance_ }, inline_resistance { inline_resistance_ } {
        setup(temp_low, temp_med, temp_high, res_low, res_med, res_high);
        adc_pin.setInterrupts(kPinInterruptOnChange|kInterruptPriorityLow);
    }

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

        //        int16_t temp = min_temp;
        //        for (int i=0; temp < max_temp && i < table_size; i++) {
        //            lookup_table[i][0] = adc_value(temp);
        //            lookup_table[i][0] = temp;
        //
        //            temp += (min_temp-max_temp)/(table_size-1);
        //        }
    };

//    uint16_t adc_value_(int16_t temp) {
//        float y = (c1 - (1/(temp+273.15))) / (2*c3);
//        float x = sqrt(pow(c2 / (3*c3),3) + pow(y,2));
//        float r = exp((x-y) - (x+y)); // resistance of thermistor
//        return (r / (pullup_resistance + r)) * (adc_pin.getTop());
//    };

    float temperature_exact() {
        // Sanity check:
        if (raw_adc_value < 1) {
            return -1; // invalid temperature from a thermistor
        }

        float v = (float)raw_adc_value * kSystemVoltage / (adc_pin.getTop()); // convert the 10 bit ADC value to a voltage
        float r = ((pullup_resistance * v) / (kSystemVoltage - v)) - inline_resistance;   // resistance of thermistor

        if ((r < 0) || (r > TEMP_MIN_DISCONNECTED_RESISTANCE)) {
            return -1;
        }

        float lnr = log(r);
        float Tinv = c1 + (c2*lnr) + (c3*pow(lnr,3));
        return (1/Tinv) - 273.15; // final temperature
    };

    float get_resistance() {
        if (raw_adc_value < 1) {
            return -1; // invalid temperature from a thermistor
        }

        float v = (float)raw_adc_value * kSystemVoltage / (adc_pin.getTop()); // convert the 10 bit ADC value to a voltage
        return ((pullup_resistance * v) / (kSystemVoltage - v)) - inline_resistance;   // resistance of thermistor
    }

    // Call back function from the ADC to tell it that the ADC has a new sample...
    void adc_has_new_value() {
        raw_adc_value = (adc_pin.getRaw() + (9 * raw_adc_value))/10;
    };
};

// Temperature debug string: {sr:{"he1t":t,"he1st":t,"he1at":t, "he1tr":t, "he1op":t}}
// PID debug string: {sr:{"he1t":t,"he1st":t,"pid1p":t, "pid1i":t, "pid1d":t, "he1op":t, "line":t, "stat":t}}

// Extruder 1
Thermistor<kADC1_PinNumber> thermistor1 {
    /*T1:*/     20.0, /*T2:*/  195.0, /*T3:*/ 255.0,
    /*R1:*/ 140000.0, /*R2:*/  593.0, /*R3:*/ 189.0, /*pullup_resistance:*/ 4700, /*inline_resistance:*/ 4700
    };

#if ADC1_AVAILABLE == 1
namespace Motate {
template<>
void ADCPin<kADC1_PinNumber>::interrupt() {
    thermistor1.adc_has_new_value();
};
}
#endif

// Extruder 2
Thermistor<kADC2_PinNumber> thermistor2 {
    /*T1:*/     20.0, /*T2:*/  190.0, /*T3:*/ 255.0,
    /*R1:*/ 140000.0, /*R2:*/  490.0, /*R3:*/ 109.0, /*pullup_resistance:*/ 4700, /*inline_resistance:*/ 4700
    };
#if ADC2_AVAILABLE == 1
namespace Motate {
template<>
void ADCPin<kADC2_PinNumber>::interrupt() {
    thermistor2.adc_has_new_value();
};
}
#endif

// Heated bed
Thermistor<kADC0_PinNumber> thermistor3 {
    /*T1:*/     20.0, /*T2:*/     42.0, /*T3:*/ 76.0,
    /*R1:*/ 140000.0, /*R2:*/  36755.0, /*R3:*/ 10000.0, /*pullup_resistance:*/ 4700, /*inline_resistance:*/ 4700
    };
#if ADC0_AVAILABLE == 1
namespace Motate {
template<>
void ADCPin<kADC0_PinNumber>::interrupt() {
    thermistor3.adc_has_new_value();
};
}
#endif

float last_reported_temp1 = 0; // keep track of what we've reported for SR generation
float last_reported_temp2 = 0;
float last_reported_temp3 = 0;


// Output 1 FET info
// DO_1: Extruder1_PWM
const int16_t fet_pin1_freq = 100;
#if TEMPERATURE_OUTPUT_ON == 1
PWMOutputPin<kOutput1_PinNumber> fet_pin1;// {kPWMPinInverted};
#else
PWMOutputPin<-1> fet_pin1;// {kPWMPinInverted};
#endif

// DO_2: Extruder2_PWM
const int16_t fet_pin2_freq = 100;
#if TEMPERATURE_OUTPUT_ON == 1
PWMOutputPin<kOutput2_PinNumber> fet_pin2;// {kPWMPinInverted};
#else
PWMOutputPin<-1> fet_pin2;// {kPWMPinInverted};
#endif

// DO_11: Heated Bed FET
// Warning, HeatBED is likely NOT a PWM pin, so it'll be binary output (duty cucle >= 50%).
const int16_t fet_pin3_freq = 100;
#if TEMPERATURE_OUTPUT_ON == 1
PWMOutputPin<kOutput11_PinNumber> fet_pin3;// {kPWMPinInverted};
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
const int16_t fet_pin1_sample_freq = 10; // every fet_pin1_sample_freq interrupts, sample
int16_t fet_pin1_sample_counter = fet_pin1_sample_freq;
SysTickEvent adc_tick_event {[&] {
    if (!--fet_pin1_sample_counter) {
        ADC_Module::startSampling();
        fet_pin1_sample_counter = fet_pin1_sample_freq;
    }
}, nullptr};

#endif


struct PID {
    static constexpr float output_max = 1.0;
    static constexpr float derivative_contribution = 0.05;

    float _p_factor;                // the scale for P values
    float _i_factor;                // the scale for I values
    float _d_factor;                // the scale for D values

    float _proportional = 0.0;      // _proportional storage
    float _integral = 0.0;          // _integral storage
    float _derivative = 0.0;        // _derivative storage
    float _previous_input = 0.0;    // _derivative storage

    float _set_point;

    Timeout _set_point_timeout;     // used to keep track of if we are at set temp and stay there
    bool _at_set_point;

    Timeout _rise_time_timeout;     // used to keep track of if we are increasing temperature fast enough
    float _min_rise_over_time;      // the amount of degrees that it must rise in the given time
    float _rise_time_checkpoint;    // when we start the timer, we set _rise_time_checkpoint to the minimum goal

    bool _enable;                   // set true to enable this heater

    PID(float P, float I, float D, float min_rise_over_time, float startSetPoint = 0.0) : _p_factor{P/100.0f}, _i_factor{I/100.0f}, _d_factor{D/100.0f}, _set_point{startSetPoint}, _at_set_point{false}, _min_rise_over_time(min_rise_over_time) {};

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

        if (fabs(e) < TEMP_SETPOINT_HYSTERESIS) {
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

        float p = _p_factor * e;
        // For output's sake, we'll store this, otherwise we don't need it:
        _proportional = p;

        _integral += e;

        if (_integral < 0.0) {
            _integral = 0.0;
        }

        float i = _integral * _i_factor;

        if (i > output_max) {
            _integral = output_max / _i_factor;
            i = output_max;
        }


        _derivative = (_d_factor * (input - _previous_input))*(derivative_contribution) + (_derivative * (1.0-derivative_contribution));
        _previous_input = input;

        // Now that we've computed all that, we'll decide when to ignore it

        // If the setpoint is "off" or the temperature is higher than MAX, always return OFF
        if ((_set_point < TEMP_OFF_BELOW) || (input > TEMP_MAX_SETPOINT)) {
            return 0; // "off"

        // If we are too far from the set point, turn the heater full on
        } else if (e > TEMP_FULL_ON_DIFFERENCE) {
            return 1; //"on"
        }

        return std::min(output_max, p + i - _derivative);
    };

    bool atSetPoint() {
        return _at_set_point;
    }

// //New-style JSON bindings. DISABLED FOR NOW.
//    auto json_bindings(const char *object_name) {
//        return JSON::bind_object(object_name,
//                                 JSON::bind("set", _set_point,    /*print precision:*/2),
//                                 JSON::bind("p",   _proportional, /*print precision:*/2),
//                                 JSON::bind("i",   _integral,     /*print precision:*/5),
//                                 JSON::bind("d",   _derivative,   /*print precision:*/5)
//                                 );
//    }
};

// NOTICE, the JSON alters incoming values for these!
// {he1p:9} == 9.0/100.0 here

PID pid1 { 9.0, 0.11, 400.0, TEMP_MIN_RISE_DEGREES_OVER_TIME }; // default values
PID pid2 { 7.5, 0.12, 400.0, TEMP_MIN_RISE_DEGREES_OVER_TIME }; // default values
PID pid3 { 7.5, 0.12, 400.0, TEMP_MIN_BED_RISE_DEGREES_OVER_TIME }; // default values
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

HeaterFan<kOutput3_PinNumber> heater_fan1;

/**** Static functions ****/


/*
 * temperature_init()
 * temperature_init() - stop spindle, set speed to zero, and reset values
 */
void temperature_init()
{
    // setup heater PWM
    fet_pin1.setFrequency(fet_pin1_freq);
    fet_pin1.setInterrupts(kInterruptOnOverflow|kInterruptPriorityLowest);

    fet_pin2.setFrequency(fet_pin2_freq);
    fet_pin3.setFrequency(fet_pin3_freq);

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
        bool sr_requested = false;

        if (pid1._enable) {
            temp = thermistor1.temperature_exact();
            fet_pin1 = pid1.getNewOutput(temp);

            if (fabs(temp - last_reported_temp1) > kTempDiffSRTrigger) {
                last_reported_temp1 = temp;
                sr_requested = true;
            }
        }

        heater_fan1.newTemp(temp);

        if (pid2._enable) {
            temp = thermistor2.temperature_exact();
            fet_pin2 = pid2.getNewOutput(temp);

            if (fabs(temp - last_reported_temp2) > kTempDiffSRTrigger) {
                last_reported_temp2 = temp;
                sr_requested = true;
            }
        }

        if (pid3._enable) {
            temp = thermistor3.temperature_exact();
            fet_pin3 = pid3.getNewOutput(temp);

            if (fabs(temp - last_reported_temp3) > kTempDiffSRTrigger) {
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
        case 1: { return (float)fet_pin1; }
        case 2: { return (float)fet_pin2; }
        case 3: { return (float)fet_pin3; }

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
        case '1': { nv->value = (float)thermistor1.raw_adc_value; break; }
        case '2': { nv->value = (float)thermistor2.raw_adc_value; break; }
        case '3': { nv->value = (float)thermistor3.raw_adc_value; break; }

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
         case 1: { return (last_reported_temp1 = thermistor1.temperature_exact()); }
         case 2: { return (last_reported_temp2 = thermistor2.temperature_exact()); }
         case 3: { return (last_reported_temp3 = thermistor3.temperature_exact()); }

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
        case '1': { nv->value = thermistor1.get_resistance(); break; }
        case '2': { nv->value = thermistor2.get_resistance(); break; }
        case '3': { nv->value = thermistor3.get_resistance(); break; }

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
