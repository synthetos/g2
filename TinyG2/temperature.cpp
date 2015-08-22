/*
 * temperature.cpp - temperature control module - drives heaters or coolers
 * This file is part of the TinyG project
 *
 * Copyright (c) 2015 Robert Giseburt
 * Copyright (c) 2015 Alden S. Hart, Jr.
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

#include "tinyg2.h"             // #1 dependency order
#include "config.h"             // #2
#include "canonical_machine.h"  // #3
#include "text_parser.h"        // #4

#include "temperature.h"
#include "planner.h"
#include "hardware.h"
#include "pwm.h"
#include "util.h"

/**** Allocate structures ****/

// This makes the Motate:: prefix unnecessary.
using namespace Motate;

/****** Create file-global objects ******/

// The should be set in hardware.h for each board.
// Luckily, we only use boards that are 3.3V logic at the moment.
const float kSystemVoltage = 3.3;

template<pin_number adc_pin_num, uint16_t min_temp = 0, uint16_t max_temp = 300, uint32_t table_size=64>
struct Thermistor {
    float c1, c2, c3, pullup_resistance;
    // We'll pull adc top value from the adc_pin.getTop()

    ADCPin<adc_pin_num> adc_pin;
    uint16_t raw_adc_value = 0;

    typedef Thermistor<adc_pin_num, min_temp, max_temp, table_size> type;

    // References for thermistor formulas:
    //  http://assets.newport.com/webDocuments-EN/images/AN04_Thermistor_Calibration_IX.PDF
    //  http://hydraraptor.blogspot.com/2012/11/more-accurate-thermistor-tables.html

    Thermistor(const float temp_low, const float temp_med, const float temp_high, const float res_low, const float res_med, const float res_high, const float pullup_resistance_)
    : pullup_resistance{ pullup_resistance_ }    {
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

    uint16_t adc_value_(int16_t temp) {
        float y = (c1 - (1/(temp+273.15))) / (2*c3);
        float x = sqrt(pow(c2 / (3*c3),3) + pow(y,2));
        float r = exp((x-y) - (x+y)); // resistance of thermistor
        return (r / (pullup_resistance + r)) * (adc_pin.getTop());
    };

    float temperature_exact() {
        // Sanity check:
        if (raw_adc_value < 1) {
            return -1; // invalid temperature from a thermistor
        }

        float v = (float)raw_adc_value * kSystemVoltage / (adc_pin.getTop()); // convert the 10 bit ADC value to a voltage
        float r = (pullup_resistance * v) / (kSystemVoltage - v);   // resistance of thermistor
        float lnr = log(r);
        float Tinv = c1 + (c2*lnr) + (c3*pow(lnr,3));
        return (1/Tinv) - 273.15; // final temperature
    };

    float get_resistance() {
        if (raw_adc_value < 1) {
            return -1; // invalid temperature from a thermistor
        }

        float v = (float)raw_adc_value * kSystemVoltage / (adc_pin.getTop()); // convert the 10 bit ADC value to a voltage
        return (pullup_resistance * v) / (kSystemVoltage - v);   // resistance of thermistor
    }

//    New-JSON style structes. DISABLED FOR NOW.
//    struct ResistanceProperty_t {
//        type &parent;
//
//        operator float() {
//            return parent.resistance();
//        };
//
//        void operator=(float) {;};
//    };
//    ResistanceProperty_t resistance_property {*this};
//
//    auto json_bindings(const char *object_name) {
//        return JSON::bind_object(object_name,
//                                 JSON::bind("temp", *this, /*print precision:*/2),
//                                 JSON::bind_typed<float>("res", resistance_property, /*print precision:*/2)
//                                 );
//    }
//    operator float() {
//        return temperature_exact();
//    };
//
//    void operator=(float) {;};

    // Call back function from the ADC to tell it that the ADC has a new sample...
    void adc_has_new_value() {
        raw_adc_value = (adc_pin.getRaw() + (9 * raw_adc_value))/10;
    };
};



Thermistor<kADC1_PinNumber> thermistor1 {
    /*T1:*/    25, /*T2:*/  160, /*T3:*/ 235,
    /*R1:*/ 86500, /*R2:*/ 800, /*R3:*/ 190, /*pullup_resistance:*/ 4700};
void ADCPin<kADC1_PinNumber>::interrupt() {
    thermistor1.adc_has_new_value();
};



// Output 1 FET info
const int16_t fet_pin1_freq = 100;
PWMOutputPin<kOutput1_PinNumber> fet_pin1;// {kPWMPinInverted};

PWMOutputPin<kOutput3_PinNumber> fan_pin1;
//PWMOutputPin<kOutput4_PinNumber> fan_pin2;

// We're going to utilize the PWMOutputPin<>'s timer interrupt to drive the ADC sampling.
const int16_t fet_pin1_sample_freq = 1; // every fet_pin1_sample_freq interrupts, sample
int16_t fet_pin1_sample_counter = fet_pin1_sample_freq;
namespace Motate {
    template<>
    void PWMOutputPin<kOutput1_PinNumber>::parentTimerType::interrupt() {
        if (!--fet_pin1_sample_counter) {
            ADC_Module::startSampling();
            fet_pin1_sample_counter = fet_pin1_sample_freq;
        }
    };
}

struct PID {
    static constexpr float output_max = 1.0;
    static constexpr float derivative_contribution = 0.05;

    float _p_factor; // the scale for P values
    float _i_factor; // the scale for I values
    float _d_factor; // the scale for D values

    float _proportional = 0.0;   // _proportional storage
    float _integral = 0.0;       // _integral storage
    float _derivative = 0.0;     // _derivative storage
    float _previous_input = 0.0; // _derivative storage

    float _set_point;

    constexpr PID(float P, float I, float D, float startSetPoint = 0.0) : _p_factor{P}, _i_factor{I}, _d_factor{D}, _set_point{startSetPoint} {};

    float getNewOutput(float input) {
        // Calculate the e (error)
        float e = _set_point - input;

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

        return std::min(output_max, p + i - _derivative);
    };

// //New-style JSON bindins. DISABLED FOR NOW.
//    auto json_bindings(const char *object_name) {
//        return JSON::bind_object(object_name,
//                                 JSON::bind("set", _set_point,    /*print precision:*/2),
//                                 JSON::bind("p",   _proportional, /*print precision:*/2),
//                                 JSON::bind("i",   _integral,     /*print precision:*/5),
//                                 JSON::bind("d",   _derivative,   /*print precision:*/5)
//                                 );
//    }
};

PID pid1 { 22.2/255.0, 1.08/255.0, 114.0/255.0}; // default values
Timeout pid_timeout;

/**** Static functions ****/


/*
 * temperature_init()
 * temperature_init() - stop spindle, set speed to zero, and reset values
 */
void temperature_init()
{
    // setup heater PWM
    fet_pin1.setFrequency(fet_pin1_freq);
    fet_pin1.setInterrupts(kInterruptOnOverflow|kInterruptPriorityLow);

    fan_pin1 = 0;
//    fan_pin2 = 1;

    temperature_reset();
}

void temperature_reset()
{
    // make setpoint 0
    fet_pin1 = 0.0f;
    pid1._set_point = 0.0;
    
    pid_timeout.set(100);
}

stat_t temperature_callback()
{
    if (pid_timeout.isPast()) {
        fet_pin1 = pid1.getNewOutput(thermistor1.temperature_exact());
        pid_timeout.set(100);
    }
    return (STAT_OK);
}

stat_t cm_set_temperature_setpoint(float temperature)
{
    pid1._set_point = 0.0;
    return (STAT_OK);
}


/********************************
 * END OF TEMPERATURE FUNCTIONS *
 ********************************/

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/*
 * cm_get_heater_p()/cm_set_heater_p() - get/set the P parameter of the PID
 */
stat_t cm_get_heater_p(nvObj_t *nv)
{
    nv->value = pid1._p_factor;
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}
stat_t cm_set_heater_p(nvObj_t *nv)
{
    pid1._p_factor = nv->value;
    return (STAT_OK);
}

/*
 * cm_get_heater_i()/cm_set_heater_i() - get/set the I parameter of the PID
 */
stat_t cm_get_heater_i(nvObj_t *nv)
{
    nv->value = pid1._i_factor;
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}
stat_t cm_set_heater_i(nvObj_t *nv)
{
    pid1._i_factor = nv->value;
    return (STAT_OK);
}

/*
 * cm_get_heater_d()/cm_set_heater_d() - get/set the D parameter of the PID
 */
stat_t cm_get_heater_d(nvObj_t *nv)
{
    nv->value = pid1._d_factor;
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}
stat_t cm_set_heater_d(nvObj_t *nv)
{
    pid1._d_factor = nv->value;
    return (STAT_OK);
}

/*
 * cm_get_pid_p() - get the active P of the PID (read-only)
 */
stat_t cm_get_pid_p(nvObj_t *nv)
{
    nv->value = pid1._proportional;
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}

/*
 * cm_get_pid_i() - get the active I of the PID (read-only)
 */
stat_t cm_get_pid_i(nvObj_t *nv)
{
    nv->value = pid1._integral;
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}

/*
 * cm_get_pid_d() - get the active D of the PID (read-only)
 */
stat_t cm_get_pid_d(nvObj_t *nv)
{
    nv->value = pid1._derivative;
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}

/*
 * cm_get_set_temperature()/cm_set_set_temperature() - get/set the set value of the PID
 */
stat_t cm_get_set_temperature(nvObj_t *nv)
{
    nv->value = pid1._set_point;
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}
stat_t cm_set_set_temperature(nvObj_t *nv)
{
    pid1._set_point = nv->value;
    return (STAT_OK);
}


/*
 * cm_get_heater_output() - get the output value (PWM duty cycle) of the PID
 */
stat_t cm_get_heater_output(nvObj_t *nv)
{
    nv->value = (float)fet_pin1;
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}

/*
 * cm_get_temperature() - get the current temperature
 */
stat_t cm_get_temperature(nvObj_t *nv)
{
    nv->value = thermistor1.temperature_exact();
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}

/*
 * cm_get_thermocouple_resistance() - get the current temperature
 */
stat_t cm_get_thermocouple_resistance(nvObj_t *nv)
{
    nv->value = thermistor1.get_resistance();
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;

    return (STAT_OK);
}



/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

//const char fmt_spep[] PROGMEM = "[spep] spindle enable polarity%5d [0=active_low,1=active_high]\n";
//const char fmt_spdp[] PROGMEM = "[spdp] spindle direction polarity%2d [0=CW_low,1=CW_high]\n";
//const char fmt_spph[] PROGMEM = "[spph] spindle pause on hold%7d [0=no,1=pause_on_hold]\n";
//const char fmt_spdw[] PROGMEM = "[spdw] spindle dwell time%12.1f seconds\n";
//const char fmt_ssoe[] PROGMEM ="[ssoe] spindle speed override ena%2d [0=disable,1=enable]\n";
//const char fmt_sso[] PROGMEM ="[sso] spindle speed override%11.3f [0.050 < sso < 2.000]\n";
//const char fmt_spe[] PROGMEM = "Spindle Enable:%7d [0=OFF,1=ON,2=PAUSE]\n";
//const char fmt_spd[] PROGMEM = "Spindle Direction:%4d [0=CW,1=CCW]\n";
//const char fmt_sps[] PROGMEM = "Spindle Speed: %7.0f rpm\n";
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