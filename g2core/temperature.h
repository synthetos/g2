/*
 * temperature.h - temperature control module - drives heaters or coolers
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

#ifndef TEMPERATURE_H_ONCE
#define TEMPERATURE_H_ONCE

/*
 * Global Scope Functions
 */

void   temperature_init();
void   temperature_reset();
stat_t temperature_callback();

stat_t cm_get_heater_enable(nvObj_t* nv);
stat_t cm_set_heater_enable(nvObj_t* nv);

stat_t cm_get_heater_p(nvObj_t* nv);
stat_t cm_set_heater_p(nvObj_t* nv);
stat_t cm_get_heater_i(nvObj_t* nv);
stat_t cm_set_heater_i(nvObj_t* nv);
stat_t cm_get_heater_d(nvObj_t* nv);
stat_t cm_set_heater_d(nvObj_t* nv);
stat_t cm_get_pid_p(nvObj_t* nv);
stat_t cm_get_pid_i(nvObj_t* nv);
stat_t cm_get_pid_d(nvObj_t* nv);

float cm_get_set_temperature(const uint8_t heater);
stat_t cm_get_set_temperature(nvObj_t* nv);

void cm_set_set_temperature(const uint8_t heater, const float value);
stat_t cm_set_set_temperature(nvObj_t* nv);

float cm_get_fan_power(const uint8_t heater);
stat_t cm_get_fan_power(nvObj_t* nv);

void cm_set_fan_power(const uint8_t heater, const float value);
stat_t cm_set_fan_power(nvObj_t* nv);

stat_t cm_get_fan_min_power(nvObj_t* nv);
stat_t cm_set_fan_min_power(nvObj_t* nv);
stat_t cm_get_fan_low_temp(nvObj_t* nv);
stat_t cm_set_fan_low_temp(nvObj_t* nv);
stat_t cm_get_fan_high_temp(nvObj_t* nv);
stat_t cm_set_fan_high_temp(nvObj_t* nv);

bool cm_get_at_temperature(const uint8_t heater);
stat_t cm_get_at_temperature(nvObj_t* nv);

float cm_get_heater_output(const uint8_t heater);
stat_t cm_get_heater_output(nvObj_t* nv);

stat_t cm_get_heater_adc(nvObj_t* nv);

float cm_get_temperature(const uint8_t heater);
stat_t cm_get_temperature(nvObj_t* nv);

stat_t cm_get_thermistor_resistance(nvObj_t* nv);


/*--- text_mode support functions ---*/

#ifdef __TEXT_MODE

#else

#endif  // __TEXT_MODE

#endif  // End of include guard: TEMPERATURE_H_ONCE
