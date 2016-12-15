/*
 * pwm.cpp - pulse width modulation drivers
 * This file is part of the g2core project
 *
 * Copyright (c) 2012 - 2016 Alden S. Hart, Jr.
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

#include "g2core.h"  // #1
#include "config.h"  // #2
#include "hardware.h"
#include "spindle.h"
#include "text_parser.h"
#include "pwm.h"
#include "canonical_machine.h"
#include "settings.h"

/***** PWM defines, structures and memory allocation *****/

pwmSingleton_t pwm;


// Setup motate PWM pins
Motate::PWMOutputPin<Motate::kSpindle_PwmPinNumber>  spindle_pwm_pin {Motate::kNormal, P1_PWM_FREQUENCY};
Motate::PWMOutputPin<Motate::kSpindle_Pwm2PinNumber> secondary_pwm_pin {Motate::kNormal, P1_PWM_FREQUENCY}; // assume the same frequency, to start with at least

/***** PWM code *****/
/*
 * pwm_init() - initialize pwm channels
 *
 *  Notes:
 *    - Whatever level interrupts you use must be enabled in main()
 *    - init assumes PWM1 output bit (D5) has been set to output previously (stepper.c)
 *    - See system.h for timer and port assignments
 *  - Don't do this: memset(&TIMER_PWM1, 0, sizeof(PWM_TIMER_t)); // zero out the timer registers
 */
void pwm_init() {}

/*
 * pwm_set_freq() - set PWM channel frequency
 *
 *  channel - PWM channel
 *  freq    - PWM frequency in Khz as a float
 *
 *  Assumes 32MHz clock.
 *  Doesn't turn time on until duty cycle is set
 */

stat_t pwm_set_freq(uint8_t chan, float freq)
{
    if (chan > PWMS) { return (STAT_NO_SUCH_DEVICE);}
    //if (freq < PWM_MIN_FREQ) { return (STAT_INPUT_LESS_THAN_MIN_VALUE);}
    //if (freq > PWM_MAX_FREQ) { return (STAT_INPUT_EXCEEDS_MAX_VALUE);}

    if (chan == PWM_1) {
        spindle_pwm_pin.setFrequency(freq);
    } else if (chan == PWM_2) {
        secondary_pwm_pin.setFrequency(freq);
    }

    return (STAT_OK);
}

/*
 * pwm_set_duty() - set PWM channel duty cycle
 *
 *  channel - PWM channel
 *  duty    - PWM duty cycle from 0% to 100%
 *
 *  Setting duty cycle to 0 disables the PWM channel with output low
 *  Setting duty cycle to 100 disables the PWM channel with output high
 *  Setting duty cycle between 0 and 100 enables PWM channel
 *
 *  The frequency must have been set previously
 */

stat_t pwm_set_duty(uint8_t chan, float duty)
{
    if (duty < 0.0) { return (STAT_INPUT_LESS_THAN_MIN_VALUE);}
    if (duty > 1.0) { return (STAT_INPUT_EXCEEDS_MAX_VALUE);}

    if (chan == PWM_1) {
        if (spindle_pwm_pin.isNull()) {
            cm_alarm(STAT_ALARM, "attempt to turn on a non-existant spindle");
        }
        spindle_pwm_pin = duty;
    } else if (chan == PWM_2) {
        if (secondary_pwm_pin.isNull()) {
            cm_alarm(STAT_ALARM, "attempt to turn on a non-existant spindle");
        }
        secondary_pwm_pin = duty;
    }

    return (STAT_OK);
}


/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/
/*
 * pwm_set_pwm() - set generic PWM parameter and reset PWM channels
 *
 * See config_app.cpp PWM settings for details of what paramters call this function
 */
stat_t pwm_set_pwm(nvObj_t *nv)
{
    set_flt(nv);
    spindle_init();
    return(STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

static const char fmt_p1frq[] = "[p1frq] pwm frequency%18.0f Hz\n";
static const char fmt_p1csl[] = "[p1csl] pwm cw speed lo%16.0f RPM\n";
static const char fmt_p1csh[] = "[p1csh] pwm cw speed hi%16.0f RPM\n";
static const char fmt_p1cpl[] = "[p1cpl] pwm cw phase lo%16.3f [0..1]\n";
static const char fmt_p1cph[] = "[p1cph] pwm cw phase hi%16.3f [0..1]\n";
static const char fmt_p1wsl[] = "[p1wsl] pwm ccw speed lo%15.0f RPM\n";
static const char fmt_p1wsh[] = "[p1wsh] pwm ccw speed hi%15.0f RPM\n";
static const char fmt_p1wpl[] = "[p1wpl] pwm ccw phase lo%15.3f [0..1]\n";
static const char fmt_p1wph[] = "[p1wph] pwm ccw phase hi%15.3f [0..1]\n";
static const char fmt_p1pof[] = "[p1pof] pwm phase off%18.3f [0..1]\n";

void pwm_print_p1frq(nvObj_t *nv) { text_print(nv, fmt_p1frq);}     // all TYPE_FLOAT
void pwm_print_p1csl(nvObj_t *nv) { text_print(nv, fmt_p1csl);}
void pwm_print_p1csh(nvObj_t *nv) { text_print(nv, fmt_p1csh);}
void pwm_print_p1cpl(nvObj_t *nv) { text_print(nv, fmt_p1cpl);}
void pwm_print_p1cph(nvObj_t *nv) { text_print(nv, fmt_p1cph);}
void pwm_print_p1wsl(nvObj_t *nv) { text_print(nv, fmt_p1wsl);}
void pwm_print_p1wsh(nvObj_t *nv) { text_print(nv, fmt_p1wsh);}
void pwm_print_p1wpl(nvObj_t *nv) { text_print(nv, fmt_p1wpl);}
void pwm_print_p1wph(nvObj_t *nv) { text_print(nv, fmt_p1wph);}
void pwm_print_p1pof(nvObj_t *nv) { text_print(nv, fmt_p1pof);}

#endif //__TEXT_MODE
