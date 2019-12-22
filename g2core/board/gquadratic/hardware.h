/*
 * hardware.h - system hardware configuration
 * For: /board/gQuadratic
 * THIS FILE IS HARDWARE PLATFORM SPECIFIC - ARM version
 *
 * This file is part of the g2core project
 *
 * Copyright (c) 2013 - 2018 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2018 Robert Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/> .
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

#include "config.h"
#include "settings.h"
#include "error.h"

#ifndef HARDWARE_H_ONCE
#define HARDWARE_H_ONCE


/*--- Hardware platform enumerations ---*/

#define G2CORE_HARDWARE_PLATFORM    "gQuadtratic"
#define G2CORE_HARDWARE_VERSION     "b"

/***** Motors & PWM channels supported by this hardware *****/
// These must be defines (not enums) so expressions like this:
//  #if (MOTORS >= 6)  will work

#define MOTORS 4                    // number of motors supported the hardware
#define PWMS 2                      // number of PWM channels supported the hardware

/*************************
 * Global System Defines *
 *************************/

#define MILLISECONDS_PER_TICK 1     // MS for system tick (systick * N)
#define SYS_ID_DIGITS 16            // actual digits in system ID (up to 16)
#define SYS_ID_LEN 40               // total length including dashes and NUL

/*************************
 * Motate Setup          *
 *************************/

#include "MotatePins.h"
#if QUADRATIC_REVISION == 'C'
#define MOTOR_1_IS_TRINAMIC
#define MOTOR_2_IS_TRINAMIC
#include "MotateSPI.h"
#endif
#include "MotateTimers.h"       // for TimerChanel<> and related...

using Motate::TimerChannel;
using Motate::pin_number;
using Motate::Pin;
using Motate::PWMOutputPin;
using Motate::OutputPin;

/*************************
 * Global System Defines *
 *************************/

#define MILLISECONDS_PER_TICK 1  // MS for system tick (systick * N)
#define SYS_ID_DIGITS 12         // actual digits in system ID (up to 16)
#define SYS_ID_LEN 40            // total length including dashes and NUL

/**** Stepper DDA and dwell timer settings ****/

//#define FREQUENCY_DDA		200000UL		// Hz step frequency. Interrupts actually fire at 2x (400 KHz)
#define FREQUENCY_DDA 400000UL  // Hz step frequency. Interrupts actually fire at 2x (300 KHz)
#define FREQUENCY_DWELL 1000UL

#define MIN_SEGMENT_MS ((float)0.125)       // S70 can handle much much smaller segements

#define PLANNER_QUEUE_SIZE (60)

/**** Motate Definitions ****/

// Timer definitions. See stepper.h and other headers for setup
typedef TimerChannel<9, 0> dda_timer_type;    // stepper pulse generation in stepper.cpp
typedef TimerChannel<10, 0> exec_timer_type;       // request exec timer in stepper.cpp
typedef TimerChannel<11, 0> fwd_plan_timer_type;   // request exec timer in stepper.cpp

// Pin assignments
pin_number                              indicator_led_pin_num = Motate::kLEDPWM_PinNumber;
static OutputPin<indicator_led_pin_num> IndicatorLed;

/**** SPI Setup ****/
#if QUADRATIC_REVISION == 'C'
typedef Motate::SPIBus<Motate::kSPI_MISOPinNumber, Motate::kSPI_MOSIPinNumber> SPIBus_used_t;
extern SPIBus_used_t spiBus;

#endif

/**** Motate Global Pin Allocations ****/

static OutputPin<Motate::kKinen_SyncPinNumber> kinen_sync_pin;

static OutputPin<Motate::kGRBL_ResetPinNumber>      grbl_reset_pin;
static OutputPin<Motate::kGRBL_FeedHoldPinNumber>   grbl_feedhold_pin;
static OutputPin<Motate::kGRBL_CycleStartPinNumber> grbl_cycle_start_pin;

static OutputPin<Motate::kGRBL_CommonEnablePinNumber> motor_common_enable_pin;

#define SPINDLE_OUTPUT_NUMBER 1           // drive our primary output as a spindle
#define SPINDLE_ENABLE_OUTPUT_NUMBER 2    // use output 2 as the enable line for the spindle
#define SPINDLE_DIRECTION_OUTPUT_NUMBER 0 // no direction control
#define MIST_ENABLE_OUTPUT_NUMBER 0 // no mist
#define FLOOD_ENABLE_OUTPUT_NUMBER 0 // no flood
// Input pins are defined in gpio.cpp

/********************************
 * Function Prototypes (Common) *
 ********************************/

void   hardware_init(void);  // master hardware init
stat_t hardware_periodic();  // callback from the main loop (time sensitive)
void   hw_hard_reset(void);
stat_t hw_flash(nvObj_t* nv);

stat_t hw_get_fb(nvObj_t *nv);
stat_t hw_get_fv(nvObj_t *nv);
stat_t hw_get_hp(nvObj_t *nv);
stat_t hw_get_hv(nvObj_t *nv);
stat_t hw_get_fbs(nvObj_t *nv);
stat_t hw_get_fbc(nvObj_t *nv);
stat_t hw_get_id(nvObj_t *nv);

#ifdef __TEXT_MODE

    void hw_print_fb(nvObj_t *nv);
    void hw_print_fv(nvObj_t *nv);
    void hw_print_fbs(nvObj_t *nv);
    void hw_print_fbc(nvObj_t *nv);
    void hw_print_hp(nvObj_t *nv);
    void hw_print_hv(nvObj_t *nv);
    void hw_print_id(nvObj_t *nv);

#else

    #define hw_print_fb tx_print_stub
    #define hw_print_fv tx_print_stub
    #define hw_print_fbs tx_print_stub
    #define hw_print_fbc tx_print_stub
    #define hw_print_hp tx_print_stub
    #define hw_print_hv tx_print_stub
    #define hw_print_id tx_print_stub

#endif // __TEXT_MODE


#endif  // end of include guard: HARDWARE_H_ONCE
