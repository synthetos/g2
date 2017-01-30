/*
 * hardware.h - system hardware configuration
 *				THIS FILE IS HARDWARE PLATFORM SPECIFIC - ARM version
 *
 * This file is part of the g2core project
 *
 * Copyright (c) 2013 - 2016 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2016 Robert Giseburt
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
#include "error.h"

#ifndef HARDWARE_H_ONCE
#define HARDWARE_H_ONCE

/*--- Hardware platform enumerations ---*/

enum hwPlatform {
    HM_PLATFORM_NONE = 0,
    HW_PLATFORM_TINYG_XMEGA,    // TinyG code base on Xmega boards.
    HW_PLATFORM_G2_DUE,         // G2 code base on native Arduino Due
    HW_PLATFORM_V9              // G2 code base on v9 boards
};

#define HW_VERSION_TINYGV6		6
#define HW_VERSION_TINYGV7		7
#define HW_VERSION_TINYGV8		8

#define HW_VERSION_TINYGV9I		4
#define HW_VERSION_TINYGV9K		5


/***** Axes, motors & PWM channels used by the application *****/
// Axes, motors & PWM channels must be defines (not enums) so expressions like this:
//  #if (MOTORS >= 6)  will work

#define AXES 6          // number of axes supported in this version
#define HOMING_AXES 4   // number of axes that can be homed (assumes Zxyabc sequence)
#define MOTORS 4        // number of motors on the board
#define COORDS 6        // number of supported coordinate systems (index starts at 1)
#define PWMS 2          // number of supported PWM channels
#define TOOLS 32        // number of entries in tool table (index starts at 1)

////////////////////////////
/////// ARM SETTINGS ///////
////////////////////////////

#include "MotatePins.h"
#include "MotateTimers.h" // for TimerChanel<> and related...
#include "MotateServiceCall.h" // for ServiceCall<>

using Motate::TimerChannel;
using Motate::ServiceCall;

using Motate::pin_number;
using Motate::Pin;
using Motate::PWMOutputPin;
using Motate::OutputPin;

/*************************
 * Global System Defines *
 *************************/

#define MILLISECONDS_PER_TICK 1			// MS for system tick (systick * N)
#define SYS_ID_DIGITS 12                // actual digits in system ID (up to 16)
#define SYS_ID_LEN 24					// total length including dashes and NUL

/************************************************************************************
 **** ARM SAM3X8E SPECIFIC HARDWARE *************************************************
 ************************************************************************************/

/**** Resource Assignment via Motate ****
 *
 * This section defines resource usage for pins, timers, PWM channels, communications
 * and other resources. Please refer to /motate/utility/SamPins.h, SamTimers.h and
 * other files for pinouts and other configuration details.
 *
 * Commenting out or #ifdef'ing out definitions below will cause the compiler to
 * drop references to these resources from the compiled code. This will reduce
 * compiled code size and runtime CPU cycles. E.g. if you are compiling for a 3 motor,
 * XYZ axis config commenting out the higher motors and axes here will remove them
 * from later code (using the motate .isNull() test).
 */

/* Interrupt usage and priority
 *
 * The following interrupts are defined w/indicated priorities
 *
 *	 0	DDA_TIMER (3) for step pulse generation
 *	 1	DWELL_TIMER (4) for dwell timing
 *	 2	LOADER software generated interrupt (STIR / SGI)
 *	 3	Serial read character interrupt
 *	 4	EXEC software generated interrupt (STIR / SGI)
 *	 5	Serial write character interrupt
 */

/**** Stepper DDA and dwell timer settings ****/

//#define FREQUENCY_DDA		200000UL		// Hz step frequency. Interrupts actually fire at 2x (400 KHz)
#define FREQUENCY_DDA		150000UL		// Hz step frequency. Interrupts actually fire at 2x (300 KHz)
#define FREQUENCY_DWELL		1000UL
#define FREQUENCY_SGI		200000UL		// 200,000 Hz means software interrupts will fire 5 uSec after being called

/**** Motate Definitions ****/

// Timer definitions. See stepper.h and other headers for setup
typedef TimerChannel<3,0> dda_timer_type;	// stepper pulse generation in stepper.cpp
typedef TimerChannel<4,0> exec_timer_type;	// request exec timer in stepper.cpp
typedef TimerChannel<5,0> fwd_plan_timer_type;	// request exec timer in stepper.cpp

// Pin assignments

pin_number indicator_led_pin_num = Motate::kLED_USBRXPinNumber;
static PWMOutputPin<indicator_led_pin_num> IndicatorLed;

// Init these to input to keep them high-z
static Pin<Motate::kSPI0_MISOPinNumber> spi_miso_pin(Motate::kInput);
static Pin<Motate::kSPI0_MOSIPinNumber> spi_mosi_pin(Motate::kInput);
static Pin<Motate::kSPI0_SCKPinNumber>  spi_sck_pin(Motate::kInput);

/**** Motate Global Pin Allocations ****/

//static OutputPin<kSocket1_SPISlaveSelectPinNumber> spi_ss1_pin;
//static OutputPin<kSocket2_SPISlaveSelectPinNumber> spi_ss2_pin;
//static OutputPin<kSocket3_SPISlaveSelectPinNumber> spi_ss3_pin;
//static OutputPin<kSocket4_SPISlaveSelectPinNumber> spi_ss4_pin;
//static OutputPin<kSocket5_SPISlaveSelectPinNumber> spi_ss5_pin;
//static OutputPin<kSocket6_SPISlaveSelectPinNumber> spi_ss6_pin;
static OutputPin<Motate::kKinen_SyncPinNumber> kinen_sync_pin;

static OutputPin<Motate::kGRBL_ResetPinNumber> grbl_reset_pin;
static OutputPin<Motate::kGRBL_FeedHoldPinNumber> grbl_feedhold_pin;
static OutputPin<Motate::kGRBL_CycleStartPinNumber> grbl_cycle_start_pin;

static OutputPin<Motate::kGRBL_CommonEnablePinNumber> motor_common_enable_pin;
static OutputPin<Motate::kSpindle_EnablePinNumber> spindle_enable_pin;
static OutputPin<Motate::kSpindle_DirPinNumber> spindle_dir_pin;

// NOTE: In the v9 and the Due the flood and mist coolants are mapped to a the same pin
//static OutputPin<kCoolant_EnablePinNumber> coolant_enable_pin;
static OutputPin<Motate::kCoolant_EnablePinNumber> flood_enable_pin;
static OutputPin<Motate::kCoolant_EnablePinNumber> mist_enable_pin;

// Input pins are defined in gpio.cpp

/********************************
 * Function Prototypes (Common) *
 ********************************/

void hardware_init(void);			// master hardware init
stat_t hardware_periodic();  // callback from the main loop (time sensitive)
void hw_hard_reset(void);
stat_t hw_flash(nvObj_t *nv);

stat_t hw_get_fbs(nvObj_t *nv);
stat_t hw_get_fbc(nvObj_t *nv);
stat_t hw_set_hv(nvObj_t *nv);
stat_t hw_get_id(nvObj_t *nv);

#ifdef __TEXT_MODE

    void hw_print_fb(nvObj_t *nv);
    void hw_print_fbs(nvObj_t *nv);
    void hw_print_fbc(nvObj_t *nv);
    void hw_print_fv(nvObj_t *nv);
    void hw_print_cv(nvObj_t *nv);
    void hw_print_hp(nvObj_t *nv);
    void hw_print_hv(nvObj_t *nv);
    void hw_print_id(nvObj_t *nv);

#else

    #define hw_print_fb tx_print_stub
    #define hw_print_fbs tx_print_stub
    #define hw_print_fbc tx_print_stub
    #define hw_print_fv tx_print_stub
    #define hw_print_cv tx_print_stub
    #define hw_print_hp tx_print_stub
    #define hw_print_hv tx_print_stub
    #define hw_print_id tx_print_stub

#endif // __TEXT_MODE

#endif	// end of include guard: HARDWARE_H_ONCE
