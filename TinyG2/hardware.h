/*
 * hardware.h - system hardware configuration
 *				THIS FILE IS HARDWARE PLATFORM SPECIFIC - ARM version
 *
 * This file is part of the TinyG project
 *
 * Copyright (c) 2013 - 2014 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2014 Robert Giseburt
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
#ifndef HARDWARE_H_ONCE
#define HARDWARE_H_ONCE

/*--- Hardware platform enumerations ---*/

enum hwPlatform {
	HM_PLATFORM_NONE = 0,

	HW_PLATFORM_TINYG_XMEGA,	// TinyG code base on Xmega boards.
								//	hwVersion 7 = TinyG v7 and earlier
								//	hwVersion 8 = TinyG v8

	HW_PLATFORM_G2_DUE,			// G2 code base on native Arduino Due

	HW_PLATFORM_TINYG_V9		// G2 code base on v9 boards
								//  hwVersion 0 = v9c
								//  hwVersion 1 = v9d
								//  hwVersion 2 = v9f
								//  hwVersion 3 = v9h
								//  hwVersion 4 = v9i
};

#define HW_VERSION_TINYGV6		6
#define HW_VERSION_TINYGV7		7
#define HW_VERSION_TINYGV8		8

#define HW_VERSION_TINYGV9C		0
#define HW_VERSION_TINYGV9D		1
#define HW_VERSION_TINYGV9F		2
#define HW_VERSION_TINYGV9H		3
#define HW_VERSION_TINYGV9I		4
#define HW_VERSION_TINYGV9K		5

////////////////////////////
/////// ARM VERSION ////////
////////////////////////////

// ARM specific code start here

#include "MotatePins.h"
#include "MotateTimers.h" // for timer_number

using namespace Motate;

#ifdef __cplusplus
extern "C"{
#endif

/*************************
 * Global System Defines *
 *************************/

#undef F_CPU							// CPU clock - set for delays
#define F_CPU 84000000UL
#define MILLISECONDS_PER_TICK 1			// MS for system tick (systick * N)
#define SYS_ID_DIGITS 12                // actual digits in system ID (up to 16)
#define SYS_ID_LEN 16					// total length including dashes and NUL

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

#define FREQUENCY_DDA		200000.0		// Hz step frequency. Interrupts actually fire at 2x (400 KHz)
#define FREQUENCY_DWELL		1000UL
#define FREQUENCY_SGI		200000UL		// 200,000 Hz means software interrupts will fire 5 uSec after being called

/**** Motate Definitions ****/

// Timer definitions. See stepper.h and other headers for setup

timer_number dda_timer_num   = 2;	// stepper pulse generation in stepper.cpp
timer_number dwell_timer_num = 3;	// dwell timing in stepper.cpp
timer_number load_timer_num  = 4;	// request load timer in stepper.cpp
timer_number exec_timer_num  = 5;	// request exec timer in stepper.cpp

// Pin assignments

pin_number indicator_led_pin_num = kLED_USBRXPinNumber;
static PWMOutputPin<indicator_led_pin_num> IndicatorLed;

// Init these to input to keep them high-z
static Pin<kSPI0_MISOPinNumber> spi_miso_pin(kInput);
static Pin<kSPI0_MOSIPinNumber> spi_mosi_pin(kInput);
static Pin<kSPI0_SCKPinNumber>  spi_sck_pin(kInput);

/**** Motate Global Pin Allocations ****/

//static OutputPin<kSocket1_SPISlaveSelectPinNumber> spi_ss1_pin;
//static OutputPin<kSocket2_SPISlaveSelectPinNumber> spi_ss2_pin;
//static OutputPin<kSocket3_SPISlaveSelectPinNumber> spi_ss3_pin;
//static OutputPin<kSocket4_SPISlaveSelectPinNumber> spi_ss4_pin;
//static OutputPin<kSocket5_SPISlaveSelectPinNumber> spi_ss5_pin;
//static OutputPin<kSocket6_SPISlaveSelectPinNumber> spi_ss6_pin;
static OutputPin<kKinen_SyncPinNumber> kinen_sync_pin;

static OutputPin<kGRBL_ResetPinNumber> grbl_reset_pin;
static OutputPin<kGRBL_FeedHoldPinNumber> grbl_feedhold_pin;
static OutputPin<kGRBL_CycleStartPinNumber> grbl_cycle_start_pin;

static OutputPin<kGRBL_CommonEnablePinNumber> motor_common_enable_pin;
static OutputPin<kSpindle_EnablePinNumber> spindle_enable_pin;
static OutputPin<kSpindle_DirPinNumber> spindle_dir_pin;
static PWMOutputPin<kSpindle_PwmPinNumber> spindle_pwm_pin;
static PWMOutputPin<kSpindle_Pwm2PinNumber> secondary_pwm_pin;

// NOTE: In the v9 and the Due the flood and mist coolants are mapped to a the same pin
//static OutputPin<kCoolant_EnablePinNumber> coolant_enable_pin;
static OutputPin<kCoolant_EnablePinNumber> flood_enable_pin;
static OutputPin<kCoolant_EnablePinNumber> mist_enable_pin;

// Input pins are defined in switch.cpp

/********************************
 * Function Prototypes (Common) *
 ********************************/

void hardware_init(void);			// master hardware init
void hw_hard_reset(void);
stat_t hw_flash(nvObj_t *nv);

stat_t hw_get_fbs(nvObj_t *nv);
stat_t hw_set_hv(nvObj_t *nv);
stat_t hw_get_id(nvObj_t *nv);

#ifdef __TEXT_MODE

	void hw_print_fb(nvObj_t *nv);
    void hw_print_fbs(nvObj_t *nv);
	void hw_print_fv(nvObj_t *nv);
	void hw_print_cv(nvObj_t *nv);
	void hw_print_hp(nvObj_t *nv);
	void hw_print_hv(nvObj_t *nv);
	void hw_print_id(nvObj_t *nv);

#else

	#define hw_print_fb tx_print_stub
    #define hw_print_fbs tx_print_stub
	#define hw_print_fv tx_print_stub
	#define hw_print_cv tx_print_stub
	#define hw_print_hp tx_print_stub
	#define hw_print_hv tx_print_stub
	#define hw_print_id tx_print_stub

#endif // __TEXT_MODE

#ifdef __cplusplus
}
#endif

#endif	// end of include guard: HARDWARE_H_ONCE
