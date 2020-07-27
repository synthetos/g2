/*
 * hardware.h - system hardware configuration
 * For: /board/gQuintic
 * THIS FILE IS HARDWARE PLATFORM SPECIFIC - ARM version
 *
 * This file is part of the g2core project
 *
 * Copyright (c) 2013 - 2019 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2019 Robert Giseburt
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

#include "MotatePins.h" // QUINTIC_REVISION comes from the pintout file


#ifndef HARDWARE_H_ONCE
#define HARDWARE_H_ONCE

/*--- Hardware platform enumerations ---*/

#define G2CORE_HARDWARE_PLATFORM    "gQuintic"
#define G2CORE_HARDWARE_VERSION_from_QUINTIC_REVISION(x)  "" #x ""
#define G2CORE_HARDWARE_VERSION G2CORE_HARDWARE_VERSION_from_QUINTIC_REVISION(QUINTIC_REVISION)


// Save some space ... we're tight
#define TOOLS 5        // number of entries in tool table (index starts at 1)


/***** Motors & PWM channels supported by this hardware *****/
// These must be defines (not enums) so expressions like this:
//  #if (MOTORS >= 6)  will work

#ifndef HAS_HOBBY_SERVO_MOTOR
#define HAS_HOBBY_SERVO_MOTOR 0
#endif

#ifndef HAS_PRESSURE
#define HAS_PRESSURE 0
#endif

#ifndef HAS_LASER
#define HAS_LASER 0
#else
#if HAS_HOBBY_SERVO_MOTOR && HAS_LASER
#error Can NOT have a laser and a hobby servo at the same time, sorry
#endif
#endif


#if QUINTIC_REVISION == 'C' or (!HAS_HOBBY_SERVO_MOTOR && !HAS_LASER)
#define MOTORS      5               // number of motors on the board - 5Trinamics OR 4 Trinamics + 1 servo
#else
#define MOTORS      6               // number of motors on the board - 5 Trinamics + 1 servo or laser
#endif

#define PWMS 2                      // number of PWM channels supported the hardware
#define AXES 6                      // axes to support -- must be 6 or 9

#define MOTOR_1_IS_TRINAMIC
#define MOTOR_2_IS_TRINAMIC
#define MOTOR_3_IS_TRINAMIC
#define MOTOR_4_IS_TRINAMIC
#if QUINTIC_REVISION == 'D'
#define MOTOR_5_IS_TRINAMIC
#endif

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
#include "MotateSPI.h"
#include "MotateTWI.h"
#include "MotateTimers.h"           // for TimerChanel<> and related...

// Temporarily disabled:
// #include "i2c_multiplexer.h"
// #include "i2c_as5601.h" // For AS5601

using Motate::TimerChannel;

using Motate::pin_number;
using Motate::Pin;
using Motate::PWMOutputPin;
using Motate::OutputPin;

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
 *   0  DDA_TIMER (3) for step pulse generation
 *   1  DWELL_TIMER (4) for dwell timing
 *   2  LOADER software generated interrupt (STIR / SGI)
 *   3  Serial read character interrupt
 *   4  EXEC software generated interrupt (STIR / SGI)
 *   5  Serial write character interrupt
 */

/**** Stepper DDA and dwell timer settings ****/

// #define FREQUENCY_DDA    200000UL    // Hz step frequency. Interrupts actually fire at 2x (400 KHz)
#define FREQUENCY_DDA  400000UL  // Hz step frequency. Interrupts actually fire at 2x (300 KHz)
#define FREQUENCY_DWELL  1000UL

// #define MIN_SEGMENT_MS ((float)0.125)       // S70 can handle much much smaller segements
#define MIN_SEGMENT_MS ((float)0.5)       // S70 can handle much much smaller segements

// #define PLANNER_QUEUE_SIZE (60)

/**** Motate Definitions ****/

// Timer definitions. See stepper.h and other headers for setup
typedef TimerChannel<9, 0> dda_timer_type;    // stepper pulse generation in stepper.cpp
typedef TimerChannel<10, 0> exec_timer_type;       // request exec timer in stepper.cpp
typedef TimerChannel<11, 0> fwd_plan_timer_type;   // request forward planner in stepper.cpp

/**** SPI Setup ****/
typedef Motate::SPIBus<Motate::kSPI_MISOPinNumber, Motate::kSPI_MOSIPinNumber, Motate::kSPI_SCKPinNumber> SPIBus_used_t;
extern SPIBus_used_t spiBus;

typedef Motate::SPIChipSelectPinMux<Motate::kSocket1_SPISlaveSelectPinNumber, Motate::kSocket2_SPISlaveSelectPinNumber, Motate::kSocket3_SPISlaveSelectPinNumber, Motate::kSocket4_SPISlaveSelectPinNumber> SPI_CS_PinMux_used_t;
extern SPI_CS_PinMux_used_t spiCSPinMux;

/**** TWI Setup ****/
typedef Motate::TWIBus<Motate::kI2C_SCLPinNumber, Motate::kI2C_SDAPinNumber> TWIBus_used_t;
extern TWIBus_used_t twiBus;

// using plex0_t = decltype(I2C_Multiplexer{twiBus, 0x0070L});
// extern HOT_DATA plex0_t plex0;
// using plex1_t = decltype(I2C_Multiplexer{twiBus, 0x0071L});
// extern HOT_DATA plex1_t plex1;

// extern HOT_DATA I2C_EEPROM eeprom;

/**** Motate Global Pin Allocations ****/

pin_number indicator_led_pin_num = Motate::kLED_USBRXPinNumber;
static OutputPin<indicator_led_pin_num> IndicatorLed;

static OutputPin<Motate::kKinen_SyncPinNumber> kinen_sync_pin;

static OutputPin<Motate::kGRBL_ResetPinNumber> grbl_reset_pin;
static OutputPin<Motate::kGRBL_FeedHoldPinNumber> grbl_feedhold_pin;
static OutputPin<Motate::kGRBL_CycleStartPinNumber> grbl_cycle_start_pin;

static OutputPin<Motate::kGRBL_CommonEnablePinNumber> motor_common_enable_pin;
//static OutputPin<Motate::kSpindle_EnablePinNumber> spindle_enable_pin;
//static OutputPin<Motate::kSpindle_DirPinNumber> spindle_dir_pin;

// Input pins are defined in gpio.cpp


/********************************
 * Function Prototypes (Common) *
 ********************************/

const configSubtable *const getSysConfig_3();

void hardware_init(void);      // master hardware init
stat_t hardware_periodic();  // callback from the main loop (time sensitive)
void hw_hard_reset(void);
stat_t hw_flash(nvObj_t *nv);

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
