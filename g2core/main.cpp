/*
 * main.cpp - g2core - An embedded rs274/ngc CNC controller
 * This file is part of the g2core project.
 *
 * Copyright (c) 2010 - 2018 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2018 Robert Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* See github.com/Synthetos/G2 for code and docs on the wiki
 */

#include "g2core.h"  // #1 There are some dependencies
#include "config.h"  // #2
#include "hardware.h"
#include "persistence.h"
#include "controller.h"
#include "canonical_machine.h"
#include "json_parser.h"			// required for unit tests only
#include "report.h"
#include "planner.h"
#include "stepper.h"
#include "coolant.h"
#include "encoder.h"
#include "spindle.h"
#include "temperature.h"
#include "gpio.h"
#include "pwm.h"
#include "xio.h"

#include "util.h"
#include "MotateUniqueID.h"

/***** NOTE: *****

   The actual main.cpp for g2core is in the Motate project.
   It calls the setup() and loop() functions in this file

*****/
/******************** System Globals *************************/

stat_t status_code;						    // allocate a variable for the ritorno macro

/************* System Globals For Debugging and Diagnostics ****************/
// See also: util.h for debugging and diagnostics

// Using motate pins for profiling
// Usage: https://github.com/synthetos/g2/wiki/Using-Pin-Changes-for-Timing-(and-light-debugging)OutputPin<Motate::kDebug1_PinNumber> debug_pin1;
OutputPin<Motate::kDebug2_PinNumber> debug_pin2;
OutputPin<Motate::kDebug3_PinNumber> debug_pin3;
OutputPin<Motate::kDebug4_PinNumber> debug_pin4;

// or these to disable the pin
//OutputPin<-1> debug_pin1;
//OutputPin<-1> debug_pin2;
//OutputPin<-1> debug_pin3;
//OutputPin<-1> debug_pin4;

// Put these lines in the .cpp file where you are using the debug pins (appropriately commented / uncommented)
/*extern OutputPin<Motate::kDebug1_PinNumber> debug_pin1;
extern OutputPin<Motate::kDebug2_PinNumber> debug_pin2;
extern OutputPin<Motate::kDebug3_PinNumber> debug_pin3;
extern OutputPin<Motate::kDebug4_PinNumber> debug_pin4;

//extern OutputPin<-1> debug_pin1;
//extern OutputPin<-1> debug_pin2;
//extern OutputPin<-1> debug_pin3;
//extern OutputPin<-1> debug_pin4;
*/

/******************** Application Code ************************/

/*
 * _application_init_services()
 * _application_init_machine()
 * _application_init_startup()
 *
 * There are a lot of dependencies in the order of these inits.
 * Don't change the ordering unless you understand this.
 */

void application_init_services(void)
{
    hardware_init();				    // system hardware setup 			- must be first
    persistence_init();				    // set up EEPROM or other NVM		- must be second
    xio_init();						    // xtended io subsystem				- must be third
}

void application_init_machine(void)
{
    cm = &cm1;                          // set global canonical machine pointer to primary machine
    cm->machine_state = MACHINE_INITIALIZING;
    canonical_machine_inits();          // combined inits for CMs and planner - do before anything might use cm or mr!

    stepper_init();                     // stepper subsystem
    encoder_init();                     // virtual encoders
    gpio_init();                        // inputs and outputs
}

void application_init_startup(void)
{
    // start the application
    controller_init();                  // should be first startup init (requires xio_init())
    config_init();					    // apply the config settings from persistence
    canonical_machine_reset(&cm1);      // initialize both CMs but only reset the primary
    gcode_parser_init();                // baseline Gcode parser
    spindle_init();                     // should be after PWM and canonical machine inits and config_init()
    spindle_reset();
    coolant_init();
    coolant_reset();
    temperature_init();
    gpio_reset();
}

/*
 * get_status_message() - global support for status messages.
 */

char *get_status_message(stat_t status)
{
    return ((char *)GET_TEXT_ITEM(stat_msg, status));
}

/*
 * main() - See Motate main.cpp for the actual main()
 */

void setup(void)
{
    // application setup
    application_init_services();
    while (SysTickTimer.getValue() < 400);  // delay 400 ms for USB to come up

    application_init_machine();
    application_init_startup();
}

void loop() {
    // main loop
    for (;;) {
        controller_run( );			// single pass through the controller
    }
}

/*
 * Traps for debugging. These must be in main.cpp for proper linker ordering
 * WARNING: These are horribly ARM-specific, and should be moved to Motate!
 */

#pragma GCC push_options
#pragma GCC optimize ("O0")

// void MemManage_Handler  ( void ) { __asm__("BKPT"); }
__attribute((naked)) void MemManage_Handler(void){

#ifndef SCB_CFSR_IACCVIOL
#define  SCB_CFSR_IACCVIOL                   ((uint32_t)0x00000001)        /*!< Instruction access violation */
#define  SCB_CFSR_DACCVIOL                   ((uint32_t)0x00000002)        /*!< Data access violation */
#define  SCB_CFSR_MUNSTKERR                  ((uint32_t)0x00000008)        /*!< Unstacking error */
#define  SCB_CFSR_MSTKERR                    ((uint32_t)0x00000010)        /*!< Stacking error */
#define  SCB_CFSR_MLSPERR                    ((uint32_t)0x00000020)        /*!< floating-point lazy state preservation error */
#define  SCB_CFSR_MMARVALID                  ((uint32_t)0x00000080)        /*!< Memory Manage Address Register address valid flag */
#endif

    // Notes for use in a debugger:
    // This is a "naked" function, so no stack can be used.
    // This means that these values below are held in registers.
    // This also means that, in spite of the optimization being disabled here, fault_address will be "optimized out"
    // But that's okay, it lets us keep SCB->MMFAR here where it's handy to view.

    uint32_t fault = (SCB->CFSR >> SCB_CFSR_MEMFAULTSR_Pos) & SCB_CFSR_MEMFAULTSR_Msk;
    if (fault & SCB_CFSR_MMARVALID) {
        // SCB->MMFAR holds the address that was accessed (read or written -- likely written) that caused this fault.
        // The stack trace will likely have some garbage in it, but the last few frames *might* be valid.
        [[maybe_unused]] void *fault_address = (void *)SCB->MMFAR;
    }

    if (fault & SCB_CFSR_IACCVIOL) {
        __asm__ volatile("BKPT 1"); // invalid instruction access
    } else
    if (fault & SCB_CFSR_DACCVIOL) {
        __asm__ volatile("BKPT 2"); // invalid data access
    } else {
        __asm__ volatile("BKPT 3"); // other memory access violation
    }

    // __asm volatile(
    //     " bkpt 10 \n"
    //     " bx lr \n");
}

// void BusFault_Handler   ( void ) { __asm__("BKPT"); }
__attribute((naked)) void BusFault_Handler(void){
__asm volatile (
 " bkpt 10 \n"
 " bx lr \n"
);
}

// void UsageFault_Handler ( void ) { __asm__("BKPT"); }
__attribute((naked)) void UsageFault_Handler(void){
__asm volatile (
 " bkpt 10 \n"
 " bx lr \n"
);
}

// void HardFault_Handler  ( void ) { __asm__("BKPT"); }
__attribute((naked)) void HardFault_Handler(void){
__asm volatile (
 " bkpt 10 \n"
 " bx lr \n"
);
}

#pragma GCC reset_options
