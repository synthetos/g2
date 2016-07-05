/*
 * gpio.cpp - digital IO handling functions
 * This file is part of the TinyG project
 *
 * Copyright (c) 2015 - 2106 Alden S. Hart, Jr.
 * Copyright (c) 2015 - 2016 Robert Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software. If not, see <http://www.gnu.org/licenses/>.
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
/* Switch Modes
 *
 *	The switches are considered to be homing switches when cycle_state is
 *	CYCLE_HOMING. At all other times they are treated as limit switches:
 *	  - Hitting a homing switch puts the current move into feedhold
 *	  - Hitting a limit switch causes the machine to shut down and go into lockdown until reset
 *
 * 	The normally open switch modes (NO) trigger an interrupt on the falling edge
 *	and lockout subsequent interrupts for the defined lockout period. This approach
 *	beats doing debouncing as an integration as switches fire immediately.
 *
 * 	The normally closed switch modes (NC) trigger an interrupt on the rising edge
 *	and lockout subsequent interrupts for the defined lockout period. Ditto on the method.
 */

#include "tinyg2.h"     // #1
#include "config.h"     // #2
#include "gpio.h"
#include "stepper.h"
#include "encoder.h"
#include "hardware.h"
#include "canonical_machine.h"

#include "text_parser.h"
#include "controller.h"
#include "util.h"
#include "report.h"
#include "xio.h"

#ifdef __AVR
    #include <avr/interrupt.h>
#else
    #include "MotateTimers.h"
    using namespace Motate;
#endif

/**** Allocate structures ****/

d_in_t   d_in[D_IN_CHANNELS];
d_out_t  d_out[D_OUT_CHANNELS];
a_in_t   a_in[A_IN_CHANNELS];
a_out_t  a_out[A_OUT_CHANNELS];

/**** Defines and static functions ****/

static bool _read_raw_pin(const uint8_t input_num_ext);
static uint8_t _condition_pin(const uint8_t input_num_ext, const int8_t pin_value);
static void _dispatch_pin(const uint8_t input_num_ext);

/**** Setup Low Level Stuff ****/
// See hardware.h for AVR setup

#ifdef __ARM

// WARNING: These return raw pin values, NOT corrected for NO/NC Active high/low
//          Also, these take EXTERNAL pin numbers -- 1-based
/* Priority only needs set once in the system during startup.
 * However, if we wish to switch the interrupt trigger, here are other options:
 *  kPinInterruptOnRisingEdge
 *  kPinInterruptOnFallingEdge
 *
 * To change the trigger, just call pin.setInterrupts(value) at any point.
 * Note that it may cause an interrupt to fire *immediately*!
 * vale defaults to kPinInterruptOnChange|kPinInterruptPriorityMedium is not specified.
 */

static IRQPin<kInput1_PinNumber>  input_1_pin  {kPullUp|kDebounce, []{_dispatch_pin(_condition_pin(1,  (input_1_pin.get()  != 0)));}};
static IRQPin<kInput2_PinNumber>  input_2_pin  {kPullUp|kDebounce, []{_dispatch_pin(_condition_pin(2,  (input_2_pin.get()  != 0)));}};
static IRQPin<kInput3_PinNumber>  input_3_pin  {kPullUp|kDebounce, []{_dispatch_pin(_condition_pin(3,  (input_3_pin.get()  != 0)));}};
static IRQPin<kInput4_PinNumber>  input_4_pin  {kPullUp|kDebounce, []{_dispatch_pin(_condition_pin(4,  (input_4_pin.get()  != 0)));}};
static IRQPin<kInput5_PinNumber>  input_5_pin  {kPullUp|kDebounce, []{_dispatch_pin(_condition_pin(5,  (input_5_pin.get()  != 0)));}};
static IRQPin<kInput6_PinNumber>  input_6_pin  {kPullUp|kDebounce, []{_dispatch_pin(_condition_pin(6,  (input_6_pin.get()  != 0)));}};
static IRQPin<kInput7_PinNumber>  input_7_pin  {kPullUp|kDebounce, []{_dispatch_pin(_condition_pin(7,  (input_7_pin.get()  != 0)));}};
static IRQPin<kInput8_PinNumber>  input_8_pin  {kPullUp|kDebounce, []{_dispatch_pin(_condition_pin(8,  (input_8_pin.get()  != 0)));}};
static IRQPin<kInput9_PinNumber>  input_9_pin  {kPullUp|kDebounce, []{_dispatch_pin(_condition_pin(9,  (input_9_pin.get()  != 0)));}};
static IRQPin<kInput10_PinNumber> input_10_pin {kPullUp|kDebounce, []{_dispatch_pin(_condition_pin(10, (input_10_pin.get() != 0)));}};
static IRQPin<kInput11_PinNumber> input_11_pin {kPullUp|kDebounce, []{_dispatch_pin(_condition_pin(11, (input_11_pin.get() != 0)));}};
static IRQPin<kInput12_PinNumber> input_12_pin {kPullUp|kDebounce, []{_dispatch_pin(_condition_pin(12, (input_12_pin.get() != 0)));}};

// Generated with:
// perl -e 'for($i=1;$i<14;$i++) { print "#if OUTPUT${i}_PWM == 1\nstatic PWMOutputPin<kOutput${i}_PinNumber>  output_${i}_pin;\n#else\nstatic PWMLikeOutputPin<kOutput${i}_PinNumber>  output_${i}_pin;\n#endif\n";}'
// BEGIN generated
#if OUTPUT1_PWM == 1
static PWMOutputPin<kOutput1_PinNumber>  output_1_pin {kPWMOn, 200000};
#else
static PWMLikeOutputPin<kOutput1_PinNumber>  output_1_pin;
#endif
#if OUTPUT2_PWM == 1
static PWMOutputPin<kOutput2_PinNumber>  output_2_pin {kPWMOn, 200000};
#else
static PWMLikeOutputPin<kOutput2_PinNumber>  output_2_pin;
#endif
#if OUTPUT3_PWM == 1
static PWMOutputPin<kOutput3_PinNumber>  output_3_pin {kPWMOn, 200000};
#else
static PWMLikeOutputPin<kOutput3_PinNumber>  output_3_pin;
#endif
#if OUTPUT4_PWM == 1
static PWMOutputPin<kOutput4_PinNumber>  output_4_pin {kPWMOn, 200000};
#else
static PWMLikeOutputPin<kOutput4_PinNumber>  output_4_pin;
#endif
#if OUTPUT5_PWM == 1
static PWMOutputPin<kOutput5_PinNumber>  output_5_pin {kPWMOn, 200000};
#else
static PWMLikeOutputPin<kOutput5_PinNumber>  output_5_pin;
#endif
#if OUTPUT6_PWM == 1
static PWMOutputPin<kOutput6_PinNumber>  output_6_pin {kPWMOn, 200000};
#else
static PWMLikeOutputPin<kOutput6_PinNumber>  output_6_pin;
#endif
#if OUTPUT7_PWM == 1
static PWMOutputPin<kOutput7_PinNumber>  output_7_pin {kPWMOn, 200000};
#else
static PWMLikeOutputPin<kOutput7_PinNumber>  output_7_pin;
#endif
#if OUTPUT8_PWM == 1
static PWMOutputPin<kOutput8_PinNumber>  output_8_pin {kPWMOn, 200000};
#else
static PWMLikeOutputPin<kOutput8_PinNumber>  output_8_pin;
#endif
#if OUTPUT9_PWM == 1
static PWMOutputPin<kOutput9_PinNumber>  output_9_pin {kPWMOn, 200000};
#else
static PWMLikeOutputPin<kOutput9_PinNumber>  output_9_pin;
#endif
#if OUTPUT10_PWM == 1
static PWMOutputPin<kOutput10_PinNumber>  output_10_pin {kPWMOn, 200000};
#else
static PWMLikeOutputPin<kOutput10_PinNumber>  output_10_pin;
#endif
#if OUTPUT11_PWM == 1
static PWMOutputPin<kOutput11_PinNumber>  output_11_pin {kPWMOn, 200000};
#else
static PWMLikeOutputPin<kOutput11_PinNumber>  output_11_pin;
#endif
#if OUTPUT12_PWM == 1
static PWMOutputPin<kOutput12_PinNumber>  output_12_pin {kPWMOn, 200000};
#else
static PWMLikeOutputPin<kOutput12_PinNumber>  output_12_pin;
#endif
#if OUTPUT13_PWM == 1
static PWMOutputPin<kOutput13_PinNumber>  output_13_pin {kPWMOn, 200000};
#else
static PWMLikeOutputPin<kOutput13_PinNumber>  output_13_pin;
#endif
// END generated

#endif //__ARM

/************************************************************************************
 **** CODE **************************************************************************
 ************************************************************************************/
/*
 * gpio_init() - initialize inputs and outputs
 * gpio_reset() - reset inputs and outputs (no initialization)
 *
 *	AVR code assumes sys_init() and st_init() have been run previously to
 *	bind the ports and set bit IO directions, respectively. See hardware.h for details
 */

void gpio_init(void)
{
#ifdef __ARM

    /* Priority only needs set once in the system during startup.
     * However, if we wish to switch the interrupt trigger, here are other options:
     *  kPinInterruptOnRisingEdge
     *  kPinInterruptOnFallingEdge
     *
     * To change the trigger, just call pin.setInterrupts(value) at any point.
     * Note that it may cause an interrupt to fire *immediately*!
     */
//    input_1_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
//    input_2_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
//    input_3_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
//    input_4_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
//    input_5_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
//    input_6_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
//    input_7_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
//    input_8_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
//    input_9_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
//    input_10_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
//    input_11_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);
//    input_12_pin.setInterrupts(kPinInterruptOnChange|kPinInterruptPriorityMedium);

    // Generated with:
    // perl -e 'for($i=1;$i<14;$i++) { print "output_${i}_pin.setFrequency(200000);\n";}'
    // BEGIN generated
    output_1_pin.setFrequency(200000);
    output_2_pin.setFrequency(200000);
    output_3_pin.setFrequency(200000);
    output_4_pin.setFrequency(200000);
    output_5_pin.setFrequency(200000);
    output_6_pin.setFrequency(200000);
    output_7_pin.setFrequency(200000);
    output_8_pin.setFrequency(200000);
    output_9_pin.setFrequency(200000);
    output_10_pin.setFrequency(200000);
    output_11_pin.setFrequency(200000);
    output_12_pin.setFrequency(200000);
    output_13_pin.setFrequency(200000);
    // END generated

    return(gpio_reset());
#endif

#ifdef __AVR

    for (uint8_t i=0; i<D_IN_PAIRS; i++) {
        // Setup input bits and interrupts
        // Must have been previously set to inputs by stepper_init()
        if (d_in[i].mode == IO_MODE_DISABLED) {
            hw.sw_port[i]->INT0MASK = 0;                    // disable interrupts
        } else {
            hw.sw_port[i]->DIRCLR = SW_MIN_BIT_bm;          // set min input
            hw.sw_port[i]->PIN6CTRL = (PIN_MODE | PORT_ISC_BOTHEDGES_gc);
            hw.sw_port[i]->INT0MASK = SW_MIN_BIT_bm;	 	// interrupt on min switch
        }
        if (d_in[i+1].mode == IO_MODE_DISABLED) {
            hw.sw_port[i]->INT1MASK = 0;
        } else {
            hw.sw_port[i]->DIRCLR = SW_MAX_BIT_bm;          // set max input
            hw.sw_port[i]->PIN7CTRL = (PIN_MODE | PORT_ISC_BOTHEDGES_gc);
            hw.sw_port[i]->INT1MASK = SW_MAX_BIT_bm;		// max on INT1
        }
        // Set interrupt level. Interrupts must be enabled in main()
        hw.sw_port[i]->INTCTRL = GPIO1_INTLVL;                  // see hardware.h for setting
    }
    return(gpio_reset());
#endif
}

void gpio_reset(void)
{
    d_in_t *in;

    for (uint8_t i=0; i<D_IN_CHANNELS; i++) {
        in = &d_in[i];
        if (in->mode == IO_MODE_DISABLED) {
            in->state = INPUT_DISABLED;
            continue;
        }
        in->state = (ioState)(_read_raw_pin(i+1) ^ (in->mode ^ 1));    // correct for NO or NC mode
        in->lockout_ms = INPUT_LOCKOUT_MS;
        in->lockout_timer = SysTickTimer_getValue();
	}

    // If the output is ACTIVE_LOW set it to 1. ACTIVE_HIGH gets set to 0.
    if (d_out[1-1].mode  != IO_MODE_DISABLED) { (output_1_pin    = (d_out[1-1].mode  == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
    if (d_out[2-1].mode  != IO_MODE_DISABLED) { (output_2_pin    = (d_out[2-1].mode  == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
    if (d_out[3-1].mode  != IO_MODE_DISABLED) { (output_3_pin    = (d_out[3-1].mode  == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
    if (d_out[4-1].mode  != IO_MODE_DISABLED) { (output_4_pin    = (d_out[4-1].mode  == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
    if (d_out[5-1].mode  != IO_MODE_DISABLED) { (output_5_pin    = (d_out[5-1].mode  == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
    if (d_out[6-1].mode  != IO_MODE_DISABLED) { (output_6_pin    = (d_out[6-1].mode  == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
    if (d_out[7-1].mode  != IO_MODE_DISABLED) { (output_7_pin    = (d_out[7-1].mode  == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
    if (d_out[8-1].mode  != IO_MODE_DISABLED) { (output_8_pin    = (d_out[8-1].mode  == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
    if (d_out[9-1].mode  != IO_MODE_DISABLED) { (output_9_pin    = (d_out[9-1].mode  == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
    if (d_out[10-1].mode != IO_MODE_DISABLED) { (output_10_pin   = (d_out[10-1].mode == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
    if (d_out[11-1].mode != IO_MODE_DISABLED) { (output_11_pin   = (d_out[11-1].mode == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
    if (d_out[12-1].mode != IO_MODE_DISABLED) { (output_12_pin   = (d_out[12-1].mode == IO_ACTIVE_LOW) ? 1.0 : 0.0); }
    if (d_out[13-1].mode != IO_MODE_DISABLED) { (output_13_pin   = (d_out[13-1].mode == IO_ACTIVE_LOW) ? 1.0 : 0.0); }

}

/*
 * _read_raw_pin() - primitive to read an input pin without any conditioning
 */
static bool _read_raw_pin(const uint8_t input_num_ext)
{
#ifdef __ARM
    switch(input_num_ext) {
        case 1: { return (input_1_pin.get() != 0); }
        case 2: { return (input_2_pin.get() != 0); }
        case 3: { return (input_3_pin.get() != 0); }
        case 4: { return (input_4_pin.get() != 0); }
        case 5: { return (input_5_pin.get() != 0); }
        case 6: { return (input_6_pin.get() != 0); }
        case 7: { return (input_7_pin.get() != 0); }
        case 8: { return (input_8_pin.get() != 0); }
        case 9: { return (input_9_pin.get() != 0); }
        case 10: { return (input_10_pin.get() != 0); }
        case 11: { return (input_11_pin.get() != 0); }
        case 12: { return (input_12_pin.get() != 0); }
        default: { return false; } // ERROR
    }
#endif //__ARM

#ifdef __AVR
    switch (input_num_ext) {
        case 1: { return ((hw.sw_port[AXIS_X]->IN & SW_MIN_BIT_bm) != 0); }
        case 2: { return ((hw.sw_port[AXIS_X]->IN & SW_MAX_BIT_bm) != 0); }
        case 3: { return ((hw.sw_port[AXIS_Y]->IN & SW_MIN_BIT_bm) != 0); }
        case 4: { return ((hw.sw_port[AXIS_Y]->IN & SW_MAX_BIT_bm) != 0); }
        case 5: { return ((hw.sw_port[AXIS_Z]->IN & SW_MIN_BIT_bm) != 0); }
        case 6: { return ((hw.sw_port[AXIS_Z]->IN & SW_MAX_BIT_bm) != 0); }
        case 7: { return ((hw.sw_port[AXIS_A]->IN & SW_MIN_BIT_bm) != 0); }
        case 8: { return ((hw.sw_port[AXIS_A]->IN & SW_MAX_BIT_bm) != 0); }
        default: { return false; } // ERROR
    }
#endif //__AVR
}

/******************************
 * Interrupt Service Routines *
 ******************************/
/*
 * ARM pin change interrupts
 *
 * NOTE: InputPin<>.get() returns a uint32_t, and will NOT necessarily be 1 for true.
 * The actual values will be the pin's port mask or 0, so you must check for non-zero.
 */


//#ifdef __ARM
//namespace Motate {
//    #if INPUT1_AVAILABLE == 1
//    MOTATE_PIN_INTERRUPT(kInput1_PinNumber) { _dispatch_pin(_condition_pin(1, (input_1_pin.get() != 0))); }
//    #endif
//    #if INPUT2_AVAILABLE == 1
//    MOTATE_PIN_INTERRUPT(kInput2_PinNumber) { _dispatch_pin(_condition_pin(2, (input_2_pin.get() != 0))); }
//    #endif
//    #if INPUT3_AVAILABLE == 1
//    MOTATE_PIN_INTERRUPT(kInput3_PinNumber) { _dispatch_pin(_condition_pin(3, (input_3_pin.get() != 0))); }
//    #endif
//    #if INPUT4_AVAILABLE == 1
//    MOTATE_PIN_INTERRUPT(kInput4_PinNumber) { _dispatch_pin(_condition_pin(4, (input_4_pin.get() != 0))); }
//    #endif
//    #if INPUT5_AVAILABLE == 1
//    MOTATE_PIN_INTERRUPT(kInput5_PinNumber) { _dispatch_pin(_condition_pin(5, (input_5_pin.get() != 0))); }
//    #endif
//    #if INPUT6_AVAILABLE == 1
//    MOTATE_PIN_INTERRUPT(kInput6_PinNumber) { _dispatch_pin(_condition_pin(6, (input_6_pin.get() != 0))); }
//    #endif
//    #if INPUT7_AVAILABLE == 1
//    MOTATE_PIN_INTERRUPT(kInput7_PinNumber) { _dispatch_pin(_condition_pin(7, (input_7_pin.get() != 0))); }
//    #endif
//    #if INPUT8_AVAILABLE == 1
//    MOTATE_PIN_INTERRUPT(kInput8_PinNumber) { _dispatch_pin(_condition_pin(8, (input_8_pin.get() != 0))); }
//    #endif
//    #if INPUT9_AVAILABLE == 1
//    MOTATE_PIN_INTERRUPT(kInput9_PinNumber) { _dispatch_pin(_condition_pin(9, (input_9_pin.get() != 0))); }
//    #endif
//    #if INPUT10_AVAILABLE == 1
//    MOTATE_PIN_INTERRUPT(kInput10_PinNumber) { _dispatch_pin(_condition_pin(9, (input_10_pin.get() != 0))); }
//    #endif
//    #if INPUT11_AVAILABLE == 1
//    MOTATE_PIN_INTERRUPT(kInput11_PinNumber) { _dispatch_pin(_condition_pin(10, (input_11_pin.get() != 0))); }
//    #endif
//    #if INPUT13_AVAILABLE == 1
//    MOTATE_PIN_INTERRUPT(kInput12_PinNumber) { _dispatch_pin(_condition_pin(11, (input_12_pin.get() != 0))); }
//    #endif
//} // namespace Motate
//#endif

#ifdef __AVR
ISR(X_MIN_ISR_vect)	{ _dispatch_pin(_condition_pin(1, (hw.sw_port[AXIS_X]->IN & SW_MIN_BIT_bm) != 0)); }
ISR(X_MAX_ISR_vect)	{ _dispatch_pin(_condition_pin(2, (hw.sw_port[AXIS_X]->IN & SW_MAX_BIT_bm) != 0)); }
ISR(Y_MIN_ISR_vect)	{ _dispatch_pin(_condition_pin(3, (hw.sw_port[AXIS_Y]->IN & SW_MIN_BIT_bm) != 0)); }
ISR(Y_MAX_ISR_vect)	{ _dispatch_pin(_condition_pin(4, (hw.sw_port[AXIS_Y]->IN & SW_MAX_BIT_bm) != 0)); }
ISR(Z_MIN_ISR_vect)	{ _dispatch_pin(_condition_pin(5, (hw.sw_port[AXIS_Z]->IN & SW_MIN_BIT_bm) != 0)); }
ISR(Z_MAX_ISR_vect)	{ _dispatch_pin(_condition_pin(6, (hw.sw_port[AXIS_Z]->IN & SW_MAX_BIT_bm) != 0)); }
ISR(A_MIN_ISR_vect)	{ _dispatch_pin(_condition_pin(7, (hw.sw_port[AXIS_A]->IN & SW_MIN_BIT_bm) != 0)); }
ISR(A_MAX_ISR_vect)	{ _dispatch_pin(_condition_pin(8, (hw.sw_port[AXIS_A]->IN & SW_MAX_BIT_bm) != 0)); }
#endif //__AVR

/*
 * _condition_pin() - debounce and condition raw pin state
 *
 *  Input numbers are external, meaning they start at 1.
 *  Return pin number 0 if no further action is required (no dispatch)
 */
static uint8_t _condition_pin(const uint8_t input_num_ext, const int8_t pin_value)
{
    d_in_t *in = &d_in[input_num_ext-1];  // array index is one less than input number

    // return if input is disabled (not supposed to happen)
    if (in->mode == IO_MODE_DISABLED) {
        in->state = INPUT_DISABLED;
        return (0);
    }

    // return if the input is in lockout period (take no action)
    if (SysTickTimer_getValue() < in->lockout_timer) {
        return (0);
    }
    // return if no change in state
    int8_t pin_value_corrected = (pin_value ^ ((int)in->mode ^ 1));	// correct for NO or NC mode
    if (in->state == (ioState)pin_value_corrected) {
        return (0);
    }

    // record the changed state
    in->state = (ioState)pin_value_corrected;
    in->lockout_timer = SysTickTimer_getValue() + in->lockout_ms;
    if (pin_value_corrected == INPUT_ACTIVE) {
        in->edge = INPUT_EDGE_LEADING;
    } else {
        in->edge = INPUT_EDGE_TRAILING;
    }
    return (input_num_ext);
}

/*
 * _dispatch_pin() - execute pin changes
 *
 *  Run _condition_pin() before calling this function.
 *  Take no action if input number is zero
 */

static void _dispatch_pin(const uint8_t input_num_ext)
{
    if (input_num_ext == 0) {               // no action if input number is zero
        return;
    }

    d_in_t *in = &d_in[input_num_ext-1];    // array index is one less than input number

    // perform homing operations if in homing mode
    if (in->homing_mode) {
        if (in->edge == INPUT_EDGE_LEADING) {   // we only want the leading edge to fire
            en_take_encoder_snapshot();
            cm_start_hold();
        }
        return;
    }

    // perform probing operations if in probing mode
    if (in->probing_mode) {
        // We want to capture either way.
        // Probing tests the start condition for the correct direction ahead of time.
        // If we see any edge, it's the right one.
        en_take_encoder_snapshot();
        cm_start_hold();
        return;
    }

    // *** NOTE: From this point on all conditionals assume we are NOT in homing or probe mode ***

    // trigger the action on leading edges

    if (in->edge == INPUT_EDGE_LEADING) {
        if (in->action == INPUT_ACTION_STOP) {
			cm_start_hold();
        }
        if (in->action == INPUT_ACTION_FAST_STOP) {
			cm_start_hold();                        // for now is same as STOP
        }
        if (in->action == INPUT_ACTION_HALT) {
            cm_halt_all();					        // hard stop, including spindle and coolant
        }
        if (in->action == INPUT_ACTION_PANIC) {
            char msg[10];
            sprintf_P(msg, PSTR("input %d"), input_num_ext);
            cm_panic(STAT_PANIC, msg);
        }
        if (in->action == INPUT_ACTION_RESET) {
            hw_hard_reset();
        }
    }

    // these functions trigger on the leading edge
    if (in->edge == INPUT_EDGE_LEADING) {
        if (in->function == INPUT_FUNCTION_LIMIT) {
            cm.limit_requested = input_num_ext;

        } else if (in->function == INPUT_FUNCTION_SHUTDOWN) {
            cm.shutdown_requested = input_num_ext;

        } else if (in->function == INPUT_FUNCTION_INTERLOCK) {
            cm.safety_interlock_disengaged = input_num_ext;
        }
    }

    // trigger interlock release on trailing edge
    if (in->edge == INPUT_EDGE_TRAILING) {
        if (in->function == INPUT_FUNCTION_INTERLOCK) {
            cm.safety_interlock_reengaged = input_num_ext;
        }
    }
    sr_request_status_report(SR_REQUEST_TIMED);   //+++++ Put this one back in.
}

/*
 * _handle_pin_changed() - ISR helper
 *
 * Since we set the interrupt to kPinInterruptOnChange _handle_pin_changed() should
 * only be called when the pin *changes* values, so we can assume that the current
 * pin value is not the same as the previous value. Note that the value may have
 * changed rapidly, and may even have changed again since the interrupt was triggered.
 * In this case a second interrupt will likely follow this one immediately after exiting.
 *
 *  input_num is the input channel, 1 - N
 *  pin_value = 1 if pin is set, 0 otherwise
 */

//static void _handle_pin_changed(const uint8_t input_num_ext, const int8_t pin_value)
//{
//    io_di_t *in = &io.in[input_num_ext-1];  // array index is one less than input number
//
//    // return if input is disabled (not supposed to happen)
//	if (in->mode == IO_MODE_DISABLED) {
//    	in->state = INPUT_DISABLED;
//        return;
//    }
//
//    // return if the input is in lockout period (take no action)
//    if (SysTickTimer.getValue() < in->lockout_timer) {
//        return;
//    }
//
//	// return if no change in state
//	int8_t pin_value_corrected = (pin_value ^ ((int)in->mode ^ 1));	// correct for NO or NC mode
//	if ( in->state == (inputState)pin_value_corrected ) {
////    	in->edge = INPUT_EDGE_NONE;        // edge should only be reset by function or opposite edge
//    	return;
//	}
//
//	// record the changed state
//    in->state = (inputState)pin_value_corrected;
//	in->lockout_timer = SysTickTimer.getValue() + in->lockout_ms;
//    if (pin_value_corrected == INPUT_ACTIVE) {
//        in->edge = INPUT_EDGE_LEADING;
//    } else {
//        in->edge = INPUT_EDGE_TRAILING;
//    }
//
//    // perform homing operations if in homing mode
//    if (in->homing_mode) {
//        if (in->edge == INPUT_EDGE_LEADING) {   // we only want the leading edge to fire
//            en_take_encoder_snapshot();
//            cm_start_hold();
//        }
//        return;
//    }
//
//    // perform probing operations if in probing mode
//    if (in->probing_mode) {
//        if (in->edge == INPUT_EDGE_LEADING) {   // we only want the leading edge to fire
//            en_take_encoder_snapshot();
//            cm_start_hold();
//        }
//        return;
//    }
//
//	// *** NOTE: From this point on all conditionals assume we are NOT in homing or probe mode ***
//
//    // trigger the action on leading edges
//
//    if (in->edge == INPUT_EDGE_LEADING) {
//        if (in->action == INPUT_ACTION_STOP) {
//			cm_start_hold();
//        }
//        if (in->action == INPUT_ACTION_FAST_STOP) {
//			cm_start_hold();                        // for now is same as STOP
//        }
//        if (in->action == INPUT_ACTION_HALT) {
//	        cm_halt_all();					        // hard stop, including spindle and coolant
//        }
//        if (in->action == INPUT_ACTION_PANIC) {
//	        char msg[10];
//	        sprintf_P(msg, PSTR("input %d"), input_num_ext);
//	        cm_panic(STAT_PANIC, msg);
//        }
//        if (in->action == INPUT_ACTION_RESET) {
//            hw_hard_reset();
//        }
//    }
//
//	// these functions trigger on the leading edge
//    if (in->edge == INPUT_EDGE_LEADING) {
//		if (in->function == INPUT_FUNCTION_LIMIT) {
//			cm.limit_requested = input_num_ext;
//
//		} else if (in->function == INPUT_FUNCTION_SHUTDOWN) {
//			cm.shutdown_requested = input_num_ext;
//
//		} else if (in->function == INPUT_FUNCTION_INTERLOCK) {
//		    cm.safety_interlock_disengaged = input_num_ext;
//		}
//    }
//
//    // trigger interlock release on trailing edge
//    if (in->edge == INPUT_EDGE_TRAILING) {
//        if (in->function == INPUT_FUNCTION_INTERLOCK) {
//		    cm.safety_interlock_reengaged = input_num_ext;
//        }
//    }
//    sr_request_status_report(SR_REQUEST_TIMED);
//}

/********************************************
 **** Digital Input Supporting Functions ****
 ********************************************/
/*
 * switch_rtc_callback() - called from RTC for each RTC tick.
 *
 *	Each switch has a counter which is initially set to negative SW_DEGLITCH_TICKS.
 *	When a switch closure is DETECTED the count increments for each RTC tick.
 *	When the count reaches zero the switch is tripped and action occurs.
 *	The counter continues to increment positive until the lockout is exceeded.
 */

#ifdef __AVR
void switch_rtc_callback(void)
{
	for (uint8_t i=0; i < NUM_SWITCHES; i++) {
		if (sw.mode[i] == SW_MODE_DISABLED || sw.debounce[i] == SW_IDLE)
            continue;

		if (++sw.count[i] == SW_LOCKOUT_TICKS) {		// state is either lockout or deglitching
			sw.debounce[i] = SW_IDLE;
            // check if the state has changed while we were in lockout...
            uint8_t old_state = sw.state[i];
            if(old_state != read_switch(i)) {
                sw.debounce[i] = SW_DEGLITCHING;
                sw.count[i] = -SW_DEGLITCH_TICKS;
            }
            continue;
		}
		if (sw.count[i] == 0) {							// trigger point
			sw.sw_num_thrown = i;						// record number of thrown switch
			sw.debounce[i] = SW_LOCKOUT;
			if ((cm.cycle_state == CYCLE_HOMING) || (cm.cycle_state == CYCLE_PROBE)) {		// regardless of switch type
				cm_request_feedhold();
			} else if (sw.mode[i] & SW_LIMIT_BIT) {		// should be a limit switch, so fire it.
				sw.limit_flag = true;					// triggers an emergency shutdown
			}
		}
	}
}
#endif //__AVR

/*
 * gpio_set_homing_mode()   - set/clear input to homing mode
 * gpio_set_probing_mode()  - set/clear input to probing mode
 * gpio_read_input()        - read conditioned input
 *
 (* Note: input_num_ext means EXTERNAL input number -- 1-based
 */
void  gpio_set_homing_mode(const uint8_t input_num_ext, const bool is_homing)
{
    if (input_num_ext == 0) {
        return;
    }
    d_in[input_num_ext-1].homing_mode = is_homing;
}

void  gpio_set_probing_mode(const uint8_t input_num_ext, const bool is_probing)
{
    if (input_num_ext == 0) {
        return;
    }
    d_in[input_num_ext-1].probing_mode = is_probing;
}

bool gpio_read_input(const uint8_t input_num_ext)
{
    if (input_num_ext == 0) {
        return false;
    }
    return (d_in[input_num_ext-1].state);
}

/* Xmega Functions (retire these as possible)
 * get_switch_mode()  - return switch mode setting
 * get_limit_thrown() - return true if a limit was tripped
 * get_switch_num()   - return switch number most recently thrown
 * set_switch_type()
 * get_switch_type()
 */

#ifdef __AVR
uint8_t get_switch_mode(uint8_t sw_num) { return (sw.mode[sw_num]);}
uint8_t get_limit_switch_thrown(void) { return(sw.limit_flag);}
uint8_t get_switch_thrown(void) { return(sw.sw_num_thrown);}
void set_switch_type( uint8_t switch_type ) { sw.switch_type = switch_type; }
uint8_t get_switch_type() { return sw.switch_type; }
#endif

/*
 * read_switch() - read a switch directly with no interrupts or deglitching
 */
#ifdef __AVR
uint8_t read_switch(uint8_t sw_num)
{
	if ((sw_num < 0) || (sw_num >= NUM_SWITCHES)) return (SW_DISABLED);

	uint8_t read = 0;
	switch (sw_num) {
		case SW_MIN_X: { read = hw.sw_port[AXIS_X]->IN & SW_MIN_BIT_bm; break;}
		case SW_MAX_X: { read = hw.sw_port[AXIS_X]->IN & SW_MAX_BIT_bm; break;}
		case SW_MIN_Y: { read = hw.sw_port[AXIS_Y]->IN & SW_MIN_BIT_bm; break;}
		case SW_MAX_Y: { read = hw.sw_port[AXIS_Y]->IN & SW_MAX_BIT_bm; break;}
		case SW_MIN_Z: { read = hw.sw_port[AXIS_Z]->IN & SW_MIN_BIT_bm; break;}
		case SW_MAX_Z: { read = hw.sw_port[AXIS_Z]->IN & SW_MAX_BIT_bm; break;}
		case SW_MIN_A: { read = hw.sw_port[AXIS_A]->IN & SW_MIN_BIT_bm; break;}
		case SW_MAX_A: { read = hw.sw_port[AXIS_A]->IN & SW_MAX_BIT_bm; break;}
	}
	if (sw.switch_type == SW_TYPE_NORMALLY_OPEN) {
		sw.state[sw_num] = ((read == 0) ? SW_CLOSED : SW_OPEN);// confusing. An NO switch drives the pin LO when thrown
		return (sw.state[sw_num]);
	} else {
		sw.state[sw_num] = ((read != 0) ? SW_CLOSED : SW_OPEN);
		return (sw.state[sw_num]);
	}
}
#endif //__AVR

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 * These functions are not part of the NIST defined functions
 ***********************************************************************************/

static stat_t _io_set_helper(nvObj_t *nv, const int8_t lower_bound, const int8_t upper_bound)
{
	if ((nv->value < lower_bound) || (nv->value >= upper_bound)) {
		return (STAT_INPUT_VALUE_UNSUPPORTED);
	}
	set_ui8(nv);		// will this work in -1 is a valid value?
    if (cm_get_machine_state() != MACHINE_INITIALIZING) {
        gpio_reset();
    }
	return (STAT_OK);
}

stat_t io_set_mo(nvObj_t *nv)			// input type or disabled
{
    return (_io_set_helper(nv, IO_MODE_DISABLED, IO_MODE_MAX));
}

stat_t io_set_ac(nvObj_t *nv)			// input action
{
	return (_io_set_helper(nv, INPUT_ACTION_NONE, INPUT_ACTION_MAX));
}

stat_t io_set_fn(nvObj_t *nv)			// input function
{
	return (_io_set_helper(nv, INPUT_FUNCTION_NONE, INPUT_FUNCTION_MAX));
}

/*
 *  io_get_input() - return input state given an nv object
 */
stat_t io_get_input(nvObj_t *nv)
{
    char *num_start = nv->token;
    if (*(nv->group) == 0) {
        // if we don't have a group, then the group name is in the token
        // skip over "in"
        num_start+=2;
    }
    nv->value = d_in[strtol(num_start, NULL, 10)-1].state;

    nv->valuetype = TYPE_BOOL;
    return (STAT_OK);
}



stat_t io_set_st(nvObj_t *nv)			// output function
{
    char *num_start = nv->token;
    if (*(nv->group) == 0) {
        // if we don't have a group, then the group name is in the token
        // skip over "out"
        num_start+=3;
    }
    // the token has been stripped down to an ASCII digit string - use it as an index
    uint8_t output_num = strtol(num_start, NULL, 10);

    // Force pins that aren't available to be "disabled"
    switch (output_num) {
        case 1:  if (output_1_pin.isNull())  { nv->value = IO_MODE_DISABLED; } break;
        case 2:  if (output_2_pin.isNull())  { nv->value = IO_MODE_DISABLED; } break;
        case 3:  if (output_3_pin.isNull())  { nv->value = IO_MODE_DISABLED; } break;
        case 4:  if (output_4_pin.isNull())  { nv->value = IO_MODE_DISABLED; } break;
        case 5:  if (output_5_pin.isNull())  { nv->value = IO_MODE_DISABLED; } break;
        case 6:  if (output_6_pin.isNull())  { nv->value = IO_MODE_DISABLED; } break;
        case 7:  if (output_7_pin.isNull())  { nv->value = IO_MODE_DISABLED; } break;
        case 8:  if (output_8_pin.isNull())  { nv->value = IO_MODE_DISABLED; } break;
        case 9:  if (output_9_pin.isNull())  { nv->value = IO_MODE_DISABLED; } break;
        case 10: if (output_10_pin.isNull()) { nv->value = IO_MODE_DISABLED; } break;
        case 11: if (output_11_pin.isNull()) { nv->value = IO_MODE_DISABLED; } break;
        case 12: if (output_12_pin.isNull()) { nv->value = IO_MODE_DISABLED; } break;
        case 13: if (output_13_pin.isNull()) { nv->value = IO_MODE_DISABLED; } break;

        default:
            break;
    }

    return (_io_set_helper(nv, IO_MODE_DISABLED, IO_MODE_MAX));
}

/*
 *  io_get_output() - return output state given an nv object
 */
stat_t io_get_output(nvObj_t *nv)
{
    char *num_start = nv->token;
    if (*(nv->group) == 0) {
        // if we don't have a group, then the group name is in the token
        // skip over "out"
        num_start+=3;
    }
    // the token has been stripped down to an ASCII digit string - use it as an index
    uint8_t output_num = strtol(num_start, NULL, 10);

    ioMode outMode = d_out[output_num-1].mode;
    if (outMode == IO_MODE_DISABLED) {
        nv->value = 0; // Inactive
    } else {
        bool invert = (outMode == 0);
        // Note: !! forces a value to boolean 0 or 1
        switch (output_num) {
            case 1:  { nv->value = (float)output_1_pin; } break;
            case 2:  { nv->value = (float)output_2_pin; } break;
            case 3:  { nv->value = (float)output_3_pin; } break;
            case 4:  { nv->value = (float)output_4_pin; } break;
            case 5:  { nv->value = (float)output_5_pin; } break;
            case 6:  { nv->value = (float)output_6_pin; } break;
            case 7:  { nv->value = (float)output_7_pin; } break;
            case 8:  { nv->value = (float)output_8_pin; } break;
            case 9:  { nv->value = (float)output_9_pin; } break;
            case 10: { nv->value = (float)output_10_pin; } break;
            case 11: { nv->value = (float)output_11_pin; } break;
            case 12: { nv->value = (float)output_12_pin; } break;
            case 13: { nv->value = (float)output_13_pin; } break;

            default:
                {
                    nv->value = 0; // inactive
                }
        }
        if (invert) {
            nv->value = 1.0 - nv->value;
        }
    }

    nv->valuetype = TYPE_FLOAT;
    return (STAT_OK);
}

/*
 *  io_set_output() - return input state given an nv object
 */
stat_t io_set_output(nvObj_t *nv)
{
    char *num_start = nv->token;
    if (*(nv->group) == 0) {
        // if we don't have a group, then the group name is in the token
        // skip over "out"
        num_start+=3;
    }
    // the token has been stripped down to an ASCII digit string - use it as an index
    uint8_t output_num = strtol(num_start, NULL, 10);

    ioMode outMode = d_out[output_num-1].mode;
    if (outMode == IO_MODE_DISABLED) {
        nv->value = 0; // Inactive?
    } else {
        bool invert = (outMode == 0);
        float value = nv->value;
        if (invert) {
            value = 1.0 - value;
        }
        switch (output_num) {
            // Generated with:
            // perl -e 'for($i=1;$i<14;$i++) { print "case ${i}:  { output_${i}_pin = value; } break;\n";}'
            // BEGIN generated
            case 1:  { output_1_pin = value; } break;
            case 2:  { output_2_pin = value; } break;
            case 3:  { output_3_pin = value; } break;
            case 4:  { output_4_pin = value; } break;
            case 5:  { output_5_pin = value; } break;
            case 6:  { output_6_pin = value; } break;
            case 7:  { output_7_pin = value; } break;
            case 8:  { output_8_pin = value; } break;
            case 9:  { output_9_pin = value; } break;
            case 10:  { output_10_pin = value; } break;
            case 11:  { output_11_pin = value; } break;
            case 12:  { output_12_pin = value; } break;
            case 13:  { output_13_pin = value; } break;
            // END generated

            default:
                {
                    nv->value = 0; // inactive
                }
        }
    }
    return (STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

	static const char fmt_gpio_mo[] PROGMEM = "[%smo] input mode%15d [-1=disabled,0=NO,1=NC]\n";
	static const char fmt_gpio_ac[] PROGMEM = "[%sac] input action%13d [0=none,1=stop,2=halt,3=stop_steps,4=panic,5=reset]\n";
	static const char fmt_gpio_fn[] PROGMEM = "[%sfn] input function%11d [0=none,1=limit,2=interlock,3=shutdown]\n";
	static const char fmt_gpio_in[] PROGMEM = "Input %s state: %5d\n";

    static const char fmt_gpio_st[] PROGMEM = "[%sst] output mode%15d [-1=disabled,0=active low,1=active high]\n";
    static const char fmt_gpio_out[] PROGMEM = "Output %s state: %5d\n";

    static void _print_di(nvObj_t *nv, const char *format)
    {
        sprintf_P(cs.out_buf, format, nv->group, (int)nv->value);
        xio_writeline(cs.out_buf);
    }
	void io_print_mo(nvObj_t *nv) {_print_di(nv, fmt_gpio_mo);}
	void io_print_ac(nvObj_t *nv) {_print_di(nv, fmt_gpio_ac);}
	void io_print_fn(nvObj_t *nv) {_print_di(nv, fmt_gpio_fn);}
	void io_print_in(nvObj_t *nv) {
        sprintf_P(cs.out_buf, fmt_gpio_in, nv->token, (int)nv->value);
        xio_writeline(cs.out_buf);
    }

    void io_print_st(nvObj_t *nv) {_print_di(nv, fmt_gpio_st);}
    void io_print_out(nvObj_t *nv) {
        sprintf_P(cs.out_buf, fmt_gpio_out, nv->token, (int)nv->value);
        xio_writeline(cs.out_buf);
    }
#endif
