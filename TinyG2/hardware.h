/*
 * hardware.h - system hardware configuration 
 * This file is part of the TinyG2 project
 *
 * Copyright (c) 2013 Alden S. Hart Jr.
 * Copyright (c) 2013 Robert Giseburt
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
#ifndef HARDWARE_H_ONCE
#define HARDWARE_H_ONCE

#include "motatePins.h"					// motateTimers.h are in files that use timers

#ifdef __cplusplus
extern "C"{
#endif

void hardware_init(void);				// master hardware init
void hardware_get_id(char_t *id);

#define SYS_ID_LEN 12					// length of system ID string from sys_get_id()

/**** Global System Defines ****/
/* CPU clock */	

#undef F_CPU							// set for delays
#define F_CPU 84000000UL				// should always precede <avr/delay.h>

#define RTC_PERIOD 10					// MS for system tick (systick * N)

/* Refer to tinyg2.h for Axes, motors & PWM channels used by the application */

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

/* Timer assignments. See stepper.h and other headers for setup */

#define DDA_TIMER_NUM 3					// stepper pulse generation in stepper.cpp
#define DDA_STATUS_REGISTER REG_TC1_SR0	// status register needed for clearing interrupts

#define DWELL_TIMER_NUM 4				// dwell timing in stepper.cpp
#define DWELL_STATUS_REGISTER REG_TC1_SR1

#define LOAD_TIMER_NUM 5				// request load timer in stepper.cpp
#define LOAD_STATUS_REGISTER REG_TC1_SR2

#define EXEC_TIMER_NUM 6				// request exec timer in stepper.cpp
#define EXEC_STATUS_REGISTER REG_TC2_SR0

/* Pin assignments */

// Communications support
Motate::pin_number i2c_sda_pin_num = 20;
Motate::pin_number i2c_scl_pin_num = 21;
Motate::pin_number spi_ss1_pin_num = 10;
Motate::pin_number spi_ss2_pin_num = 47;
Motate::pin_number spi_ss3_pin_num = 52;
Motate::pin_number spi_ss4_pin_num = 48;
Motate::pin_number spi_ss5_pin_num = 49;
Motate::pin_number spi_ss6_pin_num = 50;
Motate::pin_number kinen_sync_pin_num = 53;

// grbl compatibility
Motate::pin_number grbl_reset_pin_num = 54;
Motate::pin_number grbl_feedhold_pin_num = 55;
Motate::pin_number grbl_cycle_start_pin_num = 56;

// Gcode support
Motate::pin_number motor_enable_pin_num	  = 8;
Motate::pin_number spindle_enable_pin_num = 12;
Motate::pin_number spindle_dir_pin_num	  = 13;
Motate::pin_number spindle_pwm_pin_num	  = 11;
Motate::pin_number secondary_pwm_pin_num  = 9;
Motate::pin_number coolant_on_pin_num	  = 54;

// axes
Motate::pin_number axis_X_min_pin_num = 14;
Motate::pin_number axis_X_max_pin_num = 15;
Motate::pin_number axis_Y_min_pin_num = 16;
Motate::pin_number axis_Y_max_pin_num = 17;
Motate::pin_number axis_Z_min_pin_num = 18;
Motate::pin_number axis_Z_max_pin_num = 19;

Motate::pin_number axis_A_min_pin_num = 58;
Motate::pin_number axis_A_max_pin_num = 59;
Motate::pin_number axis_B_min_pin_num = 60;
Motate::pin_number axis_B_max_pin_num = 61;
Motate::pin_number axis_C_min_pin_num = 65;
Motate::pin_number axis_C_max_pin_num = 51;

// motors
Motate::pin_number motor_1_step_pin_num			= 2;
Motate::pin_number motor_1_dir_pin_num			= 5;
Motate::pin_number motor_1_enable_pin_num		= 22;
Motate::pin_number motor_1_microstep_0_pin_num	= 23;
Motate::pin_number motor_1_microstep_1_pin_num	= 24;
Motate::pin_number motor_1_vref_pin_num			= 66;

Motate::pin_number motor_2_step_pin_num			= 3;
Motate::pin_number motor_2_dir_pin_num			= 6;
Motate::pin_number motor_2_enable_pin_num		= 25;
Motate::pin_number motor_2_microstep_0_pin_num	= 26;
Motate::pin_number motor_2_microstep_1_pin_num	= 27;
Motate::pin_number motor_2_vref_pin_num			= 67;

#if (MOTORS >= 3)
Motate::pin_number motor_3_step_pin_num			= 4;
Motate::pin_number motor_3_dir_pin_num			= 7;
Motate::pin_number motor_3_enable_pin_num		= 28;
Motate::pin_number motor_3_microstep_0_pin_num	= 29;
Motate::pin_number motor_3_microstep_1_pin_num	= 30;
Motate::pin_number motor_3_vref_pin_num			= 62;
#else
Motate::pin_number motor_3_step_pin_num			= -1;
Motate::pin_number motor_3_dir_pin_num			= -1;
Motate::pin_number motor_3_enable_pin_num		= -1;
Motate::pin_number motor_3_microstep_0_pin_num	= -1;
Motate::pin_number motor_3_microstep_1_pin_num	= -1;
Motate::pin_number motor_3_vref_pin_num			= -1;
#endif

#if (MOTORS >= 4)
Motate::pin_number motor_4_step_pin_num			= 31;
Motate::pin_number motor_4_dir_pin_num			= 32;
Motate::pin_number motor_4_enable_pin_num		= 33;
Motate::pin_number motor_4_microstep_0_pin_num	= 35;
Motate::pin_number motor_4_microstep_1_pin_num	= 36;
Motate::pin_number motor_4_vref_pin_num			= 63;
#else
Motate::pin_number motor_4_step_pin_num			= -1;
Motate::pin_number motor_4_dir_pin_num			= -1;
Motate::pin_number motor_4_enable_pin_num		= -1;
Motate::pin_number motor_4_microstep_0_pin_num	= -1;
Motate::pin_number motor_4_microstep_1_pin_num	= -1;
Motate::pin_number motor_4_vref_pin_num			= -1;
#endif

#if (MOTORS >= 5)
Motate::pin_number motor_5_step_pin_num			= 37;
Motate::pin_number motor_5_dir_pin_num			= 38;
Motate::pin_number motor_5_enable_pin_num		= 39;
Motate::pin_number motor_5_microstep_0_pin_num	= 40;
Motate::pin_number motor_5_microstep_1_pin_num	= 41;
Motate::pin_number motor_5_vref_pin_num			= 64;
#else
Motate::pin_number motor_5_step_pin_num			= -1;
Motate::pin_number motor_5_dir_pin_num			= -1;
Motate::pin_number motor_5_enable_pin_num		= -1;
Motate::pin_number motor_5_microstep_0_pin_num	= -1;
Motate::pin_number motor_5_microstep_1_pin_num	= -1;
Motate::pin_number motor_5_vref_pin_num			= -1;
#endif

#if (MOTORS >= 6)
Motate::pin_number motor_6_step_pin_num			= 42;
Motate::pin_number motor_6_dir_pin_num			= 43;
Motate::pin_number motor_6_enable_pin_num		= 44;
Motate::pin_number motor_6_microstep_0_pin_num	= 45;
Motate::pin_number motor_6_microstep_1_pin_num	= 46;
Motate::pin_number motor_6_vref_pin_num			= 34;
#else
Motate::pin_number motor_6_step_pin_num			= -1;
Motate::pin_number motor_6_dir_pin_num			= -1;
Motate::pin_number motor_6_enable_pin_num		= -1;
Motate::pin_number motor_6_microstep_0_pin_num	= -1;
Motate::pin_number motor_6_microstep_1_pin_num	= -1;
Motate::pin_number motor_6_vref_pin_num			= -1;
#endif


/**** DEPRECATED CODE. BEST TO LEAVE IN UNTIL COMPLETELY REPLACED ****/


/*
 * INTERRUPT USAGE - TinyG uses a lot of them all over the place
 *
 *	HI	Stepper DDA pulse generation		(set in stepper.h)
 *	HI	Stepper load routine SW interrupt	(set in stepper.h)
 *	HI	Dwell timer counter 				(set in stepper.h)
 *  LO	Segment execution SW interrupt		(set in stepper.h) 
 *	MED	GPIO1 switch port					(set in gpio.h)
 *  MED	Serial RX for USB & RS-485			(set in xio_usart.h)
 *  LO	Serial TX for USB & RS-485			(set in xio_usart.h)
 *	LO	Real time clock interrupt			(set in xmega_rtc.h)
 */

/*
// DDA timer aliases
#define TC_BLOCK_DDA		TC1				// TC1 block base address (sam3x8e.h)
#define TC_CHANNEL_DDA		0				// DDA channel in DDA block
#define TC_ID_DDA			ID_TC3			// Device ID enumeration (sam3x8e.h)
#define TC_IRQn_DDA			TC3_IRQn		// Interrupt vector enumeration (same as device ID)
#define ISR_Handler_DDA		TC3_Handler		// alias for ISR handler function name
#define REG_CMR_DDA			REG_TC1_CMR0	// channel mode register
#define REG_CCR_DDA			REG_TC1_CCR0	// channel control register
#define REG_SR_DDA			REG_TC1_SR0		// status register
#define REG_RC_DDA			REG_TC1_RC0		// RC register (comparison register)
#define REG_IER_DDA			REG_TC1_IER0	// interrupt enable register
#define REG_IDR_DDA			REG_TC1_IDR0	// interrupt disable register
#define REG_IMR_DDA			REG_TC1_IMR0	// interrupt mask register

// Dwell timer aliases
#define TC_BLOCK_DWELL		TC1
#define TC_CHANNEL_DWELL	1
#define TC_ID_DWELL			ID_TC4

//#define TIMER_DWELL	 		TCD0		// Dwell timer	(see stepper.h)
//#define TIMER_LOAD			TCE0		// Loader timer	(see stepper.h)
//#define TIMER_EXEC			TCF0		// Exec timer	(see stepper.h)
//#define TIMER_5				TCC1		// unallocated timer
//#define TIMER_PWM1			TCD1		// PWM timer #1 (see pwm.c)
//#define TIMER_PWM2			TCE1		// PWM timer #2	(see pwm.c)
*/

/*** Motor, output bit & switch port assignments ***
 *** These are not all the same, and must line up in multiple places in gpio.h ***
 * Sorry if this is confusing - it's a board routing issue
 */
/*
#define PORT_MOTOR_1	PORTA			// motors mapped to ports
#define PORT_MOTOR_2 	PORTF
#define PORT_MOTOR_3	PORTE
#define PORT_MOTOR_6	PORTD
#define PORT_MOTOR_5	PORTD
#define PORT_MOTOR_6	PORTD

#define PORT_SWITCH_X 	PORTA			// Switch axes mapped to ports
#define PORT_SWITCH_Y 	PORTD
#define PORT_SWITCH_Z 	PORTE
#define PORT_SWITCH_A 	PORTF

#define PORT_OUT_V7_X	PORTA			// v7 mapping - Output bits mapped to ports
#define PORT_OUT_V7_Y 	PORTF
#define PORT_OUT_V7_Z	PORTD
#define PORT_OUT_V7_A	PORTE

#define PORT_OUT_V6_X	PORTA			// v6 and earlier mapping - Output bits mapped to ports
#define PORT_OUT_V6_Y 	PORTF
#define PORT_OUT_V6_Z	PORTE
#define PORT_OUT_V6_A	PORTD

// These next four must be changed when the PORT_MOTOR_* definitions change!
#define PORTCFG_VP0MAP_PORT_MOTOR_1_gc PORTCFG_VP0MAP_PORTA_gc
#define PORTCFG_VP1MAP_PORT_MOTOR_2_gc PORTCFG_VP1MAP_PORTF_gc
#define PORTCFG_VP2MAP_PORT_MOTOR_3_gc PORTCFG_VP2MAP_PORTE_gc
#define PORTCFG_VP3MAP_PORT_MOTOR_4_gc PORTCFG_VP3MAP_PORTD_gc

#define PORT_MOTOR_1_VPORT	VPORT0
#define PORT_MOTOR_2_VPORT	VPORT1
#define PORT_MOTOR_3_VPORT	VPORT2
#define PORT_MOTOR_4_VPORT	VPORT3
*/
/*
 * Port setup - Stepper / Switch Ports:
 *	b0	(out) step			(SET is step,  CLR is rest)
 *	b1	(out) direction		(CLR = Clockwise)
 *	b2	(out) motor enable 	(CLR = Enabled)
 *	b3	(out) microstep 0 
 *	b4	(out) microstep 1
 *	b5	(out) output bit for GPIO port1
 *	b6	(in) min limit switch on GPIO 2 (note: motor controls and GPIO2 port mappings are not the same)
 *	b7	(in) max limit switch on GPIO 2 (note: motor controls and GPIO2 port mappings are not the same)
 */
/*
#define MOTOR_PORT_DIR_gm 0x3F	// dir settings: lower 6 out, upper 2 in

enum cfgPortBits {			// motor control port bit positions
	STEP_BIT_bp = 0,		// bit 0
	DIRECTION_BIT_bp,		// bit 1
	MOTOR_ENABLE_BIT_bp,	// bit 2
	MICROSTEP_BIT_0_bp,		// bit 3
	MICROSTEP_BIT_1_bp,		// bit 4
	GPIO1_OUT_BIT_bp,		// bit 5 (4 gpio1 output bits; 1 from each axis)
	SW_MIN_BIT_bp,			// bit 6 (4 input bits for homing/limit switches)
	SW_MAX_BIT_bp			// bit 7 (4 input bits for homing/limit switches)
};

#define STEP_BIT_bm			(1<<STEP_BIT_bp)
#define DIRECTION_BIT_bm	(1<<DIRECTION_BIT_bp)
#define MOTOR_ENABLE_BIT_bm (1<<MOTOR_ENABLE_BIT_bp)
#define MICROSTEP_BIT_0_bm	(1<<MICROSTEP_BIT_0_bp)
#define MICROSTEP_BIT_1_bm	(1<<MICROSTEP_BIT_1_bp)
#define GPIO1_OUT_BIT_bm	(1<<GPIO1_OUT_BIT_bp)	// spindle and coolant output bits
#define SW_MIN_BIT_bm		(1<<SW_MIN_BIT_bp)		// minimum switch inputs
#define SW_MAX_BIT_bm		(1<<SW_MAX_BIT_bp)		// maximum switch inputs
*/
/* Bit assignments for GPIO1_OUTs for spindle, PWM and coolant */
/*
#define SPINDLE_BIT			0x08		// spindle on/off
#define SPINDLE_DIR			0x04		// spindle direction, 1=CW, 0=CCW
#define SPINDLE_PWM			0x02		// spindle PWMs output bit
#define MIST_COOLANT_BIT	0x01		// coolant on/off - these are the same due to limited ports
#define FLOOD_COOLANT_BIT	0x01		// coolant on/off

#define SPINDLE_LED			0
#define SPINDLE_DIR_LED		1
#define SPINDLE_PWM_LED		2
#define COOLANT_LED			3
*/
//#define INDICATOR_LED		SPINDLE_DIR_LED	// can use the spindle direction as an indicator LED

/**** Device singleton - global structure to allow iteration through similar devices ****/
/*
	Ports are shared between steppers and GPIO so we need a global struct.
	Each xmega port has 3 bindings; motors, switches and the output bit

	The initialization sequence is important. the order is:
		- sys_init()	binds all ports to the device struct
		- st_init() 	sets IO directions and sets stepper VPORTS and stepper specific functions
		- gpio_init()	sets up input and output functions and required interrupts	

	Care needs to be taken in routines that use ports not to write to bits that are 
	not assigned to the designated function - ur unpredicatable results will occur
*/
/*
typedef struct deviceSingleton {
//	PORT_t *st_port[MOTORS];	// bindings for stepper motor ports (stepper.c)
//	PORT_t *sw_port[MOTORS];	// bindings for switch ports (GPIO2)
//	PORT_t *out_port[MOTORS];	// bindings for output ports (GPIO1)
} deviceSingleton_t;
deviceSingleton_t device;
*/

#ifdef __cplusplus
}
#endif

#endif	// end of include guard: HARDWARE_H_ONCE
