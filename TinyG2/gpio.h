/*
 * gpio.h - Digital IO  handling functions
 * This file is part of the TinyG project
 *
 * Copyright (c) 2015 Alden S. Hart, Jr.
 * Copyright (c) 2015 Robert Giseburt
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
#ifndef GPIO_H_ONCE
#define GPIO_H_ONCE

/*
 * GPIO defines
 */

#define DI_CHANNELS	        9       // number of digital inputs supported
#define DO_CHANNELS	        4       // number of digital outputs supported
#define AI_CHANNELS	        0       // number of analog inputs supported
#define AO_CHANNELS	        0       // number of analog outputs supported

#define IO_LOCKOUT_MS       50      // milliseconds to go dead after input firing

enum ioMode {
    IO_MODE_DISABLED = -1,			// pin is disabled
    IO_MODE_ACTIVE_LOW = 0,			// pin is active low (normally open)
    IO_MODE_ACTIVE_HIGH = 1,		// pin is active high (normally closed)
	IO_MODE_MAX						// unused. Just for range checking
};
#define NORMALLY_OPEN IO_MODE_ACTIVE_LOW    // equivalent
#define NORMALLY_CLOSED IO_MODE_ACTIVE_HIGH // equivalent

enum ioAction {                     // actions are initiated from within the input's ISR
    IO_ACTION_NONE = 0,
    IO_ACTION_STOP,                 // stop at normal jerk - preserves positional accuracy
    IO_ACTION_FAST_STOP,            // stop at high jerk - preserves positional accuracy
    IO_ACTION_HALT,                 // stop immediately - not guaranteed to preserve position
    IO_ACTION_RESET,                // reset system immediately
	IO_ACTION_MAX					// unused. Just for range checking
};

enum ioFunc {                       // functions are requested from the ISR, run from the main loop
    IO_FUNCTION_NONE = 0,
    IO_FUNCTION_LIMIT,              // limit switch processing
    IO_FUNCTION_INTERLOCK,          // interlock processing
    IO_FUNCTION_SHUTDOWN,           // shutdown in support of external emergency stop
    IO_FUNCTION_SPINDLE_READY,      // signal that spindle is ready (up to speed)
	IO_FUNCTION_MAX					// unused. Just for range checking
};

enum ioState {
    IO_DISABLED = -1,               // value returned if input is disabled
    IO_INACTIVE = 0,				// aka switch open, also read as 'false'
    IO_ACTIVE = 1					// aka switch closed, also read as 'true'
};

enum ioEdgeFlag {
    IO_EDGE_NONE = 0,               // no edge detected or edge flag reset
    IO_EDGE_LEADING,				// flag is set when leading edge is detected
    IO_EDGE_TRAILING				// flag is set when trailing edge is detected
};

/*
 * GPIO structures
 */
typedef struct ioDigitalInput {		// one struct per digital input
	ioMode mode;					// -1=disabled, 0=active low (NO), 1= active high (NC)
	ioAction action;                // 0=none, 1=stop, 2=halt, 3=stop_steps, 4=reset
	ioFunc function;                // function to perform when activated / deactivated

    int8_t state;                   // input state 0=inactive, 1=active, -1=disabled
    ioEdgeFlag edge;                // keeps a transient record of edges for immediate inquiry
    bool homing_mode;               // set true when input is in homing mode.

	uint16_t lockout_ms;            // number of milliseconds for debounce lockout
	uint32_t lockout_timer;         // time to expire current debounce lockout, or 0 if no lockout
} io_di_t;

typedef struct gpioDigitalOutput {  // one struct per digital output
    ioMode mode;
} io_do_t;

typedef struct gpioAnalogInput {    // one struct per analog input
    ioMode mode;
} io_ai_t;

typedef struct gpioAnalogOutput {   // one struct per analog output
    ioMode mode;
} io_ao_t;

typedef struct gpioSingleton {      // collected gpio
	io_di_t in[DI_CHANNELS];
    io_do_t out[DO_CHANNELS];     // Note: 'do' is a reserved word
    io_ai_t analog_in[AI_CHANNELS];
    io_ao_t analog_out[AO_CHANNELS];
} io_t;
extern io_t io;

/*
 * GPIO function prototypes
 */

void gpio_init(void);
void gpio_reset(void);

stat_t io_set_mo(nvObj_t *nv);
stat_t io_set_ac(nvObj_t *nv);
stat_t io_set_fn(nvObj_t *nv);

stat_t io_get_input(nvObj_t *nv);

#ifdef __TEXT_MODE
	void io_print_mo(nvObj_t *nv);
	void io_print_ac(nvObj_t *nv);
	void io_print_fn(nvObj_t *nv);
	void io_print_in(nvObj_t *nv);
#else
	#define io_print_mo tx_print_stub
	#define io_print_ac tx_print_stub
	#define io_print_fn tx_print_stub
	#define io_print_in tx_print_stub
#endif // __TEXT_MODE

#endif // End of include guard: GPIO_H_ONCE
