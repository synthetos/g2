/*
 * controller.h - tinyg controller and main dispatch loop
 * Part of TinyG2 project
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
#ifndef CONTROLLER_H_ONCE
#define CONTROLLER_H_ONCE

#ifdef __cplusplus
extern "C"{
#endif

#define STAT_FLAG_PROMPTS_bm (1<<0)		// prompt enabled if set
#define INPUT_BUFFER_LEN 255			// text buffer size (255 max)
#define SAVED_BUFFER_LEN 100			// saved buffer size (for reporting only)
#define OUTPUT_BUFFER_LEN 512			// text buffer size
#define APPLICATION_MESSAGE_LEN 64		// application message string storage allocation
//#define STATUS_MESSAGE_LEN __			// see tinyg2.h for status message string storage allocation

#define LED_NORMAL_TIMER 1000			// blink rate for normal operation (in ms)
#define LED_ALARM_TIMER 3000				// blink rate for alarm state (in ms)

typedef struct controllerSingleton {			// main TG controller struct
	magic_t magic_start;				// magic number to test memory integrity
	uint8_t controller_state;			// controller state
	float null;							// dumping ground for items with no target
	float fw_build;						// tinyg firmware build number
	float fw_version;					// tinyg firmware version number
	float hw_platform;					// tinyg hardware compatibility - platform type
	float hw_version;					// tinyg hardware compatibility - platform revision

	// communications state variables
	uint8_t primary_src;				// primary input source device
	uint8_t secondary_src;				// secondary input source device
	uint8_t default_src;				// default source device
	uint8_t network_mode;				// 0=master, 1=repeater, 2=slave
	uint16_t linelen;					// length of currently processing line

	// system state variables
	uint32_t led_timer;					// used by idlers to flash indicator LED
	uint8_t hard_reset_requested;		// flag to perform a hard reset
	uint8_t bootloader_requested;		// flag to enter the bootloader

	//+++++ These need to be moved
	uint8_t status_report_request;		// set true to request a sr
	uint32_t status_report_tick;		// time tick to generate next status report
	uint32_t nvm_base_addr;				// NVM base address
	uint32_t nvm_profile_base;			// NVM base address of current profile

	// controller serial buffers
	char *bufp;							// pointer to primary or secondary in buffer
	char_t in_buf[INPUT_BUFFER_LEN];	// input text buffer
	char_t out_buf[OUTPUT_BUFFER_LEN];	// output text buffer
	char_t saved_buf[SAVED_BUFFER_LEN];	// save the input buffer
	magic_t magic_end;
} controller_t;

extern controller_t cs;					// controller state structure

enum cmControllerState {				// manages startup lines
	CONTROLLER_INITIALIZING = 0,		// controller is initializing - not ready for use
	CONTROLLER_NOT_CONNECTED,			// controller has not yet detected connection to USB (or other comm channel)
	CONTROLLER_CONNECTED,				// controller has connected to USB (or other comm channel)
	CONTROLLER_STARTUP,					// controller is running startup messages and lines
	CONTROLLER_READY					// controller is active and ready for use
};

/**** function prototypes ****/

void controller_init(uint8_t std_in, uint8_t std_out, uint8_t std_err);
void controller_reset();
void controller_run();
void tg_text_response(const uint8_t status, const uint8_t *buf);


#ifdef __cplusplus
}
#endif

#endif // End of include guard: CONTROLLER_H_ONCE
