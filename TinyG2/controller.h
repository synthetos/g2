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
#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

#define STAT_FLAG_PROMPTS_bm (1<<0)		// prompt enabled if set
#define INPUT_BUFFER_LEN 255			// text buffer size (255 max)
#define SAVED_BUFFER_LEN 100			// saved buffer size (for reporting only)
#define OUTPUT_BUFFER_LEN 512			// text buffer size
#define STATUS_MESSAGE_LEN 32			// status message string storage allocation
#define APPLICATION_MESSAGE_LEN 64		// application message string storage allocation

#define LED_NORMAL_COUNTER 1000			// blink rate for normal operation (in ms)
#define LED_ALARM_COUNTER 100			// blink rate for alarm state (in ms)

typedef struct controllerState {		// main TG controller struct
	uint16_t magic_start;				// magic number to test memory integrity	
	float null;							// dumping ground for items with no target
	float fw_build;						// tinyg firmware build number
	float fw_version;					// tinyg firmware version number
	float hw_platform;					// tinyg hardware compatibility - platform type
	float hw_version;					// tinyg hardware compatibility - platform revision
	uint8_t active_src;					// active source device
	uint8_t default_src;				// default source device
	uint8_t comm_mode;					// communications mode 1=JSON
	uint8_t network_mode;				// 0=master, 1=repeater, 2=slave
	uint16_t linelen;					// length of currently processing line
	uint16_t linemax;					// size of input buffer or some other size
	uint32_t led_counter;				// a convenience for flashing an LED
	uint32_t nvm_base_addr;				// NVM base address
	uint32_t nvm_profile_base;			// NVM base address of current profile
	uint8_t in_buf[INPUT_BUFFER_LEN];	// input text buffer
	uint8_t out_buf[OUTPUT_BUFFER_LEN];	// output text buffer
	uint8_t saved_buf[SAVED_BUFFER_LEN];// save the input buffer
	uint16_t magic_end;
} controller_t;

extern controller_t cs;					// controller state structure

void controller_init(uint8_t std_in, uint8_t std_out, uint8_t std_err);
void controller_reset();
void controller_run();
void tg_text_response(const uint8_t status, const uint8_t *buf);


#ifdef __cplusplus
}
#endif

#endif // _CONTROLLER_H_
