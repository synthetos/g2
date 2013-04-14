/*
 * tg_controller.h - tinyg controller and main dispatch loop
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

#define TG_FLAG_PROMPTS_bm (1<<0)	// prompt enabled if set
#define INPUT_BUFFER_LEN 255		// text buffer size (255 max)
#define SAVED_BUFFER_LEN 100		// saved buffer size (for reporting only)
#define OUTPUT_BUFFER_LEN 512		// text buffer size
#define STATUS_MESSAGE_LEN 32		// status message string storage allocation
#define APPLICATION_MESSAGE_LEN 64	// application message string storage allocation

#define LED_NORMAL_COUNTER 1000		// blink rate for normal operation
#define LED_ALARM_COUNTER 1000		// blink rate for alarm state

typedef struct controllerState {	// main TG controller struct
	uint16_t magic_start;			// magic number to test memory integrity	
	double null;					// dumping ground for items with no target
	double fw_build;				// tinyg firmware build number
	double fw_version;				// tinyg firmware version number
	double hw_version;				// tinyg hardware compatibility
	uint8_t test;
	uint8_t active_src;				// active source device
	uint8_t default_src;			// default source device
	uint8_t network_mode;			// 0=master, 1=repeater, 2=slave
	uint8_t linelen;				// length of currently processing line
	uint8_t led_state;				// 0=off, 1=on
	int32_t led_counter;			// a convenience for flashing an LED
	char in_buf[INPUT_BUFFER_LEN];	// input text buffer
	char out_buf[OUTPUT_BUFFER_LEN];// output text buffer
	char saved_buf[SAVED_BUFFER_LEN];// save the input buffer
	uint16_t magic_end;
} controller_t;
//controller_t controller_state;		// controller state structure

void controller_init(controller_t *cs, uint8_t std_in, uint8_t std_out, uint8_t std_err);
void controller_reset(controller_t *cs);
void controller_run(controller_t *cs);


#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _CONTROLLER_H_
