/*
 * controller.h - g2core controller and main dispatch loop
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2016 Robert Giseburt
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

// see also: g2core.h MESSAGE_LEN and config.h NV_ lengths
#define SAVED_BUFFER_LEN 80             // saved buffer size (for reporting only)
#define MAXED_BUFFER_LEN 255            // same as streaming RX buffer size as a worst case
#define OUTPUT_BUFFER_LEN 512           // text buffer size

#define LED_NORMAL_BLINK_RATE 3000      // blink rate for normal operation (in ms)
#define LED_ALARM_BLINK_RATE 750        // blink rate for alarm state (in ms)
#define LED_SHUTDOWN_BLINK_RATE 300     // blink rate for shutdown state (in ms)
#define LED_PANIC_BLINK_RATE 100        // blink rate for panic state (in ms)

typedef enum {                          // manages startup lines
    CONTROLLER_INITIALIZING = 0,        // controller is initializing - not ready for use
    CONTROLLER_NOT_CONNECTED,           // has not yet detected connection to USB (or other comm channel)
    CONTROLLER_CONNECTED,               // has connected to USB (or other comm channel)
    CONTROLLER_STARTUP,                 // is running startup messages and lines
    CONTROLLER_READY,                   // is active and ready for use
    CONTROLLER_PAUSED                   // is paused - presumably in preparation for queue flush
} csControllerState;

typedef struct controllerSingleton {    // main TG controller struct
    magic_t magic_start;                // magic number to test memory integrity
    float null;                         // dumping ground for items with no target

    // system identification values
    float fw_build;                     // firmware build number
    float fw_version;                   // firmware version number
    float hw_platform;                  // hardware compatibility - platform type
    float hw_version;                   // hardware compatibility - platform revision

    // system state variables
    csControllerState controller_state;
    uint32_t led_timer;                 // used to flash indicator LED
    uint32_t led_blink_rate;            // used to flash indicator LED

    // communications state variables
    // cs.comm_mode is the setting for the communications more
    // js.json_mode is the actual current mode (see also js.json_now)
    commMode comm_mode;                 // ej: 0=text mode sticky, 1=JSON mode sticky, 2=auto mode
    commMode comm_request_mode;         // mode of request (may be different thatn the setting)
    
    // controller serial buffers
    char *bufp;                         // pointer to primary or secondary in buffer
    uint16_t linelen;                   // length of currently processing line
    char out_buf[OUTPUT_BUFFER_LEN];    // output buffer
    char saved_buf[SAVED_BUFFER_LEN];   // save the input buffer

    magic_t magic_end;
} controller_t;

extern controller_t cs;                 // controller state structure

/**** function prototypes ****/

void controller_init(void);
void controller_run(void);
void controller_set_connected(bool is_connected);
void controller_set_muted(bool is_muted);
bool controller_parse_control(char *p);

#endif // End of include guard: CONTROLLER_H_ONCE
