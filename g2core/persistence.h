/*
 * persistence.h - persistence code
 * This file is part of the g2code project
 *
 * Copyright (c) 2013 - 2018 Alden S. Hart Jr.
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

#ifndef PERSISTENCE_H_ONCE
#define PERSISTENCE_H_ONCE

#include "config.h"  // needed for nvObj_t definition
#include "ff.h"
#include "util.h"                   // FIXME: this won't compile if included after <map>

#define NVM_VALUE_LEN 4             // NVM value length (float, fixed length)
#define NVM_BASE_ADDR 0x0000        // base address of usable NVM

#define IO_BUFFER_SIZE 512          // this should be evenly divisible by NVM_VALUE_LEN, and <=512 until multi-block reads are fixed (right now they are hanging...)
#define MIN_WRITE_INTERVAL 1000     // minimum interval between persistence file writes
#define MAX_WRITE_FAILURES 3
#define MAX_WRITE_CHANGES IO_BUFFER_SIZE    // maximum number of write values that change - ms: TODO

//**** persistence singleton ****

typedef struct nvmSingleton {
    float tmp_value;
    FATFS fat_fs;
    FIL file;
    uint8_t file_index;
    uint8_t io_buffer[IO_BUFFER_SIZE];
    index_t changed_nv_indexes[MAX_WRITE_CHANGES];
    uint16_t changed_nvs;
    uint32_t last_write_systick;
    uint8_t write_failures;
} nvmSingleton_t;

//**** persistence function prototypes ****

void persistence_init(void);
stat_t read_persistent_value(nvObj_t *nv);
stat_t write_persistent_value(nvObj_t *nv);
stat_t write_persistent_values_callback();

#endif  // End of include guard: PERSISTENCE_H_ONCE
