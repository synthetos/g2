/*
 * UniqueId.cpp - utility function for getting processor unique id
 * This file is part of the TinyG project
 *
 * Copyright (c) 2014 Tom Cauchois
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

#include <sam.h>
#include <string.h>
#include "UniqueId.h"

#ifdef __cplusplus
extern "C" {
#endif
    static struct uuid stored_uuid = { 0, 0, 0, 0 };
    static uint16_t uuid_string16[UNIQUE_ID_STRING_LEN] = {0};
    
    __attribute__ ((long_call, section (".ramfunc")))
    void cacheUniqueId()
    {
        // Run EEFC uuid sequence
        const int EEFC_FCMD_STUI = 0x0E;
        const int EEFC_FCMD_SPUI = 0x0F;
        const int EEFC_KEY = 0x5A;
        while ((EFC0->EEFC_FSR & EEFC_FSR_FRDY) == 0);
        EFC0->EEFC_FCR = EEFC_FCR_FCMD(EEFC_FCMD_STUI) | EEFC_FCR_FKEY(EEFC_KEY);
        while ((EFC0->EEFC_FSR & EEFC_FSR_FRDY) == 1);
        // Read unique id @ 0x00080000
        stored_uuid.d0 = *(volatile unsigned long*)0x00080000;
        stored_uuid.d1 = *(volatile unsigned long*)0x00080004;
        stored_uuid.d2 = *(volatile unsigned long*)0x00080008;
        stored_uuid.d3 = *(volatile unsigned long*)0x0008000C;
        EFC0->EEFC_FCR = EEFC_FCR_FCMD(EEFC_FCMD_SPUI) | EEFC_FCR_FKEY(EEFC_KEY);
        while ((EFC0->EEFC_FSR & EEFC_FSR_FRDY) == 0);
        
        // Memory swap needs some time to stabilize
        for (uint32_t i=0; i<1000000; i++)
            // force compiler to not optimize this
            __asm__ __volatile__("");
    }

    const uint16_t* readUniqueIdString()
    {
        if(uuid_string16[0] == 0) {
            for(int i = 0; i < UNIQUE_ID_STRING_LEN; ++i) {
                unsigned long nibble = (((i >= 8) ? stored_uuid.d1 : stored_uuid.d0) >> ((i % 8) * 4)) & 0xF;
                if(nibble < 0xA) uuid_string16[i] = nibble + '0';
                else uuid_string16[i] = (nibble - 0xA) + 'a';
            }
        }
        return uuid_string16;
    }
    
#ifdef __cplusplus
}
#endif
