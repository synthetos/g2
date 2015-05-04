/*
 * util.cpp - a random assortment of useful functions
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2014 Alden S. Hart, Jr.
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
/* util contains a dog's breakfast of supporting functions that are not specific to tinyg:
 * including:
 *	  - math and min/max utilities and extensions
 *	  - vector manipulation utilities
 */

#include "tinyg2.h"
#include "util.h"

#ifdef __AVR
#include "xmega/xmega_rtc.h"
#endif


//*** debug utilities ***

#ifdef IN_DEBUGGER
#pragma GCC push_options
#pragma GCC optimize ("O0")
void _debug_trap() {
    while (1) {
        __NOP();
    }
}
#pragma GCC reset_options
#else
void _debug_trap() {
    // We might be able to put a print here, but it MIGHT interrupt other output
    // and might be deep in an ISR, so we had better just _NOP() and hope for the best.
    __NOP();
}
#endif


/**** Vector utilities ****
 * copy_vector()			- copy vector of arbitrary length
 * vector_equal()			- test if vectors are equal
 * get_axis_vector_length()	- return the length of an axis vector
 * set_vector()				- load values into vector form
 * set_vector_by_axis()		- load a single value into a zero vector
 */

float vector[AXES];	// statically allocated global for vector utilities

/*
void copy_vector(float dst[], const float src[])
{
	memcpy(dst, src, sizeof(dst));
}
*/

uint8_t vector_equal(const float a[], const float b[])
{
	if ((fp_EQ(a[AXIS_X], b[AXIS_X])) &&
		(fp_EQ(a[AXIS_Y], b[AXIS_Y])) &&
		(fp_EQ(a[AXIS_Z], b[AXIS_Z])) &&
		(fp_EQ(a[AXIS_A], b[AXIS_A])) &&
		(fp_EQ(a[AXIS_B], b[AXIS_B])) &&
		(fp_EQ(a[AXIS_C], b[AXIS_C]))) {
		return (true);
	}
	return (false);
}

float get_axis_vector_length(const float a[], const float b[])
{
	return (sqrt(square(a[AXIS_X] - b[AXIS_X]) +
				 square(a[AXIS_Y] - b[AXIS_Y]) +
				 square(a[AXIS_Z] - b[AXIS_Z]) +
				 square(a[AXIS_A] - b[AXIS_A]) +
				 square(a[AXIS_B] - b[AXIS_B]) +
				 square(a[AXIS_C] - b[AXIS_C])));
}

float *set_vector(float x, float y, float z, float a, float b, float c)
{
	vector[AXIS_X] = x;
	vector[AXIS_Y] = y;
	vector[AXIS_Z] = z;
	vector[AXIS_A] = a;
	vector[AXIS_B] = b;
	vector[AXIS_C] = c;
	return (vector);
}

float *set_vector_by_axis(float value, uint8_t axis)
{
	clear_vector(vector);
	switch (axis) {
		case (AXIS_X): vector[AXIS_X] = value; break;
		case (AXIS_Y): vector[AXIS_Y] = value; break;
		case (AXIS_Z): vector[AXIS_Z] = value; break;
		case (AXIS_A): vector[AXIS_A] = value; break;
		case (AXIS_B): vector[AXIS_B] = value; break;
		case (AXIS_C): vector[AXIS_C] = value;
	}
	return (vector);
}

/**** Math and other general purpose functions ****/

/* Slightly faster (*) multi-value min and max functions
 * 	min3() - return minimum of 3 numbers
 * 	min4() - return minimum of 4 numbers
 * 	max3() - return maximum of 3 numbers
 * 	max4() - return maximum of 4 numbers
 *
 * Implementation tip: Order the min and max values from most to least likely in the calling args
 *
 * (*) Macro min4 is about 20uSec, inline function version is closer to 10 uSec (Xmega 32 MHz)
 * 	#define min3(a,b,c) (min(min(a,b),c))
 *	#define min4(a,b,c,d) (min(min(a,b),min(c,d)))
 *	#define max3(a,b,c) (max(max(a,b),c))
 *	#define max4(a,b,c,d) (max(max(a,b),max(c,d)))
 */

float min3(float x1, float x2, float x3)
{
	float min = x1;
	if (x2 < min) { min = x2;}
	if (x3 < min) { return (x3);}
	return (min);
}

float min4(float x1, float x2, float x3, float x4)
{
	float min = x1;
	if (x2 < min) { min = x2;}
	if (x3 < min) { min = x3;}
	if (x4 < min) { return (x4);}
	return (min);
}

float max3(float x1, float x2, float x3)
{
	float max = x1;
	if (x2 > max) { max = x2;}
	if (x3 > max) { return (x3);}
	return (max);
}

float max4(float x1, float x2, float x3, float x4)
{
	float max = x1;
	if (x2 > max) { max = x2;}
	if (x3 > max) { max = x3;}
	if (x4 > max) { return (x4);}
	return (max);
}

/**** String utilities ****
 * strcpy_U() 	   - strcpy workalike to get around initial NUL for blank string - possibly wrong
 * isnumber() 	   - isdigit that also accepts plus, minus, and decimal point
 * escape_string() - add escapes to a string - currently for quotes only
 */

/*
uint8_t * strcpy_U( uint8_t * dst, const uint8_t * src )
{
	uint16_t index = 0;
	do {
		dst[index] = src[index];
	} while (src[index++] != 0);
	return dst;
}
*/

uint8_t isnumber(char c)
{
	if (c == '.') { return (true); }
	if (c == '-') { return (true); }
	if (c == '+') { return (true); }
	return (isdigit(c));
}

char *escape_string(char *dst, char *src)
{
	char c;
	char *start_dst = dst;

	while ((c = *(src++)) != 0) {	// NUL
		if (c == '"') { *(dst++) = '\\'; }
		*(dst++) = c;
	}
	return (start_dst);
}

/*
 * pstr2str() - return an AVR style progmem string as a RAM string. No effect on ARMs
 *
 *	This function deals with FLASH memory string confusion between the AVR series and ARMs.
 *	AVRs typically have xxxxx_P() functions which take strings from FLASH as args.
 *	On ARMs there is no need for this as strings are handled identically in FLASH and RAM.
 *
 *	This function copies a string from FLASH to a pre-allocated RAM buffer - see main.c for
 *	allocation and max length. On the ARM it's a pass through that just returns the address
 *	of the input string
 */
const char *pstr2str(const char *pgm_string)
{
#ifdef __AVR
	strncpy_P(global_string_buf, pgm_string, MESSAGE_LEN);
	return (global_string_buf);
#endif
#ifdef __ARM
//	return ((char_t *)pgm_string);
	return (pgm_string);
#endif
}

/*
 * fntoa() - return ASCII string given a float and a decimal precision value
 *
 *	Like sprintf, fntoa returns length of string, less the terminating NUL character
 */
char fntoa(char *str, float n, uint8_t precision)
{
    // handle special cases
	if (isnan(n)) {
		strcpy(str, "nan");
		return (3);

	} else if (isinf(n)) {
		strcpy(str, "inf");
		return (3);
/*
	} else if (precision == 0 ) { return((char_t)sprintf((char *)str, "%0.0f", (double) n));
	} else if (precision == 1 ) { return((char_t)sprintf((char *)str, "%0.1f", (double) n));
	} else if (precision == 2 ) { return((char_t)sprintf((char *)str, "%0.2f", (double) n));
	} else if (precision == 3 ) { return((char_t)sprintf((char *)str, "%0.3f", (double) n));
	} else if (precision == 4 ) { return((char_t)sprintf((char *)str, "%0.4f", (double) n));
	} else if (precision == 5 ) { return((char_t)sprintf((char *)str, "%0.5f", (double) n));
	} else if (precision == 6 ) { return((char_t)sprintf((char *)str, "%0.6f", (double) n));
	} else if (precision == 7 ) { return((char_t)sprintf((char *)str, "%0.7f", (double) n));
	} else					    { return((char_t)sprintf((char *)str, "%f", (double) n)); }

*/
	} else if (precision == 0 ) { return(sprintf(str, "%0.0f", (double) n));
	} else if (precision == 1 ) { return(sprintf(str, "%0.1f", (double) n));
	} else if (precision == 2 ) { return(sprintf(str, "%0.2f", (double) n));
	} else if (precision == 3 ) { return(sprintf(str, "%0.3f", (double) n));
	} else if (precision == 4 ) { return(sprintf(str, "%0.4f", (double) n));
	} else if (precision == 5 ) { return(sprintf(str, "%0.5f", (double) n));
	} else if (precision == 6 ) { return(sprintf(str, "%0.6f", (double) n));
	} else if (precision == 7 ) { return(sprintf(str, "%0.7f", (double) n));
	} else					    { return(sprintf(str, "%f", (double) n)); }
}

/*
 * compute_checksum() - calculate the checksum for a string
 *
 *	Stops calculation on null termination or length value if non-zero.
 *
 * 	This is based on the the Java hashCode function.
 *	See http://en.wikipedia.org/wiki/Java_hashCode()
 */
#define HASHMASK 9999

uint16_t compute_checksum(char const *string, const uint16_t length)
{
	uint32_t h = 0;
	uint16_t len = strlen(string);
	if (length != 0) len = min(len, length);
    for (uint16_t i=0; i<len; i++) {
		h = 31 * h + string[i];
    }
    return (h % HASHMASK);
}

/*-
 *  COPYRIGHT (C) 1986 Gary S. Brown.  You may use this program, or
 *  code or tables extracted from it, as desired without restriction.
 *
 *  First, the polynomial itself and its table of feedback terms.  The
 *  polynomial is
 *  X^32+X^26+X^23+X^22+X^16+X^12+X^11+X^10+X^8+X^7+X^5+X^4+X^2+X^1+X^0
 *
 *  Note that we take it "backwards" and put the highest-order term in
 *  the lowest-order bit.  The X^32 term is "implied"; the LSB is the
 *  X^31 term, etc.  The X^0 term (usually shown as "+1") results in
 *  the MSB being 1
 *
 *  Note that the usual hardware shift register implementation, which
 *  is what we're using (we're merely optimizing it by doing eight-bit
 *  chunks at a time) shifts bits into the lowest-order term.  In our
 *  implementation, that means shifting towards the right.  Why do we
 *  do it this way?  Because the calculated CRC must be transmitted in
 *  order from highest-order term to lowest-order term.  UARTs transmit
 *  characters in order from LSB to MSB.  By storing the CRC this way
 *  we hand it to the UART in the order low-byte to high-byte; the UART
 *  sends each low-bit to hight-bit; and the result is transmission bit
 *  by bit from highest- to lowest-order term without requiring any bit
 *  shuffling on our part.  Reception works similarly
 *
 *  The feedback terms table consists of 256, 32-bit entries.  Notes
 *
 *      The table can be generated at runtime if desired; code to do so
 *      is shown later.  It might not be obvious, but the feedback
 *      terms simply represent the results of eight shift/xor opera
 *      tions for all combinations of data and CRC register values
 *
 *      The values must be right-shifted by eight bits by the "updcrc
 *      logic; the shift must be unsigned (bring in zeroes).  On some
 *      hardware you could probably optimize the shift in assembler by
 *      using byte-swap instructions
 *      polynomial $edb88320
 *
 *
 * CRC32 code derived from work by Gary S. Brown.
 */

static uint32_t crc32_tab[] = {
	0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
	0xe963a535, 0x9e6495a3,	0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
	0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
	0xf3b97148, 0x84be41de,	0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
	0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec,	0x14015c4f, 0x63066cd9,
	0xfa0f3d63, 0x8d080df5,	0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
	0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,	0x35b5a8fa, 0x42b2986c,
	0xdbbbc9d6, 0xacbcf940,	0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
	0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
	0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
	0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,	0x76dc4190, 0x01db7106,
	0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
	0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
	0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
	0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
	0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
	0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
	0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
	0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
	0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
	0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
	0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
	0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
	0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
	0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
	0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
	0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
	0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
	0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
	0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
	0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
	0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
	0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
	0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
	0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
	0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
	0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
	0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
	0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
	0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
	0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
	0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
	0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

uint32_t crc32(uint32_t crc, const void *buf, size_t size)
{
	const uint8_t *p = (uint8_t*)buf;
	crc = crc ^ ~0U;
    
	while (size--)
		crc = crc32_tab[(crc ^ *p++) & 0xFF] ^ (crc >> 8);
    
	return crc ^ ~0U;
}

/*
 * SysTickTimer_getValue() - this is a hack to get around some compatibility problems
 */

#ifdef __AVR
uint32_t SysTickTimer_getValue()
{
	return (rtc.sys_ticks);
}
#endif // __AVR

#ifdef __ARM
uint32_t SysTickTimer_getValue()
{
	return (SysTickTimer.getValue());
}
#endif // __ARM
