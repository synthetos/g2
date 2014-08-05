/*
 * xio.cpp - extended IO functions
 * This file is part of the TinyG2 project
 *
 * Copyright (c) 2013 - 2014 Alden S. Hart Jr.
 * Copyright (c) 2013 - 2014 Robert Giseburt
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
/*
 * XIO acts as an entry point into lower level IO routines - mostly serial IO. It supports
 * the USB, SPI and fiol IO sub-systems, as well as providing low level character functions
 * used by stdio (printf()). 
 */
#include "tinyg2.h"
#include "config.h"
#include "util.h"
#include "hardware.h"
#include "text_parser.h"
#include "xio.h"

/**** Allocate structures ****/

xioSingleton_t xio;

/**** CODE ****/

/*
 * xio_init()
 */

void xio_init()
{
	xio_init_assertions();
}

/* 
 * xio_init_assertions()
 * xio_test_assertions() - check memory integrity of xio sub-systems
 */

void xio_init_assertions()
{
//	cs.magic_start = MAGICNUM;
//	cs.magic_end = MAGICNUM;
}

stat_t xio_test_assertions()
{
/*
	if (ds[XIO_DEV_USB].magic_start		!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_USB].magic_end		!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_RS485].magic_start	!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_RS485].magic_end		!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_SPI1].magic_start	!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_SPI1].magic_end		!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_SPI2].magic_start	!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_SPI2].magic_end		!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_PGM].magic_start		!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_PGM].magic_end		!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (stderr != xio.stderr_shadow) 				 return (STAT_XIO_ASSERTION_FAILURE);
*/
	return (STAT_OK);
}



/*
 * read_char() - returns single char or -1 (_FDEV_ERR) is none available
 */
int read_char (void)
{
	return SerialUSB.readByte();
}

/* 
 *	read_line() - read a complete line from stdin
 *
 *	Accepts CR or LF as line terminator. Replaces CR or LF with NUL in the returned string.
 *
 *	Returns:
 *
 *	  STAT_OK		  Returns a complete null terminated string. 
 *					  Index contains total character count (less terminating NUL)
 *					  The terminating LF is not written to the string.
 *
 *	  STAT_EAGAIN	  Line is incomplete because input has no more characters.
 *					  Index is left at the first available space.
 *					  Retry later to read more of the string. Use index from previous call.
 * 
 *	  STAT_EOF		  Line is incomplete because end of file was reached (file devices)
 *					  Index can be used as a character count.
 *
 *	  STAT_BUFFER_FULL Incomplete because size was reached.
 *                    Index will equal size.
 *
 *	  STAT_FILE_SIZE_EXCEEDED returned if the starting index exceeds the size.
 *
 *	Note: uint8_t aka char_t but you might not have that typedef at this low a level
 */
stat_t read_line (uint8_t *buffer, uint16_t *index, size_t size)
{
	if (*index >= size) { return (STAT_FILE_SIZE_EXCEEDED);}

	for (int c; *index < size; (*index)++ ) {
		if ((c = read_char()) != _FDEV_ERR) {
			buffer[*index] = (uint8_t)c;
			if ((c == LF) || (c == CR)) {
				buffer[*index] = NUL;
				return (STAT_OK);
			}
			continue;
		}
		return (STAT_EAGAIN);
	}	
	return (STAT_BUFFER_FULL);
}

size_t write(uint8_t *buffer, size_t size)
{
//	SerialUSB.write(buffer, sizeof(buffer));
	size_t written = SerialUSB.write(buffer, size);
//	spi.write(buffer, size);
	return (written);
}

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/*
 * xio_set_spi() = 0=disable, 1=enable
 */
stat_t xio_set_spi(nvObj_t *nv)
{
	xio.spi_state = (uint8_t)nv->value;

#ifdef __ARM
	if (fp_EQ(nv->value, SPI_ENABLE)) {
		spi_miso_pin.setMode(kOutput);
		spi_mosi_pin.setMode(kOutput);
		spi_sck_pin.setMode(kOutput);

	} else if (fp_EQ(nv->value, SPI_DISABLE)) {
		spi_miso_pin.setMode(kInput);
		spi_mosi_pin.setMode(kInput);
		spi_sck_pin.setMode(kInput);
	}
#endif
	return (STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

static const char fmt_spi[] PROGMEM = "[spi] SPI state%20d [0=disabled,1=enabled]\n";
void xio_print_spi(nvObj_t *nv) { text_print_ui8(nv, fmt_spi);}

#endif // __TEXT_MODE
