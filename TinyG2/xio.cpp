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

/**** Structures ****
 *
 */
/*
typedef struct xioFilesys {				// describes a device for reading and writing
	uint8_t state;						// physical device state
	uint8_t next_state;					// transitional state
} xioFilesys_t;
*/
struct xioDevice_t {	// describes a device for reading and writing
	uint8_t state;						// physical device state
	uint8_t next_state;					// transitional state
	int16_t readchar() {
		return _readchar();				// Add buffer logic here. For now, its a passthrough
	};
										// Each subclass will override this.
	virtual int16_t _readchar() = 0;	// Pure virtual.
	uint16_t linelen;					// length of completed line
	uint16_t read_index;				// index into line being read
	uint16_t read_buffer_len;			// static variable set at init time.
	char_t read_buf[READ_BUFFER_LEN];	// primary input buffer
};

typedef struct xioChannel {				// describes a device for reading and writing
	uint8_t type;						// channel type - control or data
	uint8_t state;						// channel state
	int8_t device;						// device or file handle channel is bound to
} xioChannel_t;

typedef struct xioSingleton {
	uint16_t magic_start;				// magic number to test memory integrity
//	xioDevice_t d[DEV_MAX];				// allocate device structures


	xioChannel_t c[CHAN_MAX];			// allocate channel structures
//	xioFilesys_t f[FS_MAX];				// allocate file handles
	uint8_t spi_state;					// tick down-counter (unscaled)
	uint16_t magic_end;
} xioSingleton_t;
xioSingleton_t xio;
//extern xioSingleton_t xio;

// We will use a templated subclass so we don't have to create a new
// subclass for every type of Device.
// All this requires is the readByte() function to exist in type Device.
// Note that we expect Device to be a pointer type!
template<typename Device>
struct xioDeviceWrapper : xioDevice_t {	// describes a device for reading and writing
	Device _dev;
	xioDeviceWrapper(Device  dev) : _dev{dev} {};

	int16_t _readchar() final {
		return _dev->readByte();
	};
};

xioDevice_t* devices[] = {&serialUSB0Wrapper, &serialUSB1Wrapper};

/**** CODE ****/

/*
 * read_char() - returns single char or -1 (_FDEV_ERR) is none available
 */
int read_char (void)
{
    return devices[0]->readchar();
//    return SerialUSB1.readByte();
}

/*
 * xio_init()
 */

void xio_init()
{
	uint8_t i;

    xio_init_assertions();

	for (i=0; i<DEV_MAX; i++) {
		memset(&xio.d[i], 0, sizeof(xioDevice_t));	// clear states and all values
	}
	for (i=0; i<CHAN_MAX; i++) {
		memset(&xio.c[i], 0, sizeof(xioChannel_t));	// clear states and all values
		xio.c[i].type = i;							// set control or device channel by numbering convention
	}

	// set up USB device state change callbacks
	// See here for info on lambda functions:
	// http://www.cprogramming.com/c++11/c++11-lambda-closures.html

	// bindings for USBserial0
	SerialUSB.setConnectionCallback([&](bool connected) {
		xio.d[DEV_USB0].next_state = connected ? DEVICE_CONNECTED : DEVICE_NOT_CONNECTED;
	});
	xio.d[DEV_USB0].read_buffer_len = READ_BUFFER_LEN;
//	xio.d[DEV_USB0].readchar = (char_t)&SerialUSB.readByte;

	// bindings for USBserial1
	SerialUSB1.setConnectionCallback([&](bool connected) {
		xio.d[DEV_USB1].next_state = connected ? DEVICE_CONNECTED : DEVICE_NOT_CONNECTED;
	});
	xio.d[DEV_USB1].read_buffer_len = READ_BUFFER_LEN;
}

/*
 * xio_init_assertions()
 * xio_test_assertions() - check memory integrity of xio sub-systems
 */

void xio_init_assertions()
{
	xio.magic_start = MAGICNUM;
	xio.magic_end = MAGICNUM;
}

stat_t xio_test_assertions()
{
	if ((xio.magic_start != MAGICNUM) || (xio.magic_end != MAGICNUM)) return (STAT_XIO_ASSERTION_FAILURE);
	return (STAT_OK);
}

/*
 * xio_callback() - callback from main loop for various IO functions
 *
 *	The USB channel binding functionality is in here. If this gets too big or there are other
 *	things to do during the callback it may make sense to break this out into a separate function.
 *
 *	Channel binding rules
 *
 *
 */
stat_t xio_callback()
{
	if ((xio.d[DEV_USB0].next_state == 0) && (xio.d[DEV_USB1].next_state == 0)) return (STAT_OK);
	return (STAT_OK);
}

/*
 * xio_bind_device() - bind a device to a channel
 *
 *	This function is called
 */

/*
 * read_char() - returns single char or -1 (_FDEV_ERR) is none available
 */
int read_char (uint8_t dev)
{
	if (dev == DEV_USB0) {
		return SerialUSB.readByte();
	}
	if (dev == DEV_USB1) {
		return SerialUSB1.readByte();
	}
	return(_FDEV_ERR);
}

/*
 * readline() - read a complete line from stdin (NEW STYLE)
 *
 *	Reads a line of text from the next device that has one ready. With some exceptions.
 *	Accepts CR or LF as line terminator. Replaces CR or LF with NUL in the returned string.
 *
 *	This function reads from separate control and data devices, including reading from multiple
 *	control devices. It will also manage multiple data devices, but only one data device may
 *	be open at a time.
 *
 *	ARGS:
 *
 *	 type - Sets channel type to read. If TYPE_NONE will read either control or data.
 *			On return this variable is set to the channel type that was read, or
 *			TYPE_NONE if no text was returned.
 *
 *   size - Sets the maximum size of the read buffer. On return is set to the number of
 *			characters in the buffer, including the NUL termination. Set to zero if no text
 *			was returned. Will truncate a text line with a NUL if size is reached.
 *
 *	 char * Returns a pointer to the buffer containing the line, or FDEV_ERR (-1) if no text
 */
char *readline(uint8_t &type, int16_t &size)
{
	return ((char *)_FDEV_ERR);
}

/*
 * read_line() - read a complete line from stdin (OLD STYLE)
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
	size_t written = SerialUSB.write(buffer, size);
//    size_t written = SerialUSB1.write(buffer, size);
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
