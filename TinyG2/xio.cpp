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

/**** Structures ****/

struct xioDevice_t {						// description pf a device for reading and writing
	// connection and device management
	uint8_t caps;							// bitfield for capabilities flags (these are persistent)
	devflags_t flags;						// bitfield for device state flags (these are not)
	devflags_t next_flags;					// bitfield for next-state transitions

	// line reader functions
	uint16_t read_index;					// index into line being read
	uint16_t read_buf_size;					// static variable set at init time
	char_t read_buf[USB_LINE_BUFFER_SIZE];	// buffer for reading lines
};

struct xioDeviceWrapperBase {				// C++ base class for device primitives
	virtual int16_t readchar() = 0;			// Pure virtual. Will be subclassed for every device
//	virtual int16_t write() = 0;			// Pure virtual. Will be subclassed for every device
//	bool canRead() { return caps & DEV_CAN_READ; }
};

// Use a templated subclass so we don't have to create a new subclass for every type of Device.
// All this requires is the readByte() function to exist in type Device.
// Note that we expect Device to be a pointer type!
// See here for a discussion of what this means if you are not familiar with C++
// https://github.com/synthetos/g2/wiki/Dual-Endpoint-USB-Internals#c-classes-virtual-functions-and-inheritance

template<typename Device>
struct xioDeviceWrapper : xioDeviceWrapperBase {	// describes a device for reading and writing
    Device _dev;
    xioDeviceWrapper(Device  dev) : _dev{dev} {};

    int16_t readchar() final {
		return _dev->readByte();					// readByte calls the USB endpoint's read function
	};
};

// ALLOCATIONS
// Declare a device wrapper class for USB0 and USB1 and put pointer in an array as elements 0 and 1
xioDeviceWrapper<decltype(&SerialUSB)> serialUSB0Wrapper {&SerialUSB};
xioDeviceWrapper<decltype(&SerialUSB1)> serialUSB1Wrapper {&SerialUSB1};
xioDeviceWrapperBase* DeviceWrappers[] {&serialUSB0Wrapper, &serialUSB1Wrapper};

static xioDevice_t _ds[DEV_MAX];			// allocate device structures

// Singleton acts and namespace for xio
struct xioSingleton_t {
	uint16_t magic_start;					// magic number to test memory integrity
	xioDevice_t* d[DEV_MAX];				// pointers to device structures
	xioDeviceWrapperBase *DeviceWrappers[];	// access device wrappers from within the singleton
	uint8_t spi_state;						// tick down-counter (unscaled)
	uint8_t dev;							// hack to make it visible to the debugger in optimized code.
	uint16_t magic_end;
};
xioSingleton_t xio;
//extern xioSingleton_t xio; // not needed externally)

// convenience macros
#define USB0 xio.d[DEV_USB0]				// pointer to device structure for USB0 serial
#define USB1 xio.d[DEV_USB1]				// pointer to device structure for USB1 serial

/**** CODE ****/
/*
 * Device helpers
 *
 *	various flag readers
 *	downCtrl() - shuts down a specified control channel
 *	downData() - shuts down the active data channel
 */

inline bool canRead(xioDevice_t *d)	{ return d->caps & DEV_CAN_READ; }
inline bool canWrite(xioDevice_t *d) { return d->caps & DEV_CAN_WRITE; }
inline bool canBeCtrl(xioDevice_t *d) { return d->caps & DEV_CAN_BE_CTRL; }
inline bool canBeData(xioDevice_t *d) { return d->caps & DEV_CAN_BE_DATA; }
inline bool isCtrl(xioDevice_t *d) { return d->flags & DEV_IS_CTRL; }
inline bool isData(xioDevice_t *d) { return d->flags & DEV_IS_DATA; }
inline bool isPrimary(xioDevice_t *d) {return d->flags & DEV_IS_PRIMARY; }
inline bool isConnected(xioDevice_t *d) { return d->flags & DEV_IS_CONNECTED; }
inline bool isNotConnected(xioDevice_t *d) { return ~(d->flags & DEV_IS_CONNECTED); }
inline bool isNextConnected(xioDevice_t *d) { return d->next_flags & DEV_IS_CONNECTED; }
inline bool isReady(xioDevice_t *d) { return d->flags & DEV_IS_READY; }
inline bool isActive(xioDevice_t *d) { return d->flags & DEV_IS_ACTIVE; }

void downCtrl(xioDevice_t *d)
{
	d->flags &= ~(DEV_IS_CTRL | DEV_IS_CONNECTED | DEV_IS_READY | DEV_IS_ACTIVE);	// take down the channel
}

void downData()
{
	for (uint8_t dev=0; dev<DEV_MAX; dev++) {
		if (xio.d[dev]->flags && (DEV_IS_DATA | DEV_IS_ACTIVE)) {
			xio.d[dev]->flags &= ~(DEV_IS_DATA | DEV_IS_ACTIVE | DEV_IS_READY | DEV_IS_CONNECTED);	// take down the channel
		}
		// This is currently naiive and does no finalization
		// No allowance for buffered data (buffer clears, queue flushes, etc.)
		// Put flush queues here
		return;
	}
}

/*
 * xio_init()
 */

void xio_init()
{
    xio_init_assertions();

	for (uint8_t dev=0; dev<DEV_MAX; dev++) {
		memset(&xio.d[dev], 0, sizeof(xioDevice_t));	// clear states and all values
		xio.d[dev] = &_ds[dev];							// initialize pointers to device structures
	}

	// set up USB device state change callbacks
	// See here for info on lambda functions:
	// http://www.cprogramming.com/c++11/c++11-lambda-closures.html

	// setup for USBserial0
	SerialUSB.setConnectionCallback([&](bool connected) {
		USB0->next_flags = connected ? DEV_IS_CONNECTED : DEV_FLAGS_CLEAR;
	});
	USB0->read_buf_size = USB_LINE_BUFFER_SIZE;
	USB0->caps = (DEV_CAN_READ | DEV_CAN_WRITE | DEV_CAN_BE_CTRL | DEV_CAN_BE_DATA);

	// setup for USBserial1
	SerialUSB1.setConnectionCallback([&](bool connected) {
		USB1->next_flags = connected ? DEV_IS_CONNECTED : DEV_FLAGS_CLEAR;
	});
	USB1->read_buf_size = USB_LINE_BUFFER_SIZE;
	USB1->caps = (DEV_CAN_READ | DEV_CAN_WRITE | DEV_CAN_BE_CTRL | DEV_CAN_BE_DATA);
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
 *	Channel binding state machine (simple - does not do multiple ctrl channels yet)
 *	(0)	No connection
 *	(1)	Single USB (CTRL+DATA)
 *	(2) Dual USB (CTRL, DATA)
 *	(3) Force disconnect of DATA channel
 *
 *	Channel binding rules (start state --> end state)
 *	(0-->0) Initially all channels are disconnected. Channels are neither CTRL or DATA initialization
 *	(0-->1) One of the USB serial channels connects. It becomes CTRL+DATA
 *	(1-->2) The other channel connects (maybe). It becomes DATA. The first becomes CTRL
 *
 *	Channel unbinding rules
 *	(1-->0) CTRL+DATA channel disconnects. No connection exists (what about a secondary CTRL channel?)
 *	(2-->1) DATA channel disconnects. The first CTRL channel reverts to being CTRL+DATA
 *	(2-->3-->0) CTRL channel disconnects. If this is the primary CTRL channel all channels are
 *		disconnected (including DATA). If it is not the primary CTRL channel then only it disconnects.
 *
 *	Note: We will not receive a second DEV_IS_CONNECTED on a channel that has already received one,
 *		  and similarly for DEV_NOT_CONNECTED. IOW we will only get valid state transitions w/no repeats.
 */
stat_t xio_callback()
{
	// exit the callback if there is no new state information to process
	if ((USB0->next_flags == DEV_FLAGS_CLEAR) && (USB1->next_flags == DEV_FLAGS_CLEAR)) {
		return (STAT_OK);
	}

	// USB0 has just connected
	// Case 1: USB0 is the 1st channel to connect
	// Case 2: USB0 is the 2nd channel to connect
	if (isNextConnected(USB0)) {
		USB0->next_flags = DEV_FLAGS_CLEAR;						// clear the next state condition
		USB0->flags |= (DEV_IS_CONNECTED | DEV_IS_READY);		// tanned, rested and ready

		if (isNotConnected(USB1)) {								// Case 1: USB0 is the 1st channel to connect
			USB0->flags |= (DEV_IS_CTRL | DEV_IS_DATA | DEV_IS_PRIMARY | DEV_IS_ACTIVE);
		} else {												// Case 2: USB1 is the 1st channel to connect
			downData();											// shut down current data channel on USB1
			USB0->flags |= (DEV_IS_DATA | DEV_IS_ACTIVE);		// assign USB0 to be the active data channel
		}
		return (STAT_OK);
	}

	// USB1 has just connected
	if (isNextConnected(USB1)) {
		USB1->next_flags = DEV_FLAGS_CLEAR;
		USB1->flags |= (DEV_IS_CONNECTED | DEV_IS_READY);
		if (isNotConnected(USB0)) {								// Case 1: USB1 is the 1st channel to connect
			USB1->flags |= (DEV_IS_CTRL | DEV_IS_DATA | DEV_IS_PRIMARY | DEV_IS_ACTIVE);
		} else {												// Case 2: USB1 is the 2nd channel to connect
			downData();
			USB0->flags |= (DEV_IS_DATA | DEV_IS_ACTIVE);
		}
		return (STAT_OK);
	}

	// USB0 has just disconnected
	// Case 1: USB0 disconnected while it was a ctrl+data channel
	// Case 2: USB0 disconnected while it was primary ctrl channel (also takes down active data channel)
	// Case 3: USB0 disconnected while it was non-primary ctrl channel (does not take down data channel)
	// Case 4: USB0 disconnected while it was a data channel
	if (!isNextConnected(USB0)) {								// signals that USB0 just disconnected
		USB0->next_flags = DEV_FLAGS_CLEAR;
		if (USB0->flags && (DEV_IS_CTRL | DEV_IS_PRIMARY)) {		// Cases 1 & 2
			downCtrl(USB0);
			downData();
		} else if (USB0->flags && (DEV_IS_CTRL)) { downCtrl(USB0);	// Case 3
		} else if (USB0->flags && (DEV_IS_DATA)) { downData(); }	// Case 4
		return (STAT_OK);
	}

	// USB1 has just disconnected - same cases as USB0
	if (!isNextConnected(USB1)) {
		USB1->next_flags = DEV_FLAGS_CLEAR;
		if (USB1->flags && (DEV_IS_CTRL | DEV_IS_PRIMARY)) {
			downCtrl(USB1);
			downData();
		} else if (USB1->flags && (DEV_IS_CTRL)) { downCtrl(USB1);
		} else if (USB1->flags && (DEV_IS_DATA)) { downData(); }
	}
	return (STAT_OK);
}

/*
 * read_char() - returns single char or -1 (_FDEV_ERR) is none available
 */
int read_char (uint8_t dev)
{
//    return DeviceWrappers[dev]->readchar();

	if (dev == 0) {
		return SerialUSB.readByte();
	} else {
	    return SerialUSB1.readByte();
	}
}

/*
 * readline() - read a complete line from stdin (NEW STYLE)
 *
 *	Reads a line of text from the next device that has one ready. With some exceptions.
 *	Accepts CR or LF as line terminator. Replaces CR or LF with NUL in the returned string.
 *
 *	This function iterates over all open control and data devices, including reading from
 *	multiple control devices. It will also manage multiple data devices, but only one data
 *	device may be open at a time.
 *
 *	ARGS:
 *
 *	 type - Sets channel type to read. If TYPE_NONE will read either control or data.
 *			On return this variable is set to the channel type that was read, or
 *			TYPE_NONE if no text was returned.
 *
 *   size - Returns the size of the completed buffer, including NUL termination character.
 *			Lines may be returned truncated if the serial input from the physical device
 *			is longer than the read buffer for the device. The size value provided as a
 *			calling argument is not used (size doesn't matter).
 *
 *	 char_t * Returns a pointer to the buffer containing the line, or NULL (*0) if no text
 */

char_t *readline(devflags_t &flags, uint16_t &size)
{
	int c;

//	for (uint8_t dev=0; dev < DEV_MAX; dev++) {
	for (xio.dev=0; xio.dev < DEV_MAX; xio.dev++) {
		if (!isActive(xio.d[xio.dev])) continue;
//		if (!(xio.d[dev]->flags && flags)) continue;				// the types need to match
		if ((xio.d[xio.dev]->flags & flags) == false) continue;				// the types need to match

		while (xio.d[xio.dev]->read_index < xio.d[xio.dev]->read_buf_size) {
			if ((c = read_char(xio.dev)) == _FDEV_ERR) break;
			xio.d[xio.dev]->read_buf[xio.d[xio.dev]->read_index] = (char_t)c;
			if ((c == LF) || (c == CR)) {
				xio.d[xio.dev]->read_buf[xio.d[xio.dev]->read_index] = NUL;
				flags = xio.d[xio.dev]->flags;							// what type of device is this?
				size = xio.d[xio.dev]->read_index;						// how long is the string?
				xio.d[xio.dev]->read_index = 0;							// reset for next readline
				return (xio.d[xio.dev]->read_buf);
			}
			(xio.d[xio.dev]->read_index)++;
		}
	}
	size = 0;
	flags = 0;
	return (NULL);
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
