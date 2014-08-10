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
 * the USB, SPI and file IO sub-systems, as well as providing low level character functions
 * used by stdio (printf()).
 *
 * NOTE: This file is specific to TinyG2/C++/ARM. The TinyG/C/Xmega file is completely different
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

//	bool canRead() { return caps & DEV_CAN_READ; }
//	bool canWrite() { return caps & DEV_CAN_WRITE; }
//	bool canBeCtrl() { return caps & DEV_CAN_BE_CTRL; }
//	bool canBeData() { return caps & DEV_CAN_BE_DATA; }
	bool isCtrl() { return flags & DEV_IS_CTRL; }			// called as: USB0->isCtrl()
	bool isData() { return flags & DEV_IS_DATA; }
	bool isPrimary() { return flags & DEV_IS_PRIMARY; }
	bool isConnected() { return flags & DEV_IS_CONNECTED; }
	bool isNotConnected() { return ~(flags & DEV_IS_CONNECTED); }
	bool isNextConnected() { return next_flags & DEV_IS_CONNECTED; }
	bool isNextDisconnected() { return next_flags & DEV_IS_DISCONNECTED; }
	bool isNextClear() { return next_flags == DEV_FLAGS_CLEAR; }
	bool isReady() { return flags & DEV_IS_READY; }
	bool isActive() { return flags & DEV_IS_ACTIVE; }

	void downCtrl() {
		flags &= ~(DEV_IS_CTRL | DEV_IS_CONNECTED | DEV_IS_READY | DEV_IS_ACTIVE);	// take down the channel
	}
};

struct xioDeviceWrapperBase {				// C++ base class for device primitives
	virtual int16_t readchar() = 0;			// Pure virtual. Will be subclassed for every device
//	virtual int16_t write() = 0;			// Pure virtual. Will be subclassed for every device
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

xioDevice_t _ds[DEV_MAX];			// allocate device structures

// Singleton acts and namespace for xio
struct xioSingleton_t {
	uint16_t magic_start;					// magic number to test memory integrity
	xioDevice_t* d[DEV_MAX];				// pointers to device structures
	uint8_t spi_state;						// tick down-counter (unscaled)
//	uint8_t dev;							// hack to make it visible to the debugger in optimized code.
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
 *	downData() - shuts down the active data channel
 */

void downData()
{
	// This function is currently naiive and does no finalization of the data channel
	// There is no allowance for buffered data (buffer clears, queue flushes, etc.)
	for (uint8_t dev=0; dev<DEV_MAX; dev++) {
		if (xio.d[dev]->flags && (DEV_IS_DATA | DEV_IS_ACTIVE)) {
			xio.d[dev]->flags &= ~(DEV_IS_DATA | DEV_IS_ACTIVE | DEV_IS_READY | DEV_IS_CONNECTED);	// take down the channel
			// insert serial queue flush here
		}
		return;
	}
}

/*
 * xio_init()
 *
 *	A lambda function closure is provided for trapping connection state changes from USB devices.
 *	The function is installed as a callback from the lower USB layers. It is called only on edges
 *	(connect/disconnect transitions). 'Connected' is true if the USB channel has just connected,
 *	false if it has just disconnected. It is only called on an edge — when it changes - so you
 *	shouldn't see two back-to-back connected=true calls with the same callback.
 *
 *	See here for some good info on lambda functions in C++
 *	http://www.cprogramming.com/c++11/c++11-lambda-closures.html
 */
void xio_init()
{
    xio_init_assertions();

	for (uint8_t dev=0; dev<DEV_MAX; dev++) {				// initialize pointers to device structures
		xio.d[dev] = &_ds[dev];
	}

	// setup for USBserial0
	SerialUSB.setConnectionCallback([&](bool connected) {	// lambda function
		USB0->next_flags = connected ? DEV_IS_CONNECTED : DEV_IS_DISCONNECTED;
	});
	USB0->read_buf_size = USB_LINE_BUFFER_SIZE;
	USB0->caps = (DEV_CAN_READ | DEV_CAN_WRITE | DEV_CAN_BE_CTRL | DEV_CAN_BE_DATA);

	// setup for USBserial1
	SerialUSB1.setConnectionCallback([&](bool connected) {
		USB1->next_flags = connected ? DEV_IS_CONNECTED : DEV_IS_DISCONNECTED;
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
	if ((xio.magic_start != MAGICNUM) || (xio.magic_end != MAGICNUM)) {
		return (STAT_XIO_ASSERTION_FAILURE);
	}
	return (STAT_OK);
}

/*
 * xio_callback() - callback from main loop for various IO functions
 *
 *	The USB channel binding functionality is located here. If this gets too big or there are other
 *	things to do during the callback it may make sense to break this out into a separate function.
 *
 *	Channel binding state machine (simple - does not do multiple control channels yet)
 *	  (0)	No connection
 *	  (1)	Single USB (CTRL+DATA)
 *	  (2)	Dual USB (CTRL, DATA)
 *	  (3)	Force disconnect of DATA channel (transient state)
 *
 *	Channel binding rules (start state --> end state)
 *	  (0-->0)	Initially all channels are disconnected. Channels are neither CTRL or DATA
 *	  (0-->1)	One of the USB serial channels connects. It becomes CTRL+DATA
 *	  (1-->2)	The other channel connects (maybe). It becomes DATA. The first becomes CTRL
 *
 *	Channel unbinding rules
 *	  (1-->0)	CTRL+DATA channel disconnects. No connection exists (what about a secondary CTRL channel?)
 *	  (2-->1)	DATA channel disconnects. The first CTRL channel reverts to being CTRL+DATA
 *	  (2-->3-->0) CTRL channel disconnects. If this is the primary CTRL channel all channels are
 *				  disconnected (including DATA). If it is not the primary CTRL channel then only it disconnects.
 *
 *	Note: We will not receive a second DEV_IS_CONNECTED on a channel that has already received one,
 *		  and similarly for DEV_NOT_CONNECTED. IOW we will only get valid state transitions w/no repeats.
 */
stat_t xio_callback()
{
	// exit the callback if there is no new state information to process
	if (USB0->isNextClear() && USB1->isNextClear()) {
		return (STAT_OK);
	}

	// USB0 has just connected
	// Case 1: USB0 is the 1st channel to connect
	// Case 2: USB0 is the 2nd channel to connect
	if (USB0->isNextConnected()) {
		USB0->next_flags = DEV_FLAGS_CLEAR;						// clear the next state condition
		USB0->flags |= (DEV_IS_CONNECTED | DEV_IS_READY);		// tanned, rested and ready

		if (USB1->isNotConnected()) {							// Case 1: USB0 is the 1st channel to connect
			USB0->flags |= (DEV_IS_CTRL | DEV_IS_DATA | DEV_IS_PRIMARY | DEV_IS_ACTIVE);
		} else {												// Case 2: USB1 is the 1st channel to connect
			downData();											// take down any current data channel (presumably USB1)
			USB0->flags |= (DEV_IS_DATA | DEV_IS_ACTIVE);		// assign USB0 to be the active data channel
		}
	} else

	// USB1 has just connected
	if (USB1->isNextConnected()) {
		USB1->next_flags = DEV_FLAGS_CLEAR;
		USB1->flags |= (DEV_IS_CONNECTED | DEV_IS_READY);
		if (USB0->isNotConnected()) {							// Case 1: USB1 is the 1st channel to connect
			USB1->flags |= (DEV_IS_CTRL | DEV_IS_DATA | DEV_IS_PRIMARY | DEV_IS_ACTIVE);
		} else {												// Case 2: USB1 is the 2nd channel to connect
			downData();
			USB0->flags |= (DEV_IS_DATA | DEV_IS_ACTIVE);
		}
	} else

	// USB0 has just disconnected
	// Case 1: USB0 disconnected while it was a ctrl+data channel
	// Case 2: USB0 disconnected while it was primary ctrl channel (also takes down active data channel)
	// Case 3: USB0 disconnected while it was non-primary ctrl channel (does not take down data channel)
	// Case 4: USB0 disconnected while it was a data channel
	if (USB0->isNextDisconnected()) {								// signals that USB0 just disconnected
		USB0->next_flags = DEV_FLAGS_CLEAR;
		if (USB0->flags && (DEV_IS_CTRL | DEV_IS_PRIMARY)) {		// Cases 1 & 2
			USB0->downCtrl();
			downData();
		} else if (USB0->isCtrl()) { USB0->downCtrl();				// Case 3
		} else if (USB0->isData()) { downData(); }					// Case 4
	} else

	// USB1 has just disconnected - same cases as USB0
	if (USB1->isNextDisconnected()) {
		USB1->next_flags = DEV_FLAGS_CLEAR;
		if (USB1->flags && (DEV_IS_CTRL | DEV_IS_PRIMARY)) {
			USB1->downCtrl();
			downData();
		} else if (USB1->isCtrl()) { USB1->downCtrl();
		} else if (USB1->isData()) { downData(); }
	}
	return (STAT_OK);
}

/*
 * writeline() - write a terminate line of text to a device
 */
size_t writeline(uint8_t *buffer, size_t size)
{
	size_t written = SerialUSB.write(buffer, size);
//	size_t written = SerialUSB1.write(buffer, size);
	return (written);
}

/*
 * read_char() - returns single char or -1 (_FDEV_ERR) is none available
 */
int read_char (uint8_t dev)
{
    return DeviceWrappers[dev]->readchar();
}

/*
 * readline() - read a complete line from a device
 *
 *	Reads a line of text from the next active device that has one ready. With some exceptions.
 *	Accepts CR or LF as line terminator. Replaces CR or LF with NUL in the returned string.
 *
 *	This function iterates over all active control and data devices, including reading from
 *	multiple control devices. It will also manage multiple data devices, but only one data
 *	device may be active at a time.
 *
 *	ARGS:
 *
 *	 flags - Bitfield containing the type of channel(s) to read. Looks at DEV_IS_CTRL and
 *			 DEV_IS_DATA bits in the device flag field. 'Flags' is loaded with the flags of
 *			 the channel that was read on return, or 0 (DEV_FLAGS_CLEAR) if no line was returned.
 *
 *   size -  Returns the size of the completed buffer, including the NUL termination character.
 *			 Lines may be returned truncated the the length of the serial input buffer if the text
 *			 from the physical device is longer than the read buffer for the device. The size value
 *			 provided as a calling argument is ignored (size doesn't matter).
 *
 *	 char_t * Returns a pointer to the buffer containing the line, or NULL (*0) if no text
 */

char_t *readline(devflags_t &flags, uint16_t &size)
{
	int c;

	for (uint8_t dev=0; dev < DEV_MAX; dev++) {
		if (!xio.d[dev]->isActive()) continue;
		if (!(xio.d[dev]->flags && flags)) continue;				// the types need to match

		while (xio.d[dev]->read_index < xio.d[dev]->read_buf_size) {
			if ((c = read_char(dev)) == _FDEV_ERR) break;
			xio.d[dev]->read_buf[xio.d[dev]->read_index] = (char_t)c;
			if ((c == LF) || (c == CR)) {
				xio.d[dev]->read_buf[xio.d[dev]->read_index] = NUL;
				flags = xio.d[dev]->flags;							// what type of device is this?
				size = xio.d[dev]->read_index;						// how long is the string?
				xio.d[dev]->read_index = 0;							// reset for next readline
				return (xio.d[dev]->read_buf);
			}
			(xio.d[dev]->read_index)++;
		}
	}
	size = 0;
	flags = 0;
	return (NULL);
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
