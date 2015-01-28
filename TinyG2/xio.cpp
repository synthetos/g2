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
#include "canonical_machine.h"
#include "xio.h"
#include "report.h"

/**** Structures ****/

// We need a buffer to hold single character commands, like !~%
// We also want it to have a NULL character, so we make it two characters.
//char_t single_char_buffer[2] = " ";

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
	bool isNotConnected() { return !(flags & DEV_IS_CONNECTED); }
    devflags_t getNextFlags() { //since next_flags is set from an interrupt, make this as atomic as possible
        devflags_t next = next_flags;
        next_flags = DEV_FLAGS_CLEAR;
        return next;
    }
	bool isReady() { return flags & DEV_IS_READY; }
	bool isActive() { return flags & DEV_IS_ACTIVE; }
};

struct xioDeviceWrapperBase {				// C++ base class for device primitives
	virtual int16_t readchar() = 0;			// Pure virtual. Will be subclassed for every device
    virtual void flushRead() = 0;
	virtual int16_t write(uint8_t *buffer, int16_t len) = 0;			// Pure virtual. Will be subclassed for every device
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

    virtual int16_t readchar() final {
		return _dev->readByte();					// readByte calls the USB endpoint's read function
	};
    
    virtual void flushRead() final {
        return _dev->flushRead();
    }

    virtual int16_t write(uint8_t *buffer, int16_t len) final {
        return _dev->write(buffer, len);
    }
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

void _xio_callback_helper(devflags_t next_flags, xioDevice_t *device, xioDeviceWrapperBase *wrapper);

stat_t xio_callback()
{
    devflags_t usb0state = USB0->getNextFlags(), usb1state = USB1->getNextFlags();
    _xio_callback_helper(usb0state, USB0, DeviceWrappers[DEV_USB0]);
    _xio_callback_helper(usb1state, USB1, DeviceWrappers[DEV_USB1]);
    return STAT_OK;
}

bool _others_connected(xioDevice_t *dev) {
    for(int8_t i = 0; i < DEV_MAX; ++i)
        if(xio.d[i] != dev && xio.d[i]->isConnected() && xio.d[i]->isActive())
            return true;
    return false;
}

void _xio_callback_helper(devflags_t next_flags, xioDevice_t *device, xioDeviceWrapperBase *wrapper)
{
    switch(next_flags) {
        case DEV_IS_CONNECTED:
            //USB0 has just connected
            //Case 1: USB0 is the first channel to connect -
            //  set it as CTRL+DATA+PRIMARY channel
            //Case 2: USB0 is the second (or later) channel to connect -
            //  set it as DATA channel, remove DATA flag from PRIMARY channel
            //... inactive channels are counted as closed

            device->flags |= (DEV_IS_CONNECTED | DEV_IS_READY);
            if(!_others_connected(device)) {
                // Case 1
                device->flags |= (DEV_IS_CTRL | DEV_IS_DATA | DEV_IS_PRIMARY | DEV_IS_ACTIVE);
            } else {
                // Case 2
                for(int8_t i = 0; i < DEV_MAX; ++i)
                    if((xio.d[i]->flags & DEV_IS_PRIMARY) == DEV_IS_PRIMARY)
                        xio.d[i]->flags &= ~(DEV_IS_DATA);
                device->flags |= (DEV_IS_DATA | DEV_IS_ACTIVE);
            }
            break;
        case DEV_IS_DISCONNECTED:
            //USB0 has just disconnected
            //Case 1: USB0 disconnected while it was a ctrl+data channel (and no other channels are open) -
            //  finalize USB0
            //Case 2: USB0 disconnected while it was a primary ctrl channel (and other channels are open) -
            //  finalize USB0, deactivate other channels
            //Case 3: USB0 disconnected while it was a non-primary ctrl channel (and other channels are open) -
            //  finalize USB0, leave other channels alone
            //Case 4: USB0 disconnected while it was a data channel (and other channels are open, including a primary)
            //  finalize USB0, set primary channel as a CTRL+DATA channel if this was the last data channel
            //Case 5: USB0 disconnected while it was inactive -
            //  don't need to do anything!
            //... inactive channels are counted as closed
            {
                devflags_t oldflags = device->flags;
                device->flags = DEV_FLAGS_CLEAR;
                wrapper->flushRead();

                if((oldflags & DEV_IS_ACTIVE) == 0) {
                    // Case 5
                } else if((oldflags & (DEV_IS_CTRL | DEV_IS_DATA)) == (DEV_IS_CTRL | DEV_IS_DATA) || !_others_connected(device)) {
                    // Case 1
                    if((oldflags & (DEV_IS_CTRL | DEV_IS_DATA)) != (DEV_IS_CTRL | DEV_IS_DATA) || _others_connected(device)) {
                        rpt_exception(STAT_XIO_ASSERTION_FAILURE, NULL);
                    }
                } else if((oldflags & (DEV_IS_CTRL | DEV_IS_PRIMARY)) == (DEV_IS_CTRL | DEV_IS_PRIMARY)) {
                    // Case 2
                    for(int8_t i = 0; i < DEV_MAX; ++i)
                        xio.d[i]->flags &= ~DEV_IS_ACTIVE;
                } else if((oldflags & (DEV_IS_CTRL)) == DEV_IS_CTRL) {
                    // Case 3
                } else if((oldflags & (DEV_IS_DATA)) == DEV_IS_DATA) {
                    // Case 4
                    int8_t i, dataCount = 0;
                    for(i = 0; i < DEV_MAX; ++i)
                        if((xio.d[i]->flags & (DEV_IS_DATA | DEV_IS_ACTIVE)) == (DEV_IS_DATA | DEV_IS_ACTIVE))
                            dataCount++;
                    if(dataCount == 0) {
                        for(i = 0; i < DEV_MAX; ++i)
                            if((xio.d[i]->flags & DEV_IS_PRIMARY) == DEV_IS_PRIMARY)
                                xio.d[i]->flags |= DEV_IS_DATA;
                    }
                }
            }
            break;

        default: break;
    }
}

/*
 * writeline() - write a terminate line of text to a device
 */

size_t writeline(uint8_t *buffer, size_t size)
{
    size_t written = -1;
    for(int8_t i = 0; i < DEV_MAX; ++i)
        if((xio.d[i]->flags & (DEV_IS_CTRL | DEV_IS_ACTIVE)) == (DEV_IS_CTRL | DEV_IS_ACTIVE))
            written = DeviceWrappers[i]->write(buffer, size);
    return written;
}

/*
 * read_char() - returns single char or -1 (_FDEV_ERR) is none available
 *               or _FDEV_OOB if it's a special char
 */

int read_char (uint8_t dev)
{
	int c = DeviceWrappers[dev]->readchar();

	if (c == (int)CHAR_RESET) {	 			// trap kill character
		hw_request_hard_reset();
		return (_FDEV_OOB);
	}
	if (c == (int)CHAR_FEEDHOLD) {			// trap feedhold character
		cm_request_feedhold();
		return (_FDEV_OOB);
	}
	if (c == (int)CHAR_QUEUE_FLUSH) {		// trap queue flush character
		cm_request_queue_flush();
		return (_FDEV_OOB);
	}
	if (c == (int)CHAR_CYCLE_START) {		// trap cycle start character
		cm_request_end_hold();
		return (_FDEV_OOB);
	}
	return (c);
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

char_t *readline(devflags_t *flags, uint16_t *size)
{
	int c;

	for (uint8_t dev=0; dev < DEV_MAX; dev++) {
		if (!xio.d[dev]->isActive())
			continue;

		// If this channel is a DATA & CONTROL, and flags ask for control-only, we skip it
		if (!(xio.d[dev]->flags & *flags)) // the types need to match
			continue;

		while (xio.d[dev]->read_index < xio.d[dev]->read_buf_size) {
            c = read_char(dev);
            if(c == _FDEV_ERR) break;
            else if(c == _FDEV_OOB) {
                do {
                    c = read_char(dev);
                } while(c == _FDEV_OOB);
                break;
            }
			xio.d[dev]->read_buf[xio.d[dev]->read_index] = (char_t)c;
//			if ((c == '!') || (c == '%') || (c == '~')) {
//                single_char_buffer[0] = c;
//                *size = 1;
//				*flags = xio.d[dev]->flags;							// what type of device is this?
//                return single_char_buffer;
//            } else 
			if ((c == LF) || (c == CR)) {							// terminate the line and return it
				xio.d[dev]->read_buf[xio.d[dev]->read_index] = NUL;
/*
				if (!(xio.d[dev]->flags & DEV_IS_DATA)) {
					// This is a control-only channel.
					// We need to ensure that we only get JSON-lines.
					if (xio.d[dev]->read_buf[0] != '{') {
						xio.d[dev]->read_index = 0; // throw away the line
						xio.d[dev]->read_buf[0] = 0;

						*size = 0;
						*flags = 0;
						return NULL;
					}
				}
*/
				*flags = xio.d[dev]->flags;							// what type of device is this?
				*size = xio.d[dev]->read_index+1;						// how long is the string? (include the 0xa, since that's what v8 does)
				xio.d[dev]->read_index = 0;							// reset for next readline
				return (xio.d[dev]->read_buf);
			}
			(xio.d[dev]->read_index)++;
		}
	}
	*size = 0;
	*flags = 0;
	return (NULL);
}

void xio_flush_device(devflags_t flags)
{
  for( uint8_t dev=0; dev < DEV_MAX; dev++) {
    if(!xio.d[dev]->isActive())
      continue;
    if(!(xio.d[dev]->flags & flags))
      continue;
    DeviceWrappers[dev]->flushRead();
  }
}

/*
char_t *readline(devflags_t &flags, uint16_t &size)
{
	int c;

	for (uint8_t dev=0; dev < DEV_MAX; dev++) {
		if (!xio.d[dev]->isActive())
		continue;

		// If this channel is a DATA & CONTROL, and flags ask for control-only, we skip it
		if (!(xio.d[dev]->flags & flags)) // the types need to match
		continue;

		while (xio.d[dev]->read_index < xio.d[dev]->read_buf_size) {
			if ((c = read_char(dev)) == _FDEV_ERR) break;
			xio.d[dev]->read_buf[xio.d[dev]->read_index] = (char_t)c;
			if ((c == '!') || (c == '%') || (c == '~')) {
				single_char_buffer[0] = c;
				size = 1;
				flags = xio.d[dev]->flags;							// what type of device is this?
				return single_char_buffer;
				} else if ((c == LF) || (c == CR)) {
				xio.d[dev]->read_buf[xio.d[dev]->read_index] = NUL;
				if (!(xio.d[dev]->flags & DEV_IS_DATA)) {
					// This is a control-only channel.
					// We need to ensure that we only get JSON-lines.
					if (xio.d[dev]->read_buf[0] != '{') {
						xio.d[dev]->read_index = 0; // throw away the line
						xio.d[dev]->read_buf[0] = 0;
						
						size = 0;
						flags = 0;
						return NULL;
					}
				}
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
*/

stat_t read_line (uint8_t *buffer, uint16_t *index, size_t size)
{
	if (*index >= size) { return (STAT_FILE_SIZE_EXCEEDED);}

	for (int c; *index < size; (*index)++ ) {
		if ((c = read_char(0)) != _FDEV_ERR) {
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
