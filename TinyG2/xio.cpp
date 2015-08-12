/*
 * xio.cpp - extended IO functions
 * This file is part of the TinyG2 project
 *
 * Copyright (c) 2013 - 2015 Alden S. Hart Jr.
 * Copyright (c) 2013 - 2015 Robert Giseburt
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
#include "hardware.h"
#include "canonical_machine.h"  // needs cm_has_hold()
#include "xio.h"
#include "report.h"
#include "controller.h"
#include "util.h"

#ifdef __TEXT_MODE
#include "text_parser.h"
#endif

using namespace Motate;
//OutputPin<kDebug1_PinNumber> xio_debug_pin1;
//OutputPin<kDebug2_PinNumber> xio_debug_pin2;
//OutputPin<-1> xio_debug_pin2;
//OutputPin<kDebug3_PinNumber> xio_debug_pin3;


/*
 **** HIGH LEVEL EXPLANATION OF XIO ****
 *
 * The XIO subsystem serves three purposes:
 *  1) Handle the connection states of various IO channels (USB for now)
 *  2) Marshall reads/writes/etc from the rest of the system to/from the managed channels
 *  3) Maintain buffers for line-based reading on devices.
 *
 * We have three object types: xioDeviceWrapperBase, xioDeviceWrapper<Device>, and xio_t.
 *
 * xioDeviceWrapperBase  --  is an abstract base class object that manages and provides access to:
 *   *) the line read buffer and state
 *   *) the state machine for a single device
 *   *) pure-virtual functions for read/write/flush (to override later)
 *   *) a readline implementation that is device agnostic
 *
 * xioDeviceWrapper<Device> -- is a concrete template-specialized child of xioDeviceWrapperBase:
 *   *) Wraps any "device" that supports readchar(), flushRead(), and write(const uint8_t *buffer, int16_t len)
 *   *) Calls the device's setConnectionCallback() on construction, and contains the connection state machine
 *   *) Calls into the xio singleton for multi-device checks. (This mildly complicates the order that we define
 *      these structures, since they depend on each other.)
 *   *) Calls controller_set_connected() to inform the higher system when the first device has connected and
 *      the last device has disconnected.
 *
 * xio_t -- the class used by the xio singleton
 *   *) Contains the array of xioDeviceWrapperBase pointers.
 *   *) Handles system-wide readline(), write(), and flushRead()
 *   *) Handles making cross-device checks and changes for the state machine.
 *
 ***************************************/

/**** Structures ****/

// We need a buffer to hold single character commands, like !~%
// We also want it to have a NULL character, so we make it two characters.
char single_char_buffer[2] = " ";

// Checks against arbitrary flags variable (passed in)
// Prefer to use the object is*() methods over these.
bool checkForCtrl(devflags_t flags_to_check) { return flags_to_check & DEV_IS_CTRL; }
bool checkForCtrlOnly(devflags_t flags_to_check) { return (flags_to_check & (DEV_IS_CTRL|DEV_IS_DATA)) == DEV_IS_CTRL; }
bool checkForData(devflags_t flags_to_check) { return flags_to_check & DEV_IS_DATA; }
bool checkForNotActive(devflags_t flags_to_check) { return !(flags_to_check & DEV_IS_ACTIVE); }

bool checkForCtrlAndData(devflags_t flags_to_check) { return (flags_to_check & (DEV_IS_CTRL|DEV_IS_DATA)) == (DEV_IS_CTRL|DEV_IS_DATA); }
bool checkForCtrlAndPrimary(devflags_t flags_to_check) { return (flags_to_check & (DEV_IS_CTRL|DEV_IS_PRIMARY)) == (DEV_IS_CTRL|DEV_IS_PRIMARY); }


struct xioDeviceWrapperBase {				// C++ base class for device primitives
    // connection and device management
    uint8_t caps;							// bitfield for capabilities flags (these are persistent)
    devflags_t flags;						// bitfield for device state flags (these are not)
    devflags_t next_flags;					// bitfield for next-state transitions

    // line reader functions
    uint16_t read_index;					// index into line being read
    uint16_t read_buf_size;					// static variable set at init time
    char read_buf[USB_LINE_BUFFER_SIZE];	// buffer for reading lines

    // Internal use only:
    bool _ready_to_send;

    // Checks against calss flags variable:
//	bool canRead() { return caps & DEV_CAN_READ; }
//	bool canWrite() { return caps & DEV_CAN_WRITE; }
//	bool canBeCtrl() { return caps & DEV_CAN_BE_CTRL; }
//	bool canBeData() { return caps & DEV_CAN_BE_DATA; }
    bool isCtrl() { return flags & DEV_IS_CTRL; }    // called externally:      DeviceWrappers[i]->isCtrl()
    bool isData() { return flags & DEV_IS_DATA; }    // subclasses can call directly (no pointer): isCtrl()
    bool isPrimary() { return flags & DEV_IS_PRIMARY; }
    bool isConnected() { return flags & DEV_IS_CONNECTED; }
    bool isNotConnected() { return !(flags & DEV_IS_CONNECTED); }
    bool isReady() { return flags & DEV_IS_READY; }
    bool isActive() { return flags & DEV_IS_ACTIVE; }

    // Combination checks
    bool isCtrlAndActive() { return ((flags & (DEV_IS_CTRL|DEV_IS_ACTIVE)) == (DEV_IS_CTRL|DEV_IS_ACTIVE)); }
    bool isDataAndActive() { return ((flags & (DEV_IS_DATA|DEV_IS_ACTIVE)) == (DEV_IS_DATA|DEV_IS_ACTIVE)); }

    bool isNotCtrlOnly() { return ((flags & (DEV_IS_CTRL|DEV_IS_DATA)) != (DEV_IS_CTRL)); }

    // Manipulation functions
    void setData() { flags |= DEV_IS_DATA; }
    void clearData() { flags &= ~DEV_IS_DATA; }

    void setActive() { flags |= DEV_IS_ACTIVE; }
    void clearActive() { flags &= DEV_IS_ACTIVE; }

    void setPrimary() { flags |= DEV_IS_PRIMARY; }
    void clearPrimary() { flags &= ~DEV_IS_PRIMARY; }

    void setAsConnectedAndReady() { flags |= ( DEV_IS_CONNECTED | DEV_IS_READY); };
    void setAsPrimaryActiveDualRole() { flags |= (DEV_IS_CTRL | DEV_IS_DATA | DEV_IS_PRIMARY | DEV_IS_ACTIVE); };
    void setAsActiveData() { flags |= ( DEV_IS_DATA | DEV_IS_ACTIVE); };
    void clearFlags() { flags = DEV_FLAGS_CLEAR; }

    xioDeviceWrapperBase(uint8_t _caps) : caps(_caps),
                                          flags(DEV_FLAGS_CLEAR),
                                          next_flags(DEV_FLAGS_CLEAR),
                                          read_index(0),
                                          read_buf_size(USB_LINE_BUFFER_SIZE),
                                          _ready_to_send(false) {
    };

    // Pure virtuals. MUST be subclassed for every device -- even if they don't apply.
    virtual int16_t readchar() = 0;
    virtual void flushRead() = 0;       // This should call _flushLine() before flushing the device.
    virtual int16_t write(const uint8_t *buffer, int16_t len) = 0;


    // Readline and line flushing functions
    char *readline(devflags_t limit_flags, uint16_t &size) {
        if (!(limit_flags & flags)) {
        	size = 0;
        	return NULL;
        }

        // If _ready_to_send is true, we captured a line previously but couldn't return it yet (one of various reasons),
        // and we don't actually need to read from the channel. We just need to try to return it again.
        if (!_ready_to_send) {
            while (read_index < read_buf_size) {
                int c;

                if ((c = readchar()) == _FDEV_ERR) {
                    break;
                }
                read_buf[read_index] = (char)c;

                // special handling for flush character
                // if not in a feedhold substitute % with ; so it's treated as a comment and ignored.
                // if in a feedhold request a queue flush by passing the % back as a single character.
                if (c == '%') {
                    if (!cm_has_hold()) {
                        read_buf[read_index++] = ';';
                        continue;
                    } else {
                        single_char_buffer[0] = '%';    // send queue flush request
                        size = 1;
                        return single_char_buffer;
                    }
                }

                // trap other special characters
                if ((c == '!') ||                       // request feedhold
                    (c == '~') ||                       // request end feedhold
                    (c == EOT) ||                       // request job kill (end of transmission)
                    (c == CAN)) {                       // reset (aka cancel, terminate)
                    single_char_buffer[0] = c;
                    size = 1;
                    return single_char_buffer;

                } else if ((c == LF) || (c == CR)) {
//                } else if ((c == LF) || (c == CR) || (c == TAB)) {  // use this to add TAB as a line terminator
                    _ready_to_send = true;
                    break;
                }
                read_index++;
            }
        }

        // Now we have a complete line to send, check it and (maybe) return it.
        if (_ready_to_send) {
            if (!(limit_flags & DEV_IS_DATA)) {
                // This is a control-only read.
                // We need to ensure that we only get JSON-lines.
                // CHEAT: We don't properly ignore spaces here!!
                if ((read_buf[0] != '{') && (read_buf[0] != CR) && (read_buf[0] != LF)) {
                    // we'll just leave _ready_to_send set, and next time it can be read.
                    size = 0;
                    return NULL;
                }
            }

            // Here is where we would do more checks to make sure we're allowing the correct data through the correct channel.
            // For now, we only do that one test.

            read_buf[read_index] = NUL;
            size = read_index;						// how long is the string?
            read_index = 0;							// reset for next readline

            _ready_to_send = false;

            return (read_buf);
        }
        size = 0;
        return NULL;
    };

    void _flushLine() {
        _ready_to_send = false;
        read_index = 0;
    };
};

// Here we create the xio_t class, which has convenience methods to handle cross-device actions as a whole.
struct xio_t {
    uint16_t magic_start;

    xioDeviceWrapperBase* DeviceWrappers[DEV_MAX];
    const uint8_t _dev_count;

    template<typename... ds>
    xio_t(ds... args) : magic_start(MAGICNUM), DeviceWrappers {args...}, _dev_count(sizeof...(args)), magic_end(MAGICNUM) {

    };

    // ##### Connection management functions

    bool others_connected(xioDeviceWrapperBase* except) {
        for (int8_t i = 0; i < _dev_count; ++i) {
            if((DeviceWrappers[i] != except) && DeviceWrappers[i]->isConnected()) {
                return true;
            }
        }

        return false;
    };

    void remove_data_from_primary() {
        for (int8_t i = 0; i < _dev_count; ++i) {
            if (DeviceWrappers[i]->isDataAndActive()) {
                return;
            }
        }

        for (int8_t i = 0; i < _dev_count; ++i) {
            if (DeviceWrappers[i]->isPrimary()) {
                DeviceWrappers[i]->clearData();
            }
        }

    };

    void deactivate_all_channels() {
        for(int8_t i = 0; i < _dev_count; ++i) {
            DeviceWrappers[i]->clearActive();
        }
    };

    // ##### Cross-Device read/write/etc. functions

    /*
     * write() - write a block to a device
     */
    size_t write(const uint8_t *buffer, size_t size)
    {
        // There are a few issues with this function that I don't know how to resolve right now:
        // 1) If a device fails to write the data, or all the data, then it's ignored
        // 2) Only the amount written by the *last* device to match (CTRL|ACTIVE) is returned.
        //
        // In the current environment, these are not forssen to cause trouble, since these are blocking writes
        // and we expect to only really be writing to one device.

        size_t written = -1;

        for (int8_t i = 0; i < _dev_count; ++i) {
            if (DeviceWrappers[i]->isCtrlAndActive()) {
                written = DeviceWrappers[i]->write(buffer, size);
            }
        }
        return written;
    }


    /*
     * flushRead() - flush all readable devices' read buffers
     */
    void flushRead()
    {
        for (int8_t i = 0; i < _dev_count; ++i) {
            DeviceWrappers[i]->flushRead();
        }
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
     *			 Lines may be returned truncated to the length of the serial input buffer if the text
     *			 from the physical device is longer than the read buffer for the device. The size value
     *			 provided as a calling argument is ignored (size doesn't matter).
     *
     *	 char * Returns a pointer to the buffer containing the line, or NULL (*0) if no text
     */
    char *readline(devflags_t &flags, uint16_t &size)
    {
        char *ret_buffer;
        devflags_t limit_flags = flags; // Store it so it can't get mangled

        // Always check control-capable devices FIRST
        for (uint8_t dev=0; dev < _dev_count; dev++) {
            if (!DeviceWrappers[dev]->isActive())
                continue;

            // If this channel is a DATA only, skip it this pass
            if (!DeviceWrappers[dev]->isCtrl())
                continue;

            ret_buffer = DeviceWrappers[dev]->readline(DEV_IS_CTRL, size);

            if (size > 0) {
                flags = DeviceWrappers[dev]->flags;

                return ret_buffer;
            }
        }

        // We only do this second pass if this is not a CTRL-only read
        if (!checkForCtrlOnly(limit_flags)) {
            for (uint8_t dev=0; dev < _dev_count; dev++) {
                if (!DeviceWrappers[dev]->isActive())
                    continue;

                ret_buffer = DeviceWrappers[dev]->readline(limit_flags, size);

                if (size > 0) {
                    flags = DeviceWrappers[dev]->flags;

                    return ret_buffer;
                }
            }
        }

        size = 0;
        flags = 0;

        return (NULL);
    };

    uint16_t magic_end;
};

// Declare (but don't define) the xio singelton object now, define it later
// Why? xioDeviceWrapper uses it, but we need to define it to contain xioDeviceWrapper objects.
extern xio_t xio;

// Use a templated subclass so we don't have to create a new subclass for every type of Device.
// All this requires is the readByte() function to exist in type Device.
// Note that we expect Device to be a pointer type!
// See here for a discussion of what this means if you are not familiar with C++
// https://github.com/synthetos/g2/wiki/Dual-Endpoint-USB-Internals#c-classes-virtual-functions-and-inheritance

template<typename Device>
struct xioDeviceWrapper : xioDeviceWrapperBase {	// describes a device for reading and writing
    Device _dev;
    xioDeviceWrapper(Device dev, uint8_t _caps) : xioDeviceWrapperBase(_caps), _dev{dev}
    {
        _dev->setConnectionCallback([&](bool connected) {	// lambda function
            if (connected) {
                if (isNotConnected()) {
                    //USB0 has just connected
                    //Case 1: This is the first channel to connect -
                    //  set it as CTRL+DATA+PRIMARY channel
                    //Case 2: This is the second (or later) channel to connect -
                    //  set it as DATA channel, remove DATA flag from PRIMARY channel
                    //... inactive channels are counted as closed

                    setAsConnectedAndReady();

                    if(!xio.others_connected(this)) {
                        // Case 1
                        setAsPrimaryActiveDualRole();
                        // report that we now have a connection (only for the first one)
                        controller_set_connected(true);
                    } else {
                        // Case 2
                        xio.remove_data_from_primary();
                        setAsActiveData();
                    }
                } // flags & DEV_IS_DISCONNECTED

            } else { // disconnected
                if (isConnected()) {

                    //USB0 has just disconnected
                    //Case 1: This channel disconnected while it was a ctrl+data channel (and no other channels are open) -
                    //  finalize this channel
                    //Case 2: This channel disconnected while it was a primary ctrl channel (and other channels are open) -
                    //  finalize this channel, deactivate other channels
                    //Case 3: This channel disconnected while it was a non-primary ctrl channel (and other channels are open) -
                    //  finalize this channel, leave other channels alone
                    //Case 4: This channel disconnected while it was a data channel (and other channels are open, including a primary)
                    //  finalize this channel, set primary channel as a CTRL+DATA channel if this was the last data channel
                    //Case 5: This channel disconnected while it was inactive -
                    //  don't need to do anything!
                    //... inactive channels are counted as closed

                    devflags_t oldflags = flags;
                    clearFlags();
                    flushRead();

                    if(checkForNotActive(oldflags)) {
                        // Case 5
                    } else if(checkForCtrlAndData(oldflags) || !xio.others_connected(this)) {
                        // Case 1
                        if(!checkForCtrlAndData(oldflags) || xio.others_connected(this)) {
                            rpt_exception(STAT_XIO_ASSERTION_FAILURE, "xio_dev"); // where is this supposed to go!?
                        }
                        controller_set_connected(false);
                    } else if(checkForCtrlAndPrimary(oldflags)) {
                        // Case 2
                        xio.deactivate_all_channels();
                    } else if(checkForCtrl(oldflags)) {
                        // Case 3
                    } else if(checkForData(oldflags)) {
                        // Case 4
                        xio.remove_data_from_primary();
                    }
                } // flags & DEV_IS_CONNECTED
            }
        });
    };

    virtual int16_t readchar() final {
        return _dev->readByte();					// readByte calls the USB endpoint's read function
    };

    virtual void flushRead() final {
        // FLush out any partially or wholly read lines being stored:
        _flushLine();
        return _dev->flushRead();
    }

    virtual int16_t write(const uint8_t *buffer, int16_t len) final {
        return _dev->write(buffer, len);
    }
};

// ALLOCATIONS
// Declare a device wrapper class for SerialUSB and SerialUSB1
xioDeviceWrapper<decltype(&SerialUSB)> serialUSB0Wrapper {
    &SerialUSB,
    (DEV_CAN_READ | DEV_CAN_WRITE | DEV_CAN_BE_CTRL | DEV_CAN_BE_DATA)
};
xioDeviceWrapper<decltype(&SerialUSB1)> serialUSB1Wrapper {
    &SerialUSB1,
    (DEV_CAN_READ | DEV_CAN_WRITE | DEV_CAN_BE_CTRL | DEV_CAN_BE_DATA)
};

// Define the xio singleton (and initialize it to hold our two deviceWrappers)
//xio_t xio = { &serialUSB0Wrapper, &serialUSB1Wrapper };
xio_t xio = {
    &serialUSB0Wrapper,
    &serialUSB1Wrapper
};

/**** CODE ****/

/*
 * xio_init()
 *
 *	A lambda function closure is provided for trapping connection state changes from USB devices.
 *	The function is installed as a callback from the lower USB layers. It is called only on edges
 *	(connect/disconnect transitions). 'Connected' is true if the USB channel has just connected,
 *	false if it has just disconnected. It is only called on an edge � when it changes - so you
 *	shouldn't see two back-to-back connected=true calls with the same callback.
 *
 *	See here for some good info on lambda functions in C++
 *	http://www.cprogramming.com/c++11/c++11-lambda-closures.html
 */

void xio_init()
{
    // Nothing left to do here, move along...
}

stat_t xio_test_assertions()
{
    if ((BAD_MAGIC(xio.magic_start)) || (BAD_MAGIC(xio.magic_end))) {
        return(cm_panic(STAT_XIO_ASSERTION_FAILURE, "xio  magic numbers"));
    }
    return (STAT_OK);
}

/*
 * write() - write a buffer to a device
 */

size_t xio_write(const uint8_t *buffer, size_t size)
{
    return xio.write(buffer, size);
}

/*
 * readline() - read a complete line from a device
 *
 *	Defers to xio.readline().
 */

char *xio_readline(devflags_t &flags, uint16_t &size)
{
    return xio.readline(flags, size);
}

void xio_flush_read()
{
    return xio.flushRead();
}

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/*
 * xio_set_spi() = 0=disable, 1=enable
 */
//stat_t xio_set_spi(nvObj_t *nv)
//{
//	xio.spi_state = (uint8_t)nv->value;
//
//#ifdef __ARM
//	if (fp_EQ(nv->value, SPI_ENABLE)) {
//		spi_miso_pin.setMode(kOutput);
//		spi_mosi_pin.setMode(kOutput);
//		spi_sck_pin.setMode(kOutput);
//
//    } else if (fp_EQ(nv->value, SPI_DISABLE)) {
//		spi_miso_pin.setMode(kInput);
//		spi_mosi_pin.setMode(kInput);
//		spi_sck_pin.setMode(kInput);
//	}
//#endif
//	return (STAT_OK);
//}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

static const char fmt_spi[] PROGMEM = "[spi] SPI state%20d [0=disabled,1=enabled]\n";
void xio_print_spi(nvObj_t *nv) { text_print(nv, fmt_spi);} // TYPE_INT

#endif // __TEXT_MODE
