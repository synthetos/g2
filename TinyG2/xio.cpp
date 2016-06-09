/*
 * xio.cpp - extended IO functions
 * This file is part of the TinyG2 project
 *
 * Copyright (c) 2013 - 2016 Alden S. Hart Jr.
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
/*
 * XIO acts as an entry point into lower level IO routines - mostly serial IO. It supports
 * the USB, SPI and file IO sub-systems, as well as providing low level character functions
 * used by stdio (printf()).
 *
 * NOTE: This file is specific to TinyG2/C++/ARM. The TinyG/C/Xmega xio file is completely different
 */
#include "tinyg2.h"
#include "config.h"
#include "hardware.h"
#include "canonical_machine.h"  // needs cm_has_hold()
#include "xio.h"
#include "report.h"
#include "controller.h"
#include "util.h"

#include "board_xio.h"

#include "MotateBuffer.h"
using Motate::RXBuffer;
using Motate::TXBuffer;

#ifdef __TEXT_MODE
#include "text_parser.h"
#endif

/**** HIGH LEVEL EXPLANATION OF XIO ****
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
//    uint16_t read_index;					// index into line being read
//    const uint16_t read_buf_size;					// static variable set at init time
//    char read_buf[USB_LINE_BUFFER_SIZE];	// buffer for reading lines

    // Internal use only:
//    bool _ready_to_send;

    // Checks against calss flags variable:
//	bool canRead() { return caps & DEV_CAN_READ; }
//	bool canWrite() { return caps & DEV_CAN_WRITE; }
//	bool canBeCtrl() { return caps & DEV_CAN_BE_CTRL; }
//	bool canBeData() { return caps & DEV_CAN_BE_DATA; }
   	bool isAlwaysDataAndCtrl() { return caps & DEV_IS_ALWAYS_BOTH; }
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
    void setAsPrimaryActiveDualRole() {
        if (isAlwaysDataAndCtrl()) {
            flags |= (DEV_IS_CTRL | DEV_IS_DATA | DEV_IS_ACTIVE);
        } else {
            flags |= (DEV_IS_CTRL | DEV_IS_DATA | DEV_IS_PRIMARY | DEV_IS_ACTIVE);
        }
    };
    void setAsActiveData() { flags |= ( DEV_IS_DATA | DEV_IS_ACTIVE); };
    void clearFlags() { flags = DEV_FLAGS_CLEAR; }

    xioDeviceWrapperBase(uint8_t _caps) : caps(_caps),
                                          flags(DEV_FLAGS_CLEAR),
                                          next_flags(DEV_FLAGS_CLEAR)//,
//                                          read_index(0),
//                                          read_buf_size(USB_LINE_BUFFER_SIZE),
//                                          _ready_to_send(false)
    {
    };

    // Don't use pure virtuals! They massively slow down the calls.
    // But these MUST be overridden!
    virtual int16_t readchar() { return -1; };
    virtual void flushRead() {};       // This should call _flushLine() before flushing the device.
    virtual int16_t write(const char *buffer, int16_t len) { return -1; };

    virtual char *readline(devflags_t limit_flags, uint16_t &size) { return nullptr; };
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
            if((DeviceWrappers[i] != except) && (!DeviceWrappers[i]->isAlwaysDataAndCtrl()) && DeviceWrappers[i]->isConnected()) {
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
    size_t write(const char *buffer, size_t size)
    {
        // There are a few issues with this function that I don't know how to resolve right now:
        // 1) If a device fails to write the data, or all the data, then it's ignored
        // 2) Only the amount written by the *last* device to match (CTRL|ACTIVE) is returned.
        //
        // In the current environment, these are not foreseen to cause trouble,
        // since these are blocking writes and we expect to only really be writing to one device.

        size_t total_written = -1;
        for (int8_t i = 0; i < _dev_count; ++i) {
            if (DeviceWrappers[i]->isCtrlAndActive()) {
                const char *buf = buffer;
                int16_t to_write = size;
                while (to_write > 0) {
                    size_t written = DeviceWrappers[i]->write(buf, to_write);
                    buf += written;
                    to_write -= written;
                    total_written += written;
                }
            }
        }
        return total_written;
    }

    /*
     * writeline() - write a complete line to the controldevice
     *
     * The input buffer must be NUL terminated
     */

    int16_t writeline(const char *buffer)
    {
        int16_t len = strlen(buffer);
        return write(buffer, len);
    };

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


// LineRXBuffer takes the Motate RXBuffer (which handles "transfers", usually DMA), and adds G2 line-reading
// semantics to it.
template <uint16_t _size, typename owner_type, uint8_t _header_count = 8, uint16_t _line_buffer_size = 255>
struct LineRXBuffer : RXBuffer<_size, owner_type, char> { // reserve size of 128bytes
    typedef RXBuffer<_size, owner_type, char> parent_type;

    // SUPPORTING STRUCTURES
    enum _LinesHeaderStatus_t {
        FREE,
        PREPPED,
        FILLING,
        FULL
    };

    struct _LinesHeader {
        _LinesHeaderStatus_t _status    : 3; // use only three bits
        bool _is_control                : 1; // use a single bit
        bool _is_processing             : 1; // use a single bit

        bool : 0; // reset the bit field for proper alignment

        int16_t _line_count;   // number of lines in this group of lines
        int16_t _read_offset;  // start of the next line to read (first char past PROCESSING line)

        uint16_t _get_next_read_offset() { return ((_read_offset + 1) & (_size-1)); };


        // Convenience function for reset
        void reset() {
            _status = FREE;
            _is_control = false;
            _line_count = 0;
            _read_offset = 0;
        };
    };


    // Let's help the compiler clarify what we mean by a few things:
    // (This is because of the templating, it needs a little extra clarification.)
    using parent_type::_data;
    using parent_type::_getWriteOffset;
    using parent_type::_last_known_write_offset;
    using parent_type::_read_offset;
    using parent_type::isEmpty;
    using parent_type::_restartTransfer;
    using parent_type::_canBeRead;


    // START OF LineRXBuffer PROPER
    static_assert(((_header_count-1)&_header_count)==0, "_header_count must be 2^N");

    char _line_buffer[_line_buffer_size]; // hold excatly one line to return

    // General term usage:
    // * "index" indicates it's in to _headers array
    // * "offset" means it's a character in the _data array


    _LinesHeader _headers[_header_count];   // and array that we'll use as a ring buffer of _LinesHeaders
    uint8_t _first_header_index;            // index into the array of the (current) first item of _headers
    uint8_t _write_header_index;            // index into the array of the item of _headers we are writing too (unless it's FULL)

    uint16_t _scan_offset;          // offset into data of the last character scanned
    uint16_t _line_start_offset;    // offset into first character of the line
    bool     _at_start_of_line;     // true if the last character scanned was the end of a line

    LineRXBuffer(owner_type owner) : parent_type{owner} {};

    void init() { parent_type::init(); };

    uint8_t _get_next_write_header_index() { return ((_write_header_index + 1) & (_header_count-1)); };
    uint8_t _get_next_first_header_index() { return ((_first_header_index + 1) & (_header_count-1)); };
    uint8_t _get_next_header_index(uint8_t indx) { return ((indx + 1) & (_header_count-1)); };

    void _free_unused_space() {
        // Headers that were processing OR are now completely empty are now safe to clear,m
        // but only in order.
        auto _first_header = &_headers[_first_header_index];
        while ((_first_header->_is_processing) || ((_first_header->_status == FULL) && (_first_header->_line_count == 0))) {
            _read_offset = _first_header->_read_offset;
            _first_header->_is_processing = false;

            if ((_first_header->_status == FULL) && (_first_header->_line_count == 0)) {
                _first_header->reset();
                _first_header_index = _get_next_first_header_index();

                // Point _first_header to the new first header
                _first_header = &_headers[_first_header_index];
            } else {
                // We can only clear into the next one if we completely clear this one
                break;
            }
        }

        if ((_write_header_index == _first_header_index) && (_first_header->_status == FREE)) {
            // PREP it
            _first_header->_status = PREPPED;

            _at_start_of_line = true;
            _first_header->_read_offset = _line_start_offset;
        }
    };

    bool _check_write_header() {
        // _status cannot be PROCESSING, since we already cleared those in _free_unused_space()
        if (_headers[_write_header_index]._status == FULL) {
            uint8_t _next_write_header_index = _get_next_write_header_index();
            if (_next_write_header_index == _first_header_index) { // we're full full
                return false;
            }
            _write_header_index = _next_write_header_index;

            auto _write_header = &_headers[_write_header_index];
            _write_header->_status = PREPPED;
            _at_start_of_line = true;
            _write_header->_read_offset = _line_start_offset;
        }
        return true;
    };

    uint16_t _get_next_scan_offset() {
        return ((_scan_offset + 1) & (_size-1));
    }

    bool _is_more_to_scan() {
        return _canBeRead(_scan_offset);
    };

    // This function skips the "whitespace" at the BEGINNING of a line.
    // This assumes we've already located the end of a line.
    void _scan_past_line_start() {
        while (_is_more_to_scan()) {
            char c = _data[_scan_offset];
            if (c == '\r' ||
                c == '\n' ||
                c == '\t' ||
                c == ' ') {
                _scan_offset = _get_next_scan_offset();
            } else {
                break;
            }
        } // end _skipping whitespace
    };


    // Make a pass through the buffer to create headers for what has been read.

    // This function is designed to be able to exit from almost any point, and
    // come back in and resume where it left off. This allows it to scan to the
    // end of the buffer, then exit.
    void _scan_buffer() {
        _free_unused_space();

        if (!_check_write_header()) { return; }

        /* Explanation of cases and how we handle it.
         *
         * Our first task in this loop is two fold:
         *  A) Scan for the next complete line, then classify the line.
         *  B) Scan for a single-character command (!~% ^D, etc), then classify that as a line.
         *
         * Out next task is to then to manage the headers with this new information. We either:
         *  1) Add the line to the header, if the write header is FILLING and has the same classification.
         *  2) Add the line to the header, and classify it, if the write header is PENDING.
         *  3) Mark the current write header as FULL, and then (2) on the next one.
         *
         * We also have a constraint that we may run out of character at any time. This is okay, and enough 
         * state is kept that we can enter the function at any point with new characters and get the same results.
         *
         * Another constraint is that lines MAY have single character commands embedded in them. In this case,
         * we need to un-embed them. Since we may not have the end of the line yet, we need to move the command
         * to the beginning of the line.
         *
         * Note that _at_start_of_line means that we *just* parsed a character that is *at* the end of the line.
         * So, for a \r\n sequence, _at_start_of_line will go true of the \r, and we'll see the \n and it'll stay 
         * true, then the first non \r or \n char will set it to false, and *then* start the next line.
         *
         */
        while (_is_more_to_scan()) {
            auto write_header = &_headers[_write_header_index];

            bool ends_line  = false;
            bool is_control = false;

            // Look for line endings
            // Classify the line
            char c = _data[_scan_offset];
            if (c == '\r' ||
                c == '\n'
                ) {

                // We only mark ends_line for the first end-line char, and if
                // _at_start_of_line is already true, this is not the first.
                if (!_at_start_of_line) {
                    ends_line  = true;
                }
            }
            else
            if ((c == '!')         ||
                (c == '~')         ||
                (c == ENQ)         ||        // request ENQ/ack
                (c == CHAR_RESET)  ||        // ^X -  reset (aka cancel, terminate)
                (c == CHAR_ALARM)  ||        // ^D - request job kill (end of transmission)
                (cm_has_hold() && c == '%')  // flush (only in feedhold)
                ) {

                // Special case: if we're NOT _at_start_of_line, we need to move the
                // character to _line_start_offset. That means moving every character
                // forward one. THEN we back-track _scan_offset to this new start of line.
                if (!_at_start_of_line) {
                    uint16_t copy_offset = _line_start_offset;
                    while (copy_offset != _scan_offset) {
                        uint16_t next_copy_offset = (copy_offset+1)&(_size-1);
                        _data[next_copy_offset] = _data[copy_offset];
                        copy_offset = next_copy_offset;
                    }
                    // Copy the single character to the first character
                    _data[_line_start_offset] = c;
                }

                _line_start_offset = _scan_offset;

                // single-character control
                is_control = true;
                ends_line  = true;
            }
            else {
                if (_at_start_of_line) {
                    // This is the first character at the beginning of the line.
                    _line_start_offset = _scan_offset;
                }
                _at_start_of_line = false;
            }

            if (ends_line) {
                // Here we classify the line.
                // If we are is_control is already true, it's an already classified
                // single-character command.
                if (!is_control) {
                    // TODO --- Call a function to do this

                    if (_data[_line_start_offset] == '{') {
                        is_control = true;
                    }

                    // TODO ---
                }


                if (write_header->_status == PREPPED) {
                    write_header->_is_control = is_control;
                    write_header->_status = FILLING;
                }
                else if (write_header->_is_control != is_control)
                {
                    // This line goes into the next header.
                    // Now we have to end this _write_header and grab another.

                    write_header->_status = FULL;
                    if (!_check_write_header()) {
                        // We just bail if there's not another header available.
                        return;
                    }

                    // Update the pointer
                    write_header = &_headers[_write_header_index];
                    write_header->_is_control = is_control;
                    write_header->_status = FILLING;
                }

                write_header->_line_count++;
                _at_start_of_line = true;
            }

            // We do this LAST. If we had to exit before this point,
            // we will evaluate the same character again.
            _scan_offset = _get_next_scan_offset();
        } //while (_is_more_to_scan())
    };



    // This is the ONLY external interface in this class
    char *readline(bool control_only, uint16_t &line_size) {
        _scan_buffer();


        uint8_t search_header_index = _first_header_index;
        auto search_header = &_headers[search_header_index];
        bool found_control = false;
        do {
            if ((search_header->_status >= FILLING) &&
                search_header->_is_control &&
                (search_header->_line_count > 0)) {

                found_control = true;
                break;
            }
            if (search_header_index == _write_header_index)
            {
                break;
            }

            search_header_index = _get_next_header_index(search_header_index);
            search_header = &_headers[search_header_index];
        } while (1);


        if (found_control) {

            // When we get here, search_header points to a valid header that we want to either:
            // A) Get a single-character command from and return it, OR
            // B) Get a full line from and return it.

            // For B, we handle that like any line. But the single chars have to have special
            // attention.


            char c = _data[search_header->_read_offset];
            if ((c == '!')         ||
                (c == '~')         ||
                (c == ENQ)         ||        // request ENQ/ack
                (c == CHAR_RESET)  ||        // ^X -  reset (aka cancel, terminate)
                (c == CHAR_ALARM)  ||        // ^D - request job kill (end of transmission)
                (cm_has_hold() && c == '%')  // flush (only in feedhold)
                ) {
                line_size = 1;

                search_header->_read_offset = search_header->_get_next_read_offset();
                search_header->_line_count--;
                search_header->_is_processing = true;

                single_char_buffer[0] = c;
                single_char_buffer[1] = 0;

                return single_char_buffer;
            }

            // fall through to finding the end of the line in search_header


        } // end if (found_command)
        else {

            // Logic to determine that we can safely look at ONLY the first header:
            // • We always read all of the command lines first.
            // • We have already called _free_unused_space().

            search_header_index = _first_header_index;
            search_header = &_headers[search_header_index];

            if (control_only || search_header->_is_control || (search_header->_line_count == 0)) {
                line_size = 0;
                return nullptr;
            }

        } // end if (!found_command)



        // By the time we get here, search_header points to a valid header that we want to pull the first
        // full line from and return it.


        // We know we have at least one line in the _data buffer, starting at search_header->_read_offset
        // The line might "wrap" to the beginning of the ring buffer, however, and we don't want to pass
        // back a pointer to *that*, so we detect a wrapped line and copy the characters that are on the
        // other side of the "fold" to the extra space at the end of the buffer (which is there for just
        // this reason.)

        uint16_t read_offset = search_header->_read_offset;
        char *dst_ptr = _line_buffer;
        line_size = 0;

        // scan past any leftover CR or LF from the previous line
        while ((_data[read_offset] == '\n') || (_data[read_offset] == '\r')) {
            read_offset = (read_offset+1)&(_size-1);
        }

        while (line_size < (_line_buffer_size - 2)) {
//            if (!_canBeRead(read_offset)) { // This test should NEVER fail.
//                _debug_trap("readline hit unreadable and shouldn't have!");
//            }
            char c = _data[read_offset];

            if (
                 c == '\r' ||
                 c == '\n'
                ) {

                break;
            }

            line_size++;
            *dst_ptr = c;

            // update read/write positions
            dst_ptr++;
            read_offset = (read_offset+1)&(_size-1);
        }

        // previous character was last one of the line
        // update the header's next read position
        search_header->_read_offset = (read_offset+1)&(_size-1);
        // and line count
        search_header->_line_count--;
        // and processing flag
        search_header->_is_processing = true;

        *dst_ptr = 0;
        return _line_buffer;
    };
};


template<typename Device>
struct xioDeviceWrapper : xioDeviceWrapperBase {	// describes a device for reading and writing
    Device _dev;
    LineRXBuffer<512, Device> _rx_buffer;
    TXBuffer<512, Device> _tx_buffer;

    xioDeviceWrapper(Device dev, uint8_t _caps) : xioDeviceWrapperBase(_caps), _dev{dev}, _rx_buffer{_dev}, _tx_buffer{_dev}
    {
//        _dev->setDataAvailableCallback([&](const size_t &length) {
//            
//        });
    };

    void init() {
        _dev->setConnectionCallback([&](bool connected) {	// lambda function
            connectedStateChanged(connected);
        });

        _rx_buffer.init();
        _tx_buffer.init();
    };

    virtual int16_t readchar() final {
        return _rx_buffer.read();
//        return _dev->readByte();					// readByte calls the USB endpoint's read function
    };

    virtual void flushRead() final {
        // Flush out any partially or wholly read lines being stored:
        _rx_buffer.flush();
        _flushLine();
        return _dev->flushRead();
    }

    virtual int16_t write(const char *buffer, int16_t len) final {
        return _tx_buffer.write(buffer, len);
    }

    virtual char *readline(devflags_t limit_flags, uint16_t &size) final {
        if (!(limit_flags & flags)) {
            size = 0;
            return NULL;
        }

        return _rx_buffer.readline(!(limit_flags & DEV_IS_DATA), size);
    };

    void _flushLine() {

    };

    void connectedStateChanged(bool connected) {
            if (connected) {
                if (isNotConnected()) {
                    //USB0 or UART has just connected
                    //Case 1: This is the first channel to connect -
                    //  set it as CTRL+DATA+PRIMARY channel
                    //Case 2: This is the second (or later) channel to connect -
                    //  set it as DATA channel, remove DATA flag from PRIMARY channel
                    //... inactive channels are counted as closed

                    setAsConnectedAndReady();

                    if (isAlwaysDataAndCtrl()) {
                        return;
                    }

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
                    //Case 5a: This channel disconnected while it was inactive
                    //Case 5b: This channel disconnected when it's always present
                    //  don't need to do anything!
                    //... inactive channels are counted as closed

                    devflags_t oldflags = flags;
                    clearFlags();
                    flushRead();

                    if(checkForNotActive(oldflags) || isAlwaysDataAndCtrl()) {
                        // Case 5a, 5b
                    } else if(checkForCtrlAndData(oldflags) || !xio.others_connected(this)) {
                        // Case 1
                        if(!checkForCtrlAndData(oldflags) || xio.others_connected(this)) {
                            rpt_exception(STAT_XIO_ASSERTION_FAILURE, "xio_dev() assertion error"); // where is this supposed to go!?
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
    };
    };

// ALLOCATIONS
// Declare a device wrapper class for SerialUSB and SerialUSB1
#if XIO_HAS_USB == 1
xioDeviceWrapper<decltype(&SerialUSB)> serialUSB0Wrapper {
    &SerialUSB,
    (DEV_CAN_READ | DEV_CAN_WRITE | DEV_CAN_BE_CTRL | DEV_CAN_BE_DATA)
};
xioDeviceWrapper<decltype(&SerialUSB1)> serialUSB1Wrapper {
    &SerialUSB1,
    (DEV_CAN_READ | DEV_CAN_WRITE | DEV_CAN_BE_CTRL | DEV_CAN_BE_DATA)
};
#endif // XIO_HAS_USB
#if XIO_HAS_UART==1
xioDeviceWrapper<decltype(&Serial)> serial0Wrapper {
    &Serial,
    (DEV_CAN_READ | DEV_CAN_WRITE | DEV_IS_ALWAYS_BOTH)
};
#endif // XIO_HAS_UART

// Define the xio singleton (and initialize it to hold our two deviceWrappers)
//xio_t xio = { &serialUSB0Wrapper, &serialUSB1Wrapper };
xio_t xio = {
#if XIO_HAS_USB == 1
    &serialUSB0Wrapper,
    &serialUSB1Wrapper,
#endif // XIO_HAS_USB
#if XIO_HAS_UART == 1
    &serial0Wrapper
#endif
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
    board_xio_init();

#if XIO_HAS_USB == 1
    serialUSB0Wrapper.init();
    serialUSB1Wrapper.init();
#endif
#if XIO_HAS_UART == 1
    serial0Wrapper.init();
#endif
}

stat_t xio_test_assertions()
{
    if ((BAD_MAGIC(xio.magic_start)) || (BAD_MAGIC(xio.magic_end))) {
        return(cm_panic(STAT_XIO_ASSERTION_FAILURE, "xio_test_assertions()"));
    }
    return (STAT_OK);
}

/*
 * write() - write a buffer to a device
 */

size_t xio_write(const char *buffer, size_t size)
{
    return xio.write(buffer, size);
}

/*
 * xio_readline() - read a complete line from a device
 * xio_writeline() - write a complete line to control device
 * xio_flush_read() - flush read buffers
 *
 *	Defers to xio.readline(), etc.
 */

char *xio_readline(devflags_t &flags, uint16_t &size)
{
    return xio.readline(flags, size);
}

int16_t xio_writeline(const char *buffer)
{
    return xio.writeline(buffer);
}

void xio_flush_read()
{
    return xio.flushRead();
}


/***********************************************************************************
 * newlib-nano support functions
 * Here we wire printf to xio
 ***********************************************************************************/

int _write( int file, char *ptr, int len )
{
    return xio_write(ptr, len);
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
