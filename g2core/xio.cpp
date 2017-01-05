/*
 * xio.cpp - extended IO functions
 * This file is part of the g2core project
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
 */
#include "g2core.h"
#include "config.h"
#include "hardware.h"
#include "canonical_machine.h"  // needs cm_has_hold()
#include "xio.h"
#include "report.h"
#include "controller.h"
#include "util.h"
#include "settings.h"

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

// We need a buffer to hold single character commands, like !~%, ^x, etc.
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


struct xioDeviceWrapperBase {                // C++ base class for device primitives
    // connection and device management
    uint8_t caps;                            // bitfield for capabilities flags (these are persistent)
    devflags_t flags;                        // bitfield for device state flags (these are not)
    devflags_t next_flags;                   // bitfield for next-state transitions

    // line reader functions
//    uint16_t read_index;                    // index into line being read
//    const uint16_t read_buf_size;           // static variable set at init time
//    char read_buf[USB_LINE_BUFFER_SIZE];    // buffer for reading lines

    // Internal use only:
//    bool _ready_to_send;

    // Checks against class flags variable:
//  bool canRead() { return caps & DEV_CAN_READ; }
//  bool canWrite() { return caps & DEV_CAN_WRITE; }
//  bool canBeCtrl() { return caps & DEV_CAN_BE_CTRL; }
//  bool canBeData() { return caps & DEV_CAN_BE_DATA; }
    bool isCtrl() { return flags & DEV_IS_CTRL; }    // called externally:      DeviceWrappers[i]->isCtrl()
    bool isData() { return flags & DEV_IS_DATA; }    // subclasses can call directly (no pointer): isCtrl()
    bool isPrimary() { return flags & DEV_IS_PRIMARY; }

    bool isAlwaysDataAndCtrl() { return caps & DEV_IS_ALWAYS_BOTH; }
    bool isMuteAsSecondary() { return caps & DEV_IS_MUTE_SECONDARY; }

    bool isConnected() { return flags & DEV_IS_CONNECTED; }
    bool isNotConnected() { return !(flags & DEV_IS_CONNECTED); }
    bool isReady() { return flags & DEV_IS_READY; }
    bool isActive() { return flags & DEV_IS_ACTIVE; }
    bool isMuted() { return flags & DEV_IS_MUTED; }

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
        if (isAlwaysDataAndCtrl() || isMuteAsSecondary()) {
            // In both cases, it cannot be a PRIMARY
            // Also, we remove a MUTED flag
            flags = (flags & ~DEV_IS_MUTED) | (DEV_IS_CTRL | DEV_IS_DATA | DEV_IS_ACTIVE);
        } else {
            flags |= (DEV_IS_CTRL | DEV_IS_DATA | DEV_IS_PRIMARY | DEV_IS_ACTIVE);
        }
    };
    void setAsActiveData() { flags |= ( DEV_IS_DATA | DEV_IS_ACTIVE); };
    void setAsMuted() { flags = (flags & ~(DEV_IS_PRIMARY | DEV_IS_DATA | DEV_IS_CTRL)) | DEV_IS_MUTED; };
    void clearFlags() { flags = DEV_FLAGS_CLEAR; }

    xioDeviceWrapperBase(uint8_t _caps) : caps(_caps),
    flags((_caps & DEV_IS_ALWAYS_BOTH) ? (DEV_IS_CTRL | DEV_IS_DATA) : DEV_FLAGS_CLEAR),
                                          next_flags(DEV_FLAGS_CLEAR)
    {
    };

    // Don't use pure virtuals! They massively slow down the calls.
    // But these MUST be overridden!
    virtual int16_t readchar() { return -1; };
    virtual void flush() {};
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

    bool connected() {
        for (int8_t i = 0; i < _dev_count; ++i) {
            if(DeviceWrappers[i]->isConnected()) {
                return true;
            }
        }
        return false;
    };

    bool others_connected(xioDeviceWrapperBase* except) {
        for (int8_t i = 0; i < _dev_count; ++i) {
            if((DeviceWrappers[i] != except) && (!DeviceWrappers[i]->isAlwaysDataAndCtrl()) && DeviceWrappers[i]->isConnected()) {
                return true;
            }
        }
        return false;
    };

    void remove_data_from_primary() {
        // Why is this first pass here? -RG
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

    bool check_muted_secondary_channels() {
        bool muted_something = false;
        for (int8_t i = 0; i < _dev_count; ++i) {
            if (DeviceWrappers[i]->isMuteAsSecondary()) {
                DeviceWrappers[i]->setAsMuted();
                muted_something = true;
            }
        }
        return muted_something;
    }

    bool deactivate_and_unmute_channels() {
        bool unmuted_something = false;
        for(int8_t i = 0; i < _dev_count; ++i) {
            if (DeviceWrappers[i]->isMuted()) {
                unmuted_something = true;
                DeviceWrappers[i]->setAsPrimaryActiveDualRole(); // NOTE: muted secondary devices won't be set PRIMARY
            } else {
                DeviceWrappers[i]->clearActive();
            }
        }
        return unmuted_something;
    };

    // ##### Cross-Device read/write/etc. functions

    /*
     * write() - write a block to a device
     *
     * There are a few issues with this function that I don't know how to resolve right now:
     * 1) If a device fails to write the data, or all the data, then it's ignored
     * 2) Only the amount written by the *last* device to match (CTRL|ACTIVE) is returned.
     *
     * In the current environment, these are not foreseen to cause trouble since these
     * are blocking writes and we expect to only really be writing to one device.
     */
    size_t write(const char *buffer, size_t size, bool only_to_muted)
    {
        size_t total_written = -1;
        for (int8_t i = 0; i < _dev_count; ++i) {
            bool ok_channel = false;
            if (!only_to_muted) {
                ok_channel = DeviceWrappers[i]->isCtrlAndActive();
            } else {
                ok_channel = DeviceWrappers[i]->isMuted();
            }
            if (ok_channel) {
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

    int16_t writeline(const char *buffer, bool only_to_muted)
    {
        int16_t len = strlen(buffer);
        return write(buffer, len, only_to_muted);
    };

    /*
     * flush() - flush all readable devices' write buffers
     */
    void flush()
    {
        for (int8_t i = 0; i < _dev_count; ++i) {
            DeviceWrappers[i]->flush();
        }
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
     *    Reads a line of text from the next active device that has one ready. With some exceptions.
     *    Accepts CR or LF as line terminator. Replaces CR or LF with NUL in the returned string.
     *
     *    This function iterates over all active control and data devices, including reading from
     *    multiple control devices. It will also manage multiple data devices, but only one data
     *    device may be active at a time.
     *
     *    ARGS:
     *
     *     flags - Bitfield containing the type of channel(s) to read. Looks at DEV_IS_CTRL and
     *             DEV_IS_DATA bits in the device flag field. 'Flags' is loaded with the flags of
     *             the channel that was read on return, or 0 (DEV_FLAGS_CLEAR) if no line was returned.
     *
     *     size -  Returns the size of the completed buffer, including the NUL termination character.
     *             Lines may be returned truncated to the length of the serial input buffer if the text
     *             from the physical device is longer than the read buffer for the device. The size value
     *             provided as a calling argument is ignored (size doesn't matter).
     *
     *     char * Returns a pointer to the buffer containing the line, or NULL (*0) if no text
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

// Declare (but don't define) the xio singleton object now, define it later
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
struct LineRXBuffer : RXBuffer<_size, owner_type, char> {
    typedef RXBuffer<_size, owner_type, char> parent_type;

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

    char _line_buffer[_line_buffer_size]; // hold exactly one line to return
    uint32_t _line_end_guard = 0xBEEF;

    // General term usage:
    // * "index" indicates it's in to _headers array
    // * "offset" means it's a character in the _data array

    uint16_t _scan_offset;          // offset into data of the last character scanned
    uint16_t _line_start_offset;    // offset into first character of the line
    bool     _at_start_of_line;     // true if the last character scanned was the end of a line

    uint16_t _lines_found;          // count of complete non-control lines that were found during scanning.

    volatile uint16_t _last_scan_offset;  // DEBUGGING

    LineRXBuffer(owner_type owner) : parent_type{owner} {};

    void init() {
        parent_type::init();
        _at_start_of_line = true;
    };


    struct SkipSections {
        struct SkipSection {
            uint16_t start_offset; // the offset of the first character to skip
            uint16_t end_offset;   // the offset of the next character to read after skipping
        };

        static constexpr uint16_t _section_count = 16;
        SkipSection _sections[_section_count];

        uint8_t read_section_idx; // index of the first skip section to skip
        uint8_t write_section_idx; // index of the next skip section to populate

        bool is_full() {
            return ((write_section_idx+1)&(_section_count-1)) == read_section_idx;
        };
        bool is_empty() {
            return (write_section_idx == read_section_idx);
        };

        void add_skip(uint16_t start_offset, uint16_t end_offset) {
            if (!is_empty()) {
                uint8_t last_write_section_idx = write_section_idx;
                if (write_section_idx == 0) {
                    last_write_section_idx = _section_count-1;
                } else {
                    last_write_section_idx--;
                }

                if (_sections[last_write_section_idx].end_offset == start_offset) {
                    _sections[last_write_section_idx].end_offset = end_offset;
                    return;
                }
            }
            _sections[write_section_idx].start_offset = start_offset;
            _sections[write_section_idx].end_offset = end_offset;
            write_section_idx = ((write_section_idx+1)&(_section_count-1));
        };

        void pop_skip() {
            _sections[read_section_idx].start_offset = 0;
            _sections[read_section_idx].end_offset = 0;

            read_section_idx = ((read_section_idx+1)&(_section_count-1));
        };

        bool skip(volatile uint16_t &from) {
            if (!is_empty()) {
                SkipSection &next_skip = _sections[read_section_idx];

                if (next_skip.start_offset == from) {
                    from = next_skip.end_offset;

                    pop_skip();
                    return true;
                }
            }
            return false;
        };

//        const SkipSection& next_skip() {
//            return _sections[read_section_idx];
//        }
    };

    SkipSections _skip_sections;

    uint16_t _get_next_scan_offset() {
        return ((_scan_offset + 1) & (_size-1));
    }

    bool _is_more_to_scan() {
        return _canBeRead(_scan_offset);
    };

    /*
     * _scan_buffer()
     *
     * Make a pass through the RX DMA buffer to locate any control lines, and count lines.
     * Single character controls, like !, ~, %, and ^x are also considered control "lines"
     *
     * _scan_buffer() is called at the beginning of readline, and is effectively the first
     * "phase" of readline.
     *
     * This function is designed to be able to exit from almost any point, and
     * come back in and resume where it left off. This allows it to scan to the
     * end of the buffer then exit. When the function is called next it picks up
     * where it left off - i.e. avoiding rescanning the entire buffer multiple times.
     *
     * _scan_buffer() returns true if it finds a control line.
     * The control line starts at the character at _line_start_offset and includes 
     * the characters up to _scan_offset-1. If there are multiple line-ending chars 
     * ("\r\n" for example) _scan_offset will point to the *first* one.
     *
     * With ASCII art (where "." means "invalid data" or "don't care"):
     *
     * Example 1 of _scan_buffer() == true:
     *   _data = "G0X10\n{jvm:5}\n{xvm:1200}\nG0Y10\nG1Z......"
     *                   ^        ^
     *                   |        |
     *   _line_start_offset       |
     *                         _scan_offset
     *
     * Example 2 of _scan_buffer() == true:
     *   _data = "G0X10\n.........{xvm:1200}\nG0Y10\nG1Z......"
     *                            ^           ^
     *                            |           |
     *            _line_start_offset          |
     *                                  _scan_offset
     *
     * Example 2 of _scan_buffer() == true:
     *   _data = "G0X10\n!......"
     *                   ^^
     *                   ||
     *  _line_start_offset|
     *                    _scan_offset
     *
     * For _scan_buffer() == false, IGNORE _line_start_offset and _scan_offset!!!
     * Only use _read_offset, and use _lines_found>0 to determine if _data contains a line to return.
     * Also note that _read_offset needs to be moved once the data is copied to _line_buffer!
     */
    /* Explanation of cases and how we handle it.
     *
     * Our first task in this loop is two-fold (done at the same time):
     *  A) Scan the RX DMA buffer for the next complete line, then classify the line.
     *  B) Scan for a single-character command (!~% ^D, etc), then classify as a control line.
     *
     * If we find a line that classifies as "control" then we return true and stop scanning.
     *
     * We also have a constraint that we may run out of characters at any time. This is OK, 
     * and enough state is kept that we can enter the function at any point with new 
     * characters added to the RX DMA buffer and get the same results.
     *
     * Another constraint is that lines MAY have single character commands embedded in them. 
     * In this case we need to un-embed them. Since we may not have the end of the line yet, 
     * we need to move the command to the beginning of the line.
     *
     * Note that _at_start_of_line means that we *just* parsed a character that is *at* the end of the line.
     * So, for a \r\n sequence, _at_start_of_line will go true of the \r, and we'll see the \n and it'll stay
     * true, then the first non \r or \n char will set it to false, and *then* start the next line.
     */

    bool _scan_buffer() {
        _last_scan_offset = _scan_offset;
        while (_is_more_to_scan()) {
            bool ends_line  = false;
            bool is_control = false;

            char c = _data[_scan_offset];

            if (c == 0) {
                _debug_trap("scan ran into NULL");
                flush(); // consider the connection and all data trashed
                return false;
            }

            // Look for line endings
            if (c == '\r' || c == '\n') {
                if (!_at_start_of_line) {   // We only mark ends_line for the first end-line char, and if
                    ends_line  = true;      // _at_start_of_line is already true, this is not the first.
                }
            }
            // Classify the line if it's a single character 
            else
            if (_at_start_of_line &&
                ((c == '!')         ||
                 (c == '~')         ||
                 (c == ENQ)         ||        // request ENQ/ack
                 (c == CHAR_RESET)  ||        // ^X - reset (aka cancel, terminate)
                 (c == CHAR_ALARM)  ||        // ^D - request job kill (end of transmission)
                 (c == '%' && cm_has_hold())  // flush (only in feedhold or part of control header)
                )) {

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

            // bump the _scan_offset
            _scan_offset = _get_next_scan_offset();

            if (ends_line) {
                // _scan_offset is now one past the end of the line,
                // which means it is at the start of a new line
                _at_start_of_line = true;

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

                if (is_control) {       // we found a control
                    // Quick check for single-character with a \n after it
                    while (_is_more_to_scan() &&
                           ((_data[_scan_offset] == '\n') ||
                            (_data[_scan_offset] == '\r'))
                           )
                    {
                        _scan_offset = _get_next_scan_offset();
                    }
                    return true;
                } else {                // we did find one more line, though.
                    _lines_found++;
                }
            } // if ends_line
        } //while (_is_more_to_scan())
        return false; // no control was found
    };

    /*
     * readline()
     *
     * This is the ONLY external interface in this class
     *
     * Exit condition when a control is found: _line_start_offset and _scan_offset should be the same.
     * If the control was the first char of the buffer it also moves the _data_offset, marking it as read
     */
    char *readline(bool control_only, uint16_t &line_size) {
        // This is tricky: if we don't have room for more skip_sections, then we
        // can't scan any more for controls. So we don't scan, amd hope some lines are read.
        bool found_control = _skip_sections.is_full() ? false : _scan_buffer();

        _restartTransfer();

        char *dst_ptr = _line_buffer;
        line_size = 0;

        if (found_control) {
            // Optimization: if the control was found at the beginning of _data, we note that now
            // and update the _read_offset when we update _line_start_offset
            bool ctrl_is_at_beginning_of_data = (_line_start_offset == _read_offset);
            if (!ctrl_is_at_beginning_of_data) {
                _skip_sections.add_skip(_line_start_offset, _scan_offset);
            }

            // When we get here, _line_start_offset points to either:
            // A) A single-character command, OR
            // B) A full line.
            // Either way, _scan_offset is one past the end, so we don't care which.

            if (_data[_line_start_offset] == 0) {
                _debug_trap("read ran into NULL");
            }

            // scan past any leftover CR or LF from the previous line
            while ((_data[_line_start_offset] == '\n') || (_data[_line_start_offset] == '\r')) {
                _line_start_offset = (_line_start_offset+1)&(_size-1);
                if (_scan_offset == _line_start_offset) {
                    _debug_trap("read ran into scan (1)");
                }
            }

            while ((_scan_offset != _line_start_offset) &&
                   (line_size < (_line_buffer_size - 2))) {
//                if (!_canBeRead(read_offset)) { // This test should NEVER fail.
//                    _debug_trap("readline hit unreadable and shouldn't have!");
//                }

                // copy the charater to _line_buffer
                char c = _data[_line_start_offset];
                *dst_ptr = c;

                // update the line_size
                line_size++;

                // update read/write positions
                dst_ptr++;
                _line_start_offset = (_line_start_offset+1)&(_size-1);
            }

            // null-terminate the string
            *dst_ptr = 0;

            if (ctrl_is_at_beginning_of_data) {
                _read_offset = _scan_offset;
            } else {
                // special case: if the return value is '%'
                // then we actually consider everything before it to be read

                if ('%' == _line_buffer[0]) {
                    // Things that must be managed here:
                    // * _read_offset -- we're skipping data
                    // * _lines_found -- we shouldn't have any lines "left"
                    // * _skip_sections -- there's nothing to skip, we just did

                    // Things that won't be changed (further):
                    // * _scan_offset -- we're not changing past where it's scanned
                    // * _line_start_offset -- we've already adjusted it
                    // * _at_start_of_line -- should always be true when we're here

                    // move the read buffer up to where we're scanning
                    _read_offset = _scan_offset;

                    // record that we have 0 lines (of data) in the buffer
                    _lines_found = 0;

                    // and clear out any skip sections we have
                    while (!_skip_sections.is_empty()) {
                        _skip_sections.pop_skip();
                    }
                }
            }

//            if (ctrl_is_at_beginning_of_data) {
//                // attempt to request more data
//                _restartTransfer();
//            }

            return _line_buffer;
        } // end if (found_control)

        if (control_only) {
            line_size = 0;
            return nullptr;
        }

        if (_lines_found == 0) {
            // nothing to return
            line_size = 0;
            return nullptr;
        }


        // By the time we get here, we know we have at least one line in _data.


        if (_data[_read_offset] == 0) {
            _debug_trap("read ran into NULL");
        }

        // skip sections will always start at the beginning of a line
        _skip_sections.skip(_read_offset);

        // scan past any leftover CR or LF from the previous line
        char c = _data[_read_offset];
        while ((c == '\n') || (c == '\r')) {
            _read_offset = (_read_offset+1)&(_size-1);
            if (_scan_offset == _read_offset) {
                _debug_trap("read ran into scan (2)");
            }
            // this also counts as the beginning of a line
            _skip_sections.skip(_read_offset);
            c = _data[_read_offset];
        }

        while (line_size < (_line_buffer_size - 2)) {
//            if (!_canBeRead(read_offset)) { // This test should NEVER fail.
//                _debug_trap("readline hit unreadable and shouldn't have!");
//            }

            _read_offset = (_read_offset+1)&(_size-1);

            if ( c == '\r' ||
                 c == '\n'
                ) {

                break;
            }

            line_size++;
            *dst_ptr = c;

            // update read/write positions
            dst_ptr++;

            c = _data[_read_offset];
        }

        --_lines_found;

        _restartTransfer();

        // null-terminate the string
        *dst_ptr = 0;
        return _line_buffer;
    }; // readline

    // this is called from flushRead()
    void flush() {
        parent_type::flush();
        _scan_offset = _read_offset;

        // This is similar to the % "queue flush" handling above, except we flush
        // the scan to the to the read (which was just set tot he write by the parent),
        // not the other way around.

        // record that we have 0 lines (of data) in the buffer
        _lines_found = 0;

        // and clear out any skip sections we have
        while (!_skip_sections.is_empty()) {
            _skip_sections.pop_skip();
        }
    }; // flush

}; // LineRXBuffer

template<typename Device>
struct xioDeviceWrapper : xioDeviceWrapperBase {    // describes a device for reading and writing
    Device _dev;

    // TODO - make _buffer_size, _header_count, and _line_buffer_size configurable
    LineRXBuffer<1024, Device> _rx_buffer;
    TXBuffer<1024, Device> _tx_buffer;

    xioDeviceWrapper(Device dev, uint8_t _caps) : xioDeviceWrapperBase(_caps), _dev{dev}, _rx_buffer{_dev}, _tx_buffer{_dev}
    {
//        _dev->setDataAvailableCallback([&](const size_t &length) {
//
//        });
    };

    void init() {
        _dev->setConnectionCallback([&](bool connected) {    // lambda function
            connectedStateChanged(connected);
        });

        _rx_buffer.init();
        _tx_buffer.init();
    };

    virtual int16_t readchar() final {
        if (!isConnected()) {
            return -1;
        }
        return _rx_buffer.read();
//        return _dev->readByte();                    // readByte calls the USB endpoint's read function
    };

    void flush() final {
        _tx_buffer.flush();
        return _dev->flush();
    }

    virtual void flushRead() final {
        // Flush out any partially or wholly read lines being stored:
        _rx_buffer.flush();
        _flushLine();
        return _dev->flushRead();
    }

    virtual int16_t write(const char *buffer, int16_t len) final {
        if (!isConnected()) {
            return -1;
        }
        return _tx_buffer.write(buffer, len);
    }

    virtual char *readline(devflags_t limit_flags, uint16_t &size) final {
        if ((limit_flags & flags) && isConnected()) {
            return _rx_buffer.readline(!(limit_flags & DEV_IS_DATA), size);
        }

        size = 0;
        return NULL;
    };

    void _flushLine() {
        // TODO: Call to flush the RX buffer line structures
    };

    void connectedStateChanged(bool connected) {
            if (connected) {
                if (isNotConnected()) {
                    // USB0 or UART has just connected
                    // If one of the devices isAlwaysDataAndCtrl():
                    //    We treat *it* as if it's the only device connected.
                    //    We treat *the other devices* as if it's NOT connected.
                    // Case 1: This is the first channel to connect -
                    //   set it as CTRL+DATA+PRIMARY channel
                    //     mark all isMutedAsSecondary() as MUTED, and call controller_set_muted(true) if needed
                    // Case 2: This is the second (or later) channel to connect -
                    // Case 2a: This device is !isMuteAsSecondary()
                    //     set it as DATA channel, remove DATA flag from PRIMARY channel
                    //     mark all isMutedAsSecondary() as MUTED, and call controller_set_muted(true) if needed
                    //     ... inactive channels are counted as closed
                    // Case 2b: This devices isMuteAsSecondary(), and needs to be "muted."
                    //     set it as a MUTED channel, call controller_set_connected(true)
                    //       then controller_set_muted(true)


                    setAsConnectedAndReady();

                    if (isAlwaysDataAndCtrl()) {
                        // Case 1 (ignoring others)
                        setActive();
                        controller_set_connected(true);

                        // Case 2b (not ignoring others)
                        if (isMuteAsSecondary() && xio.others_connected(this)) {
                            controller_set_muted(true); // something was muted
                        }

                        return;
                    }

                    if(!xio.others_connected(this)) {
                        // Case 1
                        setAsPrimaryActiveDualRole();
                        // report that there is now have a connection (only for the first one)
                        controller_set_connected(true);
                        // make sure secondary channels (that don't show up in isConnected) are muted
                        if (xio.check_muted_secondary_channels()) {
                            controller_set_muted(true); // something was muted
                        }
                    }
                    else if (isMuteAsSecondary()) {
                        // Case 2b
                        setAsMuted();
                        controller_set_connected(true); // it DID just just get connected
                        controller_set_muted(true);     // but it muted it too
                    }
                    else {
                        // Case 2a
                        xio.remove_data_from_primary();
                        if (xio.check_muted_secondary_channels()) {
                            controller_set_muted(true); // something was muted
                        }
                        setAsActiveData();
                    }
                } // flags & DEV_IS_DISCONNECTED

            } else { // disconnected
                if (isConnected()) {

                    //USB0 has just disconnected
                    //Case 1: This channel disconnected while it was a ctrl+data channel (and no other channels are open) -
                    //  finalize this channel, unmute muted channels
                    //Case 2: This channel disconnected while it was a primary ctrl channel (and other channels are open) -
                    //  finalize this channel, unmute muted channels, deactivate other channels
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
                    flush();
                    flushRead();

                    if (checkForNotActive(oldflags) || isAlwaysDataAndCtrl()) {
                        // Case 5a, 5b
                    } else if (checkForCtrlAndData(oldflags) || !xio.others_connected(this)) {
                        // Case 1
                        if (xio.deactivate_and_unmute_channels()) {
                            controller_set_muted(false);  // something was unmuted
                        } else {
                            controller_set_connected(false);
                        }
                    } else if (checkForCtrlAndPrimary(oldflags)) {
                        // Case 2
                        if (xio.deactivate_and_unmute_channels()) {
                            controller_set_muted(false);  // something was unmuted
                        }
                    } else if (checkForCtrl(oldflags)) {
                        // Case 3
                    } else if (checkForData(oldflags)) {
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
#if USB_SERIAL_PORTS_EXPOSED == 2
xioDeviceWrapper<decltype(&SerialUSB1)> serialUSB1Wrapper {
    &SerialUSB1,
    (DEV_CAN_READ | DEV_CAN_WRITE | DEV_CAN_BE_CTRL | DEV_CAN_BE_DATA)
};
#endif
#endif // XIO_HAS_USB
#if XIO_HAS_UART==1
#if defined(XIO_UART_MUTES_WHEN_USB_CONNECTED) && (XIO_UART_MUTES_WHEN_USB_CONNECTED==1)
constexpr devflags_t _serial0ExtraFlags = DEV_IS_ALWAYS_BOTH | DEV_IS_MUTE_SECONDARY;
#else
constexpr devflags_t _serial0ExtraFlags = DEV_IS_ALWAYS_BOTH;
#endif
xioDeviceWrapper<decltype(&Serial)> serial0Wrapper {
    &Serial,
    (DEV_CAN_READ | DEV_CAN_WRITE | _serial0ExtraFlags)
};
#endif // XIO_HAS_UART

// Define the xio singleton (and initialize it to hold our two deviceWrappers)
//xio_t xio = { &serialUSB0Wrapper, &serialUSB1Wrapper };
xio_t xio = {
#if XIO_HAS_USB == 1
    &serialUSB0Wrapper,
#if USB_SERIAL_PORTS_EXPOSED == 2
    &serialUSB1Wrapper,
#endif
#endif // XIO_HAS_USB
#if XIO_HAS_UART == 1
    &serial0Wrapper
#endif
};

/**** CODE ****/

/*
 * xio_init()
 *
 *  A lambda function closure is provided for trapping connection state changes from USB devices.
 *  The function is installed as a callback from the lower USB layers. It is called only on edges
 *  (connect/disconnect transitions). 'Connected' is true if the USB channel has just connected,
 *  false if it has just disconnected. It is only called on an edge � when it changes - so you
 *  shouldn't see two back-to-back connected=true calls with the same callback.
 *
 *  See here for some good info on lambda functions in C++
 *  http://www.cprogramming.com/c++11/c++11-lambda-closures.html
 */

void xio_init()
{
    board_xio_init();

#if XIO_HAS_USB == 1
    serialUSB0Wrapper.init();
#if USB_SERIAL_PORTS_EXPOSED == 2
    serialUSB1Wrapper.init();
#endif
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

size_t xio_write(const char *buffer, size_t size, bool only_to_muted /*= false*/)
{
    return xio.write(buffer, size, only_to_muted);
}

/*
 * xio_readline() - read a complete line from a device
 * xio_writeline() - write a complete line to control device
 *
 *  Defers to xio.readline(), etc.
 */

char *xio_readline(devflags_t &flags, uint16_t &size)
{
    return xio.readline(flags, size);
}

int16_t xio_writeline(const char *buffer, bool only_to_muted /*= false*/)
{
    return xio.writeline(buffer, only_to_muted);
}

bool xio_connected()
{
    return xio.connected();
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
//    xio.spi_state = (uint8_t)nv->value;
//
//    if (fp_EQ(nv->value, SPI_ENABLE)) {
//        spi_miso_pin.setMode(kOutput);
//        spi_mosi_pin.setMode(kOutput);
//        spi_sck_pin.setMode(kOutput);
//
//    } else if (fp_EQ(nv->value, SPI_DISABLE)) {
//        spi_miso_pin.setMode(kInput);
//        spi_mosi_pin.setMode(kInput);
//        spi_sck_pin.setMode(kInput);
//    }
//    return (STAT_OK);
//}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

static const char fmt_spi[] = "[spi] SPI state%20d [0=disabled,1=enabled]\n";
void xio_print_spi(nvObj_t *nv) { text_print(nv, fmt_spi);} // TYPE_INT

#endif // __TEXT_MODE
