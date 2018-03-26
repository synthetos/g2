/*
 * i2c_eeprom/i2c_eeprom.h - suppport for talking to various I2C/SMBus EEPROM
 * modules This file is part of the G2 project
 *
 * Copyright (c) 2018 Alden S. Hart, Jr.
 * Copyright (c) 2018 Robert Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License, version 2 as
 * published by the Free Software Foundation. You should have received a copy of
 * the GNU General Public License, version 2 along with the software.  If not,
 * see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library
 * without restriction. Specifically, if other files instantiate templates or
 * use macros or inline functions from this file, or you compile this file and
 * link it with  other files to produce an executable, this file does not by
 * itself cause the resulting executable to be covered by the GNU General Public
 * License. This exception does not however invalidate any other reasons why the
 * executable file might be covered by the GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT
 * ANY WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
 * THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
 * THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef i2c_eeprom_h
#define i2c_eeprom_h

#include "MotateTWI.h"
// #include "MotateBuffer.h"
#include "MotateUtilities.h"  // for to/fromLittle/BigEndian

// Complete class for I2C_EEPROM drivers.
template <typename device_t>
class I2C_EEPROM final {
    using TWIDeviceAddressSize = Motate::TWIDeviceAddressSize;
    using TWIMessage = Motate::TWIMessage;

    // TWI and message handling properties
    device_t _device;
    TWIMessage _message;

    // Record if we're transmitting to prevent altering the buffers while they
    // are being transmitted still. Icludes TX or RX.
    volatile bool _transmitting = false;

    // We don't want to transmit until we're inited
    bool _inited = false;

    // Record what internal address we're reading/writing
    int16_t _active_address = -1;

    // Timer to keep track of when we need to do another periodic update
    Motate::Timeout _check_timer;

    // For handling callbacks
    std::function<void(bool)> _interrupt_handler;

    enum { IDLE, WAITING_TO_READ, READING, WAITING_TO_WRITE, WRITING } _state;

   public:
    // Constructor - this is the only time we directly use the SBIBus
    template <typename TWIBus_t, typename... Ts>
    I2C_EEPROM(TWIBus_t &twi_bus, const uint8_t &address, Ts... v)
        : _device{twi_bus.getDevice({address, TWIDeviceAddressSize::k7Bit}, v...)} {
        init();
    }

    // template <typename TWIBus_t, typename... Ts>
    // I2C_EEPROM(TWIBus_t &twi_bus, const uint8_t &address,
    //            std::function<void(bool)> &&_interrupt, Ts... v)
    //     : _device{twi_bus.getDevice({address, TWIDeviceAddressSize::k7Bit}, v...)},
    //       _interrupt_handler{std::move(_interrupt)} {
    //     init();
    // }

    // Prevent copying, and prevent moving (so we know if it happens)
    I2C_EEPROM(const I2C_EEPROM &) = delete;
    I2C_EEPROM(I2C_EEPROM &&other) : _device{std::move(other._device)} {};

    void init() {
        _message.message_done_callback = [&](const bool worked) {
            this->_doneReadingCallback(worked);
        };
        _inited = true;
    }

    using dir = TWIMessage::Direction;
    using ias = Motate::TWIInternalAddressSize;

    void store(const uint16_t address, uint8_t * const buffer,
               const uint32_t size) {
        if (_transmitting || !_inited) {
            return;
        }
        // preemptively say we're transmitting .. as a mutex
        _transmitting = true;

        _message.setup(buffer, size, dir::kTX, {address, ias::k2Bytes});
        _device.queueMessage(&_message);
    }

    void store(const uint16_t address, uint8_t * const buffer, const uint32_t size,
               std::function<void(bool)> &&handler) {
        _interrupt_handler = std::move(handler);
        store(address, buffer, size);
    }

    void load(const uint16_t address, uint8_t * const buffer, const uint32_t size) {
        if (_transmitting || !_inited) {
            return;
        }
        // preemptively say we're transmitting .. as a mutex
        _transmitting = true;

        _message.setup(buffer, size, dir::kRX, {address, ias::k2Bytes});
        _device.queueMessage(&_message);
    }

    void load(const uint16_t address, uint8_t * const buffer, const uint32_t size,
              std::function<void(bool)> &&handler) {
        _interrupt_handler = std::move(handler);
        load(address, buffer, size);
    }

    void _doneReadingCallback(const bool worked) {
        _transmitting = false;

        if (_interrupt_handler) {
            _interrupt_handler(worked);
        }
    }

    // We can only support interrupt inferface option 2: a function with a
    // closure or function pointer
    void setInterruptHandler(std::function<void(bool)> &&handler) {
        _interrupt_handler = std::move(handler);
    }
    void setInterruptHandler(const std::function<void(bool)> &handler) {
        _interrupt_handler = handler;
    }
};

// Deduction guides
template <typename TWIBus_t, typename... Ts>
I2C_EEPROM(TWIBus_t &, const uint8_t &, Ts...)
    ->I2C_EEPROM<typename TWIBus_t::Device_t>;

template <typename TWIBus_t, typename... Ts>
I2C_EEPROM(TWIBus_t &, const uint8_t &, std::function<void(bool)> &&, Ts...)
    ->I2C_EEPROM<typename TWIBus_t::Device_t>;

#endif  // i2c_eeprom_h
