/*
 * i2c_multiplexer/i2c_multiplexer.h - suppport for talking to various I2C/SMBus multiplexers
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

#ifndef i2c_multiplexer_h
#define i2c_multiplexer_h

#include "MotateTWI.h"
// #include "MotateBuffer.h"
#include "MotateUtilities.h"  // for to/fromLittle/BigEndian

using Motate::TWIAddress;

// Complete class for I2C_Multiplexer drivers.
// This one is weird, becaise it acts like a bus, but is another device.
// In fact, when acting like a bus it simply forwards everything to the bus,
// except for queueMessage may inject another message (to switch channels) before
// the requested message.
template <typename device_t>
class I2C_Multiplexer final {
    using TWIDeviceAddressSize = Motate::TWIDeviceAddressSize;
    using TWIMessage = Motate::TWIMessage;

    using multiplexer_t = I2C_Multiplexer<device_t>;

    // TWI and message handling properties
    device_t device_;

    int8_t active_channel_ = -1;

    // Record if we're transmitting to prevent altering the buffers while they
    // are being transmitted still. Includes TX or RX.
    volatile bool transmitting_ = false;

    // We don't want to transmit until we're inited
    bool inited_ = false;


   public:
    // Constructor - this is the only time we directly use the SBIBus
    template <typename TWIBus_t>
    I2C_Multiplexer(TWIBus_t &twi_bus, const uint8_t &address)
        : device_{twi_bus.getDevice({address, TWIDeviceAddressSize::k7Bit})} {
        init();
    }

    // Prevent copying, and prevent moving (so we know if it happens)
    I2C_Multiplexer(const I2C_Multiplexer &) = delete;
    I2C_Multiplexer(I2C_Multiplexer &&other) : device_{std::move(other.device_)} {};

    void init() {
        inited_ = true;
    }

    using dir = TWIMessage::Direction;
    using ias = Motate::TWIInternalAddressSize;

    // void store(const uint16_t address, uint8_t * const buffer,
    //            const uint32_t size) {
    //     if (transmitting_ || !inited_) {
    //         return;
    //     }
    //     // preemptively say we're transmitting .. as a mutex
    //     transmitting_ = true;

    //     message_.setup(buffer, size, dir::kTX, {address, ias::k2Bytes});
    //     device_.queueMessage(&message_);
    // }

    void doneReadingCallback_(const bool worked) {
        transmitting_ = false;
    }

   typename device_t::parent_type* getBus() const { return device_.getBus(); }

    void queueAndSendMessage(TWIMessage* msg) {
        getBus()->queueAndSendMessage(msg);
    }

#pragma mark TWIMultiplexedDevice (inside I2C_Multiplexer)

    struct TWIMultiplexedDevice : device_t {
        multiplexer_t* const parent_multiplexer_;

        // Creat a message and bufer to use to switch the multiplexer to this device
        TWIMessage message_;  // messages to switch
        // Need 4 bytes aligned properly for DMA
        union alignas(4) {
            uint8_t channel_buffer_[4];
            uint8_t channel_;
        } const;

        constexpr TWIMultiplexedDevice(multiplexer_t *const parent_multiplexer,
                                       const TWIAddress &&address,
                                       const uint8_t channel)
            : device_t{parent_multiplexer->getBus(), std::move(address)},
              parent_multiplexer_{parent_multiplexer},
              channel_{channel} {
            message_.message_done_callback = [&](const bool worked) {};
        }

        // prevent copying or deleting
        TWIMultiplexedDevice(const TWIMultiplexedDevice&) = delete;

        // update the bus upon deletion
        ~TWIMultiplexedDevice() { };

        TWIMultiplexedDevice(TWIMultiplexedDevice&& other) = delete;

        // queue message
        void queueMessage(TWIMessage* msg) override {
            if (channel_ != parent_multiplexer_->active_channel_) {
                message_.setup(channel_buffer_, 1, dir::kTX, {0, ias::kNone});
                parent_multiplexer_->device_.queueMessage(&message_);
                parent_multiplexer_->active_channel_ = channel_;
            }

            msg->device = this;
            parent_multiplexer_->queueAndSendMessage(msg);
        };
    };

    using Device_t = TWIMultiplexedDevice;

    // TWIBusDevice factory on TWIBus
    constexpr TWIMultiplexedDevice getDevice(const TWIAddress&& address, const uint8_t channel) { return {this, std::move(address), channel}; }
};

// Deduction guides
template <typename TWIBus_t>
I2C_Multiplexer(TWIBus_t &twi_bus, const uint8_t &address)
    ->I2C_Multiplexer<typename TWIBus_t::Device_t>;

template <typename TWIBus_t>
I2C_Multiplexer(TWIBus_t &twi_bus, const uint8_t &address,
           std::function<void(bool)> &&interrupt_)
    ->I2C_Multiplexer<typename TWIBus_t::Device_t>;

#endif  // i2c_multiplexer_h
