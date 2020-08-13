/*
 * as5311/as5311.h - suppport for AS5311 Linear Sensor
 * https://ams.com/AS5311
 * This file is part of the G2 project
 *
 * Copyright (c) 2020 Alden S. Hart, Jr.
 * Copyright (c) 2020 Robert Giseburt
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

#include "gpio.h"

#ifndef as5311_h
#define as5311_h

#include "MotateSPI.h"
#include "MotateUtilities.h"  // for to/fromLittle/BigEndian

// Complete class for AS5311 drivers.
template <typename device_t>
class AS5311 final : public ExternalLinearEncoder {
    using SPIMessage = Motate::SPIMessage;
    using SPIInterrupt = Motate::SPIInterrupt;
    using SPIDeviceMode = Motate::SPIDeviceMode;

    // For handling callbacks
    std::function<void(bool, float)> _interrupt_handler;

    // SPI and message handling properties
    device_t _device;
    SPIMessage _message;

    alignas(4) uint8_t _buffer[16];

    // Record if we're transmitting to prevent altering the buffers while they
    // are being transmitted still.
    volatile bool _transmitting = false;

    ExternalLinearEncoder::ReturnFormat _return_format;

    // We don't want to transmit until we're inited
    bool _inited = false;

    uint32_t _position = 0;        // raw step position
    double _mm_position_prime = 0; // full 2mm blocks
    double _mm_position_sub = 0;   // sub 2mm block position
    // actual mm posiition is mm_position_prime + mm_position_sub
   public:

    // Constructor - this is the only time we directly use the SBIBus
    template <typename SPIBus_t, typename chipSelect_t>
    constexpr AS5311(SPIBus_t &spi_bus, const chipSelect_t &_cs, std::function<void(bool, float)> &interrupt_)
        : _interrupt_handler{interrupt_},
          _device{spi_bus.getDevice(_cs, 1000000, SPIDeviceMode::kSPIMode2| SPIDeviceMode::kSPI9Bit,
                                    5,  // min_between_cs_delay_ns
                                    500,  // cs_to_sck_delay_ns
                                    0   // between_word_delay_ns
                                    )} {
        init();
    };

    template <typename SPIBus_t, typename chipSelect_t>
    constexpr AS5311(SPIBus_t &spi_bus, const chipSelect_t &_cs)
        : _device{spi_bus.getDevice(_cs, 1000000, SPIDeviceMode::kSPIMode2 | SPIDeviceMode::kSPI9Bit,
                                    5,   // min_between_cs_delay_ns
                                    500,  // cs_to_sck_delay_ns
                                    0   // between_word_delay_ns
                                    )} {
        init();
    };

    // Prevent copying, and prevent moving (so we know if it happens)
    AS5311(const AS5311 &) = delete;
    AS5311(AS5311 &&other) : _device{std::move(other.device_)} {};

    void init() {
        _message.message_done_callback = [&]() { this->doneReadingCallback_(); };
        _inited = true;
    }

    void setCallback(std::function<void(bool, float)> &&handler) override {
        _interrupt_handler = std::move(handler);
    }

    void setCallback(std::function<void(bool, float)> &handler) override {
        _interrupt_handler = handler;
    }

    virtual void requestPositionMMs() override {
        _return_format = ReturnMMs;
        getPos_();
    }

    void requestPositionFraction() override {
        _return_format = ReturnFraction;
        getPos_();
    }

   private:
    uint8_t fails_ = 0;
    void getPos_() {
        if (!_inited || _transmitting) {
            if (++fails_ > 10) {
                #ifdef IN_DEBUGGER
                __asm__("BKPT");  // about to send non-Setup message
                #endif
                // fails_ = 0;
                // _transmitting = false;
            }

            // Motate::debug.send(fails_, 1);

            // if (_interrupt_handler) {
            //     _interrupt_handler(false, 0.0);
            // }
            return;
        }
        _transmitting = true;  // preemptively say we're transmitting .. as a mutex

        fails_ = 0;
        // Motate::debug.send(fails_, 1);

        // reading, prepare the address in the scribble buffer
        // we read 3 x "9-bit" blocks, which each become 16-bit half-words
        _message.setup(nullptr, _buffer, 2, SPIMessage::DeassertAfter, SPIMessage::EndTransaction);
        _device.queueMessage(&_message);
    }

    volatile uint32_t _status;
    volatile uint32_t _max_position;
    volatile uint32_t _temp_bits;
    void doneReadingCallback_() {
        _transmitting = false;
        // _position = ((_buffer[1] & 0b1) << 11) | (_buffer[0] << 3) | (_buffer[3] << 2) | (_buffer[2] >> 6);
        // uint32_t temp = ((_buffer[1] & 0b1) << 17) | (_buffer[0] << 9) | (_buffer[2]) | ((_buffer[3] & 0b1) << 8);
        uint32_t temp = ((_buffer[1] & 0b1) << 9) | (_buffer[0] << 10) | (_buffer[2] << 1) | (_buffer[3] & 0b1);
        _position = temp >> 6;
        _max_position |= _position;
        _temp_bits |= temp;
        _status = temp & 0b111110;

        // value from the encoder is 0-4096, which stands for 0-2mm, where it rolls over to 0
        // new_mm_position_sub is the partial position from 0-2mm
        double new_mm_position_sub = (_position / 4096.0) * 2.0;

        // to convert that to full position, first find the difference, then determine
        // if there was a positive or negative rollover, and convert that to a diff
        // note we assume that between polls it's impossible to move more that +-1mm
        double position_diff = new_mm_position_sub - _mm_position_sub;
        if (position_diff < -1.0) {
            _mm_position_prime += 2.0;
        } else if (position_diff > 1.0) {
            _mm_position_prime -= 2.0;
        }
        _mm_position_sub = new_mm_position_sub;

        if (_interrupt_handler) {
            call_interrupt_();
        }
    }

    void call_interrupt_() {
        float value = 0;
        if (_return_format == ReturnMMs) {
            value = (float)(_mm_position_prime + _mm_position_sub);
        } else {
            value = (float)_position * (1.0 / 4096.0);
        }
        _interrupt_handler(true, value);
    }
};

#endif  // as5311_h
