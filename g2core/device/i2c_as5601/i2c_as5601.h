/*
 * i2c_as5601/i2c_as5601.h - suppport for AS5601 Position Sensor
 * https://ams.com/AS5601
 * This file is part of the G2 project
 *
 * Copyright (c) 2018-2019 Alden S. Hart, Jr.
 * Copyright (c) 2018-2019 Robert Giseburt
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

#ifndef i2c_as5601_h
#define i2c_as5601_h

#include "MotateTWI.h"
// #include "MotateBuffer.h"
#include "MotateUtilities.h"  // for to/fromLittle/BigEndian

// Complete class for I2C_AS5601 drivers.
template <typename device_t>
class I2C_AS5601 final : public ExternalEncoder, public gpioDigitalInputHandler {
    using TWIDeviceAddressSize = Motate::TWIDeviceAddressSize;
    using TWIMessage = Motate::TWIMessage;

    static const bool using_pins_ = false;

    // TWI and message handling properties
    device_t device_;
    TWIMessage message_;

    alignas(4) uint8_t buffer_[4];

    // Record if we're transmitting to prevent altering the buffers while they
    // are being transmitted still. Icludes TX or RX.
    volatile bool transmitting_ = false;

    // We don't want to transmit until we're inited
    // bool inited_ = false;

    // Record what internal address we're reading/writing, and make an enum
    enum class RegisterAddress : uint8_t {
        // (R)ead, (W)rite, (P)rogrammable

        // config
        kZMCO = 0x00,     // 1 byte  (R)
        kZPOS = 0x01,     // 2 bytes (R/W/P)
        kCONF = 0x07,     // 2 bytes (R/W/P)
        kABN = 0x09,      // 1 byte  (R/W/P)
        kPUSHTHR = 0x0A,  // 1 bytes (R/W/P)

        // output
        kRAW_ANGLE = 0x0C,  // 2 bytes (R)
        kANGLE = 0x0E,      // 2 bytes (R)

        // status
        kSTATUS = 0x08,     // 1 byte  (R)
        kAGC = 0x1A,        // 1 byte  (R)
        kMAGNITUDE = 0x1B,  // 2 bytes (R)

        kBURN = 0xFF,       // 1 byte  (W)
    } active_address_;

    // For handling callbacks
    std::function<void(bool, float)> interrupt_handler_;

    enum { INIT, SETUP, IDLE, SOMETHING, READING_ANGLE } state_ = SETUP;
    ExternalEncoder::ReturnFormat return_format_;

    int8_t quadrature_a_input_ = 0, quadrature_b_input_ = 0;
    int16_t position_ = 0;
    int16_t pins_position_ = -1;

    // static const uint8_t dev_address_ = 0x40; // AS5600L
    static const uint8_t dev_address_ = 0x36; // AS5601

   public:
    template <typename TWIBus_t, typename... Ts>
    I2C_AS5601(TWIBus_t &twi_bus, int8_t quadrature_a_input, int8_t quadrature_b_input,
               std::function<void(bool, float)> &&interrupt_, Ts... v)
        : gpioDigitalInputHandler{[&](const bool state, const inputEdgeFlag edge, const int8_t triggering_pin_number) {
                                      return this->handleQuadrature(state, edge, triggering_pin_number);
                                  },
                                  5, nullptr},
          device_{twi_bus.getDevice({dev_address_, TWIDeviceAddressSize::k7Bit}, v...)},
          interrupt_handler_{std::move(interrupt_)},
          quadrature_a_input_{quadrature_a_input},
          quadrature_b_input_{quadrature_b_input} {
        init();
    }

    template <typename TWIBus_t, typename... Ts>
    I2C_AS5601(TWIBus_t &twi_bus, int8_t quadrature_a_input, int8_t quadrature_b_input, std::function<void(bool, float)> &interrupt_, Ts... v)
        : gpioDigitalInputHandler{[&](const bool state, const inputEdgeFlag edge, const int8_t triggering_pin_number) {
                                      return this->handleQuadrature(state, edge, triggering_pin_number);
                                  },
                                  5, nullptr},
          device_{twi_bus.getDevice({dev_address_, TWIDeviceAddressSize::k7Bit}, v...)},
          interrupt_handler_{interrupt_},
          quadrature_a_input_{quadrature_a_input},
          quadrature_b_input_{quadrature_b_input} {
        init();
    }

    template <typename TWIBus_t, typename... Ts>
    I2C_AS5601(TWIBus_t &twi_bus, int8_t quadrature_a_input, int8_t quadrature_b_input, Ts... v)
        : gpioDigitalInputHandler{[&](const bool state, const inputEdgeFlag edge, const int8_t triggering_pin_number) {
                                      return this->handleQuadrature(state, edge, triggering_pin_number);
                                  },
                                  5, nullptr},
          device_{twi_bus.getDevice({dev_address_, TWIDeviceAddressSize::k7Bit}, v...)},
          quadrature_a_input_{quadrature_a_input},
          quadrature_b_input_{quadrature_b_input} {
        init();
    }

    // Prevent copying, and prevent moving (so we know if it happens)
    I2C_AS5601(const I2C_AS5601 &) = delete;
    I2C_AS5601(I2C_AS5601 &&other) : device_{std::move(other.device_)} {};

    void init() {
        state_ = INIT;
        if (using_pins_) {
            din_handlers[INPUT_ACTION_INTERNAL].registerHandler(this);
            gpio_set_input_lockout(quadrature_a_input_, 0);
            gpio_set_input_lockout(quadrature_b_input_, 0);
        }

        message_.message_done_callback = [&](const bool worked) { this->doneReadingCallback_(worked); };
    }

    using dir = TWIMessage::Direction;
    using ias = Motate::TWIInternalAddressSize;

    void setCallback(std::function<void(bool, float)> &&handler) override {
        interrupt_handler_ = std::move(handler);
    }

    void setCallback(std::function<void(bool, float)> &handler) override {
        interrupt_handler_ = handler;
    }

    void requestAngleDegrees() override {
        return_format_ = ReturnDegrees;
        getPos_();
    }

    // void getAngleDegrees(std::function<void(bool, float)> &&handler) {
    //     interrupt_handler_ = std::move(handler);
    //     return_format_ = ReturnDegrees;
    //     getPos_();
    // }

    // void getAngleDegrees(std::function<void(bool, float)> &handler) {
    //     interrupt_handler_ = handler;
    //     return_format_ = ReturnDegrees;
    //     getPos_();
    // }

    void requestAngleRadians() override {
        return_format_ = ReturnRadians;
        getPos_();
    }

    // void getAngleRadians(std::function<void(bool, float)> &&handler) {
    //     interrupt_handler_ = std::move(handler);
    //     return_format_ = ReturnRadians;
    //     getPos_();
    // }

    // void getAngleRadians(std::function<void(bool, float)> &handler) {
    //     interrupt_handler_ = handler;
    //     return_format_ = ReturnRadians;
    //     getPos_();
    // }

    void requestAngleFraction() override {
        return_format_ = ReturnFraction;
        getPos_();
    }


    // void getAngleFraction(std::function<void(bool, float)> &&handler) {
    //     interrupt_handler_ = std::move(handler);
    //     return_format_ = ReturnFraction;
    //     getPos_();
    // }

    // void getAngleFraction(std::function<void(bool, float)> &handler) {
    //     interrupt_handler_ = handler;
    //     return_format_ = ReturnFraction;
    //     getPos_();
    // }

   private:
    uint8_t fails_ = 0;
    void getPos_() {
        if (state_ == INIT) {
            buffer_[0] = 15;

            state_ = SETUP;

            message_.setup(buffer_, 1, dir::kTX, {(uint16_t)RegisterAddress::kABN, ias::k1Byte});
            active_address_ = RegisterAddress::kABN;
            device_.queueMessage(&message_);

            return;
        }

        if (using_pins_) {
        } else {
            if (state_ != IDLE) {
                if (++fails_ > 10) {
                    #ifdef IN_DEBUGGER
                    __asm__("BKPT");  // about to send non-Setup message
                    // TWIHS0_Handler();
                    #endif
                    fails_ = 0;
                    state_ = IDLE;
                }

                interrupt_handler_(false, 0.0);
                return;
            }

            fails_ = 0;
            state_ = READING_ANGLE;

            // if (active_address_ != RegisterAddress::kANGLE) {
                message_.setup(buffer_, 2, dir::kRX, {(uint16_t)RegisterAddress::kANGLE, ias::k1Byte});
                active_address_ = RegisterAddress::kANGLE;
            // } else {
            //     message_.setup(buffer_, 2, dir::kRX);
            // }
            device_.queueMessage(&message_);
        }
    }

    void doneReadingCallback_(const bool worked) {
        // interrupt_handler_ may try to queue another message,
        // and we don't want to fail that, so store it and mark IDLE
        auto old_state = state_;
        state_ = IDLE;

        if (old_state == SETUP) {
            getPos_(); // restart the request
        }
        else if (old_state == READING_ANGLE && interrupt_handler_) {
            if (!worked) {
                interrupt_handler_(false, 0.0);
            } else {
                position_ = (buffer_[0] << 8) | buffer_[1];
                if (pins_position_ == -1) {
                    pins_position_ = position_; // sync them
                }
                call_interrupt_();
            }
        }
    }

    void call_interrupt_() {
        float value = 0;
        if (return_format_ == ReturnDegrees) {
            value = (float)position_ * (360.0/4096.0);
        } else if (return_format_ == ReturnRadians) {
            value = (float)position_ * ((2.0 * M_PI)/4096.0);
        } else {
            value = (float)position_ * (1.0 / 4096.0);
        }
        interrupt_handler_(true, value);
    }

    bool a_state = true;
    bool b_state = true;
    bool last_trigger_was_a = false;
    bool handleQuadrature(const bool state, const inputEdgeFlag edge, const uint8_t triggering_pin_number) {
        if ((quadrature_a_input_ == 0) || (quadrature_b_input_  == 0) || pins_position_ == -1) {
            return false; // let someone else handle this
        }

        if ((triggering_pin_number != quadrature_a_input_) && (triggering_pin_number != quadrature_b_input_)) {
            return false; // not one of our inputs, let someone else handle it
        }

        // If A changes *away* from B, or B changes *toward* A, step forward, otherwise step back
        bool old_a_state = a_state;
        bool old_b_state = b_state;

        if (triggering_pin_number == quadrature_a_input_) {
            a_state = state;
            b_state = gpio_read_input(quadrature_b_input_);
            // if last_trigger_was_a then it either reversed directions or skipped a b interrupt
            // if b didn't change, then it rocked back the other way
            if (last_trigger_was_a && (old_b_state != b_state)) {
                pins_position_ = pins_position_ + ((state != b_state) ? 4 : -4);
            } else {
                pins_position_ = pins_position_ + ((state != b_state) ? 2 : -2);
            }
            last_trigger_was_a = true;
        } else if (triggering_pin_number == quadrature_b_input_) {
            a_state = gpio_read_input(quadrature_a_input_);
            b_state = state;

            // see comments for a above
            if (!last_trigger_was_a && (old_a_state != a_state)) {
                pins_position_ = pins_position_ + ((state == a_state) ? -4 : 4);
            } else {
                pins_position_ = pins_position_ + ((state == a_state) ? -2 : 2);
            }
            last_trigger_was_a = false;
        } else {
            return false; // Should never get here, likely optimized out
        }

        if (pins_position_ >= 4096) {
            pins_position_ -= 4096;
        } else if (pins_position_ < 0) {
            pins_position_ += 4096;
        }

        call_interrupt_();

        return true;  // we are consuming this event, no one else gets to see it
    }

    float getQuadratureFraction() override { return position_ / 4096.0; }
};

// Deduction guides
template <typename TWIBus_t, typename... Ts>
I2C_AS5601(TWIBus_t &, Ts...)
    ->I2C_AS5601<typename TWIBus_t::Device_t>;

#endif  // i2c_as5601_h
