/*
 * honeywell-trustability-ssc.h - suppport for talking to the Honeywell TruStability SSC line of pressure/temperature sensors
 * This file is part of the G2 project
 *
 * Copyright (c) 2020 Robert Giseburt
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


#ifndef honeywell_trustability_ssc_h
#define honeywell_trustability_ssc_h

#include "MotateSPI.h"
#include "MotateBuffer.h"
#include "MotateUtilities.h" // for to/fromLittle/BigEndian
#include "util.h" // for fp_ZERO

enum class PressureUnits { PSI, cmH2O, inH20, Pa, kPa };

struct PressureSensor {
    virtual double getPressure(const PressureUnits output_units) const;
};

// Complete class for TruStabilitySSC drivers.
template <typename device_t>
struct TruStabilitySSC final : virtual public PressureSensor {
    using SPIMessage = Motate::SPIMessage;
    using SPIInterrupt = Motate::SPIInterrupt;
    using SPIDeviceMode = Motate::SPIDeviceMode;

    // SPI and message handling properties
    device_t _device;
    SPIMessage _message;

    // Record if we're transmitting to prevent altering the buffers while they
    // are being transmitted still.
    volatile bool _transmitting = false;

    // We don't want to transmit until we're inited
    bool _inited = false;

    // Record what register we just requested, so we know what register the
    // the response is for (and to read the response.)
    int16_t _active_register = -1;

    // Timer to keep track of when we need to do another periodic update
    Motate::Timeout _check_timer;

    // Parameters of the sensor - for now, compile-time
    const uint16_t _min_output;
    const uint16_t _max_output;
    const double _min_value;
    const double _max_value;
    const PressureUnits _base_units;

    // Constructor - this is the only time we directly use the SBIBus
    template <typename SPIBus_t, typename chipSelect_t>
    TruStabilitySSC(SPIBus_t &spi_bus, const chipSelect_t &_cs, const uint16_t min_output, const uint16_t max_output,
                    const double min_value, const double max_value, const PressureUnits base_units)
        : _device{spi_bus.getDevice(_cs, 5000000, SPIDeviceMode::kSPIMode0 | SPIDeviceMode::kSPI8Bit,
                                    400,  // min_between_cs_delay_ns
                                    400,  // cs_to_sck_delay_ns
                                    80    // between_word_delay_ns
                                    )},
          _min_output{min_output},
          _max_output{max_output},
          _min_value{min_value},
          _max_value{max_value},
          _base_units{base_units} {
        init();
    };

    template <typename SPIBus_t, typename chipSelect_t>
    TruStabilitySSC(const Motate::PinOptions_t options,  // completely ignored, but for compatibility with ADCPin
                    std::function<void(bool)> &&_interrupt, SPIBus_t &spi_bus, const chipSelect_t &_cs,
                    const uint16_t min_output, const uint16_t max_output, const double min_value,
                    const double max_value, const PressureUnits base_units)
        : _device{spi_bus.getDevice(_cs, 5000000, SPIDeviceMode::kSPIMode1 | SPIDeviceMode::kSPI8Bit,
                                    400,  // min_between_cs_delay_ns
                                    400,  // cs_to_sck_delay_ns
                                    80    // between_word_delay_ns
                                    )},
          _min_output{min_output},
          _max_output{max_output},
          _min_value{min_value},
          _max_value{max_value},
          _base_units{base_units},
          _interrupt_handler{std::move(_interrupt)} {
        init();
    };

    // Prevent copying, and prevent moving (so we know if it happens)
    TruStabilitySSC(const TruStabilitySSC &) = delete;
    TruStabilitySSC(TruStabilitySSC &&other) : _device{std::move(other._device)} {};


    // ###########
    // The HoneyWell TruStability SSC devices are really simple to talk to:
    // See https://sensing.honeywell.com/spi-comms-digital-ouptu-pressure-sensors-tn-008202-3-en-final-30may12.pdf
    // Read from 2 to 4 byts of data, interpret it, repeat.
    // The bytes are: Status_and_BridgeData_MSB, BridgeData_LSB, TemperatureData_MSB, TemperatureData_LSB

    enum {
        INITING,
        WAITING_FOR_SAMPLE,
        NEEDS_SAMPLED,
    } _state;

    bool _data_needs_read = false;
    static const size_t _data_size = 4;
    struct data_t {
        union {
            uint8_t raw_data[4];
            struct {
                uint8_t bridge_msb : 6;
                uint8_t status : 2;
                uint8_t bridge_lsb : 8;
                uint8_t temperature_msb : 8;
                uint8_t temperature_lsb : 3;
                uint8_t unused : 5;
            };
        };
    };
    alignas(4) data_t _data;

    double temperature = 0;
    double pressure = 0;

    static constexpr uint8_t STALE_DATA = 0b10;

    void _postReadSampleData() {
        if (INITING != _state && _data.status == STALE_DATA) {
            // we requested data too soon, the data is stale, try again sooner
            _check_timer.set(0);
            _data_needs_read = true;
            return;
        }

        int32_t bridge_output = (_data.bridge_msb << 8) | _data.bridge_lsb;
        // uint32_t temperature_output = (_data.temperature_msb << 3) | _data.temperature_lsb;

        // Formula
        // ((ouput - output_min)*(pressure_max-pressure_min))/(outout_max-output_min)+pressure_min

        // const uint16_t _min_output, _max_output;
        // const double _min_value, _max_value;

        if (bridge_output < _min_output || bridge_output > _max_output) {
            pressure = 0.0;
            // error condition
        }

        double temp_pressure = ((double)(bridge_output - _min_output)*(double)(_max_value-_min_value))/(_max_output-_min_output)+_min_value;

        if (temp_pressure < _min_value || bridge_output > _max_value) {
            pressure = 0.0;
            // error condition
        }

        pressure = temp_pressure;
    };

    alignas(4) uint8_t _scribble_buffer[8];

    void _startNextReadWrite()
    {
        if (_transmitting || !_inited) { return; }
        _transmitting = true; // preemptively say we're transmitting .. as a mutex

        // check if we need to read reagisters
        if (!_data_needs_read)
        {
            _transmitting = false; // we're not really transmitting.
            return;
        }
        _data_needs_read = false;

        // reading, prepare the address in the scribble buffer
        _message.setup(_scribble_buffer, (uint8_t*)&_data.raw_data, 4, SPIMessage::DeassertAfter, SPIMessage::EndTransaction);
        _device.queueMessage(&_message);
    };

    void _doneReadingCallback()
    {
        _transmitting = false;

        _postReadSampleData();
    };

    void init()
    {
        _message.message_done_callback = [&] { this->_doneReadingCallback(); };

        // Establish default values, and then prepare to read the registers we can to establish starting values

        _inited = true;
        //_startNextReadWrite();
        _check_timer.set(0);
    };

    // interface to make this a drop-in replacement (after init) for an ADCPin

    std::function<void(bool)> _interrupt_handler;

    void startSampling()
    {
        // if (_check_timer.isPast()) {
        //     // if (INITING == _state) {
        //     //     _data_needs_read = true;
        //     //     _check_timer.set(1);
        //     //     _state = WAITING_FOR_SAMPLE;
        //     //     _startNextReadWrite();

        //     // } else if (WAITING_FOR_SAMPLE == _state) {
        //         _check_timer.set(1);

                _data_needs_read = true;
                _startNextReadWrite();
            // }
        // }
    };

    double getPressure(const PressureUnits output_units) const override {
        if (output_units != _base_units) {
            if (_base_units == PressureUnits::PSI && output_units == PressureUnits::cmH2O) {
                // the only currently supported configuration
                return pressure / 0.014223343334285;
            }
        }

    };

    //     // getRaw is to return the last sampled value
    //     int32_t getRaw() {
    //         if (_fault_status.value) {
    //             return -_fault_status.value;
    //         }
    //         return _rtd_value;
    //     };

    //     float getPullupResistance() {
    //         return _pullup_resistance;
    //     }
    //     void setPullupResistance(const float r) {
    //         _pullup_resistance = r;
    //     }

    //     // getValue is supposed to request a new value, block, and then return the result
    //     // PUNT - return the same as getRaw()
    //     int32_t getValue() {
    //         return getRaw();
    //     };
    // //    int32_t getBottom() {
    // //        return 0;
    // //    };
    // //    float getBottomVoltage() {
    // //        return 0;
    // //    };
    // //    int32_t getTop() {
    // //        return 32767;
    // //    };
    // //    float getTopVoltage() {
    // //        return _vref;
    // //    };

    //     void setVoltageRange(const float vref,
    //                          const float min_expected = 0,
    //                          const float max_expected = -1,
    //                          const float ideal_steps = 1)
    //     {
    // //        _vref = vref;

    //         // All of the rest are ignored, but here for compatibility of interface
    //     };
    //     float getVoltage() {
    //         float r = getRaw();
    //         if (r < 0) {
    //             return r*1000.0;
    //         }
    //         return ((r*_pullup_resistance)/32768.0);
    //     };
    //     operator float() { return getVoltage(); };

    //     float getResistance() {
    //         float r = getRaw();
    //         if (r < 0) {
    //             return r*1000.0;
    //         }
    //         return (r*_pullup_resistance)/32768.0;
    //     }

    //     void setInterrupts(const uint32_t interrupts) {
    //         // ignore this -- it's too dangerous to accidentally change the SPI interrupts
    //     };

    //     // We can only support interrupt inferface option 2: a function with a closure or function pointer
    //     void setInterruptHandler(std::function<void(bool)> &&handler) {
    //         _interrupt_handler = std::move(handler);
    //     };
    //     void setInterruptHandler(const std::function<void(bool)> &handler) {
    //         _interrupt_handler = handler;
    //     };
};

// A gpioAnalogInputPin subclass for the MAX31865

// template <typename device_t>
// struct gpioAnalogInputPin<MAX31865<device_t>> : gpioAnalogInput {
// protected: // so we know if anyone tries to reach in
//     ioEnabled enabled;                  // -1=unavailable, 0=disabled, 1=enabled
//     AnalogInputType_t type;

//     const uint8_t ext_pin_number;       // external number to configure this pin ("ai" + ext_pin_number)
//     uint8_t proxy_pin_number;           // optional external number to access this pin ("ain" + proxy_pin_number)

//     using ADCPin_t = MAX31865<device_t>;

//     ADCPin_t pin;                        // the actual pin object itself

// public:
//     // In constructor, simply forward all values to the pin
//     // To get a different behavior, override this object.
//     template <typename... T>
//     gpioAnalogInputPin(const ioEnabled _enabled, const AnalogInputType_t _type, const uint8_t _ext_pin_number, const uint8_t _proxy_pin_number, T&&... additional_values) :
//     gpioAnalogInput{},
//     enabled{_enabled},
//     type{_type},
//     ext_pin_number{_ext_pin_number},
//     proxy_pin_number{ _proxy_pin_number },
//     pin{Motate::kNormal, [&](bool e){this->adc_has_new_value(e);}, additional_values...}
//     {
//         // nothing to do here
//     };

//     // functions for use by other parts of the code, and are overridden

//     ioEnabled getEnabled() override
//     {
//         return enabled;
//     };
//     bool setEnabled(const ioEnabled m) override
//     {
//         if (enabled == IO_UNAVAILABLE) {
//             return false;
//         }
//         enabled = m;
//         return true;
//     };

//     float getValue() override
//     {
//         if (enabled != IO_ENABLED) {
//             return 0;
//         }
//         return pin.getVoltage();
//     };
//     float getResistance() override
//     {
//         if (enabled != IO_ENABLED) {
//             return -1;
//         }
//         return pin.getResistance();
//     };

//     AnalogInputType_t getType() override
//     {
//         return type;
//     };
//     bool setType(const AnalogInputType_t t) override
//     {
//         // NOTE: Allow setting type to AIN_TYPE_EXTERNAL
//         if (t == AIN_TYPE_INTERNAL) {
//             return false;
//         }
//         type = t;
//         return true;
//     };

//     AnalogCircuit_t getCircuit() override
//     {
//         return AIN_CIRCUIT_EXTERNAL;
//     };
//     bool setCircuit(const AnalogCircuit_t c) override
//     {
//         // prevent setting circuit to anything but AIN_CIRCUIT_EXTERNAL
//         if (c == AIN_CIRCUIT_EXTERNAL) {
//             return true;
//         }
//         return false;
//     };

//     float getParameter(const uint8_t p) override
//     {
//         if (p == 0) {
//             return pin.getPullupResistance();
//         }
//         return 0;
//     };
//     bool setParameter(const uint8_t p, const float v) override
//     {
//         if (p == 0) {
//             pin.setPullupResistance(v);
//             return true;
//         }
//         return false;
//     };


//     void startSampling() override {
//         pin.startSampling();
//     };


//     bool setExternalNumber(const uint8_t e) override
//     {
//         if (e == proxy_pin_number) { return true; }
//         if (proxy_pin_number > 0) {
//             // clear the old pin
//             ain_r[proxy_pin_number-1]->setPin(nullptr);
//         }
//         proxy_pin_number = e;
//         if (proxy_pin_number > 0) {
//             // set the new pin
//             ain_r[proxy_pin_number-1]->setPin(this);
//         }
//         return true;
//     };

//     const uint8_t getExternalNumber() override
//     {
//         return proxy_pin_number;
//     };

//     // support function for pin value update interrupt handling

//     void adc_has_new_value(bool err) {
// //        float raw_adc_value = pin.getRaw();
// //        history.add_sample(raw_adc_value);
//     };
// };

#endif // honeywell_trustability_ssc_h
