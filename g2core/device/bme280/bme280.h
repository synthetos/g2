/*
 * bme820.h - suppport for talking to the Bosch BME820 pressure/humidity/temperature sensor amp/ADC
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

// Many thanks to Adafruit!
// More specifically, for their driver at
//   https://github.com/adafruit/Adafruit_BME280_Library
// and their breakout board at https://adafru.it/2652

#ifndef bme820_h
#define bme820_h

#include "MotateSPI.h"
#include "MotateBuffer.h"
#include "MotateUtilities.h" // for to/fromLittle/BigEndian
#include "util.h" // for fp_ZERO

// Complete class for BME280 drivers.
template <typename device_t>
struct BME280 final {
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

    // Constructor - this is the only time we directly use the SBIBus
    template <typename SPIBus_t, typename chipSelect_t>
    BME280(SPIBus_t &spi_bus, const chipSelect_t &_cs)
        : _device{spi_bus.getDevice(_cs, 5000000, SPIDeviceMode::kSPIMode0 | SPIDeviceMode::kSPI8Bit,
                                    400,  // min_between_cs_delay_ns
                                    400,  // cs_to_sck_delay_ns
                                    80    // between_word_delay_ns
                                    )} {
        init();
    };

    template <typename SPIBus_t, typename chipSelect_t>
    BME280(const Motate::PinOptions_t options,  // completely ignored, but for compatibility with ADCPin
           std::function<void(bool)> &&_interrupt, SPIBus_t &spi_bus, const chipSelect_t &_cs)
        : _device{spi_bus.getDevice(_cs, 5000000, SPIDeviceMode::kSPIMode0 | SPIDeviceMode::kSPI8Bit,
                                    400,  // min_between_cs_delay_ns
                                    400,  // cs_to_sck_delay_ns
                                    80    // between_word_delay_ns
                                    )},
          _interrupt_handler{std::move(_interrupt)} {
        init();
    };

    // Prevent copying, and prevent moving (so we know if it happens)
    BME280(const BME280 &) = delete;
    BME280(BME280 &&other) : _device{std::move(other._device)} {};


    // ###########
    // From here on we store actual values from the BME280, and marshall data
    // from the in_buffer buffer to them, or from the values to the out_buffer.

    // Note that this includes _startNextReadWrite() and _doneReadingCallback(),
    // which are what calls the functions to put data into the out_buffer and
    // read data from the in_buffer, respectively.

    // Also, _init() is last, so it can setup a newly created BME280 object.

    enum {
        BME280_REGISTER_DIG_T1 = 0x88,
        BME280_REGISTER_DIG_T2 = 0x8A,
        BME280_REGISTER_DIG_T3 = 0x8C,

        BME280_REGISTER_DIG_P1 = 0x8E,
        BME280_REGISTER_DIG_P2 = 0x90,
        BME280_REGISTER_DIG_P3 = 0x92,
        BME280_REGISTER_DIG_P4 = 0x94,
        BME280_REGISTER_DIG_P5 = 0x96,
        BME280_REGISTER_DIG_P6 = 0x98,
        BME280_REGISTER_DIG_P7 = 0x9A,
        BME280_REGISTER_DIG_P8 = 0x9C,
        BME280_REGISTER_DIG_P9 = 0x9E,

        BME280_REGISTER_DIG_H1 = 0xA1,
        BME280_REGISTER_DIG_H2 = 0xE1,
        BME280_REGISTER_DIG_H3 = 0xE3,
        BME280_REGISTER_DIG_H4 = 0xE4,
        BME280_REGISTER_DIG_H5 = 0xE5,
        BME280_REGISTER_DIG_H6 = 0xE7,

        BME280_REGISTER_CHIPID = 0xD0,
        BME280_REGISTER_VERSION = 0xD1,
        BME280_REGISTER_SOFTRESET = 0xE0,

        BME280_REGISTER_CAL26 = 0xE1, // R calibration stored in 0xE1-0xF0

        BME280_REGISTER_CONTROLHUMID = 0xF2,
        BME280_REGISTER_STATUS = 0XF3,
        BME280_REGISTER_CONTROL = 0xF4,
        BME280_REGISTER_CONFIG = 0xF5,
        BME280_REGISTER_PRESSUREDATA = 0xF7,
        BME280_REGISTER_TEMPDATA = 0xFA,
        BME280_REGISTER_HUMIDDATA = 0xFD
    };

    enum {
        INITING,
        NEED_CALIBRATION_READ,
        NEEDS_CONFIGURED,
        WAITING_FOR_SAMPLE,
        NEEDS_SAMPLED,
    } _state;

    bool _status_needs_read = false;
    static const size_t bme280_status_data_size = 1;
    struct bme280_status_t {
        uint8_t address;
        union {
            uint8_t raw_status;
            struct {
                uint8_t im_update : 1;
                int8_t unused_0 : 1;
                uint8_t measuring : 1;
            };
        };
    };
    alignas(4) bme280_status_t bme280_status;
    void _postReadStatus() {
        if (INITING == _state && bme280_status.im_update == 0) {
            // it's done reading the calibration data into "image registers"
            // and we were waiting for that to finish the first time
            _state = NEED_CALIBRATION_READ;
        }

        // if (WAITING_FOR_SAMPLE == _state && bme280_status.im_update == 0 && bme280_status.measuring == 0) {
        //     // it's done reading the calibration data into "image registers"
        //     // and we were waiting for that to finish the first time
        //     _state = NEEDS_SAMPLED;
        //     _sample_data_needs_read = true;
        // }
    };

    bool _calibration0_needs_read = false;
    static const size_t bme280_calib_data0_size = 25;
    struct bme280_calib_data0_t {
        uint8_t address;
        union {
            uint8_t raw_bme280_calib_data0[25];
            struct {
                uint16_t dig_T1;  ///< temperature compensation value
                int16_t dig_T2;   ///< temperature compensation value
                int16_t dig_T3;   ///< temperature compensation value

                uint16_t dig_P1;  ///< pressure compensation value
                int16_t dig_P2;   ///< pressure compensation value
                int16_t dig_P3;   ///< pressure compensation value
                int16_t dig_P4;   ///< pressure compensation value
                int16_t dig_P5;   ///< pressure compensation value
                int16_t dig_P6;   ///< pressure compensation value
                int16_t dig_P7;   ///< pressure compensation value
                int16_t dig_P8;   ///< pressure compensation value
                int16_t dig_P9;   ///< pressure compensation value

                uint8_t dig_H1;  ///< humidity compensation value
            };
        };
    } __attribute__((packed));
    alignas(4) bme280_calib_data0_t bme280_calib_data0;


    bool _calibration1_needs_read = false;
    static const size_t bme280_calib_data1_size = 8;
    struct bme280_calib_data1_t {
        uint8_t address;
        union {
            uint8_t raw_bme280_calib_data1[8];
            struct {
                int16_t dig_H2; ///< humidity compensation value
                uint8_t dig_H3; ///< humidity compensation value
                int8_t dig_H4; ///< humidity compensation value
                int16_t dig_H5; ///< humidity compensation value
                int8_t dig_H6;  ///< humidity compensation value
            };
        };
    } __attribute__((packed));
    alignas(4) bme280_calib_data1_t bme280_calib_data1;

    double dig_T1;
    double dig_T2;
    double dig_T3;

    double dig_P1;
    double dig_P2;
    double dig_P3;
    double dig_P4;
    double dig_P5;
    double dig_P6;
    double dig_P7;
    double dig_P8;
    double dig_P9;

    // double dig_H1;
    // double dig_H2;
    // double dig_H3;
    // double dig_H4;
    // double dig_H5;
    // double dig_H6;

    void _postReadCalibration() {
        using Motate::fromLittleEndian;

        // Now to rearrange the values we got into something usable
        dig_T1 = fromLittleEndian((uint16_t)bme280_calib_data0.dig_T1);
        dig_T2 = fromLittleEndian((uint16_t)bme280_calib_data0.dig_T2);
        dig_T3 = fromLittleEndian((uint16_t)bme280_calib_data0.dig_T3);

        dig_P2 = fromLittleEndian((uint16_t)bme280_calib_data0.dig_P2);
        dig_P3 = fromLittleEndian((uint16_t)bme280_calib_data0.dig_P3);
        dig_P4 = fromLittleEndian((uint16_t)bme280_calib_data0.dig_P4);
        dig_P1 = fromLittleEndian((uint16_t)bme280_calib_data0.dig_P1);
        dig_P5 = fromLittleEndian((uint16_t)bme280_calib_data0.dig_P5);
        dig_P6 = fromLittleEndian((uint16_t)bme280_calib_data0.dig_P6);
        dig_P7 = fromLittleEndian((uint16_t)bme280_calib_data0.dig_P7);
        dig_P8 = fromLittleEndian((uint16_t)bme280_calib_data0.dig_P8);
        dig_P9 = fromLittleEndian((uint16_t)bme280_calib_data0.dig_P9);

        // the himidity values get mangled, as they are sized different than their memory layout

        // TODO - fix this is we read humidity!!

        // bme280_calib_data1.dig_H2 = fromBigEndian(bme280_calib_data1.dig_H2);
        // bme280_calib_data1.dig_H4 = fromBigEndian(bme280_calib_data1.dig_H4);
        // bme280_calib_data1.dig_H5 = fromBigEndian(bme280_calib_data1.dig_H5);

        _state = NEEDS_CONFIGURED;
    }


    bool _sample_data_needs_read = false;
    static const size_t bme280_sample_data_size = 8;
    struct bme280_sample_data_t {
        uint8_t address;
        union {
            uint8_t raw_bme280_sample_data[8];
            struct {
                uint8_t press_msb;
                uint8_t press_lsb;
                uint8_t press_xlsb;
                uint8_t temp_msb;
                uint8_t temp_lsb;
                uint8_t temp_xlsb;
                uint8_t hum_msb;
                uint8_t hum_lsb;
            };
        };
    } __attribute__((packed));
    alignas(4) bme280_sample_data_t bme280_sample_data;

    double temperature = 0;
    double pressure = 0;

    void _postReadSampleData() {
        double var1, var2, T;
        uint32_t adc_T_int = ((uint32_t)bme280_sample_data.temp_msb << 12) | ((uint32_t)bme280_sample_data.temp_lsb << 4) | ((uint32_t)bme280_sample_data.temp_xlsb >> 4);
        double adc_T = adc_T_int;

        uint32_t adc_P_int = ((uint32_t)bme280_sample_data.press_msb << 12) | ((uint32_t)bme280_sample_data.press_lsb << 4) | ((uint32_t)bme280_sample_data.press_xlsb >> 4);
        double adc_P = adc_P_int;

        var1 = ((adc_T) / 16384.0 - (dig_T1) / 1024.0) * (dig_T2);
        var2 = (((adc_T) / 131072.0 - (dig_T1) / 8192.0) *
                ((adc_T) / 131072.0 - (dig_T1) / 8192.0)) *
               (dig_T3);
        // t_fine = (BME280_S32_t)(var1 + var2);
        temperature = (var1 + var2) / 5120.0;
        // return T;

        double p;
        var1 = ((var1 + var2) / 2.0) - 64000.0;
        var2 = var1 * var1 * (dig_P6) / 32768.0;
        var2 = var2 + var1 * (dig_P5) * 2.0;
        var2 = (var2 / 4.0) + ((dig_P4) * 65536.0);
        var1 = ((dig_P3) * var1 * var1 / 524288.0 + (dig_P2) * var1) / 524288.0;
        var1 = (1.0 + var1 / 32768.0) * (dig_P1);
        if (fp_ZERO(var1)) {
            pressure = 0;  // avoid exception caused by division by zero
            return;
        }
        p = 1048576.0 - adc_P;
        p = (p - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = (dig_P9) * p * p / 2147483648.0;
        var2 = p * (dig_P8) / 32768.0;
        p = p + (var1 + var2 + (dig_P7)) / 16.0;

        pressure = p;
    };

    alignas(4) uint8_t _scribble_buffer[36];


    bool _configuration_is_ready_to_write = false;
    static const size_t bme280_configuration_size = 2;
    struct bme280_config_t {
        uint8_t address;
        union {
            uint8_t raw_bme280_sample_data[2];
            struct {
                uint8_t ctrl_meas;
                uint8_t config;
            };
            struct {
                int8_t ctrl_meas_mode : 2;
                int8_t ctrl_meas_osrs_p : 3;
                int8_t ctrl_meas_osrs_t : 3;
                // next byte
                int8_t config_spi3w_en : 1;
                int8_t unused_0 : 1;
                int8_t config_filter : 3;
                int8_t config_t_sb : 3;
            };
        };
    } __attribute__((packed));
    alignas(4) bme280_config_t bme280_config;

    void _prepareConfiguration() {
        bme280_config.ctrl_meas_mode   = 0b11;  // normal mode
        bme280_config.ctrl_meas_osrs_p = 0b010;  // pressure: 2x oversampling
        bme280_config.ctrl_meas_osrs_t = 0b010;  // temperature: 2x oversampling

        bme280_config.config_spi3w_en  = 0b0;    // stay in 4-wire mode
        bme280_config.config_filter    = 0b010;  // filter coeffiecent: 4
        bme280_config.config_t_sb      = 0b000;  // 0.5ms standby between measurements

        _configuration_is_ready_to_write = true;
    }

    void _postConfiguration() {
        _state = WAITING_FOR_SAMPLE;
    }

    void _startNextReadWrite()
    {
        if (_transmitting || !_inited) { return; }
        _transmitting = true; // preemptively say we're transmitting .. as a mutex

        // We request the next register, and keep track of how long it is.
        uint8_t next_reg = 0;
        uint8_t *data_buffer = nullptr;
        int8_t register_size;

        // We write before we read -- so we don't lose what we set in the registers when writing

        // check if we need to write registers
        if (_configuration_is_ready_to_write)  { next_reg = BME280_REGISTER_CONTROL & ~0x80;  data_buffer = (uint8_t*)&bme280_config; register_size = bme280_configuration_size;  _configuration_is_ready_to_write = false; } else
        // if (_fault_low_needs_written)   { next_reg = 0x80 | LFAULT_reg;  data_buffer = (uint8_t*)&_fault_low;  register_size = 2;  _fault_low_needs_written = false;  } else

        // check if we need to read reagisters
        if (_calibration0_needs_read)   { next_reg = BME280_REGISTER_DIG_T1;        data_buffer = (uint8_t*)&bme280_calib_data0;   register_size = bme280_calib_data0_size;  _calibration0_needs_read = false; } else
        if (_calibration1_needs_read)   { next_reg = BME280_REGISTER_DIG_H2;        data_buffer = (uint8_t*)&bme280_calib_data1;   register_size = bme280_calib_data1_size;  _calibration1_needs_read = false; } else
        if (_sample_data_needs_read)    { next_reg = BME280_REGISTER_PRESSUREDATA;  data_buffer = (uint8_t*)&bme280_sample_data;   register_size = bme280_sample_data_size;  _sample_data_needs_read = false; } else
        if (_status_needs_read)         { next_reg = BME280_REGISTER_STATUS;        data_buffer = (uint8_t*)&bme280_status;        register_size = bme280_status_data_size;  _status_needs_read = false; } else

        // otherwise we're done here
        {
            _active_register = -1;
            _transmitting = false; // we're not really transmitting.
            return;
        }

        _active_register = next_reg;
        *data_buffer = next_reg;

        if (next_reg & 0x80) {
            // reading, prepare the address in the scribble buffer
            _scribble_buffer[0] = data_buffer[0];
            _message.setup(_scribble_buffer, data_buffer, 1+register_size, SPIMessage::DeassertAfter, SPIMessage::EndTransaction);

        } else {
            // writing
            _message.setup(data_buffer, _scribble_buffer, 1+register_size, SPIMessage::DeassertAfter, SPIMessage::EndTransaction);
        }
        _device.queueMessage(&_message);
    };

    void _doneReadingCallback()
    {
        _transmitting = false;

        // Check to make sure it was a read, and handle it accordingly
        switch (_active_register) {
            case BME280_REGISTER_DIG_H2: _postReadCalibration(); break;
            case BME280_REGISTER_STATUS: _postReadStatus(); break;
            case (BME280_REGISTER_CONTROL & ~0x80): _postConfiguration(); break;
            case BME280_REGISTER_PRESSUREDATA: _postReadSampleData(); break;

            default:
                break;
        }

        _active_register = -1;
        _startNextReadWrite();
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
        if (_check_timer.isPast()) {
            if (INITING == _state) {
                _status_needs_read = true;
                _check_timer.set(0);
                _startNextReadWrite();
                // _state is updated to NEED_CALIBRATION_READ in _postReadState()
            }
            else if (NEED_CALIBRATION_READ == _state) {
                _calibration0_needs_read = true;
                _calibration1_needs_read = true;

                _check_timer.set(0);
                _startNextReadWrite();
                // _state is updated to NEEDS_CONFIGURED in _postReadCalibration()
            }
            else if (NEEDS_CONFIGURED == _state) {
                _check_timer.set(0);
                _prepareConfiguration();
                _startNextReadWrite();
                // _state is updated to WAITING_FOR_SAMPLE in _postConfiguration()
            }
            else if (NEEDS_SAMPLED == _state) {
                _sample_data_needs_read = true;

                _check_timer.set(7);
                _startNextReadWrite();
                _state = WAITING_FOR_SAMPLE;
            }
            else if (WAITING_FOR_SAMPLE == _state) {
                _check_timer.set(0);
                _startNextReadWrite();
                _state = NEEDS_SAMPLED;
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

#endif // bme820_h
