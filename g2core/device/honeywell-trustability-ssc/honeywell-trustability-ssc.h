/*
 * honeywell-trustability-ssc.h - suppport for talking to the Honeywell TruStability SSC line of pressure/temperature
 * sensors This file is part of the G2 project
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

#include "MotateBuffer.h"
#include "MotateSPI.h"
#include "MotateTWI.h"
#include "MotateUtilities.h"  // for to/fromLittle/BigEndian
#include "util.h"             // for fp_ZERO

enum class PressureUnits { PSI, cmH2O, inH20, Pa, kPa };

// ToDo - move this to somewhere that is can be used by other pressure sensors
struct PressureSensor {
    virtual double getPressure(const PressureUnits output_units) const;
};

// See https://en.wikipedia.org/wiki/Standard_litre_per_minute
enum class FlowUnits { LPM, SLM, NLPM };
struct FlowSensor {
    // get the pressure value from the underlying pressure sensor, if any
    virtual double getPressure(const PressureUnits output_units) const;

    // get the flow value in the units requested
    virtual double getFlow(const FlowUnits output_units) const;
};

/* VenturiFlowSensor
 * Takes a differentail PressureSensor and some base parameters for a venturi tube and outputs flow values.
 */
class VenturiFlowSensor : public FlowSensor {
    PressureSensor* ps;

    double K; // computed from the above

/*
    double upstream_diameter_mm;
    double throat_diameter_mm;
    double air_density;            // kg / m^2
    double discharge_coeffiecient; // percent/100, IOW 0.0-1.0
*/
    static double compute_k(double upstream_diameter_mm, double throat_diameter_mm, double air_density = 1.2431, double discharge_coeffiecient = 0.95) {
        // "don't just do something, stand there" - we'll be verbose and let the compiler optimize it
        double upstream_radius_m = (upstream_diameter_mm / 1000.0) / 2.0; // radius in meters
        double area_upstream = upstream_radius_m * upstream_radius_m * M_PI; // area in m^2

        double throat_radius_m = (throat_diameter_mm / 1000.0) / 2.0; // radius in meters
        double area_throat = throat_radius_m * throat_radius_m * M_PI; // area in m^2

        double area_ratio = area_upstream / area_throat;

        // K = C_disc * SQRT( 2 / dens ) * area_A/SQRT( (area_A/area_B)^2 - 1 ) * 1000

        return  discharge_coeffiecient * sqrt(2.0 / air_density) * area_upstream / sqrt((area_ratio * area_ratio) - 1) *
            1000.0;
    }

   public:
    VenturiFlowSensor(PressureSensor *ps_, double K_)
        : ps{ps_},
          K{K_} {
    }

    double getFlow(const FlowUnits output_units) const override {
        double pressure_diff = ps->getPressure(PressureUnits::Pa);
        double flow = K * sqrt(std::abs(pressure_diff)) * (pressure_diff < 0 ? -1 : 1);

        // is SLM right?
        if (output_units == FlowUnits::LPM || output_units == FlowUnits::SLM) {
            return flow * 60.0;
        }

        // this is WRONG, but shuts the compiler up:
        return 0.0;
    }

    double getPressure(const PressureUnits output_units) const override { return ps->getPressure(output_units); }
};


class HoneywellTruStabilityBase : virtual public PressureSensor {
   public:
    HoneywellTruStabilityBase(const uint16_t min_output, const uint16_t max_output, const double min_value,
                        const double max_value, const PressureUnits base_units)
        : _min_output{min_output},
          _max_output{max_output},
          _min_value{min_value},
          _max_value{max_value},
          _base_units{base_units} {};

    // Prevent copying, and prevent moving (so we know if it happens)
    HoneywellTruStabilityBase(const HoneywellTruStabilityBase &) = delete;

   protected:
    std::function<void(bool)> _interrupt_handler;

    // Parameters of the sensor - for now, compile-time
    const uint16_t _min_output;
    const uint16_t _max_output;
    const double _min_value;
    const double _max_value;
    const PressureUnits _base_units;

    double zero_offset;

    // ###########
    // The HoneyWell TruStability SSC devices are really simple to talk to, and can be in SPI or I2C configurations:

    // SPI: See https://sensing.honeywell.com/spi-comms-digital-ouptu-pressure-sensors-tn-008202-3-en-final-30may12.pdf
    // Read from 2 to 4 byts of data, interpret it, repeat.
    // The bytes are: Status_and_BridgeData_MSB, BridgeData_LSB, TemperatureData_MSB, TemperatureData_LSB

    // I2C: See https://sensing.honeywell.com/i2c-comms-digital-output-pressure-sensors-tn-008201-3-en-final-30may12.pdf
    // Same data as SPI

    enum {
        INITING,
        WAITING_FOR_SAMPLE,
        NEEDS_SAMPLED,
    } _state;

    bool _data_needs_read = false;
    static const size_t _data_size = 4;
    struct data_t {
        union {
            uint8_t raw_data[8];
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
            _data_needs_read = true;
            return;
        }

        int32_t bridge_output = (_data.bridge_msb << 8) | _data.bridge_lsb;
        // uint32_t temperature_output = (_data.temperature_msb << 3) | _data.temperature_lsb;

        // Formula
        // ((ouput - output_min)*(pressure_max-pressure_min))/(outout_max-output_min)+pressure_min

        if (bridge_output < _min_output || bridge_output > _max_output) {
            pressure = 0.0;
            // error condition
        }

        double temp_pressure =
            ((double)(bridge_output - _min_output) * (double)(_max_value - _min_value)) / (_max_output - _min_output) +
            _min_value;

        const double noise_min = ((_max_value - _min_value) * 0.0025);

        // if we're within 0.25% of zero, adjust the zero offset - slowly
        if ((temp_pressure > -noise_min) && (temp_pressure < noise_min)) {
            zero_offset = (zero_offset * 0.999) + (temp_pressure * 0.001);
        }

        pressure = (pressure * 0.9) + ((temp_pressure - zero_offset) * 0.1);
        // pressure = temp_pressure - zero_offset;

        if (_interrupt_handler) {
            _interrupt_handler(true);
            _interrupt_handler = nullptr;
        }
    };

   public:
    double getPressure(const PressureUnits output_units) const override {
        if (output_units != _base_units) {
            if (_base_units == PressureUnits::PSI && output_units == PressureUnits::cmH2O) {
                // the only currently supported configuration
                return pressure * 70.3069578296;
            }

            if (_base_units == PressureUnits::PSI && output_units == PressureUnits::Pa) {
                // the only currently supported configuration
                return pressure * 6894.75729;
            }
        }

        return pressure;
    };
};

// empty template

template <typename device_t, typename enable = void>
struct HoneywellTruStability : private HoneywellTruStabilityBase {}; // marked private to force an error

// Final SPI class
template <typename device_t>
struct HoneywellTruStability<device_t, std::enable_if_t<std::is_base_of_v<Motate::SPIBusDeviceBase, device_t>>> : public HoneywellTruStabilityBase {
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

    // Constructor - this is the only time we directly use the SBIBus
    template <typename SPIBus_t, typename chipSelect_t>
    HoneywellTruStability(SPIBus_t &spi_bus, const chipSelect_t &_cs, const uint16_t min_output, const uint16_t max_output,
                    const double min_value, const double max_value, const PressureUnits base_units)
        : HoneywellTruStabilityBase{min_output, max_output, min_value, max_value, base_units},
          _device{spi_bus.getDevice(_cs, 5000000, SPIDeviceMode::kSPIMode0 | SPIDeviceMode::kSPI8Bit,
                                    400,  // min_between_cs_delay_ns
                                    400,  // cs_to_sck_delay_ns
                                    80    // between_word_delay_ns
                                    )} {
        init();
    };

    // Prevent copying, and prevent moving (so we know if it happens)
    HoneywellTruStability(const HoneywellTruStability &) = delete;
    HoneywellTruStability(HoneywellTruStability &&other) : _device{std::move(other._device)} {};

    alignas(4) uint8_t _scribble_buffer[8];

    void _startNextReadWrite() {
        if (_transmitting || !_inited) {
            return;
        }
        _transmitting = true;  // preemptively say we're transmitting .. as a mutex

        // check if we need to read reagisters
        if (!_data_needs_read) {
            _transmitting = false;  // we're not really transmitting.
            return;
        }
        _data_needs_read = false;

        // reading, prepare the address in the scribble buffer
        _message.setup(_scribble_buffer, (uint8_t *)&_data.raw_data, 4, SPIMessage::DeassertAfter, SPIMessage::EndTransaction);
        _device.queueMessage(&_message);
    };

    void _doneReadingCallback() {
        _transmitting = false;
        _postReadSampleData();
    };

    void init() {
        _message.message_done_callback = [&] { this->_doneReadingCallback(); };

        _inited = true;
    };


    void startSampling(std::function<void(bool)> &&handler) {
        _interrupt_handler = std::move(handler);

        _data_needs_read = true;
        _startNextReadWrite();
    };
};

// Final TWI class
template <typename device_t>
struct HoneywellTruStability<device_t, std::enable_if_t<std::is_base_of_v<Motate::TWIBusDeviceBase, device_t>>> : public HoneywellTruStabilityBase {
    using TWIMessage = Motate::TWIMessage;
    using TWIInterrupt = Motate::TWIInterrupt;

    // SPI and message handling properties
    device_t _device;
    TWIMessage _message;

    // Record if we're transmitting to prevent altering the buffers while they
    // are being transmitted still.
    volatile bool _transmitting = false;

    // We don't want to transmit until we're inited
    bool _inited = false;

    // Constructor - this is the only time we directly use the TWIBus
    template <typename TWIBus_t, typename... Ts>
    HoneywellTruStability(TWIBus_t &twi_bus, const uint8_t &address, Ts... v, const uint16_t min_output, const uint16_t max_output,
                    const double min_value, const double max_value, const PressureUnits base_units)
        : HoneywellTruStabilityBase{min_output, max_output, min_value, max_value, base_units},
          _device{twi_bus.getDevice({address, Motate::TWIDeviceAddressSize::k7Bit}, v...)} {
        init();
    };

    // Prevent copying, and prevent moving (so we know if it happens)
    HoneywellTruStability(const HoneywellTruStability &) = delete;
    HoneywellTruStability(HoneywellTruStability &&other) : _device{std::move(other._device)} {};

    void _startNextReadWrite() {
        if (_transmitting || !_inited) {
            return;
        }
        _transmitting = true;  // preemptively say we're transmitting .. as a mutex

        // check if we need to read reagisters
        if (!_data_needs_read) {
            _transmitting = false;  // we're not really transmitting.
            return;
        }
        _data_needs_read = false;

        using dir = TWIMessage::Direction;

        // reading, prepare the address in the scribble buffer
        _message.setup((uint8_t *)&_data.raw_data, 4, dir::kRX);
        _device.queueMessage(&_message);
    };

    void _doneReadingCallback() {
        _transmitting = false;
        _postReadSampleData();
    };

    void init() {
        _message.message_done_callback = [&](bool){ this->_doneReadingCallback(); };

        _inited = true;
    };

    void startSampling(std::function<void(bool)> &&handler) {
        _interrupt_handler = std::move(handler);
        _data_needs_read = true;
        _startNextReadWrite();
    };
};

#endif  // honeywell_trustability_ssc_h
