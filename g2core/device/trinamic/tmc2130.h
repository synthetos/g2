/*
 * trinamic/tmc2130.h - control over a Trinamic TMC2130 stepper motor driver
 * This file is part of the G2 project
 *
 * Copyright (c) 2016-2019 Alden S. Hart, Jr.
 * Copyright (c) 2016-2019 Robert Giseburt
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

#include "controller.h"
#include "json_parser.h"       // for nv, etc
#include "text_parser.h"       // for txt_* commands

#include "stepper.h"

#include "MotateSPI.h"
#include "MotateBuffer.h"
#include "MotateUtilities.h" // for to/fromLittle/BigEndian

using Motate::OutputPin;
using Motate::kStartHigh;
using Motate::SPIMessage;
using Motate::SPIInterrupt;
using Motate::SPIDeviceMode;
using Motate::fromBigEndian;
using Motate::toBigEndian;

// Complete class for Trinamic2130 drivers.
// It's also a proper Stepper object.
template <typename device_t,
          pin_number step_num,
          pin_number dir_num,
          pin_number enable_num>
struct Trinamic2130 final : Stepper {
    typedef Trinamic2130<device_t, step_num, dir_num, enable_num> type;

    Timeout _motor_activity_timeout;         // this is the timeout object that will let us know when time is up
    uint32_t _motor_activity_timeout_ms;     // the number of ms that the timeout is reset to
    enum stPowerState {                          // used w/start and stop flags to sequence motor power
        MOTOR_OFF = 0,                      // motor is stopped and deenergized
        MOTOR_IDLE,                         // motor is stopped and may be partially energized for torque maintenance
        MOTOR_RUNNING,                      // motor is running (and fully energized)
        MOTOR_POWER_TIMEOUT_START,          // transitional state to start power-down timeout
        MOTOR_POWER_TIMEOUT_COUNTDOWN       // count down the time to de-energizing motor
    } _power_state;              // state machine for managing motor power
    stPowerMode _power_mode;                // See stPowerMode for values

    // Pins that are directly managed
    OutputPin<step_num> _step;
    OutputPin<dir_num> _dir;
    OutputPin<enable_num> _enable {kStartHigh};

    // SPI and message handling properties
    device_t _device;
    SPIMessage _message;

    // Create the type of a buffer
    struct trinamic_buffer_t {
        volatile union {
            volatile char raw_data[8];
            volatile struct {
                volatile union {
                    uint8_t addr;
                    uint8_t status;
                };
                volatile uint32_t value;
            } __attribute__((packed));
        };
    } __attribute__((packed));

    // And make two statically allocated buffers
    alignas(4) volatile trinamic_buffer_t out_buffer;
    alignas(4) volatile trinamic_buffer_t in_buffer;

    // For debugging overwrites:
    // char end_guard[9] = "DEADBEEF";

    // Record if we're transmitting to prevent altering the buffers while they
    // are being transmitted still.
    volatile bool _transmitting = false;

    // We don't want to transmit until we're inited
    bool _inited = false;

    // Record what register we just requested, so we know what register the
    // the response is for (and to read the response.)
    volatile int16_t _register_thats_reading = -1;

    // We need to have a flag for when we are doing a read *just* to get the
    // data requested. Otherwise we'll loop forever.
    bool _reading_only = false;

    // Timer to keep track of when we need to do another periodic update
    Motate::Timeout check_timer;

    // Constructor - this is the only time we directly use the SBIBus
    template <typename SPIBus_t, typename chipSelect_t>
    Trinamic2130(SPIBus_t &spi_bus, const chipSelect_t &_cs) :
        _device{spi_bus.getDevice(_cs,
                                  4000000, //4MHz
                                  SPIDeviceMode::kSPIMode0 | SPIDeviceMode::kSPI8Bit,
                                  1, // min_between_cs_delay_ns
                                  10, // cs_to_sck_delay_ns
                                  1  // between_word_delay_ns
                                  )}
    {};

    // Prevent copying, and prevent moving (so we know if it happens)
    Trinamic2130(const Trinamic2130 &) = delete;
    Trinamic2130(Trinamic2130 &&other) : _device{std::move(other._device)} {};


    // ############
    // Stepper override functions

    // We can leave this as is:
    // bool canStep() override { return true; };

    void setMicrosteps(const uint16_t microsteps) override
    {
        switch (microsteps) {
            case (  0): { CHOPCONF.MRES = 0; break; } // 256
            case (  1): { CHOPCONF.MRES = 8; break; }
            case (  2): { CHOPCONF.MRES = 7; break; }
            case (  4): { CHOPCONF.MRES = 6; break; }
            case (  8): { CHOPCONF.MRES = 5; break; }
            case ( 16): { CHOPCONF.MRES = 4; break; }
            case ( 32): { CHOPCONF.MRES = 3; break; }
            case ( 64): { CHOPCONF.MRES = 2; break; }
            case (128): { CHOPCONF.MRES = 1; break; }
            case (256): { CHOPCONF.MRES = 0; break; }
            default: return;
        }
        CHOPCONF_needs_written = true;
        _startNextReadWrite();
    };

    void enableWithTimeout(float timeout_ms) override
    {
        if (_power_mode == MOTOR_DISABLED || _power_state == MOTOR_RUNNING) {
            return;
        }

        if (timeout_ms < 0.1) {
            timeout_ms = _motor_activity_timeout_ms;
        }

        _power_state = MOTOR_POWER_TIMEOUT_COUNTDOWN;
        if (_power_mode == MOTOR_POWERED_IN_CYCLE || _power_mode == MOTOR_POWER_REDUCED_WHEN_IDLE) {
            _motor_activity_timeout.set(timeout_ms);
        }

        if (!_enable.isNull()) {
            _enable.clear();
        }
    };

    void _enableImpl() override {
        if (_power_mode == MOTOR_DISABLED || _power_state == MOTOR_RUNNING) {
            return;
        }

        _enable.clear();

        _power_state = MOTOR_RUNNING;
    };

    void _disableImpl() override {
        if (this->getPowerMode() == MOTOR_ALWAYS_POWERED) {
            return;
        }

        _enable.set();

        _motor_activity_timeout.clear();
        _power_state = MOTOR_OFF;
    };

    void stepStart() override { _step.set(); };

    void stepEnd() override { _step.clear(); };

    void setDirection(uint8_t new_direction) override
    {
        if (new_direction == DIRECTION_CW) {
            _dir.clear();
        } else {
            _dir.set(); // set the bit for CCW motion
        }
    };

    virtual void setPowerMode(stPowerMode new_pm)
    {
        _power_mode = new_pm;
        if (_power_mode == MOTOR_ALWAYS_POWERED) {
            enable();
        } else if (_power_mode == MOTOR_DISABLED) {
            disable();
        }
    };

    stPowerMode getPowerMode() override
    {
         return _power_mode;
    };

    void setPowerLevels(float active_pl, float idle_pl) override
    {
        // scale the 0.0-1.0 to 0-31
        IHOLD_IRUN.IRUN = (active_pl * 31.0);
        IHOLD_IRUN.IHOLD = (idle_pl * 31.0);

        IHOLD_IRUN_needs_written = true;
        _startNextReadWrite();
    };


    // turn off motor is only powered when moving
    // HOT - called from the DDA interrupt
    void motionStopped() override //HOT_FUNC
    {
        if (_power_mode == MOTOR_POWERED_IN_CYCLE) {
            this->enable();
        } else if (_power_mode == MOTOR_POWER_REDUCED_WHEN_IDLE) {
            _power_state = MOTOR_POWER_TIMEOUT_START;
        } else if (_power_mode == MOTOR_POWERED_ONLY_WHEN_MOVING) {
            if (_power_state == MOTOR_RUNNING) {
                // flag for periodicCheck - not actually using a timeout
                _power_state = MOTOR_POWER_TIMEOUT_START;
            }
        }
    };

    virtual void setActivityTimeout(float idle_milliseconds) override
    {
        _motor_activity_timeout_ms = idle_milliseconds;
    };

    // Note that init() and periodicCheck(bool have_actually_stopped) are both below


    // ############
    // Actual Trinamic2130 protocol functions follow

//    // Request reading a register
//    void readRegister(uint8_t reg) {
//        _registers_to_access.write(reg);
//        _startNextReadWrite();
//    };
//
//    // Request writing to a register
//    void writeRegister(uint8_t reg) {
//        _registers_to_access.write(reg | 0x80);
//        _startNextReadWrite();
//    };


    // ###########
    // From here on we store actual values from the trinamic, and marshall data
    // from the in_buffer buffer to them, or from the values to the out_buffer.

    // Note that this includes _startNextReadWrite() and _doneReadingCallback(),
    // which are what calls the functions to put data into the out_buffer and
    // read data from the in_buffer, respectively.

    // Also, _init() is last, so it can setup a newly created Trinamic object.

    enum {
        GCONF_reg      = 0x00,
        GSTAT_reg      = 0x01,
        IOIN_reg       = 0x04,
        IHOLD_IRUN_reg = 0x10,
        TPOWERDOWN_reg = 0x11,
        TSTEP_reg      = 0x12,
        TPWMTHRS_reg   = 0x13,
        TCOOLTHRS_reg  = 0x14,
        THIGH_reg      = 0x15,
        XDIRECT_reg    = 0x2D,
        VDCMIN_reg     = 0x33,
        MSCNT_reg      = 0x6A,
        CHOPCONF_reg   = 0x6C,
        COOLCONF_reg   = 0x6D,
        DRV_STATUS_reg = 0x6F,
        PWMCONF_reg    = 0x70,
    };

    // IMPORTANT NOTE: The endianness of the ARM is little endian, but other processors
    //  may be different.

    uint8_t status = 0;
    union {
        volatile uint32_t value;
        //        uint8_t bytes[4];
        volatile struct {
            uint32_t i_scale_analog      : 1; // 0
            uint32_t internal_Rsense     : 1; // 1
            uint32_t en_pwm_mode         : 1; // 2
            uint32_t enc_commutation     : 1; // 3
            uint32_t shaft               : 1; // 4
            uint32_t diag0_error         : 1; // 5
            uint32_t diag0_otpw          : 1; // 6
            uint32_t diag0_stall         : 1; // 7

            uint32_t diag1_stall         : 1; // 8
            uint32_t diag1_index         : 1; // 9
            uint32_t diag1_onstate       : 1; // 10
            uint32_t diag1_steps_skipped : 1; // 11
            uint32_t diag0_int_pushpull  : 1; // 12
            uint32_t diag1_pushpull      : 1; // 13
            uint32_t small_hysteresis    : 1; // 14
        }  __attribute__ ((packed));
    } GCONF; // 0x00 - READ/WRITE
    void _postReadGConf() {
        GCONF.value = fromBigEndian(in_buffer.value);
    };
    void _prepWriteGConf() {
        out_buffer.value = toBigEndian(GCONF.value);
    };
    volatile bool GCONF_needs_read;
    volatile bool GCONF_needs_written;

    struct {
        volatile uint32_t value;
        //        uint8_t bytes[4];
        volatile struct {
            uint32_t reset      : 1; // 0
            uint32_t drv_err    : 1; // 1
            uint32_t uv_cp      : 1; // 2
        }  __attribute__ ((packed));
    } GSTAT; // 0x01 - CLEARS ON READ
    void _postReadGStat() {
        GSTAT.value = fromBigEndian(in_buffer.value);
    };
    volatile bool GSTAT_needs_read;

    union {
        volatile uint32_t value;
        //        uint8_t bytes[4];
        volatile struct {
            uint32_t STEP         : 1; // 0
            uint32_t DIR          : 1; // 1
            uint32_t DCEN_CFG4    : 1; // 2
            uint32_t DCIN_CFG5    : 1; // 3
            uint32_t DRV_ENN_CFG6 : 1; // 4
            uint32_t DCO          : 1; // 5
            uint32_t _always_1    : 1; // 6
            uint32_t _dont_care   : 1; // 7

            uint32_t _unused_0    : 8; //  8-15
            uint32_t _unused_1    : 8; // 16-23
            uint32_t CHIP_VERSION : 8; // 24-31 - should always read 0x11
        }  __attribute__ ((packed));
    } IOIN; // 0x04 - READ ONLY
    void _postReadIOIN() {
        IOIN.value = fromBigEndian(in_buffer.value);
    };
    volatile bool IOIN_needs_read;

    union {
        volatile uint32_t value;
        //        uint8_t bytes[4];
        volatile struct {
            uint32_t IHOLD      : 5; //  0- 4
            uint32_t _unused_0  : 3; //  5- 7
            uint32_t IRUN       : 5; //  8-12
            uint32_t _unused_1  : 3; // 13-15
            uint32_t IHOLDDELAY : 4; // 16-19
        }  __attribute__ ((packed));
    } IHOLD_IRUN; // 0x10 - WRITE ONLY
    void _prepWriteIHoldIRun() {
        out_buffer.value = toBigEndian(IHOLD_IRUN.value);
    };
    volatile bool IHOLD_IRUN_needs_written;

    struct {
        volatile uint32_t value;
        //        uint8_t bytes[4];
    } TPOWERDOWN; // 0x11 - WRITE ONLY
    void _prepWriteTPowerDown() {
        out_buffer.value = toBigEndian(TPOWERDOWN.value);
    };
    volatile bool TPOWERDOWN_needs_written;

    struct {
        volatile uint32_t value;
        //        uint8_t bytes[4];
    } TSTEP; // 0x12 - READ ONLY
    void _postReadTSep() {
        TSTEP.value = fromBigEndian(in_buffer.value);
    };
    volatile bool TSTEP_needs_read;

    struct {
        volatile uint32_t value;
        //        uint8_t bytes[4];
    } TPWMTHRS; // 0x13 - WRITE ONLY
    void _prepWriteTPWMTHRS() {
        out_buffer.value = toBigEndian(TPWMTHRS.value);
    };
    volatile bool TPWMTHRS_needs_written;

    struct {
        volatile uint32_t value;
        //        uint8_t bytes[4];
    } TCOOLTHRS; // 0x14 - WRITE ONLY
    void _prepWriteTCOOLTHRS() {
        out_buffer.value = toBigEndian(TCOOLTHRS.value);
    };
    volatile bool TCOOLTHRS_needs_written;

    struct {
        volatile uint32_t value;
        //        uint8_t bytes[4];
    } THIGH; // 0x15 - WRITE ONLY
    void _prepWriteTHIGH() {
        out_buffer.value = toBigEndian(THIGH.value);
    };
    volatile bool THIGH_needs_written;

    struct {
        volatile uint32_t value;
        //        uint8_t bytes[4];
    } XDIRECT; // 0x2D - READ/WRITE
    void _postReadXDirect() {
        XDIRECT.value = fromBigEndian(in_buffer.value);
    };
    void _prepWriteXDirect() {
        out_buffer.value = toBigEndian(XDIRECT.value);
    };
    volatile bool XDIRECT_needs_read;
    volatile bool XDIRECT_needs_written;

    struct {
        volatile uint32_t value;
        //        uint8_t bytes[4];
    } VDCMIN; // 0x33 - WRITE ONLY
    void _prepWriteVDCMIN() {
        out_buffer.value = toBigEndian(VDCMIN.value);
    };
    volatile bool VDCMIN_needs_written;

    struct {
        volatile uint32_t value;
        //        uint8_t bytes[4];
    } MSCNT; // 0x6A - READ ONLY
    void _postReadMSCount() {
        MSCNT.value = fromBigEndian(in_buffer.value);
    };
    volatile bool MSCNT_needs_read;

    union {
        volatile uint32_t value;
        //        uint8_t bytes[4];
        volatile struct {
            uint32_t TOFF         : 4; //  0- 3
            uint32_t HSTRT_TFD012 : 3; //  4- 6 - HSTRT when chm==0, TFD012 when chm==1
            uint32_t HEND_OFFSET  : 4; //  7-10 - HEND when chm==0, OFFSET when chm==1
            uint32_t TFD3         : 1; // 11
            uint32_t disfdcc      : 1; // 12 -- when chm==1
            uint32_t rndtf        : 1; // 13
            uint32_t chm          : 1; // 14
            uint32_t TBL          : 2; // 15-16
            uint32_t vsense       : 1; // 17
            uint32_t vhighfs      : 1; // 18
            uint32_t vhighchm     : 1; // 19
            uint32_t SYNC         : 4; // 20-23
            uint32_t MRES         : 4; // 24-27
            uint32_t intpol       : 1; // 28
            uint32_t dedge        : 1; // 29
            uint32_t diss2g       : 1; // 30
        }  __attribute__ ((packed));
    } CHOPCONF; // 0x6C- READ/WRITE
    void _postReadChopConf() {
        CHOPCONF.value = fromBigEndian(in_buffer.value);
    };
    void _prepWriteChopConf() {
        out_buffer.value = toBigEndian(CHOPCONF.value);
    };
    volatile bool CHOPCONF_needs_read;
    volatile bool CHOPCONF_needs_written;

    union {
        volatile uint32_t value;
        volatile struct {
            uint32_t semin               : 4; //  0 -  3
            uint32_t                     : 1; //  4
            uint32_t seup                : 2; //  5 -  6
            uint32_t                     : 1; //  7
            uint32_t semax               : 4; //  8 - 11
            uint32_t                     : 1; // 12
            uint32_t sedn                : 2; // 13 - 14
            uint32_t seimin              : 1; // 15

            uint32_t sgt                 : 7; // 16-22
            uint32_t                     : 1; // 23
            uint32_t sfilt               : 1; // 24
        }  __attribute__ ((packed));
    } COOLCONF; // 0x6D - WRITE ONLY
//    void _postReadCoolConf() {
//        COOLCONF.value = fromBigEndian(in_buffer.value);
//    };
    void _prepWriteCoolConf() {
        out_buffer.value = toBigEndian(COOLCONF.value);
    };
//    volatile bool COOLCONF_needs_read;
    volatile bool COOLCONF_needs_written;

    union {
        volatile uint32_t value;
        //        uint8_t bytes[4];
        volatile struct {
            uint32_t SG_RESULT    :10; //  0- 9
            uint32_t              : 5; // 10-14
            uint32_t fsactive     : 1; // 15
            uint32_t CS_ACTUAL    : 5; // 16-20
            uint32_t              : 3; // 21-23
            uint32_t stallGuard   : 1; // 24
            uint32_t ot           : 1; // 25
            uint32_t otpw         : 1; // 26
            uint32_t s2ga         : 1; // 27
            uint32_t s2gb         : 1; // 28
            uint32_t ola          : 1; // 29
            uint32_t olb          : 1; // 30
            uint32_t stst         : 1; // 31
        }  __attribute__ ((packed));
    } DRV_STATUS; // 0x6F- READ ONLY
    void _postReadDriverStatus() {
        DRV_STATUS.value = fromBigEndian(in_buffer.value);
    };
    volatile bool DRV_STATUS_needs_read;

    union {
        volatile uint32_t value;
        //        uint8_t bytes[4];
        volatile struct {
            uint32_t PWM_AMPL      : 8; //  0- 7
            uint32_t PWM_GRAD      : 8; //  8-15
            uint32_t pwm_freq      : 2; // 16-17
            uint32_t pwm_autoscale : 1; // 18
            uint32_t pwm_symmetric : 1; // 19
            uint32_t freewheel     : 2; // 20-21
        }  __attribute__ ((packed));
    } PWMCONF; // 0x70 - WRITE ONLY
    void _prepWritePWMConf() {
        out_buffer.value = toBigEndian(PWMCONF.value);
    };
    volatile bool PWMCONF_needs_written;

    void _startNextReadWrite()
    {
        if (_transmitting || !_inited) { return; }
        _transmitting = true; // preemptively say we're transmitting .. as a mutex

        // We request the next register, or re-request that we're reading (and already requested) in order to get the response.
        int16_t next_reg;

        // We write before we read -- so we don't lose what we set in the registers when writing

        in_buffer.value = 0xdeadbeaf;

        // check if we need to write registers
        if (GCONF_needs_written)      { next_reg = 0x80 | GCONF_reg;      GCONF_needs_written = false;      _prepWriteGConf();      } else
        if (IHOLD_IRUN_needs_written) { next_reg = 0x80 | IHOLD_IRUN_reg; IHOLD_IRUN_needs_written = false; _prepWriteIHoldIRun();  } else
        if (TPOWERDOWN_needs_written) { next_reg = 0x80 | TPOWERDOWN_reg; TPOWERDOWN_needs_written = false; _prepWriteTPowerDown(); } else
        if (TPWMTHRS_needs_written)   { next_reg = 0x80 | TPWMTHRS_reg;   TPWMTHRS_needs_written = false;   _prepWriteTPWMTHRS();   } else
        if (TCOOLTHRS_needs_written)  { next_reg = 0x80 | TCOOLTHRS_reg;  TCOOLTHRS_needs_written = false;  _prepWriteTCOOLTHRS();  } else
        if (THIGH_needs_written)      { next_reg = 0x80 | THIGH_reg;      THIGH_needs_written = false;      _prepWriteTHIGH();      } else
        if (XDIRECT_needs_written)    { next_reg = 0x80 | XDIRECT_reg;    XDIRECT_needs_written = false;    _prepWriteXDirect();    } else
        if (VDCMIN_needs_written)     { next_reg = 0x80 | VDCMIN_reg;     VDCMIN_needs_written = false;     _prepWriteVDCMIN();     } else
        if (CHOPCONF_needs_written)   { next_reg = 0x80 | CHOPCONF_reg;   CHOPCONF_needs_written = false;   _prepWriteChopConf();   } else
        if (COOLCONF_needs_written)   { next_reg = 0x80 | COOLCONF_reg;   COOLCONF_needs_written = false;   _prepWriteCoolConf();   } else
        if (PWMCONF_needs_written)    { next_reg = 0x80 | PWMCONF_reg;    PWMCONF_needs_written = false;    _prepWritePWMConf();    } else

        // check if we need to read reagisters
        if (GCONF_needs_read)         { next_reg = GCONF_reg;      GCONF_needs_read = false;      } else
        if (GSTAT_needs_read)         { next_reg = GSTAT_reg;      GSTAT_needs_read = false;      } else
        if (IOIN_needs_read)          { next_reg = IOIN_reg;       IOIN_needs_read = false;       } else
        if (TSTEP_needs_read)         { next_reg = TSTEP_reg;      TSTEP_needs_read = false;      } else
        if (XDIRECT_needs_read)       { next_reg = XDIRECT_reg;    XDIRECT_needs_read = false;    } else
        if (MSCNT_needs_read)         { next_reg = MSCNT_reg;      MSCNT_needs_read = false;      } else
        if (CHOPCONF_needs_read)      { next_reg = CHOPCONF_reg;   CHOPCONF_needs_read = false;   } else
//        if (COOLCONF_needs_read)      { next_reg = COOLCONF_reg;   COOLCONF_needs_read = false;   } else
        if (DRV_STATUS_needs_read)    { next_reg = DRV_STATUS_reg; DRV_STATUS_needs_read = false; } else

        // otherwise, check to see if we need to finish a read
        if (_register_thats_reading != -1) {
            next_reg = _register_thats_reading;
            _reading_only = true;
        } else

        // otherwise we're done here
        {
            _transmitting = false; // we're not really transmitting.
            return;
        }

        out_buffer.addr = (uint8_t) next_reg;
        _message.setup((uint8_t *)&out_buffer, (uint8_t *)&in_buffer, 5, SPIMessage::DeassertAfter, SPIMessage::KeepTransaction);
        _device.queueMessage(&_message);
    };

    void _doneReadingCallback()
    {
        status = in_buffer.status;
        if (_register_thats_reading != -1) {
            switch(_register_thats_reading) {
                case GCONF_reg:      _postReadGConf(); break;
                case GSTAT_reg:      _postReadGStat(); break;
                case IOIN_reg:       _postReadIOIN(); break;
                case TSTEP_reg:      _postReadTSep(); break;
                case XDIRECT_reg:    _postReadXDirect(); break;
                case MSCNT_reg:      _postReadMSCount(); break;
                case CHOPCONF_reg:   _postReadChopConf(); break;
//                case COOLCONF_reg:   _postReadCoolConf(); break;
                case DRV_STATUS_reg: _postReadDriverStatus(); break;

                default:
                    break;
            }

            _register_thats_reading = -1;
        }

        // if we just requested a read, we should record it so we know to clock
        // in the response
        if (!_reading_only && (out_buffer.addr & 0x80) == 0) {
            _register_thats_reading = out_buffer.addr;
        } else {
            // we're not waiting for a read, let another device have a transaction
            _message.immediate_ends_transaction = true;
        }
        _reading_only = false;

        _transmitting = false;
        _startNextReadWrite();
    };

    void init() override
    {
        _message.message_done_callback = [&] { this->_doneReadingCallback(); };

        // Establish default values, and then prepare to read the registers we can to establish starting values
        //        TPWMTHRS   = {0x000001F4};   TPWMTHRS_needs_written = true;
        //        PWMCONF    = {0x000401C8};   PWMCONF_needs_written = true;
        //        TPOWERDOWN = {0x0000000A};   TPOWERDOWN_needs_written = true;

        IHOLD_IRUN.IHOLD = 7;
        IHOLD_IRUN.IRUN = 30;
        IHOLD_IRUN.IHOLDDELAY = 7;
        IHOLD_IRUN_needs_written = true;

        TPOWERDOWN.value = 256;
        TPOWERDOWN_needs_written = true;

        // With a nominal 12mhz clock, 1 "tick" is 1/12000000
        // TSTEP ≥ TPWMTHRS -> go into stealthChop
        // TSTEP is ticks-per-step, so higher TSTEP means slower motion
        // So, to convert 50mm/s to TSTEP, with 40mm/rev (M) and 200fs/rev (f), we get:
        //   - convert mm to revolutions:  r=(S/M)
        //   - then revolutions to steps
        //   - then steps to 1/256th microsteps: s=(r*f*256)
        //   - then convert microsteps/sec to ticks/microstep: T=s/(1/12000000) == T=12000000/s
        // T = 12000000/((S/M)*f*256); f=200; M=40; S=20  -> T=187.5
        TPWMTHRS.value = 24;  // 400mm/s
        TPWMTHRS_needs_written = true;
        TCOOLTHRS.value = 10;  // 300mm/s
        TCOOLTHRS_needs_written = true;
        THIGH.value = 10;      // 300mm/s
        THIGH_needs_written = true;

        XDIRECT.value = 0;
        XDIRECT_needs_written = true;

        VDCMIN.value = 0;
        VDCMIN_needs_written = true;

        GCONF.i_scale_analog      = 0;
        GCONF.internal_Rsense     = 0;
        GCONF.en_pwm_mode         = 1; // enable stealthChop™
        GCONF.enc_commutation     = 0;
        GCONF.shaft               = 0;
        GCONF.diag0_error         = 0;
        GCONF.diag0_otpw          = 0;
        GCONF.diag0_stall         = 0;
        GCONF.diag1_stall         = 0;
        GCONF.diag1_index         = 0;
        GCONF.diag1_onstate       = 0;
        GCONF.diag1_steps_skipped = 0;
        GCONF.diag0_int_pushpull  = 0;
        GCONF.diag1_pushpull      = 0;
        GCONF.small_hysteresis    = 0;
        GCONF_needs_written       = true;

        CHOPCONF.TOFF = 0x4; // was 5 "For operation with stealthChop, this parameter is not used, but it is required to enable the motor."
        CHOPCONF.HSTRT_TFD012 = 0x4;
        CHOPCONF.HEND_OFFSET = 0x0; // value is 0 for -3, 1 for -2, etc.
        CHOPCONF.TFD3 = 0x0;
        CHOPCONF.disfdcc = 0x0;
        CHOPCONF.rndtf = 0x0; // enable spreadCycle™
        CHOPCONF.chm = 0x0;
        CHOPCONF.TBL = 0x1; // was 2
        CHOPCONF.vsense = 0x1; // was 1
        CHOPCONF.vhighfs = 0x0;
        CHOPCONF.vhighchm = 0x0;
        CHOPCONF.SYNC = 5;
        CHOPCONF.MRES = 0x3;
        CHOPCONF.intpol = 0;
        CHOPCONF.dedge = 0x0;
        CHOPCONF.diss2g = 0x0;
        //        CHOPCONF.TOFF = 8;
        //        CHOPCONF.HSTRT_TFD012 = 1;
        //        CHOPCONF.HEND_OFFSET = 14;
        //        CHOPCONF.TFD3 = 0x0;
        //        CHOPCONF.disfdcc = 0x0;
        //        CHOPCONF.rndtf = 0x0;
        //        CHOPCONF.chm = 0x0;
        //        CHOPCONF.TBL = 1;
        //        CHOPCONF.vsense = true;
        //        CHOPCONF.vhighfs = 0;
        //        CHOPCONF.vhighchm = 0;
        //        CHOPCONF.SYNC = 4;
        //        CHOPCONF.MRES = 3;
        //        CHOPCONF.intpol = false;
        //        CHOPCONF.dedge = false;
        //        CHOPCONF.diss2g = false;
        CHOPCONF_needs_written = true;

        PWMCONF.PWM_AMPL = 200;
        PWMCONF.PWM_GRAD = 5; // 0 - 15
        PWMCONF.pwm_freq = 3; // approx 19MHz with the internal clock
        PWMCONF.pwm_autoscale = 1;
        PWMCONF.pwm_symmetric = 0;
        PWMCONF.freewheel = 0;
        //        PWMCONF.PWM_AMPL = 128;
        //        PWMCONF.PWM_GRAD = 4;
        //        PWMCONF.pwm_freq = 1;
        //        PWMCONF.pwm_autoscale = 1;
        //        PWMCONF.pwm_symmetric = 0;
        //        PWMCONF.freewheel = 0;
        PWMCONF_needs_written = true;

        COOLCONF.semin =  1;  // enable coolstep and set min sg (1-15)
        COOLCONF.semax = 15; // set coolstep max sg(0-15)
        COOLCONF.seup =   3; // set coolstep up rate (0-3)
        COOLCONF.sedn =   3; // set coolstep down rate (0-3)
//        COOLCONF.sgt  =  64+(64-60); // set stallGuard threshold (-64 to 63) (-60)
        COOLCONF.sgt  =   0; // set stallGuard threshold (-64 to 63) (63)
        COOLCONF.seimin=  1; // minimum current setting (0 for 1/2 IRUN, or 1 for 1/4 IRUN)
        COOLCONF.sfilt=   1; // enable stallGuard filtering (0 or 1)
        COOLCONF_needs_written = true;

        IOIN_needs_read = true;
        MSCNT_needs_read = true;

        _inited = true;
        _startNextReadWrite();
        check_timer.set(100);

        Stepper::init();
    };

    void periodicCheck(bool have_actually_stopped) override
    {
        Stepper::periodicCheck(have_actually_stopped);
        if (check_timer.isPast()) {
            check_timer.set(100);
            IOIN_needs_read = true;
            CHOPCONF_needs_read = true;
            DRV_STATUS_needs_read = true;
            TSTEP_needs_read = true;
        }
        _startNextReadWrite();
    };

    // helper to create functions that retrieve the object from the cfgArray[...].target
    // and call the correct function of that target
    template <stat_t(type::*T)(nvObj_t *nv)>
    static stat_t get_fn(nvObj_t *nv) {
        return (reinterpret_cast<type*>(cfgArray[nv->index].target)->*T)(nv);
    };

    // NV interface helpers
    stat_t get_ts(nvObj_t *nv) {
        nv->value_int = TSTEP.value;
        nv->valuetype = TYPE_INTEGER;
        return STAT_OK;
    };
    static stat_t get_ts_fn(nvObj_t *nv) { return get_fn<&type::get_ts>(nv); };
    // no set

    stat_t get_pth(nvObj_t *nv) {
        nv->value_int = TPWMTHRS.value;
        nv->valuetype = TYPE_INTEGER;
        return STAT_OK;
    };
    static stat_t get_pth_fn(nvObj_t *nv) { return get_fn<&type::get_pth>(nv); };
    stat_t set_pth(nvObj_t *nv) {
        int32_t v = nv->value_int;
        if (v < 0) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_LESS_THAN_MIN_VALUE);
        }
        if (v > 1048575) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_EXCEEDS_MAX_VALUE);
        }
        TPWMTHRS.value = v;
        TPWMTHRS_needs_written = true;
        return STAT_OK;
    };
    static stat_t set_pth_fn(nvObj_t *nv) { return get_fn<&type::set_pth>(nv); };

    stat_t get_cth(nvObj_t *nv) {
        nv->value_int = TCOOLTHRS.value;
        nv->valuetype = TYPE_INTEGER;
        return STAT_OK;
    };
    static stat_t get_cth_fn(nvObj_t *nv) { return get_fn<&type::get_cth>(nv); };
    stat_t set_cth(nvObj_t *nv) {
        int32_t v = nv->value_int;
        if (v < 0) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_LESS_THAN_MIN_VALUE);
        }
        if (v > 1048575) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_EXCEEDS_MAX_VALUE);
        }
        TCOOLTHRS.value = v;
        TCOOLTHRS_needs_written = true;
        return STAT_OK;
    };
    static stat_t set_cth_fn(nvObj_t *nv) { return get_fn<&type::set_cth>(nv); };

    stat_t get_hth(nvObj_t *nv) {
        nv->value_int = THIGH.value;
        nv->valuetype = TYPE_INTEGER;
        return STAT_OK;
    };
    static stat_t get_hth_fn(nvObj_t *nv) { return get_fn<&type::get_hth>(nv); };
    stat_t set_hth(nvObj_t *nv) {
        int32_t v = nv->value_int;
        if (v < 0) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_LESS_THAN_MIN_VALUE);
        }
        if (v > 1048575) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_EXCEEDS_MAX_VALUE);
        }
        THIGH.value = v;
        THIGH_needs_written = true;
        return STAT_OK;
    };
    static stat_t set_hth_fn(nvObj_t *nv) { return get_fn<&type::set_hth>(nv); };

    stat_t get_sgt(nvObj_t *nv) {
        int32_t v = COOLCONF.sgt;
        nv->value_int = (int32_t)(63&v)-(v&64);
        nv->valuetype = TYPE_INTEGER;
        return STAT_OK;
    };
    static stat_t get_sgt_fn(nvObj_t *nv) { return get_fn<&type::get_sgt>(nv); };
    stat_t set_sgt(nvObj_t *nv) {
        int32_t v = nv->value_int;
        if (v < -64) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_LESS_THAN_MIN_VALUE);
        }
        if (v > 63) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_EXCEEDS_MAX_VALUE);
        }
        COOLCONF.sgt = (v<0)?127&(64|((~(-v))+1)):v;
        COOLCONF_needs_written = true;
        return STAT_OK;
    };
    static stat_t set_sgt_fn(nvObj_t *nv) { return get_fn<&type::set_sgt>(nv); };

    stat_t get_csa(nvObj_t *nv) {
        nv->value_int = DRV_STATUS.CS_ACTUAL;
        nv->valuetype = TYPE_INTEGER;
        return STAT_OK;
    };
    static stat_t get_csa_fn(nvObj_t *nv) { return get_fn<&type::get_csa>(nv); };
    // no set

    stat_t get_sgr(nvObj_t *nv) {
        nv->value_int = DRV_STATUS.SG_RESULT;
        nv->valuetype = TYPE_INTEGER;
        return STAT_OK;
    };
    static stat_t get_sgr_fn(nvObj_t *nv) { return get_fn<&type::get_sgr>(nv); };
    // no set

    stat_t get_sgs(nvObj_t *nv) {
        nv->value_int = DRV_STATUS.stallGuard;
        nv->valuetype = TYPE_BOOLEAN;
        return STAT_OK;
    };
    static stat_t get_sgs_fn(nvObj_t *nv) { return get_fn<&type::get_sgs>(nv); };
    // no set


    stat_t get_tbl(nvObj_t *nv) {
        nv->value_int = CHOPCONF.TBL;
        nv->valuetype = TYPE_INTEGER;
        return STAT_OK;
    };
    static stat_t get_tbl_fn(nvObj_t *nv) { return get_fn<&type::get_tbl>(nv); };
    stat_t set_tbl(nvObj_t *nv) {
        int32_t v = nv->value_int;
        if (v < 0) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_LESS_THAN_MIN_VALUE);
        }
        if (v > 3) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_EXCEEDS_MAX_VALUE);
        }
        CHOPCONF.TBL = v;
        CHOPCONF_needs_written = true;
        return STAT_OK;
    };
    static stat_t set_tbl_fn(nvObj_t *nv) { return get_fn<&type::set_tbl>(nv); };

    stat_t get_pgrd(nvObj_t *nv) {
        nv->value_int = PWMCONF.PWM_GRAD;
        nv->valuetype = TYPE_INTEGER;
        return STAT_OK;
    };
    static stat_t get_pgrd_fn(nvObj_t *nv) { return get_fn<&type::get_pgrd>(nv); };
    stat_t set_pgrd(nvObj_t *nv) {
        int32_t v = nv->value_int;
        if (v < 0) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_LESS_THAN_MIN_VALUE);
        }
        if (v > 15) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_EXCEEDS_MAX_VALUE);
        }
        PWMCONF.PWM_GRAD = v;
        PWMCONF_needs_written = true;
        return STAT_OK;
    };
    static stat_t set_pgrd_fn(nvObj_t *nv) { return get_fn<&type::set_pgrd>(nv); };

    stat_t get_pamp(nvObj_t *nv) {
        nv->value_int = PWMCONF.PWM_AMPL;
        nv->valuetype = TYPE_INTEGER;
        return STAT_OK;
    };
    static stat_t get_pamp_fn(nvObj_t *nv) { return get_fn<&type::get_pamp>(nv); };
    stat_t set_pamp(nvObj_t *nv) {
        int32_t v = nv->value_int;
        if (v < 0) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_LESS_THAN_MIN_VALUE);
        }
        if (v > 255) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_EXCEEDS_MAX_VALUE);
        }
        PWMCONF.PWM_AMPL = v;
        PWMCONF_needs_written = true;
        return STAT_OK;
    };
    static stat_t set_pamp_fn(nvObj_t *nv) { return get_fn<&type::set_pamp>(nv); };

    stat_t get_hend(nvObj_t *nv) {
        nv->value_int = CHOPCONF.HEND_OFFSET;
        nv->valuetype = TYPE_INTEGER;
        return STAT_OK;
    };
    static stat_t get_hend_fn(nvObj_t *nv) { return get_fn<&type::get_hend>(nv); };
    stat_t set_hend(nvObj_t *nv) {
        int32_t v = nv->value_int;
        if (v < 0) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_LESS_THAN_MIN_VALUE);
        }
        if (v > 15) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_EXCEEDS_MAX_VALUE);
        }
        CHOPCONF.HEND_OFFSET = v;
        CHOPCONF_needs_written = true;
        return STAT_OK;
    };
    static stat_t set_hend_fn(nvObj_t *nv) { return get_fn<&type::set_hend>(nv); };

    stat_t get_hsrt(nvObj_t *nv) {
        nv->value_int = CHOPCONF.HSTRT_TFD012;
        nv->valuetype = TYPE_INTEGER;
        return STAT_OK;
    };
    static stat_t get_hsrt_fn(nvObj_t *nv) { return get_fn<&type::get_hsrt>(nv); };
    stat_t set_hsrt(nvObj_t *nv) {
        int32_t v = nv->value_int;
        if (v < 0) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_LESS_THAN_MIN_VALUE);
        }
        if (v > 15) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_EXCEEDS_MAX_VALUE);
        }
        CHOPCONF.HSTRT_TFD012 = v;
        CHOPCONF_needs_written = true;
        return STAT_OK;
    };
    static stat_t set_hsrt_fn(nvObj_t *nv) { return get_fn<&type::set_hsrt>(nv); };

    stat_t get_smin(nvObj_t *nv) {
        nv->value_int = COOLCONF.semin;
        nv->valuetype = TYPE_INTEGER;
        return STAT_OK;
    };
    static stat_t get_smin_fn(nvObj_t *nv) { return get_fn<&type::get_smin>(nv); };
    stat_t set_smin(nvObj_t *nv) {
        int32_t v = nv->value_int;
        if (v < 0) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_LESS_THAN_MIN_VALUE);
        }
        if (v > 15) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_EXCEEDS_MAX_VALUE);
        }
        COOLCONF.semin = v;
        COOLCONF_needs_written = true;
        return STAT_OK;
    };
    static stat_t set_smin_fn(nvObj_t *nv) { return get_fn<&type::set_smin>(nv); };

    stat_t get_smax(nvObj_t *nv) {
        nv->value_int = COOLCONF.semax;
        nv->valuetype = TYPE_INTEGER;
        return STAT_OK;
    };
    static stat_t get_smax_fn(nvObj_t *nv) { return get_fn<&type::get_smax>(nv); };
    stat_t set_smax(nvObj_t *nv) {
        int32_t v = nv->value_int;
        if (v < 0) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_LESS_THAN_MIN_VALUE);
        }
        if (v > 15) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_EXCEEDS_MAX_VALUE);
        }
        COOLCONF.semax = v;
        COOLCONF_needs_written = true;
        return STAT_OK;
    };
    static stat_t set_smax_fn(nvObj_t *nv) { return get_fn<&type::set_smax>(nv); };

    stat_t get_sup(nvObj_t *nv) {
        nv->value_int = COOLCONF.seup;
        nv->valuetype = TYPE_INTEGER;
        return STAT_OK;
    };
    static stat_t get_sup_fn(nvObj_t *nv) { return get_fn<&type::get_sup>(nv); };
    stat_t set_sup(nvObj_t *nv) {
        int32_t v = nv->value_int;
        if (v < 0) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_LESS_THAN_MIN_VALUE);
        }
        if (v > 15) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_EXCEEDS_MAX_VALUE);
        }
        COOLCONF.seup = v;
        COOLCONF_needs_written = true;
        return STAT_OK;
    };
    static stat_t set_sup_fn(nvObj_t *nv) { return get_fn<&type::set_sup>(nv); };

    stat_t get_sdn(nvObj_t *nv) {
        nv->value_int = COOLCONF.sedn;
        nv->valuetype = TYPE_INTEGER;
        return STAT_OK;
    };
    static stat_t get_sdn_fn(nvObj_t *nv) { return get_fn<&type::get_sdn>(nv); };
    stat_t set_sdn(nvObj_t *nv) {
        int32_t v = nv->value_int;
        if (v < 0) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_LESS_THAN_MIN_VALUE);
        }
        if (v > 15) {
            nv->valuetype = TYPE_NULL;
            return (STAT_INPUT_EXCEEDS_MAX_VALUE);
        }
        COOLCONF.sedn = v;
        COOLCONF_needs_written = true;
        return STAT_OK;
    };
    static stat_t set_sdn_fn(nvObj_t *nv) { return get_fn<&type::set_sdn>(nv); };

};
