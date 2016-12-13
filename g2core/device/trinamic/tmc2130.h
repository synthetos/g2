/*
 * trinamic/tmc2130.h - control over a Trinamic TMC2130 stepper motor driver
 * This file is part of the G2 project
 *
 * Copyright (c) 2016 Alden S. Hart, Jr.
 * Copyright (c) 2016 Robert Giseburt
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
            uint8_t addr;
            uint8_t status;
        };
        volatile uint32_t value;
    } __attribute__ ((packed));

    // And make two statically allocated buffers
    volatile trinamic_buffer_t out_buffer;
    volatile trinamic_buffer_t in_buffer;

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
                                  4000000, //1MHz
                                  SPIDeviceMode::kSPIMode2 | SPIDeviceMode::kSPI8Bit,
                                  0, // min_between_cs_delay_ns
                                  10, // cs_to_sck_delay_ns
                                  0  // between_word_delay_ns
                                  )}
    {};

    // Prevent copying, and prevent moving (so we know if it happens)
    Trinamic2130(const Trinamic2130 &) = delete;
    Trinamic2130(Trinamic2130 &&other) : _device{std::move(other._device)} {};


    // ############
    // Stepper override functions

    // We can leave this as is:
    // bool canStep() override { return true; };

    void setMicrosteps(const uint8_t microsteps) override
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
            default: return;
        }
        CHOPCONF_needs_written = true;
        _startNextReadWrite();
    };

    void _enableImpl() override { _enable.clear(); };

    void _disableImpl() override { _enable.set(); };

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

    void setPowerLevel(float new_pl) override
    {
        // scale the 0.0-1.0 to 0-31
        IHOLD_IRUN.IRUN = (new_pl * 31.0);

        // for now, we'll have the holding be the same
        IHOLD_IRUN.IHOLD = (new_pl * 31.0);

        IHOLD_IRUN_needs_written = true;
        _startNextReadWrite();
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

    struct {
        volatile uint32_t value;
        //        uint8_t bytes[4];
    } COOLCONF; // 0x6D - READ ONLY
    void _postReadCoolConf() {
        COOLCONF.value = fromBigEndian(in_buffer.value);
    };
    void _prepWriteCoolConf() {
        out_buffer.value = toBigEndian(COOLCONF.value);
    };
    volatile bool COOLCONF_needs_read;
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
        if (COOLCONF_needs_read)      { next_reg = COOLCONF_reg;   COOLCONF_needs_read = false;   } else
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
        //        for (uint16_t i = 10; i>0; i--) { __NOP(); }
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
                case COOLCONF_reg:   _postReadCoolConf(); break;
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
        //_startNextReadWrite();
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

        XDIRECT.value = 0;
        XDIRECT_needs_written = true;

        VDCMIN.value = 0;
        VDCMIN_needs_written = true;

        GCONF.en_pwm_mode = 1;
        GCONF_needs_written = true;

        CHOPCONF.TOFF = 0x5;
        CHOPCONF.HSTRT_TFD012 = 0x4;
        CHOPCONF.HEND_OFFSET = 0x1;
        CHOPCONF.TFD3 = 0x0;
        CHOPCONF.disfdcc = 0x0;
        CHOPCONF.rndtf = 0x0;
        CHOPCONF.chm = 0x0;
        CHOPCONF.TBL = 0x2;
        CHOPCONF.vsense = 0x1;
        CHOPCONF.vhighfs = 0x0;
        CHOPCONF.vhighchm = 0x0;
        CHOPCONF.SYNC = 0x0;
        CHOPCONF.MRES = 0x3;
        CHOPCONF.intpol = 0x0;
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
        PWMCONF.PWM_GRAD = 1;
        PWMCONF.pwm_freq = 0;
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
            _startNextReadWrite();
        }
    };
};
