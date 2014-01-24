/*
 utility/SamSPI.h - Library for the Motate system
 http://tinkerin.gs/
 
 Copyright (c) 2013 Robert Giseburt
 
 This file is part of the Motate Library.
 
 This file ("the software") is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License, version 2 as published by the
 Free Software Foundation. You should have received a copy of the GNU General Public
 License, version 2 along with the software. If not, see <http://www.gnu.org/licenses/>.
 
 As a special exception, you may use this file as part of a software library without
 restriction. Specifically, if other files instantiate templates or use macros or
 inline functions from this file, or you compile this file and link it with  other
 files to produce an executable, this file does not by itself cause the resulting
 executable to be covered by the GNU General Public License. This exception does not
 however invalidate any other reasons why the executable file might be covered by the
 GNU General Public License.
 
 THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 
 */

#ifndef SAMSPI_H_ONCE
#define SAMSPI_H_ONCE

#include "sam.h"

#include "MotatePins.h"
#include "SamCommon.h"

namespace Motate {
    
    // WHOA!! We only support master mode ... for now.
    
    
    enum SPIMode {
        
        kSPIPolarityNormal     = 0,
        kSPIPolarityReversed   = SPI_CSR_CPOL,
        
        // Using the wikipedia deifinition of "normal phase," see:
        //   http://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus#Clock_polarity_and_phase
        // Wikipedia, in turn, sites Freescale's SPI Block Guide:
        //   http://www.ee.nmt.edu/~teare/ee308l/datasheets/S12SPIV3.pdf
        
        // This makes the Phase flag INVERTED from that of the SAM3X/A datasheet.
        
        kSPIClockPhaseNormal   = SPI_CSR_NCPHA,
        kSPIClockPhaseReversed = 0,
        
        // Using the wikipedia/freescale mode numbers (and the SAM3X/A datashgeet agrees).
        // The arduino mode settings appear to mirror that of wikipedia as well,
        //  so we should all be in agreement here.
        kSPIMode0              = kSPIPolarityNormal   | kSPIClockPhaseNormal,
        kSPIMode1              = kSPIPolarityNormal   | kSPIClockPhaseReversed,
        kSPIMode2              = kSPIPolarityReversed | kSPIClockPhaseNormal,
        kSPIMode3              = kSPIPolarityReversed | kSPIClockPhaseReversed,
        
        kSPI8Bit               = SPI_CSR_BITS_8_BIT,
        kSPI9Bit               = SPI_CSR_BITS_9_BIT,
        kSPI10Bit              = SPI_CSR_BITS_10_BIT,
        kSPI11Bit              = SPI_CSR_BITS_11_BIT,
        kSPI12Bit              = SPI_CSR_BITS_12_BIT,
        kSPI13Bit              = SPI_CSR_BITS_13_BIT,
        kSPI14Bit              = SPI_CSR_BITS_14_BIT,
        kSPI15Bit              = SPI_CSR_BITS_15_BIT,
        kSPI16Bit              = SPI_CSR_BITS_16_BIT
	};
    
    // This is an internal representation of the peripheral.
    // This is *not* to be used externally.
	template<uint8_t spiPeripheralNum, int8_t spiMISOPinNumber, int8_t spiMOSIPinNumber, int8_t spSCKSPinNumber>
    struct _SPIHardware {
        // BITBANG HERE!
    };
    
    template<>
    struct _SPIHardware<0u, kSPI_MISOPinNumber, kSPI_MOSIPinNumber, kSPI_SCKPinNumber> : SamCommon< _SPIHardware<0u, kSPI_MISOPinNumber, kSPI_MOSIPinNumber, kSPI_SCKPinNumber> > {
		static Spi * const spi() { return SPI0; };
        static const uint32_t peripheralId() { return ID_SPI0; }; // ID_SPI0 .. ID_SPI1
		static const IRQn_Type spiIRQ() { return SPI0_IRQn; };
        
        static const uint8_t spiPeripheralNum=0;
        
        typedef _SPIHardware<0u, kSPI_MISOPinNumber, kSPI_MOSIPinNumber, kSPI_SCKPinNumber> this_type_t;
        typedef SamCommon< this_type_t > common;

        /* We have to play some tricks here, because templates and static members are tricky.
         * See https://groups.google.com/forum/#!topic/comp.lang.c++.moderated/yun9X6OMiY4
         *
         * Basically, we want a guard to make sure we dont itinig the SPI0 IC modules every time
         * we create a new SPI object for the individual chip selects.
         *
         * However, since we don't use the module *directly* in the code, other than to init it,
         * the optimizer removes that object and it's init in it's entrety.
         *
         * The solution: Make sure each SPI<> object calls hardware.init(), and then use a static guard
         * in init() to prevent re-running it.
         */
        
        void init() {
            static bool inited = false;
            if (inited)
                return;
            inited = true;
        
            common::enablePeripheralClock();
            disable();
            
            /* Execute a software reset of the SPI twice */
            /* Why? Because ATMEL said!  -Rob*/
            spi()->SPI_CR = SPI_CR_SWRST;
            spi()->SPI_CR = SPI_CR_SWRST;
            
            // Set Mode Register to Master mode + Mode Fault Detection Disabled
            spi()->SPI_MR = SPI_MR_MSTR | SPI_MR_MODFDIS;
        };
        
        _SPIHardware() :  SamCommon< this_type_t >() {
//            init();
        };

        static void enable() {
            spi()->SPI_CR = SPI_CR_SPIEN ;
        };
        
        static void disable () {
            spi()->SPI_CR = SPI_CR_SPIDIS;
        };
        
        bool setChannel(const uint8_t channel) {
            // if we are transmitting, we cannot switch
            while (!(spi()->SPI_SR & SPI_SR_TXEMPTY)) {
                ;
            }
            
            uint8_t channel_setting;
            
            if (!(spi()->SPI_MR & SPI_MR_PCSDEC)) {
                channel_setting = ~(1 << channel);
            } else {
                channel_setting = ~channel; // <- Is this right? -Rob
            }
            
            spi()->SPI_MR = (spi()->SPI_MR & ~SPI_MR_PCS_Msk) | SPI_MR_PCS(channel_setting);
            
            return true;
        }
        
        static int16_t read(const bool lastXfer = false, uint8_t toSendAsNoop = 0) {
            if (!(spi()->SPI_SR & SPI_SR_RDRF)) {
                if (spi()->SPI_SR & SPI_SR_TXEMPTY) {
                    spi()->SPI_TDR = toSendAsNoop;
                    if (lastXfer) {
                        spi()->SPI_CR = SPI_CR_LASTXFER;
                    }
                }
                return -1;
            }
            
            return spi()->SPI_RDR;
        }
        
        static int16_t write(uint8_t value, const bool lastXfer = false) {
            int16_t throw_away;
            // Let's see what the optimizer does with this...
            return write(value, throw_away, lastXfer);
        };
        
        static int16_t write(uint8_t value, int16_t &readValue, const bool lastXfer = false) {
            if (spi()->SPI_SR & SPI_SR_RDRF) {
                readValue = spi()->SPI_RDR;
            } else {
                readValue = -1;
            }
            
            if (spi()->SPI_SR & SPI_SR_TDRE) {

                spi()->SPI_TDR = value;

                if (lastXfer) {
                    spi()->SPI_CR = SPI_CR_LASTXFER;
                }

                return 1;
            }
            
            return -1;
        };
        
        int16_t transmit(uint8_t channel, const uint16_t data, const bool lastXfer = false) {
            uint32_t data_with_flags = data;
            
            if (lastXfer)
                data_with_flags |= SPI_TDR_LASTXFER;
            
            // NOTE: Assumes we DON'T have an external decoder/multiplexer!
            data_with_flags |= SPI_TDR_PCS(~(1<< channel));
            
            while (!(spi()->SPI_SR & SPI_SR_TDRE))
                ;
            spi()->SPI_TDR = data_with_flags;
            
            while (!(spi()->SPI_SR & SPI_SR_RDRF))
                ;
            
            uint16_t outdata = spi()->SPI_RDR;
            return outdata;
        };
    };
    
	template<int8_t spiCSPinNumber, int8_t spiMISOPinNumber=kSPI_MISOPinNumber, int8_t spiMOSIPinNumber=kSPI_MOSIPinNumber, int8_t spiSCKSPinNumber=kSPI_SCKPinNumber>
	struct SPI {
        typedef SPIChipSelectPin<spiCSPinNumber> csPinType;
        csPinType csPin;
        
        SPIOtherPin<spiMISOPinNumber> misoPin;
        SPIOtherPin<spiMOSIPinNumber> mosiPin;
        SPIOtherPin<spiSCKSPinNumber> sckPin;
        
        static _SPIHardware< csPinType::moduleId, spiMISOPinNumber, spiMOSIPinNumber, spiSCKSPinNumber > hardware;
        static const uint8_t spiPeripheralNum() { return csPinType::moduleId; };
        static const uint8_t spiChannelNumber() { return SPIChipSelectPin<spiCSPinNumber>::csOffset; };
        
        static Spi * const spi() { return hardware.spi(); };
        static const uint32_t peripheralId() { return hardware.peripheralId(); };
        static const IRQn_Type spiIRQ() { return hardware.spiIRQ(); };
        
        typedef SamCommon< SPI<spiCSPinNumber> > common;
        
        SPI(const uint32_t baud = 4000000, const uint16_t options = kSPI8Bit | kSPIMode0) {
            hardware.init();
            init(baud, options, /*fromConstructor =*/ true);

            /* TEMP HACK !! */
//            csPin.setMode(kPeripheralA);
//            misoPin.setMode(kPeripheralA);
//            mosiPin.setMode(kPeripheralA);
//            sckPin.setMode(kPeripheralA);
        };
        
        void init(const uint32_t baud, const uint16_t options, const bool fromConstructor=false) {
            setOptions(baud, options, fromConstructor);
        };
        
        void setOptions(const uint32_t baud, const uint16_t options, const bool fromConstructor=false) {
            // We derive the baud from the master clock with a divider.
            // We want the closest match *below* the value asked for. It's safer to bee too slow.
            
            uint8_t divider = SystemCoreClock / baud;
            if (divider > 255)
                divider = 255;
            else if (divider < 1)
                divider = 1;
            
            // Cruft from Arduino: TODO: Make configurable.
            // SPI_CSR_DLYBCT(1) keeps CS enabled for 32 MCLK after a completed
            // transfer. Some device needs that for working properly.
            spi()->SPI_CSR[spiChannelNumber()] = (options & (SPI_CSR_NCPHA | SPI_CSR_CPOL | SPI_CSR_BITS_Msk)) | SPI_CSR_SCBR(divider) | SPI_CSR_DLYBCT(1) | SPI_CSR_CSAAT;
            
            // Should be a non-op for already-enabled devices.
            hardware.enable();

        };
        
        bool setChannel() {
            return hardware.setChannel(spiChannelNumber());
        };
        
        uint16_t getOptions() {
            return spi()->SPI_CSR[spiChannelNumber()]/* & (SPI_CSR_NCPHA | SPI_CSR_CPOL | SPI_CSR_BITS_Msk)*/;
        };
        
		int16_t read(const bool lastXfer = false, uint8_t toSendAsNoop = 0) {
            return hardware.read(lastXfer, toSendAsNoop);
		};
        
        // WARNING: Currently only reads in bytes. For more-that-byte size data, we'll need another call.
		int16_t read(const uint8_t *buffer, const uint16_t length) {
			if (!setChannel())
                return -1;
            

			int16_t total_read = 0;
			int16_t to_read = length;
			const uint8_t *read_ptr = buffer;

            bool lastXfer = false;

			// BLOCKING!!
			while (to_read > 0) {
                
                if (to_read == 1)
                    lastXfer = true;

				int16_t ret = read(lastXfer);
                
                if (ret >= 0) {
                    *read_ptr++ = ret;
                    total_read++;
                    to_read--;
                }
			};
            
			return total_read;
		};
        
        int16_t write(uint16_t data, const bool lastXfer = false) {
            return hardware.write(data, lastXfer);
		};
        
        int16_t write(uint8_t data, int16_t &readValue, const bool lastXfer = false) {
            return hardware.write(data, lastXfer);
		};

        void flush() {
            hardware.disable();
            hardware.enable();
        };
        
        // WARNING: Currently only writes in bytes. For more-that-byte size data, we'll need another call.
		int16_t write(const uint8_t *data, const uint16_t length, bool autoFlush = true) {
			if (!setChannel())
                return -1;
            
            int16_t total_written = 0;
			const uint8_t *out_buffer = data;
			int16_t to_write = length;
            
            bool lastXfer = false;
            
			// BLOCKING!!
			do {
                if (autoFlush && to_write == 1)
                    lastXfer = true;
                
                int16_t ret = write(*out_buffer, lastXfer);
				
                if (ret > 0) {
                    out_buffer++;
                    total_written++;
                    to_write--;
                }
			} while (to_write);
            
//			// HACK! Autoflush forced...
//			if (autoFlush && total_written > 0)
//                flush();
            
			return total_written;
		}
	};
    
}

#endif /* end of include guard: SAMSPI_H_ONCE */