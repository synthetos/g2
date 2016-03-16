/*
  utility/SamUSB.h - Library for the Motate system
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

#if defined(__SAM3X8E__) || defined(__SAM3X8C__)

#include "utility/SamUSB.h"

#define TRACE_CORE(x)

uint16_t endpointSizes[10] = {64, 0, 0, 0, 0, 0, 0, 0, 0, 0};

namespace Motate {
	const uint16_t MOTATE_USBLanguageString[] = {0x0409}; // English
	const uint16_t *getUSBLanguageString(int16_t &length) {
//		length = sizeof(MOTATE_USBLanguageString);
        length = 2;
		return MOTATE_USBLanguageString;
	}

	uint32_t _inited = 0;
	uint32_t _configuration = 0;
	uint32_t _set_interface = 0; // the interface set by the host
	uint8_t  _halted = 0; // Make this into a generic _flags?? -rg
	uint8_t  _remoteWakeupEnabled = 0;

	USBProxy_t USBProxy;

	/* ############################################# */
	/* #                                           # */
	/* #        HW-SPECIFIC ENDPOINT LIMITS        # */
	/* #                                           # */
	/* ############################################# */

	// Enpoint 3 config - max 1024b buffer, with two blocks
	// Enpoint 4 config - max 1024b buffer, with two blocks
	// Enpoint 5 config - max 1024b buffer, with two blocks
	// Enpoint 6 config - max 1024b buffer, with two blocks
	// Enpoint 7 config - max 1024b buffer, with two blocks
	// Enpoint 8 config - max 1024b buffer, with two blocks
	// Enpoint 9 config - max 1024b buffer, with two blocks

	uint16_t checkEndpointSizeHardwareLimits(const uint16_t inSize, const uint8_t endpointNumber, const USBEndpointType_t endpointType, const bool otherSpeed) {
		uint16_t tempSize = inSize;

		if (endpointNumber == 0) {
			if (tempSize > 64)
				tempSize = 64;
		} else if (tempSize > 1024) {
			tempSize = 1024;
		}

		return tempSize;
	};

	/* ############################################# */
	/* #                                           # */
	/* #        INTERNAL FUNCTIONS FOR SAM         # */
	/* #                                           # */
	/* ############################################# */

	static const EndpointBufferSettings_t _enforce_enpoint_limits(const uint8_t endpoint, EndpointBufferSettings_t config) {
		if (endpoint > 9)
			return kEndpointBufferNull;

		if (endpoint == 0) {
			if ((config & kEnpointBufferSizeMask) > kEnpointBufferSizeUpTo64)
				config = (config & ~kEnpointBufferSizeMask) | kEnpointBufferSizeUpTo64;

			config = (config & ~kEndpointBufferBlocksMask) | kEndpointBufferBlocks1;
		} else {
			// Enpoint 1 config - max 1024b buffer, with three blocks
			// Enpoint 2 config - max 1024b buffer, with three blocks

			if ((config & kEnpointBufferSizeMask) > kEnpointBufferSizeUpTo1024)
				config = (config & ~kEnpointBufferSizeMask) | kEnpointBufferSizeUpTo1024;

			if (endpoint < 3) {
				if ((config & kEndpointBufferBlocksMask) > kEndpointBufferBlocksUpTo3)
					config = (config & ~kEndpointBufferBlocksMask) | kEndpointBufferBlocksUpTo3;
			} else {
				if ((config & kEndpointBufferBlocksMask) > kEndpointBufferBlocksUpTo2)
					config = (config & ~kEndpointBufferBlocksMask) | kEndpointBufferBlocksUpTo2;
			}
		}

        if (config & kEndpointTypeInterrupt) {
            if (config & kEndpointBufferBlocks1) {
                config |= UOTGHS_DEVEPTCFG_NBTRANS_1_TRANS;
            } else if (config & kEndpointBufferBlocksUpTo2) {
                config |= UOTGHS_DEVEPTCFG_NBTRANS_2_TRANS;
            } else if (config & kEndpointBufferBlocksUpTo3) {
                config |= UOTGHS_DEVEPTCFG_NBTRANS_3_TRANS;
            }
        }

		config |= UOTGHS_DEVEPTCFG_ALLOC;

		return config;
	};
    

	inline void _setEndpointConfiguration(const uint8_t endpoint, uint32_t configuration) {
		UOTGHS->UOTGHS_DEVEPTCFG[endpoint] = configuration;
	}

	inline bool _isEndpointConfigOK(const uint8_t endpoint)  {
		return (UOTGHS->UOTGHS_DEVEPTISR[endpoint] & UOTGHS_DEVEPTISR_CFGOK) != 0;
	}

	inline void _enableEndpoint(const uint8_t endpoint) {
		UOTGHS->UOTGHS_DEVEPT |= UOTGHS_DEVEPT_EPEN0 << (endpoint);
	}

	inline void _enableOverflowInterrupt(const uint8_t endpoint) { UOTGHS->UOTGHS_DEVEPTIER[endpoint] = UOTGHS_DEVEPTIER_OVERFES; }

	void _initEndpoint(uint32_t endpoint, const uint32_t configuration) {
		endpoint = endpoint & 0xF; // EP range is 0..9, hence mask is 0xF.

//		TRACE_UOTGHS_DEVICE(printf("=> UDD_InitEP : init EP %lu\r\n", ul_ep_nb);)
		uint32_t configuration_fixed = _enforce_enpoint_limits(endpoint, configuration);

		// Configure EP
		// If we get here, and it's a null endpoint, this will disable it.
		_setEndpointConfiguration(endpoint, configuration_fixed);

        // Enable overflow interrupt for OUT (Rx) endpoints
        if (endpoint > 0 && (configuration & UOTGHS_DEVEPTCFG_EPDIR)==0) {
            _enableOverflowInterrupt(endpoint);
        }
		
		// Enable EP
		if (configuration_fixed != kEndpointBufferNull) {
			_enableEndpoint(endpoint);

			if (!_isEndpointConfigOK(endpoint)) {
	//			TRACE_UOTGHS_DEVICE(printf("=> UDD_InitEP : ERROR FAILED TO INIT EP %lu\r\n", ul_ep_nb);)
				while(1) {
                    ;
                }
			}
		} else {
            while(1) {
                ;
            }
        }

		// Is this necessary? -Rob
		_resetEndpointBuffer(endpoint);
	}

	
	volatile uint8_t *_endpointBuffer[10];

	void _resetEndpointBuffer(const uint8_t endpoint) {
		// Wow, here's a brain breaker:
		_endpointBuffer[endpoint] = (((volatile uint8_t (*)[0x8000])UOTGHS_RAM_ADDR)[(endpoint)]);
	}

	// Return the number of bytes in the buffer.
	// For reads, it returns the number of bytes that hasn't been read yet.
	// For writes, it returns the number of bytes has been put into the buffer that hasn't been sent yet.
	// This does *NOT* return the total size of the buffer, nor the amount in the buffer from the beginning.
	// Note that this may not be updated enough to poll.
	// Use _isReadWriteAllowed(endpoint) to tell if an endpoint is ready and has room to be read or written.
	inline int32_t _getEndpointBufferCount(const uint8_t endpoint) {
		return ((UOTGHS->UOTGHS_DEVEPTISR[endpoint] & UOTGHS_DEVEPTISR_BYCT_Msk) >> UOTGHS_DEVEPTISR_BYCT_Pos);
	}
    
	// A few inline helpers, mainly for readability
	//  Set and test interrupts
	bool _inAResetInterrupt() { return (UOTGHS->UOTGHS_DEVISR & UOTGHS_DEVISR_EORST) != 0; }
	void _clearResetInterrupt() { UOTGHS->UOTGHS_DEVICR = UOTGHS_DEVICR_EORSTC; }
	void _enableResetInterrupt() { UOTGHS->UOTGHS_DEVIER = UOTGHS_DEVIER_EORSTES; }
	void _disableResetInterrupt() { UOTGHS->UOTGHS_DEVIDR = UOTGHS_DEVIDR_EORSTEC; }
	bool _isResetInterruptEnabled() { return (UOTGHS->UOTGHS_DEVIMR & UOTGHS_DEVIMR_EORSTE) != 0; }

	inline bool _inAStartOfFrameInterrupt() { return (UOTGHS->UOTGHS_DEVISR & UOTGHS_DEVISR_SOF) != 0; }
	inline void _clearStartOfFrameInterrupt() { UOTGHS->UOTGHS_DEVICR = UOTGHS_DEVICR_SOFC; }
	inline void _enableStartOfFrameInterrupt() { UOTGHS->UOTGHS_DEVIER = UOTGHS_DEVIER_SOFES; }
	inline void _disableStartOfFrameInterrupt() { UOTGHS->UOTGHS_DEVIDR = UOTGHS_DEVIDR_SOFEC; }
	inline bool _isStartOfFrameInterruptEnabled() { return (UOTGHS->UOTGHS_DEVIMR & UOTGHS_DEVIMR_SOFE) != 0; }

	
	// Endpoint interrupts, and subinterrupts (!)
	inline bool _inAnEndpointInterrupt(const uint8_t endpoint) { return (UOTGHS->UOTGHS_DEVISR & UOTGHS_DEVISR_PEP_0 << (endpoint)) != 0; }
	inline bool _inAnEndpointInterruptNotControl() { return (UOTGHS->UOTGHS_DEVISR & (UOTGHS_DEVISR_PEP_1|UOTGHS_DEVISR_PEP_2|UOTGHS_DEVISR_PEP_3|UOTGHS_DEVISR_PEP_4|UOTGHS_DEVISR_PEP_5|UOTGHS_DEVISR_PEP_6|UOTGHS_DEVISR_PEP_7|UOTGHS_DEVISR_PEP_8|UOTGHS_DEVISR_PEP_9)) != 0; }
    inline const uint8_t _firstEndpointOfInterrupt() {
        if (UOTGHS->UOTGHS_DEVISR & UOTGHS_DEVISR_PEP_1) { return 1; }
        if (UOTGHS->UOTGHS_DEVISR & UOTGHS_DEVISR_PEP_2) { return 2; }
        if (UOTGHS->UOTGHS_DEVISR & UOTGHS_DEVISR_PEP_3) { return 3; }
        if (UOTGHS->UOTGHS_DEVISR & UOTGHS_DEVISR_PEP_4) { return 4; }
        if (UOTGHS->UOTGHS_DEVISR & UOTGHS_DEVISR_PEP_5) { return 5; }
        if (UOTGHS->UOTGHS_DEVISR & UOTGHS_DEVISR_PEP_6) { return 6; }
        if (UOTGHS->UOTGHS_DEVISR & UOTGHS_DEVISR_PEP_7) { return 7; }
        if (UOTGHS->UOTGHS_DEVISR & UOTGHS_DEVISR_PEP_8) { return 8; }
        if (UOTGHS->UOTGHS_DEVISR & UOTGHS_DEVISR_PEP_9) { return 9; }
        return 0;
    }

	inline bool _inAnOverflowInterrupt(const uint8_t endpoint) { return (UOTGHS->UOTGHS_DEVEPTISR[endpoint] & UOTGHS_DEVEPTISR_OVERFI) != 0; }

    inline bool _isAControlEnpointInterrupted(const uint8_t endpoint, bool &keepGoing) {
        if (UOTGHS->UOTGHS_DEVISR & UOTGHS_DEVISR_PEP_0 << endpoint) {
            keepGoing = false;
            if (UOTGHS->UOTGHS_DEVEPTCFG[endpoint] & kEndpointTypeControl) {
                return true;
            }
            else {
                return false;
            }
        }
        keepGoing = true;
        return false;
    }
    
    // Pass in a uint8_t to have the enpoint number set in.
    inline bool _inAControlEndpointInterrupt(uint8_t &endpoint_found) {
        bool keepGoing = true;
        
        // The optimizer should clean this up quite a bit...
        if (             _isAControlEnpointInterrupted(0, keepGoing)) { endpoint_found = 0; return true; }
        if (keepGoing && _isAControlEnpointInterrupted(1, keepGoing)) { endpoint_found = 1; return true; }
        if (keepGoing && _isAControlEnpointInterrupted(2, keepGoing)) { endpoint_found = 2; return true; }
        if (keepGoing && _isAControlEnpointInterrupted(3, keepGoing)) { endpoint_found = 3; return true; }
        if (keepGoing && _isAControlEnpointInterrupted(4, keepGoing)) { endpoint_found = 4; return true; }
        if (keepGoing && _isAControlEnpointInterrupted(5, keepGoing)) { endpoint_found = 5; return true; }
        if (keepGoing && _isAControlEnpointInterrupted(6, keepGoing)) { endpoint_found = 6; return true; }
        if (keepGoing && _isAControlEnpointInterrupted(7, keepGoing)) { endpoint_found = 7; return true; }
        if (keepGoing && _isAControlEnpointInterrupted(8, keepGoing)) { endpoint_found = 8; return true; }
        if (keepGoing && _isAControlEnpointInterrupted(9, keepGoing)) { endpoint_found = 9; return true; }
        
        return false;
    }

    
    // Cleared automatically after the ISR fires
	inline void _enableEndpointInterrupt(const uint8_t endpoint) { UOTGHS->UOTGHS_DEVIER = UOTGHS_DEVIER_PEP_0 << (endpoint); }
	inline void _disableEndpointInterrupt(const uint8_t endpoint) { UOTGHS->UOTGHS_DEVIDR = UOTGHS_DEVIDR_PEP_0 << (endpoint); }
	inline bool _isEndpointInterruptEnabled(const uint8_t endpoint) { return (UOTGHS->UOTGHS_DEVIMR & UOTGHS_DEVIMR_PEP_0 << (endpoint)) != 0; }


	inline bool _inAReceivedSetupInterrupt(const uint8_t endpoint) { return (UOTGHS->UOTGHS_DEVEPTISR[endpoint] & UOTGHS_DEVEPTISR_RXSTPI) != 0; }
	inline void _clearReceivedSetupInterrupt(const uint8_t endpoint) { UOTGHS->UOTGHS_DEVEPTICR[endpoint] = UOTGHS_DEVEPTICR_RXSTPIC; }
	inline void _enableReceivedSetupInterrupt(const uint8_t endpoint) { UOTGHS->UOTGHS_DEVEPTIER[endpoint] = UOTGHS_DEVEPTIER_RXSTPES; }
	inline void _disableReceivedSetupInterrupt(const uint8_t endpoint) { UOTGHS->UOTGHS_DEVEPTIDR[endpoint] = UOTGHS_DEVEPTIDR_RXSTPEC; }
	inline bool _isReceivedSetupInterruptEnabled(const uint8_t endpoint) { return (UOTGHS->UOTGHS_DEVEPTIMR[endpoint] & UOTGHS_DEVEPTIMR_RXSTPE) != 0; }

	
	//  Set the address
	inline void _setUSBAddress(uint8_t address) {
		UOTGHS->UOTGHS_DEVCTRL = (UOTGHS->UOTGHS_DEVCTRL & ~UOTGHS_DEVCTRL_UADD_Msk) | UOTGHS_DEVCTRL_UADD(address);
		UOTGHS->UOTGHS_DEVCTRL |= UOTGHS_DEVCTRL_ADDEN;
	}

	// Request a STALL handshake
	inline void _requestStall(const uint8_t endpoint) {
		_enableEndpoint(endpoint); // Why!? -Rob
		
		UOTGHS->UOTGHS_DEVEPTIER[endpoint] = UOTGHS_DEVEPTIER_STALLRQS;
	}

	// Freeze/Unfreeze the USB clock
	void _freezeUSBClock() {
		// Freeze the USB clock...
		UOTGHS->UOTGHS_CTRL |= UOTGHS_CTRL_FRZCLK;
	}

	void _unfreezeUSBClock() {
		// Freeze the USB clock...
		UOTGHS->UOTGHS_CTRL &= ~UOTGHS_CTRL_FRZCLK;
	}

	void _waitForUsableUSBClock() {
		while (!UOTGHS->UOTGHS_SR & UOTGHS_SR_CLKUSABLE)
			;
	}


	// Tests / Clears

	inline bool _isFIFOControlAvailable(const uint8_t endpoint) {
		return (UOTGHS->UOTGHS_DEVEPTIMR[endpoint] & UOTGHS_DEVEPTIMR_FIFOCON) != 0;
	}

	inline bool _isTransmitINAvailable(const uint8_t endpoint) {
		return (UOTGHS->UOTGHS_DEVEPTISR[endpoint] & UOTGHS_DEVEPTISR_TXINI) != 0;
	}

	inline bool _isReceiveOUTAvailable(const uint8_t endpoint) {
		return (UOTGHS->UOTGHS_DEVEPTISR[endpoint] & UOTGHS_DEVEPTISR_RXOUTI) != 0;
	}

	inline bool _isReadWriteAllowed(const uint8_t endpoint) {
		return (UOTGHS->UOTGHS_DEVEPTISR[endpoint] & UOTGHS_DEVEPTISR_RWALL) != 0;
	}

	inline void _waitForTransmitINAvailable(const uint8_t endpoint, bool reset_needed = false) {
		while (!(UOTGHS->UOTGHS_DEVEPTISR[endpoint] & UOTGHS_DEVEPTISR_TXINI))
			reset_needed = true;

        if (reset_needed)
            _resetEndpointBuffer(endpoint);
	}

    inline void _waitForReceiveOUTAvailable(const uint8_t endpoint, bool reset_needed = false) {
		while (!(UOTGHS->UOTGHS_DEVEPTISR[endpoint] & UOTGHS_DEVEPTISR_RXOUTI))
			reset_needed = true;
        
        if (reset_needed)
            _resetEndpointBuffer(endpoint);
	}

	inline void _clearTransmitIN(const uint8_t endpoint) {
		UOTGHS->UOTGHS_DEVEPTICR[endpoint] = UOTGHS_DEVEPTICR_TXINIC;
	}

	inline void _clearReceiveOUT(const uint8_t endpoint) {
		UOTGHS->UOTGHS_DEVEPTICR[endpoint] = UOTGHS_DEVEPTICR_RXOUTIC;
	}

	inline void _clearFIFOControl(const uint8_t endpoint) {
		UOTGHS->UOTGHS_DEVEPTIDR[endpoint] = UOTGHS_DEVEPTIDR_FIFOCONC;
	}

	// Acknowledges
	inline void _ackReset() { UOTGHS->UOTGHS_DEVICR = UOTGHS_DEVICR_EORSTC; }
	inline void _ackStartOfFrame() { UOTGHS->UOTGHS_DEVICR = UOTGHS_DEVICR_SOFC; }
//	inline void _ackOutRecieved(uint8_t endpoint) { UOTGHS->UOTGHS_DEVEPTICR[endpoint] = UOTGHS_DEVEPTICR_RXOUTIC; }
//	inline void _ackFIFOControl(uint8_t endpoint) { UOTGHS->UOTGHS_DEVEPTIDR[endpoint] = UOTGHS_DEVEPTIDR_FIFOCONC; }



	/* Control endpoints are handled differently (simpler, actually):
	 *  * There is no ping-pong mode.
	 *  * The RWALL (Read/Write ALLowed) bit and the FIFOCON (FIFO CONtrol) are ignored and read zero.
	 */
	int16_t _readFromControlEndpoint(const uint8_t endpoint, uint8_t* data, int16_t len, bool continuation) {
		uint8_t *ptr_dest = data;
        
        if (!_inAReceivedSetupInterrupt(0))
            _waitForReceiveOUTAvailable(endpoint, continuation);
        
		int16_t available = _getEndpointBufferCount(endpoint);

		// If there's nothing to read, return -1
		if (available == 0 || len < 1)
			return 0;

		int32_t to_read = available < len ? available : len;
		int32_t i = to_read;

		while (i--)
			*ptr_dest++ = *_endpointBuffer[endpoint]++;

        if (_getEndpointBufferCount(endpoint) == 0) {
			_clearReceiveOUT(endpoint);
			_resetEndpointBuffer(endpoint);
		}

		return to_read;
	}

	int16_t _sendToControlEndpoint(const uint8_t endpoint, const uint8_t* data, int16_t length, bool continuation) {
		const uint8_t *ptr_src = data;

		_waitForTransmitINAvailable(endpoint, /*reset_needed = */continuation);

		int16_t to_send = endpointSizes[endpoint] - _getEndpointBufferCount(endpoint);
		if (to_send > length)
			to_send = length;
        
        // We return how much we sent...
        length = to_send;

		if (to_send == 0)
			return 0;

		while (to_send--) {
			*_endpointBuffer[endpoint]++ = *ptr_src++;
		}

		// If we filled the buffer, flush
		// Note that this flush is different for a control endpoint.
		if (_getEndpointBufferCount(endpoint) == endpointSizes[endpoint]) {
			_clearTransmitIN(endpoint);
			_resetEndpointBuffer(endpoint);
		}
		return length;
	}

	// Returns -1 if nothing was available.
	int16_t _readByteFromEndpoint(const uint8_t endpoint) {
		// We use a while in the case where the last read emptied the buffer.
		// If we get a character we just return right out.
		while (_isFIFOControlAvailable(endpoint)) {
			if (!_isReadWriteAllowed(endpoint)) {
				// We cheat and lazily clear the RXOUT interrupt.
				// Once we might actually use that interrupt, we might need to be more proactive.
				_clearReceiveOUT(endpoint);

				// Clearing FIFOCon will also mark this bank as "read".
				_clearFIFOControl(endpoint);
				_resetEndpointBuffer(endpoint);

				// FOFCon will either be low now
				// -OR- will be high again if there's another bank of data available.
				continue;
			}

			return *_endpointBuffer[endpoint]++;
		}
		return -1;
	}

//     int16_t _readFromEndpoint(const uint8_t endpoint, uint8_t* data, int16_t length) {
//             //              _resetEndpointBuffer(endpoint);
//             uint8_t *ptr_dest = data;
//             int16_t read = 0;
//
//             // Available = -1 means that FIFOCONtrol might not be high.
//             // It should only be >= 0 once we've proven that it is.
//             int16_t available = -1;
//
//             while (length > 0 && _isFIFOControlAvailable(endpoint)) {
//                     available = _getEndpointBufferCount(endpoint);
//
//                     if (!available) {
//                             // We cheat and lazily clear the RXOUT interrupt.
//                             // Once we might actually use that interru;t, we might need to be more proactive.
//                             _clearReceiveOUT(endpoint);
//
//                             // Clearing FIFOCon will also mark this bank as "read".
//                             _clearFIFOControl(endpoint);
//                             _resetEndpointBuffer(endpoint);
//
//                             // FOFCon will either be low now
//                             // -OR- will be high again if there's another bank of data available.
//                             // If it stays low, we ned to make sure we don't reset it again when we test available before returning.
//                             // BECAUSE we don't have interrupts off, and it could, possibly, go high again before then.
//                             // In that case, we would be flushing a full buffer.
//                             available = -1;
//                             continue;
//                     }
//
//                     int16_t to_read = available < length ? available : length;
//                     int16_t i = to_read;
//
//                     while (i--) {
//                             *ptr_dest++ = *_endpointBuffer[endpoint]++;
//                     }
//                     available -= to_read;
//                     length -= to_read;
//                     read += to_read;
//             }
//
//             // If available is negative, we don't want to flush.
//             if (!available) {
//                     // Flush. See notes at the top of the function.
//                     _clearReceiveOUT(endpoint);
//                     _clearFIFOControl(endpoint);
//                     _resetEndpointBuffer(endpoint);
//             }
//
//             return read;
//     }


	void _flushReadEndpoint(uint8_t endpoint) {
		while(_isFIFOControlAvailable(endpoint)) {
			_clearReceiveOUT(endpoint);
			_clearFIFOControl(endpoint);
		}
		_resetEndpointBuffer(endpoint);
	}

	// Flush an endpoint after sending data.
    void _flushEndpoint(uint8_t endpoint) {
        _clearFIFOControl(endpoint);
        _resetEndpointBuffer(endpoint);
    }

	// Send the data in a buffer to an endpoint.
	// Does not automatically flush UNLESS it fills an endpoint buffer exactly.
	int16_t _sendToEndpoint(const uint8_t endpoint, const uint8_t* data, int16_t length) {
		const uint8_t *ptr_src = data;
		int16_t sent = 0;

		// While we have more to send AND the buffer is available
		while (length > 0 && _isFIFOControlAvailable(endpoint)) {
			if (!_isReadWriteAllowed(endpoint)) {
				_flushEndpoint(endpoint);
				continue;
			}

			if (_isTransmitINAvailable(endpoint)) {
				// Ack the Transmit IN.
				_clearTransmitIN(endpoint);

				// Reset the endpoint buffer -- it probably just changed.
				_resetEndpointBuffer(endpoint);
			}

			while (_isReadWriteAllowed(endpoint) && length--) {
				*_endpointBuffer[endpoint]++ = *ptr_src++;
				sent++;
			}

			// If we filled the buffer then flush.
			// The loop will check to see if the next buffer, if any, is available.
			// Note that this flush is different than that for a control endpoint.
			if (!_isReadWriteAllowed(endpoint)) {
				_flushEndpoint(endpoint);
			}
		}

		return sent;
	}

	// Send a single byte
	// NOTE: This is the same for control endpoints, since it doesn't attempt to flush.
	// WARNING: The doesn't check to see if the device is ready to send or has buffer available.
	// ONLY USE THIS WHEN YOU KNOW THE STATE OF THE USB DEVICE.
	void _sendToEndpoint(const uint8_t endpoint, uint8_t data) {
		*_endpointBuffer[endpoint]++ = data;
	}

	/* ############################################# */
	/* #                                           # */
	/* #             SAM USB Interrupt             # */
	/* #                                           # */
	/* ############################################# */

	extern "C"
	void UOTGHS_Handler( void ) {
		// End of bus reset
		if ( _inAResetInterrupt() )
		{
			TRACE_CORE(printf(">>> End of Reset\r\n");)

			// Reset USB address to 0
			_setUSBAddress(0);

			// Configure EP 0 -- there's no opportunity to have a second configuration
			_initEndpoint(0, USBProxy.getEndpointConfig(0, /* otherSpeed = */ false));
			endpointSizes[0] = USBProxy.getEndpointSize(0, /* otherSpeed = */ false);

			_enableReceivedSetupInterrupt(0);
			_enableEndpointInterrupt(0);

			_configuration = 0;
			_ackReset();
		}
/*
		if ( _inAnEndpointInterrupt(CDC_RX) )
		{
			_ackOutRecieved(CDC_RX);

			// Handle received bytes
			while (USBD_Available(CDC_RX))
				SerialUSB.accept();

			_ackFIFOControl(CDC_RX);
		}
*/
		if (_inAStartOfFrameInterrupt())
		{
			// Every millisecond ...
			_ackStartOfFrame();
		}

		// EP 0 Interrupt
		if ( _inAnEndpointInterrupt(0) )
		{
			if ( !_inAReceivedSetupInterrupt(0) )
			{
				return;
			}
			/****
			 A SETUP request is always ACKed. When a new SETUP packet is received, the UOTGHS_DEVEPTISRx.RXSTPI is set; the Received OUT Data Interrupt (UOTGHS_DEVEPTISRx.RXOUTI) bit is not.
			 ****/

			_resetEndpointBuffer(0);
			static Setup_t setup;
			_readFromControlEndpoint(0, (uint8_t*)&setup, 8, /*continuation =*/false);
			/****
			 • the UOTGHS_DEVEPTISRx.RXSTPI bit, which is set when a new SETUP packet is received and which shall be cleared by firmware to acknowledge the packet and to **free the bank**;
			 ****/
			_clearReceivedSetupInterrupt(0);

			if (setup.isADeviceToHostRequest())
			{
				TRACE_CORE(puts(">>> EP0 Int: IN Request\r\n");)
				/****
				 • the Transmitted IN Data Interrupt (UOTGHS_DEVEPTISRx.TXINI) bit, which is set when the current bank is ready to accept a new IN packet and [...]
				 ****/
				_waitForTransmitINAvailable(0);
			}
			else
			{
				TRACE_CORE(puts(">>> EP0 Int: OUT Request\r\n");)
				/****
				 [...] which shall be cleared by firmware to send the packet.
				 ****/
				_clearTransmitIN(0);
				_resetEndpointBuffer(0);
			}

			bool ok = true;
			if (setup.isAStandardRequestType())
			{
				// Standard Requests
				if (setup.isAGetStatusRequest())
				{
					if( setup.isADeviceRequest() )
					{
						// Send the device status
						TRACE_CORE(puts(">>> EP0 Int: kGetStatus\r\n");)
						// Check current configuration for power mode (if device is configured)
						// TODO
						// Check if remote wake-up is enabled
						// TODO
						_sendToEndpoint(0, 0); // TODO
						_sendToEndpoint(0, 0);
					}
					// if( setup.isAnEndpointRequest() )
					else
					{
						// Send the endpoint status
						// Check if the endpoint if currently halted
						if( _halted == 1 )
							_sendToEndpoint(0, 1); // TODO
						else
							_sendToEndpoint(0, 0); // TODO
						_sendToEndpoint(0, 0);
					}
				}
				else if ( setup.isAClearFeatureRequest() )
				{
					// Check which is the selected feature
					if( setup.featureToSetOrClear() == Setup_t::kSetupDeviceRemoteWakeup )
					{
						// Enable remote wake-up and send a ZLP
						if( _remoteWakeupEnabled == 1 )
							_sendToEndpoint(0, 1);
						else
							_sendToEndpoint(0, 0);
						_sendToEndpoint(0, 0);
					}
					else // if( setup.featureToSetOrClear() == kSetupEndpointHalt )
					{
						_halted = 0;  // TODO
						_sendToEndpoint(0, 0);
						_sendToEndpoint(0, 0);
					}

				}
				else if (setup.isASetFeatureRequest())
				{
					// Check which is the selected feature
					if( setup.featureToSetOrClear() == Setup_t::kSetupDeviceRemoteWakeup )
					{
						// Enable remote wake-up and send a ZLP
						_remoteWakeupEnabled = 1;
						_sendToEndpoint(0, 0);
					}
					if( setup.featureToSetOrClear() == Setup_t::kSetupEndpointHalt )
					{
						// Halt endpoint
						_halted = 1;
						//USBD_Halt(USBGenericRequest_GetEndpointNumber(pRequest));
						_sendToEndpoint(0, 0);
					}
					if( setup.featureToSetOrClear() == Setup_t::kSetupTestMode )
					{
#if 0
						// 7.1.20 Test Mode Support, 9.4.9 SetFeature
						if( (setup.bmRequestType == 0 /*USBGenericRequest_DEVICE*/) &&
						   ((setup.wIndex & 0x000F) == 0) )
						{
							// the lower byte of wIndex must be zero
							// the most significant byte of wIndex is used to specify the specific test mode

							UOTGHS->UOTGHS_DEVIDR &= ~UOTGHS_DEVIDR_SUSPEC;
							UOTGHS->UOTGHS_DEVCTRL |= UOTGHS_DEVCTRL_SPDCONF_HIGH_SPEED; // remove suspend ?

							Test_Mode_Support( (setup.wIndex & 0xFF00)>>8 );
						}
#endif
					}
				}
				else if (setup.isASetAddressRequest())
				{
					TRACE_CORE(puts(">>> EP0 Int: kSetAddress\r\n");)
					_waitForTransmitINAvailable(0); // <-- Why?! -RG
					_setUSBAddress(setup.valueLow());
				}
				else if (setup.isAGetDescriptorRequest())
				{
					TRACE_CORE(puts(">>> EP0 Int: kGetDescriptor\r\n");)
					ok = USBProxy.sendDescriptorOrConfig(setup);
				}
				else if (setup.isASetDescriptorRequest())
				{
					TRACE_CORE(puts(">>> EP0 Int: kSetDescriptor\r\n");)
					ok = false;
				}
				else if (setup.isAGetConfigurationRequest())
				{
					TRACE_CORE(puts(">>> EP0 Int: kGetConfiguration\r\n");)
					_sendToEndpoint(0, _configuration);
				}
				else if (setup.isASetConfigurationRequest())
				{
					if (setup.isADeviceRequest())
					{
						TRACE_CORE(printf(">>> EP0 Int: kSetConfiguration REQUEST_DEVICE %d\r\n", setup.wValueL);)

						// _configuration should be set to 1 for high-speed, and 2 for full-speed
						_configuration = setup.valueLow();

						uint8_t first_endpoint, total_endpoints;
						total_endpoints = USBProxy.getEndpointCount(first_endpoint);
						for (uint8_t ep = first_endpoint; ep < total_endpoints; ep++) {
							_initEndpoint(ep, USBProxy.getEndpointConfig(ep, /* otherSpeed = */ _configuration == 2));
							endpointSizes[ep] = USBProxy.getEndpointSize(ep, /* otherSpeed = */ _configuration == 2);
						}
						ok = true;

/* OLD CODE
						// Enable interrupt for CDC reception from host (OUT packet)
						udd_enable_out_received_interrupt(CDC_RX);
						udd_enable_endpoint_interrupt(CDC_RX);
*/
					}
					else
					{
						TRACE_CORE(puts(">>> EP0 Int: kSetConfiguration failed!\r\n");)
						ok = false;
					}
				}
				else if (setup.isAGetInterfaceRequest())
				{
					TRACE_CORE(puts(">>> EP0 Int: kGetInterface\r\n");)
					_sendToEndpoint(0, _set_interface);
				}
				else if (setup.isASetInterfaceRequest())
				{
					_set_interface = setup.valueLow();
					TRACE_CORE(puts(">>> EP0 Int: kSetInterface\r\n");)
				}
			}
			else
			{
				TRACE_CORE(puts(">>> EP0 Int: ClassInterfaceRequest\r\n");)

				// GAH! This is ugly.
				_waitForTransmitINAvailable(0); // Old Arduino Workaround: need tempo here, else CDC serial won't open correctly

				// Note: setup.length() holds the max length of transfer
				ok = USBProxy.handleNonstandardRequest(setup);
			}

			if (ok)
			{
				TRACE_CORE(puts(">>> EP0 Int: Send packet\r\n");)
                _clearReceiveOUT(0);
				_clearTransmitIN(0);
				_resetEndpointBuffer(0);
			}
			else
			{
				TRACE_CORE(puts(">>> EP0 Int: Stall\r\n");)
				_requestStall(0);
			}
		}
		// EP 0 Interrupt
		// FIXME! Needs to handle *any* control endpoint.
		else if ( _inAnEndpointInterruptNotControl() )
		{
			if ( _inAnOverflowInterrupt(_firstEndpointOfInterrupt()) )
			{
				while (1) {
                    ;// TRAP it!
                }
			}
		}
	}

} // Motate

#endif
//__SAM3X8E__
