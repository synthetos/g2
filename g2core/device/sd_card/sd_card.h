/*
 * sd_card.h - support for talking to an SD Card over the SPI bus
 * This file is part of the G2 project
 *
 * Copyright (c) 2014 ChaN
 * Copyright (c) 2019 Matt Staniszewski
 *
 */

#ifndef sd_card_h
#define sd_card_h

using Motate::SPIMessage;
using Motate::SPIInterrupt;
using Motate::SPIDeviceMode;

/* Definitions */
#define SCRIBBLE_BUF_MAX    10  // Maximum number of bytes expected for toss

template <typename device_t>
struct SDCard final {
  private:
    // SPI and message handling properties
    device_t _device;
    SPIMessage _message;

    // Record if we're transmitting to prevent altering the buffers while they
    // are being transmitted still.
    volatile bool _transmitting = false;

    // We don't want to transmit until we're inited
    bool _inited = false;

    // Timer to keep track of when we need to do another periodic update
    Motate::Timeout check_timer;

  public:
    // Primary constructor - templated to take any SPIBus and chipSelect type
    template <typename SPIBus_t, typename chipSelect_t>
    SDCard(SPIBus_t &spi_bus, const chipSelect_t &_cs)
        : _device{spi_bus.getDevice(_cs,    // pass it the chipSelect
                                    4000000,
                                    SPIDeviceMode::kSPIMode0 | SPIDeviceMode::kSPI8Bit,
                                    0, // min_between_cs_delay_ns
                                    0, // cs_to_sck_delay_ns
                                    0   // between_word_delay_ns
                                    )}
    {
        init();
    };

    // Allow the default move constructor
    SDCard(SDCard &&other) = default;

    // Prevent copying
    SDCard(const SDCard &) = delete;

    // Toss out buffer
    uint8_t _scribble_buffer[SCRIBBLE_BUF_MAX];

    bool _spi_write = false;
    bool _spi_read = false;
    bool _deassert_cs = false;
    uint8_t _send_as_noop = 0x0;
    uint16_t _num_bytes = 0;

    uint8_t *_spi_data;

    void _startNextReadWrite()
    {
        if (_transmitting || !_inited) { return; }
        _transmitting = true; // preemptively say we're transmitting .. as a mutex

        // We write before we read -- so we don't lose what we set in the registers when writing
        if (_spi_write) { 
            _spi_write = false;
            _message.setup(_spi_data, _scribble_buffer, _num_bytes, _deassert_cs, SPIMessage::EndTransaction);

        } else if (_spi_read) {
            _spi_read = false;

            // Populate scribble buffer with no op bytes
            for (int i = 0; i < _num_bytes; i++) {
                _scribble_buffer[i] = _send_as_noop;
            }

            _message.setup(_scribble_buffer, _spi_data, _num_bytes, _deassert_cs, SPIMessage::EndTransaction);

        // otherwise we're done here
        } else {
            _transmitting = false; // we're not really transmitting.
            return;
        }
        _device.queueMessage(&_message);
    };

    void init() {
        _message.message_done_callback = [&] { this->messageDoneCallback(); };

        // Establish default values
        _spi_write = false;
        _spi_read = false;
        _deassert_cs = false;
        _send_as_noop = 0x0;
        _num_bytes = 0;
        
        // mark that init has finished and set the timer
        _inited = true;
    };

    void messageDoneCallback() {
        check_timer.set(1);  // don't send again until 1ms has past

        // Clear mutex and set up next read/write
        _transmitting = false;
        _startNextReadWrite();
    };

    void read(uint8_t *data, const uint16_t num_bytes, const bool deassert_cs = SPIMessage::RemainAsserted, const uint8_t send_as_noop = 0x0) {
        
        // Configure multi byte read
        _spi_read = true;
        _spi_data = data;
        _deassert_cs = deassert_cs;
        _send_as_noop = send_as_noop;
        _num_bytes = num_bytes;

        // Set up read
        _startNextReadWrite();
    };

    uint8_t read(const bool deassert_cs = SPIMessage::RemainAsserted, const uint8_t send_as_noop = 0x0) {
        
        uint8_t data;
        
        // Configure and set up single byte read
        read(&data, 1, deassert_cs, send_as_noop);
        
        return data;
    };

    void write(uint8_t *data, const uint16_t num_bytes, const bool deassert_cs = SPIMessage::RemainAsserted) {

        // Configure multi byte write
        _spi_write = true;
        _spi_data = data;
        _deassert_cs = deassert_cs;
        _num_bytes = num_bytes;

        // Set up write
        _startNextReadWrite();
    };

    void write(uint8_t data, const bool deassert_cs = SPIMessage::RemainAsserted) {
        
        // Configure and set up single byte write
        write(&data, 1, deassert_cs);
    };
};

#endif // sd_card_h
