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
    bool _last_xfer = false;
    uint16_t _num_bytes = 0;

    uint8_t *_spi_data;

    void _startNextReadWrite()
    {
        if (_transmitting || !_inited) { return; }
        _transmitting = true; // preemptively say we're transmitting .. as a mutex

        // Set up SPI buffers
        _scribble_buffer[0] = 0x00;

        // We write before we read -- so we don't lose what we set in the registers when writing
        if (_spi_write) { 
            _spi_write = false;
            _message.setup(_spi_data, _scribble_buffer, _num_bytes, _last_xfer, SPIMessage::EndTransaction);

        } else if (_spi_read) {
            _spi_read = false;
            _message.setup(_scribble_buffer, _spi_data, _num_bytes, _last_xfer, SPIMessage::EndTransaction);

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
        _last_xfer = false;
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

    void read(uint8_t *data, const uint16_t num_bytes, const bool last_transfer = SPIMessage::RemainAsserted) {
        
        // Configure multi byte read
        _spi_read = true;
        _spi_data = data;
        _last_xfer = last_transfer;
        _num_bytes = num_bytes;

        // Set up read
        _startNextReadWrite();
    };

    void read(uint8_t *data, const bool last_transfer = SPIMessage::RemainAsserted) {
        
        // Configure and set up single byte read
        read(data, 1, last_transfer);
    };

    void write(uint8_t *data, const uint16_t num_bytes, const bool last_transfer = SPIMessage::RemainAsserted) {

        // Configure multi byte write
        _spi_write = true;
        _spi_data = data;
        _last_xfer = last_transfer;
        _num_bytes = num_bytes;

        // Set up write
        _startNextReadWrite();
    };

    void write(uint8_t data, const bool last_transfer = SPIMessage::RemainAsserted) {
        
        // Configure and set up single byte write
        write(&data, 1, last_transfer);
    };

    // this would be called by the project or from a SysTickHandler
    void periodicCheck() {
        if (!_inited || (check_timer.isSet() && !check_timer.isPast())) {
            // not yet, too soon
            return;
        }

        //TEMP
        /*
        uint8_t rd[5] = {0x2, 0x4, 0x6, 0x8, 0xA};
        this->read(rd, 5, SPIMessage::DeassertAfter);
        this->write(rd, 5, SPIMessage::DeassertAfter);
        uint8_t rd = 0x2;
        this->read(&rd);
        this->write(rd, SPIMessage::DeassertAfter);
        this->write(0x01, SPIMessage::RemainAsserted);
        this->write(0x03, SPIMessage::DeassertAfter);
        this->write(0x05, SPIMessage::RemainAsserted);
        this->write(0x07, SPIMessage::DeassertAfter);
        static uint8_t stuff[4] = {0x02, 0x04, 0x06, 0x08};
        static uint8_t stuff2[4] = {0x0A, 0x0C, 0x0E, 0x0F};
        this->write(stuff, 4, SPIMessage::RemainAsserted);
        this->write(stuff2, 4, SPIMessage::DeassertAfter);*/
        //TEMP
    };

};

#endif // sd_card_h
