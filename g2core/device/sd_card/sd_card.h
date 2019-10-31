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

    // we only read/write 4 bytes at a time in this example
    alignas(4) uint8_t read_buffer[4];
    alignas(4) uint8_t write_buffer[4];

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

    uint8_t _scribble_buffer[1];

    bool _spi_write = false;
    bool _spi_read = false;

    uint8_t *_spi_data;

    void _startNextReadWrite()
    {
        if (_transmitting || !_inited) { return; }
        _transmitting = true; // preemptively say we're transmitting .. as a mutex

        // Set up SPI buffers
        _scribble_buffer[0] = 0x00; //ms: allow to change for sendAsNoop

        // We write before we read -- so we don't lose what we set in the registers when writing
        if (_spi_write) { 
            _spi_write = false;
            _message.setup(_spi_data, _scribble_buffer, 1, SPIMessage::DeassertAfter, SPIMessage::EndTransaction);

        } else if (_spi_read) {
            _spi_read = false;
            _message.setup(_scribble_buffer, _spi_data, 1, SPIMessage::DeassertAfter, SPIMessage::EndTransaction);

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
        
        // mark that init has finished and set the timer
        _inited = true;
    };

    void messageDoneCallback() {
        check_timer.set(1);  // don't send again until 1ms has past

        // Clear mutex and set up next read/write
        _transmitting = false;
        _startNextReadWrite();
    };

    void read(bool last_transfer = false, uint8_t send_as_noop = 0x00) {
        _spi_read = true;
        _spi_data = (uint8_t*)&send_as_noop;
        _startNextReadWrite();
    };

    void write(uint8_t data, bool last_transfer = false) {
        _spi_write = true;
        _spi_data = (uint8_t*)&data;
        _startNextReadWrite();
    };

    // this would be called by the project or from a SysTickHandler
    void periodicCheck() {
        if (!_inited || (check_timer.isSet() && !check_timer.isPast())) {
            // not yet, too soon
            return;
        }

        //TEMP
        this->write(0x01);
        this->write(0x03);
        this->write(0x05);
        this->write(0x07);
        //TEMP
    };

};

#endif // sd_card_h
