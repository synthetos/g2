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
    device_t device;
    SPIMessage message;

    // We don't want to transmit until we're inited
    bool inited = false;

    // Timer to keep track of when we need to do another periodic update
    Motate::Timeout check_timer;

    // we only read/write 4 bytes at a time in this example
    alignas(4) uint8_t read_buffer[4];
    alignas(4) uint8_t write_buffer[4];

  public:
    // Primary constructor - templated to take any SPIBus and chipSelect type
    template <typename SPIBus_t, typename chipSelect_t>
    SDCard(SPIBus_t &spi_bus, const chipSelect_t &_cs)
        : device{spi_bus.getDevice(_cs,    // pass it the chipSelect
                                  4000000, //frequency in Hz
                                  SPIDeviceMode::kSPIMode0 | SPIDeviceMode::kSPI8Bit, // device mode
                                  0,       // min_between_cs_delay_ns
                                  0,       // cs_to_sck_delay_ns
                                  0        // between_word_delay_ns
                                  )}
    {
        init();
    };

    // Allow the default move constructor
    SDCard(SDCard &&other) = default;

    // Prevent copying
    SDCard(const SDCard &) = delete;

    void init() {
        message.message_done_callback = [&] { this->messageDoneCallback(); };

        // Establish default values
        // blah blah balh

        // mark that init has finished and set the timer
        inited = true;
    };

    void messageDoneCallback() {
        check_timer.set(1);  // don't send again until 1ms has past

        // do something here with the data in read_buffer, if you wish
    }

    // this would be called by the project or from a SysTickHandler
    void periodicCheck() {
        if (!inited || (check_timer.isSet() && !check_timer.isPast())) {
            // not yet, too soon
            return;
        }
        // writing, set it up
        write_buffer[0] = 0x0B;
        write_buffer[1] = 0x0E;
        write_buffer[2] = 0x0E;
        write_buffer[3] = 0x0F;

        message.setup(write_buffer, read_buffer, /*side:*/ 4, SPIMessage::DeassertAfter, SPIMessage::EndTransaction);
        device.queueMessage(&message);
    }
};

#endif // sd_card_h
