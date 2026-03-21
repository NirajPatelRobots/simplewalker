/*
 * Class for reading from a MCP3008 ADC over SPI,
 * transforming and storing channel information.
 * Should work for MCP3004, except that attempting to access channels 4-7 will access channels 0-3
 * Measured: ~16us per read
 * TODO:
 */
#ifndef SIMPLEWALKER_PICO_ADC_READER_HPP
#define SIMPLEWALKER_PICO_ADC_READER_HPP
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <string>
#include <array>
#include <memory>
using std::shared_ptr, std::make_shared;


struct ADCChannel {
    float gain, scaled_value;
    int offset,raw_value, channel_num;
    std::string name;
    ADCChannel(const std::string &_name, int channel, int _offset, float _gain)
        : gain(_gain), scaled_value(), offset(_offset), raw_value(), channel_num(channel), name(_name) {}
    void set(int raw) {
        raw_value = raw;
        scaled_value = gain * (raw + offset);
    }
    float get_scaled_offset() const {return gain * (float)offset;}
};

class ADCReader {
    int SPI_CS_pin{1}, SPI_Tx_pin{3}, SPI_Rx_pin{0}, SPI_CLK_pin{2};
public:
    static const size_t NUM_CHANNELS = 8;
    std::array<shared_ptr<ADCChannel>, NUM_CHANNELS> channels;
    ADCReader() = default;
    void connect_SPI();
    shared_ptr<ADCChannel> set_channel(const std::string &_name, int channel, int _offset, float _gain);
    int read_ADC_raw(int channel_num) const;
    float read_ADC_scaled(int channel_num);
    float read_ADC_scaled(const std::string &channel_name);
    shared_ptr<ADCChannel> get_channel(int channel_num);
    shared_ptr<ADCChannel> get_channel(const std::string &channel_name);
    bool set_pins(int SPI_CS, int SPI_Tx, int SPI_Rx, int SPI_CLK);
};

#endif
