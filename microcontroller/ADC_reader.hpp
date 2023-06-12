/*
 * Class for reading from a MCP3008 ADC over SPI,
 * transforming and storing channel information.
 */
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
    float get_scaled_offset() { return gain * offset;}
};

class ADCReader {
    int SPI_CS_pin{1}, SPI_Tx_pin{3}, SPI_Rx_pin{0}, SPI_CLK_pin{2};
public:
    std::array<shared_ptr<ADCChannel>, 8> channels;
    ADCReader() = default;
    void connect_SPI();
    shared_ptr<ADCChannel> set_channel(const std::string &_name, int channel, int _offset, float _gain);
    int read_ADC_raw(int channel_num) const;
    float read_ADC_scaled(int channel_num);
    float read_ADC_scaled(const std::string &channel_name);
    shared_ptr<ADCChannel> get_channel(int channel_num);
    shared_ptr<ADCChannel> get_channel(const std::string &channel_name);
    float readBatteryVoltage(void);
    float readAngle(int);
    bool set_pins(int SPI_CS, int SPI_Tx, int SPI_Rx, int SPI_CLK);
};
