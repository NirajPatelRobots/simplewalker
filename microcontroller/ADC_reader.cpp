#include "ADC_reader.hpp"
# define M_PI_2 1.5707963267948966

void ADCReader::connect_SPI() {
    spi_init(spi0, 976000);
    spi_set_format( spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST );
    gpio_set_function(SPI_Tx_pin, GPIO_FUNC_SPI);
    gpio_set_function(SPI_Rx_pin, GPIO_FUNC_SPI);
    gpio_set_function(SPI_CLK_pin, GPIO_FUNC_SPI);
    gpio_init(SPI_CS_pin);
    gpio_set_dir(SPI_CS_pin, GPIO_OUT);
    gpio_put(SPI_CS_pin, 1);
}

shared_ptr<ADCChannel> ADCReader::set_channel(const std::string &_name, int channel_num, int _offset, float _gain) {
    shared_ptr<ADCChannel> channel = get_channel(channel_num);
    if (channel) {
        channel->name = _name;
        channel->offset = _offset;
        channel->gain = _gain;
    } else {
        channel = make_shared<ADCChannel>(_name, channel_num, _offset, _gain);
        channels.at(channel_num) = channel;
    }
    return channel;
}

int ADCReader::read_ADC_raw(int channel_num) const {
    asm volatile("nop \n nop \n nop"); // incantation from pico examples
    gpio_put(SPI_CS_pin, 0);
    asm volatile("nop \n nop \n nop");
    sleep_ms(2);
    uint8_t indata[3], outdata[3] = {1, 0, 0};
    outdata[1] = (8 + channel_num) << 4;
    spi_write_read_blocking(spi0, outdata, indata, 3);
    asm volatile("nop \n nop \n nop");
    gpio_put(SPI_CS_pin, 1);
    asm volatile("nop \n nop \n nop");
    return (int)(((indata[1] & 3) << 8 ) | indata[2]);
}

float ADCReader::read_ADC_scaled(int channel_num) {
    shared_ptr<ADCChannel> channel = get_channel(channel_num);
    if (!channel) return 0;
    channel->set(read_ADC_raw(channel_num));
    return channel->scaled_value;
}

float ADCReader::read_ADC_scaled(const std::string &channel_name) {
    shared_ptr<ADCChannel> channel = get_channel(channel_name);
    if (!channel) return 0;
    channel->set(read_ADC_raw(channel->channel_num));
    return channel->scaled_value;
}

shared_ptr<ADCChannel> ADCReader::get_channel(int channel_num) {
    for (auto &channel : channels) {
        if (channel && channel_num == channel->channel_num) return channel;
    }
    return shared_ptr<ADCChannel>();
}

shared_ptr<ADCChannel> ADCReader::get_channel(const std::string &channel_name) {
    for (auto &channel : channels) {
        if (channel && channel_name == channel->name) return channel;
    }
    return shared_ptr<ADCChannel>();
}

bool ADCReader::set_pins(int SPI_CS, int SPI_Tx, int SPI_Rx, int SPI_CLK) {
    SPI_CS_pin = SPI_CS;
    SPI_Tx_pin = SPI_Tx;
    SPI_Rx_pin = SPI_Rx;
    SPI_CLK_pin = SPI_CLK;
    return true;
}
