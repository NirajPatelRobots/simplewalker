
#include "hardware/spi.h"
#include "hardware/gpio.h"

class SensorReader {
    float angleOffset[4], angleGain[4];
    int read_ADC(int channel_num);
public:
    int SPI_CS_pin, SPI_Tx_pin, SPI_Rx_pin, SPI_CLK_pin;
    SensorReader(void);
    SensorReader(float[], float[]);
    float readBatteryVoltage(void);
    float readAngle(int);
};
