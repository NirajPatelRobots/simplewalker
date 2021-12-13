#include "sensorReader.hpp"
# define M_PI_2 1.5707963267948966

SensorReader::SensorReader(float angleZero[], float angleRight[]) {
    /* angleZero and angleRight are the values of the sensor when the motor is at angle zero and pi/2 (right angle) */
    for (int i=0; i < 4; i++) {
        angleOffset[i] = angleZero[i];
        angleGain[i] = M_PI_2 / (angleRight[i] - angleZero[i]);
    }
    SPI_CS_pin = 1;
    SPI_Tx_pin = 3;
    SPI_Rx_pin = 0;
    SPI_CLK_pin = 2;
    // SPI
    spi_init(spi0, 1000 * 1000); // 1MHz
    gpio_set_function(SPI_Tx_pin, GPIO_FUNC_SPI);
    gpio_set_function(SPI_Rx_pin, GPIO_FUNC_SPI);
    gpio_set_function(SPI_CLK_pin, GPIO_FUNC_SPI);
    gpio_init(SPI_CS_pin);
    gpio_set_dir(SPI_CS_pin, GPIO_OUT);
    gpio_put(SPI_CS_pin, 1);
}

SensorReader::SensorReader(void) {
    float zero[4] = {0,0,0,0};
    SensorReader(zero, zero);
}

int SensorReader::read_ADC(int channel_num) {
    gpio_put(SPI_CS_pin, 0);
    uint8_t indata[3], outdata[3] = {1, 0, 0};
    outdata[1] = (8 + channel_num) << 4;
    spi_write_read_blocking(spi0, outdata, indata, sizeof(outdata));
    gpio_put(SPI_CS_pin, 1);
    int value = ((indata[1] & 3) << 8 ) | indata[2];
    return value;
}

float SensorReader::readBatteryVoltage(void) {
    /*return battery voltage (float).*/
    /* R_high = 329 kOhm R_low = 993 kOhm
    O-------/\/\/\/---o----/\/\/\/----O
    V_Batt  R_high  sensor  R_low   GND
    V_sens = V_Batt*R_low/(R_high+R_low)
    scale = V_ref/V_sens/max_sense = 3.3/0.249/1023 */
    return 0.0130 * read_ADC(0);
}

float SensorReader::readAngle(int anglenum) {
    return ((float)read_ADC(anglenum+1) - angleOffset[anglenum]) * angleGain[anglenum];
}
