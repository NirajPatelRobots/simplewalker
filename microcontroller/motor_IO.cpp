#include "motor_IO.hpp"

MotorsIO::MotorsIO(const std::vector<motorIOSettings> &_all_motor_settings,
                         std::shared_ptr<ADCReader> &ADC)
     : all_motor_settings_(_all_motor_settings), ADC_(ADC), motor_outputs(), battery_voltage_(0) {
    for (const motorIOSettings &motor_settings : all_motor_settings_) {
        if (motor_settings.type == motorType::DC) {
            DCMotorPins *pinSettings = (DCMotorPins*)(motor_settings.motor_pin_settings);
            motor_outputs.push_back(std::make_unique<DCMotorOutput>(pinSettings->pin_forward, pinSettings->pin_reverse));
        }
        else if (motor_settings.type == motorType::Servo) {
            ServoMotorPins *pinSettings = (ServoMotorPins*)(motor_settings.motor_pin_settings);
            motor_outputs.push_back(std::make_unique<ServoMotorOutput>(pinSettings->pin));
        }

        if (motor_settings.angleSensorType == motorAngleSensorType::ADC) {
            if (ADC_) {
                ADC_->set_channel(motor_settings.name, motor_settings.sensor_channel_num,
                                  motor_settings.raw_angle_offset, motor_settings.angle_scale);
            } else {
                error = 1;
            }
        }
    }
}


bool MotorsIO::set_motor_voltage(int motor_num, float voltage) {
    int raw_angle;
    const motorIOSettings &motor_settings = all_motor_settings_[motor_num];
    if (motor_settings.angleSensorType == motorAngleSensorType::ADC && ADC_) {
        raw_angle = ADC_->get_channel(motor_settings.sensor_channel_num)->raw_value;
    } else return false;
    if (battery_voltage_ < 1) {
        return set_motor_error(motor_num, 0, 1);
    }
    if (raw_angle < motor_settings.raw_angle_min) {
        return set_motor_error(motor_num, 0.1, 1);
    }
    if (raw_angle > motor_settings.raw_angle_max) {
        return set_motor_error(motor_num, -0.1, 1);
    }
    voltage = std::min(voltage, motor_settings.max_voltage);
    motor_outputs[motor_num]->set_output(voltage / battery_voltage_);
    return true;
}

bool MotorsIO::set_motor_error(int motor_num, float fraction, int error_num) {
    error = error_num;
    motor_outputs[motor_num]->set_output(0);
    return false;
}

void MotorsIO::initialize_ADC_channels() {
    for (const motorIOSettings &motor_settings : all_motor_settings_) {
        if (motor_settings.angleSensorType == motorAngleSensorType::ADC && ADC_) {
            ADC_->set_channel(motor_settings.name, motor_settings.sensor_channel_num,
                             motor_settings.raw_angle_offset, motor_settings.angle_scale);
        }
    }
}
