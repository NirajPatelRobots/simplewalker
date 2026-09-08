/* Generate signals to use as input voltage for motor calibration */
#ifndef SIMPLEWALKER_PICO_SIGNAL_GENERATOR_HPP
#define SIMPLEWALKER_PICO_SIGNAL_GENERATOR_HPP

#include <memory>
#include "../../communication/messages.h"


class ExcitationSignalGenerator {
public:
    virtual ~ExcitationSignalGenerator() = default;
    virtual float get_next_value(float angVel) = 0;
    bool is_finished = false;
};

std::unique_ptr<ExcitationSignalGenerator> make_signal_generator(
        MotorCalibrationInputType type, float frequency_scale, float amplitude_scale);

#endif  // SIMPLEWALKER_PICO_SIGNAL_GENERATOR_HPP
