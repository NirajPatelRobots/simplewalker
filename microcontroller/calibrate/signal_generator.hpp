/* Generate signals to use as input voltage for motor calibration */
#include <memory>
#include "../../communication/messages.h"


class ExcitationSignalGenerator {
public:
    virtual float get_next_value(float angVel) = 0;
    bool is_finished = false;
};

std::shared_ptr<ExcitationSignalGenerator> make_signal_generator(
        MotorCalibrationInputType type, float frequency_scale, float amplitude_scale);
