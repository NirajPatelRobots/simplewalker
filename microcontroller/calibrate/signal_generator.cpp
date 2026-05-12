#include "signal_generator.hpp"
#include <math.h>
#include <optional>


class SineExcitationGenerator : public ExcitationSignalGenerator {
public:
    int place, loop_num;
    float frequency_scale, amplitude_scale;

    SineExcitationGenerator(float _frequency_scale, float _amplitude_scale)
        : place(), loop_num(), frequency_scale(_frequency_scale), amplitude_scale(_amplitude_scale)
    {}
    float get_next_value(float angVel) override {
        if (is_finished) return 0;
        float freq = 200.f * frequency_scale, loop_amp = amplitude_scale;
        int num_loops = 6;
        if (loop_num < num_loops)
            loop_amp *= (float) (loop_num + 1) / (num_loops + 1);
        else
            loop_amp *= 1.f - (float)(loop_num - num_loops + 1) / (num_loops + 1);
        float loop_freq = freq / abs(loop_amp);
        if (loop_num % 2 == 1) loop_amp *= -1;
        if (place++ >= 314.2 / loop_freq) {
            place = 0;
            loop_num++;
        }
        if (loop_num >= 2 * num_loops) {
            is_finished = true;
        }
        return loop_amp * sin(loop_freq * 0.01 * place);
    }
};


class SquareExcitationGenerator : public ExcitationSignalGenerator {
public:
    int place, loop_num;
    float frequency_scale, amplitude_scale;

    SquareExcitationGenerator(float _frequency_scale, float _amplitude_scale)
            : place(), loop_num(), frequency_scale(_frequency_scale), amplitude_scale(_amplitude_scale)
    {}
    float get_next_value(float _) override {
        if (is_finished) return 0;
        float amp = amplitude_scale, V;
        int num_square = 3, square_length = (int) (0.5 / frequency_scale);

        ++place;
        if (loop_num < num_square) {
            if (place < square_length)           V = amp;
            else if (place < 3*square_length)    V = -amp;
            else                                 V = amp;
            if (place >= 4*square_length) {
                loop_num++;
                place = 0;
            }
        } else {
            if (place < 4 * square_length)      V = 0.0;
            else { //finally done
                is_finished = true;
                return V;
            }
        }
        return V;
    }
};


class DeadbandExcitationGenerator : public ExcitationSignalGenerator {
    float V, V_step = 0.01, moving_angvel_thresh = 0.2;
    std::optional<float> moving_V, stationary_V;
    int direction, check_cycles, num_cycle_check;
    bool has_reversed = false;
public:
    DeadbandExcitationGenerator(float frequency_scale, float amplitude_scale) :
            V(0), moving_angvel_thresh(0.1f * fabs(amplitude_scale)), moving_V(), stationary_V(),
            direction(amplitude_scale > 0 ? 1 : -1), check_cycles(0),
            num_cycle_check(ceil(0.1 / frequency_scale))
    {}
    float get_next_value(float angVel) override {
        if (is_finished) return 0;
        static float avg_angvel = 0;
        avg_angvel += angVel * direction;
        if (++check_cycles < num_cycle_check) {
            return V * direction;
        }
        avg_angvel /= num_cycle_check;
        check_cycles = 0;
        bool is_moving_wrong_direction = avg_angvel < -moving_angvel_thresh;
        bool is_moving = avg_angvel > moving_angvel_thresh;
        avg_angvel = 0; //reset avg
        if (is_moving_wrong_direction) {
            V += V_step;
        } else if (is_moving && V >= 0) {
            if (!moving_V.has_value() || V < moving_V) {
                moving_V = V;
                V -= V_step;
            }
        } else {
            if (!stationary_V.has_value() || V > stationary_V) {
                stationary_V = V;
            }
            if (moving_V.has_value()) { // if it has moved then stopped, we're done with this direction
                if (has_reversed) {
                    is_finished = true;
                    return 0.f;
                } else {
                    has_reversed = true;
                    direction = -1;
                    V *= -1;
                    moving_V.reset();
                    stationary_V.reset();
                    num_cycle_check *= 2;  // IDK why it's quicker to recognize movement backwards
                }
            } else { // if we haven't moved yet
                V += V_step;
            }
        }
        return V * direction;
    }
};


class ConstantVoltageGenerator : public ExcitationSignalGenerator {
    float amplitude;
    int i, num_samples;
public:
    ConstantVoltageGenerator(float amplitude_scale, float frequency_scale) :
            amplitude(amplitude_scale), i(0), num_samples(ceil(1 / frequency_scale))
    {}
    float get_next_value(float _) override {
        if (is_finished) return 0;
        is_finished = (i >= num_samples);
        return amplitude;
    }
};


std::shared_ptr<ExcitationSignalGenerator> make_signal_generator(
        MotorCalibrationInputType type, float frequency_scale, float amplitude_scale) {
    switch (type) {
        case CAL_INPUT_SINES:
            return std::make_shared<SineExcitationGenerator>(frequency_scale, amplitude_scale);
        case CAL_INPUT_SQU:
            return std::make_shared<SquareExcitationGenerator>(frequency_scale, amplitude_scale);
        case CAL_INPUT_DEADBAND:
            return std::make_shared<DeadbandExcitationGenerator>(frequency_scale, amplitude_scale);
        case CAL_INPUT_CONST:
            return std::make_shared<ConstantVoltageGenerator>(amplitude_scale, frequency_scale);
        default:
            return nullptr;
    }
}
