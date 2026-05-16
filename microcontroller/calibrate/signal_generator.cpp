#include "signal_generator.hpp"
#include <math.h>


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
    float V, V_step, moving_angvel_thresh, iir_coeff, avg_angvel;
    bool has_moved;
    int direction, scale_direction;
public:
    DeadbandExcitationGenerator(float frequency_scale, float amplitude_scale) :
            V(0), V_step(0.001), moving_angvel_thresh(0.1f * fabs(amplitude_scale)),
            iir_coeff(fmin(1.0, frequency_scale)), avg_angvel(0), has_moved(false),
            direction(1), scale_direction(amplitude_scale > 0 ? 1 : -1)
    {}
    float get_next_value(float angVel) override {
        if (is_finished) return 0;
        avg_angvel = (angVel * direction * scale_direction) * iir_coeff + avg_angvel * (1 - iir_coeff);
        bool is_moving_wrong_direction = avg_angvel < -moving_angvel_thresh;
        bool is_moving = avg_angvel > moving_angvel_thresh;
        if (is_moving_wrong_direction || !(has_moved || is_moving)) {
            V += V_step;
        } else if (is_moving || V >= V_step) {
            has_moved = true;
            V -= V_step;
        } else { // if it has moved then returned to 0, we're done with this direction
            if (direction == -1) {  // done with both directions
                is_finished = true;
                return 0.f;
            } else {
                direction = -1;
                V *= -1;
                avg_angvel *= -1;
                has_moved = false;  // reset and reverse direction
            }
        }
        return V * direction * scale_direction;
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


std::unique_ptr<ExcitationSignalGenerator> make_signal_generator(
        MotorCalibrationInputType type, float frequency_scale, float amplitude_scale) {
    switch (type) {
        case CAL_INPUT_SINES:
            return std::make_unique<SineExcitationGenerator>(frequency_scale, amplitude_scale);
        case CAL_INPUT_SQU:
            return std::make_unique<SquareExcitationGenerator>(frequency_scale, amplitude_scale);
        case CAL_INPUT_DEADBAND:
            return std::make_unique<DeadbandExcitationGenerator>(frequency_scale, amplitude_scale);
        case CAL_INPUT_CONST:
            return std::make_unique<ConstantVoltageGenerator>(amplitude_scale, frequency_scale);
        default:
            return nullptr;
    }
}
