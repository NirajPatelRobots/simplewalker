/* Created by Niraj Oct 2021 */

#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define NUM_MOTORS 6
enum Motornum {
    right_hip,
    right_knee,
    left_hip,
    left_knee,
    right_hip_out,
    left_hip_out,
};

class Motors {
    static uint pin_forward[NUM_MOTORS], pin_reverse[NUM_MOTORS];
    uint pwmslice_num[NUM_MOTORS], pwmwrap;
    float angle0offsetR, angle0offsetL;
public:
    Motors(void);
    void setMotor(Motornum, float);
};
