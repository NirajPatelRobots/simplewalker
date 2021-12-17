/* test the motor, generating voltage and angle measurements.
Created October 2021 
TODO:
    pico stdin and stdout
    adjust + log other leg joint*/


#include "motors.hpp"
#include "sensorReader.hpp"
#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

volatile bool timer_fired = false;
bool timer_callback(struct repeating_timer *t) {
    timer_fired = true;
    return true;
}

int excitationVoltage(float frequency_scale, float amplitude_scale, float *V) {
    /* Choose voltage for test input to the motor.
    Starts at counter = 0, creates a waveform over each time it's called
    Made up of a sinusoid then a square wave */
    float freq = 500. * frequency_scale, amp = 2. * amplitude_scale;
    int num_loops = 6, num_square = 3, square_length = (int)(200.0 / freq);
    static int place, loop_num, extra_count;

    if ( loop_num < num_loops) { // loops
        float loop_amp = (loop_num+1)/(num_loops+1) * amp;
        float loop_freq = freq / loop_amp;
        if (loop_num % 2 == 1) loop_amp *= -1;
        *V = loop_amp * sin(loop_freq * 0.01 * place);
        if (place < 314.0 / loop_freq) {
            place++;
        } else {
            place = 0;
            loop_num++;
        }
    } else if (loop_num < 2 * num_loops) { // loops with wiggles
        float loop_amp = (1-(loop_num-num_loops+1)/(num_loops+1)) * amp;
        float loop_freq = freq / loop_amp;
        if (loop_num % 2 == 1) loop_amp *= -1;
        *V = loop_amp * sin(loop_freq * 0.01 * place) + 0.1 * amp * sin(13 * freq / amp * extra_count++);
        if (place < 314.0 / loop_freq) {
            place++;
        } else {
            place = 0;
            loop_num++;
        }
    } else {
        int square_num = (loop_num - (2 * num_loops)) / (4 *square_length);
        if (square_num < num_square) {
            if (place < square_length)           *V = amp;
            else if (place < 3*square_length)    *V = -amp;
            else                                 *V = amp;
            if (++place == 4*square_length) {
                loop_num++;
                place = 0;
            }
        } else {
            if (place++ < square_length)      *V = 0.0;
            else { //finally done
                loop_num = 0;
                place = 0;
                extra_count = 0;
                return 1;
            }
        }
    }
    return 0;
}


int checkMotorPerformance(int motorNum, float dt, Motors& motors, SensorReader& sensors,
                            float frequency_scale, float amplitude_scale) {
    float centerVal = M_PI_2, maxDisplacement = 0.45 * M_PI; //[rad]
    float kp = 0.002, V, lastV = 0.0, angle;


    //return to start
    int numInRange = 0;
    angle = 0.0;
    while (numInRange < 10) {
        angle = sensors.readAngle(motorNum);
        motors.setMotor((Motornum)motorNum, kp * (centerVal - angle));
        if (fabs(centerVal - angle) < 0.01) {
            numInRange++;
        } else {
            numInRange = 0;
        }
        sleep_ms(dt*1000);
    }
    int i = 0;
    struct repeating_timer timer;
    add_repeating_timer_us(-(int)(1000000 * dt), timer_callback, NULL, &timer);
    while(excitationVoltage(frequency_scale * dt, amplitude_scale, &V) == 0) {
        while (!timer_fired) {
            sleep_us(1);
        }
        timer_fired = false;
        float V_bat = sensors.readBatteryVoltage();
        if (V > V_bat) { // constrain to battery voltage
            V = V_bat;
        } else if (V < -V_bat) {
            V = -V_bat;
        }
        motors.setMotor((Motornum)motorNum, V / V_bat); 
        angle = sensors.readAngle(motorNum);
        if (fabs(centerVal - angle) > maxDisplacement) {
            printf("Test went out of range and was terminated\n");
            return -1;
        }
        printf("%f,%f,%f\n", dt * i, lastV, angle);
        i++;
        lastV = V; // shift i+1 because causality. So V[i] affects angle[i]
        sleep_ms(dt*1000.);
    }
    cancel_repeating_timer(&timer);
    return 0;
}



int main() {
    stdio_init_all();
    Motors motors;
    SensorReader sensors;
    //TODO: get info from sender
    int motorNum = 0;
    float dt = 0.003;
    float amp_scale = 1.0;
    float freq_scale = 1.0;
    //^info from sender
    while (1) {
        int inChar = getchar_timeout_us(400*1000);
        if (inChar == PICO_ERROR_TIMEOUT) {
            float battery_voltage = sensors.readBatteryVoltage();
            printf("Battery Voltage: %f\n", battery_voltage);
        }
        else {
            printf("read %c\n", inChar);
        }
        if (0) {
            printf("Running Amplitude %f, Frequency %f, dt %f, motorNum %f\n", amp_scale, freq_scale, dt, motorNum);
            //send in CSV format
            printf("Time (s), Voltage (V), Angle (rad)\n");
            checkMotorPerformance(motorNum, dt, motors, sensors, freq_scale, amp_scale);
        }
    }
}
