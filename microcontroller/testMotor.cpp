/* test the motor, generating voltage and angle measurements.
Created October 2021 
TODO:
    pico stdin and stdout
    adjust + log other leg joint*/


#include <motors.hpp>
#include <sensorReader.hpp>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>

void excitationVoltage(std::vector<float> &V, float frequency_scale, float amplitude_scale) {
    /* create a vector of voltages over time as inputs to the motor.
    Made up of a sinusoid then a square wave */
    int num_loops = 20, num_square = 5;
    float freq = 1000. * frequency_scale;
    float amp = 2. * amplitude_scale;
    int squarelength = (int)(200.0 / freq)
    V.assign(2*squarelength, 0.);

    for (int loop = 0; loop < num_loops; loop++) { // loops
        float loop_amp = (loop+1)/(num_loops+1) * amp;
        float loop_freq = freq / loop_amp;
        if (loop % 2 == 1) loop_amp *= -1;
        for (float t = 0; t < (M_PI / freq); t += 0.01) {
            V.push_back(loop_amp * sin(loop_freq * t));
        }
    }

    for (int loop = 0; loop < num_loops; loop++) { // loops with wiggles
        float loop_amp = (1-(loop+1)/(num_loops+1)) * amp;
        float loop_freq = freq / loop_amp;
        if (loop % 2 == 1) loop_amp *= -1;
        for (float t = 0; t < (M_PI / freq); t += 0.01) {
            V.push_back(loop_amp * sin(loop_freq * t) + 0.2 * amp * sin(13 * freq / amp * t));
        }
    }

    for (int square = 0; square < num_square; square++) {
        for (int i = 0; i < square_length; i++)      V.push_back(amp);
        for (int i = 0; i < 2*square_length; i++)    V.push_back(-amp);
        for (int i = 0; i < square_length; i++)      V.push_back(amp);
    }
    for (int i = 0; i < square_length; i++)      V.push_back(0.0);
}


int checkMotorPerformance(int motorNum, float dt, std::vector<float> &V, std::vector<float> &angle,
                          float frequency_scale, float amplitude_scale) {
    float centerVal = M_PI_2; //[rad]
    float kp = 0.002;
    float maxDisplacement = 0.45 * np.pi; //[rad]
    Motors motors();
    SensorReader sensors();
    excitationVoltage(V, frequency_scale * dt, amplitude_scale);
    angle.reserve(V.size() - 1);

    //return to start
    int numInRange = 0;
    angle.push_back(0.0)
    while (numInRange < 10) {
        angle.at(0) = sensors.readAngle(motorNum);
        motors.setMotor(motorNum, kp * (centerVal - angle.at(0)));
        if (fabs(centerVal - angle.at(0)) < 0.01) {
            numInRange++;
        } else {
            numInRange = 0;
        }
        sleep_ms(dt*1000);
    }
    
    for (int i = 0; i < V.size() - 1; i++) {
        float V_bat = sensors.readBatteryVoltage();
        angle.push_back(sensors.readAngle(motorNum));
        if (V.at(i+1) > V_bat) { // constrain to battery voltage
            V.at(i+1) = V_bat;
        } else if (V.at(i+1) < -V_bat) {
            V.at(i+1) = -V_bat;
        } 
        motors.setMotor(motorNum, V.at(i+1) / V_bat); // shift i+1 because causality. So V[i] affects angle[i]
        if (fabs(centerVal - angle.at(i)) > maxDisplacement) { // if out of range
            V.erase(V.begin() + i, V.end());
            angle.reserve(0);
            return -1;
        }
        sleep_ms(dt*1000);
    }
    V.pop_back(); //causality shift
    return 0;
}


void sendRun(std::vector<float> &V, std::vector<float> &angle, int motorNum, float dt) {
    std::cout << "Time (s), Voltage (V), Angle (rad)\n";
    for (int i = 0; i < V.size(); i++) { // CSV format is easy
        std::cout << dt * i << "," << V.at(i) << "," << angle.at(i) << "\n";
        sleep_ms(2); //give the listener some time
    }
}

int main(int argc, char *argv) {
    int motorNum = 0;
    std::vector<float> V;
    std::vector<float> angle;
    float dt = 0.01;
    if (argc <= 1) {
        float amp_scale = 1.0;
    } else {
        float amp_scale = argv[1];
        if (argc <= 2) {
            float freq_scale = 1.0;
        } else {
            float freq_scale = argv[2];
            if (argc <= 3) {
                std::string filename = "";
            } else {
                std::string filename = argv[3];
    }}}

    std::cout<<"Run Amplitude "<<amp_scale<<" Frequency "<<freq_scale<<" dt "<<dt<<" motorNum "<<motorNum<<"\n";
    int ret = checkMotorPerformance(motorNum, dt, V, angle, freq_scale, amp_scale);
    if (ret == -1) {
        std::cout<<"Test went out of range and was terminated\n";
    } else if (ret > 0) {
        return ret;
    }
    if (filename.length() > 0) {
        ret = saveRun(filename, V, angle, motorNum, dt);
        if (ret > 0) {
            std::cout<<"Data save failed\n";
        }
    }
}
