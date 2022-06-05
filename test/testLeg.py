# -*- coding: utf-8 -*-
"""
TODO:
    better loop timing
    deal with gravity and hip outward angle

Created Jun 2021
@author: Niraj
"""

import motorControl
import sensorReader
import numpy as np
import time
import testMotor
from os.path import sep


def checkLegPerformance(motorNum, dt, filename = None, frequency_scale = 1, amplitude_scale = 1):
    """test the motor by setting voltages and sensing position.
    motorNum is a list of two integers 0 to 3 for which [hip, knee] motor to read from and send to
    dt is the time [seconds]
    filename is file name to save the data to.
    frequency_scale and amplitude_scale scale the input voltage waveform"""
    centerVal = np.pi/2 # rad
    kp = 0.002
    maxDisplacement = 0.45 * np.pi # rad
    variations = 4 #how many variations of the held angle
    held_spread = np.pi/4 # how far from centerVal the variations of the held angle go
    
    motors = motorControl.MotorController()
    sensors = sensorReader.SensorReader()
    waveform = testMotor.excitationVoltage(frequency_scale*dt, amplitude_scale)
    N = np.size(waveform) - 1
    hip_angle = np.empty(N*2*variations); V_hip = np.empty(N*2*variations)
    knee_angle = np.empty(N*2*variations); V_knee = np.empty(N*2*variations)
    hipNum = motorNum[0]; kneeNum = motorNum[1]
    runTime = 0.
    i = 0
    
    for active, held in zip(motorNum, motorNum[::-1]):
        if active == hipNum:
            V_active = V_hip
            V_held = V_knee
        else:
            V_active = V_knee
            V_held = V_hip
        for variation in range(variations):
            # return to start
            held_target = centerVal + held_spread * (2 * variation/variations - 1)
            numInRange = 0
            while numInRange < 10:
                angles = sensors.readAngles()
                angle_active = angles[active]
                angle_held = angles[held]
                motors.setMotor(kp * (centerVal - angle_active), angles[active])
                motors.setMotor(kp * (held_target - angle_held), angles[held])
                if np.abs(centerVal - angle_active) < 0.01 and np.abs(held_target - angle_held) < 0.01:
                    numInRange += 1
                    #TODO include held angle voltage bias
                else:
                    numInRange = 0
                time.sleep(dt)
            motors.setMotor(0.0, held)
            
            print("Variation", variation, ",", N, "samples,", N*dt, "s")
            startTime = time.perf_counter()
            for j in range(N):
                V_bat = sensors.readBatteryVoltage()
                angles = sensors.readAngles()
                hip_angle[i] = angles[hipNum]
                knee_angle[i] = angles[kneeNum]
                if waveform[j] > V_bat: # account for voltage clipping
                    V_active[i] = V_bat
                elif V_active[i] < -V_bat:
                    V_active[i] = -V_bat
                else:
                    V_active[i] = waveform[j]
                V_held[i] = 0.0 #TODO include held angle voltage bias
                motors.setMotor(waveform[j+1] / V_bat, active) # j+1 so V[i] affects angle[i]
                if np.abs(centerVal - angle_active) > maxDisplacement or np.abs(held_target - angle_held) > maxDisplacement:
                    print("Angle out of range, terminating test")
                    break
                time.sleep(dt)
                i += 1
            runTime += time.perf_counter() - startTime
    if i < N*2*variations:
        hip_angle = hip_angle[:i]; V_hip = V_hip[:i]
        knee_angle = knee_angle[:i]; V_knee = V_knee[:i]
    print("Test run time:", round(runTime, 2), "("+str(round(i*dt, 4))+" expected)")
    
    testdata = {"hip_angle":hip_angle, "knee_angle":knee_angle, "V_hip":V_hip, "V_knee":V_knee,
                "motorNum":motorNum, "hip_out_angle":np.pi/2, "dt":dt}
    if not filename is None:
        tag = "l" + str(motorNum[0]) + str(motorNum[1]) + '_'
        saveRun(tag+filename, **testdata, test_type="leg")
    return testdata

def saveRun(filename, **kwargs):
    with open("data" + sep + filename +".motortest", 'wb') as file:
        np.savez(file, test_type="leg", **kwargs)
        

def main():
    """run the tests with a text UI"""
    print("Motor test. Choose one of:",
          "run ['frequency_scale', ['amplitude_scale', ['filename']]]",
          "motornum number (0: R motor 1, 1: R motor 2, 2: L motor 1, 3: L motor 2)",
          "save [filename]",
          "code", sep = "\n ")
    freq_scale = 1.
    amp_scale = 1.
    motorNum = [0, 1]
    dt = 0.03 # seconds
    
    while True:
        args = input(">>> ").split()
        command = args[0] if len(args) > 0 else "" #command is first, the rest of the args are available if needed
        command = command.strip().lower()
        if command == "":
            continue
        elif command == "run":
            if len(args) > 1:
                freq_scale = float(args[1])
                if len(args) > 2:
                    amp_scale = float(args[2])
                    if len(args) > 3:
                        filename = args[3]
                    else:
                        filename = None
            testdata = checkLegPerformance(motorNum, dt, filename, freq_scale, amp_scale)
        elif command == "motornum":
            try:
                motorNum = [int(args[1]), int(args[2])]
            except:
                print("Invalid motor numbers")
        elif command == "save" and len(args) > 1:
            tag = "l" + str(motorNum[0]) + str(motorNum[1]) + '_'
            saveRun(tag+args[1], test_type="leg", **testdata)
        elif command.startswith("exit"):
            break
        elif command == "code":
            while not command == "exit":
                command = input(">>> ")
                try:
                    print(eval(command))
                except Exception as e:
                    print(e)


if __name__ == "__main__":
    main()
