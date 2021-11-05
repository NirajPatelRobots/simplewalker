# -*- coding: utf-8 -*-
"""
TODO:
    better loop timing

Created Jun 2021
@author: Niraj
"""

import motorControl
import sensorReader
import numpy as np
import time
from os.path import sep


def checkMotorPerformance(motorNum, dt, filename = None, frequency_scale = 1, amplitude_scale = 1):
    """test the motor by setting voltages and sensing position.
    motorNum 0 to 3 is which motor to read from and send to
    dt is the time [seconds]
    filename is file name to save the data to.
    frequency_scale and amplitude_scale scale the input voltage waveform"""
    centerVal = np.pi/2 # rad
    kp = 0.002
    maxDisplacement = 0.45 * np.pi # rad
    
    motors = motorControl.MotorController()
    sensors = sensorReader.SensorReader()
    V = excitationVoltage(frequency_scale*dt, amplitude_scale)
    N = np.size(V)
    angle = np.empty(N-1)
    
    # return to start
    numInRange = 0
    while numInRange < 10:
        angle[0] = sensors.readAngles()[motorNum]
        V_set = kp * (centerVal - angle[0])
        motors.setMotor(V_set, motorNum)
        if np.abs(centerVal - angle[0]) < 0.01:
            numInRange += 1
        else:
            numInRange = 0
        time.sleep(dt)
    
    print("At start position,", N, "samples,", N*dt, "s")
    startTime = time.perf_counter()
    for i in range(1, N):
        V_bat = sensors.readBatteryVoltage()
        angle[i-1] = sensors.readAngles()[motorNum] # shift i-1 because causality. So V[i] affects angle[i]
        if V[i] > V_bat: # account for voltage clipping
            V[i] = V_bat
        elif V[i] < -V_bat:
            V[i] = -V_bat
        motors.setMotor(V[i] / V_bat, motorNum)
        if np.abs(angle[i-1] - centerVal) > maxDisplacement:
            angle = angle[:i]
            V = V[:i+1]
            print("Angle out of range, terminating test")
            break
        time.sleep(dt)
    V = V[:-1] # unused because of shift
    runTime = time.perf_counter() - startTime
    print("Test run time:", round(runTime, 2), "s, loop takes ", round(runTime/N, 2), "s, code takes", round(1000*(runTime/N - dt), 3), "ms)")
    
    if not filename is None:
        saveRun(filename, V, angle, motorNum, dt)
    return V, angle


def saveRun(filename, V, angle, motorNum, dt, test_type="motor"):
    with open("data" + sep + "m" +str(motorNum) + '_' + filename +".motortest", 'wb') as file:
        np.savez(file, V = V, angle = angle, dt=dt, test_type=test_type)

def loadRun(filename):
    try:
        with open("data" + sep + filename +".motortest", 'rb') as file:
            data = np.load(file, allow_pickle=True)
            V = data['V']
            angle = data['angle']
            return V, angle
    except:
        print("Load failed")
        return np.array([]), np.array([])

def excitationVoltage(frequency_scale, amplitude_scale):
    """return an array of voltages.
    The voltages are sent to the motor and used to determine parameters."""
    def _voltageLoop(maxChange, maxVal):
        """returns an array of voltages starting and ending at zero"""
        freq = maxChange / abs(maxVal)
        t = np.arange(0., np.pi/freq, 0.01)
        return maxVal * np.sin(freq*t)
    
    num_loops = 20
    num_square = 5
    freq = 1000. * frequency_scale
    amp = 1. * amplitude_scale
    
    length = int(200./freq)
    V = np.zeros((2*length))
    for i in range(num_loops):
        loop_amp = (i+1)/(num_loops+1) * amp * (-1. if i%2 == 1 else 1.)
        V = np.concatenate((V, _voltageLoop(freq, loop_amp)))
    V = np.concatenate((V, -V[::-1]))
    for i in range(num_square):
        V = np.concatenate((V, amp*np.ones(length), -amp*np.ones(2*length), amp*np.ones(length)))
    V = np.concatenate((V, np.zeros((length))))
    return V
        

def main():
    """run the tests with a text UI"""
    print("Motor test. Choose one of:",
          "run ['frequency_scale', ['amplitude_scale', ['filename']]]",
          "motornum number (0: R motor 1, 1: R motor 2, 2: L motor 1, 3: L motor 2)",
          "save [filename]",
          "code", sep = "\n ")
    freq_scale = 1.
    amp_scale = 1.
    motorNum = 0
    V = np.array([])
    angle = np.array([])
    dt = 0.03 # seconds
    filename = None
    
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
            V, angle = checkMotorPerformance(motorNum, dt, filename, freq_scale, amp_scale)
        elif command == "motornum":
            try:
                motorNum = int(args[1])
            except:
                print("Invalid motor number")
        elif command == "save" and len(args) > 1:
            saveRun(args[1], V, angle, motorNum, dt)
        elif command == "dt" and len(args) > 1:
            dt = float(args[1])
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
