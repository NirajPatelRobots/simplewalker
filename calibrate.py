# -*- coding: utf-8 -*-
"""
TODO:
    assemble motors and electronics

Created Jun 2021
@author: Niraj
"""

import motorControl
import sensorReader
import numpy as np
from scipy import signal
import time
import matplotlib.pyplot as plt
import pickle
from os import listdir, getcwd
from os.path import exists, sep, isfile, join


def checkMotorPerformance(motorNum, dt, filename = None, frequency_scale = 1, amplitude_scale = 1):
    """test the motor by setting voltages and sensing position.
    motorNum 0 to 3 is which motor to read from and send to
    dt is the time [seconds]
    filename is file name to save the data to.
    frequency_scale and amplitude_scale scale the input voltage waveform"""
    centerVal = np.pi/2 # rad
    kp = 2.
    maxDisplacement = 0.45 * np.pi # rad
    
    motors = motorControl.MotorController()
    sensors = sensorReader.SensorReader()
    V = excitationVoltage(frequency_scale/dt, amplitude_scale)
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
    
    startTime = time.perf_counter()
    for i in range(1, N):
        V_bat = sensors.readBatteryVoltage()
        angle[i-1] = sensors.readAngles()[motorNum] # shift i-1 because causality. So V[i] affects angle[i]
        if V[i] > V_bat:
            V[i] = V_bat
        motors.setMotor(V[i] / V_bat, motorNum)
        if np.abs(angle[i] - centerVal) > maxDisplacement:
            angle = angle[:i]
            V = V[:i+1]
            print("Angle out of range, terminating test")
            break
        time.sleep(dt)
    V = V[:-1] # unused because of shift
    print("Test run time:", time.perf_counter() - startTime, "(expected", str(N*dt)+')')
        
    if not filename is None:
        with open("data" + sep + "m" +str(motorNum) + '_' + filename +".motortest", 'w') as file:
            np.save(file, np.vstack(V.reshape(1,-1), angle.reshape(1,-1)))
    return V, angle

def loadRun(filename):
    try:
        with open("data" + sep + filename +".motortest", 'w') as file:
            a = np.load(file)
            V = a[0,:]
            angle = a[1,:]
            return V, angle
    except:
        print("Load failed")
        return np.array([]), np.array([])


def examineMotor(V, angle, dt, model = None, params = None):
    """return the model parameters (dict) of a motor.
    V is 1D array of driving voltage, angle is same size array of sensed angle.
    V and angle can be one array or lists of arrays.
    params is optional existing parameters which may be edited.
    Motor model d2(angle)/dt2 = param[V] * V + param[omega] * d(angle)/dt
    model is a list of strings naming additional model dynamics"""
    if params is None:
        params = {}
    if model is None:
        model = []
    
    if not isinstance(V, list):
        V = [V]
        angle = [angle]
    velocity = np.empty((0))
    accel = np.empty((0))
    ang_f = np.empty((0))
    vel_f = np.empty((0))
    acc_f = np.empty((0))
    V_f = np.empty((0))
    for i in range(len(V)):
        velocity = np.concatenate((velocity, [0], np.diff(angle[i]))) / dt
        accel = np.concatenate((accel, [0], np.diff(velocity))) / dt
        ang_f = np.concatenate((ang_f, filterData(angle[i])))
        vel_f = np.concatenate((vel_f, filterData(velocity)))
        acc_f = np.concatenate((acc_f, filterData(accel)))
        V_f = np.concatenate((V_f, filterData(V[i])))
    
    # use Linear Least Squares regression to determine parameters
    X = np.hstack((vel_f.reshape(-1,1), V_f.reshape(-1,1))) #independent variable data
    startTime = time.perf_counter()
    paramArr = np.linalg.inv(X.T @ X) @ X.T @ acc_f
    print("Regression took", time.perf_counter() - startTime, "s")
    params = {"omega": paramArr[0], "V": paramArr[1]}
    print("Parameters:", params)

    # graph voltage, angle, velocity, and acceleration filtered and unfiltered over time
    t = np.linspace(0., np.size(angle[0])*dt, num=np.size(angle[0]))
    plt.figure(1)
    plt.clf()
    plt.subplot(411)
    plt.suptitle("Raw and Filtered Data")
    plt.ylabel("Driving Voltage (V)")
    lines = plt.plot(t, V[0], t, V_f)
    plt.legend(lines, ["Raw", "Filtered"])
    plt.grid()
    plt.subplot(412)
    plt.ylabel("Angle [rad]")
    plt.plot(t, angle[0], t, ang_f)
    plt.grid()
    plt.subplot(413)
    plt.ylabel("Velocity [rad/s]")
    plt.plot(t, velocity, t, vel_f)
    plt.grid()
    plt.subplot(414)
    plt.ylabel("Acceleration [rad/s^2]")
    plt.xlabel("Time [s]")
    plt.plot(t, accel, t, acc_f)
    plt.grid()
    
    # 3D graph acceleration against voltage and speed
    fig = plt.figure(2)
    plt.clf()
    ax = fig.add_subplot(projection = '3d')
    ax.scatter(V, vel_f, zs=acc_f)
    plt.title("Acceleration")
    plt.zlabel("Acceleration [m/s^2]")
    plt.xlabel("Voltage [V]")
    plt.ylabel("Velocity [m/s]")
    
    # 3D graph acceleration error against voltage and speed
    accel_error = acc_f - params["omega"] * vel_f - params["V"] * V_f
    fig = plt.figure(3)
    plt.clf()
    ax = fig.add_subplot(projection = '3d')
    ax.scatter(V, vel_f, zs=accel_error)
    plt.title("Acceleration Error")
    plt.zlabel("Acceleration Error [m/s^2]")
    plt.xlabel("Voltage [V]")
    plt.ylabel("Velocity [m/s]")
    
    # investigate torque ripple: plot acceleration error against angle
    plt.figure(4)
    plt.clf()
    plt.title("Torque Ripple")
    plt.xlabel("Angle [rad]")
    plt.ylabel("Acceleration error [rad/s^2]")
    plt.plot(ang_f, accel_error)
    plt.show()
    
    return params
    

def filterData(data):
    """filters data."""
    # first order lowpass filter signals
    filt_time = 4. # number of samples
    filtered = signal.lfilter([1., (filt_time - 1.)], [filt_time], data)
    return filtered


def excitationVoltage(frequency_scale, amplitude_scale):
    """return an array of voltages.
    The voltages are sent to the motor and used to determine parameters."""
    def _voltageLoop(maxChange, maxVal):
        """returns an array of voltages starting and ending at zero"""
        freq = maxChange / maxVal
        t = np.arange(0., np.pi/freq, 0.01)
        return maxVal * np.sin(freq*t)
    
    num_loops = 20
    num_square = 5
    freq = 1. * frequency_scale
    amp = 1. * amplitude_scale
    
    length = int(100./freq)
    V = np.zeros((length))
    for i in range(num_loops):
        V = np.concatenate((V, ((-1)**i)*_voltageLoop(freq, (i+1)/(num_loops+1) * amp)))
    V = np.concatenate((V, -V[::-1]))
    for i in range(num_square):
        V = np.concatenate((V, amp*np.ones(length), -amp*np.ones(2*length), amp*np.ones(length)))
    V = np.concatenate((V, np.zeros((length))))
    return V

    
def saveParams(params, filename = "motorparams"):
    with open("settings"+ sep + filename +".motorparams", "wb") as file:
        pickle.dump(params, file)
    
def loadParams(filename = "motorparams"):
    """static method that returns a MotionController_data object saved at filename"""
    if exists("settings"+ sep + filename +".motorparams"):
        with open(filename+".motorparams", "rb") as file:
            return pickle.load(file)
    else:
        print("Couldn't load parameters")
        return None
        

def main():
    """run the tests and analysis with a text UI"""
    print("Motor test and calibration. Choose one of:",
          "run ['frequency_scale', ['amplitude_scale', ['filename']]]",
          "examine ['filename'|all <number>]",
          "motornum number (0: right motor 1) (1: right motor 2) (2: left motor 1) (3: left motor 2)",
          "saveparams ['filename']",
          "loadparams <'filename'>",
          "code", sep = "\n ")
    freq_scale = 1.
    amp_scale = 1.
    motornum = 0
    filename = None
    V = np.array([])
    angle = np.array([])
    dt = 0.05 # seconds
    model = []
    params = {}
    directory_path = getcwd()
    
    while True:
        args = input(">>> ").split()
        command = args[0] if len(args) > 0 else "" #command is first, the rest of the args are available if needed
        if command == "":
            continue
        command = command.strip().lower()
        if command == "run":
            if len(args) > 1:
                freq_scale = float(args[1])
                if len(args) > 2:
                    amp_scale = float(args[2])
                    if len(args) > 3:
                        filename = args[3]
            V, angle = checkMotorPerformance(motornum, dt, filename, freq_scale, amp_scale)
            filename = None
        elif command == "examine":
            if len(args) > 1:
                if args[1] == "all" and len(args) > 2:
                    filenames = [f for f in listdir(directory_path) if isfile(join(directory_path,f)) and f.startswith("m"+str(motornum))]
                    V = [None] * len(filenames)
                    angle = [None] * len(filenames)
                    for i, f in enumerate(filenames):
                        V[i], angle[i] = loadRun(f)
                else:
                    V, angle = loadRun(args[1])
            params = examineMotor(V, angle, dt, model)
        elif command == "motornum":
            try:
                motornum = int(args[1])
            except:
                print("Invalid motor number")
        elif command == "saveparams":
            if len(args) > 1:
                saveParams(params, args[1])
            else:
                saveParams(params)
        elif command == "loadparams":
            if len(args) > 1:
                ret = loadParams(args[1])
            else:
                ret = loadParams()
            if not ret is None:
                params = ret
                print("Loaded parameters:", params)
        elif command == "code":
            while not command == "exit":
                command = input(">>> ")
                try:
                    print(eval(command))
                except Exception as e:
                    print(e)
        elif command == "exit":
            break


if __name__ == "__main__":
    main()
