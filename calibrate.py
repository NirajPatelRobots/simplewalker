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
from os.path import exists


def checkMotorPerformance(motorNum, filename = None, frequency_scale = 1, amplitude_scale = 1):
    """test the motor by setting voltages and sensing position.
    motorNum 0 to 3 is which motor to read from and send to
    filename is file name to save the data to.
    frequency_scale and amplitude_scale scale the input voltage waveform"""
    # TODO
    motors = motorControl.MotorController(frequency_scale, amplitude_scale)
    sensors = sensorReader.SensorReader()
    V = excitationVoltage()
    N = np.size(V)
    angle = np.empty(N)
    
    # return to start
    targetVal = np.pi/6 # rad
    kp = 2.
    numInRange = 0
    while numInRange < 10:
        angle[0] = sensors.readAngles()[motorNum]
        V[0] = kp * (targetVal - angle[0])
        
        
        # TODO timing
    
    for i in range(N):
        # set voltage
        motors.setMotor(V[i], motorNum)
        # sense position
        angle[i] = sensors.readAngles()[motorNum]
        # if angle out of range, abort
        # TODO timing
        
    if not filename is None:
        with open(filename+".motortest", 'w') as file:
            np.save(file, np.vstack(V.reshape(1,-1), angle.reshape(1,-1)))
    return V, angle

def loadRun(filename):
    try:
        with open(filename+".motortest", 'w') as file:
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
    for i in range(len(V)):
        velocity = np.concatenate(np.array([0], np.diff(angle[i]))) / dt
        accel = np.concatenate(np.array([0], np.diff(velocity))) / dt
        ang_f = filterData(angle)
        vel_f = filterData(velocity)
        acc_f = filterData(accel)
        V_f = filterData(V[i])
    
    # use Linear Least Squares regression to determine parameters
    X = np.hstack(vel_f.reshape(-1,1), V_f.reshape(-1,1)) #independent variable data
    startTime = time.perf_counter()
    paramArr = np.inv(X.T @ X) @ X.T @ acc_f
    print("Regression took", time.perf_counter() - startTime, "s")
    params = {"omega": paramArr[0], "V": paramArr[1]}
    print("Parameters:", params)

    # graph voltage, angle, velocity, and acceleration filtered and unfiltered over time
    if len(V) == 1:
        t = np.linspace(0., np.size(angle[0])*dt, num=np.size(angle[0]))
        plt.figure(1)
        plt.clf()
        plt.subplot(411)
        plt.ylabel("Driving Voltage (V)")
        plt.plot(t, V[0], t, V_f)
        plt.legend("Raw", "Filtered")
        plt.subplot(412)
        plt.ylabel("Angle [rad]")
        plt.plot(t, angle[0], t, ang_f)
        plt.subplot(413)
        plt.ylabel("Velocity [rad/s]")
        plt.plot(t, velocity, t, vel_f)
        plt.subplot(414)
        plt.ylabel("Acceleration [rad/s^2]")
        plt.plot(t, accel, t, acc_f)
    
    # TODO 3D graph acceleration against voltage and speed
    
    # TODO 3D graph acceleration error against voltage and speed
    accel_error = acc_f - params["omega"] * vel_f - params["V"] * V_f
    
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
    filt_freq = 1. / 4 # 1 / number of samples
    filtered = signal.lfilter([filt_freq], [1., filt_freq], data)
    return filtered


def excitationVoltage(frequency_scale, amplitude_scale):
    """return an array of voltages.
    The voltages are sent to the motor and used to determine parameters."""
    
    num_loops = 20
    base_deltaV = 1.
    base_amp = 1.
    
    V = np.zeros((100))
    for i in range(num_loops):
        V = np.concatenate((V, (-1**i)*_voltageLoop(base_deltaV * frequency_scale,
                                                    (i+1)/(num_loops+1) * base_amp * amplitude_scale)))
    V = np.concatenate((V, V[:,:,-1]))
    # TODO maybe add a square wave
    

def _voltageLoop(maxChange, maxVal):
    """returns an array of voltages starting and ending at zero"""
    # TODO
    
    
def saveParams(params, filename = "motorparams"):
    with open(filename+".motorparams", "wb") as file:
        pickle.dump(params, file)
    
def loadParams(filename = "motorparams"):
    """static method that returns a MotionController_data object saved at filename"""
    if exists(filename+".motorparams"):
        with open(filename+".motorparams", "rb") as file:
            return pickle.load(file)
    else:
        print("Couldn't load parameters")
        return None
        

def main():
    """run the tests and analysis with a text UI"""
    print("Motor test and calibration. Choose one of:\n",
          "run ['frequency_scale', ['amplitude_scale', ['filename']]] \n",
          "examine ['filename'|all]\n",
          "saveparams ['filename']\n",
          "loadparams ['filename']")
    freq_scale = 1
    amp_scale = 1
    filename = None
    V = np.array([])
    angle = np.array([])
    dt = 0.05 # seconds
    model = []
    params = {}
    
    while True:
        args = input(">>> ").split()
        command = args[0] if len(args) > 0 else "" #command is first, the rest of the args are available if needed
        if command == "":
            continue
        command = command.strip().lower()
        if command == "run":
            if len(args) > 1:
                freq_scale = args[1]
                if len(args) > 2:
                    amp_scale = args[2]
                    if len(args) > 3:
                        filename = args[3]
            print("run", freq_scale, amp_scale, filename)
            # V, angle = checkMotorPerformance(filename, frequency_scale, amplitude_scale)
        elif command == "examine":
            if len(args) > 1:
                if args[1] == "all":
                    pass #TODO
                else:
                    V, angle = loadRun(args[1])
            print("Examine", V, angle, model)
            #params = examineMotor(V, angle, dt, model = model)
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
        elif command == "exit":
            break


if __name__ == "__main__":
    main()
