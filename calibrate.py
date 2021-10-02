# -*- coding: utf-8 -*-
"""
TODO:
    look into digital filter implementation
    delay in acceleration for causality? filter-related?
    better loop timing
    whole leg

Created Jun 2021
@author: Niraj
"""

import numpy as np
from scipy import signal
import time
import matplotlib.pyplot as plt
from os import listdir, getcwd
from os.path import exists, sep, isfile, join


def examineMotor(V, angle, dt, model = None, params = None):
    """return the model parameters (dict) of a motor.
    V is 1D array of driving voltage, angle is same size array of sensed angle.
    V and angle can be one array or lists of arrays.
    params is optional existing parameters which may be edited.
    Motor model d2(angle)/dt2 = param[V] * V + param[omega] * d(angle)/dt
    model is a list of strings naming additional model dynamics"""
    def make_independent_variable(V_f, vel_f, model):
        """make array used for the independent variable in regression"""
        X = np.hstack((V_f.reshape(-1,1), vel_f.reshape(-1,1)))
        for mod in model:
            if mod == "static_fric":
                static_direction = np.where(np.sign(vel_f) == np.sign(V_f), np.sign(vel_f), 0)
                X = np.hstack((X, static_direction.reshape(-1,1)))
            if mod == "Vsq":
                X = np.hstack((X, (np.abs(V_f) * V_f).reshape(-1,1)))
        return X
        
    def assign_parameters(param_array, params, model):
        """assigns parameters based on regression outputs and model"""
        params["V"] = param_array[0]
        params["omega"] = param_array[1]
        accel_predic = params["omega"] * vel_f + params["V"] * V_f
        for i, mod in enumerate(model):
            if mod == "static_fric":
                params["c"] = param_array[2+i]
                static_direction = np.where(np.sign(vel_f) == np.sign(V_f), np.sign(vel_f), 0)
                accel_predic = accel_predic + params["c"] * static_direction
            elif mod == "Vsq":
                params["Vsq"] = param_array[2+i]
                accel_predic = accel_predic + params["Vsq"] * (np.abs(V_f) * V_f)
        return params, accel_predic
    
    def determine_params(X, acc_f, model, outlier_thresh = None):
        paramArr = np.linalg.inv(X.T @ X) @ X.T @ acc_f
        if outlier_thresh is None:
            return paramArr
        else:
            params, accel_predic = assign_parameters(paramArr, {}, model)
            accel_error = acc_f - accel_predic
            X = np.copy(X)
            acc_f = np.copy(acc_f)
            numout = 0
            for i in range(len(acc_f)):
                if abs(accel_error[i]) > outlier_thresh:
                    acc_f[i] = 0
                    X[i,:] *= 0
                    numout += 1
            print("removed", numout,"outliers out of", len(acc_f))
            return determine_params(X, acc_f, model)
    
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
    V_f = np.empty((0))
    for i in range(len(V)):
        velocity = np.concatenate((velocity, [0], np.diff(angle[i]))) / dt
        accel = np.concatenate((accel, [0], np.diff(velocity))) / dt
        ang_f = np.concatenate((ang_f, filterData(angle[i])))
        V_f = np.concatenate((V_f, filterData(V[i])))
    vel_f = np.concatenate(([0], np.diff(ang_f))) / dt
    acc_f = np.concatenate(([0], np.diff(vel_f))) / dt
    t = np.linspace(0., np.size(V_f)*dt, num=np.size(V_f))
    
    # use Linear Least Squares regression to determine parameters
    X = make_independent_variable(V_f, vel_f, model)
    startTime = time.perf_counter()
    paramArr = determine_params(X, acc_f, model, outlier_thresh = 50)
    print("Regression took", time.perf_counter() - startTime, "s")
    params, accel_predic = assign_parameters(paramArr, params, model)
    print("Parameters:", params)
    accel_error = acc_f - accel_predic
    rms_error = np.sum(np.abs(accel_error)) / len(V_f)
    print("Average error =", rms_error, "rad/s^2")

    # graph voltage, angle, velocity, and acceleration filtered and unfiltered over time
    if len(V) == 1:
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
    ax.scatter(V_f, vel_f, zs=acc_f, c = acc_f)
    plt.title("Acceleration")
    ax.set_zlabel("Acceleration [rad/s^2]")
    plt.xlabel("Voltage [V]")
    plt.ylabel("Velocity [rad/s]")
    
    # 3D graph acceleration error against voltage and speed
    fig = plt.figure(3)
    plt.clf()
    ax = fig.add_subplot(projection = '3d')
    ax.scatter(V_f, vel_f, zs=accel_error, c=accel_error)
    plt.title("Acceleration Error")
    ax.set_zlabel("Acceleration Error [rad/s^2]")
    plt.xlabel("Voltage [V]")
    plt.ylabel("Velocity [rad/s]")
    
    # investigate torque ripple: plot acceleration error against angle
    plt.figure(4)
    plt.clf()
    plt.plot(ang_f, accel_error, '.')
    plt.title("Torque Ripple")
    plt.xlabel("Angle [rad]")
    plt.ylabel("Acceleration error [rad/s^2]")
    plt.grid()
    
    plt.figure(5)
    plt.clf()
    lines = plt.plot(t, acc_f, t, accel_predic)
    plt.title("Acceleration Prediction")
    plt.legend(lines, ["Filtered", "Predicted"])
    plt.xlabel("time [s]")
    plt.ylabel("Acceleration [rad/s^2]")
    plt.grid()
    
    plt.show()
    return params


def loadRun(filename):
    if not "motortest" in filename:
        filename = filename + ".motortest"
    try:
        with open("data" + sep + filename, 'rb') as file:
            data = np.load(file, allow_pickle=True)
            V = data['V']
            angle = data['angle']
            dt = data['dt']
            return V, angle, dt
    except:
        print("Load failed:", filename)
        return np.array([]), np.array([])

def filterData(data):
    """filters data."""
    # first order lowpass filter signals
    filt_time = 1.5 # number of samples
    b = [1., (filt_time - 1.)]
    a = [filt_time]
    zi = signal.lfilter_zi(b,a)
    filtered, _ = signal.lfilter(b, a, data, zi=zi*data[0])
    return filtered
    
def saveParams(params, filename = "new"):
    with open("settings"+ sep + filename +".learnedparams", "wb") as file:
        np.savez(file, **params)
    
def loadParams(filename = "motorparams"):
    if exists("settings"+ sep + filename +".learnedparams"):
        with open("settings"+ sep + filename+".learnedparams", "rb") as file:
            return dict(np.load(file))
    else:
        print("Couldn't load parameters")
        return None
        

def main():
    """run the tests and analysis with a text UI"""
    print("Motor test and calibration. Choose one of:",
          "examine ['filename'|all <number>]",
          "saveparams ['filename']",
          "loadparams <'filename'>",
          "model [[no] model_name]",
          "code", sep = "\n ")
    V = np.array([])
    angle = np.array([])
    dt = 0.03 # seconds
    model = [] # "static_fric"
    params = {}
    directory_path = getcwd() + "/data"
    
    while True:
        args = input(">>> ").split()
        command = args[0] if len(args) > 0 else "" #command is first, the rest of the args are available if needed
        command = command.strip().lower()
        if command == "":
            continue
        elif command == "examine":
            if len(args) > 1:
                if args[1] == "all" and len(args) > 2:
                    filenames = [f for f in listdir(directory_path) if isfile(join(directory_path,f)) and f.startswith(args[2])]
                    print("Loading", filenames)
                    V = [None] * len(filenames)
                    angle = [None] * len(filenames)
                    last_dt = dt
                    for i, f in enumerate(filenames):
                        V[i], angle[i], dt = loadRun(f)
                        if i > 0:
                            if not dt == last_dt:
                                print("mismatched log dt", last_dt, dt, "for", f)
                                V = V[:i]
                                angle = angle[:i]
                else:
                    V, angle, dt = loadRun(args[1])
            if len(V) > 0:
                params = examineMotor(V, angle, dt, model)
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
        elif command == "model":
            while len(args) > 1:
                if args[1] == "no":
                    model.remove(args[2])
                    args = [args[0]] + args[3:]
                else:
                    model.append(args[1])
                    args = [args[0]] + args[2:]
            print(model)
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
