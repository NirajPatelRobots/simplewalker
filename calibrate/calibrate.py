# -*- coding: utf-8 -*-
"""
TODO:
    digital filter implementations
    delay in acceleration for causality? filter-related?
    better loop timing
    clean up and organize
        organized model definition, remove param_array indexing

Created Jun 2021
@author: Niraj
"""

import numpy as np
from scipy import signal
import time
import matplotlib.pyplot as plt
from os import listdir, getcwd
from os.path import exists, sep, isfile, join


def examineMotor(testdata, model = None, params = None):
    """return the model parameters (dict) of a motor.
    V is 1D array of driving voltage, angle is same size array of sensed angle.
    V and angle can be one array or lists of arrays.
    params is optional existing parameters which may be edited.
    Motor model d2(angle)/dt2 = param[V] * V + param[omega] * d(angle)/dt
    model is a list of strings naming additional model dynamics"""

    def filter_and_derivative(testdata):
        results = {m: np.empty(0) for m in ["V_f", "angle_f", "vel", "acc", "vel_f", "acc_f"]}
        for this_data in testdata:
            this_velocity = np.diff(this_data["angle"]) / dt
            this_acc = np.diff(this_velocity) / dt
            results["vel"] = np.concatenate((results["vel"], this_velocity, [0]))
            results["acc"] = np.concatenate((results["acc"], this_acc, [0, 0]))
            results["vel_f"] = np.concatenate((results["vel_f"], filter_data(this_velocity), [0]))
            results["acc_f"] = np.concatenate((results["acc_f"], filter_data(this_acc), [0, 0]))
            for m in ["V", "angle"]:
                results[m+"_f"] = np.concatenate((results[m+"_f"], filter_data(this_data[m])))
        return results

    def make_independent_variable(results, model):
        """make array used for the independent variable in regression"""
        V_f = results["V_f"]
        vel_f = results["vel_f"]
        X = np.hstack((V_f.reshape(-1,1), vel_f.reshape(-1,1)))
        for mod in model:
            if mod == "const_fric":
                static_direction = np.where(np.sign(vel_f) == np.sign(V_f), np.sign(vel_f), 0)
                X = np.hstack((X, ((1 - slowness_factor_continuous(vel_f)) * static_direction).reshape(-1,1)))
            if mod == "static_fric":
                X = np.hstack((X, (slowness_factor_continuous(vel_f) * V_f).reshape(-1,1)))
            if mod == "offset":
                X = np.hstack((X, (1 - slowness_factor_continuous(vel_f)).reshape(-1,1)))
            elif mod == "Vsq":
                X = np.hstack((X, (np.abs(V_f) * V_f).reshape(-1,1)))
            elif mod == "knee":
                X = np.hstack((X, -np.sin(results["angle_f"]).reshape(-1,1)))
            elif mod == "hip":
                X = np.hstack((X, -np.sin(results["angle_f"]).reshape(-1,1)))
                X = np.hstack((X, np.sin(results["knee_angle_f"]).reshape(-1,1)))
        return X
    
    def determine_params(X, acc_f, results, model, remove_outliers = False):
        paramArr = np.linalg.inv(X.T @ X) @ X.T @ acc_f
        if remove_outliers:
            params, accel_predic = assign_parameters(paramArr, {}, results, model)
            accel_error = acc_f - accel_predic
            std_dev = (np.sum(accel_error**2)/results["N"])**.5
            results["outlier_thresh"] = 3 * std_dev
            outliers = np.abs(accel_error) > results["outlier_thresh"]
            acc_f_clean = np.where(outliers, 0., acc_f)
            X_clean = np.where(outliers.reshape((-1,1)) * np.ones((1,len(params))), 0., X)
            print("removed", np.sum(outliers), "outliers out of", results["N"],
                  "(Error >", round(results["outlier_thresh"], 1), "rad/s^2)")
            return determine_params(X_clean, acc_f_clean, results, model)
        else:
            return paramArr
        
    def assign_parameters(param_array, params, results, model):
        """assigns parameters based on regression outputs and model"""
        params["V"] = param_array[0]
        params["omega"] = param_array[1]
        accel_predic = params["omega"] * results["vel_f"] + params["V"] * results["V_f"]
        i = 0
        for mod in model:
            if mod == "const_fric":
                params["c"] = param_array[2+i]
                slowness = slowness_factor_continuous(results["vel_f"])
                static_direction = np.where(np.sign(results["vel_f"]) == np.sign(results["V_f"]), np.sign(results["vel_f"]), 0)
                accel_predic = accel_predic + params["c"] * (1 - slowness) * static_direction
                i += 1
            if mod == "static_fric":
                params["static_fric"] = param_array[2+i]
                slowness = slowness_factor_continuous(results["vel_f"])
                accel_predic = accel_predic + params["static_fric"] * slowness * results["V_f"]
                i += 1
            if mod == "offset":
                params["offset"] = param_array[2+i]
                accel_predic = accel_predic + (1 - slowness_factor_continuous(results["vel_f"])) * params["offset"]
                i += 1
            elif mod == "Vsq":
                params["Vsq"] = param_array[2+i]
                accel_predic = accel_predic + params["Vsq"] * (np.abs(results["V_f"]) * results["V_f"])
                i += 1
            elif mod == "knee":
                params["leg_weight_2"] = param_array[2+i]
                accel_predic = accel_predic + params["leg_weight_2"] * -np.sin(results["angle_f"]).reshape(-1,1)
                i += 1
            elif mod == "hip":
                params["leg_weight_1"] = param_array[2+i]
                params["leg_weight_12"] = param_array[3+i]
                accel_predic = accel_predic    \
                    + params["leg_weight_1"] * -np.sin(results["angle_f"]).reshape(-1,1) \
                    + params["leg_weight_12"] * np.sin(results["knee_angle_f"]).reshape(-1,1)
                i += 2
        return params, accel_predic
    
    dt = testdata[0]["dt"]
    if params is None:
        params = {}
    if model is None:
        model = []
    results = filter_and_derivative(testdata)
    results["N"] = np.size(results["V_f"])
    results["t"] = np.linspace(0., results["N"]*dt, num=results["N"])
    
    # use Linear Least Squares regression to determine parameters
    X = make_independent_variable(results, model)
    startTime = time.perf_counter()
    paramArr = determine_params(X, results["acc_f"], results, model, remove_outliers=True)
    print("Parameter determination took", time.perf_counter() - startTime, "s")
    params, results["accel_predic"] = assign_parameters(paramArr, params, results, model)
    return params, results

def graphMotorResults(testdata, params, results):
    print("Parameters:", params)
    accel_error = results["acc_f"] - results["accel_predic"]
    avg_error = np.sum(np.abs(accel_error)) / results["N"]
    print("Average error =", round(avg_error, 3), "rad/s^2, Average acceleration", 
          round(np.sum(np.abs(results["acc_f"] )) / results["N"], 3), "rad/s^2")
    print("Regression R^2:", round(1 - np.sum(accel_error**2) / np.sum(results["acc_f"]**2), 3))

    # graph voltage, angle, velocity, and acceleration filtered and unfiltered over time
    if len(testdata) == 1:
        plt.figure(1)
        plt.clf()
        ax = plt.subplot(411)
        plt.suptitle("Raw and Filtered Data")
        plt.ylabel("Driving Voltage (V)")
        lines = plt.plot(results["t"], testdata[0]["V"], results["t"], results["V_f"])
        plt.legend(lines, ["Raw", "Filtered"])
        plt.grid()
        plt.subplot(412, sharex=ax)
        plt.ylabel("Angle [rad]")
        plt.plot(results["t"], testdata[0]["angle"], results["t"], results["angle_f"])
        plt.grid()
        plt.subplot(413, sharex=ax)
        plt.ylabel("Velocity [rad/s]")
        plt.plot(results["t"], results["vel"], '.', results["t"], results["vel_f"])
        plt.grid()
        ax = plt.subplot(414, sharex=ax)
        plt.ylabel("Acceleration [rad/s^2]")
        plt.xlabel("Time [s]")
        lines = plt.plot(results["t"], results["acc"], '.', results["t"], results["acc_f"],
                         results["t"], results["accel_predic"])
        plt.legend(lines, ["Raw", "Filtered", "Predicted"])
        ax.set_ylim(np.min(results["acc_f"]) * 1.1, np.max(results["acc_f"]) * 1.1)
        plt.grid()
    
    # 3D graph acceleration against voltage and speed
    fig = plt.figure(2)
    plt.clf()
    ax = fig.add_subplot(projection = '3d')
    ax.scatter(results["V_f"], results["vel_f"], zs=results["acc_f"], c = results["acc_f"])
    plt.title("Acceleration")
    ax.set_zlabel("Acceleration [rad/s^2]")
    plt.xlabel("Voltage [V]")
    plt.ylabel("Velocity [rad/s]")
    
    # 3D graph acceleration error against voltage and speed
    fig = plt.figure(3)
    plt.clf()
    ax = fig.add_subplot(projection = '3d')
    ax.scatter(results["V_f"], results["vel_f"], zs=np.where(np.abs(accel_error) < results["outlier_thresh"], accel_error, np.NaN), c=accel_error)
    plt.title("Acceleration Error")
    ax.set_zlabel("Acceleration Error [rad/s^2]")
    plt.xlabel("Voltage [V]")
    plt.ylabel("Velocity [rad/s]")
    
    # investigate torque ripple: plot acceleration error against angle
    plt.figure(4)
    plt.clf()
    plt.plot(results["angle_f"], accel_error, '.')
    plt.title("Torque Ripple")
    plt.xlabel("Angle [rad]")
    plt.ylabel("Acceleration error [rad/s^2]")
    plt.grid()

    plt.figure(5)
    plt.clf()
    plt.plot(results["t"], np.abs(results["vel_f"]), results["t"],
             slowness_factor_continuous(results["vel_f"]) * np.max(results["vel_f"]) * 0.5)
    plt.title("Speed and Slowness Factor")
    plt.xlabel("time [s]")
    plt.ylabel("Speed [rad/s] and Slowness [arb. units]")
    plt.grid()

    plt.figure(6)
    plt.clf()
    lines = plt.plot(results["t"], results["acc_f"], results["t"], results["accel_predic"])
    plt.title("Acceleration Prediction")
    plt.legend(lines, ["Filtered", "Predicted"])
    plt.xlabel("time [s]")
    plt.ylabel("Acceleration [rad/s^2]")
    plt.grid()
    
    plt.show()
    time.sleep(0.3)


def loadRun(filename):
    if not "motortest" in filename:
        filename = filename + ".motortest"
    try:
        with open("data" + sep + filename, 'rb') as file:
            data = np.load(file, allow_pickle=True)
            return dict(data)
    except:
        print("Load failed:", filename)

def filter_data(data, filt_cutoff_samples = 12):
    n_suspicious_filtered = filt_cutoff_samples * 6
    padded_data = np.concatenate((data[0] * np.ones(n_suspicious_filtered), data))
    filtered = signal.sosfilt(signal.butter(4, 1/filt_cutoff_samples, 'lowpass', output='sos'), padded_data)
    filtered = filtered[n_suspicious_filtered:]
    return filtered

# continuous number [0,1] where 1 means it's slow enough for static friction
def slowness_factor_continuous(vel, speed_thresh = 0.06):
    #return filter_data(np.where(np.abs(vel) < speed_thresh, (speed_thresh - np.abs(vel)) / speed_thresh, 0), 4)
    return np.clip(np.arctan(filter_data(np.abs(vel) < speed_thresh, 4) * 20 - 10) / np.pi + 0.5, 0, 1)
    
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
    testdata = []
    model = ['static_fric', 'const_fric', 'offset']
    params = {}
    directory_path = getcwd() + "/data"
    test_type = ""
    
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
                    testdata = []
                    for i, f in enumerate(filenames):
                        newdata = loadRun(f)
                        if newdata is None:
                            print("couldn't load", f)
                        else:
                            testdata.append(newdata)
                            if len(testdata) == 1:
                                dt = testdata[0]["dt"]
                                test_type = testdata[0]["test_type"]
                            else:
                                if not testdata[i]["dt"] == dt:
                                    print("mismatched log timestep was", dt, "but is", testdata[i]["dt"], "for", f)
                                if not testdata[i]["test_type"] == test_type:
                                    print("mismatched test_type", test_type, testdata[i]["test_type"], "for", f)
                else:
                    newdata = loadRun(args[1])
                    if newdata is None:
                        print("couldn't load", args[1])
                    else:
                        testdata = [loadRun(args[1])]
                        test_type = testdata[0]["test_type"]
            if len(testdata) > 0:
                if test_type == "motor":
                    params, results = examineMotor(testdata, model)
                    graphMotorResults(testdata, params, results)
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
