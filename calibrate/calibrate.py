# -*- coding: utf-8 -*-
"""
TODO:
    more digital filter tuning (IIR vs FIR?)
    delay in acceleration for causality? filter-related?
    slowness:
        look at slowness factor (coulomb (and/or static?) fric too high)
        graph slowness factor and acceleration?
        some way to compare slowness functions for static and const friction? (sub-model)
    some way to deal with nonlinear parameters, like slowness threshold
    better model sticky stops. Spring that stores and releases energy?
    graph fourier transform of angle with filter response
    clean up and organize
        organized model definition
            Dict[model_name: (function(results: Dict) -> np.array)] ?
                ex "Vsq": (lambda results: return np.abs(results["V_f"]) * results["V_f"])
        file structure. New file for main() UI and graphing?
    graph individual variable contributions to model (use X matrix)
    think more about inductance (can model with 3rd derivative of angle)
    electrically isolate angle sensors to reduce noise?

Created Jun 2021
@author: Niraj
"""

import numpy as np
from scipy import signal, stats
import time
import matplotlib.pyplot as plt
from os import listdir, getcwd
from os.path import exists, sep, isfile, join
import sys

FILT_N = 50
FILT_ORDER = 4
FILT_FCN = 'butter'  # 'cheby'
FILT_RIPPLE = 0.5  # only applies to cheby filters


def examineMotor(testdata, model = None, params = None):
    """return the model parameters (dict) of a motor.
    V is 1D array of driving voltage, angle is same size array of sensed angle.
    V and angle can be one array or lists of arrays.
    params is optional existing parameters which may be edited.
    Motor model d2(angle)/dt2 = param[V] * V + param[omega] * d(angle)/dt
    model is a list of strings naming additional model dynamics"""

    def filter_and_derivative(testdata):
        results = {m: np.empty(0) for m in [m + f for m in ["V", "angle", "vel", "acc"] for f in ("", "_f")] + ["t"]}
        results["filter_period"] = testdata[0]["t"][FILT_N] - testdata[0]["t"][0]
        for this_data in testdata:
            clean_up_test_data(this_data, results)
            this_data["vel"] = np.diff(this_data["angle"], append=0) / np.diff(this_data["t"], append=0)
            this_data["acc"] = np.diff(this_data["vel"], append=0) / np.diff(this_data["t"], append=0)
            this_t = cut_first(this_data["t"])
            new_start_t = results["t"][-1] if len(results["t"]) > 0 else 0.
            results["t"] = np.concatenate((results["t"], this_t - this_t[0] + new_start_t))
            for m in ["V", "angle", "vel", "acc"]:
                results[m] = np.concatenate((results[m], cut_first(this_data[m])))
                results[m+"_f"] = np.concatenate((results[m+"_f"], filter_data(this_data[m])))

        spiky_angle_noise = round(np.std(filter_data(results["spiky_angle"], type='high')) * 1000, 3)
        clean_angle_noise = round(np.std(filter_data(results['angle'], type='high')) * 1000, 3)
        print(f"Angle noise std_dev [mm/s^2]: Spiky: {spiky_angle_noise} Clean: {clean_angle_noise}",
              f"Ratio: {round(clean_angle_noise/spiky_angle_noise, 3)}")
        results["dt"] = dt
        results["N"] = np.size(results["V_f"])
        return results

    def make_independent_variable(results, model):
        """make array used for the independent variable in regression"""
        V_f = results["V_f"]
        vel_f = results["vel_f"]
        X = np.hstack((V_f.reshape(-1,1), vel_f.reshape(-1,1)))
        for mod in model:
            if mod == "const_fric":
                static_direction = np.sign(vel_f)
                X = np.hstack((X, ((1 - slowness_factor_continuous(vel_f)) * static_direction).reshape(-1,1)))
            if mod == "const_opposing_fric":
                opposing_direction = np.where(np.sign(vel_f) == np.sign(V_f), np.sign(vel_f), 0)
                X = np.hstack((X, ((1 - slowness_factor_continuous(vel_f)) * opposing_direction).reshape(-1,1)))
            # expect param["static_fric"] + param["V"] > 0 or else friction reverses
            if mod == "static_fric":
                X = np.hstack((X, (slowness_factor_continuous(vel_f) * V_f).reshape(-1,1)))
            if mod == "offset":
                X = np.hstack((X, (1 - slowness_factor_continuous(vel_f)).reshape(-1,1)))
            elif mod == "Vsq":
                X = np.hstack((X, (np.abs(V_f) * V_f).reshape(-1,1)))
            elif mod == "theta_3dot":   # alternate strategy to modeling current
                X = np.hstack((X, (np.concatenate((np.diff(results["acc_f"]), [0])) / dt).reshape(-1,1)))
            elif mod == "knee_leg_weight_2":
                X = np.hstack((X, -np.sin(results["angle_f"]).reshape(-1,1)))
            elif mod == "hip_leg_weight_1":
                X = np.hstack((X, -np.sin(results["angle_f"]).reshape(-1,1)))
            elif mod == "hip_leg_weight_12":
                X = np.hstack((X, np.sin(results["knee_angle_f"]).reshape(-1,1)))
        return X
    
    def determine_params(X, acc_f, results, remove_outliers=True):
        paramArr = np.linalg.inv(X.T @ X) @ X.T @ acc_f
        results["accel_predic"] = X @ paramArr
        if remove_outliers:
            accel_error = acc_f - results["accel_predic"]
            results["outlier_thresh"] = 3 * np.std(accel_error)
            outliers = np.abs(accel_error) > results["outlier_thresh"]
            acc_f_clean = np.where(outliers, 0., acc_f)
            X_clean = np.where(outliers.reshape((-1,1)) * np.ones((1,X.shape[1])), 0., X)
            print("Removed", np.sum(outliers), "outliers out of", results["N"],
                  "(" + str(round(np.sum(outliers) / results["N"] * 100, 2))
                  + "%) where Error >", round(results["outlier_thresh"], 1), "rad/s^2")
            return determine_params(X_clean, acc_f_clean, results, remove_outliers=False)
        else:
            return paramArr
        
    def assign_parameters(param_array, params, model):
        """assigns parameters based on regression outputs and model"""
        params["V"] = param_array[0]
        params["omega"] = param_array[1]
        for mod, param in zip(model, param_array[2:], strict=True):
            params[mod] = param
        return params

    dt = testdata[0]["dt"] if "dt" in testdata[0] else (testdata[0]["t"][1] - testdata[0]["t"][0])
    if params is None:
        params = {}
    if model is None:
        model = []
    results = filter_and_derivative(testdata)
    
    # use Linear Least Squares regression to determine parameters
    X = make_independent_variable(results, model)
    startTime = time.perf_counter()
    paramArr = determine_params(X, results["acc_f"], results, remove_outliers=True)
    print("Parameter determination took", time.perf_counter() - startTime, "s")
    params = assign_parameters(paramArr, params, model)
    return params, results


def graphMotorResults(params, results):
    print("Filter nyquist period [s]:", round(2 * results["filter_period"], 3), "dt:", results["dt"])
    print("Parameters:", params, end="\n\n")
    accel_error = results["acc_f"] - results["accel_predic"]
    avg_error = np.sum(np.abs(accel_error)) / results["N"]
    error_std_dev = np.std(accel_error)
    print("Average error =", round(avg_error, 3), "rad/s^2, Average acceleration", 
          round(np.sum(np.abs(results["acc_f"] )) / results["N"], 3), "rad/s^2")
    print("Error std dev:", round(error_std_dev, 3), "rad/s^2")
    print("Regression R^2:", round(1 - np.sum(accel_error**2) / np.sum(results["acc_f"]**2), 3))
    accel_error_nan = np.where(np.abs(accel_error) < results["outlier_thresh"], accel_error, np.NaN)
    acc_f_nan = np.where(np.abs(accel_error) < results["outlier_thresh"], results["acc_f"], np.NaN)
    print(f"\tWithout {np.sum(np.isnan(accel_error_nan))} outliers:")
    print("Average error =", round(np.nanmean(np.abs(accel_error_nan)), 3), "rad/s^2, Average acceleration",
          round(np.nanmean(np.abs(acc_f_nan)), 3), "rad/s^2")
    error_std_dev_clean = np.nanstd(np.abs(accel_error_nan))
    print("Error std dev:", round(error_std_dev_clean, 3), "rad/s^2")
    print("Regression R^2:", round(1 - np.nansum(accel_error_nan**2) / np.nansum(acc_f_nan**2), 3))

    # graph voltage, angle, velocity, and acceleration filtered and unfiltered over time
    plt.figure(1)
    plt.clf()
    ax = plt.subplot(411)
    plt.suptitle("Raw and Filtered Data")
    plt.ylabel("Driving Voltage (V)")
    lines = plt.plot(results["t"], results["V"], results["t"], results["V_f"])
    plt.legend(lines, ["Raw", "Filtered"])
    plt.grid()
    plt.subplot(412, sharex=ax)
    plt.ylabel("Angle [rad]")
    if "spiky_angle" in results:
        plt.plot(results["t"], results["spiky_angle"], 'c')
    plt.plot(results["t"], results["angle"], results["t"], results["angle_f"])
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
    ax.scatter(results["V_f"], results["vel_f"], zs=results["acc_f"], c=results["t"])
    plt.title("Acceleration")
    ax.set_zlabel("Acceleration [rad/s^2]")
    plt.xlabel("Voltage [V]")
    plt.ylabel("Velocity [rad/s]")
    
    # 3D graph acceleration error against voltage and speed
    fig = plt.figure(3)
    plt.clf()
    ax = fig.add_subplot(projection = '3d')
    ax.scatter(results["V_f"], results["vel_f"], zs=accel_error_nan, c=results["t"])  # c=accel_error)
    plt.title("Acceleration Error")
    ax.set_zlabel("Acceleration Error [rad/s^2]")
    plt.xlabel("Voltage [V]")
    plt.ylabel("Velocity [rad/s]")
    
    # investigate torque ripple: plot acceleration error against angle
    plt.figure(4)
    plt.clf()
    plt.scatter(results["angle_f"], accel_error, c=results["t"])
    plt.title("Torque Ripple")
    plt.xlabel("Angle [rad]")
    plt.ylabel("Acceleration error [rad/s^2]")
    plt.grid()

    plt.figure(5)
    plt.clf()
    plt.plot(results["t"], np.abs(results["vel_f"]), results["t"],
             slowness_factor_continuous(results["vel_f"]))
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

    plt.figure(7)
    plt.clf()
    _, x, _ = plt.hist([accel_error, accel_error_nan], bins="auto", label=["all", "clean"], log=True)
    pdf_scale = len(accel_error) * (np.max(x) - np.min(x)) / (len(x) - 1)  # n_points * x_domain_length / n_bins
    plt.plot(x, pdf_scale * stats.norm.pdf(x, loc=np.mean(accel_error), scale=error_std_dev), label="norm w/ outliers")
    x = np.array([x_i for x_i in x if abs(x_i) < results["outlier_thresh"]])
    pdf_scale = len(accel_error) * (np.max(x) - np.min(x)) / len(x)
    plt.plot(x, pdf_scale * stats.norm.pdf(x, loc=np.mean(accel_error), scale=error_std_dev_clean), 'k', label="clean normal")
    plt.ylim(bottom=0.7 if plt.gca().get_yscale() == "log" else 0, top=plt.ylim()[1])
    plt.title("Acceleration Error")
    plt.xlabel("Acceleration Error [rad/s^2]")
    plt.ylabel("count")
    plt.legend()
    plt.grid()

    # plt.figure(8)
    # plt.clf()
    # even_time = np.linspace(start=results["t"][0], num=len(results["t"]), stop=len(results["t"]) * results["dt"])
    # plt.plot(even_time, results["t"] - even_time)
    # plt.title("Time measurement error")
    # plt.xlabel("Evenly Spaced Time [s]")
    # plt.ylabel("True Time diff. from Even Time [s]")
    # plt.grid()

    plt.show()
    time.sleep(0.3)


def loadRun(filename):
    if not "motortest" in filename:
        filename = filename + ".motortest"
    try:
        with open("data" + sep + filename, 'rb') as file:
            data = dict(np.load(file, allow_pickle=True))
            if "t" not in data:
                data["t"] = np.linspace(0., len(data["V"]) * data["dt"], num=len(data["V"]))
            return data
    except Exception as e:
        print("Load failed:", filename, " because \n", e)

def cut_first(data, n=FILT_N * 2):
    return data[n:]

def filter_data(data, type='lowpass'):
    n_suspicious_filtered = FILT_N * 2
    padded_data = np.concatenate((np.average(data[:n_suspicious_filtered]) * np.ones(n_suspicious_filtered), data))
    if FILT_FCN == 'cheby':
        filtered = signal.sosfilt(signal.cheby1(FILT_ORDER, FILT_RIPPLE, 1/FILT_N, type, output='sos'), padded_data)
    else:
        filtered = signal.sosfilt(signal.butter(FILT_ORDER, 1/FILT_N, type, output='sos'), padded_data)
    filtered = filtered[n_suspicious_filtered:]
    if type == 'lowpass' or type == 'low':
        filtered = cut_first(filtered)
    return filtered


def clean_up_test_data(this_data, results):
    # the angle is fuzzy, with noticeable spikes down from a probable true value. Remove those.
    def remove_angle_downspikes(this_data, results):
        min_drop_start_size = 0.001
        # min_drop_start_size = 0.004  # between 1 and 2 ticks
        # max_drop_size = 0.02   # between 6 and 7 ticks
        max_drop_width = 4

        future_length = max_drop_width + 1
        angle = this_data["angle"]
        prev_angle = np.roll(angle, 1)
        prev_angle[0] = prev_angle[1]
        # distance from previous angle to this angle and several future angles. [:,i] is i samples in the future.
        change_from_prev_angle = np.empty((len(angle), future_length))
        for i in range(future_length):
            change_from_prev_angle[:, i] = np.roll(angle, -i) - prev_angle
        is_down_from_prev = change_from_prev_angle < -min_drop_start_size

        clean_angle = angle.copy()
        for width in range(1, max_drop_width + 1):
            all_samples_down_from_prev = np.all(is_down_from_prev[:, 0:width], axis=1)
            change_across_spike = change_from_prev_angle[:, width]
            ends_going_up = (change_from_prev_angle[:, width-1] < change_across_spike)
            is_start_of_spike = (all_samples_down_from_prev & ends_going_up)
            for index_in_spike in range(width):
                is_this_index_in_spike = np.roll(is_start_of_spike, index_in_spike)
                interpolated_value_unshifted = (index_in_spike + 1) / (width + 1) * change_across_spike + prev_angle
                interpolated_value = np.roll(interpolated_value_unshifted, index_in_spike)
                clean_angle = np.where(is_this_index_in_spike, interpolated_value, clean_angle)
        # clean angle is now clean, replace testdata with it and save results["spiky_angle"]
        if "spiky_angle" in results:
            results["spiky_angle"] = np.concatenate((results["spiky_angle"], cut_first(angle)))
        else:
            results["spiky_angle"] = cut_first(angle)
        this_data["angle"] = clean_angle

    def remove_idle_messages(this_data):
        index_first = np.flatnonzero(this_data["t"] < 100)[0]
        for m in ["V", "angle", "t"]:
            this_data[m] = this_data[m][index_first:]
        index_last = np.flatnonzero(this_data["t"] > 100)[0]
        for m in ["V", "angle", "t"]:
            this_data[m] = this_data[m][:index_last]
        if index_first > 0 or len(this_data['angle']) > index_last:
            print(f"Cut off first {index_first}, last {len(this_data['angle']) - index_last}, now {len(this_data['angle'])}")

    startTime = time.time()
    remove_idle_messages(this_data)
    remove_angle_downspikes(this_data, results)
    duration = time.time() - startTime
    results["data clean time"] = results["data clean time"] + duration if "data clean time" in results else duration


# continuous number [0,1] where 1 means it's slow enough for static friction
def slowness_factor_continuous(vel, speed_thresh = 0.06):
    #return filter_data(np.where(np.abs(vel) < speed_thresh, (speed_thresh - np.abs(vel)) / speed_thresh, 0), 4)
    return np.arctan((np.abs(vel) < speed_thresh) * 20 - 10) / np.pi + 0.5
    # return np.arctan(speed_thresh / np.abs(vel)) / np.pi * 2
    # return np.clip(speed_thresh / np.abs(vel), 0., 1.)

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
          "examine ['filename'|all <file_prefix>]",
          "saveparams ['filename']",
          "loadparams <'filename'>",
          "model [[no] model_name]",
          "code", sep = "\n ")
    testdata = []
    model = ['static_fric', 'const_opposing_fric']
    params = {}
    directory_path = getcwd() + "/data"
    test_type = ""
    sysargs = None if (len(sys.argv) < 2) else sys.argv[1:]
    
    while True:
        if sysargs:
            args = sysargs
            print("From cmd args:", args)
            sysargs = None
        else:
            args = input(">>> ").split()
        command = args[0] if len(args) > 0 else "" #command is first, the rest of the args are available if needed
        command = command.strip().lower()
        if command == "":
            continue
        elif command == "examine":
            if len(args) > 1:
                if args[1] == "all" and len(args) > 2:
                    filenames = [f for f in listdir(directory_path) if isfile(join(directory_path,f)) and f.startswith(args[2])]
                    print("Loading", filenames, "\n")
                    testdata = []
                    for i, f in enumerate(filenames):
                        newdata = loadRun(f)
                        if newdata is None:
                            print("couldn't load", f)
                        else:
                            testdata.append(newdata)
                            if len(testdata) == 1:
                                test_type = testdata[0]["test_type"]
                            else:
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
                    graphMotorResults(params, results)
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
