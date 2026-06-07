# -*- coding: utf-8 -*-
"""
Library for motor calibration
TODO:
    some way to deal with nonlinear parameters, like slowness threshold
        Way to set them as model parameters, either definitively or a starting point for nonlinear optimization
    deadband model: modify angle measurements?
    measure battery voltage during calibration
    why angle jumps? Just remove them from test data?
    electrically isolate angle sensors to reduce noise?
    structure: clear flow of sensor data -> /(sensor model) -> believed true values -> /(motor model) -> prediction
    prediction accuracy as fcn of vel?
    deadband and low-V squ tests are characteristic like impulse response. We see ringing! Use that to measure response
        fourier analysis (compare F_V(w) with F_acc_pred(w)) to find frequency response of system
        manually first, then cli version
    Screw a second motor to the calibration motor, short its motor wires, and use it to measure with (2x) backlash?

Created Jun 2021
@author: Niraj
"""

import numpy as np
from scipy.fft import rfftfreq
import time
from filter import FilterParams, filter_data, cut_first, moving_avg, derivative, remove_spikes
import motor_model


def examineMotor(testdata, model=None, params=None, filter_params=FilterParams()):
    """return the model parameters (dict) of a motor.
    and the results of the examination (dict)
    V is 1D array of driving voltage, angle is same size array of sensed angle.
    V and angle can be one array or lists of arrays.
    params is optional existing parameters which may be edited.
    Motor model d2(angle)/dt2 = param[V] * V + param[omega] * d(angle)/dt
    model is a list of strings naming additional model dynamics"""

    results_t = dict[str, float | int | np.ndarray | np.float64 | np.int64 | dict]

    def filter_and_derivative_and_assemble(testdata, filter_params) -> results_t:
        results: results_t = {m: np.empty(0) for m in ["t", "angle_hf", "spiky_angle_hf"]
                   + [n + f for n in ["V", "angle", "vel", "acc", "spiky_angle"] for f in ("", "_f")]}
        for this_data in testdata:
            if filter_params.T is not None:
                filter_params.set_N(N=None, t_data=this_data["t"])
            results["filter_period"] = this_data["t"][filter_params.N] - this_data["t"][0]
            clean_up_test_data(this_data, results, filter_params)
            this_data["vel"] = derivative(this_data, "angle")
            this_data["acc"] = derivative(this_data, "vel")
            assemble_t(this_data, results, filter_params)
            for m in ["V", "angle", "vel", "acc", "spiky_angle"]:
                results[m] = np.concatenate((results[m], cut_first(this_data[m], filter_params)))
                results[m+"_f"] = np.concatenate((results[m+"_f"], filter_data(this_data[m], filter_params)))
            for m in ["spiky_angle", "angle"]:
                results[m + "_hf"] = np.concatenate((results[m+"_hf"], filter_data(this_data[m], filter_params, type='high')))
        results["diff_t"] = np.diff(results["t"])
        if np.any(results["diff_t"] <= 0):
            raise ValueError("ERROR: time stops or goes backwards", np.flatnonzero(results["diff_t"] <= 0))
        spiky_angle_noise = round(np.std(results["spiky_angle_hf"]) * 1000, 3)
        clean_angle_noise = round(np.std(results["angle_hf"]) * 1000, 3)
        print(f"Angle noise std_dev [mrad]: Spiky: {spiky_angle_noise} Clean: {clean_angle_noise}",
              f"Ratio: {round(clean_angle_noise/spiky_angle_noise, 3)}")
        results["N"] = np.size(results["V_f"])
        return results

    def assemble_t(this_data, results, filter_params):
        this_t = cut_first(this_data["t"], filter_params)
        new_start_t = results["t"][-1] + (this_t[1] - this_t[0]) if len(results["t"]) > 0 else 0.
        results["t"] = np.concatenate((results["t"], this_t - this_t[0] + new_start_t))
        if "log_starts" not in results:
            results["log_starts"] = {}
        results["log_starts"][new_start_t] = this_data["filename"]

    def determine_params(X, acc_f, results, remove_outliers=True):
        paramArr = np.linalg.inv(X.T @ X) @ X.T @ acc_f  # linear least squares
        if remove_outliers:
            accel_error = acc_f - X @ paramArr
            results["outlier_thresh"] = 3 * np.std(accel_error)
            outliers = np.abs(accel_error) > results["outlier_thresh"]
            acc_f_clean = np.where(outliers, 0., acc_f)
            X_clean = np.where(outliers.reshape((-1,1)) * np.ones((1,X.shape[1])), 0., X)
            results["num_outliers"] = np.sum(outliers)
            print("Removed", results["num_outliers"], "outliers out of", results["N"],
                  "(" + str(round(results["num_outliers"] / results["N"] * 100, 2))
                  + "%) where Error >", round(results["outlier_thresh"], 1), "rad/s^2")
            return determine_params(X_clean, acc_f_clean, results, remove_outliers=False)
        else:
            return paramArr

    def calc_result_stats(results):
        accel_error = results["accel_predic"] - results["acc_f"]
        results["error_std_dev"] = np.std(accel_error)
        results["outlier_thresh"] = 3 * results["error_std_dev"]
        results["num_outliers"] = np.sum(np.abs(accel_error) > results["outlier_thresh"])
        results["outlier%"] = results["num_outliers"] / results["N"] * 100
        results["accel_error"] = accel_error
        results["avg_error"] = np.average(np.abs(accel_error))
        results["R^2_with_outliers"] = 1 - np.sum(accel_error**2) / np.sum(results["acc_f"]**2)
        results["accel_error_nan"] = np.where(np.abs(accel_error) < results["outlier_thresh"], accel_error, np.nan)
        acc_f_nan = np.where(np.abs(accel_error) < results["outlier_thresh"], results["acc_f"], np.nan)
        results["avg_acc_no_outliers"] = np.nanmean(np.abs(acc_f_nan))
        results["R^2_no_outliers"] = 1 - np.nansum(results["accel_error_nan"]**2) / np.nansum(acc_f_nan**2)
        results["avg_error_no_outs"] = np.nanmean(np.abs(results["accel_error_nan"]))
        results["error_std_dev_no_outs"] = np.nanstd(np.abs(results["accel_error_nan"]))
        results["dt"] = float(np.mean(results["diff_t"]))
        results["dt_std_dev"] = float(np.std(np.where(results["diff_t"] > 10 * results["dt"], results["dt"], results["diff_t"])))
        minutes, seconds = divmod(int(np.max(results["t"])), 60)
        results["runtime"] = f'{minutes:02d}:{seconds:02d}' if minutes > 0 else f'{seconds} s'
        results["fft_freqs"] = None
        if results["dt_std_dev"] / results["dt"] < 0.01:  # if dt is stable to within 1% precision
            results["fft_freqs"] = rfftfreq(results["N"], results["dt"])

    if len(testdata) == 0:
        raise ValueError("No testdata")
    results = filter_and_derivative_and_assemble(testdata, filter_params)

    model = motor_model.Model(model, params)
    X = motor_model.make_independent_variable(results, model)
    if params is None:
        startTime = time.perf_counter()
        paramArr = determine_params(X, results["acc_f"], results, remove_outliers=True)
        print("Parameter determination took", time.perf_counter() - startTime, "s")
        params = motor_model.assign_parameters(paramArr, model)
    else:
        paramArr = [params[m] for m in ["V", "omega"] + model]
    results["accel_predic"] = X @ paramArr
    calc_result_stats(results)
    results["model_contributions"] = motor_model.calc_model_contributions(X, paramArr, model)
    return params, results


def printMotorResults(params, results):
    print(f'dt: {results["dt"]:.3} s; dt std dev: {results["dt_std_dev"]:.2e} s; total runtime:', results["runtime"])
    print("Parameters:", params, end="\n\n")
    print(f'Filter nyquist period [s]: {2 * results["filter_period"]:.3}')
    print("Average error =", round(results["avg_error"], 3), "rad/s^2, Average acceleration =",
          round(np.sum(np.abs(results["acc_f"] )) / results["N"], 3), "rad/s^2")
    print("Error std dev:", round(results["error_std_dev"], 3), "rad/s^2")
    print("Regression R^2:", round(results["R^2_with_outliers"], 3))
    print(f'\t-Without {results["num_outliers"]} outliers ({round(results["outlier%"], 2)}%):')
    print("Average error =", round(results["avg_error_no_outs"], 3), "rad/s^2, Average acceleration",
          round(results["avg_acc_no_outliers"], 3), "rad/s^2")
    print("Error std dev:", round(results["error_std_dev_no_outs"], 3), "rad/s^2")
    print("Regression R^2:", round(results["R^2_no_outliers"], 3))


def saveResultsJson(filename, results, model):
    from os import path; import json
    if path.exists(filename):
        with open(filename) as inFile:
            output = json.load(inFile)
    else:
        output = {"tests": []}
    this_test = {}
    for k, v in results.items():
        if (type(v) in [np.ndarray, np.float64] and v.size == 1) or type(v) is float:
            this_test[k] = float(v)
        elif (type(v) is np.int64 and v.size == 1) or type(v) is int:
            this_test[k] = int(v)
        elif type(v) is str:
            this_test[k] = v
    this_test["model"] = " ".join(model)
    this_test["input_file_prefix"] = path.commonprefix(list(results["log_starts"].values())).split("/")[-1]
    output["tests"].append(this_test)
    with open(filename, "w") as outFile:
        json.dump(output, outFile, indent=2)


def loadRun(filename):
    if "motortest" not in filename:
        filename = filename + ".motortest"
    try:
        with open(filename, 'rb') as file:
            data = dict(np.load(file, allow_pickle=True))
            data["filename"] = filename
            if "t" not in data:
                data["t"] = np.linspace(0., len(data["V"]) * data["dt"], num=len(data["V"]))
            return data
    except Exception as e:
        print("Load failed:", filename, " because \n", e)


def clean_up_test_data(this_data, results, filter_params):
    def remove_idle_messages(this_data, t_max=100):
        index_first = np.flatnonzero(this_data["t"] < t_max)[0]
        for m in ["V", "angle", "t"]:
            this_data[m] = this_data[m][index_first:]
        high_t = np.flatnonzero(this_data["t"] > t_max)
        index_last = high_t[0] if len(high_t) > 0 else len(this_data["t"])
        for m in ["V", "angle", "t"]:
            this_data[m] = this_data[m][:index_last]

    remove_idle_messages(this_data)
    # save "spiky_angle" so we can overwrite testdata["angle"] with clean_angle
    this_data["spiky_angle"] = this_data["angle"]
    # the angle is fuzzy, with noticeable spikes down from a probable true value. Remove those.
    this_data["angle"] = remove_spikes(this_data["angle"], [False], max_up_spike_width=2)
    this_data["angle"] = remove_spikes(this_data["angle"], [True], max_down_spike_width=4)
    # this_data["angle"] = remove_spikes(this_data["angle"], [False, True], recalc_change=True)
    FILT_MOVING_AVG_N = 10  # mostly to counter value quantization. After spike removal, before lowpass.
    assert(FILT_MOVING_AVG_N < filter_params.N)
    this_data["angle"] = moving_avg(this_data["angle"], n_samples=FILT_MOVING_AVG_N, pad_val=this_data["angle"][0])
